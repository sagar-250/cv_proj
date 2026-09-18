"""Phase 4 -- planar segmentation + clustering.

Classical, point-cloud-native replacement for SPEC.md's Phase 4 (detection)
+ Phase 5 (3D localisation) combined -- those assume a Blender RGB/depth/
index-pass image pipeline we never built (v1 goes straight from BIM to a
raycast-simulated point cloud instead, see p3_render.py). This operates
directly on points: iterative RANSAC plane extraction + DBSCAN clustering,
using only open3d (already a project dependency, no new install).

Algorithm, per stage:
  1. Ground removal: RANSAC a plane on the lowest 20cm z-band only (not the
     whole cloud -- otherwise a big mid-height slab could win the global
     plane vote). Accept as ground only if the fit is near-horizontal, its
     footprint dwarfs the building's, and it sits near the cloud's z-min.
     Reject (and flag) rather than force a bad call if these don't all hold.
  2. Iterative plane peeling on what's left: repeatedly RANSAC the single
     largest remaining plane, then DBSCAN *within* that plane's inliers to
     split spatially separate coplanar instances (two parallel walls must
     not merge just because they share an orientation). Classify each
     resulting instance by normal angle: near-horizontal -> slab,
     near-vertical -> wall, else "ambiguous_planar" (logged, not discarded --
     e.g. sloped roof faces may legitimately land here).
  3. Whatever never fit a good plane (columns, beams, and non-planar
     clutter like tree canopies) gets its own DBSCAN pass, classified by
     PCA aspect ratio: elongated + near-vertical axis -> column, elongated +
     near-horizontal axis -> beam, otherwise "clutter" (isotropic blobs --
     this is how tree canopies fall out, with no hardcoded "outside the
     footprint" rule, which is the part that should generalise to real
     messy vegetation).

Known failure modes (see also p2_p3_simulate_stages.py's degradation
parameters this must survive):
  - Partial/height-clipped walls (p2_ablate.py) are much shorter than a
    finished wall -- classification by normal angle is unaffected, but
    keep the minimum-cluster-diagonal filter low (0.3m) so a legitimately
    partial wall isn't discarded as noise.
  - 15% random dropout (DROPOUT_FRAC in p2_p3_simulate_stages.py) puts gaps
    in planar patches -- DBSCAN eps values here are chosen generously to
    survive that; re-check empirically if a stage's ray density differs a
    lot from the default (interior spherical scans are much denser near
    their origin than exterior pinhole grids at range).
  - The whole-scene registration jitter (REG_JITTER_T/DEG) is a single
    uniform rigid transform, so it does NOT add local noise to plane
    fitting -- it only rotates the global normal direction by up to
    ~1.2 degrees, well inside the wall/slab angle thresholds below.
  - Stage 00 (foundation) is a specific watch item: p3_render.py's ground
    plane sits only 5cm below the foundation slab's base, so ground-vs-
    foundation-slab confusion is plausible -- check this stage's log output.
  - Sloped roof faces (stages 14/15) may fall in the 20-70 degree
    "ambiguous_planar" band rather than being called slab -- check the
    ambiguous count for those stages rather than assuming a bug.

Output per stage: data/observations/{stage}/observations.json (one record
per discovered cluster, including clutter/ambiguous ones -- nothing is
hidden, a later matching phase filters) and point_cluster_id.npy (int32,
same order as the source .npz's xyz; cluster index or -1), the join key
p4_eval_segment.py uses against the held-out ground truth.

Run: .venv/Scripts/python.exe src/p4_segment.py [--stage NAME]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(__file__))
from p1_parse import obb_xy  # noqa: E402

IN_DIR = "data/pointclouds"
OUT_DIR = "data/observations"

# ground detection
GROUND_BAND_HEIGHT = 0.80  # NOT the raw ~10cm sensor noise on ground generation -- the
                            # whole-scene registration jitter (a rigid rotation about the
                            # scene centroid) tilts the wide (~40m radius) ground plane just
                            # enough that a small angle x large lever arm spreads its Z range
                            # to ~0.5m in practice. Confirmed empirically (measured 0.53m).
                            # A band restricted to this height keeps candidate points to
                            # ~ground only (measured: zero contamination from other classes)
                            # -- searching the FULL cloud instead lets RANSAC pick a denser
                            # coplanar cluster of walls over the sparser, wider ground plane.
GROUND_RANSAC_DIST = 0.04
GROUND_MAX_NORMAL_ANGLE_DEG = 15.0
GROUND_MIN_FOOTPRINT_RATIO = 0.70  # measured true-ground ratio (within the band) is ~0.83-0.86.
GROUND_MAX_Z_OFFSET = 0.30  # mean inlier z vs. the band's own lower edge (z_min); measured ~0.15.

# iterative plane peeling
PLANE_RANSAC_DIST = 0.03
PLANE_RANSAC_ITERS = 1000
MIN_PLANE_POINTS = 80
PLANE_DBSCAN_EPS = 0.4      # loosened from an initial 0.25 after measuring real point loss:
                            # ~30% of true structural points were being RANSAC-consumed by a
                            # plane fit, then discarded as DBSCAN noise within that plane
                            # (dropout gaps + partial/height-clipped walls fragment more than
                            # a clean scan would). Pushing min_points much lower recovers more
                            # points but badly over-fragments slabs into hundreds of tiny
                            # pieces (measured); this setting is a middle ground (~74% point
                            # retention at stage 03) -- residual over/under-segmentation of
                            # coplanar structure is a known limitation of plane-based
                            # clustering, not something tuned away here. p4_eval_segment.py's
                            # matched/merged/split/missed bucketing is what actually quantifies
                            # it, which is the intended v1 scope.
PLANE_DBSCAN_MIN_POINTS = 15
MIN_CLUSTER_DIAGONAL = 0.25
SLAB_MAX_NORMAL_ANGLE_DEG = 20.0   # normal within this of Z -> slab
WALL_MIN_NORMAL_ANGLE_DEG = 70.0   # normal within this of horizontal -> wall

# residual (non-planar) clustering
RESIDUAL_DBSCAN_EPS = 0.18
RESIDUAL_DBSCAN_MIN_POINTS = 15
ELONGATION_MIN_ASPECT = 2.5
ELONGATION_MAX_CROSS_SECTION = 0.6
COLUMN_MAX_AXIS_ANGLE_DEG = 30.0   # principal axis within this of Z -> column
BEAM_MIN_AXIS_ANGLE_DEG = 60.0     # principal axis within this of horizontal -> beam


def angle_to_z_deg(normal):
    n = normal / (np.linalg.norm(normal) + 1e-12)
    return float(np.degrees(np.arccos(np.clip(abs(n[2]), 0, 1))))


def bbox_diagonal(pts):
    return float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))


def dbscan(pts, eps, min_points):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    return labels


GROUND_MAX_PASSES = 4


def remove_ground(xyz):
    """RANSAC restricted to a wide (~0.8m) lowest-Z band, not the full cloud:
    ground is sparse and wide, so a global RANSAC pass over everything can
    lose to a denser coplanar cluster of walls (confirmed empirically -- see
    GROUND_BAND_HEIGHT's comment). Restricting candidates to the low-Z band
    keeps the fit to (in practice) only true ground points.

    Runs multiple passes over the band (removing accepted ground each time)
    rather than stopping after one: a single RANSAC pass on a tilted, noisy
    plane often only captures part of it, leaving the rest to be picked up
    later by an unrelated nearby structural plane fit (measured as high
    "clutter leakage" in eval -- this is the fix for that).

    Returns (remaining_xyz, remaining_orig_idx, ground_mask_over_full_xyz, reason)."""
    z_min = xyz[:, 2].min()
    band_mask = (xyz[:, 2] >= z_min - 0.02) & (xyz[:, 2] <= z_min + GROUND_BAND_HEIGHT)
    band_idx = np.nonzero(band_mask)[0]
    if band_idx.shape[0] < 50:
        return xyz, np.arange(xyz.shape[0]), np.zeros(xyz.shape[0], dtype=bool), "too_few_band_points"

    whole_footprint_radius = float(np.linalg.norm(xyz[:, :2] - xyz[:, :2].mean(axis=0), axis=1).max())
    ground_mask = np.zeros(xyz.shape[0], dtype=bool)
    remaining_band = band_idx.copy()
    reasons = []

    for _pass in range(GROUND_MAX_PASSES):
        if remaining_band.shape[0] < 50:
            break
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz[remaining_band])
        plane_model, inliers = pcd.segment_plane(GROUND_RANSAC_DIST, 3, 1000)
        if len(inliers) < 50:
            break
        inlier_idx = remaining_band[inliers]

        normal = np.array(plane_model[:3])
        normal_angle = angle_to_z_deg(normal)
        inlier_xy = xyz[inlier_idx][:, :2]
        footprint_radius = float(np.linalg.norm(inlier_xy - inlier_xy.mean(axis=0), axis=1).max())
        mean_z = float(xyz[inlier_idx][:, 2].mean())

        ok_angle = normal_angle < GROUND_MAX_NORMAL_ANGLE_DEG
        ok_footprint = footprint_radius > GROUND_MIN_FOOTPRINT_RATIO * whole_footprint_radius
        ok_height = abs(mean_z - z_min) < GROUND_MAX_Z_OFFSET

        if ok_angle and ok_footprint and ok_height:
            ground_mask[inlier_idx] = True
            remaining_band = np.setdiff1d(remaining_band, inlier_idx)
            reasons.append(f"pass{_pass}=accepted({len(inlier_idx)}pts)")
        else:
            reasons.append(f"pass{_pass}=rejected(angle={normal_angle:.1f},"
                            f"footprint_ratio={footprint_radius/whole_footprint_radius:.2f},"
                            f"z_offset={abs(mean_z-z_min):.3f})")
            break  # once a pass fails the checks, further passes on the same band are unlikely to help

    remaining_idx = np.nonzero(~ground_mask)[0]
    return xyz[remaining_idx], remaining_idx, ground_mask, "; ".join(reasons)


def peel_planes(xyz, remaining_idx):
    """Returns list of dicts {idx (into original xyz), class, normal} and the
    leftover (non-planar) original-index array."""
    clusters = []
    work_idx = remaining_idx.copy()

    while work_idx.shape[0] >= MIN_PLANE_POINTS:
        pts = xyz[work_idx]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        plane_model, inliers = pcd.segment_plane(PLANE_RANSAC_DIST, 3, PLANE_RANSAC_ITERS)
        if len(inliers) < MIN_PLANE_POINTS:
            break

        inlier_local = np.array(inliers)
        inlier_orig_idx = work_idx[inlier_local]
        inlier_pts = xyz[inlier_orig_idx]

        sub_labels = dbscan(inlier_pts, PLANE_DBSCAN_EPS, PLANE_DBSCAN_MIN_POINTS)
        normal = np.array(plane_model[:3])
        normal_angle = angle_to_z_deg(normal)

        def classify_planar(normal_angle):
            if normal_angle < SLAB_MAX_NORMAL_ANGLE_DEG:
                return "slab"
            elif normal_angle > WALL_MIN_NORMAL_ANGLE_DEG:
                return "wall"
            return "ambiguous_planar"

        noise_frac = (sub_labels == -1).mean()
        if noise_frac > 0.5 and inlier_orig_idx.shape[0] > 500:
            # a large, sparse plane (e.g. a ground plane that slipped past
            # remove_ground) can be sparse enough that within-plane DBSCAN
            # marks most of it as noise -- losing it all silently would be
            # far worse than keeping it as one big (possibly non-BIM,
            # e.g. ground) cluster instead of several small ones.
            clusters.append({"idx": inlier_orig_idx, "class": classify_planar(normal_angle), "normal": normal})
        else:
            for lbl in sorted(set(sub_labels.tolist()) - {-1}):
                sub_mask = sub_labels == lbl
                sub_orig_idx = inlier_orig_idx[sub_mask]
                if sub_orig_idx.shape[0] < 15 or bbox_diagonal(xyz[sub_orig_idx]) < MIN_CLUSTER_DIAGONAL:
                    continue
                clusters.append({"idx": sub_orig_idx, "class": classify_planar(normal_angle), "normal": normal})

        # remove ALL plane inliers (including DBSCAN noise, label==-1) from
        # the working set -- noise within a dominant plane is still "used up"
        work_idx = np.setdiff1d(work_idx, inlier_orig_idx, assume_unique=False)

    return clusters, work_idx


MERGE_ANGLE_DEG = 5.0     # coplanar-fragment merge: normals within this angle
MERGE_OFFSET = 0.08       # and mean point-to-plane distance within this (m) of each other


def _plane_fit(pts):
    """Least-squares plane through pts: unit normal + offset d s.t. normal.p + d = 0."""
    centroid = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - centroid)
    normal = vh[-1]
    d = -normal @ centroid
    return normal, d


def merge_coplanar_clusters(clusters, xyz):
    """Union same-class planar clusters that lie on (near enough) the same
    plane -- this is what fixes over-fragmentation from partial RANSAC
    passes and dropout gaps splitting one real surface into many pieces.
    Two clusters merge only if BOTH normals agree (within MERGE_ANGLE_DEG)
    AND each cluster's centroid is close to the other's fitted plane
    (within MERGE_OFFSET) -- this is robust to segment_plane's arbitrary
    normal sign per RANSAC call (checked via abs(cos angle))."""
    n = len(clusters)
    if n <= 1:
        return clusters

    fits = []
    for c in clusters:
        pts = xyz[c["idx"]]
        normal, d = _plane_fit(pts)
        fits.append((normal, d, pts.mean(axis=0)))

    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for i in range(n):
        if clusters[i]["class"] not in ("wall", "slab", "ambiguous_planar"):
            continue
        ni, di, ci = fits[i]
        for j in range(i + 1, n):
            if clusters[j]["class"] != clusters[i]["class"]:
                continue
            nj, dj, cj = fits[j]
            cos_angle = abs(float(np.dot(ni, nj)))
            if cos_angle < np.cos(np.radians(MERGE_ANGLE_DEG)):
                continue
            dist_i_to_j_plane = abs(float(np.dot(nj, ci)) + dj)
            dist_j_to_i_plane = abs(float(np.dot(ni, cj)) + di)
            if dist_i_to_j_plane < MERGE_OFFSET and dist_j_to_i_plane < MERGE_OFFSET:
                union(i, j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    merged = []
    for members in groups.values():
        if len(members) == 1:
            merged.append(clusters[members[0]])
        else:
            idx = np.concatenate([clusters[m]["idx"] for m in members])
            merged.append({"idx": idx, "class": clusters[members[0]]["class"], "normal": fits[members[0]][0]})
    return merged


def classify_residual(xyz, residual_idx):
    clusters = []
    if residual_idx.shape[0] < RESIDUAL_DBSCAN_MIN_POINTS:
        return clusters
    pts = xyz[residual_idx]
    labels = dbscan(pts, RESIDUAL_DBSCAN_EPS, RESIDUAL_DBSCAN_MIN_POINTS)

    for lbl in sorted(set(labels.tolist()) - {-1}):
        mask = labels == lbl
        orig_idx = residual_idx[mask]
        cluster_pts = xyz[orig_idx]
        if orig_idx.shape[0] < RESIDUAL_DBSCAN_MIN_POINTS:
            continue

        centered = cluster_pts - cluster_pts.mean(axis=0)
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)[::-1]
        eigvals, eigvecs = eigvals[order], eigvecs[:, order]
        extents = np.sqrt(np.clip(eigvals, 0, None)) * 2  # rough extent along each principal axis
        d1, d2 = extents[0], extents[1]
        aspect = d1 / max(d2, 1e-6)
        cross_section = d2

        if aspect > ELONGATION_MIN_ASPECT and cross_section < ELONGATION_MAX_CROSS_SECTION:
            axis_angle = angle_to_z_deg(eigvecs[:, 0])
            if axis_angle < COLUMN_MAX_AXIS_ANGLE_DEG:
                cls = "column"
            elif axis_angle > BEAM_MIN_AXIS_ANGLE_DEG:
                cls = "beam"
            else:
                cls = "ambiguous_elongated"
        else:
            cls = "clutter"

        clusters.append({"idx": orig_idx, "class": cls, "normal": None})

    return clusters


def make_observation(obs_id, stage_name, cluster, method):
    idx, cls = cluster["idx"], cluster["class"]
    pts = XYZ_CACHE[idx]
    aabb_min, aabb_max = pts.min(axis=0), pts.max(axis=0)
    l, w, rot_z = obb_xy(pts[:, :2])
    h = float(aabb_max[2] - aabb_min[2])
    centroid = pts.mean(axis=0)

    if method == "planar":
        score = min(1.0, pts.shape[0] / 500.0)
    else:
        score = min(1.0, pts.shape[0] / 50.0)

    return {
        "obs_id": obs_id,
        "class": cls,
        "source_stage": stage_name,
        "centroid_x": float(centroid[0]), "centroid_y": float(centroid[1]), "centroid_z": float(centroid[2]),
        "aabb_min_x": float(aabb_min[0]), "aabb_min_y": float(aabb_min[1]), "aabb_min_z": float(aabb_min[2]),
        "aabb_max_x": float(aabb_max[0]), "aabb_max_y": float(aabb_max[1]), "aabb_max_z": float(aabb_max[2]),
        "obb_dims_l": l, "obb_dims_w": w, "obb_dims_h": h, "obb_rot_z": rot_z,
        "point_count": int(pts.shape[0]),
        "score": float(score),
        "n_views": 1,
        "method": method,
    }


XYZ_CACHE = None  # set per-stage in segment_stage(), avoids threading xyz through every call


def segment_stage(stage_name):
    global XYZ_CACHE
    data = np.load(os.path.join(IN_DIR, f"{stage_name}.npz"))
    xyz = data["xyz"].astype(np.float64)
    XYZ_CACHE = xyz
    n = xyz.shape[0]

    remaining_xyz, remaining_idx, ground_mask, ground_reason = remove_ground(xyz)
    print(f"  ground: {ground_mask.sum()}/{n} points ({ground_reason})")

    planar_clusters, residual_idx = peel_planes(xyz, remaining_idx)
    n_before_merge = len(planar_clusters)
    planar_clusters = merge_coplanar_clusters(planar_clusters, xyz)
    print(f"  coplanar merge: {n_before_merge} -> {len(planar_clusters)} planar clusters")
    residual_clusters = classify_residual(xyz, residual_idx)

    # ground gets its own explicit observation too (class="ground") rather
    # than being left in the -1 bucket -- it's real clutter, correctly
    # excluded from the 4 structural classes, and should be visible as such
    # in observations.json, not conflated with genuinely lost/missed points.
    all_clusters = ([(c, "planar") for c in planar_clusters]
                    + [(c, "residual") for c in residual_clusters])
    if ground_mask.sum() > 0:
        all_clusters.append(({"idx": np.nonzero(ground_mask)[0], "class": "ground", "normal": None}, "ground"))

    cluster_id = np.full(n, -1, dtype=np.int32)
    observations = []
    for i, (c, method) in enumerate(all_clusters):
        cluster_id[c["idx"]] = i
        observations.append(make_observation(f"{stage_name}_{i:04d}", stage_name, c, method))

    counts = {}
    for o in observations:
        counts[o["class"]] = counts.get(o["class"], 0) + 1
    print(f"  clusters: {counts}, unclustered={int((cluster_id == -1).sum())}")

    out_dir = os.path.join(OUT_DIR, stage_name)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "observations.json"), "w", encoding="utf-8") as f:
        json.dump(observations, f, indent=2)
    np.save(os.path.join(out_dir, "point_cluster_id.npy"), cluster_id)

    return observations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=None, help="stage name (dir under data/pointclouds), default: all")
    args = parser.parse_args()

    if args.stage:
        stage_names = [args.stage]
    else:
        stage_names = sorted(
            os.path.splitext(os.path.basename(p))[0]
            for p in glob.glob(os.path.join(IN_DIR, "*.npz"))
        )

    for stage_name in stage_names:
        print(f"\n{stage_name}:")
        segment_stage(stage_name)

    print(f"\nWrote observations for {len(stage_names)} stage(s) under {OUT_DIR}/")


if __name__ == "__main__":
    main()
