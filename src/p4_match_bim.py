"""Phase 4 (revised) -- match point cloud to known BIM elements directly,
instead of blind segmentation.

The premise that makes this the right approach: we always have the complete
as-designed BIM before construction starts. So the task isn't "discover
unknown objects in a point cloud" (what p4_segment.py's classical RANSAC +
DBSCAN pipeline does, and does imperfectly, as its eval results show) -- it's
"given a known 3D model, work out which observed points belong to which of
its elements." That's the classical scan-vs-BIM literature (Bosché & Haas
2008/2009, cited in SPEC.md section 16), not generic segmentation.

Method:
  1. Densely sample the COMPLETE design BIM's surface once (every in-scope
     element, area-proportional sampling), each sample tagged with its
     source GlobalId. Build one KD-tree over all of it. This is reused,
     unchanged, across every stage -- it doesn't matter that most elements
     aren't built yet at an early stage; a point near an unbuilt element's
     future location is simply evidence there's nothing there yet.
  2. Per stage: the point cloud has a whole-scene registration offset baked
     in on purpose (p3_render.py: REG_JITTER, plus a GNSS- or SLAM-sized
     offset with --georef). Undo it with `--registration plan` (default,
     src/registration.py: gravity-aligned floor-plan search + robust ICP,
     handles metre/any-heading offsets and flags failures) or
     `--registration icp_old` (the original: plain ICP after keeping only
     points already within 0.5 m of the BIM; only works for offsets of a few
     cm, see results/r3_registration_summary.md). The estimate and its
     self-check are saved to registration.json; downstream steps use only
     that estimate. `--registration plan_color` (src/color_registration.py)
     also uses the scan's camera colour against the BIM's material list;
     it needs clouds from p3_render.py --color material.
  3. After alignment, nearest-neighbour query per point against the KD-tree
     with a TIGHT distance threshold (matched to the simulator's sensor
     noise + per-element jitter, not the registration jitter -- that's
     already corrected by step 2). Points beyond the threshold are
     unassigned: this is what naturally rejects ground/tree/noise, no
     separate ground-removal or clustering heuristics needed.
  4. Aggregate assigned points per GlobalId: point count, centroid, AABB,
     height-coverage fraction (vs the element's own known height, from
     bim_complete.csv) as a completion signal for partial elements.

This replaces the need for RANSAC plane-peeling, DBSCAN, and coplanar-merge
heuristics entirely for the matching step -- Phase 6 (SPEC.md's matching
cost/Hungarian assignment) becomes almost unnecessary too, since assignment
is already correct by construction (subject to registration accuracy).

Run: .venv/bin/python src/p4_match_bim.py [--stage NAME] [--registration plan|plan_color|icp_old] [--prior gnss|none]
Output: data/matches/{stage}/matches.json (one record per detected element),
        data/matches/{stage}/registration.json (estimated cloud->BIM transform + checks)
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import pandas as pd
import ifcopenshell
import ifcopenshell.geom
import open3d as o3d
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(__file__))
from p1_parse import obb_xy  # noqa: E402
import registration  # noqa: E402
import color_registration  # noqa: E402
import materials  # noqa: E402

IFC_PATH = "data/ifc/IFC Schependomlaan.ifc"
CSV_PATH = "data/bim/bim_complete.csv"
TAG = os.environ.get("RUN_TAG", "")   # e.g. RUN_TAG=interior_dart: reads data/pointclouds_<tag>, writes data/{matches,status}_<tag>
SFX = f"_{TAG}" if TAG else ""
POINTCLOUD_DIR = "data/pointclouds" + SFX
OUT_DIR = "data/matches" + SFX

SAMPLE_DENSITY = 800.0          # samples per m^2 of BIM surface area
ICP_COARSE_FILTER_DIST = 0.5    # keep only points within this of *some* BIM sample, before ICP
ICP_MAX_CORR_DIST = 0.3         # ICP's own inlier correspondence distance
ICP_TARGET_SIZE = 2_000_000     # ICP target is a random subsample, not the full ~13M-point
                                 # sample cloud -- see register_to_bim's docstring for why.
                                 # Swept 150k/500k/1M/2M: RMSE improves from 0.13 to 0.05m as
                                 # target size grows (more points = better correspondence),
                                 # while cost only grows from ~1s to ~6s/stage -- 2M is the
                                 # best quality/speed trade found, well inside ASSIGN_MAX_DIST.
ASSIGN_MAX_DIST = 0.08          # final nearest-element assignment threshold (~ sensor noise + per-element jitter)
ASSIGN_K = 8                    # neighbours examined per point for the margin check below
ASSIGN_MARGIN = 0.04            # a point is assigned only if the nearest DIFFERENT element's
                                 # sample is at least this much farther than the nearest one --
                                 # elements meet at real construction joints (wall keys into
                                 # slab, wall meets wall) where surfaces sit within mm of each
                                 # other, so a bare "nearest wins" rule produces real false
                                 # detections of not-yet-built neighbours (measured: 32 false
                                 # positives at stage 03, some with 1000+ points). This margin
                                 # eliminates them (measured: 0 false positives) at a real
                                 # recall cost (~300 -> ~196 true detections at stage 03) --
                                 # for progress comparison, a false "this is built" claim is
                                 # far more costly than a missed detection, so this trade is
                                 # the right one.
MIN_POINTS_DETECTED = 8         # an element needs at least this many assigned points to count as "detected".
                                 # Swept 3/5/8/15 against false-positive risk across 4 stages:
                                 # 15 was too conservative -- at the fully-complete reference
                                 # stage, only 73% of TRULY BUILT elements ever reach 15 points
                                 # (many legitimate elements are just sparsely covered by the
                                 # raycast), causing the Phase 7 progress estimate to
                                 # systematically UNDER-count by up to 33 percentage points at
                                 # 100% ground-truth completion. 8 recovers much of that
                                 # (measured +20 to +65 true detections per stage) while false
                                 # positives stay low (2-16 per stage, ~1-2% of detections).


def load_all_elements(f, settings):
    elements = {}
    for ifc_type in ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall"]:
        for el in f.by_type(ifc_type):
            try:
                shape = ifcopenshell.geom.create_shape(settings, el)
            except Exception:
                continue
            verts = np.array(shape.geometry.verts, dtype=np.float64).reshape(-1, 3)
            faces = np.array(shape.geometry.faces, dtype=np.int64).reshape(-1, 3)
            if verts.shape[0] == 0 or faces.shape[0] == 0:
                continue
            elements[el.GlobalId] = {"verts": verts, "faces": faces}
    return elements


def sample_triangle_mesh(verts, faces, density, rng):
    """Area-proportional random surface sampling (barycentric)."""
    v0, v1, v2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    cross = np.cross(v1 - v0, v2 - v0)
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    total_area = areas.sum()
    if total_area <= 0:
        return np.zeros((0, 3))

    n_samples = max(3, int(round(total_area * density)))
    tri_idx = rng.choice(len(faces), size=n_samples, p=areas / total_area)
    r1 = rng.random(n_samples)
    r2 = rng.random(n_samples)
    sqrt_r1 = np.sqrt(r1)
    a = 1 - sqrt_r1
    b = sqrt_r1 * (1 - r2)
    c = sqrt_r1 * r2
    pts = a[:, None] * v0[tri_idx] + b[:, None] * v1[tri_idx] + c[:, None] * v2[tri_idx]
    return pts


SAMPLE_CACHE = "data/bim/bim_surface_samples.npz"


def build_bim_sample_tree():
    rng = np.random.default_rng(0)

    if os.path.exists(SAMPLE_CACHE):
        print(f"Loading cached BIM surface samples from {SAMPLE_CACHE}...")
        cached = np.load(SAMPLE_CACHE, allow_pickle=False)
        sample_xyz, sample_gid = cached["xyz"], cached["gid"]
    else:
        print("Extracting complete BIM geometry and sampling surfaces...")
        f = ifcopenshell.open(IFC_PATH)
        settings = ifcopenshell.geom.settings()
        settings.set(settings.USE_WORLD_COORDS, True)
        elements = load_all_elements(f, settings)

        all_pts, all_gid = [], []
        for gid, e in elements.items():
            pts = sample_triangle_mesh(e["verts"], e["faces"], SAMPLE_DENSITY, rng)
            all_pts.append(pts)
            all_gid.extend([gid] * pts.shape[0])

        sample_xyz = np.concatenate(all_pts, axis=0).astype(np.float32)
        sample_gid = np.array(all_gid, dtype="<U24")
        os.makedirs(os.path.dirname(SAMPLE_CACHE), exist_ok=True)
        np.savez_compressed(SAMPLE_CACHE, xyz=sample_xyz, gid=sample_gid)
        print(f"  cached to {SAMPLE_CACHE}")

    print(f"  {len(np.unique(sample_gid))} elements, {sample_xyz.shape[0]} surface samples")
    tree = cKDTree(sample_xyz)

    icp_target = o3d.geometry.PointCloud()
    n = sample_xyz.shape[0]
    sub_idx = rng.choice(n, size=min(ICP_TARGET_SIZE, n), replace=False)
    icp_target.points = o3d.utility.Vector3dVector(sample_xyz[sub_idx])

    return tree, sample_xyz, sample_gid, icp_target


def register_to_bim(xyz, tree, icp_target):
    """ICP-correct the point cloud's whole-scene registration jitter against
    the known BIM. Coarse-filters to points already near some BIM surface so
    ground/tree clutter (no BIM correspondence at all) doesn't pull the fit
    off -- those points still get a nearest-neighbour distance to the BIM,
    it's just a large one, so they're cheap to exclude first.

    icp_target is a pre-built, DOWNSAMPLED Open3D point cloud (see
    ICP_TARGET_SIZE) -- registration_icp builds its own internal search
    structure over the target on every call regardless of caching, so using
    the full ~13M-sample cloud here (needed for the later fine-grained
    per-point assignment, via the separate scipy cKDTree) made every stage's
    ICP call take minutes. A ~150k-point random subset is still dense enough
    for a good rigid-alignment fit and is a completely different, much
    cheaper cost than the final assignment step."""
    dist, _ = tree.query(xyz, k=1, workers=-1)
    coarse_mask = dist < ICP_COARSE_FILTER_DIST
    n_coarse = int(coarse_mask.sum())
    if n_coarse < 100:
        print(f"  WARNING: only {n_coarse} points near the BIM before ICP -- skipping registration")
        return np.eye(4)

    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(xyz[coarse_mask])

    result = o3d.pipelines.registration.registration_icp(
        source, icp_target, ICP_MAX_CORR_DIST, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )
    print(f"  ICP: fitness={result.fitness:.3f}, inlier_rmse={result.inlier_rmse:.4f} "
          f"(from {n_coarse} coarse-filtered points)")
    return result.transformation


def apply_transform(xyz, T):
    xyz_h = np.concatenate([xyz, np.ones((xyz.shape[0], 1))], axis=1)
    return (xyz_h @ T.T)[:, :3]


def match_stage(stage_name, tree, icp_target, tree_gid, bim_df, reg_target=None, method="icp_old",
                prior="gnss"):
    data = np.load(os.path.join(POINTCLOUD_DIR, f"{stage_name}.npz"))
    xyz = data["xyz"].astype(np.float64)

    if method == "plan":
        T, info = registration.register(xyz, reg_target, method="plan", prior=prior,
                                        with_scale=(prior == "gnss"), verbose=True)
    elif method == "plan_color":
        # "class" colour would be ground truth (the IFC class), so refuse it
        assert "rgb_source" in data and str(data["rgb_source"]) == "camera", \
            "plan_color needs p3_render.py --color material clouds"
        T, info = color_registration.register(xyz, data["rgb"], reg_target, prior=prior,
                                              with_scale=(prior == "gnss"), verbose=True)
    else:
        T, info = register_to_bim(xyz, tree, icp_target), {"ok": True, "failed_checks": ""}
    if not info["ok"]:
        print(f"  WARNING: registration failed its own checks ({info['failed_checks']}) -- "
              f"stage results are not trustworthy")
    match_stage.last_registration = {"method": method, "prior": prior, "T_cloud_to_bim": T.tolist(), **info}
    xyz_reg = apply_transform(xyz, T)

    dist_k, idx_k = tree.query(xyz_reg, k=ASSIGN_K, workers=-1)
    gid_k = tree_gid[idx_k]
    nearest_gid, nearest_dist = gid_k[:, 0], dist_k[:, 0]

    # distance to the nearest sample belonging to a DIFFERENT element than
    # the overall-nearest one -- see ASSIGN_MARGIN's comment for why this
    # matters (construction-joint ambiguity, not sensor noise)
    diff_mask = gid_k != nearest_gid[:, None]
    second_dist = np.full(xyz.shape[0], np.inf)
    for col in range(1, ASSIGN_K):
        take = diff_mask[:, col] & np.isinf(second_dist)
        second_dist[take] = dist_k[take, col]

    ok = (nearest_dist < ASSIGN_MAX_DIST) & (second_dist - nearest_dist > ASSIGN_MARGIN)
    assigned_gid = np.where(ok, nearest_gid, "")

    n_assigned = int(ok.sum())
    print(f"  assigned {n_assigned}/{xyz.shape[0]} points to a known BIM element "
          f"(threshold {ASSIGN_MAX_DIST}m, margin {ASSIGN_MARGIN}m)")

    matches = []
    unique_gids = np.unique(assigned_gid[assigned_gid != ""])
    for gid in unique_gids:
        pt_idx = np.nonzero(assigned_gid == gid)[0]
        if pt_idx.shape[0] < MIN_POINTS_DETECTED:
            continue
        pts = xyz_reg[pt_idx]
        row = bim_df.loc[bim_df["global_id"] == gid]
        if row.empty:
            continue
        row = row.iloc[0]

        aabb_min, aabb_max = pts.min(axis=0), pts.max(axis=0)
        l, w, rot_z = obb_xy(pts[:, :2])
        h_observed = float(aabb_max[2] - aabb_min[2])
        h_true = float(row["aabb_max_z"] - row["aabb_min_z"])
        height_coverage = min(1.0, h_observed / h_true) if h_true > 1e-6 else 1.0

        matches.append({
            "global_id": gid,
            "ifc_type": row["ifc_type"],
            "point_count": int(pt_idx.shape[0]),
            "centroid_x": float(pts.mean(axis=0)[0]), "centroid_y": float(pts.mean(axis=0)[1]),
            "centroid_z": float(pts.mean(axis=0)[2]),
            "aabb_min_x": float(aabb_min[0]), "aabb_min_y": float(aabb_min[1]), "aabb_min_z": float(aabb_min[2]),
            "aabb_max_x": float(aabb_max[0]), "aabb_max_y": float(aabb_max[1]), "aabb_max_z": float(aabb_max[2]),
            "obb_dims_l": l, "obb_dims_w": w, "obb_rot_z": rot_z,
            "height_coverage": height_coverage,
        })

    return matches, xyz_reg, assigned_gid


MATCH_CACHE_DIR = "data/matches_cache" + SFX


def match_stage_cached(stage_name, tree, icp_target, tree_gid, bim_df):
    """Same result as match_stage(), but the expensive part (ICP registration
    over the full raycast cloud, ~20-90s/stage) is cached to disk after the
    first call -- viewer-export scripts that only tweak post-assignment
    presentation (coloring, downsampling) can then iterate in ~1s/stage
    instead of paying for a fresh registration every time."""
    cache_path = os.path.join(MATCH_CACHE_DIR, f"{stage_name}.npz")
    if os.path.exists(cache_path):
        data = np.load(cache_path, allow_pickle=False)
        matches = json.loads(data["matches_json"].item())
        return matches, data["xyz_reg"], data["assigned_gid"]

    matches, xyz_reg, assigned_gid = match_stage(stage_name, tree, icp_target, tree_gid, bim_df)
    os.makedirs(MATCH_CACHE_DIR, exist_ok=True)
    np.savez_compressed(
        cache_path,
        xyz_reg=xyz_reg,
        assigned_gid=assigned_gid,
        matches_json=np.array(json.dumps(matches)),
    )
    return matches, xyz_reg, assigned_gid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=None)
    parser.add_argument("--registration", choices=["plan", "plan_color", "icp_old"], default="plan")
    parser.add_argument("--prior", choices=["gnss", "none"], default="gnss",
                        help="gnss: drone photogrammetry (metres off, scale ~1%%); none: SLAM in its own frame")
    args = parser.parse_args()

    bim_df = pd.read_csv(CSV_PATH)
    tree, sample_xyz, tree_gid, icp_target = build_bim_sample_tree()
    print(f"Sample tree memory: xyz {sample_xyz.nbytes/1e6:.1f}MB, gid {tree_gid.nbytes/1e6:.1f}MB")
    reg_target = None
    if args.registration == "plan":
        reg_target = registration.BimTarget(sample_xyz.astype(np.float64))
    elif args.registration == "plan_color":
        reg_target = color_registration.ColorTarget(sample_xyz.astype(np.float64), tree_gid,
                                                    materials.load_categories())

    if args.stage:
        stage_names = [args.stage]
    else:
        stage_names = sorted(
            os.path.splitext(os.path.basename(p))[0]
            for p in glob.glob(os.path.join(POINTCLOUD_DIR, "*.npz"))
        )

    for stage_name in stage_names:
        print(f"\n{stage_name}:")
        matches, xyz_reg, assigned_gid = match_stage(stage_name, tree, icp_target, tree_gid, bim_df,
                                                     reg_target, args.registration, args.prior)
        out_dir = os.path.join(OUT_DIR, stage_name)
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "matches.json"), "w", encoding="utf-8") as f:
            json.dump(matches, f, indent=2)
        with open(os.path.join(out_dir, "registration.json"), "w", encoding="utf-8") as f:
            json.dump(match_stage.last_registration, f, indent=2)
        print(f"  {len(matches)} elements detected (>= {MIN_POINTS_DETECTED} points each)")

    print(f"\nWrote matches for {len(stage_names)} stage(s) under {OUT_DIR}/")


if __name__ == "__main__":
    main()
