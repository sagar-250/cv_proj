"""Phase 4b (ROADMAP R4) -- per-stage "could we have seen it?" test for every
element that was NOT detected.

Why: an undetected element is either not built, or built but not seen
(occluded, outside every camera's view, too thin to collect points). The
old p7_score.py answered this with one fixed mask -- "was it detected at
the final, fully-built stage?" -- which ignores that what is visible
changes from stage to stage (an unbuilt upper floor hides nothing; an
interior wall is only reachable once that floor's walkthrough happens).

Method (uses only what a real survey would also have: the BIM, the camera
poses, and our own detections -- never the ground-truth manifest). Camera
poses come in the survey frame and are mapped into the BIM frame with the
ESTIMATED registration from p4_match_bim.py (registration.json), not the
true offset:
  1. Occluders = the elements we DETECTED this stage (our belief of what is
     built). Cast every camera ray against them -> distance to first hit.
  2. Candidates = every element we did NOT detect. Cast the same rays
     against all candidates, listing every intersection along each ray
     (not just the first), and keep hits closer than the occluder hit.
     Each candidate's kept-hit count = how many points it WOULD have
     produced if it were built, assuming nothing else unbuilt was in the
     way.
  3. Drop the would-be points that p4_match_bim.py could never assign:
     those within ASSIGN_MARGIN of another element's surface (joints), the
     same rule matching applies. Without this, thin walls/columns/beams
     pressed against neighbours were "expected" to produce dozens of points
     they never could (measured: 276 built elements wrongly called
     not_started across all stages at the 30-point threshold).
  4. Scale by the sensor dropout rate. If that expected count is high
     enough, absence of points is real evidence -> not built. Otherwise
     -> unknown (excluded from progress).
  5. The same count for every element we DID detect: rays whose first hit
     among the detected elements is that element (assignable, scaled by
     dropout). p7_score.py uses it to throw out "detections" that got only
     a small fraction of the points a really built, clearly visible
     element would give -- with a noisy sensor, stray points from a
     neighbouring slab land inside an unbuilt wall's volume and pass the
     distance test (measured with --sensor realistic: 25 unbuilt elements
     "detected" at stage 05 from 8-40 slab points each).

Run: .venv/bin/python src/p4_visibility.py [--stage NAME]
     (after p3_render.py and p4_match_bim.py)
Output: data/matches/{stage}/visibility.json           {undetected global_id: expected_points}
        data/matches/{stage}/visibility_detected.json  {detected global_id: expected_points}
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import open3d as o3d

sys.path.insert(0, os.path.dirname(__file__))
import p2_p3_simulate_stages as sim  # noqa: E402
from p4_match_bim import ASSIGN_K, ASSIGN_MARGIN, build_bim_sample_tree  # noqa: E402

IFC_PATH = "data/ifc/IFC Schependomlaan.ifc"
MESH_CACHE = "data/bim/bim_meshes.npz"
POINTCLOUD_DIR = "data/pointclouds"
MATCHES_DIR = "data/matches"
SEED = 0


def load_meshes():
    """All in-scope element meshes in BIM world coords, cached after first run."""
    if os.path.exists(MESH_CACHE):
        d = np.load(MESH_CACHE)
        return d["verts"], d["faces"], d["tri_gid"]
    import ifcopenshell
    import ifcopenshell.geom
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    verts, faces, tri_gid, off = [], [], [], 0
    for ifc_type in ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall"]:
        for el in f.by_type(ifc_type):
            try:
                shape = ifcopenshell.geom.create_shape(settings, el)
            except Exception:
                continue
            v = np.array(shape.geometry.verts, dtype=np.float64).reshape(-1, 3)
            fc = np.array(shape.geometry.faces, dtype=np.int64).reshape(-1, 3)
            if v.shape[0] == 0 or fc.shape[0] == 0:
                continue
            verts.append(v)
            faces.append(fc + off)
            tri_gid.extend([el.GlobalId] * fc.shape[0])
            off += v.shape[0]
    verts = np.concatenate(verts).astype(np.float32)
    faces = np.concatenate(faces).astype(np.uint32)
    tri_gid = np.array(tri_gid, dtype="<U24")
    np.savez_compressed(MESH_CACHE, verts=verts, faces=faces, tri_gid=tri_gid)
    return verts, faces, tri_gid


def scene_from(verts, faces, tri_mask):
    """Raycasting scene of the triangles in tri_mask; returns (scene, original tri index per new tri)."""
    tri_idx = np.nonzero(tri_mask)[0]
    scene = o3d.t.geometry.RaycastingScene()
    if tri_idx.size:
        mesh = o3d.t.geometry.TriangleMesh()
        mesh.vertex.positions = o3d.core.Tensor(verts)
        mesh.triangle.indices = o3d.core.Tensor(faces[tri_idx])
        scene.add_triangles(mesh)
    return scene, tri_idx


def camera_rays(cam_eyes, cam_kinds, cam_target, rng):
    """Same ray pattern the survey used: pinhole frames aimed at the target,
    omnidirectional scans indoors."""
    out = []
    for eye, kind in zip(cam_eyes, cam_kinds):
        if kind == "pinhole":
            o, d = sim.pinhole_rays(eye, cam_target, sim.EXT_FOV_DEG, sim.EXT_GRID)
        else:
            o, d = sim.spherical_rays(eye, sim.INTERIOR_RAY_GRID, rng)
        out.append(np.concatenate([o, d], axis=1))
    return np.concatenate(out).astype(np.float32)


def assignable(pts, gids, tree, tree_gid):
    """Would p4_match_bim.py's margin rule assign a point at pts to gids?"""
    ok = np.zeros(pts.shape[0], dtype=bool)
    for lo in range(0, pts.shape[0], 500_000):
        hi = lo + 500_000
        dist_k, idx_k = tree.query(pts[lo:hi], k=ASSIGN_K, workers=-1)
        gid_k = tree_gid[idx_k]
        own = gid_k[:, 0] == gids[lo:hi]
        other = gid_k != gids[lo:hi, None]
        first_other = np.where(other.any(axis=1), other.argmax(axis=1), -1)
        d_other = np.where(first_other >= 0, dist_k[np.arange(len(first_other)), first_other], np.inf)
        ok[lo:hi] = own & (d_other - dist_k[:, 0] > ASSIGN_MARGIN)
    return ok


def expected_points(stage, verts, faces, tri_gid, tree, tree_gid):
    d = np.load(os.path.join(POINTCLOUD_DIR, f"{stage}.npz"))
    matches = json.load(open(os.path.join(MATCHES_DIR, stage, "matches.json"), encoding="utf-8"))
    detected = {m["global_id"] for m in matches}
    is_det = np.isin(tri_gid, list(detected))

    reg = json.load(open(os.path.join(MATCHES_DIR, stage, "registration.json"), encoding="utf-8"))
    T = np.array(reg["T_cloud_to_bim"])
    eyes = d["cam_eyes"] @ T[:3, :3].T + T[:3, 3]
    target = T[:3, :3] @ d["cam_target"] + T[:3, 3]
    rays = camera_rays(eyes, d["cam_kinds"], target, np.random.default_rng(SEED))
    occ_scene, occ_tri = scene_from(verts, faces, is_det)
    occ_hit = occ_scene.cast_rays(o3d.core.Tensor(rays))
    t_occ = occ_hit["t_hit"].numpy()

    hit = np.isfinite(t_occ)
    det_gid = tri_gid[occ_tri[occ_hit["primitive_ids"].numpy()[hit]]]
    det_ok = assignable(rays[hit, :3] + t_occ[hit, None] * rays[hit, 3:], det_gid, tree, tree_gid)
    g_det, n_det = np.unique(det_gid[det_ok], return_counts=True)
    det_counts = {g: float(c) * (1 - sim.DROPOUT_FRAC) for g, c in zip(g_det, n_det)}
    for g in detected:
        det_counts.setdefault(g, 0.0)

    cand_scene, cand_tri = scene_from(verts, faces, ~is_det)
    counts = {}
    if cand_tri.size:
        hits = cand_scene.list_intersections(o3d.core.Tensor(rays))
        splits = hits["ray_splits"].numpy()
        ray_id = np.repeat(np.arange(len(splits) - 1), np.diff(splits))
        t = hits["t_hit"].numpy()
        prim = hits["primitive_ids"].numpy()
        keep = t < t_occ[ray_id]
        ray_id, t, gids = ray_id[keep], t[keep], tri_gid[cand_tri[prim[keep]]]
        # one ray can cross the same element twice (front + back face): only the
        # first crossing would return a point
        uniq_g, g_idx = np.unique(gids, return_inverse=True)
        order = np.lexsort((t, g_idx, ray_id))
        ray_id, t, g_idx = ray_id[order], t[order], g_idx[order]
        first = np.ones(ray_id.size, dtype=bool)
        first[1:] = (ray_id[1:] != ray_id[:-1]) | (g_idx[1:] != g_idx[:-1])
        ray_id, t, g_idx = ray_id[first], t[first], g_idx[first]
        pts = rays[ray_id, :3] + t[:, None] * rays[ray_id, 3:]
        ok = assignable(pts, uniq_g[g_idx], tree, tree_gid)
        per = np.bincount(g_idx[ok], minlength=uniq_g.size)
        counts = {g: float(c) * (1 - sim.DROPOUT_FRAC) for g, c in zip(uniq_g, per)}
    for g in np.unique(tri_gid[~is_det]):
        counts.setdefault(g, 0.0)
    return counts, det_counts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default=None)
    args = ap.parse_args()
    verts, faces, tri_gid = load_meshes()
    tree, _xyz, tree_gid, _icp = build_bim_sample_tree()
    stages = [args.stage] if args.stage else sorted(
        os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(POINTCLOUD_DIR, "*.npz")))
    for stage in stages:
        counts, det_counts = expected_points(stage, verts, faces, tri_gid, tree, tree_gid)
        with open(os.path.join(MATCHES_DIR, stage, "visibility.json"), "w", encoding="utf-8") as f:
            json.dump(counts, f)
        with open(os.path.join(MATCHES_DIR, stage, "visibility_detected.json"), "w", encoding="utf-8") as f:
            json.dump(det_counts, f)
        vals = np.array(list(counts.values()))
        print(f"{stage}: {len(det_counts)} detected; {len(vals)} undetected, of which "
              f"{int((vals >= 30).sum())} would have had >=30 points if built")


if __name__ == "__main__":
    main()
