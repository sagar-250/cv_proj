"""Phase 3 -- raycast-sample a point cloud for each Phase 2 manifest state.

Reads data/states/{stage}/manifest.json (kept_ids fully built, partial_ids
height-clipped to their completion_fraction, everything else absent) and
raycasts it exactly as before: exterior drone orbit + interior walkthrough,
Fibonacci-sphere interior rays (no grid-aliasing rings), realistic-config
degradation (per-element pose jitter, whole-scene registration jitter,
sensor noise, dropout), fixed ground + trees shared across all stages.

This replaces the old flat-fraction p2_p3_simulate_stages.py: stages now
come from real milestone/manifest ground truth, not an arbitrary cumulative
percentage.

Run: .venv/Scripts/python.exe src/p3_render.py
Output: results/pointcloud_viewer_data.js (BIM group), one entry per stage.
"""
import glob
import json
import os
import sys

import numpy as np
import ifcopenshell
import ifcopenshell.geom

sys.path.insert(0, os.path.dirname(__file__))
import p2_p3_simulate_stages as sim  # noqa: E402
from p2_p3_simulate_stages import (  # noqa: E402
    jitter_element, clip_partial_height, make_o3d_scene,
    exterior_viewpoints, interior_viewpoints, raycast, make_ground, make_trees,
    apply_rigid_jitter, pack_entry, CLASS_COLOR,
    NOISE_SIGMA, DROPOUT_FRAC, ELEMENT_JITTER_T, ELEMENT_JITTER_DEG,
    REG_JITTER_T, REG_JITTER_DEG,
)

IFC_PATH = "data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc"
STATES_DIR = "data/states"
OUT_JS = "results/pointcloud_viewer_data.js"
TARGET_POINTS_OUT = 55_000
sim.TARGET_POINTS_OUT = TARGET_POINTS_OUT  # pack_entry reads this from its own module globals
SEED = 0


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
            elements[el.GlobalId] = {
                "global_id": el.GlobalId, "ifc_type": ifc_type,
                "verts": verts, "faces": faces,
                "z_min": float(verts[:, 2].min()), "z_max": float(verts[:, 2].max()),
            }
    return elements


def stage_elements_from_manifest(manifest, all_elements, rng):
    partial_frac = {p["global_id"]: p["completion_fraction"] for p in manifest["partial_ids"]}
    out = []
    for gid in manifest["kept_ids"]:
        if gid in all_elements:
            out.append(dict(all_elements[gid]))
    for gid, frac in partial_frac.items():
        if gid not in all_elements:
            continue
        clipped = clip_partial_height(all_elements[gid], frac)
        if clipped is not None:
            out.append(clipped)
    return [jitter_element(e, rng, ELEMENT_JITTER_T, ELEMENT_JITTER_DEG) for e in out]


def run_stage(stage_elements, rng, ground_trees):
    scene, tri_class, tri_gid, verts = make_o3d_scene(stage_elements)
    center = verts.mean(axis=0)
    footprint_radius = float(np.linalg.norm(verts[:, :2] - center[:2], axis=1).max())
    height_range = (float(verts[:, 2].min()), float(verts[:, 2].max()))

    viewpoints = exterior_viewpoints(center, footprint_radius, height_range)
    viewpoints += interior_viewpoints(stage_elements, center, footprint_radius)
    pts, classes, gids = raycast(scene, tri_class, tri_gid, viewpoints, center, rng)

    pts = pts + rng.normal(0, NOISE_SIGMA, pts.shape)
    keep = rng.random(pts.shape[0]) > DROPOUT_FRAC
    pts, classes, gids = pts[keep], classes[keep], gids[keep]

    colors = np.array([CLASS_COLOR[c] for c in classes], dtype=np.uint8)
    n_structure = pts.shape[0]

    ground_pts, ground_col, tree_pts, tree_col = ground_trees
    ground_classes = np.full(ground_pts.shape[0], "ground", dtype=object)
    tree_classes = np.full(tree_pts.shape[0], "tree", dtype=object)
    clutter_gids = np.full(ground_pts.shape[0] + tree_pts.shape[0], "", dtype=object)

    all_pts = np.concatenate([pts, ground_pts, tree_pts], axis=0)
    all_col = np.concatenate([colors, ground_col, tree_col], axis=0)
    all_gt_class = np.concatenate([classes, ground_classes, tree_classes], axis=0)
    all_gt_gid = np.concatenate([gids, clutter_gids], axis=0)
    all_pts = apply_rigid_jitter(all_pts, rng, REG_JITTER_T, REG_JITTER_DEG)

    return all_pts, all_col, all_gt_class, all_gt_gid, n_structure, center, footprint_radius


def save_stage_pointcloud(stage_name, xyz, rgb, gt_class, gt_global_id, out_dir="data/pointclouds"):
    """Plain float32 xyz in the original BIM world frame (same frame as
    bim_complete.csv), NOT the viewer's centered/axis-swapped/quantized one --
    this is what a segmentation script should load. gt_class/gt_global_id are
    held-out ground truth for evaluation only; never feed them to a
    segmentation algorithm."""
    os.makedirs(out_dir, exist_ok=True)
    np.savez_compressed(
        os.path.join(out_dir, f"{stage_name}.npz"),
        xyz=xyz.astype(np.float32),
        rgb=rgb.astype(np.uint8),
        gt_class=np.asarray(gt_class, dtype="<U16"),
        gt_global_id=np.asarray(gt_global_id, dtype="<U24"),
    )


def storey_short(storey):
    if "fundering" in storey:
        return "Foundation"
    return storey.replace(" begane grond", " GF") \
        .replace(" eerste verdieping", " 1F").replace(" tweede verdieping", " 2F") \
        .replace(" derde verdieping", " 3F").replace(" dak", " Roof").strip()


def main():
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    print("Extracting full-building element geometry (once)...")
    all_elements = load_all_elements(f, settings)
    print(f"{len(all_elements)} elements available for raycasting")

    all_verts_full = np.concatenate([e["verts"] for e in all_elements.values()], axis=0)
    full_center = all_verts_full.mean(axis=0)
    full_radius = float(np.linalg.norm(all_verts_full[:, :2] - full_center[:2], axis=1).max())
    base_z = float(all_verts_full[:, 2].min())
    clutter_rng = np.random.default_rng(SEED + 999)
    ground_trees = (*make_ground(full_center, full_radius, base_z, clutter_rng),
                    *make_trees(full_center, full_radius, base_z, clutter_rng))
    ground_trees = (ground_trees[0], ground_trees[1], ground_trees[2], ground_trees[3])

    stage_dirs = sorted(glob.glob(os.path.join(STATES_DIR, "*")))
    entries = []
    for stage_dir in stage_dirs:
        with open(os.path.join(stage_dir, "manifest.json"), encoding="utf-8") as fh:
            manifest = json.load(fh)
        i = manifest["stage_index"]
        rng = np.random.default_rng(SEED + 1000 + i)

        stage_elements = stage_elements_from_manifest(manifest, all_elements, rng)
        print(f"\nStage {i:02d} [{manifest['storey_milestone']} / {manifest['phase']}]: "
              f"{len(stage_elements)} elements ({manifest['n_fully_built']} full, "
              f"{manifest['n_partial']} partial), raycasting...")

        all_pts, all_col, all_gt_class, all_gt_gid, n_structure, center, fr = run_stage(
            stage_elements, rng, ground_trees)
        n_full = all_pts.shape[0]

        stage_name = os.path.basename(stage_dir)
        save_stage_pointcloud(stage_name, all_pts, all_col, all_gt_class, all_gt_gid)

        label = f"BIM: {i:02d} {storey_short(manifest['storey_milestone'])} - {manifest['phase']}"
        entry = pack_entry(label, all_pts, all_col, n_full, center, fr * 1.15)
        entry["stageIndex"] = i
        entries.append(entry)
        print(f"  packed: {entry['nPoints']} pts (structure {n_structure}), "
              f"saved data/pointclouds/{stage_name}.npz")

    with open(OUT_JS, encoding="utf-8") as fh:
        txt = fh.read()
    payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))

    payload["weeks"] = [w for w in payload["weeks"] if not w["label"].startswith("BIM: 0")
                         and not w["label"].startswith("BIM: 1")
                         and not w["label"].startswith("BIM: Week")]
    payload["weeks"] += entries

    with open(OUT_JS, "w", encoding="utf-8") as fh:
        fh.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, fh)
        fh.write(";\n")

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
