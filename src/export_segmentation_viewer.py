"""Export a "segmentation after registration" visualizer: for each construction
stage, color every raw scan point by which BIM element class it was matched to
by p4_match_bim.py's ICP registration + margin-gated nearest-neighbor
assignment -- i.e. exactly the segmentation that feeds p7_score.py's progress
numbers, not a separate demo run.

Point stays "confirmed" color (by ifc_type) only if its assigned element also
cleared MIN_POINTS_DETECTED (the same bar p4_match_bim.matches uses) --
otherwise it's shown as "unmatched" even if geometrically nearest-neighbor
assigned, since that's what actually counts downstream. Ground/tree/noise
points that never pass the ASSIGN_MAX_DIST/ASSIGN_MARGIN gate are unmatched
too, so this is a direct visual read of segmentation quality: what got
correctly separated from clutter, and what's left over.

This is a separate export/artifact from export_overlay_viewer.py's progress
overlay (which only cares about "was this point confirmed built", uniform
orange) -- here every matched point keeps its class identity so segmentation
mistakes (e.g. a wall's points bleeding into a neighboring slab) are visible.

Run per-stage (same reliable pattern as export_overlay_viewer.py):
  for f in data/pointclouds/*.npz; do
    stage=$(basename "$f" .npz)
    .venv/Scripts/python.exe src/export_segmentation_viewer.py --stage "$stage" >> log.txt 2>&1
  done
"""
import argparse
import base64
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(__file__))
import p4_match_bim as m  # noqa: E402
import export_overlay_viewer as ov  # noqa: E402 -- reuses get_mesh_center, storey_short

OUT_JS = "results/segmentation_viewer_data.js"
POINTS_TARGET = 55_000

CLASS_COLOR = {
    "IfcWall": [76, 120, 168],
    "IfcSlab": [245, 133, 24],
    "IfcBeam": [84, 162, 75],
    "IfcColumn": [228, 87, 86],
}
CLASS_ORDER = ["IfcWall", "IfcSlab", "IfcBeam", "IfcColumn"]
CLASS_LABEL = {"IfcWall": "Wall", "IfcSlab": "Slab", "IfcBeam": "Beam", "IfcColumn": "Column"}
UNMATCHED_COLOR = [107, 114, 128]

# label ids for stratified downsampling: 0=unmatched, 1..4 = the confirmed classes
LABEL_ID = {"IfcWall": 1, "IfcSlab": 2, "IfcBeam": 3, "IfcColumn": 4}
MIN_PER_CLASS = 1500  # floor per non-empty class so rare elements (e.g. only 23
                       # columns in the whole building) survive downsampling --
                       # a plain uniform random sample leaves columns at ~0.01%
                       # of the cloud, i.e. 2-8 points out of 55k shown, invisible
                       # next to hundreds of thousands of wall/slab points.


def process_stage(stage_name, i, mesh_center, tree, icp_target, tree_gid, bim_df):
    matches, xyz_reg, assigned_gid = m.match_stage_cached(stage_name, tree, icp_target, tree_gid, bim_df)

    confirmed_gids = {mt["global_id"] for mt in matches}
    gid_to_type = dict(zip(bim_df["global_id"], bim_df["ifc_type"]))

    n_total = xyz_reg.shape[0]
    colors = np.tile(np.array(UNMATCHED_COLOR, dtype=np.uint8), (n_total, 1))
    colors_instance = colors.copy()
    label = np.zeros(n_total, dtype=np.int8)
    class_counts = {c: 0 for c in CLASS_ORDER}
    for gid in confirmed_gids:
        ifc_type = gid_to_type.get(gid, "")
        color = CLASS_COLOR.get(ifc_type)
        if color is None:
            continue
        mask = assigned_gid == gid
        colors[mask] = color
        colors_instance[mask] = ov.gid_to_color(gid)
        label[mask] = LABEL_ID[ifc_type]
        class_counts[ifc_type] = class_counts.get(ifc_type, 0) + int(mask.sum())
    n_unmatched = n_total - sum(class_counts.values())

    if n_total > POINTS_TARGET:
        rng = np.random.default_rng(0)
        class_idx = {c: np.nonzero(label == c)[0] for c in range(5)}
        class_idx = {c: v for c, v in class_idx.items() if v.size > 0}
        guaranteed = {c: min(v.size, MIN_PER_CLASS) for c, v in class_idx.items()}
        budget_left = max(0, POINTS_TARGET - sum(guaranteed.values()))
        extra_pool = {c: class_idx[c].size - guaranteed[c] for c in class_idx}
        total_extra_pool = sum(extra_pool.values())
        quota = {}
        for c in class_idx:
            share = int(round(budget_left * extra_pool[c] / total_extra_pool)) if total_extra_pool > 0 else 0
            quota[c] = guaranteed[c] + min(share, extra_pool[c])
        idx = np.concatenate([rng.choice(class_idx[c], size=quota[c], replace=False) for c in class_idx])
        rng.shuffle(idx)
        xyz_shown, col_shown, col_instance_shown = xyz_reg[idx], colors[idx], colors_instance[idx]
    else:
        xyz_shown, col_shown, col_instance_shown = xyz_reg, colors, colors_instance

    pts_c = xyz_shown - mesh_center
    three_xyz = np.stack([pts_c[:, 0], pts_c[:, 2], pts_c[:, 1]], axis=1).astype(np.float32)

    scale = float(np.abs(three_xyz).max()) / 32000.0 if three_xyz.size else 1.0
    scale = max(scale, 1e-6)
    pos_i16 = np.clip(np.round(three_xyz / scale), -32000, 32000).astype(np.int16)
    pos_b64 = base64.b64encode(pos_i16.tobytes()).decode("ascii")
    col_b64 = base64.b64encode(col_shown.astype(np.uint8).tobytes()).decode("ascii")
    col_instance_b64 = base64.b64encode(col_instance_shown.astype(np.uint8).tobytes()).decode("ascii")

    extent = three_xyz.max(axis=0) - three_xyz.min(axis=0) if three_xyz.size else np.zeros(3)
    radius = float(np.linalg.norm(extent) / 2)

    manifest = json.load(open(os.path.join("data/states", stage_name, "manifest.json"), encoding="utf-8"))
    label = f"SEGVIEW: {i:02d} {ov.storey_short(manifest['storey_milestone'])} - {manifest['phase']}"
    entry = {
        "label": label,
        "type": "segview",
        "nPoints": int(three_xyz.shape[0]),
        "nPointsFull": n_total,
        "nOutliersDropped": 0,
        "extent": [float(v) for v in extent],
        "radius": radius,
        "coreRadius": radius * 0.9,
        "posScale": scale,
        "posQuant": "i16",
        "pos": pos_b64,
        "col": col_b64,
        "colInstance": col_instance_b64,
        "classCounts": {CLASS_LABEL[c]: class_counts[c] for c in CLASS_ORDER},
        "nUnmatched": int(n_unmatched),
    }
    print(f"  packed {label}: {n_total} pts total, "
          f"{sum(class_counts.values())} confirmed-classified, {n_unmatched} unmatched")
    return entry


def save_entry(entry):
    if not os.path.exists(OUT_JS):
        os.makedirs(os.path.dirname(OUT_JS), exist_ok=True)
        with open(OUT_JS, "w", encoding="utf-8") as fh:
            fh.write("window.SEGVIEW_DATA = {\"weeks\": []};\n")
    with open(OUT_JS, encoding="utf-8") as fh:
        txt = fh.read()
    payload = json.loads(txt[len("window.SEGVIEW_DATA = "):].rstrip("\n;"))
    payload["weeks"] = [w for w in payload["weeks"] if w["label"] != entry["label"]] + [entry]
    with open(OUT_JS, "w", encoding="utf-8") as fh:
        fh.write("window.SEGVIEW_DATA = ")
        json.dump(payload, fh)
        fh.write(";\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=None)
    args = parser.parse_args()

    mesh_center = ov.get_mesh_center()
    print(f"mesh_center = {mesh_center}")

    bim_df = pd.read_csv(m.CSV_PATH)
    tree, sample_xyz, tree_gid, icp_target = m.build_bim_sample_tree()

    stage_names = sorted(
        os.path.splitext(os.path.basename(p))[0]
        for p in glob.glob(os.path.join(m.POINTCLOUD_DIR, "*.npz"))
    )

    targets = [args.stage] if args.stage else stage_names
    for stage_name in targets:
        i = stage_names.index(stage_name)
        print(f"\n{stage_name}:")
        entry = process_stage(stage_name, i, mesh_center, tree, icp_target, tree_gid, bim_df)
        save_entry(entry)

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
