"""Export a "digital twin" overlay: the complete BIM as a wireframe ghost +
each stage's actually-detected (BIM-matched) points, for the same coordinate
frame the existing mesh view uses, so the two align pixel-for-pixel in the
viewer.

Reuses p4_match_bim.py's registration + margin-gated assignment pipeline
directly -- these ARE the confirmed-built points (post-ICP, in the BIM world
frame), free of ground/tree/noise clutter since those never pass assignment.

Centering must match export_bim_viewer_data.py's mesh exactly (mean of all
element vertices, same axis remap x,z,y) or the overlay won't line up with
the wireframe. Cached to disk (deterministic, same every run) so 16
per-stage subprocess invocations don't each pay full IFC mesh extraction.

Run per-stage (reliable pattern -- a single long-running process doing all
16 in one go was observed to eventually stall/take far longer than the
per-stage total, same class of issue as p4_match_bim.py's batch mode):
  for f in data/pointclouds/*.npz; do
    stage=$(basename "$f" .npz)
    .venv/Scripts/python.exe src/export_overlay_viewer.py --stage "$stage" >> log.txt 2>&1
  done
"""
import argparse
import base64
import colorsys
import hashlib
import json
import os
import sys

import numpy as np
import pandas as pd
import ifcopenshell
import ifcopenshell.geom

sys.path.insert(0, os.path.dirname(__file__))
import p4_match_bim as m  # noqa: E402

IFC_PATH = m.IFC_PATH
CSV_PATH = m.CSV_PATH
MAIN_TABLE = "results/main_table.csv"
OUT_JS = "results/pointcloud_viewer_data.js"
CENTER_CACHE = "data/bim/bim_mesh_center.npy"
POINTS_TARGET = 60_000
POINT_COLOR = np.array([239, 131, 84])  # accent orange -- "confirmed built"


def gid_to_color(gid):
    """Deterministic, well-spread hue per GlobalId -- same element always
    gets the same color across stages/viewers, without needing a registry."""
    h = int(hashlib.md5(gid.encode()).hexdigest(), 16)
    hue = (h % 360) / 360.0
    r, g, b = colorsys.hls_to_rgb(hue, 0.58, 0.62)
    return [int(r * 255), int(g * 255), int(b * 255)]


def storey_short(storey):
    if "fundering" in storey:
        return "Foundation"
    return storey.replace(" begane grond", " GF") \
        .replace(" eerste verdieping", " 1F").replace(" tweede verdieping", " 2F") \
        .replace(" derde verdieping", " 3F").replace(" dak", " Roof").strip()


def get_mesh_center():
    if os.path.exists(CENTER_CACHE):
        return np.load(CENTER_CACHE)
    print("Computing shared mesh center (once, cached after)...")
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    all_elements = m.load_all_elements(f, settings)
    all_verts = np.concatenate([e["verts"] for e in all_elements.values()], axis=0)
    center = all_verts.mean(axis=0)
    os.makedirs(os.path.dirname(CENTER_CACHE), exist_ok=True)
    np.save(CENTER_CACHE, center)
    return center


def process_stage(stage_name, i, mesh_center, tree, icp_target, tree_gid, bim_df, main_table):
    matches, xyz_reg, assigned_gid = m.match_stage_cached(stage_name, tree, icp_target, tree_gid, bim_df)
    assigned_mask = assigned_gid != ""
    pts = xyz_reg[assigned_mask]
    gids = assigned_gid[assigned_mask]
    n_full = pts.shape[0]

    if n_full > POINTS_TARGET:
        idx = np.random.default_rng(0).choice(n_full, size=POINTS_TARGET, replace=False)
        pts = pts[idx]
        gids = gids[idx]

    pts_c = pts - mesh_center
    three_xyz = np.stack([pts_c[:, 0], pts_c[:, 2], pts_c[:, 1]], axis=1).astype(np.float32)
    colors = np.tile(POINT_COLOR, (three_xyz.shape[0], 1)).astype(np.uint8)

    gid_color_cache = {}
    colors_instance = np.empty_like(colors)
    for j, gid in enumerate(gids):
        color = gid_color_cache.get(gid)
        if color is None:
            color = gid_to_color(gid)
            gid_color_cache[gid] = color
        colors_instance[j] = color

    scale = float(np.abs(three_xyz).max()) / 32000.0 if three_xyz.size else 1.0
    scale = max(scale, 1e-6)
    pos_i16 = np.clip(np.round(three_xyz / scale), -32000, 32000).astype(np.int16)
    pos_b64 = base64.b64encode(pos_i16.tobytes()).decode("ascii")
    col_b64 = base64.b64encode(colors.tobytes()).decode("ascii")
    col_instance_b64 = base64.b64encode(colors_instance.tobytes()).decode("ascii")

    extent = three_xyz.max(axis=0) - three_xyz.min(axis=0) if three_xyz.size else np.zeros(3)
    radius = float(np.linalg.norm(extent) / 2)

    manifest = json.load(open(os.path.join("data/states", stage_name, "manifest.json"), encoding="utf-8"))
    progress_pct = float(main_table.loc[stage_name, "progress_est_count"])
    progress_gt = float(main_table.loc[stage_name, "progress_gt_count"])

    label = f"OVERLAY: {i:02d} {storey_short(manifest['storey_milestone'])} - {manifest['phase']}"
    entry = {
        "label": label,
        "type": "overlay",
        "nPoints": int(three_xyz.shape[0]),
        "nPointsFull": n_full,
        "nOutliersDropped": 0,
        "extent": [float(v) for v in extent],
        "radius": radius,
        "coreRadius": radius * 0.9,
        "posScale": scale,
        "posQuant": "i16",
        "pos": pos_b64,
        "col": col_b64,
        "colInstance": col_instance_b64,
        "nInstances": len(gid_color_cache),
        "progressPct": round(progress_pct, 1),
        "progressGtPct": round(progress_gt, 1),
    }
    print(f"  packed {label}: {three_xyz.shape[0]} pts across {len(gid_color_cache)} elements, "
          f"progress~{progress_pct:.1f}% (gt {progress_gt:.1f}%)")
    return entry


def save_entry(entry):
    with open(OUT_JS, encoding="utf-8") as fh:
        txt = fh.read()
    payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))
    payload["weeks"] = [w for w in payload["weeks"] if w["label"] != entry["label"]] + [entry]
    with open(OUT_JS, "w", encoding="utf-8") as fh:
        fh.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, fh)
        fh.write(";\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", default=None)
    args = parser.parse_args()

    mesh_center = get_mesh_center()
    print(f"mesh_center = {mesh_center}")

    bim_df = pd.read_csv(CSV_PATH)
    tree, sample_xyz, tree_gid, icp_target = m.build_bim_sample_tree()
    main_table = pd.read_csv(MAIN_TABLE).set_index("stage")

    import glob
    stage_names = sorted(
        os.path.splitext(os.path.basename(p))[0]
        for p in glob.glob(os.path.join(m.POINTCLOUD_DIR, "*.npz"))
    )

    targets = [args.stage] if args.stage else stage_names
    for stage_name in targets:
        i = stage_names.index(stage_name)
        print(f"\n{stage_name}:")
        entry = process_stage(stage_name, i, mesh_center, tree, icp_target, tree_gid, bim_df, main_table)
        save_entry(entry)

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
