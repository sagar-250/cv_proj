"""Downsample all 5 weekly PLY point clouds and emit compact base64 blobs
for the Three.js viewer artifact (src/pointcloud_viewer.html generates from
this output; see results/pointcloud_viewer_data.js).

Important: each weekly SfM reconstruction has its OWN arbitrary coordinate
frame (bundle-adjustment gauge freedom) -- they are NOT pre-registered to the
IFC world frame or to each other. That registration is SPEC.md's Phase 9
(manual correspondences + ICP), not done here. So each week is centered and
filtered independently, and viewed one at a time.

Two real data-quality issues were found and are corrected here (see
results/p0b_pointcloud_notes.md for the investigation):

1. Axis orientation: the ground/roof flat-surface cluster sits at the *low*
   end of the raw z range and the raw normal convention is inconsistent, but
   comparing against known ground-truth colors (grass, walls) confirmed the
   PLY's raw +z is inverted relative to true "up". So the Three.js remap
   negates z: three(x,y,z) = ifc(x,-z,y).

2. Weeks 27-30 capture a much wider area than the building (streets,
   neighbouring plots, sky) -- their point count is dominated by that
   context, not by the structure. A single global percentile-clip (used for
   week 26, which is already a tight, clean crop) is not enough: the
   structure gets diluted to invisibility under uniform random sampling.
   Fixed with a k-NN local-density filter (scipy cKDTree, k=8, keep the
   densest 65%) which isolates the coherent, densely-reconstructed cluster
   (the building) from the sparse surrounding capture -- confirmed visually
   to recover a recognisable structural frame in Week 28.

Run: .venv/Scripts/python.exe src/export_pointcloud_viewer_data.py
"""
import base64
import glob
import json
import os

import numpy as np
from scipy.spatial import cKDTree

WEEKS = ["Week 26", "Week 27", "Week 28", "Week 29", "Week 30"]
BASE = "data/raw/schependomlaan/Point Clouds/{week}/PLY"
POINTS_PER_WEEK = 38_000
LOAD_CAP = 1_500_000  # cap raw points read for kNN-density cost; random order in file, so this is an unbiased subsample
DENSITY_KEEP_FRAC = 0.65
OUT_JS = "results/pointcloud_viewer_data.js"


def find_ply(week):
    matches = glob.glob(f"{BASE.format(week=week)}/*.ply")
    if not matches:
        raise FileNotFoundError(week)
    return matches[0]


def load_vertices(path, cap=None):
    n_vertex = None
    header_lines = 0
    with open(path, "r", errors="ignore") as f:
        for line in f:
            header_lines += 1
            if line.startswith("element vertex"):
                n_vertex = int(line.split()[-1])
            if line.strip() == "end_header":
                break
    n_read = min(n_vertex, cap) if cap else n_vertex
    data = np.loadtxt(path, skiprows=header_lines, max_rows=n_read, usecols=(0, 1, 2, 6, 7, 8))
    xyz = data[:, :3].astype(np.float32)
    rgb = data[:, 3:6].astype(np.uint8)
    return xyz, rgb, n_vertex


def density_filter(xyz, keep_frac, k=8):
    """Keep the keep_frac fraction of points with the smallest mean distance
    to their k nearest neighbours -- i.e. the densely-reconstructed coherent
    cluster(s), dropping sparse/isolated stray points."""
    tree = cKDTree(xyz)
    dist, _ = tree.query(xyz, k=k + 1, workers=-1)
    mean_knn = dist[:, 1:].mean(axis=1)
    thresh = np.percentile(mean_knn, keep_frac * 100)
    return mean_knn <= thresh


def main():
    weeks_out = []

    for week in WEEKS:
        path = find_ply(week)
        xyz_raw, rgb_raw, n_total = load_vertices(path, cap=LOAD_CAP)
        n_loaded = xyz_raw.shape[0]

        # coarse pre-clip so a handful of extreme-kilometre stray points don't
        # blow up the KD-tree's bounding volume / neighbour distances
        lo = np.percentile(xyz_raw, 1, axis=0)
        hi = np.percentile(xyz_raw, 99, axis=0)
        coarse = np.all((xyz_raw >= lo) & (xyz_raw <= hi), axis=1)
        xyz_c0, rgb_c0 = xyz_raw[coarse], rgb_raw[coarse]

        dense_mask = density_filter(xyz_c0, DENSITY_KEEP_FRAC)
        xyz_d, rgb_d = xyz_c0[dense_mask], rgb_c0[dense_mask]
        n_dense = xyz_d.shape[0]

        # final tight trim + sample
        lo2 = np.percentile(xyz_d, 0.5, axis=0)
        hi2 = np.percentile(xyz_d, 99.5, axis=0)
        fine = np.all((xyz_d >= lo2) & (xyz_d <= hi2), axis=1)
        xyz_f, rgb_f = xyz_d[fine], rgb_d[fine]
        n_kept = xyz_f.shape[0]

        idx = np.random.default_rng(0).choice(n_kept, size=min(n_kept, POINTS_PER_WEEK), replace=False)
        xyz_s, rgb_s = xyz_f[idx], rgb_f[idx]

        # center THIS week on its own (post-filter) centroid -- weeks are not
        # co-registered, each SfM run has its own arbitrary gauge/origin.
        center = xyz_s.mean(axis=0)
        xyz_cent = xyz_s - center
        extent = xyz_cent.max(axis=0) - xyz_cent.min(axis=0)
        radius = float(np.linalg.norm(extent) / 2)

        core_lo = np.percentile(xyz_cent, 5, axis=0)
        core_hi = np.percentile(xyz_cent, 95, axis=0)
        core_radius = float(np.linalg.norm(core_hi - core_lo) / 2)

        # remap ifc(x,y,z) -> three(x,y,z) = ifc(x,-z,y)  [confirmed: raw +z
        # is inverted relative to true "up" -- see module docstring]
        three_xyz = np.stack([xyz_cent[:, 0], -xyz_cent[:, 2], xyz_cent[:, 1]], axis=1).astype(np.float32)

        pos_b64 = base64.b64encode(three_xyz.tobytes()).decode("ascii")
        col_b64 = base64.b64encode(rgb_s.tobytes()).decode("ascii")

        weeks_out.append({
            "label": week,
            "nPoints": int(three_xyz.shape[0]),
            "nPointsFull": int(n_total),
            "nOutliersDropped": int(n_loaded - n_kept),
            "extent": [float(v) for v in extent],
            "radius": radius,
            "coreRadius": core_radius,
            "pos": pos_b64,
            "col": col_b64,
        })
        print(f"{week}: loaded {n_loaded}/{n_total}, dense-filtered to {n_dense}, "
              f"final kept {n_kept}, sampled {three_xyz.shape[0]}, "
              f"extent(x,y,z)={extent}, coreRadius={core_radius:.1f} "
              f"(full radius={radius:.1f}), "
              f"pos {len(pos_b64)/1e6:.2f}MB b64, col {len(col_b64)/1e6:.2f}MB b64")

    new_labels = {w["label"] for w in weeks_out}
    if os.path.exists(OUT_JS):
        with open(OUT_JS, encoding="utf-8") as f:
            txt = f.read()
        payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))
        payload["weeks"] = [w for w in payload["weeks"] if w["label"] not in new_labels] + weeks_out
    else:
        payload = {"weeks": weeks_out}

    with open(OUT_JS, "w", encoding="utf-8") as f:
        f.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, f)
        f.write(";\n")

    size_mb = os.path.getsize(OUT_JS) / 1e6
    print(f"\nWrote {OUT_JS} ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
