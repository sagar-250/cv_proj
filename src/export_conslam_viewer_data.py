"""Add the ConSLAM/SLAM2REF reference point cloud to the same viewer data file
used by src/pointcloud_viewer_tail.html, so it can be visually compared
side-by-side (as another tab) against the Schependomlaan weeks.

Source: ConSLAM_BIM_PC_V9_All_merged_ROI_Density_0_01_m_No_color.ply
(binary PLY, x,y,z only, ~58.7M points, ~1cm TLS-merged reference map from
SLAM2REF's sample data -- see data/raw/conslam_slam2ref/).

Axis check done separately: z has by far the smallest extent (~5.6m vs
~84m/46m for x/y) and the densest single band sits near z-min with
progressively sparser structure going up -- i.e. z is already up, no flip
needed (unlike Schependomlaan, which needed one).

No RGB in this file, so points are colored by height (a two-stop gradient)
instead of true color.

Run: .venv/Scripts/python.exe src/export_conslam_viewer_data.py
Appends a "ConSLAM Reference" entry into results/pointcloud_viewer_data.js
(must be run AFTER export_pointcloud_viewer_data.py).
"""
import base64
import json

import numpy as np
from scipy.spatial import cKDTree

PLY_PATH = "data/raw/conslam_slam2ref/inputs/01.Ref_maps_pcs/ConSLAM_BIM_PC_V9_All_merged_ROI_Density_0_01_m_No_color.ply"
OUT_JS = "results/pointcloud_viewer_data.js"
LOAD_CAP = 4_000_000
POINTS_OUT = 55_000
DENSITY_KEEP_FRAC = 0.70

# height-gradient colors, low -> high (matches viewer's dark/accent palette)
COLOR_LOW = np.array([90, 110, 130])   # cool slate
COLOR_HIGH = np.array([239, 131, 84])  # accent orange


def load_binary_ply_xyz(path, cap):
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip() == b"end_header":
                break
        offset = f.tell()
    n_vertex = int([l for l in header.split(b"\n") if b"element vertex" in l][0].split()[-1])
    n_read = min(n_vertex, cap) if cap else n_vertex
    data = np.fromfile(path, dtype=np.float32, count=n_read * 3, offset=offset).reshape(-1, 3)
    return data, n_vertex


def density_filter(xyz, keep_frac, k=8):
    tree = cKDTree(xyz)
    dist, _ = tree.query(xyz, k=k + 1, workers=-1)
    mean_knn = dist[:, 1:].mean(axis=1)
    thresh = np.percentile(mean_knn, keep_frac * 100)
    return mean_knn <= thresh


def height_to_rgb(z):
    t = (z - z.min()) / max(z.max() - z.min(), 1e-6)
    t = t[:, None]
    rgb = COLOR_LOW[None, :] * (1 - t) + COLOR_HIGH[None, :] * t
    return rgb.astype(np.uint8)


def main():
    xyz_raw, n_total = load_binary_ply_xyz(PLY_PATH, LOAD_CAP)
    n_loaded = xyz_raw.shape[0]
    print(f"loaded {n_loaded}/{n_total} points")

    lo = np.percentile(xyz_raw, 1, axis=0)
    hi = np.percentile(xyz_raw, 99, axis=0)
    coarse = np.all((xyz_raw >= lo) & (xyz_raw <= hi), axis=1)
    xyz_c0 = xyz_raw[coarse]

    dense_mask = density_filter(xyz_c0, DENSITY_KEEP_FRAC)
    xyz_d = xyz_c0[dense_mask]
    print(f"dense-filtered to {xyz_d.shape[0]}")

    lo2 = np.percentile(xyz_d, 0.5, axis=0)
    hi2 = np.percentile(xyz_d, 99.5, axis=0)
    fine = np.all((xyz_d >= lo2) & (xyz_d <= hi2), axis=1)
    xyz_f = xyz_d[fine]
    n_kept = xyz_f.shape[0]

    idx = np.random.default_rng(0).choice(n_kept, size=min(n_kept, POINTS_OUT), replace=False)
    xyz_s = xyz_f[idx]

    center = xyz_s.mean(axis=0)
    xyz_cent = xyz_s - center
    extent = xyz_cent.max(axis=0) - xyz_cent.min(axis=0)
    radius = float(np.linalg.norm(extent) / 2)

    core_lo = np.percentile(xyz_cent, 5, axis=0)
    core_hi = np.percentile(xyz_cent, 95, axis=0)
    core_radius = float(np.linalg.norm(core_hi - core_lo) / 2)

    # z already up -- no flip: three(x,y,z) = ifc(x,z,y)
    three_xyz = np.stack([xyz_cent[:, 0], xyz_cent[:, 2], xyz_cent[:, 1]], axis=1).astype(np.float32)
    rgb_s = height_to_rgb(xyz_s[:, 2])

    pos_b64 = base64.b64encode(three_xyz.tobytes()).decode("ascii")
    col_b64 = base64.b64encode(rgb_s.tobytes()).decode("ascii")

    entry = {
        "label": "ConSLAM Reference",
        "nPoints": int(three_xyz.shape[0]),
        "nPointsFull": int(n_total),
        "nOutliersDropped": int(n_loaded - n_kept),
        "extent": [float(v) for v in extent],
        "radius": radius,
        "coreRadius": core_radius,
        "pos": pos_b64,
        "col": col_b64,
    }
    print(f"ConSLAM Reference: sampled {three_xyz.shape[0]}, extent={extent}, "
          f"coreRadius={core_radius:.2f}, pos {len(pos_b64)/1e6:.2f}MB, col {len(col_b64)/1e6:.2f}MB")

    with open(OUT_JS, encoding="utf-8") as f:
        txt = f.read()
    payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))
    payload["weeks"] = [w for w in payload["weeks"] if w["label"] != entry["label"]] + [entry]

    with open(OUT_JS, "w", encoding="utf-8") as f:
        f.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, f)
        f.write(";\n")

    import os
    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
