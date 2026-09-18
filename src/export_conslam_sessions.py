"""Merge the two real ConSLAM/SLAM2REF scan *sessions* (not just the single
static reference map) into the viewer data, using each frame's own pose.

These are two genuinely distinct real captures bundled in the SLAM2REF sample
data (see data/raw/conslam_slam2ref/inputs/00.SessionData/...):

- "BIM_session_scans_296-422": 128 frames, already pre-filtered to
  Walls/Columns, poses given as 3x4 matrices already in the shared
  reference/BIM-ROI world frame.
- "DLIO_real_world_SLAM_session": 376 frames, raw (unfiltered) LiDAR, poses
  are 4x4 matrices in the SLAM algorithm's own local trajectory frame (this
  is the "query" session SLAM2REF aligns to the reference map -- so its
  frame is NOT the same world frame as the session above; that's fine here
  since every scene in this viewer is centered/viewed independently anyway).

Each session's frames are transformed by their own pose and merged into one
point cloud, then goes through the same density-filter -> center -> sample
pipeline as the other entries, colored by height (no reliable RGB in either
source).

Run: .venv/Scripts/python.exe src/export_conslam_sessions.py
(run after export_conslam_viewer_data.py; appends/replaces entries in
results/pointcloud_viewer_data.js)
"""
import base64
import glob
import json
import os

import numpy as np
from scipy.spatial import cKDTree

SESSION_BASE = "data/raw/conslam_slam2ref/inputs/00.SessionData/00.ConSLAM/00.TestA01_scans_in_BIM_ROI"
OUT_JS = "results/pointcloud_viewer_data.js"
POINTS_OUT = 55_000
DENSITY_KEEP_FRAC = 0.70

COLOR_LOW = np.array([90, 110, 130])
COLOR_HIGH = np.array([239, 131, 84])


def read_pcd_xyz(path):
    with open(path, "rb") as f:
        fields = None
        sizes = None
        types = None
        counts = None
        n_points = None
        data_mode = None
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"no DATA line in {path}")
            s = line.decode("ascii", errors="ignore").strip()
            if s.startswith("FIELDS"):
                fields = s.split()[1:]
            elif s.startswith("SIZE"):
                sizes = [int(x) for x in s.split()[1:]]
            elif s.startswith("TYPE"):
                types = s.split()[1:]
            elif s.startswith("COUNT"):
                counts = [int(x) for x in s.split()[1:]]
            elif s.startswith("POINTS"):
                n_points = int(s.split()[1])
            elif s.startswith("DATA"):
                data_mode = s.split()[1]
                break
        offset = f.tell()

    xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")

    if data_mode == "ascii":
        with open(path, "r", errors="ignore") as tf:
            lines = tf.readlines()
        header_lines = next(i for i, l in enumerate(lines) if l.strip().startswith("DATA")) + 1
        data = np.loadtxt(path, skiprows=header_lines, usecols=(xi, yi, zi))
        return data.astype(np.float32)

    elif data_mode == "binary":
        type_map = {"F4": np.float32, "F8": np.float64, "U4": np.uint32, "U1": np.uint8, "I4": np.int32}
        dtype_fields = []
        for name, size, typ, cnt in zip(fields, sizes, types, counts):
            key = f"{typ}{size}"
            np_t = type_map.get(key, np.float32)
            dtype_fields.append((name, np_t, cnt) if cnt > 1 else (name, np_t))
        dtype = np.dtype(dtype_fields)
        raw = np.fromfile(path, dtype=dtype, count=n_points, offset=offset)
        xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)
        return xyz
    else:
        raise ValueError(f"unsupported DATA mode {data_mode} in {path}")


def load_pose_matrix(row):
    vals = np.array(row, dtype=np.float64)
    if vals.size == 12:
        M = vals.reshape(3, 4)
        R, t = M[:, :3], M[:, 3]
    elif vals.size == 16:
        M = vals.reshape(4, 4)
        R, t = M[:3, :3], M[:3, 3]
    else:
        raise ValueError(f"unexpected pose length {vals.size}")
    return R, t


def merge_session(folder, cap_frames=None):
    poses = np.loadtxt(os.path.join(folder, "optimized_poses.txt"))
    scan_files = sorted(glob.glob(os.path.join(folder, "Scans", "*.pcd")))
    if len(scan_files) != poses.shape[0]:
        print(f"  WARNING: {len(scan_files)} scans vs {poses.shape[0]} poses -- using min length")
    n = min(len(scan_files), poses.shape[0])
    if cap_frames:
        n = min(n, cap_frames)

    merged = []
    for i in range(n):
        xyz_local = read_pcd_xyz(scan_files[i])
        R, t = load_pose_matrix(poses[i])
        xyz_world = xyz_local @ R.T + t
        merged.append(xyz_world)
    return np.concatenate(merged, axis=0)


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


def process_and_pack(label, xyz_raw):
    n_loaded = xyz_raw.shape[0]
    lo = np.percentile(xyz_raw, 1, axis=0)
    hi = np.percentile(xyz_raw, 99, axis=0)
    coarse = np.all((xyz_raw >= lo) & (xyz_raw <= hi), axis=1)
    xyz_c0 = xyz_raw[coarse]

    dense_mask = density_filter(xyz_c0, DENSITY_KEEP_FRAC)
    xyz_d = xyz_c0[dense_mask]

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

    three_xyz = np.stack([xyz_cent[:, 0], xyz_cent[:, 2], xyz_cent[:, 1]], axis=1).astype(np.float32)
    rgb_s = height_to_rgb(xyz_s[:, 2])

    pos_b64 = base64.b64encode(three_xyz.tobytes()).decode("ascii")
    col_b64 = base64.b64encode(rgb_s.tobytes()).decode("ascii")

    print(f"{label}: loaded {n_loaded}, kept {n_kept}, sampled {three_xyz.shape[0]}, "
          f"extent={extent}, coreRadius={core_radius:.2f}")

    return {
        "label": label,
        "nPoints": int(three_xyz.shape[0]),
        "nPointsFull": int(n_loaded),
        "nOutliersDropped": int(n_loaded - n_kept),
        "extent": [float(v) for v in extent],
        "radius": radius,
        "coreRadius": core_radius,
        "pos": pos_b64,
        "col": col_b64,
    }


def main():
    print("Merging BIM_session_scans_296-422 (128 frames, Walls/Columns)...")
    xyz_a = merge_session(os.path.join(SESSION_BASE, "BIM_session_scans_296-422"))
    entry_a = process_and_pack("ConSLAM Session A (Walls/Columns)", xyz_a)

    print("Merging DLIO_real_world_SLAM_session (376 frames, raw)...")
    xyz_b = merge_session(os.path.join(SESSION_BASE, "DLIO_real_world_SLAM_session"))
    entry_b = process_and_pack("ConSLAM Session B (raw SLAM)", xyz_b)

    with open(OUT_JS, encoding="utf-8") as f:
        txt = f.read()
    payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))

    keep_labels = {entry_a["label"], entry_b["label"]}
    payload["weeks"] = [w for w in payload["weeks"] if w["label"] not in keep_labels] + [entry_a, entry_b]

    with open(OUT_JS, "w", encoding="utf-8") as f:
        f.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, f)
        f.write(";\n")

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
