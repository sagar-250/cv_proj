"""Quick visual sanity check of a raw Schependomlaan PLY point cloud.

Usage: python visualize_pointcloud.py <path-to-ply> <out.png>

Parses the ASCII PLY header to find the vertex block (x y z nx ny nz r g b psz),
loads just those rows, and saves a top-down (XY) and a side (XZ) scatter colored
by the recorded RGB. No open3d/plyfile dependency needed for this sanity check.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_vertices(path):
    n_vertex = None
    header_lines = 0
    with open(path, "r", errors="ignore") as f:
        for line in f:
            header_lines += 1
            if line.startswith("element vertex"):
                n_vertex = int(line.split()[-1])
            if line.strip() == "end_header":
                break
    data = np.loadtxt(path, skiprows=header_lines, max_rows=n_vertex, usecols=(0, 1, 2, 6, 7, 8))
    xyz = data[:, :3]
    rgb = data[:, 3:6] / 255.0
    return xyz, rgb


def main():
    ply_path = sys.argv[1]
    out_path = sys.argv[2]

    xyz, rgb = load_vertices(ply_path)
    print(f"Loaded {xyz.shape[0]} points")
    print("X range:", xyz[:, 0].min(), xyz[:, 0].max())
    print("Y range:", xyz[:, 1].min(), xyz[:, 1].max())
    print("Z range:", xyz[:, 2].min(), xyz[:, 2].max())

    # subsample for a snappy scatter plot
    n = xyz.shape[0]
    idx = np.random.default_rng(0).choice(n, size=min(n, 150_000), replace=False)
    xyz_s, rgb_s = xyz[idx], rgb[idx]

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    axes[0].scatter(xyz_s[:, 0], xyz_s[:, 1], c=rgb_s, s=0.5, marker=".")
    axes[0].set_title("Top-down (XY)")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].set_aspect("equal")

    axes[1].scatter(xyz_s[:, 0], xyz_s[:, 2], c=rgb_s, s=0.5, marker=".")
    axes[1].set_title("Side (XZ)")
    axes[1].set_xlabel("X (m)")
    axes[1].set_ylabel("Z (m)")
    axes[1].set_aspect("equal")

    fig.suptitle(ply_path)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
