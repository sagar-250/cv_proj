"""Pictures for the interior-objects + DART structure-only experiment (one stage).

results/dart/{stage}_clouds.png   full cloud (clutter in colour) vs structure-only cloud, plan + side view
results/dart/{stage}_summary.png  bar chart: points by type kept/removed, and registration / progress numbers

Run: .venv/bin/python src/make_dart_figs.py [--stage NAME]
gt_class is used only to colour the pictures and count what was removed (eval only).
"""
import argparse
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from site_clutter import INTERIOR_CLASSES

CLUTTER = ["machinery", "temp", "pile", "worker", "scaffold", "crane", "fence", "props"]


def sub(n, k, seed=0):
    return np.random.default_rng(seed).choice(n, min(n, k), replace=False)


def reg_err(stage, tag):
    """Translation error (m) and rotation error (deg) of the estimated cloud->BIM transform vs truth."""
    reg = json.load(open(f"data/matches_{tag}/{stage}/registration.json"))
    truth = np.load(f"data/pointclouds_{tag}/{stage}.npz")["gt_T_bim_to_cloud"]
    est = np.array(reg["T_cloud_to_bim"])
    d = est @ truth                      # should be identity
    ang = np.degrees(np.arccos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(d[:3, 3])), float(ang)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="11_03_derde_verdieping_structure")
    args = ap.parse_args()
    st = args.stage
    full = np.load(f"data/pointclouds_interior/{st}.npz")
    filt = np.load(f"data/pointclouds_interior_dart/{st}.npz")
    os.makedirs("results/dart", exist_ok=True)

    fig, ax = plt.subplots(2, 2, figsize=(14, 11))
    for ci, (name, d) in enumerate([("Full cloud (interior objects in orange)", full),
                                     ("DART structure-only cloud", filt)]):
        xyz, rgb, cls = d["xyz"], d["rgb"] / 255.0, d["gt_class"]
        # interior of the building only: drop ground/trees/fence/crane far away, keep within the building box
        c = np.median(xyz, axis=0)
        near = (np.abs(xyz[:, :2] - c[:2]) < 25).all(axis=1) & ~np.isin(cls, ["ground", "tree", "crane", "fence", "noise"])
        xyz, rgb, cls = xyz[near], rgb[near], cls[near]
        s = sub(len(xyz), 150_000)
        isobj = np.isin(cls[s], list(INTERIOR_CLASSES))
        col = np.where(isobj[:, None], np.array([[1.0, 0.45, 0.0]]), rgb[s])
        for row, (a, b, lab) in enumerate([(0, 1, "plan (x, y)"), (0, 2, "side (x, z)")]):
            ax[row, ci].scatter(xyz[s, a], xyz[s, b], c=col, s=0.4, linewidths=0)
            ax[row, ci].set_aspect("equal")
            ax[row, ci].set_title(f"{name} - {lab}" if row == 0 else lab, fontsize=10)
            ax[row, ci].set_facecolor("#202020")
    fig.tight_layout()
    fig.savefig(f"results/dart/{st}_clouds.png", dpi=130)

    # summary numbers
    stats = pd.read_csv("results/dart/interior_dart_stats.csv").iloc[0]
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    types = {"structure": ["IfcWall", "IfcWallStandardCase", "IfcSlab", "IfcColumn", "IfcBeam", "IfcStair", "IfcRoof",
                           "IfcCovering", "IfcMember", "IfcPlate", "IfcFooting", "IfcCurtainWall"],
             "interior objects": list(INTERIOR_CLASSES), "other site clutter": ["pile", "worker", "scaffold", "props"]}
    near = lambda d: ~np.isin(d["gt_class"], ["ground", "tree", "crane", "fence", "noise"])
    n_full = {k: int(np.isin(full["gt_class"], v).sum()) for k, v in types.items()}
    n_f = {k: int(np.isin(filt["gt_class"], v).sum()) for k, v in types.items()}
    x = np.arange(len(types))
    ax[0].bar(x - 0.2, [n_full[k] for k in types], 0.4, label="full cloud")
    ax[0].bar(x + 0.2, [n_f[k] for k in types], 0.4, label="DART structure-only")
    ax[0].set_xticks(x, list(types), fontsize=8); ax[0].set_yscale("log"); ax[0].legend(); ax[0].set_title("points by true type (log)")
    ax[1].bar(["precision", "recall"], [stats.structure_precision, stats.structure_recall], color=["#3a7", "#37a"])
    ax[1].set_ylim(0, 1); ax[1].set_title("DART: is a kept pixel structure? / is structure kept?")
    rows = []
    for tag, label in [("interior", "full cloud"), ("interior_dart", "DART structure-only")]:
        if os.path.exists(f"data/matches_{tag}/{st}/registration.json"):
            t, r = reg_err(st, tag)
            rows.append((label, t, r))
    if rows:
        ax[2].bar([r[0] for r in rows], [r[1] for r in rows], color=["#c63", "#3a7"][:len(rows)])
        for i, r in enumerate(rows):
            ax[2].text(i, r[1], f"{r[1]:.3f} m\n{r[2]:.2f} deg", ha="center", va="bottom", fontsize=9)
        ax[2].set_title("registration error"); ax[2].set_ylabel("m")
    fig.tight_layout()
    fig.savefig(f"results/dart/{st}_summary.png", dpi=130)
    print("saved figures; registration:", rows, "stats:", dict(stats))


if __name__ == "__main__":
    main()
