"""Phase 4 eval -- score p4_segment.py's output against held-out ground truth.

Loads gt_class/gt_global_id from data/pointclouds/{stage}.npz (never given to
p4_segment.py -- held out for this script only) plus observations.json and
point_cluster_id.npy, and reports, per stage:

  - matched/merged/split/missed counts over true BIM elements (grouped by
    gt_global_id): for each true element, find its dominant cluster (the
    cluster most of its surviving points ended up in). matched = that
    cluster's own point majority is also this element (mutual best) with
    >=60% coverage; merged = that cluster is ALSO the dominant cluster for
    >=1 other element; split = the element's points spread across >=2
    clusters with no single one reaching 60%; missed = no cluster covers a
    meaningful share (occlusion/dropout/dropped-as-noise).
  - class confusion (mapped IfcWall->wall etc.) by majority vote per element.
  - clutter leakage: fraction of true ground/tree points whose cluster is
    one of the 4 structural classes (false positives bleeding through).

This is deliberately a rule-based bucketing, not SPEC.md's full Phase 6
Hungarian/IoU matcher -- enough to sanity-check cluster quality now, per the
approved plan's reduced v1 scope.

Run: .venv/Scripts/python.exe src/p4_eval_segment.py
Output: results/p4_eval_summary.csv, results/figures/p4_segmentation_quality_vs_stage.png
"""
import glob
import json
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

POINTCLOUD_DIR = "data/pointclouds"
OBS_DIR = "data/observations"
OUT_CSV = "results/p4_eval_summary.csv"
OUT_FIG = "results/figures/p4_segmentation_quality_vs_stage.png"

GT_TO_CLASS = {"IfcWall": "wall", "IfcSlab": "slab", "IfcBeam": "beam", "IfcColumn": "column"}
MATCH_COVERAGE_THRESHOLD = 0.6


def eval_stage(stage_name):
    data = np.load(os.path.join(POINTCLOUD_DIR, f"{stage_name}.npz"))
    gt_class, gt_gid = data["gt_class"], data["gt_global_id"]

    obs_path = os.path.join(OBS_DIR, stage_name, "observations.json")
    cid_path = os.path.join(OBS_DIR, stage_name, "point_cluster_id.npy")
    with open(obs_path, encoding="utf-8") as f:
        observations = json.load(f)
    cluster_id = np.load(cid_path)
    obs_class = {i: o["class"] for i, o in enumerate(observations)}

    # --- per-element outcome ---
    is_structural = gt_gid != ""
    unique_gids = np.unique(gt_gid[is_structural])

    class_correct = 0
    class_total = 0
    confusion = {}

    # for each true element: its best (majority) cluster + coverage fraction
    elem_best_cluster = {}
    for gid in unique_gids:
        elem_mask = gt_gid == gid
        n_elem_points = int(elem_mask.sum())
        elem_clusters = cluster_id[elem_mask]
        elem_clusters = elem_clusters[elem_clusters != -1]
        if elem_clusters.shape[0] == 0:
            elem_best_cluster[gid] = (None, 0.0)
            continue
        vals, counts = np.unique(elem_clusters, return_counts=True)
        best_cluster = int(vals[np.argmax(counts)])
        best_coverage = counts.max() / n_elem_points
        elem_best_cluster[gid] = (best_cluster, best_coverage)

    # a cluster is "shared" if it's the best (>=threshold) cluster for more
    # than one element -- that's the merge signal
    dominant_of = {}
    for gid, (best_cluster, coverage) in elem_best_cluster.items():
        if best_cluster is not None and coverage >= MATCH_COVERAGE_THRESHOLD:
            dominant_of.setdefault(best_cluster, []).append(gid)
    merged_clusters = {c for c, gids in dominant_of.items() if len(gids) > 1}

    outcome_counts = {"matched": 0, "merged": 0, "split": 0, "missed": 0}
    for gid in unique_gids:
        best_cluster, coverage = elem_best_cluster[gid]
        if best_cluster is None:
            outcome_counts["missed"] += 1
            continue
        if coverage < MATCH_COVERAGE_THRESHOLD:
            outcome_counts["split"] += 1
            continue
        outcome_counts["merged" if best_cluster in merged_clusters else "matched"] += 1

        elem_mask = gt_gid == gid
        true_cls = GT_TO_CLASS.get(gt_class[elem_mask][0])
        pred_cls = obs_class.get(best_cluster)
        if true_cls is not None:
            class_total += 1
            if pred_cls == true_cls:
                class_correct += 1
            confusion.setdefault(true_cls, {}).setdefault(pred_cls, 0)
            confusion[true_cls][pred_cls] += 1

    # --- clutter leakage ---
    clutter_mask = (gt_class == "ground") | (gt_class == "tree")
    clutter_clustered = cluster_id[clutter_mask] != -1
    leaked = 0
    for i in np.unique(cluster_id[clutter_mask][clutter_clustered]):
        if obs_class.get(int(i)) in ("wall", "slab", "column", "beam"):
            leaked += int(((cluster_id == i) & clutter_mask).sum())
    clutter_leakage_rate = leaked / max(int(clutter_mask.sum()), 1)

    n_ambiguous_planar = sum(1 for o in observations if o["class"] == "ambiguous_planar")
    n_ambiguous_elongated = sum(1 for o in observations if o["class"] == "ambiguous_elongated")

    return {
        "stage": stage_name,
        "n_true_elements": len(unique_gids),
        "n_matched": outcome_counts["matched"],
        "n_merged": outcome_counts["merged"],
        "n_split": outcome_counts["split"],
        "n_missed": outcome_counts["missed"],
        "class_accuracy": class_correct / max(class_total, 1),
        "clutter_leakage_rate": clutter_leakage_rate,
        "n_ambiguous_planar": n_ambiguous_planar,
        "n_ambiguous_elongated": n_ambiguous_elongated,
        "n_clusters_total": len(observations),
    }


def main():
    stage_names = sorted(
        os.path.splitext(os.path.basename(p))[0]
        for p in glob.glob(os.path.join(OBS_DIR, "*"))
        if os.path.isdir(p)
    )

    rows = [eval_stage(s) for s in stage_names]
    df = pd.DataFrame(rows)
    os.makedirs("results", exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(df.to_string(index=False))
    print(f"\nWrote {OUT_CSV}")

    os.makedirs("results/figures", exist_ok=True)
    fig, ax = plt.subplots(figsize=(11, 6))
    x = np.arange(len(df))
    bottom = np.zeros(len(df))
    for col, color in [("n_matched", "#4a8f5c"), ("n_merged", "#e8a33d"),
                       ("n_split", "#c96a4e"), ("n_missed", "#8a8f98")]:
        ax.bar(x, df[col], bottom=bottom, label=col.replace("n_", ""), color=color)
        bottom += df[col].values
    ax.set_xticks(x)
    ax.set_xticklabels(df["stage"], rotation=60, ha="right", fontsize=7)
    ax.set_ylabel("true BIM elements")
    ax.set_title("Segmentation quality vs. construction stage")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT_FIG, dpi=150)
    print(f"Wrote {OUT_FIG}")


if __name__ == "__main__":
    main()
