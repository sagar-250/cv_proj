"""R6 -- keep only STRUCTURE points: DART (SAM3 text-prompted segmentation) on the camera images.

p3_render.py --dart saves, per stage, a set of camera images with the first-hit xyz of every
pixel (data/dart/{stage}/views.npz). This script runs DART on each image, decides per pixel
whether it shows building structure (wall, slab, column) or something that is not part of the
building (machinery, ladders, material bags, workers, scaffold ...), and writes a point cloud
with only the structure pixels. p4_match_bim.py then registers and matches that cloud instead
of the full one (RUN_TAG=<tag>, see p4_match_bim.py).

Per pixel the class is the prompt with the highest mask score covering it. A pixel is kept
when that prompt is a structure prompt. Pixels no prompt claims are dropped by default
(--keep-unlabelled keeps them: more points, but clutter DART missed gets through).

Needs the SAM3 / DART environment, not the project venv:
    PYTHONPATH=/home/dice-lab/Sagar/SAM/src /home/dice-lab/Sagar/SAM/.venv/bin/python \
        src/dart_label.py --tag interior_dart --from interior [--stage NAME] [--overlays N]

Reads  data/dart/{stage}/views.npz and data/pointclouds_<from>/{stage}.npz (for the camera poses
       and eval-only registration truth, copied through untouched)
Writes data/pointclouds_<tag>/{stage}.npz            same keys as a normal stage cloud
       results/dart/{tag}_stats.csv                  per stage: structure precision / recall
       results/dart/overlays/{stage}_*.png           image | DART classes | ground truth classes

gt_* is read only for the statistics and copied through for the eval scripts, never used to
choose a point. Colour is the simulated camera colour (rgb_source stays "camera").
"""
import argparse
import glob
import os

import cv2
import numpy as np

from sam3_live import dart_runtime

STRUCTURE = ["building", "wall", "floor", "ceiling", "column"]
CLUTTER = ["construction equipment", "machine", "generator", "scissor lift", "cement mixer", "box", "ladder",
           "barrel", "bucket", "human worker"]
SCORE = 0.25   # DART's default 0.6 finds none of the plain-box machines in the simulated images   # no "scaffolding": its mask covers the whole facade and wins over it
STRUCTURE_GT = {"IfcWall", "IfcWallStandardCase", "IfcSlab", "IfcColumn", "IfcBeam", "IfcStair",
                "IfcRoof", "IfcCovering", "IfcMember", "IfcPlate", "IfcFooting", "IfcCurtainWall"}
from site_clutter import INTERIOR_CLASSES  # noqa: E402  (names only; pure numpy module)
CLUTTER_GT = {"scaffold", "crane", "fence", "props", "pile", "worker", *INTERIOR_CLASSES}
PALETTE = {"structure": (60, 200, 60), "clutter": (60, 60, 230), "none": (120, 120, 120)}


def classify(results, shape, prompts):
    """Per-pixel 0 none / 1 structure / 2 clutter from DART's masks: highest-scoring mask wins."""
    best = np.zeros(shape, np.float32)
    lab = np.zeros(shape, np.uint8)
    for prompt in prompts:
        r = results[prompt]
        kind = 1 if prompt in STRUCTURE else 2
        for score, mask in zip(r["scores"], r["masks"]):
            m = mask & (score > best)
            best[m] = score
            lab[m] = kind
    return lab


def gt_kind(names):
    """0 sky / other, 1 structure, 2 clutter, from the eval-only hit class names."""
    out = np.zeros(len(names), np.uint8)
    out[np.isin(names, list(STRUCTURE_GT))] = 1
    out[np.isin(names, list(CLUTTER_GT))] = 2
    return out


def overlay(rgb, pred, gt, path):
    def paint(lab):
        img = np.zeros(lab.shape + (3,), np.uint8)
        for k, name in enumerate(["none", "structure", "clutter"]):
            img[lab == k] = PALETTE[name]
        return img
    panel = np.concatenate([rgb, cv2.addWeighted(rgb, 0.4, paint(pred), 0.6, 0), paint(gt)], axis=1)
    cv2.imwrite(path, panel[:, :, ::-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="interior_dart")
    ap.add_argument("--from", dest="src", default="interior", help="tag of the full clouds (data/pointclouds_<from>)")
    ap.add_argument("--stage", default=None)
    ap.add_argument("--mode", choices=["clutter", "structure", "none"], default="clutter",
                    help="clutter: DART finds only clutter (machinery, ladders, ...), those pixels are removed, "
                         "everything else is kept; structure: keep only pixels DART calls structure; "
                         "none: keep every pixel (baseline, no DART)")
    ap.add_argument("--keep-unlabelled", action="store_true", help="structure mode: also keep pixels no prompt claims")
    ap.add_argument("--overlays", type=int, default=4, help="overlay images saved per stage")
    ap.add_argument("--view-stride", type=int, default=1, help="use every n-th view (speed)")
    ap.add_argument("--max-points", type=int, default=1_500_000)
    args = ap.parse_args()

    from sam3_live import config
    config.SCORE_THRESHOLD = SCORE
    prompts = {"clutter": CLUTTER, "structure": STRUCTURE + CLUTTER, "none": []}[args.mode]
    predictor = dart_runtime.load_model()[0] if prompts else None
    out_dir = f"data/pointclouds_{args.tag}"
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs("results/dart/overlays", exist_ok=True)
    stages = sorted(os.path.basename(os.path.dirname(p)) for p in glob.glob("data/dart/*/views.npz"))
    if args.stage:
        stages = [s for s in stages if s == args.stage]
    rows = []
    rng = np.random.default_rng(0)
    for stage in stages:
        v = dict(np.load(f"data/dart/{stage}/views.npz"))
        views = np.arange(0, len(v["xyz"]), args.view_stride)
        for k in ("xyz", "rgb", "gt_class_idx", "gt_gid_idx"):
            v[k] = v[k][views, ::-1]    # p3_render writes the last ray row (camera up) first: flip to upright images
        v["kind"] = v["kind"][views]
        src = np.load(f"data/pointclouds_{args.src}/{stage}.npz")
        n, h, w, _ = v["xyz"].shape
        cls_names = v["gt_class_names"][v["gt_class_idx"]]          # eval only
        keep_xyz, keep_rgb, keep_cls, keep_gid, keep_pred = [], [], [], [], []
        tp = fp = fn = tn = n_gt_clutter = n_removed = 0
        # overlays: the views that show the most interior clutter (eval-only GT, only to pick pictures)
        n_clutter = np.isin(cls_names, list(INTERIOR_CLASSES) + ["pile", "worker"]).sum(axis=(1, 2))
        show = set(np.argsort(-n_clutter)[:args.overlays].tolist())
        for i in range(n):
            if prompts:
                res = dart_runtime.run_cycle(predictor, None, v["rgb"][i][:, :, ::-1].copy(), prompts)
                pred = classify(res, (h, w), prompts)
            else:
                pred = np.zeros((h, w), np.uint8)
            hit = np.isfinite(v["xyz"][i, :, :, 0])
            if args.mode == "structure":
                take = (pred == 1) | ((pred == 0) & args.keep_unlabelled)
            else:
                take = pred != 2                  # clutter / none: drop only what DART called clutter
            take &= hit
            gt = gt_kind(cls_names[i].reshape(-1)).reshape(h, w)
            s_gt, s_pred = (gt == 1) & hit, (pred == 1) & hit
            n_gt_clutter += int(((gt == 2) & hit).sum()); n_removed += int(((gt == 2) & hit & ~take).sum())
            tp += int((s_gt & take).sum()); fn += int((s_gt & ~take).sum())
            fp += int(((gt != 1) & hit & take).sum()); tn += int(((gt != 1) & hit & ~take).sum())
            keep_xyz.append(v["xyz"][i][take]); keep_rgb.append(v["rgb"][i][take])
            keep_cls.append(cls_names[i][take])
            keep_gid.append(v["gt_gid_names"][v["gt_gid_idx"][i]][take])
            keep_pred.append(pred[take])
            if i in show:
                overlay(v["rgb"][i], pred, gt, f"results/dart/overlays/{stage}_{i:03d}_{v['kind'][i]}.png")
        xyz = np.concatenate(keep_xyz); rgb = np.concatenate(keep_rgb)
        cls = np.concatenate(keep_cls); gid = np.concatenate(keep_gid)
        if len(xyz) > args.max_points:
            sel = np.sort(rng.choice(len(xyz), args.max_points, replace=False))
            xyz, rgb, cls, gid = xyz[sel], rgb[sel], cls[sel], gid[sel]
        extras = {k: src[k] for k in src.files if k not in ("xyz", "rgb", "gt_class", "gt_global_id")}
        np.savez_compressed(f"{out_dir}/{stage}.npz", xyz=xyz.astype(np.float32), rgb=rgb,
                            gt_class=cls.astype("<U16"), gt_global_id=gid.astype("<U24"), **extras)
        prec, rec = tp / max(tp + fp, 1), tp / max(tp + fn, 1)
        clutter_kept = float(np.isin(cls, list(CLUTTER_GT)).mean()) if len(cls) else 0.0
        rows.append(dict(stage=stage, n_views=n, n_points=len(xyz), structure_precision=round(prec, 4),
                         structure_recall=round(rec, 4), clutter_removed=round(n_removed / max(n_gt_clutter, 1), 4),
                         clutter_in_cloud=round(clutter_kept, 4)))
        print(f"{stage}: {len(xyz)} pts kept, structure precision {prec:.3f} recall {rec:.3f}, "
              f"clutter removed {n_removed / max(n_gt_clutter, 1):.1%}, clutter left in cloud {clutter_kept:.3%}", flush=True)
    if rows:
        import csv
        with open(f"results/dart/{args.tag}_stats.csv", "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)


if __name__ == "__main__":
    main()
