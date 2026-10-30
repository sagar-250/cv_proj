"""Same-pose pictures of the DART clutter removal (what the camera saw -> what DART segmented -> what the cloud looks like).

For the views of one stage that show the most machinery, one row each, four panels from the SAME camera pose:
  1  camera image
  2  DART's clutter segmentation (red = machinery / ladder / ... , from dart_label.py's prompts)
  3  this view's own points after removing the segmented pixels (hole where the machine was)
  4  the merged cloud of ALL views (clutter removed) seen from this pose: the hole is filled where another
     camera (e.g. from the other side of the wall) saw that surface

Run (needs the SAM3 / DART env, like dart_label.py):
    PYTHONPATH=/home/dice-lab/Sagar/SAM/src /home/dice-lab/Sagar/SAM/.venv/bin/python src/make_dart_pose_fig.py \
        [--stage NAME] [--tag interior_dartc] [--rows 4]
Writes results/dart/{stage}_pose_{k}.png and results/dart/{stage}_pose_all.png.
gt_class is used only to pick the views and to print how much machinery the picture contains.
"""
import argparse

import cv2
import numpy as np

import dart_label
from site_clutter import INTERIOR_CLASSES
from sam3_live import dart_runtime

DART_GRID, DART_FOV = (480, 360), 80.0   # = p3_render.py (not imported: it needs ifcopenshell, absent in this env)


def camera_basis(eye, target, world_up):   # = p2_p3_simulate_stages.camera_basis
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    return forward, right, np.cross(right, forward)


SS = 2   # supersampling of the point-cloud panels
SPLAT_M = 0.12   # m: size of the surface patch one cloud point is drawn as


def project(xyz, eye, target, up, fov, grid, scale=SS):
    """Pixel (col, row) and depth of points seen by the pinhole camera p3_render used (upright image)."""
    fwd, right, up_v = camera_basis(eye.astype(float), target.astype(float), up)
    w, h = grid
    d = xyz - eye
    z = d @ fwd
    ok = z > 0.3
    t = np.tan(np.radians(fov) / 2)
    u = (d @ right) / np.where(ok, z, 1) / t
    v = (d @ up_v) / np.where(ok, z, 1) / np.tan(np.radians(fov) / 2 / (w / h))   # p3_render's own (odd) vertical scale
    col = ((u + 1) / 2 * (w * scale - 1)).round().astype(int)
    row = ((1 - v) / 2 * (h * scale - 1)).round().astype(int)
    ok &= (col >= 0) & (col < w * scale) & (row >= 0) & (row < h * scale)
    return col, row, z, ok


def splat(xyz, rgb, cam, grid, size=2):
    """Far-first painter with depth-sized splats: a point stands for the surface patch around it, so
    near points (sparse on screen) get big splats and hide what is behind them instead of letting it show through."""
    eye, target, up, fov = cam
    w, h = grid
    col, row, z, ok = project(xyz, eye, target, up, fov, grid)
    img = np.zeros((h * SS, w * SS, 3), np.uint8)
    order = np.argsort(-z[ok])
    c, r, colr, zz = col[ok][order], row[ok][order], rgb[ok][order], z[ok][order]
    rad = np.clip((SPLAT_M / zz * w * SS / (2 * np.tan(np.radians(fov) / 2))).round().astype(int), 1, 14)
    for k in np.unique(rad):                        # radius grows as depth shrinks: far first, near last
        m = rad == k
        for dy in range(-k // 2, k - k // 2):
            for dx in range(-k // 2, k - k // 2):
                img[np.clip(r[m] + dy, 0, h * SS - 1), np.clip(c[m] + dx, 0, w * SS - 1)] = colr[m]
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA) if SS > 1 else img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="11_03_derde_verdieping_structure")
    ap.add_argument("--tag", default="interior_dartc", help="clutter-removed merged cloud: data/pointclouds_<tag>")
    ap.add_argument("--rows", type=int, default=12)
    args = ap.parse_args()
    st = args.stage
    v = dict(np.load(f"data/dart/{st}/views.npz"))
    merged = np.load(f"data/pointclouds_{args.tag}/{st}.npz")
    T = np.load(f"data/pointclouds_interior/{st}.npz")["gt_T_bim_to_cloud"]   # only to get the survey-frame "up"
    up = T[:3, :3] @ np.array([0, 0, 1.0])
    for k in ("xyz", "rgb", "gt_class_idx"):
        v[k] = v[k][:, ::-1]                                                   # upright images
    names = v["gt_class_names"][v["gt_class_idx"]]
    n_mach = np.isin(names, [c for c in INTERIOR_CLASSES if c.startswith("mach")]).sum(axis=(1, 2))
    cand = np.nonzero(n_mach > 3000)[0]                       # views with a clearly visible machine
    pick = cand[np.linspace(0, len(cand) - 1, min(args.rows, len(cand))).astype(int)]   # spread over the walkthrough
    from sam3_live import config
    config.SCORE_THRESHOLD = dart_label.SCORE
    predictor = dart_runtime.load_model()[0]
    h, w = DART_GRID[1], DART_GRID[0]
    rows = []
    for i in pick:
        cam = (v["eyes"][i], v["targets"][i], up, DART_FOV)
        rgb = v["rgb"][i]
        res = dart_runtime.run_cycle(predictor, None, rgb[:, :, ::-1].copy(), dart_label.CLUTTER)
        pred = dart_label.classify(res, (h, w), dart_label.CLUTTER)
        hit = np.isfinite(v["xyz"][i, :, :, 0])
        clutter = pred == 2
        seg = rgb.copy()
        seg[clutter] = (0.35 * seg[clutter] + 0.65 * np.array([230, 40, 40])).astype(np.uint8)
        own = rgb.copy()
        own[clutter | ~hit] = 0
        # merged cloud seen from this pose (points of every view, clutter already removed)
        full = splat(merged["xyz"].astype(np.float64), merged["rgb"], cam, DART_GRID)
        rows.append(np.concatenate([rgb, seg, own, full], axis=1))
        print(f"view {i}: {int(n_mach[i])} machinery px (truth), DART clutter px {int(clutter.sum())}, "
              f"merged-cloud coverage {float((full.sum(axis=2) > 0).mean()):.2f} vs own {float((own.sum(axis=2) > 0).mean()):.2f}")
    head = np.zeros((28, rows[0].shape[1], 3), np.uint8)
    for k, t in enumerate(["camera image", "DART segmentation (red = clutter)", "this view after removal",
                           "all views merged, clutter removed (same pose)"]):
        cv2.putText(head, t, (k * w + 8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for k in range(0, len(rows), 4):                         # report-sized sheets of 4 poses
        cv2.imwrite(f"results/dart/{st}_pose_sheet{k // 4 + 1}.png",
                    np.concatenate([head] + rows[k:k + 4], axis=0)[:, :, ::-1])
    for i, row in zip(pick, rows):                           # and one image per pose, with captions
        cv2.imwrite(f"results/dart/{st}_pose_{i:03d}.png", np.concatenate([head, row], axis=0)[:, :, ::-1])
    print(f"saved {len(rows)} poses: results/dart/{st}_pose_sheet*.png and {st}_pose_NNN.png")


if __name__ == "__main__":
    main()
