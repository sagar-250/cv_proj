"""Explanatory figures for the project report (beginner-level concept pictures).

The ICP and KD-tree pictures are real small computations (a toy floor plan), not
drawings: the same ideas the pipeline uses, small enough to see.

Run: .venv/bin/python src/make_report_figs.py   -> results/report/figs/*.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon
from scipy.spatial import cKDTree

OUT = "results/report/figs"
INK, MUTED, BRICK, STEEL, GREEN, AMBER, LIGHT = "#1B2733", "#5B6B7A", "#C4572F", "#2F6690", "#3E8E5A", "#D9962B", "#EEF2F6"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                     "text.color": INK, "xtick.color": MUTED, "ytick.color": MUTED})
os.makedirs(OUT, exist_ok=True)


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", name)


# ---------------------------------------------------------------- toy floor plan
def floor_plan(n_per_m=14):
    """Outline of an L-shaped building with a partition wall, as points (the 'design')."""
    segs = [((0, 0), (10, 0)), ((10, 0), (10, 4)), ((10, 4), (6, 4)), ((6, 4), (6, 8)), ((6, 8), (0, 8)), ((0, 8), (0, 0)),
            ((3, 0), (3, 5)), ((3, 5), (6, 5))]
    pts = []
    for (x0, y0), (x1, y1) in segs:
        n = int(np.hypot(x1 - x0, y1 - y0) * n_per_m)
        t = np.linspace(0, 1, n, endpoint=False)
        pts.append(np.stack([x0 + t * (x1 - x0), y0 + t * (y1 - y0)], 1))
    return np.concatenate(pts)


def rot(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def kabsch(P, Q):
    """Rotation R and shift t that best map P onto Q (least squares, paired points)."""
    mp, mq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - mp).T @ (Q - mq))
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, mq - R @ mp


def icp_trace(scan, model, iters=40):
    tree = cKDTree(model)
    states = [scan.copy()]
    cur = scan.copy()
    for _ in range(iters):
        _, idx = tree.query(cur)
        R, t = kabsch(cur, model[idx])
        cur = cur @ R.T + t
        states.append(cur.copy())
    return states


def rms_to_truth(cur, truth):
    return float(np.sqrt(((cur - truth) ** 2).sum(1).mean()))


def fig_icp():
    rng = np.random.default_rng(3)
    model = floor_plan()
    keep = rng.random(len(model)) < 0.55
    truth = model[keep] + rng.normal(0, 0.04, (keep.sum(), 2))          # the scan, in its true place
    cases = [("Good start: turned 12 deg, 1.2 m off", 12, np.array([1.2, -0.9])),
             ("Bad start: turned 120 deg, 6 m off", 120, np.array([6.5, 5.0]))]
    fig, axes = plt.subplots(2, 4, figsize=(13, 6.6))
    for r, (title, ang, shift) in enumerate(cases):
        c = truth.mean(0)
        start = (truth - c) @ rot(ang).T + c + shift
        states = icp_trace(start, model)
        shown = [0, 1, 4, len(states) - 1]
        labels = ["start", "after 1 step", "after 4 steps", "after 40 steps (settled)"]
        tree = cKDTree(model)
        for k, (ix, lab) in enumerate(zip(shown, labels)):
            ax = axes[r, k]
            ax.scatter(model[:, 0], model[:, 1], s=2, c=MUTED, alpha=0.6)
            ax.scatter(states[ix][:, 0], states[ix][:, 1], s=4, c=BRICK)
            if k == 0:   # nearest-point lines for a sample of points
                sel = np.arange(0, len(states[0]), 25)
                _, idx = tree.query(states[0][sel])
                for a, b in zip(states[0][sel], model[idx]):
                    ax.plot([a[0], b[0]], [a[1], b[1]], c=STEEL, lw=0.6, alpha=0.8)
            err = rms_to_truth(states[ix], truth)
            ax.set_title(f"{lab}\nerror {err:.2f} m", fontsize=9.5, color=GREEN if (k == 3 and err < 0.2) else (BRICK if k == 3 else INK))
            ax.set_aspect("equal")
            ax.set_xlim(-4, 16)
            ax.set_ylim(-3, 14)
            ax.set_xticks([])
            ax.set_yticks([])
        axes[r, 0].set_ylabel(title, fontsize=10, fontweight="bold")
    fig.suptitle("ICP on a toy floor plan: grey = the design, red = the scan, blue lines = 'nearest design point' for some scan points",
                 fontsize=10.5, y=0.99)
    fig.tight_layout()
    save(fig, "icp_toy.png")


# ---------------------------------------------------------------- KD-tree
def fig_kdtree():
    rng = np.random.default_rng(11)
    pts = rng.random((22, 2)) * 10
    q = np.array([6.3, 3.4])
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))

    def build(ax, idx, depth, x0, x1, y0, y1, maxd):
        if len(idx) <= 1 or depth >= maxd:
            return
        axis = depth % 2
        order = idx[np.argsort(pts[idx, axis])]
        mid = len(order) // 2
        split = pts[order[mid], axis]
        if axis == 0:
            ax.plot([split, split], [y0, y1], c=[STEEL, BRICK, GREEN, AMBER][min(depth, 3)], lw=2.2 - 0.4 * depth)
            build(ax, order[:mid], depth + 1, x0, split, y0, y1, maxd)
            build(ax, order[mid:], depth + 1, split, x1, y0, y1, maxd)
        else:
            ax.plot([x0, x1], [split, split], c=[STEEL, BRICK, GREEN, AMBER][min(depth, 3)], lw=2.2 - 0.4 * depth)
            build(ax, order[:mid], depth + 1, x0, x1, y0, split, maxd)
            build(ax, order[mid:], depth + 1, x0, x1, split, y1, maxd)

    ax = axes[0]
    ax.scatter(pts[:, 0], pts[:, 1], c=INK, s=28, zorder=3)
    build(ax, np.arange(len(pts)), 0, 0, 10, 0, 10, 4)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_title("Building the tree: keep cutting the points in half\n(cut 1 blue, cut 2 red, cut 3 green, cut 4 amber)", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    ax = axes[1]
    d, i = cKDTree(pts).query(q)
    ax.scatter(pts[:, 0], pts[:, 1], c=MUTED, s=26, zorder=3)
    build(ax, np.arange(len(pts)), 0, 0, 10, 0, 10, 3)
    ax.scatter(*q, marker="*", s=260, c=BRICK, zorder=5)
    ax.scatter(*pts[i], c=GREEN, s=70, zorder=4)
    ax.add_patch(plt.Circle(q, d, fill=False, ec=BRICK, ls="--", lw=1.4))
    ax.plot([q[0], pts[i][0]], [q[1], pts[i][1]], c=GREEN, lw=2)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_aspect("equal")
    ax.set_title("Asking 'which point is nearest the star?'\nOnly the cells the dashed circle touches are checked", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    axes[0].set_aspect("equal")
    fig.tight_layout()
    save(fig, "kdtree_toy.png")


# ---------------------------------------------------------------- frames
def fig_frames():
    model = floor_plan(n_per_m=8)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    ax = axes[0]
    ax.scatter(model[:, 0], model[:, 1], s=3, c=STEEL)
    ax.annotate("", xy=(11.5, 0), xytext=(-1, 0), arrowprops=dict(arrowstyle="->", color=INK))
    ax.annotate("", xy=(0, 9.5), xytext=(0, -1), arrowprops=dict(arrowstyle="->", color=INK))
    ax.text(11.6, -0.3, "x", fontsize=11)
    ax.text(0.2, 9.6, "y", fontsize=11)
    ax.set_title("Design (BIM) frame\nthe coordinates the design model uses", fontsize=10)
    ax.set_aspect("equal")
    ax.set_xlim(-2, 13)
    ax.set_ylim(-2, 11)
    ax.axis("off")
    ax = axes[1]
    c = model.mean(0)
    moved = (model - c) @ rot(-28).T * 1.03 + c + np.array([6, 4])
    ax.scatter(moved[:, 0], moved[:, 1], s=3, c=BRICK)
    ax.scatter(model[:, 0], model[:, 1], s=2, c=MUTED, alpha=0.35)
    ax.annotate("", xy=(c[0] + 6, c[1] + 4), xytext=(c[0], c[1]),
                arrowprops=dict(arrowstyle="->", color=INK, lw=1.5, linestyle="--"))
    ax.text(c[0] + 3.4, c[1] + 0.5, "shift (metres)", fontsize=9.5)
    ax.text(-1.5, -1.0, "red = what the scanner reports: shifted, turned (heading)\nand slightly stretched (scale). Grey = where the design really is.", fontsize=9.5, color=BRICK, va="top")
    ax.set_title("The same building as the scanner reports it\nits own frame: shifted, turned, a little off in scale", fontsize=10)
    ax.set_aspect("equal")
    ax.set_xlim(-2, 20)
    ax.set_ylim(-3.5, 15)
    ax.axis("off")
    fig.tight_layout()
    save(fig, "frames_toy.png")


# ---------------------------------------------------------------- SfM
def box(ax, x, y, w, h, text, fc=LIGHT, ec=MUTED, fs=9.5, bold=False, color=INK):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc=fc, ec=ec, lw=1.2))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, fontweight="bold" if bold else "normal", color=color)


def arrow(ax, x0, y0, x1, y1, color=MUTED):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=14, color=color, lw=1.6))


def fig_sfm():
    fig, ax = plt.subplots(figsize=(13.8, 4.2))
    ax.set_xlim(0, 13.8)
    ax.set_ylim(0, 4.2)
    ax.axis("off")
    steps = [("1  Photos", "many overlapping\npictures of the\nsame building", LIGHT),
             ("2  Features", "find distinctive\nspots (corners,\ntexture) in each\nphoto: SIFT", LIGHT),
             ("3  Matching", "which spot in photo A\nis the same spot\nin photo B?", LIGHT),
             ("4  Structure from\nMotion (SfM)", "solve where every\ncamera stood, and\nthe 3D position of\nthe matched spots", "#E3EDF5"),
             ("5  Dense stereo\n(MVS)", "for every pixel,\nfind its depth by\ncomparing photos", "#E3EDF5"),
             ("6  Fused\npoint cloud", "all depth maps\nmerged into one\ndense 3D cloud", "#F9E8E2")]
    w, gap = 1.85, 0.4
    for i, (t, d, fc) in enumerate(steps):
        x = 0.2 + i * (w + gap)
        box(ax, x, 2.45, w, 1.25, t, fc=fc, bold=True, fs=9.5)
        ax.text(x + w / 2, 1.75, d, ha="center", va="top", fontsize=8.8, color=MUTED)
        if i < len(steps) - 1:
            arrow(ax, x + w + 0.03, 3.07, x + w + gap - 0.03, 3.07)
    ax.plot([0.2 + 3 * (w + gap), 0.2 + 4 * (w + gap) - gap], [4.0, 4.0], c=STEEL, lw=3)
    ax.text(0.2 + 3.5 * (w + gap) - gap / 2, 4.08, "sparse model", ha="center", fontsize=9, color=STEEL)
    ax.plot([0.2 + 4 * (w + gap), 0.2 + 5 * (w + gap) + w], [4.0, 4.0], c=BRICK, lw=3)
    ax.text(0.2 + 4.5 * (w + gap) + w / 2, 4.08, "dense model", ha="center", fontsize=9, color=BRICK)
    save(fig, "sfm_steps.png")


def fig_triangulation():
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1.15, 1]})
    ax = axes[0]
    P = np.array([5.0, 6.0])
    cams = [np.array([1.5, 0.8]), np.array([8.3, 0.8])]
    for k, c in enumerate(cams):
        ax.add_patch(Polygon([c + [-0.35, -0.2], c + [0.35, -0.2], c + [0, 0.35]], fc=INK))
        d = (P - c) / np.linalg.norm(P - c)
        ax.plot([c[0], P[0]], [c[1], P[1]], c=[STEEL, BRICK][k], lw=2)
        perp = np.array([-d[1], d[0]])
        m = c + d * 1.6
        ax.plot([m[0] - perp[0] * 1.0, m[0] + perp[0] * 1.0], [m[1] - perp[1] * 1.0, m[1] + perp[1] * 1.0], c=MUTED, lw=3)
        ax.text(c[0], 0.05, f"camera {k + 1}", ha="center", fontsize=9.5)
    ax.scatter(*P, s=140, c=GREEN, zorder=5)
    ax.text(P[0] + 0.25, P[1] + 0.1, "3D point\n(where the two rays cross)", fontsize=9.5)
    ax.text(3.6, 2.9, "image 1", fontsize=8.5, color=MUTED, rotation=60)
    ax.text(7.2, 2.5, "image 2", fontsize=8.5, color=MUTED, rotation=-62)
    ax.set_xlim(0, 10)
    ax.set_ylim(-0.3, 7.5)
    ax.axis("off")
    ax.set_title("Triangulation: the same spot seen from two places\ngives a ray from each camera; the rays cross at the 3D point", fontsize=10)
    ax = axes[1]
    ax.axis("off")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 8)
    ax.scatter([3.0], [5.0], s=70, c=GREEN, zorder=5)
    ax.scatter([3.7], [4.5], s=70, facecolors="none", edgecolors=BRICK, lw=2, zorder=5)
    ax.plot([3.0, 3.7], [5.0, 4.5], c=BRICK, lw=1.6)
    ax.text(0.2, 6.6, "Reprojection error", fontsize=11, fontweight="bold")
    ax.text(0.2, 5.9, "Project the solved 3D point back into a photo.", fontsize=9.5)
    ax.text(4.2, 5.2, "green: where the feature really is", fontsize=9, color=GREEN)
    ax.text(4.2, 4.5, "red ring: where the model says", fontsize=9, color=BRICK)
    ax.text(0.2, 3.0, "The gap, in pixels, is the reprojection error.\nBundle adjustment moves all cameras and points a little\nuntil the total gap is as small as possible.\nOurs: 0.4-0.55 pixels = a very consistent model.", fontsize=9.5, va="top")
    save(fig, "triangulation.png")


# ---------------------------------------------------------------- pipeline
def fig_pipeline():
    fig, ax = plt.subplots(figsize=(13, 5.6))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 5.6)
    ax.axis("off")
    nodes = [("Design model\n(IFC file)", LIGHT), ("P1  Parse\nP2  16 stages", LIGHT), ("P3  Simulate\nscans", "#F9E8E2"),
             ("Registration\n(align scan\nto design)", "#E3EDF5"), ("Match points\nto elements", LIGHT), ("Visibility\ncheck", "#F9E8E2"),
             ("P7  Score\n+ error bars", "#F9E8E2")]
    w, gap = 1.5, 0.35
    xs = [0.2 + i * (w + gap) for i in range(len(nodes))]
    for i, ((t, fc), x) in enumerate(zip(nodes, xs)):
        box(ax, x, 3.0, w, 1.35, t, fc=fc, bold=True, fs=9.3)
        if i < len(nodes) - 1:
            arrow(ax, x + w + 0.02, 3.67, x + w + gap - 0.02, 3.67)
    files = ["bim_complete.csv", "manifest.json\n(answer key)", "stage .npz\n(point cloud)", "registration.json", "matches.json", "visibility.json", "status.csv\nmain_table.csv"]
    for x, f in zip(xs, files):
        ax.text(x + w / 2, 2.75, f, ha="center", va="top", fontsize=8.2, color=MUTED, style="italic")
    # answer key
    box(ax, xs[1] - 0.2, 0.35, 3.3, 1.0, "Answer key: which elements\nreally exist at each stage", fc="#FFF3D6", ec=AMBER, fs=9)
    arrow(ax, xs[1] + 0.75, 3.0 - 0.0, xs[1] + 0.75, 1.37, color=AMBER)
    ax.add_patch(FancyArrowPatch((xs[1] + 3.05, 0.85), (xs[6] + w / 2, 0.85), arrowstyle="-", color=AMBER, lw=1.6, linestyle="--"))
    arrow(ax, xs[6] + w / 2, 0.85, xs[6] + w / 2, 2.55, color=AMBER)
    ax.text((xs[1] + 3.05 + xs[6]) / 2 + 0.3, 0.98, "used only here, to grade the result", fontsize=9, color=AMBER, ha="center")
    ax.text(0.2, 5.2, "Red-tinted boxes were changed or added since the mid-term; blue was rebuilt.", fontsize=9.5, color=MUTED)
    ax.text(0.2, 4.85, "The pipeline never sees the answer key until the final grading step.", fontsize=9.5, color=MUTED)
    save(fig, "pipeline.png")


# ---------------------------------------------------------------- raycasting
def fig_raycast():
    fig, ax = plt.subplots(figsize=(8, 4.4))
    ax.add_patch(plt.Rectangle((4, 1), 4, 4, fc=LIGHT, ec=INK, lw=2))
    ax.add_patch(plt.Rectangle((5.2, 1), 0.2, 2.2, fc=INK))
    cam = np.array([0.8, 3.0])
    ax.scatter(*cam, marker="^", s=170, c=INK, zorder=5)
    ax.text(0.2, 3.5, "virtual\ncamera", fontsize=9.5)
    for y in np.linspace(1.6, 4.6, 9):
        end = np.array([4.0, y])
        ax.plot([cam[0], end[0]], [cam[1], end[1]], c=STEEL, lw=0.8, alpha=0.8)
        ax.scatter(*end, s=24, c=BRICK, zorder=4)
    ax.text(4.15, 5.25, "each ray stops at the first surface it hits;\nthat hit becomes one point of the cloud", fontsize=9.5)
    ax.set_xlim(0, 9)
    ax.set_ylim(0.5, 6.2)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("Ray casting: fake scan = shoot rays at the design model", fontsize=10.5)
    save(fig, "raycast_toy.png")


if __name__ == "__main__":
    fig_icp()
    fig_kdtree()
    fig_frames()
    fig_sfm()
    fig_triangulation()
    fig_pipeline()
    fig_raycast()
