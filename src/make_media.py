"""Make media/ -- pictures, animations (GIF) and 3D files (PLY) that show what
the pipeline does and how the results changed. Open media/README.md for the
index.

This is a visualisation / evaluation script, so unlike the pipeline phases it
reads the ground truth on purpose (stage manifests, gt_* arrays in the cloud
npz files) to show what is really there next to what the pipeline concluded.
"Old pipeline" results are the per-stage status files of the first commit
(`git show d823a2c:data/status/...`) and results/experiments/main_table_0_before.csv.

Rendering: no OpenGL here, so BIM meshes are drawn by raycasting (Open3D's
RaycastingScene, same as the scan simulator) with simple shading and element
outlines, and point clouds are splatted with a depth buffer. Everything is
seeded, so reruns give identical files. Needs the outputs of P2-P7 and
data/bim/bim_meshes.npz (made by p4_visibility.py); does not rerun any phase,
except that the "sensor" section re-simulates one stage in memory.

Run: .venv/bin/python src/make_media.py [--only charts,truth,scans,sensor,clip,registration,visibility,errors,models,flow,site,colors]
Output: media/ (folders 00_overview ... 10_materials_color)
"""
import argparse
import glob
import io
import json
import os
import subprocess
import sys

import numpy as np
import open3d as o3d
import pandas as pd
from PIL import Image

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Patch  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))

OUT = "media"
STATES_DIR = "data/states"
STATUS_DIR = "data/status"
MATCHES_DIR = "data/matches"
CLOUD_DIR = "data/pointclouds"
MESH_CACHE = "data/bim/bim_meshes.npz"
SAMPLES = "data/bim/bim_surface_samples.npz"
BIM_CSV = "data/bim/bim_complete.csv"
EXP_DIR = "results/experiments"
OLD_COMMIT = "d823a2c"
SEED = 0

VERSIONS = [  # (file, short label) -- the pipeline as it changed, in order
    ("main_table_0_before.csv", "0. original"),
    ("main_table_1_clipfix.csv", "1. half-built wall fix"),
    ("main_table_2_clipfix_visibility.csv", "2. + visibility check"),
    ("main_table_3_gnss_plan_v1.csv", "3. + GPS offsets, new registration v1"),
    ("main_table_4_gnss_plan_v2.csv", "4. + registration v2 (current)"),
]
KEY_STAGES = ["03_00_begane_grond_envelope", "07_01_eerste_verdieping_interior",
              "11_03_derde_verdieping_structure", "15_04_dak_envelope"]

BG = np.array([1.0, 1.0, 1.0])
GHOST = np.array([0.86, 0.87, 0.89])
TYPE_COLOR = {"IfcWall": (0.60, 0.63, 0.67), "IfcSlab": (0.72, 0.68, 0.60),
              "IfcBeam": (0.40, 0.56, 0.70), "IfcColumn": (0.90, 0.50, 0.30)}
BUILT, PARTIAL = (0.33, 0.47, 0.66), (0.96, 0.52, 0.09)
ERR_COLOR = {"correct": (0.33, 0.64, 0.33), "missed": (0.89, 0.27, 0.27),
             "false": (0.70, 0.25, 0.75), "unsure": (0.95, 0.80, 0.20)}
ERR_LABEL = {"correct": "built, found", "missed": "built, but called not started",
             "false": "not built, but called built", "unsure": "built, but marked unsure"}


# ------------------------------------------------------------------ data

def stages():
    return sorted(os.path.basename(p) for p in glob.glob(os.path.join(STATES_DIR, "*")))


def short(stage):
    n, rest = stage.split("_", 1)
    floor = {"m1": "foundation", "00": "ground fl.", "01": "1st fl.", "02": "2nd fl.",
             "03": "3rd fl.", "04": "roof"}[rest.split("_")[0]]
    return f"{n} {floor} {rest.rsplit('_', 1)[1]}"


def manifest(stage):
    return json.load(open(os.path.join(STATES_DIR, stage, "manifest.json"), encoding="utf-8"))


def truth_of(stage):
    """gid -> 1.0 built / completion fraction partial; missing = not built."""
    m = manifest(stage)
    t = {g: 1.0 for g in m["kept_ids"]}
    t.update({p["global_id"]: p["completion_fraction"] for p in m["partial_ids"]})
    return t


def status_new(stage):
    return pd.read_csv(os.path.join(STATUS_DIR, stage, "status.csv")).set_index("global_id")["status"]


def status_old(stage):
    txt = subprocess.run(["git", "show", f"{OLD_COMMIT}:{STATUS_DIR}/{stage}/status.csv"],
                         capture_output=True, text=True, check=True).stdout
    return pd.read_csv(io.StringIO(txt)).set_index("global_id")["status"]


class Bim:
    """Element meshes (BIM frame) with per-element triangle lookup."""

    def __init__(self):
        d = np.load(MESH_CACHE)
        self.verts, self.faces, self.tri_gid = d["verts"].astype(np.float64), d["faces"], d["tri_gid"]
        self.gids, self.tri_elem = np.unique(self.tri_gid, return_inverse=True)
        self.gid_index = {g: i for i, g in enumerate(self.gids)}
        types = pd.read_csv(BIM_CSV).set_index("global_id")["ifc_type"]
        self.type = np.array([types.get(g, "IfcWall") for g in self.gids])
        self.center = self.verts.mean(axis=0)
        self.radius = float(np.linalg.norm(self.verts[:, :2] - self.center[:2], axis=1).max())
        self.zmin, self.zmax = self.verts[:, 2].min(), self.verts[:, 2].max()

    def elem_mask(self, gid_set):
        e = np.array([g in gid_set for g in self.gids])
        return e[self.tri_elem]

    def clamped_verts(self, partial):
        """Vertices with partial elements cut to their completion fraction (as P3 does)."""
        v = self.verts.copy()
        for g, frac in partial.items():
            if g not in self.gid_index:
                continue
            idx = np.unique(self.faces[self.tri_elem == self.gid_index[g]])
            z0, z1 = v[idx, 2].min(), v[idx, 2].max()
            v[idx, 2] = np.minimum(v[idx, 2], z0 + frac * (z1 - z0))
        return v


# ------------------------------------------------------------------ rendering

def view(bim, az=-55, el=32, dist=2.7, center=None, radius=None):
    c = bim.center.copy() if center is None else center
    c[2] = (bim.zmin + bim.zmax) / 2 if center is None else c[2]
    r = bim.radius if radius is None else radius
    a, e = np.radians(az), np.radians(el)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    return c + d * r * dist, c


def _basis(eye, target):
    f = target - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0])
    r /= np.linalg.norm(r)
    return f, r, np.cross(r, f)


def _rays(eye, target, w, h, fov):
    f, r, u = _basis(eye, target)
    F = (w / 2) / np.tan(np.radians(fov) / 2)
    X, Y = np.meshgrid((np.arange(w) + 0.5 - w / 2) / F, (np.arange(h) + 0.5 - h / 2) / F)
    d = f + X[..., None] * r - Y[..., None] * u
    d /= np.linalg.norm(d, axis=2, keepdims=True)
    o = np.broadcast_to(eye, d.shape)
    return np.concatenate([o, d], axis=2).astype(np.float32)


def _cast(verts, faces, rays):
    scene = o3d.t.geometry.RaycastingScene()
    if faces.shape[0]:
        mesh = o3d.t.geometry.TriangleMesh()
        mesh.vertex.positions = o3d.core.Tensor(verts.astype(np.float32))
        mesh.triangle.indices = o3d.core.Tensor(faces.astype(np.uint32))
        scene.add_triangles(mesh)
    hit = scene.cast_rays(o3d.core.Tensor(rays))
    return hit["t_hit"].numpy(), hit["primitive_ids"].numpy(), hit["primitive_normals"].numpy()


def render(verts, faces, tri_color, eye, target, w=900, h=640, fov=42, tri_elem=None,
           ghost_faces=None, points=None, ss=2, ghost_color=GHOST):
    """Shaded raycast render of faces (colours per triangle), optional light-grey
    'ghost' of other triangles (e.g. the not-yet-built rest of the building),
    optional point overlays [(xyz, rgb 0-1, px size, depth-tested)]."""
    W, H = w * ss, h * ss
    rays = _rays(eye, target, W, H, fov)
    t, prim, nrm = _cast(verts, faces, rays)
    hit = np.isfinite(t)
    f, r, _u = _basis(eye, target)
    light = -0.55 * f + 0.75 * np.array([0, 0, 1.0]) + 0.35 * r
    light /= np.linalg.norm(light)
    img = np.broadcast_to(BG, (H, W, 3)).copy()
    shade = 0.5 + 0.5 * np.abs(nrm @ light)
    img[hit] = np.asarray(tri_color)[prim[hit]] * shade[hit, None]
    if tri_elem is not None:  # darken element outlines
        eid = np.where(hit, tri_elem[np.where(hit, prim, 0)], -1)
        edge = np.zeros((H, W), bool)
        edge[:, 1:] |= eid[:, 1:] != eid[:, :-1]
        edge[1:, :] |= eid[1:, :] != eid[:-1, :]
        img[edge & hit] *= 0.55
    if ghost_faces is not None and ghost_faces.shape[0]:
        tg, _pg, ng = _cast(verts, ghost_faces, rays)
        gh = np.isfinite(tg)
        gcol = np.asarray(ghost_color) * (0.85 + 0.15 * np.abs(ng @ light))[..., None]
        only = gh & ~hit
        img[only] = gcol[only]
        front = gh & hit & (tg < t - 0.05)
        img[front] = 0.6 * img[front] + 0.4 * gcol[front]
    for xyz, rgb, size, ztest in (points or []):
        F = (W / 2) / np.tan(np.radians(fov) / 2)
        p = xyz - eye
        z = p @ f
        ok = z > 0.1
        p, z, col = p[ok], z[ok], np.asarray(rgb)[ok] if np.ndim(rgb) == 2 else np.broadcast_to(rgb, (ok.sum(), 3))
        _, rr, uu = _basis(eye, target)
        x = (W / 2 + F * (p @ rr) / z).astype(int)
        y = (H / 2 - F * (p @ uu) / z).astype(int)
        dist = np.linalg.norm(p, axis=1)
        order = np.argsort(-dist)
        x, y, col, dist = x[order], y[order], col[order], dist[order]
        s = size * ss
        for dx in range(-(s // 2), s - s // 2):
            for dy in range(-(s // 2), s - s // 2):
                xx, yy = x + dx, y + dy
                inb = (xx >= 0) & (yy >= 0) & (xx < W) & (yy < H)
                if ztest:
                    inb[inb] &= ~(dist[inb] > t[yy[inb], xx[inb]] + 0.08)
                img[yy[inb], xx[inb]] = col[inb]
    img = img.reshape(h, ss, w, ss, 3).mean(axis=(1, 3))
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)


def fig_to_pil(fig, dpi=100):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, facecolor="white")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def panels(images, titles, suptitle=None, legend=None, notes=None, ncols=None, scale=1.0):
    n = len(images)
    ncols = ncols or n
    nrows = int(np.ceil(n / ncols))
    h0, w0 = images[0].shape[:2]
    fw = ncols * w0 / 100 * scale
    fh = nrows * (h0 / 100 * scale + 0.45) + (0.6 if suptitle else 0) + (0.5 if legend else 0)
    fig, axes = plt.subplots(nrows, ncols, figsize=(fw, fh), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for i, (im, t) in enumerate(zip(images, titles)):
        ax = axes.flat[i]
        ax.imshow(im)
        ax.set_title(t, fontsize=11)
        if notes and notes[i]:
            ax.text(0.01, 0.02, notes[i], transform=ax.transAxes, fontsize=9, va="bottom",
                    bbox=dict(facecolor="white", alpha=0.85, edgecolor="#bbb"))
    if suptitle:
        fig.suptitle(suptitle, fontsize=14)
    if legend:
        fig.legend(handles=[Patch(color=c, label=l) for l, c in legend], loc="lower center",
                   ncol=len(legend), fontsize=10, frameon=False)
    fig.tight_layout(rect=(0, 0.05 if legend else 0, 1, 0.97 if suptitle else 1))
    return fig


def save_gif(frames, path, ms=900):
    pal = [f.convert("P", palette=Image.ADAPTIVE, colors=160) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=ms, loop=0, optimize=True)


def out_dir(name):
    d = os.path.join(OUT, name)
    os.makedirs(d, exist_ok=True)
    return d


# ------------------------------------------------------------------ 01 charts

def charts():
    d = out_dir("01_results_charts")
    tabs = [(pd.read_csv(os.path.join(EXP_DIR, f)), lab) for f, lab in VERSIONS]
    labels = [short(s) for s in tabs[0][0]["stage"]]
    x = np.arange(len(labels))

    # signed error per stage, every version
    fig, ax = plt.subplots(figsize=(12, 5))
    for (t, lab), c in zip(tabs, ["#999", "#e8a33d", "#7aa6d6", "#b27fc4", "#2e8b57"]):
        signed = t["progress_est_count"] - t["progress_gt_count"]
        ax.plot(x, signed, "o-", color=c, lw=2.6 if "current" in lab else 1.4, label=lab)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xticks(x, labels, rotation=40, ha="right")
    ax.set_ylabel("estimated - true progress (% points)")
    ax.set_title("Progress error per stage, every version of the pipeline (0 = perfect; below 0 = too low)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "progress_error_per_stage_all_versions.png"), dpi=110)
    plt.close(fig)

    # headline bars
    fig, ax = plt.subplots(figsize=(9, 4.5))
    means = [t["error_count"].mean() for t, _ in tabs]
    worst = [t["error_count"].max() for t, _ in tabs]
    xx = np.arange(len(tabs))
    ax.bar(xx - 0.2, means, 0.4, label="average error", color="#4c78a8")
    ax.bar(xx + 0.2, worst, 0.4, label="worst stage", color="#e45756")
    for i, (m, w) in enumerate(zip(means, worst)):
        ax.text(i - 0.2, m + 0.1, f"{m:.1f}", ha="center", fontsize=9)
        ax.text(i + 0.2, w + 0.1, f"{w:.1f}", ha="center", fontsize=9)
    ax.set_xticks(xx, [lab for _, lab in tabs], rotation=15, ha="right", fontsize=9)
    ax.set_ylabel("progress error (% points)")
    ax.set_title("How far off the progress number is, version by version")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(d, "progress_error_summary.png"), dpi=110)
    plt.close(fig)

    # estimated vs true progress: old vs current
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8), sharey=True)
    for ax, (t, lab) in zip(axes, [tabs[0], tabs[-1]]):
        ax.plot(x, t["progress_gt_count"], "k-", lw=3, alpha=0.35, label="true progress")
        ax.plot(x, t["progress_est_count"], "o-", color="#4c78a8", label="pipeline estimate")
        ax.set_xticks(x, labels, rotation=40, ha="right", fontsize=8)
        ax.set_title(lab)
        ax.grid(alpha=0.3)
        ax.legend()
    axes[0].set_ylabel("progress (% of elements)")
    fig.suptitle("Estimated vs true progress, old pipeline vs current")
    fig.tight_layout()
    fig.savefig(os.path.join(d, "progress_estimate_vs_truth_old_vs_new.png"), dpi=110)
    plt.close(fig)

    # where the current errors come from
    cur = tabs[-1][0]
    fig, ax = plt.subplots(figsize=(12, 4.8))
    bottom = np.zeros(len(cur))
    for col, lab, c in [("n_missed_built", "built, but called not started", "#e45756"),
                        ("n_false_built", "not built, but called built", "#b279a2"),
                        ("n_uncertain_built", "built, but marked unsure (left out)", "#eeca3b")]:
        ax.bar(x, cur[col], bottom=bottom, label=lab, color=c)
        bottom += cur[col]
    ax.set_xticks(x, labels, rotation=40, ha="right")
    ax.set_ylabel("number of elements")
    ax.set_title("Current pipeline: which elements it gets wrong, per stage")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(d, "error_sources_current.png"), dpi=110)
    plt.close(fig)

    # registration error per stage, current
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.bar(x, cur["registration_err_m"] * 100, color="#54a24b")
    ax.set_xticks(x, labels, rotation=40, ha="right")
    ax.set_ylabel("alignment error (cm)")
    ax.set_title("Current pipeline: how well each scan was lined up with the BIM "
                 "(started 1-3 m and up to ~15 degrees off)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(d, "registration_error_per_stage_current.png"), dpi=110)
    plt.close(fig)
    print("charts done")


# ------------------------------------------------------------------ 02 ground truth

def truth_render(bim, stage, w=900, h=640, eye_target=None):
    t = truth_of(stage)
    partial = {g: f for g, f in t.items() if f < 1.0}
    v = bim.clamped_verts(partial)
    built = bim.elem_mask(set(t))
    col = np.zeros((len(bim.faces), 3))
    col[:] = BUILT
    col[bim.elem_mask(set(partial))] = PARTIAL
    eye, target = eye_target or view(bim)
    return render(v, bim.faces[built], col[built], eye, target, w, h, tri_elem=bim.tri_elem[built],
                  ghost_faces=bim.faces[~built])


def truth():
    d = out_dir("02_ground_truth_stages")
    bim = Bim()
    legend = [("built", BUILT), ("half built", PARTIAL), ("not built yet (ghost)", GHOST)]
    imgs, titles, frames = [], [], []
    for s in stages():
        im = truth_render(bim, s, 600, 430)
        m = manifest(s)
        imgs.append(im)
        titles.append(f"{short(s)}: {m['n_fully_built']} built, {m['n_partial']} half")
        frames.append(fig_to_pil(panels([im], [f"Stage {short(s)}  (true state)"], legend=legend, scale=1.3)))
    fig = panels(imgs, titles, "Ground truth: the 16 construction stages we simulate", legend, ncols=4, scale=0.75)
    fig.savefig(os.path.join(d, "all_16_stages.png"), dpi=100)
    plt.close(fig)
    save_gif(frames, os.path.join(d, "construction_growth.gif"), ms=800)
    im = render(bim.verts, bim.faces, np.array([TYPE_COLOR.get(tp, (0.6, 0.6, 0.6))
                                                for tp in bim.type])[bim.tri_elem],
                *view(bim), 1100, 760, tri_elem=bim.tri_elem)
    fig = panels([im], ["The complete design model (BIM) we start from - 1355 elements"],
                 legend=[(k.replace("Ifc", ""), c) for k, c in TYPE_COLOR.items()])
    fig.savefig(os.path.join(d, "complete_design_model.png"), dpi=100)
    plt.close(fig)
    print("truth done")


# ------------------------------------------------------------------ 03 scans

def cloud_in_bim(stage, n=None, rng=None):
    """Stage cloud mapped into the BIM frame with the TRUE transform (for display)."""
    c = np.load(os.path.join(CLOUD_DIR, f"{stage}.npz"))
    xyz = c["xyz"].astype(np.float64)
    Ti = np.linalg.inv(c["gt_T_bim_to_cloud"])
    xyz = xyz @ Ti[:3, :3].T + Ti[:3, 3]
    eyes = c["cam_eyes"] @ Ti[:3, :3].T + Ti[:3, 3]
    rgb, cls = c["rgb"] / 255.0, c["gt_class"]
    if n and xyz.shape[0] > n:
        k = (rng or np.random.default_rng(SEED)).choice(xyz.shape[0], n, replace=False)
        xyz, rgb, cls = xyz[k], rgb[k], cls[k]
    return xyz, rgb, cls, eyes, c["cam_kinds"]


def scans():
    d = out_dir("03_simulated_scans")
    bim = Bim()
    empty = np.zeros((0, 3), np.uint32)
    eye, target = view(bim, dist=2.6)
    imgs, titles, frames = [], [], []
    for s in stages():
        xyz, rgb, _cls, _e, _k = cloud_in_bim(s, 150_000)
        im = render(bim.verts, empty, np.zeros((0, 3)), eye, target, 600, 430, points=[(xyz, rgb, 1, False)])
        imgs.append(im)
        titles.append(short(s))
        frames.append(fig_to_pil(panels([im], [f"Simulated scan, stage {short(s)}"], scale=1.3)))
    fig = panels(imgs, titles, "What the simulated drone + indoor scanner see at each stage "
                 "(points coloured by element type; brown = ground, green = trees)", ncols=4, scale=0.75)
    fig.savefig(os.path.join(d, "all_16_scans.png"), dpi=100)
    plt.close(fig)
    save_gif(frames, os.path.join(d, "scan_growth.gif"), ms=800)

    # turntable of the last stage
    s = stages()[-1]
    xyz, rgb, _cls, _e, _k = cloud_in_bim(s, 250_000)
    frames = []
    for az in range(0, 360, 12):
        e2, t2 = view(bim, az=az, el=28, dist=2.4)
        im = render(bim.verts, empty, np.zeros((0, 3)), e2, t2, 640, 460, points=[(xyz, rgb, 1, False)])
        frames.append(Image.fromarray(im))
    save_gif(frames, os.path.join(d, "final_scan_turntable.gif"), ms=120)

    # where the cameras stand
    s = KEY_STAGES[1]
    xyz, rgb, _cls, eyes, kinds = cloud_in_bim(s, 150_000)
    t = truth_of(s)
    built = bim.elem_mask(set(t))
    pts = [(eyes[kinds == "pinhole"], (0.1, 0.4, 0.9), 5, False),
           (eyes[kinds != "pinhole"], (0.9, 0.1, 0.1), 6, False)]
    e2, t2 = view(bim, dist=3.6, el=30)
    a = render(bim.verts, bim.faces[built], np.tile(BUILT, (built.sum(), 1)), e2, t2, 800, 560,
               tri_elem=bim.tri_elem[built], points=pts)
    b = render(bim.verts, empty, np.zeros((0, 3)), e2, t2, 800, 560,
               points=[(xyz, rgb, 1, False)] + pts)
    fig = panels([a, b], ["Camera positions around the true building", "The points they collect"],
                 f"Survey of stage {short(s)}: {np.sum(kinds == 'pinhole')} drone photos in rings + "
                 f"{np.sum(kinds != 'pinhole')} indoor 360-degree scans",
                 legend=[("drone photo position", (0.1, 0.4, 0.9)), ("indoor scanner position", (0.9, 0.1, 0.1))])
    fig.savefig(os.path.join(d, "survey_camera_positions.png"), dpi=100)
    plt.close(fig)
    print("scans done")


# ------------------------------------------------------------------ 03b simple vs realistic sensor

def sensor():
    import p3_render as p3
    import p2_p3_simulate_stages as sim
    d = out_dir("03_simulated_scans")
    bim = Bim()
    s = KEY_STAGES[1]
    m = manifest(s)
    all_el = {}
    for i, g in enumerate(bim.gids):
        tris = bim.tri_elem == i
        idx, local = np.unique(bim.faces[tris], return_inverse=True)
        v = bim.verts[idx]
        all_el[g] = {"global_id": g, "ifc_type": bim.type[i], "verts": v, "faces": local.reshape(-1, 3),
                     "z_min": float(v[:, 2].min()), "z_max": float(v[:, 2].max())}
    allv = bim.verts
    fc = allv.mean(axis=0)
    fr = float(np.linalg.norm(allv[:, :2] - fc[:2], axis=1).max())
    clutter_rng = np.random.default_rng(p3.SEED + 999)
    gt = (*sim.make_ground(fc, fr, float(allv[:, 2].min()), clutter_rng),
          *sim.make_trees(fc, fr, float(allv[:, 2].min()), clutter_rng))
    out = {}
    for mode in ["simple", "realistic"]:
        p3.SENSOR, p3.GEOREF = mode, "small"
        rng = np.random.default_rng(p3.SEED + 1000 + m["stage_index"])
        els = p3.stage_elements_from_manifest(m, all_el, rng)
        pts, col, cls, _gid, *_rest, T, _e, _k, _t = p3.run_stage(els, rng, gt)
        Ti = np.linalg.inv(T)
        out[mode] = (pts @ Ti[:3, :3].T + Ti[:3, 3], col / 255.0, cls)

    # thin horizontal slice through the first floor walls, seen from above
    z0 = 3.0 + 1.2
    fig, axes = plt.subplots(2, 2, figsize=(14, 14))
    for col, mode in enumerate(["simple", "realistic"]):
        xyz, rgb, cls = out[mode]
        sl = np.abs(xyz[:, 2] - z0) < 0.15
        noise = cls == "noise"
        for row in range(2):
            ax = axes[row, col]
            ax.scatter(xyz[sl & ~noise, 0], xyz[sl & ~noise, 1], s=0.4 if row == 0 else 2, c="#333")
            ax.scatter(xyz[sl & noise, 0], xyz[sl & noise, 1], s=3 if row == 0 else 12, c="#e45756",
                       label="ghost / floater points")
            ax.set_aspect("equal")
            if row == 0:
                ax.set_xlim(-2, 25)
                ax.set_ylim(-2, 24)
                ax.set_title(f"{mode} sensor: {xyz.shape[0]:,} points total, {noise.sum():,} junk points")
            else:
                ax.set_xlim(4, 8)
                ax.set_ylim(5, 9)
                ax.set_title(f"{mode} sensor, zoom 4 x 4 m (wall thickness ~0.1-0.3 m)")
            ax.legend(loc="upper right")
    fig.suptitle(f"Top view of a 30 cm thick slice through the first-floor walls (stage {short(s)}).\n"
                 "Simple: same 1.5 cm noise everywhere. Realistic: noise grows with distance and slanted "
                 "view, holes, missing grazing surfaces, ghost points at edges.", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(d, "sensor_simple_vs_realistic_slice.png"), dpi=100)
    plt.close(fig)

    print("sensor done")


# ------------------------------------------------------------------ 04 half-built wall fix

def clip():
    d = out_dir("04_fix_half_built_walls")
    bim = Bim()
    s = KEY_STAGES[1]
    t = truth_of(s)
    partial = {g: f for g, f in t.items() if f < 1.0}
    full = {g for g, f in t.items() if f == 1.0}

    # old clipping: drop every face with a vertex above the cut height
    keep_tri = np.ones(len(bim.faces), bool)
    for g, frac in partial.items():
        tris = np.nonzero(bim.tri_elem == bim.gid_index[g])[0]
        z = bim.verts[bim.faces[tris], 2]
        z0, z1 = z.min(), z.max()
        keep_tri[tris] = (z <= z0 + frac * (z1 - z0) + 1e-9).all(axis=1)
    new_v = bim.clamped_verts(partial)
    is_full, is_part = bim.elem_mask(full), bim.elem_mask(set(partial))
    col = np.zeros((len(bim.faces), 3))
    col[:] = BUILT
    col[is_part] = PARTIAL
    eye, target = view(bim, az=-60, el=40, dist=2.3)

    def area(v, tris):
        p = v[bim.faces[tris]]
        return 0.5 * np.linalg.norm(np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), axis=1).sum()

    ims, notes = [], []
    for v, sel, lab in [(bim.verts, is_full | (is_part & keep_tri), "old"), (new_v, is_full | is_part, "new")]:
        ims.append(render(v, bim.faces[sel], col[sel], eye, target, 760, 540, tri_elem=bim.tri_elem[sel],
                          ghost_faces=bim.faces[~(is_full | is_part)]))
        notes.append(f"surface of the {len(partial)} half-built elements left for the scanner: "
                     f"{area(v, sel & is_part):.0f} m2")
    fig = panels(ims, ["BEFORE the fix (old clipping)", "AFTER the fix (clamp to cut height)"],
                 f"Stage {short(s)}: what the scanner gets for half-built walls (orange)", notes=notes,
                 legend=[("built", BUILT), ("half built", PARTIAL), ("not built", GHOST)])
    fig.savefig(os.path.join(d, "half_built_walls_stage_before_after.png"), dpi=100)
    plt.close(fig)

    # one wall, close up
    g = max(partial, key=lambda g: np.ptp(bim.verts[np.unique(bim.faces[bim.tri_elem == bim.gid_index[g]])], axis=0)[:2].max()
            if 0.4 < partial[g] < 0.7 else 0)
    tris = bim.tri_elem == bim.gid_index[g]
    vv = bim.verts[np.unique(bim.faces[tris])]
    c = vv.mean(axis=0)
    r = float(np.ptp(vv, axis=0).max())
    eye, target = view(bim, az=-60, el=25, dist=1.4, center=c, radius=r)
    cut_v = bim.clamped_verts({g: partial[g]})
    ims = [render(bim.verts, bim.faces[tris], np.tile(GHOST, (tris.sum(), 1)), eye, target, 600, 450),
           render(bim.verts, bim.faces[tris & keep_tri], np.tile(PARTIAL, ((tris & keep_tri).sum(), 1)),
                  eye, target, 600, 450, ghost_faces=bim.faces[tris]),
           render(cut_v, bim.faces[tris], np.tile(PARTIAL, (tris.sum(), 1)), eye, target, 600, 450,
                  ghost_faces=None)]
    fig = panels(ims, ["The full wall in the design", f"Old: {partial[g]:.0%} built -> only the bottom face left",
                       f"New: {partial[g]:.0%} built -> wall cut to that height"],
                 "Why half-built walls used to vanish: the old code deleted every face that reached above "
                 "the cut, which on a wall is every side face")
    fig.savefig(os.path.join(d, "one_wall_before_after.png"), dpi=100)
    plt.close(fig)
    print("clip done")


# ------------------------------------------------------------------ 05 registration

def registration_media():
    import registration as reg
    d = out_dir("05_registration")
    s = KEY_STAGES[1]
    c = np.load(os.path.join(CLOUD_DIR, f"{s}.npz"))
    rng = np.random.default_rng(SEED)
    xyz = c["xyz"].astype(np.float64)
    xyz = xyz[c["gt_class"] != "ground"]
    xyz = xyz[rng.choice(xyz.shape[0], 80_000, replace=False)]
    T = np.array(json.load(open(os.path.join(MATCHES_DIR, s, "registration.json")))["T_cloud_to_bim"])
    al = xyz @ T[:3, :3].T + T[:3, 3]
    smp = np.load(SAMPLES)["xyz"]
    smp = smp[rng.choice(smp.shape[0], 400_000, replace=False)].astype(np.float64)

    def plan(ax, cloud, title, zoom=None):
        band = lambda p: p[(p[:, 2] > 3.5) & (p[:, 2] < 5.0)]  # first-floor walls
        b, q = band(smp), band(cloud)
        ax.scatter(b[:, 0], b[:, 1], s=0.3, c="#9aa", label="BIM (design)")
        ax.scatter(q[:, 0], q[:, 1], s=0.3, c="#e45756", label="scan")
        ax.set_aspect("equal")
        ax.set_title(title)
        lo, hi = smp[:, :2].min(axis=0) - 2, smp[:, :2].max(axis=0) + 2
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        if zoom is not None:
            ax.set_xlim(zoom[0], zoom[0] + 4)
            ax.set_ylim(zoom[1], zoom[1] + 4)
        ax.legend(markerscale=12, loc="upper right")

    corner = (3.5, 4.5)  # a wall corner on the first floor
    fig, axes = plt.subplots(2, 2, figsize=(14, 13))
    plan(axes[0, 0], xyz, "BEFORE: scan as it arrives (GPS-level error)")
    plan(axes[0, 1], al, "AFTER: scan moved onto the BIM by our registration")
    plan(axes[1, 0], xyz, "BEFORE, zoom 4 x 4 m", zoom=corner)
    plan(axes[1, 1], al, "AFTER, zoom 4 x 4 m", zoom=corner)
    err = json.load(open(os.path.join(MATCHES_DIR, s, "registration.json")))
    fig.suptitle(f"Stage {short(s)}, top view of the first-floor walls. The scan started "
                 f"{err['shift_m']:.1f} m off (plus a small turn and tilt); after registration the walls "
                 f"sit on top of each other.", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(d, "before_after_alignment.png"), dpi=90)
    plt.close(fig)

    # how the floor-plan search works: wall points, then heading scores
    pcd = reg._pcd(xyz)
    sv, shz, L = reg._split_by_normal(pcd, return_level=True)
    target = reg.BimTarget(smp, seed=SEED)
    turn = 137.0  # pretend the scan came in turned by this much (indoor case, no GPS)
    R = reg.rot_z(turn)
    cc = sv.mean(axis=0)
    sv_t = (sv - cc) @ R.T + cc
    cands = reg.plan_candidates(sv_t, shz, target, prior="none")
    yaw_best = {}
    for sc, Tc in cands:
        y = round(reg.yaw_deg(Tc)) % 360
        yaw_best[y] = max(yaw_best.get(y, 0), sc)
    ys = np.array(sorted(yaw_best))
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.6))
    ax = axes[0]
    ax.scatter(sv_t[:, 0], sv_t[:, 1], s=0.5, c="#e45756")
    ax.set_aspect("equal")
    ax.set_title(f"1. Keep only wall points (upright surfaces),\nseen from above. Scan turned {turn:.0f} deg.")
    ax = axes[1]
    ax.scatter(target.vert[::20, 0], target.vert[::20, 1], s=0.3, c="#556")
    ax.set_aspect("equal")
    ax.set_title("2. Same for the BIM: its floor plan of walls")
    ax = axes[2]
    ax.plot(ys, [yaw_best[y] for y in ys], color="#4c78a8")
    best = ys[np.argmax([yaw_best[y] for y in ys])]
    ax.axvline(best, color="#e45756", ls="--", label=f"best turn back: {best} deg (true {(-turn) % 360:.0f})")
    ax.set_xlabel("turn tried (degrees)")
    ax.set_ylabel("how well the two floor plans overlap")
    ax.set_title("3. Try every turn (1 deg steps); for each, slide the plans\nover each other (FFT) and keep the best overlap")
    ax.legend()
    fig.suptitle("How the coarse search finds the scan's position without any GPS. Then: height sweep, "
                 "check the best few candidates in 3D, fine-tune with ICP, and flag it if unsure.", fontsize=12)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "how_the_search_works.png"), dpi=90)
    plt.close(fig)

    # benchmark
    rows = []
    for f in ["results/r3_registration_bench.csv", "results/r3_registration_bench_fpfh.csv"]:
        rows.append(pd.read_csv(f))
    b = pd.concat(rows)
    v2 = pd.read_csv("results/r3_registration_bench_v2.csv")
    v2["method"] = "plan v2 (70/120 trials)"
    b = pd.concat([b, v2])
    names = {"icp_old": "old ICP", "fpfh": "feature matching\n(FPFH+RANSAC)", "plan": "floor-plan search v1",
             "plan v2 (70/120 trials)": "floor-plan search v2\n(70 of 120 trials)"}
    order = ["icp_old", "fpfh", "plan", "plan v2 (70/120 trials)"]
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
    ax = axes[0]
    for j, regime in enumerate(["gnss", "local"]):
        rates = [b[(b.method == m) & (b.regime == regime)]["success"].astype(bool).mean() * 100 for m in order]
        xx = np.arange(len(order)) + (j - 0.5) * 0.38
        ax.bar(xx, rates, 0.38, label={"gnss": "drone with GPS (1-3 m, 15 deg off)",
                                       "local": "indoor scan, no GPS (any turn, 30 m off)"}[regime])
        for xi, rr in zip(xx, rates):
            ax.text(xi, rr + 1, f"{rr:.0f}%", ha="center", fontsize=9)
    ax.set_xticks(np.arange(len(order)), [names[m] for m in order], fontsize=9)
    ax.set_ylabel("success rate (within 5 cm)")
    ax.set_ylim(0, 110)
    ax.legend(loc="center left", fontsize=9)
    ax.set_title("How often each method lines the scan up correctly")
    ax = axes[1]
    for i, m in enumerate(order):
        sub = b[b.method == m]
        jit = np.random.default_rng(i).uniform(-0.25, 0.25, len(sub))
        okc = sub["self_ok"].astype(bool).values
        ax.scatter(i + jit[okc], sub["err_m"][okc], s=9, c="#4c78a8", label="method said OK" if i == 0 else None)
        ax.scatter(i + jit[~okc], sub["err_m"][~okc], s=16, c="#e45756", marker="x",
                   label="method flagged itself" if i == 2 else None)
    ax.axhline(0.05, color="k", ls="--", lw=1, label="5 cm pass line")
    ax.set_yscale("log")
    ax.set_xticks(range(len(order)), [names[m] for m in order], fontsize=9)
    ax.set_ylabel("alignment error (m, log scale)")
    ax.set_title("Every trial's error (dangerous = above the line AND blue)")
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "benchmark_methods.png"), dpi=100)
    plt.close(fig)
    print("registration done")


# ------------------------------------------------------------------ 06 visibility

def visibility():
    d = out_dir("06_visibility_check")
    bim = Bim()
    for s in [KEY_STAGES[1], KEY_STAGES[2]]:
        st = status_new(s)
        t = truth_of(s)
        reg = json.load(open(os.path.join(MATCHES_DIR, s, "registration.json")))
        T = np.array(reg["T_cloud_to_bim"])
        c = np.load(os.path.join(CLOUD_DIR, f"{s}.npz"))
        eyes = c["cam_eyes"] @ T[:3, :3].T + T[:3, 3]
        kinds = c["cam_kinds"]
        cat = {"completed": (0.33, 0.47, 0.66), "partial": PARTIAL,
               "not_started": (0.97, 0.72, 0.72), "uncertain": (0.45, 0.45, 0.45)}
        est = np.array([st.get(g, "uncertain") for g in bim.gids])[bim.tri_elem]
        solid = est != "not_started"
        col = np.array([cat[e] for e in est[solid]])
        eye, target = view(bim, dist=3.3, el=30)
        pts = [(eyes[kinds == "pinhole"], (0.1, 0.4, 0.9), 4, False),
               (eyes[kinds != "pinhole"], (0.9, 0.1, 0.1), 6, False)]
        a = render(bim.verts, bim.faces[solid], col, eye, target, 800, 560, tri_elem=bim.tri_elem[solid],
                   ghost_faces=bim.faces[~solid], ghost_color=cat["not_started"], points=pts)
        built = bim.elem_mask(set(t))
        b = render(bim.clamped_verts({g: f for g, f in t.items() if f < 1}), bim.faces[built],
                   np.tile(BUILT, (built.sum(), 1)), eye, target, 800, 560, tri_elem=bim.tri_elem[built],
                   ghost_faces=bim.faces[~built])
        n = st.value_counts()
        fig = panels([a, b], ["What the pipeline decided", "The truth"],
                     f"Visibility check, stage {short(s)}: an element we did not find is only called "
                     "'not started' if the cameras would have seen it",
                     legend=[("found (built)", cat["completed"]), ("found half built", PARTIAL),
                             ("not found, cameras would have seen it -> not started (see-through)",
                              cat["not_started"]),
                             ("not found, hidden from cameras -> unsure, left out", cat["uncertain"]),
                             ("indoor scanner positions (blue dots: drone photos)", (0.9, 0.1, 0.1))],
                     notes=[f"{n.get('completed', 0)} built, {n.get('partial', 0)} half, "
                            f"{n.get('not_started', 0)} not started, {n.get('uncertain', 0)} unsure", ""])
        fig.savefig(os.path.join(d, f"visibility_{s[:2]}.png"), dpi=100)
        plt.close(fig)
    print("visibility done")


# ------------------------------------------------------------------ 07 old vs new per element

def error_classes(status, t, gids):
    out = np.full(len(gids), "", dtype=object)
    for i, g in enumerate(gids):
        s = status.get(g, "uncertain")
        b = g in t
        if b and s in ("completed", "partial"):
            out[i] = "correct"
        elif b and s == "not_started":
            out[i] = "missed"
        elif not b and s in ("completed", "partial"):
            out[i] = "false"
        elif b and s == "uncertain":
            out[i] = "unsure"
    return out


def errors_render(bim, s, status, w=600, h=430):
    t = truth_of(s)
    cls = error_classes(status, t, bim.gids)
    tri_cls = cls[bim.tri_elem]
    sel = tri_cls != ""
    col = np.array([ERR_COLOR.get(c, (0, 0, 0)) for c in tri_cls[sel]])
    v = bim.clamped_verts({g: f for g, f in t.items() if f < 1})
    im = render(v, bim.faces[sel], col, *view(bim), w, h, tri_elem=bim.tri_elem[sel], ghost_faces=bim.faces[~sel])
    counts = {k: int((cls == k).sum()) for k in ERR_COLOR}
    return im, counts


def errors():
    d = out_dir("07_old_vs_new_per_element")
    bim = Bim()
    old_tab = pd.read_csv(os.path.join(EXP_DIR, VERSIONS[0][0])).set_index("stage")
    new_tab = pd.read_csv(os.path.join(EXP_DIR, VERSIONS[-1][0])).set_index("stage")
    legend = [(ERR_LABEL[k], c) for k, c in ERR_COLOR.items()] + [("not built (ghost)", GHOST)]
    frames = []
    for s in stages():
        ims, notes = [truth_render(bim, s, 600, 430)], [""]
        for lab, stt, tab in [("old", status_old(s), old_tab), ("new", status_new(s), new_tab)]:
            im, cnt = errors_render(bim, s, stt)
            r = tab.loc[s]
            ims.append(im)
            notes.append(f"progress {r.progress_est_count:.1f}% vs true {r.progress_gt_count:.1f}%\n"
                         f"missed {cnt['missed']}, false {cnt['false']}, unsure {cnt['unsure']}")
        fig = panels(ims, ["Truth (blue built, orange half)", "OLD pipeline", "NEW pipeline"],
                     f"Stage {short(s)}: element by element", legend, notes=notes)
        im = fig_to_pil(fig, dpi=80)
        frames.append(im)
        if s in KEY_STAGES:
            im.save(os.path.join(d, f"stage_{s[:2]}_old_vs_new.png"))
    save_gif(frames, os.path.join(d, "all_stages_old_vs_new.gif"), ms=1500)
    print("errors done")


# ------------------------------------------------------------------ 09 busy site (R1)

def busy_site():
    import site_clutter as sc
    d = out_dir("09_busy_site")
    bim = Bim()
    all_el = {}
    for i, g in enumerate(bim.gids):
        idx, local = np.unique(bim.faces[bim.tri_elem == i], return_inverse=True)
        v = bim.verts[idx]
        all_el[g] = {"global_id": g, "ifc_type": bim.type[i], "verts": v, "faces": local.reshape(-1, 3),
                     "z_min": float(v[:, 2].min()), "z_max": float(v[:, 2].max())}
    site = sc.Site(all_el)
    dev = sc.deviations(all_el.keys())
    ims, titles = [], []
    for s in ["05_01_eerste_verdieping_structure", "10_02_tweede_verdieping_interior"]:
        m = manifest(s)
        t = truth_of(s)
        partial = {g: f for g, f in t.items() if f < 1}
        vv = bim.clamped_verts(partial)
        els = []
        for g in t:
            if g in all_el:
                e = dict(all_el[g])
                idx = np.unique(bim.faces[bim.tri_elem == bim.gid_index[g]])
                e["verts"] = vv[idx]
                els.append(e)
        els = sc.apply_deviations(els, dev)
        clutter = sc.stage_clutter(site, els, m["stage_index"], m["phase"], is_last=False)
        vs, fs, cs, off = [], [], [], 0
        for e in els + clutter:
            vs.append(e["verts"])
            fs.append(e["faces"] + off)
            off += len(e["verts"])
            if e["global_id"] in dev:
                c = (0.85, 0.1, 0.6)
            elif e["global_id"]:
                c = PARTIAL if e["global_id"] in partial else BUILT
            else:
                c = np.array(sc.CLUTTER_COLOR[e["ifc_type"]]) / 255
            cs.append(np.tile(c, (len(e["faces"]), 1)))
        V, F, C = np.concatenate(vs), np.concatenate(fs), np.concatenate(cs)
        ims.append(render(V, F, C, *view(bim, dist=3.0, el=30), 800, 560))
        titles.append(f"Stage {short(s)}")
        c0 = bim.center.copy()
        c0[2] = 3.0
        ims.append(render(V, F, C, *view(bim, az=-70, el=20, dist=0.9, center=c0), 800, 560))
        titles.append(f"Stage {short(s)}, close-up")
    legend = [(k, np.array(c) / 255) for k, c in sc.CLUTTER_COLOR.items()] + \
        [("built", BUILT), ("half built", PARTIAL), ("built 4-12 cm off design", (0.85, 0.1, 0.6))]
    fig = panels(ims, titles, "Busy site (p3_render.py --site busy): things a real site has that the BIM "
                 "does not, and a few elements built slightly off", legend, ncols=2, scale=0.9)
    fig.savefig(os.path.join(d, "busy_site_world.png"), dpi=100)
    plt.close(fig)
    print("busy site done")


# ------------------------------------------------------------------ 10 materials + colour

COLOR_CLOUD_DIR = "data/pointclouds_color"   # p3_render.py --sensor realistic --site busy --out ...
COLOR_BENCH = "results/r3_registration_bench_color.csv"


def _closest(bim, xyz):
    """(distance, element index) of the nearest BIM triangle for each point."""
    scene = o3d.t.geometry.RaycastingScene()
    mesh = o3d.t.geometry.TriangleMesh()
    mesh.vertex.positions = o3d.core.Tensor(bim.verts.astype(np.float32))
    mesh.triangle.indices = o3d.core.Tensor(bim.faces.astype(np.uint32))
    scene.add_triangles(mesh)
    q = o3d.core.Tensor(xyz.astype(np.float32))
    res = scene.compute_closest_points(q)
    d = np.linalg.norm(res["points"].numpy() - xyz, axis=1)
    return d, bim.tri_elem[res["primitive_ids"].numpy()]


def colors_media():
    import materials
    import color_registration as cr
    import p3_render
    d = out_dir("10_materials_color")
    bim = Bim()
    cats = materials.load_categories()
    elem_cat = np.array([cats.get(g, materials.DEFAULT) for g in bim.gids])
    cat_rgb = {c: np.array(v) / 255 for c, v in materials.CATALOGUE_RGB.items()}
    counts = pd.Series(elem_cat).value_counts()

    # 1. what the BIM knows: every element in its catalogue colour
    tcol = np.array([cat_rgb[c] for c in elem_cat])[bim.tri_elem]
    ims = [render(bim.verts, bim.faces, tcol, *view(bim, az=az, dist=2.6), 800, 580, tri_elem=bim.tri_elem)
           for az in (-55, 125)]
    cut = bim.verts[:, 2] < 3.2   # ground floor only, to see the inside walls
    keep = cut[bim.faces].all(axis=1)
    ims.append(render(bim.verts, bim.faces[keep], tcol[keep], *view(bim, az=-60, el=60, dist=2.2), 800, 580,
                      tri_elem=bim.tri_elem[keep]))
    legend = [(f"{c} ({counts.get(c, 0)})", cat_rgb[c]) for c in counts.index]
    fig = panels(ims, ["Front", "Back", "Ground floor from above (upper floors removed)"],
                 "What the BIM says each element is made of (src/materials.py, from the IFC's own materials), "
                 "drawn in the catalogue colour the matcher knows", ncols=3, scale=0.8)
    fig.legend(handles=[Patch(color=c, label=l) for l, c in legend], loc="lower center",
               ncol=6, fontsize=10, frameon=False)
    fig.subplots_adjust(bottom=0.16)
    fig.savefig(os.path.join(d, "bim_by_material.png"), dpi=100)
    plt.close(fig)
    write_mesh(os.path.join(d, "design_model_by_material.ply"), bim.verts, bim.faces, tcol)

    # 2. catalogue vs renderer colours: different on purpose, and many greys look alike
    fig, axes = plt.subplots(1, 2, figsize=(16, 7.5), gridspec_kw={"width_ratios": [1, 1.1]})
    ax = axes[0]
    order = materials.CATEGORIES
    for i, c in enumerate(order):
        ax.add_patch(plt.Rectangle((0, -i), 1, 0.85, color=cat_rgb[c]))
        ax.add_patch(plt.Rectangle((1.1, -i), 1, 0.85, color=np.array(p3_render.RENDER_RGB[c]) / 255))
        ax.text(2.25, -i + 0.42, f"{c}  ({counts.get(c, 0)} elements with geometry)", va="center", fontsize=10)
    ax.text(0.5, 1.05, "BIM catalogue\n(matcher knows)", ha="center", fontsize=10)
    ax.text(1.6, 1.05, "renderer\n(camera sees)", ha="center", fontsize=10)
    ax.set_xlim(0, 5)
    ax.set_ylim(-len(order) + 0.5, 1.8)
    ax.axis("off")
    ax.set_title("Two separate colour tables")
    ax = axes[1]
    for c in order:
        a = cr.chroma(np.array([materials.CATALOGUE_RGB[c]]))[0]
        b = cr.chroma(np.array([p3_render.RENDER_RGB[c]]))[0]
        ax.plot([a[0], b[0]], [a[1], b[1]], color="#999", lw=0.8)
        ax.scatter(*a, s=140, color=cat_rgb[c], edgecolor="k", marker="o")
        ax.scatter(*b, s=140, color=np.array(p3_render.RENDER_RGB[c]) / 255, edgecolor="k", marker="s")
        if np.linalg.norm(a - [1 / 3, 1 / 3]) > 0.02:   # the greys all sit on top of each other
            ax.annotate(c, a, fontsize=8, xytext=(5, 4), textcoords="offset points")
    ax.annotate("9 grey materials + scaffold + fence:\ncolour can't tell them apart", (0.335, 0.375),
                fontsize=9, ha="center")
    for c, v in p3_render.CLUTTER_RENDER_RGB.items():
        b = cr.chroma(np.array([v]))[0]
        ax.scatter(*b, s=110, color=np.array(v) / 255, edgecolor="r", marker="^")
        if np.linalg.norm(b - [1 / 3, 1 / 3]) > 0.02:
            ax.annotate(c, b, fontsize=8, color="r", xytext=(5, -9), textcoords="offset points")
    circ = plt.Circle(cr.chroma(np.array([materials.CATALOGUE_RGB["concrete_cast"]]))[0], cr.CHROMA_SIGMA,
                      fill=False, ls="--", color="k")
    ax.add_patch(circ)
    ax.set_xlabel("red share  r / (r+g+b)")
    ax.set_ylabel("green share  g / (r+g+b)")
    ax.set_aspect("equal")
    ax.set_title("Colour as the registration compares it (brightness removed)\n"
                 "o catalogue, □ renderer, ▲ site clutter; dashed circle = allowed spread")
    fig.tight_layout()
    fig.savefig(os.path.join(d, "colour_tables.png"), dpi=100)
    plt.close(fig)

    # 3. the scan: camera colour vs the old class colour, and a close-up
    s = "06_01_eerste_verdieping_envelope"
    c = np.load(os.path.join(COLOR_CLOUD_DIR, f"{s}.npz"))
    Ti = np.linalg.inv(c["gt_T_bim_to_cloud"])
    xyz = c["xyz"].astype(np.float64) @ Ti[:3, :3].T + Ti[:3, 3]
    cls_col = np.array([np.array(p3_render.CLASS_COLOR.get(k, (200, 200, 200))) / 255 for k in c["gt_class"]])
    for k, v in p3_render.site_clutter.CLUTTER_COLOR.items():
        cls_col[c["gt_class"] == k] = np.array(v) / 255
    cam = c["rgb"] / 255
    E, F0 = np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    ev = view(bim, az=-55, el=28, dist=2.4)
    # close-up: outside of a brick facade at the finished stage
    c15 = np.load(os.path.join(COLOR_CLOUD_DIR, "15_04_dak_envelope.npz"))
    T15 = np.linalg.inv(c15["gt_T_bim_to_cloud"])
    xyz15 = c15["xyz"].astype(np.float64) @ T15[:3, :3].T + T15[:3, 3]
    c0 = bim.center.copy()
    c0[2] = 3.0
    eye = c0 + np.array([np.cos(np.radians(-55)), np.sin(np.radians(-55)), 0.12]) * bim.radius * 2.6
    ims = [render(E, F0, np.zeros((0, 3)), *ev, 800, 580, points=[(xyz, cls_col, 2, False)]),
           render(E, F0, np.zeros((0, 3)), *ev, 800, 580, points=[(xyz, cam, 2, False)]),
           # white design mesh only as a depth buffer, so points behind the facade are hidden
           render(bim.verts, bim.faces, np.ones((len(bim.faces), 3)) / 0.99, eye, c0, 800, 580, fov=22, ss=1,
                  points=[(xyz15, c15["rgb"] / 255, 4, True)])]
    fig = panels(ims, ["Before: colour = IFC class (ground truth, never used)",
                       "Now: simulated camera colour (p3_render.py --color material)",
                       "Stage 15 facade: brick tint per element, sun and shadow, dirt at the foot"],
                 f"Stage {short(s)}, busy site, realistic sensor", ncols=3, scale=0.8)
    fig.savefig(os.path.join(d, "scan_camera_colour.png"), dpi=100)
    plt.close(fig)
    k = np.random.default_rng(SEED).choice(len(xyz), min(300_000, len(xyz)), replace=False)
    write_ply(os.path.join(d, f"stage_{s[:2]}_scan_camera_colour.ply"), xyz[k], cam[k])

    # 4. elements built from another material than designed (renderer only, matcher doesn't know)
    p3_render.LOOK = {}
    all_el = {g: {"z_min": 0.0} for g in bim.gids}
    look = p3_render.element_looks(all_el)
    swapped = np.array([look[g][0] != cats.get(g, materials.DEFAULT) for g in bim.gids])
    rcol = np.array([np.array(p3_render.RENDER_RGB[look[g][0]]) / 255 * np.clip(look[g][1], 0, 2) for g in bim.gids])
    hl = np.where(swapped[:, None], np.array([0.9, 0.1, 0.6]), np.clip(rcol, 0, 1) * 0.4 + 0.55)
    ims = [render(bim.verts, bim.faces, np.clip(rcol, 0, 1)[bim.tri_elem], *view(bim, dist=2.6), 800, 580,
                  tri_elem=bim.tri_elem),
           render(bim.verts, bim.faces, hl[bim.tri_elem], *view(bim, dist=2.6), 800, 580, tri_elem=bim.tri_elem)]
    fig = panels(ims, ["Renderer colour per element (tint varies per element, like brick batches)",
                       f"Pink: {swapped.sum()} elements ({swapped.mean():.0%}) built from a different "
                       "material than the BIM says"],
                 "Built is not always as designed: the matcher is not told about these", ncols=2, scale=0.85)
    fig.savefig(os.path.join(d, "material_substitutions.png"), dpi=100)
    plt.close(fig)

    # 5. colour filter + colour agreement at the right pose and at wrong poses
    rng = np.random.default_rng(SEED)
    s = "03_00_begane_grond_envelope"
    c = np.load(os.path.join(COLOR_CLOUD_DIR, f"{s}.npz"))
    Ti = np.linalg.inv(c["gt_T_bim_to_cloud"])
    k = rng.choice(len(c["xyz"]), min(200_000, len(c["xyz"])), replace=False)
    xyz = c["xyz"][k].astype(np.float64) @ Ti[:3, :3].T + Ti[:3, 3]
    rgb, gcls = c["rgb"][k], c["gt_class"][k]
    cat_chroma = cr.chroma(np.array([materials.CATALOGUE_RGB[q] for q in materials.CATEGORIES]))
    fit = cr.material_fit(rgb, cat_chroma)
    dropped = fit.max(axis=1) < cr.MIN_FIT
    groups = ["IfcWall", "IfcSlab", "IfcBeam", "ground", "tree", "crane", "scaffold", "fence", "pile", "worker"]
    share = [dropped[gcls == g].mean() * 100 if (gcls == g).any() else 0 for g in groups]
    fig, axes = plt.subplots(1, 2, figsize=(16, 6), gridspec_kw={"width_ratios": [1.3, 1]})
    axes[0].imshow(render(E, F0, np.zeros((0, 3)), *view(bim, az=-50, el=30, dist=2.6), 800, 560,
                          points=[(xyz[~dropped], np.full(3, 0.75), 2, False), (xyz[dropped], (0.85, 0.1, 0.1), 3, False)]))
    axes[0].axis("off")
    axes[0].set_title(f"Stage {short(s)}: red = dropped by the colour filter (fits no BIM material)")
    axes[1].barh(groups[::-1], share[::-1], color=["#c33" if g in ("tree", "crane", "worker") else "#888" for g in groups[::-1]])
    axes[1].set_xlabel("% of points dropped (ground truth class used only for this chart)")
    for i, v in enumerate(share[::-1]):
        axes[1].text(v + 0.5, i, f"{v:.1f}%", va="center")
    axes[1].set_xlim(0, 100)
    axes[1].set_title("Building is kept; green trees and yellow crane are partly dropped;\n"
                      "grey scaffolding and brick piles pass on purpose (colour can't tell)")
    fig.tight_layout()
    fig.savefig(os.path.join(d, "colour_filter.png"), dpi=100)
    plt.close(fig)

    def pose(dx=0.0, dy=0.0, dz=0.0, yaw=0.0):
        T = np.eye(4)
        t = np.radians(yaw)
        T[:2, :2] = [[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]]
        cc = xyz.mean(axis=0)
        T[:3, 3] = cc - T[:3, :3] @ cc + [dx, dy, dz]
        return T
    poses = [("right pose", pose()), ("shifted 3 m along x", pose(dx=3.0)),
             ("one storey too high (+2.9 m)", pose(dz=2.9)), ("turned 180 deg", pose(yaw=180))]
    frames, rows = [], []
    for name, T in poses:
        p = xyz @ T[:3, :3].T + T[:3, 3]
        dist, el = _closest(bim, p)
        near = dist < 0.05
        ci = np.array([materials.CATEGORIES.index(q) for q in elem_cat])[el]
        agree = fit[np.arange(len(p)), ci]
        col = np.full((len(p), 3), 0.82)
        col[near] = np.stack([1 - agree[near], 0.25 + 0.6 * agree[near], 0.3 * np.ones(near.sum())], axis=1)
        tight, cfit = near.mean(), agree[near].mean() if near.any() else 0
        rows.append((name, tight, cfit, tight * cfit))
        im = render(bim.verts, bim.faces, np.tile(GHOST, (len(bim.faces), 1)), *view(bim, dist=2.6), 800, 580,
                    points=[(p, col, 2, False)])
        fig = panels([im], [f"{name}: {tight:.0%} of points on a BIM surface, colour fit {cfit:.2f}, "
                            f"score {tight * cfit:.3f}"],
                     "Green = colour fits the material it lands on, red = doesn't,\n"
                     "light grey = not on any BIM surface (dark grey = the design model)", scale=1.0)
        frames.append(fig_to_pil(fig))
    save_gif(frames, os.path.join(d, "colour_agreement_poses.gif"), ms=1600)
    frames[0].save(os.path.join(d, "colour_agreement_right_pose.png"))
    frames[2].save(os.path.join(d, "colour_agreement_wrong_storey.png"))
    pd.DataFrame(rows, columns=["pose", "share_on_bim", "colour_fit", "score"]).to_csv(
        os.path.join(d, "colour_agreement_poses.csv"), index=False, float_format="%.4f")
    print(pd.DataFrame(rows, columns=["pose", "share_on_bim", "colour_fit", "score"]).to_string(index=False))

    # 6. benchmark so far (fills in as the bench runs)
    if os.path.exists(COLOR_BENCH):
        b = pd.read_csv(COLOR_BENCH)
        b["case"] = b["stage"].str[:2] + " " + b["regime"]
        piv = b.pivot_table(index="case", columns="method", values="err_m")
        fig, ax = plt.subplots(figsize=(13, 5.5))
        piv.mul(100).plot.bar(ax=ax, width=0.8, color=["#555", "#2a7", "#d84"][:piv.shape[1]])
        ax.axhline(5, color="r", ls="--", lw=1)
        ax.text(0, 5.2, "success limit 5 cm", color="r")
        ax.set_ylabel("registration error (cm, mean BIM point displacement)")
        ax.set_yscale("log")
        ax.set_ylim(0.5, max(10, piv.max().max() * 100 * 2))
        ax.set_title(f"Registration benchmark on busy + realistic colour clouds ({len(b)} runs so far): "
                     "plan vs colour versions (wrongcat = deliberately wrong material catalogue)")
        flagged = b[~b["self_ok"].astype(bool)]
        for _, r in flagged.iterrows():
            ax.text(list(piv.index).index(r["case"]), r["err_m"] * 100, "flag", ha="center", fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(d, "bench_errors.png"), dpi=100)
        plt.close(fig)
    print("colours done")


# ------------------------------------------------------------------ 08 3D files

def write_ply(path, xyz, rgb, faces=None):
    """Compact binary PLY: float32 xyz + uchar rgb (+ int32 triangles). Viewers
    work out the shading themselves, so no normals."""
    rgb = (np.clip(np.asarray(rgb, dtype=float), 0, 1) * 255).astype(np.uint8)
    head = ["ply", "format binary_little_endian 1.0", f"element vertex {len(xyz)}",
            "property float x", "property float y", "property float z",
            "property uchar red", "property uchar green", "property uchar blue"]
    if faces is not None:
        head += [f"element face {len(faces)}", "property list uchar int vertex_indices"]
    vert = np.zeros(len(xyz), dtype=[("p", "<f4", 3), ("c", "u1", 3)])
    vert["p"], vert["c"] = xyz, rgb
    with open(path, "wb") as fh:
        fh.write(("\n".join(head + ["end_header"]) + "\n").encode())
        fh.write(vert.tobytes())
        if faces is not None:
            fc = np.zeros(len(faces), dtype=[("n", "u1"), ("i", "<i4", 3)])
            fc["n"], fc["i"] = 3, faces
            fh.write(fc.tobytes())


def write_mesh(path, verts, faces, tri_color):
    """Mesh of the given triangles; every element has its own vertices, so a
    per-vertex colour taken from its triangles is exact."""
    used, inv = np.unique(faces, return_inverse=True)
    col = np.zeros((len(used), 3))
    col[inv.reshape(-1)] = np.repeat(np.asarray(tri_color), 3, axis=0)
    write_ply(path, verts[used], col, inv.reshape(-1, 3))


def models():
    d = out_dir("08_3d_models")
    bim = Bim()
    tcol = np.array([TYPE_COLOR.get(tp, (0.6, 0.6, 0.6)) for tp in bim.type])[bim.tri_elem]
    write_mesh(os.path.join(d, "design_model_by_type.ply"), bim.verts, bim.faces, tcol)
    for s in [KEY_STAGES[1], KEY_STAGES[3]]:
        t = truth_of(s)
        v = bim.clamped_verts({g: f for g, f in t.items() if f < 1})
        built = bim.elem_mask(set(t))
        col = np.tile(BUILT, (len(bim.faces), 1))
        col[bim.elem_mask({g for g, f in t.items() if f < 1})] = PARTIAL
        write_mesh(os.path.join(d, f"stage_{s[:2]}_truth.ply"), v, bim.faces[built], col[built])
        for lab, stt in [("old", status_old(s)), ("new", status_new(s))]:
            cls = error_classes(stt, t, bim.gids)[bim.tri_elem]
            sel = cls != ""
            write_mesh(os.path.join(d, f"stage_{s[:2]}_{lab}_pipeline_errors.ply"), v, bim.faces[sel],
                           np.array([ERR_COLOR[c] for c in cls[sel]]))
        c = np.load(os.path.join(CLOUD_DIR, f"{s}.npz"))
        T = np.array(json.load(open(os.path.join(MATCHES_DIR, s, "registration.json")))["T_cloud_to_bim"])
        k = np.random.default_rng(SEED).choice(c["xyz"].shape[0], min(150_000, c["xyz"].shape[0]), replace=False)
        xyz = c["xyz"][k].astype(np.float64)
        if s == KEY_STAGES[1]:
            write_ply(os.path.join(d, f"stage_{s[:2]}_scan_as_received.ply"), xyz, c["rgb"][k] / 255.0)
        write_ply(os.path.join(d, f"stage_{s[:2]}_scan_aligned_to_bim.ply"),
                  xyz @ T[:3, :3].T + T[:3, 3], c["rgb"][k] / 255.0)
    print("models done")


# ------------------------------------------------------------------ 00 flow

def flow():
    d = out_dir("00_overview")
    steps = [
        ("Design model (IFC)\nSchependomlaan, 1355 elements", "#ddd", ""),
        ("P1 parse\nelements, sizes, floors", "#ddd", ""),
        ("P2 make 16 stages\nwhat is built when\n(= ground truth)", "#ddd", ""),
        ("P3 simulate scans\ndrone + indoor scanner,\nnoise, ground, trees,\nGPS-sized offset", "#f6d8a8",
         "changed: half-built walls fixed,\nrealistic sensor option,\nGPS / indoor offsets"),
        ("P4 registration\nput the scan onto\nthe BIM", "#bfe3c0", "new: floor-plan search +\nrobust ICP, flags if unsure"),
        ("P4 matching\nwhich BIM element\neach point belongs to", "#ddd", "changed: margin rule"),
        ("P4 visibility\nwould the cameras have\nseen a missing element?", "#bfe3c0",
         "new: replaces the old\n'seen in final stage' guess"),
        ("P7 score\nbuilt / half / not started /\nunsure -> progress %", "#f6d8a8", "changed: error breakdown,\nregistration check"),
    ]
    fig, ax = plt.subplots(figsize=(18, 5.2))
    ax.axis("off")
    ax.set_xlim(0, len(steps) * 2.2)
    ax.set_ylim(0, 5)
    for i, (txt, c, note) in enumerate(steps):
        x = i * 2.2 + 0.1
        ax.add_patch(FancyBboxPatch((x, 2.2), 1.9, 2.2, boxstyle="round,pad=0.05", fc=c, ec="#555"))
        ax.text(x + 0.95, 3.3, txt, ha="center", va="center", fontsize=9)
        if note:
            ax.text(x + 0.95, 1.5, note, ha="center", va="top", fontsize=8, color="#333", style="italic")
        if i < len(steps) - 1:
            ax.annotate("", xy=(x + 2.18, 3.3), xytext=(x + 1.97, 3.3), arrowprops=dict(arrowstyle="->", lw=1.5))
    ax.legend(handles=[Patch(color="#bfe3c0", label="new step"), Patch(color="#f6d8a8", label="changed step"),
                       Patch(color="#ddd", label="unchanged")], loc="upper left", ncol=3, frameon=False)
    ax.set_title("The pipeline, start to end (every step only talks to the next through files on disk)", fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "pipeline_flow.png"), dpi=110)
    plt.close(fig)
    print("flow done")


SECTIONS = {"flow": flow, "charts": charts, "truth": truth, "scans": scans, "sensor": sensor, "clip": clip,
            "registration": registration_media, "visibility": visibility, "errors": errors, "models": models,
            "site": busy_site, "colors": colors_media}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=",".join(SECTIONS), help="comma list of sections")
    args = ap.parse_args()
    for name in args.only.split(","):
        SECTIONS[name]()


if __name__ == "__main__":
    main()
