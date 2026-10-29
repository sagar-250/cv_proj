"""Cloud-to-BIM registration that also uses colour (method "plan_color").

Builds on src/registration.py ("plan": floor-plan search + robust ICP) and
does not change it. The BIM side knows what each element is made of
(data/bim/bim_materials.csv, read from the IFC by src/materials.py) and the
nominal catalogue colour of each material (materials.CATALOGUE_RGB). The
scan has a camera colour per point (p3_render.py --color material). Colour
is used in three places:

  1. Clutter filter: before anything else, drop points whose colour fits no
     material of the building (green trees, yellow crane, hi-vis workers).
     Grey scaffolding and brick piles pass on purpose -- colour can't tell
     them apart from the building, geometry has to.
  2. Picking the answer: registration.coarse_plan still proposes the
     candidates and registration.refine still refines them. Among the
     finalists whose tight fit (share of points within TIGHT_DIST of the
     BIM) is within AMBIGUITY_RATIO of the best -- the look-alikes geometry
     can't separate -- the winner is the one with the best tight fit x
     colour fit (how well the points' colours fit the material of the BIM
     surface they land on). A pose that puts brick on concrete scores lower.
     Colour never overrules a clear geometric winner: an earlier version
     did, and a wrong material catalogue then gave a silent 3.3 m error.
  3. Final polish (fine="colored_icp"): Open3D's coloured ICP (Park et al.
     2017) at the finest level, on a single "warmth" channel (r - b) / (r +
     g + b). Geometry alone can slide along a long flat wall; a brick /
     concrete edge on it can't.

Colour is compared as chromaticity (r, g share of r + g + b), not raw
values: sun, shadow and camera exposure change brightness a lot but hue
little. Many materials share a grey chromaticity (concrete, screed, sand-
lime block, plasterboard, scaffolding); for those colour says nothing, which
is the honest answer. Very dark points have no reliable hue and are treated
as neutral.

If the cloud has no camera colour (rgb_source != "camera"), callers should
use registration.register; the class colour (rgb_source "class") is ground
truth and must never reach this code.
"""
import os
import sys

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(__file__))
import materials  # noqa: E402
import registration as reg  # noqa: E402

CHROMA_SIGMA = 0.04      # chromaticity spread allowed between catalogue and camera colour
MIN_FIT = 0.05           # clutter filter: drop points whose best material fit is below this
DARK_SUM = 90            # r + g + b below this -> hue unreliable, point counts as neutral (fit 1)
WARMTH_GAIN = 2.5        # warmth = 0.5 + WARMTH_GAIN * (r - b) / (r + g + b), clipped to [0, 1]
COLOR_ICP_DIST = 0.08    # m, correspondence distance of the coloured ICP polish
COLOR_ICP_LAMBDA = 0.9   # weight of geometry vs colour in coloured ICP (Open3D default 0.968)


def chroma(rgb):
    rgb = np.asarray(rgb, dtype=np.float64)
    s = rgb.sum(axis=1, keepdims=True) + 1e-6
    return rgb[:, :2] / s


def warmth(rgb):
    rgb = np.asarray(rgb, dtype=np.float64)
    return np.clip(0.5 + WARMTH_GAIN * (rgb[:, 0] - rgb[:, 2]) / (rgb.sum(axis=1) + 1e-6), 0, 1)


def material_fit(rgb, cat_chroma):
    """(N, K) how well each point's colour fits each catalogue material, in [0, 1]."""
    d2 = ((chroma(rgb)[:, None, :] - cat_chroma[None, :, :]) ** 2).sum(axis=2)
    fit = np.exp(-d2 / (2 * CHROMA_SIGMA ** 2))
    fit[np.asarray(rgb, dtype=np.float64).sum(axis=1) < DARK_SUM] = 1.0
    return fit


class ColorTarget:
    """registration.BimTarget plus the material of every finest-level BIM point."""

    def __init__(self, bim_xyz, bim_gid, cat_of_gid, catalogue=None, seed=0, geo=None):
        """geo: an existing registration.BimTarget built from the same bim_xyz and seed, to share."""
        catalogue = catalogue or materials.CATALOGUE_RGB
        self.geo = geo or reg.BimTarget(bim_xyz, seed=seed)
        cats = materials.CATEGORIES
        self.cat_chroma = chroma(np.array([catalogue[c] for c in cats]))
        cat_rgb = np.array([catalogue[c] for c in cats], dtype=np.float64)

        # same subsample BimTarget took, so every point has its element's material
        rng = np.random.default_rng(seed)
        n = bim_xyz.shape[0]
        sub = rng.choice(n, size=min(n, 3_000_000), replace=False)
        cat_index = {c: i for i, c in enumerate(cats)}
        uniq, inv = np.unique(bim_gid[sub], return_inverse=True)
        sub_cat = np.array([cat_index[cat_of_gid.get(g, materials.DEFAULT)] for g in uniq])[inv]
        _, idx = cKDTree(bim_xyz[sub]).query(np.asarray(self.geo.levels[-1].points), k=1, workers=-1)
        self.fine_cat = sub_cat[idx]
        self.fine = o3d.geometry.PointCloud(self.geo.levels[-1])
        w = warmth(cat_rgb[self.fine_cat])
        self.fine.colors = o3d.utility.Vector3dVector(np.repeat(w[:, None], 3, axis=1))
        self.fine_tree = cKDTree(np.asarray(self.fine.points))


def color_score(src_xyz, src_fit, target, T):
    """(share of points within TIGHT_DIST, mean colour fit of those points to the material they land on).
    src_xyz should be the cloud thinned like registration.register's tight check (finest ICP voxel),
    so the first number is the same tight fit plan ranks by."""
    p = src_xyz @ T[:3, :3].T + T[:3, 3]
    d, i = target.fine_tree.query(p, k=1, distance_upper_bound=reg.TIGHT_DIST, workers=-1)
    near = np.isfinite(d)
    if not near.any():
        return 0.0, 0.0
    agree = src_fit[np.nonzero(near)[0], target.fine_cat[i[near]]]
    return float(near.mean()), float(agree.mean())


def colored_polish(src_xyz, src_rgb, target, T):
    s = reg.scale_of(T)
    S = np.diag([s, s, s, 1.0])
    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(src_xyz * s)
    w = warmth(src_rgb)
    src.colors = o3d.utility.Vector3dVector(np.repeat(w[:, None], 3, axis=1))
    src = src.voxel_down_sample(reg.ICP_SCHEDULE[-1][1])
    src.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    est = o3d.pipelines.registration.TransformationEstimationForColoredICP(
        lambda_geometric=COLOR_ICP_LAMBDA, kernel=o3d.pipelines.registration.TukeyLoss(k=reg.TUKEY_K * 0.3))
    res = o3d.pipelines.registration.registration_colored_icp(
        src, target.fine, COLOR_ICP_DIST, T @ np.linalg.inv(S), est,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
    return res.transformation @ S


def register(xyz, rgb, target, with_scale=False, prior="none", fine="colored_icp", verbose=False):
    """Like registration.register(method="plan"), plus colour. target is a ColorTarget.
    Returns (T, info); info has the same keys plus color_fit and n_color_dropped."""
    fit_all = material_fit(rgb, target.cat_chroma)
    keep = fit_all.max(axis=1) >= MIN_FIT
    xyz, rgb = xyz[keep], rgb[keep]
    src_pcd = reg._pcd(xyz)

    finalists, c_src = reg.coarse_plan(src_pcd, target.geo, prior)
    if not finalists:
        finalists = [(0.0, np.eye(4))]
    # score on the same voxel-thinned cloud plan uses for its tight check (colours averaged per voxel)
    col_pcd = reg._pcd(xyz)
    col_pcd.colors = o3d.utility.Vector3dVector(rgb / 255.0)
    col_pcd = col_pcd.voxel_down_sample(reg.ICP_SCHEDULE[-1][1])
    p_xyz = np.asarray(col_pcd.points)
    p_fit = material_fit(np.asarray(col_pcd.colors) * 255.0, target.cat_chroma)
    refined = []
    for coarse_fit, T0 in finalists[:reg.N_FINAL]:
        T_r, fit_r, rmse_r = reg.refine(src_pcd, target.geo, T0, with_scale)
        if fine == "colored_icp":
            T_r = colored_polish(xyz, rgb, target, T_r)
        tight, agree = color_score(p_xyz, p_fit, target, T_r)
        refined.append((tight * agree, tight, agree, fit_r, rmse_r, T_r, coarse_fit))
    # colour only decides between candidates geometry finds (almost) equally good: within
    # AMBIGUITY_RATIO of the best tight fit. Measured without this gate: a wrong material
    # catalogue let colour overrule clear geometry -> 3.3 m off, not flagged (stage 00, GNSS).
    best_tight = max(r[1] for r in refined)

    def is_close(r):
        return r[1] >= reg.AMBIGUITY_RATIO * best_tight

    refined.sort(key=lambda r: -r[0] if is_close(r) else 1 - r[1])   # close ones first, by colour score
    score, tight, agree, fitness, rmse, T, coarse_fit = refined[0]
    runner = next((r for r in refined[1:] if reg._distinct(r[5], T, c_src)), None)
    if runner is None:
        ambiguity = 0.0
    elif is_close(runner):
        ambiguity = runner[0] / max(score, 1e-9)        # geometric look-alikes: colour decides
    else:
        ambiguity = runner[1] / max(best_tight, 1e-9)   # geometry already separates them
    if fine == "colored_icp":   # fitness / rmse of the polished pose, same measure as refine()
        ev = o3d.pipelines.registration.evaluate_registration(
            src_pcd.voxel_down_sample(reg.ICP_SCHEDULE[-1][1]), target.geo.levels[-1], reg.ICP_SCHEDULE[-1][0], T)
        fitness, rmse = ev.fitness, ev.inlier_rmse

    tilt = reg.tilt_deg(T)
    c = xyz.mean(axis=0)
    shift = float(np.linalg.norm(T[:3, :3] @ c + T[:3, 3] - c))
    checks = {
        "fitness": fitness > 0.2,
        "gravity": tilt <= reg.MAX_TILT_DEG,
        "gnss": prior != "gnss" or (shift <= reg.GNSS_MAX_SHIFT + 2 and abs(reg.yaw_deg(T)) <= reg.GNSS_MAX_YAW + 2),
        "unambiguous": ambiguity < reg.AMBIGUITY_RATIO,
    }
    # what colour did: the runner-up's colour fit, and the ambiguity plan would have seen
    info = {"coarse_score": coarse_fit, "tight_fit": tight, "color_fit": agree,
            "runner_color_fit": runner[2] if runner else np.nan,
            "tight_ambiguity": (runner[1] / max(best_tight, 1e-9)) if runner else 0.0,
            "ambiguity": ambiguity, "fitness": fitness, "rmse": rmse, "tilt_deg": tilt,
            "scale": reg.scale_of(T), "shift_m": shift, "n_color_dropped": int((~keep).sum()),
            "failed_checks": ",".join(k for k, v in checks.items() if not v), "ok": all(checks.values())}
    if verbose:
        print("  registration: " + ", ".join(f"{k}={v:.4f}" if isinstance(v, float) else f"{k}={v}"
                                             for k, v in info.items()))
    return T, info
