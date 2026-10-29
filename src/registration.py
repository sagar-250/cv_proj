"""Robust cloud-to-BIM registration (ROADMAP R3), replacing the plain ICP in
p4_match_bim.py.

Why plain ICP is not enough: it only converges if the cloud already starts
within roughly its correspondence distance (0.3 m) of the BIM. A drone
reconstruction georeferenced by on-board GNSS is off by metres and several
degrees of heading; an indoor SLAM scan sits in its own arbitrary frame.

Method, coarse to fine:

  Coarse, method="plan" (default) -- uses what a building scan always has:
    gravity is known (drone/SLAM IMU, ~1 deg), so only heading + x, y, z are
    unknown, and buildings are made of vertical walls and horizontal slabs.
      0. Level the cloud: rotate it so the mean normal of its near-horizontal
         surfaces points straight up. Even ~1 deg of tilt moves heights by
         ~0.5 m across a 30 m building, which smeared the height histogram
         and picked the wrong storey height (measured: true height shift was
         not even among the candidates before levelling).
      a. Split points by surface normal: near-vertical surfaces (walls) and
         near-horizontal ones (slabs, ground). No clutter rule needed --
         ground ends up in the horizontal set, most tree canopy in neither.
      b. Height: correlate the histogram of horizontal-surface heights with
         the BIM's. A one-storey cloud fits several storeys, so the top few
         height candidates are kept, not just the best.
      c. Heading + x, y: for each height candidate, rasterise the BIM walls in
         that height band into a top-down floor-plan image, and the cloud's
         wall points into the same kind of image; sweep heading and
         FFT-correlate each rotated image with the BIM's. The peak gives x, y.
      d. Height sweep: the histogram's height candidates only choose which
         wall band to rasterise; the height of each (heading, x, y)
         candidate comes from a dense 3D sweep (DZ_STEP), scoring the
         fraction of cloud points near a BIM surface. The top N_DZ_PEAKS
         peaks are all kept and decided in 3D below.
         On early, ground-heavy clouds the histogram alone missed the true
         height entirely.
      e. Verify in 3D: the top few clearly different candidates each get a
         quick coarse ICP and are scored by the fraction of cloud points
         lying within VERIFY_DIST of a BIM surface. The 2D image score alone
         picks the wrong height at early stages (measured: right heading,
         ~1 m off in height), because a foundation-stage cloud is mostly
         ground, which is horizontal but not in the BIM.
      f. The best N_FINAL distinct candidates each get the full refinement
         below; the one with most points within TIGHT_DIST of the BIM wins.
      g. Ambiguity: compare that tight fit with the best clearly different
         one's. A repeated building (a row of identical units) can
         produce near-ties; then the result is flagged ok=False, not guessed.
    With prior="gnss" the search is limited to plausible offsets (heading
    +/-20 deg, a few metres), and among near-ties the one nearest the prior
    wins -- that is what GNSS is for.

  Coarse, method="fpfh" -- FPFH features + RANSAC. Kept for comparison:
    measured to pick wrong matches on partly built, repetitive clouds.

  Fine (both): multi-scale point-to-plane ICP with a Tukey robust loss, so
    clutter, not-yet-built neighbours and as-built deviations stop pulling
    the fit. with_scale=True adds a final point-to-point ICP with scaling
    (photogrammetry scale is only as good as its GNSS, ~1%).

  Checks: gravity (tilt <= MAX_TILT_DEG), fitness, GNSS plausibility, and
  the coarse ambiguity test all feed info["ok"].

Everything is seeded; same input -> same output.
"""
import numpy as np
import open3d as o3d
from scipy.ndimage import binary_dilation
from scipy.signal import correlate

# --- fpfh coarse method ---
VOXEL = 0.3
FPFH_RADIUS = VOXEL * 5
NORMAL_RADIUS = VOXEL * 2
RANSAC_DIST = VOXEL * 1.5
RANSAC_ITERS = 60_000        # measured: 3x200k took ~90 s/stage
RANSAC_CONFIDENCE = 0.999
RANSAC_TRIALS = 2
GNSS_CROP_MARGIN = 6.0

# --- plan coarse method ---
PLAN_VOXEL = 0.1             # m, downsample before normal estimation
PLAN_NORMAL_RADIUS = 0.3
VERTICAL_MAX_NZ = 0.3        # |n_z| below this -> wall-like surface
HORIZONTAL_MIN_NZ = 0.9      # |n_z| above this -> slab-like surface
Z_BIN = 0.05                 # m, height histogram bin
N_Z_CANDIDATES = 6
Z_CANDIDATE_SEP = 0.5        # m between kept height candidates
PLAN_RES = 0.2               # m per floor-plan pixel
PLAN_BAND_PAD = 0.3          # m added above/below the cloud's height range when cutting BIM walls
YAW_STEP = 1.0               # deg
N_VERIFY = 8                 # distinct 2D candidates checked in 3D
N_DZ_PEAKS = 3               # height-sweep peaks kept per candidate (not just the best)
DZ_PEAK_SEP = 0.25           # m between kept peaks
N_FINAL = 3                  # best distinct candidates that get the full refinement
DZ_STEP = 0.1                # m, height sweep step. A 0.3 m coarse-then-fine sweep was ~3x
                             # faster but landed 0.4-0.6 m off on the foundation stage (thin
                             # beams) without flagging it, so the plain 0.1 m sweep stays.
DZ_SWEEP_DIST = 0.15         # m, inlier distance while sweeping height
VERIFY_DIST = 0.1            # m, inlier distance for the 3D check
AMBIGUITY_RATIO = 0.9        # second-best / best final tight fit above this -> ambiguous
TIGHT_DIST = 0.05            # m: finalists are ranked by the share of points this close to the BIM.
                             # The 0.15 m ICP fitness barely separates a pose 0.6 m off from
                             # the right one (0.67 vs 0.73); a tight tolerance does.
DISTINCT_SHIFT = 2.0         # m: candidates closer than this (and DISTINCT_YAW) count as the same answer
DISTINCT_YAW = 5.0           # deg

# --- priors / checks ---
MAX_TILT_DEG = 2.0
GNSS_MAX_SHIFT = 10.0        # m (3 sigma of a 2 m horizontal GNSS error + margin)
GNSS_MAX_DZ = 4.0            # m
GNSS_MAX_YAW = 20.0          # deg

# --- fine ---
ICP_SCHEDULE = [(1.0, 0.20), (0.4, 0.10), (0.15, 0.05)]  # (max corr dist, voxel) per level
TUKEY_K = 0.1


def _pcd(xyz):
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    return p


def _prep(pcd, voxel):
    down = pcd.voxel_down_sample(voxel)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=NORMAL_RADIUS, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=FPFH_RADIUS, max_nn=100))
    return down, fpfh


def _split_by_normal(pcd, return_level=False):
    """(vertical-surface xyz, horizontal-surface z) after PLAN_VOXEL downsampling.
    With return_level, first levels the cloud (see module docstring, step 0)
    and also returns the 4x4 levelling transform L (already applied)."""
    down = pcd.voxel_down_sample(PLAN_VOXEL)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=PLAN_NORMAL_RADIUS, max_nn=30))
    xyz = np.asarray(down.points)
    nrm = np.asarray(down.normals)
    L = np.eye(4)
    if return_level:
        h = nrm[np.abs(nrm[:, 2]) > HORIZONTAL_MIN_NZ]
        if h.shape[0] >= 20:
            up = (h * np.sign(h[:, 2:3])).mean(axis=0)
            up /= np.linalg.norm(up)
            axis = np.cross(up, [0, 0, 1.0])
            ang = np.degrees(np.arcsin(np.clip(np.linalg.norm(axis), 0, 1)))
            if ang > 1e-4:
                R = _rotation(axis, ang)
                c = xyz.mean(axis=0)
                L[:3, :3] = R
                L[:3, 3] = c - R @ c
                xyz = xyz @ R.T + L[:3, 3]
                nrm = nrm @ R.T
    nz = np.abs(nrm[:, 2])
    out = (xyz[nz < VERTICAL_MAX_NZ], xyz[nz > HORIZONTAL_MIN_NZ, 2])
    return out + (L,) if return_level else out


def _rotation(axis, deg):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    t = np.radians(deg)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * (K @ K)


def tilt_deg(T):
    """Angle between the transformed z-axis and the true z-axis."""
    z = T[:3, :3] @ np.array([0, 0, 1.0])
    z = z / np.linalg.norm(z)
    return float(np.degrees(np.arccos(np.clip(z[2], -1, 1))))


def scale_of(T):
    return float(np.cbrt(abs(np.linalg.det(T[:3, :3]))))


def yaw_deg(T):
    R = T[:3, :3] / scale_of(T)
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def rot_z(deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class BimTarget:
    """Precomputed BIM-side data, built once and reused for every stage."""

    def __init__(self, bim_xyz, seed=0, crop_box=None, with_fpfh=False):
        rng = np.random.default_rng(seed)
        if crop_box is not None:
            lo, hi = crop_box
            bim_xyz = bim_xyz[np.all((bim_xyz >= lo) & (bim_xyz <= hi), axis=1)]
        n = bim_xyz.shape[0]
        sub = bim_xyz[rng.choice(n, size=min(n, 3_000_000), replace=False)]
        self.full = _pcd(sub)
        if with_fpfh:
            self.down, self.fpfh = _prep(self.full, VOXEL)
        self.vert, self.horz_z = _split_by_normal(self.full)
        self.levels = []
        for _dist, vox in ICP_SCHEDULE:
            lvl = self.full.voxel_down_sample(vox)
            lvl.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(vox * 3, 0.15), max_nn=30))
            self.levels.append(lvl)


# ---------------------------------------------------------------- plan coarse

def _height_candidates(src_z, bim_z, dz_window):
    """Top height shifts dz (cloud z + dz = BIM z) by histogram correlation."""
    lo = np.floor(min(src_z.min(), bim_z.min()) / Z_BIN) * Z_BIN - 1
    hi = np.ceil(max(src_z.max(), bim_z.max()) / Z_BIN) * Z_BIN + 1
    edges = np.arange(lo, hi + Z_BIN, Z_BIN)
    hs = np.log1p(np.histogram(src_z, edges)[0].astype(float))
    hb = np.log1p(np.histogram(bim_z, edges)[0].astype(float))
    corr = correlate(hb, hs, mode="full", method="fft")
    lags = (np.arange(corr.size) - (hs.size - 1)) * Z_BIN
    allowed = (lags >= dz_window[0]) & (lags <= dz_window[1])
    corr = np.where(allowed, corr, -np.inf)
    picked = []
    for i in np.argsort(corr)[::-1]:
        if not np.isfinite(corr[i]) or len(picked) == N_Z_CANDIDATES:
            break
        if all(abs(lags[i] - p) >= Z_CANDIDATE_SEP for p in picked):
            picked.append(float(lags[i]))
    return picked


def _raster(xy, origin, shape):
    idx = np.floor((xy - origin) / PLAN_RES).astype(int)
    ok = (idx[:, 0] >= 0) & (idx[:, 1] >= 0) & (idx[:, 0] < shape[0]) & (idx[:, 1] < shape[1])
    img = np.zeros(shape, dtype=np.float32)
    img[idx[ok, 0], idx[ok, 1]] = 1.0
    return img


def plan_candidates(src_vert, src_horz_z, target, prior="none"):
    """All (score, T) coarse candidates, best first. T maps cloud -> BIM."""
    gnss = prior == "gnss"
    dz_window = (-GNSS_MAX_DZ, GNSS_MAX_DZ) if gnss else (-1e9, 1e9)
    yaws = np.arange(-GNSS_MAX_YAW, GNSS_MAX_YAW + 1e-9, YAW_STEP) if gnss else np.arange(0, 360, YAW_STEP)
    if src_vert.shape[0] < 50 or src_horz_z.size < 20:
        return []
    c = src_vert.mean(axis=0)
    out = []
    for dz in _height_candidates(src_horz_z, target.horz_z, dz_window):
        zlo = src_vert[:, 2].min() + dz - PLAN_BAND_PAD
        zhi = src_vert[:, 2].max() + dz + PLAN_BAND_PAD
        bv = target.vert[(target.vert[:, 2] >= zlo) & (target.vert[:, 2] <= zhi)]
        if bv.shape[0] < 50:
            continue
        b_origin = bv[:, :2].min(axis=0)
        b_shape = tuple(np.ceil((bv[:, :2].max(axis=0) - b_origin) / PLAN_RES).astype(int) + 1)
        bimg = binary_dilation(_raster(bv[:, :2], b_origin, b_shape)).astype(np.float32)
        for yaw in yaws:
            p = (src_vert[:, :2] - c[:2]) @ rot_z(yaw)[:2, :2].T + c[:2]
            s_origin = p.min(axis=0)
            s_shape = tuple(np.ceil((p.max(axis=0) - s_origin) / PLAN_RES).astype(int) + 1)
            simg = _raster(p, s_origin, s_shape)
            n_src = simg.sum()
            corr = correlate(bimg, simg, mode="full", method="fft")
            lag0 = np.array(simg.shape) - 1
            # shift (m) for every lag: bim cell = src cell + lag -> shift = lag*res + b_origin - s_origin
            if gnss:
                ii, jj = np.meshgrid(np.arange(corr.shape[0]), np.arange(corr.shape[1]), indexing="ij")
                sx = (ii - lag0[0]) * PLAN_RES + b_origin[0] - s_origin[0]
                sy = (jj - lag0[1]) * PLAN_RES + b_origin[1] - s_origin[1]
                corr = np.where(np.hypot(sx, sy) <= GNSS_MAX_SHIFT, corr, -1)
            k = np.unravel_index(np.argmax(corr), corr.shape)
            shift = (np.array(k) - lag0) * PLAN_RES + b_origin - s_origin
            T = np.eye(4)
            T[:3, :3] = rot_z(yaw)
            T[:3, 3] = np.array([c[0], c[1], 0]) - rot_z(yaw) @ np.array([c[0], c[1], 0])
            T[:3, 3] += np.array([shift[0], shift[1], dz])
            out.append((float(corr[k] / max(n_src, 1)), T))
    out.sort(key=lambda st: -st[0])
    return out


def _distinct(T1, T2, c):
    d_yaw = abs((yaw_deg(T1) - yaw_deg(T2) + 180) % 360 - 180)
    d_pos = np.linalg.norm((T1[:3, :3] @ c + T1[:3, 3]) - (T2[:3, :3] @ c + T2[:3, 3]))
    return d_yaw > DISTINCT_YAW or d_pos > DISTINCT_SHIFT


def coarse_plan(src_pcd, target, prior="none"):
    src_vert, src_horz_z, L = _split_by_normal(src_pcd, return_level=True)
    cands = plan_candidates(src_vert, src_horz_z, target, prior)
    if not cands:
        return [], src_vert.mean(axis=0) if src_vert.size else np.zeros(3)
    cands = [(s, T @ L) for s, T in cands]  # candidates were found on the levelled cloud
    c = src_vert.mean(axis=0)
    distinct = []
    for _s, T in cands:
        if all(_distinct(T, T2, c) for T2 in distinct):
            distinct.append(T)
        if len(distinct) == N_VERIFY:
            break

    # height sweep per candidate, then verify each in 3D after a quick coarse ICP
    dist0, vox0 = ICP_SCHEDULE[0]
    src0 = src_pcd.voxel_down_sample(vox0)
    src_sw = src_pcd.voxel_down_sample(0.3)
    tgt_z = np.asarray(target.levels[0].points)[:, 2]
    loss = o3d.pipelines.registration.TukeyLoss(k=TUKEY_K * dist0)
    swept = []
    for T in distinct:
        z_now = (np.asarray(src_sw.points) @ T[:3, :3].T + T[:3, 3])[:, 2]
        lo, hi = tgt_z.min() - z_now.max(), tgt_z.max() - z_now.min()
        if prior == "gnss":  # stay within the GNSS height error of the cloud's own frame
            dz0 = T[2, 3]
            lo, hi = max(lo, -GNSS_MAX_DZ - dz0), min(hi, GNSS_MAX_DZ - dz0)
        dzs = np.arange(lo, hi + 1e-9, DZ_STEP)
        fits = []
        for dz in dzs:
            Tz = T.copy()
            Tz[2, 3] += dz
            fits.append(o3d.pipelines.registration.evaluate_registration(
                src_sw, target.levels[0], DZ_SWEEP_DIST, Tz).fitness)
        # keep several peaks: a thick horizontal element's top face matches its
        # own bottom face one element-height away (measured: foundation beams
        # put the best sweep peak 0.59 m too high, beyond ICP's ~0.3 m reach)
        kept = []
        for i in np.argsort(fits)[::-1]:
            if all(abs(dzs[i] - k) >= DZ_PEAK_SEP for k in kept):
                kept.append(dzs[i])
            if len(kept) == N_DZ_PEAKS:
                break
        for dz in kept:
            Tz = T.copy()
            Tz[2, 3] += dz
            swept.append(Tz)
    verified = []
    for T in swept:
        res = o3d.pipelines.registration.registration_icp(
            src0, target.levels[0], dist0, T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))
        fit = o3d.pipelines.registration.evaluate_registration(
            src0, target.levels[1], VERIFY_DIST, res.transformation).fitness
        verified.append((fit, res.transformation))
    verified.sort(key=lambda ft: -ft[0])
    finalists = []
    for fit, T in verified:
        if all(_distinct(T, T2, c) for _f, T2 in finalists):
            finalists.append((fit, T))
    return finalists, c


# ---------------------------------------------------------------- fpfh coarse

def coarse_fpfh(src_pcd, target, with_scale=False, seed=0, use_gravity=True):
    src_down, src_fpfh = _prep(src_pcd, VOXEL)
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scale)
    checkers = [
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(RANSAC_DIST),
    ]
    best_T, best_score = np.eye(4), -1.0
    for trial in range(RANSAC_TRIALS):
        o3d.utility.random.seed(seed * 1000 + trial)
        res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down, target.down, src_fpfh, target.fpfh, True, RANSAC_DIST, est, 3, checkers,
            o3d.pipelines.registration.RANSACConvergenceCriteria(RANSAC_ITERS, RANSAC_CONFIDENCE))
        T = res.transformation
        if use_gravity and tilt_deg(T) > MAX_TILT_DEG * 3:
            continue
        score = o3d.pipelines.registration.evaluate_registration(
            src_down, target.down, RANSAC_DIST, T).fitness
        if score > best_score:
            best_T, best_score = T, score
    return best_T, {"coarse_score": best_score, "ambiguity": 0.0}


# ---------------------------------------------------------------- fine

def refine(src_pcd, target, T_init, with_scale=False):
    T = T_init.copy()
    s = scale_of(T)
    for i, ((dist, vox), tgt) in enumerate(zip(ICP_SCHEDULE, target.levels)):
        if with_scale and i == len(ICP_SCHEDULE) - 1:
            res = o3d.pipelines.registration.registration_icp(
                src_pcd.voxel_down_sample(vox), tgt, dist, T,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
            T = res.transformation
            break
        src = src_pcd.voxel_down_sample(vox)
        if abs(s - 1) > 1e-6:
            # point-to-plane ICP is rigid only; fold the coarse scale into the
            # source points, refine rigidly, then put the scale back
            S = np.diag([s, s, s, 1.0])
            src.transform(S)
            T_rigid = T @ np.linalg.inv(S)
        else:
            S, T_rigid = np.eye(4), T
        loss = o3d.pipelines.registration.TukeyLoss(k=TUKEY_K * max(dist, 0.3))
        res = o3d.pipelines.registration.registration_icp(
            src, tgt, dist, T_rigid,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
        T = res.transformation @ S
    ev = o3d.pipelines.registration.evaluate_registration(
        src_pcd.voxel_down_sample(ICP_SCHEDULE[-1][1]), target.levels[-1], ICP_SCHEDULE[-1][0], T)
    return T, ev.fitness, ev.inlier_rmse


def register(xyz, target, method="plan", with_scale=False, seed=0, use_gravity=True, prior="none",
             bim_xyz=None, verbose=False):
    """Estimate T (cloud frame -> BIM frame) such that T @ xyz lands on the BIM.

    method="fpfh" with prior="gnss" needs bim_xyz (the full BIM sample cloud)
    to build a cropped target around the cloud.

    Returns (T, info). info["ok"] is False when a check fails (tilt, fitness,
    GNSS plausibility, or an ambiguous coarse match) -- callers should treat
    that stage as unregistered rather than trust it."""
    src_pcd = _pcd(xyz)
    if method == "plan":
        # fully refine the best few distinct coarse candidates; the final fit
        # decides (measured: a candidate that is best after the quick ICP can
        # still settle 0.6 m off, while the true pose refines to a clearly
        # better fit). Ambiguity = second-best distinct final fit / best.
        finalists, c_src = coarse_plan(src_pcd, target, prior)
        if not finalists:
            finalists = [(0.0, np.eye(4))]
        src_tight = src_pcd.voxel_down_sample(ICP_SCHEDULE[-1][1])
        refined = []
        for coarse_fit, T0 in finalists[:N_FINAL]:
            T_r, fit_r, rmse_r = refine(src_pcd, target, T0, with_scale)
            tight = o3d.pipelines.registration.evaluate_registration(
                src_tight, target.levels[-1], TIGHT_DIST, T_r).fitness
            refined.append((tight, fit_r, rmse_r, T_r, coarse_fit))
        refined.sort(key=lambda r: -r[0])
        tight, fitness, rmse, T, coarse_fit = refined[0]
        second = next((r[0] for r in refined[1:] if _distinct(r[3], T, c_src)), 0.0)
        cinfo = {"coarse_score": coarse_fit, "tight_fit": tight, "ambiguity": second / max(tight, 1e-9)}
    else:
        if prior == "gnss":
            lo = xyz.min(axis=0) - GNSS_CROP_MARGIN
            hi = xyz.max(axis=0) + GNSS_CROP_MARGIN
            target = BimTarget(bim_xyz, seed=seed, crop_box=(lo, hi), with_fpfh=True)
        T0, cinfo = coarse_fpfh(src_pcd, target, with_scale, seed, use_gravity)
        T, fitness, rmse = refine(src_pcd, target, T0, with_scale)

    tilt = tilt_deg(T)
    c = xyz.mean(axis=0)
    shift = float(np.linalg.norm(T[:3, :3] @ c + T[:3, 3] - c))
    checks = {
        "fitness": fitness > 0.2,
        "gravity": not use_gravity or tilt <= MAX_TILT_DEG,
        "gnss": prior != "gnss" or (shift <= GNSS_MAX_SHIFT + 2 and abs(yaw_deg(T)) <= GNSS_MAX_YAW + 2),
        "unambiguous": cinfo["ambiguity"] < AMBIGUITY_RATIO,
    }
    info = {**cinfo, "fitness": fitness, "rmse": rmse, "tilt_deg": tilt, "scale": scale_of(T),
            "shift_m": shift, "failed_checks": ",".join(k for k, v in checks.items() if not v),
            "ok": all(checks.values())}
    if verbose:
        print("  registration: " + ", ".join(f"{k}={v:.4f}" if isinstance(v, float) else f"{k}={v}"
                                             for k, v in info.items()))
    return T, info
