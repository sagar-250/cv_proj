"""R2 experiment -- build the point cloud with COLMAP (photogrammetry) instead of raycasting.

Until now a stage's cloud came straight from raycasting the BIM. A real drone survey
gives photos, and a photogrammetry program (COLMAP) turns them into a cloud. This
script tries that for ONE stage and sees how the result differs and whether it fits
into our pipeline. Steps (run in order, or --step all):

  render  Draw simulated drone photos of the stage from an exterior orbit (drone
          only, no indoor scans): material colours, brick courses, a fixed sun with
          shadows, a textured ground, per-photo exposure, a little blur. COLMAP
          matches points between photos, so the surfaces get extra fine detail that
          stays fixed on the wall (the same from every camera, like real brick or
          concrete grain). Without it COLMAP finds almost nothing to match.
  colmap  Run COLMAP: find features, match photos, solve camera positions + a sparse
          model, then (needs an NVIDIA GPU) dense depth maps and a fused cloud.
  cloud   Turn the result into a stage cloud for the pipeline. COLMAP's cloud has its
          own arbitrary position and scale, so it is moved into the survey frame the
          way a real survey does it: fit the solved camera positions to the drone's
          noisy GNSS positions (a scaled rigid fit). Also measures the cloud against
          the true surfaces (eval only) and writes data/pointclouds_sfm/{stage}.npz.
          Then: RUN_TAG=sfm p4_match_bim.py / p4_visibility.py / p7_score.py.

Ground truth (the true camera poses, the true surfaces) is used only to make the GNSS
tags and to measure the cloud, never by the reconstruction or the pipeline.

Two photo sets: --views exterior (default: the drone orbit, GNSS-tagged) and --views interior
(photos taken while walking the ground floor: a grid of standing positions, 4 headings each,
90 degree field of view). Indoors there is no GNSS, so the cloud gets the metric scale and
levelling a handheld/IMU rig would give (taken from the true camera positions) and an arbitrary
start frame (`survey_offset("local")`), like an indoor scanner's; registration then runs with
--prior none. Work folders and outputs carry an `_int` suffix (data/sfm/{stage}_int,
data/pointclouds_sfm_int, results/sfm/{stage}_int_metrics.json).

Run: .venv/bin/python src/sfm_experiment.py --stage 15_04_dak_envelope [--step all] [--views interior]
Output: data/sfm/{stage}/ (images, COLMAP files), data/pointclouds_sfm/{stage}.npz,
results/sfm/{stage}_metrics.json
"""
import argparse
import json
import os
import pickle
import sys
import time

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(__file__))
import p3_render as p3  # noqa: E402
import p2_p3_simulate_stages as sim  # noqa: E402
import open3d as o3d  # noqa: E402

STATES_DIR = "data/states"
SFM_DIR = "data/sfm"
OUT_CLOUD_DIR = "data/pointclouds_sfm"
METRICS_DIR = "results/sfm"
SEED = 0

IMG_W, IMG_H = 1280, 960
FOV = 70.0                          # horizontal, degrees
SUN = np.array([np.cos(np.radians(50)) * np.cos(np.radians(40)),
                np.cos(np.radians(50)) * np.sin(np.radians(40)), np.sin(np.radians(50))])
AMBIENT, SUN_GAIN = 0.45, 0.65
FINE_CELLS = (0.06, 0.15, 0.4)      # m; fixed-on-the-surface detail (cells smaller than ~4 cm would alias)
FINE_STRENGTH = (0.16, 0.14, 0.10)
GPS_SIGMA = 1.5                     # m, per-photo GNSS tag noise (horizontal; vertical x1.5)
GROUND_RGB = (118, 122, 98)
DENSE_MAX_SIDE = 1024               # depth-map size limit (GPU memory)
INTERIOR = False                    # set from --views in main()
WALK = False                        # --views interior-walk: a continuous walk through the doorways
SUFFIX = ""                         # "_int" (grid) or "_walk" for interior photo sets
WALK_SPACING, WALK_MAX_PHOTOS = 1.6, 150   # m between photo positions (grown until the set fits), photo budget
WALK_STEP, WALK_CLEARANCE = 0.25, 0.3  # path grid (m) and the free space a handheld camera needs on each side (m)
PAIR_RADIUS, PAIR_SEQ = 6.0, 4      # which photo pairs to match: within this distance, plus time neighbours
ODOMETRY_SIGMA = 0.3                # m, noise of the rig's own position estimate used only to pick pairs
FOV_INT = 90.0                      # indoor photos need a wide field of view
N_INT_POSITIONS, N_INT_HEADINGS = 20, 4


def wd_of(stage):
    return os.path.join(SFM_DIR, stage + SUFFIX)


def load_elements():
    """All BIM elements and their looks, cached (the IFC takes a few minutes to read)."""
    cache = os.path.join(SFM_DIR, "_elements.pkl")
    if os.path.exists(cache):
        with open(cache, "rb") as fh:
            return pickle.load(fh)
    import ifcopenshell
    import ifcopenshell.geom
    f = ifcopenshell.open(p3.IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)
    all_elements = p3.load_all_elements(f, settings)
    look = p3.element_looks(all_elements)
    os.makedirs(SFM_DIR, exist_ok=True)
    with open(cache, "wb") as fh:
        pickle.dump((all_elements, look), fh)
    return all_elements, look


def stage_scene(stage, with_ground=True):
    """Open3D scene of the stage's built elements (+ a ground plane) in the BIM frame."""
    all_elements, look = load_elements()
    p3.LOOK = look
    manifest = json.load(open(os.path.join(STATES_DIR, stage, "manifest.json"), encoding="utf-8"))
    rng = np.random.default_rng(SEED + 1000 + manifest["stage_index"])   # same element jitter as p3_render
    els = p3.stage_elements_from_manifest(manifest, all_elements, rng)
    verts = np.concatenate([e["verts"] for e in els])
    center = verts.mean(axis=0)
    radius = float(np.linalg.norm(verts[:, :2] - center[:2], axis=1).max())
    height_range = (float(verts[:, 2].min()), float(verts[:, 2].max()))
    base_z = float(np.concatenate([e["verts"] for e in all_elements.values()])[:, 2].min())
    scene_els = list(els)
    if with_ground:
        r = radius * 4.0
        c = center[:2]
        gv = np.array([[c[0] - r, c[1] - r, base_z - 0.05], [c[0] + r, c[1] - r, base_z - 0.05],
                       [c[0] + r, c[1] + r, base_z - 0.05], [c[0] - r, c[1] + r, base_z - 0.05]])
        scene_els.append({"verts": gv, "faces": np.array([[0, 1, 2], [0, 2, 3]]),
                          "ifc_type": "ground", "global_id": ""})
    scene, tri_class, tri_gid, _ = sim.make_o3d_scene(scene_els)
    return scene, tri_class, tri_gid, center, radius, height_range, els


def fine_detail(p):
    """Multiplicative detail (N,) fixed on the surface: blocky noise at three scales."""
    out = np.ones(len(p))
    for cell, s in zip(FINE_CELLS, FINE_STRENGTH):
        v = np.floor(p / cell).astype(np.int64)
        out *= 1 + s * (2 * p3._hash01(v[:, 0], v[:, 1], v[:, 2]) - 1)
    return out


def shade_hits(scene, pts, normals, dirs, cls, gid, gain):
    """Colour (N,3) of first hits: material albedo + fine detail, fixed sun with shadows."""
    albedo = np.zeros((len(pts), 3))
    for g in np.unique(gid):
        m = np.nonzero(gid == g)[0]
        if g == "":
            albedo[m] = GROUND_RGB
            continue
        cat, tint, z0 = p3.LOOK[g]
        fac, mortar = p3.surface_texture(cat, pts[m], normals[m])
        a = np.asarray(p3.RENDER_RGB[cat], dtype=float) * tint
        a = a[None, :] * fac[:, None]
        a[mortar] = p3.MORTAR_RGB
        dirty = pts[m, 2] - z0 < p3.DIRT_HEIGHT
        a[dirty] = a[dirty] * 0.78 + np.array([22, 16, 10])
        albedo[m] = a
    albedo *= fine_detail(pts)[:, None]
    nf = normals * -np.sign((normals * dirs).sum(axis=1))[:, None]
    lam = np.clip(nf @ SUN, 0, None)
    lit = np.nonzero(lam > 0)[0]
    o = pts[lit] + nf[lit] * 0.02
    rays = np.concatenate([o, np.broadcast_to(SUN, o.shape)], axis=1).astype(np.float32)
    lam[lit[scene.test_occlusions(o3d.core.Tensor(rays)).numpy().astype(bool)]] = 0
    return np.clip(albedo * (AMBIENT + SUN_GAIN * lam)[:, None] * gain, 0, 255)


def camera_plan(center, radius, height_range):
    """Orbit like p3_render's drone (3 altitude bands x 20 angles), with a little pose noise."""
    rng = np.random.default_rng([SEED, 41])
    views = []
    for kind, eye in sim.exterior_viewpoints(center, radius, height_range):
        eye = eye + rng.normal(0, 0.4, 3)
        target = center + rng.normal(0, [1.5, 1.5, 1.0])
        views.append((eye, target))
    return views


def floor_top(els):
    """Surface height of the lowest floor that has a ceiling above it (skips the foundation slab)."""
    tops = sorted({round(e["z_max"], 1) for e in els if e["ifc_type"] == "IfcSlab"})
    levels, group = [], [tops[0]]
    for z in tops[1:] + [None]:
        if z is not None and z - group[-1] <= 0.5:   # slabs poured at nearly the same height are one floor
            group.append(z)
        else:
            levels.append(max(group))                 # the floor surface is the highest top of the group
            group = [z] if z is not None else []
    return levels[1] if len(levels) >= 2 else levels[0]


def interior_plan(scene, els, center, radius):
    """Photos from a grid of standing positions (the first indoor experiment, --views interior-grid):
    N_INT_HEADINGS views each. Positions inside walls or without floor below / ceiling above are skipped."""
    rng = np.random.default_rng([SEED, 61])
    z0 = floor_top(els)
    n, inset = 7, radius * 0.6
    cand = []
    for i in range(n):
        for j in range(n):
            cand.append(np.array([center[0] + ((i + 0.5) / n * 2 - 1) * inset, center[1] + ((j + 0.5) / n * 2 - 1) * inset, z0 + 1.4]))
    ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    valid = []
    for eye in cand:
        dirs = np.concatenate([np.stack([np.cos(ang), np.sin(ang), np.zeros(24)], 1), [[0, 0, 1.0], [0, 0, -1.0]]])
        rays = np.concatenate([np.broadcast_to(eye, dirs.shape), dirs], axis=1).astype(np.float32)
        t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        side, up, down = t[:24], t[24], t[25]
        if np.nanmin(np.where(np.isfinite(side), side, 99)) > 0.5 and np.isfinite(up) and up < 3.5 and np.isfinite(down) and 1.0 < down < 1.9:
            valid.append(eye)
    pick = sorted(rng.choice(len(valid), size=min(N_INT_POSITIONS, len(valid)), replace=False))
    views = []
    for k in pick:
        eye = valid[k]
        a0 = rng.uniform(0, np.pi / 2)
        for h in range(N_INT_HEADINGS):
            a = a0 + h * 2 * np.pi / N_INT_HEADINGS
            views.append((eye + rng.normal(0, 0.05, 3), eye + np.array([np.cos(a), np.sin(a), -0.05]) * 3.0))
    print(f"interior grid: floor top z={z0:.1f} m, {len(valid)} usable standing positions, {len(pick)} used, {len(views)} photos")
    return views


def interior_walk(scene, els, center, radius):
    """Photos taken while walking the whole floor, like a handheld video: a path that goes through
    every doorway, a photo every WALK_SPACING metres (and one in every narrow passage, so consecutive
    rooms share views), looking along the walking direction plus one side view. This is what makes
    indoor photogrammetry link up; isolated standing positions in separate rooms do not."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    z0 = floor_top(els)
    step = WALK_STEP
    xs = np.arange(center[0] - radius, center[0] + radius + 1e-6, step)
    ys = np.arange(center[1] - radius, center[1] + radius + 1e-6, step)
    nx, ny = len(xs), len(ys)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    pos = np.stack([gx.ravel(), gy.ravel(), np.full(nx * ny, z0 + 1.4)], 1)
    ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    dirs = np.concatenate([np.stack([np.cos(ang), np.sin(ang), np.zeros(16)], 1), [[0, 0, 1.0], [0, 0, -1.0]]])
    t = np.empty((len(pos), 18))
    for lo in range(0, len(pos), 2000):
        sl = slice(lo, lo + 2000)
        o = np.repeat(pos[sl], 18, axis=0)
        d = np.tile(dirs, (len(pos[sl]), 1))
        t[sl] = scene.cast_rays(o3d.core.Tensor(np.concatenate([o, d], 1).astype(np.float32)))["t_hit"].numpy().reshape(-1, 18)
    side = np.where(np.isfinite(t[:, :16]), t[:, :16], 99.0)
    up, down = t[:, 16], t[:, 17]
    free = (side.min(1) > WALK_CLEARANCE) & np.isfinite(up) & (up < 3.5) & np.isfinite(down) & (down > 1.0) & (down < 1.9)
    narrow = ((side[:, :8] < 0.75) & (side[:, 8:] < 0.75)).any(1) & free     # walls close on both sides: a doorway
    # connect free neighbouring cells whose straight line is clear
    idx = np.arange(nx * ny).reshape(nx, ny)
    I, J = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    rows, cols = [], []
    for di, dj in [(1, 0), (0, 1), (1, 1), (1, -1)]:
        I2, J2 = I + di, J + dj
        inside = (I2 < nx) & (J2 >= 0) & (J2 < ny)
        a, b = idx[I[inside], J[inside]], idx[I2[inside], J2[inside]]
        ok = free[a] & free[b]
        a, b = a[ok], b[ok]
        v = pos[b] - pos[a]
        L = np.linalg.norm(v, axis=1)
        tt = scene.cast_rays(o3d.core.Tensor(np.concatenate([pos[a], v / L[:, None]], 1).astype(np.float32)))["t_hit"].numpy()
        clear = ~np.isfinite(tt) | (tt > L + 0.2)
        rows += list(a[clear]); cols += list(b[clear])
    n_cells = nx * ny
    g = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n_cells, n_cells))
    ncomp, lab = connected_components(g, directed=False)
    counts = np.bincount(lab[free], minlength=ncomp)
    main = int(np.argmax(counts))
    print("walkable regions (cells): " + ", ".join(str(int(c)) for c in sorted(counts[counts > 0], reverse=True)[:8]), flush=True)
    comp = np.nonzero((lab == main) & free)[0]
    nbrs = {}
    for a_, b_ in zip(rows, cols):
        nbrs.setdefault(a_, []).append(b_)
        nbrs.setdefault(b_, []).append(a_)
    # depth-first tour of the connected floor (includes the walk back out of dead ends)
    start = comp[np.argmin(np.linalg.norm(pos[comp, :2] - pos[comp, :2].mean(0), axis=1))]
    tour, seen, stack = [start], {start}, [(start, iter(sorted(nbrs.get(start, []))))]
    while stack:
        node, it = stack[-1]
        nxt = next((c for c in it if c not in seen), None)
        if nxt is None:
            stack.pop()
            if stack:
                tour.append(stack[-1][0])
        else:
            seen.add(nxt)
            tour.append(nxt)
            stack.append((nxt, iter(sorted(nbrs.get(nxt, [])))))
    rng = np.random.default_rng([SEED, 67])
    spacing = WALK_SPACING
    while True:
        photos = []                      # (tour index, cell)
        taken = np.zeros((0, 2))
        for k, c in enumerate(tour):
            p2 = pos[c, :2]
            dmin = np.min(np.linalg.norm(taken - p2, axis=1)) if len(taken) else 99.0
            if dmin >= spacing or (narrow[c] and dmin >= 0.9):
                photos.append((k, c))
                taken = np.vstack([taken, p2])
        if 2 * len(photos) <= WALK_MAX_PHOTOS:
            break
        spacing += 0.2
    views, side_sign = [], 1
    for k, c in photos:
        a_ = pos[tour[max(0, k - 2)], :2]
        b_ = pos[tour[min(len(tour) - 1, k + 2)], :2]
        v = b_ - a_
        heading = np.arctan2(v[1], v[0]) if np.linalg.norm(v) > 1e-6 else rng.uniform(0, 2 * np.pi)
        eye = pos[c] + np.array([0, 0, rng.normal(0, 0.03)])
        for yaw in (heading + rng.normal(0, 0.12), heading + side_sign * np.radians(75) + rng.normal(0, 0.12)):
            pitch = rng.normal(-0.05, 0.08)
            views.append((eye + rng.normal(0, 0.04, 3), eye + np.array([np.cos(yaw), np.sin(yaw), pitch]) * 3.0))
        side_sign = -side_sign
    n_door = sum(1 for _, c in photos if narrow[c])
    print(f"interior walk: floor top z={z0:.1f} m, {int(free.sum())} free cells, {len(comp)} reachable, tour {len(tour)} steps, "
          f"{len(photos)} positions (spacing {spacing:.1f} m, {n_door} in doorways), {len(views)} photos")
    return views


def render(stage):
    out = os.path.join(wd_of(stage), "images")
    os.makedirs(out, exist_ok=True)
    scene, tri_class, tri_gid, center, radius, height_range, els = stage_scene(stage)
    views = (interior_walk(scene, els, center, radius) if WALK else interior_plan(scene, els, center, radius)) \
        if INTERIOR else camera_plan(center, radius, height_range)
    fov = FOV_INT if INTERIOR else FOV
    rng = np.random.default_rng([SEED, 43])
    sky = np.array([176, 196, 214], float)
    poses = []
    t0 = time.time()
    for i, (eye, target) in enumerate(views):
        origins, dirs = sim.pinhole_rays(eye, target, fov, (IMG_W, IMG_H))
        res = scene.cast_rays(o3d.core.Tensor(np.concatenate([origins, dirs], axis=1)))
        t = res["t_hit"].numpy()
        ok = np.isfinite(t)
        prim = res["primitive_ids"].numpy()[ok]
        pts = origins[ok] + t[ok, None] * dirs[ok]
        gain = np.exp(rng.normal(0, 0.06)) * np.exp(rng.normal(0, 0.03, 3))
        col = shade_hits(scene, pts, res["primitive_normals"].numpy()[ok], dirs[ok],
                         tri_class[prim], tri_gid[prim], gain)
        img = np.tile(sky, (IMG_W * IMG_H, 1))
        img[ok] = col
        img = img.reshape(IMG_H, IMG_W, 3)
        img = gaussian_filter(img, sigma=(0.7, 0.7, 0))             # lens blur / anti-aliasing
        img = img + rng.normal(0, 2.0, img.shape)                    # sensor noise
        img = np.flipud(np.clip(img, 0, 255)).astype(np.uint8)       # pinhole_rays row 0 is the bottom
        Image.fromarray(img).save(os.path.join(out, f"img_{i:03d}.jpg"), quality=95)
        poses.append({"eye": eye.tolist(), "target": target.tolist()})
        if i % 10 == 0:
            print(f"  image {i + 1}/{len(views)} ({time.time() - t0:.0f}s)", flush=True)
    json.dump({"fov": fov, "size": [IMG_W, IMG_H], "poses": poses},
              open(os.path.join(wd_of(stage), "true_poses.json"), "w"))   # eval only
    print(f"rendered {len(views)} photos -> {out}")


def colmap(stage, dense=True):
    import pycolmap
    wd = wd_of(stage)
    images, db, sparse = os.path.join(wd, "images"), os.path.join(wd, "db.db"), os.path.join(wd, "sparse")
    if os.path.exists(db):
        os.remove(db)
    os.makedirs(sparse, exist_ok=True)
    t0 = time.time()
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"
    pycolmap.extract_features(db, images, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader)
    print(f"features {time.time() - t0:.0f}s", flush=True)
    if WALK:
        # a walking rig knows roughly where it is (odometry / IMU): match photos taken near each other and
        # consecutive ones, instead of all pairs. Matching still verifies the geometry of every pair.
        truth = json.load(open(os.path.join(wd, "true_poses.json")))
        eyes = np.array([q["eye"] for q in truth["poses"]]) + np.random.default_rng([SEED, 71]).normal(0, ODOMETRY_SIGMA, (len(truth["poses"]), 3))
        names_ = [f"img_{i:03d}.jpg" for i in range(len(eyes))]
        pairs = {(i, j) for i in range(len(eyes)) for j in range(i + 1, len(eyes))
                 if np.linalg.norm(eyes[i] - eyes[j]) <= PAIR_RADIUS or j - i <= PAIR_SEQ}
        pair_file = os.path.join(wd, "pairs.txt")
        with open(pair_file, "w") as fh:
            fh.write("\n".join(f"{names_[i]} {names_[j]}" for i, j in sorted(pairs)) + "\n")
        print(f"matching {len(pairs)} of {len(eyes) * (len(eyes) - 1) // 2} possible pairs", flush=True)
        pycolmap.match_image_pairs(db, pairing_options=pycolmap.ImportedPairingOptions(match_list_path=pair_file))
    else:
        pycolmap.match_exhaustive(db)
    print(f"matching {time.time() - t0:.0f}s", flush=True)
    maps = pycolmap.incremental_mapping(db, images, sparse)
    print(f"mapping {time.time() - t0:.0f}s, {len(maps)} model(s)", flush=True)
    if not maps:
        raise SystemExit("COLMAP found no model (too few matches between the photos)")
    best = max(maps.values(), key=lambda r: r.num_reg_images())
    best.write(sparse)
    print(f"best model: {best.num_reg_images()} registered images, {best.num_points3D()} sparse points, "
          f"mean reprojection error {best.compute_mean_reprojection_error():.2f} px")
    if dense:
        make_dense(stage)


def make_dense(stage):
    """Dense depth maps (GPU, patch-match stereo) and a fused cloud from the solved cameras."""
    import pycolmap
    wd = wd_of(stage)
    mvs = os.path.join(wd, "dense")
    t0 = time.time()
    depth_dir = os.path.join(mvs, "stereo", "depth_maps")
    n_depth = len([f for f in os.listdir(depth_dir) if f.endswith(".geometric.bin")]) if os.path.isdir(depth_dir) else 0
    if n_depth:
        print(f"reusing {n_depth} finished depth maps")
        pycolmap.stereo_fusion(os.path.join(mvs, "fused.ply"), mvs, output_type="PLY")
        print(f"dense cloud {time.time() - t0:.0f}s")
        return
    pycolmap.undistort_images(mvs, os.path.join(wd, "sparse"), os.path.join(wd, "images"))
    opts = pycolmap.PatchMatchOptions()
    opts.max_image_size = DENSE_MAX_SIDE      # keeps the depth maps inside a 4 GB GPU
    opts.window_radius = 5
    opts.num_iterations = 5
    pycolmap.patch_match_stereo(mvs, options=opts)
    print(f"depth maps {time.time() - t0:.0f}s", flush=True)
    pycolmap.stereo_fusion(os.path.join(mvs, "fused.ply"), mvs, output_type="PLY")
    print(f"dense cloud {time.time() - t0:.0f}s")


def ideal(stage, stride=4):
    """Upper bound for these photos: the exact 3D point of every 4th pixel, i.e. what a perfect
    reconstruction of the same photos would give (nothing the photos cannot see). Saved like
    the COLMAP cloud, in a survey frame made with the same GNSS offset and tag noise."""
    scene, tri_class, tri_gid, center, radius, height_range, _ = stage_scene(stage)
    truth = json.load(open(os.path.join(wd_of(stage), "true_poses.json")))
    rng = np.random.default_rng([SEED, 53])
    pts = []
    for pose in truth["poses"]:
        eye, target = np.array(pose["eye"]), np.array(pose["target"])
        origins, dirs = sim.pinhole_rays(eye, target, truth["fov"], (IMG_W, IMG_H))
        sel = np.arange(IMG_W * IMG_H).reshape(IMG_H, IMG_W)[::stride, ::stride].ravel()
        res = scene.cast_rays(o3d.core.Tensor(np.concatenate([origins[sel], dirs[sel]], axis=1)))
        t = res["t_hit"].numpy()
        ok = np.isfinite(t)
        ok[ok] = tri_gid[res["primitive_ids"].numpy()[ok]] != ""                        # skip ground and sky
        pts.append(origins[sel][ok] + t[ok, None] * dirs[sel][ok])
    xyz = np.concatenate(pts) + rng.normal(0, 0.005, (sum(len(q) for q in pts), 3))
    O = sim.survey_offset("local" if INTERIOR else "gnss", np.random.default_rng([SEED, 7, len(xyz)]), center)
    xyz_s = xyz @ O[:3, :3].T + O[:3, 3]
    eyes = np.array([q["eye"] for q in truth["poses"]]) @ O[:3, :3].T + O[:3, 3]
    out = OUT_CLOUD_DIR + "_ideal"
    os.makedirs(out, exist_ok=True)
    np.savez_compressed(os.path.join(out, f"{stage}.npz"), xyz=xyz_s.astype(np.float32),
                        rgb=np.full((len(xyz_s), 3), 150, np.uint8), rgb_source=np.array("camera"),
                        gt_T_bim_to_cloud=O, cam_eyes=eyes, cam_kinds=np.array(["spherical" if INTERIOR else "pinhole"] * len(eyes)),
                        cam_target=O[:3, :3] @ center + O[:3, 3])
    print(f"wrote {out}/{stage}.npz ({len(xyz_s)} points from {len(truth['poses'])} photos)")


def read_ply(path):
    import open3d as o3d_
    pc = o3d_.io.read_point_cloud(path)
    return np.asarray(pc.points), (np.asarray(pc.colors) * 255).astype(np.uint8)


def umeyama(src, dst):
    """Scaled rigid fit (4x4, scale folded in) mapping src points onto dst."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(d0.T @ s0 / len(src))
    D = np.diag([1, 1, np.sign(np.linalg.det(U) * np.linalg.det(Vt))])
    R = U @ D @ Vt
    scale = np.trace(np.diag(S) @ D) / s0.var(0).sum()
    T = np.eye(4)
    T[:3, :3] = scale * R
    T[:3, 3] = mu_d - scale * R @ mu_s
    return T


def cloud(stage):
    import pycolmap
    wd = wd_of(stage)
    rec = pycolmap.Reconstruction(os.path.join(wd, "sparse"))
    dense_ply = os.path.join(wd, "dense", "fused.ply")
    if os.path.exists(dense_ply):
        xyz, rgb = read_ply(dense_ply)
        kind = "dense"
    else:
        p = list(rec.points3D.values())
        xyz = np.array([q.xyz for q in p])
        rgb = np.array([q.color for q in p], dtype=np.uint8)
        kind = "sparse"
    names, centres = [], []
    for im in rec.images.values():
        names.append(im.name)
        centres.append(im.projection_center())
    centres = np.array(centres)
    idx = np.array([int(n[4:7]) for n in names])
    truth = json.load(open(os.path.join(wd, "true_poses.json")))
    true_eye = np.array([truth["poses"][i]["eye"] for i in idx])

    # --- the survey frame: BIM -> survey offset as in p3_render (--georef gnss), GNSS tags on top
    scene, tri_class, tri_gid, center, radius, height_range, els = stage_scene(stage)
    rng = np.random.default_rng([SEED, 47])
    A_true = umeyama(centres, true_eye)
    if INTERIOR:
        # no GNSS indoors: metric scale and levelling come from the rig (taken from the true cameras),
        # the frame is the scanner's own start frame (an arbitrary offset, like survey_offset "local")
        O = sim.survey_offset("local", np.random.default_rng([SEED, 7, len(xyz)]), center)
        A = O @ A_true
        gps = None
    else:
        O = sim.survey_offset("gnss", np.random.default_rng([SEED, 7, len(xyz)]), center)
        gps = true_eye @ O[:3, :3].T + O[:3, 3]
        gps = gps + rng.normal(0, [GPS_SIGMA, GPS_SIGMA, 1.5 * GPS_SIGMA], gps.shape)
        A = umeyama(centres, gps)                               # COLMAP frame -> survey frame
    xyz_s = xyz @ A[:3, :3].T + A[:3, 3]
    cam_s = centres @ A[:3, :3].T + A[:3, 3]

    # --- measure the cloud against the true surfaces (eval only): back to the BIM frame with the true O
    Oi = np.linalg.inv(O)
    xyz_b = xyz_s @ Oi[:3, :3].T + Oi[:3, 3]
    verts = np.concatenate([e["verts"] for e in els]).astype(np.float32)
    faces_all, off = [], 0
    for e in els:
        faces_all.append(e["faces"] + off)
        off += len(e["verts"])
    mesh = o3d.t.geometry.TriangleMesh()
    mesh.vertex.positions = o3d.core.Tensor(verts)
    mesh.triangle.indices = o3d.core.Tensor(np.concatenate(faces_all).astype(np.uint32))
    bscene = o3d.t.geometry.RaycastingScene()
    bscene.add_triangles(mesh)
    # (the same measures again with the cloud fitted to the TRUE camera positions: this
    # isolates what COLMAP itself reconstructs from the error of the GNSS-based georeference)
    xyz_t = xyz @ A_true[:3, :3].T + A_true[:3, 3]
    dt = bscene.compute_distance(o3d.core.Tensor(xyz_t.astype(np.float32))).numpy()
    nt = dt < 3.0
    d_pts = bscene.compute_distance(o3d.core.Tensor(xyz_b.astype(np.float32))).numpy()
    near_building = d_pts < 3.0                                  # ignore far ground / sky leftovers
    # completeness: how much of the building surface (as seen from the true cameras) has a cloud point near it
    ref = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts.astype(np.float64)),
                                    o3d.utility.Vector3iVector(np.concatenate(faces_all)))
    samp = np.asarray(ref.sample_points_uniformly(200000).points)
    vis = np.zeros(len(samp), bool)
    ours_dist = cKDTree(xyz_b).query(samp)[0]
    for pose in (truth["poses"] if INTERIOR else truth["poses"][::4]):
        eye = np.array(pose["eye"])
        d = samp - eye
        n = np.linalg.norm(d, axis=1)
        rays = np.concatenate([np.broadcast_to(eye, samp.shape), d / n[:, None]], axis=1).astype(np.float32)
        t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        seen = np.abs(t - n) < 0.15
        if INTERIOR:   # indoors count only what is inside this photo's field of view
            f, r, u = sim.camera_basis(eye, np.array(pose["target"]))
            z, x, y = d @ f, d @ r, d @ u
            half = np.tan(np.radians(truth["fov"]) / 2)
            seen &= (z > 0.1) & (np.abs(x) <= half * z) & (np.abs(y) <= half * z * IMG_H / IMG_W)
        vis |= seen
    ours_dist_true = cKDTree(xyz_t).query(samp)[0]
    metrics = {
        "median_dist_true_fit_m": float(np.median(dt[nt])),
        "share_within_5cm_true_fit": float((dt[nt] < 0.05).mean()),
        "share_within_10cm_true_fit": float((dt[nt] < 0.10).mean()),
        "completeness_within_10cm_true_fit": float((ours_dist_true[vis] < 0.10).mean()),
        "stage": stage, "cloud": kind, "n_points": int(len(xyz)), "n_images": len(names),
        "n_images_total": len(truth["poses"]),
        "camera_fit_rms_to_gnss_m": None if gps is None else float(np.sqrt(((centres @ A[:3, :3].T + A[:3, 3] - gps) ** 2).sum(1).mean())),
        "scale_to_survey": float(np.cbrt(abs(np.linalg.det(A[:3, :3])))),
        "camera_pos_error_m": float(np.sqrt(((cam_s @ Oi[:3, :3].T + Oi[:3, 3] - true_eye) ** 2).sum(1).mean())),
        "median_dist_to_surface_m": float(np.median(d_pts[near_building])),
        "p90_dist_to_surface_m": float(np.percentile(d_pts[near_building], 90)),
        "share_within_5cm": float((d_pts[near_building] < 0.05).mean()),
        "share_within_10cm": float((d_pts[near_building] < 0.10).mean()),
        "share_points_near_building": float(near_building.mean()),
        "surface_seen_by_cameras_m2_share": float(vis.mean()),
        "completeness_within_10cm_of_seen_surface": float((ours_dist[vis] < 0.10).mean()),
    }
    os.makedirs(METRICS_DIR, exist_ok=True)
    json.dump(metrics, open(os.path.join(METRICS_DIR, f"{stage}{SUFFIX}_metrics.json"), "w"), indent=1)
    print(json.dumps(metrics, indent=1))

    os.makedirs(OUT_CLOUD_DIR, exist_ok=True)
    np.savez_compressed(
        os.path.join(OUT_CLOUD_DIR, f"{stage}.npz"),
        xyz=xyz_s.astype(np.float32), rgb=rgb, rgb_source=np.array("camera"),
        # eval only: BIM frame -> this cloud. The cloud sits in the frame fixed by the GNSS fit (A),
        # and the cloud's true BIM position comes from the fit to the true cameras (A_true)
        gt_T_bim_to_cloud=A @ np.linalg.inv(A_true),
        cam_eyes=cam_s, cam_kinds=np.array(["spherical" if INTERIOR else "pinhole"] * len(cam_s)),
        cam_target=O[:3, :3] @ center + O[:3, 3])
    print(f"wrote {OUT_CLOUD_DIR}/{stage}.npz ({kind} cloud, {len(xyz_s)} points)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True)
    ap.add_argument("--step", choices=["render", "colmap", "dense", "cloud", "ideal", "all"], default="all")
    ap.add_argument("--no-dense", action="store_true", help="sparse model only (no GPU needed)")
    ap.add_argument("--views", choices=["exterior", "interior-grid", "interior-walk"], default="exterior",
                    help="photo set: the drone orbit; photos from a grid of standing positions indoors (fragments); "
                         "or a continuous walk through the ground floor (links up)")
    args = ap.parse_args()
    global INTERIOR, WALK, SUFFIX, OUT_CLOUD_DIR
    INTERIOR = args.views != "exterior"
    WALK = args.views == "interior-walk"
    if WALK:
        SUFFIX, OUT_CLOUD_DIR = "_walk", "data/pointclouds_sfm_walk"
    elif INTERIOR:
        SUFFIX, OUT_CLOUD_DIR = "_int", "data/pointclouds_sfm_int"
    if args.step in ("render", "all"):
        render(args.stage)
    if args.step in ("colmap", "all"):
        colmap(args.stage, dense=not args.no_dense)
    if args.step == "ideal":
        ideal(args.stage, stride=10 if WALK else (8 if INTERIOR else 4))
    if args.step == "dense":
        make_dense(args.stage)
    if args.step in ("cloud", "all"):
        cloud(args.stage)


if __name__ == "__main__":
    main()
