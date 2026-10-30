"""Phase 3 -- raycast-sample a point cloud for each Phase 2 manifest state.

Reads data/states/{stage}/manifest.json (kept_ids fully built, partial_ids
height-clipped to their completion_fraction, everything else absent) and
raycasts it exactly as before: exterior drone orbit + interior walkthrough,
Fibonacci-sphere interior rays (no grid-aliasing rings), realistic-config
degradation (per-element pose jitter, whole-scene registration jitter,
sensor noise, dropout), fixed ground + trees shared across all stages.

This replaces the old flat-fraction p2_p3_simulate_stages.py: stages now
come from real milestone/manifest ground truth, not an arbitrary cumulative
percentage.

--site busy (R1, src/site_clutter.py) adds what a real site has around and in
the building -- scaffolding, tower crane, fence, shoring props, material
piles, workers -- and builds 4% of elements a few cm off their designed
position. Clutter is raycast like the building (it occludes and gets sensor
noise), labelled in gt_class; the deviated ids are saved as eval-only
gt_deviated_ids. It also leaves a few scheduled elements out for two stages
(late, gt_missing_ids) and builds a few walls/columns 0.6-1.5 m from their
design place (gt_misplaced_ids); the ground truth counts both as not built.

--color material (default) gives every point the colour a camera would
record, so registration can use colour (src/color_registration.py). The
element's material comes from data/bim/bim_materials.csv (src/materials.py,
read from the IFC), but the colour is NOT the catalogue colour the matching
side knows: this file has its own RENDER_RGB table (different values), and
adds per-element tint (brick batches differ), texture in world coordinates
(brick and block courses with mortar joints, concrete blotches, wood grain),
sun shading with real shadows (shadow rays) for drone photos and indoor
lighting for the scans, a sun direction that changes per survey day, per-
camera exposure and white balance, dirt at the foot of elements, and
SUBSTITUTE_FRAC of elements built from a different material than designed
(the same elements every stage). Clutter gets colours that overlap real
materials (grey scaffolding, brick piles). This is still a coloured raycast,
not photos + SfM (ROADMAP R2). Colour uses its own random streams, so xyz is
the same as before for the same --sensor/--georef/--site.
--color class gives the old look (colour = IFC class); that colour is
ground truth and must not be used by registration (npz field rgb_source
says which one it is).

Run: .venv/bin/python src/p3_render.py [--sensor simple|realistic] [--georef small|gnss|local] [--site clean|busy]
                                       [--color material|class] [--out DIR]
Output: data/pointclouds/{stage}.npz, results/pointcloud_viewer_data.js (BIM group).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import ifcopenshell
import ifcopenshell.geom

sys.path.insert(0, os.path.dirname(__file__))
import p2_p3_simulate_stages as sim  # noqa: E402
import site_clutter  # noqa: E402
import materials  # noqa: E402
from p2_p3_simulate_stages import (  # noqa: E402
    jitter_element, clip_partial_height, make_o3d_scene,
    exterior_viewpoints, interior_viewpoints, raycast, make_ground, make_trees,
    apply_rigid_jitter, pack_entry, survey_offset, CLASS_COLOR,
    NOISE_SIGMA, DROPOUT_FRAC, ELEMENT_JITTER_T, ELEMENT_JITTER_DEG,
    REG_JITTER_T, REG_JITTER_DEG, pinhole_rays,
)
import open3d as o3d  # noqa: E402

IFC_PATH = "data/ifc/IFC Schependomlaan.ifc"
STATES_DIR = "data/states"
OUT_JS = "results/pointcloud_viewer_data.js"
TARGET_POINTS_OUT = 55_000
sim.TARGET_POINTS_OUT = TARGET_POINTS_OUT  # pack_entry reads this from its own module globals
SEED = 0
DART = False
SENSOR = "simple"   # set from --sensor in main()
SITE = "clean"      # set from --site in main()
GEOREF = "small"    # set from --georef in main(): "small" = only the 8 cm / 1.2 deg REG_JITTER,
                    # "gnss" / "local" = add a survey-sized offset (see survey_offset)
COLOR = "material"  # set from --color in main()

# --color material: what the camera sees (catalogue values are in materials.py,
# deliberately different -- see the docstring)
RENDER_RGB = {
    "brick_brown": (128, 70, 50), "brick_yellow": (184, 146, 94), "calcium_silicate": (196, 196, 189),
    "aerated_concrete": (206, 205, 199), "gypsum_block": (214, 213, 207), "concrete_prefab": (172, 171, 166),
    "concrete_cast": (150, 149, 145), "screed": (146, 140, 131), "steel_dark": (54, 58, 65),
    "steel_primer": (126, 58, 45), "wood": (134, 92, 56), "plywood": (184, 150, 104),
    "board_white": (226, 226, 221), "zinc": (138, 146, 151), "tile": (108, 103, 97),
    "insulation": (232, 220, 168),
}
CLUTTER_RENDER_RGB = {   # chosen to overlap real materials, so colour can't simply filter clutter
    "scaffold": (150, 152, 150), "crane": (214, 176, 36), "fence": (148, 154, 148),
    "props": (188, 96, 42), "pile": (134, 78, 58), "worker": (226, 150, 40),
}
CLUTTER_RENDER_RGB.update(site_clutter.INTERIOR_RGB)    # multi-part machines / tools, each part its own look
MORTAR_RGB = (176, 172, 164)
# (course height, unit length, joint width) in m, for the walls built from units
COURSES = {"brick_brown": (0.0625, 0.22, 0.012), "brick_yellow": (0.0625, 0.22, 0.012),
           "calcium_silicate": (0.20, 0.90, 0.004), "aerated_concrete": (0.25, 0.60, 0.004),
           "gypsum_block": (0.50, 0.66, 0.004)}
SUBSTITUTE_FRAC = 0.06   # elements built from another material than designed
SUBSTITUTE = {           # a plausible mix-up for each category
    "brick_brown": "brick_yellow", "brick_yellow": "brick_brown", "calcium_silicate": "aerated_concrete",
    "aerated_concrete": "calcium_silicate", "gypsum_block": "board_white", "concrete_prefab": "concrete_cast",
    "concrete_cast": "concrete_prefab", "screed": "concrete_cast", "steel_dark": "steel_primer",
    "steel_primer": "steel_dark", "wood": "plywood", "plywood": "wood", "board_white": "plywood",
    "zinc": "steel_dark", "tile": "screed", "insulation": "board_white",
}
TINT_SIGMA = 0.10        # per-element brightness (log); per channel 0.035
EXPOSURE_SIGMA = 0.18    # per-camera gain (log); white balance per channel 0.05
AMBIENT, SUN = 0.45, 0.65
DIRT_HEIGHT = 0.4        # m above an element's base that gets splashed / dirty
LOOK = {}                # filled in main(): rendered category, tint and base height per global_id

# --sensor realistic (ROADMAP R2, first step -- still raycast, but with error
# that behaves like a real scan/photogrammetry cloud instead of uniform noise):
RS_SIGMA0 = 0.005          # m, depth noise floor
RS_SIGMA_PER_M = 0.0008    # m of depth noise per m of range (drone at 30 m -> ~3 cm)
RS_MIN_COS = 0.2           # grazing-angle noise blow-up is capped here
RS_BASE_DROP = 0.10
RS_GRAZING_COS = 0.3       # below this |cos(incidence)|, dropout ramps to 100%
RS_HOLE_AREA = 60.0        # m^2 of scanned surface per texture-less hole
RS_HOLE_RADIUS = (0.3, 1.5)
RS_MIXED_FRAC = 0.01       # edge "mixed pixel" ghosts pushed behind the surface
RS_FLOATER_FRAC = 0.003    # random floaters in the bounding box


def realistic_sensor(pts, rng_m, cos_inc, dirs, rng, photo):
    """Returns (noisy points, keep mask). Noise mostly along the viewing ray
    and grows with range and grazing angle; points seen almost edge-on mostly
    vanish; texture-less patches leave holes in the drone-photo (photogrammetry)
    points only -- the indoor scanner is a laser and needs no texture; a few
    edge ghosts and floaters."""
    n = pts.shape[0]
    sigma = (RS_SIGMA0 + RS_SIGMA_PER_M * rng_m) / np.maximum(cos_inc, RS_MIN_COS)
    pts = pts + dirs * (rng.normal(0, 1, n) * sigma)[:, None] + rng.normal(0, RS_SIGMA0, (n, 3))

    p_drop = RS_BASE_DROP + (1 - RS_BASE_DROP) * np.clip((RS_GRAZING_COS - cos_inc) / RS_GRAZING_COS, 0, 1)
    keep = rng.random(n) > p_drop

    # holes per unit of SURFACE, centred uniformly over it. (An earlier
    # version used one hole per 20k raw rays, centred on random rays: with
    # millions of rays that was hundreds of holes, piled up where rays are
    # densest -- around the indoor scanners -- and it wiped out whole rooms.)
    # (A later bug: holes also hit the indoor laser scans, where points are
    # densest next to the scanner, so one hole could erase most of a slab.)
    ph = np.nonzero(photo)[0]
    if ph.size:
        vox = np.unique(np.floor(pts[ph] / 0.5).astype(np.int64), axis=0)
        n_holes = max(1, int(len(vox) * 0.25 / RS_HOLE_AREA))
        centres = (vox[rng.choice(len(vox), n_holes, replace=False)] + 0.5) * 0.5
        radii = rng.uniform(*RS_HOLE_RADIUS, n_holes)
        from scipy.spatial import cKDTree
        tree = cKDTree(pts[ph])
        for c, r in zip(centres, radii):
            keep[ph[tree.query_ball_point(c, r)]] = False

    mixed = rng.random(n) < RS_MIXED_FRAC
    pts[mixed] += dirs[mixed] * rng.uniform(0.05, 1.0, int(mixed.sum()))[:, None]
    return pts, keep


def _hash01(*ints):
    """Repeatable pseudo-random number in [0, 1) per integer cell."""
    h = np.zeros(ints[0].shape, dtype=np.int64)
    for k, a in zip((73856093, 19349663, 83492791), ints):
        h ^= a.astype(np.int64) * k
    return ((h * 2654435761) & 0xFFFF) / 65536.0


def surface_texture(cat, p, n):
    """Albedo factor (N,) and mortar mask (N,) for points p (N,3) with normals n on one category."""
    fac = np.ones(len(p))
    mortar = np.zeros(len(p), dtype=bool)
    if cat in COURSES:
        h, length, joint = COURSES[cat]
        u = -n[:, 1] * p[:, 0] + n[:, 0] * p[:, 1]      # along the wall face
        k = np.floor(p[:, 2] / h)
        us = u + (k % 2) * length / 2
        b = np.floor(us / length)
        mortar = (p[:, 2] - k * h < joint) | (us - b * length < joint)
        fac = 0.82 + 0.36 * _hash01(k, b)
    elif cat == "wood":
        fac = 0.9 + 0.1 * np.sin(p[:, 2] * 45 + 7 * _hash01(np.floor(p[:, 0] / 0.15), np.floor(p[:, 1] / 0.15)))
    else:   # concrete, board, steel: soft blotches
        v = np.floor(p / 0.4).astype(np.int64)
        fac = 0.9 + 0.2 * _hash01(v[:, 0], v[:, 1], v[:, 2])
    return fac, mortar


def camera_colors(pts, classes, gids, normals, dirs, cos_inc, view, photo, scene, n_views, rng):
    """Camera colour (N,3 uint8) of each first hit, before sensor noise moves it."""
    n = len(pts)
    albedo = np.zeros((n, 3))
    base_z = np.full(n, -np.inf)
    uniq, inv = np.unique(gids, return_inverse=True)
    order = np.argsort(inv, kind="stable")
    groups = np.split(order, np.cumsum(np.bincount(inv, minlength=len(uniq)))[:-1])
    for gid, m in zip(uniq, groups):
        if gid == "":
            for cls in np.unique(classes[m]):
                albedo[m[classes[m] == cls]] = CLUTTER_RENDER_RGB.get(cls, (128, 128, 128))
            continue
        cat, tint, z0 = LOOK[gid]
        fac, mortar = surface_texture(cat, pts[m], normals[m])
        a = np.asarray(RENDER_RGB[cat], dtype=float) * tint
        a = a[None, :] * fac[:, None]
        a[mortar] = MORTAR_RGB
        albedo[m] = a
        base_z[m] = z0
    albedo *= rng.lognormal(0, 0.03, (n, 1))                         # fine grain
    dirty = pts[:, 2] - base_z < DIRT_HEIGHT
    albedo[dirty] = albedo[dirty] * 0.78 + np.array([22, 16, 10])

    # light: drone photos have sun + shadows, indoor scans their own even light
    az, el = rng.uniform(0, 2 * np.pi), np.radians(rng.uniform(25, 60))
    sun = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    nf = normals * -np.sign((normals * dirs).sum(axis=1))[:, None]   # face the camera
    lam = np.clip(nf @ sun, 0, None)
    lit = photo & (lam > 0)
    if lit.any():
        o = pts[lit] + nf[lit] * 0.02
        rays = np.concatenate([o, np.broadcast_to(sun, o.shape)], axis=1).astype(np.float32)
        shadow = scene.test_occlusions(o3d.core.Tensor(rays)).numpy().astype(bool)
        idx = np.nonzero(lit)[0]
        lam[idx[shadow]] = 0
    shade = np.where(photo, AMBIENT + SUN * lam, 0.55 + 0.4 * cos_inc)

    gain = rng.lognormal(0, EXPOSURE_SIGMA, n_views)
    wb = rng.lognormal(0, 0.05, (n_views, 3))
    rgb = albedo * shade[:, None] * gain[view, None] * wb[view] + rng.normal(0, 3, (n, 3))
    return np.clip(rgb, 0, 255).astype(np.uint8)


def element_looks(all_elements):
    """Rendered category, tint and base height per element; fixed for the whole project."""
    cats = materials.load_categories()
    rng = np.random.default_rng(SEED + 31)
    look = {}
    for gid in sorted(all_elements):
        cat = cats.get(gid, materials.DEFAULT)
        if rng.random() < SUBSTITUTE_FRAC:
            cat = SUBSTITUTE[cat]
        tint = np.exp(rng.normal(0, TINT_SIGMA) + rng.normal(0, 0.035, 3))
        look[gid] = (cat, tint, all_elements[gid]["z_min"])
    return look


def load_all_elements(f, settings):
    elements = {}
    for ifc_type in ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall"]:
        for el in f.by_type(ifc_type):
            try:
                shape = ifcopenshell.geom.create_shape(settings, el)
            except Exception:
                continue
            verts = np.array(shape.geometry.verts, dtype=np.float64).reshape(-1, 3)
            faces = np.array(shape.geometry.faces, dtype=np.int64).reshape(-1, 3)
            if verts.shape[0] == 0 or faces.shape[0] == 0:
                continue
            elements[el.GlobalId] = {
                "global_id": el.GlobalId, "ifc_type": ifc_type,
                "verts": verts, "faces": faces,
                "z_min": float(verts[:, 2].min()), "z_max": float(verts[:, 2].max()),
            }
    return elements


def stage_elements_from_manifest(manifest, all_elements, rng):
    partial_frac = {p["global_id"]: p["completion_fraction"] for p in manifest["partial_ids"]}
    out = []
    for gid in manifest["kept_ids"]:
        if gid in all_elements:
            out.append(dict(all_elements[gid]))
    for gid, frac in partial_frac.items():
        if gid not in all_elements:
            continue
        clipped = clip_partial_height(all_elements[gid], frac)
        if clipped is not None:
            out.append(clipped)
    return [jitter_element(e, rng, ELEMENT_JITTER_T, ELEMENT_JITTER_DEG) for e in out]


DART_GRID = (480, 360)      # image size of the DART camera views
DART_FOV = 80.0
DART_HEADINGS = 4           # views per interior eye
DART_EXT_STRIDE = 3         # every 3rd exterior orbit viewpoint
DART_VIEWS = []             # filled by run_stage when --dart: one dict per image


def dart_views(scene, tri_class, tri_gid, viewpoints, center, rng, color_rng):
    """Organised camera images for DART: per pixel the first-hit xyz (NaN = sky), the camera colour
    (same look as the main cloud) and the eval-only hit class. Interior eyes get DART_HEADINGS
    views turned about z; exterior drones look at the building as in the main cloud."""
    views = []
    ext = [v for v in viewpoints if v[0] == "pinhole"][::DART_EXT_STRIDE]
    for kind, eye in ext:
        views.append((eye, center, True))
    for kind, eye in viewpoints:
        if kind == "pinhole":
            continue
        for h in range(DART_HEADINGS):
            a = 2 * np.pi * h / DART_HEADINGS + rng.uniform(0, 0.4)
            views.append((eye, eye + np.array([np.cos(a), np.sin(a), -0.05]), False))
    out = []
    for vi, (eye, target, photo) in enumerate(views):
        origins, dirs = pinhole_rays(eye, target, DART_FOV, DART_GRID)
        res = scene.cast_rays(o3d.core.Tensor(np.concatenate([origins, dirs], axis=1)))
        t = res["t_hit"].numpy()
        ok = np.isfinite(t)
        prim = res["primitive_ids"].numpy()[ok]
        pts = origins[ok] + t[ok, None] * dirs[ok]
        normals = res["primitive_normals"].numpy()[ok]
        cos = np.abs((normals * dirs[ok]).sum(axis=1))
        col = camera_colors(pts, tri_class[prim], tri_gid[prim], normals, dirs[ok], cos,
                            np.zeros(len(pts), int), np.full(len(pts), photo), scene, 1, color_rng)
        w, h = DART_GRID
        xyz = np.full((h * w, 3), np.nan, np.float32)
        rgb = np.tile(np.array([176, 196, 214], np.uint8), (h * w, 1))
        cls = np.full(h * w, "sky", dtype=object)
        gid = np.full(h * w, "", dtype=object)
        xyz[ok] = pts + rng.normal(0, NOISE_SIGMA, pts.shape)
        rgb[ok] = col
        cls[ok] = tri_class[prim]
        gid[ok] = tri_gid[prim]
        out.append({"eye": eye, "target": target, "xyz": xyz.reshape(h, w, 3), "rgb": rgb.reshape(h, w, 3),
                    "gt_class": cls.reshape(h, w), "gt_gid": gid.reshape(h, w),
                    "kind": "exterior" if photo else "interior"})
    return out


def run_stage(stage_elements, rng, ground_trees, clutter=None, color_rng=None):
    # camera plan from the building alone, so clutter doesn't move the cameras
    verts = np.concatenate([e["verts"] for e in stage_elements], axis=0).astype(np.float32)
    center = verts.mean(axis=0)
    footprint_radius = float(np.linalg.norm(verts[:, :2] - center[:2], axis=1).max())
    height_range = (float(verts[:, 2].min()), float(verts[:, 2].max()))

    viewpoints = exterior_viewpoints(center, footprint_radius, height_range)
    viewpoints += interior_viewpoints(stage_elements, center, footprint_radius)
    if clutter:
        eyes = np.array([eye for kind, eye in viewpoints if kind != "pinhole"]).reshape(-1, 3)
        clutter = site_clutter.drop_blocking(clutter, eyes)
    scene, tri_class, tri_gid, _ = make_o3d_scene(stage_elements + (clutter or []))
    DART_VIEWS.clear()
    if DART:   # own seeded streams: the main cloud is unchanged by --dart
        DART_VIEWS.extend(dart_views(scene, tri_class, tri_gid, viewpoints, center,
                                     np.random.default_rng([SEED, 29, len(viewpoints)]),
                                     np.random.default_rng([SEED, 31, len(viewpoints)])))
    # (the extra ray geometry costs no random numbers, so xyz doesn't depend on --color)
    pts, classes, gids, rng_m, cos_inc, dirs, photo, normals, view = raycast(
        scene, tri_class, tri_gid, viewpoints, center, rng, return_geometry=True)
    if COLOR == "material":
        colors = camera_colors(pts, classes, gids, normals, dirs, cos_inc, view, photo, scene,
                               len(viewpoints), color_rng)
    else:
        colors = np.array([CLASS_COLOR.get(c, [200, 200, 200]) for c in classes], dtype=np.uint8)
    if SENSOR == "realistic":
        pts, keep = realistic_sensor(pts, rng_m, cos_inc, dirs, rng, photo)
        pts, classes, gids, colors = pts[keep], classes[keep], gids[keep], colors[keep]
        n_float = int(pts.shape[0] * RS_FLOATER_FRAC)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        pts = np.concatenate([pts, rng.uniform(lo, hi, (n_float, 3))])
        classes = np.concatenate([classes, np.full(n_float, "noise", dtype=object)])
        gids = np.concatenate([gids, np.full(n_float, "", dtype=object)])
        if COLOR == "material":
            float_col = np.repeat(color_rng.integers(60, 200, (n_float, 1)), 3, axis=1)
        else:
            float_col = np.tile(CLASS_COLOR.get("noise", [200, 200, 200]), (n_float, 1))
        colors = np.concatenate([colors, float_col.astype(np.uint8)])
    else:
        pts = pts + rng.normal(0, NOISE_SIGMA, pts.shape)
        keep = rng.random(pts.shape[0]) > DROPOUT_FRAC
        pts, classes, gids, colors = pts[keep], classes[keep], gids[keep], colors[keep]

    n_structure = pts.shape[0]

    ground_pts, ground_col, tree_pts, tree_col = ground_trees
    ground_classes = np.full(ground_pts.shape[0], "ground", dtype=object)
    tree_classes = np.full(tree_pts.shape[0], "tree", dtype=object)
    clutter_gids = np.full(ground_pts.shape[0] + tree_pts.shape[0], "", dtype=object)

    all_pts = np.concatenate([pts, ground_pts, tree_pts], axis=0)
    all_col = np.concatenate([colors, ground_col, tree_col], axis=0)
    all_gt_class = np.concatenate([classes, ground_classes, tree_classes], axis=0)
    all_gt_gid = np.concatenate([gids, clutter_gids], axis=0)
    all_pts, T_true = apply_rigid_jitter(all_pts, rng, REG_JITTER_T, REG_JITTER_DEG, return_transform=True)
    if GEOREF != "small":
        # separate seeded stream so --georef small output stays bit-identical
        O = survey_offset(GEOREF, np.random.default_rng([SEED, 7, len(all_pts)]), all_pts.mean(axis=0))
        all_pts = all_pts @ O[:3, :3].T + O[:3, 3]
        T_true = O @ T_true
    # camera poses are reported in the SURVEY frame, like a real SfM/SLAM
    # output; p4_visibility.py maps them back with the ESTIMATED registration
    cam_eyes = np.array([eye for _kind, eye in viewpoints], dtype=np.float64)
    cam_eyes = cam_eyes @ T_true[:3, :3].T + T_true[:3, 3]
    cam_target = T_true[:3, :3] @ center + T_true[:3, 3]
    cam_kinds = np.array([kind for kind, _eye in viewpoints])

    return (all_pts, all_col, all_gt_class, all_gt_gid, n_structure, center, footprint_radius,
            T_true, cam_eyes, cam_kinds, cam_target)  # pinhole cameras all look at cam_target


def save_dart_views(stage_name, T_true):
    """data/dart/{stage}/views.npz: all view images, in the same survey frame as the stage cloud."""
    d = os.path.join("data", "dart", stage_name)
    os.makedirs(d, exist_ok=True)
    xyz = np.stack([v["xyz"] for v in DART_VIEWS])
    xyz = xyz @ T_true[:3, :3].T + T_true[:3, 3]
    cls = np.stack([v["gt_class"] for v in DART_VIEWS]).astype(str)
    gid = np.stack([v["gt_gid"] for v in DART_VIEWS]).astype(str)
    class_names, cls_i = np.unique(cls, return_inverse=True)     # ints, not strings: 100x smaller
    gid_names, gid_i = np.unique(gid, return_inverse=True)
    np.savez_compressed(os.path.join(d, "views.npz"), xyz=xyz.astype(np.float32),
                        rgb=np.stack([v["rgb"] for v in DART_VIEWS]),
                        gt_class_names=class_names, gt_class_idx=cls_i.reshape(cls.shape).astype(np.uint8),
                        gt_gid_names=gid_names, gt_gid_idx=gid_i.reshape(gid.shape).astype(np.int32),
                        kind=np.array([v["kind"] for v in DART_VIEWS]),
                        # camera pose in the survey frame (same frame as xyz)
                        eyes=(np.array([v["eye"] for v in DART_VIEWS]) @ T_true[:3, :3].T + T_true[:3, 3]).astype(np.float32),
                        targets=(np.array([v["target"] for v in DART_VIEWS]) @ T_true[:3, :3].T + T_true[:3, 3]).astype(np.float32))
    print(f"  saved {len(DART_VIEWS)} DART views to {d}/views.npz")


def save_stage_pointcloud(stage_name, xyz, rgb, gt_class, gt_global_id, T_true, cam_eyes, cam_kinds,
                          cam_target, out_dir="data/pointclouds", deviated_ids=(), rgb_source="camera",
                          missing_ids=(), misplaced_ids=()):
    """Plain float32 xyz in the original BIM world frame (same frame as
    bim_complete.csv), NOT the viewer's centered/axis-swapped/quantized one --
    this is what a segmentation script should load. gt_class/gt_global_id and
    gt_T_bim_to_cloud (the true whole-scene registration offset) are held-out
    ground truth for evaluation only; never feed them to a segmentation or
    registration algorithm. cam_eyes/cam_kinds are the sensor positions in the
    SURVEY frame, same as xyz (a real survey knows its camera poses too, from
    SfM/SLAM); pinhole cameras look at cam_target. rgb_source is "camera"
    (simulated camera colour, fine to use) or "class" (colour = IFC class,
    ground truth)."""
    os.makedirs(out_dir, exist_ok=True)
    np.savez_compressed(
        os.path.join(out_dir, f"{stage_name}.npz"),
        xyz=xyz.astype(np.float32),
        rgb=rgb.astype(np.uint8),
        gt_class=np.asarray(gt_class, dtype="<U16"),
        gt_global_id=np.asarray(gt_global_id, dtype="<U24"),
        gt_T_bim_to_cloud=T_true,
        cam_eyes=cam_eyes,
        cam_kinds=cam_kinds,
        cam_target=cam_target,
        gt_deviated_ids=np.asarray(sorted(deviated_ids), dtype="<U24"),
        gt_missing_ids=np.asarray(sorted(missing_ids), dtype="<U24"),
        gt_misplaced_ids=np.asarray(sorted(misplaced_ids), dtype="<U24"),
        rgb_source=rgb_source,
    )


def storey_short(storey):
    if "fundering" in storey:
        return "Foundation"
    return storey.replace(" begane grond", " GF") \
        .replace(" eerste verdieping", " 1F").replace(" tweede verdieping", " 2F") \
        .replace(" derde verdieping", " 3F").replace(" dak", " Roof").strip()


def main():
    global SENSOR, GEOREF, SITE, COLOR, LOOK, DART
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", choices=["simple", "realistic"], default="simple")
    ap.add_argument("--georef", choices=["small", "gnss", "local"], default="small")
    ap.add_argument("--site", choices=["clean", "busy"], default="clean")
    ap.add_argument("--color", choices=["material", "class"], default="material")
    ap.add_argument("--interior", type=int, default=0,
                    help="busy site only: machinery / temporary items per built floor (0 = none)")
    ap.add_argument("--stage", default=None, help="render only this stage directory name (e.g. 11_03_derde_verdieping_structure)")
    ap.add_argument("--dart", action="store_true",
                    help="also save camera images + organised xyz per stage in data/dart/ for dart_label.py")
    ap.add_argument("--out", default="data/pointclouds", help="output dir for the stage .npz files")
    args = ap.parse_args()
    DART = args.dart
    SENSOR, GEOREF, SITE, COLOR = args.sensor, args.georef, args.site, args.color
    print(f"Sensor model: {SENSOR}, georeference: {GEOREF}, site: {SITE}, colour: {COLOR}")
    CLASS_COLOR.update(site_clutter.CLUTTER_COLOR)
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    print("Extracting full-building element geometry (once)...")
    all_elements = load_all_elements(f, settings)
    print(f"{len(all_elements)} elements available for raycasting")
    LOOK = element_looks(all_elements)

    all_verts_full = np.concatenate([e["verts"] for e in all_elements.values()], axis=0)
    full_center = all_verts_full.mean(axis=0)
    full_radius = float(np.linalg.norm(all_verts_full[:, :2] - full_center[:2], axis=1).max())
    base_z = float(all_verts_full[:, 2].min())
    clutter_rng = np.random.default_rng(SEED + 999)
    ground_trees = (*make_ground(full_center, full_radius, base_z, clutter_rng),
                    *make_trees(full_center, full_radius, base_z, clutter_rng))
    ground_trees = (ground_trees[0], ground_trees[1], ground_trees[2], ground_trees[3])

    site, dev, late, moved = None, {}, set(), {}
    if SITE == "busy":
        site = site_clutter.Site(all_elements)
        dev = site_clutter.deviations(all_elements.keys())
        late, moved = site_clutter.late_and_misplaced(all_elements, dev)
        print(f"Busy site: {len(dev)} elements built 4-12 cm off, {len(moved)} misplaced, "
              f"{len(late)} late")
    first_stage = {}   # gid -> loop index of the first stage that schedules it

    stage_dirs = sorted(glob.glob(os.path.join(STATES_DIR, "*")))
    entries = []
    for stage_dir in stage_dirs:
        with open(os.path.join(stage_dir, "manifest.json"), encoding="utf-8") as fh:
            manifest = json.load(fh)
        i = manifest["stage_index"]
        rng = np.random.default_rng(SEED + 1000 + i)

        stage_elements = stage_elements_from_manifest(manifest, all_elements, rng)
        clutter, deviated, missing, misplaced = None, (), (), ()
        k = len(entries)
        for e in stage_elements:
            first_stage.setdefault(e["global_id"], k)
        if site is not None:
            missing = [e["global_id"] for e in stage_elements
                       if e["global_id"] in late and k - first_stage[e["global_id"]] < site_clutter.LATE_STAGES]
            stage_elements = [e for e in stage_elements if e["global_id"] not in set(missing)]
            stage_elements = site_clutter.apply_deviations(stage_elements, {**dev, **moved})
            deviated = [e["global_id"] for e in stage_elements if e["global_id"] in dev]
            misplaced = [e["global_id"] for e in stage_elements if e["global_id"] in moved]
            clutter = site_clutter.stage_clutter(site, stage_elements, i, manifest["phase"],
                                                 is_last=stage_dir == stage_dirs[-1],
                                                 interior=args.interior)
        if args.stage and os.path.basename(stage_dir) != args.stage:
            entries.append(None)     # keep k (stage position) counting for the late-element logic
            continue
        print(f"\nStage {i:02d} [{manifest['storey_milestone']} / {manifest['phase']}]: "
              f"{len(stage_elements)} elements ({manifest['n_fully_built']} full, "
              f"{manifest['n_partial']} partial, {len(missing)} late, {len(misplaced)} misplaced), "
              f"raycasting...")

        (all_pts, all_col, all_gt_class, all_gt_gid, n_structure, center, fr,
         T_true, cam_eyes, cam_kinds, cam_target) = run_stage(stage_elements, rng, ground_trees, clutter,
                                                              np.random.default_rng([SEED, 11, i]))
        n_full = all_pts.shape[0]
        if DART:
            save_dart_views(os.path.basename(stage_dir), T_true)

        stage_name = os.path.basename(stage_dir)
        save_stage_pointcloud(stage_name, all_pts, all_col, all_gt_class, all_gt_gid,
                              T_true, cam_eyes, cam_kinds, cam_target, out_dir=args.out,
                              deviated_ids=deviated, rgb_source="camera" if COLOR == "material" else "class",
                              missing_ids=missing, misplaced_ids=misplaced)

        label = f"BIM: {i:02d} {storey_short(manifest['storey_milestone'])} - {manifest['phase']}"
        entry = pack_entry(label, all_pts, all_col, n_full, center, fr * 1.15)
        entry["stageIndex"] = i
        entries.append(entry)
        print(f"  packed: {entry['nPoints']} pts (structure {n_structure}), "
              f"saved {args.out}/{stage_name}.npz")

    if args.out != "data/pointclouds":
        print(f"\n--out given: left {OUT_JS} unchanged (it shows the default run)")
        return
    if not os.path.exists(OUT_JS):   # made by export_pointcloud_viewer_data.py; not needed for the clouds
        print(f"\n{OUT_JS} not found: viewer not updated (run export_pointcloud_viewer_data.py first)")
        return
    with open(OUT_JS, encoding="utf-8") as fh:
        txt = fh.read()
    payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))

    payload["weeks"] = [w for w in payload["weeks"] if not w["label"].startswith("BIM: 0")
                         and not w["label"].startswith("BIM: 1")
                         and not w["label"].startswith("BIM: Week")]
    payload["weeks"] += entries

    with open(OUT_JS, "w", encoding="utf-8") as fh:
        fh.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, fh)
        fh.write(";\n")

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
