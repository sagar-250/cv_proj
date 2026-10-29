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
gt_deviated_ids.

Run: .venv/bin/python src/p3_render.py [--sensor simple|realistic] [--georef small|gnss|local] [--site clean|busy]
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
from p2_p3_simulate_stages import (  # noqa: E402
    jitter_element, clip_partial_height, make_o3d_scene,
    exterior_viewpoints, interior_viewpoints, raycast, make_ground, make_trees,
    apply_rigid_jitter, pack_entry, survey_offset, CLASS_COLOR,
    NOISE_SIGMA, DROPOUT_FRAC, ELEMENT_JITTER_T, ELEMENT_JITTER_DEG,
    REG_JITTER_T, REG_JITTER_DEG,
)

IFC_PATH = "data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc"
STATES_DIR = "data/states"
OUT_JS = "results/pointcloud_viewer_data.js"
TARGET_POINTS_OUT = 55_000
sim.TARGET_POINTS_OUT = TARGET_POINTS_OUT  # pack_entry reads this from its own module globals
SEED = 0
SENSOR = "simple"   # set from --sensor in main()
SITE = "clean"      # set from --site in main()
GEOREF = "small"    # set from --georef in main(): "small" = only the 8 cm / 1.2 deg REG_JITTER,
                    # "gnss" / "local" = add a survey-sized offset (see survey_offset)

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


def realistic_sensor(pts, rng_m, cos_inc, dirs, rng):
    """Returns (noisy points, keep mask). Noise mostly along the viewing ray
    and grows with range and grazing angle; points seen almost edge-on mostly
    vanish; texture-less patches leave holes; a few edge ghosts and floaters."""
    n = pts.shape[0]
    sigma = (RS_SIGMA0 + RS_SIGMA_PER_M * rng_m) / np.maximum(cos_inc, RS_MIN_COS)
    pts = pts + dirs * (rng.normal(0, 1, n) * sigma)[:, None] + rng.normal(0, RS_SIGMA0, (n, 3))

    p_drop = RS_BASE_DROP + (1 - RS_BASE_DROP) * np.clip((RS_GRAZING_COS - cos_inc) / RS_GRAZING_COS, 0, 1)
    keep = rng.random(n) > p_drop

    # holes per unit of SURFACE, centred uniformly over it. (An earlier
    # version used one hole per 20k raw rays, centred on random rays: with
    # millions of rays that was hundreds of holes, piled up where rays are
    # densest -- around the indoor scanners -- and it wiped out whole rooms.)
    vox = np.unique(np.floor(pts / 0.5).astype(np.int64), axis=0)
    n_holes = max(1, int(len(vox) * 0.25 / RS_HOLE_AREA))
    centres = (vox[rng.choice(len(vox), n_holes, replace=False)] + 0.5) * 0.5
    radii = rng.uniform(*RS_HOLE_RADIUS, n_holes)
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    for c, r in zip(centres, radii):
        keep[tree.query_ball_point(c, r)] = False

    mixed = rng.random(n) < RS_MIXED_FRAC
    pts[mixed] += dirs[mixed] * rng.uniform(0.05, 1.0, int(mixed.sum()))[:, None]
    return pts, keep


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


def run_stage(stage_elements, rng, ground_trees, clutter=None):
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
    if SENSOR == "realistic":
        pts, classes, gids, rng_m, cos_inc, dirs = raycast(
            scene, tri_class, tri_gid, viewpoints, center, rng, return_geometry=True)
        pts, keep = realistic_sensor(pts, rng_m, cos_inc, dirs, rng)
        pts, classes, gids = pts[keep], classes[keep], gids[keep]
        n_float = int(pts.shape[0] * RS_FLOATER_FRAC)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        pts = np.concatenate([pts, rng.uniform(lo, hi, (n_float, 3))])
        classes = np.concatenate([classes, np.full(n_float, "noise", dtype=object)])
        gids = np.concatenate([gids, np.full(n_float, "", dtype=object)])
    else:
        pts, classes, gids = raycast(scene, tri_class, tri_gid, viewpoints, center, rng)
        pts = pts + rng.normal(0, NOISE_SIGMA, pts.shape)
        keep = rng.random(pts.shape[0]) > DROPOUT_FRAC
        pts, classes, gids = pts[keep], classes[keep], gids[keep]

    colors = np.array([CLASS_COLOR.get(c, [200, 200, 200]) for c in classes], dtype=np.uint8)
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


def save_stage_pointcloud(stage_name, xyz, rgb, gt_class, gt_global_id, T_true, cam_eyes, cam_kinds,
                          cam_target, out_dir="data/pointclouds", deviated_ids=()):
    """Plain float32 xyz in the original BIM world frame (same frame as
    bim_complete.csv), NOT the viewer's centered/axis-swapped/quantized one --
    this is what a segmentation script should load. gt_class/gt_global_id and
    gt_T_bim_to_cloud (the true whole-scene registration offset) are held-out
    ground truth for evaluation only; never feed them to a segmentation or
    registration algorithm. cam_eyes/cam_kinds are the sensor positions in the
    SURVEY frame, same as xyz (a real survey knows its camera poses too, from
    SfM/SLAM); pinhole cameras look at cam_target."""
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
    )


def storey_short(storey):
    if "fundering" in storey:
        return "Foundation"
    return storey.replace(" begane grond", " GF") \
        .replace(" eerste verdieping", " 1F").replace(" tweede verdieping", " 2F") \
        .replace(" derde verdieping", " 3F").replace(" dak", " Roof").strip()


def main():
    global SENSOR, GEOREF, SITE
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensor", choices=["simple", "realistic"], default="simple")
    ap.add_argument("--georef", choices=["small", "gnss", "local"], default="small")
    ap.add_argument("--site", choices=["clean", "busy"], default="clean")
    args = ap.parse_args()
    SENSOR, GEOREF, SITE = args.sensor, args.georef, args.site
    print(f"Sensor model: {SENSOR}, georeference: {GEOREF}, site: {SITE}")
    CLASS_COLOR.update(site_clutter.CLUTTER_COLOR)
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    print("Extracting full-building element geometry (once)...")
    all_elements = load_all_elements(f, settings)
    print(f"{len(all_elements)} elements available for raycasting")

    all_verts_full = np.concatenate([e["verts"] for e in all_elements.values()], axis=0)
    full_center = all_verts_full.mean(axis=0)
    full_radius = float(np.linalg.norm(all_verts_full[:, :2] - full_center[:2], axis=1).max())
    base_z = float(all_verts_full[:, 2].min())
    clutter_rng = np.random.default_rng(SEED + 999)
    ground_trees = (*make_ground(full_center, full_radius, base_z, clutter_rng),
                    *make_trees(full_center, full_radius, base_z, clutter_rng))
    ground_trees = (ground_trees[0], ground_trees[1], ground_trees[2], ground_trees[3])

    site, dev = None, {}
    if SITE == "busy":
        site = site_clutter.Site(all_elements)
        dev = site_clutter.deviations(all_elements.keys())
        print(f"Busy site: {len(dev)} elements built off their designed position")

    stage_dirs = sorted(glob.glob(os.path.join(STATES_DIR, "*")))
    entries = []
    for stage_dir in stage_dirs:
        with open(os.path.join(stage_dir, "manifest.json"), encoding="utf-8") as fh:
            manifest = json.load(fh)
        i = manifest["stage_index"]
        rng = np.random.default_rng(SEED + 1000 + i)

        stage_elements = stage_elements_from_manifest(manifest, all_elements, rng)
        clutter, deviated = None, ()
        if site is not None:
            stage_elements = site_clutter.apply_deviations(stage_elements, dev)
            deviated = [e["global_id"] for e in stage_elements if e["global_id"] in dev]
            clutter = site_clutter.stage_clutter(site, stage_elements, i, manifest["phase"],
                                                 is_last=stage_dir == stage_dirs[-1])
        print(f"\nStage {i:02d} [{manifest['storey_milestone']} / {manifest['phase']}]: "
              f"{len(stage_elements)} elements ({manifest['n_fully_built']} full, "
              f"{manifest['n_partial']} partial), raycasting...")

        (all_pts, all_col, all_gt_class, all_gt_gid, n_structure, center, fr,
         T_true, cam_eyes, cam_kinds, cam_target) = run_stage(stage_elements, rng, ground_trees, clutter)
        n_full = all_pts.shape[0]

        stage_name = os.path.basename(stage_dir)
        save_stage_pointcloud(stage_name, all_pts, all_col, all_gt_class, all_gt_gid,
                              T_true, cam_eyes, cam_kinds, cam_target, deviated_ids=deviated)

        label = f"BIM: {i:02d} {storey_short(manifest['storey_milestone'])} - {manifest['phase']}"
        entry = pack_entry(label, all_pts, all_col, n_full, center, fr * 1.15)
        entry["stageIndex"] = i
        entries.append(entry)
        print(f"  packed: {entry['nPoints']} pts (structure {n_structure}), "
              f"saved data/pointclouds/{stage_name}.npz")

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
