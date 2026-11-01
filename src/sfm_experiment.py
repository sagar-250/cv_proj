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

Run: .venv/bin/python src/sfm_experiment.py --stage 15_04_dak_envelope [--step all]
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


def render(stage):
    out = os.path.join(SFM_DIR, stage, "images")
    os.makedirs(out, exist_ok=True)
    scene, tri_class, tri_gid, center, radius, height_range, _ = stage_scene(stage)
    views = camera_plan(center, radius, height_range)
    rng = np.random.default_rng([SEED, 43])
    sky = np.array([176, 196, 214], float)
    poses = []
    t0 = time.time()
    for i, (eye, target) in enumerate(views):
        origins, dirs = sim.pinhole_rays(eye, target, FOV, (IMG_W, IMG_H))
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
    json.dump({"fov": FOV, "size": [IMG_W, IMG_H], "poses": poses},
              open(os.path.join(SFM_DIR, stage, "true_poses.json"), "w"))   # eval only
    print(f"rendered {len(views)} photos -> {out}")


def colmap(stage, dense=True):
    import pycolmap
    wd = os.path.join(SFM_DIR, stage)
    images, db, sparse = os.path.join(wd, "images"), os.path.join(wd, "db.db"), os.path.join(wd, "sparse")
    if os.path.exists(db):
        os.remove(db)
    os.makedirs(sparse, exist_ok=True)
    t0 = time.time()
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"
    pycolmap.extract_features(db, images, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader)
    print(f"features {time.time() - t0:.0f}s", flush=True)
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
    wd = os.path.join(SFM_DIR, stage)
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
    truth = json.load(open(os.path.join(SFM_DIR, stage, "true_poses.json")))
    rng = np.random.default_rng([SEED, 53])
    pts = []
    for pose in truth["poses"]:
        eye, target = np.array(pose["eye"]), np.array(pose["target"])
        origins, dirs = sim.pinhole_rays(eye, target, FOV, (IMG_W, IMG_H))
        sel = np.arange(IMG_W * IMG_H).reshape(IMG_H, IMG_W)[::stride, ::stride].ravel()
        res = scene.cast_rays(o3d.core.Tensor(np.concatenate([origins[sel], dirs[sel]], axis=1)))
        t = res["t_hit"].numpy()
        ok = np.isfinite(t)
        ok[ok] = tri_gid[res["primitive_ids"].numpy()[ok]] != ""                        # skip ground and sky
        pts.append(origins[sel][ok] + t[ok, None] * dirs[sel][ok])
    xyz = np.concatenate(pts) + rng.normal(0, 0.005, (sum(len(q) for q in pts), 3))
    O = sim.survey_offset("gnss", np.random.default_rng([SEED, 7, len(xyz)]), center)
    xyz_s = xyz @ O[:3, :3].T + O[:3, 3]
    eyes = np.array([q["eye"] for q in truth["poses"]]) @ O[:3, :3].T + O[:3, 3]
    out = OUT_CLOUD_DIR + "_ideal"
    os.makedirs(out, exist_ok=True)
    np.savez_compressed(os.path.join(out, f"{stage}.npz"), xyz=xyz_s.astype(np.float32),
                        rgb=np.full((len(xyz_s), 3), 150, np.uint8), rgb_source=np.array("camera"),
                        gt_T_bim_to_cloud=O, cam_eyes=eyes, cam_kinds=np.array(["pinhole"] * len(eyes)),
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
    wd = os.path.join(SFM_DIR, stage)
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
    O = sim.survey_offset("gnss", np.random.default_rng([SEED, 7, len(xyz)]), center)
    gps = true_eye @ O[:3, :3].T + O[:3, 3]
    gps = gps + rng.normal(0, [GPS_SIGMA, GPS_SIGMA, 1.5 * GPS_SIGMA], gps.shape)
    A = umeyama(centres, gps)                                   # COLMAP frame -> survey frame
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
    A_true = umeyama(centres, true_eye)
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
    for pose in truth["poses"][::4]:
        eye = np.array(pose["eye"])
        d = samp - eye
        n = np.linalg.norm(d, axis=1)
        rays = np.concatenate([np.broadcast_to(eye, samp.shape), d / n[:, None]], axis=1).astype(np.float32)
        t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        vis |= np.abs(t - n) < 0.15
    ours_dist_true = cKDTree(xyz_t).query(samp)[0]
    metrics = {
        "median_dist_true_fit_m": float(np.median(dt[nt])),
        "share_within_5cm_true_fit": float((dt[nt] < 0.05).mean()),
        "share_within_10cm_true_fit": float((dt[nt] < 0.10).mean()),
        "completeness_within_10cm_true_fit": float((ours_dist_true[vis] < 0.10).mean()),
        "stage": stage, "cloud": kind, "n_points": int(len(xyz)), "n_images": len(names),
        "n_images_total": len(truth["poses"]),
        "camera_fit_rms_to_gnss_m": float(np.sqrt(((centres @ A[:3, :3].T + A[:3, 3] - gps) ** 2).sum(1).mean())),
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
    json.dump(metrics, open(os.path.join(METRICS_DIR, f"{stage}_metrics.json"), "w"), indent=1)
    print(json.dumps(metrics, indent=1))

    os.makedirs(OUT_CLOUD_DIR, exist_ok=True)
    np.savez_compressed(
        os.path.join(OUT_CLOUD_DIR, f"{stage}.npz"),
        xyz=xyz_s.astype(np.float32), rgb=rgb, rgb_source=np.array("camera"),
        # eval only: BIM frame -> this cloud. The cloud sits in the frame fixed by the GNSS fit (A),
        # and the cloud's true BIM position comes from the fit to the true cameras (A_true)
        gt_T_bim_to_cloud=A @ np.linalg.inv(A_true),
        cam_eyes=cam_s, cam_kinds=np.array(["pinhole"] * len(cam_s)),
        cam_target=O[:3, :3] @ center + O[:3, 3])
    print(f"wrote {OUT_CLOUD_DIR}/{stage}.npz ({kind} cloud, {len(xyz_s)} points)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True)
    ap.add_argument("--step", choices=["render", "colmap", "dense", "cloud", "ideal", "all"], default="all")
    ap.add_argument("--no-dense", action="store_true", help="sparse model only (no GPU needed)")
    args = ap.parse_args()
    if args.step in ("render", "all"):
        render(args.stage)
    if args.step in ("colmap", "all"):
        colmap(args.stage, dense=not args.no_dense)
    if args.step == "ideal":
        ideal(args.stage)
    if args.step == "dense":
        make_dense(args.stage)
    if args.step in ("cloud", "all"):
        cloud(args.stage)


if __name__ == "__main__":
    main()
