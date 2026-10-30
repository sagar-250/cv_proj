"""Simulate biweekly progress-stage point clouds from the Schependomlaan
design BIM (SPEC.md Phase 2/3, Open3D instead of Blender for v1), with an
`easy` and a `realistic` config per the user's requested pipeline.

Why not uniform surface sampling: a cloud with points on every face (inside
and out, evenly dense) is something no drone/LiDAR could ever produce, and
any progress-comparison algorithm scores near-perfect on it -- proving
nothing. Instead we raycast from simulated camera/LiDAR viewpoints and keep
only first hits: an orbiting exterior flight (drone) plus an interior
walk-through per floor (so floor slabs and interior wall faces are actually
visible, not just the outer shell).

Build order (SPEC.md section 8, cross-checked against the real
`Planning/XML` schedule -- confirms bottom-up per-storey sequencing: slab
poured -> walls/blockwork -> facade brick veneer -> next storey's slab ->
... -> roof steel -> roof cladding):
  1. Sort by storey, bottom-up.
  2. Within a storey: columns -> beams -> slabs -> walls.
  3. Within a category: sort by (x, y) for a contiguous work front.
  4. `realistic` config: jitter 8% of adjacent pairs in that order (seeded)
     so the built/unbuilt boundary isn't a razor-sharp floor plane.

Note: the "Comparison As Planned As Built" spreadsheet has real per-week
missing-element GlobalIds, but they don't overlap this design IFC's
GlobalIds at all (same export mismatch Phase 0 found for the as-planned
IFCs) -- usable to sanity-check the *sequencing logic* qualitatively, not
for an exact per-element real-week ground truth here.

`easy` config: clean build order, no jitter, no partial elements, no
registration offset -- a sanity check that the code runs and produces
sensible geometry.

`realistic` config (what's reported on):
  - Order jitter (5-10% adjacent swaps).
  - Partial completion: elements newly appearing in a stage have a chance
    of being height-clipped (wall poured to half height, column cast but
    not full height) rather than fully present or fully absent.
  - Per-element pose jitter: a few cm translation + a fraction of a degree
    rotation, applied before raycasting (as-built deviates from as-designed).
  - Registration jitter: a small whole-scene rigid transform per stage
    (independent SfM reconstructions are never perfectly registered to the
    BIM frame) -- this is what actually exercises a registration step,
    instead of the circularity trap of identical geometry always aligning
    perfectly.
  - Gaussian position noise + random dropout (sensor realism).
  - Fixed ground + trees (same clutter every stage -- it's the same site).

Run: .venv/Scripts/python.exe src/p2_p3_simulate_stages.py [--config easy|realistic]
"""
import argparse
import base64
import json
import os

import numpy as np
import ifcopenshell
import ifcopenshell.geom
import open3d as o3d

IFC_PATH = "data/ifc/IFC Schependomlaan.ifc"
OUT_JS = "results/pointcloud_viewer_data.js"

N_STAGES = 8
CATEGORY_ORDER = {"IfcColumn": 0, "IfcBeam": 1, "IfcSlab": 2, "IfcWall": 3, "IfcWallStandardCase": 3}
CLASS_COLOR = {
    "IfcSlab": [176, 168, 152],
    "IfcWall": [150, 156, 163],
    "IfcWallStandardCase": [150, 156, 163],
    "IfcBeam": [120, 149, 173],
    "IfcColumn": [239, 131, 84],
}
GROUND_COLOR = [96, 88, 74]
TREE_COLOR = [76, 110, 68]

NOISE_SIGMA = 0.015
DROPOUT_FRAC = 0.15
EXT_GRID = (150, 100)
EXT_FOV_DEG = 70.0
N_ORBIT_ANGLES = 20
N_ALTITUDE_BANDS = 3
INTERIOR_GRID_N = (3, 3)        # viewpoints per floor, in a shrunk XY grid
INTERIOR_RAY_GRID = (110, 55)   # equirectangular ray fan per interior viewpoint
TARGET_POINTS_OUT = 130_000
SEED = 0

ORDER_JITTER_FRAC = 0.08
PARTIAL_CHANCE = 0.5
PARTIAL_HEIGHT_RANGE = (0.25, 0.85)
ELEMENT_JITTER_T = 0.02     # 2 cm
ELEMENT_JITTER_DEG = 0.3
REG_JITTER_T = 0.08         # 8 cm
REG_JITTER_DEG = 1.2


def load_elements(f, settings):
    elements = []
    for ifc_type in ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall", "IfcWallStandardCase"]:
        for el in f.by_type(ifc_type):
            try:
                shape = ifcopenshell.geom.create_shape(settings, el)
            except Exception:
                continue
            verts = np.array(shape.geometry.verts, dtype=np.float64).reshape(-1, 3)
            faces = np.array(shape.geometry.faces, dtype=np.int64).reshape(-1, 3)
            if verts.shape[0] == 0 or faces.shape[0] == 0:
                continue
            centroid = verts.mean(axis=0)
            elements.append({
                "global_id": el.GlobalId,
                "ifc_type": ifc_type,
                "verts": verts,
                "faces": faces,
                "z_min": float(verts[:, 2].min()),
                "z_max": float(verts[:, 2].max()),
                "cx": float(centroid[0]),
                "cy": float(centroid[1]),
            })
    return elements


def build_order(elements, rng, jitter_frac):
    ordered = sorted(elements, key=lambda e: (
        round(e["z_min"], 1),
        CATEGORY_ORDER[e["ifc_type"]],
        round(e["cx"], 1), round(e["cy"], 1),
    ))
    if jitter_frac > 0:
        n_swaps = int(len(ordered) * jitter_frac)
        for _ in range(n_swaps):
            i = rng.integers(0, len(ordered) - 1)
            ordered[i], ordered[i + 1] = ordered[i + 1], ordered[i]
    return ordered


def rotation_matrix(axis, deg):
    axis = axis / np.linalg.norm(axis)
    theta = np.radians(deg)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def jitter_element(e, rng, t_mag, deg_mag):
    """Small per-element rigid perturbation: as-built deviates from as-designed."""
    verts = e["verts"]
    centroid = verts.mean(axis=0)
    axis = rng.normal(size=3)
    R = rotation_matrix(axis, rng.uniform(-deg_mag, deg_mag))
    t = rng.normal(0, t_mag, 3)
    new_verts = (verts - centroid) @ R.T + centroid + t
    e2 = dict(e)
    e2["verts"] = new_verts
    return e2


def clip_partial_height(e, frac):
    """Clip a wall/column's mesh to the bottom `frac` of its height -- a
    partially built element (poured/erected but not finished).

    Clamps every vertex above the cutoff down to it. Walls and columns are
    vertical extrusions, so this yields exactly the shorter prism (side faces
    shortened, top cap moved down). The earlier version dropped every face
    with any vertex above the cutoff -- on an extruded wall that is every
    side face, leaving only the bottom cap hidden against the slab, so
    "partial" walls were effectively invisible to the raycaster (measured:
    ~100 partial elements per stage scored not_started)."""
    verts = e["verts"]
    z0, z1 = e["z_min"], e["z_max"]
    if z1 - z0 < 1e-6:
        return e
    cutoff = z0 + frac * (z1 - z0)
    new_verts = verts.copy()
    new_verts[:, 2] = np.minimum(new_verts[:, 2], cutoff)
    e2 = dict(e)
    e2["verts"] = new_verts
    e2["z_max"] = float(cutoff)
    return e2


def make_o3d_scene(elements):
    all_verts, all_faces, tri_class, tri_gid = [], [], [], []
    offset = 0
    for e in elements:
        all_verts.append(e["verts"])
        all_faces.append(e["faces"] + offset)
        tri_class.extend([e["ifc_type"]] * e["faces"].shape[0])
        tri_gid.extend([e["global_id"]] * e["faces"].shape[0])
        offset += e["verts"].shape[0]
    verts = np.concatenate(all_verts, axis=0).astype(np.float32)
    faces = np.concatenate(all_faces, axis=0).astype(np.uint32)

    mesh = o3d.t.geometry.TriangleMesh()
    mesh.vertex.positions = o3d.core.Tensor(verts)
    mesh.triangle.indices = o3d.core.Tensor(faces)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh)
    return scene, np.array(tri_class), np.array(tri_gid), verts


def camera_basis(eye, target, world_up=np.array([0, 0, 1.0])):
    forward = target - eye
    forward /= np.linalg.norm(forward)
    if abs(np.dot(forward, world_up)) > 0.999:
        world_up = np.array([1.0, 0, 0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return forward, right, up


def pinhole_rays(eye, target, fov_deg, grid_wh):
    forward, right, up = camera_basis(eye, target)
    w, h = grid_wh
    fov = np.radians(fov_deg)
    aspect = w / h
    xs = np.tan(fov / 2) * np.linspace(-1, 1, w)
    ys = np.tan(fov / 2 / aspect) * np.linspace(-1, 1, h)
    xx, yy = np.meshgrid(xs, ys)
    dirs = (forward[None, None, :] + xx[:, :, None] * right[None, None, :] + yy[:, :, None] * up[None, None, :])
    dirs = dirs.reshape(-1, 3)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    origins = np.tile(eye, (dirs.shape[0], 1))
    return origins.astype(np.float32), dirs.astype(np.float32)


def spherical_rays(eye, grid_wh, rng):
    """Omnidirectional ray fan from a single point -- like a LiDAR scan.

    Uses a Fibonacci-sphere lattice (near-uniform solid-angle coverage) with
    a small random rotation, not a regular (azimuth, elevation) grid: a
    regular grid hits any flat surface (floor, ceiling, wall) at a fixed set
    of discrete radii -- every elevation band traces its own ring on that
    surface, which shows up as visible concentric-circle clustering. The
    Fibonacci lattice has no periodic structure, so it doesn't alias like
    that; the per-viewpoint random rotation also decorrelates any residual
    pattern between viewpoints.
    """
    w, h = grid_wh
    n = w * h
    i = np.arange(n)
    golden = np.pi * (3.0 - np.sqrt(5.0))
    z = 1 - 2 * (i + 0.5) / n
    r = np.sqrt(np.clip(1 - z * z, 0, 1))
    theta = golden * i
    dirs = np.stack([r * np.cos(theta), r * np.sin(theta), z], axis=1).astype(np.float64)

    # random rotation per viewpoint so successive scans don't share the
    # exact same lattice orientation
    axis = rng.normal(size=3)
    R = rotation_matrix(axis, rng.uniform(0, 360))
    dirs = (dirs @ R.T).astype(np.float32)

    origins = np.tile(eye, (dirs.shape[0], 1)).astype(np.float32)
    return origins, dirs


def exterior_viewpoints(center, footprint_radius, height_range):
    viewpoints = []
    dist = footprint_radius * 1.7 + 3.0
    for band in range(N_ALTITUDE_BANDS):
        alt_frac = 0.35 + 0.5 * band / max(N_ALTITUDE_BANDS - 1, 1)
        z = height_range[0] + alt_frac * (height_range[1] - height_range[0]) + 2.0
        for i in range(N_ORBIT_ANGLES):
            theta = 2 * np.pi * i / N_ORBIT_ANGLES
            eye = center + np.array([dist * np.cos(theta), dist * np.sin(theta), z - center[2]])
            viewpoints.append(("pinhole", eye))
    return viewpoints


def interior_viewpoints(elements, footprint_center, footprint_radius):
    """One viewpoint grid per distinct storey level (from slab z_max values),
    positioned inside a shrunk footprint, each an omnidirectional scan."""
    slab_tops = sorted({round(e["z_max"], 1) for e in elements if e["ifc_type"] == "IfcSlab"})
    # merge levels closer than 1.5m (same floor's multiple slab pours)
    levels = []
    for z in slab_tops:
        if not levels or z - levels[-1] > 1.5:
            levels.append(z)

    inset = footprint_radius * 0.45
    gx, gy = INTERIOR_GRID_N
    viewpoints = []
    for z in levels:
        for i in range(gx):
            for j in range(gy):
                fx = (i + 0.5) / gx * 2 - 1
                fy = (j + 0.5) / gy * 2 - 1
                eye = np.array([
                    footprint_center[0] + fx * inset,
                    footprint_center[1] + fy * inset,
                    z + 1.4,
                ])
                viewpoints.append(("spherical", eye))
    return viewpoints


def raycast(scene, tri_class, tri_gid, viewpoints, center, rng, return_geometry=False):
    """First-hit raycast. With return_geometry, also returns per-hit range (m),
    |cos| of the incidence angle, the ray direction, and whether the hit came
    from a drone photo (pinhole) rather than an indoor scan -- what a range-
    and angle-dependent sensor model needs, plus the hit triangle's normal
    (as Open3D gives it, not flipped toward the camera) and the index of the
    viewpoint that saw it (for camera colour: sun shading, exposure)."""
    all_hits, all_classes, all_gids = [], [], []
    all_rng, all_cos, all_dir, all_photo, all_nrm, all_view = [], [], [], [], [], []
    for vi, (kind, eye) in enumerate(viewpoints):
        if kind == "pinhole":
            origins, dirs = pinhole_rays(eye, center, EXT_FOV_DEG, EXT_GRID)
        else:
            origins, dirs = spherical_rays(eye, INTERIOR_RAY_GRID, rng)
        rays = o3d.core.Tensor(np.concatenate([origins, dirs], axis=1))
        result = scene.cast_rays(rays)
        t_hit = result["t_hit"].numpy()
        prim_ids = result["primitive_ids"].numpy()
        valid = np.isfinite(t_hit)
        if not valid.any():
            continue
        hit_pts = origins[valid] + t_hit[valid, None] * dirs[valid]
        all_hits.append(hit_pts)
        all_classes.append(tri_class[prim_ids[valid]])
        all_gids.append(tri_gid[prim_ids[valid]])
        if return_geometry:
            normals = result["primitive_normals"].numpy()[valid]
            all_rng.append(t_hit[valid])
            all_cos.append(np.abs((normals * dirs[valid]).sum(axis=1)))
            all_dir.append(dirs[valid])
            all_photo.append(np.full(int(valid.sum()), kind == "pinhole"))
            all_nrm.append(normals)
            all_view.append(np.full(int(valid.sum()), vi))
    out = (np.concatenate(all_hits, axis=0), np.concatenate(all_classes, axis=0),
           np.concatenate(all_gids, axis=0))
    if return_geometry:
        out += (np.concatenate(all_rng), np.concatenate(all_cos), np.concatenate(all_dir),
                np.concatenate(all_photo), np.concatenate(all_nrm), np.concatenate(all_view))
    return out


def make_ground(center, footprint_radius, base_z, rng, n=9000):
    r = footprint_radius * 1.6
    xy = (rng.random((n, 2)) - 0.5) * 2 * r + center[:2]
    z = base_z - 0.05 + rng.normal(0, 0.03, n)
    pts = np.stack([xy[:, 0], xy[:, 1], z], axis=1)
    colors = np.tile(GROUND_COLOR, (n, 1)).astype(int)
    colors += rng.integers(-12, 12, (n, 3))
    return pts.astype(np.float32), np.clip(colors, 0, 255).astype(np.uint8)


def make_trees(center, footprint_radius, base_z, rng, n_trees=4, pts_per_tree=700):
    """Generated once with a dedicated rng and reused unchanged across all
    stages -- trees don't move between construction weeks."""
    all_pts, all_colors = [], []
    for _ in range(n_trees):
        angle = rng.random() * 2 * np.pi
        dist = footprint_radius * (1.05 + rng.random() * 0.5)
        tx, ty = center[0] + dist * np.cos(angle), center[1] + dist * np.sin(angle)
        trunk_h = 2.2 + rng.random() * 2.0
        canopy_r = 1.0 + rng.random() * 0.8
        canopy_h = 1.8 + rng.random() * 1.6
        canopy_center_z = base_z + trunk_h + canopy_h * 0.5
        local = rng.normal(0, 1, (pts_per_tree, 3))
        local /= np.linalg.norm(local, axis=1, keepdims=True) + 1e-9
        local *= rng.random((pts_per_tree, 1)) ** (1 / 3)
        pts = local * np.array([canopy_r, canopy_r, canopy_h * 0.5])
        pts += np.array([tx, ty, canopy_center_z])
        colors = np.tile(TREE_COLOR, (pts_per_tree, 1)).astype(int)
        colors += rng.integers(-18, 18, (pts_per_tree, 3))
        all_pts.append(pts)
        all_colors.append(np.clip(colors, 0, 255).astype(np.uint8))
    return np.concatenate(all_pts).astype(np.float32), np.concatenate(all_colors)


def apply_rigid_jitter(pts, rng, t_mag, deg_mag, return_transform=False):
    """Whole-scene rigid offset. With return_transform, also returns the 4x4
    T (BIM frame -> cloud frame) so registration error can be measured."""
    center = pts.mean(axis=0)
    axis = rng.normal(size=3)
    R = rotation_matrix(axis, rng.uniform(-deg_mag, deg_mag))
    t = rng.normal(0, t_mag, 3)
    out = (pts - center) @ R.T + center + t
    if not return_transform:
        return out
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = center - R @ center + t
    return out, T


def survey_offset(regime, rng, center):
    """Whole-scene 4x4 offset (BIM frame -> survey frame) of the size a real
    survey has before registration (ROADMAP R3):
      gnss : drone photogrammetry georeferenced by on-board GNSS + compass.
             ~N(0, 2 m) horizontal, ~N(0, 1 m) vertical, heading +/-15 deg,
             tilt ~N(0, 0.5 deg), scale ~N(1, 1%).
      local: indoor SLAM/LiDAR in its own start frame. Any heading, up to
             30 m horizontally / 5 m vertically, tilt ~N(0, 0.5 deg), metric scale.
    Rotation and scale act about `center`."""
    if regime == "gnss":
        t = np.array([rng.normal(0, 2), rng.normal(0, 2), rng.normal(0, 1)])
        yaw = rng.uniform(-15, 15)
        scale = rng.normal(1, 0.01)
    else:
        t = np.concatenate([rng.uniform(-30, 30, 2), rng.uniform(-5, 5, 1)])
        yaw = rng.uniform(-180, 180)
        scale = 1.0
    R = rotation_matrix(np.array([0, 0, 1.0]), yaw)
    tilt_axis = np.array([rng.normal(), rng.normal(), 0.0])
    R = rotation_matrix(tilt_axis, rng.normal(0, 0.5)) @ R
    T = np.eye(4)
    T[:3, :3] = scale * R
    T[:3, 3] = center - scale * R @ center + t
    return T


def pack_entry(label, pts, colors, n_full, focus_center, focus_radius):
    n = pts.shape[0]
    if n > TARGET_POINTS_OUT:
        idx = np.random.default_rng(SEED).choice(n, size=TARGET_POINTS_OUT, replace=False)
        pts, colors = pts[idx], colors[idx]

    pts_c = pts - focus_center
    extent = pts_c.max(axis=0) - pts_c.min(axis=0)
    radius = float(np.linalg.norm(extent) / 2)

    three_xyz = np.stack([pts_c[:, 0], pts_c[:, 2], pts_c[:, 1]], axis=1).astype(np.float32)

    # quantize to int16 (halves position storage vs float32) -- points already
    # carry cm-level sensor noise, so sub-mm quantization error is invisible,
    # and it roughly doubles how many points fit the artifact's size budget.
    scale = float(np.abs(three_xyz).max()) / 32000.0 if three_xyz.size else 1.0
    scale = max(scale, 1e-6)
    pos_i16 = np.clip(np.round(three_xyz / scale), -32000, 32000).astype(np.int16)
    pos_b64 = base64.b64encode(pos_i16.tobytes()).decode("ascii")
    col_b64 = base64.b64encode(colors.tobytes()).decode("ascii")

    return {
        "label": label,
        "nPoints": int(three_xyz.shape[0]),
        "nPointsFull": int(n_full),
        "nOutliersDropped": int(n_full - three_xyz.shape[0]),
        "extent": [float(v) for v in extent],
        "radius": radius,
        "coreRadius": focus_radius,   # house-focused framing, not full ground extent
        "posScale": scale,
        "posQuant": "i16",
        "pos": pos_b64,
        "col": col_b64,
    }


def run_stage(ordered, prev_n, cur_n, config, rng, ground_trees):
    stage_elements = [dict(e) for e in ordered[:cur_n]]
    new_from = prev_n

    if config == "realistic":
        for i in range(new_from, cur_n):
            e = stage_elements[i]
            if e["ifc_type"] in ("IfcWall", "IfcWallStandardCase", "IfcColumn") and rng.random() < PARTIAL_CHANCE:
                frac = rng.uniform(*PARTIAL_HEIGHT_RANGE)
                clipped = clip_partial_height(e, frac)
                stage_elements[i] = clipped
        stage_elements = [e for e in stage_elements if e is not None]
        stage_elements = [jitter_element(e, rng, ELEMENT_JITTER_T, ELEMENT_JITTER_DEG) for e in stage_elements]

    scene, tri_class, tri_gid, verts = make_o3d_scene(stage_elements)
    center = verts.mean(axis=0)
    footprint_radius = float(np.linalg.norm(verts[:, :2] - center[:2], axis=1).max())
    height_range = (float(verts[:, 2].min()), float(verts[:, 2].max()))

    viewpoints = exterior_viewpoints(center, footprint_radius, height_range)
    viewpoints += interior_viewpoints(stage_elements, center, footprint_radius)
    pts, classes, _gids = raycast(scene, tri_class, tri_gid, viewpoints, center, rng)

    if config == "realistic":
        pts = pts + rng.normal(0, NOISE_SIGMA, pts.shape)
        keep = rng.random(pts.shape[0]) > DROPOUT_FRAC
        pts, classes = pts[keep], classes[keep]

    colors = np.array([CLASS_COLOR[c] for c in classes], dtype=np.uint8)
    n_structure = pts.shape[0]

    ground_pts, ground_col, tree_pts, tree_col = ground_trees
    all_pts = np.concatenate([pts, ground_pts, tree_pts], axis=0)
    all_col = np.concatenate([colors, ground_col, tree_col], axis=0)

    if config == "realistic":
        all_pts = apply_rigid_jitter(all_pts, rng, REG_JITTER_T, REG_JITTER_DEG)

    return all_pts, all_col, n_structure, center, footprint_radius


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", choices=["easy", "realistic"], default="realistic")
    args = parser.parse_args()

    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    print(f"Config: {args.config}")
    print("Extracting per-element geometry...")
    elements = load_elements(f, settings)
    master_rng = np.random.default_rng(SEED)
    jitter_frac = ORDER_JITTER_FRAC if args.config == "realistic" else 0.0
    ordered = build_order(elements, master_rng, jitter_frac)
    print(f"{len(ordered)} in-scope elements, build order computed (jitter={jitter_frac}).")

    # whole-building reference frame + fixed clutter, generated once
    all_verts_full = np.concatenate([e["verts"] for e in ordered], axis=0)
    full_center = all_verts_full.mean(axis=0)
    full_radius = float(np.linalg.norm(all_verts_full[:, :2] - full_center[:2], axis=1).max())
    base_z = float(all_verts_full[:, 2].min())
    clutter_rng = np.random.default_rng(SEED + 999)
    ground_pts, ground_col = make_ground(full_center, full_radius, base_z, clutter_rng)
    tree_pts, tree_col = make_trees(full_center, full_radius, base_z, clutter_rng)
    ground_trees = (ground_pts, ground_col, tree_pts, tree_col)

    entries = []
    prev_n = 0
    for i in range(N_STAGES):
        frac = (i + 1) / N_STAGES
        cur_n = max(1, round(len(ordered) * frac))
        rng = np.random.default_rng(SEED + 1000 + i)
        print(f"\nStage {i+1}/{N_STAGES} ({frac*100:.0f}%): elements {prev_n}->{cur_n}, raycasting...")
        all_pts, all_col, n_structure, center, fr = run_stage(
            ordered, prev_n, cur_n, args.config, rng, ground_trees)
        n_full = all_pts.shape[0]

        suffix = "realistic" if args.config == "realistic" else "easy"
        label = f"BIM: Week {i+1} ({suffix})"
        # focus framing on the structure's own footprint, not the wider ground plane
        entry = pack_entry(label, all_pts, all_col, n_full, center, fr * 1.15)
        entries.append(entry)
        print(f"  packed {label}: {entry['nPoints']} pts (structure {n_structure}, clutter "
              f"{ground_pts.shape[0] + tree_pts.shape[0]})")
        prev_n = cur_n

    with open(OUT_JS, encoding="utf-8") as fh:
        txt = fh.read()
    payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))

    # drop any previous stage entries (old 4-stage or opposite-config runs)
    old_prefix = "BIM: Simulated Stage"
    payload["weeks"] = [w for w in payload["weeks"]
                         if not w["label"].startswith(old_prefix) and not w["label"].startswith("BIM: Week")]
    payload["weeks"] += entries

    with open(OUT_JS, "w", encoding="utf-8") as fh:
        fh.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, fh)
        fh.write(";\n")

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
