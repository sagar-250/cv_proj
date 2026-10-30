"""R1 -- a busier, more realistic construction site for the scan simulator.

The plain simulator scans a clean building on an empty lot. A real site has
things around and inside the building that are not in the BIM, and that block
the view or sit right next to the elements we are trying to recognise:

  scaffolding  pipe frame 0.3-1.0 m outside the facade, with plank decks every
               2 m, up to 1 m above the current top of the building. Removed at
               the last stage.
  crane        a tower crane next to the building: open lattice mast, jib over
               the building, counterweight. Removed at the last stage.
  fence        site fence around the lot: thin frame + wire mesh (mostly
               see-through, like real fence panels) on concrete feet.
  props        shoring props (thin poles, look like columns) under the newest
               floor while it is a "structure" stage.
  piles        pallets of blocks and stacks of timber/rebar, on the ground and
               on the built floors; new random ones every stage.
  workers      people-sized boxes on the ground and on the built floors.

It also makes the as-built geometry deviate from the design: a fixed 4% of
elements are built 4-12 cm off their designed position (persistent across
stages: a wall built in the wrong place stays there). The ids are stored in
the stage npz as eval-only `gt_deviated_ids`; these elements still count as
built in the ground truth.

Two coarser as-built errors (both eval-only, saved in the stage npz):
  late       3% of elements are missing for the first 2 stages after the
             schedule says they are built (`gt_missing_ids`), then turn up.
  misplaced  1.5% of walls/columns are built 0.6-1.5 m from their designed
             place and stay there (`gt_misplaced_ids`). They must be torn
             down and rebuilt, so the ground truth counts both as not built.

Clutter items are returned as pseudo-elements (same dict shape as P3's BIM
elements, global_id "" and ifc_type = the clutter class), so the raycaster
gives them occlusion, sensor noise and a gt_class label for free. The site
layout is built from the BIM's own footprint (simulator side; the pipeline
never sees any of this). All randomness is seeded.

Used by p3_render.py --site busy.
"""
import numpy as np
import pandas as pd
from scipy.ndimage import binary_closing, binary_fill_holes, distance_transform_edt

BIM_CSV = "data/bim/bim_complete.csv"
SEED = 0
CLUTTER_COLOR = {
    "scaffold": [214, 178, 40], "crane": [140, 80, 170], "fence": [110, 150, 110],
    "props": [190, 190, 70], "pile": [150, 105, 65], "worker": [230, 60, 60],
}
DEVIATION_FRAC = 0.04
DEVIATION_RANGE = (0.04, 0.12)   # m, horizontal
LATE_FRAC, LATE_STAGES = 0.03, 2
MISPLACED_FRAC = 0.015
MISPLACED_RANGE = (0.6, 1.5)      # m, horizontal
MISPLACED_TYPES = ("IfcWall", "IfcWallStandardCase", "IfcColumn")
RES = 0.25                        # footprint raster (m)
SCAFFOLD_IN, SCAFFOLD_OUT = 0.3, 1.0
SCAFFOLD_SPACING = 2.0
LIFT = 2.0
PIPE = 0.05
FENCE_MARGIN = 9.0
N_GROUND_PILES, N_FLOOR_PILES = 7, 4
N_GROUND_WORKERS, N_FLOOR_WORKERS = 5, 6


# ------------------------------------------------------------------ meshes

def box(c, size, yaw=0.0):
    """8 verts / 12 tris of a box centred at c, size (sx, sy, sz), turned yaw rad about z."""
    sx, sy, sz = np.asarray(size) / 2
    v = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    cy, sn = np.cos(yaw), np.sin(yaw)
    v = v @ np.array([[cy, -sn, 0], [sn, cy, 0], [0, 0, 1]]).T + np.asarray(c)
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return v, f


def bar(p, q, t=PIPE):
    """Square bar of thickness t from point p to point q."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    L = np.linalg.norm(d)
    if L < 1e-6:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    u = d / L
    a = np.cross(u, [0, 0, 1.0]) if abs(u[2]) < 0.9 else np.cross(u, [1.0, 0, 0])
    a /= np.linalg.norm(a)
    b = np.cross(u, a)
    v, f = box([0, 0, 0], [t, t, L])
    v = v[:, 0:1] * a + v[:, 1:2] * b + (v[:, 2:3] + L / 2) * u + p
    return v, f


def quad(corners, z, thick=0.04):
    """Flat deck between 4 xy corners (in order) at height z: top + bottom face."""
    c = np.asarray(corners, float)
    v = np.concatenate([np.c_[c, np.full(4, z)], np.c_[c, np.full(4, z - thick)]])
    f = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6]])
    return v, f


def item(cls, parts):
    vs, fs, off = [], [], 0
    for v, f in parts:
        if len(f) == 0:
            continue
        vs.append(v)
        fs.append(f + off)
        off += len(v)
    v = np.concatenate(vs)
    return {"global_id": "", "ifc_type": cls, "verts": v, "faces": np.concatenate(fs),
            "z_min": float(v[:, 2].min()), "z_max": float(v[:, 2].max())}


# ------------------------------------------------------------------ site layout (fixed)

class Site:
    """Footprint and fixed site furniture, built once from the full BIM."""

    def __init__(self, all_elements):
        bim = pd.read_csv(BIM_CSV)
        ext_walls = bim[(bim.ifc_type == "IfcWall") & (bim.is_external == True)  # noqa: E712
                        & bim.storey.isin(["00 begane grond", "01 eerste verdieping"])]
        floors = bim[(bim.ifc_type == "IfcSlab") & (bim.storey == "01 eerste verdieping")]
        boxes = pd.concat([ext_walls, floors])
        self.base_z = min(e["z_min"] for e in all_elements.values())
        self.origin = boxes[["aabb_min_x", "aabb_min_y"]].min().values - 4.0
        top = boxes[["aabb_max_x", "aabb_max_y"]].max().values + 4.0
        shape = tuple(np.ceil((top - self.origin) / RES).astype(int))
        m = np.zeros(shape, bool)
        for _, r in boxes.iterrows():
            i0, j0 = np.floor((np.array([r.aabb_min_x, r.aabb_min_y]) - self.origin) / RES).astype(int)
            i1, j1 = np.ceil((np.array([r.aabb_max_x, r.aabb_max_y]) - self.origin) / RES).astype(int)
            m[i0:i1 + 1, j0:j1 + 1] = True
        self.mask = binary_fill_holes(binary_closing(m, iterations=4))
        self.dist_out = distance_transform_edt(~self.mask) * RES   # m outside the footprint
        gy, gx = np.gradient(self.dist_out)
        self.grad = np.stack([gy, gx], axis=-1)                    # (i, j) = (x, y) order
        self.lo = self.origin + 4.0
        self.hi = top - 4.0
        self.scaffold_base = self._scaffold_standards()
        self.fence = self._fence()

    def outside(self, xy, margin):
        """True where xy is at least margin m outside the footprint."""
        ij = np.floor((np.atleast_2d(xy) - self.origin) / RES).astype(int)
        ok = (ij >= 0).all(axis=1) & (ij[:, 0] < self.mask.shape[0]) & (ij[:, 1] < self.mask.shape[1])
        out = np.ones(len(ij), bool)
        out[ok] = self.dist_out[ij[ok, 0], ij[ok, 1]] >= margin
        return out

    def _scaffold_standards(self):
        """Inner/outer standard positions in a ring around the footprint, ~2 m apart, with neighbours."""
        ring = np.argwhere(np.abs(self.dist_out - SCAFFOLD_IN) < RES * 0.75)
        pts = self.origin + (ring + 0.5) * RES
        # greedy even spacing along the ring
        keep = []
        for k in np.random.default_rng(SEED).permutation(len(pts)):
            if all(np.linalg.norm(pts[k] - pts[j]) > SCAFFOLD_SPACING * 0.9 for j in keep):
                keep.append(k)
        inner = pts[keep]
        g = self.grad[ring[keep, 0], ring[keep, 1]]
        g /= np.linalg.norm(g, axis=1, keepdims=True) + 1e-9
        outer = inner + g * (SCAFFOLD_OUT - SCAFFOLD_IN)
        pairs = set()
        for i in range(len(inner)):
            d = np.linalg.norm(inner - inner[i], axis=1)
            for j in np.argsort(d)[1:3]:
                if d[j] < SCAFFOLD_SPACING * 1.6:
                    pairs.add((min(i, j), max(i, j)))
        return inner, outer, sorted(pairs)

    def _fence(self):
        lo, hi = self.lo - FENCE_MARGIN, self.hi + FENCE_MARGIN
        z0 = self.base_z
        corners = [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]
        parts = []
        for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
            L = np.hypot(bx - ax, by - ay)
            n = int(np.ceil(L / 3.5))
            for k in range(n):
                if (ax, ay) == corners[0] and k == n // 2:
                    continue  # gate opening
                p = np.array([ax, ay]) + (np.array([bx - ax, by - ay]) * k / n)
                q = np.array([ax, ay]) + (np.array([bx - ax, by - ay]) * (k + 1) / n)
                for s in (p, q):
                    parts.append(box([s[0], s[1], z0 + 0.075], [0.6, 0.2, 0.15]))
                    parts.append(bar([*s, z0], [*s, z0 + 2.0], 0.04))
                parts.append(bar([*p, z0 + 2.0], [*q, z0 + 2.0], 0.04))
                parts.append(bar([*p, z0 + 0.15], [*q, z0 + 0.15], 0.04))
                for w in np.linspace(0, 1, int(np.linalg.norm(q - p) / 0.25))[1:-1]:
                    s = p + (q - p) * w
                    parts.append(bar([*s, z0 + 0.15], [*s, z0 + 2.0], 0.01))
        return item("fence", parts)

    def crane(self, top_z):
        x = self.hi[0] + 5.0
        y = (self.lo[1] + self.hi[1]) / 2
        z0, h = self.base_z, top_z + 7.0
        parts, w = [], 0.8
        legs = [(x - w, y - w), (x + w, y - w), (x + w, y + w), (x - w, y + w)]
        for lx, ly in legs:
            parts.append(bar([lx, ly, z0], [lx, ly, z0 + h], 0.14))
        for z in np.arange(z0 + 3.0, z0 + h, 3.0):
            for a, b in zip(legs, legs[1:] + legs[:1]):
                parts.append(bar([*a, z], [*b, z], 0.08))
        parts.append(box([x, y, z0 + 0.5], [4.0, 4.0, 1.0]))            # foundation block
        jib_lo, jib_hi = x - 30.0, x + 9.0
        for dy in (-0.6, 0.6):
            parts.append(bar([jib_lo, y + dy, z0 + h], [jib_hi, y + dy, z0 + h], 0.12))
        parts.append(bar([jib_lo, y, z0 + h + 1.2], [jib_hi, y, z0 + h + 1.2], 0.1))
        for xx in np.arange(jib_lo, jib_hi, 3.0):
            parts.append(bar([xx, y - 0.6, z0 + h], [xx, y, z0 + h + 1.2], 0.06))
            parts.append(bar([xx, y + 0.6, z0 + h], [xx, y, z0 + h + 1.2], 0.06))
        parts.append(box([x + 7.5, y, z0 + h - 1.0], [2.5, 2.2, 2.0]))  # counterweight
        parts.append(box([x, y + 1.6, z0 + h - 1.5], [1.6, 1.6, 2.0]))  # cab
        hook_x = x - 14.0
        parts.append(bar([hook_x, y, z0 + h], [hook_x, y, top_z + 3.0], 0.03))
        return item("crane", parts)

    def scaffold(self, top_z):
        inner, outer, pairs = self.scaffold_base
        z0 = self.base_z
        lifts = np.arange(z0 + LIFT, top_z + 1.0 + 1e-6, LIFT)
        if len(lifts) == 0:
            return None
        zt = lifts[-1] + 1.0
        parts = []
        for a, b in zip(inner, outer):
            parts.append(bar([*a, z0], [*a, zt]))
            parts.append(bar([*b, z0], [*b, zt]))
            for z in lifts:
                parts.append(bar([*a, z], [*b, z]))
        for i, j in pairs:
            for z in np.concatenate([lifts, lifts + 1.0]):   # ledgers + guard rail
                parts.append(bar([*outer[i], z], [*outer[j], z]))
            for z in lifts:
                parts.append(bar([*inner[i], z], [*inner[j], z]))
                parts.append(quad([inner[i], inner[j], outer[j], outer[i]], z + 0.06))
        return item("scaffold", parts)


# ------------------------------------------------------------------ per stage

def deviations(all_gids):
    """Persistent as-built position errors: gid -> (dx, dy, 0)."""
    gids = sorted(all_gids)
    rng = np.random.default_rng([SEED, 13])
    pick = rng.random(len(gids)) < DEVIATION_FRAC
    mag = rng.uniform(*DEVIATION_RANGE, len(gids))
    ang = rng.uniform(0, 2 * np.pi, len(gids))
    return {g: np.array([m * np.cos(a), m * np.sin(a), 0.0])
            for g, p, m, a in zip(gids, pick, mag, ang) if p}


def late_and_misplaced(all_elements, dev):
    """Coarse as-built errors: (set of late gids, gid -> (dx, dy, 0) for misplaced ones)."""
    gids = sorted(all_elements)
    rng = np.random.default_rng([SEED, 17])
    late = {g for g, r in zip(gids, rng.random(len(gids))) if r < LATE_FRAC}
    cand = [g for g in gids if all_elements[g]["ifc_type"] in MISPLACED_TYPES
            and g not in dev and g not in late]
    rng = np.random.default_rng([SEED, 19])
    pick = rng.random(len(cand)) < MISPLACED_FRAC
    mag = rng.uniform(*MISPLACED_RANGE, len(cand))
    ang = rng.uniform(0, 2 * np.pi, len(cand))
    moved = {g: np.array([m * np.cos(a), m * np.sin(a), 0.0])
             for g, p, m, a in zip(cand, pick, mag, ang) if p}
    return late, moved


def apply_deviations(elements, dev):
    out = []
    for e in elements:
        d = dev.get(e["global_id"])
        if d is not None:
            e = dict(e)
            e["verts"] = e["verts"] + d
        out.append(e)
    return out


def _floor_spots(slabs, rng, n, clear):
    """n random spots on top of built slabs (xy inside a slab, z = its top)."""
    spots = []
    if not slabs:
        return spots
    for _ in range(n * 20):
        if len(spots) == n:
            break
        s = slabs[rng.integers(len(slabs))]
        lo, hi = s["verts"].min(axis=0), s["verts"].max(axis=0)
        if (hi[:2] - lo[:2]).min() < 2.0:
            continue
        xy = rng.uniform(lo[:2] + 0.6, hi[:2] - 0.6)
        if all(np.linalg.norm(xy - c[:2]) > clear for c in spots):
            spots.append(np.array([xy[0], xy[1], hi[2]]))
    return spots


def _ground_spots(site, rng, n, near=(2.0, 7.0)):
    spots = []
    for _ in range(n * 50):
        if len(spots) == n:
            break
        xy = rng.uniform(site.lo - FENCE_MARGIN + 1, site.hi + FENCE_MARGIN - 1)
        ij = np.floor((xy - site.origin) / RES).astype(int)
        inside = (ij >= 0).all() and ij[0] < site.mask.shape[0] and ij[1] < site.mask.shape[1]
        d = site.dist_out[ij[0], ij[1]] if inside else 99.0
        if near[0] < d < near[1] and all(np.linalg.norm(xy - s[:2]) > 2.0 for s in spots):
            spots.append(np.array([xy[0], xy[1], site.base_z]))
    return spots


def pile(c, rng):
    kind = rng.choice(["pallets", "timber", "rebar"])
    yaw = rng.uniform(0, np.pi)
    if kind == "pallets":
        n = rng.integers(1, 4)
        return item("pile", [box(c + [k * 1.3 * np.cos(yaw), k * 1.3 * np.sin(yaw), 0.5], [1.2, 1.0, 1.0], yaw)
                             for k in range(n)])
    L = rng.uniform(3.0, 6.0)
    hgt = 0.4 if kind == "timber" else 0.25
    return item("pile", [box(c + [0, 0, hgt / 2 + 0.1], [L, 1.0, hgt], yaw),
                         box(c + [0, 0, 0.05], [0.15, 1.0, 0.1], yaw)])


def worker(c, rng):
    yaw = rng.uniform(0, np.pi)
    return item("worker", [box(c + [0, 0, 0.75], [0.45, 0.28, 1.5], yaw),
                           box(c + [0, 0, 1.62], [0.22, 0.22, 0.24], yaw)])


def props(slabs, rng):
    """Shoring poles under the newest floor (structure stages). A "floor" is a
    slab level with real area; small sloped roof bits don't count."""
    area = {}
    for sl in slabs:
        lo, hi = sl["verts"].min(axis=0), sl["verts"].max(axis=0)
        z = round(sl["z_max"] / 0.3) * 0.3
        area[z] = area.get(z, 0.0) + (hi[0] - lo[0]) * (hi[1] - lo[1])
    floors = sorted(z for z, a in area.items() if a >= 30.0)
    if len(floors) < 2:
        return None
    top, below = floors[-1], max(z for z in floors if z < floors[-1] - 1.5)
    parts = []
    for sl in slabs:
        if abs(round(sl["z_max"] / 0.3) * 0.3 - top) > 1e-6:
            continue
        lo, hi = sl["verts"].min(axis=0), sl["verts"].max(axis=0)
        for x in np.arange(lo[0] + 0.75, hi[0] - 0.5, 1.5):
            for y in np.arange(lo[1] + 0.75, hi[1] - 0.5, 1.5):
                if rng.random() < 0.7:
                    parts.append(bar([x, y, below], [x, y, sl["z_min"]], 0.06))
    return item("props", parts) if parts else None


def stage_clutter(site, stage_elements, stage_index, phase, is_last):
    """All clutter items for one stage (list of pseudo-elements)."""
    rng = np.random.default_rng([SEED, 11, stage_index])
    top_z = max(e["z_max"] for e in stage_elements)
    slabs = [e for e in stage_elements if e["ifc_type"] == "IfcSlab"]
    items = [site.fence]
    if not is_last:
        items.append(site.crane(max(top_z, site.base_z + 14.0)))
        if top_z - site.base_z > 2.0:
            items.append(site.scaffold(top_z))
    if phase == "structure":
        items.append(props(slabs, rng))
    upper = [s for s in slabs if s["z_max"] > site.base_z + 1.0] or slabs
    for c in _ground_spots(site, rng, N_GROUND_PILES):
        items.append(pile(c, rng))
    for c in _floor_spots(upper, rng, N_FLOOR_PILES, 2.0):
        items.append(pile(c, rng))
    for c in _ground_spots(site, rng, N_GROUND_WORKERS, near=(1.5, 5.0)):
        items.append(worker(c, rng))
    for c in _floor_spots(upper, rng, N_FLOOR_WORKERS, 1.5):
        items.append(worker(c, rng))
    return [it for it in items if it is not None]


def drop_blocking(items, eyes, pad=0.5):
    """Drop small clutter items that would swallow an interior scanner position."""
    keep = []
    for it in items:
        lo, hi = it["verts"].min(axis=0) - pad, it["verts"].max(axis=0) + pad
        small = it["ifc_type"] in ("pile", "worker")
        if small and np.any(np.all((eyes >= lo) & (eyes <= hi), axis=1)):
            continue
        keep.append(it)
    return keep
