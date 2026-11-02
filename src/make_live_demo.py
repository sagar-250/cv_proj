"""Live demo page: the whole pipeline, step by step, running in a browser.

Builds results/live_demo.html, one self-contained page (three.js from a CDN,
everything else inline) on the real Schependomlaan design model:
  1. the design model with its materials (procedural brick / block textures),
  2. the 16 construction stages from data/states/*/manifest.json, on a clean or
     busy site (site_clutter.py, rebuilt here with its own code and seeds) and
     with the simple or realistic sensor (p3_render.py realistic_sensor),
  3. a drone orbit + indoor walk-through scan by ray casting (first hits,
     camera colour from material x sun x shadow ray, noise, dropout),
  4. a random survey offset (any heading, +-30 m) where plain ICP gets stuck,
  5. the "plan" registration of src/registration.py, animated step by step,
  6. clutter removal: the colour filter of color_registration.py runs live;
     DART (dart_label.py, needs SAM3) is shown from its recorded run only,
  7. matching (p4_match_bim.py) and the visibility check (p4_visibility.py),
  8. scoring (p7_score.py) against the stage truth, with the R5 90% range,
     plus recorded runs: the committed full pipeline (data/status,
     results/main_table.csv), COLMAP (sfm_experiment.py) and the improvement ladder.

Things a browser cannot redo are only shown when their run data is in the repo
(recorded_runs); otherwise that part is left out.

The algorithms are re-written in plain JS in src/live_demo_core.js (no
dependencies; it also runs in Node, see src/live_demo_check.js). They are
browser-sized: 0.4 m floor-plan pixels and 2 deg heading steps instead of
0.2 m / 1 deg, a ~5x sparser scan, and the visibility check replays every 3rd
ray. Ground truth (the stage manifest, the true offset) is kept in a separate
eval object and only used for the "true error" and "truth" numbers, never by
registration, matching or visibility. src/live_demo_page.html is the page;
/*CORE*/ and /*DATA*/ are filled in here.

Run: .venv/bin/python src/make_live_demo.py
Output: results/live_demo.html (open in a browser; ~3.5 MB)
"""
import base64
import glob
import json
import os

import numpy as np
import pandas as pd

SRC = os.path.dirname(os.path.abspath(__file__))
OUT_HTML = "results/live_demo.html"
CATS = ["board_white", "brick_brown", "brick_yellow", "calcium_silicate", "concrete_cast",
        "concrete_prefab", "screed", "steel_dark", "steel_primer", "wood", "plywood",
        "insulation", "aerated_concrete", "zinc", "gypsum_block", "tile"]
TYPES = ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall"]


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()


def demo_data():
    """BIM mesh (mm-quantised uint16 vertices), per-element table, stage states, committed results."""
    mesh = np.load("data/bim/bim_meshes.npz", allow_pickle=True)
    verts, faces, tri_gid = mesh["verts"].astype(np.float64), mesh["faces"], mesh["tri_gid"]
    bim = pd.read_csv("data/bim/bim_complete.csv")
    cat_of = dict(pd.read_csv("data/bim/bim_materials.csv")[["global_id", "category"]].values)
    gids = bim["global_id"].tolist()
    idx_of = {g: i for i, g in enumerate(gids)}
    keep = np.array([g in idx_of for g in tri_gid])
    faces, tri_gid = faces[keep], tri_gid[keep]
    tri_elem = np.array([idx_of[g] for g in tri_gid], dtype=np.uint16)
    origin = verts.min(axis=0)
    q = np.round((verts - origin) * 1000).astype(np.int64)
    assert q.max() < 65535 and len(verts) < 65536
    storeys = sorted(bim["storey"].unique().tolist())
    elem = {
        "gid": gids,
        "type": [TYPES.index(t) if t in TYPES else 3 for t in bim["ifc_type"]],
        "storey": [storeys.index(s) for s in bim["storey"]],
        "mat": [CATS.index(cat_of.get(g, "concrete_cast")) for g in gids],
        "ext": [1 if str(v) == "True" else 0 for v in bim["is_external"]],
        "vol": [round(float(v), 4) for v in bim["volume"]],
        "zmin": [round(float(v), 3) for v in bim["aabb_min_z"]],
        "zmax": [round(float(v), 3) for v in bim["aabb_max_z"]],
    }
    stages, state = [], []
    for path in sorted(glob.glob("data/states/*/manifest.json")):
        m = json.load(open(path, encoding="utf-8"))
        s = np.zeros(len(gids), dtype=np.uint8)            # 0 absent, 255 built, 1..254 part-built fraction
        for g in m["kept_ids"]:
            s[idx_of[g]] = 255
        for p in m["partial_ids"]:
            s[idx_of[p["global_id"]]] = int(np.clip(round(p["completion_fraction"] * 254), 1, 254))
        stages.append(os.path.basename(os.path.dirname(path)))
        state.append(s)
    cols = ["stage", "progress_est_count", "progress_gt_count", "progress_est_volume", "progress_gt_volume",
            "registration_err_m", "n_uncertain", "n_completed", "n_partial", "n_not_started"]
    table = [{c: (r[c] if c == "stage" else round(float(r[c]), 4)) for c in cols}
             for _, r in pd.read_csv("results/main_table.csv").iterrows()]
    print(f"{len(gids)} elements, {len(faces)} triangles, {len(stages)} stages")
    return {"origin": origin.round(4).tolist(), "verts": b64(q.astype(np.uint16)), "n_verts": int(len(verts)),
            "faces": b64(faces.astype(np.uint16)), "n_faces": int(len(faces)), "tri_elem": b64(tri_elem),
            "cats": CATS, "types": TYPES, "storeys": storeys, "elem": elem,
            "stages": stages, "state": b64(np.stack(state)), "main_table": table}


INTERIOR = 0          # no machinery / temporary items on the floors (p3_render.py --interior)
LEFT_OUT = ("worker", "pile")   # workers, material piles and machinery are plain boxes that look like they float in the 3D view,
                                # so the demo keeps only scaffold, crane, fence and shoring props
STATUS_CODE = {"not_started": 0, "partial": 1, "completed": 2, "uncertain": 3}


def busy_site(gids, idx_of):
    """The busy site of p3_render.py --site busy, rebuilt with its own code and seeds:
    per stage the clutter meshes (scaffold, crane, fence, props, piles, workers, indoor machinery),
    the late / misplaced elements, and the as-built offsets. Element geometry comes from
    data/bim/bim_meshes.npz (the same IFC geometry p3 extracts)."""
    import sys
    sys.path.insert(0, SRC)
    import p3_render
    import site_clutter
    mesh = np.load("data/bim/bim_meshes.npz", allow_pickle=True)
    verts, faces, tri_gid = mesh["verts"].astype(np.float64), mesh["faces"], mesh["tri_gid"]
    ifc_type = dict(pd.read_csv("data/bim/bim_complete.csv")[["global_id", "ifc_type"]].values)
    order = np.argsort(tri_gid, kind="stable")
    bounds = np.searchsorted(tri_gid[order], gids + [chr(0x10FFFF)])
    all_el = {}
    for k, g in enumerate(gids):
        f = faces[order[bounds[k]:bounds[k + 1]]]
        if len(f) == 0:
            continue
        vi, inv = np.unique(f, return_inverse=True)
        v = verts[vi]
        all_el[g] = {"global_id": g, "ifc_type": "IfcWall" if ifc_type[g].startswith("IfcWall") else ifc_type[g],
                     "verts": v, "faces": inv.reshape(-1, 3), "z_min": float(v[:, 2].min()), "z_max": float(v[:, 2].max())}
    site = site_clutter.Site(all_el)
    dev = site_clutter.deviations(all_el.keys())
    late, moved = site_clutter.late_and_misplaced(all_el, dev)
    classes = sorted(p3_render.CLUTTER_RENDER_RGB)
    paths = sorted(glob.glob("data/states/*/manifest.json"))
    first_stage, stages, all_v, store, uniq = {}, [], [], {}, []
    for k, path in enumerate(paths):
        m = json.load(open(path, encoding="utf-8"))
        i = m["stage_index"]
        se = p3_render.stage_elements_from_manifest(m, all_el, np.random.default_rng(p3_render.SEED + 1000 + i))
        for e in se:
            first_stage.setdefault(e["global_id"], k)
        missing = [e["global_id"] for e in se
                   if e["global_id"] in late and k - first_stage[e["global_id"]] < site_clutter.LATE_STAGES]
        se = [e for e in se if e["global_id"] not in set(missing)]
        se = site_clutter.apply_deviations(se, {**dev, **moved})
        misplaced = [e["global_id"] for e in se if e["global_id"] in moved]
        items = site_clutter.stage_clutter(site, se, i, m["phase"], is_last=k == len(paths) - 1, interior=INTERIOR)
        flat = []
        for it in items:
            flat += it if isinstance(it, list) else [it]
        flat = [it for it in flat if it["ifc_type"] not in LEFT_OUT]
        ids = []
        for it in flat:   # each distinct item is stored once (fence, crane and scaffold repeat across stages)
            key = (it["ifc_type"], np.round(it["verts"], 3).tobytes(), it["faces"].tobytes())
            if key not in store:
                store[key] = len(store)
                uniq.append(it)
                all_v.append(it["verts"])
            ids.append(store[key])
        stages.append({"items": ids, "missing": sorted(idx_of[g] for g in missing),
                       "misplaced": sorted(idx_of[g] for g in misplaced)})
    origin = np.concatenate(all_v).min(axis=0) - 0.01
    items_out = []
    for it in uniq:
        if it["ifc_type"] == "props":   # thousands of vertical 6 cm bars (8 verts each): store (x, y, z0, z1)
            v = it["verts"].reshape(-1, 8, 3)
            assert len(it["verts"]) % 8 == 0 and len(it["faces"]) == 12 * len(v)
            poles = np.c_[v[:, :, :2].mean(axis=1), v[:, :, 2].min(axis=1), v[:, :, 2].max(axis=1)]
            q = np.round((poles - np.r_[origin[:2], origin[2], origin[2]]) * 1000)
            items_out.append({"c": classes.index("props"), "poles": b64(q.astype(np.uint16)), "t": 0.06})
            continue
        q = np.round((it["verts"] - origin) * 1000)
        assert q.max() < 65535
        wide = len(it["verts"]) >= 65536          # big items (scaffold) need 32-bit indices
        items_out.append({"c": classes.index(it["ifc_type"]), "v": b64(q.astype(np.uint16)), "w": 4 if wide else 2,
                          "f": b64(it["faces"].astype(np.uint32 if wide else np.uint16))})
    n_tri = [sum(len(uniq[i]["faces"]) for i in s["items"] if uniq[i]["ifc_type"] != "props") for s in stages]
    print(f"busy site: {len(dev)} off 4-12 cm, {len(moved)} misplaced, {len(late)} late, "
          f"{len(uniq)} distinct clutter items, triangles per stage {min(n_tri)}-{max(n_tri)}")
    return {"origin": origin.round(4).tolist(), "classes": classes,
            "rgb": {c: list(map(int, p3_render.CLUTTER_RENDER_RGB[c])) for c in classes},
            "offset": {str(idx_of[g]): [round(float(d[0]), 4), round(float(d[1]), 4)] for g, d in {**dev, **moved}.items()},
            "deviated": sorted(idx_of[g] for g in dev), "items": items_out, "stages": stages}


def recorded_runs(gids, idx_of):
    """Results of runs a browser cannot redo: the committed full-pipeline run (per-element status),
    the R5 calibration table, the COLMAP cloud of stage 07, DART pictures and numbers, the
    improvement ladder. Each part is skipped (with a note) when its files are missing."""
    rec = {}
    status = []
    for path in sorted(glob.glob("data/status/*/status.csv")):
        st = pd.read_csv(path)
        code = np.full(len(gids), 3, np.uint8)
        code[[idx_of[g] for g in st["global_id"]]] = [STATUS_CODE[s] for s in st["status"]]
        status.append(code)
    if len(status) == 16:
        rec["status"] = b64(np.stack(status))
    if os.path.exists("results/r5_calibration.csv"):
        rec["calib"] = pd.read_csv("results/r5_calibration.csv").round(6).to_dict("records")
    if os.path.exists("results/improvement_ladder.csv"):
        lad = pd.read_csv("results/improvement_ladder.csv")
        rec["ladder"] = [{"group": r.group, "step": r.step, "err": round(float(r.mean_error), 2)} for r in lad.itertuples()]
    # COLMAP (sfm_experiment.py), stage 07: cloud placed with the pipeline's own registration estimate
    st = "07_01_eerste_verdieping_interior"
    pc, reg = f"data/pointclouds_sfm/{st}.npz", f"data/matches_sfm/{st}/registration.json"
    if os.path.exists(pc) and os.path.exists(reg):
        z = np.load(pc, allow_pickle=True)
        T = np.array(json.load(open(reg))["T_cloud_to_bim"])
        sel = np.random.default_rng(0).choice(len(z["xyz"]), 90000, replace=False)
        xyz = z["xyz"][sel] @ T[:3, :3].T + T[:3, 3]
        o = xyz.min(axis=0)
        tabs = {}
        for tag in ["sfm", "sfm_ideal"]:
            row = pd.read_csv(f"results/experiments/main_table_{tag}.csv").set_index("stage").loc[st]
            tabs[tag] = {k: round(float(row[k]), 2) for k in ["progress_est_count", "progress_gt_count", "registration_err_m"] if k in row}
        rec["colmap"] = {"stage": st, "origin": o.round(4).tolist(), "n": len(sel),
                         "xyz": b64(np.round((xyz - o) * 1000).astype(np.uint16)), "rgb": b64(z["rgb"][sel].astype(np.uint8)),
                         "tables": tabs}
    else:
        print("no COLMAP run data, skipped")
    # DART (dart_label.py, needs SAM3): pictures + numbers only
    pics = sorted(glob.glob("results/dart/*_pose_[0-9]*.png"))
    if pics:
        from PIL import Image
        import io
        imgs = []
        for p in [pics[0], pics[len(pics) // 2]]:
            im = Image.open(p).convert("RGB")
            im.thumbnail((1400, 1400))
            buf = io.BytesIO()
            im.save(buf, "JPEG", quality=70)
            imgs.append(base64.b64encode(buf.getvalue()).decode())
        runs = {}
        for tag in ["interior", "interior_img", "interior_dartc", "interior_dart"]:
            f = f"results/experiments/main_table_{tag}.csv"
            if os.path.exists(f):
                r = pd.read_csv(f).iloc[0]
                runs[tag] = {"est": round(float(r.progress_est_count), 1), "gt": round(float(r.progress_gt_count), 1)}
        stats = {}
        for tag in ["interior_img", "interior_dartc", "interior_dart"]:
            f = f"results/dart/{tag}_stats.csv"
            if os.path.exists(f):
                stats[tag] = pd.read_csv(f).iloc[0].to_dict()
        rec["dart"] = {"stage": "11_03_derde_verdieping_structure", "images": imgs, "runs": runs, "stats": stats}
    else:
        print("no DART run data, skipped")
    return rec


def main():
    core = open(os.path.join(SRC, "live_demo_core.js"), encoding="utf-8").read()
    page = open(os.path.join(SRC, "live_demo_page.html"), encoding="utf-8").read()
    data = demo_data()
    gids = data["elem"]["gid"]
    idx_of = {g: i for i, g in enumerate(gids)}
    data["busy"] = busy_site(gids, idx_of)
    data["recorded"] = recorded_runs(gids, idx_of)
    for k, v in data.items():
        print(f"  {k:10s} {len(json.dumps(v)) // 1024:6d} KB")
    data = json.dumps(data, separators=(",", ":"))
    html = page.replace("/*CORE*/", core, 1).replace("/*DATA*/", data, 1)
    # the page is written as an artifact body; wrap it into a full document for a plain browser
    doc = ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
           '<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">'
           '</head><body style="margin:0">' + html + "</body></html>")
    os.makedirs(os.path.dirname(OUT_HTML), exist_ok=True)
    with open(OUT_HTML, "w", encoding="utf-8") as fh:
        fh.write(doc)
    print(f"wrote {OUT_HTML} ({len(doc) // 1024} KB)")


if __name__ == "__main__":
    main()
