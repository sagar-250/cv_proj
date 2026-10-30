"""Extract triangulated mesh geometry from the Schependomlaan design IFC for
the in-scope structural classes (SPEC.md phase 1 scope) and pack it as a
mesh entry into results/pointcloud_viewer_data.js, so it renders alongside
the point clouds in the same viewer artifact.

One merged BufferGeometry per class (not per element) to keep draw calls low
-- ~1700 elements would otherwise mean ~1700 draw calls.

Run: .venv/Scripts/python.exe src/export_bim_viewer_data.py
"""
import base64
import json
import os

import numpy as np
import ifcopenshell
import ifcopenshell.geom

IFC_PATH = "data/ifc/IFC Schependomlaan.ifc"
OUT_JS = "results/pointcloud_viewer_data.js"

# class -> (display color hex-as-rgb, label)
CLASSES = {
    "IfcSlab": ([176, 168, 152], "Slabs"),
    "IfcWall": ([150, 156, 163], "Walls"),
    "IfcWallStandardCase": ([150, 156, 163], "Walls"),
    "IfcBeam": ([120, 149, 173], "Beams"),
    "IfcColumn": ([239, 131, 84], "Columns"),
}


def extract_class_mesh(f, settings, ifc_types):
    all_verts = []
    all_faces = []
    vert_offset = 0
    n_elements = 0
    for ifc_type in ifc_types:
        for el in f.by_type(ifc_type):
            try:
                shape = ifcopenshell.geom.create_shape(settings, el)
            except Exception:
                continue
            verts = np.array(shape.geometry.verts, dtype=np.float32).reshape(-1, 3)
            faces = np.array(shape.geometry.faces, dtype=np.uint32).reshape(-1, 3)
            all_verts.append(verts)
            all_faces.append(faces + vert_offset)
            vert_offset += verts.shape[0]
            n_elements += 1
    if not all_verts:
        return None, None, 0
    return np.concatenate(all_verts, axis=0), np.concatenate(all_faces, axis=0), n_elements


def main():
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    # merge synonyms (IfcWall/IfcWallStandardCase) into one visual group by label
    groups = {}
    for ifc_type, (color, label) in CLASSES.items():
        groups.setdefault(label, {"types": [], "color": color})
        groups[label]["types"].append(ifc_type)

    # first pass: gather all verts across all groups to compute a shared center
    group_data = {}
    all_pts_for_center = []
    for label, g in groups.items():
        verts, faces, n_el = extract_class_mesh(f, settings, g["types"])
        if verts is None:
            print(f"{label}: no geometry")
            continue
        group_data[label] = {"verts": verts, "faces": faces, "color": g["color"], "n_elements": n_el}
        all_pts_for_center.append(verts)
        print(f"{label}: {n_el} elements, {verts.shape[0]} verts, {faces.shape[0]} tris")

    all_pts = np.concatenate(all_pts_for_center, axis=0)
    center = all_pts.mean(axis=0)
    extent = all_pts.max(axis=0) - all_pts.min(axis=0)
    radius = float(np.linalg.norm(extent) / 2)
    print(f"\nOverall: {all_pts.shape[0]} verts, extent(x,y,z)={extent}, radius={radius:.2f}")

    parts = []
    for label, gd in group_data.items():
        verts_c = gd["verts"] - center
        # remap ifc(x,y,z) -> three(x,y,z) = ifc(x,z,y)  [Z-up -> Y-up], same
        # convention as the point cloud exporters. This BIM's own axes are
        # the authoritative IFC world frame (not photogrammetry output), so
        # no sign flip is needed here.
        three_verts = np.stack([verts_c[:, 0], verts_c[:, 2], verts_c[:, 1]], axis=1).astype(np.float32)
        pos_b64 = base64.b64encode(three_verts.tobytes()).decode("ascii")
        idx_b64 = base64.b64encode(gd["faces"].astype(np.uint32).tobytes()).decode("ascii")
        parts.append({
            "label": label,
            "color": gd["color"],
            "nElements": gd["n_elements"],
            "nVerts": int(three_verts.shape[0]),
            "nTris": int(gd["faces"].shape[0]),
            "pos": pos_b64,
            "idx": idx_b64,
        })
        print(f"  packed {label}: pos {len(pos_b64)/1e6:.2f}MB, idx {len(idx_b64)/1e6:.2f}MB")

    core_lo = np.percentile(all_pts - center, 5, axis=0)
    core_hi = np.percentile(all_pts - center, 95, axis=0)
    # remap core box to three axes too for the radius calc
    core_extent_three = np.array([core_hi[0]-core_lo[0], core_hi[2]-core_lo[2], core_hi[1]-core_lo[1]])
    core_radius = float(np.linalg.norm(core_extent_three) / 2)

    entry = {
        "label": "BIM: Schependomlaan (design IFC)",
        "type": "mesh",
        "nElements": sum(p["nElements"] for p in parts),
        "radius": radius,
        "coreRadius": core_radius,
        "parts": parts,
    }

    if os.path.exists(OUT_JS):
        with open(OUT_JS, encoding="utf-8") as fh:
            txt = fh.read()
        payload = json.loads(txt[len("window.POINTCLOUD_DATA = "):].rstrip("\n;"))
    else:
        payload = {"weeks": []}

    payload["weeks"] = [w for w in payload["weeks"] if w.get("label") != entry["label"]] + [entry]

    with open(OUT_JS, "w", encoding="utf-8") as fh:
        fh.write("window.POINTCLOUD_DATA = ")
        json.dump(payload, fh)
        fh.write(";\n")

    print(f"\nUpdated {OUT_JS} ({os.path.getsize(OUT_JS)/1e6:.2f} MB)")


if __name__ == "__main__":
    main()
