"""Phase 1 -- parse the design BIM into the canonical ground-truth table.

Per SPEC.md section 7: one row per in-scope structural element, with
centroid, AABB, a Z-aligned OBB (PCA in the XY plane -- an AABB alone is a
poor descriptor for a wall at 45 degrees), volume, and enough metadata to
drive a realistic build-order simulation later (storey via the official
spatial-containment relation, not the file's Elevation attribute -- that
attribute is in millimetres while this file's geometry is in metres, a unit
mismatch confirmed by inspection; IsExternal, needed to sequence envelope
walls before interior partitions).

Run: .venv/Scripts/python.exe src/p1_parse.py
Output: data/bim/bim_complete.csv, results/p1_centroids.png
"""
import numpy as np
import pandas as pd
import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.element as Element
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

IFC_PATH = "data/ifc/IFC Schependomlaan.ifc"
OUT_CSV = "data/bim/bim_complete.csv"
OUT_PLOT = "results/p1_centroids.png"

# NOTE: in IFC2X3, IfcWallStandardCase is a subtype of IfcWall, and
# ifcopenshell's by_type() already returns subtypes -- by_type("IfcWall")
# alone returns all 934 walls, which already includes all 282
# IfcWallStandardCase instances (verified: 100% overlap). Listing both here
# would double-count those 282.
IN_SCOPE_TYPES = ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall"]

CLASS_COLOR = {
    "IfcColumn": "#ef8354", "IfcBeam": "#7895ad", "IfcSlab": "#b0a898", "IfcWall": "#969ca3",
}


def get_is_external(el):
    psets = Element.get_psets(el)
    for props in psets.values():
        if "IsExternal" in props:
            return bool(props["IsExternal"])
    return None


def obb_xy(verts_xy):
    """PCA-based oriented box in the XY plane; Z stays world-aligned."""
    mean = verts_xy.mean(axis=0)
    centered = verts_xy - mean
    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    order = np.argsort(eigvals)[::-1]
    eigvecs = eigvecs[:, order]
    rot_z = float(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    local = centered @ eigvecs
    l = float(local[:, 0].max() - local[:, 0].min())
    w = float(local[:, 1].max() - local[:, 1].min())
    return l, w, rot_z


def mesh_volume(verts, faces):
    """Signed-tetrahedron volume from the origin; robust for closed meshes."""
    v0, v1, v2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    return float(np.abs(np.sum(np.cross(v0, v1) * v2) / 6.0))


def main():
    f = ifcopenshell.open(IFC_PATH)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.USE_WORLD_COORDS, True)

    rows = []
    for ifc_type in IN_SCOPE_TYPES:
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
            aabb_min, aabb_max = verts.min(axis=0), verts.max(axis=0)
            l, w, rot_z = obb_xy(verts[:, :2])
            h = float(aabb_max[2] - aabb_min[2])
            volume = mesh_volume(verts, faces)

            container = Element.get_container(el)
            storey = container.Name if container is not None else "unknown"

            is_external = get_is_external(el) if ifc_type == "IfcWall" else None

            rows.append({
                "global_id": el.GlobalId,
                "ifc_type": ifc_type,
                "storey": storey,
                "is_external": is_external,
                "centroid_x": centroid[0], "centroid_y": centroid[1], "centroid_z": centroid[2],
                "aabb_min_x": aabb_min[0], "aabb_min_y": aabb_min[1], "aabb_min_z": aabb_min[2],
                "aabb_max_x": aabb_max[0], "aabb_max_y": aabb_max[1], "aabb_max_z": aabb_max[2],
                "obb_dims_l": l, "obb_dims_w": w, "obb_dims_h": h, "obb_rot_z": rot_z,
                "volume": volume,
                "elevation_bottom": aabb_min[2],
            })

    df = pd.DataFrame(rows)
    assert df.isnull().sum().sum() - df["is_external"].isnull().sum() == 0, "unexpected NaNs outside is_external"
    assert (df["volume"] > 0).all(), "non-positive volume found"

    import os
    os.makedirs("data/bim", exist_ok=True)
    df.to_csv(OUT_CSV, index=False)
    print(f"Wrote {OUT_CSV}: {len(df)} rows")
    print(df["ifc_type"].value_counts())
    print(df["storey"].value_counts())

    fig, ax = plt.subplots(figsize=(8, 8))
    for t, color in CLASS_COLOR.items():
        sub = df[df["ifc_type"] == t]
        ax.scatter(sub["centroid_x"], sub["centroid_y"], s=6, c=color, label=t, alpha=0.7)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Schependomlaan -- element centroids, top-down")
    ax.legend(markerscale=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT_PLOT, dpi=150)
    print(f"Wrote {OUT_PLOT}")


if __name__ == "__main__":
    main()
