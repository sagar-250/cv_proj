"""What each BIM element is made of, and what colour that should look like.

Two colour tables, on purpose kept apart:

  CATALOGUE_RGB -- what the matching side KNOWS: one nominal colour per
      material category, the kind of value you would take from a product
      sheet (brick "bruin", steel RAL 7016, ...). Registration may use it,
      because it comes from the design model, not from the scan.
  (p3_render.py has its own RENDER_* tables for what the camera actually
      SEES. They are different numbers, plus per-element tint, texture,
      sun and shadow, exposure, dirt and a few elements built from another
      material than designed. If the two sides shared one table, a colour
      method would look perfect for the wrong reason.)

Material comes from the IFC's own IfcMaterial / IfcMaterialLayerSet. A
layered wall or slab shows its first layer that is not insulation or an air
gap ("99 Isolatie", "99 Lucht frame"); the IFC has no per-face material
(checked: ifcopenshell gives one material id for every triangle of a layered
wall), so one category per element is the best the file allows.

The categories are chosen so that several of them look alike on purpose:
cast concrete, prefab concrete, screed, calcium-silicate block, aerated
concrete and gypsum block are all light grey; white board is near-white;
zinc and the darker steel are grey too. Only a few (brick, primer steel,
wood, insulation) have a clearly different colour. A method that only works
when every material has its own colour would be over-fitted to the
simulator.

Run: .venv/bin/python src/materials.py
Output: data/bim/bim_materials.csv (global_id, ifc_material, category)
"""
import csv
import os

import ifcopenshell
import ifcopenshell.util.element

IFC_PATH = "data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc"
OUT_CSV = "data/bim/bim_materials.csv"

# (substring of the IFC material name, category); first match wins, so more
# specific names come first
NAME_RULES = [
    ("baksteen - bruin", "brick_brown"),
    ("baksteen - geel", "brick_yellow"),
    ("kalkzandsteen", "calcium_silicate"),
    ("cellenbeton", "aerated_concrete"),
    ("gipsblokken", "gypsum_block"),
    ("Beton gewapend - prefab", "concrete_prefab"),
    ("Beton", "concrete_cast"),
    ("Cementdekvloer", "screed"),
    ("Staal - RAL 7016", "steel_dark"),
    ("Staal", "steel_primer"),
    ("constructief staal", "steel_primer"),
    ("Hout", "wood"),
    ("multiplex", "plywood"),
    ("spaanplaat - wit", "board_white"),
    ("spaanplaat", "plywood"),
    ("gipsvezelplaat", "board_white"),
    ("Lichte scheidingswand", "board_white"),
    ("Zink", "zinc"),
    ("vloertegel", "tile"),
    ("Schoonloopmat", "tile"),
    ("Isolatie", "insulation"),
]
HIDDEN = ("Isolatie", "Lucht")   # layers you cannot see when another layer is there
DEFAULT = "concrete_cast"        # elements with no material in the IFC (4 walls)

# nominal colours, sRGB 0-255 (catalogue side only)
CATALOGUE_RGB = {
    "brick_brown": (150, 85, 62),
    "brick_yellow": (196, 160, 104),
    "calcium_silicate": (205, 203, 196),
    "aerated_concrete": (215, 213, 205),
    "gypsum_block": (222, 220, 214),
    "concrete_prefab": (182, 180, 174),
    "concrete_cast": (168, 166, 160),
    "screed": (158, 154, 146),
    "steel_dark": (62, 66, 72),          # RAL 7016 anthracite
    "steel_primer": (140, 68, 52),       # red-oxide shop primer
    "wood": (150, 104, 64),
    "plywood": (196, 164, 118),
    "board_white": (232, 230, 224),
    "zinc": (150, 155, 158),
    "tile": (120, 116, 110),
    "insulation": (226, 214, 150),
}
CATEGORIES = sorted(CATALOGUE_RGB)


def layer_names(mat):
    if mat is None:
        return []
    if mat.is_a("IfcMaterial"):
        return [mat.Name]
    if mat.is_a("IfcMaterialLayerSetUsage"):
        mat = mat.ForLayerSet
    if mat.is_a("IfcMaterialLayerSet"):
        return [layer.Material.Name for layer in mat.MaterialLayers if layer.Material]
    if mat.is_a("IfcMaterialList"):
        return [m.Name for m in mat.Materials]
    return []


def category_of(names):
    visible = [n for n in names if not any(h in n for h in HIDDEN)] or names
    for n in visible:
        for key, cat in NAME_RULES:
            if key in n:
                return cat
    return DEFAULT


def load_categories(path=OUT_CSV):
    """{global_id: category}"""
    with open(path, encoding="utf-8") as fh:
        return {r["global_id"]: r["category"] for r in csv.DictReader(fh)}


def main():
    f = ifcopenshell.open(IFC_PATH)
    rows = []
    for ifc_type in ["IfcColumn", "IfcBeam", "IfcSlab", "IfcWall"]:
        for el in f.by_type(ifc_type):
            names = layer_names(ifcopenshell.util.element.get_material(el))
            rows.append({"global_id": el.GlobalId, "ifc_material": "|".join(names),
                         "category": category_of(names)})
    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["global_id", "ifc_material", "category"])
        w.writeheader()
        w.writerows(rows)
    counts = {}
    for r in rows:
        counts[r["category"]] = counts.get(r["category"], 0) + 1
    for cat, n in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"  {cat:18s} {n}")
    print(f"{len(rows)} elements -> {OUT_CSV}")


if __name__ == "__main__":
    main()
