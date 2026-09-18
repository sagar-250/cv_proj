"""Phase 0 -- data sanity check (SPEC.md section 6).

Prints per-class element counts for the design IFC and tests whether GlobalId
survives the Synchro as-planned export by comparing against the Week 28 export.
Writes the results to results/p0_inventory.md.

Run: .venv/Scripts/python.exe src/p0_sanity_check.py
"""
import ifcopenshell

DESIGN_IFC = "data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc"
WEEK28_IFC = (
    "data/raw/schependomlaan/As Planned models/"
    "Week 28 10 july IFC Schependomlaan incl planningsdata.ifc"
)
OUT_MD = "results/p0_inventory.md"

CLASSES = [
    "IfcWall", "IfcWallStandardCase", "IfcColumn", "IfcBeam", "IfcSlab",
    "IfcDoor", "IfcWindow", "IfcBuildingStorey",
]


def main():
    f = ifcopenshell.open(DESIGN_IFC)

    counts = {}
    lines = ["# Phase 0 -- Data Inventory\n", "## Design IFC element counts\n",
             "| IfcType | Count |", "|---|---|"]
    print(f"{'IfcType':24s} Count")
    for t in CLASSES:
        n = len(f.by_type(t))
        counts[t] = n
        print(f"{t:24s} {n}")
        lines.append(f"| {t} | {n} |")

    in_scope = ["IfcWall", "IfcWallStandardCase", "IfcColumn", "IfcBeam", "IfcSlab"]
    in_scope_total = sum(counts[t] for t in in_scope)
    lines.append("")
    lines.append(f"In-scope structural elements (wall+column+beam+slab): **{in_scope_total}**")
    print(f"\nIn-scope structural elements (wall+column+beam+slab): {in_scope_total}")

    sparse = [t for t in ["IfcColumn", "IfcBeam"] if counts[t] < 20]
    if sparse:
        msg = f"WARNING: sparse class(es) for training: {sparse} (< 20 instances)"
        print(msg)
        lines.append("")
        lines.append(f"**{msg}**")

    # GlobalId overlap test: does GlobalId survive the Synchro export?
    g = ifcopenshell.open(WEEK28_IFC)
    a = {e.GlobalId for e in f.by_type("IfcProduct")}
    b = {e.GlobalId for e in g.by_type("IfcProduct")}
    overlap = len(a & b)
    overlap_pct = 100 * overlap / len(a) if a else 0.0

    print(f"\noverlap {overlap} | design {len(a)} | week28 {len(b)}"
          f" | overlap% {overlap_pct:.1f}")

    lines.append("\n## GlobalId overlap: design IFC vs Week 28 as-planned export\n")
    lines.append(f"- design IfcProduct count: {len(a)}")
    lines.append(f"- week28 IfcProduct count: {len(b)}")
    lines.append(f"- overlapping GlobalIds: {overlap} ({overlap_pct:.1f}% of design)")

    if overlap_pct > 80:
        decision = (
            "Overlap is high -- GlobalId survives the Synchro export. "
            "The weekly as-planned IFCs can be used as additional real "
            "evaluation states alongside the synthetic ablations."
        )
    else:
        decision = (
            "Overlap is near zero -- the Synchro export reassigned GlobalIds. "
            "Do not use the as-planned IFCs for element identity. Use only the "
            "design IFC and synthetic ablated states; note this as a limitation."
        )
    print(f"\nDecision: {decision}")
    lines.append(f"\n**Decision:** {decision}\n")

    with open(OUT_MD, "w", encoding="utf-8") as out:
        out.write("\n".join(lines))
    print(f"\nWrote {OUT_MD}")


if __name__ == "__main__":
    main()
