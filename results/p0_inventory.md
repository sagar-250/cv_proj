# Phase 0 -- Data Inventory

## Design IFC element counts

| IfcType | Count |
|---|---|
| IfcWall | 934 |
| IfcWallStandardCase | 282 |
| IfcColumn | 23 |
| IfcBeam | 174 |
| IfcSlab | 279 |
| IfcDoor | 205 |
| IfcWindow | 259 |
| IfcBuildingStorey | 6 |

In-scope structural elements (wall+column+beam+slab): **1692**

## GlobalId overlap: design IFC vs Week 28 as-planned export

- design IfcProduct count: 3822
- week28 IfcProduct count: 1555
- overlapping GlobalIds: 0 (0.0% of design)

**Decision:** Overlap is near zero -- the Synchro export reassigned GlobalIds. Do not use the as-planned IFCs for element identity. Use only the design IFC and synthetic ablated states; note this as a limitation.
