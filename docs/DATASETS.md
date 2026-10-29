# Datasets

Status: candidates only. Nothing is verified beyond search results. Check each official page before downloading.

## What we already have
- `data/raw/` is **not in this checkout** (it is gitignored). Only the derived files are in git: `data/bim/`, `data/states/`, `data/status/`.
- The raw Schependomlaan data (design IFC, weekly as-planned IFCs, drone point clouds weeks 26-30, schedule) must be re-downloaded from a fork, see `SPEC.md` section 4.1.
- ConSLAM / SLAM2REF samples were used by the viewer scripts earlier but are also not on disk now.

## What we need (checklist)
1. The BIM of the building.
2. The BIM at different construction stages. We can make these ourselves (P2), as we do now.
3. Real scans or point clouds of the same site at those stages, so we can test on real data.
4. How the real scan lines up with the BIM (a known pose or alignment), so we can check our registration.
5. Which BIM elements are really built in each real scan (the true answer), otherwise we cannot measure error on real data.
6. Real sensor noise examples, so the simulation can copy them.
7. More than one building, so we know the method is not tuned to one.
8. Real clutter examples (scaffolding, materials, workers).

No public dataset gives all of this together. Real data usually lacks item 5.

## Candidates

| Dataset | Has | Helps with items | Concern |
|---|---|---|---|
| Schependomlaan (we use) | Design IFC, weekly as-planned IFCs, schedule, drone point clouds weeks 26-30 | 1, 3 | As-planned IDs do not match design IDs (0% overlap), so no per-element truth; clouds are in the raw repo only |
| ConSLAM | Real site, repeated visits, terrestrial scans, handheld sequences | 3, 4, 6 | Indoor, handheld, no BIM stage truth |
| Hilti SLAM 2023 | Multi-session construction sequences, laser-scanner ground truth | 4, 6 | No BIM; license reportedly non-commercial |
| Buildings 12(10) 1754 (Kavaliauskas 2022) | 6 IFC models with laser scans, task is built vs not built | 1, 3, 5 | No public download found; contact authors |
| BIMNet | IFC-linked real scans | 1, 3, 5 | Access by request form |
| DeKH / BIMStruct3D | Hospital scan with ground-truth BIM and labels | 1, 3, 7 | Release terms unknown |
| LivingBIM | One building: design BIM plus as-built clouds | 1, 3 | Access unknown |
| SIP (Site in Pieces) | Construction-phase LiDAR with point labels | 8 | Terrestrial, access unknown |
| UseGeo, EuroSDR RPAS | Drone images with LiDAR ground truth (not construction) | 6 | Not construction |
| buildingSMART sample IFCs | Many IFC models | 7 | BIM only |
| Synthetic Point Cloud set, iVISION-2DCD | Synthetic scans / images | reference only | Synthetic |

Sources: arXiv 2404.09765 (Hilti), HAL hal-04039392 (ConSLAM), MDPI Buildings 12(10) 1754, arXiv 2512.09062 (SIP), arXiv 2607.03553 (iVISION-2DCD), Crossref 10.3390/DATA11010016.
