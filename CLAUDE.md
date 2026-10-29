# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Research prototype: BIM-based construction progress monitoring, evaluated in simulation. A complete design IFC (Schependomlaan, Dutch residential) is ablated into incremental construction stages, point clouds are simulated for each stage, matched back to the BIM, and per-element status plus overall progress % are scored against exact ground truth. `docs/SPEC.md` is the original spec (phases P0–P9, rules, rationale); read it before large changes. The implementation deliberately deviates from it (see below).

## Commands

There is no build, lint, or test suite. Each phase is a standalone script run from the repo root with `.venv/bin/python` (a Linux venv made with `python3 -m virtualenv .venv`, since `python3-venv` is not installed; older docstrings still say `.venv/Scripts/python.exe` from the original Windows setup). Deps: ifcopenshell, numpy, scipy, pandas, matplotlib, open3d-cpu (smaller wheel, same API; the network here is slow), scikit-learn, python-pptx, tabulate.

```
python src/p0_sanity_check.py        # element counts, GlobalId overlap -> results/p0_inventory.md
python src/p1_parse.py               # IFC -> data/bim/bim_complete.csv
python src/materials.py              # IFC materials -> data/bim/bim_materials.csv (16 colour categories)
python src/p2_ablate.py              # -> data/states/{stage}/manifest.json
python src/p3_render.py [--sensor simple|realistic] [--georef small|gnss|local] [--site clean|busy] [--color material|class] [--out DIR]   # -> data/pointclouds/ (+ viewer data, default --out only)
python src/p4_match_bim.py [--stage NAME] [--registration plan|plan_color|icp_old] [--prior gnss|none]
                                     # -> data/matches/{stage}/matches.json + registration.json
python src/p4_visibility.py [--stage NAME]  # -> data/matches/{stage}/visibility.json (needs registration.json)
python src/p7_score.py [--mask visibility|final]  # -> data/status/, results/main_table.csv, figures
python src/r3_registration_bench.py [--trials N] [--methods icp_old,fpfh,plan,plan_color,plan_color_rank,plan_color_wrongcat] [--clouds DIR] [--tag _x] [--resume] [--from-log LOG]  # registration test bench; saves per trial, --resume skips finished trials
python src/p4_segment.py [--stage NAME]     # older blind RANSAC/DBSCAN baseline
python src/p4_eval_segment.py        # scores p4_segment against held-out GT
python src/make_presentation.py      # -> results/presentation/*.pptx
python src/make_media.py [--only charts,truth,...]  # -> media/ pictures, GIFs, PLY files (index: media/README.md)
```

Run order is the phase order; every phase communicates with the next only through files on disk. `--stage NAME` takes a stage directory name such as `05_01_eerste_verdieping_structure`. Only the design IFC is needed for P1-P7 (`data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc`, a 49 MB sparse checkout); P3/P4 take minutes and ~4 GB RAM on this 11 GB machine, so don't run two P4-sized jobs at once. Keep before/after result tables in `results/experiments/` when changing the pipeline.

## Architecture

- **Raw data is not in git.** `data/raw/` (5.5 GB Schependomlaan clone, ConSLAM/SLAM2REF samples) is gitignored; clone per SPEC §4.1. Also gitignored and regenerable: `data/renders/`, `data/pointclouds/`, `data/matches/`, `data/matches_cache/`, `data/observations/`, `data/bim/bim_surface_samples.npz`. Committed: `data/bim/`, `data/states/*/manifest.json`, `data/status/*/status.csv`, `results/`.
- **Stages** (`p2_ablate.py`): 16 milestone states, per storey bottom-up: structure → envelope (external walls) → interior (partitions); storeys without a category skip it. Ordered by (x, y) work front with 8% seeded adjacent swaps. The manifest lists `kept_ids`, `partial_ids` (height-clipped, with `completion_fraction`), and is the ground truth for everything downstream. Stage dir names are `NN_{storey}_{phase}`, sorted lexicographically everywhere.
- **Simulation instead of Blender** (`p2_p3_simulate_stages.py` holds shared helpers; `p3_render.py` imports from it): Open3D raycasting from an exterior drone orbit plus per-floor interior walkthroughs (first hits only), with per-element pose jitter, a whole-scene offset (`REG_JITTER`, plus `survey_offset` for `--georef gnss|local`), sensor noise and dropout (`--sensor realistic` makes them range/angle dependent), and fixed ground and trees as clutter. `--site busy` (`site_clutter.py`) adds scaffolding, a tower crane, fence, shoring props, material piles and workers as raycast pseudo-elements (`global_id` "", `gt_class` = clutter type), and builds a fixed 4% of elements 4-12 cm off design (eval-only `gt_deviated_ids`; still count as built). Partial elements are made by clamping vertex z to the cut height (`clip_partial_height`). Each stage npz also stores eval-only `gt_T_bim_to_cloud` and the camera poses in the survey frame. `rgb` is the simulated camera colour by default (`--color material`: material from the IFC, but rendered with its own colour table, tint, brick/block texture, sun + shadow rays, exposure, dirt, and 6% of elements built from a different material); `rgb_source` says "camera" or "class". Class colour (`--color class`, the old look) is ground truth and must never reach registration or matching. Colour uses its own seeded RNG streams, so xyz does not depend on `--color`. SPEC phases P3/P5 (Blender RGB/depth/index passes) were never built.
- **Registration** (`registration.py`, used by `p4_match_bim.py --registration plan`): level the cloud from its horizontal-surface normals, FFT-correlate a top-down wall raster against the BIM's over all headings, sweep height in 3D, verify the top candidates with ICP, flag ambiguous/implausible results (`info["ok"]`), then multi-scale robust point-to-plane ICP (+ scale for GNSS photogrammetry). The old plain ICP (`icp_old`) only handles cm-level offsets. `color_registration.py` (`plan_color`) wraps it without changing it: drops points whose colour fits no BIM material, ranks finalists by tight fit x colour agreement with the BIM material they land on (chromaticity, so lighting matters little), and polishes with Open3D coloured ICP. The catalogue colours it knows (`materials.CATALOGUE_RGB`) are deliberately different from the renderer's. Results: `results/r3_registration_summary*.md`.
- **Detection = BIM matching, not segmentation** (`p4_match_bim.py`, the live path): sample the complete BIM surface once with GlobalId tags, register (above), then tight-threshold nearest-neighbour assignment per point with a joint margin (`ASSIGN_MARGIN`). `p4_segment.py` + `p4_eval_segment.py` are the earlier classical baseline kept for comparison (results in `results/p4_eval_summary.csv`). Evaluation GT fields (`gt_class`, `gt_global_id`) in the point-cloud npz files must never be read by the matching/segmentation code, only by the eval scripts.
- **Scoring** (`p7_score.py`): `height_coverage` replaces SPEC's 3D IoU. ≥0.5 completed (C=1), 0.15–0.5 partial (C=coverage). An undetected element is not_started only if `p4_visibility.py` says the cameras would have collected enough assignable points had it been built (raycast against the elements we did detect, using the *estimated* registration); otherwise `uncertain` and excluded. `--mask final` restores the old proxy (detected at the final stage), which is circular. Progress is computed count- and volume-weighted and compared with the manifest GT in `results/main_table.csv`, which also has error-source counts (`n_missed_built`, `n_false_built`, ...) and the registration error.
- **IFC quirks** (`p1_parse.py`): `by_type("IfcWall")` already includes `IfcWallStandardCase` (IFC2X3 subtype); storey comes from the spatial-containment relation, not the `Elevation` attribute (mm vs the file's metres); the as-planned Synchro IFC and the as-built comparison spreadsheet share no GlobalIds with the design IFC, so only the design IFC and synthetic stages are usable as ground truth.
- **Viewers** (`src/export_*.py`, `src/*_viewer*.html`, `visualize_pointcloud.py`): static HTML pages with base64-encoded data bundles in `results/*_viewer_data.js`. The `*_head.html` / `*_tail.html` / `*_body.html` files are fragments the export scripts assemble into the final HTML. Order matters: `export_pointcloud_viewer_data.py` creates `pointcloud_viewer_data.js`, then `export_conslam_viewer_data.py` and `export_conslam_sessions.py` append to it, and `p3_render.py` also writes into it.

## Conventions

- Fixed random seeds everywhere; runs must be exactly reproducible.
- Phases write results to disk and print the numbers. Do not leave results only in memory.
- Keep code plain: NumPy, pandas, JSON, CSV, no config layers or premature classes. Each script has a docstring explaining its method and why; update it when behaviour changes.
- Ground truth (manifests, `gt_*` arrays in the npz files) is for evaluation only. Registration, matching and visibility must never read it.
- Plan and progress live in `TODO.md` (checklist), `docs/ROADMAP.md` (R0-R7 plan), `docs/PROGRESS.md` (what's done, results, dated log) and `docs/DATASETS.md`. Update the log when you change the pipeline. The user prefers plain, simple language in docs and replies.
