# What has been done so far

Plain summary of the current pipeline. For the plan ahead, see `ROADMAP.md` and `../TODO.md`. For data, see `DATASETS.md`. For the original spec, see `SPEC.md`.

## The idea
We have the full design model (BIM) of a building. We pretend the building is only partly built, simulate what a drone or scanner would capture, then check whether our code can work out which parts are built and how far along the project is. Because we made the partial states ourselves, we know the right answer exactly.

## Building used
Schependomlaan, a Dutch residential block. 1692 structural elements in scope: 1216 walls (934 `IfcWall` including 282 standard-case), 23 columns, 174 beams, 279 slabs (`results/p0_inventory.md`).

## Steps, in order

| Step | Script | What it does | Output |
|---|---|---|---|
| P0 | `p0_sanity_check.py` | Counts elements; checks if element IDs in the weekly as-planned files match the design file. They do not (0% overlap), so only the design IFC is usable. | `results/p0_inventory.md` |
| P1 | `p1_parse.py` | Reads the IFC into a table: centre, bounding box, volume, storey, interior/exterior. | `data/bim/bim_complete.csv` |
| P2 | `p2_ablate.py` | Makes 16 "partly built" stages: each floor goes structure, then outer walls, then inner walls. Some elements are only partly built (height cut). Saves which elements exist. | `data/states/*/manifest.json` |
| P3 | `p3_render.py` | Simulates a scan of each stage by shooting rays from drone and indoor viewpoints. Adds noise, dropped points, a shifted alignment, ground and trees. | `data/pointclouds/` (not in git) |
| P4 | `p4_match_bim.py` | Undoes the alignment shift with ICP, then assigns each point to the nearest BIM element. | `data/matches/` (not in git) |
| P4 (old) | `p4_segment.py`, `p4_eval_segment.py` | Earlier try: find planes and clusters without using the BIM. Kept as a baseline. | `results/p4_eval_summary.csv` |
| P7 | `p7_score.py` | Marks each element completed, partial, not started or uncertain, and computes project progress %. | `data/status/`, `results/main_table.csv` |
| Extras | `export_*.py`, `*_viewer*.html`, `make_presentation.py` | Browser viewers and the mid-review slides. | `results/` |

## Current result
Progress error, count-weighted, percentage points (estimate minus truth), Linux runs on 2026-10-27:

| Version | Mean error | Worst stage | Notes |
|---|---|---|---|
| Before (committed code) | 4.8 | 7.9 | Always too low |
| + half-built wall fix | 2.1 | 4.6 | Same "unknown" rule as before |
| + per-stage visibility test | 1.5 | 4.2 | `p7_score.py --mask visibility` |
| + GPS-sized offsets (metres) + new registration + visibility margin rule | 0.8 | 3.6 | Current. Every stage registered within 0.5-3 cm (`--georef gnss`, `--registration plan`) |

The old "unknown" rule (elements never detected in the finished building) is circular: it uses the final stage's detections, so it can never be wrong at the last stage. The visibility test does not peek at that.

What is left: about 25% of elements (15% of volume) are never seen by any camera even when built: tiny column pieces, short wall stubs, small beams, and slabs hidden under other slabs. They are honestly marked "unknown". Better camera paths (R2) are the fix, not better matching.

Tables: `results/experiments/main_table_0_before.csv`, `main_table_1_clipfix.csv`, `main_table_2_clipfix_visibility.csv`, `main_table_3_gnss_plan_v1.csv` (stage 01 registration bug), `main_table_4_gnss_plan_v2.csv` (same as current `results/main_table.csv`). Worst stage left: the last one (-3.6 points, 13 built elements missed). The old blind-segmentation baseline still matches very few elements in early stages.

## Why the simulation is too easy
Real-world gaps (details in `ROADMAP.md`): clean raycast instead of photo-based reconstruction, one simple alignment shift, fixed camera paths, a perfect BIM, only trees as clutter, and only one building. The plan R0 to R7 fixes these.

## Log
- 2026-10-24: CLAUDE.md written; pipeline flow reviewed; realism plan (R0-R7) drafted; first dataset search done.
- 2026-10-27: TODO.md, PROGRESS.md and DATASETS.md created. Found that `data/raw/` is not present in this checkout. Downloaded only the design IFC (49 MB, sparse git checkout of the jakob-beetz fork) into `data/raw/schependomlaan/Design model IFC/`. Set up a Linux `.venv` (via `virtualenv`, since `python3-venv` is missing).
- 2026-10-27: Found and fixed the half-built wall bug in `clip_partial_height` (`p2_p3_simulate_stages.py`): it dropped every side face of a wall, so a "half-built" wall was just its hidden bottom face (checked: 2 of 12 faces kept, both at the base). Now it shortens the wall instead.
- 2026-10-27: p3 now saves the true alignment offset (`gt_T_bim_to_cloud`, eval only) and the camera positions in each stage file. Added `p4_visibility.py` (would we have seen this element if it were built?) and made it the default "unknown" rule in `p7_score.py`. p7's table now also counts missed-built and false-built elements.
- 2026-10-27: Added `p3_render.py --sensor realistic` (range/angle-dependent noise, grazing-angle dropout, texture-less holes, edge ghosts, floaters). Not yet run through the pipeline.
- 2026-10-27: Started R3: `src/registration.py` (feature matching + RANSAC + robust ICP, gravity and GNSS priors, scale) and `src/r3_registration_bench.py`. First test: the old ICP failed every trial even at GPS-sized offsets (1-2 m); the new method worked when the building was mostly built but failed early stages (look-alike floors). Added GNSS cropping and scale refinement; re-testing.
- 2026-10-28: Registration benchmark, 100 offsets per method (`results/r3_registration_summary.md`): old ICP 0% success (every failure silent); FPFH + RANSAC 30-40% (`..._fpfh.md`); new floor-plan method 98% in both cases (GPS drone survey and indoor scan without GPS), 1 silent near-miss (8.5 cm). Two fixes on the way: levelling the cloud before the search (tilt smeared heights), and a 3D height sweep (the height histogram was fooled by ground at early stages).
- 2026-10-28: First full pipeline run with GPS-sized offsets + new registration (`results/experiments/main_table_3_gnss_plan_v1.csv`): mean progress error 1.0 point (better than before, despite much harder alignment). But stage 01 (not in the benchmark) registered 0.6 m too high without flagging it: the height sweep matched the top of the foundation beams to their bottom faces. Fix: keep the top 3 height peaks, fully refine the best 3 candidates, rank them by the share of points within 5 cm. Stage 01 added to the benchmark. Pipeline rerun: stage 01 now 1.4 cm off; mean progress error 0.8 points, worst 3.6 (final stage). Benchmark rerun still going.
- 2026-10-28: Paused the benchmark rerun at 70 of 120 trials to free the machine. The benchmark now saves each trial as soon as it finishes and can pick up where it left off (`--resume`); the 70 finished trials were rebuilt from the run's log (`--from-log`) into `results/r3_registration_bench_v2.csv`, so none need rerunning. So far 68/70 right, 1 wrong but flagged, 1 silent near-miss of 6.8 cm (stage 07, GPS case). Resume command is in TODO.md (R3).
- 2026-10-27: Correction: an earlier note said `n_uncertain` jumps between 3 and 330. That was a display mistake (numbers cut off); it is steady at 320-378.
- 2026-10-29: Added `media/` (made by `src/make_media.py`, index in `media/README.md`): pictures, GIF animations and 3D (PLY) files showing the ground-truth stages, the simulated scans, the half-built wall fix before/after, registration before/after and how its search works, the visibility check, and the old vs new pipeline element by element. Rebuild it after pipeline changes.
- 2026-10-29: First run with the realistic sensor (`--sensor realistic --georef gnss`). Registration still fine (all stages within ~3 cm), but progress error went 0.8 -> 1.3. Found and fixed two simulator bugs and one pipeline weakness:
  - Simulator bug 1: "texture-less holes" were one per 20k raw rays (hundreds per stage), centred on random rays, so they piled up around the indoor scanners and erased whole rooms. Now one hole per 60 m2 of surface, spread evenly.
  - Simulator bug 2: holes also hit the indoor laser scans. A laser needs no texture, and next to the scanner points are so dense that one hole could erase most of a slab (one slab: 3097 points -> 54). Holes now only affect drone-photo points.
  - Pipeline weakness: with a noisy sensor, a few stray points from a slab land inside the space of an unbuilt wall and it gets called built (stage 05: 25 such walls). New rule in `p7_score.py`: a detection with less than 10% of the points the cameras should have given it is thrown out (`p4_visibility.py` now also writes `visibility_detected.json`). With the hole bugs still in, it cut false "built" 105 -> 26 and halved the volume error, but missed more built elements; rerun with both fixes pending.
  - Tried and dropped: cleaning the cloud (outlier removal + plane smoothing) also removed sparse points from far beams; a surface-direction (normal) check removed too many good points; a noise-adaptive distance threshold changed nothing.
- 2026-10-29: R1 started: `src/site_clutter.py` + `p3_render.py --site busy` (scaffolding, crane, fence, shoring props, material piles, workers; 4% of elements built 4-12 cm off). Picture: `media/09_busy_site/`.
- 2026-10-29: Colour (R2/R3). New `src/materials.py`: each element's material from the IFC's own material list, sorted into 16 colour groups (`data/bim/bim_materials.csv`; 50 of the 51 hardwood elements have no geometry). `p3_render.py --color material` (now the default) gives each point a camera colour. The renderer has its own colour table, different from the catalogue the matcher knows, and adds per-element tint, brick/block courses, sun with shadow rays, exposure and white balance per camera, dirt, and 6% of elements built from a different material. Still a coloured raycast, not photos + SfM. Checked: `--color class` gives exactly the old clouds, and xyz doesn't depend on `--color`. New `src/color_registration.py` (`plan_color`) wraps `registration.py` without changing it: a colour filter (drops about half of tree and 40% of crane points, no building points), colour-aware choice between finalists, and an optional coloured-ICP polish (about 100 s slower, no gain in the first test). Pictures: `media/10_materials_color/`.
  - First benchmark try (`results/experiments/r3_bench_color_v0_ungated.*`): with a deliberately wrong material catalogue, stage 00 (GPS case) ended up 3.3 m off and NOT flagged, because colour could overrule clear geometry. Fix: colour now only decides between candidates whose geometric fit is within 10% of the best (the same margin as the ambiguity check). Benchmark restarted.
  - Pose check on stage 03 (`media/10_materials_color/colour_agreement_poses.csv`): colour clearly penalises a pose one storey too high (colour fit 0.57 vs 0.70) but not a 3 m shift or a 180 degree turn (0.71, 0.68), since the facade is brick all round. Geometry still does most of the work.
