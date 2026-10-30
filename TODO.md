# TODO: making the simulation realistic

Full reasoning is in `docs/ROADMAP.md`. This file is the checklist. Tick items off as they are done.
Rule: finish a phase (its "Done when") before starting the next one.

Order: R0 -> R2 -> R3 -> R1 -> R4 -> R5 -> R6 -> R7 (R1 can run alongside R2).

## R0. Datasets and test rules
- [ ] Decide which datasets to use (see `docs/DATASETS.md`)
- [ ] Check license, size and download method of each chosen one
- [ ] Download them into `data/raw/` (gitignored)
- [ ] Write `results/datasets.md`
- [ ] Freeze metrics, train/test split, seeds. Keep one building completely unseen
- Done when: datasets.md exists and the split is frozen

## R2. Realistic sensing
- [ ] Render real-looking images (textures, light, shadows, blur) from the BIM
  - [x] First step: camera colour per point (`p3_render.py --color material`, now the default). Material from the IFC (`src/materials.py` -> `data/bim/bim_materials.csv`, 16 categories), rendered with its own colour table, per-element tint, brick/block courses, sun + shadows, exposure, dirt, 6% of elements built from another material. Still a coloured raycast, not photos
- [ ] Realistic drone flight paths with pose noise and coverage gaps
- [ ] Build the cloud from the images with COLMAP/OpenMVS (SfM)
- [x] Add a LiDAR-style sensor with range-dependent noise (`p3_render.py --sensor realistic`: range/angle noise, grazing dropout, holes, edge ghosts, floaters). Run through the pipeline: mean error 0.8 -> 1.7, registration still within 3.3 cm (`main_table_8/9`)
- [x] Survey-sized alignment offsets (`p3_render.py --georef gnss|local`)
- Done when: one stage goes images -> SfM -> cloud, and its errors are compared with a real cloud

## R3. Stronger registration (replace the ICP)
- [x] Coarse alignment, then robust ICP (`src/registration.py`). FPFH + RANSAC was tried first and picked wrong matches on partly built clouds; replaced by a gravity-aligned floor-plan search + height sweep (FPFH kept for comparison)
- [x] Estimate scale; use gravity / GNSS hints; flag failures and look-alike (ambiguous) matches instead of guessing
- [x] Test bench: `src/r3_registration_bench.py` (100 offsets per method, GNSS and indoor-SLAM cases)
- [ ] Full 100-offset run finished and >=95% success with zero silent failures
  - Paused 2026-10-28 at 70 of 120 trials (results so far in `results/r3_registration_bench_v2.csv`). Finish later with:
    `.venv/bin/python src/r3_registration_bench.py --trials 10 --methods plan --clouds data/pointclouds_georef_small --tag _v2 --resume`
    (runs only the 50 missing trials, ~1.5 h; don't change `registration.py` before then or the two halves won't match)
  - So far: 68/70 right; 1 wrong but flagged (stage 01 indoor, 3.6 m); 1 silent near-miss (stage 07 GPS #3, 6.8 cm)
- [x] Wired into `p4_match_bim.py` (`--registration plan`, default) with the result saved in `registration.json`
- [ ] Use colour in registration (`src/color_registration.py`, `--registration plan_color`): clutter filter by colour, finalists ranked by tight fit x colour agreement with the BIM material, coloured ICP polish
  - [x] Bench on busy + realistic clouds: `plan` vs `plan_color_rank` vs `plan_color_wrongcat` (wrong material catalogue), 1 offset per case (`--clouds data/pointclouds_color --tag _color`), log in `results/experiments/r3_bench_color.log`
    - First try stopped: with the wrong catalogue colour overruled geometry (stage 00, 3.3 m off, not flagged). Colour now only breaks near-ties
    - Result: all three 12/12 right, no silent failures, errors the same (1.2-1.5 cm median). Colour never decided: geometry always had a clear winner. Safe, but no gain on this building (`results/r3_registration_summary_color.md`)
  - [x] Pictures: `media/10_materials_color/` (`make_media.py --only colors`)
  - [x] Add `plan_color` (with coloured ICP) to the same bench with `--resume`: 12/12 right, no gain over `plan_color_rank`
  - [x] Full pipeline (P4 -> P7) with `plan_color` vs `plan` on the busy GPS clouds (`main_table_14` vs `13`): count error 1.95 -> 2.09, volume 2.24 -> 2.08, no real gain; `plan` stays the default
  - [ ] Stage 11 changes by 17 missed elements between two poses only ~2 cm apart: check how sensitive matching is to small pose changes
- Done when: >=95% success over those 100 offsets, and 0 wrong results that were not flagged

## R1. Realistic world
- [ ] Activity states: formwork, rebar, concrete, finished
- [ ] As-built deviations (elements slightly off, a few wrong or missing)
  - [x] 4% of elements built 4-12 cm off design, same ones every stage (`--site busy`)
  - [x] A few elements missing or built in a clearly wrong place: 3% late (missing for 2 stages), 1.5% of walls/columns 0.6-1.5 m off. Scored as not built
- [x] Clutter: scaffolding, crane, fence, shoring props, material piles, workers (`src/site_clutter.py`, `p3_render.py --site busy`)
- [ ] Run the pipeline on the busy site and fix what breaks
  - [x] First run (`main_table_10/11`): little breaks; registration within 4.9 cm, errors close to the clean realistic run
  - [x] Rerun with late + misplaced elements (`main_table_12/13`): 3 of 78 late and 7 of 108 misplaced element-stages wrongly called built (all 7 tiny wall pieces); registration fine, one false-alarm flag (stage 10)
  - [ ] Clutter caused 28 of 67 false "built" (scaffold decks taken for unbuilt balcony slabs; workers and piles inside unbuilt ground-floor walls). Ideas: require a detection to persist into the next scan, or use colour
  - [ ] Left: unbuilt slab "found" from noisy wall-top points under it; stray points making a half-built wall look complete (a sturdier height measure was tried and made things worse)
- [x] Ground truth stores the new errors: in the stage npz (`gt_deviated_ids`, `gt_missing_ids`, `gt_misplaced_ids`), since manifests are the schedule and the same for every site
- Done when: a viewer shows a stage with clutter and deviations (done: `media/09_busy_site/busy_site_world.png`; open items above remain)

## R4. Better matching and detection
- [ ] Soft matching that understands occlusion
- [x] Work out which elements each camera could really see (`p4_visibility.py`, default in p7_score.py)
- [ ] Re-check the visibility threshold after adding the joint-margin rule (was 30 points; 276 built elements wrongly "not started" across all stages)
  - [x] Sweep done (`results/experiments/r4_visibility_threshold_sweep_*.csv`): 80 is better on both simple and realistic runs (fewer missed, lower worst stage, ~4% more unsure)
  - [ ] Adopt it only after the R0 split is frozen (now it would be tuned on the test runs)
- [ ] Optional: learned segmentation, compared with the geometric method
- Done when: P4 eval rerun on the new data

## R5. Scoring with uncertainty
- [ ] Status that knows about activities
- [x] Confidence per element; "uncertain" as a real answer: evidence groups (status / position vs construction front / family / points vs expected), bias and spread per group from `src/r5_calibrate.py`
- [x] Progress number with error bars (`progress_cal_*`, `progress_lo/hi_*` in main_table.csv; error-vs-completion panel in `results/figures/p7_progress_error.png`). Calibrated on clean realistic, tested on busy: count error 1.76 -> 0.84 (worst 5.0 -> 2.3), truth inside the 90% range 14/16 stages (+-1.6 points); volume 14/16 but +-5.6 points. Second test (busy + late/misplaced): 1.95 -> 0.69, 15/16 inside
- [ ] Volume-weighted: correction barely helps (a few big elements dominate); "partial" slabs/beams are nearly always fully built (height coverage means little for flat elements)
- [ ] Recalibrate once the R0 split is frozen; test on a third condition
- Done when: main_table.csv has intervals and error-vs-completion curves exist

## R6. Real-data check
- [ ] Run the pipeline unchanged on real datasets
- [ ] Write a sim-vs-real gap report
- Done when: the report says which simulated effects mattered

## R7. Wrap-up
- [ ] Turn each realism feature off one at a time and measure the drop
- [ ] Run on the unseen building
- [ ] Refresh figures, presentation, CLAUDE.md

## Known issues in the current pipeline (found while documenting)
- [x] Progress estimate always below ground truth by 3-8 points. Cause: `clip_partial_height` dropped a wall's side faces. Fixed; mean error 4.8 -> 2.1 points (-> 1.5 with the visibility test)
- [x] p7_score.py docstring said 136 uncertain elements (10%); updated
- [ ] About 25% of elements (15% of volume) are never seen by any camera even when built (tiny pieces, hidden slabs). Needs better camera paths (R2), not better matching
- [ ] `p4_segment.py` baseline matches almost nothing in early stages (n_split dominates in `p4_eval_summary.csv`).

## Ideas for later
- [ ] Material recognition from texture: camera colour changes with light, exposure and material mix, so raw colour is a weak cue. Train an ML model that looks at a small image/texture patch and says which material it probably is. Then check it against the material the BIM says the element it was matched to should be (agrees / disagrees), as extra evidence for matching and for "built as designed".
- [ ] Use colour registration for scan-to-scan merging: when the same building is scanned in two runs (two flights, or two survey days), align the two clouds with each other. Colour should help more here than against the BIM, since both clouds show the same real surfaces and materials.
