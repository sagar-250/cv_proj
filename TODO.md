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
- [ ] Realistic drone flight paths with pose noise and coverage gaps
- [ ] Build the cloud from the images with COLMAP/OpenMVS (SfM)
- [x] Add a LiDAR-style sensor with range-dependent noise (`p3_render.py --sensor realistic`: range/angle noise, grazing dropout, holes, edge ghosts, floaters). Not yet run through the pipeline
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
- Done when: >=95% success over those 100 offsets, and 0 wrong results that were not flagged

## R1. Realistic world
- [ ] Activity states: formwork, rebar, concrete, finished
- [ ] As-built deviations (elements slightly off, a few wrong or missing)
  - [x] 4% of elements built 4-12 cm off design, same ones every stage (`--site busy`)
  - [ ] A few elements missing or built in a clearly wrong place
- [x] Clutter: scaffolding, crane, fence, shoring props, material piles, workers (`src/site_clutter.py`, `p3_render.py --site busy`)
- [ ] Run the pipeline on the busy site and fix what breaks
- [ ] Manifests store the new ground truth
- Done when: a viewer shows a stage with clutter and deviations

## R4. Better matching and detection
- [ ] Soft matching that understands occlusion
- [x] Work out which elements each camera could really see (`p4_visibility.py`, default in p7_score.py)
- [ ] Re-check the visibility threshold after adding the joint-margin rule (was 30 points; 276 built elements wrongly "not started" across all stages)
- [ ] Optional: learned segmentation, compared with the geometric method
- Done when: P4 eval rerun on the new data

## R5. Scoring with uncertainty
- [ ] Status that knows about activities
- [ ] Confidence per element; "uncertain" as a real answer
- [ ] Progress number with error bars
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
