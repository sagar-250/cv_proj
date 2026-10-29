# Realism Roadmap (R0–R7)

Goal: make the simulation as close to real-world conditions as possible, and make the pipeline (P0–P7, see `SPEC.md` and `CLAUDE.md`) work well on it. Real deployment is out of reach, so the simulation is the proxy and real datasets calibrate and validate it.

Rules carried over from SPEC.md: phases run in order, each has a Definition of Done (DoD), phases communicate through files on disk, fixed seeds everywhere.

## What is unrealistic today, and the fix

| Current | Why it is unrealistic | Fix |
|---|---|---|
| First-hit raycast, Gaussian noise, random dropout | Real clouds come from photogrammetry or LiDAR with structured errors: holes on glass and textureless surfaces, range-dependent noise, ghost surfaces | Render images, reconstruct with SfM/MVS (R2) |
| One rigid registration jitter, undone by ICP | Real clouds have scale drift, non-rigid warp, weak or wrong georeference | Realistic pose and georeference error model (R3) |
| Fixed orbit and walkthrough paths | Real flights have coverage gaps, motion blur, lighting changes, battery limits | Planned or noisy flight paths, weather and lighting (R2) |
| BIM is the exact design | Real as-built deviates: tolerances, shifted elements, unmodelled items | As-built deviation model (R1) |
| Clutter is only ground and trees | Real sites have scaffolding, cranes, formwork, materials, workers | Clutter library (R1) |
| Partial completion is only height-clipping | Real work goes formwork, rebar, concrete, finish | Activity-level states (R1) |
| One building | Possible overfit to Schependomlaan | Held-out buildings and real data (R0, R6) |

## Phases

### R0. Datasets and benchmark protocol
- Find and verify candidate datasets; record license, size, and whether each has BIM, poses, ground truth. Candidates: ConSLAM and SLAM2REF samples (already in `data/raw/`), Hilti SLAM Challenge, *Buildings* 12(10) 1754 BIM + laser-scan pairs, BIMNet, buildingSMART Sample-Test-Files, Schependomlaan weeks 26-30 point clouds.
- Freeze the evaluation protocol: per-element F1, progress error in percentage points, error-vs-completion curve, splits, seeds. Hold at least one building out entirely.
- **DoD:** `results/datasets.md` written, splits frozen.

### R1. Realistic world model (`p2_ablate.py`, new `p2b_world.py`)
- Activity-level states per element: formwork, rebar, concrete, finished.
- As-built deviations: position and size tolerances, a few wrong or missing elements.
- Temporary works and clutter library: scaffolding, cranes, material stacks, workers, fences.
- Manifest keeps exact ground truth, extended with activity labels.
- **DoD:** manifests carry the new fields; a viewer shows a stage with clutter and deviations.

### R2. Realistic sensing (largest change)
- Render RGB and depth with textures, lighting, shadows, motion blur (Blender or BlenderProc; Open3D raycasting stays as the fast fallback). This restores SPEC P3.
- Realistic drone paths: grid or orbit missions with pose noise and coverage gaps.
- Reconstruct with COLMAP or OpenMVS (or similar) to get a genuine photogrammetric cloud.
- Keep a LiDAR-style simulator as a second sensor, with range-dependent noise.
- **DoD:** one stage goes images -> SfM -> cloud on disk; its error statistics are compared with a real cloud from R0 and the gap is documented.

### R3. Robust registration (replaces the ICP in `p4_match_bim.py`)
- Coarse-to-fine: FPFH or learned features with RANSAC, then robust ICP (Huber or trimmed loss).
- Scale estimation (SfM clouds have arbitrary scale), gravity and ground-plane priors, per-storey refinement.
- Report error against the known true transform and failure rates over many seeds.
- **DoD:** registration success at a target rate (e.g. >=95%) over 100 randomized offsets; failure modes listed.

### R4. Robust element matching and detection
- Soft assignment with occlusion reasoning instead of hard nearest-neighbour thresholds.
- Per-element visibility computed from cameras, replacing the "never observable" proxy in `p7_score.py`.
- Optional learned segmentation (YOLOv8-seg or a point network) trained with R1 clutter and activity labels, compared with the geometric method.
- **DoD:** P4 evaluation rerun on R2 data with a baseline row so versions can be compared.

### R5. Scoring and uncertainty
- Activity-aware status instead of `height_coverage` only.
- Per-element confidence with `uncertain` as a real output, plus calibration checks.
- Progress estimate with error bars.
- **DoD:** `results/main_table.csv` regenerated with intervals, plus error-vs-completion and error-vs-trajectory curves.

### R6. Real-data reality check
- Run the unchanged pipeline on ConSLAM/SLAM2REF and any Hilti or Buildings 12(10) data.
- Report the sim-to-real gap per metric and feed findings back into R1 and R2.
- **DoD:** gap report exists and names which simulated effects mattered.

### R7. Ablation, generalisation, write-up
- Ablate each realism factor and measure the accuracy drop.
- Run on held-out buildings.
- Regenerate figures, presentation and `CLAUDE.md`.

## Order
R0 -> R2 -> R3 -> R1 -> R4 -> R5 -> R6 -> R7. R1 can run in parallel with R2.

## Open decisions
1. Renderer: Blender/BlenderProc (recommended) vs staying in Open3D with a better sensor model.
2. Compute: GPU availability for rendering and photogrammetry.
3. Scope: learned detector in R4, or only the robust geometric matcher.
