# Media: pictures, animations and 3D files

Everything here is made by one script, `src/make_media.py`, from the pipeline's own output files. To rebuild it after the pipeline changes, run:

```
.venv/bin/python src/make_media.py            # everything, about 1 minute
.venv/bin/python src/make_media.py --only charts,errors   # just some parts
```

Images are PNG, animations are GIF (they play in any browser or image viewer), and 3D files are PLY (see [How to open the 3D files](#how-to-open-the-3d-files)).

"Old pipeline" means the code as it was in the first commit (`d823a2c`). "New" or "current" means the pipeline after the changes listed in `docs/PROGRESS.md`.

---

## 00. The big picture

| File | What it shows |
|---|---|
| [pipeline_flow.png](00_overview/pipeline_flow.png) | Every step from the design model to the progress number. It also marks which steps are new (green) and which we changed (orange). |

## 01. Results in numbers (old vs new)

| File | What it shows |
|---|---|
| [progress_error_summary.png](01_results_charts/progress_error_summary.png) | **The headline.** How far off the progress % is, for each version of the pipeline. The average went from 4.8 points to 0.8, and the worst stage from 7.9 to 3.6. |
| [progress_error_per_stage_all_versions.png](01_results_charts/progress_error_per_stage_all_versions.png) | The same error, stage by stage, with one line per version. The old pipeline was always too low, because it missed half-built walls. |
| [progress_estimate_vs_truth_old_vs_new.png](01_results_charts/progress_estimate_vs_truth_old_vs_new.png) | The estimated progress curve next to the true one, for the old and the current pipeline. |
| [error_sources_current.png](01_results_charts/error_sources_current.png) | For the current pipeline, which elements it gets wrong at each stage: built ones it called "not started", unbuilt ones it called "built", and built ones it left out as "unsure". |
| [registration_error_per_stage_current.png](01_results_charts/registration_error_per_stage_current.png) | How well each scan was lined up with the design model. It's within about 1–3 cm at every stage, even though the scans started metres off. |

## 02. Ground truth: the construction stages

| File | What it shows |
|---|---|
| [complete_design_model.png](02_ground_truth_stages/complete_design_model.png) | The full design model (BIM) we start from, coloured by element type. |
| [all_16_stages.png](02_ground_truth_stages/all_16_stages.png) | The 16 stages we simulate: built elements are blue, half-built ones orange, and the rest of the building is a light ghost. |
| [construction_growth.gif](02_ground_truth_stages/construction_growth.gif) | **Animation:** the building going up, stage by stage. |

## 03. The simulated scans (what the "drone" and "indoor scanner" see)

| File | What it shows |
|---|---|
| [survey_camera_positions.png](03_simulated_scans/survey_camera_positions.png) | Where the cameras stand: drone photos in rings around the building, plus 360-degree scans inside each floor. Next to it are the points they collect. |
| [all_16_scans.png](03_simulated_scans/all_16_scans.png) | The point cloud of every stage. The colours show the element type, brown is ground and green is trees. |
| [scan_growth.gif](03_simulated_scans/scan_growth.gif) | **Animation:** the scans stage by stage. |
| [final_scan_turntable.gif](03_simulated_scans/final_scan_turntable.gif) | **Animation:** the last stage's scan turning around. |
| [sensor_simple_vs_realistic_slice.png](03_simulated_scans/sensor_simple_vs_realistic_slice.png) | The simple sensor model next to the new realistic one, seen from above as a thin slice through the walls. The realistic one has noise that grows with distance, holes, missing surfaces seen at a slant, and junk points (red). |

## 04. Fix: half-built walls used to vanish (before / after)

| File | What it shows |
|---|---|
| [one_wall_before_after.png](04_fix_half_built_walls/one_wall_before_after.png) | One wall that is 48% built. The old code deleted every face that reached above the cut, so only the bottom face was left and the scanner couldn't see the wall. The new code cuts the wall down to that height. |
| [half_built_walls_stage_before_after.png](04_fix_half_built_walls/half_built_walls_stage_before_after.png) | The same thing for a whole stage. The note on each side shows how much surface of the half-built (orange) walls was left for the scanner. |

## 05. Registration: putting the scan on top of the design (before / after)

| File | What it shows |
|---|---|
| [before_after_alignment.png](05_registration/before_after_alignment.png) | A top view of the first-floor walls. Before: the scan (red) is about 1 m off and slightly turned, as with GPS. After: it sits on the design (grey). The bottom row zooms into 4 x 4 m. |
| [how_the_search_works.png](05_registration/how_the_search_works.png) | How the new method finds the position without GPS. It keeps only wall points, makes a floor plan, and tries every turn, keeping the one where the two floor plans overlap best. Here it found 221 deg when the truth was 223 deg, and the fine-tuning step then fixes the last 2 deg. |
| [benchmark_methods.png](05_registration/benchmark_methods.png) | The test bench. The old ICP got 0% right. Feature matching got 30–40%. The new floor-plan search got 97–98%. It failed on 4 of 170 trials: 2 times it flagged the failure itself, and 2 times it was a near miss (7 and 9 cm, against a 5 cm pass mark) that it didn't flag. |

## 06. Visibility check: "not built" or just "not seen"?

| File | What it shows |
|---|---|
| [visibility_07.png](06_visibility_check/visibility_07.png), [visibility_11.png](06_visibility_check/visibility_11.png) | When an element isn't found, the new pipeline asks whether the cameras would have seen it if it were built. If yes, it's "not started" (pink, see-through). If not, it's "unsure" (dark grey) and left out of the progress number. The truth is shown alongside. The old pipeline guessed this from the final stage instead, which is cheating. |

## 07. Old vs new, element by element

| File | What it shows |
|---|---|
| [all_stages_old_vs_new.gif](07_old_vs_new_per_element/all_stages_old_vs_new.gif) | **Animation:** for every stage, the truth, the old pipeline's answer and the new one's. Green means right, red means a built element called "not started", purple means an unbuilt one called "built", and yellow means a built one left out as "unsure". |
| [stage_03](07_old_vs_new_per_element/stage_03_old_vs_new.png), [stage_07](07_old_vs_new_per_element/stage_07_old_vs_new.png), [stage_11](07_old_vs_new_per_element/stage_11_old_vs_new.png), [stage_15](07_old_vs_new_per_element/stage_15_old_vs_new.png) | Single frames for four stages. For example, at stage 07 the old pipeline missed 104 built elements and the new one misses 12. |

## 08. 3D files (turn them around yourself)

| File | What it is |
|---|---|
| [design_model_by_type.ply](08_3d_models/design_model_by_type.ply) | The full design model, coloured by element type. |
| `stage_07_truth.ply`, `stage_15_truth.ply` | What is really built at that stage: blue built, orange half built (cut to its real height). |
| `stage_07_old_pipeline_errors.ply`, `stage_07_new_pipeline_errors.ply` (and the same for 15) | The old and new pipeline's answers, in the same colours as section 07. Open both and compare. |
| `stage_07_scan_as_received.ply` | The scan as it arrives, metres off from the design. |
| `stage_07_scan_aligned_to_bim.ply`, `stage_15_scan_aligned_to_bim.ply` | The scan after our registration moved it onto the design. Open it together with a truth or error file and they line up. |

### How to open the 3D files

- **In the browser:** drag the `.ply` file onto <https://3dviewer.net>. It opens the file on your own computer and doesn't upload it.
- **Desktop apps:** MeshLab or CloudCompare (both free). These can also show several files together, for example a scan plus a truth model.
- **Windows:** 3D Viewer opens PLY files directly.

## 09. Busy construction site (R1, new)

| File | What it shows |
|---|---|
| [busy_site_world.png](09_busy_site/busy_site_world.png) | The busier site the scanner can now see (`p3_render.py --site busy`). It adds scaffolding along the walls, a tower crane, a site fence, shoring poles under the newest floor, material piles and workers. A few elements (pink) are built 4–12 cm away from where the design says. None of this is in the BIM, so the pipeline has to cope with it. |

## Other viewers already in the project

- `src/pointcloud_viewer.html`: an interactive point-cloud viewer in the browser for all stages. It reads `results/pointcloud_viewer_data.js`.
- `src/segmentation_viewer.html`: the older segmentation baseline.
- `results/presentation/`: the mid-review slides and their screenshots.
