# BIM-Based Construction Progress Monitoring via Drone Simulation

**Project specification for implementation. Read this file fully before writing code.**

---

## 0. How to use this document

You are implementing a research prototype. The deliverable is working code plus a
results table and a set of plots, not a production system.

Rules of engagement:

- Build in the phase order given. Each phase has a **Definition of Done**. Do not
  start phase N+1 until phase N's DoD passes.
- Every phase writes its output to `data/` or `results/` as files on disk. Nothing
  is passed in memory between phases. This makes the pipeline restartable and
  debuggable.
- Prefer boring, inspectable code. NumPy, pandas, JSON, CSV. No frameworks,
  no config-driven abstraction layers, no premature classes.
- Fixed random seeds everywhere. Every experiment must be exactly reproducible.
- When a phase produces numbers, print them and save them. Do not let results
  exist only in a notebook cell.

---

## 1. Problem and research question

A BIM model represents the **intended** state of a building. It says nothing about
what has actually been built on site. Manual progress inspection does not scale.

**Research question:** can drone-collected imagery automatically determine which
BIM components have been constructed, and from that estimate overall project
progress?

We answer this in **simulation first**, where ground truth is exact by construction,
before touching real site data.

### Why simulation first

On a real site you never know the true completion state, so you cannot separate
detection error from matching error from scoring error. In simulation we control
the completion state exactly, so we can produce error-versus-completion curves and
error-versus-trajectory comparisons that are impossible to produce on real data.
That controllability is the contribution.

---

## 2. Scope

**In scope for the prototype:**

- Structural components only: `IfcColumn`, `IfcBeam`, `IfcWall` /
  `IfcWallStandardCase`, `IfcSlab`.
- Camera pose simulation, not drone flight dynamics. We move a camera along a
  path; we do not simulate rotors, wind, or battery.
- Known pose and known depth from the renderer. No structure-from-motion at this
  stage. This is a deliberate simplification, not an oversight.
- Binary-plus-partial element status, not activity-level percent complete
  (formwork / rebar / concrete). See §11 for how this differs from prior work.

**Explicitly out of scope for the prototype** (record as future work):

- MEP, finishes, doors, windows, furniture.
- Real UAV deployment.
- SfM/MVS reconstruction from unposed imagery.
- Schedule updating or completion-date prediction.

---

## 3. Pipeline overview

```
IFC (complete)
   │
   ├─► [P1] Parse ────────────► bim_complete.csv          (ground truth table)
   │
   ├─► [P2] Ablate ───────────► states/{level}_{seed}/    (incomplete IFC + manifest)
   │
   ├─► [P3] Render ───────────► renders/{state}/{traj}/   (RGB, depth, index, pose)
   │
   ├─► [P4] Detect ───────────► detections/               (masks + class + score)
   │
   ├─► [P5] Localise ─────────► observations/             (3D objects, world coords)
   │
   ├─► [P6] Match ────────────► matches/                  (obs_id → GlobalId)
   │
   ├─► [P7] Score ────────────► status/                   (per-element C_i, progress %)
   │
   └─► [P8] Evaluate ─────────► results/                  (metrics, plots)
```

Phase 9 is an optional reality check against real drone data.

---

## 4. Data

### 4.1 Primary dataset: Schependomlaan

Dutch residential project, TU Eindhoven / RAAMAC. Released for scientific and
academic use.

**Important:** the canonical repo `openBIMstandards/DataSetSchependomlaan` has been
emptied and now only contains a README pointing elsewhere. Its old release-zip link
is dead. The complete dataset survives in forks:

- `https://github.com/jakob-beetz/DataSetSchependomlaan`
- `https://github.com/JimmyTUM/DataSetSchependomlaan`

IFC files only (clean, canonical copies) are at the buildingSMART sample repo:
`https://github.com/buildingSMART/Sample-Test-Files` under `IFC 2x3/Schependomlaan`.

Clone with `git clone --depth 1`. **After cloning, check file sizes.** Some forks
store large binaries as Git LFS pointer stubs (~130 bytes). If the point clouds are
stubs, either install `git-lfs` and re-pull, or proceed without them since phases
1-8 do not need them.

Contents relevant to us:

| Path | What it is | We use it for |
|---|---|---|
| `Design model IFC/IFC Schependomlaan.ifc` | Complete as-designed model from Archicad | **`BIM_complete`. The single most important file.** |
| `As Planned models/Week {26,27,28,29,30,37} ...ifc` | Synchro-exported as-planned states at six dates | Optional real timestep sequence (see caveat below) |
| `Planning/` | Schedule in PDF and XML | Construction-order priors for ablation |
| `Point Clouds/` | Drone SfM clouds, weeks 26-30, PLY / ASCII / some LAS | Phase 9 only |
| `Comparison As Planned As Built/comparison_all weeks.xlsx` | RAAMAC's algorithm output per element per week | Phase 9 reference. **Not verified ground truth** |
| `Event log/` | As-planned and as-built event logs, CSV / XLSX | Not used in prototype |

**Caveat on the as-planned IFCs:** the repo README documents (with figures) that the
Synchro export mangles the storey tree and merges original property sets into a
single `SynchroResourcePropertyset`. Whether GlobalIds survive the round-trip is
unverified. Phase 0 tests this. Always parse geometry from the **design** IFC.

**Caveat on building type:** low-rise Dutch residential. Walls, slabs and roofs
dominate. Exposed columns and beams may be sparse. Count them in Phase 0 and adjust
the class list if a class has too few instances to train on.

### 4.2 Secondary IFC models, for generalisation

`https://github.com/buildingSMART/Sample-Test-Files` — many IFC 2x3 and IFC4 models.
Pick two or three with different typologies (an office frame with real columns and
beams would complement Schependomlaan well). Hold at least one out entirely as a
test building, never used for tuning.

### 4.3 Real-site datasets, phase 9 and future work

- **ConSLAM** — `https://github.com/mac137/ConSLAM` — real indoor construction site,
  RGB + LiDAR sequences with ground-truth poses, plus TLS reference clouds and a
  derived BIM. The best option if you later want real data *with* poses.
- **Buildings 12(10) 1754** — `https://www.mdpi.com/2075-5309/12/10/1754` — open
  access, six BIM + laser-scan pairs, including variants where walls and columns
  were deliberately removed from the cloud to mimic incomplete construction.
- **BIMNet** — `https://github.com/LydJason/BIMNet` — large IFC-linked point cloud
  dataset, access by request form.

### 4.4 What is NOT available

The Pal et al. 2023 paper (Automation in Construction 157, 105157) that motivated
this project releases **no data**. Its statement is "Data will be made available on
request." Neither its case-study BIMs/point clouds nor its 2458-image segmentation
dataset are public. Do not plan around obtaining them.

---

## 5. Environment

```bash
python -m venv .venv && source .venv/bin/activate
pip install ifcopenshell numpy scipy pandas matplotlib \
            open3d scikit-learn opencv-python OpenEXR trimesh tqdm
pip install ultralytics          # YOLOv8-seg, phase 4 track B
```

**Renderer: use Blender, not Isaac Sim.** Blender gives scripted camera paths,
Cycles/EEVEE rendering, and depth + object-index passes, which is everything phases
3 through 5 need. Isaac Sim plus PegasusSimulator costs a week of setup and only
buys flight dynamics, which are explicitly out of scope. Record this as a deliberate
decision. Revisit only in the extensions phase.

- Blender 4.x, run headless: `blender --background --python script.py`
- Bonsai (`bonsaibim.org`) is the Blender IFC add-on. Useful for visual inspection.
  For the pipeline, prefer scripted `ifcopenshell` + `ifcopenshell.geom` so runs are
  reproducible from the command line.

Repo layout:

```
.
├── SPEC.md                  (this file)
├── src/
│   ├── p1_parse.py
│   ├── p2_ablate.py
│   ├── p3_render.py         (run under Blender's python)
│   ├── p4_detect.py
│   ├── p5_localise.py
│   ├── p6_match.py
│   ├── p7_score.py
│   ├── p8_evaluate.py
│   ├── trajectories.py
│   └── geom.py              (shared bbox / IoU / projection helpers)
├── data/
│   ├── raw/                 (cloned datasets, gitignored)
│   ├── bim/
│   ├── states/
│   └── renders/             (gitignored, large)
├── results/
└── configs/experiment.yaml
```

---

## 6. Phase 0 — Data sanity check

Do this before anything else. Thirty minutes, saves a week.

```python
import ifcopenshell
f = ifcopenshell.open("data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc")
for t in ["IfcWall","IfcWallStandardCase","IfcColumn","IfcBeam","IfcSlab",
          "IfcDoor","IfcWindow","IfcBuildingStorey"]:
    print(f"{t:24s} {len(f.by_type(t))}")

# Does GlobalId survive the Synchro export?
g = ifcopenshell.open("data/raw/schependomlaan/As Planned models/"
                      "Week 28 10 july IFC Schependomlaan incl planningsdata.ifc")
a = {e.GlobalId for e in f.by_type("IfcProduct")}
b = {e.GlobalId for e in g.by_type("IfcProduct")}
print("overlap", len(a & b), "| design", len(a), "| week28", len(b))
```

**Decision rule:** if overlap is high, you have a free real multi-timestep ground
truth; use the weekly IFCs as additional evaluation states alongside your synthetic
ones. If overlap is near zero, the export reassigned IDs. Do not fight it. Use only
the design IFC and your own synthetic states, and note the limitation.

**Definition of Done:** element counts per class printed and recorded in
`results/p0_inventory.md`; GlobalId overlap decision recorded.

---

## 7. Phase 1 — Parse the BIM

**Input:** design IFC.
**Output:** `data/bim/bim_complete.csv`.

For every element of an in-scope class, extract:

| Column | Notes |
|---|---|
| `global_id` | IFC GlobalId. The primary key for the whole project |
| `ifc_type` | Normalise `IfcWallStandardCase` → `IfcWall` |
| `storey` | From the spatial containment relation |
| `centroid_x/y/z` | World coordinates, metres |
| `aabb_min_x/y/z`, `aabb_max_x/y/z` | Axis-aligned bounding box, world frame |
| `obb_dims_l/w/h`, `obb_rot_z` | Oriented box. Walls are long and thin; AABB alone is a poor descriptor for a wall at 45° |
| `volume` | Mesh volume, m³. Used for volume-weighted progress |
| `elevation_bottom` | For construction-order sorting |

Use `ifcopenshell.geom` with `settings.set(settings.USE_WORLD_COORDS, True)` so all
geometry is already in the project frame. Compute the OBB from the mesh vertices via
PCA in the XY plane, keeping Z world-aligned (buildings are gravity-aligned; a full
3D OBB adds noise).

**Definition of Done:** CSV with one row per in-scope element, no NaNs, volumes all
positive, and a matplotlib top-down scatter of centroids that visually looks like a
floor plan.

---

## 8. Phase 2 — Generate incomplete states

**Input:** `bim_complete.csv`, design IFC.
**Output:** `data/states/{level}_{seed}/` each containing `kept_ids.json`,
`manifest.json`, and either a filtered IFC or a kept-ID list for the renderer.

### Critical design point: do not remove elements at random

Random removal produces floating slabs and walls resting on nothing. A detector
trained on that learns an unrealistic prior, and the resulting accuracy numbers mean
nothing. Remove elements respecting construction logic.

**Build order:**

1. Sort by storey, bottom-up.
2. Within a storey: columns → beams → slabs → walls.
3. Within a category: sort by a stable spatial key (e.g. x then y) so removal is a
   contiguous work front rather than scattered holes.
4. To reach completion level `p`, keep the first `round(p × N)` elements in this
   build order.
5. Add controlled jitter: swap a small fraction (5-10%) of adjacent pairs in the
   ordering, seeded, so states are not perfectly deterministic. This is what the
   seed varies.

Generate levels **25, 50, 75, 90, 100 percent**, with **3 seeds each** = 15 states.
Write `manifest.json` per state:

```json
{
  "level": 0.5, "seed": 1,
  "n_total": 412, "n_kept": 206,
  "kept_ids": ["1a2B...", "..."],
  "removed_ids": ["..."],
  "counts_by_type": {"IfcWall": 120, "IfcColumn": 14, ...}
}
```

This manifest **is the ground truth**. Everything in phase 8 is measured against it.

**Definition of Done:** 15 state directories; for each, a rendered preview image
confirming the building looks like a plausible partially-built structure with no
floating geometry.

---

## 9. Phase 3 — Render

**Input:** a state, a trajectory type.
**Output:** `data/renders/{state}/{traj}/frame_{nnnn}.{png,exr,idx.png,json}`.

### Object index trick

Assign every element a unique Blender `pass_index` derived deterministically from
its GlobalId (e.g. an incrementing integer stored in a `global_id → index` lookup
saved as JSON). Enable the **Object Index pass**. The renderer then gives you
per-pixel component identity for free. This gives you:

- perfect segmentation labels for training in phase 4 track B,
- an oracle detector for phase 4 track A,
- exact visibility ground truth for the coverage metric.

Do not skip this. It is the single highest-leverage thing in the pipeline.

### Passes to write, per frame

| File | Content |
|---|---|
| `frame_0007.png` | RGB, 1920×1080 |
| `frame_0007.exr` | Z depth in metres, float32. **EXR, not PNG.** PNG quantises depth to 8 or 16 bits and destroys back-projection accuracy |
| `frame_0007.idx.png` | Object index pass, 16-bit single channel |
| `frame_0007.json` | Intrinsics (fx, fy, cx, cy, width, height), extrinsics (4×4 world-to-camera), visible GlobalIds with pixel counts |

**Coordinate convention:** state it once in `geom.py` and never deviate. Blender's
camera looks down its local −Z with +Y up. Convert to an OpenCV-style convention
(+Z forward, +Y down) at write time and record the conversion in the docstring. More
bugs in this project will come from this than from anything else.

### Trajectories (`trajectories.py`)

Each returns a list of `(position, look_at)` or `(position, quaternion)`.

| Name | Definition |
|---|---|
| `grid` | Lawnmower raster at 3 altitude bands, camera pitched 45° down |
| `perimeter` | Circular orbit at radius 1.5-2× footprint diagonal, 3 altitudes, camera aimed at building centroid |
| `waypoint` | Hand-authored path: one standoff view per façade plus one nadir pass |
| `adaptive` | Two-stage. Run `perimeter` at low density, run phases 4-6, find elements with zero or low coverage, then generate targeted viewpoints normal to those elements' largest faces and re-render |

Defaults: 60-70% frame overlap, 24mm-equivalent focal length, 100-300 frames per
state per trajectory. Make frame count a parameter and hold it roughly constant
across trajectories so comparisons are fair.

The `adaptive` trajectory is a genuine contribution. Implement it last but do
implement it.

**Definition of Done:** for one state, depth back-projection of a known element's
mask centroid lands within 2 cm of that element's centroid in `bim_complete.csv`.
Write this as an assertion test. If it fails, the coordinate convention is wrong.

---

## 10. Phase 4 — Detection

Run **two tracks** and report both. The gap between them is itself a result.

**Track A, oracle.** Use the object-index pass directly as the detector output.
Perfect masks, perfect classes. This isolates matching and scoring error from
detection error and gives an upper bound on achievable accuracy. Implement this
first; it lets you build and debug phases 5-8 immediately without training anything.

**Track B, learned.** Train an instance segmentation model on the rendered RGB, using
index-pass masks as labels. Four classes: column, beam, wall, slab.

- YOLOv8-seg (`ultralytics`) is the pragmatic choice. Mask R-CNN with a ResNet-101
  backbone matches the comparison literature if you need that for the writeup.
- **Split by scene, not by frame.** Frames from the same state and trajectory are
  near-duplicates; a random frame split leaks and inflates scores by a lot. Hold out
  entire states, and ideally an entire building.
- Augment: horizontal flip, brightness/contrast/hue jitter, random crop. Do not
  vertical-flip; buildings are gravity-oriented.
- Report mAP at IoU 0.5 and 0.75, per class and overall.

**Sim-to-real note:** rendered imagery has no clutter, no scaffolding, no workers,
no material stacking. A model trained purely on renders will degrade sharply on real
photos. Do not claim otherwise. If you want to soften this, add domain randomisation
(random HDRI lighting, material roughness jitter, scattered distractor objects) and
report with-and-without.

**Definition of Done:** track A produces masks identical to the index pass by
construction; track B reports mAP on a held-out state with the split documented.

---

## 11. Phase 5 — 3D localisation

**Input:** detections + depth + pose.
**Output:** `observations/{state}_{traj}.json` — a list of fused 3D objects.

Per detection, per frame:

1. Take depth values over the mask. Use the **median**, and reject pixels more than
   2 median-absolute-deviations from it, to kill silhouette-edge pixels that sample
   background depth. This one filter matters more than anything else in this phase.
2. Back-project surviving pixels to camera coords via the intrinsics, then to world
   coords via the inverse extrinsics.
3. Store the resulting point set, its centroid, its AABB and OBB, the class, the
   detection score, and the source frame id.

Then fuse across frames:

- Cluster detection centroids with DBSCAN (`eps` ≈ 0.5 m, `min_samples` = 2), or
  merge greedily where 3D AABB IoU > 0.3 **and** classes agree.
- For each cluster, union the point sets and recompute centroid, AABB, OBB, and an
  estimated volume (use a voxel occupancy count at 5 cm resolution rather than a
  convex hull; hulls massively overestimate for L-shaped or partial observations).
- Record `n_views` per observation. Low `n_views` correlates with bad localisation
  and is a useful confidence signal in phase 7.

**Definition of Done:** on track A, at 100% completion, the number of fused
observations is within 5% of the number of visible elements, and mean centroid error
is under 10 cm.

---

## 12. Phase 6 — Matching

**Input:** observations + `bim_complete.csv`.
**Output:** `matches/{state}_{traj}.csv` — `obs_id, global_id, score, decision`.

Build a cost matrix over (observation × BIM element) with a weighted score:

```
score = w1 · exp(-d_centroid / τ)        # τ ≈ 1.0 m
      + w2 · IoU_3D(obb_obs, obb_bim)
      + w3 · 1[class_obs == class_bim]
      + w4 · dim_similarity(dims_obs, dims_bim)
```

where `dim_similarity = min(l_o/l_b, l_b/l_o) · min(w_o/w_b, ...) · ...`.

Start with `w = [0.3, 0.4, 0.2, 0.1]`. Tune on **one building**, report on another.

Solve with `scipy.optimize.linear_sum_assignment` (Hungarian), not greedy
nearest-neighbour. Greedy fails badly on repetitive structures like a row of
identical columns, which is exactly what buildings are made of. After solving,
reject any assigned pair whose score falls below a threshold `θ`; those observations
become unmatched (spurious detections).

Gate the cost matrix first: only consider pairs within, say, 5 m and of compatible
class. This turns an O(N·M) matrix into something sparse and fast.

**Definition of Done:** on track A at 100% completion, matching precision and recall
both exceed 0.95. If not, the problem is in phase 5 or the coordinate convention,
not in the matching weights. Do not tune weights to paper over an upstream bug.

---

## 13. Phase 7 — Status assignment and scoring

Per BIM element, assign exactly one status:

| Status | Condition | `C_i` |
|---|---|---|
| `completed` | Matched, 3D IoU ≥ 0.5 | 1.0 |
| `partial` | Matched, IoU in [0.15, 0.5) | `clip(vol_observed / vol_expected, 0, 1)` |
| `not_started` | Unmatched, but visible from ≥ 1 viewpoint per the index-pass visibility ground truth | 0.0 |
| `uncertain` | Unmatched **and** never visible from any viewpoint | excluded |

**The `uncertain` exclusion is the most important rule in this phase.** An element
the drone never looked at must not be scored as "not built." Exclude it from both
numerator and denominator, and report the exclusion rate as the coverage metric.
Scoring never-observed elements as zero silently conflates "not built" with "not
looked at," which is the single easiest way to produce meaningless progress numbers.

Aggregate:

```
Progress = Σ(w_i · C_i) / Σ(w_i) × 100     over non-uncertain elements
```

Run with two weightings and report both:

- `w_i = 1` (element count), as in the proposal.
- `w_i = volume_i`, which is closer to how earned value actually works and is
  noticeably more stable when element sizes vary.

**Definition of Done:** `status/{state}_{traj}.csv` with per-element status and
`C_i`, plus a single progress percentage and a coverage percentage.

---

## 14. Phase 8 — Evaluation

**Metrics**, computed per (completion level × seed × trajectory × detection track):

| Metric | Definition |
|---|---|
| Accuracy | correctly classified elements / total non-uncertain |
| Precision / Recall / F1 | of the `completed` class |
| Progress error | `abs(progress_est − progress_gt)` in percentage points |
| Localisation error | mean and 95th percentile centroid distance, matched elements, cm |
| Coverage | % of elements visible from ≥ 1 viewpoint |
| Confusion matrix | over {completed, partial, not_started} |

`progress_gt` comes from the state manifest: `n_kept / n_total` for `w=1`, or the
volume ratio for `w=volume`.

**Required plots** (`results/figures/`):

1. **Progress error vs completion level**, one line per trajectory, error bars across
   seeds. *This is the headline figure of the project.* No prior work can produce it,
   because no prior work has exact ground truth at controlled completion levels.
2. Coverage vs trajectory, bar chart.
3. Oracle (track A) vs learned (track B) accuracy, grouped bars. The gap quantifies
   how much of the remaining error is a detection problem versus a geometry problem.
4. Localisation error CDF.
5. Confusion matrix heatmap at 50% completion.

**Required table:** `results/main_table.csv`, one row per configuration, all metrics
as columns. This is what goes in the report.

**Expected finding to look for:** error should be highest at intermediate completion
levels (around 50%), where partial elements and occlusion are worst, and lowest at
0% and 100%. If your curve is flat, something is wrong — probably the `uncertain`
exclusion is masking failures.

---

## 15. Phase 9 — Reality check (optional but strongly recommended)

The first question any reviewer asks is "does this transfer to real data?" Answer it
cheaply rather than hand-waving.

Take Schependomlaan's real drone point cloud for week 28 and the as-planned IFC for
the same week. Skip phases 3-5 entirely. Instead:

1. Register cloud to IFC: manual picking of 3+ correspondences for a coarse 7-DOF
   similarity transform, then ICP for fine alignment (`open3d`). Expect a few cm
   residual. Report it.
2. Occupancy check in the Braun et al. style: count cloud points inside each
   element's AABB, dilated by a registration tolerance ε (start at 3 cm). Classify
   as built if density exceeds ~1000 points/m² of surface area.
3. Run phases 7 and 8 on that output.

Then compare against `comparison_all weeks.xlsx`. **State clearly that this
spreadsheet is RAAMAC's algorithm output, not human-verified ground truth.** You are
measuring agreement between two methods. If you want real ground truth, hand-label a
random subset of 50 elements from the drone imagery and report against that instead.

This phase takes days, not months, and enormously strengthens the writeup.

---

## 16. Positioning against prior work

This project is **not** a reimplementation of Pal et al. 2023 (ALPMS). Differences:

| | ALPMS | This project |
|---|---|---|
| Granularity | Schedule activity (formwork, rebar, concrete, tiles) | BIM element (is this column built) |
| Output | % complete from mask area ÷ face area | Per-element status + weighted overall progress |
| As-built source | Real sites, SfM/MVS, unknown pose | Simulated render, known pose and depth |
| Hard problem | Synthesising an orthographic face view when no camera has one (projective transform or NeRF) | Detection, 3D localisation, matching to IFC IDs |
| Ground truth | Manually annotated masks, ~500 elements | Programmatic ablation, exact by construction |
| Drone | Passive data source, manual capture | Trajectory is an experimental variable |

Our lineage is the **occupancy-based element-level** line, not the activity-level
line. Cite and, where practical, implement as baselines:

- Bosché & Haas 2008; Bosché, Haas & Akinci 2009 — scan-vs-BIM, retrieval of 3D CAD
  objects in range images. The origin of the approach.
- Turkan et al. 2012, 2013 — 4D schedule + 3D sensing; percent complete as
  recognised ÷ expected objects. **This is literally our `w=1` formula.** The
  closest prior formulation of our scoring rule.
- Braun et al. 2020, *Autom. Constr.* 116:103210 — fuses point clouds, semantic data
  and computer vision. Source of the 1000 points/m² occupancy threshold. **Closest
  methodological ancestor; implement as the phase 9 baseline.**
- Son & Kim 2010; Son, Kim & Cho 2017 — colour + 3D structural component recognition
  and automated schedule updates.
- Golparvar-Fard et al. 2009 (D4AR) — 4D model over time-lapse photos. Crude,
  foundational, very readable.
- Han & Golparvar-Fard 2015 — appearance-based material classification on
  BIM-registered patches. The binary method ALPMS criticises.
- Kropp, Koch & König 2018 — interior state recognition with 4D-BIM-registered image
  sequences.
- Hamledari, McCabe & Davari 2017 — under-construction indoor partition components,
  hand-crafted CV.
- Omar, Mahdjoubi & Kheder 2018; Puri & Turkan 2020 — partial completion via point
  count and density between as-planned and as-built clouds. **Crude, purely
  geometric, cheap to implement. Worth adding as a second baseline.**

All DOIs are in the Pal et al. reference list.

**Anticipated criticism:** "why not activity-level percent complete?" The answer,
which should appear in the proposal and the report: we isolate the estimation and
matching problem under exact ground truth before layering on appearance-based
activity reasoning. Activity-level is named in the extensions.

---

## 17. Risks and honest limitations

Record these in the report. Do not let a reviewer find them first.

1. **Sim-to-real gap.** Renders have no clutter, scaffolding, workers, weather, or
   material stacking. Phase 4 track B numbers are an optimistic upper bound. Phase 9
   partially addresses this; domain randomisation helps a little.
2. **Ablation realism.** Removing whole elements produces a clean binary present /
   absent world. Real sites have half-poured walls and elements at intermediate
   states that no IFC subset can represent. Our `partial` status is a crude proxy.
3. **Known pose is a strong assumption.** It removes the entire SfM/registration
   error budget, which is a dominant error source in real deployments. This is a
   deliberate scoping choice and must be stated as such, not buried.
4. **Single building typology.** Schependomlaan is low-rise residential. Hold out a
   second building of different typology or the generalisation claim is unsupported.
5. **Comparison spreadsheet is not ground truth.** See §15.
6. **Class imbalance.** If columns and beams are scarce in Schependomlaan, per-class
   metrics for those classes will be noisy. Report support counts alongside every
   per-class number.

---

## 18. Suggested order of work

| Order | Work | Why |
|---|---|---|
| 1 | Phase 0 | Kills bad assumptions in 30 minutes |
| 2 | Phase 1 | Everything downstream keys off this table |
| 3 | Phase 3 for one 100% state, `perimeter` only | Get the depth back-projection assertion passing. This is the riskiest technical step; do it early |
| 4 | Phase 4 track A (oracle) | Free detector, unblocks everything |
| 5 | Phases 5, 6, 7 | Full pipeline end to end on one state |
| 6 | Phase 8 on that one state | Confirms metrics code works |
| 7 | Phase 2, all 15 states | Now scale out |
| 8 | Phase 3, all states × all trajectories | The long compute job. Run overnight |
| 9 | Phase 8 full sweep, headline figure | First real result |
| 10 | Phase 4 track B | Add the learned detector, rerun the sweep |
| 11 | `adaptive` trajectory | The novel bit |
| 12 | Phase 9 | Reality check |
| 13 | Second building | Generalisation |

Get one complete vertical slice working end to end before scaling anything
horizontally. A working pipeline over one state beats four half-built phases.

---

## 19. Conventions

- All lengths in metres, all angles in radians internally; convert only for display.
- All world coordinates in the IFC project frame. Never introduce a second frame.
- GlobalId is the universal key. Never use array indices or filenames as identity.
- Every script takes `--state`, `--traj`, `--out` and is runnable standalone.
- Every script writes a `_meta.json` beside its output recording the git commit,
  timestamp, and full argument list. Reproducibility is not optional in research code.
- Seeds: set `numpy`, `random`, and torch seeds from a single `--seed` argument.