"""Phase 2 -- generate realistic incomplete BIM states with ground-truth
manifests, replacing the earlier flat-fraction ablation.

Milestone model (per storey, bottom-up by mean elevation_bottom):
  1. structure  = columns + beams + slab poured for this storey
  2. envelope   = exterior walls erected
  3. interior   = interior partition walls built
This matches both the real schedule (Planning/XML: slab -> reinforcement ->
pour -> exterior walls/blockwork -> facade, repeated per floor, roof last)
and real construction practice (envelope closes before partitions go in).
A storey without a given category (e.g. foundation has no interior walls,
roof has none either) simply skips that milestone -- the count of stages
falls out of the data, it isn't a fixed target.

This is what makes "one floor's shell is done but its interior walls
haven't been built yet" a real, named intermediate state instead of a
special case: it's exactly the boundary between a storey's envelope and
interior milestones.

Within a milestone, elements are ordered by (x, y) for a contiguous work
front, then 8% of adjacent pairs are swapped (seeded) so the build/not-built
boundary isn't a razor-sharp line -- real work fronts are ragged.

Partial completion: newly-appearing walls/columns in a stage have a 50%
chance of being height-clipped to a random fraction (0.25-0.85) of their
final height -- "poured/erected but not finished" -- rather than a clean
binary present/absent.

Output per stage: data/states/{NN}_{storey}_{phase}/manifest.json, the
ground truth every later phase (matching, scoring) is evaluated against.

Run: .venv/Scripts/python.exe src/p2_ablate.py
"""
import json
import os

import numpy as np
import pandas as pd
import ifcopenshell
import ifcopenshell.geom

IFC_PATH = "data/raw/schependomlaan/Design model IFC/IFC Schependomlaan.ifc"
CSV_PATH = "data/bim/bim_complete.csv"
STATES_DIR = "data/states"

SEED = 0
ORDER_JITTER_FRAC = 0.08
PARTIAL_CHANCE = 0.5
PARTIAL_HEIGHT_RANGE = (0.25, 0.85)
RESOLVE_CHANCE = 0.45  # chance an existing partial element gets finished each later stage

PHASE_TYPES = {
    "structure": ["IfcColumn", "IfcBeam", "IfcSlab"],
    "envelope": ["IfcWall:ext"],
    "interior": ["IfcWall:int"],
}
PHASE_ORDER = ["structure", "envelope", "interior"]


def slugify(storey_name):
    return storey_name.strip().replace(" ", "_").replace("-", "m")


def build_milestones(df, rng):
    """Returns a list of (storey, phase, element_ids_in_this_milestone)."""
    storey_order = df.groupby("storey")["elevation_bottom"].mean().sort_values().index.tolist()

    milestones = []
    for storey in storey_order:
        sub = df[df["storey"] == storey]
        for phase in PHASE_ORDER:
            types = PHASE_TYPES[phase]
            if phase == "structure":
                group = sub[sub["ifc_type"].isin(types)]
            elif phase == "envelope":
                group = sub[(sub["ifc_type"] == "IfcWall") & (sub["is_external"] == True)]  # noqa: E712
            else:
                group = sub[(sub["ifc_type"] == "IfcWall") & (sub["is_external"] == False)]  # noqa: E712
            if len(group) == 0:
                continue
            ordered = group.sort_values(["centroid_x", "centroid_y"])
            ids = ordered["global_id"].tolist()

            n_swaps = int(len(ids) * ORDER_JITTER_FRAC)
            for _ in range(n_swaps):
                i = rng.integers(0, max(len(ids) - 1, 1))
                if len(ids) > 1:
                    ids[i], ids[i + 1] = ids[i + 1], ids[i]

            milestones.append((storey, phase, ids))
    return milestones


def main():
    df = pd.read_csv(CSV_PATH)
    n_total = len(df)
    rng = np.random.default_rng(SEED)

    milestones = build_milestones(df, rng)
    print(f"{len(milestones)} milestones computed from {n_total} elements:")
    for storey, phase, ids in milestones:
        print(f"  {storey:24s} {phase:10s} +{len(ids)}")

    os.makedirs(STATES_DIR, exist_ok=True)
    cumulative_built = []
    partial_state = {}  # global_id -> completion_fraction, persists across stages
    type_by_id = dict(zip(df["global_id"], df["ifc_type"]))

    for i, (storey, phase, new_ids) in enumerate(milestones):
        stage_rng = np.random.default_rng(SEED + 1000 + i)

        # catch-up: some previously-partial elements get finished this stage
        # (real crews return and finish work started earlier)
        for gid in list(partial_state.keys()):
            if stage_rng.random() < RESOLVE_CHANCE:
                del partial_state[gid]

        cumulative_built.extend(new_ids)
        is_final_stage = (i == len(milestones) - 1)

        # THIS stage's new elements get a chance at partial completion
        # (never on the final stage -- by design-completion, nothing is
        # still mid-pour)
        if not is_final_stage:
            for gid in new_ids:
                if type_by_id[gid] in ("IfcWall", "IfcColumn") and stage_rng.random() < PARTIAL_CHANCE:
                    partial_state[gid] = float(stage_rng.uniform(*PARTIAL_HEIGHT_RANGE))
        else:
            partial_state.clear()

        kept_ids = [g for g in cumulative_built if g not in partial_state]
        removed_ids = [g for g in df["global_id"] if g not in cumulative_built]

        counts_by_type = (
            df[df["global_id"].isin(cumulative_built)]["ifc_type"].value_counts().to_dict()
        )

        stage_name = f"{i:02d}_{slugify(storey)}_{phase}"
        stage_dir = os.path.join(STATES_DIR, stage_name)
        os.makedirs(stage_dir, exist_ok=True)

        manifest = {
            "stage_index": i,
            "storey_milestone": storey,
            "phase": phase,
            "seed": SEED,
            "n_total": n_total,
            "n_built_cumulative": len(cumulative_built),
            "n_fully_built": len(kept_ids),
            "n_partial": len(partial_state),
            "n_not_started": len(removed_ids),
            "kept_ids": kept_ids,
            "partial_ids": [{"global_id": g, "completion_fraction": f} for g, f in partial_state.items()],
            "removed_ids": removed_ids,
            "counts_by_type_cumulative": counts_by_type,
        }
        with open(os.path.join(stage_dir, "manifest.json"), "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)

        print(f"stage {i:02d} [{stage_name}]: built={len(cumulative_built)}/{n_total} "
              f"(fully={len(kept_ids)}, partial={len(partial_state)})")

    print(f"\nWrote {len(milestones)} stage manifests under {STATES_DIR}/")


if __name__ == "__main__":
    main()
