"""Phase 7 -- status assignment and progress scoring.

Per SPEC.md section 13, adapted to what our BIM-matching pipeline (p4_match_bim.py)
actually gives us instead of the image-pipeline's IoU_3D: each detected element
carries a `height_coverage` (observed height / true height, since our simulator's
only partial-completion mechanism is height-clipping walls/columns -- see
p2_ablate.py). That's the direct, correct completeness signal here, playing the
same role SPEC's 3D IoU threshold does:

  - height_coverage >= 0.5           -> completed,    C_i = 1.0
  - 0.15 <= height_coverage < 0.5    -> partial,       C_i = height_coverage
  - not detected, but element IS in the "ever observable" set -> not_started, C_i = 0.0
  - not detected, AND element is NOT in the "ever observable" set -> uncertain, excluded

"ever observable" is approximated from the fully-complete stage's own match
results: our raycaster only traces geometry that actually exists at a given
stage, so there's no direct way to test "would a ray have hit this empty
space" for a not-yet-built element without re-raycasting the complete design
against every stage's camera set. Instead, we use the one stage where every
element DOES exist (the final, 100%-built stage) as a proxy: if an element
is never detected even when fully built and fully raycast, it's a
structurally hard-to-observe element (occluded, small, or just poorly
covered by our exterior-orbit + interior-walkthrough camera scheme) at
EVERY stage, not specifically "not built yet" -- exactly the conflation
SPEC.md section 13 calls "the single easiest way to produce meaningless
progress numbers." Measured on the committed run: ~320-380/1355 elements
(~25%) fall in this set every stage.

`--mask visibility` (ROADMAP R4, the default) replaces that fixed proxy with
a per-stage test from p4_visibility.py: an undetected element counts as
not_started only if, given this stage's cameras and the elements we did
detect, it would have produced at least VISIBLE_MIN_EXPECTED points had it
been built. `--mask final` keeps the old behaviour for comparison.

In the same mode, a detected element is thrown out ("too few points") if it
got fewer than DETECT_MIN_FRACTION of the points p4_visibility.py says it
would give if built and visible (visibility_detected.json); it is then
treated like any undetected element. This removes "detections" made of a
few stray points from a neighbouring surface, which a noisy sensor produces
inside the volume of unbuilt walls. `--min-fraction 0` turns it off.

Progress = sum(w_i * C_i) / sum(w_i) * 100, over non-uncertain elements,
computed with two weightings (count, volume) per SPEC. Ground truth progress
comes directly from each stage's manifest.json (which we have, since this is
simulated data -- a real deployment wouldn't have this for comparison).

Run: .venv/bin/python src/p7_score.py [--mask visibility|final] [--min-fraction F]
Output: data/status/{stage}/status.csv (per-element), results/main_table.csv
(per-stage progress estimate vs ground truth), results/figures/p7_progress_error.png
"""
import argparse
import glob
import json
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BIM_CSV = "data/bim/bim_complete.csv"
STATES_DIR = "data/states"
MATCHES_DIR = "data/matches"
POINTCLOUD_DIR = "data/pointclouds"
STATUS_DIR = "data/status"
OUT_TABLE = "results/main_table.csv"
OUT_FIG = "results/figures/p7_progress_error.png"

COMPLETED_THRESHOLD = 0.5
PARTIAL_THRESHOLD = 0.15
VISIBLE_MIN_EXPECTED = 30   # expected points (if built) needed to call an unseen element not_started
DETECT_MIN_FRACTION = 0.1   # a detection needs at least this share of its expected points (set by --min-fraction)


def score_stage(stage_name, bim_df, well_covered_ids):
    manifest = json.load(open(os.path.join(STATES_DIR, stage_name, "manifest.json"), encoding="utf-8"))
    matches = json.load(open(os.path.join(MATCHES_DIR, stage_name, "matches.json"), encoding="utf-8"))
    match_by_gid = {m["global_id"]: m for m in matches}
    n_rejected = 0
    if well_covered_ids is None:  # per-stage visibility mask
        vis = json.load(open(os.path.join(MATCHES_DIR, stage_name, "visibility.json"), encoding="utf-8"))
        well_covered_ids = {g for g, n in vis.items() if n >= VISIBLE_MIN_EXPECTED}
        det_path = os.path.join(MATCHES_DIR, stage_name, "visibility_detected.json")
        if DETECT_MIN_FRACTION > 0 and os.path.exists(det_path):
            exp_det = json.load(open(det_path, encoding="utf-8"))
            for g, m in list(match_by_gid.items()):
                e = exp_det.get(g, 0.0)
                if m["point_count"] < DETECT_MIN_FRACTION * e:
                    del match_by_gid[g]   # too few points: treat as not detected
                    n_rejected += 1
                    if e >= VISIBLE_MIN_EXPECTED:
                        well_covered_ids = well_covered_ids | {g}

    rows = []
    for _, elem in bim_df.iterrows():
        gid = elem["global_id"]
        m = match_by_gid.get(gid)
        if m is not None and m["height_coverage"] >= COMPLETED_THRESHOLD:
            status, c_i = "completed", 1.0
        elif m is not None and m["height_coverage"] >= PARTIAL_THRESHOLD:
            status, c_i = "partial", float(np.clip(m["height_coverage"], 0, 1))
        elif gid in well_covered_ids:
            status, c_i = "not_started", 0.0
        else:
            status, c_i = "uncertain", None  # excluded from both numerator and denominator
        rows.append({
            "global_id": gid, "ifc_type": elem["ifc_type"], "volume": elem["volume"],
            "status": status, "c_i": c_i,
            "detected": m is not None,
            "height_coverage": m["height_coverage"] if m is not None else 0.0,
        })
    status_df = pd.DataFrame(rows)
    scored = status_df[status_df["status"] != "uncertain"]

    progress_count = 100 * scored["c_i"].sum() / len(scored)
    progress_volume = 100 * (scored["c_i"] * scored["volume"]).sum() / scored["volume"].sum()

    # ground truth, from the manifest directly (only available because this is simulated
    # data) -- scored over the SAME non-uncertain element set as the estimate, so both can
    # reach exactly 100% at the fully-built stage and the comparison is apples-to-apples
    built_ids = set(manifest["kept_ids"])
    partial_frac = {p["global_id"]: p["completion_fraction"] for p in manifest["partial_ids"]}
    scored_bim = bim_df[bim_df["global_id"].isin(scored["global_id"])]
    gt_c = scored_bim["global_id"].map(lambda g: 1.0 if g in built_ids else partial_frac.get(g, 0.0))
    gt_progress_count = 100 * gt_c.sum() / len(scored_bim)
    gt_progress_volume = 100 * (gt_c * scored_bim["volume"]).sum() / scored_bim["volume"].sum()

    # where the error comes from, per element (ground truth is eval-only)
    truth = status_df["global_id"].map(lambda g: "built" if g in built_ids else
                                       ("partial" if g in partial_frac else "none"))
    st = status_df["status"]
    diag = {
        "n_missed_built": int(((truth != "none") & (st == "not_started")).sum()),
        "n_false_built": int(((truth == "none") & st.isin(["completed", "partial"])).sum()),
        "n_uncertain_built": int(((truth != "none") & (st == "uncertain")).sum()),
        "n_uncertain_unbuilt": int(((truth == "none") & (st == "uncertain")).sum()),
    }

    # registration: the method's own verdict, and (eval only) its true error --
    # mean displacement of the stage's points vs where they really belong
    reg_ok, reg_err = None, None
    reg_path = os.path.join(MATCHES_DIR, stage_name, "registration.json")
    if os.path.exists(reg_path):
        reg = json.load(open(reg_path, encoding="utf-8"))
        reg_ok = bool(reg.get("ok", True))
        pc = np.load(os.path.join(POINTCLOUD_DIR, f"{stage_name}.npz"))
        xyz = pc["xyz"][::20].astype(np.float64)
        T_est, T_true_inv = np.array(reg["T_cloud_to_bim"]), np.linalg.inv(pc["gt_T_bim_to_cloud"])
        reg_err = float(np.linalg.norm((xyz @ T_est[:3, :3].T + T_est[:3, 3])
                                       - (xyz @ T_true_inv[:3, :3].T + T_true_inv[:3, 3]), axis=1).mean())

    out_dir = os.path.join(STATUS_DIR, stage_name)
    os.makedirs(out_dir, exist_ok=True)
    status_df.to_csv(os.path.join(out_dir, "status.csv"), index=False)

    return {
        "stage": stage_name,
        "progress_est_count": progress_count,
        "progress_est_volume": progress_volume,
        "progress_gt_count": gt_progress_count,
        "progress_gt_volume": gt_progress_volume,
        "error_count": abs(progress_count - gt_progress_count),
        "error_volume": abs(progress_volume - gt_progress_volume),
        "n_uncertain": int((status_df["status"] == "uncertain").sum()),
        "n_completed": int((status_df["status"] == "completed").sum()),
        "n_partial": int((status_df["status"] == "partial").sum()),
        "n_not_started": int((status_df["status"] == "not_started").sum()),
        "signed_error_count": progress_count - gt_progress_count,
        "registration_ok": reg_ok,
        "registration_err_m": reg_err,
        "n_rejected_few_points": n_rejected,
        **diag,
    }


def main():
    global DETECT_MIN_FRACTION
    ap = argparse.ArgumentParser()
    ap.add_argument("--mask", choices=["visibility", "final"], default="visibility")
    ap.add_argument("--min-fraction", type=float, default=DETECT_MIN_FRACTION)
    args = ap.parse_args()
    DETECT_MIN_FRACTION = args.min_fraction
    bim_df = pd.read_csv(BIM_CSV)
    stage_names = sorted(
        os.path.basename(p) for p in glob.glob(os.path.join(STATES_DIR, "*")) if os.path.isdir(p)
    )

    if args.mask == "final":
        # "ever observable" proxy: elements detected at the final (fully-built) stage
        final_matches = json.load(open(os.path.join(MATCHES_DIR, stage_names[-1], "matches.json"),
                                       encoding="utf-8"))
        well_covered_ids = {m["global_id"] for m in final_matches}
        print(f"Coverage mask: {len(well_covered_ids)}/{len(bim_df)} elements ever detected "
              f"at the fully-built reference stage ({stage_names[-1]})")
    else:
        well_covered_ids = None
        print(f"Coverage mask: per-stage visibility (>= {VISIBLE_MIN_EXPECTED} expected points)")

    rows = [score_stage(s, bim_df, well_covered_ids) for s in stage_names]
    df = pd.DataFrame(rows)
    os.makedirs("results", exist_ok=True)
    df.to_csv(OUT_TABLE, index=False)
    print(df.to_string(index=False))
    print(f"\nWrote {OUT_TABLE}")

    os.makedirs("results/figures", exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    x = np.arange(len(df))

    ax = axes[0]
    ax.plot(x, df["progress_gt_count"], "o-", label="ground truth (count)", color="#4a8f5c")
    ax.plot(x, df["progress_est_count"], "s--", label="estimated (count)", color="#ef8354")
    ax.plot(x, df["progress_gt_volume"], "o-", label="ground truth (volume)", color="#2f6690", alpha=0.5)
    ax.plot(x, df["progress_est_volume"], "s--", label="estimated (volume)", color="#c96a4e", alpha=0.5)
    ax.set_xticks(x)
    ax.set_xticklabels(df["stage"], rotation=60, ha="right", fontsize=7)
    ax.set_ylabel("progress %")
    ax.set_title("Estimated vs. ground-truth progress")
    ax.legend(fontsize=8)

    ax2 = axes[1]
    ax2.plot(x, df["error_count"], "o-", label="count-weighted error", color="#ef8354")
    ax2.plot(x, df["error_volume"], "s-", label="volume-weighted error", color="#c96a4e")
    ax2.set_xticks(x)
    ax2.set_xticklabels(df["stage"], rotation=60, ha="right", fontsize=7)
    ax2.set_ylabel("absolute progress error (pp)")
    ax2.set_title("Progress error vs. construction stage")
    ax2.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(OUT_FIG, dpi=150)
    print(f"Wrote {OUT_FIG}")


if __name__ == "__main__":
    main()
