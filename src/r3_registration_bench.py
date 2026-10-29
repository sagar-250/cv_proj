"""R3 benchmark -- how often does registration land the cloud on the BIM?

Takes the Phase 3 stage clouds (which already carry a small known offset,
gt_T_bim_to_cloud) and adds a further seeded random offset per trial, in two
regimes modelling what a real survey looks like before registration:

  gnss : drone photogrammetry georeferenced by on-board GNSS + compass, no
         ground control. Translation ~N(0, 2 m) horizontal / N(0, 1 m)
         vertical, heading +/-15 deg, tilt ~N(0, 0.5 deg) (IMU knows
         gravity), scale ~N(1, 1%). Registration uses prior="gnss".
  local: indoor SLAM / LiDAR in its own start frame (no GNSS indoors). Any
         heading, up to 30 m translation, tilt ~N(0, 0.5 deg), true metric
         scale (LiDAR measures metres). Registration uses no prior.

(An earlier draft had a "nogcp" regime with arbitrary pose AND +/-10% scale;
dropped as unrealistic -- every survey drone has GNSS, and a sensor with no
GNSS is a LiDAR/SLAM rig whose scale is already metric.)

Each trial is scored by the mean displacement of BIM surface points after
mapping BIM -> cloud -> estimated BIM (0 = perfect). Success = under
SUCCESS_M. Compared methods: `icp_old` (the plain ICP that p4_match_bim.py
used), `fpfh` and `plan` (src/registration.py's two coarse methods, same
robust ICP refinement). Besides the success rate, the key number is SILENT
failures: wrong results the method did not flag (info["ok"] True). Those
must be zero -- a flagged failure can be handled, a silent one corrupts the
progress numbers.

Stopping and resuming: each finished trial is written to the CSV right away.
`--resume` loads that CSV and skips trials already in it (the offsets come
from fixed seeds, so the skipped trials are exactly the ones done before).
`--from-log FILE` first rebuilds the CSV from a run's printed log, for runs
stopped before this was added (fitness/tilt/ambiguity are not in the log, so
those columns are blank for such rows; the summary does not use them).
Only resume with the same --clouds, --trials and registration code.

Run: .venv/bin/python src/r3_registration_bench.py [--trials N] [--methods icp_old,fpfh,plan] [--tag _x] [--clouds DIR] [--resume] [--from-log FILE]
Output: results/r3_registration_bench.csv, results/r3_registration_summary.md
"""
import argparse
import os
import re
import sys
import time

import numpy as np
import open3d as o3d
import pandas as pd
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(__file__))
import registration  # noqa: E402
from p2_p3_simulate_stages import survey_offset as random_offset  # noqa: E402

SAMPLE_CACHE = "data/bim/bim_surface_samples.npz"
POINTCLOUD_DIR = "data/pointclouds"
OUT_CSV = "results/r3_registration_bench.csv"
OUT_MD = "results/r3_registration_summary.md"
STAGES = ["00_m1_fundering_structure", "01_m1_fundering_envelope", "03_00_begane_grond_envelope",
          "07_01_eerste_verdieping_interior", "11_03_derde_verdieping_structure",
          "15_04_dak_envelope"]
SUCCESS_M = 0.05
SEED = 0

# old method's constants, copied from p4_match_bim.py as it was before R3
OLD_COARSE = 0.5
OLD_CORR = 0.3


def apply(T, xyz):
    return xyz @ T[:3, :3].T + T[:3, 3]


def icp_old(xyz, tree, icp_target):
    dist, _ = tree.query(xyz, k=1, workers=-1)
    mask = dist < OLD_COARSE
    if mask.sum() < 100:
        return np.eye(4)
    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(xyz[mask])
    res = o3d.pipelines.registration.registration_icp(
        src, icp_target, OLD_CORR, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
    return res.transformation


LOG_LINE = re.compile(r"^(\S+)\s+(gnss|local)\s+#(\d+)\s+(\S+)\s+err=\s*([\d.]+) m\s+(OK|FAIL)\s+"
                      r"flagged=(True|False)\s*(.*?)\s*\(([\d.]+)s\)$")


def rows_from_log(path):
    """Rebuild CSV rows from the per-trial lines a run printed (stage names are cut to 24 chars)."""
    rows = []
    for line in open(path):
        mt = LOG_LINE.match(line.rstrip("\n"))
        if not mt:
            continue
        short, regime, k, m, err, _, flagged, checks, secs = mt.groups()
        stage = [s for s in STAGES if s.startswith(short)]
        assert len(stage) == 1, short
        rows.append({"stage": stage[0], "regime": regime, "trial": int(k), "method": m,
                     "err_m": float(err), "success": float(err) < SUCCESS_M, "secs": float(secs),
                     "self_ok": flagged == "False", "fitness": np.nan, "tilt_deg": np.nan,
                     "ambiguity": np.nan, "failed_checks": checks})
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10, help="offsets per stage per regime")
    ap.add_argument("--methods", default="icp_old,fpfh,plan")
    ap.add_argument("--tag", default="", help="suffix for the output files")
    ap.add_argument("--clouds", default=POINTCLOUD_DIR, help="stage cloud dir (default data/pointclouds)")
    ap.add_argument("--resume", action="store_true", help="skip trials already in the output CSV")
    ap.add_argument("--from-log", help="rebuild the output CSV from this log first (implies --resume)")
    args = ap.parse_args()
    out_csv = OUT_CSV.replace(".csv", f"{args.tag}.csv")
    out_md = OUT_MD.replace(".md", f"{args.tag}.md")
    methods = args.methods.split(",")
    os.makedirs("results", exist_ok=True)

    rows = []
    if args.from_log:
        rows = rows_from_log(args.from_log)
        pd.DataFrame(rows).to_csv(out_csv, index=False)
        print(f"rebuilt {len(rows)} trials from {args.from_log} -> {out_csv}")
    elif args.resume and os.path.exists(out_csv):
        rows = pd.read_csv(out_csv, keep_default_na=False, na_values=[""]).to_dict("records")
        for r in rows:
            r["failed_checks"] = "" if pd.isna(r["failed_checks"]) else r["failed_checks"]
        print(f"resuming: {len(rows)} trials already in {out_csv}")
    done = {(r["stage"], r["regime"], int(r["trial"]), r["method"]) for r in rows}

    bim = np.load(SAMPLE_CACHE)["xyz"].astype(np.float64)
    rng = np.random.default_rng(SEED)
    probe = bim[rng.choice(bim.shape[0], 20_000, replace=False)]

    tree = icp_target = target = None
    if "icp_old" in methods:
        tree = cKDTree(bim)
        icp_target = o3d.geometry.PointCloud()
        icp_target.points = o3d.utility.Vector3dVector(bim[rng.choice(bim.shape[0], 2_000_000, replace=False)])
    if "fpfh" in methods or "plan" in methods:
        t0 = time.time()
        target = registration.BimTarget(bim, seed=SEED, with_fpfh="fpfh" in methods)
        print(f"BIM target prepared in {time.time() - t0:.1f}s")

    for stage in STAGES:
        d = np.load(os.path.join(args.clouds, f"{stage}.npz"))
        xyz0 = d["xyz"].astype(np.float64)
        T_true = d["gt_T_bim_to_cloud"]      # eval only
        center = xyz0.mean(axis=0)
        for regime in ["gnss", "local"]:
            trial_rng = np.random.default_rng([SEED, STAGES.index(stage), regime == "local"])
            for k in range(args.trials):
                O = random_offset(regime, trial_rng, center)
                xyz = apply(O, xyz0)
                T_bim_to_obs = O @ T_true
                for m in methods:
                    if (stage, regime, k, m) in done:
                        continue
                    t0 = time.time()
                    if m == "icp_old":
                        T_est, info = icp_old(xyz, tree, icp_target), {}
                    else:
                        T_est, info = registration.register(
                            xyz, target, method=m, with_scale=(regime == "gnss"), seed=k,
                            prior="gnss" if regime == "gnss" else "none", bim_xyz=bim)
                    secs = time.time() - t0
                    err = np.linalg.norm(apply(T_est @ T_bim_to_obs, probe) - probe, axis=1).mean()
                    rows.append({"stage": stage, "regime": regime, "trial": k, "method": m,
                                 "err_m": err, "success": err < SUCCESS_M, "secs": secs,
                                 "self_ok": info.get("ok", True), "fitness": info.get("fitness", np.nan),
                                 "tilt_deg": info.get("tilt_deg", np.nan),
                                 "ambiguity": info.get("ambiguity", np.nan),
                                 "failed_checks": info.get("failed_checks", "")})
                    print(f"{stage[:24]:24s} {regime:5s} #{k:02d} {m:8s} err={err:8.3f} m  "
                          f"{'OK ' if err < SUCCESS_M else 'FAIL'} flagged={not info.get('ok', True)} "
                          f"{info.get('failed_checks', '')} ({secs:.1f}s)", flush=True)
                    pd.DataFrame(rows).to_csv(out_csv, index=False)   # save as we go, so a stop loses nothing

    df = pd.DataFrame(rows)
    df = df[df["method"].isin(methods) & (df["trial"] < args.trials)].reset_index(drop=True)
    df["success"] = df["success"].astype(bool)
    df["self_ok"] = df["self_ok"].astype(bool)

    # "self_ok" = the method's own claim it succeeded; only useful if it agrees with reality.
    # icp_old has no self-check, so every one of its failures is silent.
    df["silent_fail"] = ~df["success"] & df["self_ok"]
    df["flagged_fail"] = ~df["success"] & ~df["self_ok"]
    df["false_alarm"] = df["success"] & ~df["self_ok"]
    summ = df.groupby(["method", "regime"]).agg(
        trials=("success", "size"), success_rate=("success", "mean"),
        silent_fails=("silent_fail", "sum"), flagged_fails=("flagged_fail", "sum"),
        false_alarms=("false_alarm", "sum"),
        median_err_m=("err_m", "median"), mean_secs=("secs", "mean")).reset_index()
    by_stage = df.groupby(["method", "stage"])["success"].mean().unstack(0)
    lines = ["# R3 registration benchmark\n",
             f"Success = mean BIM-point displacement < {SUCCESS_M} m. "
             f"{args.trials} offsets per stage per regime, {len(STAGES)} stages.\n",
             "silent_fails = wrong AND not flagged (must be 0); flagged_fails = wrong but the method "
             "said so; false_alarms = right but flagged.\n",
             summ.to_markdown(index=False, floatfmt=".3f"), "\n## Success rate by stage\n",
             by_stage.to_markdown(floatfmt=".2f")]
    open(out_md, "w").write("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))


if __name__ == "__main__":
    main()
