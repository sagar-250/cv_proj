"""R5 -- calibrate the progress error bars on one run.

p7_score.py puts every scored element in an evidence group (`bin` in
status.csv: status / element family / point-count band, e.g.
"completed/below/wall/r<0.5" = a wall on a storey below the construction
front, called completed with less than half the points the cameras should
have given it). The position matters because a "not started" element below
the front is far more often a missed built one than one above it. On a calibration run, where the truth is
known, this script measures per group how far the pipeline's C_i is from the
true completion gt_c: the mean difference (bias), its spread (var) and the
uncertainty of that mean (se). It also writes pooled rows per status/position
and per status, used for groups with too few elements. p7_score.py then corrects C_i by the bias
and turns var and se into a 90% range on the progress number. On top of that
comes a whole-stage term (`_stage_count`, `_stage_volume`): the rms error of
the corrected progress per stage, measured leave-one-stage-out, because the
errors inside one stage share causes (registration, the camera plan, the
construction front) and do not average out.

Calibrate on one simulated condition and test on another (e.g. calibrate on
the clean realistic run, test on the busy site); calibrating and testing on
the same run would make the ranges look better than they are.

Run: .venv/bin/python src/r5_calibrate.py [--status DIR] [--out CSV]
     (DIR holds {stage}/status.csv from p7_score.py; default data/status)
"""
import argparse
import glob
import os

import numpy as np
import pandas as pd

import p7_score


def groups(df):
    rows = []
    for key, g in list(df.groupby("bin")) + list(df.groupby("status_pos")) + list(df.groupby("status")):
        rows.append({"bin": key, "n": len(g), "bias": g["diff"].mean(),
                     "var": g["diff"].var(ddof=1) if len(g) > 1 else 0.25,
                     "se": g["diff"].std(ddof=1) / len(g) ** 0.5 if len(g) > 1 else 0.5})
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--status", default="data/status")
    ap.add_argument("--out", default="results/r5_calibration.csv")
    args = ap.parse_args()

    parts = []
    for p in sorted(glob.glob(os.path.join(args.status, "*", "status.csv"))):
        d = pd.read_csv(p)
        d["stage"] = os.path.basename(os.path.dirname(p))
        parts.append(d)
    df = pd.concat(parts, ignore_index=True)
    assert "gt_c" in df and "bin" in df, "status.csv is from an older p7_score.py; rerun it"
    df = df[df["status"] != "uncertain"].copy()
    df["diff"] = df["gt_c"] - df["c_i"]

    df["status_pos"] = df["bin"].str.split("/").str[:2].str.join("/")
    out = groups(df)

    # Whole-stage error the groups don't explain (errors within a stage are not
    # independent). Measured leave-one-stage-out: fit the groups on the other
    # stages, correct this one, compare with the truth.
    for name in ("count", "volume"):
        res = []
        for st, d in df.groupby("stage"):
            w = np.ones(len(d)) if name == "count" else d["volume"].to_numpy()
            est, _, _ = p7_score.calibrated(d, w, groups(df[df["stage"] != st]))
            res.append(est - 100 * np.sum(w * d["gt_c"]) / np.sum(w))
        rms = float(np.sqrt(np.mean(np.square(res))))
        print(f"leave-one-stage-out error of the corrected {name} progress: rms {rms:.2f} points")
        out.loc[len(out)] = {"bin": f"_stage_{name}", "n": len(res), "bias": 0.0, "var": (rms / 100) ** 2, "se": 0.0}
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    out.to_csv(args.out, index=False)
    print(f"{len(df)} scored element-stages from {len(parts)} stages in {args.status}")
    print(out.round(4).to_string(index=False))
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
