#!/usr/bin/env python3
"""Audit causal filtered VIO-FC yaw-reference dwell times from rosfree logs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def runs_above(time: np.ndarray, value: np.ndarray, threshold: float) -> list[dict]:
    active = np.abs(value) >= threshold
    runs: list[dict] = []
    start = None
    previous = None
    for index, is_active in enumerate(active):
        contiguous = previous is None or time[index] - previous <= 0.65
        if is_active and (start is None or not contiguous):
            if start is not None:
                runs.append({"start_s": float(time[start]), "end_s": float(previous)})
            start = index
        elif not is_active and start is not None:
            runs.append({"start_s": float(time[start]), "end_s": float(previous)})
            start = None
        previous = time[index]
    if start is not None:
        runs.append({"start_s": float(time[start]), "end_s": float(time[-1])})
    for run in runs:
        run["duration_s"] = run["end_s"] - run["start_s"]
    return runs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vio-yaw-diag", type=Path, required=True)
    parser.add_argument("--guard-diag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--flight", required=True)
    parser.add_argument("--thresholds-deg", type=float, nargs="+",
                        default=[0.5, 1.0, 1.5, 2.0])
    args = parser.parse_args()

    all_data = pd.read_csv(args.vio_yaw_diag)
    data = all_data[
        all_data.update_type.isin(["MSCKF", "SLAM", "SLAM_DELAYED"])
        & (all_data.dx_yaw_projection_valid == 1)
    ].copy()
    data = data.sort_values("timestamp").drop_duplicates("timestamp", keep="first")
    time = data.timestamp.to_numpy(dtype=float)
    error = data.guard_reference_error_before_deg.to_numpy(dtype=float)
    rows: list[dict] = []
    summary: dict[str, object] = {
        "flight": args.flight,
        "valid_visual_timestamps": int(len(data)),
        "excluded_invalid_visual_rows": int(
            np.count_nonzero(
                all_data.update_type.isin(["MSCKF", "SLAM", "SLAM_DELAYED"])
                & (all_data.dx_yaw_projection_valid != 1)
            )
        ),
        "thresholds": {},
    }
    for threshold in args.thresholds_deg:
        runs = runs_above(time, error, threshold)
        for run in runs:
            rows.append({"threshold_deg": threshold, **run})
        durations = [run["duration_s"] for run in runs]
        summary["thresholds"][str(threshold)] = {
            "run_count": len(runs),
            "max_dwell_s": max(durations, default=0.0),
            "runs_ge_5s": sum(value >= 5.0 for value in durations),
            "runs_ge_10s": sum(value >= 10.0 for value in durations),
            "runs_ge_15s": sum(value >= 15.0 for value in durations),
        }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.output_dir / "reference_error_dwell_runs.csv", index=False)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    fig, axis = plt.subplots(figsize=(15, 5))
    if args.guard_diag is not None:
        raw = pd.read_csv(args.guard_diag, comment="#")
        raw = raw[
            np.isfinite(raw.vio_fc_relative_yaw_error_deg)
        ].sort_values("camera_time_s")
        raw_time = raw.camera_time_s.to_numpy(dtype=float)
        raw_error = raw.vio_fc_relative_yaw_error_deg.to_numpy(dtype=float)
        axis.plot(raw_time, raw_error, color="0.72", linewidth=0.65,
                  label="raw VIO-FC relative yaw error")
        raw_step = np.abs(np.diff(raw_error))
        filtered_step = np.abs(np.diff(error))
        summary["spike_suppression"] = {
            "raw_abs_step_p95_deg": float(np.percentile(raw_step, 95)),
            "filtered_abs_step_p95_deg": float(np.percentile(filtered_step, 95)),
            "raw_abs_step_max_deg": float(np.max(raw_step)),
            "filtered_abs_step_max_deg": float(np.max(filtered_step)),
        }
    axis.plot(time, error, color="tab:blue", linewidth=1.1,
              label="causal robust filtered VIO-FC yaw error")
    for threshold in args.thresholds_deg:
        axis.axhline(threshold, linestyle="--", linewidth=0.7,
                     label=f"+/-{threshold:g} deg")
        axis.axhline(-threshold, linestyle="--", linewidth=0.7)
    axis.set_xlabel("camera time [s]")
    axis.set_ylabel("relative yaw error [deg]")
    axis.set_title(f"{args.flight}: filtered VIO-FC yaw-reference dwell")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", ncol=3)
    fig.tight_layout()
    fig.savefig(args.output_dir / "reference_error_dwell.png", dpi=180)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
