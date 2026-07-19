#!/usr/bin/env python3
"""Audit whether accepted visual updates drive VIO-minus-FC yaw error."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaw-diag", type=Path, required=True)
    parser.add_argument("--attitude-comparison", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--t0", type=float)
    parser.add_argument("--t1", type=float)
    parser.add_argument(
        "--clip-deg", type=float, nargs="*", default=(0.01, 0.02, 0.05, 0.10),
        help="Per-update SO(3) yaw limits for offline robust-influence audit.")
    args = parser.parse_args()

    yaw = pd.read_csv(args.yaw_diag)
    attitude = pd.read_csv(args.attitude_comparison)
    if args.t0 is not None:
        yaw = yaw[yaw.timestamp >= args.t0]
        attitude = attitude[attitude.camera_time_s >= args.t0]
    if args.t1 is not None:
        yaw = yaw[yaw.timestamp <= args.t1]
        attitude = attitude[attitude.camera_time_s <= args.t1]
    visual = yaw[(yaw.update_type == "MSCKF") & (yaw.dx_yaw_projection_valid == 1)].copy()
    visual["local_cumsum_deg"] = visual.delta_yaw_update_deg.cumsum()
    error_at_update = np.interp(
        visual.timestamp,
        attitude.camera_time_s,
        attitude.so3_error_z_deg,
    )
    visual["vio_minus_fc_so3_yaw_deg"] = error_at_update
    visual["abs_delta_deg"] = visual.delta_yaw_update_deg.abs()
    clip_summary = {}
    for clip_deg in args.clip_deg:
        key = f"clipped_{clip_deg:g}deg"
        visual[key] = visual.delta_yaw_update_deg.clip(-clip_deg, clip_deg)
        visual[f"{key}_cumsum"] = visual[key].cumsum()
        clip_summary[key] = {
            "step_sum_deg": float(visual[key].sum()),
            "removed_step_sum_deg": float(
                visual.delta_yaw_update_deg.sum() - visual[key].sum()),
            "clipped_update_count": int(
                (visual.abs_delta_deg > clip_deg).sum()),
        }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    trace_path = args.output_dir / "visual_yaw_correction_audit.csv"
    visual.to_csv(trace_path, index=False)
    top_path = args.output_dir / "largest_visual_yaw_updates.csv"
    visual.nlargest(30, "abs_delta_deg").to_csv(top_path, index=False)

    time0 = float(visual.timestamp.iloc[0])
    relative = visual.timestamp.to_numpy() - time0
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    axes[0].plot(relative, visual.delta_yaw_update_deg, linewidth=0.8)
    axes[0].axhline(0.0, color="black", linewidth=0.6)
    axes[0].set_ylabel("Accepted visual yaw step / deg")
    axes[0].grid(alpha=0.25)
    axes[1].plot(relative, visual.local_cumsum_deg, label="cumulative accepted visual yaw", linewidth=1.1)
    for clip_deg in args.clip_deg:
        key = f"clipped_{clip_deg:g}deg_cumsum"
        axes[1].plot(relative, visual[key], label=f"clip {clip_deg:g} deg/update", linewidth=0.9)
    axes[1].plot(relative, error_at_update, label="VIO − FC SO(3) yaw error", linewidth=1.1)
    axes[1].set_ylabel("deg")
    axes[1].legend(loc="best")
    axes[1].grid(alpha=0.25)
    axes[2].plot(relative, visual.num_features, label="accepted features", linewidth=0.9)
    axes[2].plot(relative, visual.tracking_feature_count, label="tracking features", linewidth=0.8)
    axes[2].set_ylabel("feature count")
    axes[2].set_xlabel(f"Seconds since {time0:.3f} s")
    axes[2].legend(loc="best")
    axes[2].grid(alpha=0.25)
    fig.suptitle("Accepted MSCKF visual yaw correction versus VIO−FC yaw error")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    plot_path = args.output_dir / "visual_yaw_correction_audit.png"
    fig.savefig(plot_path, dpi=170)
    plt.close(fig)

    correlation = float(np.corrcoef(
        visual.local_cumsum_deg.to_numpy(), error_at_update
    )[0, 1])
    summary = {
        "time_start_s": float(visual.timestamp.iloc[0]),
        "time_end_s": float(visual.timestamp.iloc[-1]),
        "accepted_msckf_update_count": int(len(visual)),
        "visual_yaw_step_sum_deg": float(visual.delta_yaw_update_deg.sum()),
        "visual_yaw_step_median_abs_deg": float(visual.abs_delta_deg.median()),
        "visual_yaw_step_p95_abs_deg": float(visual.abs_delta_deg.quantile(0.95)),
        "visual_yaw_step_max_abs_deg": float(visual.abs_delta_deg.max()),
        "vio_minus_fc_yaw_start_deg": float(error_at_update[0]),
        "vio_minus_fc_yaw_end_deg": float(error_at_update[-1]),
        "correlation_cumulative_visual_vs_vio_fc_yaw": correlation,
        "robust_clip_audit": clip_summary,
        "trace_csv": str(trace_path.resolve()),
        "largest_updates_csv": str(top_path.resolve()),
        "plot": str(plot_path.resolve()),
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
