#!/usr/bin/env python3
"""Relate historical visual yaw steps to causal FC/gyro agreement.

Besides the original agreement-bin audit, this script evaluates a causal
directional guard.  The evidence state is a leaky integral of individually
clipped visual-yaw updates.  It therefore ignores isolated spikes and detects
the sustained same-sign feedback that distinguishes fly1/fly3 from fly2/fly4.
No GPS or trajectory truth is read by this script.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-yaw-diag", type=Path, required=True)
    parser.add_argument("--guard-diag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--t0", type=float, required=True)
    parser.add_argument("--t1", type=float, required=True)
    parser.add_argument("--bias-tau-s", type=float, default=30.0)
    parser.add_argument("--bias-threshold-deg", type=float, default=0.25)
    parser.add_argument("--evidence-clip-deg", type=float, default=0.05)
    parser.add_argument("--step-cap-deg", type=float, default=0.10)
    parser.add_argument("--directional-scale", type=float, default=0.5)
    parser.add_argument(
        "--update-types", nargs="+", default=["MSCKF", "SLAM"],
        help="visual update types contributing to the causal evidence",
    )
    parser.add_argument("--vio-fc-attitude-csv", type=Path)
    parser.add_argument("--reference-error-tau-s", type=float, default=5.0)
    parser.add_argument("--reference-error-threshold-deg", type=float, default=0.5)
    args = parser.parse_args()

    yaw = pd.read_csv(args.baseline_yaw_diag)
    yaw = yaw[
        (yaw.timestamp >= args.t0)
        & (yaw.timestamp <= args.t1)
        & yaw.update_type.isin(args.update_types)
        & (yaw.dx_yaw_projection_valid == 1)
    ].copy().sort_values("timestamp")
    guard = pd.read_csv(args.guard_diag, comment="#").sort_values("camera_time_s")
    merged = pd.merge_asof(
        yaw,
        guard,
        left_on="timestamp",
        right_on="camera_time_s",
        direction="nearest",
        tolerance=0.02,
    )
    merged["matched_guard"] = merged.camera_time_s.notna()
    matched = merged[merged.matched_guard].copy()
    edges = [-np.inf, 0.10, 0.20, 0.30, 0.35, 0.50, 1.0, np.inf]
    labels = ["<=0.10", "0.10-0.20", "0.20-0.30", "0.30-0.35",
              "0.35-0.50", "0.50-1.00", ">1.00"]
    matched["agreement_bin_deg"] = pd.cut(
        matched.imu_fc_so3_error_deg, bins=edges, labels=labels)
    grouped = matched.groupby("agreement_bin_deg", observed=False).agg(
        update_count=("delta_yaw_update_deg", "size"),
        yaw_step_sum_deg=("delta_yaw_update_deg", "sum"),
        yaw_step_mean_deg=("delta_yaw_update_deg", "mean"),
        yaw_step_median_abs_deg=("delta_yaw_update_deg", lambda s: s.abs().median()),
        yaw_step_p95_abs_deg=("delta_yaw_update_deg", lambda s: s.abs().quantile(0.95)),
        imu_fc_error_median_deg=("imu_fc_so3_error_deg", "median"),
    ).reset_index()

    thresholds = (0.10, 0.15, 0.20, 0.25, 0.30, 0.35)
    scales = (0.0, 0.5, 0.8, 0.9)
    counterfactual = []
    raw_sum = float(matched.delta_yaw_update_deg.sum())
    for threshold in thresholds:
        selected = matched.imu_fc_so3_error_deg <= threshold
        selected_sum = float(matched.loc[selected, "delta_yaw_update_deg"].sum())
        for scale in scales:
            total = raw_sum - (1.0 - scale) * selected_sum
            counterfactual.append({
                "agreement_threshold_deg": threshold,
                "guarded_scale": scale,
                "selected_update_count": int(selected.sum()),
                "selected_raw_yaw_sum_deg": selected_sum,
                "counterfactual_total_yaw_sum_deg": total,
            })
    counterfactual_df = pd.DataFrame(counterfactual)

    # Causal robust evidence: decay the prior bias to the current update, use
    # that prior state for the decision, then add a clipped copy of the raw
    # update.  An update is attenuated only when it continues an already
    # established same-sign bias and FC/gyro agree over the camera interval.
    matched = matched.sort_values("timestamp").copy()
    evidence = 0.0
    last_t: float | None = None
    evidence_before: list[float] = []
    directional_selected: list[bool] = []
    robust_scale: list[float] = []
    applied_step: list[float] = []
    for row in matched.itertuples(index=False):
        timestamp = float(row.timestamp)
        if last_t is not None:
            dt = max(0.0, timestamp - last_t)
            evidence *= float(np.exp(-dt / args.bias_tau_s))
        evidence_before.append(evidence)
        raw_step = float(row.delta_yaw_update_deg)
        same_direction = (
            abs(evidence) >= args.bias_threshold_deg
            and raw_step * evidence > 0.0
        )
        selected = bool(
            same_direction
            and float(row.imu_fc_so3_error_deg) <= 0.35
        )
        scale = args.directional_scale if selected else 1.0
        # Huber-like cap is applied to the current-yaw component only.  It is
        # represented here as an additional gain scale so the online Joseph
        # covariance update can use exactly the same effective gain.
        if abs(raw_step) > args.step_cap_deg:
            scale = min(scale, args.step_cap_deg / abs(raw_step))
        directional_selected.append(selected)
        robust_scale.append(scale)
        applied_step.append(scale * raw_step)
        evidence += float(np.clip(
            raw_step, -args.evidence_clip_deg, args.evidence_clip_deg))
        last_t = timestamp

    matched["visual_yaw_bias_before_deg"] = evidence_before
    matched["directional_selected"] = directional_selected
    matched["robust_directional_scale"] = robust_scale
    matched["counterfactual_applied_yaw_step_deg"] = applied_step
    matched["counterfactual_cumsum_yaw_deg"] = np.cumsum(applied_step)
    directional_summary = {
        "bias_tau_s": args.bias_tau_s,
        "bias_threshold_deg": args.bias_threshold_deg,
        "evidence_clip_deg": args.evidence_clip_deg,
        "step_cap_deg": args.step_cap_deg,
        "directional_scale": args.directional_scale,
        "selected_update_count": int(np.count_nonzero(directional_selected)),
        "spike_capped_update_count": int(np.count_nonzero(
            np.abs(matched.delta_yaw_update_deg.to_numpy()) > args.step_cap_deg)),
        "raw_yaw_step_sum_deg": float(matched.delta_yaw_update_deg.sum()),
        "counterfactual_yaw_step_sum_deg": float(np.sum(applied_step)),
        "final_visual_yaw_bias_deg": float(evidence),
    }

    reference_summary = None
    if args.vio_fc_attitude_csv is not None:
        reference = pd.read_csv(args.vio_fc_attitude_csv).sort_values("camera_time_s")
        raw_error = reference.so3_error_z_deg.to_numpy(dtype=float)
        ref_time = reference.camera_time_s.to_numpy(dtype=float)
        filtered_error = np.empty_like(raw_error)
        filtered_error[0] = raw_error[0]
        for index in range(1, len(raw_error)):
            dt = max(0.0, ref_time[index] - ref_time[index - 1])
            alpha = 1.0 - np.exp(-dt / args.reference_error_tau_s)
            innovation = np.clip(raw_error[index] - filtered_error[index - 1], -1.0, 1.0)
            filtered_error[index] = filtered_error[index - 1] + alpha * innovation
        reference["filtered_so3_error_z_deg"] = filtered_error
        matched = pd.merge_asof(
            matched.sort_values("timestamp"),
            reference[["camera_time_s", "so3_error_z_deg", "filtered_so3_error_z_deg"]],
            left_on="timestamp", right_on="camera_time_s", direction="nearest",
            tolerance=0.05, suffixes=("", "_reference"),
        )
        error = matched.filtered_so3_error_z_deg.to_numpy(dtype=float)
        step = matched.delta_yaw_update_deg.to_numpy(dtype=float)
        agree = matched.imu_fc_so3_error_deg.to_numpy(dtype=float) <= 0.35
        valid = np.isfinite(error)
        # With the repository's left attitude error, a positive state yaw
        # correction moves Log(R_FC^-1 R_VIO) yaw approximately negative.
        worsens_minus = valid & agree & (
            np.abs(error) >= args.reference_error_threshold_deg
        ) & (np.abs(error - step) > np.abs(error))
        worsens_plus = valid & agree & (
            np.abs(error) >= args.reference_error_threshold_deg
        ) & (np.abs(error + step) > np.abs(error))
        matched["reference_error_directional_selected"] = worsens_minus
        reference_summary = {
            "error_tau_s": args.reference_error_tau_s,
            "error_threshold_deg": args.reference_error_threshold_deg,
            "repository_minus_convention_count": int(np.count_nonzero(worsens_minus)),
            "repository_minus_convention_raw_sum_deg": float(step[worsens_minus].sum()),
            "opposite_plus_convention_count": int(np.count_nonzero(worsens_plus)),
            "opposite_plus_convention_raw_sum_deg": float(step[worsens_plus].sum()),
            "filtered_error_median_abs_deg": float(np.nanmedian(np.abs(error))),
            "filtered_error_p95_abs_deg": float(np.nanpercentile(np.abs(error), 95)),
        }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    matched.to_csv(args.output_dir / "matched_visual_updates.csv", index=False)
    grouped.to_csv(args.output_dir / "agreement_bin_summary.csv", index=False)
    counterfactual_df.to_csv(
        args.output_dir / "counterfactual_yaw_sum_sweep.csv", index=False)
    time_rel = matched.timestamp.to_numpy() - args.t0
    nrows = 4 if reference_summary is not None else 3
    fig, axes = plt.subplots(nrows, 1, figsize=(14, 3 * nrows), sharex=True)
    axes[0].plot(time_rel, matched.delta_yaw_update_deg, color="0.55",
                 linewidth=0.7, label="raw visual yaw update")
    axes[0].plot(time_rel, matched.counterfactual_applied_yaw_step_deg,
                 color="tab:blue", linewidth=0.8,
                 label="robust directional applied")
    axes[0].axhline(args.step_cap_deg, color="tab:red", linestyle="--", linewidth=0.8)
    axes[0].axhline(-args.step_cap_deg, color="tab:red", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("yaw step [deg]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(time_rel, matched.visual_yaw_bias_before_deg,
                 color="tab:orange", linewidth=1.0,
                 label="causal clipped leaky bias")
    axes[1].axhline(args.bias_threshold_deg, color="tab:red", linestyle="--", linewidth=0.8)
    axes[1].axhline(-args.bias_threshold_deg, color="tab:red", linestyle="--", linewidth=0.8)
    axes[1].set_ylabel("bias evidence [deg]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)
    axes[2].plot(time_rel, matched.imu_fc_so3_error_deg,
                 color="tab:green", linewidth=0.9, label="gyro-bg vs FC SO(3)")
    axes[2].axhline(0.35, color="tab:red", linestyle="--", linewidth=0.8,
                    label="agreement gate")
    axes[2].set_ylabel("SO(3) error [deg]")
    axes[2].set_xlabel(f"time since {args.t0:.1f} s [s]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)
    if reference_summary is not None:
        axes[3].plot(time_rel, matched.so3_error_z_deg, color="0.65",
                     linewidth=0.6, label="raw VIO-FC SO(3) yaw error")
        axes[3].plot(time_rel, matched.filtered_so3_error_z_deg,
                     color="tab:purple", linewidth=1.1,
                     label="causal robust low-pass error")
        chosen = matched.reference_error_directional_selected.to_numpy(dtype=bool)
        axes[3].scatter(time_rel[chosen],
                        matched.filtered_so3_error_z_deg.to_numpy()[chosen],
                        s=7, color="tab:red", label="would worsen error")
        axes[3].axhline(args.reference_error_threshold_deg, color="tab:red",
                        linestyle="--", linewidth=0.8)
        axes[3].axhline(-args.reference_error_threshold_deg, color="tab:red",
                        linestyle="--", linewidth=0.8)
        axes[3].set_ylabel("VIO-FC yaw [deg]")
        axes[3].set_xlabel(f"time since {args.t0:.1f} s [s]")
        axes[3].legend(loc="upper right")
        axes[3].grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(args.output_dir / "directional_visual_yaw_guard.png", dpi=180)
    plt.close(fig)
    summary = {
        "visual_update_count": int(len(yaw)),
        "matched_guard_count": int(len(matched)),
        "raw_matched_yaw_step_sum_deg": raw_sum,
        "agreement_bins": grouped.to_dict(orient="records"),
        "directional_guard": directional_summary,
        "reference_error_guard": reference_summary,
        "outputs": {
            "matched": str((args.output_dir / "matched_visual_updates.csv").resolve()),
            "bins": str((args.output_dir / "agreement_bin_summary.csv").resolve()),
            "counterfactual": str((args.output_dir / "counterfactual_yaw_sum_sweep.csv").resolve()),
            "curve": str((args.output_dir / "directional_visual_yaw_guard.png").resolve()),
        },
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
