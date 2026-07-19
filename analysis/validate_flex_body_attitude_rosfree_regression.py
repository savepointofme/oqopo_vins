#!/usr/bin/env python3
"""Validate and plot output-only flex attitude from full ROS-free replays."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


REPO_ROOT = Path(__file__).resolve().parents[1]
FLIGHTS = ("fly1", "fly2", "fly3", "fly4")


def windows_to_local(path: str) -> Path:
    return Path(path)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def wrap_deg(values: np.ndarray) -> np.ndarray:
    return (np.asarray(values, dtype=float) + 180.0) % 360.0 - 180.0


def percentile_metrics(values: np.ndarray) -> dict[str, float]:
    absolute = np.abs(np.asarray(values, dtype=float))
    return {
        "median_deg": float(np.median(absolute)),
        "p95_deg": float(np.percentile(absolute, 95.0)),
    }


def load_flex(path: Path) -> pd.DataFrame:
    table = pd.read_csv(path, comment="#")
    if table.empty:
        raise RuntimeError(f"empty flex output: {path}")
    return table


def load_fc(path: Path) -> tuple[np.ndarray, Rotation, np.ndarray]:
    table = pd.read_csv(path, comment="#")
    times = table.camera_time_s.to_numpy(dtype=float)
    quaternions = table[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    valid = table.valid.to_numpy(dtype=int).astype(bool)
    return times, Rotation.from_quat(quaternions), valid


def evaluate_flight(
    flight: str,
    run_dir: Path,
    historical_dir: Path,
    fc_path: Path,
    windows: list[list[float]],
    plot_path: Path,
) -> tuple[dict[str, Any], pd.DataFrame]:
    table = load_flex(run_dir / "aircraft_body_attitude.csv")
    times = table.camera_time_s.to_numpy(dtype=float)
    fc_times, fc_rotations, fc_valid = load_fc(fc_path)
    overlap = (times >= fc_times[0]) & (times <= fc_times[-1])
    if not np.any(overlap):
        raise RuntimeError(f"{flight}: no FC/output overlap")
    interpolated_fc = Slerp(fc_times, fc_rotations)(times[overlap])
    fc_yaw = np.full(len(table), np.nan)
    fc_yaw[overlap] = np.degrees(
        np.unwrap(interpolated_fc.as_euler("xyz", degrees=False)[:, 2])
    )
    nominal_yaw = np.degrees(
        np.unwrap(np.radians(table.d455_nominal_body_yaw_deg.to_numpy(dtype=float)))
    )
    body_yaw = np.degrees(
        np.unwrap(np.radians(table.aircraft_body_yaw_deg.to_numpy(dtype=float)))
    )
    baseline_residual = np.full(len(table), np.nan)
    shadow_residual = np.full(len(table), np.nan)
    baseline_residual[overlap] = wrap_deg(nominal_yaw[overlap] - fc_yaw[overlap])
    shadow_residual[overlap] = wrap_deg(body_yaw[overlap] - fc_yaw[overlap])
    table["direct_fc_yaw_deg"] = fc_yaw
    table["unwrapped_nominal_body_yaw_deg"] = nominal_yaw
    table["unwrapped_shadow_body_yaw_deg"] = body_yaw
    table["baseline_fc_relative_yaw_error_deg"] = baseline_residual
    table["shadow_fc_relative_yaw_error_deg"] = shadow_residual
    dt = np.diff(times, prepend=np.nan)
    flex = table.output_flex_yaw_deg.to_numpy(dtype=float)
    table["filtered_output_rate_deg_s"] = np.divide(
        np.diff(flex, prepend=flex[0]),
        dt,
        out=np.zeros(len(table)),
        where=np.isfinite(dt) & (dt > 0.0),
    )

    event_mask = np.zeros(len(table), dtype=bool)
    events: list[dict[str, Any]] = []
    for start, end in windows:
        selected = overlap & (times >= start) & (times <= end)
        event_mask |= selected
        view = table.loc[selected]
        updates = view[view.status == "updated_relative_so3"]
        if view.empty or updates.empty:
            raise RuntimeError(f"{flight}: empty event {start}-{end}")
        before = percentile_metrics(
            view.baseline_fc_relative_yaw_error_deg.to_numpy(dtype=float)
        )
        after = percentile_metrics(
            view.shadow_fc_relative_yaw_error_deg.to_numpy(dtype=float)
        )
        events.append(
            {
                "window_s": [start, end],
                "first_fc_update_time_s": float(
                    updates.last_fc_sample_time_s.iloc[0]
                ),
                "last_fc_update_time_s": float(
                    updates.last_fc_sample_time_s.iloc[-1]
                ),
                "observed_flex_start_deg": float(
                    updates.observed_flex_yaw_deg.iloc[0]
                ),
                "observed_flex_end_deg": float(
                    updates.observed_flex_yaw_deg.iloc[-1]
                ),
                "observed_flex_net_change_deg": float(
                    updates.observed_flex_yaw_deg.iloc[-1]
                    - updates.observed_flex_yaw_deg.iloc[0]
                ),
                "filtered_flex_start_deg": float(
                    updates.output_flex_yaw_deg.iloc[0]
                ),
                "filtered_flex_end_deg": float(
                    updates.output_flex_yaw_deg.iloc[-1]
                ),
                "baseline_error": before,
                "shadow_error": after,
                "median_improvement_ratio": float(
                    (before["median_deg"] - after["median_deg"])
                    / max(before["median_deg"], 1e-12)
                ),
            }
        )

    normal = overlap & ~event_mask
    normal_before = percentile_metrics(baseline_residual[normal])
    normal_after = percentile_metrics(shadow_residual[normal])
    exhausted = table.status.to_numpy(dtype=str) == "hold_fc_stream_exhausted"
    exhausted_change = np.abs(np.diff(flex, prepend=flex[0]))[exhausted]
    trajectory = run_dir / "traj.txt"
    historical_trajectory = historical_dir / "traj.txt"
    bias = run_dir / "traj.txt.bias"
    historical_bias = historical_dir / "traj.txt.bias"
    result = {
        "run_dir": str(run_dir),
        "initialization_time_s": float(times[0]),
        "initial_observed_flex_yaw_deg": float(table.observed_flex_yaw_deg.iloc[0]),
        "initial_observer_target_flex_yaw_deg": float(
            table.observer_target_flex_yaw_deg.iloc[0]
        ),
        "initial_output_flex_yaw_deg": float(table.output_flex_yaw_deg.iloc[0]),
        "trajectory_sha256_matches_frozen": sha256(trajectory)
        == sha256(historical_trajectory),
        "bias_sha256_matches_frozen": sha256(bias) == sha256(historical_bias),
        "trajectory_sha256": sha256(trajectory),
        "historical_trajectory_sha256": sha256(historical_trajectory),
        "normal_baseline_error": normal_before,
        "normal_shadow_error": normal_after,
        "maximum_absolute_observed_flex_deg": float(
            np.max(np.abs(table.observed_flex_yaw_deg.to_numpy(dtype=float)))
        ),
        "maximum_absolute_filtered_flex_deg": float(np.max(np.abs(flex))),
        "maximum_absolute_observer_target_flex_deg": float(
            np.max(np.abs(table.observer_target_flex_yaw_deg.to_numpy(dtype=float)))
        ),
        "maximum_absolute_filtered_rate_deg_s": float(
            np.max(np.abs(table.filtered_output_rate_deg_s.to_numpy(dtype=float)))
        ),
        "maximum_single_frame_step_deg": float(
            np.max(np.abs(np.diff(flex, prepend=flex[0])))
        ),
        "fc_stream_valid_rows": int(np.sum(fc_valid)),
        "fc_stream_invalid_rows": int(np.sum(~fc_valid)),
        "fc_exhausted_output_rows": int(np.sum(exhausted)),
        "maximum_flex_change_while_fc_exhausted_deg": float(
            np.max(exhausted_change) if len(exhausted_change) else 0.0
        ),
        "events": events,
    }

    figure, axes = plt.subplots(4, 1, figsize=(18, 13), sharex=True)
    axes[0].plot(times, fc_yaw, label="direct FC", linewidth=0.8)
    axes[0].plot(times, nominal_yaw, label="raw D455 + init mount", linewidth=0.8)
    axes[0].plot(times, body_yaw, label="shadow aircraft body", linewidth=1.1)
    axes[1].plot(times, table.observed_flex_yaw_deg, label="observed flex", linewidth=0.8)
    axes[1].plot(
        times,
        table.observer_target_flex_yaw_deg,
        label="observer target",
        linewidth=0.8,
    )
    axes[1].plot(times, flex, label="smooth output flex", linewidth=1.1)
    axes[2].plot(times, baseline_residual, label="before", linewidth=0.8)
    axes[2].plot(times, shadow_residual, label="after", linewidth=1.0)
    axes[3].plot(times, table.filtered_output_rate_deg_s, label="flex output rate", linewidth=0.8)
    for axis in axes:
        for start, end in windows:
            axis.axvspan(start, end, color="tab:orange", alpha=0.12)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper left")
    axes[0].set_ylabel("yaw (deg)")
    axes[1].set_ylabel("flex (deg)")
    axes[2].set_ylabel("FC-relative error (deg)")
    axes[3].set_ylabel("deg/s")
    axes[3].set_xlabel("camera time (s)")
    figure.suptitle(f"{flight} ROS-free output-only yaw-flex")
    figure.tight_layout()
    figure.savefig(plot_path, dpi=150)
    plt.close(figure)
    return result, table


def write_report(summary: dict[str, Any], path: Path) -> None:
    lines = [
        "# ROS-free continuous yaw-flex four-flight validation",
        "",
        f"Status: **{summary['status']}**",
        "",
        "Baseline commit: `f8664c8ac7b9cf1bc80c734d6db453eb3b28ee7a`.",
        "The flex path is output-only and was disabled by default.",
        "",
        "| flight | init (s) | init flex obs/output (deg) | normal median before→after (deg) | normal P95 before→after (deg) | traj/bias exact | max flex/rate | FC exhausted hold |",
        "|---|---:|---:|---:|---:|---|---:|---|",
    ]
    for flight, item in summary["flights"].items():
        before = item["normal_baseline_error"]
        after = item["normal_shadow_error"]
        lines.append(
            f"| {flight} | {item['initialization_time_s']:.6f} | "
            f"{item['initial_observed_flex_yaw_deg']:.9f}/"
            f"{item['initial_output_flex_yaw_deg']:.9f} | "
            f"{before['median_deg']:.3f}→{after['median_deg']:.3f} | "
            f"{before['p95_deg']:.3f}→{after['p95_deg']:.3f} | "
            f"{item['trajectory_sha256_matches_frozen']}/"
            f"{item['bias_sha256_matches_frozen']} | "
            f"{item['maximum_absolute_filtered_flex_deg']:.3f}°/"
            f"{item['maximum_absolute_filtered_rate_deg_s']:.3f}°/s | "
            f"{item['fc_exhausted_output_rows']} rows, "
            f"max Δ={item['maximum_flex_change_while_fc_exhausted_deg']:.9f}° |"
        )
    lines.extend(["", "## fly3 target windows", ""])
    for event in summary["flights"]["fly3"]["events"]:
        before = event["baseline_error"]
        after = event["shadow_error"]
        lines.append(
            f"- {event['window_s'][0]:.0f}–{event['window_s'][1]:.0f} s: "
            f"observed {event['observed_flex_start_deg']:.3f}→"
            f"{event['observed_flex_end_deg']:.3f}° "
            f"(net {event['observed_flex_net_change_deg']:+.3f}°); "
            f"smooth output {event['filtered_flex_start_deg']:.3f}→"
            f"{event['filtered_flex_end_deg']:.3f}°; median error "
            f"{before['median_deg']:.3f}→{after['median_deg']:.3f}° "
            f"({event['median_improvement_ratio']:.1%} improvement)."
        )
    lines.extend(
        [
            "",
            "The frozen trajectory and bias outputs are byte-identical, so the observer did not modify VIO, P/V, bias, or the original trajectory.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manifest",
        type=Path,
        default=REPO_ROOT
        / "analysis/manifests/continuous_yaw_flex_stride12_20260719.json",
    )
    parser.add_argument("--fc-stream-dir", type=Path, required=True)
    parser.add_argument("--run", action="append", required=True, help="flight=directory")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError(f"refusing to overwrite {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    run_dirs = {name: Path(path) for name, path in (row.split("=", 1) for row in args.run)}
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    summary: dict[str, Any] = {
        "schema_version": 1,
        "status": "ROSFREE_FLEX_SHADOW_PASSED",
        "flights": {},
    }
    tables: dict[str, pd.DataFrame] = {}
    for flight in FLIGHTS:
        spec = manifest["flights"][flight]
        historical = windows_to_local(spec["run_dir"])
        result, table = evaluate_flight(
            flight,
            run_dirs[flight],
            historical,
            args.fc_stream_dir
            / f"{flight}_fc_body_attitude_camera_time.csv",
            spec.get("positive_windows_s", []),
            args.output_dir / f"{flight}_full.png",
        )
        summary["flights"][flight] = result
        tables[flight] = table
        if not result["trajectory_sha256_matches_frozen"] or not result[
            "bias_sha256_matches_frozen"
        ]:
            summary["status"] = "ROSFREE_FLEX_SHADOW_FAILED"
        if (
            result["initial_observed_flex_yaw_deg"] != 0.0
            or result["initial_observer_target_flex_yaw_deg"] != 0.0
            or result["initial_output_flex_yaw_deg"] != 0.0
        ):
            summary["status"] = "ROSFREE_FLEX_SHADOW_FAILED"
        if result["maximum_flex_change_while_fc_exhausted_deg"] != 0.0:
            summary["status"] = "ROSFREE_FLEX_SHADOW_FAILED"
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    write_report(summary, args.output_dir / "report.md")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["status"] == "ROSFREE_FLEX_SHADOW_PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
