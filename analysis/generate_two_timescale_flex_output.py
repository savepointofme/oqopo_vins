#!/usr/bin/env python3
"""Generate a causal two-timescale yaw-flex shadow from a raw SO(3) trace."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def clipped(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tau-baseline-s", type=float, required=True)
    parser.add_argument("--tau-flex-s", type=float, default=12.0)
    parser.add_argument("--huber-deg", type=float, default=1.5)
    parser.add_argument("--maximum-rate-deg-s", type=float, default=0.25)
    parser.add_argument("--maximum-absolute-flex-deg", type=float, default=6.0)
    parser.add_argument("--maximum-filter-dt-s", type=float, default=0.45)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError(f"refusing to overwrite {args.output}")
    if args.tau_baseline_s <= args.tau_flex_s:
        raise RuntimeError("baseline time constant must exceed flex time constant")

    table = pd.read_csv(args.input, comment="#")
    if table.empty:
        raise RuntimeError(f"empty input: {args.input}")
    times = table.camera_time_s.to_numpy(dtype=float)
    observed = table.observed_flex_yaw_deg.to_numpy(dtype=float)
    statuses = table.status.astype(str).to_numpy()
    fc_times = table.last_fc_sample_time_s.to_numpy(dtype=float)

    baseline = 0.0
    target = 0.0
    output = 0.0
    last_measurement_time = float(times[0])
    last_output_time = float(times[0])
    fc_release_valid = True
    baseline_trace = np.zeros(len(table))
    local_trace = np.zeros(len(table))
    target_trace = np.zeros(len(table))
    output_trace = np.zeros(len(table))
    output_rate_trace = np.zeros(len(table))
    update_count = 0

    for index in range(1, len(table)):
        status = statuses[index]
        is_update = status.startswith("updated_") and math.isfinite(fc_times[index])
        if is_update:
            measurement_time = float(fc_times[index])
            dt = measurement_time - last_measurement_time
            if dt > 0.0 and math.isfinite(dt):
                filter_dt = min(dt, args.maximum_filter_dt_s)
                alpha_baseline = 1.0 - math.exp(-filter_dt / args.tau_baseline_s)
                # The slow path represents the full long-term VIO-FC level.
                # Clipping this innovation prevents it from following a large
                # but gradual drift and leaks that drift into the flex path.
                # Its small alpha already attenuates isolated FC spikes.
                baseline += alpha_baseline * (float(observed[index]) - baseline)
                local_observation = float(observed[index]) - baseline
                alpha_flex = 1.0 - math.exp(-filter_dt / args.tau_flex_s)
                step = alpha_flex * clipped(local_observation - target, args.huber_deg)
                step = clipped(step, args.maximum_rate_deg_s * filter_dt)
                target = clipped(target + step, args.maximum_absolute_flex_deg)
                last_measurement_time = measurement_time
                update_count += 1
            fc_release_valid = True
        elif status in {"hold_invalid_fc", "hold_fc_stream_exhausted"}:
            fc_release_valid = False

        local_observation = float(observed[index]) - baseline
        output_dt = float(times[index] - last_output_time)
        output_rate = 0.0
        if output_dt > 0.0 and math.isfinite(output_dt) and fc_release_valid:
            output_step = clipped(target - output, args.maximum_rate_deg_s * output_dt)
            output += output_step
            output_rate = output_step / output_dt
        last_output_time = float(times[index])
        baseline_trace[index] = baseline
        local_trace[index] = local_observation
        target_trace[index] = target
        output_trace[index] = output
        output_rate_trace[index] = output_rate

    d455 = Rotation.from_quat(
        table[["d455_qx", "d455_qy", "d455_qz", "d455_qw"]].to_numpy(dtype=float)
    )
    original_body = Rotation.from_quat(
        table[
            [
                "aircraft_body_qx",
                "aircraft_body_qy",
                "aircraft_body_qz",
                "aircraft_body_qw",
            ]
        ].to_numpy(dtype=float)
    )
    nominal_mount = d455[0].inv() * original_body[0]
    nominal_body = d455 * nominal_mount
    nominal_rpy = nominal_body.as_euler("xyz", degrees=True)
    corrected_rpy = nominal_rpy.copy()
    corrected_rpy[:, 2] -= output_trace
    corrected_body = Rotation.from_euler("xyz", corrected_rpy, degrees=True)
    corrected_quat = corrected_body.as_quat()

    result = table.copy()
    result["two_timescale_baseline_yaw_deg"] = baseline_trace
    result["local_flex_observation_deg"] = local_trace
    result["observer_target_flex_yaw_deg"] = target_trace
    result["output_flex_yaw_deg"] = output_trace
    result["output_flex_rate_deg_s"] = output_rate_trace
    result["d455_nominal_body_yaw_deg"] = nominal_rpy[:, 2]
    result["aircraft_body_yaw_deg"] = corrected_rpy[:, 2]
    result[
        [
            "aircraft_body_qx",
            "aircraft_body_qy",
            "aircraft_body_qz",
            "aircraft_body_qw",
        ]
    ] = corrected_quat

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write("# schema=two_timescale_body_axis_so3_yaw_flex_shadow_v1\n")
        stream.write(
            f"# tau_baseline_s={args.tau_baseline_s};tau_flex_s={args.tau_flex_s};"
            f"huber_deg={args.huber_deg};max_rate_deg_s={args.maximum_rate_deg_s};"
            f"max_abs_flex_deg={args.maximum_absolute_flex_deg}\n"
        )
        result.to_csv(stream, index=False, float_format="%.9f")

    summary = {
        "schema_version": 1,
        "input": str(args.input),
        "output": str(args.output),
        "tau_baseline_s": args.tau_baseline_s,
        "tau_flex_s": args.tau_flex_s,
        "update_count": update_count,
        "initial_baseline_deg": float(baseline_trace[0]),
        "initial_output_flex_deg": float(output_trace[0]),
        "maximum_absolute_baseline_deg": float(np.max(np.abs(baseline_trace))),
        "maximum_absolute_local_observation_deg": float(np.max(np.abs(local_trace))),
        "maximum_absolute_output_flex_deg": float(np.max(np.abs(output_trace))),
        "maximum_absolute_output_rate_deg_s": float(np.max(np.abs(output_rate_trace))),
    }
    args.output.with_suffix(".summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
