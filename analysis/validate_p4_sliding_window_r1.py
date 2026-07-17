#!/usr/bin/env python3
"""Validate the formal P4 fixed-time sliding-window release contract."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def parse_vector(value: str, expected_size: int) -> list[float]:
    result = [float(item) for item in value.split(";") if item]
    if len(result) != expected_size or not all(math.isfinite(item) for item in result):
        raise ValueError(f"invalid {expected_size}-vector: {value}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--trajectory", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--window-duration", type=float, default=8.0)
    parser.add_argument("--min-advance", type=float, default=0.5)
    parser.add_argument(
        "--max-keyframes",
        type=int,
        default=36,
        help="formal time-window graph resource bound, not a target count",
    )
    args = parser.parse_args()

    with args.trace.open(newline="", encoding="utf-8") as stream:
        all_rows = list(csv.DictReader(stream))
    solved_statuses = {
        "shadow_window_solved",
        "gauge_stability_release",
        "direct_state_release",
    }
    solved = [row for row in all_rows if row["solve_status"] in solved_statuses]
    failures: list[str] = []
    if len(solved) < 2:
        failures.append(f"only {len(solved)} solved sliding windows")

    previous_end = None
    previous_begin = None
    previous_frames: set[float] | None = None
    stale_left_observed = False
    new_entered_observed = False
    seen_fingerprints: set[str] = set()
    solve_times: list[float] = []
    fc_counts: list[int] = []
    imu_counts: list[int] = []
    visual_counts: list[int] = []
    keyframe_counts: list[int] = []
    invocation_indices: list[int] = []
    invocation_counts: list[int] = []
    stage1_solve_times: list[float] = []
    final_stage_solve_times: list[float] = []
    warm_start_flags: list[int] = []
    state_signatures: list[tuple[float, ...]] = []

    for row in solved:
        begin = float(row["window_begin_timestamp_s"])
        end = float(row["window_end_timestamp_s"])
        duration = float(row["window_duration_s"])
        invocation = int(row["optimizer_invocation_index"])
        invocation_count = int(row["optimizer_invocation_count"])
        keyframes = int(row["selected_keyframe_count"])
        frames = {
            float(item)
            for item in row["selected_frame_timestamps_s"].split(";")
            if item
        }
        fingerprint = row["window_fingerprint"]
        invocation_indices.append(invocation)
        invocation_counts.append(invocation_count)
        stage1_solve_times.append(float(row["stage1_solve_wall_time_s"]))
        final_stage_solve_times.append(float(row["final_stage_solve_wall_time_s"]))
        warm_start_flags.append(int(row["warm_start_used"]))
        keyframe_counts.append(keyframes)
        fc_counts.append(int(row["raw_fc_count"]))
        imu_counts.append(int(row["imu_count"]))
        visual_counts.append(int(row["visual_frame_count"]))
        solve_times.append(float(row["solve_wall_time_s"]))

        if invocation_count != 1:
            failures.append(f"window {invocation} did not run exactly one dynamic graph solve")
        if row["shared_window_bias_model"] != "0":
            failures.append(f"window {invocation} still uses the rejected shared window bias")
        if row["staged_solver_enabled"] != "0":
            failures.append(f"window {invocation} still declares the rejected staged solver")

        if abs(duration - args.window_duration) > 1.0e-6:
            failures.append(f"window {invocation} duration {duration} is not fixed")
        if abs((end - begin) - args.window_duration) > 1.0e-6:
            failures.append(f"window {invocation} begin/end span is not fixed")
        if keyframes <= 0 or keyframes > args.max_keyframes:
            failures.append(f"window {invocation} keyframe count {keyframes} is invalid")
        if not frames or min(frames) < begin - 1.0e-6 or max(frames) > end + 1.0e-6:
            failures.append(f"window {invocation} selected frames are outside the window")
        if fingerprint in seen_fingerprints:
            failures.append(f"window {invocation} duplicates a previous fingerprint")
        seen_fingerprints.add(fingerprint)
        state_signature: list[float] = []
        for field, size in (
            ("q_GtoI_xyzw", 4),
            ("p_IinG_m", 3),
            ("v_IinG_mps", 3),
            ("bg_rad_s", 3),
            ("ba_mps2", 3),
        ):
            try:
                state_signature.extend(parse_vector(row[field], size))
            except ValueError as error:
                failures.append(f"window {invocation} {error}")
        state_signatures.append(tuple(state_signature))

        if previous_end is not None:
            if end - previous_end + 1.0e-6 < args.min_advance:
                failures.append(f"window {invocation} end did not advance by time")
            if begin <= previous_begin + 1.0e-9:
                failures.append(f"window {invocation} begin did not move forward")
            stale_left_observed |= bool(previous_frames - frames)
            new_entered_observed |= bool(frames - previous_frames)
        previous_begin = begin
        previous_end = end
        previous_frames = frames

    if invocation_indices and invocation_indices != sorted(set(invocation_indices)):
        failures.append("optimizer invocation indices are duplicated or non-monotonic")
    expected_invocation = 0
    for invocation, count in zip(invocation_indices, invocation_counts):
        expected_invocation += count
        if invocation != expected_invocation:
            failures.append("optimizer invocation indices do not match per-window stage counts")
            break
    if warm_start_flags and warm_start_flags[0] != 0:
        failures.append("first sliding window unexpectedly reports a warm start")
    if any(flag != 1 for flag in warm_start_flags[1:]):
        failures.append("one or more later sliding windows did not use a warm start")
    if state_signatures and len(set(state_signatures)) != len(state_signatures):
        failures.append("one or more solved windows reused an identical q/p/v/bg/ba output")
    if len(solved) >= 2 and not stale_left_observed:
        failures.append("no selected timestamp left the moving window")
    if len(solved) >= 2 and not new_entered_observed:
        failures.append("no new selected timestamp entered the moving window")

    with args.metadata.open(encoding="utf-8") as stream:
        metadata = json.load(stream)
    if metadata.get("schema") != "openvins_online_multisensor_alignment_v14":
        failures.append("unexpected metadata schema")
    if metadata.get("released_to_openvins") is not True:
        failures.append("formal P4 state was not released to OpenVINS")
    if metadata.get("alignment_window_closed") is not True:
        failures.append("P4 finite startup interval was not closed after release")
    if metadata.get("direct_sliding_state_release") is not True:
        failures.append("release did not use the direct sliding-state lifecycle")
    if metadata.get("sliding_overlap_consistency_passed") is not True:
        failures.append("same-timestamp overlap consistency did not pass")
    required_span = float(
        metadata.get("sliding_overlap_stability_required_s", math.inf)
    )
    observed_span = float(
        metadata.get("sliding_overlap_stable_duration_s", -math.inf)
    )
    if not math.isfinite(required_span) or not math.isfinite(observed_span) or observed_span + 1.0e-9 < required_span:
        failures.append("release lacks the required sensor-time stability span")
    for field in (
        "sliding_overlap_attitude_max_deg",
        "sliding_overlap_position_max_m",
        "sliding_overlap_velocity_max_mps",
        "sliding_overlap_gyro_bias_max_rad_s",
        "sliding_overlap_accel_bias_max_mps2",
        "sliding_overlap_normalized_max_sigma",
    ):
        value = metadata.get(field)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            failures.append(f"metadata {field} is not finite")
    if metadata.get("future_data_used") is not False:
        failures.append("P4 release used future data")
    if metadata.get("gps_xy_or_course_used_by_alignment") is not False:
        failures.append("P4 release used GPS XY/course")
    if metadata.get("shared_window_bias_model") is not False:
        failures.append("metadata does not declare per-keyframe bias topology")
    if metadata.get("staged_solver_enabled") is not False:
        failures.append("metadata still declares the rejected staged solver")
    if metadata.get("fc_position_velocity_weight_model") != "terminal_absolute_plus_density_invariant_increments":
        failures.append("FC p/v factors do not preserve the terminal boundary with density-invariant increments")
    fc_weight_sum = metadata.get("fc_position_velocity_time_weight_sum")
    if not isinstance(fc_weight_sum, (int, float)) or abs(float(fc_weight_sum) - 1.0) > 1.0e-9:
        failures.append("FC p/v time-integral weights do not sum to one")
    final_selected_keyframe_count = (
        int(all_rows[-1]["selected_keyframe_count"]) if all_rows else None
    )
    if metadata.get("fc_position_velocity_factor_count") != final_selected_keyframe_count:
        failures.append("FC p/v factor count does not match the graph keyframe count")
    terminal_normalized_residual = metadata.get("fc_terminal_max_normalized_residual")
    if (
        not isinstance(terminal_normalized_residual, (int, float))
        or not math.isfinite(float(terminal_normalized_residual))
        or float(terminal_normalized_residual) > 3.0
    ):
        failures.append("released terminal q/p/v does not satisfy the synchronized FC boundary")
    clone_count = metadata.get("initial_history_clone_count")
    landmark_count = metadata.get("initial_persistent_landmark_count")
    consumed_feature_count = metadata.get("startup_consumed_feature_count")
    covariance_dimension = metadata.get("initial_joint_covariance_dimension")
    if clone_count != 0:
        failures.append("terminal-state handoff unexpectedly injected graph clones")
    if landmark_count != 0:
        failures.append("terminal-state handoff unexpectedly injected graph landmarks")
    if consumed_feature_count != 0:
        failures.append("terminal-state handoff unexpectedly retained consumed feature IDs")
    if covariance_dimension != 0:
        failures.append("terminal-state handoff unexpectedly carries a joint history covariance")
    if metadata.get("initial_history_covariance_recovered") is not False:
        failures.append("terminal-state handoff incorrectly declares history covariance")
    if metadata.get("handoff_covariance_inflation_applied") is not True:
        failures.append("native covariance handoff inflation was not applied")
    if metadata.get("handoff_covariance_model") != "openvins_dynamic_initializer_terminal_state":
        failures.append("unexpected covariance handoff model")
    inflation = metadata.get("handoff_covariance_inflation", {})
    expected_inflation = {
        "orientation": 10.0,
        "velocity": 100.0,
        "gyro_bias": 10.0,
        "accelerometer_bias": 100.0,
    }
    for name, expected in expected_inflation.items():
        value = inflation.get(name)
        if not isinstance(value, (int, float)) or abs(float(value) - expected) > 1.0e-9:
            failures.append(f"unexpected {name} handoff inflation")
    raw_std = metadata.get("handoff_raw_covariance_std")
    applied_std = metadata.get("handoff_applied_covariance_std")
    if not isinstance(raw_std, list) or len(raw_std) != 15 or not all(
        isinstance(value, (int, float)) and math.isfinite(float(value)) and float(value) > 0.0
        for value in raw_std
    ):
        failures.append("raw handoff covariance standard deviations are invalid")
    if not isinstance(applied_std, list) or len(applied_std) != 15 or not all(
        isinstance(value, (int, float)) and math.isfinite(float(value)) and float(value) > 0.0
        for value in applied_std
    ):
        failures.append("applied handoff covariance standard deviations are invalid")
    if isinstance(raw_std, list) and len(raw_std) == 15 and isinstance(applied_std, list) and len(applied_std) == 15:
        expected_std_scale = [math.sqrt(10.0)] * 3 + [1.0] * 3 + [10.0] * 3 + [math.sqrt(10.0)] * 3 + [10.0] * 3
        for index, expected in enumerate(expected_std_scale):
            raw = float(raw_std[index])
            applied = float(applied_std[index])
            if raw <= 0.0 or abs(applied / raw - expected) > 1.0e-6:
                failures.append(f"handoff covariance std scale mismatch at state index {index}")
                break
    if metadata.get("sliding_bias_trend_passed") is not True:
        failures.append("terminal graph bias retained a significant cross-window trend")
    if metadata.get("nonlinear_solve_attempt_count") != sum(invocation_counts):
        failures.append("metadata optimizer invocation count disagrees with trace")
    release_rows = [
        row for row in all_rows if row["solve_status"] == "direct_state_release"
    ]
    if len(release_rows) != 1 or release_rows[0] is not all_rows[-1]:
        failures.append("trace does not end with exactly one formal release window")
    if args.trajectory is not None:
        if not args.trajectory.is_file() or args.trajectory.stat().st_size == 0:
            failures.append("formal navigation trajectory is missing or empty")

    report = {
        "status": "PASS" if not failures else "FAIL",
        "trace": str(args.trace),
        "metadata": str(args.metadata),
        "trajectory": str(args.trajectory) if args.trajectory is not None else None,
        "total_window_receipts": len(all_rows),
        "solved_window_count": len(solved),
        "optimizer_invocation_indices": invocation_indices,
        "optimizer_invocation_counts": invocation_counts,
        "later_windows_all_warm_started": bool(warm_start_flags) and all(
            flag == 1 for flag in warm_start_flags[1:]
        ),
        "unique_state_output_count": len(set(state_signatures)),
        "first_window_begin_timestamp_s": float(solved[0]["window_begin_timestamp_s"]) if solved else None,
        "last_window_begin_timestamp_s": float(solved[-1]["window_begin_timestamp_s"]) if solved else None,
        "first_window_end_timestamp_s": float(solved[0]["window_end_timestamp_s"]) if solved else None,
        "last_window_end_timestamp_s": float(solved[-1]["window_end_timestamp_s"]) if solved else None,
        "stale_selected_timestamp_left": stale_left_observed,
        "new_selected_timestamp_entered": new_entered_observed,
        "keyframe_count_min": min(keyframe_counts) if keyframe_counts else None,
        "keyframe_count_max": max(keyframe_counts) if keyframe_counts else None,
        "fc_count_min": min(fc_counts) if fc_counts else None,
        "fc_count_max": max(fc_counts) if fc_counts else None,
        "imu_count_min": min(imu_counts) if imu_counts else None,
        "imu_count_max": max(imu_counts) if imu_counts else None,
        "visual_count_min": min(visual_counts) if visual_counts else None,
        "visual_count_max": max(visual_counts) if visual_counts else None,
        "solve_wall_time_s_max": max(solve_times) if solve_times else None,
        "stage1_solve_wall_time_s_max": max(stage1_solve_times) if stage1_solve_times else None,
        "final_stage_solve_wall_time_s_max": max(final_stage_solve_times) if final_stage_solve_times else None,
        "overlap_stability_required_s": required_span,
        "overlap_stable_duration_s": observed_span,
        "overlap_state_count": metadata.get("sliding_overlap_state_count"),
        "overlap_attitude_max_deg": metadata.get(
            "sliding_overlap_attitude_max_deg"
        ),
        "overlap_position_max_m": metadata.get("sliding_overlap_position_max_m"),
        "overlap_velocity_max_mps": metadata.get(
            "sliding_overlap_velocity_max_mps"
        ),
        "overlap_gyro_bias_max_rad_s": metadata.get(
            "sliding_overlap_gyro_bias_max_rad_s"
        ),
        "overlap_accel_bias_max_mps2": metadata.get(
            "sliding_overlap_accel_bias_max_mps2"
        ),
        "failures": failures,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
