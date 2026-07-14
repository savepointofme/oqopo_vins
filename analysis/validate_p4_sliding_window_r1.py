#!/usr/bin/env python3
"""Validate P4-R1 fixed-time sliding-window shadow-run semantics."""

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
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--window-duration", type=float, default=8.0)
    parser.add_argument("--min-advance", type=float, default=0.5)
    parser.add_argument("--max-keyframes", type=int, default=10)
    args = parser.parse_args()

    with args.trace.open(newline="", encoding="utf-8") as stream:
        all_rows = list(csv.DictReader(stream))
    solved = [row for row in all_rows if row["solve_status"] == "shadow_window_solved"]
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

    for row in solved:
        begin = float(row["window_begin_timestamp_s"])
        end = float(row["window_end_timestamp_s"])
        duration = float(row["window_duration_s"])
        invocation = int(row["optimizer_invocation_index"])
        keyframes = int(row["selected_keyframe_count"])
        frames = {
            float(item)
            for item in row["selected_frame_timestamps_s"].split(";")
            if item
        }
        fingerprint = row["window_fingerprint"]
        invocation_indices.append(invocation)
        keyframe_counts.append(keyframes)
        fc_counts.append(int(row["raw_fc_count"]))
        imu_counts.append(int(row["imu_count"]))
        visual_counts.append(int(row["visual_frame_count"]))
        solve_times.append(float(row["solve_wall_time_s"]))

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
        for field, size in (
            ("q_GtoI_xyzw", 4),
            ("p_IinG_m", 3),
            ("v_IinG_mps", 3),
            ("bg_rad_s", 3),
            ("ba_mps2", 3),
        ):
            try:
                parse_vector(row[field], size)
            except ValueError as error:
                failures.append(f"window {invocation} {error}")

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
    if len(solved) >= 2 and not stale_left_observed:
        failures.append("no selected timestamp left the moving window")
    if len(solved) >= 2 and not new_entered_observed:
        failures.append("no new selected timestamp entered the moving window")

    with args.metadata.open(encoding="utf-8") as stream:
        metadata = json.load(stream)
    if metadata.get("schema") != "openvins_p4_r1_sliding_window_shadow_v1":
        failures.append("unexpected metadata schema")
    if metadata.get("released_to_openvins") is not False:
        failures.append("R1 shadow run released state to OpenVINS")
    if metadata.get("alignment_active") is not True:
        failures.append("R1 alignment was closed")
    if metadata.get("optimizer_invocation_count") != len(
        [row for row in all_rows if int(row["optimizer_invocation_index"]) > 0]
    ):
        failures.append("metadata optimizer invocation count disagrees with trace")

    report = {
        "status": "PASS" if not failures else "FAIL",
        "trace": str(args.trace),
        "metadata": str(args.metadata),
        "total_window_receipts": len(all_rows),
        "solved_window_count": len(solved),
        "optimizer_invocation_indices": invocation_indices,
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
        "failures": failures,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
