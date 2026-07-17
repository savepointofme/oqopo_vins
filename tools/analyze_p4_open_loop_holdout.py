#!/usr/bin/env python3
"""Causal out-of-window IMU rollout check for P4 sliding-window states.

This diagnostic never uses GPS.  Each solved graph endpoint is propagated with
board IMU samples that occur strictly after the solve window.  The prediction
is then compared with the future FC navigation p/v stream that was not part of
that graph.  No FC correction is fed back into the rollout.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def split_vector(text: str, size: int) -> np.ndarray:
    values = np.asarray([float(value) for value in text.split(";")], dtype=float)
    if values.shape != (size,):
        raise ValueError(f"expected {size} values, got {text!r}")
    return values


def skew(value: np.ndarray) -> np.ndarray:
    x, y, z = value
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_exp(value: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(value))
    cross = skew(value)
    if angle < 1.0e-8:
        return np.eye(3) + cross + 0.5 * cross @ cross
    return (
        np.eye(3)
        + math.sin(angle) / angle * cross
        + (1.0 - math.cos(angle)) / (angle * angle) * cross @ cross
    )


def jpl_quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    quaternion = quaternion / np.linalg.norm(quaternion)
    vector = quaternion[:3]
    scalar = quaternion[3]
    return (
        (2.0 * scalar * scalar - 1.0) * np.eye(3)
        - 2.0 * scalar * skew(vector)
        + 2.0 * np.outer(vector, vector)
    )


def load_windows(path: Path) -> list[dict[str, object]]:
    windows: list[dict[str, object]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            windows.append(
                {
                    "window_id": int(row["window_id"]),
                    "time": float(row["window_end_timestamp_s"]),
                    "q": split_vector(row["q_GtoI_xyzw"], 4),
                    "p": split_vector(row["p_IinG_m"], 3),
                    "v": split_vector(row["v_IinG_mps"], 3),
                    "bg": split_vector(row["bg_rad_s"], 3),
                    "ba": split_vector(row["ba_mps2"], 3),
                    "outcome": row["solve_status"],
                }
            )
    if not windows:
        raise ValueError(f"no sliding windows in {path}")
    return windows


def load_numeric_csv(
    path: Path, minimum_time: float, maximum_time: float, *, imu_layout: bool = False
) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for line in stream:
            if not line or line.startswith("#"):
                continue
            fields = line.rstrip().split(",")
            timestamp = float(fields[0])
            if timestamp < minimum_time:
                continue
            if timestamp > maximum_time:
                break
            if imu_layout:
                # D455 recording layout: t_rel,t_device,t_ns,gyro_domain,
                # accel_domain,wx,wy,wz,ax,ay,az.
                rows.append([timestamp, *[float(value) for value in fields[5:11]]])
            else:
                rows.append([float(value) for value in fields])
    if len(rows) < 2:
        raise ValueError(f"insufficient rows in {path} over requested interval")
    return np.asarray(rows, dtype=float)


def load_fc_calibration(path: Path) -> tuple[np.ndarray, float]:
    rotation = None
    attitude_offset = None
    with path.open("r", encoding="utf-8-sig") as stream:
        for line in stream:
            if not line.startswith("#"):
                break
            if line.startswith("# R_FtoI_calibrated_row_major="):
                values = [float(value) for value in line.split("=", 1)[1].split(",")]
                rotation = np.asarray(values, dtype=float).reshape(3, 3)
            elif line.startswith("# fc_attitude_to_board_time_offset_s="):
                attitude_offset = float(line.split("=", 1)[1])
    if rotation is None or attitude_offset is None:
        raise ValueError(f"missing locked FC/board calibration declaration in {path}")
    return rotation, attitude_offset


def interpolate_rows(rows: np.ndarray, timestamp: float) -> np.ndarray:
    times = rows[:, 0]
    upper = int(np.searchsorted(times, timestamp, side="left"))
    if upper == 0:
        return rows[0].copy()
    if upper >= len(rows):
        return rows[-1].copy()
    before = rows[upper - 1]
    after = rows[upper]
    span = after[0] - before[0]
    alpha = 0.0 if span <= 0.0 else (timestamp - before[0]) / span
    result = (1.0 - alpha) * before + alpha * after
    result[0] = timestamp
    return result


def imu_segment(imu: np.ndarray, start: float, end: float) -> np.ndarray:
    if not end > start:
        raise ValueError("holdout endpoint must follow window endpoint")
    times = imu[:, 0]
    first = max(0, int(np.searchsorted(times, start, side="right")) - 1)
    last = min(len(imu), int(np.searchsorted(times, end, side="left")) + 1)
    middle = imu[first:last]
    selected = middle[(middle[:, 0] > start) & (middle[:, 0] < end)]
    return np.vstack(
        [interpolate_rows(imu, start), selected, interpolate_rows(imu, end)]
    )


def propagate(
    rotation_g_to_i: np.ndarray,
    position: np.ndarray,
    velocity: np.ndarray,
    gyro_bias: np.ndarray,
    accel_bias: np.ndarray,
    samples: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gravity = np.asarray([0.0, 0.0, 9.81])
    rotation = rotation_g_to_i.copy()
    position = position.copy()
    velocity = velocity.copy()
    for index in range(1, len(samples)):
        before = samples[index - 1]
        after = samples[index]
        dt = after[0] - before[0]
        omega_mid = 0.5 * (before[1:4] + after[1:4]) - gyro_bias
        accel_mid = 0.5 * (before[4:7] + after[4:7]) - accel_bias
        rotation_mid = so3_exp(-0.5 * omega_mid * dt) @ rotation
        accel_global = rotation_mid.T @ accel_mid - gravity
        position = position + velocity * dt + 0.5 * accel_global * dt * dt
        velocity = velocity + accel_global * dt
        rotation = so3_exp(-omega_mid * dt) @ rotation
    return rotation, position, velocity


def evaluate_window(
    window: dict[str, object], imu: np.ndarray, fc: np.ndarray, duration: float,
    rotation_f_to_i: np.ndarray, attitude_offset: float,
) -> dict[str, float | int | str]:
    start = float(window["time"])
    end = start + duration
    fc_times = fc[:, 0]
    measurement_times = fc_times[(fc_times > start + 1.0e-9) & (fc_times <= end)]
    if len(measurement_times) == 0:
        raise ValueError(f"window {window['window_id']} has no future FC measurements")

    rotation = jpl_quaternion_to_rotation(np.asarray(window["q"]))
    position = np.asarray(window["p"]).copy()
    velocity = np.asarray(window["v"]).copy()
    gyro_bias = np.asarray(window["bg"])
    accel_bias = np.asarray(window["ba"])
    current_time = start
    position_errors: list[float] = []
    velocity_errors: list[float] = []
    horizontal_velocity_errors: list[float] = []
    along_velocity_errors: list[float] = []
    for timestamp in measurement_times:
        segment = imu_segment(imu, current_time, float(timestamp))
        rotation, position, velocity = propagate(
            rotation, position, velocity, gyro_bias, accel_bias, segment
        )
        truth = interpolate_rows(fc, float(timestamp))
        # FC columns: t,qx,qy,qz,qw,vx,vy,vz,px,py,pz,...
        reference_velocity = truth[5:8]
        reference_position = truth[8:11]
        velocity_error = velocity - reference_velocity
        position_errors.append(float(np.linalg.norm(position - reference_position)))
        velocity_errors.append(float(np.linalg.norm(velocity_error)))
        horizontal_velocity_errors.append(float(np.linalg.norm(velocity_error[:2])))
        horizontal_speed = float(np.linalg.norm(reference_velocity[:2]))
        if horizontal_speed > 1.0e-6:
            along_velocity_errors.append(
                float(np.dot(velocity_error[:2], reference_velocity[:2]) / horizontal_speed)
            )
        current_time = float(timestamp)

    initial_fc = interpolate_rows(fc, start)
    initial_position_error = float(
        np.linalg.norm(np.asarray(window["p"]) - initial_fc[8:11])
    )
    initial_velocity_error = float(
        np.linalg.norm(np.asarray(window["v"]) - initial_fc[5:8])
    )
    attitude_fc = interpolate_rows(fc, start - attitude_offset)
    rotation_fc = jpl_quaternion_to_rotation(attitude_fc[1:5])
    rotation_graph = jpl_quaternion_to_rotation(np.asarray(window["q"]))
    rotation_expected = rotation_f_to_i @ rotation_fc
    rotation_delta = rotation_graph @ rotation_expected.T
    full_attitude_delta = math.degrees(
        math.acos(float(np.clip((np.trace(rotation_delta) - 1.0) * 0.5, -1.0, 1.0)))
    )
    graph_gravity = rotation_graph @ np.asarray([0.0, 0.0, 1.0])
    expected_gravity = rotation_expected @ np.asarray([0.0, 0.0, 1.0])
    tilt_delta = math.degrees(
        math.acos(float(np.clip(np.dot(graph_gravity, expected_gravity), -1.0, 1.0)))
    )
    return {
        "window_id": int(window["window_id"]),
        "window_end_s": start,
        "holdout_duration_s": float(measurement_times[-1] - start),
        "future_fc_count": int(len(measurement_times)),
        "initial_position_error_m": initial_position_error,
        "initial_velocity_error_mps": initial_velocity_error,
        "graph_to_locked_fc_attitude_delta_deg": full_attitude_delta,
        "graph_to_locked_fc_tilt_delta_deg": tilt_delta,
        "holdout_position_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "holdout_position_final_m": position_errors[-1],
        "holdout_position_growth_m": position_errors[-1] - initial_position_error,
        "holdout_velocity_rmse_mps": float(np.sqrt(np.mean(np.square(velocity_errors)))),
        "holdout_velocity_final_mps": velocity_errors[-1],
        "holdout_velocity_growth_mps": velocity_errors[-1] - initial_velocity_error,
        "holdout_horizontal_velocity_rmse_mps": float(
            np.sqrt(np.mean(np.square(horizontal_velocity_errors)))
        ),
        "holdout_along_velocity_mean_mps": float(np.mean(along_velocity_errors)),
        "holdout_along_velocity_final_mps": float(along_velocity_errors[-1]),
        "ba_norm_mps2": float(np.linalg.norm(np.asarray(window["ba"]))),
        "solve_status": str(window["outcome"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--fc", type=Path, required=True)
    parser.add_argument("--holdout-duration", type=float, default=4.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    windows = load_windows(args.run_dir / "online_alignment_sliding_windows.csv")
    minimum_time = min(float(window["time"]) for window in windows) - 0.1
    maximum_time = max(float(window["time"]) for window in windows) + args.holdout_duration + 0.1
    imu = load_numeric_csv(
        args.dataset / "imu0" / "data.csv",
        minimum_time,
        maximum_time,
        imu_layout=True,
    )
    fc = load_numeric_csv(args.fc, minimum_time, maximum_time)
    rotation_f_to_i, attitude_offset = load_fc_calibration(args.fc)
    results = [
        evaluate_window(
            window,
            imu,
            fc,
            args.holdout_duration,
            rotation_f_to_i,
            attitude_offset,
        )
        for window in windows
        if float(window["time"]) + args.holdout_duration <= fc[-1, 0]
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)

    final = results[-1]
    print(
        "windows={window_count} final_window={window_id} "
        "v_rmse={holdout_velocity_rmse_mps:.6f} "
        "v_final={holdout_velocity_final_mps:.6f} "
        "p_final={holdout_position_final_m:.6f} "
        "along_final={holdout_along_velocity_final_mps:.6f}".format(
            window_count=len(results), **final
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
