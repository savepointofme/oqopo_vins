#!/usr/bin/env python3
"""Build a fair FC full-state row at a P4 release camera timestamp."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array(
        [
            [x * x - y * y - z * z + w * w, 2 * (x * y + z * w), 2 * (x * z - y * w)],
            [2 * (x * y - z * w), -x * x + y * y - z * z + w * w, 2 * (y * z + x * w)],
            [2 * (x * z + y * w), 2 * (y * z - x * w), -x * x - y * y + z * z + w * w],
        ],
        dtype=float,
    )


def rot_to_quat(rot: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rot))
    q = np.zeros(4, dtype=float)
    if rot[0, 0] >= trace and rot[0, 0] >= rot[1, 1] and rot[0, 0] >= rot[2, 2]:
        q[0] = math.sqrt(max(0.0, (1.0 + 2.0 * rot[0, 0] - trace) / 4.0))
        q[1] = (rot[0, 1] + rot[1, 0]) / (4.0 * q[0])
        q[2] = (rot[0, 2] + rot[2, 0]) / (4.0 * q[0])
        q[3] = (rot[1, 2] - rot[2, 1]) / (4.0 * q[0])
    elif rot[1, 1] >= trace and rot[1, 1] >= rot[0, 0] and rot[1, 1] >= rot[2, 2]:
        q[1] = math.sqrt(max(0.0, (1.0 + 2.0 * rot[1, 1] - trace) / 4.0))
        q[0] = (rot[0, 1] + rot[1, 0]) / (4.0 * q[1])
        q[2] = (rot[1, 2] + rot[2, 1]) / (4.0 * q[1])
        q[3] = (rot[2, 0] - rot[0, 2]) / (4.0 * q[1])
    elif rot[2, 2] >= trace and rot[2, 2] >= rot[0, 0] and rot[2, 2] >= rot[1, 1]:
        q[2] = math.sqrt(max(0.0, (1.0 + 2.0 * rot[2, 2] - trace) / 4.0))
        q[0] = (rot[0, 2] + rot[2, 0]) / (4.0 * q[2])
        q[1] = (rot[1, 2] + rot[2, 1]) / (4.0 * q[2])
        q[3] = (rot[0, 1] - rot[1, 0]) / (4.0 * q[2])
    else:
        q[3] = math.sqrt(max(0.0, (1.0 + trace) / 4.0))
        q[0] = (rot[1, 2] - rot[2, 1]) / (4.0 * q[3])
        q[1] = (rot[2, 0] - rot[0, 2]) / (4.0 * q[3])
        q[2] = (rot[0, 1] - rot[1, 0]) / (4.0 * q[3])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0.0 else q


def slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    return (math.sin((1.0 - alpha) * theta) * q0 + math.sin(alpha * theta) * q1) / math.sin(theta)


def load_online(path: Path) -> tuple[dict[str, str], list[list[float]]]:
    declarations: dict[str, str] = {}
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for raw in stream:
            stripped = raw.strip()
            if not stripped:
                continue
            if stripped.startswith("#"):
                payload = stripped[1:].strip()
                if "=" in payload:
                    key, value = payload.split("=", 1)
                    declarations[key.strip()] = value.strip()
                continue
            values = next(csv.reader([stripped]))
            if len(values) < 17:
                raise ValueError(f"online FC row has {len(values)} columns, expected at least 17")
            rows.append([float(value) for value in values[:17]])
    if len(rows) < 2:
        raise ValueError("online FC stream needs at least two rows")
    return declarations, rows


def interpolate(rows: list[list[float]], timestamp: float) -> list[float]:
    times = np.asarray([row[0] for row in rows])
    upper = int(np.searchsorted(times, timestamp, side="left"))
    if upper == 0 or upper >= len(rows):
        raise ValueError(f"timestamp {timestamp:.9f} is outside FC stream support")
    lower = upper - 1
    t0, t1 = times[lower], times[upper]
    alpha = float((timestamp - t0) / (t1 - t0))
    out = np.asarray(rows[lower], dtype=float) * (1.0 - alpha) + np.asarray(rows[upper], dtype=float) * alpha
    out[0] = timestamp
    out[1:5] = slerp(np.asarray(rows[lower][1:5]), np.asarray(rows[upper][1:5]), alpha)
    return out.tolist()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--online-fc", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    declarations, rows = load_online(args.online_fc)
    metadata = json.loads(args.metadata.read_text(encoding="utf-8"))
    camera_time = float(metadata["result_state_timestamp_s"])
    board_time = camera_time + float(metadata["locked_camera_to_imu_time_offset_s"])
    attitude_time = board_time - float(declarations["fc_attitude_to_board_time_offset_s"])
    navigation_time = board_time - float(declarations["fc_navigation_to_board_time_offset_s"])
    attitude = interpolate(rows, attitude_time)
    navigation = interpolate(rows, navigation_time)

    mount = np.asarray(
        [float(value) for value in declarations["R_FtoI_calibrated_row_major"].split(",")],
        dtype=float,
    ).reshape(3, 3)
    lever = np.asarray([float(value) for value in declarations["p_IinF_m"].split(",")], dtype=float)
    r_gtof = quat_to_rot(np.asarray(attitude[1:5]))
    r_gtoi = mount @ r_gtof
    q_gtoi = rot_to_quat(r_gtoi)
    position = np.asarray(navigation[8:11]) + r_gtof.T @ lever
    velocity = np.asarray(navigation[5:8])
    if np.linalg.norm(lever) > 1.0e-12:
        raise ValueError("non-zero FC-to-IMU lever requires angular-rate velocity correction")

    output_row = [camera_time, *q_gtoi.tolist(), *velocity.tolist(), *position.tolist(), *([0.0] * 6)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        stream.write("# Fair same-release FC single-row control generated from the online P4 stream\n")
        stream.write(f"# source_online_fc={args.online_fc}\n")
        stream.write(f"# source_online_alignment_metadata={args.metadata}\n")
        stream.write(f"# camera_release_timestamp_s={camera_time:.12f}\n")
        stream.write(f"# attitude_query_timestamp_s={attitude_time:.12f}\n")
        stream.write(f"# navigation_query_timestamp_s={navigation_time:.12f}\n")
        stream.write("# t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz\n")
        stream.write(",".join(f"{value:.12f}" for value in output_row) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
