#!/usr/bin/env python3
"""Apply a causal flex-yaw correction to VIO XY increments for evaluation.

This produces an independent experimental trajectory.  It never overwrites
the ROS-free trajectory and does not claim estimator-state integration.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_numeric(path: Path, columns: list[str]) -> pd.DataFrame:
    table = pd.read_csv(
        path, comment="#", sep=r"\s+", header=None, names=columns, engine="python"
    )
    if table.empty:
        raise RuntimeError(f"empty input: {path}")
    return table


def rotate_xy(values: np.ndarray, angle_rad: np.ndarray) -> np.ndarray:
    cosine = np.cos(angle_rad)
    sine = np.sin(angle_rad)
    result = values.copy()
    result[:, 0] = cosine * values[:, 0] - sine * values[:, 1]
    result[:, 1] = sine * values[:, 0] + cosine * values[:, 1]
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--traj", type=Path, required=True)
    parser.add_argument("--bias", type=Path)
    parser.add_argument("--flex-output", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--method-name", required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError(f"refusing to overwrite {args.output_dir}")
    args.output_dir.mkdir(parents=True)

    traj_columns = ["t", "x", "y", "z", "qx", "qy", "qz", "qw"]
    trajectory = read_numeric(args.traj, traj_columns)
    flex = pd.read_csv(args.flex_output, comment="#")
    if flex.empty:
        raise RuntimeError(f"empty flex output: {args.flex_output}")
    flex_times = flex.camera_time_s.to_numpy(dtype=float)
    traj_times = trajectory.t.to_numpy(dtype=float)
    if traj_times[0] < flex_times[0] - 1e-6 or traj_times[-1] > flex_times[-1] + 1e-6:
        raise RuntimeError("flex output does not cover trajectory")

    flex_rad = np.radians(
        np.interp(
            traj_times,
            flex_times,
            flex.output_flex_yaw_deg.to_numpy(dtype=float),
        )
    )
    positions = trajectory[["x", "y", "z"]].to_numpy(dtype=float)
    increments = np.diff(positions, axis=0)
    midpoint_flex = 0.5 * (flex_rad[:-1] + flex_rad[1:])
    corrected_increments = rotate_xy(increments, -midpoint_flex)
    corrected_positions = np.empty_like(positions)
    corrected_positions[0] = positions[0]
    corrected_positions[1:] = positions[0] + np.cumsum(corrected_increments, axis=0)

    shadow_quaternions = flex[
        [
            "aircraft_body_qx",
            "aircraft_body_qy",
            "aircraft_body_qz",
            "aircraft_body_qw",
        ]
    ].to_numpy(dtype=float)
    shadow_at_traj = Slerp(flex_times, Rotation.from_quat(shadow_quaternions))(
        traj_times
    ).as_quat()
    corrected = trajectory.copy()
    corrected[["x", "y", "z"]] = corrected_positions
    corrected[["qx", "qy", "qz", "qw"]] = shadow_at_traj
    traj_out = args.output_dir / "traj.txt"
    with traj_out.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(
            "# Experimental XY trajectory: each post-init VIO XY increment "
            "rotated by negative causal flex yaw\n"
        )
        corrected.to_csv(
            stream,
            sep=" ",
            header=False,
            index=False,
            float_format="%.9f",
        )

    bias_out: Path | None = None
    if args.bias is not None:
        bias_columns = ["t", "vx", "vy", "vz", "bgx", "bgy", "bgz", "bax", "bay", "baz"]
        bias = read_numeric(args.bias, bias_columns)
        bias_flex = np.radians(
            np.interp(
                bias.t.to_numpy(dtype=float),
                flex_times,
                flex.output_flex_yaw_deg.to_numpy(dtype=float),
            )
        )
        velocity = bias[["vx", "vy", "vz"]].to_numpy(dtype=float)
        bias[["vx", "vy", "vz"]] = rotate_xy(velocity, -bias_flex)
        bias_out = args.output_dir / "traj.txt.bias"
        with bias_out.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(
                "# Experimental velocity: VIO global XY velocity rotated by "
                "negative causal flex yaw\n"
            )
            bias.to_csv(
                stream,
                sep=" ",
                header=False,
                index=False,
                float_format="%.9f",
            )

    receipt = {
        "schema_version": 1,
        "method_name": args.method_name,
        "operation": "rotate_each_post_init_vio_xy_increment_by_negative_causal_flex_yaw",
        "source_traj": str(args.traj),
        "source_traj_sha256": sha256(args.traj),
        "source_bias": str(args.bias) if args.bias else None,
        "source_bias_sha256": sha256(args.bias) if args.bias else None,
        "source_flex_output": str(args.flex_output),
        "source_flex_output_sha256": sha256(args.flex_output),
        "output_traj_sha256": sha256(traj_out),
        "output_bias_sha256": sha256(bias_out) if bias_out else None,
        "sample_count": int(len(trajectory)),
        "initial_flex_deg": float(np.degrees(flex_rad[0])),
        "maximum_absolute_flex_deg": float(np.max(np.abs(np.degrees(flex_rad)))),
    }
    (args.output_dir / "xy_flex_application_receipt.json").write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
