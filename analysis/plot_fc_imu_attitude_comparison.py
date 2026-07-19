#!/usr/bin/env python3
"""Compare FC attitude with open-loop D455 gyro integration.

The script uses only an existing stride-12 trajectory to obtain the actual VIO
initial attitude and the saved historical gyro-bias estimate. It does not run
the estimator. A fixed FC-to-D455 mount is frozen at initialization, then both
raw gyro and gyro-minus-bg are integrated in SO(3).
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


def load_tum(path: Path) -> tuple[np.ndarray, Rotation]:
    data = np.loadtxt(path, comments="#")
    finite = np.isfinite(data[:, [0, 4, 5, 6, 7]]).all(axis=1)
    data = data[finite]
    return data[:, 0], Rotation.from_quat(data[:, 4:8])


def load_bias(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, comments="#")
    finite = np.isfinite(data[:, [0, 4, 5, 6]]).all(axis=1)
    data = data[finite]
    return data[:, 0], data[:, 4:7]


def load_imu(path: Path) -> tuple[np.ndarray, np.ndarray]:
    table = pd.read_csv(path)
    table = table.rename(columns={table.columns[0]: table.columns[0].lstrip("#")})
    time = table["t_ns"].to_numpy(dtype=float) * 1e-9
    omega = table[["wx", "wy", "wz"]].to_numpy(dtype=float)
    finite = np.isfinite(time) & np.isfinite(omega).all(axis=1)
    return time[finite], omega[finite]


def load_fc(path: Path) -> tuple[np.ndarray, Rotation]:
    times: list[float] = []
    quaternions: list[list[float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = csv.DictReader(line for line in stream if not line.startswith("#"))
        for row in rows:
            if int(row["valid"]) != 1:
                continue
            time = float(row["camera_time_s"])
            quaternion = [float(row[key]) for key in ("qx", "qy", "qz", "qw")]
            if np.isfinite([time, *quaternion]).all():
                times.append(time)
                quaternions.append(quaternion)
    time = np.asarray(times)
    order = np.argsort(time)
    time = time[order]
    quaternion = np.asarray(quaternions)[order]
    unique = np.concatenate(([True], np.diff(time) > 1e-9))
    return time[unique], Rotation.from_quat(quaternion[unique])


def integrate_attitude(
    times: np.ndarray,
    omega: np.ndarray,
    initial_R_ItoG: np.ndarray,
) -> np.ndarray:
    matrices = np.empty((len(times), 3, 3), dtype=float)
    matrices[0] = initial_R_ItoG
    for index, dt in enumerate(np.diff(times), start=1):
        midpoint = 0.5 * (omega[index - 1] + omega[index])
        matrices[index] = (
            matrices[index - 1] @ Rotation.from_rotvec(midpoint * dt).as_matrix()
        )
    return matrices


def wrap_deg(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def continuous_rpy(rotation: Rotation, anchor: np.ndarray | None = None) -> np.ndarray:
    radians = rotation.as_euler("xyz")
    degrees = np.rad2deg(radians)
    # Roll needs continuity because FRD roll sits on the +/-180 branch. Keep
    # yaw wrapped so local FC/IMU differences are not compressed by many turns.
    for index in (0,):
        degrees[:, index] = np.rad2deg(np.unwrap(radians[:, index]))
        if anchor is not None:
            turns = np.round((anchor[index] - degrees[0, index]) / 360.0)
            degrees[:, index] += 360.0 * turns
    return degrees


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vio-traj", type=Path, required=True)
    parser.add_argument("--vio-bias", type=Path, required=True)
    parser.add_argument("--imu-csv", type=Path, required=True)
    parser.add_argument("--fc-attitude", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--flight", default="fly1")
    parser.add_argument("--t0", type=float)
    parser.add_argument("--t1", type=float)
    parser.add_argument("--mount-time", type=float)
    args = parser.parse_args()

    vio_time, vio_rotation = load_tum(args.vio_traj)
    bias_time, bias = load_bias(args.vio_bias)
    imu_time, imu_omega = load_imu(args.imu_csv)
    fc_time, fc_rotation = load_fc(args.fc_attitude)

    common_start = max(vio_time[0], bias_time[0], imu_time[0], fc_time[0])
    common_end = min(vio_time[-1], bias_time[-1], imu_time[-1], fc_time[-1])
    requested_mount_time = args.mount_time if args.mount_time is not None else common_start
    mount_index = int(np.searchsorted(vio_time, requested_mount_time, side="left"))
    if mount_index >= len(vio_time) or vio_time[mount_index] > common_end:
        raise ValueError("mount time lies outside common data")
    mount_time = float(vio_time[mount_index])
    initial_vio = vio_rotation[mount_index]
    fc_slerp = Slerp(fc_time, fc_rotation)
    initial_fc = fc_slerp([mount_time])[0]
    nominal_R_I_from_B = initial_vio.as_matrix().T @ initial_fc.as_matrix()

    integration_end = min(common_end, args.t1 if args.t1 is not None else common_end)
    interior = imu_time[(imu_time > mount_time) & (imu_time <= integration_end)]
    integration_time = np.concatenate(([mount_time], interior))
    omega = np.column_stack([
        np.interp(integration_time, imu_time, imu_omega[:, axis])
        for axis in range(3)
    ])
    bg = np.column_stack([
        np.interp(integration_time, bias_time, bias[:, axis])
        for axis in range(3)
    ])
    raw_R_ItoG = integrate_attitude(integration_time, omega, initial_vio.as_matrix())
    corrected_R_ItoG = integrate_attitude(
        integration_time, omega - bg, initial_vio.as_matrix()
    )

    raw_body = Rotation.from_matrix(raw_R_ItoG @ nominal_R_I_from_B)
    corrected_body = Rotation.from_matrix(corrected_R_ItoG @ nominal_R_I_from_B)
    fc_at_imu = fc_slerp(integration_time)

    plot_start = max(mount_time, args.t0 if args.t0 is not None else mount_time)
    keep = integration_time >= plot_start
    sample_time = integration_time[keep]
    fc_at_imu = fc_at_imu[keep]
    raw_body = raw_body[keep]
    corrected_body = corrected_body[keep]
    bg = bg[keep]

    fc_rpy = continuous_rpy(fc_at_imu)
    corrected_rpy = continuous_rpy(corrected_body, fc_rpy[0])
    raw_rpy = continuous_rpy(raw_body, fc_rpy[0])
    fc_matrix = fc_at_imu.as_matrix()
    corrected_error = Rotation.from_matrix(
        np.swapaxes(fc_matrix, 1, 2) @ corrected_body.as_matrix()
    )
    raw_error = Rotation.from_matrix(
        np.swapaxes(fc_matrix, 1, 2) @ raw_body.as_matrix()
    )
    corrected_log = np.rad2deg(corrected_error.as_rotvec())
    raw_log = np.rad2deg(raw_error.as_rotvec())
    corrected_euler = wrap_deg(np.rad2deg(corrected_error.as_euler("xyz")))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / f"{args.flight}_fc_imu_attitude_comparison.csv"
    np.savetxt(
        csv_path,
        np.column_stack((
            sample_time, fc_rpy, corrected_rpy, raw_rpy,
            corrected_log, raw_log, corrected_euler, bg,
        )),
        delimiter=",",
        header=(
            "camera_time_s,fc_roll_deg,fc_pitch_deg,fc_yaw_wrapped_deg,"
            "gyro_bg_roll_deg,gyro_bg_pitch_deg,gyro_bg_yaw_wrapped_deg,"
            "gyro_raw_roll_deg,gyro_raw_pitch_deg,gyro_raw_yaw_wrapped_deg,"
            "gyro_bg_so3_error_x_deg,gyro_bg_so3_error_y_deg,gyro_bg_so3_error_z_deg,"
            "gyro_raw_so3_error_x_deg,gyro_raw_so3_error_y_deg,gyro_raw_so3_error_z_deg,"
            "gyro_bg_error_euler_roll_deg,gyro_bg_error_euler_pitch_deg,"
            "gyro_bg_error_euler_yaw_deg,bg_x_radps,bg_y_radps,bg_z_radps"
        ),
        comments="",
        fmt="%.9f",
    )

    relative_time = sample_time - mount_time
    names = ("roll", "pitch", "yaw")
    labels = ("Roll / deg", "Pitch / deg", "Yaw / deg (wrapped)")
    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    for axis, name, label, index in zip(axes[0], names, labels, range(3)):
        axis.plot(relative_time, fc_rpy[:, index], label="FC body", linewidth=1.35)
        axis.plot(relative_time, corrected_rpy[:, index], label="gyro − saved bg", linewidth=1.0)
        axis.plot(relative_time, raw_rpy[:, index], label="raw gyro", linewidth=0.75, alpha=0.7)
        axis.set_title(name.capitalize())
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    for axis, name, index in zip(axes[1], names, range(3)):
        axis.plot(relative_time, corrected_log[:, index], label=f"gyro−bg Log(Rerr) {name}", linewidth=1.1)
        axis.plot(relative_time, raw_log[:, index], label=f"raw gyro Log(Rerr) {name}", linewidth=0.75, alpha=0.7)
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.set_ylabel("IMU − FC / deg")
        axis.set_xlabel(f"Seconds since mount freeze at {mount_time:.3f} s")
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    fig.suptitle(
        f"{args.flight}: FC vs open-loop D455 gyro integration; fixed initial mount removed\n"
        r"$R_{err}=R_{FC}^{T}R_{gyro+mount}$; saved stride-12 $b_g(t)$",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    png_path = args.output_dir / f"{args.flight}_fc_imu_attitude_comparison.png"
    fig.savefig(png_path, dpi=170)
    plt.close(fig)

    attitude_fig, attitude_axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    for axis, name, label, index in zip(attitude_axes, names, labels, range(3)):
        axis.plot(relative_time, fc_rpy[:, index], label="FC body", linewidth=1.35)
        axis.plot(relative_time, corrected_rpy[:, index], label="gyro − saved bg", linewidth=1.0)
        axis.plot(relative_time, raw_rpy[:, index], label="raw gyro", linewidth=0.75, alpha=0.7)
        axis.set_ylabel(label)
        axis.set_title(name.capitalize())
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    attitude_axes[-1].set_xlabel(f"Seconds since mount freeze at {mount_time:.3f} s")
    attitude_fig.suptitle(
        f"{args.flight}: FC and open-loop D455 gyro three-axis attitude; fixed initial mount removed"
    )
    attitude_fig.tight_layout(rect=(0, 0, 1, 0.96))
    attitude_path = args.output_dir / f"{args.flight}_fc_imu_three_axis_attitude.png"
    attitude_fig.savefig(attitude_path, dpi=170)
    plt.close(attitude_fig)

    error_fig, error_axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    for axis, name, index in zip(error_axes, names, range(3)):
        axis.plot(relative_time, corrected_log[:, index], label=f"gyro−bg Log(Rerr) {name}", linewidth=1.1)
        axis.plot(relative_time, raw_log[:, index], label=f"raw gyro Log(Rerr) {name}", linewidth=0.75, alpha=0.7)
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.set_ylabel("IMU − FC / deg")
        axis.set_title(name.capitalize())
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    error_axes[-1].set_xlabel(f"Seconds since mount freeze at {mount_time:.3f} s")
    error_fig.suptitle(
        f"{args.flight}: open-loop D455 gyro minus FC attitude error; "
        r"$R_{err}=R_{FC}^{T}R_{gyro+mount}$"
    )
    error_fig.tight_layout(rect=(0, 0, 1, 0.96))
    error_path = args.output_dir / f"{args.flight}_fc_imu_so3_error.png"
    error_fig.savefig(error_path, dpi=170)
    plt.close(error_fig)

    def stats(error: np.ndarray) -> dict[str, dict[str, float]]:
        absolute = np.abs(error)
        return {
            name: {
                "median_abs_deg": float(np.median(absolute[:, index])),
                "p95_abs_deg": float(np.percentile(absolute[:, index], 95)),
                "max_abs_deg": float(np.max(absolute[:, index])),
                "final_deg": float(error[-1, index]),
            }
            for index, name in enumerate(names)
        }

    metadata = {
        "flight": args.flight,
        "time_start_s": float(sample_time[0]),
        "time_end_s": float(sample_time[-1]),
        "mount_time_s": mount_time,
        "sample_count": int(len(sample_time)),
        "integration": "R_ItoG[k+1] = R_ItoG[k] Exp((gyro-bg)_midpoint dt)",
        "nominal_mount_definition": "R_I_from_B = R_ItoG(init)^T * R_BtoG(init)",
        "error_definition": "R_err = R_FC_to_G^T * R_gyro_body_to_G",
        "gyro_minus_bg_so3_error": stats(corrected_log),
        "raw_gyro_so3_error": stats(raw_log),
        "csv": str(csv_path.resolve()),
        "plot": str(png_path.resolve()),
        "attitude_plot": str(attitude_path.resolve()),
        "error_plot": str(error_path.resolve()),
    }
    metadata_path = args.output_dir / f"{args.flight}_fc_imu_attitude_comparison.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
