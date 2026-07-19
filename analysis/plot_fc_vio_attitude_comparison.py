#!/usr/bin/env python3
"""Plot FC and VIO body-equivalent attitude on one frozen time axis.

The fixed FC-to-D455 installation rotation is estimated once at the first
valid post-initialization VIO sample. It is then held constant. All subsequent
curves therefore show attitude *changes* beyond that initial mount, not the
constant installation angle itself.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_tum(path: Path) -> tuple[np.ndarray, Rotation]:
    data = np.loadtxt(path, comments="#")
    if data.ndim != 2 or data.shape[1] < 8:
        raise ValueError(f"expected TUM t xyz qxyzw: {path}")
    finite = np.isfinite(data[:, [0, 4, 5, 6, 7]]).all(axis=1)
    data = data[finite]
    return data[:, 0], Rotation.from_quat(data[:, 4:8])


def load_fc(path: Path) -> tuple[np.ndarray, Rotation]:
    times: list[float] = []
    quaternions: list[list[float]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = csv.DictReader(line for line in stream if not line.startswith("#"))
        for row in rows:
            if int(row["valid"]) != 1:
                continue
            values = [float(row[key]) for key in ("qx", "qy", "qz", "qw")]
            time = float(row["camera_time_s"])
            if np.isfinite([time, *values]).all():
                times.append(time)
                quaternions.append(values)
    if len(times) < 2:
        raise ValueError(f"fewer than two valid FC samples: {path}")
    times_array = np.asarray(times)
    order = np.argsort(times_array)
    times_array = times_array[order]
    quaternions_array = np.asarray(quaternions)[order]
    unique = np.concatenate(([True], np.diff(times_array) > 1e-9))
    return times_array[unique], Rotation.from_quat(quaternions_array[unique])


def wrap_deg(values: np.ndarray) -> np.ndarray:
    return (values + 180.0) % 360.0 - 180.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vio-traj", type=Path, required=True)
    parser.add_argument("--fc-attitude", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--flight", default="fly1")
    parser.add_argument("--t0", type=float)
    parser.add_argument("--t1", type=float)
    parser.add_argument(
        "--mount-time", type=float,
        help="post-init time used to freeze the nominal mount; default first common VIO sample",
    )
    args = parser.parse_args()

    vio_time, vio_rotation = load_tum(args.vio_traj)
    fc_time, fc_rotation = load_fc(args.fc_attitude)
    common_start = max(vio_time[0], fc_time[0])
    common_end = min(vio_time[-1], fc_time[-1])
    common = (vio_time >= common_start) & (vio_time <= common_end)
    common_time = vio_time[common]
    common_vio_rotation = vio_rotation[common]
    if len(common_time) < 2:
        raise ValueError(f"no common interval in [{common_start}, {common_end}]")
    common_fc_at_vio = Slerp(fc_time, fc_rotation)(common_time)

    mount_time = args.mount_time if args.mount_time is not None else common_time[0]
    mount_index = int(np.searchsorted(common_time, mount_time, side="left"))
    if mount_index >= len(common_time):
        raise ValueError(f"mount time {mount_time} is after common data")

    # R_I_from_B maps the aircraft body frame into the D455 IMU frame.
    # Freeze it exactly once at the first common post-init sample.
    nominal_R_I_from_B = (
        common_vio_rotation[mount_index].as_matrix().T
        @ common_fc_at_vio[mount_index].as_matrix()
    )
    vio_body_matrix = common_vio_rotation.as_matrix() @ nominal_R_I_from_B
    vio_body = Rotation.from_matrix(vio_body_matrix)

    fc_rpy_rad = common_fc_at_vio.as_euler("xyz")
    vio_rpy_rad = vio_body.as_euler("xyz")
    fc_rpy_deg = np.rad2deg(fc_rpy_rad)
    vio_rpy_deg = np.rad2deg(vio_rpy_rad)
    # Keep roll continuous because aircraft FRD roll sits on the +/-180 branch.
    # Keep yaw wrapped so local FC/VIO differences remain visually resolvable.
    for index in (0,):
        fc_rpy_deg[:, index] = np.rad2deg(np.unwrap(fc_rpy_rad[:, index]))
        vio_rpy_deg[:, index] = np.rad2deg(np.unwrap(vio_rpy_rad[:, index]))
        shift_turns = np.round(
            (fc_rpy_deg[mount_index, index] - vio_rpy_deg[mount_index, index])
            / 360.0
        )
        vio_rpy_deg[:, index] += 360.0 * shift_turns

    # Frozen comparison convention: R_err = R_FC^{-1} R_VIO_body.
    error_rotation = Rotation.from_matrix(
        np.swapaxes(common_fc_at_vio.as_matrix(), 1, 2) @ vio_body_matrix
    )
    error_log_deg = np.rad2deg(error_rotation.as_rotvec())
    error_euler_deg = wrap_deg(np.rad2deg(error_rotation.as_euler("xyz")))

    plot_start = max(common_time[mount_index], args.t0 if args.t0 is not None else -np.inf)
    plot_end = min(common_end, args.t1 if args.t1 is not None else np.inf)
    keep = (common_time >= plot_start) & (common_time <= plot_end)
    sample_time = common_time[keep]
    fc_rpy_deg = fc_rpy_deg[keep]
    vio_rpy_deg = vio_rpy_deg[keep]
    error_log_deg = error_log_deg[keep]
    error_euler_deg = error_euler_deg[keep]
    if len(sample_time) < 2:
        raise ValueError(f"no plot samples in [{plot_start}, {plot_end}]")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / f"{args.flight}_fc_vio_attitude_comparison.csv"
    columns = np.column_stack((
        sample_time,
        fc_rpy_deg,
        vio_rpy_deg,
        error_log_deg,
        error_euler_deg,
    ))
    np.savetxt(
        csv_path,
        columns,
        delimiter=",",
        header=(
            "camera_time_s,fc_roll_deg,fc_pitch_deg,fc_yaw_wrapped_deg,"
            "vio_body_roll_deg,vio_body_pitch_deg,vio_body_yaw_wrapped_deg,"
            "so3_error_x_deg,so3_error_y_deg,so3_error_z_deg,"
            "error_euler_roll_deg,error_euler_pitch_deg,error_euler_yaw_deg"
        ),
        comments="",
        fmt="%.9f",
    )

    relative_time = sample_time - common_time[mount_index]
    names = ("roll", "pitch", "yaw")
    axis_labels = ("Roll / deg", "Pitch / deg", "Yaw / deg (wrapped)")
    fig, axes = plt.subplots(2, 3, figsize=(18, 9), sharex=True)
    for axis, name, label, index in zip(axes[0], names, axis_labels, range(3)):
        axis.plot(relative_time, fc_rpy_deg[:, index], label="FC body", linewidth=1.35)
        axis.plot(relative_time, vio_rpy_deg[:, index], label="VIO + frozen mount", linewidth=1.05)
        axis.set_title(name.capitalize())
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    for axis, name, index in zip(axes[1], names, range(3)):
        axis.plot(relative_time, error_log_deg[:, index], label=f"Log(Rerr) {name}", linewidth=1.2)
        axis.plot(relative_time, error_euler_deg[:, index], "--", label=f"Euler error {name}", linewidth=0.8, alpha=0.7)
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.set_ylabel("VIO − FC / deg")
        axis.set_xlabel(f"Seconds since nominal mount freeze at {common_time[mount_index]:.3f} s")
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    fig.suptitle(
        f"{args.flight}: FC vs VIO three-axis attitude; fixed initial mount removed\n"
        r"$R_{err}=R_{FC}^{T}R_{VIO+mount}$; FC interpolated to frozen stride-12 VIO timestamps",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    png_path = args.output_dir / f"{args.flight}_fc_vio_attitude_comparison.png"
    fig.savefig(png_path, dpi=170)
    plt.close(fig)

    attitude_fig, attitude_axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    for axis, name, label, index in zip(attitude_axes, names, axis_labels, range(3)):
        axis.plot(relative_time, fc_rpy_deg[:, index], label="FC body", linewidth=1.35)
        axis.plot(relative_time, vio_rpy_deg[:, index], label="VIO + frozen mount", linewidth=1.05)
        axis.set_ylabel(label)
        axis.set_title(name.capitalize())
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    attitude_axes[-1].set_xlabel(f"Seconds since nominal mount freeze at {common_time[mount_index]:.3f} s")
    attitude_fig.suptitle(f"{args.flight}: FC and VIO three-axis attitude; fixed initial mount removed")
    attitude_fig.tight_layout(rect=(0, 0, 1, 0.96))
    attitude_path = args.output_dir / f"{args.flight}_fc_vio_three_axis_attitude.png"
    attitude_fig.savefig(attitude_path, dpi=170)
    plt.close(attitude_fig)

    error_fig, error_axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    for axis, name, index in zip(error_axes, names, range(3)):
        axis.plot(relative_time, error_log_deg[:, index], label=f"Log(Rerr) {name}", linewidth=1.2)
        axis.plot(relative_time, error_euler_deg[:, index], "--", label=f"Euler error {name}", linewidth=0.8, alpha=0.7)
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.set_ylabel("VIO − FC / deg")
        axis.set_title(name.capitalize())
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
    error_axes[-1].set_xlabel(f"Seconds since nominal mount freeze at {common_time[mount_index]:.3f} s")
    error_fig.suptitle(
        f"{args.flight}: VIO minus FC attitude error; "
        r"$R_{err}=R_{FC}^{T}R_{VIO+mount}$"
    )
    error_fig.tight_layout(rect=(0, 0, 1, 0.96))
    error_path = args.output_dir / f"{args.flight}_fc_vio_so3_error.png"
    error_fig.savefig(error_path, dpi=170)
    plt.close(error_fig)

    abs_error = np.abs(error_log_deg)
    metadata = {
        "flight": args.flight,
        "vio_traj": str(args.vio_traj.resolve()),
        "fc_attitude": str(args.fc_attitude.resolve()),
        "time_start_s": float(sample_time[0]),
        "time_end_s": float(sample_time[-1]),
        "nominal_mount_time_s": float(common_time[mount_index]),
        "sample_count": int(len(sample_time)),
        "nominal_mount_definition": "R_I_from_B = R_ItoG(init)^T * R_BtoG(init)",
        "error_definition": "R_err = R_FC_to_G^T * R_VIO_body_to_G",
        "so3_log_error_median_abs_deg": dict(zip(names, np.median(abs_error, axis=0).tolist())),
        "so3_log_error_p95_abs_deg": dict(zip(names, np.percentile(abs_error, 95, axis=0).tolist())),
        "so3_log_error_max_abs_deg": dict(zip(names, np.max(abs_error, axis=0).tolist())),
        "csv": str(csv_path.resolve()),
        "plot": str(png_path.resolve()),
        "attitude_plot": str(attitude_path.resolve()),
        "error_plot": str(error_path.resolve()),
    }
    metadata_path = args.output_dir / f"{args.flight}_fc_vio_attitude_comparison.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
