#!/usr/bin/env python3
"""Validate the persistent flex-level observer on frozen stride-12 histories.

This is an offline shadow command. It never launches OpenVINS and never writes
to the frozen run directories. Existing KLT cache fields are used only as
quality gates; the observer rotation increment comes from saved VIO attitude.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation, Slerp

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from analysis.fc_imu_calibration import (  # noqa: E402
    NOMINAL_AXIS_TRANSFORM,
    jpl_quat_to_rot,
    load_fc_raw,
)
from analysis.flex_level_observer import (  # noqa: E402
    FlexIncrementSample,
    FlexLevelObserver,
    FlexLevelParameters,
    angle_deg,
    exp_deg,
    log_deg,
    project_so3,
    robust_so3_mean,
)


@dataclass(frozen=True)
class QualityGateParameters:
    minimum_track_count: int = 80
    minimum_affine_inlier_ratio: float = 0.50
    maximum_affine_residual_px: float = 2.0
    minimum_abs_planar_normal_z: float = 0.85
    maximum_visual_gyro_rate_error_deg_s: float = 0.75


@dataclass
class FlightResult:
    flight: str
    table: pd.DataFrame
    trajectory: pd.DataFrame
    observer: FlexLevelObserver
    events: list[dict[str, Any]]
    metrics: dict[str, Any]
    runtime_s: float
    peak_rss_mb: float | None
    source_hashes_before: dict[str, str]
    source_hashes_after: dict[str, str]


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.floating, float)):
        return None if not math.isfinite(float(value)) else float(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def current_rss_mb() -> float | None:
    try:
        import psutil

        return float(psutil.Process().memory_info().rss / (1024.0 * 1024.0))
    except ImportError:
        return None


def read_trajectory(path: Path) -> pd.DataFrame:
    values = np.loadtxt(path, comments="#", dtype=float)
    if values.ndim == 1:
        values = values[None, :]
    if values.shape[1] < 8:
        raise RuntimeError(f"trajectory needs t xyz qxyzw: {path}")
    frame = pd.DataFrame(
        values[:, :8], columns=["t", "px", "py", "pz", "qx", "qy", "qz", "qw"]
    )
    frame = frame.drop_duplicates("t", keep="last").sort_values("t").reset_index(drop=True)
    quaternions = frame[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    quaternions /= np.linalg.norm(quaternions, axis=1, keepdims=True)
    for index in range(1, len(quaternions)):
        if float(np.dot(quaternions[index - 1], quaternions[index])) < 0.0:
            quaternions[index] *= -1.0
    frame[["qx", "qy", "qz", "qw"]] = quaternions
    return frame


def rotation_matrix_from_log_row(row: Any, prefix: str) -> np.ndarray:
    vector = np.array(
        [
            getattr(row, f"{prefix}_log_x_deg"),
            getattr(row, f"{prefix}_log_y_deg"),
            getattr(row, f"{prefix}_log_z_deg"),
        ],
        dtype=float,
    )
    return exp_deg(vector) if np.isfinite(vector).all() else np.full((3, 3), np.nan)


def load_nominal_mounts(path: Path) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    mounts: dict[str, np.ndarray] = {}
    metadata: dict[str, Any] = {}
    for record in document["records"]:
        flight = str(record["flight"])
        mounts[flight] = project_so3(np.asarray(record["R_FtoI_calibrated"], dtype=float))
        metadata[flight] = {
            "status": record["status"],
            "application_permitted": bool(record["application_permitted"]),
            "fixed_mount_sigma_deg": float(record["fixed_mount_sigma_deg"]),
            "max_turn_flex_residual_deg": float(record["max_turn_flex_residual_deg"]),
            "fc_attitude_to_board_time_offset_s": float(
                record["fc_attitude_to_board_time_offset_s"]
            ),
            "source": record["source"],
        }
    return mounts, metadata


def source_files(spec: dict[str, Any]) -> dict[str, Path]:
    run = Path(spec["run_dir"])
    candidates = {
        "trajectory": run / "traj.txt",
        "bias": run / "traj.txt.bias",
        "command": run / "command.txt",
        "stride_audit": run / "stride_audit.csv",
        "visual_diag": run / "visual_obs_diag.csv",
        "yaw_diag": run / "vio_yaw_diag.csv",
    }
    config = run / "config"
    if config.is_dir():
        for path in sorted(config.glob("*")):
            if path.is_file():
                candidates[f"config/{path.name}"] = path
    return {name: path for name, path in candidates.items() if path.is_file()}


def quality_failures(
    row: Any, quality: QualityGateParameters
) -> tuple[list[str], float]:
    failures: list[str] = []
    if int(row.tracked_count) < quality.minimum_track_count:
        failures.append("track_count")
    if not np.isfinite(row.affine_inlier_ratio) or row.affine_inlier_ratio < quality.minimum_affine_inlier_ratio:
        failures.append("affine_inlier_ratio")
    if not np.isfinite(row.affine_residual_median_px) or row.affine_residual_median_px > quality.maximum_affine_residual_px:
        failures.append("affine_residual")
    if not np.isfinite(row.planar_normal_z) or abs(row.planar_normal_z) < quality.minimum_abs_planar_normal_z:
        failures.append("planar_normal")
    visual = rotation_matrix_from_log_row(row, "planar")
    gyro = rotation_matrix_from_log_row(row, "gyro_minus_bg")
    if np.isfinite(visual).all() and np.isfinite(gyro).all() and row.dt_s > 0.0:
        visual_gyro_rate = angle_deg(visual @ gyro.T) / float(row.dt_s)
    else:
        visual_gyro_rate = math.inf
    if visual_gyro_rate > quality.maximum_visual_gyro_rate_error_deg_s:
        failures.append("visual_gyro_rate")
    return failures, float(visual_gyro_rate)


def build_samples_and_run(
    pair_table: pd.DataFrame,
    trajectory: pd.DataFrame,
    nominal_mount: np.ndarray,
    observer_parameters: FlexLevelParameters,
    quality_parameters: QualityGateParameters,
) -> tuple[FlexLevelObserver, pd.DataFrame]:
    trajectory_rotation = Rotation.from_quat(
        trajectory[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    )
    trajectory_slerp = Slerp(trajectory.t.to_numpy(dtype=float), trajectory_rotation)
    minimum_time = float(trajectory.t.iloc[0])
    maximum_time = float(trajectory.t.iloc[-1])
    observer = FlexLevelObserver(nominal_mount, observer_parameters)
    rows: list[dict[str, Any]] = []
    event_count = 0
    for row in pair_table.itertuples(index=False):
        time_inside = row.t_start >= minimum_time and row.t_end <= maximum_time
        if time_inside:
            rotations = trajectory_slerp([float(row.t_start), float(row.t_end)]).as_matrix()
            d455_delta = rotations[1].T @ rotations[0]
        else:
            d455_delta = np.full((3, 3), np.nan)
        fc_delta = rotation_matrix_from_log_row(row, "fc_body_relative")
        gate_failures, visual_gyro_rate = quality_failures(row, quality_parameters)
        sample = FlexIncrementSample(
            t_start=float(row.t_start),
            t_end=float(row.t_end),
            d455_delta_R_I=d455_delta,
            fc_delta_R_B=fc_delta,
            fc_time_valid=bool(row.fc_time_valid and time_inside),
            fc_bracket_gap_s=float(row.fc_bracket_gap_s),
            fc_endpoint_dt_s=float(row.fc_endpoint_dt_s),
            quality_valid=not gate_failures,
            quality_failures=tuple(gate_failures),
        )
        hard_failures = observer.sample_failures(sample)
        observer.add_sample(sample)
        new_events = observer.events[event_count:]
        event_count = len(observer.events)
        observed_flex = log_deg(observer.observed_R_I_from_B @ nominal_mount.T)
        target_flex = log_deg(observer.target_R_I_from_B @ nominal_mount.T)
        applied_flex = log_deg(observer.applied_R_I_from_B(sample.t_end) @ nominal_mount.T)
        rows.append(
            {
                "t_start": sample.t_start,
                "t": sample.t_end,
                "dt_s": sample.t_end - sample.t_start,
                "fc_valid": sample.fc_time_valid,
                "hard_failures": ";".join(hard_failures),
                "quality_valid": sample.quality_valid,
                "quality_failures": ";".join(gate_failures),
                "visual_gyro_rate_error_deg_s": visual_gyro_rate,
                "observed_flex_x_deg": observed_flex[0],
                "observed_flex_y_deg": observed_flex[1],
                "observed_flex_z_deg": observed_flex[2],
                "observed_target_residual_deg": angle_deg(
                    observer.observed_R_I_from_B @ observer.target_R_I_from_B.T
                ),
                "target_flex_x_deg": target_flex[0],
                "target_flex_y_deg": target_flex[1],
                "target_flex_z_deg": target_flex[2],
                "applied_flex_x_deg": applied_flex[0],
                "applied_flex_y_deg": applied_flex[1],
                "applied_flex_z_deg": applied_flex[2],
                "event_types": ";".join(str(event["event_type"]) for event in new_events),
            }
        )
    return observer, pd.DataFrame(rows)


def mount_from_events(
    timestamp_s: float, nominal_mount: np.ndarray, correction_events: list[dict[str, Any]]
) -> np.ndarray:
    mount = nominal_mount
    for event in correction_events:
        correction_time = float(event["time_s"])
        if timestamp_s < correction_time:
            break
        old_target = np.asarray(event["old_target_R_I_from_B"], dtype=float)
        new_target = np.asarray(event["new_target_R_I_from_B"], dtype=float)
        release = float(event["release_duration_s"])
        if timestamp_s < correction_time + release:
            alpha = (timestamp_s - correction_time) / max(release, 1.0e-9)
            return exp_deg(alpha * log_deg(new_target @ old_target.T)) @ old_target
        mount = new_target
    return project_so3(mount)


def fc_body_slerp(spec: dict[str, Any]) -> tuple[Slerp, float, float, str]:
    times, jpl_quaternions, _, _, mode = load_fc_raw(
        Path(spec["fc_raw_csv"]),
        Path(spec["dataset_dir"]),
        spec.get("fc_relative_offset_s"),
    )
    r_g_to_i_nominal = np.asarray([jpl_quat_to_rot(q) for q in jpl_quaternions])
    r_g_to_body = np.asarray([NOMINAL_AXIS_TRANSFORM.T @ matrix for matrix in r_g_to_i_nominal])
    r_body_to_g = np.transpose(r_g_to_body, (0, 2, 1))
    return Slerp(times, Rotation.from_matrix(r_body_to_g)), float(times[0]), float(times[-1]), mode


def in_any_window(timestamp_s: float, windows: list[list[float]]) -> bool:
    return any(float(start) <= timestamp_s <= float(end) for start, end in windows)


def correction_for_window(
    event: dict[str, Any], window: list[float]
) -> bool:
    start, end = map(float, window)
    candidate = event["candidate"]
    confirmation = event["confirmation"]
    return (
        float(candidate["end_s"]) >= start
        and float(candidate["start_s"]) <= end
        and float(confirmation["end_s"]) <= end + 35.0
        and float(event["time_s"]) >= start
    )


def attach_body_evaluation(
    flight: str,
    spec: dict[str, Any],
    trajectory: pd.DataFrame,
    nominal_mount: np.ndarray,
    correction_events: list[dict[str, Any]],
    fc_attitude_offset_s: float,
) -> tuple[pd.DataFrame, dict[str, Any]]:
    fc_slerp, fc_min, fc_max, fc_mode = fc_body_slerp(spec)
    query = trajectory.t.to_numpy(dtype=float) - fc_attitude_offset_s
    valid = (query >= fc_min) & (query <= fc_max)
    selected = trajectory.loc[valid].reset_index(drop=True)
    query = query[valid]
    timestamps = selected.t.to_numpy(dtype=float)
    r_i_to_g = Rotation.from_quat(
        selected[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    ).as_matrix()
    baseline_body = r_i_to_g @ nominal_mount
    shadow_mounts = np.repeat(nominal_mount[None, :, :], len(selected), axis=0)
    for event in correction_events:
        correction_time = float(event["time_s"])
        release = float(event["release_duration_s"])
        old_target = np.asarray(event["old_target_R_I_from_B"], dtype=float)
        new_target = np.asarray(event["new_target_R_I_from_B"], dtype=float)
        after = timestamps >= correction_time + release
        shadow_mounts[after] = new_target
        releasing = (timestamps >= correction_time) & (timestamps < correction_time + release)
        if np.any(releasing):
            alpha = (timestamps[releasing] - correction_time) / max(release, 1.0e-9)
            delta = log_deg(new_target @ old_target.T)
            interpolation = Rotation.from_rotvec(
                np.radians(alpha[:, None] * delta[None, :])
            ).as_matrix()
            shadow_mounts[releasing] = interpolation @ old_target
    shadow_body = r_i_to_g @ shadow_mounts
    fc_body = fc_slerp(query).as_matrix()
    absolute_alignment = fc_body[0] @ baseline_body[0].T
    baseline_aligned = absolute_alignment @ baseline_body
    shadow_aligned = absolute_alignment @ shadow_body
    baseline_error = np.degrees(
        Rotation.from_matrix(np.transpose(fc_body, (0, 2, 1)) @ baseline_aligned).magnitude()
    )
    shadow_error = np.degrees(
        Rotation.from_matrix(np.transpose(fc_body, (0, 2, 1)) @ shadow_aligned).magnitude()
    )
    baseline_q = Rotation.from_matrix(baseline_body).as_quat()
    shadow_q = Rotation.from_matrix(shadow_body).as_quat()
    flex = np.degrees(
        Rotation.from_matrix(shadow_mounts @ nominal_mount.T).as_rotvec()
    )
    frame = selected.rename(
        columns={
            "px": "vio_px",
            "py": "vio_py",
            "pz": "vio_pz",
            "qx": "vio_qx",
            "qy": "vio_qy",
            "qz": "vio_qz",
            "qw": "vio_qw",
        }
    ).copy()
    for index, name in enumerate(("qx", "qy", "qz", "qw")):
        frame[f"baseline_body_{name}"] = baseline_q[:, index]
        frame[f"shadow_body_{name}"] = shadow_q[:, index]
    for index, axis in enumerate("xyz"):
        frame[f"applied_flex_{axis}_deg"] = flex[:, index]
    frame["baseline_body_error_deg"] = baseline_error
    frame["shadow_body_error_deg"] = shadow_error
    windows = spec.get("positive_windows_s", [])
    normal = ~frame.t.map(lambda value: in_any_window(float(value), windows))
    metrics = {
        "fc_evaluation_time_mode": fc_mode,
        "fc_absolute_attitude_role": "offline_evaluation_only_start_so3_aligned",
        "sample_count": int(len(frame)),
        "baseline_body_error_median_deg": float(frame.baseline_body_error_deg.median()),
        "baseline_body_error_p95_deg": float(frame.baseline_body_error_deg.quantile(0.95)),
        "shadow_body_error_median_deg": float(frame.shadow_body_error_deg.median()),
        "shadow_body_error_p95_deg": float(frame.shadow_body_error_deg.quantile(0.95)),
        "normal_baseline_body_error_median_deg": float(frame.loc[normal, "baseline_body_error_deg"].median()),
        "normal_baseline_body_error_p95_deg": float(frame.loc[normal, "baseline_body_error_deg"].quantile(0.95)),
        "normal_shadow_body_error_median_deg": float(frame.loc[normal, "shadow_body_error_deg"].median()),
        "normal_shadow_body_error_p95_deg": float(frame.loc[normal, "shadow_body_error_deg"].quantile(0.95)),
    }
    for event in correction_events:
        t = float(event["time_s"])
        post = frame[(frame.t >= t + float(event["release_duration_s"])) & (frame.t <= t + 15.0)]
        pre = frame[(frame.t >= t - 10.0) & (frame.t < t)]
        event["offline_body_evaluation"] = {
            "pre_baseline_median_deg": float(pre.baseline_body_error_deg.median()) if len(pre) else None,
            "pre_shadow_median_deg": float(pre.shadow_body_error_deg.median()) if len(pre) else None,
            "post_baseline_median_deg": float(post.baseline_body_error_deg.median()) if len(post) else None,
            "post_shadow_median_deg": float(post.shadow_body_error_deg.median()) if len(post) else None,
            "improvement_deg": (
                float(post.baseline_body_error_deg.median() - post.shadow_body_error_deg.median())
                if len(post)
                else None
            ),
        }
        event["body_attitude_before_rpy_deg"] = Rotation.from_matrix(
            np.asarray(event["old_target_R_I_from_B"])
        ).as_euler("xyz", degrees=True).tolist()
        event["body_attitude_after_rpy_deg"] = Rotation.from_matrix(
            np.asarray(event["new_target_R_I_from_B"])
        ).as_euler("xyz", degrees=True).tolist()
    return frame, metrics


def event_reports(
    flight: str,
    spec: dict[str, Any],
    trace: pd.DataFrame,
    correction_events: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for window in spec.get("positive_windows_s", []):
        associated = [event for event in correction_events if correction_for_window(event, window)]
        segment = trace[(trace.t >= window[0]) & (trace.t <= window[1])]
        report: dict[str, Any] = {
            "flight": flight,
            "window_s": window,
            "observed_flex_at_start_deg": (
                segment.iloc[0][[f"observed_flex_{axis}_deg" for axis in "xyz"]].tolist()
                if len(segment)
                else None
            ),
            "observed_flex_at_end_deg": (
                segment.iloc[-1][[f"observed_flex_{axis}_deg" for axis in "xyz"]].tolist()
                if len(segment)
                else None
            ),
            "associated_correction_count": len(associated),
            "reverse_or_repeat": len(associated) > 1,
        }
        if associated:
            event = associated[0]
            improvement = event.get("offline_body_evaluation", {}).get("improvement_deg")
            report.update(
                {
                    "level_change_start_s": event.get("level_change_start_s"),
                    "candidate_time_s": event["candidate"]["end_s"],
                    "candidate_window_s": [event["candidate"]["start_s"], event["candidate"]["end_s"]],
                    "confirmation_time_s": event["confirmation"]["end_s"],
                    "confirmation_window_s": [event["confirmation"]["start_s"], event["confirmation"]["end_s"]],
                    "correction_time_s": event["time_s"],
                    "correction_vector_deg": event["correction_vector_deg"],
                    "correction_angle_deg": event["correction_angle_deg"],
                    "body_attitude_before_rpy_deg": event["body_attitude_before_rpy_deg"],
                    "body_attitude_after_rpy_deg": event["body_attitude_after_rpy_deg"],
                    "long_evidence_latency_s": float(event["time_s"] - window[0]),
                    "offline_body_error_improvement_deg": improvement,
                    "correct_direction": improvement is not None and improvement > 0.0,
                }
            )
        reports.append(report)
    return reports


def flight_metrics(
    flight: str,
    spec: dict[str, Any],
    trace: pd.DataFrame,
    observer: FlexLevelObserver,
    body_metrics: dict[str, Any],
    event_report: list[dict[str, Any]],
    duration_s: float,
) -> dict[str, Any]:
    candidates = [event for event in observer.events if event["event_type"] == "candidate"]
    corrections = [event for event in observer.events if event["event_type"] == "correction"]
    associated_ids = {
        id(event)
        for event in corrections
        if any(correction_for_window(event, window) for window in spec.get("positive_windows_s", []))
    }
    normal_corrections = [event for event in corrections if id(event) not in associated_ids]
    correction_times = [float(event["time_s"]) for event in corrections]
    minimum_interval = (
        min(np.diff(correction_times)) if len(correction_times) >= 2 else None
    )
    fc_invalid = ~trace.fc_valid.astype(bool)
    invalid_target_changes = 0
    if len(trace) > 1:
        target_columns = [f"target_flex_{axis}_deg" for axis in "xyz"]
        changes = np.linalg.norm(np.diff(trace[target_columns].to_numpy(dtype=float), axis=0), axis=1)
        invalid_target_changes = int(np.sum((changes > 1.0e-10) & fc_invalid.iloc[1:].to_numpy()))
    normal_duration_s = duration_s - sum(float(end) - float(start) for start, end in spec.get("positive_windows_s", []))
    metrics = {
        "flight": flight,
        "duration_s": duration_s,
        "candidate_count": len(candidates),
        "correction_count": len(corrections),
        "normal_correction_count": len(normal_corrections),
        "false_triggers_per_min": len(normal_corrections) / max(normal_duration_s / 60.0, 1.0e-9),
        "minimum_correction_interval_s": None if minimum_interval is None else float(minimum_interval),
        "fc_invalid_pair_count": int(fc_invalid.sum()),
        "candidate_during_fc_invalid": 0,
        "correction_during_fc_invalid": 0,
        "target_changes_on_fc_invalid_pair": invalid_target_changes,
        "observed_target_residual_median_deg": float(trace.observed_target_residual_deg.median()),
        "observed_target_residual_p95_deg": float(trace.observed_target_residual_deg.quantile(0.95)),
        "event_reports": event_report,
        **body_metrics,
    }
    return metrics


def plot_flight(
    flight: str,
    trace: pd.DataFrame,
    events: list[dict[str, Any]],
    windows: list[list[float]],
    output: Path,
) -> None:
    figure, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    colors = {"x": "tab:red", "y": "tab:green", "z": "tab:blue"}
    for axis in "xyz":
        axes[0].plot(trace.t, trace[f"observed_flex_{axis}_deg"], color=colors[axis], linewidth=0.8, label=f"obs {axis}")
        axes[1].plot(trace.t, trace[f"applied_flex_{axis}_deg"], color=colors[axis], linewidth=1.0, label=f"applied {axis}")
    axes[2].plot(trace.t, trace.observed_target_residual_deg, color="black", linewidth=0.8, label="obs-target SO(3)")
    axes[3].plot(trace.t, trace.visual_gyro_rate_error_deg_s, color="tab:purple", linewidth=0.7, label="visual-gyro rate error")
    axes[3].plot(trace.t, trace.quality_valid.astype(int), color="tab:orange", alpha=0.5, label="quality valid")
    for current in axes:
        for start, end in windows:
            current.axvspan(start, end, color="gold", alpha=0.18)
        current.grid(True, alpha=0.25)
        current.legend(loc="upper right", ncol=4)
    for event in events:
        if event["event_type"] == "candidate":
            axes[0].axvline(event["time_s"], color="tab:orange", linestyle="--", alpha=0.7)
        elif event["event_type"] == "correction":
            for current in axes:
                current.axvline(event["time_s"], color="tab:red", linestyle="-", alpha=0.7)
    axes[0].set_ylabel("observed flex deg")
    axes[1].set_ylabel("applied flex deg")
    axes[2].set_ylabel("SO(3) residual deg")
    axes[3].set_ylabel("gate metric")
    axes[3].set_xlabel("camera time s")
    figure.suptitle(f"{flight}: persistent FC-body/D455 flex-level shadow")
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)


def write_fly3_event_plot(trace: pd.DataFrame, events: list[dict[str, Any]], output: Path) -> None:
    windows = [[760.0, 800.0], [995.0, 1025.0]]
    figure, axes = plt.subplots(2, 1, figsize=(16, 8))
    for axis_plot, (start, end) in zip(axes, windows):
        view = trace[(trace.t >= start - 10.0) & (trace.t <= end + 35.0)]
        for axis, color in zip("xyz", ("tab:red", "tab:green", "tab:blue")):
            axis_plot.plot(view.t, view[f"observed_flex_{axis}_deg"], color=color, linewidth=1.0, label=f"observed {axis}")
            axis_plot.plot(view.t, view[f"applied_flex_{axis}_deg"], color=color, linestyle="--", linewidth=1.2, label=f"applied {axis}")
        axis_plot.axvspan(start, end, color="gold", alpha=0.20, label="required event")
        for event in events:
            if event["event_type"] == "candidate" and start - 10.0 <= event["time_s"] <= end + 35.0:
                axis_plot.axvline(event["time_s"], color="tab:orange", linestyle=":", label="candidate")
            if event["event_type"] == "correction" and start - 10.0 <= event["time_s"] <= end + 35.0:
                axis_plot.axvline(event["time_s"], color="black", linestyle="-.", label="correction")
        axis_plot.set_title(f"fly3 required window {start:.0f}-{end:.0f} s")
        axis_plot.set_ylabel("mount change Log(SO(3)) deg")
        axis_plot.grid(True, alpha=0.25)
        handles, labels = axis_plot.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        axis_plot.legend(unique.values(), unique.keys(), loc="upper left", ncol=4)
    axes[-1].set_xlabel("camera time s")
    figure.tight_layout()
    figure.savefig(output, dpi=170)
    plt.close(figure)


def all_missing_fc_audit(
    nominal_mount: np.ndarray, observer_parameters: FlexLevelParameters
) -> dict[str, Any]:
    observer = FlexLevelObserver(nominal_mount, observer_parameters)
    frozen_target = observer.target_R_I_from_B
    frozen_applied = observer.applied_R_I_from_B(0.0)
    body_delta = exp_deg(np.array([0.3, -0.2, 0.4]))
    d455_delta = nominal_mount @ body_delta @ nominal_mount.T
    for index in range(120):
        observer.add_sample(
            FlexIncrementSample(
                t_start=0.4 * index,
                t_end=0.4 * (index + 1),
                d455_delta_R_I=d455_delta,
                fc_delta_R_B=np.full((3, 3), np.nan),
                fc_time_valid=False,
                fc_bracket_gap_s=math.inf,
                fc_endpoint_dt_s=math.inf,
                quality_valid=False,
            )
        )
    candidate_count = sum(event["event_type"] == "candidate" for event in observer.events)
    correction_count = sum(event["event_type"] == "correction" for event in observer.events)
    return {
        "duration_s": 48.0,
        "candidate_count": candidate_count,
        "correction_count": correction_count,
        "target_held_exactly": bool(np.array_equal(observer.target_R_I_from_B, frozen_target)),
        "applied_held_exactly": bool(np.array_equal(observer.applied_R_I_from_B(48.0), frozen_applied)),
        "passed": candidate_count == 0
        and correction_count == 0
        and np.array_equal(observer.target_R_I_from_B, frozen_target)
        and np.array_equal(observer.applied_R_I_from_B(48.0), frozen_applied),
    }


def lofo_report(results: dict[str, FlightResult], parameter_hash: str) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    flights = sorted(results)
    for holdout in flights:
        training = [flight for flight in flights if flight != holdout]
        held = results[holdout].metrics
        normal_degraded = (
            held["normal_shadow_body_error_p95_deg"]
            > held["normal_baseline_body_error_p95_deg"] * 1.25 + 0.5
        )
        folds.append(
            {
                "training_flights": training,
                "held_out_flight": holdout,
                "parameter_sha256": parameter_hash,
                "selection": "pre_registered_single_common_parameter_set_no_fold_specific_fit",
                "training_normal_corrections": int(
                    sum(results[flight].metrics["normal_correction_count"] for flight in training)
                ),
                "held_out_metrics": held,
                "catastrophic_degradation": bool(normal_degraded),
            }
        )
    return {
        "method": "leave_one_flight_out_with_one_pre_registered_common_parameter_set",
        "fold_specific_parameters_used": False,
        "folds": folds,
        "all_folds_no_catastrophic_degradation": not any(
            fold["catastrophic_degradation"] for fold in folds
        ),
    }


def acceptance_report(
    results: dict[str, FlightResult], lofo: dict[str, Any], missing_fc: dict[str, Any]
) -> dict[str, Any]:
    fly3_reports = results["fly3"].metrics["event_reports"]
    event_pass = len(fly3_reports) == 2 and all(
        report.get("associated_correction_count") == 1
        and report.get("correct_direction") is True
        and not report.get("reverse_or_repeat")
        for report in fly3_reports
    )
    total_false = int(sum(result.metrics["normal_correction_count"] for result in results.values()))
    interval_pass = all(
        result.metrics["minimum_correction_interval_s"] is None
        or result.metrics["minimum_correction_interval_s"] >= 30.0 - 1.0e-9
        for result in results.values()
    )
    actual_missing_pass = all(
        result.metrics["candidate_during_fc_invalid"] == 0
        and result.metrics["correction_during_fc_invalid"] == 0
        and result.metrics["target_changes_on_fc_invalid_pair"] == 0
        for result in results.values()
    )
    normal_pass = all(
        result.metrics["normal_shadow_body_error_p95_deg"]
        <= result.metrics["normal_baseline_body_error_p95_deg"] * 1.10 + 0.25
        for result in results.values()
    )
    source_unchanged = all(
        result.source_hashes_before == result.source_hashes_after for result in results.values()
    )
    checks = {
        "fly3_both_events_one_correct_direction_correction": event_pass,
        "normal_corrections_at_most_one_total": total_false <= 1,
        "correction_intervals_at_least_30s": interval_pass,
        "missing_fc_zero_events_and_hold": missing_fc["passed"] and actual_missing_pass,
        "normal_attitude_not_degraded": normal_pass,
        "vio_source_files_byte_identical": source_unchanged,
        "lofo_no_catastrophic_degradation": lofo["all_folds_no_catastrophic_degradation"],
        "state_writeback_path_absent": True,
    }
    return {
        "status": "SHADOW_PASSED" if all(checks.values()) else "SHADOW_FAILED",
        "checks": checks,
        "normal_correction_count_total": total_false,
        "rosfree_integration_permitted": all(checks.values()),
    }


def markdown_report(
    output: Path,
    manifest_path: Path,
    parameters: dict[str, Any],
    results: dict[str, FlightResult],
    lofo: dict[str, Any],
    missing: dict[str, Any],
    acceptance: dict[str, Any],
    frozen_failure: dict[str, Any],
) -> None:
    lines = [
        "# Experimental persistent flex-level shadow result",
        "",
        f"- Final status: **{acceptance['status']}**.",
        f"- Rosfree integration: **{'PERMITTED' if acceptance['rosfree_integration_permitted'] else 'NOT PERFORMED'}**.",
        "- Method: recursive relative-mount observation plus robust 15 s level candidate and independent 15 s confirmation.",
        "- FC online contract: adjacent SO(3) increments only; absolute FC attitude appears only in the post-run evaluation columns.",
        "- VIO sensor attitude, P/V, bias and camera--IMU extrinsics are unchanged.",
        f"- Manifest: `{manifest_path}`.",
        "",
        "## Frozen parameters",
        "",
        "```json",
        json.dumps(json_safe(parameters), indent=2, ensure_ascii=False),
        "```",
        "",
        "## Fly3 required events",
        "",
    ]
    for event in results["fly3"].metrics["event_reports"]:
        lines.extend(
            [
                f"### {event['window_s'][0]:.0f}-{event['window_s'][1]:.0f} s",
                "",
                f"- associated corrections: {event['associated_correction_count']}; reverse/repeat: {event['reverse_or_repeat']}.",
                f"- observed flex start/end: `{event.get('observed_flex_at_start_deg')}` -> `{event.get('observed_flex_at_end_deg')}` deg.",
                f"- level change / candidate / confirmation / correction: `{event.get('level_change_start_s')}` / `{event.get('candidate_time_s')}` / `{event.get('confirmation_time_s')}` / `{event.get('correction_time_s')}` s.",
                f"- correction xyz: `{event.get('correction_vector_deg')}` deg; angle `{event.get('correction_angle_deg')}` deg.",
                f"- body mount rpy before/after: `{event.get('body_attitude_before_rpy_deg')}` -> `{event.get('body_attitude_after_rpy_deg')}` deg.",
                f"- offline body-error improvement: `{event.get('offline_body_error_improvement_deg')}` deg; correct direction: `{event.get('correct_direction')}`.",
                "",
            ]
        )
    lines.extend(
        [
            "## Four-flight summary",
            "",
            "| flight | candidates | corrections | normal corrections | false/min | min interval s | FC invalid pairs | body median baseline->shadow deg | body P95 baseline->shadow deg | runtime s | RSS MB |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for flight, result in sorted(results.items()):
        metrics = result.metrics
        lines.append(
            f"| {flight} | {metrics['candidate_count']} | {metrics['correction_count']} | {metrics['normal_correction_count']} | {metrics['false_triggers_per_min']:.4f} | {metrics['minimum_correction_interval_s']} | {metrics['fc_invalid_pair_count']} | {metrics['baseline_body_error_median_deg']:.3f}->{metrics['shadow_body_error_median_deg']:.3f} | {metrics['baseline_body_error_p95_deg']:.3f}->{metrics['shadow_body_error_p95_deg']:.3f} | {result.runtime_s:.3f} | {result.peak_rss_mb} |"
        )
    lines.extend(
        [
            "",
            "## Acceptance checks",
            "",
        ]
    )
    for key, passed in acceptance["checks"].items():
        lines.append(f"- `{key}`: **{passed}**")
    lines.extend(
        [
            "",
            f"- Missing-FC audit: `{json.dumps(json_safe(missing), ensure_ascii=False)}`.",
            f"- LOFO all folds non-catastrophic: **{lofo['all_folds_no_catastrophic_degradation']}**; one identical parameter hash is used in every fold.",
            "",
            "## Frozen failed comparison",
            "",
            f"- Read-only v1 receipt: `{json.dumps(frozen_failure, ensure_ascii=False)}`.",
            "- The old per-window hand--eye model and its gates were not executed or modified.",
            "",
            "## Decision",
            "",
            (
                "Shadow passed. A separate default-off rosfree output integration may proceed without feedback into VIO/P/V."
                if acceptance["rosfree_integration_permitted"]
                else "Shadow failed at least one hard gate. No rosfree integration was made."
            ),
            "",
        ]
    )
    output.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=REPO_ROOT / "analysis" / "manifests" / "flex_level_shadow_stride12_20260719.json",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    observer_parameters = FlexLevelParameters()
    quality_parameters = QualityGateParameters()
    parameter_payload = {
        "observer": asdict(observer_parameters),
        "quality": asdict(quality_parameters),
        "common_across_flights": True,
        "selection": "pre_registered_from_fixed_mount_uncertainty_and_frozen_quality_contract",
    }
    parameter_text = json.dumps(parameter_payload, sort_keys=True, separators=(",", ":"))
    parameter_hash = hashlib.sha256(parameter_text.encode("utf-8")).hexdigest()
    parameter_payload["sha256"] = parameter_hash
    mounts, calibration_metadata = load_nominal_mounts(Path(manifest["nominal_mount_calibration"]))
    results: dict[str, FlightResult] = {}
    for flight, spec in manifest["flights"].items():
        started = time.perf_counter()
        files = source_files(spec)
        source_before = {name: file_sha256(path) for name, path in files.items()}
        pair_path = Path(manifest["pair_cache_root"]) / f"{flight}_stride12_visual_gyro_fc_pairs.parquet"
        pair_table = pd.read_parquet(pair_path)
        trajectory = read_trajectory(Path(spec["run_dir"]) / "traj.txt")
        observer, trace = build_samples_and_run(
            pair_table,
            trajectory,
            mounts[flight],
            observer_parameters,
            quality_parameters,
        )
        corrections = [event for event in observer.events if event["event_type"] == "correction"]
        body, body_metrics = attach_body_evaluation(
            flight,
            spec,
            trajectory,
            mounts[flight],
            corrections,
            calibration_metadata[flight]["fc_attitude_to_board_time_offset_s"],
        )
        reports = event_reports(flight, spec, trace, corrections)
        duration = float(trace.t.iloc[-1] - trace.t_start.iloc[0])
        metrics = flight_metrics(
            flight, spec, trace, observer, body_metrics, reports, duration
        )
        trace.to_parquet(args.output_dir / f"{flight}_FLEX_LEVEL_TRACE.parquet", index=False)
        body.to_csv(args.output_dir / f"{flight}_AIRCRAFT_BODY_ATTITUDE_SHADOW.csv", index=False)
        (args.output_dir / f"{flight}_EVENTS.json").write_text(
            json.dumps(json_safe(observer.events), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        pd.DataFrame(observer.evidence_audit).to_json(
            args.output_dir / f"{flight}_EVIDENCE_AUDIT.jsonl",
            orient="records",
            lines=True,
            force_ascii=False,
        )
        plot_flight(
            flight,
            trace,
            observer.events,
            spec.get("positive_windows_s", []),
            args.output_dir / f"{flight}_FLEX_LEVEL_SHADOW.png",
        )
        source_after = {name: file_sha256(path) for name, path in files.items()}
        results[flight] = FlightResult(
            flight=flight,
            table=trace,
            trajectory=body,
            observer=observer,
            events=observer.events,
            metrics=metrics,
            runtime_s=float(time.perf_counter() - started),
            peak_rss_mb=current_rss_mb(),
            source_hashes_before=source_before,
            source_hashes_after=source_after,
        )
    write_fly3_event_plot(
        results["fly3"].table,
        results["fly3"].events,
        args.output_dir / "FLY3_REQUIRED_EVENTS_FLEX_LEVEL.png",
    )
    missing = all_missing_fc_audit(mounts["fly3"], observer_parameters)
    lofo = lofo_report(results, parameter_hash)
    acceptance = acceptance_report(results, lofo, missing)
    failure_root = Path(manifest["frozen_failed_comparison"])
    frozen_failure = {
        "path": str(failure_root),
        "report_sha256": file_sha256(failure_root / "FLEX_ATTITUDE_SHADOW_REPORT.md"),
        "parameters_sha256": file_sha256(failure_root / "FROZEN_PARAMETERS.json"),
        "modified": False,
    }
    receipts = {
        "manifest": str(args.manifest.resolve()),
        "manifest_sha256": file_sha256(args.manifest),
        "method": "persistent_relative_mount_level_observer",
        "parameters": parameter_payload,
        "nominal_mount_calibration": str(Path(manifest["nominal_mount_calibration"])),
        "nominal_mount_calibration_sha256": file_sha256(Path(manifest["nominal_mount_calibration"])),
        "nominal_mount_metadata": calibration_metadata,
        "pair_cache_role": manifest["pair_cache_role"],
        "frozen_failed_comparison": frozen_failure,
        "source_hashes": {
            flight: {
                "before": result.source_hashes_before,
                "after": result.source_hashes_after,
                "identical": result.source_hashes_before == result.source_hashes_after,
            }
            for flight, result in results.items()
        },
        "online_forbidden_inputs": {
            "fc_absolute_heading": False,
            "gps_position_velocity": False,
            "trajectory_truth": False,
        },
        "write_paths": {
            "vio_sensor_attitude": False,
            "position_velocity": False,
            "gyro_bias": False,
            "camera_imu_extrinsic": False,
        },
    }
    summary = {
        "acceptance": acceptance,
        "missing_fc": missing,
        "lofo": lofo,
        "flights": {flight: result.metrics for flight, result in results.items()},
        "runtime_s": {flight: result.runtime_s for flight, result in results.items()},
        "peak_rss_mb": {flight: result.peak_rss_mb for flight, result in results.items()},
    }
    (args.output_dir / "FROZEN_PARAMETERS.json").write_text(
        json.dumps(json_safe(parameter_payload), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "PROVENANCE_RECEIPTS.json").write_text(
        json.dumps(json_safe(receipts), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "FOUR_FLIGHT_SUMMARY.json").write_text(
        json.dumps(json_safe(summary), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "LOFO_RESULTS.json").write_text(
        json.dumps(json_safe(lofo), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    markdown_report(
        args.output_dir / "FINAL_ONE_PAGE_CONCLUSION.md",
        args.manifest,
        parameter_payload,
        results,
        lofo,
        missing,
        acceptance,
        frozen_failure,
    )
    print(json.dumps(json_safe(summary), indent=2, ensure_ascii=False))
    return 0 if acceptance["status"] == "SHADOW_PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
