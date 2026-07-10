#!/usr/bin/env python3
"""Build and index the OpenVINS master flight data table.

This tool keeps estimator-raw, fixed navigation-frame, dashboard-aligned, and
evaluation-aligned values in separate columns. It is intentionally conservative:
missing sources become empty columns instead of being inferred from another
coordinate product.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import re
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterable


SCHEMA: list[dict[str, str]] = [
    {"name": "run_id", "dtype": "string", "group": "index", "source": "run directory or manifest"},
    {"name": "flight_id", "dtype": "string", "group": "index", "source": "metadata/inferred"},
    {"name": "config_id", "dtype": "string", "group": "index", "source": "metadata/inferred"},
    {"name": "frame_id", "dtype": "int64", "group": "index", "source": "diag.csv or row number"},
    {"name": "timestamp_master", "dtype": "float64", "group": "time", "source": "camera/VIO output time"},
    {"name": "camera_timestamp_raw", "dtype": "float64", "group": "time", "source": "diag.csv t"},
    {"name": "vio_state_timestamp", "dtype": "float64", "group": "time", "source": "traj.txt t"},
    {"name": "fc_timestamp_raw", "dtype": "float64", "group": "time", "source": "FC source, optional"},
    {"name": "fc_timestamp_aligned", "dtype": "float64", "group": "time", "source": "FC source, optional"},
    {"name": "fc_interp_dt_s", "dtype": "float64", "group": "time", "source": "FC raw interpolation nearest endpoint delta"},
    {"name": "fc_interpolated", "dtype": "bool", "group": "time", "source": "FC raw interpolation status"},
    {"name": "fc_out_of_range", "dtype": "bool", "group": "time", "source": "FC raw interpolation validity"},
    {"name": "gps_timestamp_raw", "dtype": "float64", "group": "time", "source": "analysis/GPS source"},
    {"name": "gps_timestamp_aligned", "dtype": "float64", "group": "time", "source": "analysis/GPS source"},
    {"name": "vio_sample_delay_ms", "dtype": "float64", "group": "time", "source": "analysis data"},
    {"name": "segment_id", "dtype": "string", "group": "index", "source": "analysis segment index"},
    {"name": "init_status", "dtype": "string", "group": "index", "source": "diag.csv initialized"},
    {"name": "stride_input", "dtype": "int64", "group": "stride", "source": "run metadata or command"},
    {"name": "stride_visual_update", "dtype": "int64", "group": "stride", "source": "camera stride audit or command"},
    {"name": "stride_analysis_sample", "dtype": "int64", "group": "stride", "source": "analysis metadata"},
    {"name": "overlap_theory_along", "dtype": "float64", "group": "stride", "source": "future footprint model"},
    {"name": "overlap_polygon_area", "dtype": "float64", "group": "stride", "source": "future footprint model"},
    {"name": "overlap_valid", "dtype": "bool", "group": "stride", "source": "future footprint model"},
    {"name": "fc_q_x", "dtype": "float64", "group": "fc", "source": "FC raw converted to OpenVINS q_GtoI JPL, SLERP"},
    {"name": "fc_q_y", "dtype": "float64", "group": "fc", "source": "FC raw converted to OpenVINS q_GtoI JPL, SLERP"},
    {"name": "fc_q_z", "dtype": "float64", "group": "fc", "source": "FC raw converted to OpenVINS q_GtoI JPL, SLERP"},
    {"name": "fc_q_w", "dtype": "float64", "group": "fc", "source": "FC raw converted to OpenVINS q_GtoI JPL, SLERP"},
    {"name": "fc_roll_raw", "dtype": "float64", "group": "fc", "source": "FC source, optional"},
    {"name": "fc_pitch_raw", "dtype": "float64", "group": "fc", "source": "FC source, optional"},
    {"name": "fc_yaw_raw", "dtype": "float64", "group": "fc", "source": "FC source, optional"},
    {"name": "fc_roll_calibrated", "dtype": "float64", "group": "fc", "source": "future FC calibration"},
    {"name": "fc_pitch_calibrated", "dtype": "float64", "group": "fc", "source": "future FC calibration"},
    {"name": "fc_yaw_calibrated", "dtype": "float64", "group": "fc", "source": "future FC calibration"},
    {"name": "fc_velocity_e", "dtype": "float64", "group": "fc", "source": "FC source, optional"},
    {"name": "fc_velocity_n", "dtype": "float64", "group": "fc", "source": "FC source, optional"},
    {"name": "fc_velocity_u", "dtype": "float64", "group": "fc", "source": "FC source, optional"},
    {"name": "fc_position_e", "dtype": "float64", "group": "fc", "source": "FC raw WGS84 converted to local ENU"},
    {"name": "fc_position_n", "dtype": "float64", "group": "fc", "source": "FC raw WGS84 converted to local ENU"},
    {"name": "fc_position_u", "dtype": "float64", "group": "fc", "source": "FC raw WGS84 converted to local ENU"},
    {"name": "fc_source_path", "dtype": "string", "group": "fc", "source": "FC raw CSV path"},
    {"name": "vio_raw_q_x", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_q_y", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_q_z", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_q_w", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_position_x", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_position_y", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_position_z", "dtype": "float64", "group": "vio_raw", "source": "traj.txt"},
    {"name": "vio_raw_velocity_x", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias"},
    {"name": "vio_raw_velocity_y", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias"},
    {"name": "vio_raw_velocity_z", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias"},
    {"name": "vio_gyro_bias_x", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias or diag.csv"},
    {"name": "vio_gyro_bias_y", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias or diag.csv"},
    {"name": "vio_gyro_bias_z", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias or diag.csv"},
    {"name": "vio_accel_bias_x", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias or diag.csv"},
    {"name": "vio_accel_bias_y", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias or diag.csv"},
    {"name": "vio_accel_bias_z", "dtype": "float64", "group": "vio_raw", "source": "traj.txt.bias or diag.csv"},
    {"name": "vio_nav_q_x", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_q_y", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_q_z", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_q_w", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_position_e", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_position_n", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_position_u", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_velocity_e", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_velocity_n", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "vio_nav_velocity_u", "dtype": "float64", "group": "vio_nav", "source": "traj_nav.txt"},
    {"name": "nav_transform_source_path", "dtype": "string", "group": "vio_nav", "source": "nav_frame_metadata.json"},
    {"name": "nav_transform_version", "dtype": "string", "group": "vio_nav", "source": "nav_frame_metadata.json"},
    {"name": "nav_transform_future_data_used", "dtype": "bool", "group": "vio_nav", "source": "nav_frame_metadata.json"},
    {"name": "nav_transform_valid", "dtype": "bool", "group": "vio_nav", "source": "nav_frame_metadata.json"},
    {"name": "vio_dashboard_aligned_q_x", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_q_y", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_q_z", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_q_w", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_position_e", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_position_n", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_position_u", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_velocity_e", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_velocity_n", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_dashboard_aligned_velocity_u", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_id", "dtype": "string", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_method", "dtype": "string", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_rotation_yaw_deg", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_translation_e", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_translation_n", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_translation_u", "dtype": "float64", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_source_path", "dtype": "string", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_evaluation_only", "dtype": "bool", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_uses_future_data", "dtype": "bool", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "dashboard_alignment_applied", "dtype": "bool", "group": "vio_dashboard_aligned", "source": "dashboard alignment metadata"},
    {"name": "vio_eval_aligned_position_e", "dtype": "float64", "group": "vio_eval_aligned", "source": "analysis/data/gps_time_aligned_samples.csv"},
    {"name": "vio_eval_aligned_position_n", "dtype": "float64", "group": "vio_eval_aligned", "source": "analysis/data/gps_time_aligned_samples.csv"},
    {"name": "vio_eval_aligned_position_u", "dtype": "float64", "group": "vio_eval_aligned", "source": "analysis/data/gps_time_aligned_samples.csv"},
    {"name": "vio_eval_aligned_velocity_e", "dtype": "float64", "group": "vio_eval_aligned", "source": "analysis/data/gps_time_aligned_samples.csv"},
    {"name": "vio_eval_aligned_velocity_n", "dtype": "float64", "group": "vio_eval_aligned", "source": "analysis/data/gps_time_aligned_samples.csv"},
    {"name": "vio_eval_aligned_velocity_u", "dtype": "float64", "group": "vio_eval_aligned", "source": "analysis/data/gps_time_aligned_samples.csv"},
    {"name": "alignment_id", "dtype": "string", "group": "alignment", "source": "analysis metadata"},
    {"name": "alignment_mode", "dtype": "string", "group": "alignment", "source": "analysis metadata"},
    {"name": "alignment_rotation_yaw_deg", "dtype": "float64", "group": "alignment", "source": "analysis metadata"},
    {"name": "alignment_translation_e", "dtype": "float64", "group": "alignment", "source": "analysis metadata"},
    {"name": "alignment_translation_n", "dtype": "float64", "group": "alignment", "source": "analysis metadata"},
    {"name": "alignment_translation_u", "dtype": "float64", "group": "alignment", "source": "analysis metadata"},
    {"name": "alignment_applied", "dtype": "bool", "group": "alignment", "source": "analysis data present"},
    {"name": "truth_position_e", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "truth_position_n", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "truth_position_u", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "truth_velocity_e", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "truth_velocity_n", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "truth_velocity_u", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "truth_course_deg", "dtype": "float64", "group": "truth", "source": "analysis GPS/reference"},
    {"name": "gps_quality", "dtype": "string", "group": "truth", "source": "GPS source, optional"},
    {"name": "gps_velocity_source", "dtype": "string", "group": "truth", "source": "analysis data"},
    {"name": "err_e", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_n", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_u", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_xy", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_3d", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_along", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_cross", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_vertical", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_v_e", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_v_n", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_v_u", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_v_along", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_v_cross", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "err_v_vertical", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "course_error_deg", "dtype": "float64", "group": "error", "source": "analysis data"},
    {"name": "feature_klt_raw", "dtype": "int64", "group": "diagnostic", "source": "diag.csv"},
    {"name": "feature_tracked", "dtype": "int64", "group": "diagnostic", "source": "diag.csv"},
    {"name": "feature_msckf_accepted", "dtype": "int64", "group": "diagnostic", "source": "diag.csv"},
    {"name": "feature_slam_count", "dtype": "int64", "group": "diagnostic", "source": "diag.csv"},
    {"name": "mean_track_length", "dtype": "float64", "group": "diagnostic", "source": "future tracker summary"},
    {"name": "visual_update_executed", "dtype": "bool", "group": "diagnostic", "source": "future per-frame update log"},
    {"name": "data_missing_flags", "dtype": "string", "group": "diagnostic", "source": "builder"},
    {"name": "repo_commit", "dtype": "string", "group": "provenance", "source": "git"},
    {"name": "dirty_worktree", "dtype": "bool", "group": "provenance", "source": "git"},
    {"name": "runner_path", "dtype": "string", "group": "provenance", "source": "run metadata"},
    {"name": "config_path", "dtype": "string", "group": "provenance", "source": "run metadata"},
    {"name": "config_hash", "dtype": "string", "group": "provenance", "source": "future hash"},
    {"name": "calibration_id", "dtype": "string", "group": "provenance", "source": "config snapshot"},
    {"name": "command_path", "dtype": "string", "group": "provenance", "source": "run directory"},
    {"name": "analysis_source", "dtype": "string", "group": "provenance", "source": "analysis directory"},
]


def schema_names() -> list[str]:
    return [c["name"] for c in SCHEMA]


def write_csv_rows(path: Path, rows: Iterable[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})


def read_metadata(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    if not path.exists():
        return meta
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            key, value = line.split("=", 1)
            meta[key.strip()] = value.strip()
    return meta


def safe_git(args: list[str]) -> str:
    try:
        return subprocess.check_output(["git", *args], text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""


def infer_flight_id(text: str) -> str:
    m = re.search(r"(fly[1-4])", text, re.IGNORECASE)
    return m.group(1).lower() if m else ""


def infer_stride(text: str, meta: dict[str, str] | None = None) -> str:
    if meta and meta.get("stride"):
        return meta["stride"]
    m = re.search(r"stride(\d+)", text, re.IGNORECASE)
    return m.group(1) if m else ""


def parse_command_flags(command_path: Path) -> dict[str, str]:
    if not command_path.exists():
        return {}
    body = " ".join(
        line.strip()
        for line in command_path.read_text(encoding="utf-8", errors="replace").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )
    if not body:
        return {}
    try:
        tokens = shlex.split(body, posix=True)
    except ValueError:
        tokens = body.split()
    out: dict[str, str] = {}
    for i, token in enumerate(tokens):
        if token.startswith("--"):
            if i + 1 < len(tokens) and not tokens[i + 1].startswith("--"):
                out[token[2:]] = tokens[i + 1]
            else:
                out[token[2:]] = "true"
    return out


def first_fc_init_timestamp(path_text: str) -> float | None:
    if not path_text:
        return None
    path = Path(wsl_to_windows_path(path_text))
    if not path.exists():
        return None
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                first = row[0].strip()
                if not first or first.startswith("#") or first.lower() in {"t", "t_s", "time", "timestamp"}:
                    continue
                try:
                    return float(first)
                except ValueError:
                    continue
    except OSError:
        return None
    return None


def fc_init_time_audit(fc_init: str, start_time: str | float | int | None, threshold_s: float = 0.25) -> dict[str, str]:
    timestamp = first_fc_init_timestamp(str(fc_init or ""))
    out = {
        "fc_init_timestamp": "" if timestamp is None else f"{timestamp:.6f}",
        "fc_init_dt_start_s": "",
        "fc_init_time_status": "missing" if not fc_init else "unreadable",
    }
    if timestamp is None:
        return out
    try:
        start = float(start_time) if start_time not in (None, "") else math.nan
    except (TypeError, ValueError):
        start = math.nan
    if not math.isfinite(start):
        out["fc_init_time_status"] = "no_start_time"
        return out
    dt_s = timestamp - start
    out["fc_init_dt_start_s"] = f"{dt_s:.6f}"
    out["fc_init_time_status"] = "ok" if abs(dt_s) <= threshold_s else "mismatch"
    return out


def scan_result_run_dirs(root: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    if not root.exists():
        return rows
    for run_dir in sorted(p for p in root.rglob("*") if p.is_dir()):
        files = {child.name for child in run_dir.iterdir() if child.is_file()}
        if not ({"run_metadata.txt", "command.sh", "traj.txt", "diag.csv"} & files):
            continue
        meta = read_metadata(run_dir / "run_metadata.txt")
        flags = parse_command_flags(run_dir / "command.sh")
        text = str(run_dir)
        stride = infer_stride(text, meta) or flags.get("camera-frame-stride", "")
        flight_id = meta.get("fly") or infer_flight_id(text)
        fc_init = meta.get("fc_init", flags.get("init-from-fc", ""))
        start_time = meta.get("start_time", flags.get("start-time", ""))
        row = {
            "source": "local_result",
            "run_id": run_dir.name,
            "flight_id": flight_id,
            "stride_input": stride,
            "stride_visual_update": flags.get("visual-update-stride", ""),
            "yaw_mode": meta.get("yaw_mode", flags.get("yaw-mode", "")),
            "height_mode": meta.get("height_mode", flags.get("height-mode", "")),
            "dataset": meta.get("dataset", flags.get("dataset", "")),
            "gps": meta.get("gps", flags.get("gps", "")),
            "fc_init": fc_init,
            "start_time": start_time,
            "config": meta.get("config", flags.get("config", "")),
            "runner": meta.get("runner", ""),
            "run_dir": str(run_dir),
            "has_traj": str((run_dir / "traj.txt").exists()).lower(),
            "has_bias": str((run_dir / "traj.txt.bias").exists()).lower(),
            "has_diag": str((run_dir / "diag.csv").exists()).lower(),
            "has_yaw_diag": str((run_dir / "yaw_diag.csv").exists()).lower(),
            "has_dashboard_alignment": str((run_dir / "dashboard_alignment.json").exists()).lower(),
            "has_master_parquet": str((run_dir / "master_data" / "MASTER_FLIGHT_DATA.parquet").exists()).lower(),
            "has_master_csv": str((run_dir / "master_data" / "MASTER_FLIGHT_DATA.csv").exists()).lower(),
            "has_adaptive_stride": str((run_dir / "traj.txt.adaptive_stride.csv").exists()).lower(),
            "has_analysis_samples": str((run_dir / "analysis" / "data" / "gps_time_aligned_samples.csv").exists()).lower(),
            "status": "observed",
        }
        row.update(fc_init_time_audit(fc_init, start_time))
        rows.append(row)
    return rows


def manifest_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    strides = data.get("common_stride_sweep") or []
    rows: list[dict[str, str]] = []
    for flight_id, flight in sorted((data.get("flights") or {}).items()):
        for stride in strides:
            role = "sweep"
            if stride == data.get("default_stride"):
                role = "default"
            elif stride == data.get("control_stride"):
                role = "control"
            fc_init = flight.get("fc_init", "")
            start_time = str(flight.get("start_time", ""))
            rows.append({
                "source": "baseline_manifest",
                "run_id": f"{flight_id}_stride{stride}_manifest",
                "flight_id": flight_id,
                "stride_input": str(stride),
                "stride_visual_update": "",
                "yaw_mode": (data.get("runtime_flags") or {}).get("yaw_mode", ""),
                "height_mode": (data.get("runtime_flags") or {}).get("height_mode", ""),
                "dataset": flight.get("dataset", ""),
                "gps": flight.get("gps", ""),
                "fc_init": fc_init,
                "start_time": start_time,
                "config": data.get("config", ""),
                "runner": data.get("runner", ""),
                "run_dir": "",
                "has_traj": "false",
                "has_bias": "false",
                "has_diag": "false",
                "has_yaw_diag": "false",
                "has_dashboard_alignment": "false",
                "has_master_parquet": "false",
                "has_master_csv": "false",
                "has_adaptive_stride": "false",
                "has_analysis_samples": "false",
                "status": f"manifest_{role}",
                **fc_init_time_audit(fc_init, start_time),
            })
    return rows


def cmd_schema(args: argparse.Namespace) -> int:
    path = Path(args.out) if args.out else None
    if path:
        if path.suffix.lower() == ".json":
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(SCHEMA, indent=2), encoding="utf-8")
        else:
            write_csv_rows(path, SCHEMA, ["name", "dtype", "group", "source"])
    else:
        for col in SCHEMA:
            print(f"{col['name']},{col['dtype']},{col['group']},{col['source']}")
    return 0


def cmd_index(args: argparse.Namespace) -> int:
    rows: list[dict[str, str]] = []
    if args.baseline_manifest:
        rows.extend(manifest_rows(Path(args.baseline_manifest)))
    for root in args.result_root:
        rows.extend(scan_result_run_dirs(Path(root)))
    rows.sort(key=lambda r: (r.get("flight_id", ""), int(r.get("stride_input") or 0), r.get("source", ""), r.get("run_id", "")))
    fields = [
        "source", "run_id", "flight_id", "stride_input", "stride_visual_update",
        "yaw_mode", "height_mode", "dataset", "gps", "fc_init", "start_time",
        "fc_init_timestamp", "fc_init_dt_start_s", "fc_init_time_status",
        "config", "runner", "run_dir", "has_traj", "has_bias", "has_diag",
        "has_yaw_diag", "has_dashboard_alignment", "has_master_parquet", "has_master_csv",
        "has_adaptive_stride", "has_analysis_samples", "status",
    ]
    write_csv_rows(Path(args.out), rows, fields)
    print(f"wrote {len(rows)} rows to {args.out}")
    return 0


def require_pandas():
    try:
        import pandas as pd  # type: ignore
    except Exception as exc:
        raise SystemExit(f"pandas is required for build: {exc}") from exc
    return pd


def require_numpy():
    try:
        import numpy as np  # type: ignore
    except Exception as exc:
        raise SystemExit(f"numpy is required for FC interpolation: {exc}") from exc
    return np


def wsl_to_windows_path(value: str) -> str:
    m = re.match(r"^/mnt/([A-Za-z])/(.*)$", value.strip()) if value else None
    if not m:
        return value
    rest = m.group(2).replace("/", "\\")
    return f"{m.group(1).upper()}:\\{rest}"


def parse_fc_offset_from_init(path_text: str) -> float | None:
    path = Path(wsl_to_windows_path(path_text))
    if path.exists():
        text = path.read_text(encoding="utf-8", errors="replace")
        m = re.search(r"fc_rel_to_cam_offset\s+([-+0-9.]+)", text)
        if m:
            return float(m.group(1))
    m = re.search(r"offset(m?)(\d+)p(\d+)", path_text)
    if m:
        value = float(f"{m.group(2)}.{m.group(3)}")
        return -value if m.group(1) == "m" else value
    return None


def parse_fc_time_utc(text: str) -> float:
    text = text.strip()
    if not text:
        raise ValueError("empty FC GPS time")
    if "_" in text:
        base, ms = text.rsplit("_", 1)
        micros = int(ms.ljust(3, "0")[:3]) * 1000
    else:
        base = text
        micros = 0
    stamp = dt.datetime.strptime(base, "%Y-%m-%d %H:%M:%S")
    stamp = stamp.replace(microsecond=micros, tzinfo=dt.timezone.utc)
    return stamp.timestamp()


def find_fc_raw_csv(meta: dict[str, str]) -> Path | None:
    dataset = Path(wsl_to_windows_path(meta.get("dataset", "")))
    roots = [p for p in [dataset, dataset.parent] if p.exists()]
    candidates: list[Path] = []
    for root in roots:
        for path in root.rglob("*.csv"):
            try:
                head = path.read_bytes()[:300].decode("utf-8-sig", errors="replace")
            except Exception:
                continue
            if "GPS" in head and "Ve" in head and "Vn" in head and "Vu" in head and path.stat().st_size > 100000:
                candidates.append(path)
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_size)


def fc_rotations(roll_deg: float, pitch_deg: float, yaw_deg: float):
    np = require_numpy()

    def rx(angle: float):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)

    def ry(angle: float):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)

    def rz(angle: float):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)

    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(-yaw_deg)
    r_b_to_ned = rz(y) @ ry(p) @ rx(r)
    r_enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    r_g_to_frd = r_b_to_ned.T @ r_enu_to_ned
    r_i_from_frd = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    return r_i_from_frd @ r_g_to_frd


def rot_to_jpl_quat(rot):
    np = require_numpy()
    q = np.zeros(4)
    trace = float(np.trace(rot))
    if rot[0, 0] >= trace and rot[0, 0] >= rot[1, 1] and rot[0, 0] >= rot[2, 2]:
        q[0] = math.sqrt((1.0 + 2.0 * rot[0, 0] - trace) / 4.0)
        q[1] = (rot[0, 1] + rot[1, 0]) / (4.0 * q[0])
        q[2] = (rot[0, 2] + rot[2, 0]) / (4.0 * q[0])
        q[3] = (rot[1, 2] - rot[2, 1]) / (4.0 * q[0])
    elif rot[1, 1] >= trace and rot[1, 1] >= rot[0, 0] and rot[1, 1] >= rot[2, 2]:
        q[1] = math.sqrt((1.0 + 2.0 * rot[1, 1] - trace) / 4.0)
        q[0] = (rot[0, 1] + rot[1, 0]) / (4.0 * q[1])
        q[2] = (rot[1, 2] + rot[2, 1]) / (4.0 * q[1])
        q[3] = (rot[2, 0] - rot[0, 2]) / (4.0 * q[1])
    elif rot[2, 2] >= trace and rot[2, 2] >= rot[0, 0] and rot[2, 2] >= rot[1, 1]:
        q[2] = math.sqrt((1.0 + 2.0 * rot[2, 2] - trace) / 4.0)
        q[0] = (rot[0, 2] + rot[2, 0]) / (4.0 * q[2])
        q[1] = (rot[1, 2] + rot[2, 1]) / (4.0 * q[2])
        q[3] = (rot[0, 1] - rot[1, 0]) / (4.0 * q[2])
    else:
        q[3] = math.sqrt((1.0 + trace) / 4.0)
        q[0] = (rot[1, 2] - rot[2, 1]) / (4.0 * q[3])
        q[1] = (rot[2, 0] - rot[0, 2]) / (4.0 * q[3])
        q[2] = (rot[0, 1] - rot[1, 0]) / (4.0 * q[3])
    if q[3] < 0:
        q = -q
    return q / np.linalg.norm(q)


def slerp_quat(q0, q1, alpha: float):
    np = require_numpy()
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1.0 - alpha) * theta) / math.sin(theta)) * q0 + (math.sin(alpha * theta) / math.sin(theta)) * q1


def hamilton_quat_to_rot(q):
    np = require_numpy()
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def rot_to_hamilton_quat(rot):
    np = require_numpy()
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    if q[3] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


def lla_to_enu(lat, lon, alt, lat0: float, lon0: float, alt0: float):
    np = require_numpy()
    radius = 6378137.0
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)
    east = (lon_rad - lon0_rad) * math.cos(lat0_rad) * radius
    north = (lat_rad - lat0_rad) * radius
    up = alt - alt0
    return east, north, up


def load_fc_raw(path: Path, offset_s: float):
    pd = require_pandas()
    rows: list[dict[str, Any]] = []
    fc0_unix: float | None = None
    with path.open(newline="", encoding="utf-8-sig", errors="replace") as f:
        reader = csv.reader(f)
        next(reader, None)
        for raw in reader:
            if len(raw) < 11:
                continue
            try:
                unix = parse_fc_time_utc(raw[3])
                if fc0_unix is None:
                    fc0_unix = unix
                rows.append({
                    "t": (unix - fc0_unix) - offset_s,
                    "raw_unix": unix,
                    "pitch": float(raw[0]),
                    "roll": float(raw[1]),
                    "yaw": float(raw[2]),
                    "satellites": int(float(raw[4])),
                    "lat": float(raw[5]),
                    "lon": float(raw[6]),
                    "alt": float(raw[7]),
                    "ve": float(raw[8]),
                    "vn": float(raw[9]),
                    "vu": float(raw[10]),
                })
            except Exception:
                continue
    if not rows:
        return pd.DataFrame()
    raw = pd.DataFrame(rows)
    duplicate_count = int(raw["t"].duplicated().sum())
    non_monotonic_count = int((raw["t"].diff().dropna() < 0).sum())
    fc = raw.drop_duplicates("t").sort_values("t").reset_index(drop=True)
    fc.attrs["duplicate_timestamp_count"] = duplicate_count
    fc.attrs["non_monotonic_timestamp_count"] = non_monotonic_count
    lat0, lon0, alt0 = float(fc["lat"].iloc[0]), float(fc["lon"].iloc[0]), float(fc["alt"].iloc[0])
    fc["e"], fc["n"], fc["u"] = lla_to_enu(fc["lat"], fc["lon"], fc["alt"], lat0, lon0, alt0)
    quats = [rot_to_jpl_quat(fc_rotations(r.roll, r.pitch, r.yaw)) for r in fc.itertuples()]
    fc["q_x"] = [float(q[0]) for q in quats]
    fc["q_y"] = [float(q[1]) for q in quats]
    fc["q_z"] = [float(q[2]) for q in quats]
    fc["q_w"] = [float(q[3]) for q in quats]
    return fc


def interpolate_fc_to_times(fc, times, tolerance_s: float):
    pd = require_pandas()
    np = require_numpy()
    if fc is None or fc.empty:
        return pd.DataFrame(index=range(len(times)))
    src_t = fc["t"].to_numpy(dtype=float)
    out: list[dict[str, Any]] = []
    numeric_cols = ["roll", "pitch", "yaw", "ve", "vn", "vu", "e", "n", "u", "satellites", "raw_unix"]
    for t in times:
        t = float(t)
        if t < src_t[0] or t > src_t[-1]:
            nearest_dt = min(abs(t - src_t[0]), abs(t - src_t[-1]))
            out.append({"fc_interp_dt_s": nearest_dt, "fc_interpolated": False, "fc_out_of_range": True})
            continue
        idx = int(np.searchsorted(src_t, t, side="left"))
        if idx == 0:
            a = b = fc.iloc[0]
            alpha = 0.0
        elif idx >= len(fc):
            a = b = fc.iloc[-1]
            alpha = 0.0
        else:
            a = fc.iloc[idx - 1]
            b = fc.iloc[idx]
            alpha = (t - float(a["t"])) / (float(b["t"]) - float(a["t"]))
        endpoint_dt = min(abs(t - float(a["t"])), abs(t - float(b["t"])))
        if endpoint_dt > tolerance_s:
            out.append({"fc_interp_dt_s": endpoint_dt, "fc_interpolated": False, "fc_out_of_range": False})
            continue
        row: dict[str, Any] = {
            "fc_timestamp_aligned": t,
            "fc_timestamp_raw": float(a["raw_unix"] + alpha * (b["raw_unix"] - a["raw_unix"])),
            "fc_interp_dt_s": endpoint_dt,
            "fc_interpolated": True,
            "fc_out_of_range": False,
        }
        for col in numeric_cols:
            if col in {"yaw"}:
                delta = (float(b[col]) - float(a[col]) + 180.0) % 360.0 - 180.0
                row[col] = float(a[col]) + alpha * delta
            else:
                row[col] = float(a[col]) + alpha * (float(b[col]) - float(a[col]))
        q = slerp_quat(
            np.array([a["q_x"], a["q_y"], a["q_z"], a["q_w"]], dtype=float),
            np.array([b["q_x"], b["q_y"], b["q_z"], b["q_w"]], dtype=float),
            float(alpha),
        )
        row.update({"fc_q_x": q[0], "fc_q_y": q[1], "fc_q_z": q[2], "fc_q_w": q[3]})
        out.append(row)
    return pd.DataFrame(out)


def attach_fc_raw(master, args: argparse.Namespace, meta: dict[str, str]) -> dict[str, str]:
    source: dict[str, str] = {"fc_raw": ""}
    fc_raw = Path(args.fc_raw) if args.fc_raw else find_fc_raw_csv(meta)
    if fc_raw is None or not fc_raw.exists():
        return source
    offset = args.fc_offset_s
    if offset is None:
        offset = parse_fc_offset_from_init(meta.get("fc_init", ""))
    if offset is None:
        return source
    fc = load_fc_raw(fc_raw, float(offset))
    if fc.empty:
        return source
    interp = interpolate_fc_to_times(fc, master["timestamp_master"].to_numpy(dtype=float), args.fc_tolerance_s)
    mapping = {
        "fc_q_x": "fc_q_x",
        "fc_q_y": "fc_q_y",
        "fc_q_z": "fc_q_z",
        "fc_q_w": "fc_q_w",
        "fc_roll_raw": "roll",
        "fc_pitch_raw": "pitch",
        "fc_yaw_raw": "yaw",
        "fc_velocity_e": "ve",
        "fc_velocity_n": "vn",
        "fc_velocity_u": "vu",
        "fc_position_e": "e",
        "fc_position_n": "n",
        "fc_position_u": "u",
        "gps_quality": "satellites",
        "fc_timestamp_raw": "fc_timestamp_raw",
        "fc_timestamp_aligned": "fc_timestamp_aligned",
        "fc_interp_dt_s": "fc_interp_dt_s",
        "fc_interpolated": "fc_interpolated",
        "fc_out_of_range": "fc_out_of_range",
    }
    for dst, src in mapping.items():
        if src in interp.columns:
            master[dst] = interp[src]
    master["fc_source_path"] = str(fc_raw)
    source.update({
        "fc_raw": str(fc_raw),
        "fc_offset_s": f"{float(offset):.6f}",
        "fc_rows": str(len(fc)),
        "fc_time_range": f"{fc['t'].min():.6f}..{fc['t'].max():.6f}",
        "fc_duplicate_timestamps": str(fc.attrs.get("duplicate_timestamp_count", 0)),
        "fc_non_monotonic_steps": str(fc.attrs.get("non_monotonic_timestamp_count", 0)),
    })
    return source


def apply_dashboard_alignment(master, metadata_path: Path) -> dict[str, str]:
    np = require_numpy()
    source: dict[str, str] = {"dashboard_alignment": ""}
    if not metadata_path.exists():
        return source
    try:
        meta = json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"warning: failed to read dashboard alignment metadata {metadata_path}: {exc}", file=sys.stderr)
        return source
    rotation = np.asarray(meta.get("rotation_matrix"), dtype=float)
    translation = np.asarray(meta.get("translation"), dtype=float)
    if rotation.shape != (3, 3) or translation.shape != (3,):
        print(f"warning: dashboard alignment metadata has invalid transform: {metadata_path}", file=sys.stderr)
        return source
    pos_cols = ["vio_raw_position_x", "vio_raw_position_y", "vio_raw_position_z"]
    valid_pos = master[pos_cols].notna().all(axis=1)
    if valid_pos.any():
        raw_pos = master.loc[valid_pos, pos_cols].to_numpy(dtype=float)
        aligned = raw_pos @ rotation.T + translation
        master.loc[valid_pos, "vio_dashboard_aligned_position_e"] = aligned[:, 0]
        master.loc[valid_pos, "vio_dashboard_aligned_position_n"] = aligned[:, 1]
        master.loc[valid_pos, "vio_dashboard_aligned_position_u"] = aligned[:, 2]
    vel_cols = ["vio_raw_velocity_x", "vio_raw_velocity_y", "vio_raw_velocity_z"]
    valid_vel = master[vel_cols].notna().all(axis=1)
    if valid_vel.any():
        raw_vel = master.loc[valid_vel, vel_cols].to_numpy(dtype=float)
        aligned_vel = raw_vel @ rotation.T
        master.loc[valid_vel, "vio_dashboard_aligned_velocity_e"] = aligned_vel[:, 0]
        master.loc[valid_vel, "vio_dashboard_aligned_velocity_n"] = aligned_vel[:, 1]
        master.loc[valid_vel, "vio_dashboard_aligned_velocity_u"] = aligned_vel[:, 2]
    quat_cols = ["vio_raw_q_x", "vio_raw_q_y", "vio_raw_q_z", "vio_raw_q_w"]
    valid_quat = master[quat_cols].notna().all(axis=1)
    if valid_quat.any():
        raw_q = master.loc[valid_quat, quat_cols].to_numpy(dtype=float)
        aligned_q = []
        bad_q = 0
        for q in raw_q:
            norm = float(np.linalg.norm(q))
            if norm <= 1e-12 or not np.isfinite(norm):
                aligned_q.append([np.nan, np.nan, np.nan, np.nan])
                bad_q += 1
                continue
            aligned_q.append(rot_to_hamilton_quat(rotation @ hamilton_quat_to_rot(q)))
        aligned_q_arr = np.asarray(aligned_q, dtype=float)
        idx = master.index[valid_quat]
        master.loc[idx, "vio_dashboard_aligned_q_x"] = aligned_q_arr[:, 0]
        master.loc[idx, "vio_dashboard_aligned_q_y"] = aligned_q_arr[:, 1]
        master.loc[idx, "vio_dashboard_aligned_q_z"] = aligned_q_arr[:, 2]
        master.loc[idx, "vio_dashboard_aligned_q_w"] = aligned_q_arr[:, 3]
        if bad_q:
            print(f"warning: dashboard alignment skipped {bad_q} invalid raw quaternions", file=sys.stderr)
    master["dashboard_alignment_id"] = "dashboard_xy_yaw_translation"
    master["dashboard_alignment_method"] = meta.get("alignment_method", "dashboard_xy_yaw_translation")
    master["dashboard_alignment_rotation_yaw_deg"] = meta.get("yaw_deg")
    master["dashboard_alignment_translation_e"] = float(translation[0])
    master["dashboard_alignment_translation_n"] = float(translation[1])
    master["dashboard_alignment_translation_u"] = float(translation[2])
    master["dashboard_alignment_source_path"] = str(metadata_path)
    master["dashboard_alignment_evaluation_only"] = True
    master["dashboard_alignment_uses_future_data"] = True
    master["dashboard_alignment_applied"] = True
    source["dashboard_alignment"] = str(metadata_path)
    return source


def read_traj(path: Path):
    pd = require_pandas()
    if not path.exists():
        return None
    return pd.read_csv(
        path,
        sep=r"\s+",
        comment="#",
        names=["t", "x", "y", "z", "qx", "qy", "qz", "qw"],
        engine="python",
    ).dropna(how="all")


def read_traj_nav(path: Path):
    pd = require_pandas()
    if not path.exists():
        return None
    return pd.read_csv(
        path,
        sep=r"\s+",
        comment="#",
        names=["t", "x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw"],
        engine="python",
    ).dropna(how="all")


def read_bias(path: Path):
    pd = require_pandas()
    if not path.exists():
        return None
    return pd.read_csv(
        path,
        sep=r"\s+",
        comment="#",
        names=["t", "vx", "vy", "vz", "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"],
        engine="python",
    ).dropna(how="all")


def merge_nearest(left, right, left_on: str, right_on: str, tolerance: float):
    pd = require_pandas()
    if right is None or right.empty:
        return left
    return pd.merge_asof(
        left.sort_values(left_on),
        right.sort_values(right_on),
        left_on=left_on,
        right_on=right_on,
        direction="nearest",
        tolerance=tolerance,
    )


def fill_if_present(df, source, mapping: dict[str, str]) -> None:
    if source is None:
        return
    for dst, src in mapping.items():
        if src in source.columns:
            df[dst] = source[src]


def print_build_summary(master, sources: dict[str, str]) -> None:
    print(f"rows={len(master)} cols={len(master.columns)} "
          f"time={master['timestamp_master'].min():.6f}..{master['timestamp_master'].max():.6f}")
    for key, value in sources.items():
        if value:
            print(f"source.{key}={value}")
    groups = sorted({c["group"] for c in SCHEMA})
    for group in groups:
        cols = [c["name"] for c in SCHEMA if c["group"] == group and c["name"] in master.columns]
        if not cols:
            continue
        nonempty_cols = [c for c in cols if master[c].notna().any()]
        row_valid = master[nonempty_cols].notna().any(axis=1).sum() if nonempty_cols else 0
        print(f"coverage.{group}: cols={len(nonempty_cols)}/{len(cols)} rows_with_any={int(row_valid)}/{len(master)}")
    warnings: list[str] = []
    if not master.filter(regex=r"^fc_").notna().any().any():
        warnings.append("FC raw columns are empty; FC/VIO attitude comparison cannot be audited from this table.")
    elif "fc_interpolated" in master.columns:
        valid = master["fc_interpolated"].fillna(False).astype(bool)
        invalid = ~valid
        if valid.any():
            max_dt = master.loc[valid, "fc_interp_dt_s"].max()
            print(f"fc.interpolated_rows={int(valid.sum())}/{len(master)}")
            print(f"fc.max_interpolation_age_s={max_dt:.6f}")
            if "fc_out_of_range" in master.columns:
                out_of_range = master["fc_out_of_range"].fillna(False).astype(bool)
                print(f"fc.out_of_range_rows={int(out_of_range.sum())}/{len(master)}")
            q_cols = ["fc_q_x", "fc_q_y", "fc_q_z", "fc_q_w"]
            if all(c in master.columns for c in q_cols):
                q = master.loc[valid, q_cols].dropna()
                if len(q):
                    norms = (q.pow(2).sum(axis=1) ** 0.5)
                    non_unit = int(((norms - 1.0).abs() > 1e-6).sum())
                    print(f"fc.non_unit_quaternion_rows={non_unit}/{len(q)}")
            warnings.append(f"FC interpolated rows={int(valid.sum())}/{len(master)}, max endpoint dt={max_dt:.6f}s.")
        else:
            warnings.append("FC raw was found, but no master rows passed the FC interpolation tolerance.")
        print(f"fc.invalid_or_missing_rows={int(invalid.sum())}/{len(master)}")
    nav_cols = [c for c in master.columns if c.startswith("vio_nav_")]
    if nav_cols and not master[nav_cols].notna().any().any():
        warnings.append("vio_nav_* trajectory columns are empty; no formal traj_nav.txt was found or matched.")
    dash_cols = [c for c in master.columns if c.startswith("vio_dashboard_aligned_")]
    if dash_cols and master[dash_cols].notna().any().any():
        warnings.append("vio_dashboard_aligned_* is evaluation/display-only and uses future trajectory fitting.")
    if warnings:
        for item in warnings:
            print(f"warning: {item}", file=sys.stderr)


def cmd_build(args: argparse.Namespace) -> int:
    pd = require_pandas()
    run_dir = Path(args.run_dir)
    meta = read_metadata(run_dir / "run_metadata.txt")
    flags = parse_command_flags(run_dir / "command.sh")
    traj_path = run_dir / "traj.txt"
    if not traj_path.exists() and (run_dir / "traj_raw.txt").exists():
        traj_path = run_dir / "traj_raw.txt"
    traj = read_traj(traj_path)
    if traj is None or traj.empty:
        raise SystemExit(f"missing or empty traj.txt/traj_raw.txt in {run_dir}")
    nav_traj_path = run_dir / "traj_nav.txt"
    nav_traj = read_traj_nav(nav_traj_path)
    nav_meta_path = run_dir / "nav_frame_metadata.json"
    nav_meta: dict[str, Any] = {}
    if nav_meta_path.exists():
        try:
            nav_meta = json.loads(nav_meta_path.read_text(encoding="utf-8"))
        except Exception as exc:
            print(f"warning: failed to read nav metadata {nav_meta_path}: {exc}", file=sys.stderr)
    bias = read_bias(run_dir / "traj.txt.bias")
    diag_path = run_dir / "diag.csv"
    diag = pd.read_csv(diag_path) if diag_path.exists() else None

    if diag is not None and "t" in diag.columns:
        master = pd.DataFrame({"timestamp_master": diag["t"].astype(float)})
        master["camera_timestamp_raw"] = master["timestamp_master"]
        if "frame_id" in diag.columns:
            master["frame_id"] = diag["frame_id"]
        else:
            master["frame_id"] = range(1, len(master) + 1)
    else:
        master = pd.DataFrame({"timestamp_master": traj["t"].astype(float)})
        master["camera_timestamp_raw"] = master["timestamp_master"]
        master["frame_id"] = range(1, len(master) + 1)

    master["run_id"] = args.run_id or run_dir.name
    master["flight_id"] = args.flight_id or meta.get("fly") or infer_flight_id(str(run_dir))
    master["config_id"] = args.config_id or Path(meta.get("config", flags.get("config", ""))).name
    master["stride_input"] = infer_stride(str(run_dir), meta) or flags.get("camera-frame-stride", "")
    master["stride_visual_update"] = flags.get("visual-update-stride", "")
    master["runner_path"] = meta.get("runner", "")
    master["config_path"] = meta.get("config", flags.get("config", ""))
    master["command_path"] = str(run_dir / "command.sh") if (run_dir / "command.sh").exists() else ""
    master["repo_commit"] = safe_git(["rev-parse", "HEAD"])
    master["dirty_worktree"] = bool(safe_git(["status", "--short"]))
    master["nav_transform_valid"] = False
    master["dashboard_alignment_applied"] = False
    master["alignment_applied"] = False
    master["data_missing_flags"] = ""
    sources: dict[str, str] = {
        "traj": str(traj_path),
        "traj_nav": str(nav_traj_path) if nav_traj_path.exists() else "",
        "nav_metadata": str(nav_meta_path) if nav_meta_path.exists() else "",
        "bias": str(run_dir / "traj.txt.bias") if (run_dir / "traj.txt.bias").exists() else "",
        "diag": str(diag_path) if diag_path.exists() else "",
        "analysis": "",
    }

    traj_small = traj.rename(columns={
        "t": "vio_state_timestamp",
        "x": "vio_raw_position_x",
        "y": "vio_raw_position_y",
        "z": "vio_raw_position_z",
        "qx": "vio_raw_q_x",
        "qy": "vio_raw_q_y",
        "qz": "vio_raw_q_z",
        "qw": "vio_raw_q_w",
    })
    master = merge_nearest(master, traj_small, "timestamp_master", "vio_state_timestamp", args.traj_tolerance_s)

    if nav_traj is not None and not nav_traj.empty:
        nav_small = nav_traj.rename(columns={
            "t": "vio_nav_timestamp",
            "x": "vio_nav_position_e",
            "y": "vio_nav_position_n",
            "z": "vio_nav_position_u",
            "vx": "vio_nav_velocity_e",
            "vy": "vio_nav_velocity_n",
            "vz": "vio_nav_velocity_u",
            "qx": "vio_nav_q_x",
            "qy": "vio_nav_q_y",
            "qz": "vio_nav_q_z",
            "qw": "vio_nav_q_w",
        })
        master = merge_nearest(master, nav_small, "timestamp_master", "vio_nav_timestamp", args.traj_tolerance_s)
        master["nav_transform_valid"] = True
    if nav_meta:
        master["nav_transform_source_path"] = str(nav_meta_path)
        master["nav_transform_version"] = str(nav_meta.get("fc_imu_extrinsic_version", "")) + "|" + str(nav_meta.get("gps_imu_lever_arm_version", ""))
        master["nav_transform_future_data_used"] = bool(nav_meta.get("whether_future_data_used", False))

    if bias is not None:
        bias_small = bias.rename(columns={
            "t": "bias_timestamp",
            "vx": "vio_raw_velocity_x",
            "vy": "vio_raw_velocity_y",
            "vz": "vio_raw_velocity_z",
            "bg_x": "vio_gyro_bias_x",
            "bg_y": "vio_gyro_bias_y",
            "bg_z": "vio_gyro_bias_z",
            "ba_x": "vio_accel_bias_x",
            "ba_y": "vio_accel_bias_y",
            "ba_z": "vio_accel_bias_z",
        })
        master = merge_nearest(master, bias_small, "timestamp_master", "bias_timestamp", args.traj_tolerance_s)

    fc_sources = attach_fc_raw(master, args, meta)
    sources.update(fc_sources)

    dashboard_alignment_path = Path(args.dashboard_alignment_json) if args.dashboard_alignment_json else (run_dir / "dashboard_alignment.json")
    dashboard_sources = apply_dashboard_alignment(master, dashboard_alignment_path)
    sources.update(dashboard_sources)

    if diag is not None:
        diag_small = diag.copy()
        diag_small["diag_timestamp"] = diag_small["t"]
        diag_map = {
            "init_status": "initialized",
            "feature_klt_raw": "klt_raw",
            "feature_tracked": "tracked",
            "feature_msckf_accepted": "n_acc",
            "feature_slam_count": "slam_count",
        }
        merged = merge_nearest(master[["timestamp_master"]], diag_small, "timestamp_master", "diag_timestamp", args.traj_tolerance_s)
        fill_if_present(master, merged, diag_map)

    analysis_samples = run_dir / "analysis" / "data" / "gps_time_aligned_samples.csv"
    if not analysis_samples.exists():
        analysis_samples = run_dir / "analysis_extended" / "data" / "gps_time_aligned_samples.csv"
    if analysis_samples.exists():
        aligned = pd.read_csv(analysis_samples)
        aligned_key = "vio_state_t" if "vio_state_t" in aligned.columns else "t"
        aligned_small = aligned.rename(columns={aligned_key: "analysis_match_t"})
        merged = merge_nearest(master[["timestamp_master"]], aligned_small, "timestamp_master", "analysis_match_t", args.analysis_tolerance_s)
        aligned_map = {
            "gps_timestamp_aligned": "gps_update_t",
            "vio_sample_delay_ms": "vio_sample_delay_ms",
            "truth_position_e": "gps_E",
            "truth_position_n": "gps_N",
            "truth_position_u": "gps_U",
            "truth_velocity_e": "gps_vE",
            "truth_velocity_n": "gps_vN",
            "truth_velocity_u": "gps_vU",
            "truth_course_deg": "gps_course_deg",
            "gps_velocity_source": "gps_velocity_source",
            "vio_eval_aligned_position_e": "vio_E",
            "vio_eval_aligned_position_n": "vio_N",
            "vio_eval_aligned_position_u": "vio_U",
            "vio_eval_aligned_velocity_e": "vio_vE",
            "vio_eval_aligned_velocity_n": "vio_vN",
            "vio_eval_aligned_velocity_u": "vio_vU",
            "err_e": "err_E",
            "err_n": "err_N",
            "err_u": "err_U",
            "err_xy": "err_XY",
            "err_3d": "err_3D",
            "err_along": "err_along",
            "err_cross": "err_cross",
            "err_vertical": "err_vertical",
            "err_v_e": "err_vE",
            "err_v_n": "err_vN",
            "err_v_u": "err_vU",
            "err_v_along": "err_v_along",
            "err_v_cross": "err_v_cross",
            "err_v_vertical": "err_v_vertical",
            "course_error_deg": "course_error_deg",
        }
        fill_if_present(master, merged, aligned_map)
        master["alignment_mode"] = "start-heading"
        master["alignment_id"] = "analysis_start_heading"
        master["alignment_applied"] = True
        master["analysis_source"] = str(analysis_samples)
        sources["analysis"] = str(analysis_samples)
        summary_path = analysis_samples.parent.parent / "metadata" / "global_summary.json"
        if summary_path.exists():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                yaw_deg = float(summary["start_heading_rotation_deg"])
                master["alignment_rotation_yaw_deg"] = yaw_deg
                valid_align = master[
                    [
                        "vio_raw_position_x",
                        "vio_raw_position_y",
                        "vio_raw_position_z",
                        "vio_eval_aligned_position_e",
                        "vio_eval_aligned_position_n",
                        "vio_eval_aligned_position_u",
                    ]
                ].notna().all(axis=1)
                if valid_align.any():
                    first = master.loc[valid_align].iloc[0]
                    yaw = math.radians(yaw_deg)
                    c, s = math.cos(yaw), math.sin(yaw)
                    raw_rot = (
                        c * float(first["vio_raw_position_x"]) - s * float(first["vio_raw_position_y"]),
                        s * float(first["vio_raw_position_x"]) + c * float(first["vio_raw_position_y"]),
                        float(first["vio_raw_position_z"]),
                    )
                    master["alignment_translation_e"] = float(first["vio_eval_aligned_position_e"]) - raw_rot[0]
                    master["alignment_translation_n"] = float(first["vio_eval_aligned_position_n"]) - raw_rot[1]
                    master["alignment_translation_u"] = float(first["vio_eval_aligned_position_u"]) - raw_rot[2]
            except Exception as exc:
                print(f"warning: failed to read analysis alignment metadata: {exc}", file=sys.stderr)

    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    missing = [name for name in schema_names() if name not in master.columns]
    if missing:
        master = pd.concat([master, pd.DataFrame({name: pd.NA for name in missing}, index=master.index)], axis=1)
    master = master[schema_names()]
    csv_path = out_root / "MASTER_FLIGHT_DATA.csv"
    master.to_csv(csv_path, index=False)
    wrote = [str(csv_path)]

    parquet_path = out_root / "MASTER_FLIGHT_DATA.parquet"
    try:
        master.to_parquet(parquet_path, index=False)
        wrote.append(str(parquet_path))
    except Exception as exc:
        print(f"warning: parquet not written: {exc}", file=sys.stderr)

    if args.xlsx:
        xlsx_path = out_root / "MASTER_FLIGHT_DATA.xlsx"
        try:
            master.to_excel(xlsx_path, index=False)
            wrote.append(str(xlsx_path))
        except Exception as exc:
            print(f"warning: xlsx not written: {exc}", file=sys.stderr)

    print("wrote " + ", ".join(wrote))
    print_build_summary(master, sources)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_schema = sub.add_parser("schema", help="write or print the master data schema")
    p_schema.add_argument("--out", help="CSV or JSON path")
    p_schema.set_defaults(func=cmd_schema)

    p_index = sub.add_parser("index", help="index baseline manifest and local result run dirs")
    p_index.add_argument("--result-root", action="append", default=[], help="result root to scan; can repeat")
    p_index.add_argument("--baseline-manifest", help="baseline/latest manifest JSON")
    p_index.add_argument("--out", required=True, help="output CSV")
    p_index.set_defaults(func=cmd_index)

    p_build = sub.add_parser("build", help="build one run's master flight data table")
    p_build.add_argument("--run-dir", required=True)
    p_build.add_argument("--out-root", required=True)
    p_build.add_argument("--run-id", default="")
    p_build.add_argument("--flight-id", default="")
    p_build.add_argument("--config-id", default="")
    p_build.add_argument("--traj-tolerance-s", type=float, default=0.02)
    p_build.add_argument("--analysis-tolerance-s", type=float, default=0.25)
    p_build.add_argument("--fc-raw", default="", help="optional raw FC/MEMS CSV; auto-detected from dataset when omitted")
    p_build.add_argument("--fc-offset-s", type=float, default=None, help="FC relative-to-camera offset; default parses fc_init metadata")
    p_build.add_argument("--fc-tolerance-s", type=float, default=0.25, help="max endpoint distance for FC interpolation")
    p_build.add_argument("--dashboard-alignment-json", default="", help="dashboard display alignment metadata JSON")
    p_build.add_argument("--xlsx", action="store_true")
    p_build.set_defaults(func=cmd_build)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
