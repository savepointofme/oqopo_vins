#!/usr/bin/env python3
"""Run canonical evaluation and assemble the calibration-factorial evidence."""

from __future__ import annotations

import argparse
import csv
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import p4_yaw_calibration_factorial_20260716_ultra as experiment


ROOT = experiment.ROOT_WIN
SNAPSHOT = ROOT / "provenance/analysis_tool_snapshot"
FFA = SNAPSHOT / "analysis/full_flight_error_analysis.py"
FET = SNAPSHOT / "analysis/flight_eval_tool.py"
ROOT_CAUSE = SNAPSHOT / "analysis/p4_p5_yaw_drift_root_cause.py"
ANALYSIS_ROOT = ROOT / "analysis"
SUMMARY_ROOT = ROOT / "summary"


def configure_variant(name: str | None) -> None:
    global ANALYSIS_ROOT, SUMMARY_ROOT
    if name:
        ANALYSIS_ROOT = ROOT / "analysis" / name
        SUMMARY_ROOT = ROOT / "summary" / name
    else:
        ANALYSIS_ROOT = ROOT / "analysis"
        SUMMARY_ROOT = ROOT / "summary"


def evaluation_gps(flight: dict[str, Any]) -> Path:
    return Path(flight.get("eval_gps_win", flight["gps_win"]))


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def invoke(argv: list[str], log: Path, *, cwd: Path | None = None) -> dict[str, Any]:
    log.parent.mkdir(parents=True, exist_ok=True)
    start = time.time()
    environment = dict(os.environ)
    environment["MPLBACKEND"] = "Agg"
    result = subprocess.run(
        argv, cwd=str(cwd) if cwd else None, env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding="utf-8", errors="replace",
    )
    log.write_text(result.stdout, encoding="utf-8")
    return {
        "argv": argv,
        "exit_code": result.returncode,
        "runtime_s": time.time() - start,
        "log": str(log),
    }


def run_manifest(run_id: str) -> dict[str, Any]:
    return json.loads((ROOT / f"runs/{run_id}/run_manifest.json").read_text(encoding="utf-8"))


def run_is_valid(run_id: str) -> bool:
    manifest = run_manifest(run_id)
    validation = ROOT / f"runs/{run_id}/run_validation.json"
    return (
        manifest.get("status") == "success"
        and validation.exists()
        and json.loads(validation.read_text(encoding="utf-8")).get("pass") is True
    )


def fet_output_dir(run_id: str) -> Path:
    condition, flight = run_id.split("_", 1)
    return (
        ANALYSIS_ROOT / "flight_eval_tool" / run_id
        / f"20260716_{flight}_{condition}_p4_cal_factorial_成功"
    )


def build_run_spec(run_id: str) -> Path:
    condition, flight = run_id.split("_", 1)
    f = experiment.FLIGHTS[flight]
    run = ROOT / "runs" / run_id
    spec = {
        "experiment_id": f"P4_YAW_CAL_FACTORIAL_{run_id}",
        "flight_name": flight,
        "method_name": f"{condition}_p4_cal_factorial",
        "date": "20260716",
        "status": "success",
        "source_experiment_folder": run_id,
        "t0": f["start_s"],
        "t1": f["until_s"],
        "alignment": {"mode": "start_heading", "course_window_s": 60.0},
        "sampling": {"mode": "gps_update_after_vio", "max_delay_s": 0.2},
        "segmentation": {"mode": "geometry"},
        "velocity_source": "flight_controller_raw",
        "gps_csv": str(evaluation_gps(f)),
        "vio_traj": str(run / "traj_nav.txt"),
        "vio_bias": str(run / "traj.txt.bias"),
        "diag_csv": str(run / "diag.csv"),
        "vio_yaw_diag": str(run / "vio_yaw_diag.csv"),
        "notes": (
            "Calibration factorial; GPS update-time start-heading diagnostic; "
            "scale preserved; no full-trajectory best fit; GPS horizontal/course/yaw evaluation-only."
        ),
    }
    path = ANALYSIS_ROOT / "run_specs" / f"{run_id}.json"
    write_json(path, spec)
    return path


def ffa_command(run_id: str, alignment: str, output: Path) -> list[str]:
    condition, flight = run_id.split("_", 1)
    f = experiment.FLIGHTS[flight]
    run = ROOT / "runs" / run_id
    return [
        sys.executable, str(FFA),
        "--gps", str(evaluation_gps(f)),
        "--vio-traj", str(run / "traj_nav.txt"),
        "--vio-bias", str(run / "traj.txt.bias"),
        "--vio-diag", str(run / "diag.csv"),
        "--vio-yaw-diag", str(run / "vio_yaw_diag.csv"),
        "--out-dir", str(output),
        "--t0", str(f["start_s"]),
        "--t1", str(f["until_s"]),
        "--evaluation-window-mode", "explicit_or_overlap",
        "--alignment-mode", alignment,
        "--flight-name", flight,
        "--method-name", condition,
        "--experiment-id", f"P4_YAW_CAL_FACTORIAL_{run_id}_{alignment}",
        "--source-package", str(run),
        "--experiment-config", str(run / "run_manifest.json"),
        "--run-status", "success",
        "--crop-reason", "Fixed registered first-error replay window; no candidate-dependent crop.",
        "--notes", (
            "GPS timestamps are the statistics grid. traj_nav is G_nav. "
            "No full-trajectory best fit; GPS E/N/course/yaw are offline references."
        ),
    ]


def analyze_one(run_id: str) -> dict[str, Any]:
    status_path = ANALYSIS_ROOT / "status" / f"{run_id}.json"
    status: dict[str, Any] = {
        "run_id": run_id,
        "started_at": now_iso(),
        "valid_input": run_is_valid(run_id),
        "steps": {},
    }
    if not status["valid_input"]:
        status.update({"status": "skipped_invalid_run", "finished_at": now_iso()})
        write_json(status_path, status)
        return status

    spec = build_run_spec(run_id)
    inspect = invoke(
        [sys.executable, str(FET), "inspect-run", "--run-spec", str(spec)],
        ANALYSIS_ROOT / f"logs/{run_id}_flight_eval_inspect.log",
    )
    status["steps"]["flight_eval_inspect"] = inspect

    if inspect["exit_code"] == 0:
        single = invoke(
            [
                sys.executable, str(FET), "single", "--run-spec", str(spec),
                "--out-root", str(ANALYSIS_ROOT / "flight_eval_tool"),
            ],
            ANALYSIS_ROOT / f"logs/{run_id}_flight_eval_single.log",
        )
    else:
        single = {"exit_code": 98, "skipped": "inspect-run failed"}
    status["steps"]["flight_eval_single"] = single

    start_out = ANALYSIS_ROOT / "full_flight_start_heading" / run_id
    absolute_out = ANALYSIS_ROOT / "full_flight_absolute" / run_id
    status["steps"]["full_flight_start_heading"] = invoke(
        ffa_command(run_id, "start_heading", start_out),
        ANALYSIS_ROOT / f"logs/{run_id}_full_flight_start_heading.log",
    )
    status["steps"]["full_flight_absolute"] = invoke(
        ffa_command(run_id, "absolute_navigation_no_post_alignment", absolute_out),
        ANALYSIS_ROOT / f"logs/{run_id}_full_flight_absolute.log",
    )

    status["status"] = (
        "success"
        if all(step.get("exit_code") == 0 for step in status["steps"].values())
        else "partial_or_failed"
    )
    status["finished_at"] = now_iso()
    status["outputs"] = {
        "flight_eval_tool": str(fet_output_dir(run_id)),
        "start_heading": str(start_out),
        "absolute": str(absolute_out),
    }
    write_json(status_path, status)
    return status


def run_phase_analysis(run_id: str, alignment: str = "start_heading") -> dict[str, Any]:
    condition, flight = run_id.split("_", 1)
    f = experiment.FLIGHTS[flight]
    phase = f["phase_contract"]
    active_run = ROOT / "runs" / run_id
    fixed_run_id = f"C0_{flight}"
    fixed_run = ROOT / "runs" / fixed_run_id
    if alignment == "start_heading":
        full_flight_root = "full_flight_start_heading"
        phase_root = "phase_root_cause"
        status_suffix = "phase"
    elif alignment == "absolute_navigation_no_post_alignment":
        full_flight_root = "full_flight_absolute"
        phase_root = "phase_root_cause_absolute"
        status_suffix = "phase_absolute"
    else:
        raise ValueError(f"unsupported phase alignment: {alignment}")
    active_csv = ANALYSIS_ROOT / full_flight_root / run_id / "data/gps_time_aligned_samples.csv"
    fixed_csv = ANALYSIS_ROOT / full_flight_root / fixed_run_id / "data/gps_time_aligned_samples.csv"
    out = ANALYSIS_ROOT / phase_root / run_id
    if not (active_csv.exists() and fixed_csv.exists()):
        result = {
            "run_id": run_id,
            "alignment": alignment,
            "exit_code": 97,
            "reason": f"missing canonical {alignment} CSV",
        }
        write_json(ANALYSIS_ROOT / f"status/{run_id}_{status_suffix}.json", result)
        return result
    argv = [
        sys.executable, str(ROOT_CAUSE),
        "--flight", flight,
        "--active-run", str(active_run),
        "--fixed-run", str(fixed_run),
        "--active-official-csv", str(active_csv),
        "--fixed-official-csv", str(fixed_csv),
        "--fc-nav", str(f["fc_win"]),
        "--imu-csv", str(f["dataset_win"] / "imu0/data.csv"),
        "--focus-start", str(phase["focus"][0]),
        "--focus-end", str(phase["focus"][1]),
        "--turn-start", str(phase["turn"][0]),
        "--turn-end", str(phase["turn"][1]),
        "--straight-end", str(phase["post_straight_late"][1]),
        "--out-dir", str(out),
    ]
    result = invoke(
        argv,
        ANALYSIS_ROOT / f"logs/{run_id}_{status_suffix}_root_cause.log",
    )
    result["run_id"] = run_id
    result["alignment"] = alignment
    result["output"] = str(out)
    write_json(ANALYSIS_ROOT / f"status/{run_id}_{status_suffix}.json", result)
    return result


def read_single_csv(path: Path) -> dict[str, str]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no rows in {path}")
    return rows[0]


def float_or_nan(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_float_file(path: Path) -> float:
    try:
        return float(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return math.nan


def resource_metric(path: Path, label: str) -> float:
    if not path.exists():
        return math.nan
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if stripped.startswith(label):
            return float_or_nan(stripped.split(":", 1)[1].strip())
    return math.nan


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def state_delta(candidate: dict[str, Any], baseline: dict[str, Any]) -> float:
    values: list[float] = []
    for key in [
        "q_Gnav_to_I_xyzw", "p_I_in_Gnav_m", "v_I_in_Gnav_mps",
        "gyro_bias_rad_s", "accel_bias_mps2",
    ]:
        a = candidate.get("state", {}).get(key, [])
        b = baseline.get("state", {}).get(key, [])
        if len(a) == len(b):
            values.extend(abs(float(x) - float(y)) for x, y in zip(a, b))
    return max(values) if values else math.nan


def state_vector_delta_norm(
    candidate: dict[str, Any], baseline: dict[str, Any], key: str
) -> float:
    a = candidate.get("state", {}).get(key, [])
    b = baseline.get("state", {}).get(key, [])
    if len(a) != len(b) or not a:
        return math.nan
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def quaternion_delta_deg(candidate: dict[str, Any], baseline: dict[str, Any]) -> float:
    key = "q_Gnav_to_I_xyzw"
    a = candidate.get("state", {}).get(key, [])
    b = baseline.get("state", {}).get(key, [])
    if len(a) != 4 or len(b) != 4:
        return math.nan
    an = math.sqrt(sum(float(value) ** 2 for value in a))
    bn = math.sqrt(sum(float(value) ** 2 for value in b))
    if an <= 0.0 or bn <= 0.0:
        return math.nan
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)) / (an * bn))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def summarize() -> None:
    summary = SUMMARY_ROOT
    start_rows: list[dict[str, Any]] = []
    absolute_rows: list[dict[str, Any]] = []
    phase_rows: list[dict[str, Any]] = []
    phase_absolute_rows: list[dict[str, Any]] = []
    fingerprint_rows: list[dict[str, Any]] = []
    runtime_rows: list[dict[str, Any]] = []
    schemas: dict[str, Any] = {}

    fingerprints: dict[str, dict[str, Any]] = {}
    for condition in experiment.CONDITIONS:
        for flight in experiment.FLIGHTS:
            run_id = f"{condition}_{flight}"
            for mode, target in [
                ("start_heading", start_rows),
                ("absolute_navigation_no_post_alignment", absolute_rows),
            ]:
                root_name = "full_flight_start_heading" if mode == "start_heading" else "full_flight_absolute"
                path = ANALYSIS_ROOT / root_name / run_id / "tables/global_summary.csv"
                if path.exists():
                    row = read_single_csv(path)
                    target.append({"run_id": run_id, "condition": condition, "flight": flight, **row})

            phase_path = ANALYSIS_ROOT / "phase_root_cause" / run_id / "phase_summary.csv"
            if phase_path.exists():
                with phase_path.open("r", encoding="utf-8-sig", newline="") as stream:
                    for row in csv.DictReader(stream):
                        if row.get("method") == "active":
                            phase_rows.append({"run_id": run_id, "condition": condition, "flight": flight, **row})
            phase_absolute_path = (
                ANALYSIS_ROOT
                / "phase_root_cause_absolute"
                / run_id
                / "phase_summary.csv"
            )
            if phase_absolute_path.exists():
                with phase_absolute_path.open("r", encoding="utf-8-sig", newline="") as stream:
                    for row in csv.DictReader(stream):
                        if row.get("method") == "active":
                            phase_absolute_rows.append({
                                "run_id": run_id,
                                "condition": condition,
                                "flight": flight,
                                **row,
                            })

            fp_path = ROOT / f"runs/{run_id}/p4_release_fingerprint.json"
            if fp_path.exists():
                fp = json.loads(fp_path.read_text(encoding="utf-8"))
                fingerprints[run_id] = fp
                fingerprint_rows.append({
                    "run_id": run_id,
                    "condition": condition,
                    "flight": flight,
                    "readiness_level": fp.get("readiness_level"),
                    "result_state_timestamp_s": fp.get("result_state_timestamp_s"),
                    "first_openvins_output_time_s": fp.get("first_openvins_output_time_s"),
                    "window_fingerprint": fp.get("window_fingerprint"),
                    "feedback_state_mask": fp.get("candidate_feedback_state_mask"),
                    "attitude_residual_deg": fp.get("candidate_post_correction_attitude_residual_deg"),
                    "position_nis": fp.get("candidate_last_position_nis"),
                    "velocity_nis": fp.get("candidate_last_velocity_nis"),
                    "visual_compensated_p95_px": fp.get("candidate_visual_compensated_p95_px"),
                    "supported_update_counts": json.dumps(fp.get("candidate_group_supported_update_counts")),
                    "stable_update_counts": json.dumps(fp.get("candidate_group_post_feedback_stable_update_counts")),
                    "state_json": json.dumps(fp.get("state")),
                })

            validation_path = ROOT / f"runs/{run_id}/run_validation.json"
            if validation_path.exists():
                validation = json.loads(validation_path.read_text(encoding="utf-8"))
                schemas[run_id] = {
                    key: value.get("columns", [])
                    for key, value in validation.get("diagnostic_schemas", {}).items()
                }
            resource = ROOT / f"runs/{run_id}/resource_usage.txt"
            manifest = run_manifest(run_id)
            start_epoch = read_float_file(ROOT / f"runs/{run_id}/process_start_epoch_s.txt")
            end_epoch = read_float_file(ROOT / f"runs/{run_id}/process_end_epoch_s.txt")
            runtime_rows.append({
                "run_id": run_id,
                "condition": condition,
                "flight": flight,
                "status": manifest.get("status"),
                "failure_reason": manifest.get("execution", {}).get("failure_reason", ""),
                "runtime_s": end_epoch - start_epoch,
                "max_rss_kb": resource_metric(
                    resource, "Maximum resident set size (kbytes)"
                ),
                "resource_usage_path": str(resource) if resource.exists() else "",
            })

    write_csv(summary / "global_metrics_start_heading.csv", start_rows)
    write_csv(summary / "global_metrics_absolute.csv", absolute_rows)
    write_csv(summary / "phase_metrics_start_heading.csv", phase_rows)
    write_csv(summary / "phase_metrics_absolute.csv", phase_absolute_rows)
    write_csv(summary / "p4_release_fingerprints.csv", fingerprint_rows)
    write_csv(summary / "runtime_and_failures.csv", runtime_rows)

    schema_names = sorted({name for run in schemas.values() for name in run})
    schema_comparison: dict[str, Any] = {"all_identical": True, "files": {}}
    for name in schema_names:
        values = {run_id: files.get(name) for run_id, files in schemas.items()}
        unique = {json.dumps(value) for value in values.values()}
        identical = len(unique) == 1
        schema_comparison["files"][name] = {"identical": identical, "runs": values}
        schema_comparison["all_identical"] &= identical
    write_json(summary / "diagnostic_schema_comparison.json", schema_comparison)

    pairs = [
        ("zminus_mixed", "C6", "C0"),
        ("zminus_june12_kd", "C7", "C2"),
        ("june12_toff_on_june12_kd", "C1", "C2"),
        ("june12_toff_on_mixed_kd", "C3", "C0"),
        ("june12_kd_on_june_tci_toff0", "C2", "C0"),
        ("june12_kd_on_june_tci_june_toff", "C1", "C3"),
        ("june12_kd_on_old_tci_toff0", "C5", "C4"),
        ("old_tci_on_mixed_kd", "C4", "C0"),
        ("old_tci_on_june12_kd", "C5", "C2"),
    ]

    fp_delta_rows: list[dict[str, Any]] = []
    for flight in experiment.FLIGHTS:
        baseline = fingerprints.get(f"C0_{flight}")
        if baseline is None:
            continue
        for condition in experiment.CONDITIONS:
            run_id = f"{condition}_{flight}"
            candidate = fingerprints.get(run_id)
            if candidate is None:
                continue
            fp_delta_rows.append({
                "run_id": run_id,
                "condition": condition,
                "flight": flight,
                "same_window_fingerprint_as_C0": candidate.get("window_fingerprint") == baseline.get("window_fingerprint"),
                "state_max_abs_delta_vs_C0": state_delta(candidate, baseline),
                "release_time_delta_s_vs_C0": float_or_nan(candidate.get("result_state_timestamp_s")) - float_or_nan(baseline.get("result_state_timestamp_s")),
                "first_output_delta_s_vs_C0": float_or_nan(candidate.get("first_openvins_output_time_s")) - float_or_nan(baseline.get("first_openvins_output_time_s")),
                "attitude_residual_delta_deg_vs_C0": float_or_nan(candidate.get("candidate_post_correction_attitude_residual_deg")) - float_or_nan(baseline.get("candidate_post_correction_attitude_residual_deg")),
            })
    write_csv(summary / "p4_fingerprint_deltas.csv", fp_delta_rows)

    fp_pair_rows: list[dict[str, Any]] = []
    for pair_name, candidate_id, baseline_id in pairs:
        for flight in experiment.FLIGHTS:
            candidate = fingerprints.get(f"{candidate_id}_{flight}")
            baseline = fingerprints.get(f"{baseline_id}_{flight}")
            if candidate is None or baseline is None:
                continue
            fp_pair_rows.append({
                "pair": pair_name,
                "candidate": candidate_id,
                "baseline": baseline_id,
                "flight": flight,
                "same_window_fingerprint": (
                    candidate.get("window_fingerprint")
                    == baseline.get("window_fingerprint")
                ),
                "state_max_abs_delta": state_delta(candidate, baseline),
                "q_angle_delta_deg": quaternion_delta_deg(candidate, baseline),
                "p_delta_norm_m": state_vector_delta_norm(
                    candidate, baseline, "p_I_in_Gnav_m"
                ),
                "v_delta_norm_mps": state_vector_delta_norm(
                    candidate, baseline, "v_I_in_Gnav_mps"
                ),
                "bg_delta_norm_rad_s": state_vector_delta_norm(
                    candidate, baseline, "gyro_bias_rad_s"
                ),
                "ba_delta_norm_mps2": state_vector_delta_norm(
                    candidate, baseline, "accel_bias_mps2"
                ),
                "release_time_delta_s": (
                    float_or_nan(candidate.get("result_state_timestamp_s"))
                    - float_or_nan(baseline.get("result_state_timestamp_s"))
                ),
                "first_output_delta_s": (
                    float_or_nan(candidate.get("first_openvins_output_time_s"))
                    - float_or_nan(baseline.get("first_openvins_output_time_s"))
                ),
                "attitude_residual_delta_deg": (
                    float_or_nan(candidate.get("candidate_post_correction_attitude_residual_deg"))
                    - float_or_nan(baseline.get("candidate_post_correction_attitude_residual_deg"))
                ),
                "position_nis_delta": (
                    float_or_nan(candidate.get("candidate_last_position_nis"))
                    - float_or_nan(baseline.get("candidate_last_position_nis"))
                ),
                "velocity_nis_delta": (
                    float_or_nan(candidate.get("candidate_last_velocity_nis"))
                    - float_or_nan(baseline.get("candidate_last_velocity_nis"))
                ),
                "visual_compensated_p95_delta_px": (
                    float_or_nan(candidate.get("candidate_visual_compensated_p95_px"))
                    - float_or_nan(baseline.get("candidate_visual_compensated_p95_px"))
                ),
            })
    write_csv(summary / "p4_fingerprint_pair_deltas.csv", fp_pair_rows)

    start_index = {(row["condition"], row["flight"]): row for row in start_rows}
    absolute_index = {(row["condition"], row["flight"]): row for row in absolute_rows}
    phase_index = {(row["condition"], row["flight"], row["phase"]): row for row in phase_rows}
    phase_absolute_index = {
        (row["condition"], row["flight"], row["phase"]): row
        for row in phase_absolute_rows
    }
    delta_rows: list[dict[str, Any]] = []
    global_metrics = [
        "xy_rmse_m", "final_xy_error_m", "vertical_rmse_m", "vxy_vec_rmse_mps",
        "speed_rmse_mps", "yaw/course_rmse_deg", "yaw/course_final_deg",
    ]
    phase_metrics = [
        "course_error_deg_median", "course_error_deg_p95",
        "course_error_deg_slope_per_s", "attitude_yaw_error_deg_median",
        "position_error_xy_m_median", "position_error_xy_m_p95",
        "position_error_xy_m_slope_per_s", "err_along_median",
        "err_cross_median", "err_U_median", "velocity_error_xy_mps_median",
        "velocity_error_xy_mps_p95", "velocity_error_xy_mps_slope_per_s",
    ]
    for pair_name, candidate, baseline in pairs:
        for flight in experiment.FLIGHTS:
            for scope, index in [
                ("global_start_heading", start_index),
                ("global_absolute", absolute_index),
            ]:
                a = index.get((candidate, flight))
                b = index.get((baseline, flight))
                if a and b:
                    for metric in global_metrics:
                        av, bv = float_or_nan(a.get(metric)), float_or_nan(b.get(metric))
                        delta_rows.append({
                            "pair": pair_name, "candidate": candidate, "baseline": baseline,
                            "flight": flight, "scope": scope, "phase": "global",
                            "metric": metric, "candidate_value": av, "baseline_value": bv,
                            "signed_delta": av - bv,
                            "absolute_magnitude_delta": abs(av) - abs(bv),
                            "improved_by_abs_magnitude": abs(av) < abs(bv),
                        })
            for scope, index in [
                ("fixed_phase_start_heading", phase_index),
                ("fixed_phase_absolute", phase_absolute_index),
            ]:
                for phase in ["pre_turn", "turn", "post_straight_early", "post_straight_late"]:
                    a = index.get((candidate, flight, phase))
                    b = index.get((baseline, flight, phase))
                    if not (a and b):
                        continue
                    for metric in phase_metrics:
                        av, bv = float_or_nan(a.get(metric)), float_or_nan(b.get(metric))
                        delta_rows.append({
                            "pair": pair_name, "candidate": candidate, "baseline": baseline,
                            "flight": flight, "scope": scope, "phase": phase,
                            "metric": metric, "candidate_value": av, "baseline_value": bv,
                            "signed_delta": av - bv,
                            "absolute_magnitude_delta": abs(av) - abs(bv),
                            "improved_by_abs_magnitude": abs(av) < abs(bv),
                        })
    write_csv(summary / "hypothesis_pair_deltas.csv", delta_rows)

    completed = [row["run_id"] for row in runtime_rows if row["status"] == "success"]
    failed = [row["run_id"] for row in runtime_rows if row["status"] != "success"]
    key_rows = [
        row for row in delta_rows
        if row["metric"] in {"course_error_deg_median", "yaw/course_rmse_deg"}
        and (
            row["scope"] == "global_start_heading"
            or (
                row["scope"] == "fixed_phase_start_heading"
                and row["phase"] in {"turn", "post_straight_late"}
            )
        )
    ]
    table = [
        "| Pair | Flight | Scope/phase | Metric | Candidate | Baseline | Δ|error| |",
        "|---|---|---|---|---:|---:|---:|",
    ]
    for row in key_rows:
        table.append(
            f"| {row['pair']} | {row['flight']} | {row['phase']} | {row['metric']} | "
            f"{row['candidate_value']:.6g} | {row['baseline_value']:.6g} | "
            f"{row['absolute_magnitude_delta']:.6g} |"
        )
    report = f"""# P4 yaw calibration factorial results

Generated at `{now_iso()}` from the frozen estimator binary and frozen analysis
tool snapshot. GPS update-time rows are authoritative. Start-heading preserves
scale and absolute navigation uses no post alignment; no best-fit result is
used here.

- Analysis root: `{ANALYSIS_ROOT}`
- fly1 evaluation GPS: `{evaluation_gps(experiment.FLIGHTS['fly1'])}`
- fly3 evaluation GPS: `{evaluation_gps(experiment.FLIGHTS['fly3'])}`

## Completeness

- Completed/validated estimator runs ({len(completed)}): {', '.join(completed) if completed else 'none'}
- Failed/incomplete estimator runs ({len(failed)}): {', '.join(failed) if failed else 'none'}
- Diagnostic CSV schemas identical: `{schema_comparison['all_identical']}`

## Key paired heading/course deltas

Negative `Δ|error|` means the candidate reduced absolute error magnitude.

{chr(10).join(table)}

## Evidence map

- `global_metrics_start_heading.csv`: canonical GPS-grid relative-drift diagnostics.
- `global_metrics_absolute.csv`: primary direct `G_nav` errors with no post-fit.
- `phase_metrics_start_heading.csv` and `phase_metrics_absolute.csv`: fixed
  pre-turn/turn/post-straight metrics under both registered alignments.
- `hypothesis_pair_deltas.csv`: start-heading and absolute clean z-minus, K/D,
  toff, and old-T_C_I pairs.
- `p4_release_fingerprints.csv`, `p4_fingerprint_deltas.csv`, and
  `p4_fingerprint_pair_deltas.csv`: release identity,
  q/p/v/bg/ba, NIS, support, and state deltas.
- `diagnostic_schema_comparison.json`: residual/NIS/support diagnostic comparability.
- Per-run standard PNG/SVG/CSV/Markdown/HTML outputs are under `analysis/`.

Final causal interpretation must use cross-flight sign consistency and the P4
fingerprint split defined in `RUN_MATRIX.md`; a same-flight gain alone is not
accepted.
"""
    (summary / "RESULTS.md").write_text(report, encoding="utf-8", newline="\n")


def compare_fet() -> list[dict[str, Any]]:
    results = []
    for flight in experiment.FLIGHTS:
        runs = [fet_output_dir(f"{condition}_{flight}") for condition in experiment.CONDITIONS]
        runs = [path for path in runs if (path / "metadata/RUN_STATUS.txt").exists() or (path / "tables/global_summary.csv").exists()]
        if not runs:
            results.append({"flight": flight, "exit_code": 97, "reason": "no completed flight_eval_tool runs"})
            continue
        out = ANALYSIS_ROOT / "flight_eval_compare" / flight
        argv = [sys.executable, str(FET), "compare", "--runs", *map(str, runs), "--out-dir", str(out)]
        result = invoke(argv, ANALYSIS_ROOT / f"logs/flight_eval_compare_{flight}.log")
        result["flight"] = flight
        result["output"] = str(out)
        results.append(result)
    write_json(ANALYSIS_ROOT / "status/flight_eval_compare.json", results)
    return results


def run_phase_batch(max_workers: int, alignment: str) -> bool:
    run_ids = [
        f"{condition}_{flight}"
        for condition in experiment.CONDITIONS
        for flight in experiment.FLIGHTS
    ]
    results: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(run_phase_analysis, run_id, alignment): run_id
            for run_id in run_ids
        }
        for future in as_completed(futures):
            run_id = futures[future]
            try:
                result = future.result()
            except Exception as exc:
                result = {
                    "run_id": run_id,
                    "alignment": alignment,
                    "exit_code": 96,
                    "error": repr(exc),
                }
                suffix = (
                    "phase" if alignment == "start_heading" else "phase_absolute"
                )
                write_json(ANALYSIS_ROOT / f"status/{run_id}_{suffix}.json", result)
            results.append(result)
            print(run_id, alignment, result.get("exit_code"))
    return all(result.get("exit_code") == 0 for result in results)


def run_all(max_workers: int) -> int:
    run_ids = [f"{condition}_{flight}" for condition in experiment.CONDITIONS for flight in experiment.FLIGHTS]
    statuses: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {pool.submit(analyze_one, run_id): run_id for run_id in run_ids}
        for future in as_completed(futures):
            run_id = futures[future]
            try:
                status = future.result()
            except Exception as exc:
                status = {"run_id": run_id, "status": "exception", "error": repr(exc)}
                write_json(ANALYSIS_ROOT / f"status/{run_id}.json", status)
            statuses.append(status)
            print(run_id, status.get("status"))

    phase_start_ok = run_phase_batch(max_workers, "start_heading")
    phase_absolute_ok = run_phase_batch(
        max_workers, "absolute_navigation_no_post_alignment"
    )

    compare_fet()
    summarize()
    write_json(ANALYSIS_ROOT / "status/analysis_batch.json", {
        "finished_at": now_iso(), "max_workers": max_workers, "runs": statuses,
    })
    return 0 if (
        all(status.get("status") == "success" for status in statuses)
        and phase_start_ok
        and phase_absolute_ok
    ) else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--summarize-only", action="store_true")
    parser.add_argument("--phase-absolute-only", action="store_true")
    parser.add_argument("--variant")
    args = parser.parse_args()
    if args.variant:
        variant = Path(args.variant)
        if variant.name != args.variant or args.variant in {".", ".."}:
            parser.error("--variant must be one safe directory name")
    configure_variant(args.variant)
    if args.summarize_only and args.phase_absolute_only:
        parser.error("--summarize-only and --phase-absolute-only are mutually exclusive")
    if args.summarize_only:
        summarize()
        return 0
    if args.phase_absolute_only:
        ok = run_phase_batch(
            args.max_workers, "absolute_navigation_no_post_alignment"
        )
        summarize()
        return 0 if ok else 1
    return run_all(args.max_workers)


if __name__ == "__main__":
    raise SystemExit(main())
