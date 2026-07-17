#!/usr/bin/env python3
"""Canonical GPS-grid analysis for the locked K/D split experiment."""

from __future__ import annotations

import argparse
import csv
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

import p4_intrinsics_distortion_split_20260716 as experiment
import analyze_p4_yaw_calibration_factorial_20260716_ultra as engine


ROOT = experiment.ROOT_WIN
SNAPSHOT = ROOT / "provenance/analysis_tool_snapshot"
ANALYSIS_ROOT = ROOT / "analysis"
SUMMARY_ROOT = ROOT / "summary"
FFA = SNAPSHOT / "analysis/full_flight_error_analysis.py"
FET = SNAPSHOT / "analysis/flight_eval_tool.py"
ROOT_CAUSE = SNAPSHOT / "analysis/p4_p5_yaw_drift_root_cause.py"
COMMON_SEGMENT_ROOT = Path(
    r"C:\Users\baloney\Desktop\实验目录\_OPENVINS_ORGANIZED_20260625"
    r"\01_final_locked_stride_sweep\ADAPTIVE_STRIDE_VALIDATION_OFFICIAL_20260621"
)


def configure_engine() -> None:
    engine.experiment = experiment
    engine.ROOT = ROOT
    engine.SNAPSHOT = SNAPSHOT
    engine.FFA = FFA
    engine.FET = FET
    engine.ROOT_CAUSE = ROOT_CAUSE
    engine.ANALYSIS_ROOT = ANALYSIS_ROOT
    engine.SUMMARY_ROOT = SUMMARY_ROOT


configure_engine()


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


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


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_one(path: Path) -> dict[str, str]:
    rows = read_rows(path)
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    return rows[0]


def number(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def evaluation_gps(flight: dict[str, Any]) -> Path:
    return Path(flight.get("eval_gps_win", flight["gps_win"]))


def fet_output_dir(run_id: str) -> Path:
    condition, flight = run_id.rsplit("_", 1)
    return (
        ANALYSIS_ROOT / "flight_eval_tool" / run_id
        / f"20260716_{flight}_{condition}_intrinsics_distortion_split_成功"
    )


def build_run_spec(run_id: str) -> Path:
    condition, flight = run_id.rsplit("_", 1)
    info = experiment.FLIGHTS[flight]
    run = ROOT / "runs" / run_id
    spec = {
        "experiment_id": f"P4_KD_SPLIT_{experiment.SCOPE.upper()}_{run_id}",
        "flight_name": flight,
        "method_name": f"{condition}_kd_split_{experiment.SCOPE}",
        "date": "20260716",
        "status": "success",
        "source_experiment_folder": run_id,
        "t0": info["start_s"],
        "t1": info["until_s"],
        "alignment": {"mode": "start_heading", "course_window_s": 60.0},
        "sampling": {"mode": "gps_update_after_vio", "max_delay_s": 0.2},
        "segmentation": {"mode": "geometry"},
        "velocity_source": "flight_controller_raw",
        "gps_csv": str(evaluation_gps(info)),
        "vio_traj": str(run / "traj_nav.txt"),
        "vio_bias": str(run / "traj.txt.bias"),
        "diag_csv": str(run / "diag.csv"),
        "vio_yaw_diag": str(run / "vio_yaw_diag.csv"),
        "notes": (
            "K/D split; GPS update-time start-heading; FC Ve/Vn/Vu reference; "
            "scale preserved; no best-fit; GPS horizontal/course/yaw evaluation-only."
        ),
    }
    path = ANALYSIS_ROOT / "run_specs" / f"{run_id}.json"
    write_json(path, spec)
    return path


def ffa_command(run_id: str, alignment: str, output: Path) -> list[str]:
    condition, flight = run_id.rsplit("_", 1)
    info = experiment.FLIGHTS[flight]
    run = ROOT / "runs" / run_id
    argv = [
        sys.executable, str(FFA),
        "--gps", str(evaluation_gps(info)),
        "--vio-traj", str(run / "traj_nav.txt"),
        "--vio-bias", str(run / "traj.txt.bias"),
        "--vio-diag", str(run / "diag.csv"),
        "--vio-yaw-diag", str(run / "vio_yaw_diag.csv"),
        "--out-dir", str(output),
        "--alignment-mode", alignment,
        "--flight-name", flight,
        "--method-name", condition,
        "--experiment-id", f"P4_KD_SPLIT_{experiment.SCOPE.upper()}_{run_id}_{alignment}",
        "--source-package", str(run),
        "--experiment-config", str(run / "run_manifest.json"),
        "--run-status", "success",
        "--notes", (
            "GPS timestamps are the statistics grid; rich GPS contains FC Ve,Vn,Vu. "
            "traj_nav is G_nav; no full-trajectory best fit; GPS E/N/course/yaw are offline references."
        ),
    ]
    if experiment.SCOPE == "focus":
        argv.extend([
            "--t0", str(info["start_s"]),
            "--t1", str(info["until_s"]),
            "--evaluation-window-mode", "explicit_or_overlap",
            "--crop-reason", "Preregistered focus replay window; no candidate-dependent crop.",
        ])
    else:
        argv.extend([
            "--evaluation-window-mode", "airborne_auto",
            "--crop-reason", "Canonical automatic airborne full-flight window; post-touchdown excluded.",
        ])
        segment = COMMON_SEGMENT_ROOT / f"COMMON_{flight}/COMMON_LAP_SEGMENTS.csv"
        if segment.exists():
            argv.extend(["--segment-index-csv", str(segment)])
    return argv


engine.evaluation_gps = evaluation_gps
engine.fet_output_dir = fet_output_dir
engine.build_run_spec = build_run_spec
engine.ffa_command = ffa_command


def run_analysis_batch(max_workers: int) -> int:
    run_ids = [
        f"{condition}_{flight}"
        for condition in experiment.CONDITIONS
        for flight in experiment.FLIGHTS
    ]
    statuses: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {pool.submit(engine.analyze_one, run_id): run_id for run_id in run_ids}
        for future in as_completed(futures):
            run_id = futures[future]
            try:
                status = future.result()
            except Exception as exc:
                status = {"run_id": run_id, "status": "exception", "error": repr(exc)}
                write_json(ANALYSIS_ROOT / f"status/{run_id}.json", status)
            statuses.append(status)
            print(run_id, status.get("status"))

    phase_start_ok = engine.run_phase_batch(max_workers, "start_heading")
    phase_absolute_ok = engine.run_phase_batch(
        max_workers, "absolute_navigation_no_post_alignment"
    )
    compare_fet()
    summarize()
    write_json(ANALYSIS_ROOT / "status/analysis_batch.json", {
        "finished_at": now_iso(),
        "scope": experiment.SCOPE,
        "max_workers": max_workers,
        "runs": statuses,
        "phase_start_ok": phase_start_ok,
        "phase_absolute_ok": phase_absolute_ok,
    })
    return 0 if (
        all(status.get("status") == "success" for status in statuses)
        and phase_start_ok
        and phase_absolute_ok
    ) else 1


def compare_fet() -> list[dict[str, Any]]:
    results = []
    for flight in experiment.FLIGHTS:
        runs = [
            fet_output_dir(f"{condition}_{flight}")
            for condition in experiment.CONDITIONS
        ]
        runs = [
            path for path in runs
            if (path / "metadata/RUN_STATUS.txt").exists()
            or (path / "tables/global_summary.csv").exists()
        ]
        if not runs:
            results.append({"flight": flight, "exit_code": 97, "reason": "no completed runs"})
            continue
        out = ANALYSIS_ROOT / "flight_eval_compare" / flight
        result = engine.invoke(
            [sys.executable, str(FET), "compare", "--runs", *map(str, runs), "--out-dir", str(out)],
            ANALYSIS_ROOT / f"logs/flight_eval_compare_{flight}.log",
        )
        result.update({"flight": flight, "output": str(out)})
        results.append(result)
    write_json(ANALYSIS_ROOT / "status/flight_eval_compare.json", results)
    return results


def quat_to_euler_zyx_deg(q: list[float]) -> dict[str, float]:
    if len(q) != 4:
        return {"roll_deg": math.nan, "pitch_deg": math.nan, "yaw_deg": math.nan}
    x, y, z, w = [float(item) for item in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0:
        return {"roll_deg": math.nan, "pitch_deg": math.nan, "yaw_deg": math.nan}
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return {
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
    }


def wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def state_norm_delta(candidate: dict[str, Any], baseline: dict[str, Any], key: str) -> float:
    a = candidate.get("state", {}).get(key, [])
    b = baseline.get("state", {}).get(key, [])
    if not a or len(a) != len(b):
        return math.nan
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def initialization_rows(fingerprints: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for flight in experiment.FLIGHTS:
        baseline = fingerprints.get(f"C0_{flight}")
        if not baseline:
            continue
        base_euler = quat_to_euler_zyx_deg(
            baseline.get("state", {}).get("q_Gnav_to_I_xyzw", [])
        )
        for condition in experiment.CONDITIONS:
            run_id = f"{condition}_{flight}"
            candidate = fingerprints.get(run_id)
            if not candidate:
                continue
            euler = quat_to_euler_zyx_deg(
                candidate.get("state", {}).get("q_Gnav_to_I_xyzw", [])
            )
            rows.append({
                "run_id": run_id,
                "condition": condition,
                "flight": flight,
                "same_window_fingerprint_as_C0": candidate.get("window_fingerprint") == baseline.get("window_fingerprint"),
                "release_time_delta_s": number(candidate.get("result_state_timestamp_s")) - number(baseline.get("result_state_timestamp_s")),
                "first_output_delta_s": number(candidate.get("first_openvins_output_time_s")) - number(baseline.get("first_openvins_output_time_s")),
                "q_total_angle_delta_deg": engine.quaternion_delta_deg(candidate, baseline),
                "q_roll_delta_deg": wrap_deg(euler["roll_deg"] - base_euler["roll_deg"]),
                "q_pitch_delta_deg": wrap_deg(euler["pitch_deg"] - base_euler["pitch_deg"]),
                "q_yaw_delta_deg": wrap_deg(euler["yaw_deg"] - base_euler["yaw_deg"]),
                "p_delta_norm_m": state_norm_delta(candidate, baseline, "p_I_in_Gnav_m"),
                "v_delta_norm_mps": state_norm_delta(candidate, baseline, "v_I_in_Gnav_mps"),
                "bg_delta_norm_rad_s": state_norm_delta(candidate, baseline, "gyro_bias_rad_s"),
                "ba_delta_norm_mps2": state_norm_delta(candidate, baseline, "accel_bias_mps2"),
                "position_nis_delta": number(candidate.get("candidate_last_position_nis")) - number(baseline.get("candidate_last_position_nis")),
                "velocity_nis_delta": number(candidate.get("candidate_last_velocity_nis")) - number(baseline.get("candidate_last_velocity_nis")),
                "support_counts": json.dumps(candidate.get("candidate_group_supported_update_counts")),
            })
    return rows


PAIR_DEFINITIONS = [
    ("K_main_at_current_D", "KONLY", "C0"),
    ("D_main_at_current_K", "DONLY", "C0"),
    ("KD_total", "KD", "C0"),
    ("D_effect_at_june_K", "KD", "KONLY"),
    ("K_effect_at_june_D", "KD", "DONLY"),
]

GLOBAL_METRICS = [
    "xy_rmse_m", "final_xy_error_m", "vertical_rmse_m", "final_vertical_error_m",
    "vxy_vec_rmse_mps", "speed_rmse_mps", "yaw/course_rmse_deg", "yaw/course_final_deg",
]
PHASE_METRICS = [
    "course_error_deg_median", "course_error_deg_p95", "course_error_deg_slope_per_s",
    "attitude_yaw_error_deg_median", "attitude_yaw_error_deg_slope_per_s",
    "position_error_xy_m_median", "position_error_xy_m_p95", "position_error_xy_m_slope_per_s",
    "err_U_median", "err_U_p95", "err_U_slope_per_s",
    "velocity_error_xy_mps_median", "velocity_error_xy_mps_p95", "velocity_error_xy_mps_slope_per_s",
]


def metric_delta_rows(
    global_start: list[dict[str, Any]],
    global_absolute: list[dict[str, Any]],
    phase_start: list[dict[str, Any]],
    phase_absolute: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    global_indexes = {
        "global_start_heading": {(r["condition"], r["flight"]): r for r in global_start},
        "global_absolute": {(r["condition"], r["flight"]): r for r in global_absolute},
    }
    phase_indexes = {
        "fixed_phase_start_heading": {(r["condition"], r["flight"], r["phase"]): r for r in phase_start},
        "fixed_phase_absolute": {(r["condition"], r["flight"], r["phase"]): r for r in phase_absolute},
    }
    rows: list[dict[str, Any]] = []
    for pair, candidate, baseline in PAIR_DEFINITIONS:
        if candidate not in experiment.CONDITIONS or baseline not in experiment.CONDITIONS:
            continue
        for flight in experiment.FLIGHTS:
            for scope, index in global_indexes.items():
                a, b = index.get((candidate, flight)), index.get((baseline, flight))
                if not (a and b):
                    continue
                for metric in GLOBAL_METRICS:
                    av, bv = number(a.get(metric)), number(b.get(metric))
                    rows.append({
                        "pair": pair, "candidate": candidate, "baseline": baseline,
                        "flight": flight, "scope": scope, "phase": "global",
                        "metric": metric, "candidate_value": av, "baseline_value": bv,
                        "signed_delta": av - bv,
                        "absolute_magnitude_delta": abs(av) - abs(bv),
                        "improved_by_abs_magnitude": abs(av) < abs(bv),
                    })
            for scope, index in phase_indexes.items():
                for phase in ["pre_turn", "turn", "post_straight_early", "post_straight_late"]:
                    a = index.get((candidate, flight, phase))
                    b = index.get((baseline, flight, phase))
                    if not (a and b):
                        continue
                    for metric in PHASE_METRICS:
                        av, bv = number(a.get(metric)), number(b.get(metric))
                        rows.append({
                            "pair": pair, "candidate": candidate, "baseline": baseline,
                            "flight": flight, "scope": scope, "phase": phase,
                            "metric": metric, "candidate_value": av, "baseline_value": bv,
                            "signed_delta": av - bv,
                            "absolute_magnitude_delta": abs(av) - abs(bv),
                            "improved_by_abs_magnitude": abs(av) < abs(bv),
                        })
    return rows


def factorial_rows(
    global_start: list[dict[str, Any]],
    global_absolute: list[dict[str, Any]],
    phase_start: list[dict[str, Any]],
    phase_absolute: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    if not all(cid in experiment.CONDITIONS for cid in ["C0", "KONLY", "DONLY", "KD"]):
        return []
    rows: list[dict[str, Any]] = []
    scopes = [
        ("global_start_heading", {(r["condition"], r["flight"]): r for r in global_start}, GLOBAL_METRICS, ["global"]),
        ("global_absolute", {(r["condition"], r["flight"]): r for r in global_absolute}, GLOBAL_METRICS, ["global"]),
        ("fixed_phase_start_heading", {(r["condition"], r["flight"], r["phase"]): r for r in phase_start}, PHASE_METRICS, ["pre_turn", "turn", "post_straight_early", "post_straight_late"]),
        ("fixed_phase_absolute", {(r["condition"], r["flight"], r["phase"]): r for r in phase_absolute}, PHASE_METRICS, ["pre_turn", "turn", "post_straight_early", "post_straight_late"]),
    ]
    for scope, index, metrics, phases in scopes:
        for flight in experiment.FLIGHTS:
            for phase in phases:
                key = (flight,) if phase == "global" else (flight, phase)
                values = {
                    cid: index.get((cid, *key))
                    for cid in ["C0", "KONLY", "DONLY", "KD"]
                }
                if not all(values.values()):
                    continue
                for metric in metrics:
                    c0 = number(values["C0"].get(metric))
                    k = number(values["KONLY"].get(metric))
                    d = number(values["DONLY"].get(metric))
                    kd = number(values["KD"].get(metric))
                    rows.append({
                        "flight": flight,
                        "scope": scope,
                        "phase": phase,
                        "metric": metric,
                        "C0": c0,
                        "KONLY": k,
                        "DONLY": d,
                        "KD": kd,
                        "K_main_at_current_D": k - c0,
                        "D_main_at_current_K": d - c0,
                        "K_effect_at_june_D": kd - d,
                        "D_effect_at_june_K": kd - k,
                        "KD_interaction": kd - k - d + c0,
                    })
    return rows


def focus_trigger(delta_rows: list[dict[str, Any]]) -> dict[str, Any]:
    thresholds = {
        "global_course_rmse_deg": 0.5,
        "global_xy_rmse_m": 20.0,
        "turn_or_post_course_median_deg": 0.75,
        "turn_or_post_course_slope_deg_s": 0.01,
    }
    selected: list[str] = []
    factors: dict[str, Any] = {}
    for condition, pair in [("KONLY", "K_main_at_current_D"), ("DONLY", "D_main_at_current_K")]:
        reasons = []
        for row in delta_rows:
            if row["pair"] != pair or row["scope"] != "global_start_heading":
                continue
            delta = abs(number(row["signed_delta"]))
            if row["metric"] == "yaw/course_rmse_deg" and delta >= thresholds["global_course_rmse_deg"]:
                reasons.append({"threshold": "global_course_rmse_deg", **row})
            if row["metric"] == "xy_rmse_m" and delta >= thresholds["global_xy_rmse_m"]:
                reasons.append({"threshold": "global_xy_rmse_m", **row})
        for row in delta_rows:
            if (
                row["pair"] != pair
                or row["scope"] != "fixed_phase_start_heading"
                or row["phase"] not in {"turn", "post_straight_late"}
            ):
                continue
            delta = abs(number(row["signed_delta"]))
            if row["metric"] == "course_error_deg_median" and delta >= thresholds["turn_or_post_course_median_deg"]:
                reasons.append({"threshold": "turn_or_post_course_median_deg", **row})
            if row["metric"] == "course_error_deg_slope_per_s" and delta >= thresholds["turn_or_post_course_slope_deg_s"]:
                reasons.append({"threshold": "turn_or_post_course_slope_deg_s", **row})
        triggered = bool(reasons)
        factors[condition] = {"triggered": triggered, "reasons": reasons}
        if triggered:
            selected.append(condition)
    return {
        "schema": "openvins_p4_kd_focus_to_full_trigger_v1",
        "evaluated_at": now_iso(),
        "thresholds": thresholds,
        "factors": factors,
        "selected_full_conditions": ["C0", *selected],
        "full_confirmation_required": bool(selected),
        "selection_is_offline_only": True,
        "online_gate_or_threshold_changed": False,
    }


def runtime_rows() -> list[dict[str, Any]]:
    path = ROOT / "RUN_STATUS.csv"
    return read_rows(path) if path.exists() else []


def summarize() -> None:
    global_start: list[dict[str, Any]] = []
    global_absolute: list[dict[str, Any]] = []
    phase_start: list[dict[str, Any]] = []
    phase_absolute: list[dict[str, Any]] = []
    fingerprints: dict[str, dict[str, Any]] = {}
    fingerprint_rows: list[dict[str, Any]] = []
    schemas: dict[str, dict[str, Any]] = {}

    for condition in experiment.CONDITIONS:
        for flight in experiment.FLIGHTS:
            run_id = f"{condition}_{flight}"
            for root_name, target in [
                ("full_flight_start_heading", global_start),
                ("full_flight_absolute", global_absolute),
            ]:
                path = ANALYSIS_ROOT / root_name / run_id / "tables/global_summary.csv"
                if path.exists():
                    target.append({"run_id": run_id, "condition": condition, "flight": flight, **read_one(path)})
            for root_name, target in [
                ("phase_root_cause", phase_start),
                ("phase_root_cause_absolute", phase_absolute),
            ]:
                path = ANALYSIS_ROOT / root_name / run_id / "phase_summary.csv"
                if path.exists():
                    for row in read_rows(path):
                        if row.get("method") == "active":
                            target.append({"run_id": run_id, "condition": condition, "flight": flight, **row})
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
                    "position_nis": fp.get("candidate_last_position_nis"),
                    "velocity_nis": fp.get("candidate_last_velocity_nis"),
                    "visual_compensated_p95_px": fp.get("candidate_visual_compensated_p95_px"),
                    "support_counts": json.dumps(fp.get("candidate_group_supported_update_counts")),
                    "state_json": json.dumps(fp.get("state")),
                })
            validation = ROOT / f"runs/{run_id}/run_validation.json"
            if validation.exists():
                value = json.loads(validation.read_text(encoding="utf-8"))
                schemas[run_id] = {
                    name: item.get("columns", [])
                    for name, item in value.get("diagnostic_schemas", {}).items()
                }

    write_csv(SUMMARY_ROOT / "global_metrics_start_heading.csv", global_start)
    write_csv(SUMMARY_ROOT / "global_metrics_absolute.csv", global_absolute)
    write_csv(SUMMARY_ROOT / "phase_metrics_start_heading.csv", phase_start)
    write_csv(SUMMARY_ROOT / "phase_metrics_absolute.csv", phase_absolute)
    write_csv(SUMMARY_ROOT / "p4_release_fingerprints.csv", fingerprint_rows)
    init = initialization_rows(fingerprints)
    write_csv(SUMMARY_ROOT / "initialization_state_deltas_vs_C0.csv", init)
    deltas = metric_delta_rows(global_start, global_absolute, phase_start, phase_absolute)
    write_csv(SUMMARY_ROOT / "factor_pair_deltas.csv", deltas)
    factorial = factorial_rows(global_start, global_absolute, phase_start, phase_absolute)
    write_csv(SUMMARY_ROOT / "factorial_effects_and_interaction.csv", factorial)
    write_csv(SUMMARY_ROOT / "runtime_and_failures.csv", runtime_rows())

    schema_names = sorted({name for run in schemas.values() for name in run})
    schema_audit: dict[str, Any] = {"all_identical": True, "files": {}}
    for name in schema_names:
        values = {run_id: files.get(name) for run_id, files in schemas.items()}
        identical = len({json.dumps(value) for value in values.values()}) == 1
        schema_audit["files"][name] = {"identical": identical, "runs": values}
        schema_audit["all_identical"] &= identical
    write_json(SUMMARY_ROOT / "diagnostic_schema_comparison.json", schema_audit)

    trigger: dict[str, Any] | None = None
    if experiment.SCOPE == "focus":
        trigger = focus_trigger(deltas)
        write_json(SUMMARY_ROOT / "FOCUS_TRIGGER_DECISION.json", trigger)
        write_csv(
            SUMMARY_ROOT / "FULL_CONFIRMATION_SELECTION.csv",
            [
                {"condition": cid, "selected": cid in trigger["selected_full_conditions"]}
                for cid in ["C0", "KONLY", "DONLY", "KD"]
            ],
        )

    status = runtime_rows()
    completed = [row["run_id"] for row in status if row.get("status") == "success"]
    failed = [row["run_id"] for row in status if row.get("status") != "success"]
    config_rows = read_rows(ROOT / "ACTUAL_CONFIG_AUDIT.csv")
    config_all_pass = bool(config_rows) and all(row.get("pass", "").lower() == "true" for row in config_rows)
    geometry = json.loads(
        (ROOT / "provenance/CALIBRATION_GEOMETRY_AUDIT.json").read_text(encoding="utf-8")
    )
    gps_headers = {}
    for flight, info in experiment.FLIGHTS.items():
        with evaluation_gps(info).open("r", encoding="utf-8-sig") as stream:
            gps_headers[flight] = stream.readline().strip()
    velocity_contract = all(
        all(column in header for column in ["Ve", "Vn", "Vu"])
        for header in gps_headers.values()
    )

    key = [
        row for row in deltas
        if row["pair"] in {"K_main_at_current_D", "D_main_at_current_K", "KD_total"}
        and (
            (row["scope"] == "global_start_heading" and row["metric"] in {"yaw/course_rmse_deg", "xy_rmse_m", "vertical_rmse_m", "vxy_vec_rmse_mps"})
            or (row["scope"] == "fixed_phase_start_heading" and row["phase"] in {"turn", "post_straight_late"} and row["metric"] in {"course_error_deg_median", "course_error_deg_slope_per_s", "position_error_xy_m_slope_per_s"})
        )
    ]
    table = [
        "| Pair | Flight | Scope | Phase | Metric | Candidate | C0/baseline | Signed delta |",
        "|---|---|---|---|---|---:|---:|---:|",
    ]
    for row in key:
        table.append(
            f"| {row['pair']} | {row['flight']} | {row['scope']} | {row['phase']} | "
            f"{row['metric']} | {number(row['candidate_value']):.6g} | "
            f"{number(row['baseline_value']):.6g} | {number(row['signed_delta']):+.6g} |"
        )
    trigger_text = ""
    if trigger:
        trigger_text = (
            f"- Full confirmation required: `{trigger['full_confirmation_required']}`\n"
            f"- Selected conditions: `{', '.join(trigger['selected_full_conditions'])}`\n"
        )
    report = f"""# K/D split {experiment.SCOPE} results

Generated at `{now_iso()}` from the frozen runner and frozen canonical analysis
snapshot. GPS update times are the statistics grid. Start-heading preserves
scale; absolute navigation has no post alignment. No best-fit result is used.

## Completeness and contracts

- Completed/validated runs ({len(completed)}): {', '.join(completed) if completed else 'none'}
- Failed/incomplete runs ({len(failed)}): {', '.join(failed) if failed else 'none'}
- Actual config snapshots all pass: `{config_all_pass}`
- Diagnostic schemas identical: `{schema_audit['all_identical']}`
- Rich GPS has FC `Ve,Vn,Vu` for both flights: `{velocity_contract}`
- June12 resolution/model/order/parser audit: `{'PASS' if geometry.get('pass') else 'FAIL'}`
{trigger_text}
## Key factor deltas

Signed deltas are candidate minus its clean baseline. Error magnitudes still
require sign/context checks in `factor_pair_deltas.csv`.

{chr(10).join(table)}

## Evidence map

- `global_metrics_start_heading.csv` and `global_metrics_absolute.csv`: XY, Z,
  course, speed, and FC-vector velocity metrics on the GPS grid.
- `phase_metrics_start_heading.csv` and `phase_metrics_absolute.csv`: fixed
  pre-turn/turn/post-straight medians, p95 values, and slopes.
- `factor_pair_deltas.csv`: K-only, D-only, KD, and conditional effects.
- `factorial_effects_and_interaction.csv`: `KD-K-D+C0` interaction for every
  available primary/phase metric.
- `initialization_state_deltas_vs_C0.csv`: release-window identity and
  q/p/v/bg/ba changes, including quaternion angle and ZYX diagnostics.
- `p4_release_fingerprints.csv`: P4 release state, NIS, support, and fingerprint.
- Per-run canonical curves/CSVs are under `analysis/full_flight_*`; standard
  `flight_eval_tool.py` outputs and cross-condition comparisons are under
  `analysis/flight_eval_tool` and `analysis/flight_eval_compare`.
"""
    (SUMMARY_ROOT / "RESULTS.md").write_text(report, encoding="utf-8", newline="\n")


def cross_scope_finalize() -> None:
    focus = experiment.SPLIT_ROOT_WIN / "summary"
    full = experiment.SPLIT_ROOT_WIN / "full_confirmation/summary"
    payload = {
        "generated_at": now_iso(),
        "focus_results": str(focus / "RESULTS.md"),
        "focus_trigger": json.loads((focus / "FOCUS_TRIGGER_DECISION.json").read_text(encoding="utf-8")),
        "full_results": str(full / "RESULTS.md") if (full / "RESULTS.md").exists() else None,
        "full_available": (full / "RESULTS.md").exists(),
    }
    write_json(experiment.SPLIT_ROOT_WIN / "CROSS_SCOPE_EVIDENCE_INDEX.json", payload)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--summarize-only", action="store_true")
    parser.add_argument("--cross-scope-finalize", action="store_true")
    args = parser.parse_args()
    if args.cross_scope_finalize:
        cross_scope_finalize()
        return 0
    if args.summarize_only:
        summarize()
        return 0
    return run_analysis_batch(args.max_workers)


if __name__ == "__main__":
    raise SystemExit(main())
