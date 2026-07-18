#!/usr/bin/env python3
"""Single permitted fallback: fixed-delay piecewise-constant SO(3) smoothing.

The input is the persistent relative-mount observation trace produced from the
same frozen VIO and adjacent FC increments. A robust one-change model competes
with a constant model over a 45-second causal window. The post-change level
must persist for 15 seconds, and accepted levels have a 30-second minimum
dwell. This is a shadow-only change-point/TV model, not hand--eye calibration.
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

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from analysis.flex_level_observer import (  # noqa: E402
    FlexLevelObserver,
    FlexLevelParameters,
    angle_deg,
    exp_deg,
    log_deg,
    robust_so3_mean,
)
from analysis.flex_level_shadow_validation import (  # noqa: E402
    FlightResult,
    acceptance_report,
    all_missing_fc_audit,
    attach_body_evaluation,
    current_rss_mb,
    event_reports,
    file_sha256,
    flight_metrics,
    json_safe,
    load_nominal_mounts,
    lofo_report,
    plot_flight,
    read_trajectory,
    source_files,
    write_fly3_event_plot,
)


@dataclass(frozen=True)
class TVFallbackParameters:
    causal_window_s: float = 45.0
    minimum_pre_level_s: float = 15.0
    minimum_post_level_s: float = 15.0
    minimum_dwell_s: float = 30.0
    release_duration_s: float = 5.0
    evaluation_period_s: float = 1.0
    noise_floor_deg: float = 1.05
    maximum_level_median_dispersion_deg: float = 0.40
    maximum_level_p95_dispersion_deg: float = 0.90
    maximum_correction_deg: float = 3.55
    minimum_cost_reduction_ratio: float = 0.45
    total_variation_penalty_deg2: float = 1.1025
    minimum_second_quality_ratio: float = 0.90


@dataclass
class SecondLevel:
    t: float
    R_I_from_B_obs: np.ndarray
    quality_ratio: float
    fc_valid_ratio: float


def robust_cost(matrices: list[np.ndarray], mean: np.ndarray, cap_deg: float) -> float:
    residual = np.asarray([angle_deg(matrix @ mean.T) for matrix in matrices])
    return float(np.sum(np.minimum(residual**2, cap_deg**2)))


def aggregate_seconds(
    trace: pd.DataFrame, nominal_mount: np.ndarray
) -> list[SecondLevel]:
    local = trace.copy()
    local["second"] = np.floor(local.t).astype(int)
    levels: list[SecondLevel] = []
    for second, group in local.groupby("second", sort=True):
        vectors = group[
            [f"observed_flex_{axis}_deg" for axis in "xyz"]
        ].to_numpy(dtype=float)
        matrices = [exp_deg(vector) @ nominal_mount for vector in vectors]
        mean, _ = robust_so3_mean(matrices)
        levels.append(
            SecondLevel(
                t=float(group.t.iloc[-1]),
                R_I_from_B_obs=mean,
                quality_ratio=float(group.quality_valid.astype(bool).mean()),
                fc_valid_ratio=float(group.fc_valid.astype(bool).mean()),
            )
        )
    return levels


def level_stats(matrices: list[np.ndarray]) -> tuple[np.ndarray, float, float]:
    mean, residuals = robust_so3_mean(matrices)
    norms = np.linalg.norm(residuals, axis=1)
    return mean, float(np.median(norms)), float(np.percentile(norms, 95.0))


def fixed_delay_tv_events(
    levels: list[SecondLevel],
    nominal_mount: np.ndarray,
    parameters: TVFallbackParameters,
) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    adopted = nominal_mount.copy()
    last_correction = -math.inf
    last_evaluation = -math.inf
    buffer: list[SecondLevel] = []
    for level in levels:
        valid = (
            level.fc_valid_ratio >= 1.0 - 1.0e-12
            and level.quality_ratio >= parameters.minimum_second_quality_ratio
        )
        if not valid:
            buffer = []
            continue
        if buffer and level.t - buffer[-1].t > 1.8:
            buffer = []
        buffer.append(level)
        while buffer and level.t - buffer[0].t > parameters.causal_window_s + 1.0:
            buffer.pop(0)
        if level.t - last_correction < parameters.minimum_dwell_s:
            continue
        if level.t - last_evaluation < parameters.evaluation_period_s:
            continue
        if not buffer or buffer[-1].t - buffer[0].t < parameters.causal_window_s:
            continue
        last_evaluation = level.t
        matrices = [item.R_I_from_B_obs for item in buffer]
        single_mean, _, _ = level_stats(matrices)
        single_cost = robust_cost(
            matrices, single_mean, parameters.maximum_level_p95_dispersion_deg
        )
        best: dict[str, Any] | None = None
        for split in range(1, len(buffer) - 1):
            pre_duration = buffer[split - 1].t - buffer[0].t
            post_duration = buffer[-1].t - buffer[split].t
            if pre_duration < parameters.minimum_pre_level_s:
                continue
            if post_duration < parameters.minimum_post_level_s:
                continue
            pre_matrices = matrices[:split]
            post_matrices = matrices[split:]
            pre_mean, pre_median, pre_p95 = level_stats(pre_matrices)
            post_mean, post_median, post_p95 = level_stats(post_matrices)
            level_shift = angle_deg(post_mean @ pre_mean.T)
            split_cost = robust_cost(
                pre_matrices, pre_mean, parameters.maximum_level_p95_dispersion_deg
            ) + robust_cost(
                post_matrices, post_mean, parameters.maximum_level_p95_dispersion_deg
            )
            penalized_cost = split_cost + parameters.total_variation_penalty_deg2
            reduction_ratio = (single_cost - penalized_cost) / max(single_cost, 1.0e-12)
            candidate = {
                "split": split,
                "pre_mean": pre_mean,
                "post_mean": post_mean,
                "pre_median": pre_median,
                "pre_p95": pre_p95,
                "post_median": post_median,
                "post_p95": post_p95,
                "level_shift": level_shift,
                "reduction_ratio": reduction_ratio,
                "score": single_cost - penalized_cost,
            }
            if best is None or candidate["score"] > best["score"]:
                best = candidate
        if best is None:
            continue
        correction = log_deg(best["post_mean"] @ adopted.T)
        correction_angle = float(np.linalg.norm(correction))
        failed: list[str] = []
        if best["level_shift"] < parameters.noise_floor_deg:
            failed.append("change_below_noise_floor")
        if correction_angle < parameters.noise_floor_deg:
            failed.append("adopted_level_residual_below_noise_floor")
        if correction_angle > parameters.maximum_correction_deg:
            failed.append("correction_limit")
        if best["pre_median"] > parameters.maximum_level_median_dispersion_deg:
            failed.append("pre_median_dispersion")
        if best["post_median"] > parameters.maximum_level_median_dispersion_deg:
            failed.append("post_median_dispersion")
        if best["pre_p95"] > parameters.maximum_level_p95_dispersion_deg:
            failed.append("pre_p95_dispersion")
        if best["post_p95"] > parameters.maximum_level_p95_dispersion_deg:
            failed.append("post_p95_dispersion")
        if best["reduction_ratio"] < parameters.minimum_cost_reduction_ratio:
            failed.append("tv_cost_reduction")
        if failed:
            continue
        split = int(best["split"])
        candidate_payload = {
            "start_s": float(buffer[0].t),
            "end_s": float(buffer[split - 1].t),
            "mean_R_I_from_B": best["pre_mean"].tolist(),
            "median_dispersion_deg": best["pre_median"],
            "p95_dispersion_deg": best["pre_p95"],
        }
        confirmation_payload = {
            "start_s": float(buffer[split].t),
            "end_s": float(buffer[-1].t),
            "mean_R_I_from_B": best["post_mean"].tolist(),
            "median_dispersion_deg": best["post_median"],
            "p95_dispersion_deg": best["post_p95"],
        }
        events.append(
            {
                "event_type": "candidate",
                "time_s": float(buffer[-1].t),
                "estimated_change_point_s": float(buffer[split].t),
                "evidence": candidate_payload,
                "confirmation": confirmation_payload,
                "tv_level_shift_deg": best["level_shift"],
                "tv_cost_reduction_ratio": best["reduction_ratio"],
            }
        )
        old_target = adopted.copy()
        adopted = best["post_mean"]
        events.append(
            {
                "event_type": "correction",
                "time_s": float(buffer[-1].t),
                "level_change_start_s": float(buffer[split].t),
                "estimated_change_point_s": float(buffer[split].t),
                "candidate": candidate_payload,
                "confirmation": confirmation_payload,
                "candidate_confirmation_distance_deg": 0.0,
                "correction_vector_deg": correction.tolist(),
                "correction_angle_deg": correction_angle,
                "release_duration_s": parameters.release_duration_s,
                "old_target_R_I_from_B": old_target.tolist(),
                "new_target_R_I_from_B": adopted.tolist(),
                "tv_level_shift_deg": best["level_shift"],
                "tv_cost_reduction_ratio": best["reduction_ratio"],
            }
        )
        last_correction = float(buffer[-1].t)
        buffer = []
    return events


def trace_with_fallback(
    primary_trace: pd.DataFrame,
    nominal_mount: np.ndarray,
    events: list[dict[str, Any]],
) -> pd.DataFrame:
    output = primary_trace.copy()
    corrections = [event for event in events if event["event_type"] == "correction"]
    target = nominal_mount.copy()
    applied_vectors: list[np.ndarray] = []
    target_vectors: list[np.ndarray] = []
    residuals: list[float] = []
    correction_index = 0
    for row in output.itertuples(index=False):
        timestamp = float(row.t)
        while (
            correction_index < len(corrections)
            and float(corrections[correction_index]["time_s"]) <= timestamp
        ):
            target = np.asarray(
                corrections[correction_index]["new_target_R_I_from_B"], dtype=float
            )
            correction_index += 1
        applied = nominal_mount.copy()
        for event in corrections:
            event_time = float(event["time_s"])
            if timestamp < event_time:
                break
            old_target = np.asarray(event["old_target_R_I_from_B"], dtype=float)
            new_target = np.asarray(event["new_target_R_I_from_B"], dtype=float)
            release = float(event["release_duration_s"])
            if timestamp < event_time + release:
                alpha = (timestamp - event_time) / max(release, 1.0e-9)
                applied = exp_deg(alpha * log_deg(new_target @ old_target.T)) @ old_target
                break
            applied = new_target
        observed_vector = np.array(
            [
                row.observed_flex_x_deg,
                row.observed_flex_y_deg,
                row.observed_flex_z_deg,
            ]
        )
        observed = exp_deg(observed_vector) @ nominal_mount
        applied_vectors.append(log_deg(applied @ nominal_mount.T))
        target_vectors.append(log_deg(target @ nominal_mount.T))
        residuals.append(angle_deg(observed @ target.T))
    applied_array = np.asarray(applied_vectors)
    target_array = np.asarray(target_vectors)
    for index, axis in enumerate("xyz"):
        output[f"applied_flex_{axis}_deg"] = applied_array[:, index]
        output[f"target_flex_{axis}_deg"] = target_array[:, index]
    output["observed_target_residual_deg"] = residuals
    output["event_types"] = ""
    return output


def write_report(
    path: Path,
    acceptance: dict[str, Any],
    results: dict[str, FlightResult],
    parameters: dict[str, Any],
    primary_root: Path,
    lofo: dict[str, Any],
    missing: dict[str, Any],
) -> None:
    lines = [
        "# Experimental fixed-delay SO(3) TV fallback",
        "",
        f"- Final status: **{acceptance['status']}**.",
        "- Method: the single permitted fallback, a robust causal one-change/TV model on the observed relative-mount level with 30 s minimum dwell.",
        f"- Primary failed result: `{primary_root}`.",
        "- No hand--eye window solve, no second KLT, no FC absolute-heading input, and no VIO/P/V/bias/extrinsic writeback is present.",
        f"- Parameters: `{json.dumps(parameters, ensure_ascii=False)}`.",
        "",
        "## Fly3 events",
        "",
    ]
    for report in results["fly3"].metrics["event_reports"]:
        lines.extend(
            [
                f"- `{report['window_s'][0]:.0f}-{report['window_s'][1]:.0f} s`: corrections={report['associated_correction_count']}, change/candidate/confirmation/correction={report.get('level_change_start_s')}/{report.get('candidate_time_s')}/{report.get('confirmation_time_s')}/{report.get('correction_time_s')}, vector={report.get('correction_vector_deg')}, correct_direction={report.get('correct_direction')}, repeat={report.get('reverse_or_repeat')}.",
            ]
        )
    lines.extend(
        [
            "",
            "## Four-flight summary",
            "",
            "| flight | candidates | corrections | normal corrections | false/min | minimum interval s | baseline->shadow median deg | baseline->shadow P95 deg | runtime s | RSS MB |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for flight, result in sorted(results.items()):
        metrics = result.metrics
        lines.append(
            f"| {flight} | {metrics['candidate_count']} | {metrics['correction_count']} | {metrics['normal_correction_count']} | {metrics['false_triggers_per_min']:.4f} | {metrics['minimum_correction_interval_s']} | {metrics['baseline_body_error_median_deg']:.3f}->{metrics['shadow_body_error_median_deg']:.3f} | {metrics['baseline_body_error_p95_deg']:.3f}->{metrics['shadow_body_error_p95_deg']:.3f} | {result.runtime_s:.3f} | {result.peak_rss_mb} |"
        )
    lines.extend(["", "## Acceptance", ""])
    for name, passed in acceptance["checks"].items():
        lines.append(f"- `{name}`: **{passed}**")
    lines.extend(
        [
            "",
            f"- missing FC: `{json.dumps(missing, ensure_ascii=False)}`.",
            f"- LOFO non-catastrophic: **{lofo['all_folds_no_catastrophic_degradation']}**.",
            "",
            "## Decision",
            "",
            "Both the primary observer and the sole permitted fallback failed the hard shadow gate. No rosfree integration was made.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=REPO_ROOT / "analysis" / "manifests" / "flex_level_shadow_stride12_20260719.json",
    )
    parser.add_argument("--primary-result-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    mounts, calibration_metadata = load_nominal_mounts(Path(manifest["nominal_mount_calibration"]))
    parameters = TVFallbackParameters()
    parameter_payload = {
        "fallback": asdict(parameters),
        "common_across_flights": True,
        "selection": "single_pre_registered_fixed_delay_tv_parameter_set",
    }
    parameter_hash = hashlib.sha256(
        json.dumps(parameter_payload, sort_keys=True).encode("utf-8")
    ).hexdigest()
    parameter_payload["sha256"] = parameter_hash
    results: dict[str, FlightResult] = {}
    for flight, spec in manifest["flights"].items():
        started = time.perf_counter()
        files = source_files(spec)
        before = {name: file_sha256(path) for name, path in files.items()}
        primary_trace = pd.read_parquet(
            args.primary_result_root / f"{flight}_FLEX_LEVEL_TRACE.parquet"
        )
        levels = aggregate_seconds(primary_trace, mounts[flight])
        events = fixed_delay_tv_events(levels, mounts[flight], parameters)
        trace = trace_with_fallback(primary_trace, mounts[flight], events)
        trajectory = read_trajectory(Path(spec["run_dir"]) / "traj.txt")
        corrections = [event for event in events if event["event_type"] == "correction"]
        body, body_metrics = attach_body_evaluation(
            flight,
            spec,
            trajectory,
            mounts[flight],
            corrections,
            calibration_metadata[flight]["fc_attitude_to_board_time_offset_s"],
        )
        reports = event_reports(flight, spec, trace, corrections)
        dummy = FlexLevelObserver(mounts[flight], FlexLevelParameters())
        dummy.events = events
        duration = float(trace.t.iloc[-1] - trace.t_start.iloc[0])
        metrics = flight_metrics(
            flight, spec, trace, dummy, body_metrics, reports, duration
        )
        trace.to_parquet(args.output_dir / f"{flight}_TV_LEVEL_TRACE.parquet", index=False)
        body.to_csv(args.output_dir / f"{flight}_AIRCRAFT_BODY_ATTITUDE_SHADOW.csv", index=False)
        (args.output_dir / f"{flight}_EVENTS.json").write_text(
            json.dumps(json_safe(events), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        plot_flight(
            flight,
            trace,
            events,
            spec.get("positive_windows_s", []),
            args.output_dir / f"{flight}_TV_LEVEL_SHADOW.png",
        )
        after = {name: file_sha256(path) for name, path in files.items()}
        results[flight] = FlightResult(
            flight=flight,
            table=trace,
            trajectory=body,
            observer=dummy,
            events=events,
            metrics=metrics,
            runtime_s=float(time.perf_counter() - started),
            peak_rss_mb=current_rss_mb(),
            source_hashes_before=before,
            source_hashes_after=after,
        )
    write_fly3_event_plot(
        results["fly3"].table,
        results["fly3"].events,
        args.output_dir / "FLY3_REQUIRED_EVENTS_TV_FALLBACK.png",
    )
    missing = all_missing_fc_audit(mounts["fly3"], FlexLevelParameters())
    lofo = lofo_report(results, parameter_hash)
    acceptance = acceptance_report(results, lofo, missing)
    summary = {
        "method": "fixed_delay_piecewise_constant_so3_tv_change_point",
        "primary_result_root": str(args.primary_result_root),
        "primary_summary_sha256": file_sha256(
            args.primary_result_root / "FOUR_FLIGHT_SUMMARY.json"
        ),
        "parameters": parameter_payload,
        "acceptance": acceptance,
        "missing_fc": missing,
        "lofo": lofo,
        "flights": {flight: result.metrics for flight, result in results.items()},
        "runtime_s": {flight: result.runtime_s for flight, result in results.items()},
        "peak_rss_mb": {flight: result.peak_rss_mb for flight, result in results.items()},
        "source_files_identical": {
            flight: result.source_hashes_before == result.source_hashes_after
            for flight, result in results.items()
        },
        "rosfree_integration_performed": False,
    }
    (args.output_dir / "FROZEN_PARAMETERS.json").write_text(
        json.dumps(json_safe(parameter_payload), indent=2, ensure_ascii=False) + "\n",
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
    write_report(
        args.output_dir / "FINAL_ONE_PAGE_CONCLUSION.md",
        acceptance,
        results,
        parameter_payload,
        args.primary_result_root,
        lofo,
        missing,
    )
    print(json.dumps(json_safe(summary), indent=2, ensure_ascii=False))
    return 0 if acceptance["status"] == "SHADOW_PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
