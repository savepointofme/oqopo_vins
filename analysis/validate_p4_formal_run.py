#!/usr/bin/env python3
"""Validate one completed formal P4 ROS-free replay artifact set."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def finite_number(value: object) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--command")
    parser.add_argument("--allow-p5-active", action="store_true")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    metadata_path = Path(args.metadata)
    trajectory_path = Path(args.trajectory)
    failures: list[str] = []
    require(metadata_path.is_file(), "metadata missing", failures)
    require(trajectory_path.is_file(), "trajectory missing", failures)
    if failures:
        metadata: dict[str, object] = {}
    else:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    require(
        metadata.get("schema") == "openvins_online_multisensor_alignment_v15",
        "unexpected metadata schema",
        failures,
    )
    require(
        metadata.get("algorithm_id")
        == "openvins_p4_fc_yaw_pv_per_keyframe_bias_reprojection_v4",
        "unexpected formal P4 algorithm id",
        failures,
    )
    require(bool(metadata.get("released_to_openvins")), "P4 did not release", failures)
    require(
        metadata.get("status") in {"NAVIGATION_READY", "FULL_ALIGNMENT_READY"},
        "P4 release status is not navigation-ready",
        failures,
    )
    require(metadata.get("successful_release_count") == 1, "release is not unique", failures)
    require(bool(metadata.get("alignment_window_closed")), "alignment window stayed open", failures)
    require(
        metadata.get("shared_window_bias_model") is False,
        "bg/ba are not per-keyframe states",
        failures,
    )
    require(not bool(metadata.get("staged_solver_enabled")), "staged solver is enabled", failures)
    require(
        not bool(metadata.get("handoff_covariance_inflation_applied")),
        "formal graph posterior was altered by handoff inflation",
        failures,
    )
    require(
        metadata.get("handoff_covariance_model")
        == "formal_joint_terminal_schur_marginal",
        "unexpected formal covariance handoff model",
        failures,
    )
    raw_std = np.asarray(metadata.get("handoff_raw_covariance_std", []), dtype=float)
    applied_std = np.asarray(
        metadata.get("handoff_applied_covariance_std", []), dtype=float
    )
    require(
        raw_std.shape == (15,) and applied_std.shape == (15,),
        "handoff covariance std vectors are not 15-D",
        failures,
    )
    if raw_std.shape == (15,) and applied_std.shape == (15,):
        require(
            np.allclose(applied_std, raw_std, rtol=1.0e-9, atol=1.0e-12),
            "formal handoff did not preserve the terminal Schur marginal",
            failures,
        )
    require(
        abs(float(metadata.get("selected_window_duration_s", 0.0)) - 8.0) <= 1.0e-9,
        "selected formal window is not 8 seconds",
        failures,
    )
    direct_sliding_release = bool(metadata.get("direct_sliding_state_release"))
    if direct_sliding_release:
        require(
            int(metadata.get("nonlinear_solve_attempt_count", 0)) >= 2,
            "direct release was not supported by multiple advanced joint windows",
            failures,
        )
        require(
            bool(metadata.get("sliding_overlap_consistency_passed")),
            "cross-window state consistency failed",
            failures,
        )
        require(
            float(metadata.get("sliding_overlap_stable_duration_s", 0.0)) >= 2.0,
            "cross-window consistency is shorter than 2 seconds",
            failures,
        )
        require(
            bool(metadata.get("sliding_bias_trend_passed")),
            "shared bg/ba trend did not stabilize",
            failures,
        )
        require(
            metadata.get("candidate_refinement_count") == 0,
            "direct sliding release unexpectedly used candidate refinement",
            failures,
        )
    else:
        require(bool(metadata.get("formal_candidate_holdout_passed")), "causal holdout failed", failures)
        require(
            float(metadata.get("formal_candidate_holdout_duration_s", 0.0)) >= 2.0,
            "causal holdout is shorter than 2 seconds",
            failures,
        )
        require(
            not bool(metadata.get("formal_refinement_release")),
            "an unverified post-holdout refinement was released",
            failures,
        )
        require(
            metadata.get("candidate_refinement_count") == 0,
            "formal lifecycle unexpectedly used candidate refinement",
            failures,
        )
    require(
        metadata.get("fc_position_velocity_weight_model")
        == "terminal_yaw_gauge_plus_dense_pv_terminal_and_correlated_increments",
        "unexpected FC trajectory covariance model",
        failures,
    )

    keyframes = int(metadata.get("fc_position_velocity_factor_count", 0))
    require(keyframes >= 2, "fewer than two keyframes", failures)
    factors = {
        factor.get("family"): factor
        for factor in metadata.get("factor_contributions", [])
        if isinstance(factor, dict)
    }
    expected = {
        "prior": (3, 9),
        "imu_preintegration": (keyframes - 1, 15 * (keyframes - 1)),
        "visual_reprojection": (None, None),
        "fc_pose_velocity_attitude": (2, 1 + 6 * keyframes),
    }
    for family, (blocks, dimension) in expected.items():
        factor = factors.get(family)
        require(factor is not None, f"missing factor family: {family}", failures)
        if factor is None:
            continue
        if blocks is not None:
            require(factor.get("residual_blocks") == blocks, f"wrong block count: {family}", failures)
        if dimension is not None:
            require(factor.get("residual_dimension") == dimension, f"wrong residual dimension: {family}", failures)
        require(finite_number(factor.get("residual_rms")), f"non-finite residual RMS: {family}", failures)
        require(
            finite_number(factor.get("jacobian_frobenius"))
            and float(factor["jacobian_frobenius"]) > 0.0,
            f"missing Jacobian influence: {family}",
            failures,
        )
    visual = factors.get("visual_reprojection", {})
    require(int(visual.get("residual_blocks", 0)) >= 60, "fewer than 60 visual factors", failures)
    require(
        visual.get("residual_dimension") == 2 * visual.get("residual_blocks", 0),
        "formal image reprojection factors are not two-dimensional",
        failures,
    )
    require(
        finite_number(metadata.get("fc_terminal_attitude_residual_deg"))
        and float(metadata["fc_terminal_attitude_residual_deg"]) <= 5.0,
        "terminal FC attitude residual exceeds the formal handoff bound",
        failures,
    )
    require(
        finite_number(metadata.get("fc_terminal_position_residual_m"))
        and float(metadata["fc_terminal_position_residual_m"]) <= 3.5,
        "terminal FC position residual exceeds the formal handoff bound",
        failures,
    )
    require(
        finite_number(metadata.get("fc_terminal_velocity_residual_mps"))
        and float(metadata["fc_terminal_velocity_residual_mps"]) <= 0.75,
        "terminal FC velocity residual exceeds the formal handoff bound",
        failures,
    )
    require(
        finite_number(metadata.get("fc_window_attitude_max_residual_deg"))
        and float(metadata["fc_window_attitude_max_residual_deg"]) <= 10.0,
        "joint window contains an FC-board attitude transient",
        failures,
    )
    require(
        finite_number(metadata.get("rate_residual_max_rad_s"))
        and float(metadata["rate_residual_max_rad_s"]) <= 3.5,
        "joint window contains an unmatched FC/board-IMU angular-rate spike",
        failures,
    )
    require(
        finite_number(metadata.get("final_stage_solve_wall_time_s"))
        and float(metadata["final_stage_solve_wall_time_s"]) <= 2.0,
        "formal Ceres solve exceeded 2 seconds",
        failures,
    )

    covariance = np.asarray(metadata.get("covariance_15x15", []), dtype=float)
    require(covariance.shape == (15, 15), "release covariance is not 15x15", failures)
    if covariance.shape == (15, 15):
        require(np.isfinite(covariance).all(), "release covariance is non-finite", failures)
        require(
            float(np.max(np.abs(covariance - covariance.T))) <= 1.0e-8,
            "release covariance is not symmetric",
            failures,
        )
        eigenvalues = np.linalg.eigvalsh(0.5 * (covariance + covariance.T))
        require(float(eigenvalues.min()) >= -1.0e-9, "release covariance is not PSD", failures)

    observability = {
        state.get("state"): state
        for state in metadata.get("state_observability", [])
        if isinstance(state, dict)
    }
    for state_name in ("attitude", "position", "velocity", "gyro_bias", "accelerometer_bias"):
        state = observability.get(state_name, {})
        require(bool(state.get("observable")), f"state is not observable: {state_name}", failures)
        require(
            state.get("estimate_status") == "estimated_current_data",
            f"state is not supported by current data: {state_name}",
            failures,
        )
    require(
        metadata.get("startup_misalignment_status") == "fixed_external_calibration",
        "mounting calibration was modified online",
        failures,
    )
    require(
        metadata.get("time_offset_status") == "fixed_external_calibration",
        "FC attitude time calibration was modified online",
        failures,
    )
    for key in ("future_data_used", "post_alignment_used", "gps_xy_or_course_used_by_alignment", "old_initializer_fallback_used"):
        require(not bool(metadata.get(key)), f"forbidden path used: {key}", failures)

    timestamps: list[float] = []
    if trajectory_path.is_file():
        for line in trajectory_path.read_text(encoding="utf-8", errors="replace").splitlines():
            fields = line.strip().split()
            if not fields:
                continue
            try:
                timestamps.append(float(fields[0]))
            except ValueError:
                continue
    require(len(timestamps) >= 10, "trajectory has fewer than ten states", failures)
    require(all(math.isfinite(t) for t in timestamps), "trajectory timestamp is non-finite", failures)
    require(
        all(later > earlier for earlier, later in zip(timestamps, timestamps[1:])),
        "trajectory timestamps are not strictly increasing",
        failures,
    )

    if args.command:
        command = Path(args.command).read_text(encoding="utf-8", errors="replace")
        p5_active = re.search(r"(?:^|\s)--adaptive-stride(?:\s|$)", command) is not None
        if args.allow_p5_active:
            require(p5_active, "P5-active replay omitted --adaptive-stride", failures)
        else:
            require(not p5_active, "P5 was active in the P4-only replay", failures)
        require("--camera-frame-stride 12" in command, "P4 replay did not use fixed stride 12", failures)

    result = {
        "status": "PASS" if not failures else "FAIL",
        "metadata": str(metadata_path),
        "trajectory": str(trajectory_path),
        "trajectory_state_count": len(timestamps),
        "selected_window_duration_s": metadata.get("selected_window_duration_s"),
        "keyframe_count": keyframes,
        "release_lifecycle": "direct_sliding" if direct_sliding_release else "verified_frozen_candidate",
        "holdout_duration_s": metadata.get("formal_candidate_holdout_duration_s"),
        "overlap_stable_duration_s": metadata.get("sliding_overlap_stable_duration_s"),
        "refinement_count": metadata.get("candidate_refinement_count"),
        "final_solve_wall_time_s": metadata.get("final_stage_solve_wall_time_s"),
        "fc_terminal_max_normalized_residual": metadata.get("fc_terminal_max_normalized_residual"),
        "failures": failures,
    }
    output_path = Path(args.out)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
