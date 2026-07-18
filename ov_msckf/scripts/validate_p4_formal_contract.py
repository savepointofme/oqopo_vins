#!/usr/bin/env python3
"""Dependency-light executable checks for the formal P4/P5 source contract.

This is not a replacement for the native C++ tests or flight validation.  It
does make the high-risk lifecycle/topology switches and the correlated FC
likelihood independently inspectable on hosts that do not have ROS/Ceres.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(repo: Path, relative: str) -> str:
    return (repo / relative).read_text(encoding="utf-8")


def absolute_fc_covariance(
    timestamps: list[float], terminal: np.ndarray, fraction: float
) -> np.ndarray:
    require(len(timestamps) >= 2, "FC covariance needs at least two timestamps")
    require(0.0 < fraction < 1.0, "FC process fraction must be in (0,1)")
    span = timestamps[-1] - timestamps[0]
    require(span > 0.0, "FC timestamps must span positive time")
    prefixes = [(1.0 - fraction) * terminal]
    for previous, current in zip(timestamps, timestamps[1:]):
        dt = current - previous
        require(dt > 0.0, "FC timestamps must increase")
        prefixes.append(prefixes[-1] + fraction * terminal * dt / span)
    count = len(timestamps)
    dimension = terminal.shape[0]
    covariance = np.zeros((dimension * count, dimension * count))
    for row in range(count):
        for column in range(count):
            covariance[
                dimension * row : dimension * (row + 1),
                dimension * column : dimension * (column + 1),
            ] = prefixes[min(row, column)]
    return covariance


def terminal_increment_transform(count: int) -> np.ndarray:
    transform = np.zeros((6 * count, 6 * count))
    transform[0:6, 6 * (count - 1) : 6 * count] = np.eye(6)
    for index in range(1, count):
        transform[6 * index : 6 * (index + 1), 6 * (index - 1) : 6 * index] = -np.eye(6)
        transform[6 * index : 6 * (index + 1), 6 * index : 6 * (index + 1)] = np.eye(6)
    return transform


def correlated_cost(timestamps: list[float]) -> float:
    terminal = np.diag([4.0, 5.0, 6.0, 0.25, 0.36, 0.49])
    origin = np.array([0.3, -0.2, 0.1, 0.05, -0.02, 0.01])
    rate = np.array([0.1, -0.05, 0.03, 0.02, -0.01, 0.005])
    absolute_error = np.concatenate([origin + time * rate for time in timestamps])
    absolute = absolute_fc_covariance(timestamps, terminal, 0.25)
    transform = terminal_increment_transform(len(timestamps))
    residual = transform @ absolute_error
    covariance = transform @ absolute @ transform.T
    require(np.linalg.eigvalsh(covariance).min() > 0.0, "FC residual covariance must be SPD")
    require(np.linalg.norm(covariance[0:6, -6:]) > 1.0e-8, "terminal/increment correlation missing")
    require(np.allclose(absolute[-6:, -6:], terminal, atol=1.0e-12), "terminal covariance changed")
    return float(residual @ np.linalg.solve(covariance, residual))


def check_fc_model() -> int:
    coarse = correlated_cost([0.0, 1.0, 2.0])
    fine = correlated_cost([0.0, 0.5, 1.0, 1.5, 2.0])
    require(abs(coarse - fine) < 1.0e-10, "FC likelihood depends on keyframe density")
    return 5


def check_source_contract(repo: Path) -> int:
    runner = read(repo, "ov_msckf/src/run_serial_msckf_ros_free.cpp")
    initializer = read(repo, "ov_msckf/src/core/OnlineAlignmentInitializer.cpp")
    p5 = read(repo, "ov_msckf/src/core/VioManager.cpp")
    handoff = read(repo, "ov_msckf/src/core/VioManagerHelper.cpp")

    checks = {
        "formal lifecycle selected": "online_options.formal_causal_lifecycle = true" in runner,
        "upstream gauge disabled": "online_options.upstream_dynamic_init_fc_gauge = false" in runner,
        "formal algorithm identity emitted":
            "openvins_p4_fc_yaw_pv_per_keyframe_bias_reprojection_v4" in runner,
        "formal/legacy configurations fail closed":
            "formal_lifecycle_must_be_terminal_only_and_mutually_exclusive" in initializer,
        "standard per-keyframe 15-D CPI factor installed":
            "new ov_init::Factor_ImuCPIv1(" in initializer and
            "states[i].bg.data()," in initializer and
            "states[i].ba.data(), states[i].p.data()" in initializer,
        "production graph does not instantiate shared-bias CPI adapter":
            "new p4::Factor_P4ImuSharedBias(" not in initializer,
        "dense correlated FC factor installed":
            "Factor_P4FcTrajectory" in initializer and
            "terminal_yaw_gauge_plus_dense_pv_terminal_and_correlated_increments" in initializer and
            'yaw_types = {"quat_yaw"}' in initializer and
            "no_fc_attitude_increments" in initializer,
        "declared P3 mount and attitude-time uncertainty propagated":
            "yaw_sigma_includes_declared_mount_and_attitude_time_uncertainty" in initializer,
        "explicit multi-view landmark reprojection installed":
            "new ov_init::Factor_ImageReprojCalib(" in initializer and
            "multi_view_explicit_landmark_reprojection_all_observations" in initializer,
        "immutable causal holdout path":
            "formal_finite_window_solution_causal_holdout_no_feedback" in initializer,
        "verified endpoint release receipt":
            "formal_holdout_verified_release" in initializer,
        "invalid overlap covariance rejected":
            "P_current + P_previous is not the covariance" in initializer,
        "P5 feature termination populated":
            "trigger_input.feature_termination_imminent =" in p5,
        "camera-clock timestamp atomically handed off":
            "state->_timestamp = result.timestamp" in handoff,
        "verified formal release preserves its graph marginal":
            "verified_formal_candidate_release" in handoff and
            "result.diagnostics.formal_candidate_holdout_passed" in handoff and
            "formal_joint_terminal_schur_marginal" in handoff and
            "result.diagnostics.direct_sliding_state_release &&" in handoff,
    }
    failed = [name for name, passed in checks.items() if not passed]
    require(not failed, "source contract failures: " + ", ".join(failed))
    return len(checks)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    repo = args.repo.resolve()
    checks = check_fc_model() + check_source_contract(repo)
    print(f"formal P4/P5 static+math contract: {checks} checks passed")
    print("native C++ build/tests and fly1-fly4 validation remain separate requirements")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (AssertionError, OSError, ValueError, np.linalg.LinAlgError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        sys.exit(1)
