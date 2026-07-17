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
    covariance = np.zeros((9 * count, 9 * count))
    for row in range(count):
        for column in range(count):
            covariance[
                9 * row : 9 * (row + 1),
                9 * column : 9 * (column + 1),
            ] = prefixes[min(row, column)]
    return covariance


def terminal_increment_transform(count: int) -> np.ndarray:
    transform = np.zeros((9 * count, 9 * count))
    transform[0:9, 9 * (count - 1) : 9 * count] = np.eye(9)
    for index in range(1, count):
        transform[9 * index : 9 * (index + 1), 9 * (index - 1) : 9 * index] = -np.eye(9)
        transform[9 * index : 9 * (index + 1), 9 * index : 9 * (index + 1)] = np.eye(9)
    return transform


def correlated_cost(timestamps: list[float]) -> float:
    terminal = np.diag([0.01, 0.02, 0.03, 4.0, 5.0, 6.0, 0.25, 0.36, 0.49])
    origin = np.array([0.02, -0.01, 0.03, 0.3, -0.2, 0.1, 0.05, -0.02, 0.01])
    rate = np.array([0.005, -0.004, 0.003, 0.1, -0.05, 0.03, 0.02, -0.01, 0.005])
    absolute_error = np.concatenate([origin + time * rate for time in timestamps])
    absolute = absolute_fc_covariance(timestamps, terminal, 0.25)
    transform = terminal_increment_transform(len(timestamps))
    residual = transform @ absolute_error
    covariance = transform @ absolute @ transform.T
    require(np.linalg.eigvalsh(covariance).min() > 0.0, "FC residual covariance must be SPD")
    require(np.linalg.norm(covariance[0:9, -9:]) > 1.0e-8, "terminal/increment correlation missing")
    require(np.allclose(absolute[-9:, -9:], terminal, atol=1.0e-12), "terminal covariance changed")
    return float(residual @ np.linalg.solve(covariance, residual))


def check_fc_model() -> int:
    coarse = correlated_cost([0.0, 1.0, 2.0])
    fine = correlated_cost([0.0, 0.5, 1.0, 1.5, 2.0])
    require(abs(coarse - fine) < 1.0e-10, "FC likelihood depends on keyframe density")
    return 5


def check_source_contract(repo: Path) -> int:
    runner = read(repo, "ov_msckf/src/run_serial_msckf_ros_free.cpp")
    initializer = read(repo, "ov_msckf/src/core/OnlineAlignmentInitializer.cpp")
    shared = read(repo, "ov_msckf/src/core/p4/factors/Factor_P4ImuSharedBias.cpp")
    p5 = read(repo, "ov_msckf/src/core/VioManager.cpp")
    handoff = read(repo, "ov_msckf/src/core/VioManagerHelper.cpp")

    checks = {
        "formal lifecycle selected": "online_options.formal_causal_lifecycle = true" in runner,
        "upstream gauge disabled": "online_options.upstream_dynamic_init_fc_gauge = false" in runner,
        "formal algorithm identity emitted":
            "openvins_p4_fc_pva_shared_bias_epipolar_v1" in runner,
        "formal/legacy configurations fail closed":
            "formal_lifecycle_must_be_terminal_only_and_mutually_exclusive" in initializer,
        "shared bg tied at both CPI endpoints":
            "parameters[5], parameters[1], parameters[6]" in shared,
        "shared ba tied at both CPI endpoints":
            "parameters[4], parameters[5], parameters[1], parameters[6],\n      parameters[3], parameters[7]" in shared,
        "shared bias Jacobians summed":
            "sum_blocks(1, 1, 6)" in shared and "sum_blocks(3, 3, 8)" in shared,
        "dense correlated FC factor installed":
            "Factor_P4FcTrajectory" in initializer and
            "single_dense_terminal_plus_correlated_increments" in initializer,
        "declared P3 mount and attitude-time uncertainty propagated":
            "Ptheta_includes_declared_mount_and_attitude_time_uncertainty" in initializer,
        "landmark-free epipolar factor installed": "Factor_P4Epipolar" in initializer,
        "immutable causal holdout path":
            "formal_fixed_candidate_causal_holdout_no_feedback" in initializer,
        "one-refinement release receipt": "formal_refined_release" in initializer,
        "invalid overlap covariance rejected":
            "P_current + P_previous is not the covariance" in initializer,
        "P5 feature termination populated":
            "trigger_input.feature_termination_imminent =" in p5,
        "camera-clock timestamp atomically handed off":
            "state->_timestamp = result.timestamp" in handoff,
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
