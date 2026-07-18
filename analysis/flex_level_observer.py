#!/usr/bin/env python3
"""Low-frequency body--D455 flex level observer.

The implementation is deliberately state-isolated: it consumes adjacent SO(3)
increments and emits only an adopted mount plus a derived aircraft attitude.
It has no VIO state, P/V, bias, covariance, or camera-extrinsic write path.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


def project_so3(matrix: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=float))
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def exp_deg(vector_deg: np.ndarray) -> np.ndarray:
    return Rotation.from_rotvec(np.radians(np.asarray(vector_deg, dtype=float))).as_matrix()


def log_deg(matrix: np.ndarray) -> np.ndarray:
    return np.degrees(Rotation.from_matrix(project_so3(matrix)).as_rotvec())


def angle_deg(matrix: np.ndarray) -> float:
    return float(np.linalg.norm(log_deg(matrix)))


def clip_rotvec_deg(vector_deg: np.ndarray, maximum_deg: float) -> np.ndarray:
    vector = np.asarray(vector_deg, dtype=float)
    magnitude = float(np.linalg.norm(vector))
    if magnitude <= maximum_deg or magnitude <= 1.0e-12:
        return vector.copy()
    return vector * (maximum_deg / magnitude)


def robust_so3_mean(matrices: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Return an iteratively reweighted left-invariant SO(3) mean and residuals."""

    if not matrices:
        raise ValueError("robust_so3_mean requires at least one rotation")
    stack = np.asarray([project_so3(matrix) for matrix in matrices])
    chordal = np.sum(stack, axis=0)
    mean = project_so3(chordal)
    for _ in range(30):
        residuals = np.asarray([log_deg(matrix @ mean.T) for matrix in stack])
        norms = np.linalg.norm(residuals, axis=1)
        median = float(np.median(norms))
        mad = float(np.median(np.abs(norms - median)))
        huber_scale = max(0.05, median + 1.5 * 1.4826 * mad)
        weights = np.minimum(1.0, huber_scale / np.maximum(norms, 1.0e-12))
        step = np.average(residuals, axis=0, weights=weights)
        if float(np.linalg.norm(step)) < 1.0e-10:
            break
        mean = exp_deg(step) @ mean
    residuals = np.asarray([log_deg(matrix @ mean.T) for matrix in stack])
    return project_so3(mean), residuals


@dataclass(frozen=True)
class FlexLevelParameters:
    evidence_duration_s: float = 15.0
    confirmation_duration_s: float = 15.0
    evidence_evaluation_period_s: float = 1.0
    correction_cooldown_s: float = 30.0
    release_duration_s: float = 5.0
    noise_floor_deg: float = 1.05
    maximum_window_median_dispersion_deg: float = 0.40
    maximum_window_p95_dispersion_deg: float = 0.90
    maximum_window_half_shift_deg: float = 0.55
    maximum_candidate_confirmation_distance_deg: float = 0.70
    minimum_level_direction_fraction: float = 0.90
    maximum_correction_deg: float = 3.55
    minimum_pair_dt_s: float = 0.20
    maximum_pair_dt_s: float = 0.65
    maximum_fc_bracket_gap_s: float = 0.45
    maximum_fc_endpoint_dt_s: float = 0.25
    maximum_fc_rate_deg_s: float = 35.0
    maximum_d455_rate_deg_s: float = 35.0
    maximum_fc_vio_residual_rate_deg_s: float = 10.0
    minimum_quality_ratio: float = 0.90
    maximum_continuity_gap_s: float = 0.80


@dataclass(frozen=True)
class FlexIncrementSample:
    t_start: float
    t_end: float
    d455_delta_R_I: np.ndarray
    fc_delta_R_B: np.ndarray
    fc_time_valid: bool
    fc_bracket_gap_s: float
    fc_endpoint_dt_s: float
    quality_valid: bool
    quality_failures: tuple[str, ...] = ()


@dataclass
class LevelEvidence:
    start_s: float
    end_s: float
    mean_R_I_from_B: np.ndarray
    level_delta_deg: np.ndarray
    median_dispersion_deg: float
    p95_dispersion_deg: float
    half_shift_deg: float
    level_direction_fraction: float
    quality_ratio: float
    passed: bool
    failed_gates: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "start_s": self.start_s,
            "end_s": self.end_s,
            "mean_R_I_from_B": self.mean_R_I_from_B.tolist(),
            "level_delta_deg": self.level_delta_deg.tolist(),
            "level_delta_angle_deg": float(np.linalg.norm(self.level_delta_deg)),
            "median_dispersion_deg": self.median_dispersion_deg,
            "p95_dispersion_deg": self.p95_dispersion_deg,
            "half_shift_deg": self.half_shift_deg,
            "level_direction_fraction": self.level_direction_fraction,
            "quality_ratio": self.quality_ratio,
            "passed": self.passed,
            "failed_gates": list(self.failed_gates),
        }


@dataclass
class Observation:
    t_start: float
    t_end: float
    R_I_from_B_obs: np.ndarray
    quality_valid: bool


class FlexLevelObserver:
    """Two-stage persistent mount-level detector with bounded smooth release."""

    def __init__(
        self,
        nominal_R_I_from_B: np.ndarray,
        parameters: FlexLevelParameters | None = None,
    ) -> None:
        self.parameters = parameters or FlexLevelParameters()
        self.nominal_R_I_from_B = project_so3(nominal_R_I_from_B)
        self._observed = self.nominal_R_I_from_B.copy()
        self._target = self.nominal_R_I_from_B.copy()
        self._release_start = self._target.copy()
        self._release_delta_deg = np.zeros(3)
        self._release_start_s = -math.inf
        self._last_correction_s = -math.inf
        self._last_sample_end_s = -math.inf
        self._last_evaluation_s = -math.inf
        self._phase = "search"
        self._phase_observations: list[Observation] = []
        self._candidate: LevelEvidence | None = None
        self._departure_start_s: float | None = None
        self.events: list[dict[str, Any]] = []
        self.evidence_audit: list[dict[str, Any]] = []

    @staticmethod
    def propagate_mount_observation(
        current_R_I_from_B: np.ndarray,
        d455_delta_R_I: np.ndarray,
        fc_delta_R_B: np.ndarray,
    ) -> np.ndarray:
        return project_so3(
            np.asarray(d455_delta_R_I)
            @ np.asarray(current_R_I_from_B)
            @ np.asarray(fc_delta_R_B).T
        )

    @property
    def observed_R_I_from_B(self) -> np.ndarray:
        return self._observed.copy()

    @property
    def target_R_I_from_B(self) -> np.ndarray:
        return self._target.copy()

    @property
    def last_correction_s(self) -> float:
        return self._last_correction_s

    def applied_R_I_from_B(self, timestamp_s: float) -> np.ndarray:
        if not np.isfinite(self._release_start_s):
            return self._target.copy()
        alpha = float(
            np.clip(
                (timestamp_s - self._release_start_s)
                / max(self.parameters.release_duration_s, 1.0e-9),
                0.0,
                1.0,
            )
        )
        return exp_deg(alpha * self._release_delta_deg) @ self._release_start

    def body_R_B_to_G(self, R_I_to_G: np.ndarray, timestamp_s: float) -> np.ndarray:
        return project_so3(np.asarray(R_I_to_G) @ self.applied_R_I_from_B(timestamp_s))

    def sample_failures(self, sample: FlexIncrementSample) -> list[str]:
        p = self.parameters
        failures: list[str] = []
        dt = float(sample.t_end - sample.t_start)
        finite = np.isfinite(sample.d455_delta_R_I).all() and np.isfinite(sample.fc_delta_R_B).all()
        if not finite:
            failures.append("nonfinite_rotation")
        if not sample.fc_time_valid:
            failures.append("fc_time_invalid")
        if not (p.minimum_pair_dt_s <= dt <= p.maximum_pair_dt_s):
            failures.append("pair_dt")
        if sample.fc_bracket_gap_s > p.maximum_fc_bracket_gap_s:
            failures.append("fc_bracket_gap")
        if sample.fc_endpoint_dt_s > p.maximum_fc_endpoint_dt_s:
            failures.append("fc_endpoint_dt")
        if finite and dt > 0.0:
            if angle_deg(sample.fc_delta_R_B) / dt > p.maximum_fc_rate_deg_s:
                failures.append("fc_rate")
            if angle_deg(sample.d455_delta_R_I) / dt > p.maximum_d455_rate_deg_s:
                failures.append("d455_rate")
            predicted = self._target @ sample.fc_delta_R_B @ self._target.T
            if angle_deg(sample.d455_delta_R_I @ predicted.T) / dt > p.maximum_fc_vio_residual_rate_deg_s:
                failures.append("fc_vio_residual_rate")
        return failures

    def _reset_evidence(self, reason: str, timestamp_s: float) -> None:
        if self._phase_observations or self._candidate is not None:
            self.events.append(
                {
                    "event_type": "evidence_reset",
                    "time_s": float(timestamp_s),
                    "reason": reason,
                    "phase": self._phase,
                }
            )
        self._phase = "search"
        self._phase_observations = []
        self._candidate = None
        self._last_evaluation_s = -math.inf

    def _trim_phase(self, duration_s: float) -> None:
        if not self._phase_observations:
            return
        end = self._phase_observations[-1].t_end
        while (
            len(self._phase_observations) > 1
            and end - self._phase_observations[0].t_start
            > duration_s + self.parameters.maximum_pair_dt_s
        ):
            self._phase_observations.pop(0)

    def _evaluate(self, duration_s: float, require_shift: bool) -> LevelEvidence | None:
        observations = self._phase_observations
        if not observations:
            return None
        span = observations[-1].t_end - observations[0].t_start
        if span < duration_s:
            return None
        matrices = [item.R_I_from_B_obs for item in observations]
        mean, residuals = robust_so3_mean(matrices)
        dispersion = np.linalg.norm(residuals, axis=1)
        midpoint = len(matrices) // 2
        first_mean, _ = robust_so3_mean(matrices[: max(midpoint, 1)])
        second_mean, _ = robust_so3_mean(matrices[max(midpoint, 1) :])
        level_delta = log_deg(mean @ self._target.T)
        level_angle = float(np.linalg.norm(level_delta))
        direction = level_delta / max(level_angle, 1.0e-12)
        sample_deltas = np.asarray([log_deg(matrix @ self._target.T) for matrix in matrices])
        direction_fraction = float(np.mean(sample_deltas @ direction > 0.0))
        quality_ratio = float(np.mean([item.quality_valid for item in observations]))
        median_dispersion = float(np.median(dispersion))
        p95_dispersion = float(np.percentile(dispersion, 95.0))
        half_shift = angle_deg(second_mean @ first_mean.T)
        failures: list[str] = []
        p = self.parameters
        if require_shift and level_angle < p.noise_floor_deg:
            failures.append("below_noise_floor")
        if median_dispersion > p.maximum_window_median_dispersion_deg:
            failures.append("median_dispersion")
        if p95_dispersion > p.maximum_window_p95_dispersion_deg:
            failures.append("p95_dispersion")
        if half_shift > p.maximum_window_half_shift_deg:
            failures.append("window_not_level")
        if direction_fraction < p.minimum_level_direction_fraction:
            failures.append("level_direction_fraction")
        if quality_ratio < p.minimum_quality_ratio:
            failures.append("quality_ratio")
        return LevelEvidence(
            start_s=float(observations[0].t_start),
            end_s=float(observations[-1].t_end),
            mean_R_I_from_B=mean,
            level_delta_deg=level_delta,
            median_dispersion_deg=median_dispersion,
            p95_dispersion_deg=p95_dispersion,
            half_shift_deg=half_shift,
            level_direction_fraction=direction_fraction,
            quality_ratio=quality_ratio,
            passed=not failures,
            failed_gates=failures,
        )

    def add_sample(self, sample: FlexIncrementSample) -> None:
        p = self.parameters
        hard_failures = self.sample_failures(sample)
        continuity_break = (
            np.isfinite(self._last_sample_end_s)
            and sample.t_start - self._last_sample_end_s > p.maximum_continuity_gap_s
        )
        self._last_sample_end_s = float(sample.t_end)
        if hard_failures or continuity_break:
            reason = ";".join(hard_failures) if hard_failures else "continuity_gap"
            self._reset_evidence(reason, sample.t_end)
            return

        self._observed = self.propagate_mount_observation(
            self._observed, sample.d455_delta_R_I, sample.fc_delta_R_B
        )
        observed_shift = angle_deg(self._observed @ self._target.T)
        if observed_shift >= p.noise_floor_deg and self._departure_start_s is None:
            self._departure_start_s = float(sample.t_end)
        elif observed_shift < 0.5 * p.noise_floor_deg:
            self._departure_start_s = None

        if sample.t_end - self._last_correction_s < p.correction_cooldown_s:
            self._phase_observations = []
            self._candidate = None
            self._phase = "search"
            return

        self._phase_observations.append(
            Observation(
                t_start=float(sample.t_start),
                t_end=float(sample.t_end),
                R_I_from_B_obs=self._observed.copy(),
                quality_valid=bool(sample.quality_valid),
            )
        )

        if self._phase == "search":
            self._trim_phase(p.evidence_duration_s)
            if sample.t_end - self._last_evaluation_s < p.evidence_evaluation_period_s:
                return
            evidence = self._evaluate(p.evidence_duration_s, require_shift=True)
            if evidence is None:
                return
            self._last_evaluation_s = float(sample.t_end)
            self.evidence_audit.append({"phase": "candidate", **evidence.to_dict()})
            if not evidence.passed:
                return
            self._candidate = evidence
            self.events.append(
                {
                    "event_type": "candidate",
                    "time_s": evidence.end_s,
                    "level_change_start_s": self._departure_start_s,
                    "evidence": evidence.to_dict(),
                }
            )
            self._phase = "confirmation"
            self._phase_observations = []
            self._last_evaluation_s = -math.inf
            return

        self._trim_phase(p.confirmation_duration_s)
        if sample.t_end - self._last_evaluation_s < p.evidence_evaluation_period_s:
            return
        confirmation = self._evaluate(p.confirmation_duration_s, require_shift=True)
        if confirmation is None:
            return
        self._last_evaluation_s = float(sample.t_end)
        self.evidence_audit.append({"phase": "confirmation", **confirmation.to_dict()})
        assert self._candidate is not None
        level_distance = angle_deg(
            confirmation.mean_R_I_from_B @ self._candidate.mean_R_I_from_B.T
        )
        rejection_reasons = list(confirmation.failed_gates)
        if level_distance > p.maximum_candidate_confirmation_distance_deg:
            rejection_reasons.append("candidate_confirmation_level_distance")
        correction_delta = log_deg(confirmation.mean_R_I_from_B @ self._target.T)
        correction_angle = float(np.linalg.norm(correction_delta))
        if correction_angle > p.maximum_correction_deg:
            rejection_reasons.append("correction_limit")
        if rejection_reasons:
            self.events.append(
                {
                    "event_type": "candidate_rejected",
                    "time_s": confirmation.end_s,
                    "candidate": self._candidate.to_dict(),
                    "confirmation": confirmation.to_dict(),
                    "candidate_confirmation_distance_deg": level_distance,
                    "failed_gates": rejection_reasons,
                }
            )
            self._reset_evidence("confirmation_rejected", confirmation.end_s)
            return

        old_target = self._target.copy()
        old_applied = self.applied_R_I_from_B(confirmation.end_s)
        bounded_delta = clip_rotvec_deg(correction_delta, p.maximum_correction_deg)
        self._target = exp_deg(bounded_delta) @ old_target
        self._release_start = old_applied
        self._release_delta_deg = log_deg(self._target @ self._release_start.T)
        self._release_start_s = float(confirmation.end_s)
        self._last_correction_s = float(confirmation.end_s)
        self.events.append(
            {
                "event_type": "correction",
                "time_s": confirmation.end_s,
                "level_change_start_s": self._departure_start_s,
                "candidate": self._candidate.to_dict(),
                "confirmation": confirmation.to_dict(),
                "candidate_confirmation_distance_deg": level_distance,
                "correction_vector_deg": bounded_delta.tolist(),
                "correction_angle_deg": float(np.linalg.norm(bounded_delta)),
                "release_duration_s": p.release_duration_s,
                "old_target_R_I_from_B": old_target.tolist(),
                "new_target_R_I_from_B": self._target.tolist(),
            }
        )
        self._departure_start_s = None
        self._phase = "search"
        self._phase_observations = []
        self._candidate = None
        self._last_evaluation_s = -math.inf

    def parameter_dict(self) -> dict[str, Any]:
        return asdict(self.parameters)
