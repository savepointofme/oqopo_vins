from __future__ import annotations

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from analysis.flex_level_observer import (
    FlexIncrementSample,
    FlexLevelObserver,
    FlexLevelParameters,
    angle_deg,
    exp_deg,
)


NOMINAL = Rotation.from_euler("zyx", [83.0, -1.0, -178.0], degrees=True).as_matrix()


def parameters() -> FlexLevelParameters:
    return FlexLevelParameters(
        evidence_duration_s=3.0,
        confirmation_duration_s=3.0,
        evidence_evaluation_period_s=1.0,
        correction_cooldown_s=6.0,
        release_duration_s=2.0,
        noise_floor_deg=0.25,
        maximum_window_median_dispersion_deg=0.08,
        maximum_window_p95_dispersion_deg=0.12,
        maximum_window_half_shift_deg=0.08,
        maximum_candidate_confirmation_distance_deg=0.12,
        minimum_level_direction_fraction=0.8,
        maximum_correction_deg=2.0,
        minimum_pair_dt_s=0.5,
        maximum_pair_dt_s=1.5,
    )


def transition_sample(
    index: int,
    old_mount: np.ndarray,
    new_mount: np.ndarray,
    *,
    valid: bool = True,
) -> FlexIncrementSample:
    body_delta = Rotation.from_euler(
        "xyz",
        [0.7 + 0.1 * (index % 3), -0.4 + 0.05 * (index % 2), 1.1],
        degrees=True,
    ).as_matrix()
    d455_delta = new_mount @ body_delta @ old_mount.T
    return FlexIncrementSample(
        t_start=float(index),
        t_end=float(index + 1),
        d455_delta_R_I=d455_delta,
        fc_delta_R_B=body_delta,
        fc_time_valid=valid,
        fc_bracket_gap_s=0.1,
        fc_endpoint_dt_s=0.05,
        quality_valid=valid,
    )


class FlexLevelObserverTest(unittest.TestCase):
    def test_noncommuting_three_axis_order_recovers_transition(self) -> None:
        old_mount = Rotation.from_euler("xyz", [4.0, -7.0, 13.0], degrees=True).as_matrix()
        flex_delta = Rotation.from_euler("zyx", [1.2, -0.8, 0.5], degrees=True).as_matrix()
        new_mount = flex_delta @ old_mount
        body_delta = Rotation.from_euler("xyz", [3.0, -5.0, 8.0], degrees=True).as_matrix()
        d455_delta = new_mount @ body_delta @ old_mount.T
        recovered = FlexLevelObserver.propagate_mount_observation(
            old_mount, d455_delta, body_delta
        )
        np.testing.assert_allclose(recovered, new_mount, atol=1.0e-12)
        wrong = d455_delta @ old_mount @ body_delta
        self.assertGreater(angle_deg(wrong @ new_mount.T), 5.0)

    def test_fixed_mount_is_invariant_under_noncommuting_motion(self) -> None:
        mount = NOMINAL.copy()
        observed = mount.copy()
        for index in range(20):
            body_delta = Rotation.from_euler(
                "xyz", [0.3 * index, -0.2 * index, 0.5 * index], degrees=True
            ).as_matrix()
            d455_delta = mount @ body_delta @ mount.T
            observed = FlexLevelObserver.propagate_mount_observation(
                observed, d455_delta, body_delta
            )
        np.testing.assert_allclose(observed, mount, atol=1.0e-11)

    def test_two_independent_level_windows_then_smooth_release(self) -> None:
        observer = FlexLevelObserver(NOMINAL, parameters())
        true_mount = NOMINAL.copy()
        shifted = exp_deg(np.array([0.8, -0.2, 0.1])) @ NOMINAL
        for index in range(8):
            old = true_mount
            true_mount = shifted
            observer.add_sample(transition_sample(index, old, true_mount))
        event_types = [event["event_type"] for event in observer.events]
        self.assertIn("candidate", event_types)
        self.assertIn("correction", event_types)
        correction = next(event for event in observer.events if event["event_type"] == "correction")
        accepted = float(correction["time_s"])
        self.assertAlmostEqual(
            angle_deg(observer.applied_R_I_from_B(accepted) @ NOMINAL.T), 0.0, places=9
        )
        self.assertGreater(
            angle_deg(observer.applied_R_I_from_B(accepted + 1.0) @ NOMINAL.T), 0.2
        )
        np.testing.assert_allclose(
            observer.applied_R_I_from_B(accepted + 2.0),
            observer.target_R_I_from_B,
            atol=1.0e-10,
        )

    def test_confirmation_needs_same_level_not_another_increment(self) -> None:
        observer = FlexLevelObserver(NOMINAL, parameters())
        true_mount = NOMINAL.copy()
        shifted = exp_deg(np.array([0.7, 0.0, 0.0])) @ NOMINAL
        for index in range(8):
            old = true_mount
            true_mount = shifted
            observer.add_sample(transition_sample(index, old, true_mount))
        corrections = [event for event in observer.events if event["event_type"] == "correction"]
        self.assertEqual(len(corrections), 1)
        self.assertLess(corrections[0]["candidate_confirmation_distance_deg"], 1.0e-8)

    def test_missing_fc_holds_level_and_creates_no_candidate(self) -> None:
        observer = FlexLevelObserver(NOMINAL, parameters())
        before = observer.target_R_I_from_B
        for index in range(20):
            observer.add_sample(transition_sample(index, NOMINAL, NOMINAL, valid=False))
        np.testing.assert_array_equal(observer.target_R_I_from_B, before)
        self.assertFalse(
            any(event["event_type"] in {"candidate", "correction"} for event in observer.events)
        )

    def test_cooldown_prevents_second_correction_within_minimum_interval(self) -> None:
        observer = FlexLevelObserver(NOMINAL, parameters())
        true_mount = NOMINAL.copy()
        levels = [exp_deg(np.array([0.7, 0.0, 0.0])) @ NOMINAL] * 8
        levels += [exp_deg(np.array([1.4, 0.0, 0.0])) @ NOMINAL] * 8
        for index, level in enumerate(levels):
            old = true_mount
            true_mount = level
            observer.add_sample(transition_sample(index, old, true_mount))
        correction_times = [
            event["time_s"] for event in observer.events if event["event_type"] == "correction"
        ]
        self.assertTrue(
            all(
                right - left >= parameters().correction_cooldown_s
                for left, right in zip(correction_times, correction_times[1:])
            )
        )


if __name__ == "__main__":
    unittest.main()
