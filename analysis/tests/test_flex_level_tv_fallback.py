from __future__ import annotations

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from analysis.flex_level_observer import exp_deg
from analysis.flex_level_tv_fallback import (
    SecondLevel,
    TVFallbackParameters,
    fixed_delay_tv_events,
)


NOMINAL = Rotation.from_euler("zyx", [83.0, -1.0, -178.0], degrees=True).as_matrix()


class FixedDelayTVFallbackTest(unittest.TestCase):
    def test_persistent_piecewise_level_is_detected_after_fixed_delay(self) -> None:
        shifted = exp_deg(np.array([1.8, -0.4, 0.2])) @ NOMINAL
        levels = [
            SecondLevel(
                t=float(index),
                R_I_from_B_obs=NOMINAL if index < 24 else shifted,
                quality_ratio=1.0,
                fc_valid_ratio=1.0,
            )
            for index in range(70)
        ]
        events = fixed_delay_tv_events(levels, NOMINAL, TVFallbackParameters())
        corrections = [event for event in events if event["event_type"] == "correction"]
        self.assertEqual(len(corrections), 1)
        self.assertGreaterEqual(corrections[0]["time_s"] - 24.0, 15.0)
        self.assertGreaterEqual(corrections[0]["correction_angle_deg"], 1.05)

    def test_short_vibration_and_missing_fc_do_not_create_events(self) -> None:
        vibration = exp_deg(np.array([1.8, 0.0, 0.0])) @ NOMINAL
        levels = []
        for index in range(80):
            mount = vibration if 20 <= index < 25 else NOMINAL
            levels.append(
                SecondLevel(
                    t=float(index),
                    R_I_from_B_obs=mount,
                    quality_ratio=1.0,
                    fc_valid_ratio=0.0 if 35 <= index < 40 else 1.0,
                )
            )
        events = fixed_delay_tv_events(levels, NOMINAL, TVFallbackParameters())
        self.assertFalse(any(event["event_type"] == "correction" for event in events))


if __name__ == "__main__":
    unittest.main()
