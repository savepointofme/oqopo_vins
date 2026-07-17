#!/usr/bin/env python3
"""Materialize a P4 FC input with a Kalibr FC-to-board rotation declaration."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


NOMINAL_BOARD_FROM_FC = np.array(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=float
)


def fractional_rotation(rotation: np.ndarray, fraction: float) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1.0e-12:
        return np.eye(3)
    axis = np.array(
        [rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]],
        dtype=float,
    ) / (2.0 * math.sin(angle))
    scaled = angle * fraction
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return np.eye(3) + math.sin(scaled) * skew + (1.0 - math.cos(scaled)) * (skew @ skew)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--kalibr-result", required=True)
    parser.add_argument("--fraction", type=float, required=True)
    parser.add_argument("--mount-sigma-deg", type=float, required=True)
    parser.add_argument("--variant", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    if not 0.0 <= args.fraction <= 1.0:
        raise ValueError("fraction must be in [0,1]")
    result = json.loads(Path(args.kalibr_result).read_text(encoding="utf-8"))
    residual = np.asarray(result["R_board_from_nominal_board_axes"], dtype=float)
    if residual.shape != (3, 3):
        raise ValueError("Kalibr residual rotation must be 3x3")
    applied = fractional_rotation(residual, args.fraction) @ NOMINAL_BOARD_FROM_FC
    if not np.allclose(applied.T @ applied, np.eye(3), atol=1.0e-9):
        raise ValueError("materialized rotation is not orthonormal")
    if abs(np.linalg.det(applied) - 1.0) > 1.0e-9:
        raise ValueError("materialized rotation determinant is not +1")

    matrix_text = ",".join(f"{value:.12g}" for value in applied.reshape(-1))
    replacements = {
        "R_FtoI_calibrated_row_major": matrix_text,
        "fc_board_mount_sigma_deg": f"{args.mount_sigma_deg:.9f}",
        "fc_board_calibration_source": str(Path(args.kalibr_result).resolve()),
    }
    source = Path(args.source)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output_lines: list[str] = []
    replaced: set[str] = set()
    for line in source.read_text(encoding="utf-8").splitlines():
        if line.startswith("# ") and "=" in line:
            key = line[2:].split("=", 1)[0]
            if key in replacements:
                output_lines.append(f"# {key}={replacements[key]}")
                replaced.add(key)
                continue
        output_lines.append(line)
    missing = set(replacements) - replaced
    if missing:
        raise ValueError(f"source is missing required declarations: {sorted(missing)}")
    insertion = next(index for index, line in enumerate(output_lines) if not line.startswith("#"))
    output_lines[insertion:insertion] = [
        f"# kalibr_mount_variant={args.variant}",
        f"# kalibr_mount_fraction={args.fraction:.9f}",
        "# kalibr_function=IccImu.findOrientationPrior",
        "# camera_imu_calibration=unchanged_june12",
        "# fc_navigation_time_contract=unchanged",
    ]
    output.write_text("\n".join(output_lines) + "\n", encoding="utf-8")
    print(output)
    print(f"fraction={args.fraction:.9f}")
    print(f"mount_sigma_deg={args.mount_sigma_deg:.9f}")
    print(f"R_FtoI={matrix_text}")


if __name__ == "__main__":
    main()
