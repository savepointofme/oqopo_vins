#!/usr/bin/env python3
"""Generate C0/C1/C2 configs while holding T_C_I, time, init and backend fixed."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


HISTORICAL_INTRINSICS = "[386.750, 387.233, 330.249, 239.916]"
HISTORICAL_DISTORTION = "[-0.043, 0.035, -0.001, 0.001]"


def replace_once(text: str, old: str, new: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"expected one occurrence of {old!r}, got {text.count(old)}")
    return text.replace(old, new, 1)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    base_dir = args.repo / "baseline/clean_p4/config"
    estimator = (base_dir / "estimator_config.yaml").read_text(encoding="utf-8")
    imucam = (base_dir / "kalibr_imucam_chain.yaml").read_text(encoding="utf-8")
    imu = (args.repo / "baseline/latest/config/kalibr_imu_chain.yaml").read_text(encoding="utf-8")
    estimator = replace_once(
        estimator,
        'relative_config_imu: "../../latest/config/kalibr_imu_chain.yaml"',
        'relative_config_imu: "kalibr_imu_chain.yaml"',
    )
    conditions = {
        "C0_june12_ground_kd_locked": {
            "intrinsics_online": False,
            "imucam": imucam,
        },
        "C1_historical_fly3_online_final_kd_locked": {
            "intrinsics_online": False,
            "imucam": replace_once(
                replace_once(
                    imucam,
                    "intrinsics: [382.9994702701404, 382.2768370977604, 332.5997999732952, 236.54464292967157]",
                    f"intrinsics: {HISTORICAL_INTRINSICS}",
                ),
                "distortion_coeffs: [-0.06073119983113053, 0.0428789308670328, -0.003107075293581619, 0.0010996103095215512]",
                f"distortion_coeffs: {HISTORICAL_DISTORTION}",
            ),
        },
        "C2_june12_initial_online_kd_only": {
            "intrinsics_online": True,
            "imucam": imucam,
        },
    }
    manifest = {
        "schema_version": "openvins-baseline-calibration-isolation-config/v1",
        "single_variable": "camera intrinsics/distortion group",
        "T_C_I_same_all_conditions": True,
        "camera_imu_time_offset_same_all_conditions": True,
        "initialization_same_all_conditions": "P4 I2",
        "backend_same_all_conditions": True,
        "fixed_fc_board_correction": None,
        "conditions": {},
    }
    for name, spec in conditions.items():
        condition_dir = args.out_dir / name
        condition_dir.mkdir(parents=True, exist_ok=True)
        imucam_path = condition_dir / "kalibr_imucam_chain.yaml"
        imucam_path.write_text(spec["imucam"], encoding="utf-8")
        imu_path = condition_dir / "kalibr_imu_chain.yaml"
        imu_path.write_text(imu, encoding="utf-8")
        config = estimator
        if spec["intrinsics_online"]:
            config = replace_once(
                config, "calib_cam_intrinsics: false", "calib_cam_intrinsics: true"
            )
        config_path = condition_dir / "estimator_config.yaml"
        config_path.write_text(config, encoding="utf-8")
        manifest["conditions"][name] = {
            "config": str(config_path),
            "imucam": str(imucam_path),
            "imu": str(imu_path),
            "intrinsics_online": spec["intrinsics_online"],
        }
    (args.out_dir / "CALIBRATION_ISOLATION_CONFIG_MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == "__main__":
    main()
