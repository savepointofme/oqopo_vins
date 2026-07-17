#!/usr/bin/env python3
"""Prepare, provenance, and validate the P4 calibration factorial replay.

This helper never edits estimator source.  It writes only to the dedicated
experiment root and keeps the estimator command identical apart from the
per-condition camera/IMU calibration and the two declared post-P4 z-minus
conditions.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import shlex
import shutil
import subprocess
import sys
from typing import Any, Iterable


ROOT_WIN = Path(
    r"C:\Users\baloney\Desktop\实验目录\P4_yaw_calibration_factorial_20260716_ultra"
)
ROOT_WSL = PurePosixPath(
    "/mnt/c/Users/baloney/Desktop/实验目录/P4_yaw_calibration_factorial_20260716_ultra"
)
REPO_WIN = Path(r"D:\vscode_dir\open_vins_p4_sliding_r1")
REPO_WSL = PurePosixPath("/mnt/d/vscode_dir/open_vins_p4_sliding_r1")
MAIN_REPO_WIN = Path(r"D:\vscode_dir\open_vins")

ROOT = Path(str(ROOT_WSL)) if os.name == "posix" else ROOT_WIN
REPO = Path(str(REPO_WSL)) if os.name == "posix" else REPO_WIN

JUNE_T_C_I = [
    [0.9998730063222916, -0.015860686216656983, 0.0015523726228432147, -0.026296287593010406],
    [-0.01582807339579776, -0.9996913355440527, -0.01914956215494984, 0.028572654300092745],
    [0.0018556186571181857, 0.019122559213813277, -0.9998154251703234, -0.019145062017658972],
    [0.0, 0.0, 0.0, 1.0],
]

A_FLY4_T_C_I = [
    [0.99995797, -0.00282896, 0.00872137, -0.02561949],
    [-0.00277724, -0.99997852, -0.00593747, 0.03162669],
    [0.00873798, 0.00591300, -0.99994434, -0.02768631],
    [0.0, 0.0, 0.0, 1.0],
]

MIXED_INTRINSICS = [386.750, 387.233, 330.249, 239.916]
MIXED_DISTORTION = [-0.043, 0.035, -0.001, 0.001]
JUNE_INTRINSICS = [382.9994702701404, 382.2768370977604, 332.5997999732952, 236.54464292967157]
JUNE_DISTORTION = [-0.06073119983113053, 0.0428789308670328, -0.003107075293581619, 0.0010996103095215512]
JUNE_TOFF_S = 0.00015117492772148775
Z_MINUS_DEG_EXACT = -0.75178641


CONDITIONS: dict[str, dict[str, Any]] = {
    "C0": {
        "label": "current_mixed_locked",
        "intrinsics_profile": "current_mixed_fly3_online_terminal",
        "intrinsics": MIXED_INTRINSICS,
        "distortion": MIXED_DISTORTION,
        "tci_profile": "june12_ground",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "C1": {
        "label": "pure_june12_locked",
        "intrinsics_profile": "june12_ground",
        "intrinsics": JUNE_INTRINSICS,
        "distortion": JUNE_DISTORTION,
        "tci_profile": "june12_ground",
        "T_C_I": JUNE_T_C_I,
        "toff_s": JUNE_TOFF_S,
        "z_minus_deg": None,
    },
    "C2": {
        "label": "june12_kd_june12_tci_toff0",
        "intrinsics_profile": "june12_ground",
        "intrinsics": JUNE_INTRINSICS,
        "distortion": JUNE_DISTORTION,
        "tci_profile": "june12_ground",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "C3": {
        "label": "mixed_kd_june12_tci_june12_toff",
        "intrinsics_profile": "current_mixed_fly3_online_terminal",
        "intrinsics": MIXED_INTRINSICS,
        "distortion": MIXED_DISTORTION,
        "tci_profile": "june12_ground",
        "T_C_I": JUNE_T_C_I,
        "toff_s": JUNE_TOFF_S,
        "z_minus_deg": None,
    },
    "C4": {
        "label": "mixed_kd_old_A_fly4_tci_toff0",
        "intrinsics_profile": "current_mixed_fly3_online_terminal",
        "intrinsics": MIXED_INTRINSICS,
        "distortion": MIXED_DISTORTION,
        "tci_profile": "old_A_fly4_exact_artifact",
        "T_C_I": A_FLY4_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "C5": {
        "label": "june12_kd_old_A_fly4_tci_toff0",
        "intrinsics_profile": "june12_ground",
        "intrinsics": JUNE_INTRINSICS,
        "distortion": JUNE_DISTORTION,
        "tci_profile": "old_A_fly4_exact_artifact",
        "T_C_I": A_FLY4_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "C6": {
        "label": "mixed_kd_june12_tci_zminus_toff0",
        "intrinsics_profile": "current_mixed_fly3_online_terminal",
        "intrinsics": MIXED_INTRINSICS,
        "distortion": MIXED_DISTORTION,
        "tci_profile": "june12_ground",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": Z_MINUS_DEG_EXACT,
    },
    "C7": {
        "label": "june12_kd_june12_tci_zminus_toff0",
        "intrinsics_profile": "june12_ground",
        "intrinsics": JUNE_INTRINSICS,
        "distortion": JUNE_DISTORTION,
        "tci_profile": "june12_ground",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": Z_MINUS_DEG_EXACT,
    },
}


FLIGHTS: dict[str, dict[str, Any]] = {
    "fly1": {
        "dataset_win": Path(r"C:\Users\baloney\Desktop\20260517_gsmq_d455_fly1\d455_20260517_174810"),
        "dataset_wsl": PurePosixPath("/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"),
        "gps_win": REPO_WIN / "config/d455_fly1/fc_gps_cam_time.csv",
        "gps_wsl": REPO_WSL / "config/d455_fly1/fc_gps_cam_time.csv",
        "eval_gps_win": Path(
            r"C:\Users\baloney\Desktop\20260517_gsmq_d455_fly1\result"
            r"\fc_rebuild_20260614\gps_from_fc_absolute_cam_time.csv"
        ),
        "fc_win": Path(r"D:\vscode_dir\open_vins\readonly_audits\P4_global_baseline_fullflight_fc_board_calibration_20260714_v3\p4_inputs\fly1_fc_navigation_online.csv"),
        "fc_wsl": PurePosixPath("/mnt/d/vscode_dir/open_vins/readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs/fly1_fc_navigation_online.csv"),
        "start_s": 930.0,
        "until_s": 1300.0,
        "phase_contract": {
            "focus": [1027.0, 1213.0],
            "pre_turn": [1059.0, 1089.0],
            "turn": [1089.0, 1151.0],
            "post_straight_early": [1151.0, 1182.0],
            "post_straight_late": [1182.0, 1213.0],
        },
    },
    "fly3": {
        "dataset_win": Path(r"C:\Users\baloney\Desktop\20260527_gsmq_d455_fly3\d455_20260526_174946"),
        "dataset_wsl": PurePosixPath("/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946"),
        "gps_win": Path(r"C:\Users\baloney\Desktop\20260527_gsmq_d455_fly3\result\fc_rebuild_20260527\gps_from_mems_offset438p0_cam_time.csv"),
        "gps_wsl": PurePosixPath("/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv"),
        "eval_gps_win": Path(r"C:\Users\baloney\Desktop\20260527_gsmq_d455_fly3\result\fc_rebuild_20260527\gps_from_mems_offset438p0_cam_time.csv"),
        "fc_win": Path(r"D:\vscode_dir\open_vins\readonly_audits\P4_global_baseline_fullflight_fc_board_calibration_20260714_v3\p4_inputs\fly3_fc_navigation_online.csv"),
        "fc_wsl": PurePosixPath("/mnt/d/vscode_dir/open_vins/readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs/fly3_fc_navigation_online.csv"),
        "start_s": 618.0,
        "until_s": 947.0,
        "phase_contract": {
            "focus": [663.2, 897.0],
            "pre_turn": [663.2, 693.2],
            "turn": [693.2, 740.0],
            "post_straight_early": [740.0, 818.5],
            "post_straight_late": [818.5, 897.0],
        },
    },
}


ENV_KEYS = {
    "PATH", "LD_LIBRARY_PATH", "LANG", "LANGUAGE", "LC_ALL", "LC_CTYPE",
    "DISPLAY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR", "WSL_DISTRO_NAME",
    "WSL_INTEROP", "WSLENV", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "PYTHONPATH",
}


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def json_dump(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def run_text(argv: list[str], *, cwd: Path | None = None) -> str:
    result = subprocess.run(
        argv, cwd=str(cwd) if cwd else None, check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        encoding="utf-8", errors="replace",
    )
    return result.stdout


def git_snapshot(repo: Path) -> dict[str, Any]:
    return {
        "repo": str(repo),
        "commit": run_text(["git", "-C", str(repo), "rev-parse", "HEAD"]).strip(),
        "branch": run_text(["git", "-C", str(repo), "branch", "--show-current"]).strip(),
        "status_short": run_text(["git", "-C", str(repo), "status", "--short"]).splitlines(),
    }


def relevant_environment(source: dict[str, str] | None = None) -> dict[str, str | None]:
    env = source if source is not None else dict(os.environ)
    selected = {key: env.get(key) for key in sorted(ENV_KEYS)}
    selected.update({key: value for key, value in sorted(env.items()) if key.startswith("P4_")})
    return selected


def capture_wsl_environment() -> dict[str, str | None]:
    if os.name == "posix":
        return relevant_environment()
    raw = subprocess.run(
        ["wsl.exe", "bash", "-lc", "env -0"], check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout
    parsed: dict[str, str] = {}
    for item in raw.decode("utf-8", errors="replace").split("\0"):
        if "=" in item:
            key, value = item.split("=", 1)
            parsed[key] = value
    return relevant_environment(parsed)


def matrix_row(condition_id: str, flight: str) -> dict[str, Any]:
    condition = CONDITIONS[condition_id]
    f = FLIGHTS[flight]
    run_id = f"{condition_id}_{flight}"
    return {
        "run_id": run_id,
        "condition": condition_id,
        "condition_label": condition["label"],
        "flight": flight,
        "start_s": f["start_s"],
        "until_s": f["until_s"],
        "intrinsics_profile": condition["intrinsics_profile"],
        "tci_profile": condition["tci_profile"],
        "toff_s": condition["toff_s"],
        "toff_ms": condition["toff_s"] * 1000.0,
        "z_minus_deg": "" if condition["z_minus_deg"] is None else condition["z_minus_deg"],
        "z_minus_stage": "none" if condition["z_minus_deg"] is None else "post_p4_release_left_rotvec",
        "stride": 12,
        "p5": "off",
        "height_mode": "guarded_gps_z_only",
        "status": "planned",
        "run_dir": str(ROOT_WIN / "runs" / run_id),
    }


def calibration_yaml(condition_id: str) -> str:
    condition = CONDITIONS[condition_id]
    rows = []
    for row in condition["T_C_I"]:
        rows.append("    - [" + ", ".join(f"{float(value):.17g}" for value in row) + "]")
    intrinsics = ", ".join(f"{float(value):.17g}" for value in condition["intrinsics"])
    distortion = ", ".join(f"{float(value):.17g}" for value in condition["distortion"])
    return (
        "%YAML:1.0\n\n"
        f"# P4 yaw calibration factorial {condition_id}: {condition['label']}\n"
        "# All online camera calibration flags remain false in estimator_config.yaml.\n"
        "cam0:\n"
        "  T_cam_imu:\n" + "\n".join(rows) + "\n"
        "  cam_overlaps: []\n"
        "  camera_model: pinhole\n"
        f"  distortion_coeffs: [{distortion}]\n"
        "  distortion_model: radtan\n"
        f"  intrinsics: [{intrinsics}]\n"
        "  resolution: [640, 480]\n"
        "  rostopic: /camera/color/image_raw\n"
        f"  timeshift_cam_imu: {condition['toff_s']:.17g}\n"
    )


def wsl_run_dir(run_id: str) -> PurePosixPath:
    return ROOT_WSL / "runs" / run_id


def command_argv(run_id: str) -> list[str]:
    condition_id, flight = run_id.split("_", 1)
    condition = CONDITIONS[condition_id]
    f = FLIGHTS[flight]
    out = wsl_run_dir(run_id)
    config = out / "config_snapshot/estimator_config.yaml"
    argv = [
        str(ROOT_WSL / "provenance/frozen_binary/run_serial_msckf_ros_free"),
        "--config", str(config),
        "--dataset", str(f["dataset_wsl"]),
        "--gps", str(f["gps_wsl"]),
        "--gps-time-offset", "0",
        "--start-time", str(f["start_s"]),
        "--until-time", str(f["until_s"]),
        "--initialization-mode", "online_multisensor_alignment",
        "--init-from-fc", str(f["fc_wsl"]),
        "--init-from-fc-position-frame", "global_gnav",
        "--online-alignment-release-policy", "practical_navigation_start",
        "--camera-frame-stride", "12",
        "--adaptive-stride-log", str(out / "adaptive_stride.csv"),
        "--visual-flow-curl-diag", str(out / "visual_flow_curl_diag.csv"),
        "--visual-residual-frame-summary", str(out / "visual_residual_frame_summary.csv"),
        "--camera-stride-audit", str(out / "stride_audit.csv"),
        "--state-safety-diag", str(out / "state_safety.csv"),
        "--state-safety-eig-every", "100",
        "--diag-csv", str(out / "diag.csv"),
        "--diag-events", str(out / "events.txt"),
        "--output", str(out / "traj.txt"),
        "--output-raw", str(out / "traj_raw.txt"),
        "--output-nav", str(out / "traj_nav.txt"),
        "--nav-frame-metadata-json", str(out / "nav_frame_metadata.json"),
        "--online-alignment-metadata-json", str(out / "online_alignment_metadata.json"),
        "--yaw-mode", "baseline",
        "--vio-yaw-diag", str(out / "vio_yaw_diag.csv"),
        "--visual-obs-diag", str(out / "visual_observability_diag.csv"),
        "--yaw-update-mechanism-diag", str(out / "yaw_update_mechanism_diag.csv"),
        "--imu-propagation-yaw-diag", str(out / "imu_propagation_yaw_diag.csv"),
        "--slam-feature-yaw-contrib-diag", str(out / "slam_feature_yaw_contrib_diag.csv"),
        "--visual-feature-residual-diag", str(out / "visual_feature_residual_diag.csv"),
    ]
    if condition["z_minus_deg"] is not None:
        argv.extend([
            "--post-alignment-camera-extrinsic-left-rotvec-deg",
            "0.0", "0.0", f"{condition['z_minus_deg']:.8f}",
        ])
    argv.extend([
        "--gps-alt-update",
        "--height-mode", "guarded",
        "--gps-alt-sigma", "2.0",
        "--gps-alt-min-pzz", "0.01",
        "--gps-alt-min-t-after-init", "10",
        "--gps-alt-max-res", "80",
        "--gps-alt-guard-dxy", "0.5",
        "--gps-alt-guard-kxy", "5.0",
        "--gps-alt-coupled-diag", str(out / "gps_alt_coupled_diag.csv"),
        "--no-dashboard",
    ])
    return argv


def dataset_identity_files(flight: str) -> list[Path]:
    f = FLIGHTS[flight]
    dataset = f["dataset_win"]
    return [
        f["gps_win"], f["fc_win"],
        dataset / "imu0/data.csv",
        dataset / "cam0/data.csv",
        dataset / "sync/data.csv",
    ]


def file_identity(path: Path) -> dict[str, Any]:
    value: dict[str, Any] = {"path": str(path), "exists": path.exists()}
    if path.exists() and path.is_file():
        stat = path.stat()
        value.update({
            "size_bytes": stat.st_size,
            "mtime": dt.datetime.fromtimestamp(stat.st_mtime, dt.timezone.utc).astimezone().isoformat(),
            "sha256": sha256(path),
        })
    return value


def write_run_files(
    run_id: str,
    source: dict[str, Any],
    main_source: dict[str, Any],
    binary_hash: str,
    env: dict[str, Any],
) -> None:
    condition_id, flight = run_id.split("_", 1)
    condition = CONDITIONS[condition_id]
    out = ROOT_WIN / "runs" / run_id
    config_dir = out / "config_snapshot"
    config_dir.mkdir(parents=True, exist_ok=False)
    shutil.copy2(REPO_WIN / "baseline/latest/config/estimator_config.yaml", config_dir)
    shutil.copy2(REPO_WIN / "baseline/latest/config/kalibr_imu_chain.yaml", config_dir)
    shutil.copy2(ROOT_WIN / f"calibrations/{condition_id}/kalibr_imucam_chain.yaml", config_dir)

    argv = command_argv(run_id)
    command_line = shlex.join(argv)
    (out / "command.sh").write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\nexec " + command_line + "\n",
        encoding="utf-8", newline="\n",
    )
    (out / "command.txt").write_text(command_line + "\n", encoding="utf-8", newline="\n")
    json_dump(out / "command.argv.json", argv)

    config_files = [
        file_identity(config_dir / "estimator_config.yaml"),
        file_identity(config_dir / "kalibr_imu_chain.yaml"),
        file_identity(config_dir / "kalibr_imucam_chain.yaml"),
    ]
    inputs = [file_identity(path) for path in dataset_identity_files(flight)]
    manifest = {
        "schema": "openvins_p4_yaw_calibration_factorial_run_v1",
        "experiment_id": "P4_yaw_calibration_factorial_20260716_ultra",
        "run_id": run_id,
        "status": "planned",
        "created_at": now_iso(),
        "condition": {
            "id": condition_id,
            **condition,
            "time_offset_semantics": "t_imu=t_cam+shift",
            "z_minus_application_stage": (
                "none" if condition["z_minus_deg"] is None
                else "post_p4_release_left_rotvec; exact prior readonly perturbation"
            ),
        },
        "flight": {
            "id": flight,
            "requested_start_s": FLIGHTS[flight]["start_s"],
            "requested_until_s": FLIGHTS[flight]["until_s"],
            "time_basis": "dataset camera/IMU relative seconds; same basis as existing focus replay",
            "phase_contract": FLIGHTS[flight]["phase_contract"],
        },
        "locked_contract": {
            "initialization": "online_multisensor_alignment / practical_navigation_start",
            "p4_only": True,
            "p5_active": False,
            "adaptive_stride_active": False,
            "adaptive_stride_shadow": False,
            "camera_frame_adaptive_active": False,
            "visual_update_adaptive_active": False,
            "camera_frame_stride": 12,
            "post_alignment_visual_roi": "full",
            "dynamic_roi": False,
            "agl_scale_reset": False,
            "yaw_mode": "baseline",
            "height_mode": "guarded",
            "gps_z_estimator_input": "guarded height contract only",
            "gps_xy_course_yaw_estimator_input": False,
            "gps_horizontal_reference_only": True,
            "future_error_or_truth_gate_input": False,
            "runtime_dashboard": "disabled for bounded factorial batch; offline dashboards required",
        },
        "source": source,
        "analysis_source_snapshot": main_source,
        "binary": {
            "path_windows": str(ROOT_WIN / "provenance/frozen_binary/run_serial_msckf_ros_free"),
            "path_wsl": str(ROOT_WSL / "provenance/frozen_binary/run_serial_msckf_ros_free"),
            "sha256": binary_hash,
        },
        "command": {"argv": argv, "shell": command_line},
        "config_snapshot": config_files,
        "inputs": inputs,
        "environment": env,
        "environment_capture_policy": "allowlisted runtime variables only; unrelated secret-bearing variables excluded",
        "execution": {"status": "not_started"},
    }
    json_dump(out / "run_manifest.json", manifest)


def markdown_matrix(rows: list[dict[str, Any]], source: dict[str, Any], binary_hash: str) -> str:
    condition_lines = []
    for cid, condition in CONDITIONS.items():
        z = "none" if condition["z_minus_deg"] is None else f"{condition['z_minus_deg']:.8f}° post-P4"
        condition_lines.append(
            f"| {cid} | {condition['label']} | {condition['intrinsics_profile']} | "
            f"{condition['tci_profile']} | {condition['toff_s'] * 1000:.8f} | {z} |"
        )
    return f"""# P4 yaw calibration factorial run matrix

## Outcome and completion contract

This is a calibration-only replay on one frozen estimator executable.  It runs
eight conditions on fly1 and fly3 (16 runs), retains every failure, validates
P4 release/state/diagnostic identity before formal analysis, and produces both
GPS-grid start-heading diagnostics and primary absolute `G_nav` evaluation.

- Output root: `{ROOT_WIN}`
- Estimator source commit: `{source['commit']}`
- Estimator branch: `{source['branch']}`
- Frozen binary SHA256: `{binary_hash}`
- Source worktree is dirty and is recorded verbatim in every run manifest.
- No estimator source is edited by this experiment.

## Locked online contract

- P4: `online_multisensor_alignment`, release policy
  `practical_navigation_start`.
- Fixed camera stride 12. P5 active/shadow, adaptive stride, camera-frame
  adaptive sampling, adaptive visual updates, dynamic ROI, and AGL scale reset
  are all disabled.
- Yaw mode remains `baseline`; height remains the existing guarded GPS-Z
  contract. GPS E/N/course/yaw and all future/reference errors remain outside
  online estimation and online gates.
- Runtime dashboards are disabled only for this bounded factorial batch;
  offline interactive dashboards and standard PNG/SVG plots are required.
- C6/C7 reproduce the prior readonly z-minus exactly via
  `--post-alignment-camera-extrinsic-left-rotvec-deg 0 0 -0.75178641`; it is
  applied only after P4 release. P4 fingerprint changes are therefore examined
  separately from backend trajectory changes.

## Calibration conditions

| ID | Label | K/D profile | T_C_I profile | toff ms (`t_imu=t_cam+shift`) | z-minus |
|---|---|---|---|---:|---|
{chr(10).join(condition_lines)}

C7 deliberately uses `toff=0`: C6−C0 and C7−C2 are clean z-minus pairs with
only K/D profile changed between the two pairs. C1−C2 and C3−C0 isolate the
June-12 time offset under June-12 and mixed K/D respectively.

The optional old-T_C_I rows C4/C5 are enabled because the exact artifact was
found at `20260528_gsmq_d455_fly4/result/sigma1_stab_20260610/A_fly4/`.
Its calibration artifact SHA256 is
`CA1913E2B38650E1B15EE95E3FEAD0E2CADD51C85C273ADBF858A21E82B3861B`;
the original run records commit `e52d99cee021a613a6e23367ca6b56a422b561ae`.

## Replay windows and fixed phase definitions

| Flight | Replay request | Time basis | Pre-turn | Turn | Post-turn straight |
|---|---|---|---|---|---|
| fly1 | 930–1300 s | existing focus camera/IMU-relative basis | 1059–1089 | 1089–1151 | 1151–1213 |
| fly3 | 618–947 s | existing screen/focus camera/IMU-relative basis | 663.2–693.2 | 693.2–740 | 740–897 |

The phase boundaries are frozen from existing geometry/event audits before any
factorial result is viewed; they are not selected from candidate errors.

## Validation before evaluation

Each successful process must have non-empty and monotonic `traj_nav`, raw and
bias sidecars with finite q/p/v/bg/ba, a passed frame contract, a released P4
state with no future data or GPS horizontal/course use, finite residual/NIS,
support-count fingerprints, and comparable diagnostic schemas. The stride
audit must report configured stride 12, adaptive modes disabled, zero P5
tracking-only/trigger activity, and guarded height flags unchanged.

Formal analysis uses original GPS update times. The canonical start-heading
analysis preserves scale and uses no full-trajectory fit; the absolute analysis
compares `traj_nav` directly in `G_nav` with no position, heading, SE(2), SE(3),
or Sim(3) post-fit. GPS/FC references remain offline evaluation inputs.

## Hypothesis decisions

- z-minus improves only the mixed-K/D pair and disappears/reverses for
  June-12 K/D: intrinsics–T_C_I compensation.
- z-minus improves both K/D profiles in the same direction on fly1 and fly3:
  candidate T_C_I error.
- C1−C2 and C3−C0 are small: time offset is not the main cause.
- Any within-flight improvement that reverses across fly1/fly3 is rejected as
  a flight-specific compensation.
- Any P4 fingerprint change is reported as initialization re-solve change and
  is not directly attributed to backend calibration behavior.

The machine-readable 16-row plan is `RUN_MATRIX.csv`; each row has a complete
pre-run manifest and a three-file config snapshot under `runs/<run_id>/`.
"""


def prepare() -> None:
    if os.name == "posix":
        raise RuntimeError("prepare must run from Windows so Windows and WSL identities are both frozen")
    if (ROOT_WIN / "RUN_MATRIX.csv").exists():
        raise FileExistsError("RUN_MATRIX.csv already exists; refusing to overwrite the experiment")
    required = [
        ROOT_WIN / "provenance/frozen_binary/run_serial_msckf_ros_free",
        ROOT_WIN / "provenance/frozen_binary/test_online_alignment_initializer",
    ]
    for path in required:
        if not path.exists():
            raise FileNotFoundError(path)

    source = git_snapshot(REPO_WIN)
    main_source = git_snapshot(MAIN_REPO_WIN)
    binary = required[0]
    binary_hash = sha256(binary)
    env = capture_wsl_environment()

    provenance = ROOT_WIN / "provenance"
    (provenance / "source_commit.txt").write_text(source["commit"] + "\n", encoding="utf-8")
    (provenance / "git_status.txt").write_text("\n".join(source["status_short"]) + "\n", encoding="utf-8")
    diff = subprocess.run(
        ["git", "-C", str(REPO_WIN), "diff", "--binary"], check=True,
        stdout=subprocess.PIPE,
    ).stdout
    (provenance / "git_diff.patch").write_bytes(diff)
    json_dump(provenance / "environment.json", env)
    for script_name in [
        "p4_yaw_calibration_factorial_20260716_ultra.py",
        "analyze_p4_yaw_calibration_factorial_20260716_ultra.py",
        "run_p4_yaw_calibration_factorial_20260716_ultra.sh",
    ]:
        shutil.copy2(
            REPO_WIN / "tools" / script_name,
            provenance / "source_snapshots" / script_name,
        )

    for cid in CONDITIONS:
        cdir = ROOT_WIN / "calibrations" / cid
        cdir.mkdir(parents=True, exist_ok=False)
        (cdir / "kalibr_imucam_chain.yaml").write_text(
            calibration_yaml(cid), encoding="utf-8", newline="\n"
        )
        json_dump(cdir / "calibration_contract.json", CONDITIONS[cid])

    rows = [matrix_row(cid, flight) for cid in CONDITIONS for flight in FLIGHTS]
    with (ROOT_WIN / "RUN_MATRIX.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (ROOT_WIN / "RUN_MATRIX.md").write_text(
        markdown_matrix(rows, source, binary_hash), encoding="utf-8", newline="\n"
    )

    for row in rows:
        write_run_files(row["run_id"], source, main_source, binary_hash, env)

    experiment = {
        "schema": "openvins_p4_yaw_calibration_factorial_experiment_v1",
        "created_at": now_iso(),
        "root": str(ROOT_WIN),
        "source": source,
        "analysis_source_snapshot": main_source,
        "frozen_binary_sha256": binary_hash,
        "run_count": len(rows),
        "conditions": CONDITIONS,
        "flights": {
            key: {
                "start_s": value["start_s"],
                "until_s": value["until_s"],
                "phase_contract": value["phase_contract"],
            }
            for key, value in FLIGHTS.items()
        },
        "environment": env,
    }
    json_dump(provenance / "EXPERIMENT_PROVENANCE.json", experiment)

    snapshot_files = [path for path in provenance.rglob("*") if path.is_file()]
    with (provenance / "provenance_file_sha256.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["path", "sha256", "size_bytes"])
        for path in sorted(snapshot_files):
            if path.name == "provenance_file_sha256.csv":
                continue
            writer.writerow([str(path.relative_to(ROOT_WIN)), sha256(path), path.stat().st_size])
    print(f"prepared {len(rows)} runs at {ROOT_WIN}")


def rewrite_planned_commands() -> None:
    """Correct/regenerate command material before any process has started."""
    if os.name == "posix":
        raise RuntimeError("rewrite-planned must run from Windows")
    for condition in CONDITIONS:
        for flight in FLIGHTS:
            run_id = f"{condition}_{flight}"
            path = ROOT_WIN / "runs" / run_id / "run_manifest.json"
            manifest = json.loads(path.read_text(encoding="utf-8"))
            if manifest.get("status") != "planned":
                raise RuntimeError(f"refusing to rewrite non-planned run {run_id}")
            argv = command_argv(run_id)
            command_line = shlex.join(argv)
            out = path.parent
            (out / "command.sh").write_text(
                "#!/usr/bin/env bash\nset -euo pipefail\nexec " + command_line + "\n",
                encoding="utf-8", newline="\n",
            )
            (out / "command.txt").write_text(
                command_line + "\n", encoding="utf-8", newline="\n"
            )
            json_dump(out / "command.argv.json", argv)
            manifest["command"] = {"argv": argv, "shell": command_line}
            manifest["binary"]["path_wsl"] = str(
                ROOT_WSL / "provenance/frozen_binary/run_serial_msckf_ros_free"
            )
            manifest["planned_material_refreshed_at"] = now_iso()
            json_dump(path, manifest)


def refresh_provenance_hashes() -> None:
    provenance = ROOT_WIN / "provenance"
    for script_name in [
        "p4_yaw_calibration_factorial_20260716_ultra.py",
        "analyze_p4_yaw_calibration_factorial_20260716_ultra.py",
        "run_p4_yaw_calibration_factorial_20260716_ultra.sh",
    ]:
        shutil.copy2(
            REPO_WIN / "tools" / script_name,
            provenance / "source_snapshots" / script_name,
        )
    snapshot_files = [path for path in provenance.rglob("*") if path.is_file()]
    with (provenance / "provenance_file_sha256.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["path", "sha256", "size_bytes"])
        for path in sorted(snapshot_files):
            if path.name == "provenance_file_sha256.csv":
                continue
            writer.writerow([str(path.relative_to(ROOT_WIN)), sha256(path), path.stat().st_size])


def refresh_frozen_binaries() -> None:
    """Freeze a just-built runner/test and bind every planned manifest to it."""
    if os.name == "posix":
        raise RuntimeError("refresh-freeze must run from Windows")
    for condition in CONDITIONS:
        for flight in FLIGHTS:
            manifest = json.loads(
                (ROOT_WIN / f"runs/{condition}_{flight}/run_manifest.json").read_text(
                    encoding="utf-8"
                )
            )
            if manifest.get("status") != "planned":
                raise RuntimeError("cannot refresh binary after a run has started")

    source_binary = REPO_WIN / "build_p4_sliding_r1/run_serial_msckf_ros_free"
    source_test = REPO_WIN / "build_p4_sliding_r1/test_online_alignment_initializer"
    frozen_binary = ROOT_WIN / "provenance/frozen_binary/run_serial_msckf_ros_free"
    frozen_test = ROOT_WIN / "provenance/frozen_binary/test_online_alignment_initializer"
    shutil.copy2(source_binary, frozen_binary)
    shutil.copy2(source_test, frozen_test)
    binary_hash = sha256(frozen_binary)
    test_hash = sha256(frozen_test)
    source = git_snapshot(REPO_WIN)
    build_snapshot = {
        "refreshed_at": now_iso(),
        "source": source,
        "runner": file_identity(frozen_binary),
        "preflight_test": file_identity(frozen_test),
        "binary_is_authoritative_for_all_runs": True,
    }
    json_dump(ROOT_WIN / "provenance/build_source_snapshot.json", build_snapshot)
    (ROOT_WIN / "provenance/source_commit.txt").write_text(
        source["commit"] + "\n", encoding="utf-8"
    )
    (ROOT_WIN / "provenance/git_status.txt").write_text(
        "\n".join(source["status_short"]) + "\n", encoding="utf-8"
    )

    rewrite_planned_commands()
    for condition in CONDITIONS:
        for flight in FLIGHTS:
            run_id = f"{condition}_{flight}"
            path = ROOT_WIN / "runs" / run_id / "run_manifest.json"
            manifest = json.loads(path.read_text(encoding="utf-8"))
            manifest["source"] = source
            manifest["binary"]["sha256"] = binary_hash
            manifest["binary"]["preflight_test_sha256"] = test_hash
            manifest["binary"]["frozen_at"] = now_iso()
            json_dump(path, manifest)

    matrix_path = ROOT_WIN / "RUN_MATRIX.md"
    lines = matrix_path.read_text(encoding="utf-8").splitlines()
    lines = [
        f"- Frozen binary SHA256: `{binary_hash}`"
        if line.startswith("- Frozen binary SHA256:") else line
        for line in lines
    ]
    matrix_path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    experiment_path = ROOT_WIN / "provenance/EXPERIMENT_PROVENANCE.json"
    experiment_data = json.loads(experiment_path.read_text(encoding="utf-8"))
    experiment_data["source"] = source
    experiment_data["frozen_binary_sha256"] = binary_hash
    experiment_data["preflight_test_sha256"] = test_hash
    experiment_data["binary_refreshed_at"] = now_iso()
    json_dump(experiment_path, experiment_data)
    refresh_provenance_hashes()
    print(json.dumps({"runner_sha256": binary_hash, "test_sha256": test_hash}))


def load_manifest(run_id: str) -> tuple[Path, dict[str, Any]]:
    path = ROOT / "runs" / run_id / "run_manifest.json"
    return path, json.loads(path.read_text(encoding="utf-8"))


def record_start(run_id: str) -> None:
    path, manifest = load_manifest(run_id)
    current_hash = sha256(ROOT / "provenance/frozen_binary/run_serial_msckf_ros_free")
    if current_hash != manifest["binary"]["sha256"]:
        raise RuntimeError(f"frozen binary hash changed: {current_hash}")
    manifest["environment"] = relevant_environment()
    manifest["execution"] = {
        "status": "running",
        "start_time": now_iso(),
        "pid": os.getpid(),
        "binary_sha256_at_start": current_hash,
    }
    manifest["status"] = "running"
    json_dump(path, manifest)
    json_dump(path.parent / "environment.json", manifest["environment"])


def parse_numeric_rows(path: Path) -> tuple[int, float, float, list[float], int]:
    count = 0
    first_t = math.nan
    last_t = math.nan
    first_values: list[float] = []
    nonfinite = 0
    previous = -math.inf
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        text = line.strip()
        if not text or text.startswith("#"):
            continue
        values = [float(item) for item in text.split()]
        if not values:
            continue
        if count == 0:
            first_t = values[0]
            first_values = values
        if values[0] <= previous:
            raise ValueError(f"non-monotonic timestamp in {path}: {values[0]} <= {previous}")
        previous = values[0]
        last_t = values[0]
        nonfinite += sum(not math.isfinite(value) for value in values)
        count += 1
    return count, first_t, last_t, first_values, nonfinite


def csv_header_and_count(path: Path) -> tuple[list[str], int]:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        return header, sum(1 for _ in reader)


def validate_run(run_id: str) -> dict[str, Any]:
    path, manifest = load_manifest(run_id)
    out = path.parent
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, detail: Any) -> None:
        checks[name] = {"pass": bool(passed), "detail": detail}

    exit_code_path = out / "exit_code.txt"
    exit_code = int(exit_code_path.read_text().strip()) if exit_code_path.exists() else -999
    check("process_exit_code", exit_code == 0, exit_code)
    frame_code_path = out / "frame_contract_validation_exit_code.txt"
    frame_code = int(frame_code_path.read_text().strip()) if frame_code_path.exists() else -999
    check("frame_validator_exit_code", frame_code == 0, frame_code)

    required = [
        "traj_nav.txt", "traj_raw.txt", "traj.txt.bias", "diag.csv", "events.txt",
        "online_alignment_metadata.json", "nav_frame_metadata.json", "stride_audit.csv",
        "visual_residual_frame_summary.csv", "vio_yaw_diag.csv",
        "visual_observability_diag.csv", "yaw_update_mechanism_diag.csv",
        "imu_propagation_yaw_diag.csv", "slam_feature_yaw_contrib_diag.csv",
        "visual_feature_residual_diag.csv", "state_safety.csv",
    ]
    for name in required:
        p = out / name
        check(f"file_{name}", p.exists() and p.stat().st_size > 0, p.stat().st_size if p.exists() else 0)

    trajectories: dict[str, Any] = {}
    for name in ["traj_nav.txt", "traj_raw.txt", "traj.txt.bias"]:
        p = out / name
        if p.exists() and p.stat().st_size:
            try:
                count, first_t, last_t, first_values, nonfinite = parse_numeric_rows(p)
                trajectories[name] = {
                    "rows": count, "first_t": first_t, "last_t": last_t,
                    "first_values": first_values, "nonfinite_values": nonfinite,
                }
                check(f"{name}_nonempty_monotonic_finite", count > 0 and nonfinite == 0, trajectories[name])
            except Exception as exc:  # preserve the exact validation failure
                check(f"{name}_nonempty_monotonic_finite", False, repr(exc))
    if all(name in trajectories for name in ["traj_nav.txt", "traj_raw.txt", "traj.txt.bias"]):
        counts = [trajectories[name]["rows"] for name in ["traj_nav.txt", "traj_raw.txt", "traj.txt.bias"]]
        check("trajectory_row_counts_equal", len(set(counts)) == 1, counts)
        nav_first = trajectories["traj_nav.txt"]["first_values"]
        bias_first = trajectories["traj.txt.bias"]["first_values"]
        check("nav_has_q_p_v", len(nav_first) >= 11, len(nav_first))
        check("bias_has_v_bg_ba", len(bias_first) >= 10, len(bias_first))
        if len(nav_first) >= 11:
            qnorm = math.sqrt(sum(value * value for value in nav_first[7:11]))
            check("first_nav_quaternion_unit", abs(qnorm - 1.0) <= 1e-5, qnorm)

    frame_path = out / "frame_contract_validation.json"
    frame = json.loads(frame_path.read_text(encoding="utf-8")) if frame_path.exists() else {}
    check("frame_contract_pass", frame.get("pass") is True, frame.get("errors", "missing"))

    metadata_path = out / "online_alignment_metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
    expected_toff = float(manifest["condition"]["toff_s"])
    check("p4_released", metadata.get("released_to_openvins") is True, metadata.get("status"))
    check("p4_readiness", metadata.get("readiness_level") in {"NAVIGATION_READY", "FULL_ALIGNMENT_READY"}, metadata.get("readiness_level"))
    check("p4_no_future_data", metadata.get("future_data_used") is False, metadata.get("future_data_used"))
    check("p4_no_gps_xy_course", metadata.get("gps_xy_or_course_used_by_alignment") is False, metadata.get("gps_xy_or_course_used_by_alignment"))
    actual_toff = metadata.get("locked_camera_to_imu_time_offset_s")
    check("p4_locked_toff", actual_toff is not None and abs(float(actual_toff) - expected_toff) <= 1e-12, {"expected": expected_toff, "actual": actual_toff})
    state = metadata.get("state", {})
    state_lengths = {
        key: len(state.get(key, []))
        for key in ["q_Gnav_to_I_xyzw", "p_I_in_Gnav_m", "v_I_in_Gnav_mps", "gyro_bias_rad_s", "accel_bias_mps2"]
    }
    check("p4_state_q_p_v_bg_ba", state_lengths == {
        "q_Gnav_to_I_xyzw": 4, "p_I_in_Gnav_m": 3, "v_I_in_Gnav_mps": 3,
        "gyro_bias_rad_s": 3, "accel_bias_mps2": 3,
    }, state_lengths)
    nis_values = [
        metadata.get("candidate_last_position_nis"), metadata.get("candidate_last_velocity_nis"),
        metadata.get("candidate_max_position_nis"), metadata.get("candidate_max_velocity_nis"),
    ]
    check("p4_nis_finite", all(value is not None and math.isfinite(float(value)) for value in nis_values), nis_values)
    support = metadata.get("candidate_group_supported_update_counts", [])
    check("p4_support_counts_present", isinstance(support, list) and len(support) == 5, support)

    stride_path = out / "stride_audit.csv"
    stride_row: dict[str, str] = {}
    if stride_path.exists():
        with stride_path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
            rows = list(csv.DictReader(stream))
        if rows:
            stride_row = rows[-1]
    check("stride_configured_12", stride_row.get("configured_stride") == "12", stride_row.get("configured_stride"))
    check("visual_adaptive_off", stride_row.get("visual_update_adaptive_enabled") == "0", stride_row.get("visual_update_adaptive_enabled"))
    check("camera_adaptive_off", stride_row.get("camera_frame_adaptive_enabled") == "0", stride_row.get("camera_frame_adaptive_enabled"))
    p5_zero_fields = [
        "p5_tracking_only_frame_count", "p5_tracking_only_observation_drop_count",
        "p5_tracking_only_clone_violation_count", "p5_backend_trigger_count",
        "p5_backend_clone_commit_count", "p5_tracking_gap_change_count",
        "p5_immediate_contraction_count",
    ]
    p5_values = {key: stride_row.get(key) for key in p5_zero_fields}
    check("p5_inactive", all(value is not None and float(value) == 0.0 for value in p5_values.values()), p5_values)

    schemas: dict[str, Any] = {}
    for name in [
        "diag.csv", "visual_residual_frame_summary.csv", "vio_yaw_diag.csv",
        "visual_observability_diag.csv", "yaw_update_mechanism_diag.csv",
        "imu_propagation_yaw_diag.csv", "slam_feature_yaw_contrib_diag.csv",
        "visual_feature_residual_diag.csv", "state_safety.csv",
    ]:
        p = out / name
        if p.exists() and p.stat().st_size:
            try:
                header, count = csv_header_and_count(p)
                schemas[name] = {"columns": header, "rows": count, "sha256": sha256(p)}
                check(f"schema_{name}", count > 0 and len(header) > 1, {"columns": len(header), "rows": count})
            except Exception as exc:
                check(f"schema_{name}", False, repr(exc))

    fingerprint = {
        "run_id": run_id,
        "condition": manifest["condition"]["id"],
        "flight": manifest["flight"]["id"],
        "status": metadata.get("status"),
        "readiness_level": metadata.get("readiness_level"),
        "result_state_timestamp_s": metadata.get("result_state_timestamp_s"),
        "first_openvins_output_time_s": metadata.get("first_openvins_output_time_s"),
        "selected_window_duration_s": metadata.get("selected_window_duration_s"),
        "window_fingerprint": metadata.get("window_fingerprint"),
        "candidate_feedback_state_mask": metadata.get("candidate_feedback_state_mask"),
        "candidate_post_correction_attitude_residual_deg": metadata.get("candidate_post_correction_attitude_residual_deg"),
        "candidate_relative_position_residual_m": metadata.get("candidate_relative_position_residual_m"),
        "candidate_relative_velocity_residual_mps": metadata.get("candidate_relative_velocity_residual_mps"),
        "candidate_last_position_nis": metadata.get("candidate_last_position_nis"),
        "candidate_last_velocity_nis": metadata.get("candidate_last_velocity_nis"),
        "candidate_max_position_nis": metadata.get("candidate_max_position_nis"),
        "candidate_max_velocity_nis": metadata.get("candidate_max_velocity_nis"),
        "candidate_visual_compensated_p95_px": metadata.get("candidate_visual_compensated_p95_px"),
        "candidate_covariance_normalized_residual": metadata.get("candidate_covariance_normalized_residual"),
        "candidate_closed_loop_attitude_correction_deg": metadata.get("candidate_closed_loop_attitude_correction_deg"),
        "candidate_closed_loop_position_correction_m": metadata.get("candidate_closed_loop_position_correction_m"),
        "candidate_closed_loop_velocity_correction_mps": metadata.get("candidate_closed_loop_velocity_correction_mps"),
        "candidate_closed_loop_gyro_bias_correction_rad_s": metadata.get("candidate_closed_loop_gyro_bias_correction_rad_s"),
        "candidate_closed_loop_accel_bias_correction_mps2": metadata.get("candidate_closed_loop_accel_bias_correction_mps2"),
        "candidate_group_supported_update_counts": metadata.get("candidate_group_supported_update_counts"),
        "candidate_group_post_feedback_stable_update_counts": metadata.get("candidate_group_post_feedback_stable_update_counts"),
        "candidate_persistent_error_state": metadata.get("candidate_persistent_error_state"),
        "candidate_persistent_std": metadata.get("candidate_persistent_std"),
        "state": state,
        "covariance_order": metadata.get("covariance_order"),
        "covariance_15x15": metadata.get("covariance_15x15"),
        "locked_camera_to_imu_time_offset_s": actual_toff,
        "future_data_used": metadata.get("future_data_used"),
        "gps_xy_or_course_used_by_alignment": metadata.get("gps_xy_or_course_used_by_alignment"),
    }
    json_dump(out / "p4_release_fingerprint.json", fingerprint)

    passed = all(item["pass"] for item in checks.values())
    result = {
        "schema": "openvins_p4_yaw_calibration_factorial_validation_v1",
        "run_id": run_id,
        "validated_at": now_iso(),
        "pass": passed,
        "checks": checks,
        "trajectory": trajectories,
        "diagnostic_schemas": schemas,
        "stride_audit": stride_row,
        "p4_release_fingerprint": fingerprint,
    }
    json_dump(out / "run_validation.json", result)
    return result


def finalize(run_id: str, exit_code: int, frame_code: int) -> None:
    path, manifest = load_manifest(run_id)
    execution = manifest.setdefault("execution", {})
    execution.update({
        "end_time": now_iso(),
        "exit_code": exit_code,
        "frame_contract_validation_exit_code": frame_code,
        "binary_sha256_at_end": sha256(ROOT / "provenance/frozen_binary/run_serial_msckf_ros_free"),
    })
    try:
        validation = validate_run(run_id)
        valid = validation["pass"]
    except Exception as exc:
        valid = False
        json_dump(path.parent / "run_validation_exception.json", {"error": repr(exc), "time": now_iso()})
    manifest["status"] = "success" if exit_code == 0 and frame_code == 0 and valid else "failed"
    execution["status"] = manifest["status"]
    if manifest["status"] == "failed":
        execution["failure_reason"] = (
            f"process_exit={exit_code}; frame_exit={frame_code}; validation_pass={valid}"
        )
    json_dump(path, manifest)


def status_summary() -> None:
    rows = []
    for cid in CONDITIONS:
        for flight in FLIGHTS:
            run_id = f"{cid}_{flight}"
            path, manifest = load_manifest(run_id)
            execution = manifest.get("execution", {})
            validation_path = path.parent / "run_validation.json"
            validation_pass = None
            if validation_path.exists():
                validation_pass = json.loads(validation_path.read_text(encoding="utf-8")).get("pass")
            rows.append({
                "run_id": run_id,
                "condition": cid,
                "flight": flight,
                "status": manifest.get("status"),
                "exit_code": execution.get("exit_code"),
                "frame_exit_code": execution.get("frame_contract_validation_exit_code"),
                "validation_pass": validation_pass,
                "failure_reason": execution.get("failure_reason", ""),
                "run_dir": str(path.parent),
            })
    out = ROOT / "RUN_STATUS.csv"
    with out.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    json_dump(ROOT / "RUN_STATUS.json", rows)
    for row in rows:
        print(",".join(str(row[key]) for key in ["run_id", "status", "exit_code", "validation_pass"]))


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("prepare")
    sub.add_parser("rewrite-planned")
    sub.add_parser("refresh-freeze")
    sub.add_parser("refresh-provenance")
    start = sub.add_parser("record-start")
    start.add_argument("run_id")
    validate = sub.add_parser("validate-run")
    validate.add_argument("run_id")
    finish = sub.add_parser("finalize")
    finish.add_argument("run_id")
    finish.add_argument("--exit-code", type=int, required=True)
    finish.add_argument("--frame-code", type=int, required=True)
    sub.add_parser("status")
    args = parser.parse_args()
    if args.command == "prepare":
        prepare()
    elif args.command == "rewrite-planned":
        rewrite_planned_commands()
        refresh_provenance_hashes()
    elif args.command == "refresh-freeze":
        refresh_frozen_binaries()
    elif args.command == "refresh-provenance":
        refresh_provenance_hashes()
    elif args.command == "record-start":
        record_start(args.run_id)
    elif args.command == "validate-run":
        result = validate_run(args.run_id)
        print(json.dumps({"run_id": args.run_id, "pass": result["pass"]}))
        return 0 if result["pass"] else 1
    elif args.command == "finalize":
        finalize(args.run_id, args.exit_code, args.frame_code)
    elif args.command == "status":
        status_summary()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
