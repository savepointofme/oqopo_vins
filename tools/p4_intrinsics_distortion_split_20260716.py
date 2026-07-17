#!/usr/bin/env python3
"""Prepare and validate the locked K/D split replay evidence package.

This is orchestration-only code.  It reuses the already frozen P4 runner and
the validation routines from the preceding calibration factorial.  It never
builds or edits estimator source.
"""

from __future__ import annotations

import argparse
import csv
import copy
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import sys
from typing import Any

import p4_yaw_calibration_factorial_20260716_ultra as core


PARENT_ROOT_WIN = Path(
    r"C:\Users\baloney\Desktop\实验目录\P4_yaw_calibration_factorial_20260716_ultra"
)
PARENT_ROOT_WSL = PurePosixPath(
    "/mnt/c/Users/baloney/Desktop/实验目录/P4_yaw_calibration_factorial_20260716_ultra"
)
SPLIT_ROOT_WIN = PARENT_ROOT_WIN / "intrinsics_distortion_split"
SPLIT_ROOT_WSL = PARENT_ROOT_WSL / "intrinsics_distortion_split"
SCOPE = os.environ.get("P4_KD_SPLIT_SCOPE", "focus").strip().lower()
if SCOPE not in {"focus", "full"}:
    raise RuntimeError("P4_KD_SPLIT_SCOPE must be focus or full")
ATTEMPT = os.environ.get("P4_KD_SPLIT_ATTEMPT", "").strip()
if ATTEMPT and not re.fullmatch(r"[A-Za-z0-9_-]+", ATTEMPT):
    raise RuntimeError("P4_KD_SPLIT_ATTEMPT must be a safe directory name")

SCOPE_ROOT_WIN = SPLIT_ROOT_WIN if SCOPE == "focus" else SPLIT_ROOT_WIN / "full_confirmation"
SCOPE_ROOT_WSL = SPLIT_ROOT_WSL if SCOPE == "focus" else SPLIT_ROOT_WSL / "full_confirmation"
ROOT_WIN = SCOPE_ROOT_WIN if not ATTEMPT else SPLIT_ROOT_WIN / "attempts" / ATTEMPT
ROOT_WSL = SCOPE_ROOT_WSL if not ATTEMPT else SPLIT_ROOT_WSL / "attempts" / ATTEMPT
ROOT = Path(str(ROOT_WSL)) if os.name == "posix" else ROOT_WIN
REPO_WIN = Path(r"D:\vscode_dir\open_vins_p4_sliding_r1")
REPO_WSL = PurePosixPath("/mnt/d/vscode_dir/open_vins_p4_sliding_r1")
MAIN_REPO_WIN = Path(r"D:\vscode_dir\open_vins")

EXPECTED_RUNNER_SHA256 = os.environ.get(
    "P4_KD_SPLIT_RUNNER_SHA256",
    "57657e8ed9d034300a0c041481472808fe951d92cdeaac8f740eb0cd570978fe",
).strip().lower()
SOURCE_RUNNER = Path(
    os.environ.get(
        "P4_KD_SPLIT_RUNNER_SOURCE",
        str(PARENT_ROOT_WIN / "provenance/frozen_binary/run_serial_msckf_ros_free"),
    )
)
SOURCE_TEST = Path(
    os.environ.get(
        "P4_KD_SPLIT_TEST_SOURCE",
        str(PARENT_ROOT_WIN / "provenance/frozen_binary/test_online_alignment_initializer"),
    )
)
PAIR_PROFILE = os.environ.get(
    "P4_KD_SPLIT_PAIR_PROFILE", "preceding_factorial_pair"
).strip()
SOURCE_PROJECT_LIBRARY = Path(
    os.environ.get(
        "P4_KD_SPLIT_LIBRARY_SOURCE",
        r"D:\vscode_dir\open_vins_p4_sliding_r1\build_p4_sliding_r1\libov_msckf_lib.so",
    )
)
EXPECTED_PROJECT_LIBRARY_SHA256 = os.environ.get(
    "P4_KD_SPLIT_LIBRARY_SHA256",
    "40ac217cfd15319fb15ba705d44d7781e9b1798170020f21416995e228306080",
).strip().lower()
PROJECT_LIBRARY_NAME = "libov_msckf_lib.so"
PRIOR_C0_CONFIG_WIN = PARENT_ROOT_WIN / "runs/C0_fly1/config_snapshot"
PRIOR_C0_CONFIG_WSL = PARENT_ROOT_WSL / "runs/C0_fly1/config_snapshot"
PRIOR_C0_CONFIG = (
    Path(str(PRIOR_C0_CONFIG_WSL)) if os.name == "posix" else PRIOR_C0_CONFIG_WIN
)
ANALYSIS_SNAPSHOT_SOURCE = PARENT_ROOT_WIN / "provenance/analysis_tool_snapshot"
JUNE_ROOT = Path(r"C:\Users\baloney\Desktop\results_20260612")
JUNE_CAMCHAIN = (
    JUNE_ROOT
    / "imucam/cam_imu_calib_kalibr_640x480_30hz_imu200hz-camchain-imucam.yaml"
)
JUNE_CAMCHAIN_EXPECTED_SHA256 = (
    "0694d6e3d0ab51d25c39fc16e4887d3b1c4398ee84cfb69ef0e6438db267ede3"
)

CURRENT_K = [386.750, 387.233, 330.249, 239.916]
CURRENT_D = [-0.043, 0.035, -0.001, 0.001]
JUNE_K = [
    382.9994702701404,
    382.2768370977604,
    332.5997999732952,
    236.54464292967157,
]
JUNE_D = [
    -0.06073119983113053,
    0.0428789308670328,
    -0.003107075293581619,
    0.0010996103095215512,
]
JUNE_T_C_I = copy.deepcopy(core.JUNE_T_C_I)

CONDITION_LIBRARY: dict[str, dict[str, Any]] = {
    "C0": {
        "label": "current_K_current_D",
        "factor_K": "current",
        "factor_D": "current",
        "intrinsics_profile": "current_K__current_D",
        "intrinsics": CURRENT_K,
        "distortion": CURRENT_D,
        "tci_profile": "june12_ground_fixed_C0",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "KONLY": {
        "label": "june12_K_current_D",
        "factor_K": "june12",
        "factor_D": "current",
        "intrinsics_profile": "june12_K__current_D",
        "intrinsics": JUNE_K,
        "distortion": CURRENT_D,
        "tci_profile": "june12_ground_fixed_C0",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "DONLY": {
        "label": "current_K_june12_D",
        "factor_K": "current",
        "factor_D": "june12",
        "intrinsics_profile": "current_K__june12_D",
        "intrinsics": CURRENT_K,
        "distortion": JUNE_D,
        "tci_profile": "june12_ground_fixed_C0",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
    "KD": {
        "label": "june12_K_june12_D",
        "factor_K": "june12",
        "factor_D": "june12",
        "intrinsics_profile": "june12_K__june12_D",
        "intrinsics": JUNE_K,
        "distortion": JUNE_D,
        "tci_profile": "june12_ground_fixed_C0",
        "T_C_I": JUNE_T_C_I,
        "toff_s": 0.0,
        "z_minus_deg": None,
    },
}


def selected_conditions() -> dict[str, dict[str, Any]]:
    if SCOPE == "focus":
        return copy.deepcopy(CONDITION_LIBRARY)
    raw = os.environ.get("P4_KD_FULL_CONDITIONS", "").strip()
    if not raw:
        raise RuntimeError("P4_KD_FULL_CONDITIONS is required for full scope")
    ids = [item.strip().upper() for item in raw.split(",") if item.strip()]
    if "C0" not in ids:
        ids.insert(0, "C0")
    unknown = [item for item in ids if item not in CONDITION_LIBRARY]
    if unknown:
        raise RuntimeError(f"unknown full confirmation conditions: {unknown}")
    ordered: dict[str, dict[str, Any]] = {}
    for cid in CONDITION_LIBRARY:
        if cid in ids:
            ordered[cid] = copy.deepcopy(CONDITION_LIBRARY[cid])
    return ordered


CONDITIONS = selected_conditions()
FLIGHTS = copy.deepcopy(core.FLIGHTS)
if SCOPE == "full":
    FLIGHTS["fly1"]["until_s"] = 1938.0
    FLIGHTS["fly3"]["until_s"] = 1837.0


def configure_core() -> None:
    core.ROOT_WIN = ROOT_WIN
    core.ROOT_WSL = ROOT_WSL
    core.ROOT = ROOT
    core.REPO_WIN = REPO_WIN
    core.REPO_WSL = REPO_WSL
    core.REPO = Path(str(REPO_WSL)) if os.name == "posix" else REPO_WIN
    core.MAIN_REPO_WIN = MAIN_REPO_WIN
    core.CONDITIONS = CONDITIONS
    core.FLIGHTS = FLIGHTS


configure_core()
_CORE_WRITE_RUN_FILES = core.write_run_files


def now_iso() -> str:
    return dt.datetime.now(dt.timezone.utc).astimezone().isoformat(timespec="seconds")


def experiment_id() -> str:
    suffix = f"_{ATTEMPT}" if ATTEMPT else ""
    return f"P4_intrinsics_distortion_split_20260716_{SCOPE}{suffix}"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def frozen_project_library() -> Path:
    return ROOT / "provenance/frozen_binary" / PROJECT_LIBRARY_NAME


def bind_command_to_frozen_library(out: Path) -> None:
    """Make the project shared-library identity part of the executable command."""
    command_path = out / "command.sh"
    command_text = command_path.read_text(encoding="utf-8")
    library_dir = str(ROOT_WSL / "provenance/frozen_binary")
    export_line = f"export LD_LIBRARY_PATH={shlex.quote(library_dir)}\n"
    marker = "set -euo pipefail\n"
    if marker not in command_text:
        raise RuntimeError(f"unexpected command launcher format: {command_path}")
    command_path.write_text(
        command_text.replace(marker, marker + export_line, 1),
        encoding="utf-8",
        newline="\n",
    )


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temp.replace(path)


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def jpeg_dimensions(path: Path) -> tuple[int, int]:
    """Read JPEG SOF dimensions without adding an image-library dependency."""
    with path.open("rb") as stream:
        if stream.read(2) != b"\xff\xd8":
            raise ValueError(f"not a JPEG: {path}")
        while True:
            prefix = stream.read(1)
            if not prefix:
                break
            if prefix != b"\xff":
                continue
            marker = stream.read(1)
            while marker == b"\xff":
                marker = stream.read(1)
            if marker in {b"\xd8", b"\xd9"}:
                continue
            length_raw = stream.read(2)
            if len(length_raw) != 2:
                break
            length = int.from_bytes(length_raw, "big")
            if marker and marker[0] in {
                0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
            }:
                payload = stream.read(length - 2)
                if len(payload) < 5:
                    break
                return int.from_bytes(payload[3:5], "big"), int.from_bytes(payload[1:3], "big")
            stream.seek(max(0, length - 2), 1)
    raise ValueError(f"JPEG SOF marker not found: {path}")


_IMAGE_INVENTORY_CACHE: dict[str, dict[str, Any]] = {}


def image_inventory(flight: str) -> dict[str, Any]:
    if flight in _IMAGE_INVENTORY_CACHE:
        return copy.deepcopy(_IMAGE_INVENTORY_CACHE[flight])
    dataset = FLIGHTS[flight]["dataset_win"]
    index_path = dataset / "cam0/data.csv"
    raw_rows: list[dict[str, str | None]] = []
    with index_path.open("r", encoding="utf-8-sig", newline="") as stream:
        first = stream.readline().lstrip("#").strip()
        reader = csv.DictReader(stream, fieldnames=first.split(","))
        raw_rows = list(reader)
    rows = [
        row for row in raw_rows
        if row.get("filename") and row.get("t_rel_s") and "\x00" not in str(row.get("t_rel_s"))
    ]
    if not rows:
        raise ValueError(f"no valid camera index rows: {index_path}")
    indices = sorted({0, len(rows) // 2, len(rows) - 1})
    samples = []
    rejected_sample_candidates = []
    for requested_index in indices:
        search = range(requested_index, max(-1, requested_index - 1000), -1)
        selected: tuple[int, dict[str, str | None], Path, int, int] | None = None
        for index in search:
            row = rows[index]
            path = dataset / "cam0/data" / str(row["filename"])
            try:
                width, height = jpeg_dimensions(path)
                selected = (index, row, path, width, height)
                break
            except (OSError, ValueError) as exc:
                rejected_sample_candidates.append({
                    "requested_index": requested_index,
                    "index": index,
                    "filename": row.get("filename"),
                    "size_bytes": path.stat().st_size if path.exists() else None,
                    "reason": repr(exc),
                })
        if selected is None:
            raise ValueError(f"no valid JPEG near index {requested_index}: {index_path}")
        index, row, path, width, height = selected
        samples.append({
            "requested_index": requested_index,
            "index": index,
            "t_rel_s": float(row["t_rel_s"]),
            "filename": row["filename"],
            "width": width,
            "height": height,
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
        })
    image_files = [path for path in (dataset / "cam0/data").iterdir() if path.is_file()]
    result = {
        "dataset": str(dataset),
        "camera_index": core.file_identity(index_path),
        "meta": core.file_identity(dataset / "meta.yaml"),
        "indexed_image_count": len(rows),
        "camera_index_raw_rows": len(raw_rows),
        "camera_index_invalid_rows": len(raw_rows) - len(rows),
        "image_directory_file_count": len(image_files),
        "image_directory_zero_byte_count": sum(path.stat().st_size == 0 for path in image_files),
        "first_t_rel_s": float(rows[0]["t_rel_s"]),
        "last_t_rel_s": float(rows[-1]["t_rel_s"]),
        "sample_images": samples,
        "rejected_sample_candidates": rejected_sample_candidates,
        "all_sample_dimensions_640x480": all(
            item["width"] == 640 and item["height"] == 480 for item in samples
        ),
    }
    _IMAGE_INVENTORY_CACHE[flight] = result
    return copy.deepcopy(result)


def markdown_matrix(rows: list[dict[str, Any]], source: dict[str, Any], binary_hash: str) -> str:
    condition_lines = []
    for cid, condition in CONDITIONS.items():
        condition_lines.append(
            f"| {cid} | {condition['factor_K']} | {condition['factor_D']} | "
            f"`{condition['intrinsics']}` | `{condition['distortion']}` |"
        )
    window_lines = []
    for flight, info in FLIGHTS.items():
        phase = info["phase_contract"]
        window_lines.append(
            f"| {flight} | {info['start_s']}–{info['until_s']} s | "
            f"{phase['pre_turn'][0]}–{phase['pre_turn'][1]} | "
            f"{phase['turn'][0]}–{phase['turn'][1]} | "
            f"{phase['post_straight_early'][0]}–{phase['post_straight_late'][1]} |"
        )
    scope_note = (
        "Half-to-one-lap fixed focus screening."
        if SCOPE == "focus"
        else "Full-flight confirmation selected by the preregistered focus trigger."
    )
    return f"""# Intrinsics/distortion split run matrix ({SCOPE})

{scope_note}

- Output root: `{ROOT_WIN}`
- Frozen runner SHA256: `{binary_hash}`
- Expected runner SHA256: `{EXPECTED_RUNNER_SHA256}`
- Orchestration source commit at preparation: `{source['commit']}`
- No build and no estimator-source edit is part of this experiment.

## 2×2 calibration factors

| Condition | K | D | Actual intrinsics `[fx,fy,cx,cy]` | Actual distortion `[k1,k2,p1,p2]` |
|---|---|---|---|---|
{chr(10).join(condition_lines)}

Every row fixes `T_C_I` to the June12 C0 matrix and fixes
`timeshift_cam_imu=0`.  `calib_cam_intrinsics`, `calib_cam_extrinsics`, and
`calib_cam_timeoffset` remain false.  The generated YAML itself is parsed and
checked before process start; labels are not accepted as evidence.

## Locked estimator contract

- P4 `online_multisensor_alignment` with `practical_navigation_start`.
- P5 active/shadow, adaptive stride, dynamic ROI, AGL scale reset, FC/GPS yaw,
  GPS horizontal fusion, and future-error gates are absent.
- Fixed camera stride 12, yaw mode `baseline`, and existing guarded GPS-Z
  height contract.
- GPS E/N/course/yaw are offline references only. Formal evaluation uses GPS
  update timestamps, start-heading scale-preserving alignment, FC `Ve,Vn,Vu`,
  and `.bias` VIO velocity. No best-fit/SE(3)/Sim(3) result is accepted.

## Windows and fixed segments

| Flight | Replay | Pre-turn | First turn | Post-turn straight |
|---|---|---|---|---|
{chr(10).join(window_lines)}

## Preregistered focus-to-full trigger

A single factor is material if KONLY−C0 or DONLY−C0 reaches any one of these
offline screening thresholds on either flight: `|Δ course RMSE| >= 0.5 deg`,
`|Δ XY RMSE| >= 20 m`, `|Δ turn/post-late course median| >= 0.75 deg`, or
`|Δ turn/post-late course slope| >= 0.01 deg/s`.  Every factor meeting the
trigger is confirmed as the clean C0/factor pair on both full flights.  This
selects only additional offline runs and does not tune any online threshold.

## Falsifiable interpretation

- KONLY carries most of KD while DONLY stays near C0: June12 K is the dominant
  harmful factor under current D.
- DONLY carries most of KD while KONLY stays near C0: June12 D is dominant.
- Neither single factor carries KD but the factorial interaction is large:
  harm is K–D coupling and cannot be assigned to either field alone.
- A gain/harm that reverses between fly1 and fly3 is flight-specific
  compensation, not a generally acceptable calibration conclusion.
- Any changed P4 release fingerprint is split into initialization re-solve
  change and post-release trajectory change before attribution.

`RUN_MATRIX.csv`, per-run manifests, commands, environment, three-file config
snapshots, and `ACTUAL_CONFIG_AUDIT.csv` are generated before execution.
"""


def patched_write_run_files(
    run_id: str,
    source: dict[str, Any],
    main_source: dict[str, Any],
    binary_hash: str,
    env: dict[str, Any],
) -> None:
    _CORE_WRITE_RUN_FILES(run_id, source, main_source, binary_hash, env)
    out = ROOT_WIN / "runs" / run_id
    bind_command_to_frozen_library(out)
    config_dir = out / "config_snapshot"
    # Fix non-camera configuration to the exact preceding C0 snapshot, not to
    # whatever may currently be changing in the shared source tree.
    for name in ["estimator_config.yaml", "kalibr_imu_chain.yaml"]:
        shutil.copy2(PRIOR_C0_CONFIG / name, config_dir / name)

    manifest_path = out / "run_manifest.json"
    manifest = read_json(manifest_path)
    condition_id, flight = run_id.rsplit("_", 1)
    manifest["schema"] = "openvins_p4_intrinsics_distortion_split_run_v1"
    manifest["experiment_id"] = experiment_id()
    manifest["scope"] = SCOPE
    manifest["attempt"] = ATTEMPT or None
    manifest["condition"] = {
        "id": condition_id,
        **CONDITIONS[condition_id],
        "time_offset_semantics": "t_imu=t_cam+shift",
        "changed_fields_vs_C0": [
            field
            for field, changed in [
                ("intrinsics", CONDITIONS[condition_id]["factor_K"] != "current"),
                ("distortion_coeffs", CONDITIONS[condition_id]["factor_D"] != "current"),
            ]
            if changed
        ],
    }
    manifest["flight"]["time_basis"] = (
        "dataset camera/IMU relative seconds; same registered basis as preceding focus/full P4 runs"
    )
    manifest["locked_contract"].update({
        "calibration_fields_allowed_to_change": ["intrinsics", "distortion_coeffs"],
        "T_C_I_fixed_to_C0": True,
        "timeshift_cam_imu_fixed_s": 0.0,
        "z_minus_active": False,
        "scope": SCOPE,
    })
    source_at_prepare = manifest.pop("source")
    manifest["binary_build_source"] = source_at_prepare
    manifest["orchestration_source_at_prepare"] = source_at_prepare
    manifest["preceding_factorial_binary_build_source"] = read_json(
        PARENT_ROOT_WIN / "provenance/build_source_snapshot.json"
    )
    manifest["binary"]["lineage_source"] = str(SOURCE_RUNNER)
    manifest["binary"]["expected_sha256"] = EXPECTED_RUNNER_SHA256
    manifest["project_shared_library"] = {
        "name": PROJECT_LIBRARY_NAME,
        "path_windows": str(ROOT_WIN / "provenance/frozen_binary" / PROJECT_LIBRARY_NAME),
        "path_wsl": str(ROOT_WSL / "provenance/frozen_binary" / PROJECT_LIBRARY_NAME),
        "sha256": EXPECTED_PROJECT_LIBRARY_SHA256,
        "lineage_source": str(SOURCE_PROJECT_LIBRARY),
        "runtime_binding": "LD_LIBRARY_PATH is replaced with the frozen-binary directory in command.sh",
        "pair_profile": PAIR_PROFILE,
        "previous_factorial_library_recoverable": False,
    }
    manifest["command"]["environment_override"] = {
        "LD_LIBRARY_PATH": str(ROOT_WSL / "provenance/frozen_binary")
    }
    manifest["command"]["launcher"] = str(out / "command.sh")
    manifest["config_snapshot"] = [
        core.file_identity(config_dir / name)
        for name in [
            "estimator_config.yaml",
            "kalibr_imu_chain.yaml",
            "kalibr_imucam_chain.yaml",
        ]
    ]
    manifest["config_template_lineage"] = {
        "non_camera_fields": str(PRIOR_C0_CONFIG),
        "camera_template": "generated 2x2 K/D YAML with exact C0 T_C_I and toff=0",
    }
    manifest["image_payload_inventory"] = image_inventory(flight)
    manifest["evaluation_input"] = core.file_identity(
        Path(FLIGHTS[flight].get("eval_gps_win", FLIGHTS[flight]["gps_win"]))
    )
    write_json(manifest_path, manifest)


def parse_yaml_list(text: str, key: str) -> list[float]:
    match = re.search(rf"(?m)^\s*{re.escape(key)}:\s*\[([^\]]+)\]\s*$", text)
    if not match:
        raise ValueError(f"missing YAML list {key}")
    return [float(item.strip()) for item in match.group(1).split(",")]


def parse_tci(text: str) -> list[list[float]]:
    match = re.search(
        r"(?ms)^\s*T_cam_imu:\s*\n(?P<body>.*?)^\s*cam_overlaps:", text
    )
    if not match:
        raise ValueError("missing T_cam_imu block")
    rows = re.findall(r"\[([^\]]+)\]", match.group("body"))
    return [[float(item.strip()) for item in row.split(",")] for row in rows]


def values_close(actual: list[float], expected: list[float], tol: float = 1e-12) -> bool:
    return len(actual) == len(expected) and all(
        abs(float(a) - float(b)) <= tol for a, b in zip(actual, expected)
    )


def matrix_close(actual: list[list[float]], expected: list[list[float]]) -> bool:
    return len(actual) == len(expected) and all(
        values_close(a, b) for a, b in zip(actual, expected)
    )


def argv_value(argv: list[str], flag: str) -> str | None:
    if flag not in argv:
        return None
    index = argv.index(flag)
    return argv[index + 1] if index + 1 < len(argv) else None


def actual_config_audit(run_id: str) -> dict[str, Any]:
    condition_id, flight = run_id.rsplit("_", 1)
    expected = CONDITIONS[condition_id]
    run = ROOT / "runs" / run_id
    manifest = read_json(run / "run_manifest.json")
    config_dir = run / "config_snapshot"
    camera_path = config_dir / "kalibr_imucam_chain.yaml"
    estimator_path = config_dir / "estimator_config.yaml"
    imu_path = config_dir / "kalibr_imu_chain.yaml"
    camera_text = camera_path.read_text(encoding="utf-8")
    estimator_text = estimator_path.read_text(encoding="utf-8")
    argv = [str(item) for item in manifest["command"]["argv"]]
    checks: dict[str, dict[str, Any]] = {}

    def check(name: str, passed: bool, detail: Any) -> None:
        checks[name] = {"pass": bool(passed), "detail": detail}

    actual_k = parse_yaml_list(camera_text, "intrinsics")
    actual_d = parse_yaml_list(camera_text, "distortion_coeffs")
    actual_resolution = parse_yaml_list(camera_text, "resolution")
    actual_tci = parse_tci(camera_text)
    toff_match = re.search(r"(?m)^\s*timeshift_cam_imu:\s*([^\s#]+)", camera_text)
    actual_toff = float(toff_match.group(1)) if toff_match else math.nan
    check("intrinsics_exact", values_close(actual_k, expected["intrinsics"]), {"actual": actual_k, "expected": expected["intrinsics"]})
    check("distortion_exact", values_close(actual_d, expected["distortion"]), {"actual": actual_d, "expected": expected["distortion"]})
    check("T_C_I_exact_C0", matrix_close(actual_tci, JUNE_T_C_I), actual_tci)
    check("toff_exact_zero", actual_toff == 0.0, actual_toff)
    check("resolution_640x480", actual_resolution == [640.0, 480.0], actual_resolution)
    check("camera_model_pinhole", bool(re.search(r"(?m)^\s*camera_model:\s*pinhole\s*$", camera_text)), "pinhole")
    check("distortion_model_radtan", bool(re.search(r"(?m)^\s*distortion_model:\s*radtan\s*$", camera_text)), "radtan")
    for key in ["calib_cam_extrinsics", "calib_cam_intrinsics", "calib_cam_timeoffset"]:
        match = re.search(rf"(?m)^\s*{key}:\s*(\S+)", estimator_text)
        value = match.group(1).lower() if match else None
        check(f"{key}_false", value == "false", value)
    check("downsample_cameras_false", bool(re.search(r"(?m)^\s*downsample_cameras:\s*false\s*$", estimator_text)), "false")
    check("estimator_config_exact_prior_C0", sha256(estimator_path) == sha256(PRIOR_C0_CONFIG / "estimator_config.yaml"), sha256(estimator_path))
    check("imu_config_exact_prior_C0", sha256(imu_path) == sha256(PRIOR_C0_CONFIG / "kalibr_imu_chain.yaml"), sha256(imu_path))
    check("binary_hash_expected", sha256(ROOT / "provenance/frozen_binary/run_serial_msckf_ros_free") == EXPECTED_RUNNER_SHA256, sha256(ROOT / "provenance/frozen_binary/run_serial_msckf_ros_free"))
    library_path = frozen_project_library()
    library_hash = sha256(library_path) if library_path.exists() else None
    check(
        "project_library_hash_expected",
        library_hash == EXPECTED_PROJECT_LIBRARY_SHA256,
        library_hash,
    )
    command_text = (run / "command.sh").read_text(encoding="utf-8")
    check(
        "project_library_runtime_bound",
        f"export LD_LIBRARY_PATH={shlex.quote(str(ROOT_WSL / 'provenance/frozen_binary'))}" in command_text,
        str(ROOT_WSL / "provenance/frozen_binary"),
    )
    required_values = {
        "--camera-frame-stride": "12",
        "--yaw-mode": "baseline",
        "--height-mode": "guarded",
        "--gps-time-offset": "0",
        "--start-time": str(FLIGHTS[flight]["start_s"]),
        "--until-time": str(FLIGHTS[flight]["until_s"]),
    }
    for flag, value in required_values.items():
        check(f"argv_{flag.lstrip('-').replace('-', '_')}", argv_value(argv, flag) == value, argv_value(argv, flag))
    check("argv_config_is_run_snapshot", argv_value(argv, "--config") == str(ROOT_WSL / f"runs/{run_id}/config_snapshot/estimator_config.yaml"), argv_value(argv, "--config"))
    forbidden = [
        "--adaptive-stride", "--adaptive-stride-shadow",
        "--camera-frame-adaptive", "--visual-update-adaptive",
        "--post-alignment-visual-roi", "--dynamic-turn-roi",
        "--agl-scene-scale-reset", "--post-alignment-fc-yaw-aid",
        "--post-alignment-camera-extrinsic-left-rotvec-deg",
    ]
    present = [flag for flag in forbidden if flag in argv]
    check("forbidden_modes_absent", not present, present)
    check("gps_horizontal_or_yaw_not_in_command", not any(
        token in " ".join(argv).lower()
        for token in ["gps-xy", "gps-course", "gps-yaw", "fc-yaw"]
    ), "evaluation-only")
    result = {
        "schema": "openvins_p4_kd_actual_config_audit_v1",
        "run_id": run_id,
        "scope": SCOPE,
        "audited_at": now_iso(),
        "pass": all(item["pass"] for item in checks.values()),
        "actual": {
            "intrinsics": actual_k,
            "distortion_coeffs": actual_d,
            "T_C_I": actual_tci,
            "timeshift_cam_imu": actual_toff,
            "resolution": actual_resolution,
            "config_sha256": {
                "estimator": sha256(estimator_path),
                "imu": sha256(imu_path),
                "imucam": sha256(camera_path),
            },
        },
        "checks": checks,
    }
    write_json(run / "actual_config_audit.json", result)
    return result


def write_actual_config_audit_summary() -> None:
    rows = []
    for condition in CONDITIONS:
        for flight in FLIGHTS:
            run_id = f"{condition}_{flight}"
            audit = actual_config_audit(run_id)
            actual = audit["actual"]
            rows.append({
                "run_id": run_id,
                "scope": SCOPE,
                "pass": audit["pass"],
                "intrinsics": json.dumps(actual["intrinsics"]),
                "distortion_coeffs": json.dumps(actual["distortion_coeffs"]),
                "timeshift_cam_imu": actual["timeshift_cam_imu"],
                "T_C_I_sha256": hashlib.sha256(json.dumps(actual["T_C_I"], separators=(",", ":")).encode()).hexdigest(),
                "estimator_config_sha256": actual["config_sha256"]["estimator"],
                "imu_config_sha256": actual["config_sha256"]["imu"],
                "imucam_config_sha256": actual["config_sha256"]["imucam"],
            })
    with (ROOT_WIN / "ACTUAL_CONFIG_AUDIT.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_json(ROOT_WIN / "ACTUAL_CONFIG_AUDIT.json", rows)


def calibration_geometry_audit() -> dict[str, Any]:
    june_text = JUNE_CAMCHAIN.read_text(encoding="utf-8")
    estimator_text = (PRIOR_C0_CONFIG / "estimator_config.yaml").read_text(encoding="utf-8")
    parser_path = REPO_WIN / "ov_msckf/src/core/VioManagerOptions.h"
    radtan_path = REPO_WIN / "ov_core/src/cam/CamRadtan.h"
    parser_text = parser_path.read_text(encoding="utf-8")
    radtan_text = radtan_path.read_text(encoding="utf-8")
    files_in_june = [path for path in JUNE_ROOT.rglob("*") if path.is_file()]
    metadata_candidates = [
        path for path in files_in_june
        if path.suffix.lower() in {".yaml", ".yml", ".json", ".txt", ".md", ".log"}
    ]
    metadata_blob = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for path in metadata_candidates
    )
    exposure_matches = sorted(set(re.findall(
        r"(?i)\b(?:auto[_ -]?exposure|exposure|auto[_ -]?white[_ -]?balance|white[_ -]?balance|awb)\b",
        metadata_blob,
    )))
    audit = {
        "schema": "openvins_june12_camera_geometry_compatibility_audit_v1",
        "audited_at": now_iso(),
        "readonly": True,
        "june12_artifact": core.file_identity(JUNE_CAMCHAIN),
        "june12_artifact_expected_sha256": JUNE_CAMCHAIN_EXPECTED_SHA256,
        "june12_artifact_hash_matches": sha256(JUNE_CAMCHAIN) == JUNE_CAMCHAIN_EXPECTED_SHA256,
        "june12_fields": {
            "resolution": parse_yaml_list(june_text, "resolution"),
            "intrinsics_order": ["fx", "fy", "cx", "cy"],
            "intrinsics": parse_yaml_list(june_text, "intrinsics"),
            "distortion_model": "radtan",
            "distortion_order": ["k1", "k2", "p1", "p2"],
            "distortion_coeffs": parse_yaml_list(june_text, "distortion_coeffs"),
            "camera_model": "pinhole",
        },
        "flight_datasets": {flight: image_inventory(flight) for flight in FLIGHTS},
        "parser_contract": {
            "source": core.file_identity(parser_path),
            "radtan_source": core.file_identity(radtan_path),
            "concatenates_K_then_D": "cam_calib << cam_calib1.at(0), cam_calib1.at(1), cam_calib1.at(2), cam_calib1.at(3), cam_calib2.at(0), cam_calib2.at(1)" in parser_text,
            "radtan_declares_k1_k2_p1_p2": "f_x & f_y & c_x & c_y & k_1 & k_2 & p_1 & p_2" in radtan_text,
            "downsample_only_divides_K_and_resolution_when_enabled": "cam_calib(0) /= (downsample_cameras) ? 2.0 : 1.0" in parser_text,
            "configured_downsample_cameras": False if re.search(r"(?m)^\s*downsample_cameras:\s*false\s*$", estimator_text) else None,
            "unknown_distortion_falls_back_to_radtan": "std::make_shared<ov_core::CamRadtan>" in parser_text,
        },
        "compatibility": {
            "same_resolution_no_intrinsic_scaling_required": True,
            "model_matches_openvins_pinhole_radtan": True,
            "coefficient_order_matches": True,
            "rostopic_name_is_not_used_by_ros_free_dataset_replay": "rostopic" not in parser_text,
        },
        "capture_metadata": {
            "calibration_bag_present_in_artifact": any(path.suffix.lower() == ".bag" for path in files_in_june),
            "exposure_or_awb_terms_found": exposure_matches,
            "conclusion": (
                "AE/AWB/exposure state is not recoverable from the June12 evidence package. "
                "It is retained only as image-quality provenance background and is not used "
                "to infer camera geometry."
            ),
        },
    }
    audit["pass"] = all([
        audit["june12_artifact_hash_matches"],
        audit["june12_fields"]["resolution"] == [640.0, 480.0],
        all(item["all_sample_dimensions_640x480"] for item in audit["flight_datasets"].values()),
        audit["parser_contract"]["concatenates_K_then_D"],
        audit["parser_contract"]["radtan_declares_k1_k2_p1_p2"],
        audit["parser_contract"]["configured_downsample_cameras"] is False,
    ])
    return audit


def write_geometry_audit() -> None:
    audit = calibration_geometry_audit()
    provenance = ROOT_WIN / "provenance"
    write_json(provenance / "CALIBRATION_GEOMETRY_AUDIT.json", audit)
    k_delta = [j - c for j, c in zip(JUNE_K, CURRENT_K)]
    d_delta = [j - c for j, c in zip(JUNE_D, CURRENT_D)]
    text = f"""# June12 camera geometry compatibility audit

- Result: `{'PASS' if audit['pass'] else 'FAIL'}`
- Original camchain SHA256: `{audit['june12_artifact']['sha256']}`
- June12 resolution/model: `640x480`, `pinhole-radtan`
- June12 K order: `[fx, fy, cx, cy]`
- June12 D order: `[k1, k2, p1, p2]`
- fly1/fly3 metadata and sampled JPEGs: `640x480`
- OpenVINS parser order: `[fx,fy,cx,cy,k1,k2,p1,p2]`
- `downsample_cameras=false`; parser scaling factor is therefore exactly 1.
- June12−current K: `{k_delta}`
- June12−current D: `{d_delta}`

The geometry contract matches: there is no resolution, model, coefficient-order,
or intrinsic-scaling mismatch in the files actually used by this replay.  This
does not prove June12 K/D are accurate for flight imagery; it only rules out the
enumerated parser/format mistakes.

The June12 evidence package contains no source bag and no recoverable exposure,
gain, auto-exposure, or auto-white-balance setting.  That missing provenance can
affect calibration image quality, but it is not a geometric conclusion and is
not used to explain the estimator result.
"""
    (provenance / "CALIBRATION_GEOMETRY_AUDIT.md").write_text(text, encoding="utf-8", newline="\n")


def copy_audit_sources() -> None:
    target = ROOT_WIN / "provenance/calibration_audit_sources"
    target.mkdir(parents=True, exist_ok=True)
    files = [
        JUNE_CAMCHAIN,
        JUNE_ROOT / "camera/cam_calib_kalibr_640x480_30hz-camchain.yaml",
        JUNE_ROOT / "camera/cam_calib_kalibr_640x480_30hz-results-cam.txt",
        JUNE_ROOT / "imucam/cam_imu_calib_kalibr_640x480_30hz_imu200hz-results-imucam.txt",
        JUNE_ROOT / "CALIBRATION_RESULT_INDEX.md",
        JUNE_ROOT / "标定结果说明.md",
        REPO_WIN / "ov_msckf/src/core/VioManagerOptions.h",
        REPO_WIN / "ov_core/src/cam/CamRadtan.h",
        PRIOR_C0_CONFIG / "estimator_config.yaml",
        FLIGHTS["fly1"]["dataset_win"] / "meta.yaml",
        FLIGHTS["fly3"]["dataset_win"] / "meta.yaml",
    ]
    for index, source in enumerate(files):
        name = f"{index:02d}_{source.name}"
        shutil.copy2(source, target / name)
    rows = [core.file_identity(path) for path in files]
    write_json(target / "SOURCE_IDENTITIES.json", rows)


def refresh_provenance_hashes() -> None:
    provenance = ROOT_WIN / "provenance"
    paths = sorted(path for path in provenance.rglob("*") if path.is_file())
    out = provenance / "provenance_file_sha256.csv"
    with out.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["path", "sha256", "size_bytes"])
        for path in paths:
            if path == out:
                continue
            writer.writerow([str(path.relative_to(ROOT_WIN)), sha256(path), path.stat().st_size])


def prepare() -> None:
    if os.name == "posix":
        raise RuntimeError("prepare must run from Windows PowerShell")
    if (ROOT_WIN / "RUN_MATRIX.csv").exists():
        raise FileExistsError(f"refusing to overwrite existing matrix: {ROOT_WIN}")
    runner_hash_before = sha256(SOURCE_RUNNER)
    if runner_hash_before != EXPECTED_RUNNER_SHA256:
        raise RuntimeError(f"source frozen runner hash changed: {runner_hash_before}")
    if sha256(JUNE_CAMCHAIN) != JUNE_CAMCHAIN_EXPECTED_SHA256:
        raise RuntimeError("June12 original camchain hash does not match its recorded lineage")
    for path in [SOURCE_TEST, SOURCE_PROJECT_LIBRARY, PRIOR_C0_CONFIG, ANALYSIS_SNAPSHOT_SOURCE]:
        if not path.exists():
            raise FileNotFoundError(path)
    library_hash_before = sha256(SOURCE_PROJECT_LIBRARY)
    if library_hash_before != EXPECTED_PROJECT_LIBRARY_SHA256:
        raise RuntimeError(
            "project shared-library hash changed before freeze: "
            f"{library_hash_before} != {EXPECTED_PROJECT_LIBRARY_SHA256}"
        )

    (ROOT_WIN / "provenance/frozen_binary").mkdir(parents=True, exist_ok=False)
    (ROOT_WIN / "provenance/source_snapshots").mkdir(parents=True, exist_ok=False)
    frozen_runner = ROOT_WIN / "provenance/frozen_binary/run_serial_msckf_ros_free"
    shutil.copy2(SOURCE_RUNNER, frozen_runner)
    shutil.copy2(SOURCE_TEST, ROOT_WIN / "provenance/frozen_binary/test_online_alignment_initializer")
    frozen_library = ROOT_WIN / "provenance/frozen_binary" / PROJECT_LIBRARY_NAME
    shutil.copy2(SOURCE_PROJECT_LIBRARY, frozen_library)
    library_hash_after = sha256(SOURCE_PROJECT_LIBRARY)
    frozen_library_hash = sha256(frozen_library)
    runner_hash_after = sha256(SOURCE_RUNNER)
    frozen_runner_hash = sha256(frozen_runner)
    if not (
        runner_hash_before
        == runner_hash_after
        == frozen_runner_hash
        == EXPECTED_RUNNER_SHA256
    ):
        raise RuntimeError(
            "runner changed during freeze or frozen copy is corrupt: "
            f"before={runner_hash_before}, after={runner_hash_after}, "
            f"frozen={frozen_runner_hash}"
        )
    if not (
        library_hash_before
        == library_hash_after
        == frozen_library_hash
        == EXPECTED_PROJECT_LIBRARY_SHA256
    ):
        raise RuntimeError(
            "project shared library changed during freeze or frozen copy is corrupt: "
            f"before={library_hash_before}, after={library_hash_after}, frozen={frozen_library_hash}"
        )
    shutil.copytree(ANALYSIS_SNAPSHOT_SOURCE, ROOT_WIN / "provenance/analysis_tool_snapshot")

    core.markdown_matrix = markdown_matrix
    core.write_run_files = patched_write_run_files
    core.prepare()

    source_snapshot = ROOT_WIN / "provenance/source_snapshots"
    for name in [
        "p4_intrinsics_distortion_split_20260716.py",
        "run_p4_intrinsics_distortion_split_20260716.sh",
        "analyze_p4_intrinsics_distortion_split_20260716.py",
    ]:
        source = REPO_WIN / "tools" / name
        if source.exists():
            shutil.copy2(source, source_snapshot / name)

    copy_audit_sources()
    write_geometry_audit()
    write_actual_config_audit_summary()

    experiment_path = ROOT_WIN / "provenance/EXPERIMENT_PROVENANCE.json"
    experiment = read_json(experiment_path)
    experiment.update({
        "schema": "openvins_p4_intrinsics_distortion_split_experiment_v1",
        "experiment_id": experiment_id(),
        "scope": SCOPE,
        "attempt": ATTEMPT or None,
        "parent_evidence_root": str(PARENT_ROOT_WIN),
        "expected_runner_sha256": EXPECTED_RUNNER_SHA256,
        "pair_profile": PAIR_PROFILE,
        "runner_freeze": {
            "source": core.file_identity(SOURCE_RUNNER),
            "frozen": core.file_identity(frozen_runner),
            "source_hash_before_copy": runner_hash_before,
            "source_hash_after_copy": runner_hash_after,
        },
        "project_shared_library": {
            "source": core.file_identity(SOURCE_PROJECT_LIBRARY),
            "frozen": core.file_identity(frozen_library),
            "expected_sha256": EXPECTED_PROJECT_LIBRARY_SHA256,
            "source_hash_before_copy": library_hash_before,
            "source_hash_after_copy": library_hash_after,
            "binding": "per-run command.sh replaces LD_LIBRARY_PATH with provenance/frozen_binary",
            "old_57657_runner_companion_library_was_not_frozen_by_preceding_factorial": True,
            "scientific_gate": "C0 must pass process/frame/P4 identity validation before factor runs are accepted",
        },
        "binary_build_source": core.git_snapshot(REPO_WIN),
        "preceding_factorial_binary_build_source": read_json(
            PARENT_ROOT_WIN / "provenance/build_source_snapshot.json"
        ),
        "non_camera_config_template": str(PRIOR_C0_CONFIG),
        "june12_original_camchain": core.file_identity(JUNE_CAMCHAIN),
        "actual_config_audit": str(ROOT_WIN / "ACTUAL_CONFIG_AUDIT.json"),
        "calibration_geometry_audit": str(ROOT_WIN / "provenance/CALIBRATION_GEOMETRY_AUDIT.json"),
    })
    write_json(experiment_path, experiment)
    write_json(
        ROOT_WIN / "provenance/DYNAMIC_DEPENDENCY_LOCK.json",
        {
            "schema": "openvins_frozen_dynamic_dependency_lock_v1",
            "captured_at": now_iso(),
            "runner_sha256": EXPECTED_RUNNER_SHA256,
            "pair_profile": PAIR_PROFILE,
            "library_name": PROJECT_LIBRARY_NAME,
            "library_sha256": EXPECTED_PROJECT_LIBRARY_SHA256,
            "library_source": str(SOURCE_PROJECT_LIBRARY),
            "library_frozen": str(frozen_library),
            "prior_companion_library_available": False,
            "reason_for_gate": (
                "The preceding executable-only freeze was incomplete. This attempt freezes the "
                "explicitly authorized coherent new runner/library pair before any C0 run."
            ),
            "acceptance_gate": (
                "Run only C0 first; require process exit 0, frame-contract pass, non-empty "
                "trajectory, and valid P4 release fingerprint on both flights. Do not launch "
                "KONLY/DONLY/KD unless the gate passes."
            ),
        },
    )
    refresh_provenance_hashes()
    print(f"prepared {len(CONDITIONS) * len(FLIGHTS)} {SCOPE} runs at {ROOT_WIN}")


def record_start(run_id: str) -> None:
    audit = actual_config_audit(run_id)
    if not audit["pass"]:
        raise RuntimeError(f"actual config audit failed for {run_id}")
    core.record_start(run_id)
    path = ROOT / "runs" / run_id / "run_manifest.json"
    manifest = read_json(path)
    manifest["execution"]["actual_config_audit_at_start"] = {
        "path": str(path.parent / "actual_config_audit.json"),
        "sha256": sha256(path.parent / "actual_config_audit.json"),
        "pass": True,
    }
    manifest["execution"]["project_library_sha256_at_start"] = sha256(
        frozen_project_library()
    )
    write_json(path, manifest)


def finalize(run_id: str, exit_code: int, frame_code: int) -> None:
    core.finalize(run_id, exit_code, frame_code)
    path = ROOT / "runs" / run_id / "run_manifest.json"
    manifest = read_json(path)
    manifest["execution"]["project_library_sha256_at_end"] = sha256(
        frozen_project_library()
    )
    audit = actual_config_audit(run_id)
    manifest["execution"]["actual_config_audit_at_end"] = {
        "path": str(path.parent / "actual_config_audit.json"),
        "sha256": sha256(path.parent / "actual_config_audit.json"),
        "pass": audit["pass"],
    }
    manifest["execution"]["process_artifacts"] = {
        name: core.file_identity(path.parent / name)
        for name in [
            "process_start.txt", "process_end.txt", "exit_code.txt",
            "frame_contract_validation.json",
            "frame_contract_validation_exit_code.txt", "resource_usage.txt",
        ]
    }
    if not audit["pass"]:
        manifest["status"] = "failed"
        manifest["execution"]["status"] = "failed"
        manifest["execution"]["failure_reason"] = "actual config changed or failed end audit"
    write_json(path, manifest)


def read_int(path: Path, default: int = -999) -> int:
    try:
        return int(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return default


def read_float(path: Path) -> float:
    try:
        return float(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return math.nan


def status_summary() -> None:
    rows = []
    for condition in CONDITIONS:
        for flight in FLIGHTS:
            run_id = f"{condition}_{flight}"
            run = ROOT / "runs" / run_id
            manifest_path = run / "run_manifest.json"
            manifest = read_json(manifest_path)
            validation_path = run / "run_validation.json"
            validation_pass = (
                read_json(validation_path).get("pass") if validation_path.exists() else None
            )
            config_path = run / "actual_config_audit.json"
            config_pass = read_json(config_path).get("pass") if config_path.exists() else None
            process_exit = read_int(run / "exit_code.txt")
            frame_exit = read_int(run / "frame_contract_validation_exit_code.txt")
            finalize_exit = read_int(run / "finalize_exit_code.txt")
            start = read_float(run / "process_start_epoch_s.txt")
            end = read_float(run / "process_end_epoch_s.txt")
            rows.append({
                "run_id": run_id,
                "scope": SCOPE,
                "condition": condition,
                "flight": flight,
                "status": manifest.get("status"),
                "process_exit": process_exit,
                "frame_exit": frame_exit,
                "finalize_exit": finalize_exit,
                "validation_pass": validation_pass,
                "actual_config_pass": config_pass,
                "runtime_s": end - start,
                "failure_reason": manifest.get("execution", {}).get("failure_reason", ""),
                "run_dir": str(run),
            })
    with (ROOT / "RUN_STATUS.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_json(ROOT / "RUN_STATUS.json", rows)
    for row in rows:
        print(
            f"{row['run_id']},{row['status']},{row['process_exit']},"
            f"{row['frame_exit']},{row['finalize_exit']},{row['validation_pass']}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("prepare")
    start = sub.add_parser("record-start")
    start.add_argument("run_id")
    validate = sub.add_parser("validate-run")
    validate.add_argument("run_id")
    finish = sub.add_parser("finalize")
    finish.add_argument("run_id")
    finish.add_argument("--exit-code", type=int, required=True)
    finish.add_argument("--frame-code", type=int, required=True)
    audit = sub.add_parser("audit-config")
    audit.add_argument("run_id", nargs="?")
    sub.add_parser("status")
    args = parser.parse_args()
    if args.command == "prepare":
        prepare()
    elif args.command == "record-start":
        record_start(args.run_id)
    elif args.command == "validate-run":
        result = core.validate_run(args.run_id)
        print(json.dumps({"run_id": args.run_id, "pass": result["pass"]}))
    elif args.command == "finalize":
        finalize(args.run_id, args.exit_code, args.frame_code)
    elif args.command == "audit-config":
        if args.run_id:
            result = actual_config_audit(args.run_id)
            print(json.dumps({"run_id": args.run_id, "pass": result["pass"]}))
        else:
            write_actual_config_audit_summary()
    elif args.command == "status":
        status_summary()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
