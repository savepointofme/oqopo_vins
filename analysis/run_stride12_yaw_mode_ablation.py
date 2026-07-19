#!/usr/bin/env python3
"""Replay a frozen stride-12 command with exactly one yaw-update mode changed.

The source command is treated as immutable provenance.  All generated outputs
are redirected to a new directory and output-only flex observer arguments are
removed so that the run isolates the VIO yaw-update policy.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import List


PATH_FLAGS = {
    "--output",
    "--camera-stride-audit",
    "--gps-alt-coupled-diag",
    "--diag-csv",
    "--diag-events",
    "--vio-yaw-diag",
    "--visual-obs-diag",
    "--output-raw",
    "--output-nav",
    "--nav-frame-metadata-json",
    "--canonical-init-state-json",
}
PAIR_FLAGS_TO_REMOVE = {
    "--flex-body-attitude-output",
    "--flex-fc-attitude",
    "--flex-fc-factor-diag",
    "--fc-gyro-agreement-deg",
    "--fc-gyro-visual-yaw-scale",
    "--fc-gyro-visual-yaw-diag",
    "--fc-gyro-visual-yaw-reference-tau-s",
    "--fc-gyro-visual-yaw-reference-threshold-deg",
    "--fc-gyro-visual-yaw-reference-clip-deg",
    "--fc-gyro-visual-yaw-reference-dwell-s",
    "--fc-gyro-visual-yaw-step-cap-deg",
}
SINGLE_FLAGS_TO_REMOVE = {
    "--enable-flex-body-attitude",
    "--enable-flex-aware-fc-yaw-update",
    "--enable-fc-gyro-visual-yaw-guard",
}


def windows_to_wsl(path: Path) -> str:
    resolved = path.resolve()
    drive, tail = os.path.splitdrive(str(resolved))
    if not drive:
        return str(resolved).replace("\\", "/")
    normalized_tail = tail.lstrip("\\/").replace("\\", "/")
    return f"/mnt/{drive[0].lower()}/{normalized_tail}"


def rewrite_command(source: List[str], output_dir_wsl: str, executable: str,
                    yaw_mode: str, until_time: float | None,
                    enable_fc_gyro_guard: bool,
                    agreement_deg: float, guarded_scale: float,
                    reference_tau_s: float, reference_threshold_deg: float,
                    reference_clip_deg: float, reference_dwell_s: float,
                    step_cap_deg: float) -> List[str]:
    output: List[str] = [executable]
    i = 1
    yaw_mode_found = False
    fc_path = None
    while i < len(source):
        token = source[i]
        if token in SINGLE_FLAGS_TO_REMOVE:
            i += 1
            continue
        if token == "--flex-fc-attitude":
            fc_path = source[i + 1]
            i += 2
            continue
        if token in PAIR_FLAGS_TO_REMOVE:
            i += 2
            continue
        if token == "--vio-yaw-update-mode":
            output.extend([token, yaw_mode])
            yaw_mode_found = True
            i += 2
            continue
        if token in PATH_FLAGS:
            output.extend([token, f"{output_dir_wsl}/{Path(source[i + 1]).name}"])
            i += 2
            continue
        if token == "--until-time" and until_time is not None:
            output.extend([token, f"{until_time:.9f}"])
            i += 2
            continue
        if token in {"--run-name", "--dash-title"}:
            output.extend([token, Path(output_dir_wsl).name])
            i += 2
            continue
        output.append(token)
        i += 1
    if not yaw_mode_found:
        output.extend(["--vio-yaw-update-mode", yaw_mode])
    if enable_fc_gyro_guard:
        if not fc_path:
            raise ValueError("source command has no --flex-fc-attitude stream")
        output.extend([
            "--enable-fc-gyro-visual-yaw-guard",
            "--flex-fc-attitude", fc_path,
            "--fc-gyro-agreement-deg", f"{agreement_deg:.9f}",
            "--fc-gyro-visual-yaw-scale", f"{guarded_scale:.9f}",
            "--fc-gyro-visual-yaw-reference-tau-s", f"{reference_tau_s:.9f}",
            "--fc-gyro-visual-yaw-reference-threshold-deg", f"{reference_threshold_deg:.9f}",
            "--fc-gyro-visual-yaw-reference-clip-deg", f"{reference_clip_deg:.9f}",
            "--fc-gyro-visual-yaw-reference-dwell-s", f"{reference_dwell_s:.9f}",
            "--fc-gyro-visual-yaw-step-cap-deg", f"{step_cap_deg:.9f}",
            "--fc-gyro-visual-yaw-diag",
            f"{output_dir_wsl}/fc_gyro_visual_yaw_guard.csv",
        ])
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-command-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--executable",
        default="/tmp/open_vins_flex_baseline_build_20260719/run_serial_msckf_ros_free",
    )
    parser.add_argument(
        "--yaw-mode",
        choices=("baseline", "global_yaw_oc_projection", "hard_gyro_yaw",
                 "fc_gyro_guarded_visual_yaw"),
        required=True,
    )
    parser.add_argument("--enable-fc-gyro-guard", action="store_true")
    parser.add_argument("--agreement-deg", type=float, default=0.35)
    parser.add_argument("--guarded-scale", type=float, default=0.95)
    parser.add_argument("--reference-tau-s", type=float, default=5.0)
    parser.add_argument("--reference-threshold-deg", type=float, default=1.0)
    parser.add_argument("--reference-clip-deg", type=float, default=1.0)
    parser.add_argument("--reference-dwell-s", type=float, default=10.0)
    parser.add_argument("--step-cap-deg", type=float, default=0.0)
    parser.add_argument("--until-time", type=float)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()

    source = json.loads(args.source_command_json.read_text(encoding="utf-8"))
    if not isinstance(source, list) or not source:
        raise ValueError("source command JSON must contain a non-empty argv list")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise FileExistsError(f"refusing non-empty output directory: {args.output_dir}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_dir_wsl = windows_to_wsl(args.output_dir)
    command = rewrite_command(
        source, output_dir_wsl, args.executable, args.yaw_mode, args.until_time,
        args.enable_fc_gyro_guard, args.agreement_deg, args.guarded_scale,
        args.reference_tau_s, args.reference_threshold_deg, args.reference_clip_deg,
        args.reference_dwell_s, args.step_cap_deg)
    (args.output_dir / "command.json").write_text(
        json.dumps(command, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    command_sh = (
        "#!/usr/bin/env bash\nset -euo pipefail\n"
        + " ".join(shlex.quote(item) for item in command)
        + "\n"
    )
    with (args.output_dir / "command.sh").open(
            "w", encoding="utf-8", newline="\n") as stream:
        stream.write(command_sh)
    provenance = {
        "source_command_json": str(args.source_command_json.resolve()),
        "yaw_mode": args.yaw_mode,
        "until_time": args.until_time,
        "executable": args.executable,
        "removed_output_only_flex_observer": True,
        "enable_fc_gyro_guard": args.enable_fc_gyro_guard,
        "agreement_deg": args.agreement_deg,
        "guarded_scale": args.guarded_scale,
        "reference_tau_s": args.reference_tau_s,
        "reference_threshold_deg": args.reference_threshold_deg,
        "reference_clip_deg": args.reference_clip_deg,
        "reference_dwell_s": args.reference_dwell_s,
        "step_cap_deg": args.step_cap_deg,
    }
    (args.output_dir / "provenance.json").write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    if not args.run:
        return 0

    log_path = args.output_dir / "log.txt"
    with log_path.open("wb") as log:
        completed = subprocess.run(
            ["wsl.exe", "bash", f"{output_dir_wsl}/command.sh"],
            stdout=log, stderr=subprocess.STDOUT, check=False)
    (args.output_dir / "exit_code.txt").write_text(
        f"{completed.returncode}\n", encoding="ascii")
    return completed.returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
