#!/usr/bin/env python3
"""Prepare and optionally execute a reproducible flex-aware FC factor replay.

The input is a frozen stride-12 command.json. The script preserves estimator
arguments, rewrites result paths into a new directory, and changes only the
explicit flex-factor switch. It never overwrites the source run.
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
    "--flex-fc-factor-diag",
}
SINGLE_FLAGS_TO_REMOVE = {
    "--enable-flex-body-attitude",
    "--enable-flex-aware-fc-yaw-update",
}


def windows_to_wsl(path: Path) -> str:
    resolved = path.resolve()
    drive, tail = os.path.splitdrive(str(resolved))
    if not drive:
        return str(resolved).replace("\\", "/")
    normalized_tail = tail.lstrip("\\/").replace("\\", "/")
    return f"/mnt/{drive[0].lower()}/{normalized_tail}"


def rewrite_command(source: List[str], output_dir_wsl: str, executable: str,
                    enabled: bool, until_time: float | None) -> List[str]:
    output: List[str] = [executable]
    i = 1
    fc_path = None
    while i < len(source):
        token = source[i]
        if token in SINGLE_FLAGS_TO_REMOVE:
            i += 1
            continue
        if token in PAIR_FLAGS_TO_REMOVE:
            i += 2
            continue
        if token == "--flex-fc-attitude":
            fc_path = source[i + 1]
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
        if token == "--run-name":
            output.extend([token, Path(output_dir_wsl).name])
            i += 2
            continue
        if token == "--dash-title":
            output.extend([token, Path(output_dir_wsl).name])
            i += 2
            continue
        output.append(token)
        i += 1
    if enabled:
        if not fc_path:
            raise ValueError("source command has no --flex-fc-attitude stream")
        output.extend([
            "--enable-flex-aware-fc-yaw-update",
            "--flex-fc-attitude", fc_path,
            "--flex-fc-factor-diag",
            f"{output_dir_wsl}/flex_fc_relative_yaw_factor.csv",
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
    parser.add_argument("--mode", choices=("enabled", "disabled"), required=True)
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
        source, output_dir_wsl, args.executable,
        args.mode == "enabled", args.until_time,
    )
    (args.output_dir / "command.json").write_text(
        json.dumps(command, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    command_sh = "#!/usr/bin/env bash\nset -euo pipefail\n" + \
        " ".join(shlex.quote(item) for item in command) + "\n"
    with (args.output_dir / "command.sh").open(
            "w", encoding="utf-8", newline="\n") as stream:
        stream.write(command_sh)
    provenance = {
        "source_command_json": str(args.source_command_json.resolve()),
        "mode": args.mode,
        "until_time": args.until_time,
        "executable": args.executable,
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
