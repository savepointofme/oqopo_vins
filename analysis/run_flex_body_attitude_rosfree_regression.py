#!/usr/bin/env python3
"""Re-run the frozen desktop stride-12 commands with optional flex output."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import time


REPO_ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def windows_to_wsl(path: str) -> Path:
    normalized = path.replace("\\", "/")
    if len(normalized) >= 3 and normalized[1:3] == ":/":
        return Path("/mnt") / normalized[0].lower() / normalized[3:]
    return Path(normalized)


def set_option(command: list[str], name: str, value: str) -> None:
    if name in command:
        command[command.index(name) + 1] = value
    else:
        command.extend((name, value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manifest",
        type=Path,
        default=REPO_ROOT
        / "analysis/manifests/continuous_yaw_flex_stride12_20260719.json",
    )
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--fc-stream-dir", type=Path)
    parser.add_argument("--mode", choices=("enabled", "disabled"), required=True)
    parser.add_argument("--flights", nargs="+", default=["fly1", "fly2", "fly3", "fly4"])
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--until-time", type=float)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    args.output_root.mkdir(parents=True, exist_ok=False)
    summary: dict[str, object] = {
        "schema_version": 1,
        "data_identity": manifest["data_identity"],
        "baseline_commit": manifest["baseline_commit"],
        "mode": args.mode,
        "runner": str(args.runner),
        "runner_sha256": sha256(args.runner),
        "flights": {},
    }
    for flight in args.flights:
        frozen = windows_to_wsl(manifest["flights"][flight]["run_dir"])
        historical_path = frozen / "command.txt"
        config = frozen / "config" / "config_run.yaml"
        command = shlex.split(historical_path.read_text(encoding="utf-8"))
        command[0] = str(args.runner)
        output = args.output_root / flight
        output.mkdir()
        set_option(command, "--config", str(config))
        set_option(command, "--camera-stride-audit", str(output / "stride_audit.csv"))
        set_option(command, "--gps-alt-coupled-diag", str(output / "gpsz_coupled_diag.csv"))
        set_option(command, "--diag-csv", str(output / "diag.csv"))
        set_option(command, "--diag-events", str(output / "events.txt"))
        set_option(command, "--vio-yaw-diag", str(output / "vio_yaw_diag.csv"))
        set_option(command, "--visual-obs-diag", str(output / "visual_obs_diag.csv"))
        set_option(command, "--output", str(output / "traj.txt"))
        set_option(command, "--run-name", f"{flight}_stride12_flex_{args.mode}")
        set_option(command, "--dash-title", f"{flight} stride12 flex {args.mode}")
        if args.until_time is not None:
            set_option(command, "--until-time", f"{args.until_time:.9f}")
        if args.headless and "--no-display" not in command:
            command.append("--no-display")
        if args.mode == "enabled":
            if args.fc_stream_dir is None:
                raise RuntimeError("enabled mode requires --fc-stream-dir")
            command.append("--enable-flex-body-attitude")
            set_option(
                command,
                "--flex-fc-attitude",
                str(args.fc_stream_dir / f"{flight}_fc_body_attitude_camera_time.csv"),
            )
            set_option(
                command,
                "--flex-body-attitude-output",
                str(output / "aircraft_body_attitude.csv"),
            )
        (output / "command.json").write_text(
            json.dumps(command, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        (output / "command.sh").write_text(
            "#!/usr/bin/env bash\nset -euo pipefail\n" + shlex.join(command) + "\n",
            encoding="utf-8",
        )
        print(f"[{flight}] start mode={args.mode}", flush=True)
        start = time.perf_counter()
        with (output / "stdout.log").open("w", encoding="utf-8") as log:
            completed = subprocess.run(
                command, stdout=log, stderr=subprocess.STDOUT, check=False
            )
        elapsed = time.perf_counter() - start
        summary["flights"][flight] = {
            "exit_code": completed.returncode,
            "wall_time_s": elapsed,
            "historical_command": str(historical_path),
            "historical_config": str(config),
            "historical_config_sha256": sha256(config),
            "output_dir": str(output),
        }
        (args.output_root / "run_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"[{flight}] exit={completed.returncode} wall={elapsed:.1f}s", flush=True)
        if completed.returncode != 0:
            return completed.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
