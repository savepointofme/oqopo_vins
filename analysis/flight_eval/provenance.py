"""provenance.py — metadata 四件套.

没有 provenance 的分析结果一律视为无效。
  run_spec_resolved.json  解析后输入（由 run_spec.write_resolved 写）
  analysis_config.json    全部阈值/开关（冻结）
  status.json             分步状态（失败即停）
  command.txt             复现命令
  provenance.json         可复现摘要（时间/版本/哈希/输出清单）
"""
from __future__ import annotations

import datetime as _dt
import json
import os
import sys

from . import METRIC_DEFINITIONS_VERSION, __version__, io


def _now() -> str:
    return _dt.datetime.now().astimezone().isoformat(timespec="seconds")


class StatusTracker:
    """分步状态跟踪。任一步失败 → state=error 并停在该步。"""

    STEPS = ["inspect", "align", "sample", "metrics", "segment", "plots", "report"]

    def __init__(self):
        self.steps = []
        self.warnings = []
        self.failed_step = None

    def ok(self, name: str, message: str = "") -> None:
        self.steps.append({"name": name, "state": "ok", "message": message})

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def fail(self, name: str, message: str) -> None:
        self.steps.append({"name": name, "state": "error", "message": message})
        self.failed_step = name

    def to_dict(self, status_hint: str = "success") -> dict:
        state = "error" if self.failed_step else status_hint
        return {
            "state": state,
            "steps": self.steps,
            "warnings": self.warnings,
            "failed_step": self.failed_step,
            "generated_at": _now(),
        }

    def write(self, metadata_dir: str, status_hint: str = "success") -> None:
        os.makedirs(metadata_dir, exist_ok=True)
        with open(os.path.join(metadata_dir, "status.json"), "w", encoding="utf-8") as f:
            json.dump(self.to_dict(status_hint), f, ensure_ascii=False, indent=2)


def write_analysis_config(metadata_dir: str, *, sampling: dict, alignment: dict,
                          seg_cfg: dict, align_mode: str = "start_heading") -> None:
    cfg = {
        "sampling": sampling,
        "alignment": alignment,
        "segment": seg_cfg,
        "align_mode": align_mode,
        "metric_definitions_version": METRIC_DEFINITIONS_VERSION,
    }
    with open(os.path.join(metadata_dir, "analysis_config.json"), "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)


def write_command(metadata_dir: str, argv: list[str] | None = None) -> str:
    argv = argv or sys.argv
    cmd = "python3 " + " ".join(argv)
    with open(os.path.join(metadata_dir, "command.txt"), "w", encoding="utf-8") as f:
        f.write(cmd + "\n")
    return cmd


def write_provenance(metadata_dir: str, *, spec, command: str,
                     output_files: list[str], git_commit: str | None = None,
                     vio_velocity_source: str = "", reference_velocity_source: str = "") -> None:
    input_hashes = {}
    for name, rp in spec.inputs.items():
        if rp.state == "ok":
            input_hashes[name] = io.sha256(rp)
    prov = {
        "generated_at": _now(),
        "tool_version": __version__,
        "git_commit": git_commit or "TODO·本地核验",
        "command": command,
        "experiment_id": spec.experiment_id,
        "metric_definitions_version": METRIC_DEFINITIONS_VERSION,
        "reference_velocity_source": reference_velocity_source,
        "vio_velocity_source": vio_velocity_source,
        "input_hashes": input_hashes,
        "output_files": output_files,
    }
    with open(os.path.join(metadata_dir, "provenance.json"), "w", encoding="utf-8") as f:
        json.dump(prov, f, ensure_ascii=False, indent=2)
