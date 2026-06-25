"""run_spec.py — run spec schema 校验、resolve、落盘.

run spec 是单条实验的唯一输入清单。运行时把 resolved spec 写入
metadata/run_spec_resolved.json。必填缺失 → 报错；可选缺失 → unavailable。
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Optional

from . import io

REQUIRED = ["experiment_id", "flight_name", "method_name", "date", "status",
            "gps_csv", "vio_traj", "t0", "t1"]

DEFAULT_ALIGNMENT = {"mode": "start_heading", "course_window_s": 60.0}
DEFAULT_SAMPLING = {"mode": "gps_update_after_vio", "max_delay_s": 0.2}
DEFAULT_SEGMENTATION = {"mode": "four_side_lap"}


class RunSpecError(ValueError):
    pass


@dataclass
class RunSpec:
    raw: dict
    experiment_id: str
    flight_name: str
    method_name: str
    date: str
    status: str
    t0: float
    t1: float
    source_experiment_folder: str = ""
    alignment: dict = field(default_factory=lambda: dict(DEFAULT_ALIGNMENT))
    sampling: dict = field(default_factory=lambda: dict(DEFAULT_SAMPLING))
    segmentation: dict = field(default_factory=lambda: dict(DEFAULT_SEGMENTATION))
    velocity_source: str = "flight_controller_raw"
    notes: str = ""
    # resolved inputs: name -> io.ResolvedPath
    inputs: dict = field(default_factory=dict)

    # ---- 派生 ----
    @property
    def out_folder_name(self) -> str:
        """<日期_飞行_模式_配置_状态>，中文状态。"""
        zh = {"success": "成功", "fail": "失败", "partial": "部分"}.get(self.status, self.status)
        return f"{self.date}_{self.flight_name}_{self.method_name}_{zh}"

    def analysis_dir(self, out_root: str) -> str:
        return os.path.join(out_root, self.source_experiment_folder or self.experiment_id,
                            self.out_folder_name)

    def resolved_dict(self) -> dict:
        return {
            "experiment_id": self.experiment_id,
            "flight_name": self.flight_name,
            "method_name": self.method_name,
            "date": self.date,
            "status": self.status,
            "source_experiment_folder": self.source_experiment_folder,
            "t0": self.t0,
            "t1": self.t1,
            "alignment": self.alignment,
            "sampling": self.sampling,
            "segmentation": self.segmentation,
            "velocity_source": {"reference": self.velocity_source, "vio": "resolved_at_runtime"},
            "inputs": {k: v.to_dict() for k, v in self.inputs.items()},
            "notes": self.notes,
        }


# 可选输入键
OPTIONAL_INPUTS = ["vio_bias", "diag_csv", "vio_yaw_diag", "lk_traj", "lk_flow_csv"]
REQUIRED_INPUTS = ["gps_csv", "vio_traj"]


def load(path: str) -> RunSpec:
    """读取 + 校验 + resolve 一份 run spec JSON。"""
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)

    missing = [k for k in REQUIRED if k not in d]
    if missing:
        raise RunSpecError(f"run spec 缺少必填字段: {missing}")

    spec = RunSpec(
        raw=d,
        experiment_id=d["experiment_id"],
        flight_name=d["flight_name"],
        method_name=d["method_name"],
        date=str(d["date"]),
        status=d["status"],
        t0=float(d["t0"]),
        t1=float(d["t1"]),
        source_experiment_folder=d.get("source_experiment_folder", ""),
        alignment={**DEFAULT_ALIGNMENT, **d.get("alignment", {})},
        sampling={**DEFAULT_SAMPLING, **d.get("sampling", {})},
        segmentation={**DEFAULT_SEGMENTATION, **d.get("segmentation", {})},
        velocity_source=d.get("velocity_source", "flight_controller_raw"),
        notes=d.get("notes", ""),
    )

    # resolve 所有输入路径
    for key in REQUIRED_INPUTS + OPTIONAL_INPUTS:
        spec.inputs[key] = io.resolve(d.get(key))

    # 必填输入不可达 → 报错
    for key in REQUIRED_INPUTS:
        rp = spec.inputs[key]
        if rp.state != "ok":
            raise RunSpecError(f"必填输入 {key} 不可达: {rp.reason}")

    if spec.t1 <= spec.t0:
        raise RunSpecError(f"t1({spec.t1}) 必须大于 t0({spec.t0})")

    return spec


def write_resolved(spec: RunSpec, metadata_dir: str) -> str:
    os.makedirs(metadata_dir, exist_ok=True)
    p = os.path.join(metadata_dir, "run_spec_resolved.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(spec.resolved_dict(), f, ensure_ascii=False, indent=2)
    return p
