"""lk_only.py — 纯视觉 (LK-only) 诊断.

优先级:
  1. 已有 LK-only 轨迹 CSV → 直接读取，走与 VIO 相同流程。
  2. 仅有 LK flow pair 诊断 → 用光流相似/仿射/单应 + 高度尺度构建诊断轨迹。
     仅用 GPS 评价、不用 GPS 修正；标 LK_ONLY_DIAGNOSTIC，不称可部署里程计。
  3. 信息不足 → 明确说明缺什么，不伪造轨迹。

输出:
  data/lk_only_traj.csv
  data/lk_only_gps_aligned_samples.csv
  tables/lk_only_segment_error_summary.csv
  reports/LK_ONLY_ANALYSIS.md
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from . import io


@dataclass
class LKResult:
    available: bool
    mode: str           # "trajectory" | "LK_ONLY_DIAGNOSTIC" | "unavailable"
    traj: pd.DataFrame | None = None     # columns: t, E, N, U
    diagnostic: pd.DataFrame | None = None
    missing: list = None
    scale_source: str = ""


def load_or_build(run_spec) -> LKResult:
    """根据 run spec 决定 LK-only 处理路径。"""
    lk_traj = run_spec.inputs.get("lk_traj")
    lk_flow = run_spec.inputs.get("lk_flow_csv")

    # 优先级 1: 现成轨迹
    if lk_traj is not None and lk_traj.state == "ok":
        df = io.read_csv(lk_traj)
        cols = {c.lower(): c for c in df.columns}
        need = ["t", "e", "n", "u"]
        if all(k in cols for k in need):
            out = pd.DataFrame({
                "t": df[cols["t"]].astype(float),
                "E": df[cols["e"]].astype(float),
                "N": df[cols["n"]].astype(float),
                "U": df[cols["u"]].astype(float),
            })
            return LKResult(True, "trajectory", traj=out, scale_source="provided")
        return LKResult(False, "unavailable",
                        missing=["lk_traj 缺少 t/E/N/U 列"])

    # 优先级 2: 由 flow 诊断构建
    if lk_flow is not None and lk_flow.state == "ok":
        try:
            flow = io.read_csv(lk_flow)
            yaw_diag_cols = {
                "t", "rate_lk_heading_degps", "omega_gps_degps",
                "n_inliers", "ransac_resid_px",
            }
            if yaw_diag_cols.issubset(flow.columns):
                return LKResult(
                    True,
                    "LK_YAW_DIAGNOSTIC",
                    diagnostic=flow.sort_values("t").reset_index(drop=True),
                    scale_source="not_applicable",
                )
            traj, scale_src = _build_from_flow(flow, run_spec)
            return LKResult(True, "LK_ONLY_DIAGNOSTIC", traj=traj, scale_source=scale_src)
        except Exception as e:  # noqa: BLE001
            return LKResult(False, "unavailable", missing=[f"flow 构建失败: {e}"])

    # 优先级 3: 信息不足
    return LKResult(False, "unavailable",
                    missing=["未提供 lk_traj 或 lk_flow_csv"])


def _build_from_flow(flow: pd.DataFrame, run_spec) -> tuple[pd.DataFrame, str]:
    """由逐帧 LK 光流相似/仿射增量积分构建诊断轨迹.

    期望 flow 含: t, dx_px, dy_px, (可选 dyaw_rad, scale).
    尺度: 优先用 altitude 列；否则用 GPS altitude（仅作尺度，不修正轨迹）。
    TODO·本地核验 flow 诊断文件真实字段。
    """
    cols = {c.lower(): c for c in flow.columns}
    if not all(k in cols for k in ("t", "dx_px", "dy_px")):
        raise ValueError("flow 缺少 t/dx_px/dy_px 列")

    t = flow[cols["t"]].astype(float).values
    dx = flow[cols["dx_px"]].astype(float).values
    dy = flow[cols["dy_px"]].astype(float).values

    # 尺度来源
    if "altitude" in cols:
        alt = flow[cols["altitude"]].astype(float).values
        scale_src = "flow.altitude"
    else:
        alt = np.full_like(t, 80.0)   # TODO·接 GPS altitude 插值
        scale_src = "gps_altitude(TODO)"

    # 像素位移 → 地面位移（简化针孔模型: ground = px * alt / focal）
    focal_px = 600.0  # TODO·本地核验相机内参
    gx = dx * alt / focal_px
    gy = dy * alt / focal_px

    dyaw = flow[cols["dyaw_rad"]].astype(float).values if "dyaw_rad" in cols else np.zeros_like(t)
    yaw = np.cumsum(dyaw)
    E = np.zeros_like(t)
    N = np.zeros_like(t)
    for i in range(1, len(t)):
        c, s = np.cos(yaw[i]), np.sin(yaw[i])
        E[i] = E[i - 1] + c * gx[i] - s * gy[i]
        N[i] = N[i - 1] + s * gx[i] + c * gy[i]
    out = pd.DataFrame({"t": t, "E": E, "N": N, "U": np.zeros_like(t)})
    return out, scale_src
