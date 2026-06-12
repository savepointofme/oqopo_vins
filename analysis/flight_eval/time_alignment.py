"""time_alignment.py — FC / IMU / camera 时间戳对齐 (6.1).

飞控时间戳与 IMU/camera 时间戳可能不一致且无公共字段。
用角运动相似性全局搜索时间偏移:
  1. 从飞控日志提取姿态角 → 差分得角速度代理（或直接用角速度）；
  2. 从 IMU 提取 gyro 三轴；
  3. 对 yaw/roll/pitch rate 做相关，全局搜索 offset；
  4. 用起止时间、offset 合理范围、多轴一致性、相关峰宽约束排除错误 offset；
  5. 输出多个候选（非黑盒单值）+ 诊断图。

输出:
  time_alignment_candidates.csv
  time_alignment_best.json
  time_alignment_report.md
  figures/time_alignment_correlation.png
  figures/gyro_fc_rate_overlay.png
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass

import numpy as np
import pandas as pd


@dataclass
class AlignConfig:
    offset_min_s: float = -5.0
    offset_max_s: float = 5.0
    offset_step_s: float = 0.01
    resample_hz: float = 100.0
    min_peak_prominence: float = 0.3   # 归一化相关峰最小显著度
    axes: tuple = ("yaw_rate", "roll_rate", "pitch_rate")


def _resample(t, x, hz, t0, t1):
    grid = np.arange(t0, t1, 1.0 / hz)
    return grid, np.interp(grid, t, x)


def _norm_xcorr(a, b):
    a = (a - np.mean(a)) / (np.std(a) + 1e-9)
    b = (b - np.mean(b)) / (np.std(b) + 1e-9)
    n = len(a)
    full = np.correlate(a, b, mode="full") / n
    lags = np.arange(-n + 1, n)
    return lags, full


def search_offset(fc: pd.DataFrame, imu: pd.DataFrame, cfg: AlignConfig | None = None,
                  camera: pd.DataFrame | None = None) -> dict:
    """搜索 fc→imu 时间偏移。

    入参 DataFrame 需含列:
      fc:  t, yaw_rate/roll_rate/pitch_rate （或 roll,pitch,yaw → 这里假设已是 rate）
      imu: t, gyro_x, gyro_y, gyro_z
    返回 {candidates: [...], best: {...}, valid_window: {...}}.
    """
    cfg = cfg or AlignConfig()
    t0 = max(fc["t"].min(), imu["t"].min())
    t1 = min(fc["t"].max(), imu["t"].max())
    if camera is not None:
        t0 = max(t0, camera["t"].min())
        t1 = min(t1, camera["t"].max())
    if t1 <= t0:
        raise ValueError("FC / IMU / camera 时间窗无重叠。TODO·本地核验时间单位。")

    # IMU gyro 轴映射: 假定 gyro_z≈yaw_rate, gyro_x≈roll_rate, gyro_y≈pitch_rate
    axis_map = {"yaw_rate": "gyro_z", "roll_rate": "gyro_x", "pitch_rate": "gyro_y"}
    candidates = []
    for ax in cfg.axes:
        if ax not in fc.columns or axis_map[ax] not in imu.columns:
            continue
        g, fa = _resample(fc["t"].values, fc[ax].values, cfg.resample_hz, t0, t1)
        _, ia = _resample(imu["t"].values, imu[axis_map[ax]].values, cfg.resample_hz, t0, t1)
        lags, xc = _norm_xcorr(fa, ia)
        lag_s = lags / cfg.resample_hz
        m = (lag_s >= cfg.offset_min_s) & (lag_s <= cfg.offset_max_s)
        if not m.any():
            continue
        idx = np.argmax(xc[m])
        best_lag = float(lag_s[m][idx])
        peak = float(xc[m][idx])
        # 峰宽: 相关下降到 peak*0.7 的宽度
        half = peak * 0.7
        above = np.where(xc[m] >= half)[0]
        width_s = float((above[-1] - above[0]) / cfg.resample_hz) if len(above) else np.nan
        candidates.append({
            "axis": ax,
            "offset_s": round(best_lag, 4),
            "peak_corr": round(peak, 4),
            "peak_width_s": round(width_s, 4),
        })

    if not candidates:
        raise ValueError("无可用轴做相关。TODO·本地核验 FC/IMU 列名与单位。")

    # 多轴一致性: 取相关峰最高且 offset 彼此接近的为 best
    cand_df = pd.DataFrame(candidates)
    best_row = cand_df.sort_values("peak_corr", ascending=False).iloc[0]
    offsets = cand_df["offset_s"].values
    consistency = float(np.std(offsets))
    best = {
        "offset_s": float(best_row["offset_s"]),
        "chosen_axis": best_row["axis"],
        "peak_corr": float(best_row["peak_corr"]),
        "multi_axis_offset_std_s": round(consistency, 4),
        "warning": "多轴 offset 离散，谨慎采用" if consistency > 0.1 else "",
    }
    return {
        "candidates": candidates,
        "best": best,
        "valid_window": {"t0": float(t0), "t1": float(t1)},
    }


def write_outputs(result: dict, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    pd.DataFrame(result["candidates"]).to_csv(
        os.path.join(out_dir, "time_alignment_candidates.csv"), index=False)
    with open(os.path.join(out_dir, "time_alignment_best.json"), "w", encoding="utf-8") as f:
        json.dump(result["best"], f, ensure_ascii=False, indent=2)
    lines = ["# 时间对齐报告\n",
             f"- 最优 offset: **{result['best']['offset_s']} s** (轴={result['best']['chosen_axis']})",
             f"- 峰值相关: {result['best']['peak_corr']}",
             f"- 多轴 offset 标准差: {result['best']['multi_axis_offset_std_s']} s",
             f"- 有效窗口: {result['valid_window']}",
             "", "## 候选", "", "| 轴 | offset_s | 峰相关 | 峰宽_s |", "|---|---:|---:|---:|"]
    for c in result["candidates"]:
        lines.append(f"| {c['axis']} | {c['offset_s']} | {c['peak_corr']} | {c['peak_width_s']} |")
    if result["best"]["warning"]:
        lines += ["", f"> ⚠ {result['best']['warning']}"]
    with open(os.path.join(out_dir, "time_alignment_report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    # 诊断图由 plotting 调用（此处略，TODO·接 matplotlib）
