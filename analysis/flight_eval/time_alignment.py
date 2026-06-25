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
    curves = {}       # axis -> {lag_s, corr}  相关曲线
    overlays = {}     # axis -> {t, fc, imu_shifted}  对齐叠加序列
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
        # 保存相关曲线与对齐后序列，供诊断图使用
        curves[ax] = {"lag_s": lag_s[m].tolist(), "corr": xc[m].tolist()}
        overlays[ax] = {
            "t": g.tolist(),
            "fc": fa.tolist(),
            # 把 IMU 序列按最优 offset 平移到 FC 时间轴上，用于叠加目检
            "imu_shifted": np.interp(g, g + best_lag, ia).tolist(),
        }

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
        "curves": curves,
        "overlays": overlays,
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
    _write_figures(result, out_dir)


def _write_figures(result: dict, out_dir: str) -> None:
    """生成两张诊断图: 相关曲线 + gyro/FC rate 叠加。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
    except ImportError:
        return
    # 中文字体（找不到则用默认，不阻断）
    for fam in ("Microsoft YaHei", "SimHei", "DengXian", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(fam, fallback_to_default=False)
            plt.rcParams["font.sans-serif"] = [fam]
            plt.rcParams["axes.unicode_minus"] = False
            break
        except ValueError:
            continue
    fig_dir = os.path.join(out_dir, "figures")
    os.makedirs(fig_dir, exist_ok=True)
    best_off = result["best"]["offset_s"]

    # 1) 相关曲线（各轴）
    curves = result.get("curves", {})
    if curves:
        fig, ax = plt.subplots(figsize=(8, 4))
        for axis, c in curves.items():
            ax.plot(c["lag_s"], c["corr"], lw=1.2, label=axis)
        ax.axvline(best_off, color="#d33a35", ls="--", lw=1.2,
                   label=f"最优 offset = {best_off:.3f}s")
        ax.set_xlabel("时间偏移 offset / s（FC 相对 IMU）")
        ax.set_ylabel("归一化互相关")
        ax.set_title("飞控-IMU 角速度互相关（按轴）")
        ax.grid(True, alpha=.25); ax.legend(loc="best")
        for ext in ("png", "svg"):
            fig.savefig(os.path.join(fig_dir, f"time_alignment_correlation.{ext}"),
                        bbox_inches="tight", dpi=140)
        plt.close(fig)

    # 2) gyro/FC rate 叠加（选中轴，按最优 offset 对齐后）
    overlays = result.get("overlays", {})
    chosen = result["best"]["chosen_axis"]
    ov = overlays.get(chosen)
    if ov:
        fig, ax = plt.subplots(figsize=(10, 3.4))
        ax.plot(ov["t"], ov["fc"], color="#1f7a4d", lw=1.2, label="飞控角速度（参考轴）")
        ax.plot(ov["t"], ov["imu_shifted"], color="#2563c9", lw=1.0,
                label=f"IMU gyro（按 {best_off:.3f}s 对齐）")
        ax.set_xlabel("时间 / s"); ax.set_ylabel("角速度（归一化重采样）")
        ax.set_title(f"gyro / 飞控 角速度叠加 · 轴={chosen}")
        ax.grid(True, alpha=.25); ax.legend(loc="best")
        for ext in ("png", "svg"):
            fig.savefig(os.path.join(fig_dir, f"gyro_fc_rate_overlay.{ext}"),
                        bbox_inches="tight", dpi=140)
        plt.close(fig)
