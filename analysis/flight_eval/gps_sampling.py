"""gps_sampling.py — 以 GPS 更新时间为采样网格.

最终统计必须以 GPS 更新时间为主采样网格。GPS 频率低（~5Hz），
VIO ~30Hz。不要把 GPS 上采样到 30Hz 后统计。

对每个 GPS 更新时间 t_gps:
  1. 找到其后第一个 VIO 更新后状态 → vio_t；
  2. 记录 vio_delay_s = vio_t - t_gps；
  3. 延迟超阈值 → valid=False；
  4. 不做线性插值伪造 GPS 更新点；
  5. GPS 缺口两侧不算伪速度；
  6. 缺口/无效样本保留行（valid/gap 标记），不删除。
"""
from __future__ import annotations

import numpy as np
import pandas as pd


def detect_gps_gaps(t_gps: np.ndarray, nominal_dt: float | None = None,
                    gap_factor: float = 3.0) -> list[tuple[float, float]]:
    """检测 GPS 更新间隔异常大的缺口。返回 [(t_start, t_end), ...]。"""
    if len(t_gps) < 3:
        return []
    dt = np.diff(t_gps)
    if nominal_dt is None:
        nominal_dt = float(np.median(dt))
    gaps = []
    for i, d in enumerate(dt):
        if d > gap_factor * nominal_dt:
            gaps.append((float(t_gps[i]), float(t_gps[i + 1])))
    return gaps


def sample_on_gps_grid(t_gps: np.ndarray, t_vio: np.ndarray,
                       max_delay_s: float) -> pd.DataFrame:
    """为每个 t_gps 找其后第一个 VIO 状态。

    返回 DataFrame: gps_idx, t, vio_idx, vio_t, vio_delay_s, valid。
    无后继 VIO（VIO 早结束）→ vio_idx=-1, valid=False。
    """
    t_vio = np.asarray(t_vio)
    rows = []
    for gi, tg in enumerate(t_gps):
        j = np.searchsorted(t_vio, tg, side="left")  # 第一个 >= tg
        if j >= len(t_vio):
            rows.append((gi, float(tg), -1, np.nan, np.nan, False))
            continue
        vt = float(t_vio[j])
        delay = vt - float(tg)
        valid = delay <= max_delay_s
        rows.append((gi, float(tg), int(j), vt, delay, bool(valid)))
    return pd.DataFrame(
        rows, columns=["gps_idx", "t", "vio_idx", "vio_t", "vio_delay_s", "valid"]
    )


def mark_gaps(samples: pd.DataFrame, gaps: list[tuple[float, float]]) -> pd.DataFrame:
    """标记落在 GPS 缺口内的样本 gap=True 且 valid=False。"""
    samples = samples.copy()
    samples["gap"] = False
    for a, b in gaps:
        m = (samples["t"] > a) & (samples["t"] < b)
        samples.loc[m, "gap"] = True
        samples.loc[m, "valid"] = False
    return samples


def quality_table(samples: pd.DataFrame, gaps: list[tuple[float, float]],
                  velocity_source: str) -> dict:
    """生成 gps_sampling_quality 指标 dict。"""
    delays = samples.loc[~samples["gap"], "vio_delay_s"].dropna().values
    valid = samples["valid"].sum()
    return {
        "gps_sample_count": int(len(samples)),
        "valid_aligned_count": int(valid),
        "invalid_delay_count": int(((samples["vio_delay_s"] > 0) & (~samples["valid"]) & (~samples["gap"])).sum()),
        "mean_delay_s": float(np.mean(delays)) if len(delays) else None,
        "max_delay_s": float(np.max(delays)) if len(delays) else None,
        "p95_delay_s": float(np.percentile(delays, 95)) if len(delays) else None,
        "gps_gap_count": int(len(gaps)),
        "max_gps_gap_s": float(max((b - a for a, b in gaps), default=0.0)),
        "velocity_source": velocity_source,
    }
