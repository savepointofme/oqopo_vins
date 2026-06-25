"""segmentation.py — four-side flight segments + refined straight windows.

不固定时间粗切。基于 GPS 轨迹/course/速度/局部几何检测直线，
主评价使用一圈四边结构：两条主直线边 + 两条转向连接边。straight
边界会裁剪到 GPS course 稳定窗口；被裁掉的过渡样本并入相邻
connector/partial，不丢弃任何样本。500m fine split 不作为默认主口径。

段级漂移区分:
  局部段漂移（主判据）: 段起点误差归零后，段内产生的漂移。
  全局段误差（参考）  : 到达本段时的累计误差水平，不替代局部。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd


@dataclass
class SegConfig:
    mode: str = "four_side_lap"
    min_straight_len_m: float = 2000.0
    straight_split_len_m: float = 0.0
    add_straight_split: bool = False
    min_speed_mps: float = 5.0
    max_course_rate_degps: float = 3.0     # TODO·本地核验合适阈值
    max_line_fit_rmse_m: float = 5.0       # TODO·本地核验
    max_heading_std_deg: float = 5.0       # TODO·本地核验
    short_len_m: float = 100.0
    primary_heading_tolerance_deg: float = 20.0
    min_primary_side_len_m: float = 2000.0
    heading_tol_deg: float = 4.0
    stable_min_s: float = 2.5
    course_rate_tol_deg_s: float = 1.5
    stable_gap_bridge_s: float = 1.0
    terminal_heading_window_s: float = 3.0
    min_refined_straight_len_m: float = 1000.0

    def to_dict(self) -> dict:
        return self.__dict__.copy()


def _course_rate_degps(df: pd.DataFrame) -> np.ndarray:
    c = np.radians(df["gps_course_deg"].values)
    t = df["t"].values
    dc = np.degrees(np.unwrap(np.diff(c)))
    dt = np.diff(t)
    rate = np.zeros(len(df))
    rate[1:] = np.abs(dc / np.where(dt == 0, np.nan, dt))
    rate[0] = rate[1] if len(rate) > 1 else 0
    return rate


def classify_straight(df: pd.DataFrame, cfg: SegConfig) -> np.ndarray:
    """逐样本判定是否处于直线（速度足够 + course rate 小）。返回 bool 数组。

    局部 line-fit residual / heading std 作为可选加强判据（窗口滑动）。
    """
    rate = _course_rate_degps(df)
    speed = df["gps_speed_xy"].values
    is_line = (speed > cfg.min_speed_mps) & (rate < cfg.max_course_rate_degps)
    return is_line


def _runs(mask: np.ndarray):
    """返回连续 True 段的 (start, end_exclusive) 列表。"""
    out = []
    i = 0
    n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def segment(df: pd.DataFrame, cfg: SegConfig | None = None) -> pd.DataFrame:
    """对主表分段，写回 df 的 segment_id / segment_type / leg_id / segment_dist_m。

    返回 segment_index 表（每段一行）。
    """
    cfg = cfg or SegConfig()
    if cfg.mode == "four_side_lap":
        return segment_four_side_laps(df, cfg)
    n = len(df)
    seg_id = np.full(n, -1, dtype=int)
    seg_type = np.array(["unknown"] * n, dtype=object)
    leg_id = np.full(n, -1, dtype=int)

    is_line = classify_straight(df, cfg)
    dist = df["cum_dist"].values

    rows = []
    sid = 0
    leg_no = 0

    # 标记长直线 leg，再精细切分
    line_runs = _runs(is_line)
    long_legs = []
    for (a, b) in line_runs:
        seg_len = dist[b - 1] - dist[a]
        if seg_len >= cfg.min_straight_len_m:
            long_legs.append((a, b))

    covered = np.zeros(n, dtype=bool)

    for (a, b) in long_legs:
        leg_no += 1
        leg_start_dist = dist[a]
        if not cfg.add_straight_split or cfg.straight_split_len_m <= 0:
            sid += 1
            idx = np.arange(a, b)
            seg_id[idx] = sid
            seg_type[idx] = "straight"
            leg_id[idx] = leg_no
            covered[idx] = True
            rows.append({
                "segment_id": sid, "segment_type": "straight", "leg_id": leg_no,
                "label": f"直线段 L{leg_no}",
                "i0": int(a), "i1": int(b - 1),
            })
            continue
        # Optional debug split only; main official mode does not use it.
        sub_start = a
        while sub_start < b:
            lo = dist[sub_start] - leg_start_dist
            target = (np.floor(lo / cfg.straight_split_len_m) + 1) * cfg.straight_split_len_m
            # 找到该子段终点
            sub_end = sub_start
            while sub_end < b and (dist[sub_end] - leg_start_dist) < target:
                sub_end += 1
            # 若剩余不足一个 split 长度且几何上仍是直线 → 并入本子段（不丢给转弯）
            remaining = dist[b - 1] - dist[sub_end - 1] if sub_end <= b else 0
            if remaining < cfg.straight_split_len_m:
                sub_end = b
            sid += 1
            idx = np.arange(sub_start, sub_end)
            seg_id[idx] = sid
            seg_type[idx] = "straight"
            leg_id[idx] = leg_no
            covered[idx] = True
            lo_m = dist[sub_start] - leg_start_dist
            hi_m = dist[sub_end - 1] - leg_start_dist
            rows.append({
                "segment_id": sid, "segment_type": "straight", "leg_id": leg_no,
                "label": f"直线段 L{leg_no} · {int(round(lo_m))}–{int(round(hi_m))}m",
                "i0": int(sub_start), "i1": int(sub_end - 1),
            })
            sub_start = sub_end

    # 未覆盖的样本 → turn / transition / short
    uncovered = ~covered
    for (a, b) in _runs(uncovered):
        sid += 1
        seg_len = dist[b - 1] - dist[a]
        rate = _course_rate_degps(df.iloc[a:b])
        if seg_len < cfg.short_len_m:
            stype = "short"
            label = f"短段 S{sid} · {int(round(seg_len))}m"
        elif np.nanmean(rate) >= cfg.max_course_rate_degps:
            stype = "turn"
            label = f"转弯 T{sid}"
        else:
            stype = "transition"
            label = f"过渡 X{sid}"
        idx = np.arange(a, b)
        seg_id[idx] = sid
        seg_type[idx] = stype
        rows.append({
            "segment_id": sid, "segment_type": stype, "leg_id": -1,
            "label": label, "i0": int(a), "i1": int(b - 1),
        })

    df["segment_id"] = seg_id
    df["segment_type"] = seg_type
    df["leg_id"] = leg_id
    # 段内里程
    df["segment_dist_m"] = 0.0
    for r in rows:
        idx = np.arange(r["i0"], r["i1"] + 1)
        df.loc[df.index[idx], "segment_dist_m"] = dist[idx] - dist[idx[0]]

    seg_index = pd.DataFrame(rows).sort_values("segment_id").reset_index(drop=True)
    # 补充时间/里程范围
    for col in ("t_start", "t_end", "d_start", "d_end", "dist_m", "duration_s", "sample_count",
                "segment_heading_deg",
                "heading_window_start", "heading_window_end",
                "start_course_err_deg", "end_course_err_deg",
                "course_err_p50_deg", "course_err_p90_deg", "course_err_max_deg"):
        seg_index[col] = np.nan
    for col in ("heading_source", "heading_quality"):
        seg_index[col] = ""
    for k, r in seg_index.iterrows():
        seg = df.iloc[r["i0"]:r["i1"] + 1]
        v = seg[seg["valid"]] if seg["valid"].any() else seg
        heading_deg = _net_heading_deg(df, int(r["i0"]), int(r["i1"]) + 1)
        course_stats = _course_error_stats(df, int(r["i0"]), int(r["i1"]) + 1, heading_deg)
        seg_index.loc[k, "t_start"] = float(seg["t"].iloc[0])
        seg_index.loc[k, "t_end"] = float(seg["t"].iloc[-1])
        seg_index.loc[k, "d_start"] = float(seg["cum_dist"].iloc[0])
        seg_index.loc[k, "d_end"] = float(seg["cum_dist"].iloc[-1])
        seg_index.loc[k, "dist_m"] = round(float(seg["cum_dist"].iloc[-1] - seg["cum_dist"].iloc[0]), 1)
        seg_index.loc[k, "duration_s"] = round(float(seg["t"].iloc[-1] - seg["t"].iloc[0]), 1)
        seg_index.loc[k, "sample_count"] = int(len(v))
        seg_index.loc[k, "segment_heading_deg"] = heading_deg
        seg_index.loc[k, "heading_source"] = "generic_net_heading"
        seg_index.loc[k, "heading_quality"] = "generic"
        seg_index.loc[k, "heading_window_start"] = float(seg["t"].iloc[0])
        seg_index.loc[k, "heading_window_end"] = float(seg["t"].iloc[-1])
        for key, value in course_stats.items():
            seg_index.loc[k, key] = value
    return seg_index


def _wrap_rad(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _wrap_deg(a):
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def _circular_mean_rad(angle_rad: np.ndarray) -> float:
    finite = np.isfinite(angle_rad)
    if not finite.any():
        return float("nan")
    return float(np.arctan2(np.nanmean(np.sin(angle_rad[finite])),
                            np.nanmean(np.cos(angle_rad[finite]))))


def _net_heading_deg(df: pd.DataFrame, i0: int, i1_excl: int) -> float:
    part = df.iloc[i0:i1_excl]
    if len(part) < 2:
        return 0.0
    dE = float(part["gps_E"].iloc[-1] - part["gps_E"].iloc[0])
    dN = float(part["gps_N"].iloc[-1] - part["gps_N"].iloc[0])
    if np.hypot(dE, dN) >= 5.0:
        return float(np.degrees(np.arctan2(dN, dE)) % 360.0)
    heading = _circular_mean_rad(np.deg2rad(part["gps_course_deg"].to_numpy()))
    return float(np.degrees(heading) % 360.0) if np.isfinite(heading) else 0.0


def _course_error_stats(df: pd.DataFrame, i0: int, i1_excl: int,
                        heading_deg: float) -> dict:
    part = df.iloc[i0:i1_excl]
    diff = np.abs(_wrap_deg(part["gps_course_deg"].to_numpy() - heading_deg))
    diff = diff[np.isfinite(diff)]
    if len(diff) == 0:
        return {
            "start_course_err_deg": None,
            "end_course_err_deg": None,
            "course_err_p50_deg": None,
            "course_err_p90_deg": None,
            "course_err_max_deg": None,
        }
    return {
        "start_course_err_deg": round(float(diff[0]), 2),
        "end_course_err_deg": round(float(diff[-1]), 2),
        "course_err_p50_deg": round(float(np.percentile(diff, 50)), 2),
        "course_err_p90_deg": round(float(np.percentile(diff, 90)), 2),
        "course_err_max_deg": round(float(np.max(diff)), 2),
    }


def _course_rate_local_degps(t: np.ndarray, course_deg: np.ndarray) -> np.ndarray:
    rate = np.zeros(len(course_deg), dtype=float)
    if len(course_deg) < 2:
        return rate
    dc = np.degrees(np.unwrap(np.diff(np.deg2rad(course_deg))))
    dt = np.diff(t)
    rate[1:] = np.abs(dc / np.where(dt == 0, np.nan, dt))
    rate[0] = rate[1]
    return rate


def _first_stable_run(stable: np.ndarray, t: np.ndarray, min_s: float):
    for a, b in _runs(stable):
        if b > a and float(t[b - 1] - t[a]) >= min_s:
            return a, b
    return None


def _last_stable_run(stable: np.ndarray, t: np.ndarray, min_s: float):
    found = None
    for a, b in _runs(stable):
        if b > a and float(t[b - 1] - t[a]) >= min_s:
            found = (a, b)
    return found


def _stable_clusters(stable: np.ndarray, t: np.ndarray,
                     min_s: float, max_gap_s: float) -> list[tuple[int, int]]:
    """Return stable clusters, bridging only very short unstable gaps."""
    runs = _runs(stable)
    if not runs:
        return []
    clusters = []
    ca, cb = runs[0]
    for a, b in runs[1:]:
        gap_s = float(t[a] - t[cb - 1]) if cb > ca else float("inf")
        if gap_s <= max_gap_s:
            cb = b
        else:
            if cb > ca and float(t[cb - 1] - t[ca]) >= min_s:
                clusters.append((ca, cb))
            ca, cb = a, b
    if cb > ca and float(t[cb - 1] - t[ca]) >= min_s:
        clusters.append((ca, cb))
    return clusters


def _choose_stable_cluster(clusters: list[tuple[int, int]],
                           dist: np.ndarray) -> tuple[int, int] | None:
    if not clusters:
        return None
    d0, d1 = float(dist[0]), float(dist[-1])
    mid_lo = d0 + 0.40 * (d1 - d0)
    mid_hi = d0 + 0.60 * (d1 - d0)

    def cluster_len(c: tuple[int, int]) -> float:
        a, b = c
        return float(dist[b - 1] - dist[a])

    mid_clusters = [
        c for c in clusters
        if float(dist[c[1] - 1]) >= mid_lo and float(dist[c[0]]) <= mid_hi
    ]
    pool = mid_clusters if mid_clusters else clusters
    return max(pool, key=cluster_len)


def _line_fit_stats(df: pd.DataFrame, i0: int, i1_excl: int,
                    heading_deg: float) -> dict:
    part = df.iloc[i0:i1_excl]
    if len(part) < 3:
        return {
            "line_fit_rmse_m": None,
            "line_fit_p95_m": None,
            "line_fit_max_m": None,
            "line_fit_slope": None,
        }
    theta = np.deg2rad(float(heading_deg))
    ca, sa = np.cos(theta), np.sin(theta)
    e = part["gps_E"].to_numpy(dtype=float) - float(part["gps_E"].iloc[0])
    n = part["gps_N"].to_numpy(dtype=float) - float(part["gps_N"].iloc[0])
    along = e * ca + n * sa
    cross = -e * sa + n * ca
    finite = np.isfinite(along) & np.isfinite(cross)
    if finite.sum() < 3 or float(np.nanmax(along[finite]) - np.nanmin(along[finite])) < 1.0:
        return {
            "line_fit_rmse_m": None,
            "line_fit_p95_m": None,
            "line_fit_max_m": None,
            "line_fit_slope": None,
        }
    slope, intercept = np.polyfit(along[finite], cross[finite], 1)
    residual = cross[finite] - (slope * along[finite] + intercept)
    abs_res = np.abs(residual)
    return {
        "line_fit_rmse_m": round(float(np.sqrt(np.mean(residual ** 2))), 3),
        "line_fit_p95_m": round(float(np.percentile(abs_res, 95)), 3),
        "line_fit_max_m": round(float(np.max(abs_res)), 3),
        "line_fit_slope": round(float(slope), 8),
    }


def _terminal_course_heading_deg(df: pd.DataFrame, i0: int, i1_excl: int,
                                 cfg: SegConfig,
                                 source_name: str) -> dict:
    """Heading at the segment end, using nearby GPS/FC velocity course."""
    part = df.iloc[i0:i1_excl]
    if len(part) == 0:
        return {
            "heading_deg": 0.0,
            "heading_source": "empty_segment_fallback",
            "heading_quality": "fallback_empty_segment",
            "heading_window_start": None,
            "heading_window_end": None,
        }
    course_deg = part["gps_course_deg"].to_numpy(dtype=float)
    speed = part["gps_speed_xy"].to_numpy(dtype=float)
    t = part["t"].to_numpy(dtype=float)
    moving = np.isfinite(course_deg) & (speed >= cfg.min_speed_mps)
    window_s = max(float(cfg.terminal_heading_window_s), float(cfg.stable_min_s))
    terminal = moving & (t >= float(t[-1]) - window_s)

    if terminal.sum() < 3:
        moving_idx = np.flatnonzero(moving)
        if len(moving_idx) > 0:
            dt = np.diff(t)
            finite_dt = dt[np.isfinite(dt) & (dt > 0)]
            med_dt = float(np.median(finite_dt)) if len(finite_dt) else 0.2
            need = max(3, int(np.ceil(window_s / max(med_dt, 1e-6))))
            keep = moving_idx[-min(len(moving_idx), need):]
            terminal = np.zeros(len(part), dtype=bool)
            terminal[keep] = True

    heading = _circular_mean_rad(np.deg2rad(course_deg[terminal]))
    if np.isfinite(heading):
        quality = "terminal_course_window" if terminal.sum() >= 3 else "terminal_course_sparse"
        win_idx = np.flatnonzero(terminal)
        return {
            "heading_deg": float(np.degrees(heading) % 360.0),
            "heading_source": source_name,
            "heading_quality": quality,
            "heading_window_start": float(t[win_idx[0]]),
            "heading_window_end": float(t[win_idx[-1]]),
        }

    fallback = _net_heading_deg(df, i0, i1_excl)
    return {
        "heading_deg": fallback,
        "heading_source": "fallback_net_displacement",
        "heading_quality": "fallback_no_terminal_course",
        "heading_window_start": float(part["t"].iloc[0]),
        "heading_window_end": float(part["t"].iloc[-1]),
    }


def _refine_straight_window(df: pd.DataFrame, a: int, b: int,
                            cfg: SegConfig) -> dict:
    """Trim a coarse primary side to the stable GPS-course straight window."""
    part = df.iloc[a:b]
    t = part["t"].to_numpy()
    dist = part["cum_dist"].to_numpy()
    course_deg = part["gps_course_deg"].to_numpy()
    speed = part["gps_speed_xy"].to_numpy()
    raw_heading = _net_heading_deg(df, a, b)
    if len(part) < 3:
        return {
            "i0": a, "i1_excl": b, "heading_deg": raw_heading,
            "heading_window_start": float(part["t"].iloc[0]) if len(part) else None,
            "heading_window_end": float(part["t"].iloc[-1]) if len(part) else None,
            "heading_source": "gps_course_raw_segment_fallback",
            "heading_quality": "fallback_raw_segment",
        }

    d0, d1 = float(dist[0]), float(dist[-1])
    mid = (dist >= d0 + 0.40 * (d1 - d0)) & (dist <= d0 + 0.60 * (d1 - d0))
    moving = np.isfinite(course_deg) & (speed >= cfg.min_speed_mps)
    seed = mid & moving
    if seed.sum() < 3:
        seed = moving
    main_rad = _circular_mean_rad(np.deg2rad(course_deg[seed]))
    if not np.isfinite(main_rad):
        main_rad = np.deg2rad(raw_heading)
    main_deg = float(np.degrees(main_rad) % 360.0)

    rate = _course_rate_local_degps(t, course_deg)
    stable = (
        moving
        & (np.abs(_wrap_deg(course_deg - main_deg)) <= cfg.heading_tol_deg)
        & (rate <= cfg.course_rate_tol_deg_s)
    )
    cluster = _choose_stable_cluster(
        _stable_clusters(stable, t, cfg.stable_min_s, cfg.stable_gap_bridge_s),
        dist,
    )
    if cluster is None:
        quality = "fallback_raw_segment"
        refined_a, refined_b = a, b
        heading_mask = moving
    else:
        refined_a, refined_b = a + cluster[0], a + cluster[1]
        refined_dist = float(df["cum_dist"].iloc[refined_b - 1] - df["cum_dist"].iloc[refined_a])
        if refined_b <= refined_a + 2 or refined_dist < cfg.min_refined_straight_len_m:
            quality = "fallback_raw_segment"
            refined_a, refined_b = a, b
            heading_mask = moving
        else:
            quality = "refined_stable_window"
            local0, local1 = refined_a - a, refined_b - a
            heading_mask = stable.copy()
            keep = np.zeros_like(heading_mask, dtype=bool)
            keep[local0:local1] = True
            heading_mask &= keep

    if heading_mask.sum() < 3:
        heading_mask = moving
    heading = _circular_mean_rad(np.deg2rad(course_deg[heading_mask]))
    heading_deg = float(np.degrees(heading) % 360.0) if np.isfinite(heading) else main_deg
    line_stats = _line_fit_stats(df, int(refined_a), int(refined_b), heading_deg)
    if (
        quality == "refined_stable_window"
        and line_stats["line_fit_rmse_m"] is not None
        and float(line_stats["line_fit_rmse_m"]) > cfg.max_line_fit_rmse_m
    ):
        quality = "fallback_raw_segment_line_fit_failed"
        refined_a, refined_b = a, b
        heading_mask = moving
        heading = _circular_mean_rad(np.deg2rad(course_deg[heading_mask]))
        heading_deg = float(np.degrees(heading) % 360.0) if np.isfinite(heading) else main_deg
        line_stats = _line_fit_stats(df, int(refined_a), int(refined_b), heading_deg)

    out = {
        "i0": int(refined_a),
        "i1_excl": int(refined_b),
        "heading_deg": heading_deg,
        "heading_window_start": float(df["t"].iloc[refined_a]),
        "heading_window_end": float(df["t"].iloc[refined_b - 1]),
        "heading_source": (
            "gps_course_stable_window"
            if quality == "refined_stable_window"
            else "gps_course_raw_segment_fallback"
        ),
        "heading_quality": quality,
    }
    out.update(line_stats)
    return out


def _apply_segment_route_frame(df: pd.DataFrame, seg_index: pd.DataFrame) -> None:
    """Attach a fixed segment frame without overwriting canonical route errors.

    ``err_along``/``err_cross`` are defined against the instantaneous GPS
    course by :mod:`flight_eval.metrics`.  A fixed segment frame is useful for
    the segment browser, but it is a separate diagnostic and therefore uses
    explicit ``segment_err_*`` column names.
    """
    df["segment_heading_deg"] = np.nan
    df["heading_source"] = ""
    df["heading_quality"] = ""
    for _, row in seg_index.iterrows():
        idx = np.arange(int(row["i0"]), int(row["i1"]) + 1)
        theta = np.deg2rad(float(row["segment_heading_deg"]))
        ca, sa = np.cos(theta), np.sin(theta)
        eE = df.loc[df.index[idx], "err_E"].to_numpy()
        eN = df.loc[df.index[idx], "err_N"].to_numpy()
        evE = df.loc[df.index[idx], "err_vE"].to_numpy()
        evN = df.loc[df.index[idx], "err_vN"].to_numpy()
        df.loc[df.index[idx], "segment_heading_deg"] = float(row["segment_heading_deg"])
        df.loc[df.index[idx], "heading_source"] = row.get("heading_source", "")
        df.loc[df.index[idx], "heading_quality"] = row.get("heading_quality", "")
        df.loc[df.index[idx], "segment_err_along"] = eE * ca + eN * sa
        df.loc[df.index[idx], "segment_err_cross"] = -eE * sa + eN * ca
        df.loc[df.index[idx], "segment_err_v_along"] = evE * ca + evN * sa
        df.loc[df.index[idx], "segment_err_v_cross"] = -evE * sa + evN * ca
    df["segment_projection_identity_error_m2"] = np.abs(
        df["segment_err_along"] ** 2 + df["segment_err_cross"] ** 2
        - (df["err_E"] ** 2 + df["err_N"] ** 2)
    )
    df["segment_velocity_projection_identity_error_m2ps2"] = np.abs(
        df["segment_err_v_along"] ** 2 + df["segment_err_v_cross"] ** 2
        - (df["err_vE"] ** 2 + df["err_vN"] ** 2)
    )


def _primary_axis(course: np.ndarray, speed: np.ndarray, min_speed: float) -> float:
    """Find the dominant bidirectional route axis, modulo 180 degrees."""
    moving = np.isfinite(course) & np.isfinite(speed) & (speed >= min_speed)
    if moving.sum() < 20:
        raise ValueError("insufficient moving samples for four-side lap segmentation")
    candidates = np.linspace(-np.pi / 2.0, np.pi / 2.0, 1801)
    score = []
    for angle in candidates:
        residual = np.minimum(
            np.abs(_wrap_rad(course[moving] - angle)),
            np.abs(_wrap_rad(course[moving] - angle - np.pi)),
        )
        score.append(np.median(residual))
    return float(candidates[int(np.argmin(score))])


def segment_four_side_laps(df: pd.DataFrame, cfg: SegConfig) -> pd.DataFrame:
    """Split a repeated out-and-back route into four meaningful sides per lap.

    Side 1: primary-direction straight; side 2: far-end connector/turn;
    side 3: opposite-direction straight; side 4: near-end connector/turn.
    Start/end remnants remain explicit partial segments and are excluded from
    complete-lap comparison plots.
    """
    n = len(df)
    course = np.deg2rad(df["gps_course_deg"].to_numpy())
    speed = df["gps_speed_xy"].to_numpy()
    dist = df["cum_dist"].to_numpy()
    axis = _primary_axis(course, speed, cfg.min_speed_mps)
    residual0 = np.abs(_wrap_rad(course - axis))
    residual1 = np.abs(_wrap_rad(course - axis - np.pi))
    direction = (residual1 < residual0).astype(int)
    residual = np.minimum(residual0, residual1)
    stable = (
        np.isfinite(residual)
        & (speed >= cfg.min_speed_mps)
        & (residual <= np.deg2rad(cfg.primary_heading_tolerance_deg))
    )

    primary_runs = []
    for a, b in _runs(stable):
        split = a
        for i in range(a + 1, b):
            if direction[i] != direction[i - 1]:
                if dist[i - 1] - dist[split] >= cfg.min_primary_side_len_m:
                    primary_runs.append((split, i, int(direction[i - 1])))
                split = i
        if dist[b - 1] - dist[split] >= cfg.min_primary_side_len_m:
            primary_runs.append((split, b, int(direction[b - 1])))

    # Pick the lap origin direction that yields the most complete A-B-A cycles.
    candidates = []
    for origin_direction in (0, 1):
        origin_indices = [
            i for i, run in enumerate(primary_runs) if run[2] == origin_direction
        ]
        complete = []
        for left, right in zip(origin_indices[:-1], origin_indices[1:]):
            between = [
                i for i in range(left + 1, right)
                if primary_runs[i][2] != origin_direction
            ]
            if between:
                complete.append((left, between[0], right))
        candidates.append((len(complete), origin_direction, complete))
    _, origin_direction, laps = max(candidates, key=lambda item: item[0])
    if not laps:
        raise ValueError("no complete four-side lap found")

    runs_needed = set()
    for left, opposite, right in laps:
        runs_needed.update((left, opposite, right))
    refined = {
        run_idx: _refine_straight_window(df, primary_runs[run_idx][0],
                                         primary_runs[run_idx][1], cfg)
        for run_idx in runs_needed
    }

    segment_id = np.full(n, -1, dtype=int)
    segment_type = np.array(["partial"] * n, dtype=object)
    leg_id = np.full(n, -1, dtype=int)
    lap_id = np.full(n, -1, dtype=int)
    side_id = np.full(n, -1, dtype=int)
    rows = []
    sid = 0

    def add(a: int, b: int, lap: int, side: int, stype: str, label: str,
            *, raw_a: int | None = None, raw_b: int | None = None,
            straight_meta: dict | None = None) -> None:
        nonlocal sid
        if b <= a:
            return
        sid += 1
        idx = np.arange(a, b)
        segment_id[idx] = sid
        segment_type[idx] = stype
        leg_id[idx] = side if stype == "straight" else -1
        lap_id[idx] = lap
        side_id[idx] = side
        raw_a = a if raw_a is None else raw_a
        raw_b = b if raw_b is None else raw_b
        if straight_meta is not None:
            heading_deg = straight_meta["heading_deg"]
            heading_source = straight_meta["heading_source"]
            heading_quality = straight_meta["heading_quality"]
            heading_window_start = straight_meta["heading_window_start"]
            heading_window_end = straight_meta["heading_window_end"]
            line_stats = {
                "line_fit_rmse_m": straight_meta.get("line_fit_rmse_m"),
                "line_fit_p95_m": straight_meta.get("line_fit_p95_m"),
                "line_fit_max_m": straight_meta.get("line_fit_max_m"),
                "line_fit_slope": straight_meta.get("line_fit_slope"),
            }
        elif stype in ("connector", "turn", "transition"):
            meta = _terminal_course_heading_deg(
                df, a, b, cfg, "gps_course_terminal_window_for_turn"
            )
            heading_deg = meta["heading_deg"]
            heading_source = meta["heading_source"]
            heading_quality = meta["heading_quality"]
            heading_window_start = meta["heading_window_start"]
            heading_window_end = meta["heading_window_end"]
            line_stats = {
                "line_fit_rmse_m": None,
                "line_fit_p95_m": None,
                "line_fit_max_m": None,
                "line_fit_slope": None,
            }
        else:
            meta = _terminal_course_heading_deg(
                df, a, b, cfg, "gps_course_terminal_window_for_partial"
            )
            heading_deg = meta["heading_deg"]
            heading_source = meta["heading_source"]
            heading_quality = meta["heading_quality"]
            heading_window_start = meta["heading_window_start"]
            heading_window_end = meta["heading_window_end"]
            line_stats = {
                "line_fit_rmse_m": None,
                "line_fit_p95_m": None,
                "line_fit_max_m": None,
                "line_fit_slope": None,
            }
        row = {
            "segment_id": sid, "segment_type": stype, "leg_id": side,
            "lap_id": lap, "side_id": side, "label": label,
            "i0": int(a), "i1": int(b - 1),
            "i0_raw": int(raw_a), "i1_raw": int(raw_b - 1),
            "route_axis_deg": float(np.degrees(axis) % 180.0),
            "segment_heading_deg": round(float(heading_deg), 3),
            "heading_source": heading_source,
            "heading_quality": heading_quality,
            "heading_window_start": heading_window_start,
            "heading_window_end": heading_window_end,
        }
        row.update(line_stats)
        row.update(_course_error_stats(df, a, b, heading_deg))
        rows.append(row)

    first_start = refined[laps[0][0]]["i0"]
    add(0, first_start, -1, 0, "partial", "起始残段（不计入完整圈）")
    side_names = {
        1: "主航线去程边", 2: "远端转向连接边",
        3: "主航线回程边", 4: "近端转向连接边",
    }
    for lap_no, (left, opposite, right) in enumerate(laps, start=1):
        a0_raw, a1_raw, _ = primary_runs[left]
        b0_raw, b1_raw, _ = primary_runs[opposite]
        w1 = refined[left]
        w3 = refined[opposite]
        w_next = refined[right]
        add(w1["i0"], w1["i1_excl"], lap_no, 1, "straight",
            f"第{lap_no}圈·边1 {side_names[1]}",
            raw_a=a0_raw, raw_b=a1_raw, straight_meta=w1)
        add(w1["i1_excl"], w3["i0"], lap_no, 2, "connector",
            f"第{lap_no}圈·边2 {side_names[2]}")
        add(w3["i0"], w3["i1_excl"], lap_no, 3, "straight",
            f"第{lap_no}圈·边3 {side_names[3]}",
            raw_a=b0_raw, raw_b=b1_raw, straight_meta=w3)
        add(w3["i1_excl"], w_next["i0"], lap_no, 4, "connector",
            f"第{lap_no}圈·边4 {side_names[4]}")
    last_start = refined[laps[-1][2]]["i0"]
    add(last_start, n, -1, 0, "partial", "结束残段（不计入完整圈）")

    df["segment_id"] = segment_id
    df["segment_type"] = segment_type
    df["leg_id"] = leg_id
    df["lap_id"] = lap_id
    df["side_id"] = side_id
    df["segment_dist_m"] = 0.0
    for row in rows:
        idx = np.arange(row["i0"], row["i1"] + 1)
        df.loc[df.index[idx], "segment_dist_m"] = dist[idx] - dist[idx[0]]

    seg_index = pd.DataFrame(rows)
    for col in (
        "t_start_raw", "t_end_raw", "t_start", "t_end",
        "d_start", "d_end", "dist_m", "duration_s", "sample_count",
    ):
        seg_index[col] = np.nan
    for k, row in seg_index.iterrows():
        part = df.iloc[int(row["i0"]):int(row["i1"]) + 1]
        raw_part = df.iloc[int(row["i0_raw"]):int(row["i1_raw"]) + 1]
        valid = part[part["valid"]] if part["valid"].any() else part
        seg_index.loc[k, "t_start_raw"] = float(raw_part["t"].iloc[0])
        seg_index.loc[k, "t_end_raw"] = float(raw_part["t"].iloc[-1])
        seg_index.loc[k, "t_start"] = float(part["t"].iloc[0])
        seg_index.loc[k, "t_end"] = float(part["t"].iloc[-1])
        seg_index.loc[k, "d_start"] = float(part["cum_dist"].iloc[0])
        seg_index.loc[k, "d_end"] = float(part["cum_dist"].iloc[-1])
        seg_index.loc[k, "dist_m"] = round(float(part["cum_dist"].iloc[-1] - part["cum_dist"].iloc[0]), 1)
        seg_index.loc[k, "duration_s"] = round(float(part["t"].iloc[-1] - part["t"].iloc[0]), 1)
        seg_index.loc[k, "sample_count"] = int(len(valid))
    _apply_segment_route_frame(df, seg_index)
    return seg_index


def segment_errors(df: pd.DataFrame, seg_index: pd.DataFrame) -> pd.DataFrame:
    """Per-segment summary using the refined segment heading frame.

    ``local_final_along_error_m`` and ``local_final_cross_error_m`` are the
    change in ``VIO - GPS`` error since the segment start, projected onto the
    fixed ``segment_heading_deg`` frame.  The corresponding RMSE values use
    the same start-subtracted series.
    """
    if "segment_heading_deg" not in seg_index.columns:
        raise ValueError("segment_index missing segment_heading_deg")
    _apply_segment_route_frame(df, seg_index)
    rows = []
    for _, r in seg_index.iterrows():
        seg = df.iloc[int(r["i0"]):int(r["i1"]) + 1]
        v = seg[seg["valid"]]
        if len(v) < 2:
            continue

        dist_m = float(v["cum_dist"].iloc[-1] - v["cum_dist"].iloc[0])
        theta = np.deg2rad(float(r["segment_heading_deg"]))
        ca, sa = np.cos(theta), np.sin(theta)
        local_e = v["err_E"].to_numpy() - float(v["err_E"].iloc[0])
        local_n = v["err_N"].to_numpy() - float(v["err_N"].iloc[0])
        local_vert = (
            v["err_vertical"].to_numpy()
            - float(v["err_vertical"].iloc[0])
        )
        local_along = local_e * ca + local_n * sa
        local_cross = -local_e * sa + local_n * ca

        lf_along = float(local_along[-1])
        lf_cross = float(local_cross[-1])
        lf_vert = float(local_vert[-1])
        lf_xy = float(np.hypot(lf_along, lf_cross))

        local_pct       = round(lf_xy / dist_m * 100, 2) if dist_m > 50 else None
        local_cross_pct = round(abs(lf_cross) / dist_m * 100, 2) if dist_m > 50 else None

        along_rmse = float(np.sqrt(np.mean(local_along ** 2)))
        cross_rmse = float(np.sqrt(np.mean(local_cross ** 2)))
        vert_rmse = float(np.sqrt(np.mean(local_vert ** 2)))
        v_along_rmse = float(np.sqrt(np.mean(v["segment_err_v_along"] ** 2)))
        v_cross_rmse = float(np.sqrt(np.mean(v["segment_err_v_cross"] ** 2)))
        v_vert_rmse = float(np.sqrt(np.mean(v["err_v_vertical"] ** 2)))

        e1_row = v.iloc[-1]
        global_rmse = float(np.sqrt(np.mean(v["err_XY"] ** 2)))

        rows.append({
            "segment_id":                   int(r["segment_id"]),
            "segment_type":                 r["segment_type"],
            "lap_id":                       int(r.get("lap_id", -1)),
            "side_id":                      int(r.get("side_id", -1)),
            "label":                        r.get("label", f"segment {int(r['segment_id'])}"),
            "t_start_raw":                  round(float(r.get("t_start_raw", r["t_start"])), 3),
            "t_end_raw":                    round(float(r.get("t_end_raw", r["t_end"])), 3),
            "t_start":                      round(float(r["t_start"]), 3),
            "t_end":                        round(float(r["t_end"]), 3),
            "dist_m":                       round(dist_m, 1),
            "segment_heading_deg":          round(float(r["segment_heading_deg"]), 3),
            "heading_source":               r.get("heading_source", ""),
            "heading_quality":              r.get("heading_quality", ""),
            "heading_window_start":         r.get("heading_window_start"),
            "heading_window_end":           r.get("heading_window_end"),
            "line_fit_rmse_m":              r.get("line_fit_rmse_m"),
            "line_fit_p95_m":               r.get("line_fit_p95_m"),
            "line_fit_max_m":               r.get("line_fit_max_m"),
            "line_fit_slope":               r.get("line_fit_slope"),
            "start_course_err_deg":         r.get("start_course_err_deg"),
            "end_course_err_deg":           r.get("end_course_err_deg"),
            "course_err_p50_deg":           r.get("course_err_p50_deg"),
            "course_err_p90_deg":           r.get("course_err_p90_deg"),
            "course_err_max_deg":           r.get("course_err_max_deg"),
            # 局部系（主判据）
            "local_final_along_error_m":    round(lf_along, 2),
            "local_final_cross_error_m":    round(lf_cross, 2),
            "local_final_vertical_error_m": round(lf_vert, 2),
            "local_final_xy_error_m":       round(lf_xy, 2),
            "local_drift_percent":          local_pct,
            "local_cross_drift_percent":    local_cross_pct,
            "along_rmse_m":                 round(along_rmse, 2),
            "cross_rmse_m":                 round(cross_rmse, 2),
            "vertical_rmse_m":              round(vert_rmse, 2),
            "v_along_rmse_mps":             round(v_along_rmse, 3),
            "v_cross_rmse_mps":             round(v_cross_rmse, 3),
            "v_vertical_rmse_mps":           round(v_vert_rmse, 3),
            "local_final_v_along_error_mps": round(float(e1_row["segment_err_v_along"]), 3),
            "local_final_v_cross_error_mps": round(float(e1_row["segment_err_v_cross"]), 3),
            "local_final_v_vertical_error_mps": round(float(e1_row["err_v_vertical"]), 3),
            # 全局参考
            "global_xy_rmse_m":             round(global_rmse, 2),
            "global_final_xy_error_m":      round(float(e1_row["err_XY"]), 2),
            "speed_rmse_mps":               round(float(np.sqrt(np.mean(v["err_speed_xy"] ** 2))), 3),
            "vxy_vec_rmse_mps":             round(float(np.sqrt(np.mean(v["err_vXY_vec"] ** 2))), 3),
        })
    return pd.DataFrame(rows)
