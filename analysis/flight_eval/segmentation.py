"""segmentation.py — 精细 straight / turn 分段 + 局部/全局段漂移.

不固定时间粗切。基于 GPS 轨迹/course/速度/局部几何检测直线，
对长直线精细切分（500/1000m），直线之间归类为 turn/transition/short/unknown。
不丢弃任何样本。

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
    straight_split_len_m: float = 500.0
    min_speed_mps: float = 5.0
    max_course_rate_degps: float = 3.0     # TODO·本地核验合适阈值
    max_line_fit_rmse_m: float = 5.0       # TODO·本地核验
    max_heading_std_deg: float = 5.0       # TODO·本地核验
    short_len_m: float = 100.0
    primary_heading_tolerance_deg: float = 20.0
    min_primary_side_len_m: float = 2000.0

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
        # 精细切分: 0-split, split-2split, ...，余段并入最后一个子段
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
    for col in ("t_start", "t_end", "d_start", "d_end", "dist_m", "duration_s", "sample_count"):
        seg_index[col] = np.nan
    for k, r in seg_index.iterrows():
        seg = df.iloc[r["i0"]:r["i1"] + 1]
        v = seg[seg["valid"]] if seg["valid"].any() else seg
        seg_index.loc[k, "t_start"] = float(seg["t"].iloc[0])
        seg_index.loc[k, "t_end"] = float(seg["t"].iloc[-1])
        seg_index.loc[k, "d_start"] = float(seg["cum_dist"].iloc[0])
        seg_index.loc[k, "d_end"] = float(seg["cum_dist"].iloc[-1])
        seg_index.loc[k, "dist_m"] = round(float(seg["cum_dist"].iloc[-1] - seg["cum_dist"].iloc[0]), 1)
        seg_index.loc[k, "duration_s"] = round(float(seg["t"].iloc[-1] - seg["t"].iloc[0]), 1)
        seg_index.loc[k, "sample_count"] = int(len(v))
    return seg_index


def _wrap_rad(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


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

    segment_id = np.full(n, -1, dtype=int)
    segment_type = np.array(["partial"] * n, dtype=object)
    leg_id = np.full(n, -1, dtype=int)
    lap_id = np.full(n, -1, dtype=int)
    side_id = np.full(n, -1, dtype=int)
    rows = []
    sid = 0

    def add(a: int, b: int, lap: int, side: int, stype: str, label: str) -> None:
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
        rows.append({
            "segment_id": sid, "segment_type": stype, "leg_id": side,
            "lap_id": lap, "side_id": side, "label": label,
            "i0": int(a), "i1": int(b - 1),
            "route_axis_deg": float(np.degrees(axis) % 180.0),
        })

    first_start = primary_runs[laps[0][0]][0]
    add(0, first_start, -1, 0, "partial", "起始残段（不计入完整圈）")
    side_names = {
        1: "主航线去程边", 2: "远端转向连接边",
        3: "主航线回程边", 4: "近端转向连接边",
    }
    for lap_no, (left, opposite, right) in enumerate(laps, start=1):
        a0, a1, _ = primary_runs[left]
        b0, b1, _ = primary_runs[opposite]
        next0, _, _ = primary_runs[right]
        add(a0, a1, lap_no, 1, "straight", f"第{lap_no}圈·边1 {side_names[1]}")
        add(a1, b0, lap_no, 2, "connector", f"第{lap_no}圈·边2 {side_names[2]}")
        add(b0, b1, lap_no, 3, "straight", f"第{lap_no}圈·边3 {side_names[3]}")
        add(b1, next0, lap_no, 4, "connector", f"第{lap_no}圈·边4 {side_names[4]}")
    last_start = primary_runs[laps[-1][2]][0]
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
    for col in ("t_start", "t_end", "d_start", "d_end", "dist_m", "duration_s", "sample_count"):
        seg_index[col] = np.nan
    for k, row in seg_index.iterrows():
        part = df.iloc[int(row["i0"]):int(row["i1"]) + 1]
        valid = part[part["valid"]] if part["valid"].any() else part
        seg_index.loc[k, "t_start"] = float(part["t"].iloc[0])
        seg_index.loc[k, "t_end"] = float(part["t"].iloc[-1])
        seg_index.loc[k, "d_start"] = float(part["cum_dist"].iloc[0])
        seg_index.loc[k, "d_end"] = float(part["cum_dist"].iloc[-1])
        seg_index.loc[k, "dist_m"] = round(float(part["cum_dist"].iloc[-1] - part["cum_dist"].iloc[0]), 1)
        seg_index.loc[k, "duration_s"] = round(float(part["t"].iloc[-1] - part["t"].iloc[0]), 1)
        seg_index.loc[k, "sample_count"] = int(len(valid))
    return seg_index


def segment_errors(df: pd.DataFrame, seg_index: pd.DataFrame) -> pd.DataFrame:
    """每段误差汇总：段局部坐标系（主）+ 全局累计误差（参考）。

    局部坐标系定义：
      ea  = 该段 GPS 起止点净位移方向（end - start，归一化）
      ec  = ea 的左手垂直方向
      原点 = 该段 GPS 起始点
      VIO  先做段起点对齐（VIO 起点平移到 GPS 起点），再在局部系中量测

    这样对于回程边（side 3），ea 指向出发方向的反向，局部 along 误差
    仅反映段内 VIO 相对 GPS 的偏移积累，不包含之前段的累计漂移。
    """
    rows = []
    for _, r in seg_index.iterrows():
        seg = df.iloc[int(r["i0"]):int(r["i1"]) + 1]
        v = seg[seg["valid"]]
        if len(v) < 2:
            continue

        gps_e = v["gps_E"].values
        gps_n = v["gps_N"].values
        gps_u = v["gps_U"].values if "gps_U" in v.columns else np.zeros(len(v))
        vio_e = v["vio_E"].values
        vio_n = v["vio_N"].values
        vio_u = v["vio_U"].values if "vio_U" in v.columns else np.zeros(len(v))

        dist_m = float(v["cum_dist"].iloc[-1] - v["cum_dist"].iloc[0])

        # 段航向：GPS 起止点净位移方向
        dE = gps_e[-1] - gps_e[0]
        dN = gps_n[-1] - gps_n[0]
        seg_span = float(np.hypot(dE, dN))
        if seg_span >= 10.0:
            ea = np.array([dE / seg_span, dN / seg_span], dtype=float)
        else:
            # 直线短或原地：取该段 GPS 平均航向
            cr = np.deg2rad(v["gps_course_deg"].values)
            nm = float(np.hypot(np.mean(np.cos(cr)), np.mean(np.sin(cr))))
            ea = np.array([np.mean(np.cos(cr)), np.mean(np.sin(cr))], dtype=float)
            if nm > 0.05:
                ea /= nm
            else:
                ea = np.array([1.0, 0.0])
        ec = np.array([-ea[1], ea[0]], dtype=float)  # 左手垂直

        # GPS 相对段起点（局部系）
        gps_rel_e, gps_rel_n = gps_e - gps_e[0], gps_n - gps_n[0]
        gps_along = gps_rel_e * ea[0] + gps_rel_n * ea[1]
        gps_cross = gps_rel_e * ec[0] + gps_rel_n * ec[1]
        gps_vert  = gps_u - gps_u[0]

        # VIO 相对 VIO 段起点（局部系），段起点对齐到 GPS 起点
        vio_rel_e, vio_rel_n = vio_e - vio_e[0], vio_n - vio_n[0]
        vio_along = vio_rel_e * ea[0] + vio_rel_n * ea[1]
        vio_cross = vio_rel_e * ec[0] + vio_rel_n * ec[1]
        vio_vert  = vio_u - vio_u[0]

        # 局部误差（逐样本）
        loc_err_along = vio_along - gps_along
        loc_err_cross = vio_cross - gps_cross
        loc_err_vert  = vio_vert  - gps_vert

        lf_along = float(loc_err_along[-1])
        lf_cross = float(loc_err_cross[-1])
        lf_vert  = float(loc_err_vert[-1])
        lf_xy    = float(np.hypot(lf_along, lf_cross))

        local_pct       = round(lf_xy / dist_m * 100, 2) if dist_m > 50 else None
        local_cross_pct = round(abs(lf_cross) / dist_m * 100, 2) if dist_m > 50 else None

        along_rmse = float(np.sqrt(np.mean(loc_err_along ** 2)))
        cross_rmse = float(np.sqrt(np.mean(loc_err_cross ** 2)))
        vert_rmse  = float(np.sqrt(np.mean(loc_err_vert ** 2)))

        e1_row = v.iloc[-1]
        global_rmse = float(np.sqrt(np.mean(v["err_XY"] ** 2)))

        rows.append({
            "segment_id":                   int(r["segment_id"]),
            "segment_type":                 r["segment_type"],
            "lap_id":                       int(r.get("lap_id", -1)),
            "side_id":                      int(r.get("side_id", -1)),
            "label":                        r.get("label", f"segment {int(r['segment_id'])}"),
            "dist_m":                       round(dist_m, 1),
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
            # 全局参考
            "global_xy_rmse_m":             round(global_rmse, 2),
            "global_final_xy_error_m":      round(float(e1_row["err_XY"]), 2),
            "speed_rmse_mps":               round(float(np.sqrt(np.mean(v["err_speed_xy"] ** 2))), 3),
            "vxy_vec_rmse_mps":             round(float(np.sqrt(np.mean(v["err_vXY_vec"] ** 2))), 3),
        })
    return pd.DataFrame(rows)
