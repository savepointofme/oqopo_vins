#!/usr/bin/env python3
"""Standard GPS-grid analysis for one flight run, with optional comparisons."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any
import zipfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

plt.rcParams["font.sans-serif"] = [
    "Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS",
]
plt.rcParams["axes.unicode_minus"] = False


SUMMARY_FIELDS = [
    "condition", "flight", "duration_s", "gps_distance_km",
    "final_xy_error_m", "final_xy_drift_percent", "xy_rmse_m",
    "xy_rmse_percent", "final_along_error_m", "final_cross_error_m",
    "vertical_rmse_m", "speed_rmse_mps", "vxy_vec_rmse_mps",
    "yaw/course_final_deg", "yaw/course_rmse_deg", "notes",
]


def _numeric_frame(path: Path, comment: str | None = "#") -> pd.DataFrame:
    return pd.read_csv(path, comment=comment)


def load_gps(path: Path) -> pd.DataFrame:
    raw = _numeric_frame(path)
    if raw.shape[1] < 4:
        raise ValueError(f"GPS file needs at least four columns: {path}")
    numeric = raw.apply(pd.to_numeric, errors="coerce")
    out = numeric.iloc[:, :4].dropna().copy()
    out.columns = ["t_raw", "c1", "c2", "c3"]
    source_columns = {str(c).strip().lower(): c for c in raw.columns}
    velocity_columns = [
        next((source_columns[n] for n in names if n in source_columns), None)
        for names in [("ve", "gps_ve"), ("vn", "gps_vn"), ("vu", "gps_vu")]
    ]
    has_fc_velocity = all(name is not None for name in velocity_columns)
    if has_fc_velocity:
        for target, source in zip(["gps_vE", "gps_vN", "gps_vU"], velocity_columns):
            out[target] = numeric.loc[out.index, source]
    scale = 1e9 if out["t_raw"].abs().median() > 1e6 else 1.0
    out["t"] = out["t_raw"] / scale
    out = out.sort_values("t").drop_duplicates("t")
    c1 = out["c1"].to_numpy()
    c2 = out["c2"].to_numpy()
    is_wgs84 = bool(np.all(np.abs(c1) <= 90) and np.all(np.abs(c2) <= 180))
    if is_wgs84:
        lat = np.deg2rad(c1)
        lon = np.deg2rad(c2)
        alt = out["c3"].to_numpy()
        lat0, lon0, alt0 = lat[0], lon[0], alt[0]
        a = 6378137.0
        e2 = 6.69437999014e-3
        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        n = a / np.sqrt(1.0 - e2 * sin_lat * sin_lat)
        ecef = np.column_stack([
            (n + alt) * cos_lat * np.cos(lon),
            (n + alt) * cos_lat * np.sin(lon),
            (n * (1.0 - e2) + alt) * sin_lat,
        ])
        sin0, cos0 = math.sin(lat0), math.cos(lat0)
        sin_lon0, cos_lon0 = math.sin(lon0), math.cos(lon0)
        n0 = a / math.sqrt(1.0 - e2 * sin0 * sin0)
        origin = np.array([
            (n0 + alt0) * cos0 * cos_lon0,
            (n0 + alt0) * cos0 * sin_lon0,
            (n0 * (1.0 - e2) + alt0) * sin0,
        ])
        rot = np.array([
            [-sin_lon0, cos_lon0, 0.0],
            [-sin0 * cos_lon0, -sin0 * sin_lon0, cos0],
            [cos0 * cos_lon0, cos0 * sin_lon0, sin0],
        ])
        enu = (ecef - origin) @ rot.T
        out[["gps_E", "gps_N", "gps_U"]] = enu
    else:
        out["gps_E"] = out["c1"] - out["c1"].iloc[0]
        out["gps_N"] = out["c2"] - out["c2"].iloc[0]
        out["gps_U"] = out["c3"] - out["c3"].iloc[0]
    columns = ["t", "gps_E", "gps_N", "gps_U"]
    if has_fc_velocity:
        columns += ["gps_vE", "gps_vN", "gps_vU"]
    result = out[columns].reset_index(drop=True)
    result.attrs["velocity_source"] = (
        "flight_controller_enu" if has_fc_velocity else "gps_position_difference"
    )
    return result


def _input_lines(source: str | Path) -> list[str]:
    text = str(source)
    if "::" in text:
        archive, member = text.split("::", 1)
        with zipfile.ZipFile(archive) as zf:
            return zf.read(member).decode("utf-8", errors="replace").splitlines()
    return Path(text).read_text(encoding="utf-8", errors="replace").splitlines()


def load_traj(path: str | Path) -> pd.DataFrame:
    rows: list[list[float]] = []
    for line in _input_lines(path):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        vals = [float(v) for v in s.split()]
        if len(vals) >= 4:
            rows.append(vals[:8])
    if not rows:
        raise ValueError(f"No trajectory rows in {path}")
    cols = ["t", "x", "y", "z", "qx", "qy", "qz", "qw"][: len(rows[0])]
    return pd.DataFrame(rows, columns=cols).sort_values("t").drop_duplicates("t")


def load_bias_velocity(path: str | Path | None) -> pd.DataFrame | None:
    if path is None:
        return None
    if "::" not in str(path) and not Path(path).exists():
        return None
    rows = []
    for line in _input_lines(path):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        vals = [float(v) for v in s.split()]
        if len(vals) >= 4:
            rows.append(vals[:4])
    if not rows:
        return None
    return pd.DataFrame(rows, columns=["t", "vx", "vy", "vz"]).sort_values("t").drop_duplicates("t")


def smooth_velocity(t: np.ndarray, xyz: np.ndarray, window: int) -> np.ndarray:
    n = len(t)
    if n < 3:
        return np.full_like(xyz, np.nan)
    w = min(window, n if n % 2 else n - 1)
    w = max(3, w)
    if w % 2 == 0:
        w -= 1
    poly = min(3, w - 1)
    smoothed = savgol_filter(xyz, w, poly, axis=0, mode="interp") if w >= 5 else xyz
    return np.gradient(smoothed, t, axis=0, edge_order=1)


def segmented_velocity(
    t: np.ndarray,
    xyz: np.ndarray,
    max_gap_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Differentiate raw GPS updates only inside contiguous runs."""
    dt = np.diff(t)
    gap_before = np.r_[False, dt > max_gap_s]
    run_id = np.cumsum(gap_before)
    velocity = np.full_like(xyz, np.nan, dtype=float)
    for rid in np.unique(run_id):
        idx = np.flatnonzero(run_id == rid)
        if len(idx) < 3:
            continue
        run_velocity = np.gradient(xyz[idx], t[idx], axis=0, edge_order=1)
        # Run endpoints use one-sided differences beside missing data. Retain
        # their position rows, but exclude their velocity/course values.
        velocity[idx[1:-1]] = run_velocity[1:-1]
    return velocity, gap_before, run_id


def interp_columns(src_t: np.ndarray, values: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(dst_t, src_t, values[:, i]) for i in range(values.shape[1])])


def circular_mean(angle_rad: np.ndarray, weights: np.ndarray | None = None) -> float:
    if weights is None:
        weights = np.ones_like(angle_rad)
    return math.atan2(float(np.sum(weights * np.sin(angle_rad))),
                      float(np.sum(weights * np.cos(angle_rad))))


def wrap_deg(v: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(v) + 180.0) % 360.0 - 180.0


def metric_stats(v: np.ndarray) -> dict[str, float | int]:
    a = np.asarray(v, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {k: float("nan") for k in
                ["signed_mean", "mae", "rmse", "median", "p95", "max", "final"]} | {"n": 0}
    av = np.abs(a)
    return {
        "signed_mean": float(np.mean(a)),
        "mae": float(np.mean(av)),
        "rmse": float(np.sqrt(np.mean(a * a))),
        "median": float(np.median(av)),
        "p95": float(np.percentile(av, 95)),
        "max": float(np.max(av)),
        "final": float(a[-1]),
        "n": int(len(a)),
    }


def start_align(
    gps_t: np.ndarray,
    gps_pos: np.ndarray,
    gps_vel: np.ndarray,
    vio_pos: np.ndarray,
    vio_vel: np.ndarray,
    heading_window_s: float,
    heading_stable_window_s: float,
    min_speed_mps: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    speed_g = np.linalg.norm(gps_vel[:, :2], axis=1)
    speed_v = np.linalg.norm(vio_vel[:, :2], axis=1)
    course_g = np.arctan2(gps_vel[:, 1], gps_vel[:, 0])
    stable = np.zeros(len(gps_t), dtype=bool)
    search_end = gps_t[0] + heading_window_s
    for start in gps_t[gps_t <= search_end]:
        trial = (
            (gps_t >= start)
            & (gps_t <= start + heading_stable_window_s)
            & np.isfinite(speed_g)
            & np.isfinite(speed_v)
            & (speed_g > min_speed_mps)
            & (speed_v > min_speed_mps * 0.5)
        )
        if trial.sum() >= 10:
            mean_course = circular_mean(course_g[trial], speed_g[trial])
            course_spread = wrap_deg(np.rad2deg(course_g[trial] - mean_course))
        else:
            course_spread = np.array([])
        if trial.sum() >= 10 and float(np.std(course_spread)) <= 5.0:
            stable = trial
            break
    if stable.sum() < 3:
        stable = (
            np.isfinite(speed_g) & np.isfinite(speed_v)
            & (speed_g > min_speed_mps) & (speed_v > min_speed_mps * 0.5)
        )
        idx = np.flatnonzero(stable)[: max(3, min(30, stable.sum()))]
        stable = np.zeros(len(gps_t), dtype=bool)
        stable[idx] = True
    if stable.sum() < 2:
        yaw = 0.0
    else:
        # ENU course is atan2(N, E). The difference is therefore directly the
        # standard positive XY rotation that maps VIO motion onto GPS motion.
        hg = circular_mean(np.arctan2(gps_vel[stable, 1], gps_vel[stable, 0]), speed_g[stable])
        hv = circular_mean(np.arctan2(vio_vel[stable, 1], vio_vel[stable, 0]), speed_v[stable])
        yaw = hg - hv
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    pos_r = vio_pos @ r.T
    vel_r = vio_vel @ r.T
    pos_r += gps_pos[0] - pos_r[0]
    return pos_r, vel_r, math.degrees(yaw)


def local_geometry(pos: np.ndarray, course: np.ndarray, half: int = 5) -> tuple[np.ndarray, np.ndarray]:
    n = len(pos)
    heading_std = np.full(n, np.nan)
    line_rmse = np.full(n, np.nan)
    unwrapped = np.unwrap(course)
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        heading_std[i] = math.degrees(float(np.std(unwrapped[a:b])))
        pts = pos[a:b, :2]
        if len(pts) < 3:
            continue
        centered = pts - pts.mean(axis=0)
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        line_rmse[i] = float(np.sqrt(np.mean((centered @ normal) ** 2)))
    return heading_std, line_rmse


def segment_flight(
    df: pd.DataFrame,
    min_speed: float,
    max_course_rate: float,
    max_heading_std: float,
    max_line_rmse: float,
    min_straight_len: float,
    split_len: float,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    t = df["t"].to_numpy()
    dist = df["cum_dist_gps"].to_numpy()
    course_deg = df["gps_course_deg"].interpolate(limit_direction="both").to_numpy()
    course = np.deg2rad(course_deg)
    course_rate = np.abs(np.rad2deg(np.gradient(np.unwrap(course), t)))
    heading_std, line_rmse = local_geometry(df[["gps_E", "gps_N"]].to_numpy(), course)
    candidate = (
        (df["gps_speed_xy"].to_numpy() > min_speed)
        & (course_rate <= max_course_rate)
        & (heading_std <= max_heading_std)
        & (line_rmse <= max_line_rmse)
        & (~df["gps_gap_before"].to_numpy(dtype=bool))
    )
    # Bridge small geometry gaps inside otherwise straight legs.
    false_runs = contiguous_runs(~candidate)
    for a, b in false_runs:
        if a > 0 and b < len(candidate) - 1 and dist[b] - dist[a] <= 200.0:
            candidate[a:b + 1] = True
    straight_leg = np.full(len(df), -1, dtype=int)
    short = np.zeros(len(df), dtype=bool)
    leg = 0
    for a, b in contiguous_runs(candidate):
        length = dist[b] - dist[a]
        if length >= min_straight_len:
            straight_leg[a:b + 1] = leg
            leg += 1
        else:
            short[a:b + 1] = True
    seg_id = np.full(len(df), -1, dtype=int)
    seg_type = np.full(len(df), "unknown", dtype=object)
    rows: list[dict[str, Any]] = []
    sid = 0
    for leg_id in range(leg):
        idx = np.flatnonzero(straight_leg == leg_id)
        a, b = idx[0], idx[-1]
        start_dist = dist[a]
        bins = np.floor((dist[idx] - start_dist) / split_len).astype(int)
        for bin_id in np.unique(bins):
            sub = idx[bins == bin_id]
            seg_id[sub] = sid
            seg_type[sub] = "straight"
            rows.append(segment_row(df, sid, "straight", leg_id, sub[0], sub[-1],
                                    f"straight split {bin_id}, remainder retained"))
            sid += 1
    remaining = seg_id < 0
    raw_type = np.full(len(df), "unknown", dtype=object)
    raw_type[short] = "short"
    raw_type[remaining & (course_rate > max_course_rate)] = "turn"
    straight_edges = np.flatnonzero(straight_leg >= 0)
    if len(straight_edges):
        for i in np.flatnonzero(remaining & (raw_type == "unknown")):
            if np.min(np.abs(dist[straight_edges] - dist[i])) <= 300.0:
                raw_type[i] = "transition"
    for a, b in contiguous_value_runs(raw_type, remaining):
        kind = str(raw_type[a])
        idx = np.arange(a, b + 1)
        seg_id[idx] = sid
        seg_type[idx] = kind
        rows.append(segment_row(df, sid, kind, -1, a, b, "all non-straight samples retained"))
        sid += 1
    df["segment_id"] = seg_id
    df["segment_type"] = seg_type
    df["leg_id"] = straight_leg
    return df, pd.DataFrame(rows).sort_values("t_start").reset_index(drop=True)


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    idx = np.flatnonzero(mask)
    if not len(idx):
        return []
    cuts = np.flatnonzero(np.diff(idx) > 1)
    starts = np.r_[0, cuts + 1]
    ends = np.r_[cuts, len(idx) - 1]
    return [(int(idx[a]), int(idx[b])) for a, b in zip(starts, ends)]


def contiguous_value_runs(values: np.ndarray, mask: np.ndarray) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for i in range(len(values)):
        if not mask[i]:
            if start is not None:
                runs.append((start, i - 1))
                start = None
            continue
        if start is None:
            start = i
        elif values[i] != values[i - 1] or not mask[i - 1]:
            runs.append((start, i - 1))
            start = i
    if start is not None:
        runs.append((start, len(values) - 1))
    return runs


def segment_row(df: pd.DataFrame, sid: int, kind: str, leg: int, a: int, b: int, notes: str) -> dict[str, Any]:
    course_series = df["gps_course_deg"].iloc[a:b + 1].interpolate(
        limit_direction="both").dropna()
    course = np.unwrap(np.deg2rad(course_series.to_numpy()))
    heading_mean = math.degrees(circular_mean(course)) if len(course) else np.nan
    heading_change = math.degrees(course[-1] - course[0]) if len(course) else np.nan
    return {
        "segment_id": sid, "segment_type": kind, "leg_id": leg,
        "t_start": df["t"].iloc[a], "t_end": df["t"].iloc[b],
        "dist_start_m": df["cum_dist_gps"].iloc[a],
        "dist_end_m": df["cum_dist_gps"].iloc[b],
        "length_m": df["cum_dist_gps"].iloc[b] - df["cum_dist_gps"].iloc[a],
        "heading_mean_deg": heading_mean,
        "heading_change_deg": heading_change,
        "gps_speed_mean": df["gps_speed_xy"].iloc[a:b + 1].mean(),
        "notes": notes,
    }


def segment_summaries(
    df: pd.DataFrame,
    method: str,
    index: pd.DataFrame,
    min_drift_distance_m: float,
) -> pd.DataFrame:
    rows = []
    for _, seg in index.iterrows():
        s = df[df["segment_id"] == seg["segment_id"]]
        gps_dist = float(s["cum_dist_gps"].iloc[-1] - s["cum_dist_gps"].iloc[0])
        err0 = s[["err_E", "err_N"]].iloc[0].to_numpy()
        err1 = s[["err_E", "err_N"]].iloc[-1].to_numpy()
        error_growth = float(np.linalg.norm(err1 - err0))
        rows.append({
            "method": method, "segment_id": int(seg["segment_id"]),
            "segment_type": seg["segment_type"], "leg_id": int(seg["leg_id"]),
            "length_m": seg["length_m"], "duration_s": s["t"].iloc[-1] - s["t"].iloc[0],
            "gps_dist_m": gps_dist, "final_xy_error_m": s["err_XY"].iloc[-1],
            "start_xy_error_m": s["err_XY"].iloc[0],
            "xy_error_growth_m": error_growth,
            "xy_rmse_m": metric_stats(s["err_XY"].to_numpy())["rmse"],
            "xy_drift_percent": (
                100.0 * error_growth / gps_dist
                if gps_dist >= min_drift_distance_m else np.nan
            ),
            "along_rmse_m": metric_stats(s["err_along"].to_numpy())["rmse"],
            "cross_rmse_m": metric_stats(s["err_cross"].to_numpy())["rmse"],
            "vertical_rmse_m": metric_stats(s["err_vertical"].to_numpy())["rmse"],
            "speed_rmse_mps": metric_stats(s["err_speed_xy"].to_numpy())["rmse"],
            "vxy_vec_rmse_mps": metric_stats(s["err_vXY_vec"].to_numpy())["rmse"],
            "v_along_rmse_mps": metric_stats(s["err_v_along"].to_numpy())["rmse"],
            "v_cross_rmse_mps": metric_stats(s["err_v_cross"].to_numpy())["rmse"],
            "v_vertical_rmse_mps": metric_stats(s["err_v_vertical"].to_numpy())["rmse"],
            "yaw/course_error_mean_deg": metric_stats(s["course_error_deg"].to_numpy())["signed_mean"],
            "yaw/course_error_final_deg": metric_stats(
                s["course_error_deg"].to_numpy())["final"],
        })
    return pd.DataFrame(rows)


def build_aligned(
    gps: pd.DataFrame,
    traj: pd.DataFrame,
    bias: pd.DataFrame | None,
    t0: float | None,
    t1: float | None,
    heading_window: float,
    heading_stable_window: float,
    min_speed: float,
    max_vio_sample_delay_s: float,
    gps_gap_threshold_s: float,
) -> tuple[pd.DataFrame, float]:
    requested_lo = max(gps["t"].min(), t0 if t0 is not None else traj["t"].min())
    # Historical package windows can begin a fraction of a second before the
    # first emitted VIO state. Preserve that physical t0 and hold the first VIO
    # state over at most one second, matching the canonical start-yaw evaluator.
    lo = requested_lo if traj["t"].min() - requested_lo <= 1.0 else traj["t"].min()
    hi = min(gps["t"].max(), traj["t"].max(), t1 if t1 is not None else np.inf)
    g = gps[(gps["t"] >= lo) & (gps["t"] <= hi)].copy().reset_index(drop=True)
    if len(g) < 10:
        raise ValueError(f"Too few GPS samples in overlap [{lo}, {hi}]")
    tg = g["t"].to_numpy()
    gps_pos = g[["gps_E", "gps_N", "gps_U"]].to_numpy()
    tp = traj["t"].to_numpy()
    # The runner applies a GPS update after a camera update and then writes the
    # post-update state at t_cam. Map each GPS update to that first state;
    # interpolation at t_gps can mix pre/post-update states.
    post_idx = np.searchsorted(tp, tg, side="left")
    valid_post = post_idx < len(tp)
    post_idx_clip = np.clip(post_idx, 0, len(tp) - 1)
    sample_delay = tp[post_idx_clip] - tg
    valid_post &= sample_delay >= 0.0
    valid_post &= sample_delay <= max_vio_sample_delay_s
    g = g.loc[valid_post].reset_index(drop=True)
    tg = g["t"].to_numpy()
    gps_pos = g[["gps_E", "gps_N", "gps_U"]].to_numpy()
    post_idx = post_idx[valid_post]
    vio_t = tp[post_idx]
    sample_delay = vio_t - tg
    vio_pos = traj[["x", "y", "z"]].to_numpy()[post_idx]
    _, gap_before, gps_run_id = segmented_velocity(
        tg, gps_pos, gps_gap_threshold_s)
    if {"gps_vE", "gps_vN", "gps_vU"}.issubset(g.columns):
        gps_vel = g[["gps_vE", "gps_vN", "gps_vU"]].to_numpy()
        gps_velocity_source = "flight_controller_enu"
    else:
        gps_vel, _, _ = segmented_velocity(tg, gps_pos, gps_gap_threshold_s)
        gps_velocity_source = "gps_position_difference"
    if bias is not None:
        tb = bias["t"].to_numpy()
        bias_idx = np.searchsorted(tb, vio_t, side="left")
        bias_idx = np.clip(bias_idx, 0, len(tb) - 1)
        choose_prev = (
            (bias_idx > 0)
            & (np.abs(tb[bias_idx - 1] - vio_t) < np.abs(tb[bias_idx] - vio_t))
        )
        bias_idx[choose_prev] -= 1
        if np.max(np.abs(tb[bias_idx] - vio_t)) > 0.01:
            raise ValueError("Bias/state velocity timestamps do not match trajectory timestamps")
        vio_vel = bias[["vx", "vy", "vz"]].to_numpy()[bias_idx]
    else:
        raw_vel = smooth_velocity(tp, traj[["x", "y", "z"]].to_numpy(), 31)
        vio_vel = raw_vel[post_idx]
    vio_pos, vio_vel, yaw_align = start_align(
        tg, gps_pos, gps_vel, vio_pos, vio_vel, heading_window,
        heading_stable_window, min_speed)
    out = pd.DataFrame({
        "t": tg,
        "gps_update_t": tg,
        "vio_state_t": vio_t,
        "vio_sample_delay_ms": 1000.0 * sample_delay,
        "gps_gap_before": gap_before,
        "gps_run_id": gps_run_id,
    })
    out[["gps_E", "gps_N", "gps_U"]] = gps_pos
    out[["gps_vE", "gps_vN", "gps_vU"]] = gps_vel
    out[["vio_E", "vio_N", "vio_U"]] = vio_pos
    out[["vio_vE", "vio_vN", "vio_vU"]] = vio_vel
    out["gps_speed_xy"] = np.linalg.norm(gps_vel[:, :2], axis=1)
    out["gps_speed_3d"] = np.linalg.norm(gps_vel, axis=1)
    out["vio_speed_xy"] = np.linalg.norm(vio_vel[:, :2], axis=1)
    out["vio_speed_3d"] = np.linalg.norm(vio_vel, axis=1)
    out["gps_course_deg"] = np.rad2deg(np.arctan2(gps_vel[:, 1], gps_vel[:, 0]))
    out["vio_course_deg"] = np.rad2deg(np.arctan2(vio_vel[:, 1], vio_vel[:, 0]))
    pos_err = vio_pos - gps_pos
    vel_err = vio_vel - gps_vel
    out[["err_E", "err_N", "err_U"]] = pos_err
    out["err_XY"] = np.linalg.norm(pos_err[:, :2], axis=1)
    out["err_3D"] = np.linalg.norm(pos_err, axis=1)
    out[["err_vE", "err_vN", "err_vU"]] = vel_err
    out["err_speed_xy"] = out["vio_speed_xy"] - out["gps_speed_xy"]
    out["err_vXY_vec"] = np.linalg.norm(vel_err[:, :2], axis=1)
    course = np.deg2rad(out["gps_course_deg"].to_numpy())
    along = np.column_stack([np.cos(course), np.sin(course)])
    cross = np.column_stack([-np.sin(course), np.cos(course)])
    out["err_along"] = np.sum(pos_err[:, :2] * along, axis=1)
    out["err_cross"] = np.sum(pos_err[:, :2] * cross, axis=1)
    out["err_vertical"] = pos_err[:, 2]
    out["err_v_along"] = np.sum(vel_err[:, :2] * along, axis=1)
    out["err_v_cross"] = np.sum(vel_err[:, :2] * cross, axis=1)
    out["err_v_vertical"] = vel_err[:, 2]
    out["course_error_deg"] = wrap_deg(out["vio_course_deg"] - out["gps_course_deg"])
    dg = np.linalg.norm(np.diff(gps_pos[:, :2], axis=0), axis=1)
    dv = np.linalg.norm(np.diff(vio_pos[:, :2], axis=0), axis=1)
    out["cum_dist_gps"] = np.r_[0.0, np.cumsum(dg)]
    out["cum_dist_vio"] = np.r_[0.0, np.cumsum(dv)]
    out["gps_velocity_valid"] = np.isfinite(out["gps_vE"]) & np.isfinite(out["gps_vN"])
    out["gps_velocity_source"] = gps_velocity_source
    return out, yaw_align


def global_summary(df: pd.DataFrame, condition: str, flight: str, notes: str, yaw_align: float) -> dict[str, Any]:
    distance = float(df["cum_dist_gps"].iloc[-1])
    metrics = {name: metric_stats(df[name].to_numpy()) for name in [
        "err_E", "err_N", "err_U", "err_XY", "err_3D", "err_vE", "err_vN",
        "err_vU", "err_speed_xy", "err_vXY_vec", "err_along", "err_cross",
        "err_vertical", "err_v_along", "err_v_cross", "err_v_vertical",
        "course_error_deg",
    ]}
    return {
        "condition": condition, "flight": flight,
        "duration_s": float(df["t"].iloc[-1] - df["t"].iloc[0]),
        "gps_distance_km": distance / 1000.0,
        "total_gps_distance_m": distance, "total_gps_distance_km": distance / 1000.0,
        "final_xy_error_m": float(df["err_XY"].iloc[-1]),
        "final_xy_drift_percent": 100.0 * float(df["err_XY"].iloc[-1]) / distance,
        "xy_rmse_m": metrics["err_XY"]["rmse"],
        "xy_rmse_percent": 100.0 * float(metrics["err_XY"]["rmse"]) / distance,
        "xy_rmse_percent_of_distance": 100.0 * float(metrics["err_XY"]["rmse"]) / distance,
        "final_along_error_m": metrics["err_along"]["final"],
        "final_cross_error_m": metrics["err_cross"]["final"],
        "final_vertical_error_m": float(df["err_vertical"].iloc[-1]),
        "vertical_rmse_m": metrics["err_vertical"]["rmse"],
        "speed_rmse_mps": metrics["err_speed_xy"]["rmse"],
        "vxy_vec_rmse_mps": metrics["err_vXY_vec"]["rmse"],
        "yaw/course_final_deg": metrics["course_error_deg"]["final"],
        "yaw/course_rmse_deg": metrics["course_error_deg"]["rmse"],
        "start_heading_rotation_deg": yaw_align,
        "analysis_start_time_s": float(df["t"].iloc[0]),
        "analysis_end_time_s": float(df["t"].iloc[-1]),
        "gps_update_rows": int(len(df)),
        "gps_gap_rows": int(df["gps_gap_before"].sum()),
        "gps_velocity_valid_rows": int(df["gps_velocity_valid"].sum()),
        "gps_velocity_invalid_rows": int((~df["gps_velocity_valid"]).sum()),
        "gps_velocity_source": str(df["gps_velocity_source"].iloc[0]),
        "vio_sample_delay_median_ms": float(df["vio_sample_delay_ms"].median()),
        "vio_sample_delay_p95_ms": float(df["vio_sample_delay_ms"].quantile(0.95)),
        "vio_sample_delay_max_ms": float(df["vio_sample_delay_ms"].max()),
        "notes": notes,
        "metrics": metrics,
    }


def sampling_quality_table(
    df: pd.DataFrame, requested_t0: float | None, requested_t1: float | None,
    crop_reason: str,
) -> pd.DataFrame:
    return pd.DataFrame([{
        "requested_start_time_s": requested_t0,
        "requested_end_time_s": requested_t1,
        "actual_first_gps_update_s": float(df["gps_update_t"].iloc[0]),
        "actual_last_gps_update_s": float(df["gps_update_t"].iloc[-1]),
        "gps_update_rows": int(len(df)),
        "gps_gap_rows": int(df["gps_gap_before"].sum()),
        "gps_velocity_valid_rows": int(df["gps_velocity_valid"].sum()),
        "gps_velocity_invalid_rows": int((~df["gps_velocity_valid"]).sum()),
        "gps_velocity_source": str(df["gps_velocity_source"].iloc[0]),
        "vio_sample_delay_median_ms": float(df["vio_sample_delay_ms"].median()),
        "vio_sample_delay_p95_ms": float(df["vio_sample_delay_ms"].quantile(0.95)),
        "vio_sample_delay_max_ms": float(df["vio_sample_delay_ms"].max()),
        "sampling_rule": "first_post_gps_update_vio_state",
        "crop_reason": crop_reason,
    }])


def write_figures(df: pd.DataFrame, seg: pd.DataFrame, out: Path, title: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    dist = df["cum_dist_gps"].to_numpy() / 1000.0
    t = df["t"].to_numpy()
    gaps = df["gps_gap_before"].to_numpy(dtype=bool)

    def save(name: str) -> None:
        plt.tight_layout()
        plt.savefig(out / name, dpi=150)
        plt.savefig(out / f"{Path(name).stem}.svg")
        plt.close()

    def broken(values: pd.Series | np.ndarray) -> np.ndarray:
        result = np.asarray(values, dtype=float).copy()
        result[gaps] = np.nan
        return result

    def decorate(ax: Any, ylabel: str, title_cn: str | None = None) -> None:
        ax.set_ylabel(ylabel)
        if title_cn:
            ax.set_title(title_cn)
        ax.grid(alpha=.25)
        ax.legend(loc="best")

    vertical_divergence = np.flatnonzero(np.abs(df["err_U"].to_numpy()) > 50.0)
    divergence_t = (
        float(df["t"].iloc[vertical_divergence[0]])
        if len(vertical_divergence) else None
    )

    plt.figure(figsize=(8, 7))
    plt.plot(broken(df["gps_E"]), broken(df["gps_N"]), label="GPS参考轨迹", lw=1.8)
    plt.plot(broken(df["vio_E"]), broken(df["vio_N"]), label="VIO估计轨迹", lw=1.2)
    plt.xlabel("东向位置 E（米）")
    plt.ylabel("北向位置 N（米）")
    plt.axis("equal")
    plt.grid(alpha=.25)
    plt.legend()
    plt.title(f"{title}：水平轨迹对比（GPS缺口处断线）")
    save("trajectory_xy_gps_vio_lk.png")

    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for a, gps_c, vio_c, axis in zip(
        ax,
        ["gps_E", "gps_N", "gps_U"],
        ["vio_E", "vio_N", "vio_U"],
        ["E", "N", "U"],
    ):
        a.plot(t, broken(df[gps_c]), label=f"GPS {axis}方向", lw=1.7)
        a.plot(t, broken(df[vio_c]), label=f"VIO {axis}方向", lw=1.1)
        decorate(a, f"{axis}方向位置（米）")
    ax[0].set_title("东、北、天三个方向的位置对比")
    if divergence_t is not None:
        ax[2].axvline(divergence_t, color="tab:red", ls="--", lw=1.2,
                      label="垂直误差首次超过50米")
        ax[2].legend(loc="best")
    ax[-1].set_xlabel("GPS更新时间（秒）")
    save("position_enu_gps_vio.png")

    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for a, gps_c, vio_c, axis in zip(
        ax,
        ["gps_vE", "gps_vN", "gps_vU"],
        ["vio_vE", "vio_vN", "vio_vU"],
        ["E", "N", "U"],
    ):
        a.plot(t, broken(df[gps_c]), label=f"飞控GPS {axis}方向原始速度", lw=1.7)
        a.plot(t, broken(df[vio_c]), label=f"VIO {axis}方向速度", lw=1.1)
        decorate(a, f"{axis}方向速度（米/秒）")
    ax[0].set_title("飞控原始Ve/Vn/Vu与VIO状态速度对比（空白表示GPS无更新）")
    ax[-1].set_xlabel("GPS更新时间（秒）")
    save("velocity_enu_gps_vio.png")

    plt.figure(figsize=(10, 4.5))
    plt.plot(dist, broken(df["err_XY"]), label="水平位置误差", color="tab:red")
    plt.xlabel("GPS累计水平航程（千米）")
    plt.ylabel("XY水平位置误差（米）")
    plt.title("水平位置误差随航程变化")
    plt.grid(alpha=.25)
    plt.legend()
    save("trajectory_xy_error_over_distance.png")

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for a, c, lab in zip(ax, ["err_E", "err_N", "err_U"], ["E", "N", "U"]):
        a.plot(t, broken(df[c]), label=f"{lab}方向位置误差")
        decorate(a, f"{lab}方向误差（米）")
    ax[0].set_title("东、北、天三个方向的位置误差")
    if divergence_t is not None:
        ax[2].axvline(divergence_t, color="tab:red", ls="--", lw=1.2,
                      label="垂直误差首次超过50米")
        ax[2].legend(loc="best")
    ax[-1].set_xlabel("GPS更新时间（秒）")
    save("enu_error_time.png")

    plt.figure(figsize=(10, 5))
    labels = {
        "err_along": "沿航向位置误差",
        "err_cross": "横航向位置误差",
        "err_vertical": "垂直位置误差",
    }
    for c, label in labels.items():
        plt.plot(dist, broken(df[c]), label=label)
    plt.xlabel("GPS累计水平航程（千米）")
    plt.ylabel("位置误差（米）")
    plt.title("沿航向、横航向和垂直方向的位置误差")
    plt.grid(alpha=.25)
    plt.legend()
    save("along_cross_vertical_error_distance.png")

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for a, c, lab in zip(ax, ["err_vE", "err_vN", "err_vU"], ["E", "N", "U"]):
        a.plot(t, broken(df[c]), label=f"{lab}方向速度误差")
        decorate(a, f"{lab}方向误差（米/秒）")
    ax[0].set_title("VIO状态速度减飞控原始GPS速度（空白表示GPS无更新）")
    ax[-1].set_xlabel("GPS更新时间（秒）")
    save("velocity_error_time.png")

    plt.figure(figsize=(10, 4.5))
    plt.plot(dist, broken(df["err_speed_xy"]), label="水平速度大小误差")
    plt.plot(dist, broken(df["err_vXY_vec"]), label="水平速度向量误差")
    plt.xlabel("GPS累计水平航程（千米）")
    plt.ylabel("水平速度误差（米/秒）")
    plt.title("XY平面速度误差随航程变化")
    plt.grid(alpha=.25)
    plt.legend()
    save("speed_error_distance.png")

    plt.figure(figsize=(10, 4.5))
    plt.plot(t, broken(df["course_error_deg"]), label="VIO航向 - GPS航向")
    plt.xlabel("GPS更新时间（秒）")
    plt.ylabel("航向误差（度）")
    plt.title("航向误差（仅使用有效GPS速度点）")
    plt.grid(alpha=.25)
    plt.legend()
    save("course_yaw_error_time.png")

    valid_seg = seg.dropna(subset=["xy_drift_percent"])
    plt.figure(figsize=(11, 4.5))
    plt.scatter(valid_seg["segment_id"], valid_seg["xy_drift_percent"], s=16, alpha=.75,
                label="有效航段")
    plt.xlabel("航段编号")
    plt.ylabel("航段内水平漂移率（%）")
    plt.title("各有效航段的局部水平漂移率")
    plt.grid(alpha=.25)
    plt.legend()
    save("segment_xy_drift_bar.png")

    plt.figure(figsize=(11, 4.5))
    plt.scatter(seg["segment_id"], seg["vxy_vec_rmse_mps"], s=16, alpha=.75,
                label="航段速度误差")
    plt.xlabel("航段编号")
    plt.ylabel("水平速度向量RMSE（米/秒）")
    plt.title("各航段的水平速度误差")
    plt.grid(alpha=.25)
    plt.legend()
    save("segment_velocity_error_bar.png")

    plt.figure(figsize=(10, 5))
    for leg, s in df[df["leg_id"] >= 0].groupby("leg_id"):
        d0 = s["cum_dist_gps"].iloc[0]
        plt.plot((s["cum_dist_gps"] - d0) / 1000.0, s["err_XY"], label=f"直线航段 {leg}")
    plt.xlabel("进入当前直线航段后的航程（千米）")
    plt.ylabel("水平位置误差（米）")
    plt.title("各长直线航段的水平误差剖面")
    plt.grid(alpha=.25)
    if (df["leg_id"] >= 0).any():
        plt.legend(ncol=2, fontsize=8)
    save("straight_leg_error_profiles.png")

    bins = pd.cut(df["gps_U"], bins=min(6, max(2, df["gps_U"].nunique())), duplicates="drop")
    h = df.groupby(bins, observed=True)["err_XY"].apply(lambda x: metric_stats(x.to_numpy())["rmse"])
    plt.figure(figsize=(9, 4.5))
    h.plot(kind="bar", label="各高度层水平RMSE")
    plt.ylabel("水平位置RMSE（米）")
    plt.xlabel("GPS高度分层（米）")
    plt.title("不同飞行高度层的水平位置误差")
    plt.grid(axis="y", alpha=.25)
    plt.legend()
    save("height_layer_error_summary.png")

    write_interactive_dashboard(df, out / "interactive_flight_analysis.html", title)


def write_interactive_dashboard(df: pd.DataFrame, path: Path, title: str) -> None:
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        return
    fig = make_subplots(
        rows=4, cols=2,
        subplot_titles=[
            "XY平面轨迹", "XY水平位置误差",
            "ENU位置：GPS与VIO", "ENU位置误差",
            "ENU速度：GPS与VIO", "ENU速度误差",
            "沿航向、横航向、垂直误差", "XY平面速度误差",
        ],
    )
    plot_df = df.copy()
    plot_df.loc[plot_df["gps_gap_before"], [
        "gps_E", "gps_N", "gps_U", "vio_E", "vio_N", "vio_U",
        "err_E", "err_N", "err_U", "err_XY", "err_along", "err_cross",
        "err_vertical", "gps_vE", "gps_vN", "gps_vU", "vio_vE", "vio_vN",
        "vio_vU", "err_vE", "err_vN", "err_vU", "err_speed_xy",
        "err_vXY_vec",
    ]] = np.nan
    fig.add_trace(go.Scattergl(x=plot_df["gps_E"], y=plot_df["gps_N"], name="GPS参考轨迹"), 1, 1)
    fig.add_trace(go.Scattergl(x=plot_df["vio_E"], y=plot_df["vio_N"], name="VIO估计轨迹"), 1, 1)
    fig.add_trace(go.Scattergl(
        x=plot_df["cum_dist_gps"] / 1000.0, y=plot_df["err_XY"],
        name="水平位置误差"), 1, 2)
    colors = {"E": "#1f77b4", "N": "#ff7f0e", "U": "#2ca02c"}
    axis_cn = {"E": "东向", "N": "北向", "U": "天向"}
    for axis in ["E", "N", "U"]:
        fig.add_trace(go.Scattergl(
            x=plot_df["t"], y=plot_df[f"gps_{axis}"], name=f"GPS{axis_cn[axis]}位置",
            line={"color": colors[axis], "dash": "dot"}), 2, 1)
        fig.add_trace(go.Scattergl(
            x=plot_df["t"], y=plot_df[f"vio_{axis}"], name=f"VIO{axis_cn[axis]}位置",
            line={"color": colors[axis]}), 2, 1)
        fig.add_trace(go.Scattergl(
            x=plot_df["t"], y=plot_df[f"err_{axis}"], name=f"{axis_cn[axis]}位置误差"), 2, 2)
        fig.add_trace(go.Scattergl(
            x=plot_df["t"], y=plot_df[f"gps_v{axis}"],
            name=f"飞控GPS{axis_cn[axis]}原始速度",
            line={"color": colors[axis], "dash": "dot"}), 3, 1)
        fig.add_trace(go.Scattergl(
            x=plot_df["t"], y=plot_df[f"vio_v{axis}"], name=f"VIO{axis_cn[axis]}速度",
            line={"color": colors[axis]}), 3, 1)
        fig.add_trace(go.Scattergl(
            x=plot_df["t"], y=plot_df[f"err_v{axis}"], name=f"{axis_cn[axis]}速度误差"), 3, 2)
    for c, label in {
        "err_along": "沿航向位置误差",
        "err_cross": "横航向位置误差",
        "err_vertical": "垂直位置误差",
    }.items():
        fig.add_trace(go.Scattergl(
            x=plot_df["cum_dist_gps"] / 1000.0, y=plot_df[c], name=label), 4, 1)
    fig.add_trace(go.Scattergl(
        x=plot_df["cum_dist_gps"] / 1000.0, y=plot_df["err_speed_xy"],
        name="水平速度大小误差"), 4, 2)
    fig.add_trace(go.Scattergl(
        x=plot_df["cum_dist_gps"] / 1000.0, y=plot_df["err_vXY_vec"],
        name="水平速度向量误差"), 4, 2)
    fig.update_layout(
        title=f"{title}：GPS更新时刻交互分析", height=1350, hovermode="x unified",
        legend={"groupclick": "toggleitem"},
    )
    fig.update_xaxes(title_text="GPS累计航程（千米）", row=1, col=2)
    fig.update_xaxes(title_text="GPS累计航程（千米）", row=4, col=1)
    fig.update_xaxes(title_text="GPS累计航程（千米）", row=4, col=2)
    fig.write_html(path, include_plotlyjs=True)


def write_report(
    args: argparse.Namespace, summary: dict[str, Any], index: pd.DataFrame,
    seg: pd.DataFrame, out: Path, warnings: list[str],
) -> None:
    m = summary["metrics"]
    best_straight = seg[seg["segment_type"] == "straight"].sort_values("xy_rmse_m").head(8)
    turns = seg[seg["segment_type"].isin(["turn", "transition"])].head(12)
    text = f"""# Full Flight Error Analysis

## 1. Input files and time window

- Flight: `{args.flight_name}`
- Method: `{args.method_name}`
- GPS: `{args.gps}`
- VIO trajectory: `{args.vio_traj}`
- VIO bias/velocity: `{args.vio_bias or "not supplied"}`
- Experiment ID: `{args.experiment_id or "not supplied"}`
- Source package: `{args.source_package or "not supplied"}`
- Source member: `{args.source_member or "not supplied"}`
- Experiment config: `{args.experiment_config or "not supplied"}`
- Evaluated GPS-time window: {summary['duration_s']:.2f} s

## 2. Alignment method

Start alignment is canonical: one horizontal rotation from the first stable
course window, followed by translation at the first common GPS sample. The
applied heading rotation was {summary['start_heading_rotation_deg']:+.3f} deg.
No full-trajectory best fit is used in the reported drift statistics.

## 3. GPS sampling rule

Each sorted, de-duplicated GPS update contributes one master row. Because the
experiment applies GPS at a camera frame and writes the state after that
update, each GPS row uses the first VIO state at or after the GPS timestamp,
with a maximum accepted delay of {args.max_vio_sample_delay_s * 1000.0:.0f} ms.
No VIO interpolation is performed across a GPS update discontinuity. GPS
velocity source: `{summary['gps_velocity_source']}`. Flight-controller
`Ve/Vn/Vu` is used directly when present. Position differentiation is only a
compatibility fallback for historical four-column GPS files.

The requested analysis window is `{args.t0}` to `{args.t1}` seconds. The last
included GPS update is {summary['analysis_end_time_s']:.6f} seconds. Crop
reason: {args.crop_reason or "explicit experiment analysis window"}.

Sampling diagnostics are in `../tables/gps_sampling_quality.csv`.

## 4. Total distance and duration

- Duration: {summary['duration_s']:.2f} s
- GPS horizontal distance: {summary['total_gps_distance_km']:.3f} km
- GPS update rows: {summary['gps_update_rows']}
- GPS gap boundaries: {summary['gps_gap_rows']}
- Valid velocity rows: {summary['gps_velocity_valid_rows']}
- GPS velocity source: {summary['gps_velocity_source']}
- VIO post-update delay median / P95 / max:
  {summary['vio_sample_delay_median_ms']:.3f} /
  {summary['vio_sample_delay_p95_ms']:.3f} /
  {summary['vio_sample_delay_max_ms']:.3f} ms

## 5. Global statistics

| Metric | Value |
|---|---:|
| Final XY error | {summary['final_xy_error_m']:.3f} m |
| Final XY drift | {summary['final_xy_drift_percent']:.3f}% |
| XY RMSE | {summary['xy_rmse_m']:.3f} m |
| XY RMSE / distance | {summary['xy_rmse_percent']:.3f}% |
| Course error RMSE | {summary['yaw/course_rmse_deg']:.3f} deg |
| Course error final | {summary['yaw/course_final_deg']:.3f} deg |

## 6. Axis-wise position error

| Axis | Mean | MAE | RMSE | P95 | Max | Final |
|---|---:|---:|---:|---:|---:|---:|
| E | {m['err_E']['signed_mean']:.3f} | {m['err_E']['mae']:.3f} | {m['err_E']['rmse']:.3f} | {m['err_E']['p95']:.3f} | {m['err_E']['max']:.3f} | {m['err_E']['final']:.3f} |
| N | {m['err_N']['signed_mean']:.3f} | {m['err_N']['mae']:.3f} | {m['err_N']['rmse']:.3f} | {m['err_N']['p95']:.3f} | {m['err_N']['max']:.3f} | {m['err_N']['final']:.3f} |
| U | {m['err_U']['signed_mean']:.3f} | {m['err_U']['mae']:.3f} | {m['err_U']['rmse']:.3f} | {m['err_U']['p95']:.3f} | {m['err_U']['max']:.3f} | {m['err_U']['final']:.3f} |

## 7. Axis-wise velocity error

| Axis | Mean | MAE | RMSE | P95 | Max | Final |
|---|---:|---:|---:|---:|---:|---:|
| vE | {m['err_vE']['signed_mean']:.3f} | {m['err_vE']['mae']:.3f} | {m['err_vE']['rmse']:.3f} | {m['err_vE']['p95']:.3f} | {m['err_vE']['max']:.3f} | {m['err_vE']['final']:.3f} |
| vN | {m['err_vN']['signed_mean']:.3f} | {m['err_vN']['mae']:.3f} | {m['err_vN']['rmse']:.3f} | {m['err_vN']['p95']:.3f} | {m['err_vN']['max']:.3f} | {m['err_vN']['final']:.3f} |
| vU | {m['err_vU']['signed_mean']:.3f} | {m['err_vU']['mae']:.3f} | {m['err_vU']['rmse']:.3f} | {m['err_vU']['p95']:.3f} | {m['err_vU']['max']:.3f} | {m['err_vU']['final']:.3f} |

## 8. Along-track / cross-track / vertical error

| Component | RMSE | Final |
|---|---:|---:|
| Along | {m['err_along']['rmse']:.3f} m | {summary['final_along_error_m']:.3f} m |
| Cross | {m['err_cross']['rmse']:.3f} m | {summary['final_cross_error_m']:.3f} m |
| Vertical | {m['err_vertical']['rmse']:.3f} m | {summary['final_vertical_error_m']:.3f} m |

## 9. Percent drift relative to distance

Percent values divide the run-level final or RMSE error by total GPS horizontal
distance. Segment percentages use each segment's own GPS distance.

## 10. Segment decomposition method

Straight candidates require speed, course-rate, heading-variation, and local
line-fit tests. Geometric straight legs longer than the minimum are split by
cumulative distance; their final short remainder remains straight. Every other
sample is retained as turn, transition, short, or unknown.

## 11. Segment error summary

{seg.head(20).to_markdown(index=False)}

## 12. Straight-leg details

{best_straight.to_markdown(index=False) if len(best_straight) else "No >=2 km straight leg was detected."}

## 13. Turn / transition details

{turns.to_markdown(index=False) if len(turns) else "No turn/transition segment was detected."}

## 14. Figure list

`../plots/` contains mandatory position/velocity comparisons, component errors,
XY errors, trajectory, along/cross/vertical errors, segment plots, and
`interactive_flight_analysis.html`. Every static figure is written as PNG and
SVG.

## 15. Warnings and missing inputs

{chr(10).join("- " + w for w in warnings) if warnings else "- None."}
"""
    (out / "FULL_FLIGHT_ERROR_ANALYSIS.md").write_text(text, encoding="utf-8")


def analyze_run(args: argparse.Namespace) -> dict[str, Any]:
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    plots = out / "plots"
    tables = out / "tables"
    data = out / "data"
    reports = out / "reports"
    metadata = out / "metadata"
    for directory in [plots, tables, data, reports, metadata]:
        directory.mkdir(parents=True, exist_ok=True)
    gps = load_gps(Path(args.gps))
    traj = load_traj(args.vio_traj)
    bias = load_bias_velocity(args.vio_bias) if args.vio_bias else None
    aligned, yaw_align = build_aligned(
        gps, traj, bias, args.t0, args.t1, args.heading_window_s,
        args.heading_stable_window_s, args.min_speed_mps,
        args.max_vio_sample_delay_s, args.gps_gap_threshold_s)
    aligned, index = segment_flight(
        aligned, args.min_speed_mps, args.max_course_rate_degps,
        args.max_heading_std_deg, args.max_line_fit_rmse_m,
        args.min_straight_len_m, args.straight_split_len_m)
    seg = segment_summaries(
        aligned, args.method_name, index, args.min_segment_drift_distance_m)
    summary = global_summary(aligned, args.method_name, args.flight_name, args.notes, yaw_align)
    warnings = []
    if bias is None:
        warnings.append("VIO bias/velocity file missing; VIO velocity was derived from trajectory position.")
    if args.lk_traj or args.lk_flow_csv:
        warnings.append("LK inputs were declared but LK reconstruction is not implemented without a valid LK trajectory schema.")
    else:
        warnings.append("No LK-only trajectory or pair diagnostics were supplied; no LK metrics were invented.")
    if args.t1 is not None:
        warnings.append(
            f"Plots and all statistics were cropped at t1={args.t1:.3f} s: "
            f"{args.crop_reason or 'explicit experiment validity boundary'}.")
    invalid_velocity = int((~aligned["gps_velocity_valid"]).sum())
    if invalid_velocity:
        warnings.append(
            f"{invalid_velocity} GPS rows have no valid reference velocity and "
            "are excluded from velocity/course statistics.")
    vertical_divergence = np.flatnonzero(np.abs(aligned["err_U"].to_numpy()) > 50.0)
    if len(vertical_divergence):
        warnings.append(
            "Vertical position error first exceeds 50 m at "
            f"t={aligned['t'].iloc[vertical_divergence[0]]:.3f} s.")
    aligned.to_csv(data / "gps_time_aligned_samples.csv", index=False)
    metric_rows = []
    for metric, stats in summary["metrics"].items():
        metric_rows.append({"metric": metric, **stats})
    pd.DataFrame(metric_rows).to_csv(tables / "metric_statistics.csv", index=False)
    pd.DataFrame([{k: v for k, v in summary.items() if k != "metrics"}]).to_csv(
        tables / "global_summary.csv", index=False)
    sampling_quality_table(
        aligned, args.t0, args.t1, args.crop_reason).to_csv(
            tables / "gps_sampling_quality.csv", index=False)
    (metadata / "global_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8")
    index.to_csv(tables / "segment_index.csv", index=False)
    seg.to_csv(tables / "segment_error_summary.csv", index=False)
    seg[seg["segment_type"] == "straight"].to_csv(
        tables / "straight_leg_summary.csv", index=False)
    seg[seg["segment_type"].isin(["turn", "transition"])].to_csv(
        tables / "turn_transition_summary.csv", index=False)
    if aligned["gps_U"].notna().any():
        layers = pd.cut(aligned["gps_U"], bins=6, duplicates="drop")
        height = aligned.groupby(layers, observed=True).agg(
            n=("t", "size"), gps_u_mean_m=("gps_U", "mean"),
            xy_rmse_m=("err_XY", lambda x: metric_stats(x.to_numpy())["rmse"]),
            vertical_rmse_m=("err_vertical", lambda x: metric_stats(x.to_numpy())["rmse"]),
        ).reset_index()
        height["height_layer"] = height["gps_U"].astype(str)
        height.drop(columns=["gps_U"]).to_csv(
            tables / "height_layer_summary.csv", index=False)
    provenance = {
        "experiment_id": args.experiment_id,
        "source_package": args.source_package,
        "source_member": args.source_member,
        "gps": args.gps,
        "vio_traj": args.vio_traj,
        "vio_bias": args.vio_bias,
        "flight_name": args.flight_name,
        "method_name": args.method_name,
        "t0": args.t0,
        "t1": args.t1,
        "crop_reason": args.crop_reason,
        "experiment_config": args.experiment_config,
        "run_status": args.run_status,
        "notes": args.notes,
    }
    (metadata / "analysis_provenance.json").write_text(
        json.dumps(provenance, indent=2), encoding="utf-8")
    (metadata / "RUN_STATUS.txt").write_text(
        f"{args.run_status or 'not supplied'}\n", encoding="utf-8-sig")
    directory_index = f"""# Experiment Run Results

- Experiment ID: `{args.experiment_id or "not supplied"}`
- Flight: `{args.flight_name}`
- Method: `{args.method_name}`
- Status: `{args.run_status or "not supplied"}`
- Configuration: `{args.experiment_config or "not supplied"}`

## Directories

- `plots/`: PNG, SVG, and interactive HTML plots.
- `tables/`: summary and segment CSV tables.
- `data/`: large GPS-time aligned sample data.
- `reports/`: complete Markdown analysis report.
- `metadata/`: provenance, full JSON summary, and run status.
"""
    (out / "README.md").write_text(directory_index, encoding="utf-8")
    if args.expected_xy_rmse_m is not None:
        expected = float(args.expected_xy_rmse_m)
        actual = float(summary["xy_rmse_m"])
        pd.DataFrame([{
            "metric": "xy_rmse_m",
            "expected_reference": expected,
            "actual": actual,
            "difference": actual - expected,
            "relative_difference_percent": 100.0 * (actual - expected) / expected,
            "note": "Reference may use a different documented start-yaw window.",
        }]).to_csv(tables / "reference_metric_check.csv", index=False)
    write_figures(
        aligned, seg, plots, f"飞行 {args.flight_name}，方法 {args.method_name}")
    write_report(args, summary, index, seg, reports, warnings)
    return {
        "summary": summary,
        "aligned": aligned,
        "segments": seg,
        "index": index,
        "out": out,
        "plots": plots,
        "tables": tables,
        "data": data,
        "reports": reports,
        "metadata": metadata,
    }


def aggregate_condition(rows: pd.DataFrame) -> pd.Series:
    ok = rows[~rows["notes"].str.contains("FAILED", case=False, na=False)]
    source = ok if len(ok) else rows
    total_dist = source["gps_distance_km"].sum()
    weights = source["gps_distance_km"].clip(lower=1e-9)
    result = {
        "condition": rows["condition"].iloc[0],
        "duration_s": source["duration_s"].sum(),
        "gps_distance_km": total_dist,
    }
    for c in ["final_xy_error_m", "final_xy_drift_percent", "xy_rmse_m",
              "xy_rmse_percent", "final_along_error_m", "final_cross_error_m",
              "vertical_rmse_m", "speed_rmse_mps", "vxy_vec_rmse_mps",
              "yaw/course_final_deg", "yaw/course_rmse_deg"]:
        result[c] = float(np.average(source[c], weights=weights))
    result["notes"] = (
        "Distance-weighted across non-failed flights; "
        + "; ".join(rows.loc[rows["notes"].astype(str) != "", "notes"].astype(str))
    )
    return pd.Series(result)


def comparison_figures(results: list[dict[str, Any]], out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    flights = sorted({r["summary"]["flight"] for r in results})
    valid = [r for r in results if "FAILED" not in str(r["summary"].get("notes", "")).upper()]
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    for ax, flight in zip(axes.flat, flights):
        rr = [r for r in valid if r["summary"]["flight"] == flight]
        if rr:
            ax.plot(rr[0]["aligned"]["gps_E"], rr[0]["aligned"]["gps_N"], "k", label="GPS")
        for r in rr:
            d = r["aligned"]
            ax.plot(d["vio_E"], d["vio_N"], label=r["summary"]["condition"])
        ax.set_title(flight); ax.axis("equal"); ax.grid(alpha=.3); ax.legend(fontsize=7)
    plt.tight_layout(); plt.savefig(out / "condition_trajectory_overlay.png", dpi=150); plt.close()
    for name, y, ylabel in [
        ("condition_xy_error_distance.png", "err_XY", "XY error [m]"),
        ("condition_yaw_error_time.png", "course_error_deg", "course error [deg]"),
    ]:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        for ax, flight in zip(axes.flat, flights):
            for r in [x for x in valid if x["summary"]["flight"] == flight]:
                d = r["aligned"]
                x = d["cum_dist_gps"] / 1000 if y == "err_XY" else d["t"]
                ax.plot(x, d[y], label=r["summary"]["condition"])
            ax.set_title(flight); ax.set_ylabel(ylabel); ax.grid(alpha=.3); ax.legend(fontsize=7)
        plt.tight_layout(); plt.savefig(out / name, dpi=150); plt.close()
    table = pd.concat([
        r["segments"].assign(condition=r["summary"]["condition"], flight=r["summary"]["flight"])
        for r in valid
    ], ignore_index=True)
    pivot = table.groupby(["condition", "segment_type"])["xy_drift_percent"].median().unstack()
    pivot.plot(kind="bar", figsize=(10, 5))
    plt.ylabel("median segment XY drift [%]"); plt.grid(axis="y", alpha=.3)
    plt.tight_layout(); plt.savefig(out / "condition_segment_drift_comparison.png", dpi=150); plt.close()


def primary_figure_montages(results: list[dict[str, Any]], out: Path) -> None:
    names = [
        "trajectory_xy_gps_vio_lk.png",
        "trajectory_xy_error_over_distance.png",
        "enu_error_time.png",
        "along_cross_vertical_error_distance.png",
        "velocity_error_time.png",
        "speed_error_distance.png",
        "course_yaw_error_time.png",
        "segment_xy_drift_bar.png",
        "segment_velocity_error_bar.png",
        "straight_leg_error_profiles.png",
        "height_layer_error_summary.png",
    ]
    for name in names:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        for ax, result in zip(axes.flat, sorted(results, key=lambda x: x["summary"]["flight"])):
            image = result["plots"] / name
            ax.imshow(plt.imread(image))
            ax.set_title(result["summary"]["flight"])
            ax.axis("off")
        plt.tight_layout()
        plt.savefig(out / name, dpi=150)
        plt.close()


def write_primary_package_report(results: list[dict[str, Any]], out: Path) -> None:
    summaries = pd.DataFrame([
        {k: v for k, v in r["summary"].items() if k != "metrics"} for r in results
    ])
    segments = pd.concat([
        r["segments"].assign(flight=r["summary"]["flight"]) for r in results
    ], ignore_index=True)
    by_type = segments.groupby("segment_type").agg(
        segments=("segment_id", "size"),
        median_xy_rmse_m=("xy_rmse_m", "median"),
        median_xy_drift_percent=("xy_drift_percent", "median"),
        median_vxy_vec_rmse_mps=("vxy_vec_rmse_mps", "median"),
    ).reset_index()
    straight = segments[segments["segment_type"] == "straight"].sort_values("xy_rmse_m").head(12)
    turns = segments[segments["segment_type"].isin(["turn", "transition"])].sort_values(
        "xy_rmse_m", ascending=False).head(12)
    text = f"""# Full Flight Error Analysis - OC Package

## 1. Input files and time window

This report covers `cond3_oc_gpsz` from the 2026-06-05 three-condition package
for fly1 through fly4. Exact GPS and trajectory paths are in
`analysis/oc_package_manifest_20260605.json`. Canonical windows are fly1
930-1744 s, fly2 700-2500 s, fly3 618-1600 s, and fly4 924.4-2816 s.

## 2. Alignment method

Each flight uses the earliest stable 5 s GPS course window in the initial 60 s,
the VIO velocity bearing at that window start, and first-sample translation.
Scale is preserved. No full-flight best fit contributes to these metrics.

## 3. GPS sampling rule

GPS timestamps are the master grid. Each GPS update is paired with the first
post-update VIO state at or after that timestamp. GPS velocity is computed
only inside contiguous GPS runs, with run endpoints excluded; final statistics
do not use a 30 Hz GPS upsample.

## 4. Total distance and duration

{summaries[['flight','duration_s','gps_distance_km']].to_markdown(index=False)}

## 5. Global statistics

{summaries[['flight','final_xy_error_m','final_xy_drift_percent','xy_rmse_m','xy_rmse_percent','yaw/course_rmse_deg']].to_markdown(index=False)}

## 6. Axis-wise position error

Full signed/absolute E, N, U, XY, and 3D statistics are in each run's
`global_summary.json` under `runs/<flight>/cond3_oc_gpsz/`.

## 7. Axis-wise velocity error

{summaries[['flight','speed_rmse_mps','vxy_vec_rmse_mps']].to_markdown(index=False)}

## 8. Along-track / cross-track / vertical error

{summaries[['flight','final_along_error_m','final_cross_error_m','final_vertical_error_m','vertical_rmse_m']].to_markdown(index=False)}

## 9. Percent drift relative to distance

Global percentages divide final XY or XY RMSE by each flight's total horizontal
GPS distance. Segment percentages divide by local segment GPS distance.

## 10. Segment decomposition method

Straight detection uses GPS speed, course rate, heading variation, and local
line-fit residual. Legs at least 2 km are split every 500 m. The final short
remainder remains straight; all other samples remain assigned.

## 11. Segment error summary

{by_type.to_markdown(index=False)}

## 12. Straight-leg details

{straight.to_markdown(index=False)}

## 13. Turn / transition details

{turns.to_markdown(index=False)}

## 14. Figure list

`plots/` contains the standard 11 OC multi-flight panels plus the four
three-condition comparison figures.

## 15. Warnings and missing inputs

- GPS is an evaluation reference, not perfect ground truth.
- No LK-only trajectory or sufficient LK reconstruction inputs were present.
- Course error is velocity-direction error and is separately labeled from
  estimator body yaw.
- Per-flight reports and source-aligned samples remain authoritative; package
  aggregates must not hide a failed or truncated comparison condition.
"""
    (out / "FULL_FLIGHT_ERROR_ANALYSIS.md").write_text(text, encoding="utf-8")


def write_comparison_report(run_table: pd.DataFrame, aggregate: pd.DataFrame, out: Path, package: str) -> None:
    oc = run_table[run_table["condition"].str.contains("cond3")]
    nonoc = run_table[run_table["condition"].str.contains("cond2")]
    nogpsz = run_table[run_table["condition"].str.contains("cond1")]
    valid_nonoc = nonoc[~nonoc["notes"].str.contains("FAILED", case=False)]
    global_delta = oc.merge(valid_nonoc, on="flight", suffixes=("_oc", "_original"))
    for c in ["xy_rmse_m", "vxy_vec_rmse_mps", "yaw/course_rmse_deg"]:
        global_delta[f"delta_{c}"] = global_delta[f"{c}_oc"] - global_delta[f"{c}_original"]
    global_delta = global_delta[[
        "flight", "xy_rmse_m_oc", "xy_rmse_m_original", "delta_xy_rmse_m",
        "vxy_vec_rmse_mps_oc", "vxy_vec_rmse_mps_original", "delta_vxy_vec_rmse_mps",
        "yaw/course_rmse_deg_oc", "yaw/course_rmse_deg_original",
        "delta_yaw/course_rmse_deg",
    ]]
    gps_z_delta = oc.merge(nogpsz, on="flight", suffixes=("_oc_gpsz", "_oc_no_gpsz"))
    gps_z_delta["delta_xy_rmse_m"] = (
        gps_z_delta["xy_rmse_m_oc_gpsz"] - gps_z_delta["xy_rmse_m_oc_no_gpsz"])
    gps_z_delta = gps_z_delta[[
        "flight", "xy_rmse_m_oc_gpsz", "xy_rmse_m_oc_no_gpsz", "delta_xy_rmse_m"
    ]]
    seg_all = pd.read_csv(out / "all_segment_error_summary.csv")
    seg_oc = seg_all[seg_all["condition"] == "cond3_oc_gpsz"]
    seg_original = seg_all[
        (seg_all["condition"] == "cond2_gpsz_original") & (seg_all["flight"] != "fly2")
    ]
    seg_delta = seg_oc.merge(
        seg_original, on=["flight", "segment_id", "segment_type"],
        suffixes=("_oc", "_original"))
    for c in ["xy_rmse_m", "along_rmse_m", "cross_rmse_m", "vxy_vec_rmse_mps"]:
        seg_delta[f"delta_{c}"] = seg_delta[f"{c}_oc"] - seg_delta[f"{c}_original"]
    seg_delta["yaw_abs_improved"] = (
        seg_delta["yaw/course_error_mean_deg_oc"].abs()
        < seg_delta["yaw/course_error_mean_deg_original"].abs())
    segment_delta_table = seg_delta.groupby("segment_type").agg(
        matched_segments=("segment_id", "size"),
        oc_xy_win_percent=("delta_xy_rmse_m", lambda x: 100.0 * (x < 0).mean()),
        median_delta_xy_rmse_m=("delta_xy_rmse_m", "median"),
        median_delta_along_rmse_m=("delta_along_rmse_m", "median"),
        median_delta_cross_rmse_m=("delta_cross_rmse_m", "median"),
        median_delta_vxy_vec_rmse_mps=("delta_vxy_vec_rmse_mps", "median"),
        oc_course_mean_abs_win_percent=("yaw_abs_improved", lambda x: 100.0 * x.mean()),
    ).reset_index()
    oc_xy = oc["xy_rmse_m"].median()
    nonoc_xy = valid_nonoc["xy_rmse_m"].median()
    oc_yaw = oc["yaw/course_rmse_deg"].median()
    nonoc_yaw = valid_nonoc["yaw/course_rmse_deg"].median()
    verdict = (
        f"Across valid per-flight runs, OC+GPS-Z median XY RMSE is {oc_xy:.2f} m "
        f"versus {nonoc_xy:.2f} m for original+GPS-Z; median course RMSE is "
        f"{oc_yaw:.2f} deg versus {nonoc_yaw:.2f} deg. This is mixed rather than "
        "a universal OC win; inspect the per-flight and segment tables."
    )
    text = f"""# Three-Condition OC Comparison Analysis

## 1. Package path

`{package}`

## 2. Located conditions

- `cond1_nogpsz`: global yaw OC projection, GPS-Z off.
- `cond2_gpsz_original`: original OpenVINS yaw update, GPS-Z on.
- `cond3_oc_gpsz`: global yaw OC projection, GPS-Z on.

## 3. Which condition is OC

Conditions 1 and 3 use OC. Condition 3 is the controlled OC comparison against
Condition 2 because both have GPS-Z enabled.

## 4. Input completeness table

{run_table[['condition','flight','duration_s','gps_distance_km','notes']].to_markdown(index=False)}

## 5. Global comparison

{aggregate.to_markdown(index=False)}

## 6. Segment comparison

Matched OC-minus-original deltas below exclude failed Fly2 original. Negative
values favor OC.

{segment_delta_table.to_markdown(index=False)}

## 7. Straight-leg comparison

On matched valid straight segments, OC wins XY RMSE in
{segment_delta_table.loc[segment_delta_table['segment_type'] == 'straight', 'oc_xy_win_percent'].iloc[0]:.1f}% of segments.
The median OC-minus-original straight-segment XY RMSE delta is
{segment_delta_table.loc[segment_delta_table['segment_type'] == 'straight', 'median_delta_xy_rmse_m'].iloc[0]:+.2f} m.
This is not a broad straight-leg improvement. Use
`all_straight_leg_summary.csv` for each leg and split.

## 8. Turn/transition comparison

Matched turn segments show a median OC-minus-original XY RMSE delta of
{segment_delta_table.loc[segment_delta_table['segment_type'] == 'turn', 'median_delta_xy_rmse_m'].iloc[0]:+.2f} m,
with OC winning {segment_delta_table.loc[segment_delta_table['segment_type'] == 'turn', 'oc_xy_win_percent'].iloc[0]:.1f}%.
The result is mixed and does not show a turn-specific OC advantage.

## 9. Along/cross-track comparison

For matched straight segments, median OC-minus-original along RMSE is
{segment_delta_table.loc[segment_delta_table['segment_type'] == 'straight', 'median_delta_along_rmse_m'].iloc[0]:+.2f} m
and cross RMSE is
{segment_delta_table.loc[segment_delta_table['segment_type'] == 'straight', 'median_delta_cross_rmse_m'].iloc[0]:+.2f} m.
Both worsen in the median, so OC is not merely trading along-track error for
lower cross-track heading drift.

## 10. Velocity comparison

Per-flight OC-minus-original global deltas (negative favors OC):

{global_delta.to_markdown(index=False)}

OC improves horizontal velocity-vector RMSE on Fly1, but worsens it on Fly3
and Fly4. This mirrors the mixed position result.

## 11. Yaw/course comparison

Course is derived independently from GPS and aligned VIO velocity. It is not a
smoothed GPS heading injected as estimator truth.

OC improves course RMSE on Fly1 but worsens it on Fly3 and Fly4. The segment
table also shows that lower absolute mean course error is not common across
matched straight or turn segments.

## 12. OC verdict

{verdict}

The comparison does not support claiming that OC fixes all drift globally.
Against original+GPS-Z, OC improves Fly1 but worsens Fly3 and Fly4 in XY RMSE,
velocity-vector RMSE, and course RMSE. The matched segment results likewise do
not show a general straight or turn advantage.

The OC+GPS-Z versus OC-without-GPS-Z table isolates the GPS-Z change, not OC:

{gps_z_delta.to_markdown(index=False)}

GPS-Z improves OC XY RMSE on all four flights and is decisive on Fly3. Because
the controlled OC-versus-original comparison does not consistently improve
course, cross-track, or local segment shape, the package evidence says the
largest global gain is the GPS-Z scale/height anchor rather than OC alone.

## 13. Limitations

- Fly2 original+GPS-Z is a failed partial run and is excluded from condition
  aggregates and comparison plots while retained in the per-flight table and
  detailed run outputs.
- Fly3 no-GPS-Z is shorter than the two GPS-Z runs.
- GPS is a reference, not perfect ground truth.
- No LK-only inputs were present.

## 14. Recommended next use of the skill

Run the same entry point and definitions for future flights; provide a manifest
when comparing conditions and preserve failed/truncated runs with explicit
notes.
"""
    (out / "THREE_CONDITION_OC_COMPARISON_ANALYSIS.md").write_text(text, encoding="utf-8")


def run_manifest(path: Path) -> None:
    spec = json.loads(path.read_text(encoding="utf-8"))
    root = Path(spec["out_dir"])
    root.mkdir(parents=True, exist_ok=True)
    results = []
    for run in spec["runs"]:
        ns = argparse.Namespace(**{
            "gps": run["gps"], "vio_traj": run["vio_traj"],
            "vio_bias": run.get("vio_bias"), "vio_diag": run.get("vio_diag"),
            "vio_yaw_diag": run.get("vio_yaw_diag"), "lk_traj": run.get("lk_traj"),
            "lk_flow_csv": run.get("lk_flow_csv"),
            "out_dir": str(root / "runs" / run["flight_name"] / run["method_name"]),
            "t0": run.get("t0"), "t1": run.get("t1"),
            "flight_name": run["flight_name"], "method_name": run["method_name"],
            "notes": run.get("notes", ""), "heading_window_s": spec.get("heading_window_s", 60.0),
            "heading_stable_window_s": spec.get("heading_stable_window_s", 5.0),
            "min_speed_mps": spec.get("min_speed_mps", 5.0),
            "max_course_rate_degps": spec.get("max_course_rate_degps", 1.0),
            "max_line_fit_rmse_m": spec.get("max_line_fit_rmse_m", 25.0),
            "max_heading_std_deg": spec.get("max_heading_std_deg", 5.0),
            "min_straight_len_m": spec.get("min_straight_len_m", 2000.0),
            "straight_split_len_m": spec.get("straight_split_len_m", 500.0),
            "experiment_id": run.get("experiment_id", ""),
            "source_package": run.get("source_package", spec.get("package_path", "")),
            "source_member": run.get("source_member", ""),
            "experiment_config": run.get("experiment_config", ""),
            "expected_xy_rmse_m": run.get("expected_xy_rmse_m"),
            "run_status": run.get("run_status", ""),
            "min_segment_drift_distance_m": spec.get(
                "min_segment_drift_distance_m", 50.0),
            "max_vio_sample_delay_s": spec.get("max_vio_sample_delay_s", 0.1),
            "gps_gap_threshold_s": spec.get("gps_gap_threshold_s", 0.5),
            "crop_reason": run.get("crop_reason", ""),
        })
        results.append(analyze_run(ns))
    run_table = pd.DataFrame([
        {k: v for k, v in r["summary"].items() if k in SUMMARY_FIELDS} for r in results
    ])
    run_table.to_csv(root / "flight_condition_comparison_summary.csv", index=False)
    aggregate = pd.DataFrame([
        aggregate_condition(group)
        for _, group in run_table.groupby("condition", sort=False)
    ]).reset_index(drop=True)
    aggregate.to_csv(root / "condition_comparison_summary.csv", index=False)
    all_seg = pd.concat([
        r["segments"].assign(condition=r["summary"]["condition"], flight=r["summary"]["flight"])
        for r in results
    ], ignore_index=True)
    all_seg.to_csv(root / "all_segment_error_summary.csv", index=False)
    all_seg[all_seg["segment_type"] == "straight"].to_csv(root / "all_straight_leg_summary.csv", index=False)
    all_seg[all_seg["segment_type"].isin(["turn", "transition"])].to_csv(
        root / "all_turn_transition_summary.csv", index=False)
    primary = spec.get("primary_condition", "cond3_oc_gpsz")
    pr = [r for r in results if r["summary"]["condition"] == primary]
    pd.concat([r["aligned"].assign(flight=r["summary"]["flight"]) for r in pr],
              ignore_index=True).to_csv(root / "gps_time_aligned_samples.csv", index=False)
    pd.DataFrame([{k: v for k, v in r["summary"].items() if k != "metrics"} for r in pr]).to_csv(
        root / "global_summary.csv", index=False)
    (root / "global_summary.json").write_text(
        json.dumps([r["summary"] for r in pr], indent=2), encoding="utf-8")
    pd.concat([r["index"].assign(flight=r["summary"]["flight"]) for r in pr],
              ignore_index=True).to_csv(root / "segment_index.csv", index=False)
    pseg = pd.concat([r["segments"].assign(flight=r["summary"]["flight"]) for r in pr],
                     ignore_index=True)
    pseg.to_csv(root / "segment_error_summary.csv", index=False)
    pseg[pseg["segment_type"] == "straight"].to_csv(root / "straight_leg_summary.csv", index=False)
    pseg[pseg["segment_type"].isin(["turn", "transition"])].to_csv(
        root / "turn_transition_summary.csv", index=False)
    comparison_figures(results, root / "figures")
    primary_figure_montages(pr, root / "figures")
    height_rows = []
    for r in pr:
        d = r["aligned"]
        layers = pd.cut(d["gps_U"], bins=6, duplicates="drop")
        h = d.groupby(layers, observed=True).agg(
            n=("t", "size"), gps_u_mean_m=("gps_U", "mean"),
            xy_rmse_m=("err_XY", lambda x: metric_stats(x.to_numpy())["rmse"]),
            vertical_rmse_m=("err_vertical", lambda x: metric_stats(x.to_numpy())["rmse"]),
        ).reset_index()
        h["height_layer"] = h["gps_U"].astype(str)
        h["flight"] = r["summary"]["flight"]
        height_rows.append(h.drop(columns=["gps_U"]))
    pd.concat(height_rows, ignore_index=True).to_csv(root / "height_layer_summary.csv", index=False)
    write_comparison_report(run_table, aggregate, root, spec.get("package_path", str(path)))
    write_primary_package_report(pr, root)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--manifest", help="JSON manifest for reusable multi-run comparison")
    p.add_argument("--run-spec", help="JSON specification for one experiment run")
    p.add_argument("--gps")
    p.add_argument("--vio-traj")
    p.add_argument("--vio-bias")
    p.add_argument("--vio-diag")
    p.add_argument("--vio-yaw-diag")
    p.add_argument("--lk-traj")
    p.add_argument("--lk-flow-csv")
    p.add_argument("--out-dir")
    p.add_argument("--t0", type=float)
    p.add_argument("--t1", type=float)
    p.add_argument("--crop-reason", default="")
    p.add_argument("--flight-name", default="flight")
    p.add_argument("--method-name", default="vio")
    p.add_argument("--notes", default="")
    p.add_argument("--experiment-id", default="")
    p.add_argument("--source-package", default="")
    p.add_argument("--source-member", default="")
    p.add_argument("--experiment-config", default="")
    p.add_argument("--expected-xy-rmse-m", type=float)
    p.add_argument("--run-status", default="")
    p.add_argument("--heading-window-s", type=float, default=60.0)
    p.add_argument("--heading-stable-window-s", type=float, default=5.0)
    p.add_argument("--min-speed-mps", type=float, default=5.0)
    p.add_argument("--max-course-rate-degps", type=float, default=1.0)
    p.add_argument("--max-line-fit-rmse-m", type=float, default=25.0)
    p.add_argument("--max-heading-std-deg", type=float, default=5.0)
    p.add_argument("--min-straight-len-m", type=float, default=2000.0)
    p.add_argument("--straight-split-len-m", type=float, default=500.0)
    p.add_argument("--min-segment-drift-distance-m", type=float, default=50.0)
    p.add_argument("--max-vio-sample-delay-s", type=float, default=0.1)
    p.add_argument("--gps-gap-threshold-s", type=float, default=0.5)
    return p


def main() -> None:
    args = parser().parse_args()
    if args.manifest:
        run_manifest(Path(args.manifest))
        return
    if args.run_spec:
        spec = json.loads(Path(args.run_spec).read_text(encoding="utf-8"))
        defaults = vars(parser().parse_args([]))
        defaults.update(spec)
        defaults["run_spec"] = args.run_spec
        args = argparse.Namespace(**defaults)
    required = ["gps", "vio_traj", "out_dir"]
    missing = [x for x in required if not getattr(args, x)]
    if missing:
        raise SystemExit("Missing required single-run arguments: " + ", ".join("--" + x.replace("_", "-") for x in missing))
    analyze_run(args)


if __name__ == "__main__":
    main()
