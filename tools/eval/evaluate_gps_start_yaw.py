#!/usr/bin/env python3
"""Evaluate VIO reference trajectories against raw ArduPilot GPS.

Spatial alignment is intentionally restricted to:
  1. translation at the first clearly-moving GPS point
  2. yaw from a short initial motion window

No global SE(3) / Sim(3) trajectory fit is used for metrics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


DEFAULT_CASES = [
    {
        "fly": "fly1",
        "gps": r"D:\vscode_dir\20260509_fly2_gps\GPS.csv",
        "trajectories": [
            r"D:\vscode_dir\open_vins\20260509_fly1\reference\one_traj_estimate_stereo.txt",
        ],
    },
    {
        "fly": "fly2",
        "gps": r"D:\vscode_dir\20260509_fly3_gps\GPS.csv",
        "trajectories": [
            r"D:\vscode_dir\open_vins\20260509_fly2\reference\two_traj_estimate_mono.txt",
            r"D:\vscode_dir\open_vins\20260509_fly2\reference\two_traj_estimate_stereo.txt",
        ],
    },
    {
        "fly": "fly3",
        "gps": r"D:\vscode_dir\20260509_fly4_gps\GPS.csv",
        "trajectories": [
            r"D:\vscode_dir\open_vins\20260509_fly3\reference\three_traj_estimate_stereo.txt",
        ],
    },
    {
        "fly": "fly4",
        "gps": r"D:\vscode_dir\20260509_fly5_gps\GPS.csv",
        "trajectories": [
            r"D:\vscode_dir\open_vins\20260509_fly4\reference\four_traj_estimate_mono.txt",
            r"D:\vscode_dir\open_vins\20260509_fly4\reference\four_traj_estimate_stereo.txt",
        ],
    },
]


@dataclass
class GpsData:
    time_us: np.ndarray
    rel_t: np.ndarray
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray
    enu: np.ndarray


@dataclass
class Trajectory:
    path: Path
    t_abs: np.ndarray
    t_rel: np.ndarray
    p: np.ndarray
    q_xyzw: np.ndarray


@dataclass
class EvalData:
    gps_t: np.ndarray
    traj_t: np.ndarray
    gps_p: np.ndarray
    vio_p: np.ndarray
    vio_aligned: np.ndarray
    errors: np.ndarray


def wrap_angle(a: float | np.ndarray) -> float | np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def path_length(p: np.ndarray) -> float:
    if len(p) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def lla_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, alt_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.column_stack(
        (
            (n + alt_m) * cos_lat * np.cos(lon),
            (n + alt_m) * cos_lat * np.sin(lon),
            (n * (1.0 - WGS84_E2) + alt_m) * sin_lat,
        )
    )


def ecef_to_enu(ecef: np.ndarray, lat0_deg: float, lon0_deg: float, origin_ecef: np.ndarray) -> np.ndarray:
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    slat, clat = math.sin(lat0), math.cos(lat0)
    slon, clon = math.sin(lon0), math.cos(lon0)
    rot = np.array(
        [
            [-slon, clon, 0.0],
            [-slat * clon, -slat * slon, clat],
            [clat * clon, clat * slon, slat],
        ]
    )
    return (ecef - origin_ecef.reshape(1, 3)) @ rot.T


def load_gps(path: Path, min_status: int = 4) -> GpsData:
    rows = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                status = int(float(row.get("Status", min_status)))
                if status < min_status:
                    continue
                rows.append(
                    (
                        int(float(row["TimeUS"])),
                        float(row["Lat"]),
                        float(row["Lng"]),
                        float(row["Alt"]),
                    )
                )
            except (KeyError, ValueError):
                continue
    if len(rows) < 30:
        raise ValueError(f"Not enough GPS rows in {path}")
    arr = np.asarray(rows, dtype=object)
    time_us = arr[:, 0].astype(np.int64)
    lat = arr[:, 1].astype(np.float64)
    lon = arr[:, 2].astype(np.float64)
    alt = arr[:, 3].astype(np.float64)
    ecef = lla_to_ecef(lat, lon, alt)
    enu = ecef_to_enu(ecef, float(lat[0]), float(lon[0]), ecef[0])
    rel_t = (time_us - time_us[0]).astype(np.float64) * 1e-6
    return GpsData(time_us=time_us, rel_t=rel_t, lat=lat, lon=lon, alt=alt, enu=enu)


def load_traj(path: Path) -> Trajectory:
    rows = []
    with path.open("r") as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 8:
                continue
            try:
                rows.append([float(x) for x in parts[:8]])
            except ValueError:
                continue
    if len(rows) < 30:
        raise ValueError(f"Not enough trajectory rows in {path}")
    arr = np.asarray(rows, dtype=np.float64)
    t_abs = arr[:, 0]
    return Trajectory(path=path, t_abs=t_abs, t_rel=t_abs - t_abs[0], p=arr[:, 1:4], q_xyzw=arr[:, 4:8])


def interp_cols(src_t: np.ndarray, src_y: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(query_t, src_t, src_y[:, i]) for i in range(src_y.shape[1])])


def smooth(values: np.ndarray, n: int) -> np.ndarray:
    n = max(1, int(n))
    return np.convolve(values, np.ones(n, dtype=np.float64) / float(n), mode="same")


def speed_series(t: np.ndarray, p: np.ndarray, dt: float, t_max: float, smooth_s: float) -> tuple[np.ndarray, np.ndarray]:
    grid = np.arange(0.0, min(float(t[-1]), t_max) + 0.5 * dt, dt)
    pos = interp_cols(t, p, grid)
    sp = np.linalg.norm(np.diff(pos, axis=0), axis=1) / dt
    mid = grid[:-1] + 0.5 * dt
    return mid, smooth(sp, int(round(smooth_s / dt)))


def first_sustained_motion(t: np.ndarray, xy: np.ndarray, threshold: float = 0.7, min_s: float = 3.0) -> float:
    dt = 0.1
    ts, sp = speed_series(t, xy, dt=dt, t_max=float(t[-1]), smooth_s=1.5)
    n = max(1, int(round(min_s / dt)))
    for th in (threshold, 0.5, 0.3):
        for i in range(0, max(0, len(sp) - n)):
            if np.all(sp[i : i + n] > th):
                return float(ts[i])
    return float(ts[int(np.argmax(sp))])


def normalized_score(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    if len(a) < 10 or len(b) < 10 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float("nan"), float("nan")
    aa = (a - np.mean(a)) / np.std(a)
    bb = (b - np.mean(b)) / np.std(b)
    return float(np.sqrt(np.mean((aa - bb) ** 2))), float(np.mean(aa * bb))


def estimate_time_offset(
    gps: GpsData,
    traj: Trajectory,
    scan_min: float = -300.0,
    scan_max: float = 300.0,
    coarse_step: float = 0.2,
    fine_step: float = 0.01,
) -> tuple[float, dict]:
    move_start = first_sustained_motion(gps.rel_t, gps.enu[:, :2])
    win0 = max(0.0, move_start - 10.0)
    win1 = min(float(gps.rel_t[-1]), move_start + 120.0)
    dt = 0.1
    gps_st, gps_sp = speed_series(gps.rel_t, gps.enu[:, :2], dt, win1 + 20.0, 3.0)
    vio_st, vio_sp = speed_series(traj.t_rel, traj.p[:, :2], dt, float(traj.t_rel[-1]), 3.0)
    mask = (gps_st >= win0) & (gps_st <= win1)
    total = int(mask.sum())
    if total < 20:
        raise ValueError("Not enough GPS speed samples for time alignment")

    def score(offset: float) -> tuple[float, float, int]:
        q = gps_st[mask] + offset
        valid = (q >= vio_st[0]) & (q <= vio_st[-1])
        if int(valid.sum()) < max(20, int(0.55 * total)):
            return float("nan"), float("nan"), int(valid.sum())
        rmse, corr = normalized_score(gps_sp[mask][valid], np.interp(q[valid], vio_st, vio_sp))
        return rmse, corr, int(valid.sum())

    coarse_offsets = np.arange(scan_min, scan_max + 0.5 * coarse_step, coarse_step)
    coarse = []
    for off in coarse_offsets:
        rmse, corr, n = score(float(off))
        if math.isfinite(rmse):
            coarse.append((rmse, corr, n, float(off)))
    if not coarse:
        raise ValueError("No valid time offset found")
    coarse_best = min(coarse, key=lambda x: x[0])
    fine_offsets = np.arange(coarse_best[3] - 2.0, coarse_best[3] + 2.0 + 0.5 * fine_step, fine_step)
    fine = []
    for off in fine_offsets:
        rmse, corr, n = score(float(off))
        if math.isfinite(rmse):
            fine.append((rmse, corr, n, float(off)))
    best = min(fine, key=lambda x: x[0]) if fine else coarse_best
    return best[3], {
        "gps_motion_start_s": move_start,
        "speed_window_s": [win0, win1],
        "coarse_best_offset_s": coarse_best[3],
        "best_offset_s": best[3],
        "best_norm_speed_rmse": best[0],
        "best_speed_corr": best[1],
        "speed_pairs": best[2],
    }


def fit_motion_direction(t: np.ndarray, xy: np.ndarray) -> np.ndarray:
    # Use the displacement between short averaged endpoint chunks first. It is
    # less jumpy than a single endpoint pair, while still respecting the initial
    # direction instead of doing any trajectory-wide fit.
    if len(xy) >= 8:
        n = max(3, min(25, len(xy) // 5))
        v = np.mean(xy[-n:], axis=0) - np.mean(xy[:n], axis=0)
        if np.linalg.norm(v) >= 1.0:
            return v
    x = t - float(np.mean(t))
    vx = float(np.dot(x, xy[:, 0] - np.mean(xy[:, 0])) / (np.dot(x, x) + 1e-12))
    vy = float(np.dot(x, xy[:, 1] - np.mean(xy[:, 1])) / (np.dot(x, x) + 1e-12))
    v = np.array([vx, vy], dtype=np.float64)
    if np.linalg.norm(v) < 1e-9:
        v = xy[-1] - xy[0]
    return v


def rotation_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def make_eval(gps: GpsData, traj: Trajectory, time_offset: float, direction_window_s: float) -> tuple[EvalData, dict]:
    move_start = first_sustained_motion(gps.rel_t, gps.enu[:, :2])
    align_start = max(move_start, -time_offset)
    direction_window_s = min(float(direction_window_s), 30.0)
    align_end = min(float(gps.rel_t[-1]), align_start + direction_window_s)
    dir_mask = (gps.rel_t >= align_start) & (gps.rel_t <= align_end)
    if int(dir_mask.sum()) < 5:
        raise ValueError("Not enough samples in initial direction window")
    q_dir = gps.rel_t[dir_mask] + time_offset
    valid_dir = (q_dir >= traj.t_rel[0]) & (q_dir <= traj.t_rel[-1])
    if int(valid_dir.sum()) < 5:
        raise ValueError("Initial direction window has no trajectory overlap")
    gps_dir_p = gps.enu[dir_mask][valid_dir]
    vio_dir_p = interp_cols(traj.t_rel, traj.p, q_dir[valid_dir])
    gps_dir = fit_motion_direction(gps.rel_t[dir_mask][valid_dir], gps_dir_p[:, :2])
    vio_dir = fit_motion_direction(gps.rel_t[dir_mask][valid_dir], vio_dir_p[:, :2])
    yaw_gps = math.atan2(gps_dir[1], gps_dir[0])
    yaw_vio = math.atan2(vio_dir[1], vio_dir[0])
    yaw_align = float(wrap_angle(yaw_gps - yaw_vio))
    r2 = rotation_z(yaw_align)

    eval_mask = gps.rel_t >= align_start
    q = gps.rel_t[eval_mask] + time_offset
    valid = (q >= traj.t_rel[0]) & (q <= traj.t_rel[-1])
    gps_t = gps.rel_t[eval_mask][valid]
    gps_p = gps.enu[eval_mask][valid]
    traj_t = q[valid]
    vio_p = interp_cols(traj.t_rel, traj.p, traj_t)
    gps_start = np.interp(align_start, gps.rel_t, gps.enu[:, 0]), np.interp(align_start, gps.rel_t, gps.enu[:, 1]), np.interp(align_start, gps.rel_t, gps.enu[:, 2])
    vio_start = interp_cols(traj.t_rel, traj.p, np.array([align_start + time_offset]))[0]

    vio_aligned = np.empty_like(vio_p)
    vio_aligned[:, :2] = (vio_p[:, :2] - vio_start[:2]) @ r2.T + np.array(gps_start[:2])
    vio_aligned[:, 2] = vio_p[:, 2] - vio_start[2] + gps_start[2]
    errors = vio_aligned - gps_p
    info = {
        "align_start_gps_rel_s": float(align_start),
        "direction_window_s": [float(align_start), float(align_end)],
        "direction_window_duration_s": float(align_end - align_start),
        "yaw_gps_deg": math.degrees(yaw_gps),
        "yaw_vio_deg": math.degrees(yaw_vio),
        "yaw_align_deg": math.degrees(yaw_align),
    }
    return EvalData(gps_t=gps_t, traj_t=traj_t, gps_p=gps_p, vio_p=vio_p, vio_aligned=vio_aligned, errors=errors), info


def stats(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=np.float64)
    return {
        "rmse": float(np.sqrt(np.mean(values * values))) if len(values) else float("nan"),
        "mean": float(np.mean(values)) if len(values) else float("nan"),
        "median": float(np.median(values)) if len(values) else float("nan"),
        "p90": float(np.percentile(values, 90)) if len(values) else float("nan"),
        "max": float(np.max(values)) if len(values) else float("nan"),
    }


def rpe_metrics(data: EvalData, delta_s: float) -> dict:
    errs = []
    yaw_errs = []
    for i, t in enumerate(data.gps_t):
        target = t + delta_s
        j = int(np.searchsorted(data.gps_t, target))
        if j >= len(data.gps_t):
            break
        if abs(data.gps_t[j] - target) > 0.35:
            continue
        dg = data.gps_p[j] - data.gps_p[i]
        dv = data.vio_aligned[j] - data.vio_aligned[i]
        errs.append(np.linalg.norm(dv - dg))
        if np.linalg.norm(dg[:2]) > 1.0 and np.linalg.norm(dv[:2]) > 1.0:
            yaw_errs.append(math.degrees(float(wrap_angle(math.atan2(dv[1], dv[0]) - math.atan2(dg[1], dg[0])))))
    return {
        f"rpe_{delta_s:g}s_rmse_m": stats(np.asarray(errs))["rmse"],
        f"rpe_{delta_s:g}s_median_m": stats(np.asarray(errs))["median"],
        f"rpe_{delta_s:g}s_yaw_rmse_deg": stats(np.asarray(yaw_errs))["rmse"],
        f"rpe_{delta_s:g}s_count": len(errs),
    }


def whole_metrics(data: EvalData) -> dict:
    e2 = np.linalg.norm(data.errors[:, :2], axis=1)
    e3 = np.linalg.norm(data.errors, axis=1)
    ez = np.abs(data.errors[:, 2])
    gps_len_xy = path_length(data.gps_p[:, :2])
    gps_len_3d = path_length(data.gps_p)
    vio_len_xy = path_length(data.vio_aligned[:, :2])
    vio_len_3d = path_length(data.vio_aligned)
    final_e2 = float(np.linalg.norm(data.errors[-1, :2]))
    final_e3 = float(np.linalg.norm(data.errors[-1]))
    final_z = float(data.errors[-1, 2])
    out = {
        "samples": len(data.gps_t),
        "duration_s": float(data.gps_t[-1] - data.gps_t[0]) if len(data.gps_t) else 0.0,
        "gps_path_xy_m": gps_len_xy,
        "gps_path_3d_m": gps_len_3d,
        "vio_path_xy_m": vio_len_xy,
        "vio_path_3d_m": vio_len_3d,
        "path_scale_xy_ratio": vio_len_xy / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
        "path_scale_3d_ratio": vio_len_3d / gps_len_3d if gps_len_3d > 1e-9 else float("nan"),
        "path_length_xy_error_pct": 100.0 * (vio_len_xy - gps_len_xy) / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
        "path_length_3d_error_pct": 100.0 * (vio_len_3d - gps_len_3d) / gps_len_3d if gps_len_3d > 1e-9 else float("nan"),
        "ate2d_rmse_m": stats(e2)["rmse"],
        "ate2d_mean_m": stats(e2)["mean"],
        "ate2d_median_m": stats(e2)["median"],
        "ate2d_p90_m": stats(e2)["p90"],
        "ate2d_max_m": stats(e2)["max"],
        "ate3d_rmse_m": stats(e3)["rmse"],
        "ate3d_mean_m": stats(e3)["mean"],
        "ate3d_median_m": stats(e3)["median"],
        "ate_z_rmse_m": stats(ez)["rmse"],
        "ate_z_mean_abs_m": stats(ez)["mean"],
        "ate_z_p90_abs_m": stats(ez)["p90"],
        "final_2d_error_m": final_e2,
        "final_3d_error_m": final_e3,
        "final_z_error_m": final_z,
        "final_abs_z_error_m": abs(final_z),
        "final_2d_drift_pct": 100.0 * final_e2 / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
        "final_3d_drift_pct": 100.0 * final_e3 / gps_len_3d if gps_len_3d > 1e-9 else float("nan"),
    }
    out.update(rpe_metrics(data, 1.0))
    out.update(rpe_metrics(data, 5.0))
    return out


def point_line_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return np.linalg.norm(p - a, axis=1)
    t = np.clip(((p - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(p - proj, axis=1)


def rdp_indices(points: np.ndarray, eps: float) -> list[int]:
    if len(points) <= 2:
        return list(range(len(points)))
    d = point_line_distance(points[1:-1], points[0], points[-1])
    idx = int(np.argmax(d)) + 1
    if float(d[idx - 1]) > eps:
        left = rdp_indices(points[: idx + 1], eps)
        right = rdp_indices(points[idx:], eps)
        return left[:-1] + [i + idx for i in right]
    return [0, len(points) - 1]


def detect_edges(data: EvalData, min_edge_len_m: float = 45.0) -> list[tuple[int, int]]:
    xy = data.gps_p[:, :2]
    xy_s = np.column_stack([smooth(xy[:, 0], 7), smooth(xy[:, 1], 7)])
    best_edges: list[tuple[int, int]] = []
    best_score = 1e9
    for eps in (4.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 28.0):
        verts = sorted(set(rdp_indices(xy_s, eps)))
        edges = []
        for a, b in zip(verts[:-1], verts[1:]):
            if b <= a + 3:
                continue
            straight = float(np.linalg.norm(xy[b] - xy[a]))
            duration = float(data.gps_t[b] - data.gps_t[a])
            if straight >= min_edge_len_m and duration >= 5.0:
                edges.append((a, b))
        if not edges:
            continue
        rem = len(edges) % 4
        mod_penalty = 0 if rem == 0 else min(rem, 4 - rem)
        score = mod_penalty * 100 + abs(len(edges) - (8 if len(edges) > 6 else 4))
        if score < best_score:
            best_score = score
            best_edges = edges
    return best_edges


def edge_metrics(data: EvalData, edges: list[tuple[int, int]]) -> list[dict]:
    rows = []
    for k, (a, b) in enumerate(edges, 1):
        gps = data.gps_p[a : b + 1]
        vio = data.vio_aligned[a : b + 1]
        if len(gps) < 2:
            continue
        gps_d = gps[-1] - gps[0]
        vio_d = vio[-1] - vio[0]
        gps_len_xy = path_length(gps[:, :2])
        vio_len_xy = path_length(vio[:, :2])
        gps_straight = float(np.linalg.norm(gps_d[:2]))
        vio_straight = float(np.linalg.norm(vio_d[:2]))
        heading_g = math.atan2(gps_d[1], gps_d[0])
        heading_v = math.atan2(vio_d[1], vio_d[0])
        end_err = vio_d - gps_d
        local_err = (vio - vio[0]) - (gps - gps[0])
        local_e2 = np.linalg.norm(local_err[:, :2], axis=1)
        local_e3 = np.linalg.norm(local_err, axis=1)
        local_ez = np.abs(local_err[:, 2])
        gps_step = np.diff(gps[:, :2], axis=0)
        vio_step = np.diff(vio[:, :2], axis=0)
        gps_step_len = np.linalg.norm(gps_step, axis=1)
        vio_step_len = np.linalg.norm(vio_step, axis=1)
        angle_mask = (gps_step_len > 0.25) & (vio_step_len > 0.25)
        if np.any(angle_mask):
            gps_step_yaw = np.arctan2(gps_step[angle_mask, 1], gps_step[angle_mask, 0])
            vio_step_yaw = np.arctan2(vio_step[angle_mask, 1], vio_step[angle_mask, 0])
            step_yaw_err = np.rad2deg(wrap_angle(vio_step_yaw - gps_step_yaw))
            step_yaw_abs = np.abs(step_yaw_err)
        else:
            step_yaw_err = np.asarray([], dtype=np.float64)
            step_yaw_abs = np.asarray([], dtype=np.float64)
        unit = gps_d[:2] / (gps_straight + 1e-12)
        left = np.array([-unit[1], unit[0]])
        along = float(np.dot(end_err[:2], unit))
        lateral = float(np.dot(end_err[:2], left))
        local_rmse_2d = stats(local_e2)["rmse"]
        rows.append(
            {
                "edge_id": k,
                "lap": (k - 1) // 4 + 1,
                "edge_in_lap": (k - 1) % 4 + 1,
                "start_gps_rel_s": float(data.gps_t[a]),
                "end_gps_rel_s": float(data.gps_t[b]),
                "duration_s": float(data.gps_t[b] - data.gps_t[a]),
                "gps_path_xy_m": gps_len_xy,
                "vio_path_xy_m": vio_len_xy,
                "gps_straight_m": gps_straight,
                "vio_straight_m": vio_straight,
                "rmse_2d_m": local_rmse_2d,
                "mean_2d_m": stats(local_e2)["mean"],
                "p90_2d_m": stats(local_e2)["p90"],
                "max_2d_m": stats(local_e2)["max"],
                "rmse_3d_m": stats(local_e3)["rmse"],
                "mean_3d_m": stats(local_e3)["mean"],
                "rmse_z_m": stats(local_ez)["rmse"],
                "mean_abs_z_m": stats(local_ez)["mean"],
                "endpoint_error_2d_m": float(np.linalg.norm(end_err[:2])),
                "endpoint_error_3d_m": float(np.linalg.norm(end_err)),
                "drift_pct_of_gps_path": 100.0 * float(np.linalg.norm(end_err[:2])) / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
                "rmse_pct_of_gps_path": 100.0 * local_rmse_2d / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
                "heading_gps_deg": math.degrees(heading_g),
                "heading_vio_deg": math.degrees(heading_v),
                "heading_offset_deg": math.degrees(float(wrap_angle(heading_v - heading_g))),
                "mean_abs_heading_error_deg": stats(step_yaw_abs)["mean"],
                "rmse_heading_error_deg": stats(step_yaw_err)["rmse"],
                "p90_abs_heading_error_deg": stats(step_yaw_abs)["p90"],
                "heading_error_samples": int(len(step_yaw_err)),
                "straight_scale_ratio": vio_straight / gps_straight if gps_straight > 1e-9 else float("nan"),
                "path_scale_ratio": vio_len_xy / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
                "path_scale_error_pct": 100.0 * (vio_len_xy - gps_len_xy) / gps_len_xy if gps_len_xy > 1e-9 else float("nan"),
                "endpoint_along_error_m": along,
                "endpoint_lateral_error_m": lateral,
                "endpoint_z_error_m": float(end_err[2]),
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: float, digits: int = 2, suffix: str = "") -> str:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return "n/a"
    if not math.isfinite(v):
        return "n/a"
    return f"{v:.{digits}f}{suffix}"


def global_metric_text(row: dict) -> str:
    lines = [
        "Alignment",
        f"time offset: {fmt(row.get('time_offset_s'), 3, ' s')}",
        f"yaw align:   {fmt(row.get('yaw_align_deg'), 2, ' deg')}",
        f"yaw window:  {fmt(row.get('direction_window_duration_s'), 1, ' s')}",
        f"duration:    {fmt(row.get('duration_s'), 1, ' s')}",
        f"samples:     {row.get('samples', 'n/a')}",
        "",
        "ATE / Error",
        f"ATE XY RMSE: {fmt(row.get('ate2d_rmse_m'), 2, ' m')}",
        f"ATE XY mean: {fmt(row.get('ate2d_mean_m'), 2, ' m')}",
        f"ATE XY p90:  {fmt(row.get('ate2d_p90_m'), 2, ' m')}",
        f"ATE 3D RMSE: {fmt(row.get('ate3d_rmse_m'), 2, ' m')}",
        f"height RMSE: {fmt(row.get('ate_z_rmse_m'), 2, ' m')}",
        f"height p90:  {fmt(row.get('ate_z_p90_abs_m'), 2, ' m')}",
        f"final XY:    {fmt(row.get('final_2d_error_m'), 2, ' m')}",
        f"final Z:     {fmt(row.get('final_z_error_m'), 2, ' m')}",
        "",
        "RPE",
        f"1s RMSE:     {fmt(row.get('rpe_1s_rmse_m'), 2, ' m')}",
        f"1s yaw RMSE: {fmt(row.get('rpe_1s_yaw_rmse_deg'), 2, ' deg')}",
        f"5s RMSE:     {fmt(row.get('rpe_5s_rmse_m'), 2, ' m')}",
        f"5s yaw RMSE: {fmt(row.get('rpe_5s_yaw_rmse_deg'), 2, ' deg')}",
        "",
        "Length / Drift",
        f"GPS XY len:  {fmt(row.get('gps_path_xy_m'), 1, ' m')}",
        f"VIO XY len:  {fmt(row.get('vio_path_xy_m'), 1, ' m')}",
        f"path scale:  {fmt(row.get('path_scale_xy_ratio'), 4)}",
        f"path diff:   {fmt(row.get('path_length_xy_error_pct'), 2, ' %')}",
        f"final drift: {fmt(row.get('final_2d_drift_pct'), 2, ' %')}",
        f"edges:       {row.get('edges_detected', 'n/a')}",
    ]
    return "\n".join(lines)


def edge_metric_text(row: dict) -> str:
    return "\n".join(
        [
            f"end XY:      {fmt(row.get('endpoint_error_2d_m'), 1, ' m')}",
            f"drift:       {fmt(row.get('drift_pct_of_gps_path'), 1, '%')}",
            f"RMSE XY:     {fmt(row.get('rmse_2d_m'), 1, ' m')}",
            f"p90 XY:      {fmt(row.get('p90_2d_m'), 1, ' m')}",
            f"RMSE Z:      {fmt(row.get('rmse_z_m'), 1, ' m')}",
            f"end Z:       {fmt(row.get('endpoint_z_error_m'), 1, ' m')}",
            f"mean yaw:    {fmt(row.get('mean_abs_heading_error_deg'), 1, ' deg')}",
            f"end yaw off: {fmt(row.get('heading_offset_deg'), 1, ' deg')}",
            f"scale line:  {fmt(row.get('straight_scale_ratio'), 3)}",
            f"scale path:  {fmt(row.get('path_scale_ratio'), 3)}",
        ]
    )


def plot_xy(data: EvalData, edges: list[tuple[int, int]], row: dict, out: Path, title: str) -> None:
    fig, (ax, ax_text) = plt.subplots(1, 2, figsize=(13.8, 7.4), gridspec_kw={"width_ratios": [3.0, 1.35]})
    ax.plot(data.gps_p[:, 0], data.gps_p[:, 1], "g-", lw=2, label="GPS")
    ax.plot(data.vio_aligned[:, 0], data.vio_aligned[:, 1], color="tab:blue", lw=1.5, label="VIO start+yaw aligned")
    for i, (a, b) in enumerate(edges, 1):
        ax.scatter(data.gps_p[[a, b], 0], data.gps_p[[a, b], 1], s=18, c=["black", "red"])
        mid = 0.5 * (data.gps_p[a, :2] + data.gps_p[b, :2])
        ax.text(mid[0], mid[1], str(i), fontsize=10)
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_title(title)
    ax.legend()
    ax_text.axis("off")
    ax_text.text(
        0.0,
        0.0,
        global_metric_text(row),
        transform=ax_text.transAxes,
        va="bottom",
        ha="left",
        family="monospace",
        fontsize=10.5,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "white", "edgecolor": "0.75", "alpha": 0.95},
    )
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_errors(data: EvalData, out: Path, title: str) -> None:
    t = data.gps_t - data.gps_t[0]
    e2 = np.linalg.norm(data.errors[:, :2], axis=1)
    e3 = np.linalg.norm(data.errors, axis=1)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, e2, label="2D error")
    axes[0].plot(t, e3, label="3D error", alpha=0.8)
    axes[0].set_ylabel("Error [m]")
    axes[0].legend()
    axes[1].plot(t, data.errors[:, 0], label="East")
    axes[1].plot(t, data.errors[:, 1], label="North")
    axes[1].plot(t, data.errors[:, 2], label="Up")
    axes[1].set_ylabel("Axis error [m]")
    axes[1].legend()
    axes[2].plot(t, np.linalg.norm(np.gradient(data.vio_aligned, data.gps_t, axis=0), axis=1), label="VIO")
    axes[2].plot(t, np.linalg.norm(np.gradient(data.gps_p, data.gps_t, axis=0), axis=1), label="GPS")
    axes[2].set_ylabel("Speed [m/s]")
    axes[2].set_xlabel("Time from eval start [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def plot_edge_groups(
    data: EvalData,
    edges: list[tuple[int, int]],
    edge_rows: list[dict],
    out_dir: Path,
    title_prefix: str,
) -> None:
    if not edges:
        return
    for lap0 in range(0, len(edges), 4):
        group = edges[lap0 : lap0 + 4]
        fig = plt.figure(figsize=(13.5, 3.15 * len(group) + 1.0))
        gs = fig.add_gridspec(
            len(group),
            2,
            width_ratios=[3.7, 1.25],
            wspace=0.18,
            hspace=0.48,
        )
        axes = [fig.add_subplot(gs[i, 0]) for i in range(len(group))]
        text_axes = [fig.add_subplot(gs[i, 1]) for i in range(len(group))]
        for ax, tax, (edge_idx, (a, b)) in zip(axes, text_axes, enumerate(group, lap0 + 1)):
            gps = data.gps_p[a : b + 1]
            vio = data.vio_aligned[a : b + 1]
            d = gps[-1, :2] - gps[0, :2]
            h = math.atan2(d[1], d[0])
            r = rotation_z(-h)
            gps_l = (gps[:, :2] - gps[0, :2]) @ r.T
            vio_l = (vio[:, :2] - vio[0, :2]) @ r.T
            ax.plot(gps_l[:, 0], gps_l[:, 1], "g-", lw=2, label="GPS")
            ax.plot(vio_l[:, 0], vio_l[:, 1], color="tab:blue", lw=1.5, label="VIO")
            ax.scatter([0], [0], c="black", s=22, label="start" if edge_idx == lap0 + 1 else None)
            ax.scatter([gps_l[-1, 0]], [gps_l[-1, 1]], c="green", s=28, marker="o")
            ax.scatter([vio_l[-1, 0]], [vio_l[-1, 1]], c="tab:blue", s=28, marker="x")
            ax.axis("equal")
            ax.grid(True, alpha=0.3)
            ax.set_title(f"edge {edge_idx}")
            ax.set_xlabel("Along GPS edge [m]")
            ax.set_ylabel("Lateral [m]")
            tax.axis("off")
            if edge_idx - 1 < len(edge_rows):
                tax.text(
                    0.0,
                    0.5,
                    edge_metric_text(edge_rows[edge_idx - 1]),
                    transform=tax.transAxes,
                    va="center",
                    ha="left",
                    family="monospace",
                    fontsize=10,
                    bbox={
                        "boxstyle": "round,pad=0.35",
                        "facecolor": "white",
                        "edgecolor": "0.75",
                        "alpha": 0.9,
                    },
                )
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper right")
        lap = lap0 // 4 + 1
        fig.suptitle(f"{title_prefix} edge-local comparison, lap {lap}")
        fig.savefig(out_dir / f"edges_lap{lap}.png", dpi=180)
        plt.close(fig)


def evaluate_one(fly: str, gps_path: Path, traj_path: Path, out_root: Path, args) -> tuple[dict, list[dict]]:
    gps = load_gps(gps_path)
    traj = load_traj(traj_path)
    traj_name = traj_path.stem
    out_dir = out_root / fly / traj_name
    out_dir.mkdir(parents=True, exist_ok=True)
    offset, time_info = estimate_time_offset(gps, traj, args.scan_min, args.scan_max)
    data, align_info = make_eval(gps, traj, offset, args.direction_window_s)
    edges = detect_edges(data, args.min_edge_len_m)
    metrics = whole_metrics(data)
    edge_rows = edge_metrics(data, edges)
    row = {
        "fly": fly,
        "trajectory": traj_name,
        "gps_file": str(gps_path),
        "traj_file": str(traj_path),
        "time_offset_s": offset,
        **time_info,
        **align_info,
        "edges_detected": len(edges),
        **metrics,
    }
    write_csv(out_dir / "matched_samples.csv", [
        {
            "gps_rel_s": float(tg),
            "traj_rel_s": float(tv),
            "gps_x": float(g[0]),
            "gps_y": float(g[1]),
            "gps_z": float(g[2]),
            "vio_x": float(v[0]),
            "vio_y": float(v[1]),
            "vio_z": float(v[2]),
            "err_x": float(e[0]),
            "err_y": float(e[1]),
            "err_z": float(e[2]),
            "err_2d": float(np.linalg.norm(e[:2])),
            "err_3d": float(np.linalg.norm(e)),
        }
        for tg, tv, g, v, e in zip(data.gps_t, data.traj_t, data.gps_p, data.vio_aligned, data.errors)
    ])
    write_csv(out_dir / "edge_metrics.csv", edge_rows)
    with (out_dir / "summary.json").open("w") as f:
        json.dump(row, f, indent=2)
    plot_xy(data, edges, row, out_dir / "trajectory_xy.png", f"{fly} {traj_name}")
    plot_errors(data, out_dir / "errors_timeseries.png", f"{fly} {traj_name}")
    plot_edge_groups(data, edges, edge_rows, out_dir, f"{fly} {traj_name}")
    return row, [{**{"fly": fly, "trajectory": traj_name}, **r} for r in edge_rows]


def discover_trajectories(case: dict) -> list[Path]:
    paths: list[Path] = []
    seen: set[str] = set()

    def add(path: Path) -> None:
        key = str(path.resolve()) if path.exists() else str(path)
        if key not in seen:
            seen.add(key)
            paths.append(path)

    for traj_str in case["trajectories"]:
        add(Path(traj_str))

    root = Path(r"D:\vscode_dir\open_vins") / f"20260509_{case['fly']}" / "result"
    if root.exists():
        for path in sorted(root.glob("pr17*.txt")):
            # Tiny files are usually aborted runs containing only a header or one line.
            if path.stat().st_size >= 1024:
                add(path)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, default=Path("evaluation_start_yaw"))
    parser.add_argument("--scan-min", type=float, default=-300.0)
    parser.add_argument("--scan-max", type=float, default=300.0)
    parser.add_argument("--direction-window-s", type=float, default=30.0)
    parser.add_argument("--min-edge-len-m", type=float, default=45.0)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = []
    all_edge_rows = []
    for case in DEFAULT_CASES:
        fly = case["fly"]
        gps_path = Path(case["gps"])
        for traj_path in discover_trajectories(case):
            if not gps_path.exists() or not traj_path.exists():
                print(f"skip missing: {fly} gps={gps_path.exists()} traj={traj_path.exists()} {traj_path}")
                continue
            print(f"evaluating {fly}: {traj_path.name}")
            try:
                row, edge_rows = evaluate_one(fly, gps_path, traj_path, args.out_dir, args)
            except ValueError as exc:
                print(f"skip invalid: {fly} {traj_path.name}: {exc}")
                continue
            summary_rows.append(row)
            all_edge_rows.extend(edge_rows)
            print(
                f"  offset={row['time_offset_s']:.3f}s yaw={row['yaw_align_deg']:.2f}deg "
                f"ATE2D={row['ate2d_rmse_m']:.2f}m final={row['final_2d_error_m']:.2f}m edges={row['edges_detected']}"
            )
    write_csv(args.out_dir / "summary_metrics.csv", summary_rows)
    write_csv(args.out_dir / "all_edge_metrics.csv", all_edge_rows)
    with (args.out_dir / "run_config.json").open("w") as f:
        json.dump({"cases": DEFAULT_CASES}, f, indent=2)
    print(f"wrote {args.out_dir}")


if __name__ == "__main__":
    main()
