"""trajectory.py — ENU 转换、start-heading 对齐、累计里程.

坐标系: ENU，GPS 为参考。
主指标使用 start alignment（起点对齐），不使用全轨迹 best-fit。

start-heading 对齐:
  1. 用有效窗口起点对齐平移；
  2. 用前 course_window_s 秒稳定 GPS course 估初始航向差；
  3. 只对 VIO/LK 做初始平移和航向旋转；
  4. 速度只旋转、不平移:  v_aligned = R · v
"""
from __future__ import annotations

import numpy as np
import pandas as pd

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


# --------------------------------------------------------------------------- #
# 经纬高 → ENU
# --------------------------------------------------------------------------- #
def lla_to_enu(lat, lon, alt, lat0, lon0, alt0) -> np.ndarray:
    """以 (lat0,lon0,alt0) 为原点，返回 (N,3) 的 [E,N,U]。"""
    lat = np.radians(np.asarray(lat, float))
    lon = np.radians(np.asarray(lon, float))
    lat0r, lon0r = np.radians(lat0), np.radians(lon0)

    def ecef(la, lo, al):
        s = np.sin(la)
        N = WGS84_A / np.sqrt(1 - WGS84_E2 * s * s)
        x = (N + al) * np.cos(la) * np.cos(lo)
        y = (N + al) * np.cos(la) * np.sin(lo)
        z = (N * (1 - WGS84_E2) + al) * s
        return np.stack([x, y, z], axis=-1)

    p = ecef(lat, lon, np.asarray(alt, float))
    p0 = ecef(lat0r, lon0r, alt0)
    d = p - p0
    slat, clat = np.sin(lat0r), np.cos(lat0r)
    slon, clon = np.sin(lon0r), np.cos(lon0r)
    # ECEF→ENU 旋转矩阵
    R = np.array([
        [-slon, clon, 0.0],
        [-slat * clon, -slat * slon, clat],
        [clat * clon, clat * slon, slat],
    ])
    return d @ R.T


# --------------------------------------------------------------------------- #
# 航向与旋转
# --------------------------------------------------------------------------- #
def course_deg(vE, vN) -> np.ndarray:
    """ENU 速度 → 航向角（deg, atan2(vE, vN) 北零顺时针为正约定）。

    这里采用数学约定 atan2(vN, vE)（东为 0、逆时针）以与误差投影一致；
    报告中标注清楚即可。
    """
    return np.degrees(np.arctan2(np.asarray(vN), np.asarray(vE)))


def rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def estimate_initial_heading(t, vE, vN, t0, window_s) -> float:
    """用 t0..t0+window 内的速度估初始航向（弧度，数学约定）。"""
    mask = (t >= t0) & (t <= t0 + window_s)
    if mask.sum() < 2:
        mask = slice(0, min(len(t), 50))
    ve, vn = np.mean(np.asarray(vE)[mask]), np.mean(np.asarray(vN)[mask])
    return float(np.arctan2(vn, ve))


def start_align(gps_EN, est_EN, gps_vEN, est_vEN, heading_gps, heading_est):
    """对 est（VIO/LK）做起点平移 + 航向旋转。

    平移: est 起点 → gps 起点。
    旋转: R = rot(heading_gps - heading_est)。
    位置:  p' = R (p - p_est0) + p_gps0
    速度:  v' = R v   （仅旋转，无平移）
    返回 (est_EN_aligned, est_vEN_aligned, R, dtheta)。
    """
    dtheta = heading_gps - heading_est
    R = rot2(dtheta)
    p0_est, p0_gps = est_EN[0], gps_EN[0]
    aligned = (R @ (est_EN - p0_est).T).T + p0_gps
    v_aligned = (R @ est_vEN.T).T
    return aligned, v_aligned, R, dtheta


def cumulative_distance(EN: np.ndarray) -> np.ndarray:
    """累计水平里程（m）。"""
    d = np.zeros(len(EN))
    if len(EN) > 1:
        step = np.linalg.norm(np.diff(EN, axis=0), axis=1)
        d[1:] = np.cumsum(step)
    return d


def velocity_from_position(t: np.ndarray, EN: np.ndarray) -> np.ndarray:
    """位置差分得速度（fallback）。中心差分并处理重复时间戳。

    正式 GPS CSV 通常已在读取时去重；这里仍对重复时间戳做防御性
    聚合，避免任何调用者把 ``dt=0`` 交给 ``np.gradient`` 后污染整条
    航向对齐链路。
    """
    t = np.asarray(t, dtype=float)
    position = np.asarray(EN, dtype=float)
    if t.ndim != 1 or position.shape[0] != t.size:
        raise ValueError("time and position lengths do not match")
    if t.size < 2:
        return np.full_like(position, np.nan, dtype=float)

    order = np.argsort(t, kind="stable")
    t_sorted = t[order]
    p_sorted = position[order]
    unique_t, inverse, counts = np.unique(
        t_sorted, return_inverse=True, return_counts=True)
    if unique_t.size < 2:
        return np.full_like(position, np.nan, dtype=float)

    flat = p_sorted.reshape(p_sorted.shape[0], -1)
    unique_flat = np.zeros((unique_t.size, flat.shape[1]), dtype=float)
    np.add.at(unique_flat, inverse, flat)
    unique_flat /= counts[:, None]
    edge_order = 2 if unique_t.size >= 3 else 1
    unique_velocity = np.gradient(
        unique_flat, unique_t, axis=0, edge_order=edge_order)
    sorted_velocity = unique_velocity[inverse].reshape(p_sorted.shape)
    velocity = np.empty_like(sorted_velocity)
    velocity[order] = sorted_velocity
    return velocity
