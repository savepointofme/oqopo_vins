"""metrics.py — 误差定义唯一真源 + 统计.

对每个 GPS 对齐样本计算 ENU / 速度 / 航向坐标系误差。
全部误差只在此处定义；其它模块/报告/前端不得重新定义。

航向坐标系（按 GPS course，数学约定 atan2(vN,vE)）:
  e_along = [cos(course), sin(course)]
  e_cross = [-sin(course), cos(course)]
"""
from __future__ import annotations

import numpy as np
import pandas as pd


def build_sample_table(t, cum_dist,
                       gps_EN, gps_U, gps_vEN, gps_vU,
                       vio_EN, vio_U, vio_vEN, vio_vU,
                       gps_course_rad, vio_course_rad,
                       valid, gap,
                       lk_EN=None, lk_U=None) -> pd.DataFrame:
    """组装 gps_time_aligned_samples 主表（含全部误差列）。

    所有 *_EN 为 (N,2) 的 [E,N]；course 为弧度（数学约定）。
    """
    df = pd.DataFrame()
    df["t"] = t
    df["cum_dist"] = cum_dist
    # GPS
    df["gps_E"], df["gps_N"], df["gps_U"] = gps_EN[:, 0], gps_EN[:, 1], gps_U
    df["gps_vE"], df["gps_vN"], df["gps_vU"] = gps_vEN[:, 0], gps_vEN[:, 1], gps_vU
    df["gps_speed_xy"] = np.linalg.norm(gps_vEN, axis=1)
    df["gps_speed_3d"] = np.sqrt(df["gps_speed_xy"] ** 2 + gps_vU ** 2)
    df["gps_course_deg"] = np.degrees(gps_course_rad)
    # VIO
    df["vio_E"], df["vio_N"], df["vio_U"] = vio_EN[:, 0], vio_EN[:, 1], vio_U
    df["vio_vE"], df["vio_vN"], df["vio_vU"] = vio_vEN[:, 0], vio_vEN[:, 1], vio_vU
    df["vio_speed_xy"] = np.linalg.norm(vio_vEN, axis=1)
    df["vio_speed_3d"] = np.sqrt(df["vio_speed_xy"] ** 2 + vio_vU ** 2)
    df["vio_course_deg"] = np.degrees(vio_course_rad)

    # ---- 位置误差 (vio - gps) ----
    eE = vio_EN[:, 0] - gps_EN[:, 0]
    eN = vio_EN[:, 1] - gps_EN[:, 1]
    eU = vio_U - gps_U
    df["err_E"], df["err_N"], df["err_U"] = eE, eN, eU
    df["err_XY"] = np.hypot(eE, eN)
    df["err_3D"] = np.sqrt(eE ** 2 + eN ** 2 + eU ** 2)

    # ---- 航向坐标系投影 ----
    ca, sa = np.cos(gps_course_rad), np.sin(gps_course_rad)
    df["err_along"] = eE * ca + eN * sa
    df["err_cross"] = -eE * sa + eN * ca
    df["err_vertical"] = eU
    df["route_projection_identity_error_m2"] = np.abs(
        df["err_along"] ** 2 + df["err_cross"] ** 2 - (eE ** 2 + eN ** 2)
    )

    # ---- 速度误差 ----
    evE = vio_vEN[:, 0] - gps_vEN[:, 0]
    evN = vio_vEN[:, 1] - gps_vEN[:, 1]
    evU = vio_vU - gps_vU
    df["err_vE"], df["err_vN"], df["err_vU"] = evE, evN, evU
    df["err_speed_xy"] = df["vio_speed_xy"] - df["gps_speed_xy"]
    df["err_vXY_vec"] = np.hypot(evE, evN)
    df["err_v_along"] = evE * ca + evN * sa
    df["err_v_cross"] = -evE * sa + evN * ca
    df["err_v_vertical"] = evU
    df["route_velocity_projection_identity_error_m2ps2"] = np.abs(
        df["err_v_along"] ** 2 + df["err_v_cross"] ** 2 - (evE ** 2 + evN ** 2)
    )

    # Both route components are an orthonormal projection in XY. Height U/Z
    # is deliberately excluded from these identities.
    pos_tol = 1e-8 * max(1.0, float(np.nanmax(eE ** 2 + eN ** 2)))
    vel_tol = 1e-8 * max(1.0, float(np.nanmax(evE ** 2 + evN ** 2)))
    if float(df["route_projection_identity_error_m2"].max()) > pos_tol:
        raise ValueError("XY沿航向/垂直航线位置投影校验失败")
    if float(df["route_velocity_projection_identity_error_m2ps2"].max()) > vel_tol:
        raise ValueError("XY沿航向/垂直航线速度投影校验失败")

    # ---- 航向误差（wrap 到 ±180） ----
    dc = np.degrees(vio_course_rad - gps_course_rad)
    df["course_err_deg"] = (dc + 180) % 360 - 180

    # ---- LK-only（可选） ----
    if lk_EN is not None:
        df["lk_E"], df["lk_N"], df["lk_U"] = lk_EN[:, 0], lk_EN[:, 1], lk_U
        df["err_lk_E"] = lk_EN[:, 0] - gps_EN[:, 0]
        df["err_lk_N"] = lk_EN[:, 1] - gps_EN[:, 1]
        df["err_lk_XY"] = np.hypot(df["err_lk_E"], df["err_lk_N"])

    df["valid"] = valid
    df["gap"] = gap
    return df


# --------------------------------------------------------------------------- #
# 统计
# --------------------------------------------------------------------------- #
def stats(arr: np.ndarray) -> dict:
    """对一个误差序列给出 signed mean / abs mean / RMSE / median / p95 / max / final / n。"""
    a = np.asarray(arr, float)
    a = a[~np.isnan(a)]
    if len(a) == 0:
        return {k: None for k in ("mean", "abs_mean", "rmse", "median", "p95", "max", "final", "n")}
    return {
        "mean": float(np.mean(a)),
        "abs_mean": float(np.mean(np.abs(a))),
        "rmse": float(np.sqrt(np.mean(a ** 2))),
        "median": float(np.median(a)),
        "p95": float(np.percentile(np.abs(a), 95)),
        "max": float(np.max(np.abs(a))),
        "final": float(a[-1]),
        "n": int(len(a)),
    }


def global_summary(df: pd.DataFrame) -> dict:
    """整段指标 → global_summary。只用 valid 样本统计。"""
    v = df[df["valid"]].copy()
    if len(v) == 0:
        raise ValueError("没有有效样本，无法生成 global_summary。")
    dist = float(v["cum_dist"].iloc[-1] - v["cum_dist"].iloc[0])
    last = v.iloc[-1]
    xy_rmse = float(np.sqrt(np.mean(v["err_XY"] ** 2)))

    def rmse(col):
        return float(np.sqrt(np.mean(v[col] ** 2)))

    s = {
        "duration_s": round(float(v["t"].iloc[-1] - v["t"].iloc[0]), 1),
        "gps_distance_m": round(dist, 1),
        "gps_distance_km": round(dist / 1000, 3),
        "final_xy_error_m": round(float(last["err_XY"]), 2),
        "final_xy_drift_percent": round(float(last["err_XY"]) / dist * 100, 3) if dist else None,
        "xy_rmse_m": round(xy_rmse, 2),
        "xy_rmse_percent_of_distance": round(xy_rmse / dist * 100, 3) if dist else None,
        "final_along_error_m": round(float(last["err_along"]), 2),
        "final_cross_error_m": round(float(last["err_cross"]), 2),
        "final_vertical_error_m": round(float(last["err_vertical"]), 2),
        "speed_rmse_mps": round(rmse("err_speed_xy"), 3),
        "vxy_vec_rmse_mps": round(rmse("err_vXY_vec"), 3),
        "v_along_rmse_mps": round(rmse("err_v_along"), 3),
        "v_cross_rmse_mps": round(rmse("err_v_cross"), 3),
        "v_vertical_rmse_mps": round(rmse("err_v_vertical"), 3),
        "route_projection_identity_max_m2": float(
            v["route_projection_identity_error_m2"].max()
        ),
        "route_velocity_projection_identity_max_m2ps2": float(
            v["route_velocity_projection_identity_error_m2ps2"].max()
        ),
        "yaw_or_course_rmse_deg": round(rmse("course_err_deg"), 2),
        "yaw_or_course_final_deg": round(float(last["course_err_deg"]), 2),
    }
    return s
