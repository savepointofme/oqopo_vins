"""fc_gps.py — 从飞控原始日志构建标准 GPS CSV.

输出标准列: ts_ns, lat, lon, alt, Ve, Vn, Vu, satellites
关键: 速度取飞控原始 Ve/Vn/Vu（NED 的 Vd 转 Vu=-Vd），不用位置差分伪造。

支持输入:
  1. CSV/TSV          —— 已是表格，按别名映射归一化（无需额外依赖）。
  2. PX4 ULog (.ulg)  —— 经 pyulog 读取 vehicle_gps_position。
  3. ArduPilot (.bin) —— 经 pymavlink 读取 GPS 消息。
缺少速度列时报错而非伪造（符合速度来源规则）。
"""
from __future__ import annotations

import os

import numpy as np
import pandas as pd

OUT_COLS = ["ts_ns", "lat", "lon", "alt", "Ve", "Vn", "Vu", "satellites"]


def build_fc_gps(fc_log: str, out_csv: str) -> pd.DataFrame:
    """主入口: 按扩展名分派解析器，写出标准 gps.csv。"""
    ext = os.path.splitext(fc_log)[1].lower()
    if ext in (".csv", ".tsv", ".txt"):
        df = _from_table(fc_log, sep="\t" if ext == ".tsv" else None)
    elif ext == ".ulg":
        df = _from_px4_ulog(fc_log)
    elif ext in (".bin", ".log"):
        df = _from_ardupilot_bin(fc_log)
    else:
        raise ValueError(f"未知飞控日志类型: {ext}（支持 .csv/.tsv/.ulg/.bin）")

    _validate(df)
    df = df[OUT_COLS].sort_values("ts_ns").reset_index(drop=True)
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    df.to_csv(out_csv, index=False)
    return df


def _validate(df: pd.DataFrame) -> None:
    for c in ("ts_ns", "lat", "lon", "Ve", "Vn", "Vu"):
        if c not in df.columns:
            raise ValueError(
                f"飞控日志缺少必要列 {c}。速度必须来自飞控原始 Ve/Vn/Vu，"
                "不允许位置差分伪造。")
    if "alt" not in df.columns:
        df["alt"] = np.nan
    if "satellites" not in df.columns:
        df["satellites"] = np.nan


# --------------------------------------------------------------------------- #
# 1) CSV/TSV
# --------------------------------------------------------------------------- #
def _from_table(path: str, sep=None) -> pd.DataFrame:
    raw = pd.read_csv(path, sep=sep, engine="python")
    low = {c.lower(): c for c in raw.columns}

    def pick(*names):
        for n in names:
            if n.lower() in low:
                return low[n.lower()]
        return None

    out = pd.DataFrame()
    tcol = pick("ts_ns", "timestamp_ns", "t_ns", "timestamp")
    if tcol is not None:
        ts = raw[tcol].astype(float)
        # 若数值过小（秒级），换算为 ns
        out["ts_ns"] = (ts * 1e9 if ts.max() < 1e12 else ts).astype("int64")
    else:
        sec = pick("t", "time", "time_s", "sec", "cam_time", "time_cam")
        if sec is None:
            raise ValueError("CSV 缺少时间列（ts_ns 或 t）")
        out["ts_ns"] = (raw[sec].astype(float) * 1e9).astype("int64")

    out["lat"] = raw[pick("lat", "latitude", "lat_deg")].astype(float)
    out["lon"] = raw[pick("lon", "lng", "longitude", "lon_deg")].astype(float)
    acol = pick("alt", "altitude", "height", "alt_m", "amsl")
    if acol is not None:
        out["alt"] = raw[acol].astype(float)

    ve = pick("Ve", "vel_e", "vel_e_m_s", "ve")
    vn = pick("Vn", "vel_n", "vel_n_m_s", "vn")
    vu = pick("Vu", "vel_u", "vel_u_m_s", "vu")
    vd = pick("Vd", "vel_d", "vel_d_m_s", "vd", "vel_down_m_s")
    if ve and vn:
        out["Ve"] = raw[ve].astype(float)
        out["Vn"] = raw[vn].astype(float)
        if vu:
            out["Vu"] = raw[vu].astype(float)
        elif vd:
            out["Vu"] = -raw[vd].astype(float)   # NED 下向 → 上向
    scol = pick("satellites", "sats", "num_sat", "nsats", "satellites_used")
    if scol is not None:
        out["satellites"] = raw[scol].astype(float)
    return out


# --------------------------------------------------------------------------- #
# 2) PX4 ULog
# --------------------------------------------------------------------------- #
def _from_px4_ulog(path: str) -> pd.DataFrame:
    try:
        from pyulog import ULog
    except ImportError as e:  # noqa: F841
        raise RuntimeError("解析 .ulg 需要 pyulog：pip install pyulog")
    ulog = ULog(path, ["vehicle_gps_position"])
    if not ulog.data_list:
        raise ValueError("ULog 中未找到 vehicle_gps_position")
    d = ulog.data_list[0].data
    out = pd.DataFrame()
    out["ts_ns"] = (np.asarray(d["timestamp"], dtype="float64") * 1e3).astype("int64")  # us→ns
    out["lat"] = np.asarray(d["lat"]) * 1e-7      # PX4: 1e-7 deg
    out["lon"] = np.asarray(d["lon"]) * 1e-7
    out["alt"] = np.asarray(d["alt"]) * 1e-3      # mm→m
    out["Ve"] = np.asarray(d["vel_e_m_s"])
    out["Vn"] = np.asarray(d["vel_n_m_s"])
    out["Vu"] = -np.asarray(d["vel_d_m_s"])       # NED 下向 → 上向
    if "satellites_used" in d:
        out["satellites"] = np.asarray(d["satellites_used"])
    return out


# --------------------------------------------------------------------------- #
# 3) ArduPilot .bin
# --------------------------------------------------------------------------- #
def _from_ardupilot_bin(path: str) -> pd.DataFrame:
    try:
        from pymavlink import mavutil
    except ImportError:
        raise RuntimeError("解析 .bin 需要 pymavlink：pip install pymavlink")
    mlog = mavutil.mavlink_connection(path)
    rows = []
    while True:
        m = mlog.recv_match(type="GPS")
        if m is None:
            break
        d = m.to_dict()
        # ArduPilot GPS: Spd(地速 m/s), GCrs(航向 deg), VZ(下向 m/s), Lat/Lng(deg), Alt(m)
        spd = d.get("Spd", np.nan)
        crs = np.radians(d.get("GCrs", np.nan))
        ve = spd * np.sin(crs)
        vn = spd * np.cos(crs)
        vz = d.get("VZ", np.nan)
        rows.append({
            "ts_ns": int(d.get("TimeUS", 0)) * 1000,   # us→ns
            "lat": d.get("Lat"), "lon": d.get("Lng"), "alt": d.get("Alt"),
            "Ve": ve, "Vn": vn, "Vu": -vz if vz == vz else np.nan,
            "satellites": d.get("NSats"),
        })
    if not rows:
        raise ValueError(".bin 中未找到 GPS 消息")
    return pd.DataFrame(rows)
