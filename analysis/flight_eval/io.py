"""io.py — 输入路径解析与文件读取.

支持两种路径形式:
  1. 普通本地路径:           D:/path/to/gps.csv
  2. zip 成员路径:            zip://D:/path/pkg.zip::fly4/traj.txt

读取器:
  read_gps_csv     飞控/GPS CSV → DataFrame（含 Ve,Vn,Vu 速度列）
  read_tum         VIO TUM 轨迹（t x y z qx qy qz qw）
  read_bias        OpenVINS .bias（# t_cam vx vy vz bg ba）→ VIO 速度
  read_csv         通用 CSV

设计原则: 输入缺失必须显式报错或返回 unavailable，绝不静默跳过。

TODO·本地核验:
  - 飞控日志真实列名（lat/lon/alt/Ve/Vn/Vu/satellites/ts_ns 的实际 header）。
  - TUM 与 .bias 的真实分隔符 / 注释行格式。
  - zip 内部目录层级。
"""
from __future__ import annotations

import io as _io
import os
import re
import zipfile
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd

ZIP_RE = re.compile(r"^zip://(?P<archive>.+?)::(?P<member>.+)$")


# --------------------------------------------------------------------------- #
# 路径解析
# --------------------------------------------------------------------------- #
@dataclass
class ResolvedPath:
    """解析后的输入路径。state ∈ {ok, unavailable}。"""

    raw: str
    archive: Optional[str]      # zip 路径（非 zip 为 None）
    member: Optional[str]       # zip 内成员
    state: str = "ok"
    reason: str = ""

    @property
    def is_zip(self) -> bool:
        return self.archive is not None

    def to_dict(self) -> dict:
        return {
            "raw": self.raw,
            "archive": self.archive,
            "member": self.member,
            "state": self.state,
            "reason": self.reason,
        }


def resolve(path: Optional[str]) -> ResolvedPath:
    """把一个路径字符串解析为 ResolvedPath，并检查可达性。

    None / "" / "optional" 视为未提供 → state=unavailable。
    """
    if not path or path.strip().lower() in {"optional", "none", "null"}:
        return ResolvedPath(raw=path or "", archive=None, member=None,
                            state="unavailable", reason="not provided")

    m = ZIP_RE.match(path.strip())
    if m:
        archive, member = m.group("archive"), m.group("member")
        rp = ResolvedPath(raw=path, archive=archive, member=member)
        if not os.path.isfile(archive):
            rp.state, rp.reason = "unavailable", f"zip not found: {archive}"
            return rp
        try:
            with zipfile.ZipFile(archive) as zf:
                names = set(zf.namelist())
            if member not in names:
                rp.state, rp.reason = "unavailable", f"member not in zip: {member}"
        except zipfile.BadZipFile:
            rp.state, rp.reason = "unavailable", "bad zip file"
        return rp

    rp = ResolvedPath(raw=path, archive=None, member=path)
    if not os.path.isfile(path):
        rp.state, rp.reason = "unavailable", f"file not found: {path}"
    return rp


def open_bytes(rp: ResolvedPath) -> _io.BytesIO:
    """打开 ResolvedPath 为二进制流（普通文件或 zip 成员）。"""
    if rp.state != "ok":
        raise FileNotFoundError(f"{rp.raw} unavailable: {rp.reason}")
    if rp.is_zip:
        with zipfile.ZipFile(rp.archive) as zf:
            return _io.BytesIO(zf.read(rp.member))
    return _io.BytesIO(open(rp.member, "rb").read())


def sha256(rp: ResolvedPath) -> Optional[str]:
    """输入文件 sha256（供 provenance）。"""
    import hashlib

    if rp.state != "ok":
        return None
    h = hashlib.sha256()
    h.update(open_bytes(rp).read())
    return h.hexdigest()


# --------------------------------------------------------------------------- #
# 读取器
# --------------------------------------------------------------------------- #
# 飞控/GPS 列名候选（TODO·本地核验真实 header，必要时在此扩充映射）
_GPS_ALIASES = {
    "ts_ns": ["ts_ns", "timestamp_ns", "t_ns"],
    "t": ["t", "time", "time_s", "sec"],
    "lat": ["lat", "latitude"],
    "lon": ["lon", "lng", "longitude"],
    "alt": ["alt", "altitude", "height"],
    "Ve": ["Ve", "vel_e", "v_e", "ve", "velE"],
    "Vn": ["Vn", "vel_n", "v_n", "vn", "velN"],
    "Vu": ["Vu", "vel_u", "v_u", "vu", "velU"],
    "satellites": ["satellites", "sats", "num_sat", "nsats"],
}


def _pick(df: pd.DataFrame, names: list[str]) -> Optional[str]:
    lower = {c.lower(): c for c in df.columns}
    for n in names:
        if n.lower() in lower:
            return lower[n.lower()]
    return None


def read_gps_csv(rp: ResolvedPath) -> pd.DataFrame:
    """读取飞控/GPS CSV。

    标准化输出列: t, lat, lon, alt, Ve, Vn, Vu, satellites
    时间: 若有 ts_ns 则换算为秒；否则用 t。
    速度列缺失 → 仅返回位置列，调用方据此决定 fallback。
    """
    df = pd.read_csv(open_bytes(rp))
    out = pd.DataFrame()
    cols = {k: _pick(df, v) for k, v in _GPS_ALIASES.items()}

    if cols["ts_ns"] is not None:
        out["t"] = df[cols["ts_ns"]].astype(float) * 1e-9
    elif cols["t"] is not None:
        out["t"] = df[cols["t"]].astype(float)
    else:
        raise ValueError("GPS CSV 缺少时间列（ts_ns 或 t）。TODO·本地核验列名。")

    for key in ("lat", "lon", "alt", "Ve", "Vn", "Vu", "satellites"):
        if cols[key] is not None:
            out[key] = df[cols[key]].astype(float)

    out.attrs["has_velocity"] = all(c in out.columns for c in ("Ve", "Vn", "Vu"))
    return out.sort_values("t").reset_index(drop=True)


def read_tum(rp: ResolvedPath) -> pd.DataFrame:
    """读取 TUM 轨迹: t x y z qx qy qz qw（空白分隔，# 注释）。"""
    arr = np.loadtxt(open_bytes(rp), comments="#")
    if arr.ndim == 1:
        arr = arr[None, :]
    if arr.shape[1] < 8:
        raise ValueError(f"TUM 列数={arr.shape[1]}<8。TODO·本地核验格式。")
    return pd.DataFrame(
        arr[:, :8], columns=["t", "x", "y", "z", "qx", "qy", "qz", "qw"]
    ).sort_values("t").reset_index(drop=True)


def read_bias(rp: ResolvedPath) -> Optional[pd.DataFrame]:
    """读取 OpenVINS .bias: # t_cam vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z.

    返回含 vx,vy,vz 的 DataFrame；若无速度列返回 None。
    .bias 通常包含 VIO 状态速度（不只是 bias）—— 优先用作 VIO 速度。
    """
    if rp.state != "ok":
        return None
    arr = np.loadtxt(open_bytes(rp), comments="#")
    if arr.ndim == 1:
        arr = arr[None, :]
    if arr.shape[1] < 4:
        return None
    cols = ["t", "vx", "vy", "vz"] + [f"c{i}" for i in range(arr.shape[1] - 4)]
    df = pd.DataFrame(arr, columns=cols)
    return df[["t", "vx", "vy", "vz"]].sort_values("t").reset_index(drop=True)


def read_csv(rp: ResolvedPath) -> pd.DataFrame:
    return pd.read_csv(open_bytes(rp))
