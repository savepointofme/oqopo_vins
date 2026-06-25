"""Input path resolution and file readers for the flight evaluation tool."""
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


@dataclass
class ResolvedPath:
    raw: str
    archive: Optional[str]
    member: Optional[str]
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
    if not path or path.strip().lower() in {"optional", "none", "null"}:
        return ResolvedPath(
            raw=path or "",
            archive=None,
            member=None,
            state="unavailable",
            reason="not provided",
        )

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
    if rp.state != "ok":
        raise FileNotFoundError(f"{rp.raw} unavailable: {rp.reason}")
    if rp.is_zip:
        with zipfile.ZipFile(rp.archive) as zf:
            return _io.BytesIO(zf.read(rp.member))
    return _io.BytesIO(open(rp.member, "rb").read())


def sha256(rp: ResolvedPath) -> Optional[str]:
    import hashlib

    if rp.state != "ok":
        return None
    h = hashlib.sha256()
    h.update(open_bytes(rp).read())
    return h.hexdigest()


_GPS_ALIASES = {
    "ts_ns": ["ts_ns", "timestamp_ns", "t_ns", "timestamp"],
    "t": ["t", "time", "time_s", "sec", "time_cam", "cam_time"],
    "lat": ["lat", "latitude", "lat_deg"],
    "lon": ["lon", "lng", "longitude", "lon_deg"],
    "alt": ["alt", "altitude", "height", "alt_m", "amsl", "alt_ellipsoid"],
    "Ve": ["Ve", "vel_e", "v_e", "ve", "velE", "vel_e_m_s", "vn_e"],
    "Vn": ["Vn", "vel_n", "v_n", "vn", "velN", "vel_n_m_s", "vn_n"],
    "Vu": ["Vu", "vel_u", "v_u", "vu", "velU", "vel_u_m_s"],
    "Vd": ["Vd", "vel_d", "v_d", "vd", "velD", "vel_d_m_s", "vel_down_m_s"],
    "satellites": ["satellites", "sats", "num_sat", "nsats", "satellites_used"],
}


def _pick(df: pd.DataFrame, names: list[str]) -> Optional[str]:
    lower = {c.lower(): c for c in df.columns}
    for name in names:
        if name.lower() in lower:
            return lower[name.lower()]
    return None


def read_gps_csv(rp: ResolvedPath) -> pd.DataFrame:
    df = pd.read_csv(open_bytes(rp))
    out = pd.DataFrame()
    cols = {key: _pick(df, names) for key, names in _GPS_ALIASES.items()}

    if cols["ts_ns"] is not None:
        out["t"] = df[cols["ts_ns"]].astype(float) * 1e-9
    elif cols["t"] is not None:
        out["t"] = df[cols["t"]].astype(float)
    else:
        raise ValueError("GPS CSV missing timestamp column: ts_ns or t")

    for key in ("lat", "lon", "alt", "Ve", "Vn", "Vu", "Vd", "satellites"):
        if cols[key] is not None:
            out[key] = df[cols[key]].astype(float)

    if "Vu" not in out.columns and "Vd" in out.columns:
        out["Vu"] = -out["Vd"]
    out.drop(columns=[c for c in ("Vd",) if c in out.columns], inplace=True)

    out.attrs["has_velocity"] = all(c in out.columns for c in ("Ve", "Vn", "Vu"))
    return out.sort_values("t").reset_index(drop=True)


def read_tum(rp: ResolvedPath) -> pd.DataFrame:
    """Read TUM trajectory rows.

    The evaluator needs t x y z. Quaternion values are preserved when present;
    short rows with missing quaternion values are accepted and filled with NaN.
    """
    rows = []
    text = open_bytes(rp).read().decode("utf-8", errors="replace")
    for lineno, line in enumerate(text.splitlines(), start=1):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        parts = s.split()
        if len(parts) < 4:
            raise ValueError(f"TUM line {lineno} has {len(parts)} columns; need at least t x y z")
        vals = [float(x) for x in parts[:4]]
        if len(parts) >= 8:
            vals.extend(float(x) for x in parts[4:8])
        else:
            vals.extend([np.nan, np.nan, np.nan, np.nan])
        rows.append(vals)
    if not rows:
        raise ValueError("TUM trajectory has no data rows")
    arr = np.asarray(rows, dtype=float)
    return pd.DataFrame(
        arr[:, :8], columns=["t", "x", "y", "z", "qx", "qy", "qz", "qw"]
    ).sort_values("t").reset_index(drop=True)


def read_bias(rp: ResolvedPath) -> Optional[pd.DataFrame]:
    """Read OpenVINS bias/state rows and return VIO velocity columns if present."""
    if rp.state != "ok":
        return None
    names = ["t", "vx", "vy", "vz", "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"]
    rows = []
    text = open_bytes(rp).read().decode("utf-8", errors="replace")
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        parts = s.split()
        if len(parts) < 4:
            continue
        vals = [float(x) for x in parts[:len(names)]]
        if len(vals) < len(names):
            vals.extend([np.nan] * (len(names) - len(vals)))
        rows.append(vals)
    if not rows:
        return None
    arr = np.asarray(rows, dtype=float)
    return pd.DataFrame(arr[:, :len(names)], columns=names).sort_values("t").reset_index(drop=True)


def read_csv(rp: ResolvedPath) -> pd.DataFrame:
    return pd.read_csv(open_bytes(rp))
