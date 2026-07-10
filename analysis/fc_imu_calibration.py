#!/usr/bin/env python3
"""Calibrate residual FC-to-OpenVINS-IMU time offset and fixed SO(3) rotation.

This script uses relative rotations only. It never reads dashboard alignment or
official trajectory alignment columns for calibration.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np


REL_INTERVALS_DEFAULT = (0.10, 0.20, 0.50, 1.00)
NOMINAL_AXIS_TRANSFORM = np.array(
    [[0.0, 1.0, 0.0],
     [1.0, 0.0, 0.0],
     [0.0, 0.0, -1.0]],
    dtype=float,
)


@dataclass
class RunData:
    flight: str
    run_path: Path
    dataset_path: Path | None = None
    fc_path: Path | None = None
    traj_path: Path | None = None
    start_time: float | None = None
    fc_offset_s: float | None = None
    fc_time_mode: str = ""
    vio_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    vio_q: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    fc_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    fc_q: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    fc_dup_count: int = 0
    fc_nonmono_count: int = 0
    rejected: bool = False
    reject_reason: str = ""


@dataclass
class RelSample:
    flight: str
    t0: float
    interval: float
    A: np.ndarray
    B: np.ndarray
    angle_deg: float
    yaw_rate_deg_s: float
    angle_rate_deg_s: float


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=float)


def exp_so3(w: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(w))
    W = skew(w)
    if theta < 1e-12:
        return np.eye(3) + W
    return np.eye(3) + math.sin(theta) / theta * W + (1.0 - math.cos(theta)) / (theta * theta) * (W @ W)


def log_so3(R: np.ndarray) -> np.ndarray:
    R = project_so3(R)
    tr = float(np.trace(R))
    cos_theta = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    theta = math.acos(cos_theta)
    if theta < 1e-10:
        return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if abs(math.pi - theta) < 1e-5:
        axis = np.empty(3)
        axis[0] = math.sqrt(max(0.0, (R[0, 0] + 1.0) / 2.0))
        axis[1] = math.sqrt(max(0.0, (R[1, 1] + 1.0) / 2.0))
        axis[2] = math.sqrt(max(0.0, (R[2, 2] + 1.0) / 2.0))
        if R[0, 1] < 0:
            axis[1] = -axis[1]
        if R[0, 2] < 0:
            axis[2] = -axis[2]
        n = np.linalg.norm(axis)
        return theta * axis / n if n > 1e-12 else np.array([theta, 0, 0], dtype=float)
    return theta / (2.0 * math.sin(theta)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def project_so3(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    S = np.eye(3)
    S[2, 2] = np.linalg.det(U @ Vt)
    return U @ S @ Vt


def angle_deg(R: np.ndarray) -> float:
    return math.degrees(float(np.linalg.norm(log_so3(R))))


def rot_to_rpy_deg(R: np.ndarray) -> np.ndarray:
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.degrees([roll, pitch, yaw])


def hamilton_quat_to_rot(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def jpl_quat_to_rot(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    qv = q[:3]
    qw = q[3]
    return (2.0 * qw * qw - 1.0) * np.eye(3) - 2.0 * qw * skew(qv) + 2.0 * np.outer(qv, qv)


def rot_to_jpl_quat(R: np.ndarray) -> np.ndarray:
    R = project_so3(R)
    q = np.zeros(4)
    tr = float(np.trace(R))
    if R[0, 0] >= tr and R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        q[0] = math.sqrt(max(0.0, 1.0 + 2.0 * R[0, 0] - tr) / 4.0)
        q[1] = (R[0, 1] + R[1, 0]) / (4.0 * q[0])
        q[2] = (R[0, 2] + R[2, 0]) / (4.0 * q[0])
        q[3] = (R[1, 2] - R[2, 1]) / (4.0 * q[0])
    elif R[1, 1] >= tr and R[1, 1] >= R[2, 2]:
        q[1] = math.sqrt(max(0.0, 1.0 + 2.0 * R[1, 1] - tr) / 4.0)
        q[0] = (R[0, 1] + R[1, 0]) / (4.0 * q[1])
        q[2] = (R[1, 2] + R[2, 1]) / (4.0 * q[1])
        q[3] = (R[2, 0] - R[0, 2]) / (4.0 * q[1])
    elif R[2, 2] >= tr:
        q[2] = math.sqrt(max(0.0, 1.0 + 2.0 * R[2, 2] - tr) / 4.0)
        q[0] = (R[0, 2] + R[2, 0]) / (4.0 * q[2])
        q[1] = (R[1, 2] + R[2, 1]) / (4.0 * q[2])
        q[3] = (R[0, 1] - R[1, 0]) / (4.0 * q[2])
    else:
        q[3] = math.sqrt(max(0.0, 1.0 + tr) / 4.0)
        q[0] = (R[1, 2] - R[2, 1]) / (4.0 * q[3])
        q[1] = (R[2, 0] - R[0, 2]) / (4.0 * q[3])
        q[2] = (R[0, 1] - R[1, 0]) / (4.0 * q[3])
    if q[3] < 0.0:
        q = -q
    return q / np.linalg.norm(q)


def jpl_to_hamilton_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def rot_to_hamilton_xyzw(R: np.ndarray) -> np.ndarray:
    return jpl_to_hamilton_xyzw(rot_to_jpl_quat(R))


def normalize_quats(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).copy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    for i in range(1, len(q)):
        if float(np.dot(q[i - 1], q[i])) < 0.0:
            q[i] *= -1.0
    return q


def slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1.0 - alpha) * theta) / math.sin(theta)) * q0 + (math.sin(alpha * theta) / math.sin(theta)) * q1


def interp_quat(t: np.ndarray, q: np.ndarray, query: float, max_gap: float) -> np.ndarray | None:
    if len(t) == 0 or query < t[0] or query > t[-1]:
        return None
    idx = int(np.searchsorted(t, query, side="left"))
    if idx == 0:
        return q[0] if abs(query - t[0]) <= max_gap else None
    if idx >= len(t):
        return q[-1] if abs(query - t[-1]) <= max_gap else None
    t0, t1 = float(t[idx - 1]), float(t[idx])
    if min(abs(query - t0), abs(query - t1)) > max_gap or t1 <= t0:
        return None
    return slerp(q[idx - 1], q[idx], (query - t0) / (t1 - t0))


def wsl_to_windows_path(text: str) -> str:
    if text.startswith("/mnt/") and len(text) > 6:
        drive = text[5].upper()
        rest = text[7:].replace("/", "\\")
        return f"{drive}:\\{rest}"
    return text


def windows_to_wsl_path(path: Path) -> str:
    s = str(path)
    m = re.match(r"^([A-Za-z]):\\(.*)$", s)
    if not m:
        return s.replace("\\", "/")
    return f"/mnt/{m.group(1).lower()}/{m.group(2).replace(chr(92), '/')}"


def read_metadata(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line and not line.lstrip().startswith("#"):
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def parse_command_flags(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    import shlex
    text = " ".join(x.strip() for x in path.read_text(encoding="utf-8", errors="replace").splitlines())
    try:
        tokens = shlex.split(text)
    except ValueError:
        tokens = text.split()
    out: dict[str, str] = {}
    for i, token in enumerate(tokens):
        if token.startswith("--"):
            out[token[2:]] = tokens[i + 1] if i + 1 < len(tokens) and not tokens[i + 1].startswith("--") else "true"
    return out


def parse_fc_offset_from_text(text: str) -> float | None:
    if not text:
        return None
    m = re.search(r"fc_rel_to_cam_offset\s+(-?\d+(?:\.\d+)?)", text)
    if m:
        return float(m.group(1))
    m = re.search(r"offset(m?)(\d+)p(\d+)", text)
    if m:
        val = float(f"{m.group(2)}.{m.group(3)}")
        return -val if m.group(1) == "m" else val
    return None


def parse_fc_init_timestamp(path: Path) -> float | None:
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                return float(row[0])
            except ValueError:
                continue
    return None


def parse_fc_time_utc(text: str) -> float:
    text = text.strip()
    if not text:
        raise ValueError("empty FC time")
    if "_" in text:
        base, ms = text.rsplit("_", 1)
        micros = int(ms.ljust(3, "0")[:3]) * 1000
    else:
        base, micros = text, 0
    stamp = dt.datetime.strptime(base, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(microsecond=micros, tzinfo=dt.timezone.utc).timestamp()


def iter_clean_csv_rows(path: Path) -> Iterable[list[str]]:
    with path.open("rb") as f:
        for raw in f:
            raw = raw.replace(b"\x00", b"").strip()
            if not raw or raw.startswith(b"#"):
                continue
            yield next(csv.reader([raw.decode("utf-8-sig", "replace")]))


def camera_epoch_unix(dataset_dir: Path) -> float:
    for row in iter_clean_csv_rows(dataset_dir / "imu0" / "data.csv"):
        return float(row[1]) - float(row[0])
    raise RuntimeError(f"no IMU data in {dataset_dir}")


def fc_rotations(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(-yaw_deg)
    R_B_to_NED = rz(y) @ ry(p) @ rx(r)
    R_ENU_to_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    R_G_to_FRD = R_B_to_NED.T @ R_ENU_to_NED
    return NOMINAL_AXIS_TRANSFORM @ R_G_to_FRD


def find_fc_attitude_csv(dataset: Path | None) -> Path | None:
    if dataset is None:
        return None
    roots = [dataset, dataset.parent]
    if dataset.parent.parent not in roots:
        roots.append(dataset.parent.parent)
    seen: set[Path] = set()
    candidates: list[Path] = []
    for root in roots:
        if not root.exists() or root in seen:
            continue
        seen.add(root)
        try:
            candidates.extend(root.glob("*.csv"))
            candidates.extend((root / "result").glob("*.csv"))
        except OSError:
            continue
    for path in candidates:
        try:
            first = path.open("r", encoding="utf-8-sig", errors="replace").readline()
        except OSError:
            continue
        if ("俯仰" in first and "滚转" in first and "偏航" in first) or ("pitch" in first.lower() and "yaw" in first.lower()):
            return path
    return None


def load_fc_raw(path: Path, dataset: Path | None, offset_s: float | None) -> tuple[np.ndarray, np.ndarray, int, int, str]:
    rows: list[tuple[float, float, float, float]] = []
    fc0: float | None = None
    cam_epoch = None if offset_s is not None else camera_epoch_unix(dataset) if dataset else None
    with path.open(newline="", encoding="utf-8-sig", errors="replace") as f:
        reader = csv.reader(f)
        next(reader, None)
        for raw in reader:
            if len(raw) < 11:
                continue
            try:
                unix = parse_fc_time_utc(raw[3])
                if fc0 is None:
                    fc0 = unix
                t = (unix - fc0) - float(offset_s) if offset_s is not None else unix - float(cam_epoch)
                pitch, roll, yaw = float(raw[0]), float(raw[1]), float(raw[2])
                rows.append((t, roll, pitch, yaw))
            except Exception:
                continue
    if not rows:
        raise RuntimeError(f"no valid FC attitude rows parsed from {path}")
    raw_t = np.array([r[0] for r in rows], dtype=float)
    dup_count = int(len(raw_t) - len(np.unique(raw_t)))
    nonmono_count = int(np.sum(np.diff(raw_t) < 0.0))
    order = np.argsort(raw_t, kind="mergesort")
    raw_t = raw_t[order]
    raw_rpy = [rows[i] for i in order]
    keep = np.ones(len(raw_t), dtype=bool)
    keep[1:] = np.diff(raw_t) > 1e-9
    t = raw_t[keep]
    q = []
    for _, roll, pitch, yaw in np.asarray(raw_rpy, dtype=float)[keep]:
        q.append(rot_to_jpl_quat(fc_rotations(float(roll), float(pitch), float(yaw))))
    mode = f"relative_offset {offset_s:.6f}" if offset_s is not None else "absolute_utc_camera_epoch"
    return t, normalize_quats(np.asarray(q)), dup_count, nonmono_count, mode


def read_traj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                rows.append([float(x) for x in parts[:8]])
            except ValueError:
                continue
    if not rows:
        raise RuntimeError(f"empty trajectory {path}")
    arr = np.asarray(rows, dtype=float)
    q = normalize_quats(arr[:, 4:8])
    return arr[:, 0], q


def build_run_from_dir(run_dir: Path) -> RunData:
    meta = read_metadata(run_dir / "run_metadata.txt")
    flags = parse_command_flags(run_dir / "command.sh")
    flight = meta.get("fly") or re.search(r"(fly[1-4])", str(run_dir), re.I).group(1).lower()
    dataset_text = meta.get("dataset") or flags.get("dataset", "")
    dataset = Path(wsl_to_windows_path(dataset_text)) if dataset_text else None
    fc_init_text = meta.get("fc_init") or flags.get("init-from-fc", "")
    fc_init = Path(wsl_to_windows_path(fc_init_text)) if fc_init_text else None
    start_time = float(meta.get("start_time") or flags.get("start-time", "nan"))
    traj = run_dir / "traj_raw.txt"
    if not traj.exists():
        traj = run_dir / "traj.txt"
    data = RunData(flight=flight, run_path=run_dir, dataset_path=dataset, traj_path=traj, start_time=start_time)
    if not traj.exists():
        data.rejected, data.reject_reason = True, "missing traj_raw.txt/traj.txt"
        return data
    if fc_init and fc_init.exists():
        init_t = parse_fc_init_timestamp(fc_init)
        if init_t is not None and math.isfinite(start_time) and abs(init_t - start_time) > 0.25:
            data.rejected, data.reject_reason = True, f"fc_init_time_mismatch init={init_t:.3f} start={start_time:.3f}"
            return data
        text = fc_init.read_text(encoding="utf-8", errors="replace")
        data.fc_offset_s = parse_fc_offset_from_text(text) if parse_fc_offset_from_text(text) is not None else parse_fc_offset_from_text(str(fc_init))
    raw_fc = find_fc_attitude_csv(dataset)
    data.fc_path = raw_fc
    if raw_fc is None:
        data.rejected, data.reject_reason = True, "missing continuous FC attitude CSV"
        return data
    try:
        data.vio_t, data.vio_q = read_traj(traj)
        data.fc_t, data.fc_q, data.fc_dup_count, data.fc_nonmono_count, data.fc_time_mode = load_fc_raw(raw_fc, dataset, data.fc_offset_s)
    except Exception as exc:
        data.rejected, data.reject_reason = True, str(exc)
    return data


def default_candidates(repo: Path) -> list[Path]:
    out: list[Path] = []
    current = repo / "result" / "stride12_robustness_8h" / "current_stride12"
    for fly in ("fly1", "fly2", "fly3", "fly4"):
        matches = sorted(current.glob(f"{fly}_stride12_*"))
        out.extend(matches[-1:])
    desktop = Path(os.environ.get("USERPROFILE", "C:/Users/baloney")) / "Desktop"
    fixed = desktop / "openvins_fc_init_fix_validation_20260708"
    for fly in ("fly3", "fly4"):
        matches = sorted(fixed.glob(f"{fly}_stride12_*"))
        out.extend(matches[-1:])
    return out


def interp_R_vio(run: RunData, t: float, max_gap: float) -> np.ndarray | None:
    q = interp_quat(run.vio_t, run.vio_q, t, max_gap)
    return None if q is None else hamilton_quat_to_rot(q)


def interp_R_fc(run: RunData, t: float, max_gap: float) -> np.ndarray | None:
    q = interp_quat(run.fc_t, run.fc_q, t, max_gap)
    return None if q is None else jpl_quat_to_rot(q).T


def make_samples(runs: list[RunData], delta_t_fc: float, intervals: tuple[float, ...], sample_step_s: float,
                 min_angle_deg: float, max_interp_gap_s: float) -> list[RelSample]:
    samples: list[RelSample] = []
    max_interval = max(intervals)
    for run in runs:
        if run.rejected:
            continue
        start = max(float(run.vio_t[0]), float(run.fc_t[0]) - delta_t_fc)
        end = min(float(run.vio_t[-1]), float(run.fc_t[-1]) - delta_t_fc) - max_interval
        if end <= start:
            continue
        base_times = np.arange(start, end, sample_step_s)
        for interval in intervals:
            for t0 in base_times:
                t1 = float(t0 + interval)
                Rv0 = interp_R_vio(run, float(t0), max_interp_gap_s)
                Rv1 = interp_R_vio(run, t1, max_interp_gap_s)
                Rf0 = interp_R_fc(run, float(t0 + delta_t_fc), max_interp_gap_s)
                Rf1 = interp_R_fc(run, t1 + delta_t_fc, max_interp_gap_s)
                if Rv0 is None or Rv1 is None or Rf0 is None or Rf1 is None:
                    continue
                A = Rv0.T @ Rv1
                B = Rf0.T @ Rf1
                wa = log_so3(A)
                wb = log_so3(B)
                a_deg = math.degrees(float(np.linalg.norm(wa)))
                b_deg = math.degrees(float(np.linalg.norm(wb)))
                if min(a_deg, b_deg) < min_angle_deg:
                    continue
                if abs(a_deg - b_deg) > max(5.0, 0.75 * max(a_deg, b_deg)):
                    continue
                yaw_rate = math.degrees(float(wa[2])) / interval
                samples.append(RelSample(run.flight, float(t0), interval, A, B, 0.5 * (a_deg + b_deg), yaw_rate, a_deg / interval))
    return samples


def solve_rotation(samples: list[RelSample], identity: bool = False, robust_iters: int = 4) -> np.ndarray:
    if identity:
        return np.eye(3)
    if len(samples) < 3:
        return np.eye(3)
    weights = np.ones(len(samples), dtype=float)
    X = np.eye(3)
    for _ in range(max(1, robust_iters)):
        H = np.zeros((3, 3), dtype=float)
        for w, s in zip(weights, samples):
            a = log_so3(s.A)
            b = log_so3(s.B)
            H += w * np.outer(a, b)
        U, _, Vt = np.linalg.svd(H)
        S = np.eye(3)
        S[2, 2] = np.linalg.det(U @ Vt)
        X = project_so3(U @ S @ Vt)
        res = np.asarray([residual_angle_rad(s, X) for s in samples])
        scale = max(np.median(res), math.radians(1.0))
        weights = np.minimum(1.0, (1.5 * scale) / np.maximum(res, 1e-12))
    return X


def residual_angle_rad(s: RelSample, X: np.ndarray) -> float:
    E = (s.A @ X).T @ (X @ s.B)
    return float(np.linalg.norm(log_so3(E)))


def statistics(samples: list[RelSample], X: np.ndarray) -> dict[str, float | int]:
    if not samples:
        return {"count": 0}
    deg = np.degrees([residual_angle_rad(s, X) for s in samples])
    return {
        "count": int(len(deg)),
        "median_deg": float(np.median(deg)),
        "p90_deg": float(np.percentile(deg, 90)),
        "p95_deg": float(np.percentile(deg, 95)),
        "max_deg": float(np.max(deg)),
        "inlier_ratio_5deg": float(np.mean(deg <= 5.0)),
    }


def robust_objective(samples: list[RelSample], X: np.ndarray) -> float:
    if not samples:
        return float("inf")
    r = np.asarray([residual_angle_rad(s, X) for s in samples])
    c = math.radians(5.0)
    loss = np.where(r <= c, 0.5 * r * r, c * (r - 0.5 * c))
    return float(np.mean(loss))


def scan_delta(runs: list[RunData], intervals: tuple[float, ...], sample_step_s: float, min_angle_deg: float,
               max_interp_gap_s: float, optimize_rotation: bool, deltas: np.ndarray) -> tuple[float, np.ndarray, list[dict[str, float]]]:
    rows: list[dict[str, float]] = []
    best_delta = float(deltas[0])
    best_obj = float("inf")
    best_X = np.eye(3)
    for delta in deltas:
        samples = make_samples(runs, float(delta), intervals, sample_step_s, min_angle_deg, max_interp_gap_s)
        X = solve_rotation(samples, identity=not optimize_rotation)
        obj = robust_objective(samples, X)
        st = statistics(samples, X)
        rows.append({
            "delta_t_fc_s": float(delta),
            "objective": obj,
            "sample_count": float(st.get("count", 0)),
            "median_deg": float(st.get("median_deg", float("nan"))),
            "p95_deg": float(st.get("p95_deg", float("nan"))),
        })
        if obj < best_obj:
            best_delta, best_obj, best_X = float(delta), obj, X
    return best_delta, best_X, rows


def calibrate_scheme(runs: list[RunData], scheme: str, intervals: tuple[float, ...], sample_step_s: float,
                     min_angle_deg: float, max_interp_gap_s: float) -> tuple[float, np.ndarray, list[RelSample], list[dict[str, float]]]:
    if scheme == "A":
        delta, X = 0.0, np.eye(3)
        return delta, X, make_samples(runs, delta, intervals, sample_step_s, min_angle_deg, max_interp_gap_s), []
    if scheme == "C":
        delta = 0.0
        samples = make_samples(runs, delta, intervals, sample_step_s, min_angle_deg, max_interp_gap_s)
        return delta, solve_rotation(samples), samples, []
    optimize_rotation = scheme == "D"
    coarse = np.arange(-0.50, 0.5001, 0.005)
    best_delta, _, rows = scan_delta(runs, intervals, sample_step_s, min_angle_deg, max_interp_gap_s, optimize_rotation, coarse)
    fine = np.arange(max(-0.50, best_delta - 0.010), min(0.50, best_delta + 0.010) + 1e-12, 0.001)
    best_delta, X, fine_rows = scan_delta(runs, intervals, sample_step_s, min_angle_deg, max_interp_gap_s, optimize_rotation, fine)
    rows.extend(fine_rows)
    samples = make_samples(runs, best_delta, intervals, sample_step_s, min_angle_deg, max_interp_gap_s)
    if not optimize_rotation:
        X = np.eye(3)
    return best_delta, X, samples, rows


def group_label(s: RelSample) -> list[str]:
    labels = []
    if abs(s.yaw_rate_deg_s) <= 1.5 and s.angle_rate_deg_s < 5.0:
        labels.append("straight")
    if s.yaw_rate_deg_s > 3.0:
        labels.append("left_turn")
    if s.yaw_rate_deg_s < -3.0:
        labels.append("right_turn")
    labels.append("high_angular_rate" if s.angle_rate_deg_s >= 10.0 else "low_angular_rate")
    return labels


def grouped_stats(samples: list[RelSample], X: np.ndarray) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    keys = sorted({s.flight for s in samples})
    keys += ["straight", "left_turn", "right_turn", "low_angular_rate", "high_angular_rate", "all"]
    for key in keys:
        if key == "all":
            subset = samples
        elif key.startswith("fly"):
            subset = [s for s in samples if s.flight == key]
        else:
            subset = [s for s in samples if key in group_label(s)]
        st = statistics(subset, X)
        if st.get("count", 0):
            row = {"group": key, **st}
            rows.append(row)
    return rows


def absolute_attitude_check(runs: list[RunData], scheme: str, delta_t_fc: float, X: np.ndarray,
                            window_s: float, sample_step_s: float, max_interp_gap_s: float) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run in runs:
        if run.rejected:
            continue
        start = max(float(run.vio_t[0]), float(run.fc_t[0]) - delta_t_fc)
        end = min(float(run.vio_t[-1]), float(run.fc_t[-1]) - delta_t_fc)
        window_id = 0
        t0 = start
        while t0 + max(30.0, sample_step_s * 3.0) <= end:
            t1 = min(t0 + window_s, end)
            times = np.arange(t0, t1 + 1e-9, sample_step_s)
            vio_R: list[np.ndarray] = []
            fc_R: list[np.ndarray] = []
            used_t: list[float] = []
            for t in times:
                Rv = interp_R_vio(run, float(t), max_interp_gap_s)
                Rf = interp_R_fc(run, float(t + delta_t_fc), max_interp_gap_s)
                if Rv is None or Rf is None:
                    continue
                vio_R.append(Rv)
                fc_R.append(Rf)
                used_t.append(float(t))
            if len(used_t) >= 4:
                anchor = vio_R[0] @ (fc_R[0] @ X.T).T
                errors = np.asarray([angle_deg((anchor @ Rf @ X.T).T @ Rv) for Rv, Rf in zip(vio_R, fc_R)])
                elapsed = np.asarray(used_t) - used_t[0]
                rates = [0.0]
                for i in range(1, len(vio_R)):
                    dt_s = max(1e-9, used_t[i] - used_t[i - 1])
                    rates.append(angle_deg(vio_R[i - 1].T @ vio_R[i]) / dt_s)
                rate_arr = np.asarray(rates)
                def corr(a: np.ndarray, b: np.ndarray) -> float:
                    if len(a) < 3 or float(np.std(a)) < 1e-12 or float(np.std(b)) < 1e-12:
                        return float("nan")
                    return float(np.corrcoef(a, b)[0, 1])
                rows.append({
                    "row_type": "absolute_attitude_check",
                    "scheme": scheme,
                    "flight": run.flight,
                    "window_id": window_id,
                    "window_t0": used_t[0],
                    "window_t1": used_t[-1],
                    "count": int(len(errors)),
                    "median_deg": float(np.median(errors)),
                    "p95_deg": float(np.percentile(errors, 95)),
                    "max_deg": float(np.max(errors)),
                    "time_error_corr": corr(elapsed, errors),
                    "rate_error_corr": corr(rate_arr, errors),
                })
            window_id += 1
            t0 = t1
    return rows


def excitation_report(samples: list[RelSample]) -> dict[str, float]:
    if not samples:
        return {}
    axes = np.asarray([log_so3(s.A) for s in samples])
    rms = np.degrees(np.sqrt(np.mean(axes * axes, axis=0)))
    return {"roll_rms_deg": float(rms[0]), "pitch_rms_deg": float(rms[1]), "yaw_rms_deg": float(rms[2])}


def load_runs(paths: list[str]) -> list[RunData]:
    runs: list[RunData] = []
    for text in paths:
        p = Path(text)
        runs.append(build_run_from_dir(p))
    return runs


def inspect_runs(runs: list[RunData]) -> list[dict[str, object]]:
    rows = []
    for r in runs:
        if r.rejected:
            rows.append({
                "flight": r.flight,
                "run_path": str(r.run_path),
                "usable": False,
                "rejection_reason": r.reject_reason,
            })
            continue
        vio_dt = np.diff(r.vio_t)
        fc_dt = np.diff(r.fc_t)
        overlap = max(0.0, min(r.vio_t[-1], r.fc_t[-1]) - max(r.vio_t[0], r.fc_t[0]))
        max_gap = float(np.max(fc_dt)) if len(fc_dt) else float("nan")
        rows.append({
            "flight": r.flight,
            "run_path": str(r.run_path),
            "vio_time_range": f"{r.vio_t[0]:.3f}..{r.vio_t[-1]:.3f}",
            "fc_time_range": f"{r.fc_t[0]:.3f}..{r.fc_t[-1]:.3f}",
            "overlap_duration_s": overlap,
            "fc_sample_rate_hz": 1.0 / float(np.median(fc_dt)) if len(fc_dt) else float("nan"),
            "vio_sample_rate_hz": 1.0 / float(np.median(vio_dt)) if len(vio_dt) else float("nan"),
            "duplicate_timestamp_count": r.fc_dup_count,
            "non_monotonic_timestamp_count": r.fc_nonmono_count,
            "maximum_interpolation_gap_s": max_gap,
            "usable": overlap > 60.0,
            "rejection_reason": "" if overlap > 60.0 else "overlap <= 60s",
        })
    return rows


def print_coordinate_audit() -> None:
    print("当前阶段: 坐标和四元数约定审计")
    print("使用的数据: tools/fc_to_init_csv.py, analysis/master_flight_data.py, ov_core/src/utils/quat_ops.h")
    print("发现的证据:")
    print("  FC 原始姿态: NED ZYX Euler, body=FRD; 当前转换使用 yaw_sign=-1。")
    print("  nominal FC->IMU 轴转换: R_I_FRD = [[0,1,0],[1,0,0],[0,0,-1]]，飞机前向对应 IMU +Y。")
    print("  fc_q_*: 已经从 FC 原始姿态转换到 OpenVINS nominal IMU 约定的 JPL q_GtoI，不是原始 FC 四元数。")
    print("  vio_raw_q_* / traj.txt: Hamilton [qx,qy,qz,qw]，由 runner 中 R_ItoW0 写出。")
    print("  相对旋转计算: R_WI=Hamilton(vio_raw_q), R_WF_nom=JPL(fc_q_GtoI)^T; A=R_WI(t)^T R_WI(t+dT), B=R_WF(t+dt)^T R_WF(t+dT+dt)。")
    print("  时间偏移符号: FC attitude used at VIO time t is R_F(t + delta_t_fc)。")


def numeric_direction_tests() -> None:
    X = rz(math.radians(4.0)) @ ry(math.radians(-2.0)) @ rx(math.radians(1.0))
    rel_pairs = [
        (rz(0.2) @ ry(-0.1), rz(0.25) @ ry(-0.07) @ rx(0.03)),
        (ry(-0.3) @ rx(0.1), rz(0.1) @ ry(-0.24) @ rx(0.18)),
        (rz(-0.4) @ rx(0.15), rz(-0.28) @ ry(0.09) @ rx(0.21)),
        (ry(0.2) @ rz(0.3), ry(0.33) @ rz(0.35) @ rx(-0.08)),
    ]
    samples = []
    residuals = []
    for i, (Rf0, Rf1) in enumerate(rel_pairs):
        Ri0 = Rf0 @ X.T
        Ri1 = Rf1 @ X.T
        A = Ri0.T @ Ri1
        B = Rf0.T @ Rf1
        residuals.append(angle_deg((A @ X).T @ (X @ B)))
        samples.append(RelSample("synthetic", float(i), 1.0, A, B, angle_deg(A), 3.0, angle_deg(A)))
    X_hat = solve_rotation(samples)
    print(f"当前数值结果: 合成 AX=XB 方向测试 residual_max={max(residuals):.6g} deg, X recovery error={angle_deg(X_hat.T @ X):.6g} deg")


def real_sample_direction_test(runs: list[RunData], delta_t_fc: float = 0.0) -> None:
    for run in runs:
        if run.rejected:
            continue
        samples = make_samples([run], delta_t_fc, (0.50,), 2.0, 1.0, 0.30)
        if not samples:
            continue
        s = samples[0]
        det_a = float(np.linalg.det(s.A))
        det_b = float(np.linalg.det(s.B))
        orth_a = float(np.linalg.norm(s.A.T @ s.A - np.eye(3)))
        orth_b = float(np.linalg.norm(s.B.T @ s.B - np.eye(3)))
        residual = math.degrees(residual_angle_rad(s, np.eye(3)))
        print(
            "当前数值结果: "
            f"真实样本相对旋转检查 flight={s.flight} t0={s.t0:.6f}s interval={s.interval:.2f}s "
            f"delta_t_check={delta_t_fc:.3f}s det(A)={det_a:.12g} det(B)={det_b:.12g} "
            f"orth(A)={orth_a:.3g} orth(B)={orth_b:.3g} "
            f"angle_A={angle_deg(s.A):.6f}deg angle_B={angle_deg(s.B):.6f}deg "
            f"identity_residual={residual:.6f}deg"
        )
        return
    print("当前数值结果: 真实样本相对旋转检查 unavailable: no usable sample passed filters")


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys: list[str] = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def safe_git_commit() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return ""


def yaml_list(values: Iterable[object], indent: int = 0) -> str:
    pad = " " * indent
    return "\n".join(f"{pad}- {v}" for v in values)


def write_yaml(path: Path, accepted: bool, delta: float, X: np.ndarray, runs: list[RunData],
               train_stats: dict[str, object], val_stats: dict[str, object], intervals: tuple[float, ...],
               sample_filters: dict[str, object], reasons: list[str]) -> None:
    R_full = X @ NOMINAL_AXIS_TRANSFORM
    q_full = rot_to_hamilton_xyzw(R_full)
    q_residual = rot_to_hamilton_xyzw(X)
    rpy_full = rot_to_rpy_deg(R_full)
    rpy_residual = rot_to_rpy_deg(X)
    lines = [
        "version: fc_imu_calibration_v1",
        f"created_from_commit: {safe_git_commit()}",
        "frame_from: F_raw_FC_FRD",
        "frame_to: I_OpenVINS_IMU",
        "quaternion_order: qx_qy_qz_qw",
        "rotation_direction: R_I_F maps raw FC FRD body-frame vectors into OpenVINS IMU-frame vectors",
        "R_I_F_matrix:",
    ]
    for row in R_full:
        lines.append("  - [" + ", ".join(f"{v:.12g}" for v in row) + "]")
    lines += [
        "q_I_F: [" + ", ".join(f"{v:.12g}" for v in q_full) + "]",
        "full_rpy_deg: [" + ", ".join(f"{v:.12g}" for v in rpy_full) + "]",
        "R_residual_matrix:",
    ]
    for row in X:
        lines.append("  - [" + ", ".join(f"{v:.12g}" for v in row) + "]")
    lines += [
        "q_residual: [" + ", ".join(f"{v:.12g}" for v in q_residual) + "]",
        "residual_rpy_deg: [" + ", ".join(f"{v:.12g}" for v in rpy_residual) + "]",
        f"delta_t_fc_s: {delta:.9f}",
        "delta_t_definition: FC attitude used at VIO time t is R_F(t + delta_t_fc)",
        "nominal_axis_transform: frd_to_xright_yfwd_zup",
        "nominal_axis_transform_matrix:",
    ]
    for row in NOMINAL_AXIS_TRANSFORM:
        lines.append("  - [" + ", ".join(f"{v:.12g}" for v in row) + "]")
    lines += [
        "composition_order: R_I_F = R_residual * R_nominal_axis_transform",
        "training_runs:",
        yaml_list([f"{r.flight}: {r.run_path}" for r in runs if not r.rejected], 2),
        "validation_runs:",
        yaml_list(sorted({r.flight for r in runs if not r.rejected}), 2),
        "relative_rotation_intervals_s: [" + ", ".join(f"{v:.2f}" for v in intervals) + "]",
        "sample_filters:",
    ]
    for k, v in sample_filters.items():
        lines.append(f"  {k}: {v}")
    lines.append("training_statistics:")
    for k, v in train_stats.items():
        lines.append(f"  {k}: {v}")
    lines.append("validation_statistics:")
    for k, v in val_stats.items():
        lines.append(f"  {k}: {v}")
    lines.append(f"accepted: {'true' if accepted else 'false'}")
    if reasons:
        lines.append("rejection_reasons:")
        lines.append(yaml_list(reasons, 2))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def evaluate_scheme(name: str, runs_train: list[RunData], runs_eval: list[RunData], intervals: tuple[float, ...],
                    sample_step_s: float, min_angle_deg: float, max_interp_gap_s: float) -> dict[str, object]:
    delta, X, train_samples, scan = calibrate_scheme(runs_train, name, intervals, sample_step_s, min_angle_deg, max_interp_gap_s)
    eval_samples = make_samples(runs_eval, delta, intervals, sample_step_s, min_angle_deg, max_interp_gap_s)
    row = {"scheme": name, "delta_t_fc_s": delta, **{f"train_{k}": v for k, v in statistics(train_samples, X).items()},
           **{f"validation_{k}": v for k, v in statistics(eval_samples, X).items()}}
    row["rotation_rpy_deg"] = ",".join(f"{v:.4f}" for v in rot_to_rpy_deg(X))
    row["scan_rows"] = len(scan)
    return {"row": row, "delta": delta, "X": X, "train_samples": train_samples, "eval_samples": eval_samples, "scan": scan}


def command_inspect(args: argparse.Namespace) -> int:
    paths = args.run_dir or [str(p) for p in default_candidates(Path.cwd())]
    runs = load_runs(paths)
    print_coordinate_audit()
    numeric_direction_tests()
    real_sample_direction_test(runs)
    rows = inspect_runs(runs)
    print("当前阶段: 数据可用性")
    print("使用的数据: fly1~fly4 current stride12 plus fixed fly3/fly4 validation runs when present")
    for row in rows:
        print(json.dumps(row, ensure_ascii=False, default=str))
    usable = [r for r in runs if not r.rejected]
    print(f"当前数值结果: usable_runs={len(usable)} rejected_runs={len(runs)-len(usable)}")
    print("尚未排除的问题: 需要通过时间扫描和 leave-one-flight-out 验证 residual rotation 是否跨飞行一致。")
    print("下一步: run-all 将执行 Baseline A/B/C 与联合 D。")
    return 0 if usable else 2


def command_run_all(args: argparse.Namespace) -> int:
    intervals = tuple(args.intervals)
    paths = args.run_dir or [str(p) for p in default_candidates(Path.cwd())]
    all_runs = load_runs(paths)
    print_coordinate_audit()
    numeric_direction_tests()
    real_sample_direction_test(all_runs)
    inspect_rows = inspect_runs(all_runs)
    usable_by_flight: dict[str, RunData] = {}
    rejected_rows: list[dict[str, object]] = []
    for row, run in zip(inspect_rows, all_runs):
        if run.rejected or not row.get("usable", False):
            rejected_rows.append(row)
            continue
        # Prefer fixed validation directories over older repo current runs when
        # both exist for the same flight.
        old = usable_by_flight.get(run.flight)
        if old is None or "openvins_fc_init_fix_validation" in str(run.run_path):
            usable_by_flight[run.flight] = run
    runs = [usable_by_flight[k] for k in sorted(usable_by_flight)]
    print("当前阶段: 数据可用性")
    print("使用的数据:")
    for r in runs:
        print(f"  use {r.flight}: {r.run_path}")
    for row in rejected_rows:
        print(f"  reject {row.get('flight')}: {row.get('run_path')} reason={row.get('rejection_reason')}")
    if len(runs) < 2:
        print("标定未通过: usable flights < 2", file=sys.stderr)
        return 2

    out_dir = Path(args.out_dir) if args.out_dir else Path("result") / ("fc_imu_calibration_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S"))
    out_dir.mkdir(parents=True, exist_ok=True)

    print("当前阶段: 时间偏移扫描与手眼旋转估计")
    scheme_results = []
    scans: list[dict[str, object]] = []
    for scheme in ("A", "B", "C", "D"):
        res = evaluate_scheme(scheme, runs, runs, intervals, args.sample_step_s, args.min_angle_deg, args.max_interp_gap_s)
        scheme_results.append(res)
        for srow in res["scan"]:
            scans.append({"scheme": scheme, **srow})
        row = res["row"]
        print(f"  scheme {scheme}: delta={row['delta_t_fc_s']:.4f}s train_median={row.get('train_median_deg', float('nan')):.3f}deg "
              f"train_p95={row.get('train_p95_deg', float('nan')):.3f}deg validation_median={row.get('validation_median_deg', float('nan')):.3f}deg "
              f"validation_p95={row.get('validation_p95_deg', float('nan')):.3f}deg")
    write_csv(out_dir / "time_offset_scan.csv", scans)
    write_csv(out_dir / "calibration_summary.csv", [r["row"] for r in scheme_results])
    cand = scheme_results[-1]
    X = cand["X"]
    delta = float(cand["delta"])

    print("当前阶段: leave-one-flight-out 验证")
    lofo_rows = []
    single_rows = []
    single_flight_X = []
    for hold in sorted({r.flight for r in runs}):
        tr = [r for r in runs if r.flight != hold]
        va = [r for r in runs if r.flight == hold]
        if not tr or not va:
            continue
        res = evaluate_scheme("D", tr, va, intervals, args.sample_step_s, args.min_angle_deg, args.max_interp_gap_s)
        row = {"row_type": "lofo", "holdout": hold, **res["row"]}
        lofo_rows.append(row)
        single = evaluate_scheme("D", va, va, intervals, args.sample_step_s, args.min_angle_deg, args.max_interp_gap_s)
        single_row = {"row_type": "single_flight", "flight": hold, **single["row"]}
        single_rows.append(single_row)
        single_flight_X.append((hold, single["X"], single["delta"]))
        print(f"  holdout {hold}: train_delta={row['delta_t_fc_s']:.4f}s val_median={row.get('validation_median_deg', float('nan')):.3f}deg "
              f"val_p95={row.get('validation_p95_deg', float('nan')):.3f}deg")
        print(f"  single {hold}: delta={single_row['delta_t_fc_s']:.4f}s median={single_row.get('validation_median_deg', float('nan')):.3f}deg "
              f"p95={single_row.get('validation_p95_deg', float('nan')):.3f}deg")

    rot_spread_rows = []
    for i in range(len(single_flight_X)):
        for j in range(i + 1, len(single_flight_X)):
            a, Xa, da = single_flight_X[i]
            b, Xb, db = single_flight_X[j]
            rot_spread_rows.append({
                "row_type": "single_flight_rotation_spread",
                "flight_a": a,
                "flight_b": b,
                "rotation_diff_deg": angle_deg(Xa.T @ Xb),
                "delta_diff_s": abs(da - db),
            })

    group_rows = []
    for name, res in zip(("A", "B", "C", "D"), scheme_results):
        samples = res["eval_samples"]
        for row in grouped_stats(samples, res["X"]):
            group_rows.append({"row_type": "group", "scheme": name, **row})
    absolute_rows = []
    absolute_rows.extend(absolute_attitude_check(runs, "A", 0.0, np.eye(3), args.absolute_window_s,
                                                args.sample_step_s, args.max_interp_gap_s))
    absolute_rows.extend(absolute_attitude_check(runs, "D", delta, X, args.absolute_window_s,
                                                args.sample_step_s, args.max_interp_gap_s))
    write_csv(out_dir / "validation_summary.csv", lofo_rows + single_rows + group_rows + absolute_rows + rot_spread_rows)

    A = scheme_results[0]["row"]
    D = scheme_results[3]["row"]
    reasons = []
    def val_float(row: dict[str, object], key: str) -> float:
        try:
            return float(row.get(key, float("nan")))
        except Exception:
            return float("nan")
    if not (val_float(D, "validation_median_deg") < 0.9 * val_float(A, "validation_median_deg")):
        reasons.append("joint validation median did not drop by at least 10% versus Baseline A")
    if not (val_float(D, "validation_p95_deg") < 0.9 * val_float(A, "validation_p95_deg")):
        reasons.append("joint validation P95 did not drop by at least 10% versus Baseline A")
    if abs(delta) >= 0.495:
        reasons.append("best delta_t_fc is on search boundary")
    if rot_spread_rows:
        max_rot_spread = max(float(r["rotation_diff_deg"]) for r in rot_spread_rows)
        if max_rot_spread > args.max_rotation_spread_deg:
            reasons.append(f"single-flight rotation spread {max_rot_spread:.3f}deg exceeds {args.max_rotation_spread_deg:.3f}deg")
    if single_rows:
        single_deltas = [float(r["delta_t_fc_s"]) for r in single_rows]
        max_single_delta_spread = max(single_deltas) - min(single_deltas)
        if any(abs(d) >= 0.495 for d in single_deltas):
            reasons.append("one or more single-flight delta_t_fc is on search boundary")
        if max_single_delta_spread > args.max_single_delta_spread_s:
            reasons.append(f"single-flight delta_t_fc spread {max_single_delta_spread:.3f}s exceeds {args.max_single_delta_spread_s:.3f}s")
    if lofo_rows:
        if any(val_float(r, "validation_count") < 20 for r in lofo_rows):
            reasons.append("one or more leave-one-flight-out folds has too few validation samples")
    for group in ("left_turn", "right_turn"):
        a_rows = [r for r in group_rows if r.get("scheme") == "A" and r.get("group") == group]
        d_rows = [r for r in group_rows if r.get("scheme") == "D" and r.get("group") == group]
        if a_rows and d_rows and not (float(d_rows[0]["median_deg"]) < float(a_rows[0]["median_deg"])):
            reasons.append(f"{group} median did not improve")
    excitation = excitation_report(cand["train_samples"])
    if min(excitation.get("roll_rms_deg", 0), excitation.get("pitch_rms_deg", 0), excitation.get("yaw_rms_deg", 0)) < args.min_axis_rms_deg:
        reasons.append("three-axis excitation is weak")
    accepted = len(reasons) == 0

    sample_filters = {
        "min_relative_rotation_deg": args.min_angle_deg,
        "max_interp_gap_s": args.max_interp_gap_s,
        "sample_step_s": args.sample_step_s,
        "time_search_range_s": "[-0.50, 0.50]",
        "coarse_step_s": 0.005,
        "fine_step_s": 0.001,
    }
    write_yaml(out_dir / "fc_imu_calibration.yaml", accepted, delta, X, runs, statistics(cand["train_samples"], X),
               statistics(cand["eval_samples"], X), intervals, sample_filters, reasons)

    print("当前阶段: 最终是否接受标定")
    print(f"当前数值结果: delta_t_fc={delta:.6f}s, residual_rpy_deg={rot_to_rpy_deg(X)}, accepted={accepted}")
    if reasons:
        print("尚未排除的问题:")
        for reason in reasons:
            print(f"  - {reason}")
    print(f"下一步: {'将已验证的 FC 时间和安装旋转接入多帧初始化，并做单帧对照实验。' if accepted else '针对不可观方向、动态延迟或数据时间轴问题补充最小验证，不接入初始化。'}")
    print(f"输出目录: {out_dir}")
    return 0


def command_validate(args: argparse.Namespace) -> int:
    runs = load_runs(args.run_dir or [str(p) for p in default_candidates(Path.cwd())])
    runs = [r for r in runs if not r.rejected]
    X = np.asarray(json.loads(args.rotation_matrix), dtype=float) if args.rotation_matrix else np.eye(3)
    samples = make_samples(runs, args.delta_t_fc_s, tuple(args.intervals), args.sample_step_s, args.min_angle_deg, args.max_interp_gap_s)
    print(json.dumps(statistics(samples, X), indent=2, ensure_ascii=False))
    return 0


def command_calibrate(args: argparse.Namespace) -> int:
    runs = load_runs(args.run_dir or [str(p) for p in default_candidates(Path.cwd())])
    runs = [r for r in runs if not r.rejected]
    delta, X, samples, _ = calibrate_scheme(runs, "D", tuple(args.intervals), args.sample_step_s, args.min_angle_deg, args.max_interp_gap_s)
    print(f"delta_t_fc_s={delta:.9f}")
    print("R_I_F_residual=")
    print(X)
    print("residual_rpy_deg=", rot_to_rpy_deg(X))
    print("stats=", statistics(samples, X))
    return 0


def generate_synth(delta_true: float, X_true: np.ndarray, sign_flip: bool = False, world_left: np.ndarray | None = None,
                   world_right: np.ndarray | None = None) -> RunData:
    t_fc = np.arange(0.0, 40.0, 0.02)
    def Rf_func(t: float) -> np.ndarray:
        return rz(0.08 * t + 0.25 * math.sin(0.4 * t)) @ ry(0.10 * math.sin(0.7 * t)) @ rx(0.08 * math.cos(0.5 * t))
    # Real FC rows store OpenVINS-style JPL q_GtoI, so the synthetic FC stream
    # stores the passive transpose of the generated R_WF orientation.
    q_fc = np.asarray([
        rot_to_jpl_quat(((world_right if world_right is not None else np.eye(3)) @ Rf_func(t)).T)
        for t in t_fc
    ])
    t_vio = np.arange(2.0, 35.0, 0.05)
    q_vio = np.asarray([
        rot_to_hamilton_xyzw((world_left if world_left is not None else np.eye(3)) @ (Rf_func(t + delta_true) @ X_true.T))
        for t in t_vio
    ])
    if sign_flip:
        q_fc[::7] *= -1.0
        q_vio[::5] *= -1.0
    run = RunData("synthetic", Path("synthetic"))
    run.fc_t, run.fc_q = t_fc, normalize_quats(q_fc)
    run.vio_t, run.vio_q = t_vio, normalize_quats(q_vio)
    return run


def command_self_test(_: argparse.Namespace) -> int:
    tests = []
    X_true = rz(math.radians(6.0)) @ ry(math.radians(-3.0)) @ rx(math.radians(2.0))
    def quick_test_estimate(run: RunData, expected_delta: float) -> tuple[float, np.ndarray, list[RelSample]]:
        intervals = (0.20, 0.50, 1.00)
        coarse = np.arange(max(-0.20, expected_delta - 0.08), min(0.20, expected_delta + 0.08) + 1e-12, 0.01)
        d0, _, _ = scan_delta([run], intervals, 1.0, 0.5, 0.05, True, coarse)
        fine = np.arange(max(-0.20, d0 - 0.015), min(0.20, d0 + 0.015) + 1e-12, 0.001)
        d, Xh, _ = scan_delta([run], intervals, 1.0, 0.5, 0.05, True, fine)
        samples = make_samples([run], d, intervals, 1.0, 0.5, 0.05)
        return d, Xh, samples
    for name, delta, X, flip, wl, wr in [
        ("fixed_rotation", 0.0, X_true, False, None, None),
        ("time_offset", 0.123, np.eye(3), False, None, None),
        ("joint", -0.087, X_true, False, None, None),
        ("quat_sign_flip", -0.087, X_true, True, None, None),
        ("world_rotation_invariance", -0.087, X_true, False, rz(0.9) @ ry(0.2), rz(-0.4) @ rx(0.3)),
    ]:
        run = generate_synth(delta, X, flip, wl, wr)
        d_hat, X_hat, samples = quick_test_estimate(run, delta)
        tests.append((name, abs(d_hat - delta), angle_deg(X_hat.T @ X), len(samples)))
    low = generate_synth(0.0, np.eye(3))
    low.fc_q[:] = low.fc_q[0]
    low.vio_q[:] = low.vio_q[0]
    low_samples = make_samples([low], 0.0, REL_INTERVALS_DEFAULT, 0.2, 1.0, 0.05)
    tests.append(("low_excitation_reject", 0.0, 0.0, len(low_samples)))
    ok = True
    for name, dt_err, rot_err, count in tests:
        print(f"{name}: dt_err={dt_err:.6f}s rot_err={rot_err:.6f}deg samples={count}")
        if name == "low_excitation_reject":
            ok = ok and count == 0
        else:
            ok = ok and dt_err <= 0.006 and rot_err <= 0.20 and count > 20
    return 0 if ok else 2


def add_common(p: argparse.ArgumentParser) -> None:
    p.add_argument("--run-dir", action="append", default=[], help="run directory; can repeat")
    p.add_argument("--intervals", type=float, nargs="+", default=list(REL_INTERVALS_DEFAULT))
    p.add_argument("--sample-step-s", type=float, default=8.00)
    p.add_argument("--min-angle-deg", type=float, default=1.0)
    p.add_argument("--max-interp-gap-s", type=float, default=0.30)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("inspect")
    add_common(p)
    p.set_defaults(func=command_inspect)
    p = sub.add_parser("calibrate")
    add_common(p)
    p.set_defaults(func=command_calibrate)
    p = sub.add_parser("validate")
    add_common(p)
    p.add_argument("--delta-t-fc-s", type=float, default=0.0)
    p.add_argument("--rotation-matrix", default="", help="JSON 3x3 matrix")
    p.set_defaults(func=command_validate)
    p = sub.add_parser("run-all")
    add_common(p)
    p.add_argument("--out-dir", default="")
    p.add_argument("--max-rotation-spread-deg", type=float, default=5.0)
    p.add_argument("--max-single-delta-spread-s", type=float, default=0.25)
    p.add_argument("--min-axis-rms-deg", type=float, default=0.05)
    p.add_argument("--absolute-window-s", type=float, default=120.0)
    p.set_defaults(func=command_run_all)
    p = sub.add_parser("self-test")
    p.set_defaults(func=command_self_test)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
