#!/usr/bin/env python3
"""Align GSMQ FC relative time to D455 camera/IMU time by turn events.

The FC GPS clock and the Jetson clock can have an arbitrary absolute offset.
This script estimates the relative offset used by:

    t_cam = (fc_time - first_valid_fc_time) - offset

It compares the timing of GPS-course/FC-yaw turn events against D455 gyro
events. The amplitude and sign are intentionally normalized away; the useful
signal is when turns happen, not how large the yaw-rate magnitude is.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def parse_fc_time_utc(text: str) -> float:
    text = text.strip()
    if "_" in text:
        base, ms = text.rsplit("_", 1)
        micros = int(ms.ljust(3, "0")[:3]) * 1000
    else:
        base = text
        micros = 0
    stamp = dt.datetime.strptime(base, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(microsecond=micros, tzinfo=dt.timezone.utc).timestamp()


def iter_clean_csv_rows(path: Path):
    with path.open("rb") as f:
        for raw in f:
            raw = raw.replace(b"\x00", b"").strip()
            if not raw or raw.startswith(b"#"):
                continue
            yield raw.decode("utf-8-sig", "replace")


def unwrap_deg(yaw_deg: np.ndarray) -> np.ndarray:
    return np.rad2deg(np.unwrap(np.deg2rad(np.asarray(yaw_deg, dtype=float))))


def robust_zscore(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    med = np.nanmedian(x)
    mad = np.nanmedian(np.abs(x - med))
    scale = 1.4826 * mad if mad > 1e-12 else np.nanstd(x)
    if not np.isfinite(scale) or scale < 1e-12:
        return np.zeros_like(x)
    z = (x - med) / scale
    return np.clip(z, -8.0, 8.0)


def moving_average(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    kernel = np.ones(n, dtype=float) / float(n)
    return np.convolve(x, kernel, mode="same")


def gps_course_signal(fc: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    # Some FC logs contain repeated GPS timestamps. Collapse them before
    # differentiating course; otherwise np.gradient sees zero dt and emits NaNs.
    _, keep = np.unique(fc["t_rel"], return_index=True)
    keep = np.sort(keep)
    lat = fc["lat"][keep]
    lon = fc["lon"][keep]
    t = fc["t_rel"][keep]
    lat0 = math.radians(float(lat[0]))
    lon0 = math.radians(float(lon[0]))
    cos_lat = math.cos(lat0)
    earth_r = 6378137.0
    east = (np.radians(lon) - lon0) * cos_lat * earth_r
    north = (np.radians(lat) - lat0) * earth_r

    course_t, course, speed = [], [], []
    for ti in t:
        j0 = int(np.searchsorted(t, ti - 3.0))
        j1 = int(np.searchsorted(t, ti + 3.0))
        if j1 <= j0 + 4:
            continue
        dt = t[j1 - 1] - t[j0]
        if dt <= 0.0:
            continue
        de = east[j1 - 1] - east[j0]
        dn = north[j1 - 1] - north[j0]
        v = math.hypot(de, dn) / dt
        if v < 10.0:
            continue
        course_t.append(float(ti))
        course.append(math.degrees(math.atan2(dn, de)))
        speed.append(v)

    course_t = np.asarray(course_t, dtype=float)
    course = unwrap_deg(course)
    if len(course_t) < 5:
        raise RuntimeError("not enough moving GPS course samples")
    rate = np.abs(np.gradient(course, course_t))
    return course_t, rate


def load_fc(fc_path: Path):
    rows = []
    with fc_path.open("r", newline="", encoding="utf-8-sig", errors="replace") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for raw in reader:
            if len(raw) < 11:
                continue
            try:
                unix = parse_fc_time_utc(raw[3])
                sat = int(float(raw[4]))
                lat = float(raw[5])
                lon = float(raw[6])
                alt = float(raw[7])
                ve = float(raw[8])
                vn = float(raw[9])
                vu = float(raw[10])
                yaw = float(raw[2])
            except Exception:
                continue
            if sat <= 0 or (lat == 0.0 and lon == 0.0):
                continue
            rows.append((unix, yaw, lat, lon, alt, ve, vn, vu, sat))
    if len(rows) < 10:
        raise RuntimeError(f"too few valid FC rows in {fc_path}")
    arr = np.asarray(rows, dtype=float)
    rel_t = arr[:, 0] - arr[0, 0]
    yaw = unwrap_deg(arr[:, 1])
    return {
        "header": header,
        "t_rel": rel_t,
        "yaw": yaw,
        "lat": arr[:, 2],
        "lon": arr[:, 3],
        "alt": arr[:, 4],
        "ve": arr[:, 5],
        "vn": arr[:, 6],
        "vu": arr[:, 7],
        "sat": arr[:, 8],
    }


def load_imu(dataset: Path):
    path = dataset / "imu0" / "data.csv"
    ts, ws = [], []
    def imu_lines():
        with path.open("rb") as f:
            for raw in f:
                raw = raw.replace(b"\x00", b"").strip()
                if not raw:
                    continue
                yield raw.decode("utf-8-sig", "replace")

    lines = imu_lines()
    header = next(csv.reader([next(lines)]))
    idx = {name: i for i, name in enumerate(header)}
    for line in lines:
        row = next(csv.reader([line]))
        try:
            ts.append(float(row[idx["#t_rel_s"]]))
            ws.append([float(row[idx["wx"]]), float(row[idx["wy"]]), float(row[idx["wz"]])])
        except Exception:
            continue
    if len(ts) < 10:
        raise RuntimeError(f"too few IMU rows in {path}")
    return np.asarray(ts, dtype=float), np.asarray(ws, dtype=float)


def resample_signal(src_t: np.ndarray, src_y: np.ndarray, grid: np.ndarray) -> np.ndarray:
    return np.interp(grid, src_t, src_y, left=np.nan, right=np.nan)


def make_fc_source(fc: dict[str, np.ndarray], source: str) -> tuple[np.ndarray, np.ndarray]:
    if source == "gps_course_rate":
        return gps_course_signal(fc)
    elif source == "fc_yaw_rate":
        src_t = fc["t_rel"]
        yaw = fc["yaw"]
        return src_t, np.abs(np.gradient(yaw, src_t))
    else:
        raise ValueError(f"unknown FC signal source: {source}")


def make_fc_signal(src_t: np.ndarray, src_signal: np.ndarray, grid: np.ndarray, offset: float, smooth_samples: int) -> np.ndarray:
    fc_cam_t = src_t - offset
    signal = moving_average(src_signal, smooth_samples)
    return resample_signal(fc_cam_t, signal, grid)


def make_imu_signals(imu_t: np.ndarray, gyro: np.ndarray, grid: np.ndarray, smooth_samples: int):
    imu_abs_norm = np.linalg.norm(gyro, axis=1)
    # In the D455 aircraft mounting used here, yaw usually projects strongly to
    # IMU z, but the norm gives a useful fallback when the axis convention is
    # being audited.
    raw = {
        "gyro_norm": imu_abs_norm,
        "abs_wz": np.abs(gyro[:, 2]),
        "abs_wx": np.abs(gyro[:, 0]),
        "abs_wy": np.abs(gyro[:, 1]),
    }
    out = {}
    for key, val in raw.items():
        out[key] = resample_signal(imu_t, moving_average(val, smooth_samples), grid)
    return out


def score_pair(a: np.ndarray, b: np.ndarray) -> tuple[float, int]:
    valid = np.isfinite(a) & np.isfinite(b)
    if int(valid.sum()) < 20:
        return -1e9, int(valid.sum())
    aa = robust_zscore(a[valid])
    bb = robust_zscore(b[valid])
    score = float(np.mean(aa * bb))
    return score, int(valid.sum())


def scan_offsets(fc: dict[str, np.ndarray], imu_t: np.ndarray, gyro: np.ndarray, args):
    grid = np.arange(float(imu_t[0]), float(imu_t[-1]), args.grid_dt)
    smooth = max(1, int(round(args.smooth_s / args.grid_dt)))
    imu_signals = make_imu_signals(imu_t, gyro, grid, smooth)
    if args.imu_signal != "auto":
        imu_signals = {args.imu_signal: imu_signals[args.imu_signal]}
    fc_src_t, fc_src_signal = make_fc_source(fc, args.fc_signal)
    offsets = np.arange(args.offset_min, args.offset_max + 0.5 * args.offset_step, args.offset_step)

    rows = []
    for off in offsets:
        fc_sig = make_fc_signal(fc_src_t, fc_src_signal, grid, float(off), smooth)
        best_key, best_score, best_n = None, -1e9, 0
        scores = {}
        for key, imu_sig in imu_signals.items():
            sc, n = score_pair(fc_sig, imu_sig)
            scores[key] = sc
            if sc > best_score:
                best_key, best_score, best_n = key, sc, n
        rows.append(
            {
                "offset": float(off),
                "best_signal": best_key,
                "score": float(best_score),
                "pairs": int(best_n),
                **{f"score_{k}": float(v) for k, v in scores.items()},
            }
        )
    rows.sort(key=lambda r: r["score"], reverse=True)
    return rows, grid, imu_signals


def choose_init_candidates(fc: dict[str, np.ndarray], offset: float, dataset_t1: float, count: int):
    _, keep = np.unique(fc["t_rel"], return_index=True)
    keep = np.sort(keep)
    t = fc["t_rel"][keep] - offset
    speed = np.sqrt(fc["ve"][keep] ** 2 + fc["vn"][keep] ** 2 + fc["vu"][keep] ** 2)
    yaw_rate = np.abs(np.gradient(fc["yaw"][keep], fc["t_rel"][keep]))
    alt = fc["alt"][keep]
    good = (t > 30.0) & (t < dataset_t1 - 40.0) & (speed > 25.0) & np.isfinite(yaw_rate)
    if not good.any():
        return []
    # Prefer straight-ish, steady-speed windows after takeoff. This is for init;
    # turn windows are useful for alignment but fragile as init anchors.
    score = np.full_like(t, np.inf, dtype=float)
    speed_med = np.nanmedian(speed[good])
    score[good] = yaw_rate[good] + 0.05 * np.abs(speed[good] - speed_med)
    order = np.argsort(score)
    selected = []
    for idx in order:
        if not np.isfinite(score[idx]):
            continue
        ti = float(t[idx])
        if all(abs(ti - old["start_time_s"]) > 25.0 for old in selected):
            selected.append(
                {
                    "start_time_s": round(ti, 3),
                    "fc_rel_s": round(float(fc["t_rel"][idx]), 3),
                    "speed_mps": round(float(speed[idx]), 3),
                    "abs_yaw_rate_deg_s": round(float(yaw_rate[idx]), 3),
                    "alt_m": round(float(alt[idx]), 3),
                }
            )
        if len(selected) >= count:
            break
    return selected


def write_outputs(fc, imu_t, gyro, rows, grid, imu_signals, args):
    args.out_dir.mkdir(parents=True, exist_ok=True)
    scan_csv = args.out_dir / "offset_scan.csv"
    fields = ["offset", "best_signal", "score", "pairs", "score_gyro_norm", "score_abs_wz", "score_abs_wx", "score_abs_wy"]
    with scan_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    best = rows[0]
    smooth = max(1, int(round(args.smooth_s / args.grid_dt)))
    fc_src_t, fc_src_signal = make_fc_source(fc, args.fc_signal)
    fc_sig = make_fc_signal(fc_src_t, fc_src_signal, grid, best["offset"], smooth)
    imu_sig = imu_signals[best["best_signal"]]
    valid = np.isfinite(fc_sig) & np.isfinite(imu_sig)

    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
    top = rows[: min(20, len(rows))]
    axs[0].plot([r["offset"] for r in rows], [r["score"] for r in rows], lw=1.2)
    axs[0].scatter([r["offset"] for r in top], [r["score"] for r in top], s=18)
    axs[0].set_title("FC yaw-change vs D455 gyro event alignment")
    axs[0].set_xlabel("fc_rel_to_cam_offset [s] in t_cam = fc_rel - offset")
    axs[0].set_ylabel("normalized event score")
    axs[0].grid(alpha=0.3)

    axs[1].plot(grid[valid], robust_zscore(fc_sig[valid]), label=args.fc_signal)
    axs[1].plot(grid[valid], robust_zscore(imu_sig[valid]), label=f"IMU {best['best_signal']}")
    axs[1].set_title(f"Best overlay: offset={best['offset']:.3f}s, score={best['score']:.3f}")
    axs[1].set_xlabel("camera time [s]")
    axs[1].set_ylabel("robust z-score")
    axs[1].grid(alpha=0.3)
    axs[1].legend()

    fc_cam_t = fc["t_rel"] - best["offset"]
    speed = np.sqrt(fc["ve"] ** 2 + fc["vn"] ** 2 + fc["vu"] ** 2)
    axs[2].plot(fc_cam_t, speed, label="FC speed")
    axs[2].plot(fc_cam_t, fc["alt"] - np.nanmin(fc["alt"]), label="FC altitude rel")
    axs[2].set_xlabel("camera time [s]")
    axs[2].set_ylabel("m/s or rel m")
    axs[2].grid(alpha=0.3)
    axs[2].legend()
    fig.tight_layout()
    fig.savefig(args.out_dir / "alignment_diagnostic.png", dpi=160)
    plt.close(fig)

    candidates = choose_init_candidates(fc, best["offset"], float(imu_t[-1]), args.init_count)
    summary = {
        "fc": str(args.fc),
        "dataset": str(args.dataset),
        "best_offset_s": best["offset"],
        "best_score": best["score"],
        "best_signal": best["best_signal"],
        "fc_signal": args.fc_signal,
        "pairs": best["pairs"],
        "top_offsets": top,
        "init_candidates": candidates,
        "time_model": "t_cam = (fc_time - first_valid_fc_time) - offset",
        "scan_csv": str(scan_csv),
        "plot": str(args.out_dir / "alignment_diagnostic.png"),
    }
    with (args.out_dir / "alignment_summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", type=Path, required=True)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--offset-min", type=float, default=-100.0)
    ap.add_argument("--offset-max", type=float, default=650.0)
    ap.add_argument("--offset-step", type=float, default=0.1)
    ap.add_argument("--grid-dt", type=float, default=0.1)
    ap.add_argument("--smooth-s", type=float, default=1.0)
    ap.add_argument("--fc-signal", choices=["gps_course_rate", "fc_yaw_rate"], default="gps_course_rate")
    ap.add_argument("--imu-signal", choices=["auto", "gyro_norm", "abs_wz", "abs_wx", "abs_wy"], default="auto")
    ap.add_argument("--init-count", type=int, default=8)
    args = ap.parse_args()

    fc = load_fc(args.fc)
    imu_t, gyro = load_imu(args.dataset)
    rows, grid, imu_signals = scan_offsets(fc, imu_t, gyro, args)
    summary = write_outputs(fc, imu_t, gyro, rows, grid, imu_signals, args)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
