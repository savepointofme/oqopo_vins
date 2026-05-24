#!/usr/bin/env python3
"""Calibrate raw ArduPilot GPS time against an OpenVINS/TUM trajectory.

The GPS log has its own TimeUS clock. This script estimates a constant offset
by matching the early horizontal speed profile of GPS and TUM, then writes:

  - GPS CSV with OpenVINS-style ns timestamps
  - TUM truth files in camera time and IMU time
  - ASL/EuRoC 17-column groundtruth CSV files for run_serial_msckf_ros_free --gt

Kalibr convention for timeshift_cam_imu is treated as:
    t_imu = t_cam + timeshift_cam_imu
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

import numpy as np


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
NS_PER_SEC = 1_000_000_000


def sec_text_to_ns(text: str) -> int:
    return int((Decimal(text) * Decimal(NS_PER_SEC)).to_integral_value(rounding=ROUND_HALF_UP))


def ns_to_sec_text(ns: int) -> str:
    sign = "-" if ns < 0 else ""
    ns_abs = abs(int(ns))
    return f"{sign}{ns_abs // NS_PER_SEC}.{ns_abs % NS_PER_SEC:09d}"


@dataclass
class GpsRaw:
    time_us: np.ndarray
    rel_s: np.ndarray
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray
    enu: np.ndarray


@dataclass
class TumTraj:
    t_ns: np.ndarray
    t: np.ndarray
    p: np.ndarray
    q_xyzw: np.ndarray


def read_imu_span(path: Path) -> tuple[int, int]:
    first = None
    last = None
    with path.open("r", newline="") as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            ts = int(s.split(",", 1)[0])
            if first is None:
                first = ts
            last = ts
    if first is None or last is None:
        raise ValueError(f"No IMU samples found in {path}")
    return first, last


def lla_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, alt_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * np.cos(lon)
    y = (n + alt_m) * cos_lat * np.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return np.column_stack((x, y, z))


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
        ],
        dtype=np.float64,
    )
    return (ecef - origin_ecef.reshape(1, 3)) @ rot.T


def read_raw_gps(path: Path, min_status: int) -> GpsRaw:
    rows: list[tuple[int, float, float, float]] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        fields = set(reader.fieldnames or [])
        required = {"TimeUS", "Lat", "Lng", "Alt"}
        missing = required - fields
        if missing:
            raise ValueError(f"{path} missing columns: {sorted(missing)}")
        has_status = "Status" in fields
        for row in reader:
            try:
                status = int(float(row["Status"])) if has_status else min_status
                if has_status and status < min_status:
                    continue
                rows.append(
                    (
                        int(float(row["TimeUS"])),
                        float(row["Lat"]),
                        float(row["Lng"]),
                        float(row["Alt"]),
                    )
                )
            except ValueError:
                continue

    if len(rows) < 20:
        raise ValueError(f"Not enough GPS rows in {path}")

    arr_obj = np.asarray(rows, dtype=object)
    time_us = arr_obj[:, 0].astype(np.int64)
    lat = arr_obj[:, 1].astype(np.float64)
    lon = arr_obj[:, 2].astype(np.float64)
    alt = arr_obj[:, 3].astype(np.float64)
    ecef = lla_to_ecef(lat, lon, alt)
    origin = ecef[0]
    enu = ecef_to_enu(ecef, float(lat[0]), float(lon[0]), origin)
    return GpsRaw(time_us=time_us, rel_s=(time_us - time_us[0]).astype(np.float64) * 1e-6, lat=lat, lon=lon, alt=alt, enu=enu)


def read_tum(path: Path) -> TumTraj:
    rows: list[list[float]] = []
    t_ns_rows: list[int] = []
    with path.open("r") as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 8:
                continue
            try:
                vals = [float(x) for x in parts[:8]]
                t_ns = sec_text_to_ns(parts[0])
                t_ns_rows.append(t_ns)
                rows.append(vals)
            except ValueError:
                continue
    if len(rows) < 20:
        raise ValueError(f"Not enough TUM rows in {path}")
    arr = np.asarray(rows, dtype=np.float64)
    t_ns = np.asarray(t_ns_rows, dtype=np.int64)
    return TumTraj(t_ns=t_ns, t=t_ns.astype(np.float64) * 1e-9, p=arr[:, 1:4], q_xyzw=arr[:, 4:8])


def interp_columns(src_t: np.ndarray, src_y: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(query_t, src_t, src_y[:, i]) for i in range(src_y.shape[1])])


def smooth_moving_average(values: np.ndarray, dt: float, window_s: float) -> np.ndarray:
    n = max(1, int(round(window_s / dt)))
    kernel = np.ones(n, dtype=np.float64) / float(n)
    return np.convolve(values, kernel, mode="same")


def horizontal_speed(t: np.ndarray, xy: np.ndarray, dt: float, t_max: float, smooth_s: float) -> tuple[np.ndarray, np.ndarray]:
    grid = np.arange(0.0, min(float(t[-1]), t_max) + 0.5 * dt, dt, dtype=np.float64)
    if grid.size < 4:
        raise ValueError("Time range too short for speed alignment")
    pos = interp_columns(t, xy, grid)
    speed = np.linalg.norm(np.diff(pos, axis=0), axis=1) / dt
    mid_t = grid[:-1] + 0.5 * dt
    return mid_t, smooth_moving_average(speed, dt, smooth_s)


def normalized_rmse_and_corr(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    a_std = float(np.std(a))
    b_std = float(np.std(b))
    if a_std < 1e-9 or b_std < 1e-9:
        return float("nan"), float("nan")
    an = (a - float(np.mean(a))) / a_std
    bn = (b - float(np.mean(b))) / b_std
    diff = an - bn
    rmse = float(np.sqrt(np.mean(diff * diff)))
    corr = float(np.mean(an * bn))
    return rmse, corr


def estimate_gps_start_offset(
    gps: GpsRaw,
    tum: TumTraj,
    scan_min: float,
    scan_max: float,
    scan_step: float,
    align_window: tuple[float, float],
    resample_dt: float,
    smooth_s: float,
) -> tuple[float, list[dict[str, float]]]:
    tum_rel = tum.t - tum.t[0]
    gps_t, gps_speed = horizontal_speed(gps.rel_s, gps.enu[:, :2], resample_dt, align_window[1] + 30.0, smooth_s)
    tum_t, tum_speed = horizontal_speed(tum_rel, tum.p[:, :2], resample_dt, align_window[1] + scan_max + 30.0, smooth_s)

    gps_mask = (gps_t >= align_window[0]) & (gps_t <= align_window[1])
    if int(gps_mask.sum()) < 20:
        raise ValueError("Not enough GPS samples in alignment window")

    scan_rows: list[dict[str, float]] = []
    offsets = np.arange(scan_min, scan_max + 0.5 * scan_step, scan_step, dtype=np.float64)
    for offset in offsets:
        query_tum = gps_t[gps_mask] + float(offset)
        valid = (query_tum >= tum_t[0]) & (query_tum <= tum_t[-1])
        if int(valid.sum()) < 20:
            score = float("nan")
            corr = float("nan")
        else:
            a = gps_speed[gps_mask][valid]
            b = np.interp(query_tum[valid], tum_t, tum_speed)
            score, corr = normalized_rmse_and_corr(a, b)
        scan_rows.append(
            {
                "gps_start_offset_from_tum0_s": float(offset),
                "norm_speed_rmse": score,
                "norm_speed_corr": corr,
                "pairs": int(valid.sum()),
            }
        )

    finite_rows = [r for r in scan_rows if math.isfinite(r["norm_speed_rmse"])]
    if not finite_rows:
        raise ValueError("No finite offset candidate found")
    best = min(finite_rows, key=lambda r: r["norm_speed_rmse"])
    return float(best["gps_start_offset_from_tum0_s"]), scan_rows


def write_scan_csv(path: Path, rows: list[dict[str, float]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["gps_start_offset_from_tum0_s", "norm_speed_rmse", "norm_speed_corr", "pairs"],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_aligned_gps(path: Path, gps: GpsRaw, first_timestamp_ns: int) -> tuple[int, int]:
    first_ns = 0
    last_ns = 0
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["#timestamp_ns", "lat_deg", "lon_deg", "alt_m"])
        for time_us, lat, lon, alt in zip(gps.time_us, gps.lat, gps.lon, gps.alt):
            ts_ns = first_timestamp_ns + int(time_us - gps.time_us[0]) * 1000
            if first_ns == 0:
                first_ns = ts_ns
            last_ns = ts_ns
            writer.writerow([ts_ns, f"{lat:.10f}", f"{lon:.10f}", f"{alt:.3f}"])
    return first_ns, last_ns


def crop_tum(tum: TumTraj, max_rel_s: float | None) -> TumTraj:
    if max_rel_s is None or max_rel_s <= 0:
        return tum
    keep = (tum.t - tum.t[0]) <= max_rel_s
    if int(keep.sum()) < 20:
        raise ValueError("truth-max-rel-s leaves too few TUM rows")
    return TumTraj(t_ns=tum.t_ns[keep], t=tum.t[keep], p=tum.p[keep], q_xyzw=tum.q_xyzw[keep])


def write_tum(path: Path, tum: TumTraj, time_shift_s: float, note: str) -> tuple[float, float]:
    shift_ns = int(round(time_shift_s * NS_PER_SEC))
    with path.open("w", newline="\n") as f:
        f.write(f"# TUM truth ({note})\n")
        f.write("# t x y z qx qy qz qw\n")
        for t_ns, p, q in zip(tum.t_ns, tum.p, tum.q_xyzw):
            ts_text = ns_to_sec_text(int(t_ns) + shift_ns)
            f.write(
                f"{ts_text} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n"
            )
    return float((int(tum.t_ns[0]) + shift_ns) * 1e-9), float((int(tum.t_ns[-1]) + shift_ns) * 1e-9)


def compute_velocities(t: np.ndarray, p: np.ndarray) -> np.ndarray:
    v = np.zeros_like(p)
    if len(t) < 2:
        return v
    eps = 1e-9
    for i in range(len(t)):
        if i == 0:
            j0, j1 = 0, 1
        elif i == len(t) - 1:
            j0, j1 = len(t) - 2, len(t) - 1
        else:
            j0, j1 = i - 1, i + 1
        dt = float(t[j1] - t[j0])
        if abs(dt) > eps:
            v[i] = (p[j1] - p[j0]) / dt
    return v


def write_asl_gt(path: Path, tum: TumTraj, time_shift_s: float) -> tuple[int, int]:
    shift_ns = int(round(time_shift_s * NS_PER_SEC))
    t = (tum.t_ns.astype(np.float64) + float(shift_ns)) * 1e-9
    v = compute_velocities(t, tum.p)
    first_ns = int(tum.t_ns[0]) + shift_ns
    last_ns = int(tum.t_ns[-1]) + shift_ns
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "#timestamp [ns]",
                "p_RS_R_x [m]",
                "p_RS_R_y [m]",
                "p_RS_R_z [m]",
                "q_RS_w []",
                "q_RS_x []",
                "q_RS_y []",
                "q_RS_z []",
                "v_RS_R_x [m s^-1]",
                "v_RS_R_y [m s^-1]",
                "v_RS_R_z [m s^-1]",
                "b_w_RS_S_x [rad s^-1]",
                "b_w_RS_S_y [rad s^-1]",
                "b_w_RS_S_z [rad s^-1]",
                "b_a_RS_S_x [m s^-2]",
                "b_a_RS_S_y [m s^-2]",
                "b_a_RS_S_z [m s^-2]",
            ]
        )
        for ti_ns, p, q, vi in zip(tum.t_ns + shift_ns, tum.p, tum.q_xyzw, v):
            writer.writerow(
                [
                    int(ti_ns),
                    f"{p[0]:.9f}",
                    f"{p[1]:.9f}",
                    f"{p[2]:.9f}",
                    f"{q[3]:.9f}",
                    f"{q[0]:.9f}",
                    f"{q[1]:.9f}",
                    f"{q[2]:.9f}",
                    f"{vi[0]:.9f}",
                    f"{vi[1]:.9f}",
                    f"{vi[2]:.9f}",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                ]
            )
    return first_ns, last_ns


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gps", type=Path, default=Path("20260509_fly1/mav0/gps/GPS.csv"))
    parser.add_argument("--imu", type=Path, default=Path("20260509_fly1/mav0/imu0/data.csv"))
    parser.add_argument("--tum", type=Path, default=Path(r"C:/Users/baloney/Downloads/traj_tum.txt"))
    parser.add_argument("--out-dir", type=Path, default=Path("20260509_fly1/result/gps_tum_time_alignment"))
    parser.add_argument("--min-status", type=int, default=4)
    parser.add_argument("--scan-min", type=float, default=-20.0)
    parser.add_argument("--scan-max", type=float, default=80.0)
    parser.add_argument("--scan-step", type=float, default=0.01)
    parser.add_argument("--align-window-start", type=float, default=0.0)
    parser.add_argument("--align-window-end", type=float, default=160.0)
    parser.add_argument("--resample-dt", type=float, default=0.05)
    parser.add_argument("--smooth-s", type=float, default=3.0)
    parser.add_argument(
        "--fixed-gps-start-offset",
        type=float,
        default=None,
        help="Use this GPS-start offset from first TUM timestamp instead of the scanned best.",
    )
    parser.add_argument("--timeshift-cam-imu", type=float, default=0.013080436567075273)
    parser.add_argument(
        "--truth-max-rel-s",
        type=float,
        default=540.0,
        help="Crop truth before known late drift. Use <=0 to keep full trajectory.",
    )
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    gps = read_raw_gps(args.gps, args.min_status)
    tum_full = read_tum(args.tum)
    tum_truth = crop_tum(tum_full, args.truth_max_rel_s)
    imu_first_ns, imu_last_ns = read_imu_span(args.imu)

    scanned_best_offset, scan_rows = estimate_gps_start_offset(
        gps=gps,
        tum=tum_full,
        scan_min=args.scan_min,
        scan_max=args.scan_max,
        scan_step=args.scan_step,
        align_window=(args.align_window_start, args.align_window_end),
        resample_dt=args.resample_dt,
        smooth_s=args.smooth_s,
    )
    best_offset = args.fixed_gps_start_offset if args.fixed_gps_start_offset is not None else scanned_best_offset

    shift_ns = int(round(args.timeshift_cam_imu * NS_PER_SEC))
    gps_first_cam_ns = int(tum_full.t_ns[0]) + int(round(best_offset * NS_PER_SEC))
    gps_first_imu_ns = gps_first_cam_ns + shift_ns

    # raw GPS TimeUS to camera-time OpenVINS seconds:
    #   t_gps_cam = TimeUS * 1e-6 + gps_abs_offset_cam_s
    gps_abs_offset_cam_s = float(gps_first_cam_ns * 1e-9 - int(gps.time_us[0]) * 1e-6)
    gps_abs_offset_imu_s = float(gps_first_imu_ns * 1e-9 - int(gps.time_us[0]) * 1e-6)

    scan_csv = args.out_dir / "offset_scan.csv"
    write_scan_csv(scan_csv, scan_rows)

    gps_cam_csv = args.out_dir / "aligned_gps_cam_time.csv"
    gps_imu_csv = args.out_dir / "aligned_gps_imu_time.csv"
    gps_cam_span = write_aligned_gps(gps_cam_csv, gps, gps_first_cam_ns)
    gps_imu_span = write_aligned_gps(gps_imu_csv, gps, gps_first_imu_ns)

    tum_cam = args.out_dir / "truth_tum_cam_time.txt"
    tum_imu = args.out_dir / "truth_tum_imu_time.txt"
    tum_cam_span = write_tum(tum_cam, tum_truth, 0.0, "camera/OpenVINS output time")
    tum_imu_span = write_tum(tum_imu, tum_truth, args.timeshift_cam_imu, "IMU time = camera time + timeshift_cam_imu")

    asl_cam = args.out_dir / "truth_asl_cam_time.csv"
    asl_imu = args.out_dir / "truth_asl_imu_time.csv"
    asl_cam_span = write_asl_gt(asl_cam, tum_truth, 0.0)
    asl_imu_span = write_asl_gt(asl_imu, tum_truth, args.timeshift_cam_imu)

    summary = {
        "inputs": {
            "gps": str(args.gps),
            "imu": str(args.imu),
            "tum": str(args.tum),
        },
        "method": {
            "alignment": "horizontal_speed_profile_correlation",
            "gps_time_model_camera_time": "t_gps_cam = TimeUS * 1e-6 + gps_abs_offset_cam_s",
            "gps_time_model_imu_time": "t_gps_imu = TimeUS * 1e-6 + gps_abs_offset_imu_s",
            "align_window_gps_rel_s": [args.align_window_start, args.align_window_end],
            "resample_dt_s": args.resample_dt,
            "smooth_s": args.smooth_s,
            "scan_step_s": args.scan_step,
            "truth_max_rel_s": args.truth_max_rel_s,
            "fixed_gps_start_offset_s": args.fixed_gps_start_offset,
        },
        "calibration": {
            "timeshift_cam_imu_s": args.timeshift_cam_imu,
            "timeshift_cam_imu_ns": shift_ns,
            "scanned_best_gps_start_offset_from_tum0_s": scanned_best_offset,
            "best_gps_start_offset_from_tum0_s": best_offset,
            "gps_abs_offset_cam_s": gps_abs_offset_cam_s,
            "gps_abs_offset_imu_s": gps_abs_offset_imu_s,
            "gps_first_timeus_s": float(int(gps.time_us[0]) * 1e-6),
            "gps_first_cam_time_ns": gps_first_cam_ns,
            "gps_first_imu_time_ns": gps_first_imu_ns,
            "gps_first_cam_time_s": float(gps_first_cam_ns * 1e-9),
            "gps_first_imu_time_s": float(gps_first_imu_ns * 1e-9),
            "tum0_camera_time_ns": int(tum_full.t_ns[0]),
            "tum0_imu_time_ns": int(tum_full.t_ns[0]) + shift_ns,
            "tum0_camera_time_s": float(tum_full.t[0]),
            "tum0_imu_time_s": float((int(tum_full.t_ns[0]) + shift_ns) * 1e-9),
        },
        "spans": {
            "imu_ns": [imu_first_ns, imu_last_ns],
            "gps_cam_ns": list(gps_cam_span),
            "gps_imu_ns": list(gps_imu_span),
            "truth_tum_cam_s": list(tum_cam_span),
            "truth_tum_imu_s": list(tum_imu_span),
            "truth_asl_cam_ns": list(asl_cam_span),
            "truth_asl_imu_ns": list(asl_imu_span),
        },
        "outputs": {
            "offset_scan_csv": str(scan_csv),
            "aligned_gps_cam_time_csv": str(gps_cam_csv),
            "aligned_gps_imu_time_csv": str(gps_imu_csv),
            "truth_tum_cam_time": str(tum_cam),
            "truth_tum_imu_time": str(tum_imu),
            "truth_asl_cam_time": str(asl_cam),
            "truth_asl_imu_time": str(asl_imu),
        },
    }
    summary_path = args.out_dir / "alignment_summary.json"
    with summary_path.open("w") as f:
        json.dump(summary, f, indent=2)

    print(f"best_gps_start_offset_from_tum0_s: {best_offset:.6f}")
    print(f"gps_abs_offset_cam_s: {gps_abs_offset_cam_s:.9f}")
    print(f"gps_abs_offset_imu_s: {gps_abs_offset_imu_s:.9f}")
    print(f"wrote: {summary_path}")


if __name__ == "__main__":
    main()
