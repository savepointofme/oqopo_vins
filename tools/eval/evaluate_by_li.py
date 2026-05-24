#!/usr/bin/env python3
"""Evaluate one or more odometry trajectories against GPS ground truth.

Features requested:
- Hard time alignment at takeoff point (GPS takeoff time == traj first timestamp)
- Fixed-scale evaluation only (scale = 1, SE3)
- Evaluate multiple traj files together and compare in one report
- Output final X%D where D is traveled distance (GPS mileage)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np


# WGS-84 constants
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2 - _F)


@dataclass
class GpsRaw:
    time_us: np.ndarray
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray
    vz: np.ndarray | None
    status: np.ndarray | None


@dataclass
class Trajectory:
    t_abs: np.ndarray  # absolute timestamp [s]
    p: np.ndarray  # [N,3]


@dataclass
class AlignResult:
    rot: np.ndarray  # [3,3]
    trans: np.ndarray  # [3]


def geodetic_to_ecef(lat_deg: np.ndarray, lon_deg: np.ndarray, alt_m: np.ndarray) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    N = _A / np.sqrt(1.0 - _E2 * sin_lat * sin_lat)

    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N * (1.0 - _E2) + alt_m) * sin_lat
    return np.column_stack((x, y, z))


def ecef_to_enu(ecef: np.ndarray, ref_lat_deg: float, ref_lon_deg: float, ref_ecef: np.ndarray) -> np.ndarray:
    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)

    slat, clat = math.sin(lat0), math.cos(lat0)
    slon, clon = math.sin(lon0), math.cos(lon0)

    R = np.array(
        [
            [-slon, clon, 0.0],
            [-slat * clon, -slat * slon, clat],
            [clat * clon, clat * slon, slat],
        ],
        dtype=np.float64,
    )
    d = ecef - ref_ecef.reshape(1, 3)
    return d @ R.T


def load_gps_raw(path: Path, min_status: int = 4) -> GpsRaw:
    rows: List[Tuple[float, float, float, float, float | None, int | None]] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        fields = set(reader.fieldnames or [])
        required = {"TimeUS", "Lat", "Lng", "Alt"}
        missing = required - fields
        if missing:
            raise ValueError(f"GPS.csv missing columns: {sorted(missing)}")

        has_status = "Status" in fields
        has_vz = "VZ" in fields

        for r in reader:
            try:
                status = int(float(r["Status"])) if has_status else None
                if has_status and min_status is not None and status is not None and status < min_status:
                    continue
                vz = float(r["VZ"]) if has_vz else None
                rows.append(
                    (
                        float(r["TimeUS"]),
                        float(r["Lat"]),
                        float(r["Lng"]),
                        float(r["Alt"]),
                        vz,
                        status,
                    )
                )
            except (ValueError, KeyError):
                continue

    if len(rows) < 10:
        raise ValueError("Not enough valid GPS rows after filtering.")

    arr = np.array(rows, dtype=object)
    time_us = arr[:, 0].astype(np.float64)
    lat = arr[:, 1].astype(np.float64)
    lon = arr[:, 2].astype(np.float64)
    alt = arr[:, 3].astype(np.float64)

    vz = None
    if arr[0, 4] is not None:
        vz = np.array([float(v) if v is not None else np.nan for v in arr[:, 4]], dtype=np.float64)

    status = None
    if arr[0, 5] is not None:
        status = arr[:, 5].astype(np.int32)

    return GpsRaw(time_us=time_us, lat=lat, lon=lon, alt=alt, vz=vz, status=status)


def detect_takeoff_idx(gps: GpsRaw) -> int:
    alt_diff = np.diff(gps.alt, prepend=gps.alt[0])

    for i in range(1, gps.time_us.size):
        cond_vz = False
        if gps.vz is not None and np.isfinite(gps.vz[i]):
            cond_vz = gps.vz[i] < -0.5
        cond_alt = (alt_diff[i] > 0.1) and (gps.alt[i] > 35.0)
        if cond_vz or cond_alt:
            return i
    return 0


def load_vio_traj(path: Path) -> Trajectory:
    rows = []
    with path.open("r") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 4:
                continue
            try:
                ts = float(parts[0])
                tx, ty, tz = map(float, parts[1:4])
                rows.append((ts, tx, ty, tz))
            except ValueError:
                continue

    if len(rows) < 10:
        raise ValueError(f"Not enough valid VIO points in {path}")

    arr = np.array(rows, dtype=np.float64)
    return Trajectory(t_abs=arr[:, 0], p=arr[:, 1:4])


def interp_positions(src_t: np.ndarray, src_p: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    out = np.empty((dst_t.shape[0], 3), dtype=np.float64)
    for i in range(3):
        out[:, i] = np.interp(dst_t, src_t, src_p[:, i])
    return out


def align_se3_unit_scale(src: np.ndarray, dst: np.ndarray) -> AlignResult:
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("src and dst must both be Nx3")

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)

    src0 = src - mu_src
    dst0 = dst - mu_dst

    cov = (dst0.T @ src0) / src.shape[0]
    U, _, Vt = np.linalg.svd(cov)

    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1.0

    R = U @ D @ Vt
    t = mu_dst - (R @ mu_src)
    return AlignResult(rot=R, trans=t)


def apply_alignment(p: np.ndarray, align: AlignResult) -> np.ndarray:
    return (align.rot @ p.T).T + align.trans.reshape(1, 3)


def path_length_m(p: np.ndarray) -> float:
    if p.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def stats_ate(gt: np.ndarray, est: np.ndarray) -> dict:
    err = np.linalg.norm(est - gt, axis=1)
    return {
        "count": int(err.size),
        "rmse_m": float(np.sqrt(np.mean(err**2))),
        "mean_m": float(np.mean(err)),
        "median_m": float(np.median(err)),
        "max_m": float(np.max(err)),
        "std_m": float(np.std(err)),
    }


def stats_up(gt: np.ndarray, est: np.ndarray) -> dict:
    # Height-direction error on local Up axis (z).
    err = est[:, 2] - gt[:, 2]
    abs_err = np.abs(err)
    return {
        "rmse_m": float(np.sqrt(np.mean(err**2))),
        "mae_m": float(np.mean(abs_err)),
        "bias_m": float(np.mean(err)),
        "max_abs_m": float(np.max(abs_err)),
        "std_m": float(np.std(err)),
    }


def gps_window_to_local(
    gps: GpsRaw,
    takeoff_idx: int,
    duration_s: float,
    axis: str,
) -> Tuple[np.ndarray, np.ndarray, dict]:
    t0_us = gps.time_us[takeoff_idx]
    if duration_s > 0:
        t1_us = t0_us + duration_s * 1e6
        mask = (gps.time_us >= t0_us) & (gps.time_us <= t1_us)
    else:
        mask = gps.time_us >= t0_us

    t_us = gps.time_us[mask]
    lat = gps.lat[mask]
    lon = gps.lon[mask]
    alt = gps.alt[mask]

    if t_us.size < 10:
        raise ValueError("Not enough GPS points in selected takeoff window.")

    ecef = geodetic_to_ecef(lat, lon, alt)
    enu = ecef_to_enu(ecef, lat[0], lon[0], ecef[0])
    if axis.lower() == "neu":
        p = np.column_stack((enu[:, 1], enu[:, 0], enu[:, 2]))
    else:
        p = enu

    meta = {
        "takeoff_idx": int(takeoff_idx),
        "takeoff_time_us": float(t0_us),
        "num_gps_window_points": int(t_us.size),
        "gps_window_duration_s": float((t_us[-1] - t_us[0]) / 1e6),
        "axis": axis.lower(),
    }
    return t_us, p, meta


def evaluate_one(
    gps_time_us_win: np.ndarray,
    gps_pos_win: np.ndarray,
    takeoff_time_us: float,
    traj: Trajectory,
) -> dict:
    # Hard alignment: GPS takeoff timestamp is set to trajectory first timestamp.
    time_offset = traj.t_abs[0] - (takeoff_time_us / 1e6)
    gps_t_abs = gps_time_us_win / 1e6 + time_offset

    t0 = max(gps_t_abs.min(), traj.t_abs.min())
    t1 = min(gps_t_abs.max(), traj.t_abs.max())
    if t1 <= t0:
        raise ValueError("No temporal overlap after hard takeoff alignment.")

    mask = (gps_t_abs >= t0) & (gps_t_abs <= t1)
    t_eval = gps_t_abs[mask]
    gt_eval = gps_pos_win[mask]

    if t_eval.size < 10:
        raise ValueError("Not enough overlap samples.")

    est_interp = interp_positions(traj.t_abs, traj.p, t_eval)

    align = align_se3_unit_scale(est_interp, gt_eval)
    est_aligned = apply_alignment(est_interp, align)

    ate = stats_ate(gt_eval, est_aligned)
    up = stats_up(gt_eval, est_aligned)
    D = path_length_m(gt_eval)
    pct_D = (ate["rmse_m"] / D * 100.0) if D > 1e-9 else float("nan")

    return {
        "ate": ate,
        "up": up,
        "D_m": D,
        "rmse_over_D_percent": pct_D,
        "time_span_s": float(t_eval[-1] - t_eval[0]),
        "num_eval_points": int(t_eval.size),
        "t_eval_abs": t_eval,
        "aligned_traj": est_aligned,
        "gt_eval": gt_eval,
    }


def make_compare_plot(
    gt_xy: np.ndarray,
    gt_t_rel: np.ndarray,
    gt_z: np.ndarray,
    aligned_xy_list: List[Tuple[str, np.ndarray]],
    height_series: List[Tuple[str, np.ndarray, np.ndarray]],
    out_png: Path,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # Left: XY trajectory
    axes[0].plot(gt_xy[:, 0], gt_xy[:, 1], "k-", lw=2.2, label="GPS (GT)")
    for name, p in aligned_xy_list:
        axes[0].plot(p[:, 0], p[:, 1], lw=1.7, label=name)
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].set_title("Trajectory Comparison (Aligned, Unit Scale)")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    # Right: Height over time
    axes[1].plot(gt_t_rel, gt_z, "k-", lw=2.2, label="GPS Up")
    for name, t_rel, z_est in height_series:
        axes[1].plot(t_rel, z_est, lw=1.6, label=f"{name} Up")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Up [m]")
    axes[1].set_title("Height Comparison")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate odometry accuracy against GPS (hard takeoff alignment)")
    parser.add_argument("--gps", type=Path, default=Path("GPS.csv"), help="Path to GPS.csv")
    parser.add_argument(
        "--traj",
        type=Path,
        nargs="+",
        default=[Path("stereo_baseline_traj_estimate_format.txt"),Path("traj_estimate_stereo.txt"),],
        help="One or more trajectory files to compare",
    )
    parser.add_argument("--out", type=Path, default=Path("eval_output"), help="Output directory")
    parser.add_argument("--min-status", type=int, default=4, help="Minimum GPS status to keep")
    parser.add_argument("--duration-s", type=float, default=500,help="GPS window duration from takeoff (<=0 means till end)")
    parser.add_argument("--axis", choices=["enu", "neu"], default="enu", help="GPS local axis order")
    args = parser.parse_args()

    gps = load_gps_raw(args.gps, min_status=args.min_status)
    takeoff_idx = detect_takeoff_idx(gps)
    gps_t_us_win, gps_p_win, gps_meta = gps_window_to_local(
        gps,
        takeoff_idx=takeoff_idx,
        duration_s=args.duration_s,
        axis=args.axis,
    )

    args.out.mkdir(parents=True, exist_ok=True)

    summary = {
        "config": {
            "gps_file": str(args.gps),
            "traj_files": [str(p) for p in args.traj],
            "min_status": args.min_status,
            "duration_s": args.duration_s,
            "axis": args.axis,
            "scale_mode": "fixed_1",
            "time_alignment": "hard_takeoff",
        },
        "gps_meta": gps_meta,
        "results": [],
    }

    aligned_for_plot: List[Tuple[str, np.ndarray]] = []
    height_for_plot: List[Tuple[str, np.ndarray, np.ndarray]] = []
    gt_for_plot = None
    gt_t_for_plot = None
    gt_z_for_plot = None

    for traj_path in args.traj:
        traj = load_vio_traj(traj_path)
        res = evaluate_one(gps_t_us_win, gps_p_win, gps_meta["takeoff_time_us"], traj)

        result_item = {
            "traj_file": str(traj_path),
            "count": res["ate"]["count"],
            "time_span_s": res["time_span_s"],
            "D_m": res["D_m"],
            "ATE_RMSE_m": res["ate"]["rmse_m"],
            "ATE_mean_m": res["ate"]["mean_m"],
            "ATE_median_m": res["ate"]["median_m"],
            "ATE_max_m": res["ate"]["max_m"],
            "ATE_std_m": res["ate"]["std_m"],
            "Up_RMSE_m": res["up"]["rmse_m"],
            "Up_MAE_m": res["up"]["mae_m"],
            "Up_Bias_m": res["up"]["bias_m"],
            "Up_MaxAbs_m": res["up"]["max_abs_m"],
            "Up_STD_m": res["up"]["std_m"],
            "percent_D": res["rmse_over_D_percent"],
            "final_metric": f"{res['rmse_over_D_percent']:.3f}%D",
        }
        summary["results"].append(result_item)

        name = traj_path.stem
        aligned_for_plot.append((name, res["aligned_traj"]))
        t_rel = res["t_eval_abs"] - res["t_eval_abs"][0]
        height_for_plot.append((name, t_rel, res["aligned_traj"][:, 2]))
        if gt_for_plot is None:
            gt_for_plot = res["gt_eval"]
            gt_t_for_plot = t_rel
            gt_z_for_plot = res["gt_eval"][:, 2]

    summary["results"].sort(key=lambda x: x["ATE_RMSE_m"])

    json_path = args.out / "metrics_compare.json"
    csv_path = args.out / "metrics_compare.csv"
    plot_path = args.out / "trajectory_compare_multi.png"

    with json_path.open("w") as f:
        json.dump(summary, f, indent=2)

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "traj_file",
            "count",
            "time_span_s",
            "D_m",
            "ATE_RMSE_m",
            "ATE_mean_m",
            "ATE_median_m",
            "ATE_max_m",
            "ATE_std_m",
            "Up_RMSE_m",
            "Up_MAE_m",
            "Up_Bias_m",
            "Up_MaxAbs_m",
            "Up_STD_m",
            "percent_D",
            "final_metric",
        ])
        for r in summary["results"]:
            writer.writerow([
                r["traj_file"],
                r["count"],
                f"{r['time_span_s']:.6f}",
                f"{r['D_m']:.6f}",
                f"{r['ATE_RMSE_m']:.6f}",
                f"{r['ATE_mean_m']:.6f}",
                f"{r['ATE_median_m']:.6f}",
                f"{r['ATE_max_m']:.6f}",
                f"{r['ATE_std_m']:.6f}",
                f"{r['Up_RMSE_m']:.6f}",
                f"{r['Up_MAE_m']:.6f}",
                f"{r['Up_Bias_m']:.6f}",
                f"{r['Up_MaxAbs_m']:.6f}",
                f"{r['Up_STD_m']:.6f}",
                f"{r['percent_D']:.6f}",
                r["final_metric"],
            ])

    if gt_for_plot is not None and aligned_for_plot and gt_t_for_plot is not None and gt_z_for_plot is not None:
        make_compare_plot(
            gt_xy=gt_for_plot,
            gt_t_rel=gt_t_for_plot,
            gt_z=gt_z_for_plot,
            aligned_xy_list=aligned_for_plot,
            height_series=height_for_plot,
            out_png=plot_path,
        )

    print("Evaluation finished.")
    print(f"Takeoff idx: {gps_meta['takeoff_idx']}, takeoff TimeUS: {gps_meta['takeoff_time_us']:.0f}")
    print(f"Metrics JSON: {json_path}")
    print(f"Metrics CSV:  {csv_path}")
    print(f"Plot:         {plot_path}")
    print("\nComparison (sorted by ATE RMSE):")
    for i, r in enumerate(summary["results"], 1):
        print(
            f"{i}. {Path(r['traj_file']).name}: "
            f"RMSE={r['ATE_RMSE_m']:.3f} m, UpRMSE={r['Up_RMSE_m']:.3f} m, D={r['D_m']:.3f} m, "
            f"Final={r['final_metric']}"
        )


if __name__ == "__main__":
    main()
