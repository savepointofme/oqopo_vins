#!/usr/bin/env python3
"""Compare VIO yaw, raw gyro yaw, bias-corrected gyro yaw, and GPS course yaw."""

from __future__ import annotations

import argparse
import csv
import math
from bisect import bisect_left
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EARTH_R = 6378137.0


def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0.0 else q


def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=float,
    )


def q_exp_rotvec(rv: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return q_norm(np.array([0.5 * rv[0], 0.5 * rv[1], 0.5 * rv[2], 1.0], dtype=float))
    axis = rv / theta
    s = math.sin(0.5 * theta)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(0.5 * theta)], dtype=float)


def yaw_from_q_body_to_world(q: np.ndarray) -> float:
    qx, qy, qz, qw = q_norm(q)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def interp_scalar(ts: np.ndarray, ys: np.ndarray, t: float) -> float:
    i = bisect_left(ts, t)
    if i <= 0:
        return float(ys[0])
    if i >= len(ts):
        return float(ys[-1])
    a, b = ts[i - 1], ts[i]
    u = (t - a) / (b - a) if b > a else 0.0
    return float((1.0 - u) * ys[i - 1] + u * ys[i])


def interp_vec(ts: np.ndarray, vals: np.ndarray, t: float) -> np.ndarray:
    if len(ts) == 0:
        return np.zeros(3)
    i = bisect_left(ts, t)
    if i <= 0:
        return vals[0]
    if i >= len(ts):
        return vals[-1]
    a, b = ts[i - 1], ts[i]
    u = (t - a) / (b - a) if b > a else 0.0
    return (1.0 - u) * vals[i - 1] + u * vals[i]


def unwrap_deg(yaw_deg: list[float] | np.ndarray) -> np.ndarray:
    return np.rad2deg(np.unwrap(np.deg2rad(np.asarray(yaw_deg, dtype=float))))


def load_traj(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ts, qs, pos = [], [], []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            if len(cols) < 8:
                continue
            vals = [float(x) for x in cols[:8]]
            ts.append(vals[0])
            pos.append(vals[1:4])
            qs.append(vals[4:8])
    if not ts:
        raise RuntimeError(f"no trajectory samples in {path}")
    yaw = unwrap_deg([math.degrees(yaw_from_q_body_to_world(np.asarray(q))) for q in qs])
    return np.asarray(ts), yaw, np.asarray(pos)


def load_bias(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.exists():
        return np.empty(0), np.empty((0, 3))
    ts, bgs = [], []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            if len(cols) < 7:
                continue
            ts.append(float(cols[0]))
            bgs.append([float(cols[4]), float(cols[5]), float(cols[6])])
    return np.asarray(ts), np.asarray(bgs, dtype=float)


def load_imu(path: Path, t0: float, t1: float) -> tuple[np.ndarray, np.ndarray]:
    ts, ws = [], []
    with path.open("r", newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["#t_rel_s"])
            if t < t0:
                continue
            if t > t1:
                break
            ts.append(t)
            ws.append([float(row["wx"]), float(row["wy"]), float(row["wz"])])
    if len(ts) < 2:
        raise RuntimeError(f"not enough IMU samples in {path}")
    return np.asarray(ts), np.asarray(ws)


def integrate_gyro_yaw(
    imu_t: np.ndarray,
    gyro: np.ndarray,
    bias_t: np.ndarray,
    bias_g: np.ndarray,
    yaw0_deg: float,
    use_bias: bool,
) -> tuple[np.ndarray, np.ndarray]:
    q = np.array([0.0, 0.0, math.sin(math.radians(yaw0_deg) * 0.5), math.cos(math.radians(yaw0_deg) * 0.5)])
    out_t = [float(imu_t[0])]
    out_yaw = [yaw0_deg]
    t_prev = float(imu_t[0])
    for t_raw, w_raw in zip(imu_t[1:], gyro[1:]):
        t = float(t_raw)
        dt = t - t_prev
        t_prev = t
        if dt <= 0.0 or dt > 0.1:
            out_t.append(t)
            out_yaw.append(out_yaw[-1])
            continue
        bg = interp_vec(bias_t, bias_g, t) if use_bias else np.zeros(3)
        q = q_norm(q_mul(q, q_exp_rotvec((w_raw - bg) * dt)))
        out_t.append(t)
        out_yaw.append(math.degrees(yaw_from_q_body_to_world(q)))
    return np.asarray(out_t), unwrap_deg(out_yaw)


def load_gps_course(
    path: Path,
    t0: float,
    t1: float,
    window_s: float,
    gps_time_offset: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rows = []
    with path.open("r", newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["ts_ns"]) * 1e-9 + gps_time_offset
            if t < t0 - window_s - 2.0:
                continue
            if t > t1 + window_s + 2.0:
                break
            rows.append((t, float(row["lat"]), float(row["lon"])))
    if len(rows) < 3:
        raise RuntimeError(f"not enough GPS samples in {path}")

    lat0 = math.radians(rows[0][1])
    lon0 = math.radians(rows[0][2])
    cos_lat = math.cos(lat0)
    ts = np.asarray([r[0] for r in rows])
    east = np.asarray([(math.radians(r[2]) - lon0) * cos_lat * EARTH_R for r in rows])
    north = np.asarray([(math.radians(r[1]) - lat0) * EARTH_R for r in rows])

    course_t, course, speed = [], [], []
    half = window_s * 0.5
    for t in ts:
        if t < t0 or t > t1:
            continue
        j0 = max(0, bisect_left(ts, t - half))
        j1 = min(len(ts) - 1, bisect_left(ts, t + half))
        if j1 <= j0:
            continue
        dt = ts[j1] - ts[j0]
        if dt <= 0.0:
            continue
        de = east[j1] - east[j0]
        dn = north[j1] - north[j0]
        v = math.hypot(de, dn) / dt
        course_t.append(t)
        course.append(math.degrees(math.atan2(dn, de)))
        speed.append(v)
    if len(course_t) < 2:
        raise RuntimeError("not enough valid GPS course samples")
    return np.asarray(course_t), unwrap_deg(course), np.asarray(speed), east, north


def align_to_reference(y: np.ndarray, ref: np.ndarray) -> np.ndarray:
    return y + (ref[0] - y[0])


def residual_stats(err: np.ndarray) -> dict[str, float]:
    if len(err) == 0:
        return {
            "samples": 0,
            "mean_deg": math.nan,
            "mean_abs_deg": math.nan,
            "rms_deg": math.nan,
            "p50_abs_deg": math.nan,
            "p90_abs_deg": math.nan,
            "final_deg": math.nan,
            "drift_deg": math.nan,
        }
    abs_err = np.abs(err)
    return {
        "samples": int(len(err)),
        "mean_deg": float(np.mean(err)),
        "mean_abs_deg": float(np.mean(abs_err)),
        "rms_deg": float(np.sqrt(np.mean(err * err))),
        "p50_abs_deg": float(np.percentile(abs_err, 50)),
        "p90_abs_deg": float(np.percentile(abs_err, 90)),
        "final_deg": float(err[-1]),
        "drift_deg": float(err[-1] - err[0]),
    }


def write_metrics(path: Path, rows: list[dict[str, object]]) -> None:
    fields = [
        "speed_threshold_mps",
        "comparison",
        "samples",
        "mean_deg",
        "mean_abs_deg",
        "rms_deg",
        "p50_abs_deg",
        "p90_abs_deg",
        "final_deg",
        "drift_deg",
        "t_first_s",
        "t_last_s",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_timeseries(path: Path, data: dict[str, np.ndarray]) -> None:
    fields = list(data.keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for vals in zip(*(data[k] for k in fields)):
            writer.writerow([f"{float(v):.9f}" for v in vals])


def plot_report(
    path: Path,
    run_name: str,
    t: np.ndarray,
    gps_y: np.ndarray,
    speed: np.ndarray,
    vio_y: np.ndarray,
    raw_y: np.ndarray,
    bias_y: np.ndarray,
    valid_mask: np.ndarray,
    bias_t: np.ndarray,
    bg_z: np.ndarray,
    threshold: float,
) -> None:
    rel_t = t - t[0]
    vio_gps = vio_y - gps_y
    raw_gps = raw_y - gps_y
    bias_gps = bias_y - gps_y
    vio_bias = vio_y - bias_y

    fig, axs = plt.subplots(5, 1, figsize=(14, 13), sharex=False, constrained_layout=True)
    fig.suptitle(f"Yaw comparison, {run_name} (GPS course metrics speed > {threshold:g} m/s)")

    axs[0].plot(rel_t, gps_y, label="GPS course", color="tab:green", linewidth=1.5)
    axs[0].plot(rel_t, vio_y, label="VIO yaw aligned to GPS", color="tab:blue", linewidth=1.1)
    axs[0].plot(rel_t, raw_y, label="raw gyro integrated yaw aligned to GPS", color="tab:red", linewidth=1.0)
    axs[0].plot(rel_t, bias_y, label="bias-corrected gyro yaw aligned to GPS", color="tab:orange", linewidth=1.0)
    axs[0].scatter(rel_t[valid_mask], gps_y[valid_mask], s=8, color="black", alpha=0.25, label="metric samples")
    axs[0].set_ylabel("heading deg")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="best", ncol=2)

    axs[1].plot(rel_t, vio_gps, label="VIO - GPS", color="tab:blue")
    axs[1].plot(rel_t, raw_gps, label="raw gyro - GPS", color="tab:red")
    axs[1].plot(rel_t, bias_gps, label="bias gyro - GPS", color="tab:orange")
    axs[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axs[1].set_ylabel("GPS delta deg")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="best")

    axs[2].plot(rel_t, vio_bias, label="VIO - bias-corrected gyro", color="tab:purple")
    axs[2].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axs[2].set_ylabel("delta deg")
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(loc="best")

    axs[3].plot(rel_t, speed, label="GPS horizontal speed", color="tab:gray")
    axs[3].axhline(threshold, color="black", linestyle="--", linewidth=0.9, alpha=0.7, label=f"{threshold:g} m/s")
    axs[3].set_ylabel("m/s")
    axs[3].grid(True, alpha=0.3)
    axs[3].legend(loc="best")

    if len(bias_t) > 0:
        axs[4].plot(bias_t - t[0], bg_z, label="bg_z", color="tab:brown")
    axs[4].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axs[4].set_ylabel("rad/s")
    axs[4].set_xlabel("time since first GPS course sample (s)")
    axs[4].grid(True, alpha=0.3)
    axs[4].legend(loc="best")

    fig.savefig(path, dpi=160)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True, type=Path)
    ap.add_argument("--dataset", required=True, type=Path)
    ap.add_argument("--gps", required=True, type=Path)
    ap.add_argument("--out-dir", type=Path)
    ap.add_argument("--window-s", type=float, default=2.0)
    ap.add_argument("--speed-thresholds", type=float, nargs="+", default=[1.0, 2.0])
    ap.add_argument("--plot-threshold", type=float, default=2.0)
    ap.add_argument("--gps-time-offset", type=float, default=0.0)
    args = ap.parse_args()

    out_dir = args.out_dir or (args.run_dir / "yaw_speed_gated_report")
    out_dir.mkdir(parents=True, exist_ok=True)

    traj_t, vio_yaw, _ = load_traj(args.run_dir / "traj.txt")
    bias_t, bias_g = load_bias(args.run_dir / "traj.txt.bias")
    imu_t, gyro = load_imu(args.dataset / "imu0" / "data.csv", float(traj_t[0]), float(traj_t[-1]))
    raw_t, raw_yaw = integrate_gyro_yaw(imu_t, gyro, bias_t, bias_g, float(vio_yaw[0]), use_bias=False)
    corr_t, corr_yaw = integrate_gyro_yaw(imu_t, gyro, bias_t, bias_g, float(vio_yaw[0]), use_bias=True)
    gps_t, gps_course, gps_speed, _, _ = load_gps_course(
        args.gps, float(traj_t[0]), float(traj_t[-1]), args.window_s, args.gps_time_offset
    )

    vio = np.asarray([interp_scalar(traj_t, vio_yaw, t) for t in gps_t])
    raw = np.asarray([interp_scalar(raw_t, raw_yaw, t) for t in gps_t])
    corr = np.asarray([interp_scalar(corr_t, corr_yaw, t) for t in gps_t])

    # Align GPS-referenced traces at the first GPS-course sample so the metrics
    # report relative yaw drift/consistency instead of arbitrary initial yaw.
    vio_a = align_to_reference(vio, gps_course)
    raw_a = align_to_reference(raw, gps_course)
    corr_a = align_to_reference(corr, gps_course)
    vio_to_corr = vio + (corr[0] - vio[0])

    timeseries = {
        "t_s": gps_t,
        "t_rel_s": gps_t - gps_t[0],
        "gps_speed_mps": gps_speed,
        "gps_course_yaw_deg": gps_course,
        "vio_yaw_aligned_deg": vio_a,
        "raw_gyro_yaw_aligned_deg": raw_a,
        "bias_gyro_yaw_aligned_deg": corr_a,
        "vio_yaw_aligned_to_biasgyro_deg": vio_to_corr,
        "vio_minus_gps_deg": vio_a - gps_course,
        "rawgyro_minus_gps_deg": raw_a - gps_course,
        "biasgyro_minus_gps_deg": corr_a - gps_course,
        "vio_minus_biasgyro_deg": vio_to_corr - corr,
    }
    ts_path = out_dir / "yaw_comparison_timeseries.csv"
    write_timeseries(ts_path, timeseries)

    rows: list[dict[str, object]] = []
    for threshold in args.speed_thresholds:
        mask = gps_speed > threshold
        if np.any(mask):
            t_first = float(gps_t[mask][0])
            t_last = float(gps_t[mask][-1])
        else:
            t_first = math.nan
            t_last = math.nan
        comparisons = {
            "VIO yaw vs GPS/course yaw": vio_a[mask] - gps_course[mask],
            "raw gyro integrated yaw vs GPS/course yaw": raw_a[mask] - gps_course[mask],
            "bias-corrected gyro integrated yaw vs GPS/course yaw": corr_a[mask] - gps_course[mask],
            "VIO yaw vs bias-corrected gyro yaw": (vio_to_corr[mask] - corr[mask]),
        }
        for name, err in comparisons.items():
            stats = residual_stats(err)
            rows.append(
                {
                    "speed_threshold_mps": threshold,
                    "comparison": name,
                    "t_first_s": t_first,
                    "t_last_s": t_last,
                    **stats,
                }
            )

    metrics_path = out_dir / "yaw_metrics_speed_gated.csv"
    write_metrics(metrics_path, rows)

    bg_z = bias_g[:, 2] if len(bias_g) else np.empty(0)
    plot_threshold = args.plot_threshold
    plot_path = out_dir / f"yaw_comparison_speed_gt_{plot_threshold:g}mps.png"
    plot_report(
        plot_path,
        args.run_dir.name,
        gps_t,
        gps_course,
        gps_speed,
        vio_a,
        raw_a,
        corr_a,
        gps_speed > plot_threshold,
        bias_t,
        bg_z,
        plot_threshold,
    )

    bg_path = out_dir / "bg_z_timeseries.csv"
    with bg_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "t_rel_s", "bg_z_rad_s"])
        for t, z in zip(bias_t, bg_z):
            writer.writerow([f"{float(t):.9f}", f"{float(t - traj_t[0]):.9f}", f"{float(z):.12g}"])

    print(f"run_dir: {args.run_dir}")
    print(f"plot: {plot_path}")
    print(f"metrics: {metrics_path}")
    print(f"timeseries: {ts_path}")
    print(f"bg_z: {bg_path}")
    print(f"GPS course samples: {len(gps_t)}")
    for threshold in args.speed_thresholds:
        print(f"speed > {threshold:g} m/s samples: {int(np.sum(gps_speed > threshold))}")


if __name__ == "__main__":
    main()
