#!/usr/bin/env python3
"""Audit posterior/gyro agreement for four-flight body-axis yaw-flex input."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


FLIGHTS = ("fly1", "fly2", "fly3", "fly4")
DATASETS = {
    "fly1": Path(
        "C:/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"
    ),
    "fly2": Path(
        "C:/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722"
    ),
    "fly3": Path(
        "C:/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946"
    ),
    "fly4": Path(
        "C:/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549"
    ),
}


def read_trajectory(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        comment="#",
        sep=r"\s+",
        header=None,
        names=["t", "x", "y", "z", "qx", "qy", "qz", "qw"],
        engine="python",
    )


def read_bias(path: Path) -> pd.DataFrame:
    return pd.read_csv(
        path,
        comment="#",
        sep=r"\s+",
        header=None,
        names=["t", "vx", "vy", "vz", "bgx", "bgy", "bgz", "bax", "bay", "baz"],
        engine="python",
    )


def read_imu(path: Path) -> pd.DataFrame:
    table = pd.read_csv(path)
    table = table.rename(columns={table.columns[0]: table.columns[0].lstrip("#")})
    return pd.DataFrame(
        {
            "t": table.t_ns.to_numpy(dtype=float) * 1e-9,
            "wx": table.wx.to_numpy(dtype=float),
            "wy": table.wy.to_numpy(dtype=float),
            "wz": table.wz.to_numpy(dtype=float),
        }
    )


def integrate_gyro(
    imu_t: np.ndarray,
    imu_w: np.ndarray,
    bias_t: np.ndarray,
    bias_bg: np.ndarray,
    start: float,
    end: float,
) -> np.ndarray:
    first = int(np.searchsorted(imu_t, start, side="right"))
    last = int(np.searchsorted(imu_t, end, side="left"))
    times = np.concatenate(([start], imu_t[first:last], [end]))
    omega = np.column_stack(
        [np.interp(times, imu_t, imu_w[:, axis]) for axis in range(3)]
    )
    bias = np.column_stack(
        [np.interp(times, bias_t, bias_bg[:, axis]) for axis in range(3)]
    )
    values = omega - bias
    result = np.eye(3)
    for index, dt in enumerate(np.diff(times)):
        midpoint = 0.5 * (values[index] + values[index + 1])
        result = Rotation.from_rotvec(-midpoint * dt).as_matrix() @ result
    return result


def flex_increment_deg(
    measured_delta_imu: np.ndarray,
    fc_delta_body: np.ndarray,
    nominal_imu_from_body: np.ndarray,
) -> float:
    predicted = nominal_imu_from_body @ fc_delta_body @ nominal_imu_from_body.T
    mismatch = measured_delta_imu @ predicted.T
    log_imu = Rotation.from_matrix(mismatch).as_rotvec()
    return float(-np.degrees((nominal_imu_from_body.T @ log_imu)[2]))


def robust_stats(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    median = float(np.median(values))
    mad_sigma = float(1.4826 * np.median(np.abs(values - median)))
    return {
        "median_deg": median,
        "mad_sigma_deg": mad_sigma,
        "median_absolute_deg": float(np.median(np.abs(values))),
        "p95_absolute_deg": float(np.percentile(np.abs(values), 95.0)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fc-stream-dir", type=Path, required=True)
    parser.add_argument("--run", action="append", required=True, help="flight=run-dir")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError(f"refusing to overwrite {args.output_dir}")
    args.output_dir.mkdir(parents=True)
    runs = {
        name: Path(path) for name, path in (item.split("=", 1) for item in args.run)
    }

    summary = {"schema_version": 1, "flights": {}}
    for flight in FLIGHTS:
        run = runs[flight]
        flex = pd.read_csv(run / "aircraft_body_attitude.csv", comment="#")
        init_time = float(flex.camera_time_s.iloc[0])
        trajectory = read_trajectory(run / "traj.txt")
        bias = read_bias(run / "traj.txt.bias")
        imu = read_imu(DATASETS[flight] / "imu0" / "data.csv")
        fc_all = pd.read_csv(
            args.fc_stream_dir / f"{flight}_fc_body_attitude_camera_time.csv",
            comment="#",
        )
        valid = fc_all.valid.to_numpy(dtype=int).astype(bool)
        fc_valid = fc_all.loc[valid].reset_index(drop=True)
        fc_full_times = fc_valid.camera_time_s.to_numpy(dtype=float)
        fc_full_rotation = Rotation.from_quat(
            fc_valid[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
        )
        selected = fc_full_times >= init_time
        selected &= fc_full_times <= float(trajectory.t.iloc[-1])
        fc = fc_valid.loc[selected].reset_index(drop=True)
        fc_times = fc.camera_time_s.to_numpy(dtype=float)
        fc_rotation = Rotation.from_quat(
            fc[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
        )
        vio_slerp = Slerp(
            trajectory.t.to_numpy(dtype=float),
            Rotation.from_quat(
                trajectory[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
            ),
        )
        vio_rotation = vio_slerp(fc_times)
        initial_fc = Slerp(fc_full_times, fc_full_rotation)([init_time])[0]
        initial_vio = vio_slerp([init_time])[0]
        nominal = initial_vio.inv().as_matrix() @ initial_fc.as_matrix()

        imu_t = imu.t.to_numpy(dtype=float)
        imu_w = imu[["wx", "wy", "wz"]].to_numpy(dtype=float)
        bias_t = bias.t.to_numpy(dtype=float)
        bias_bg = bias[["bgx", "bgy", "bgz"]].to_numpy(dtype=float)
        rows = []
        posterior_cumulative = 0.0
        gyro_cumulative = 0.0
        for index in range(1, len(fc)):
            start = float(fc_times[index - 1])
            end = float(fc_times[index])
            fc_delta = (
                fc_rotation[index].inv().as_matrix()
                @ fc_rotation[index - 1].as_matrix()
            )
            posterior_delta = (
                vio_rotation[index].inv().as_matrix()
                @ vio_rotation[index - 1].as_matrix()
            )
            gyro_delta = integrate_gyro(imu_t, imu_w, bias_t, bias_bg, start, end)
            posterior_increment = flex_increment_deg(posterior_delta, fc_delta, nominal)
            gyro_increment = flex_increment_deg(gyro_delta, fc_delta, nominal)
            posterior_cumulative += posterior_increment
            gyro_cumulative += gyro_increment
            rows.append(
                {
                    "t": end,
                    "dt_s": end - start,
                    "posterior_flex_increment_deg": posterior_increment,
                    "gyro_flex_increment_deg": gyro_increment,
                    "posterior_minus_gyro_increment_deg": posterior_increment
                    - gyro_increment,
                    "posterior_flex_cumulative_deg": posterior_cumulative,
                    "gyro_flex_cumulative_deg": gyro_cumulative,
                }
            )
        trace = pd.DataFrame(rows)
        trace.to_csv(
            args.output_dir / f"{flight}_gyro_vio_flex_agreement.csv", index=False
        )
        difference = trace.posterior_minus_gyro_increment_deg.to_numpy(dtype=float)
        correlation = float(
            np.corrcoef(
                trace.posterior_flex_increment_deg,
                trace.gyro_flex_increment_deg,
            )[0, 1]
        )
        item = {
            "initialization_time_s": init_time,
            "interval_count": int(len(trace)),
            "posterior_gyro_increment_correlation": correlation,
            "posterior_minus_gyro": robust_stats(difference),
            "posterior_cumulative_end_deg": float(posterior_cumulative),
            "gyro_cumulative_end_deg": float(gyro_cumulative),
            "fly3_windows": [],
        }
        if flight == "fly3":
            for start, end in ((760.0, 800.0), (995.0, 1025.0)):
                window = trace[(trace.t >= start) & (trace.t <= end)]
                item["fly3_windows"].append(
                    {
                        "window_s": [start, end],
                        "posterior_net_deg": float(
                            window.posterior_flex_increment_deg.sum()
                        ),
                        "gyro_net_deg": float(window.gyro_flex_increment_deg.sum()),
                        "posterior_minus_gyro": robust_stats(
                            window.posterior_minus_gyro_increment_deg.to_numpy(
                                dtype=float
                            )
                        ),
                    }
                )
        summary["flights"][flight] = item

    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
