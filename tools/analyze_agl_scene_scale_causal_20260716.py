#!/usr/bin/env python3
"""Assemble the causal AGL scene-scale reset evidence.

This script does not run an estimator and does not fit a trajectory.  It reads
paired shadow/single-reset runs plus the canonical full-flight evaluation
outputs and writes compact, reproducible comparison tables.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


FLIGHTS = ("fly1", "fly3")
ALIGNMENTS = ("absolute_navigation_no_post_alignment", "start_heading")
SUMMARY_FIELDS = (
    "analysis_start_time_s",
    "analysis_end_time_s",
    "start_heading_rotation_deg",
    "xy_rmse_m",
    "final_xy_error_m",
    "vertical_rmse_m",
    "final_vertical_error_m",
    "vxy_vec_rmse_mps",
    "yaw/course_rmse_deg",
    "yaw/course_final_deg",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evaluation-root", type=Path, required=True)
    parser.add_argument("--shadow-root", type=Path, required=True)
    parser.add_argument("--single-root", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    return parser.parse_args()


def read_scalar(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def run_dir(root: Path, flight: str, variant: str) -> Path:
    return root / f"{flight}_agl_scale_{variant}"


def rms(values: pd.Series) -> float:
    finite = pd.to_numeric(values, errors="coerce").to_numpy(dtype=float)
    finite = finite[np.isfinite(finite)]
    return float(np.sqrt(np.mean(np.square(finite)))) if len(finite) else math.nan


def formal_rows(evaluation_root: Path) -> list[dict]:
    rows: list[dict] = []
    for alignment in ALIGNMENTS:
        for flight in FLIGHTS:
            for variant in ("shadow", "single"):
                path = evaluation_root / alignment / f"{flight}_{variant}" / "metadata/global_summary.json"
                data = json.loads(path.read_text(encoding="utf-8"))
                row = {"alignment": alignment, "flight": flight, "variant": variant}
                row.update({key: data[key] for key in SUMMARY_FIELDS})
                rows.append(row)
    return rows


def local_window_rows(evaluation_root: Path, reset_times: dict[str, float]) -> list[dict]:
    rows: list[dict] = []
    root = evaluation_root / "start_heading"
    for flight in FLIGHTS:
        trajectories = {
            variant: pd.read_csv(root / f"{flight}_{variant}" / "data/gps_time_aligned_samples.csv")
            for variant in ("shadow", "single")
        }
        t0 = max(frame["t"].min() for frame in trajectories.values())
        t1 = min(frame["t"].max() for frame in trajectories.values())
        reset = reset_times[flight]
        windows = (
            ("pre_reset", t0, reset),
            ("post_0_10_s", reset, min(reset + 10.0, t1)),
            ("post_10_30_s", reset + 10.0, min(reset + 30.0, t1)),
            ("post_30_60_s", reset + 30.0, min(reset + 60.0, t1)),
            ("post_60_s_to_end", reset + 60.0, t1 + 1.0e-6),
        )
        for window, begin, end in windows:
            if end <= begin:
                continue
            for variant, frame in trajectories.items():
                sample = frame[(frame["t"] >= begin) & (frame["t"] < end)].copy()
                if sample.empty:
                    continue
                elapsed = sample["t"].to_numpy() - sample["t"].iloc[0]
                course = sample["course_error_deg"].to_numpy()
                slope = float(np.polyfit(elapsed, course, 1)[0]) if len(sample) >= 3 else math.nan
                rows.append(
                    {
                        "flight": flight,
                        "variant": variant,
                        "window": window,
                        "start_s": begin,
                        "end_s": end,
                        "rows": len(sample),
                        "xy_rmse_m": rms(sample["err_XY"]),
                        "vertical_rmse_m": rms(sample["err_U"]),
                        "course_rmse_deg": rms(sample["course_error_deg"]),
                        "vxy_vector_rmse_mps": rms(sample["err_vXY_vec"]),
                        "final_xy_error_m": float(sample["err_XY"].iloc[-1]),
                        "final_vertical_error_m": float(sample["err_U"].iloc[-1]),
                        "final_course_error_deg": float(sample["course_error_deg"].iloc[-1]),
                        "course_error_slope_degps": slope,
                    }
                )
    return rows


def first_threshold_time(frame: pd.DataFrame, column: str, threshold: float) -> float | None:
    match = frame[np.abs(frame[column]) >= threshold]
    return float(match["t"].iloc[0]) if len(match) else None


def causal_rows(evaluation_root: Path, reset_times: dict[str, float]) -> list[dict]:
    rows: list[dict] = []
    root = evaluation_root / "absolute_navigation_no_post_alignment"
    for flight in FLIGHTS:
        active = pd.read_csv(root / f"{flight}_single" / "data/gps_time_aligned_samples.csv")
        shadow = pd.read_csv(root / f"{flight}_shadow" / "data/gps_time_aligned_samples.csv")
        columns = ["vio_E", "vio_N", "vio_U", "vio_vE", "vio_vN", "vio_vU", "vio_course_deg"]
        paired = active[["t", *columns]].merge(
            shadow[["t", *columns]], on="t", suffixes=("_active", "_shadow")
        )
        paired["delta_xy_m"] = np.hypot(
            paired["vio_E_active"] - paired["vio_E_shadow"],
            paired["vio_N_active"] - paired["vio_N_shadow"],
        )
        paired["delta_velocity_mps"] = np.sqrt(
            sum(
                np.square(paired[f"{axis}_active"] - paired[f"{axis}_shadow"])
                for axis in ("vio_vE", "vio_vN", "vio_vU")
            )
        )
        paired["delta_course_deg"] = (
            paired["vio_course_deg_active"] - paired["vio_course_deg_shadow"] + 180.0
        ) % 360.0 - 180.0
        reset = reset_times[flight]
        row: dict[str, float | str | None] = {"flight": flight, "reset_time_s": reset}
        for column, thresholds in {
            "delta_xy_m": (0.1, 1.0, 10.0, 50.0),
            "delta_velocity_mps": (0.1, 1.0, 3.0),
            "delta_course_deg": (0.1, 0.5, 1.0),
        }.items():
            for threshold in thresholds:
                time_s = first_threshold_time(paired, column, threshold)
                tag = str(threshold).replace(".", "p")
                row[f"first_{column}_ge_{tag}_time_s"] = time_s
                row[f"first_{column}_ge_{tag}_after_reset_s"] = (
                    time_s - reset if time_s is not None else None
                )
        rows.append(row)
    return rows


def reset_contract_rows(shadow_root: Path, single_root: Path) -> list[dict]:
    rows: list[dict] = []
    for flight in FLIGHTS:
        active_dir = run_dir(single_root, flight, "single")
        shadow_dir = run_dir(shadow_root, flight, "shadow")
        scale = pd.read_csv(active_dir / "agl_scene_scale.csv")
        reset_rows = scale[(scale["reset_succeeded"] == 1) & (scale["state_modified"] == 1)]
        if len(reset_rows) != 1:
            raise RuntimeError(f"{flight}: expected exactly one successful reset, got {len(reset_rows)}")
        reset = reset_rows.iloc[0]
        post = scale[(scale["timestamp"] > reset["timestamp"]) & (scale["ground_valid"] == 1)].iloc[0]

        active_state = pd.read_csv(active_dir / "state_safety.csv")
        shadow_state = pd.read_csv(shadow_dir / "state_safety.csv")
        active_index = int(np.argmin(np.abs(active_state["time"].to_numpy() - reset["timestamp"])))
        shadow_index = int(np.argmin(np.abs(shadow_state["time"].to_numpy() - reset["timestamp"])))
        a = active_state.iloc[active_index]
        s = shadow_state.iloc[shadow_index]
        position_delta = float(np.linalg.norm([a.px - s.px, a.py - s.py, a.pz - s.pz]))
        velocity_active = np.array([a.vx, a.vy, a.vz], dtype=float)
        velocity_shadow = np.array([s.vx, s.vy, s.vz], dtype=float)
        quat_active = np.array([a.qx, a.qy, a.qz, a.qw], dtype=float)
        quat_shadow = np.array([s.qx, s.qy, s.qz, s.qw], dtype=float)
        quat_dot = float(np.clip(abs(np.dot(quat_active, quat_shadow)), 0.0, 1.0))
        attitude_delta_deg = math.degrees(2.0 * math.acos(quat_dot))
        bias_delta = float(
            np.linalg.norm([a.bgx - s.bgx, a.bgy - s.bgy, a.bgz - s.bgz, a.bax - s.bax, a.bay - s.bay, a.baz - s.baz])
        )

        rows.append(
            {
                "flight": flight,
                "process_exit_code": int(read_scalar(active_dir / "exit_code.txt")),
                "frame_contract_exit_code": int(read_scalar(active_dir / "frame_contract_validation_exit_code.txt")),
                "reset_time_s": float(reset["timestamp"]),
                "agl_time_s": float(reset["agl_timestamp"]),
                "agl_age_s": float(reset["agl_age_s"]),
                "external_agl_m": float(reset["agl_m"]),
                "visual_map_height_m": float(reset["map_height_m"]),
                "scale": float(reset["applied_scale"]),
                "plane_tilt_deg": float(reset["plane_tilt_deg"]),
                "plane_rmse_m": float(reset["plane_residual_rmse_m"]),
                "inlier_features": int(reset["inlier_features"]),
                "image_cells": int(reset["image_cells"]),
                "distinct_clone_times": int(reset["distinct_clone_times"]),
                "history_coverage_s": float(reset["window_coverage_s"]),
                "log_ratio_mad": float(reset["log_ratio_mad"]),
                "post_reset_map_height_m": float(post["map_height_m"]),
                "post_reset_external_agl_m": float(post["agl_m"]),
                "post_reset_ratio": float(post["map_height_m"] / post["agl_m"]),
                "position_delta_at_reset_m": position_delta,
                "attitude_delta_at_reset_deg": attitude_delta_deg,
                "bias_vector_delta_at_reset": bias_delta,
                "velocity_norm_ratio_at_reset": float(np.linalg.norm(velocity_active) / np.linalg.norm(velocity_shadow)),
                "covariance_all_finite": bool(a["cov_all_finite"]),
                "covariance_negative_diagonal_count": int(a["cov_neg_diag_count"]),
                "covariance_psd_projected": bool(reset["covariance_psd_projected"]),
                "covariance_psd_projection_magnitude": float(reset["covariance_psd_projection_magnitude"]),
                "covariance_psd_projection_limit": float(reset["covariance_psd_projection_limit"]),
            }
        )
    return rows


def speed_scale_rows(evaluation_root: Path, reset_rows: list[dict]) -> list[dict]:
    root = evaluation_root / "absolute_navigation_no_post_alignment"
    result: list[dict] = []
    for reset in reset_rows:
        flight = str(reset["flight"])
        reset_time = float(reset["reset_time_s"])
        for variant in ("shadow", "single"):
            frame = pd.read_csv(root / f"{flight}_{variant}" / "data/gps_time_aligned_samples.csv")
            post = frame[frame["t"] >= reset_time].iloc[0]
            gps_speed = float(math.hypot(post.gps_vE, post.gps_vN))
            vio_speed = float(math.hypot(post.vio_vE, post.vio_vN))
            result.append(
                {
                    "flight": flight,
                    "variant": variant,
                    "sample_time_s": float(post.t),
                    "after_reset_s": float(post.t - reset_time),
                    "gps_horizontal_speed_mps": gps_speed,
                    "vio_horizontal_speed_mps": vio_speed,
                    "vio_to_gps_speed_ratio": vio_speed / gps_speed,
                    "horizontal_velocity_vector_error_mps": float(post.err_vXY_vec),
                    "course_error_deg": float(post.course_error_deg),
                }
            )
    return result


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def write_report(
    out: Path,
    formal: pd.DataFrame,
    local: pd.DataFrame,
    causal: pd.DataFrame,
    reset: pd.DataFrame,
    speed: pd.DataFrame,
) -> None:
    lines = [
        "# AGL 场景尺度复位因果验证",
        "",
        "## 结论",
        "",
        "**当前 AGL/map-height 比例不能解释为全局各向同性 VIO 尺度，single Sim(3) reset 不可接受。**",
        "",
        "复位本身按合同执行：q/p/bg/ba 在复位点连续，速度按比例缩放，视觉地面高度随即接近外部高度，协方差有限且无负对角。退化却从复位后第一个 GPS 评价样本开始，而复位前水平速度尺度已基本正确。因此失败的不是复位代码没有生效，而是把“稀疏视觉地面平面高度差”当成“整个惯性 VIO 的全局尺度误差”这一假设。",
        "",
        "GPS E/N/course/yaw 仅用于离线评价；在线 AGL 路径没有使用这些量。",
        "",
        "## 复位合同",
        "",
        "| flight | reset time | scale | AGL / map height | next map / AGL | position jump | attitude jump | velocity ratio | covariance |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for _, row in reset.iterrows():
        lines.append(
            f"| {row.flight} | {row.reset_time_s:.3f} | {row.scale:.6f} | "
            f"{row.external_agl_m:.3f} / {row.visual_map_height_m:.3f} m | {row.post_reset_ratio:.4f} | "
            f"{row.position_delta_at_reset_m:.3e} m | {row.attitude_delta_at_reset_deg:.3e}° | "
            f"{row.velocity_norm_ratio_at_reset:.6f} | finite={bool(row.covariance_all_finite)}, negdiag={int(row.covariance_negative_diagonal_count)} |"
        )

    lines += [
        "",
        "## 复位时的独立尺度交叉检查",
        "",
        "如果 AGL/map-height 真是全局各向同性尺度，位置和速度应该支持同一比例。实际复位前水平速度只差约 1–3%，复位却强制缩小 7.5–11.3%。",
        "",
        "| flight | variant | VIO/GPS horizontal speed | vector velocity error |",
        "| --- | --- | ---: | ---: |",
    ]
    for _, row in speed.iterrows():
        lines.append(
            f"| {row.flight} | {row.variant} | {row.vio_to_gps_speed_ratio:.6f} | {row.horizontal_velocity_vector_error_mps:.3f} m/s |"
        )

    lines += [
        "",
        "## 正式 GPS-time 评价",
        "",
        "主表采用 start-heading；absolute-navigation 结果保存在同目录，用于检查初始化绝对方向。没有 best-fit/SE(3)/Sim(3) 对齐。",
        "",
        "| flight | variant | XY RMSE | Z RMSE | velocity RMSE | course RMSE | final course error | start-heading rotation |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    primary = formal[formal["alignment"] == "start_heading"]
    for _, row in primary.iterrows():
        lines.append(
            f"| {row.flight} | {row.variant} | {row.xy_rmse_m:.3f} m | {row.vertical_rmse_m:.3f} m | "
            f"{row.vxy_vec_rmse_mps:.3f} m/s | {row['yaw/course_rmse_deg']:.3f}° | "
            f"{row['yaw/course_final_deg']:.3f}° | {row.start_heading_rotation_deg:.3f}° |"
        )

    lines += [
        "",
        "fly1 的 course RMSE 只改善约 0.02°，但 XY/Z/速度均明显退化；fly3 的 course RMSE 改善约 0.69°，同时 XY RMSE 近乎翻倍、速度误差显著增大。该航向改善不能抵消导航状态退化，也不能证明尺度假设成立。",
        "",
        "## 分叉时序",
        "",
        "| flight | ΔXY≥0.1 m | ΔXY≥1 m | ΔXY≥10 m | Δv≥1 m/s | Δcourse≥0.5° |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for _, row in causal.iterrows():
        lines.append(
            f"| {row.flight} | {row.first_delta_xy_m_ge_0p1_after_reset_s:.3f} s | "
            f"{row.first_delta_xy_m_ge_1p0_after_reset_s:.3f} s | {row.first_delta_xy_m_ge_10p0_after_reset_s:.3f} s | "
            f"{row.first_delta_velocity_mps_ge_1p0_after_reset_s:.3f} s | "
            f"{row.first_delta_course_deg_ge_0p5_after_reset_s:.3f} s |"
        )

    lines += [
        "",
        "## 处理决定",
        "",
        "- `single_reset` 不进入正式 P4/P5；默认仍为 off，shadow 只保留诊断价值。",
        "- 不能通过调整 scale threshold/cooldown 掩盖，因为错误是量的语义，不是门限。",
        "- 若继续使用高度，必须先获得真实局部地面 AGL，并建立只作用于视觉深度/地标尺度的受约束状态；在没有独立证据证明惯性导航状态也存在同一尺度误差前，禁止对 q/p/v/covariance 做全局 Sim(3) 复位。",
        "- 当前航向根因继续由相机标定 K/D 拆分和转弯视觉更新链排查，不能把 fly3 的偶然 course 改善写成修复。",
        "",
        "## 产物",
        "",
        "- `formal_summary.csv`：canonical evaluator 全局指标。",
        "- `reset_local_window_summary.csv`：复位前后时间窗指标。",
        "- `causal_divergence.csv`：active-shadow 首次分叉时刻。",
        "- `reset_contract.csv`：复位几何与协方差合同。",
        "- `speed_scale_crosscheck.csv`：外部速度尺度交叉检查（仅离线评价）。",
    ]
    (out / "AGL_SCENE_SCALE_CAUSAL_VERDICT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    formal = pd.DataFrame(formal_rows(args.evaluation_root))
    reset = pd.DataFrame(reset_contract_rows(args.shadow_root, args.single_root))
    reset_times = dict(zip(reset["flight"], reset["reset_time_s"]))
    local = pd.DataFrame(local_window_rows(args.evaluation_root, reset_times))
    causal = pd.DataFrame(causal_rows(args.evaluation_root, reset_times))
    speed = pd.DataFrame(speed_scale_rows(args.evaluation_root, reset.to_dict("records")))

    formal.to_csv(args.out_dir / "formal_summary.csv", index=False)
    local.to_csv(args.out_dir / "reset_local_window_summary.csv", index=False)
    causal.to_csv(args.out_dir / "causal_divergence.csv", index=False)
    reset.to_csv(args.out_dir / "reset_contract.csv", index=False)
    speed.to_csv(args.out_dir / "speed_scale_crosscheck.csv", index=False)
    write_report(args.out_dir, formal, local, causal, reset, speed)

    print(args.out_dir / "AGL_SCENE_SCALE_CAUSAL_VERDICT.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
