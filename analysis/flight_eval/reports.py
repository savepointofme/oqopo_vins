"""Chinese Markdown reports for a single flight experiment."""
from __future__ import annotations

import os


def _fmt(value, unit=""):
    return "—" if value is None else f"{value}{unit}"


def full_flight_report(spec, summary: dict, quality: dict, seg_err,
                       *, reference_velocity_source: str,
                       vio_velocity_source: str, lk_mode: str) -> str:
    fc_fallback = not reference_velocity_source.startswith("flight_controller")
    lines = [
        f"# 飞行实验误差分析报告 · {spec.experiment_id}",
        "",
        f"- 飞行 / 方法：**{spec.flight_name} / {spec.method_name}**",
        f"- 日期 / 状态：{spec.date} / {spec.status}",
        f"- 有效窗口：t0={spec.t0}s，t1={spec.t1}s",
        f"- 对齐：{spec.alignment['mode']}，初始航向窗口 "
        f"{spec.alignment['course_window_s']}s",
        f"- GPS参考速度：**{reference_velocity_source}**"
        + ("（位置差分备用值）" if fc_fallback else ""),
        f"- VIO速度：{vio_velocity_source}",
        f"- 纯视觉分析：{lk_mode}",
        "",
        "## 1. 单条实验全局指标",
        "",
        "| 指标 | 数值 |",
        "|---|---:|",
    ]
    rows = [
        ("GPS总里程", _fmt(summary.get("gps_distance_km"), " km")),
        ("飞行时长", _fmt(summary.get("duration_s"), " s")),
        ("终点XY误差", _fmt(summary.get("final_xy_error_m"), " m")),
        ("终点XY漂移率", _fmt(summary.get("final_xy_drift_percent"), " %")),
        ("XY误差RMSE", _fmt(summary.get("xy_rmse_m"), " m")),
        ("终点沿航向误差（XY）", _fmt(summary.get("final_along_error_m"), " m")),
        ("终点垂直航线误差（XY）", _fmt(summary.get("final_cross_error_m"), " m")),
        ("终点高度U误差", _fmt(summary.get("final_vertical_error_m"), " m")),
        ("XY速度向量RMSE", _fmt(summary.get("vxy_vec_rmse_mps"), " m/s")),
        ("沿航向速度RMSE（XY）", _fmt(summary.get("v_along_rmse_mps"), " m/s")),
        ("垂直航线速度RMSE（XY）", _fmt(summary.get("v_cross_rmse_mps"), " m/s")),
        ("高度速度vU RMSE", _fmt(summary.get("v_vertical_rmse_mps"), " m/s")),
        ("航向误差RMSE", _fmt(summary.get("yaw_or_course_rmse_deg"), "°")),
    ]
    lines.extend(f"| {name} | {value} |" for name, value in rows)

    lines.extend([
        "",
        "## 2. GPS条次与时间对齐",
        "",
        f"- GPS条次：{quality['gps_sample_count']}；有效对齐："
        f"{quality['valid_aligned_count']}；延迟无效：{quality['invalid_delay_count']}。",
        f"- GPS缺口：{quality['gps_gap_count']}；最长缺口："
        f"{quality['max_gps_gap_s']} s。",
        f"- VIO延迟 mean / p95 / max：{quality['mean_delay_s']} / "
        f"{quality['p95_delay_s']} / {quality['max_delay_s']} s。",
        "",
        "## 3. 航线坐标定义",
        "",
        "- 沿航向和垂直航线分量均由GPS航向在XY平面内正交投影得到。",
        "- 垂直航线表示XY平面内与当前航向垂直的侧向，不是Z轴。",
        "- 高度U/Z及高度速度vU始终单独绘图、单独统计。",
        f"- 位置投影恒等式最大残差："
        f"{summary.get('route_projection_identity_max_m2', '—')} m²。",
        f"- 速度投影恒等式最大残差："
        f"{summary.get('route_velocity_projection_identity_max_m2ps2', '—')} "
        "(m/s)²。",
        "",
        "## 4. 每圈四边分析",
        "",
        "每个完整圈固定分为四边：去程主边、远端连接边、回程主边、"
        "近端连接边。起始和结束残段不进入圈间比较。",
        "",
    ])
    if seg_err is not None and len(seg_err):
        lines.extend([
            "| 圈/边 | 分段说明 | 长度m | 局部XY漂移率 | 沿航向尺度比例 | "
            "全局XY RMSE m | 垂直航线RMSE m |",
            "|---|---|---:|---:|---:|---:|---:|",
        ])
        complete = seg_err[(seg_err["lap_id"] > 0) & (seg_err["side_id"] > 0)]
        for _, row in complete.iterrows():
            lines.append(
                f"| 第{int(row['lap_id'])}圈/边{int(row['side_id'])} | "
                f"{row['label']} | {row['dist_m']} | "
                f"{_fmt(row['local_drift_percent'])} | "
                f"{_fmt(row.get('scale_ratio_mean'))} | "
                f"{row['global_xy_rmse_m']} | {row['cross_rmse_m']} |"
            )

    lines.extend([
        "",
        "## 5. 数据可信度说明",
        "",
        "- 曲线只在GPS更新条次上统计，并匹配该次更新后的VIO状态。",
        "- 图中空白表示GPS缺口或无效对齐，不通过删除尖峰制造连续曲线。",
        "- 速度参考优先使用飞控原始Ve/Vn/Vu，不使用位置差分替代。",
        "- 时间窗口在SLAM清零或相机失败尾段之前截止。",
        "- 复现信息位于metadata目录。",
    ])
    if fc_fallback:
        lines.append("- 警告：本次参考速度使用了位置差分备用值，速度结论仅供参考。")
    return "\n".join(lines)


def lk_only_report(spec, lk_result, summary_lk: dict | None) -> str:
    lines = [f"# 纯视觉LK分析 · {spec.experiment_id}", ""]
    if not lk_result.available:
        lines.extend(["## 不可用", "", f"缺少：{lk_result.missing}"])
        return "\n".join(lines)

    lines.append(f"- 模式：**{lk_result.mode}**")
    if lk_result.mode == "LK_YAW_DIAGNOSTIC":
        lines.extend([
            "- 数据直接来自原始LK光流，不使用VIO状态。",
            "- 本数据可评价视觉航向变化、跟踪内点和RANSAC残差。",
            "- 它不包含可独立积分验证的位置尺度，因此不冒充纯视觉位置轨迹。",
        ])
        if summary_lk:
            lines.extend([
                f"- 样本数：{summary_lk.get('sample_count')}",
                f"- 内点数中位数：{summary_lk.get('median_inliers')}",
                f"- RANSAC残差中位数："
                f"{summary_lk.get('median_ransac_resid_px')} px",
                f"- LK与GPS航向角速度差RMSE："
                f"{summary_lk.get('lk_minus_gps_rate_rmse_degps')} °/s",
            ])
    elif lk_result.mode == "LK_ONLY_DIAGNOSTIC":
        lines.append(
            "- 该轨迹由光流诊断量积分构造，仅用于诊断，不作为可部署里程计。"
        )
    elif summary_lk:
        lines.append(
            f"- 终点XY误差：{summary_lk.get('final_xy_error_m')} m。"
        )
    return "\n".join(lines)


def write(text: str, reports_dir: str, name: str) -> str:
    os.makedirs(reports_dir, exist_ok=True)
    path = os.path.join(reports_dir, name)
    with open(path, "w", encoding="utf-8") as file:
        file.write(text)
    return path
