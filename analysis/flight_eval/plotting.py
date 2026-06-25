"""plotting.py — 中文静态图 (SVG).

所有图: 中文标题/坐标轴/图例/单位/说明。
语义配色: GPS=绿(参考), VIO=蓝, LK-only=橙(诊断), 误差=红, 无效=灰, 选中段=紫。
空白必须解释（GPS 未更新 / 样本 invalid / 速度列缺失 / 时间不覆盖），
不通过删除尖峰伪造连续曲线。

依赖: matplotlib（SVG）。Plotly HTML 输出已废弃；交互页使用 SVG v2 dashboard。
"""
from __future__ import annotations

import os

import numpy as np
import pandas as pd

COLORS = {
    "gps": "#1f7a4d", "vio": "#2563c9", "lk": "#e07b1a",
    "err": "#d33a35", "invalid": "#9aa3ad", "sel": "#8b5cf6",
}


def set_chinese_font():
    """Select an installed CJK font instead of trusting a family name."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    selected = None
    for fam in ("Microsoft YaHei", "SimHei", "DengXian", "Noto Sans CJK SC"):
        try:
            font_manager.findfont(fam, fallback_to_default=False)
            selected = fam
            break
        except ValueError:
            continue
    if selected is None:
        raise RuntimeError("未找到可用中文字体（微软雅黑/黑体/等线）")
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = [selected]
    plt.rcParams["axes.unicode_minus"] = False
    return selected


# 默认静态图清单（名称 → 标题/问题），供 dashboard 与报告引用。
# 柱状图不再默认输出；分段对比保留轨迹示意和误差剖面。
PLOT_SPECS = [
    ("xy_trajectory_gps_vio_lk", "XY 轨迹对比", "VIO/LK 何时何地相对 GPS 发散？"),
    ("xy_error_distance", "XY 误差 vs 里程", "水平误差是否随里程线性累积？"),
    ("enu_position_compare", "ENU 位置对比", "估计是否跟随参考？"),
    ("enu_position_error", "ENU 各方向位置误差", "E/N/U各方向误差如何变化？"),
    ("cross_vertical_error", "XY沿航向 / 垂直航线位置误差", "沿航向和垂直航线误差如何随里程增长？"),
    ("along_cross_velocity_error", "XY沿航向 / 垂直航线速度误差", "速度误差主要沿航向还是垂直航线？"),
    ("enu_velocity_compare", "ENU 速度对比", "VIO 速度是否跟随飞控原始速度？"),
    ("enu_velocity_component_error", "ENU 各方向速度误差", "vE/vN/vU误差如何变化？"),
    ("speed_error_distance", "速度模值/向量误差", "标量匹配但向量不匹配→航向误差？"),
    ("course_yaw_error", "航向 / course 误差", "是否存在持续增长的航向漂移？"),
    ("straight_leg_error_profiles", "每圈四边误差剖面", "同一条边在不同圈内的误差如何增长？"),
    ("gps_sampling_quality", "GPS 采样质量", "统计网格是否可信？"),
]


def _save(fig, plots_dir: str, name: str):
    """Save one SVG plot. PNG output is intentionally disabled."""
    os.makedirs(plots_dir, exist_ok=True)
    fig.savefig(os.path.join(plots_dir, f"{name}.svg"), bbox_inches="tight")


def _finish(ax):
    ax.grid(True, alpha=.25)
    ax.legend(loc="best")


def _note(fig, text: str):
    fig.text(.01, .01, text, ha="left", va="bottom", fontsize=9, color="#4b5563")


def plot_all(df: pd.DataFrame, seg_err: pd.DataFrame, quality: dict,
             plots_dir: str, *, velocity_source: str, lk_available: bool):
    """生成全部静态图。失败的单张图记录但不中断整体。"""
    set_chinese_font()
    import matplotlib.pyplot as plt

    v = df[df["valid"]]
    fc_note = "" if velocity_source.startswith("flight_controller") else "（速度源=位置差分 fallback）"

    # 1. XY 轨迹
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(df["gps_E"], df["gps_N"], color=COLORS["gps"], lw=2, label="GPS（飞控参考）")
    ax.plot(df["vio_E"], df["vio_N"], color=COLORS["vio"], lw=1.6, label="VIO")
    if lk_available and "lk_E" in df:
        ax.plot(df["lk_E"], df["lk_N"], color=COLORS["lk"], lw=1.4, ls="--", label="LK-only（诊断）")
    ax.scatter([df["gps_E"].iloc[0]], [df["gps_N"].iloc[0]], facecolors="none",
               edgecolors=COLORS["gps"], s=60, label="起点")
    ax.set_xlabel("东向 E / m"); ax.set_ylabel("北向 N / m")
    ax.set_title("XY 轨迹对比（ENU · 起点对齐）"); ax.legend(); ax.set_aspect("equal")
    _save(fig, plots_dir, "xy_trajectory_gps_vio_lk"); plt.close(fig)

    # 2. XY 误差 vs 里程
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(v["cum_dist"] / 1000, v["err_XY"], color=COLORS["vio"], lw=1.6, label="VIO XY 误差")
    if lk_available and "err_lk_XY" in v:
        ax.plot(v["cum_dist"] / 1000, v["err_lk_XY"], color=COLORS["lk"], lw=1.4, ls="--", label="LK-only XY 误差")
    ax.set_xlabel("GPS 累计里程 / km"); ax.set_ylabel("XY 误差 / m")
    ax.set_title("XY 误差随里程累积"); ax.legend()
    _save(fig, plots_dir, "xy_error_distance"); plt.close(fig)

    # 3. ENU各方向位置参考与误差
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, comp, zh in zip(axes, ("E", "N", "U"), ("东向E", "北向N", "高度U")):
        ax.plot(v["t"], v[f"gps_{comp}"], color=COLORS["gps"], lw=1.5,
                label=f"GPS {zh}参考")
        ax.plot(v["t"], v[f"vio_{comp}"], color=COLORS["vio"], lw=1.1,
                label=f"VIO {zh}")
        ax.set_ylabel(f"{zh} / m"); _finish(ax)
    axes[0].set_title("各方向位置对比（仅GPS有效条次）")
    axes[-1].set_xlabel("GPS更新时间 / s")
    _note(fig, "E/N为XY平面坐标；U为高度，三者分别展示。")
    _save(fig, plots_dir, "enu_position_compare"); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, comp, zh in zip(axes, ("E", "N", "U"), ("东向E", "北向N", "高度U")):
        ax.axhline(0, color="#bcc3cc", ls="--", lw=.8)
        ax.plot(v["t"], v[f"err_{comp}"], color=COLORS["err"], lw=1.1,
                label=f"{zh}位置误差（VIO-GPS）")
        ax.set_ylabel("误差 / m"); _finish(ax)
    axes[0].set_title("各方向位置误差")
    axes[-1].set_xlabel("GPS更新时间 / s")
    _save(fig, plots_dir, "enu_position_error"); plt.close(fig)

    # 4. 航线平面分解与高度误差明确分开
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].axhline(0, color="#bcc3cc", ls="--", lw=0.8)
    ax[0].plot(v["cum_dist"] / 1000, v["err_along"], color=COLORS["vio"],
               lw=1.5, label="沿航向位置误差")
    ax[0].plot(v["cum_dist"] / 1000, v["err_cross"], color=COLORS["err"],
               lw=1.5, label="垂直航线位置误差（XY平面）")
    ax[0].set_ylabel("航线平面误差 / m")
    ax[0].set_title("XY平面：沿航向与垂直航线的位置误差分解")
    _finish(ax[0])
    ax[1].axhline(0, color="#bcc3cc", ls="--", lw=0.8)
    ax[1].plot(v["cum_dist"] / 1000, v["err_U"], color=COLORS["lk"],
               lw=1.4, label="高度方向 U 误差")
    ax[1].set_xlabel("GPS累计里程 / km")
    ax[1].set_ylabel("高度误差 / m")
    ax[1].set_title("高度误差（不属于航线平面分解）")
    _finish(ax[1])
    _note(fig, "定义：沿航向与垂直航线均只投影 XY 平面；高度 U/Z 单独统计。")
    _save(fig, plots_dir, "cross_vertical_error"); plt.close(fig)

    # 6. 各方向速度对比（参考来自飞控原始速度）
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, comp, zh in zip(axes, ("E", "N", "U"), ("东向vE", "北向vN", "高度vU")):
        ax.plot(v["t"], v[f"gps_v{comp}"], color=COLORS["gps"], lw=1.4,
                label=f"飞控{zh}参考")
        ax.plot(v["t"], v[f"vio_v{comp}"], color=COLORS["vio"], lw=1.0,
                label=f"VIO {zh}")
        ax.set_ylabel("速度 / m/s"); _finish(ax)
    axes[0].set_title(f"各方向速度对比{fc_note}")
    axes[-1].set_xlabel("GPS更新时间 / s")
    _note(fig, "参考速度直接取飞控GPS的Ve/Vn/Vu列，不由位置相减计算。")
    _save(fig, plots_dir, "enu_velocity_compare"); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, comp, zh in zip(axes, ("E", "N", "U"), ("东向vE", "北向vN", "高度vU")):
        ax.axhline(0, color="#bcc3cc", ls="--", lw=.8)
        ax.plot(v["t"], v[f"err_v{comp}"], color=COLORS["err"], lw=1.0,
                label=f"{zh}误差（VIO-飞控）")
        ax.set_ylabel("误差 / m/s"); _finish(ax)
    axes[0].set_title("各方向速度误差")
    axes[-1].set_xlabel("GPS更新时间 / s")
    _save(fig, plots_dir, "enu_velocity_component_error"); plt.close(fig)

    # 7. XY 平面速度误差按航线方向分解
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax[0].axhline(0, color="#bcc3cc", ls="--", lw=.8)
    ax[0].plot(v["t"], v["err_v_along"], color=COLORS["vio"], lw=1.2,
               label="沿航向速度误差")
    ax[0].plot(v["t"], v["err_v_cross"], color=COLORS["err"], lw=1.2,
               label="垂直航线速度误差（XY平面）")
    ax[0].set_ylabel("速度误差 / m/s")
    ax[0].set_title("XY平面：沿航向与垂直航线的速度误差分解")
    _finish(ax[0])
    ax[1].axhline(0, color="#bcc3cc", ls="--", lw=.8)
    ax[1].plot(v["t"], v["err_vU"], color=COLORS["lk"], lw=1.1,
               label="高度方向速度误差 vU")
    ax[1].set_xlabel("GPS更新时间 / s")
    ax[1].set_ylabel("vU误差 / m/s")
    ax[1].set_title("高度方向速度误差（不属于航线平面分解）")
    _finish(ax[1])
    _note(fig, "正值表示 VIO 在对应方向上的速度分量大于飞控 GPS 参考。")
    _save(fig, plots_dir, "along_cross_velocity_error"); plt.close(fig)
    _save(fig, plots_dir, "enu_velocity_error"); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(v["cum_dist"] / 1000, v["err_speed_xy"], color=COLORS["vio"],
            lw=1.1, label="XY速度大小误差")
    ax.plot(v["cum_dist"] / 1000, v["err_vXY_vec"], color=COLORS["err"],
            lw=1.1, label="XY速度向量误差模")
    ax.set_xlabel("GPS累计里程 / km"); ax.set_ylabel("速度误差 / m/s")
    ax.set_title("XY平面速度误差")
    _finish(ax)
    _note(fig, "速度大小误差可正可负；速度向量误差模始终非负，并包含方向不一致造成的误差。")
    _save(fig, plots_dir, "speed_error_distance"); plt.close(fig)

    # 9. 航向误差
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.axhline(0, color="#bcc3cc", ls="--", lw=0.8)
    ax.plot(v["cum_dist"] / 1000, v["course_err_deg"], color=COLORS["err"], lw=1.4)
    ax.set_xlabel("GPS 累计里程 / km"); ax.set_ylabel("航向误差 / °")
    ax.set_title("VIO course − GPS course")
    _save(fig, plots_dir, "course_yaw_error"); plt.close(fig)

    # 10. 四边一圈分段示意与圈间对比
    complete = seg_err[(seg_err.get("lap_id", -1) > 0) & (seg_err.get("side_id", -1) > 0)].copy()
    if len(complete):
        side_names = {1: "边1 去程主边", 2: "边2 远端连接边",
                      3: "边3 回程主边", 4: "边4 近端连接边"}
        fig, ax = plt.subplots(figsize=(8, 6))
        palette = {1: "#2563c9", 2: "#e07b1a", 3: "#8b5cf6", 4: "#d33a35"}
        for side in range(1, 5):
            m = df["side_id"] == side
            ax.plot(df.loc[m, "gps_E"], df.loc[m, "gps_N"], ".", ms=2.5,
                    color=palette[side], label=side_names[side])
        partial = df["lap_id"] < 0
        ax.plot(df.loc[partial, "gps_E"], df.loc[partial, "gps_N"], ".",
                ms=2, color=COLORS["invalid"], label="起始/结束残段")
        ax.set_xlabel("东向 E / m"); ax.set_ylabel("北向 N / m")
        ax.set_title("航线分段示意：每个完整圈固定四条边")
        ax.set_aspect("equal", adjustable="box"); _finish(ax)
        _note(fig, "边1和边3为相反方向的主航线；边2和边4为两端转向连接边。残段不进入圈间比较。")
        _save(fig, plots_dir, "lap_side_segmentation_map"); plt.close(fig)

        fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharey=True)
        for side, ax in zip(range(1, 5), axes.flat):
            for lap in sorted(complete["lap_id"].unique()):
                part = df[(df["lap_id"] == lap) & (df["side_id"] == side) & df["valid"]]
                if len(part) < 2:
                    continue
                e0 = part[["err_E", "err_N"]].iloc[0].to_numpy()
                local = np.linalg.norm(part[["err_E", "err_N"]].to_numpy() - e0, axis=1)
                ax.plot(part["segment_dist_m"] / 1000, local, lw=1.2, label=f"第{int(lap)}圈")
            ax.set_title(side_names[side]); ax.set_xlabel("进入该边后的航程 / km")
            ax.set_ylabel("相对该边起点的XY误差增长 / m"); _finish(ax)
        fig.suptitle("每圈四条边的局部XY误差剖面")
        fig.subplots_adjust(top=.88, bottom=.13, hspace=.48, wspace=.22)
        _note(fig, "曲线从每条边起点重新归零，用于比较同一条边在不同圈中的误差增长。")
        _save(fig, plots_dir, "straight_leg_error_profiles"); plt.close(fig)

    # 13. GPS 采样质量
    fig, ax = plt.subplots(figsize=(8, 3))
    ng = df[~df["gap"]]
    bad = ng[~ng["valid"]]
    good = ng[ng["valid"]]
    ax.scatter(good["t"], good["vio_delay_s"] * 1000, s=4, color=COLORS["vio"],
               alpha=.5, label="有效GPS条次")
    ax.scatter(bad["t"], bad["vio_delay_s"] * 1000, s=10,
               color=COLORS["err"], label="延迟超阈值")
    p95_ms = 1000 * float(quality.get("p95_delay_s") or 0.0)
    ax.axhline(p95_ms, color=COLORS["err"], ls="--", label=f"95%延迟={p95_ms:.1f} ms")
    ax.set_xlabel("GPS更新时间 / s"); ax.set_ylabel("GPS更新到VIO记录的延迟 / ms")
    ax.set_title(f"GPS条次与更新后VIO状态的时间对齐质量")
    _finish(ax)
    fig.subplots_adjust(bottom=.24)
    _note(fig, f"共{quality['gps_sample_count']}条GPS，{quality['invalid_delay_count']}条延迟无效，"
               f"{quality['gps_gap_count']}个GPS缺口。锯齿来自相机帧相位，不是误差尖峰。")
    _save(fig, plots_dir, "gps_sampling_quality"); plt.close(fig)

    # More component plots can be added without changing the route-frame
    # definition used by the mandatory analysis above.


def plot_plotly_html(df: pd.DataFrame, plots_dir: str):
    """Deprecated: flight_eval no longer emits Plotly HTML artifacts."""
    raise RuntimeError(
        "ERROR: Plotly HTML plotting is deprecated for flight_eval. "
        "Use the SVG v2 / window.RUN_DATA dashboard instead."
    )


def plot_lk_yaw_diagnostic(diag: pd.DataFrame, plots_dir: str):
    """Plot an actual raw-vision diagnostic without pretending it is a trajectory."""
    set_chinese_font()
    import matplotlib.pyplot as plt
    required = {"t", "rate_lk_heading_degps", "omega_gps_degps",
                "n_inliers", "ransac_resid_px"}
    if not required.issubset(diag.columns):
        return
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(diag["t"], diag["rate_lk_heading_degps"], ".", ms=2,
               color=COLORS["vio"], label="纯视觉LK图像旋转率")
    ax[0].plot(diag["t"], diag["omega_gps_degps"], color=COLORS["gps"],
               lw=1, label="GPS航向角速度参考")
    ax[0].set_ylabel("角速度 / °/s"); ax[0].set_title("纯视觉LK航向变化诊断")
    _finish(ax[0])
    ax[1].plot(diag["t"], diag["n_inliers"], color=COLORS["vio"], lw=.9,
               label="RANSAC内点数")
    ax[1].set_ylabel("内点数量"); _finish(ax[1])
    ax[2].plot(diag["t"], diag["ransac_resid_px"], color=COLORS["err"], lw=.9,
               label="RANSAC残差")
    ax[2].set_ylabel("像素"); ax[2].set_xlabel("时间 / s"); _finish(ax[2])
    _note(fig, "这是不使用VIO状态的原始图像光流诊断；它评价视觉航向变化和跟踪质量，不冒充纯视觉位置轨迹。")
    _save(fig, plots_dir, "pure_visual_lk_yaw_diagnostic"); plt.close(fig)
