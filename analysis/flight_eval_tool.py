#!/usr/bin/env python3
"""flight_eval_tool.py — 统一飞行试验评价 CLI（薄分发层）.

主线: single（单条实验完整分析）。其余子命令服务于它或在其后运行。

用法:
  python3 analysis/flight_eval_tool.py single      --run-spec <run>.json --out-root <root>
  python3 analysis/flight_eval_tool.py compare     --runs <d1> <d2> ... --out-dir <cmp>
  python3 analysis/flight_eval_tool.py align-time  --fc-log <raw> --imu-csv <imu> [--camera-csv <cam>] --out <dir>
  python3 analysis/flight_eval_tool.py build-fc-gps --fc-log <raw> --out gps.csv
  python3 analysis/flight_eval_tool.py inspect-run --run-spec <run>.json
  python3 analysis/flight_eval_tool.py dashboard   --analysis-dir <dir>

业务逻辑全在 flight_eval/ 包内；本文件只做参数解析与分发。
"""
from __future__ import annotations

import argparse
import json
import io as _io
import os
import sys

# Windows GBK terminal can't print Unicode (✓ ✗ 中文); force UTF-8 output.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

import numpy as np
import pandas as pd

# 允许从 analysis/ 直接运行
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flight_eval import (  # noqa: E402
    dashboard, gps_sampling, io, metrics, plotting, provenance,
    reports, run_spec, segmentation, time_alignment, trajectory, lk_only,
)


# --------------------------------------------------------------------------- #
# single
# --------------------------------------------------------------------------- #
def cmd_single(args):
    spec = run_spec.load(args.run_spec)
    out_dir = spec.analysis_dir(args.out_root)
    dirs = {k: os.path.join(out_dir, k) for k in ("plots", "tables", "data", "reports", "metadata")}
    for d in dirs.values():
        os.makedirs(d, exist_ok=True)

    st = provenance.StatusTracker()
    command = provenance.write_command(dirs["metadata"])
    run_spec.write_resolved(spec, dirs["metadata"])
    st.ok("inspect", "run spec resolved")

    # ---- 读取 GPS（飞控参考） ----
    gps_full = io.read_gps_csv(spec.inputs["gps_csv"])
    has_vel = gps_full.attrs.get("has_velocity", False)
    ref_src = "flight_controller_raw" if has_vel else "gps_position_diff"
    if not has_vel:
        st.warn("飞控速度列缺失 → 参考速度使用位置差分 fallback")

    # 限制到有效窗口（保留 mask 供 fc_log 行过滤）
    _gps_window_mask = (gps_full["t"] >= spec.t0) & (gps_full["t"] <= spec.t1)
    gps = gps_full[_gps_window_mask].reset_index(drop=True)
    lat0, lon0, alt0 = gps["lat"].iloc[0], gps["lon"].iloc[0], gps.get("alt", pd.Series([0])).iloc[0]
    gps_ENU = trajectory.lla_to_enu(gps["lat"], gps["lon"], gps.get("alt", 0), lat0, lon0, alt0)
    gps_EN = gps_ENU[:, :2]
    gps_U = gps_ENU[:, 2]
    if has_vel:
        gps_vEN = gps[["Ve", "Vn"]].values
        gps_vU = gps["Vu"].values
    else:
        gps_v = trajectory.velocity_from_position(gps["t"].values, gps_EN)
        gps_vEN, gps_vU = gps_v, np.gradient(gps_U, gps["t"].values)
    gps_course = np.arctan2(gps_vEN[:, 1], gps_vEN[:, 0])

    # ---- 读取 VIO（位姿 + 速度优先 .bias） ----
    vio = io.read_tum(spec.inputs["vio_traj"])
    bias = io.read_bias(spec.inputs.get("vio_bias")) if spec.inputs.get("vio_bias") else None
    vio_velocity_source = "traj.bias" if bias is not None else "vio_position_diff"
    st.ok("align", f"vio velocity source = {vio_velocity_source}")

    # ---- GPS 更新时间采样 ----
    gaps = gps_sampling.detect_gps_gaps(gps["t"].values)
    samp = gps_sampling.sample_on_gps_grid(gps["t"].values, vio["t"].values,
                                           spec.sampling["max_delay_s"])
    samp = gps_sampling.mark_gaps(samp, gaps)
    st.ok("sample", f"{samp['valid'].sum()}/{len(samp)} valid")

    # 取每个 GPS 样本对应的 VIO 状态（位置/速度），在有效窗口内
    vio_idx = samp["vio_idx"].clip(lower=0).values
    vio_xy = vio[["x", "y"]].values[vio_idx]
    vio_z = vio["z"].values[vio_idx]
    if bias is not None:
        # 用最近邻把 .bias 速度对到 vio 时间
        bvi = np.searchsorted(bias["t"].values, vio["t"].values[vio_idx]).clip(0, len(bias) - 1)
        vio_vxy = bias[["vx", "vy"]].values[bvi]
        vio_vz = bias["vz"].values[bvi]
    else:
        vio_v = trajectory.velocity_from_position(vio["t"].values, vio[["x", "y"]].values)
        vio_vxy = vio_v[vio_idx]
        vio_vz = np.gradient(vio["z"].values, vio["t"].values)[vio_idx]

    # ---- start-heading 对齐（位置 + 速度同旋转） ----
    h_gps = trajectory.estimate_initial_heading(gps["t"].values, gps_vEN[:, 0], gps_vEN[:, 1],
                                                spec.t0, spec.alignment["course_window_s"])
    # Use the same GPS-aligned time grid and same window so VIO initial heading
    # is estimated from the same first course_window_s seconds as the GPS.
    h_vio = trajectory.estimate_initial_heading(gps["t"].values, vio_vxy[:, 0], vio_vxy[:, 1],
                                                spec.t0, spec.alignment["course_window_s"])
    vio_EN_al, vio_vEN_al, R, dtheta = trajectory.start_align(
        gps_EN, vio_xy, gps_vEN, vio_vxy, h_gps, h_vio)
    vio_U_al = vio_z - vio_z[0] + gps_U[0]
    vio_course = np.arctan2(vio_vEN_al[:, 1], vio_vEN_al[:, 0])

    # ---- LK-only（可选） ----
    lk = lk_only.load_or_build(spec)
    lk_EN = lk_U = None
    if lk.available and lk.traj is not None:
        lk_xy = np.interp(gps["t"].values, lk.traj["t"], lk.traj["E"]), \
                np.interp(gps["t"].values, lk.traj["t"], lk.traj["N"])
        lk_EN = np.column_stack(lk_xy)
        lk_U = np.interp(gps["t"].values, lk.traj["t"], lk.traj["U"])

    # ---- 误差主表 ----
    cum = trajectory.cumulative_distance(gps_EN)
    df = metrics.build_sample_table(
        gps["t"].values, cum,
        gps_EN, gps_U, gps_vEN, gps_vU,
        vio_EN_al, vio_U_al, vio_vEN_al, vio_vz,
        gps_course, vio_course,
        samp["valid"].values, samp["gap"].values,
        lk_EN=lk_EN, lk_U=lk_U)
    df["vio_delay_s"] = samp["vio_delay_s"].values
    st.ok("metrics", "sample table built")

    # ---- 分段 ----
    seg_cfg = segmentation.SegConfig(**spec.segmentation)
    seg_index = segmentation.segment(df, seg_cfg)
    seg_err = segmentation.segment_errors(df, seg_index)
    st.ok("segment", f"{len(seg_index)} segments")

    # ---- 扩充大表：GPS 大地坐标 + VIO 四元数 + bias ----
    # GPS lat/lon/alt/satellites（与 gps 表行对行对齐）
    if "lat" in gps.columns:
        df["gps_lat"] = gps["lat"].values
    if "lon" in gps.columns:
        df["gps_lon"] = gps["lon"].values
    if "alt" in gps.columns:
        df["gps_alt"] = gps["alt"].values
    if "satellites" in gps.columns:
        df["gps_satellites"] = gps["satellites"].values
    # VIO 四元数（由 vio_idx 对齐到最近 VIO 时刻）
    for qcol in ("qx", "qy", "qz", "qw"):
        if qcol in vio.columns:
            df[f"vio_{qcol}"] = vio[qcol].values[vio_idx]
    # VIO bias（bg/ba：由 bvi 对齐，仅在 bias 可用时添加）
    if bias is not None:
        for bcol in ("bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"):
            if bcol in bias.columns:
                df[f"vio_{bcol}"] = bias[bcol].values[bvi]
    # MEMS 姿态角（可选 fc_log，仅当 run spec 提供且与 GPS CSV 行对齐时添加）
    fc_log_rp = spec.inputs.get("fc_log")
    if fc_log_rp is not None and fc_log_rp.state == "ok":
        try:
            raw_fc = pd.read_csv(io.open_bytes(fc_log_rp), encoding="utf-8-sig")
            if raw_fc.shape[1] >= 4:
                # 前三列固定为 pitch/roll/yaw；第四列为 GPS 时间
                raw_fc.columns = (
                    ["mems_pitch_deg", "mems_roll_deg", "mems_yaw_deg"] +
                    list(raw_fc.columns[3:])
                )
                gps_time_col = raw_fc.columns[3]
                # 全量 GPS 有效行
                fc_gps_rows_all = raw_fc[raw_fc[gps_time_col].notna()].reset_index(drop=True)
                # 与全量 GPS CSV 行对行对齐，再用窗口 mask 过滤到窗口内
                if len(fc_gps_rows_all) == len(gps_full):
                    fc_gps_win = fc_gps_rows_all[_gps_window_mask.values].reset_index(drop=True)
                    if len(fc_gps_win) == len(gps):
                        df["mems_pitch_deg"] = fc_gps_win["mems_pitch_deg"].values
                        df["mems_roll_deg"] = fc_gps_win["mems_roll_deg"].values
                        df["mems_yaw_deg"] = fc_gps_win["mems_yaw_deg"].values
                        st.ok("fc_log", f"mems pitch/roll/yaw added ({len(gps)} rows)")
                    else:
                        st.warn(f"fc_log 窗口行数({len(fc_gps_win)})与GPS窗口({len(gps)})不符，跳过姿态列")
                else:
                    st.warn(f"fc_log GPS行数({len(fc_gps_rows_all)})与全量GPS CSV({len(gps_full)})不符，跳过姿态列")
        except Exception as _e:
            st.warn(f"fc_log 读取失败（不阻断）: {_e}")

    # ---- 输出 data/ tables/ ----
    df.to_csv(os.path.join(dirs["data"], "gps_time_aligned_samples.csv"), index=False)
    summary = metrics.global_summary(df)
    pd.DataFrame([summary]).to_csv(os.path.join(dirs["tables"], "global_summary.csv"), index=False)
    quality = gps_sampling.quality_table(samp, gaps, ref_src)
    pd.DataFrame([quality]).to_csv(os.path.join(dirs["tables"], "gps_sampling_quality.csv"), index=False)
    seg_index.to_csv(os.path.join(dirs["tables"], "segment_index.csv"), index=False)
    seg_err.to_csv(os.path.join(dirs["tables"], "segment_error_summary.csv"), index=False)
    seg_err[seg_err["segment_type"] == "straight"].to_csv(
        os.path.join(dirs["tables"], "straight_leg_summary.csv"), index=False)
    seg_err[seg_err["segment_type"].isin(["turn", "transition", "connector"])].to_csv(
        os.path.join(dirs["tables"], "turn_transition_summary.csv"), index=False)
    lk_summary = None
    if lk.diagnostic is not None:
        diag = lk.diagnostic
        lk_summary = {
            "sample_count": int(len(diag)),
            "median_inliers": float(diag["n_inliers"].median()),
            "median_ransac_resid_px": float(diag["ransac_resid_px"].median()),
            "median_flow_px": (
                float(diag["med_flow_px"].median())
                if "med_flow_px" in diag else None
            ),
            "lk_minus_gps_rate_rmse_degps": float(np.sqrt(np.mean(
                (diag["rate_lk_heading_degps"] - diag["omega_gps_degps"]) ** 2
            ))),
        }
        pd.DataFrame([lk_summary]).to_csv(
            os.path.join(dirs["tables"], "pure_visual_lk_summary.csv"), index=False)

    # ---- 图 + 报告 + dashboard ----
    try:
        plotting.plot_all(df, seg_err, quality, dirs["plots"],
                          velocity_source=ref_src, lk_available=lk.available)
        if lk.diagnostic is not None:
            plotting.plot_lk_yaw_diagnostic(lk.diagnostic, dirs["plots"])
        plotting.plot_plotly_html(df, dirs["plots"])
        st.ok("plots", "static + plotly")
    except Exception as e:  # noqa: BLE001
        st.warn(f"绘图部分失败（不阻断）: {e}")

    rep = reports.full_flight_report(spec, summary, quality, seg_err,
                                     reference_velocity_source=ref_src,
                                     vio_velocity_source=vio_velocity_source,
                                     lk_mode=lk.mode)
    reports.write(rep, dirs["reports"], "FULL_FLIGHT_ERROR_ANALYSIS.md")
    reports.write(reports.lk_only_report(spec, lk, lk_summary),
                  dirs["reports"], "LK_ONLY_ANALYSIS.md")

    meta = {
        "experiment_id": spec.experiment_id, "flight": spec.flight_name, "method": spec.method_name,
        "status": spec.status, "t0": spec.t0, "t1": spec.t1,
        "alignment": spec.alignment["mode"], "course_window_s": spec.alignment["course_window_s"],
        "velocity_source": ref_src, "vio_velocity_source": vio_velocity_source,
        "lk": {"available": lk.available, "mode": lk.mode,
               "reason": (', '.join(lk.missing) if lk.missing else '输入文件不可用') if not lk.available else None},
        "out_folder": spec.out_folder_name, "gaps": gaps,
    }
    if not args.no_dashboard:
        payload = dashboard.build_payload(df, seg_index, seg_err, summary, quality, meta)
        dashboard.write_dashboard(payload, dirs["reports"])
        st.ok("report", "markdown + dashboard")
    else:
        st.ok("report", "markdown (dashboard skipped)")

    # ---- provenance ----
    provenance.write_analysis_config(dirs["metadata"], sampling=spec.sampling,
                                     alignment=spec.alignment, seg_cfg=seg_cfg.to_dict())
    out_files = []
    for root, _, files in os.walk(out_dir):
        for fn in files:
            out_files.append(os.path.relpath(os.path.join(root, fn), out_dir))
    provenance.write_provenance(dirs["metadata"], spec=spec, command=command,
                                output_files=out_files,
                                vio_velocity_source=vio_velocity_source,
                                reference_velocity_source=ref_src)
    st.write(dirs["metadata"], spec.status)
    print(f"[single] 完成 → {out_dir}")
    print(f"  终点XY误差={summary['final_xy_error_m']}m  漂移={summary['final_xy_drift_percent']}%  "
          f"参考速度={ref_src}  VIO速度={vio_velocity_source}")


# --------------------------------------------------------------------------- #
# compare（只读已完成的 single 目录）
# --------------------------------------------------------------------------- #
def cmd_compare(args):
    rows = []
    for d in args.runs:
        gs = os.path.join(d, "tables", "global_summary.csv")
        if not os.path.isfile(gs):
            print(f"[compare] 跳过（无 global_summary）: {d}")
            continue
        s = pd.read_csv(gs).iloc[0].to_dict()
        s["run"] = os.path.basename(d.rstrip("/\\"))
        rows.append(s)
    if not rows:
        print("[compare] 无可用 single-run 目录")
        return
    os.makedirs(args.out_dir, exist_ok=True)
    cmp_df = pd.DataFrame(rows).set_index("run")
    cmp_df.to_csv(os.path.join(args.out_dir, "condition_comparison_summary.csv"))
    with open(os.path.join(args.out_dir, "condition_comparison_report.md"), "w", encoding="utf-8") as f:
        f.write("# 多条件对比\n\n（只读已完成的 single-run 目录，不重新解析原始轨迹）\n\n")
        f.write(cmp_df.to_markdown())
    print(f"[compare] 完成 → {args.out_dir}（{len(rows)} 条件）")


# --------------------------------------------------------------------------- #
# align-time
# --------------------------------------------------------------------------- #
def cmd_align_time(args):
    fc = pd.read_csv(args.fc_log)
    imu = pd.read_csv(args.imu_csv)
    cam = pd.read_csv(args.camera_csv) if args.camera_csv else None
    cfg = time_alignment.AlignConfig()
    if args.offset_range:
        lo, hi = (float(x) for x in args.offset_range.split(","))
        cfg.offset_min_s, cfg.offset_max_s = lo, hi
    result = time_alignment.search_offset(fc, imu, cfg, cam)
    time_alignment.write_outputs(result, args.out)
    print(f"[align-time] best offset={result['best']['offset_s']}s → {args.out}")


# --------------------------------------------------------------------------- #
# build-fc-gps
# --------------------------------------------------------------------------- #
def cmd_build_fc_gps(args):
    """从飞控原始日志提取 GPS + 速度（Ve,Vn,Vu）+ 卫星数 → 标准 gps.csv。

    支持 .csv/.tsv（表格归一化）、.ulg（PX4，pyulog）、.bin（ArduPilot，pymavlink）。
    速度取飞控原始 Ve/Vn/Vu（NED 的 Vd 自动转 Vu=-Vd），缺速度列则报错不伪造。
    """
    from flight_eval import fc_gps
    df = fc_gps.build_fc_gps(args.fc_log, args.out)
    print(f"[build-fc-gps] 完成 → {args.out}（{len(df)} 行）")
    has_v = df[["Ve", "Vn", "Vu"]].notna().all(axis=None)
    print(f"  速度列完整={bool(has_v)}  时间范围={df['ts_ns'].iloc[0]*1e-9:.2f}~{df['ts_ns'].iloc[-1]*1e-9:.2f}s")


# --------------------------------------------------------------------------- #
# inspect-run
# --------------------------------------------------------------------------- #
def cmd_inspect_run(args):
    try:
        spec = run_spec.load(args.run_spec)
    except run_spec.RunSpecError as e:
        print(f"[inspect-run] ✗ run spec 无效: {e}")
        sys.exit(1)
    print(f"[inspect-run] {spec.experiment_id} → 输出目录名: {spec.out_folder_name}")
    ok = True
    for name, rp in spec.inputs.items():
        flag = "✓" if rp.state == "ok" else ("·" if name in run_spec.OPTIONAL_INPUTS else "✗")
        if rp.state != "ok" and name not in run_spec.OPTIONAL_INPUTS:
            ok = False
        print(f"  {flag} {name:14s} {rp.state:12s} {rp.reason}")
    print("[inspect-run] " + ("具备分析条件 ✓" if ok else "缺少必填输入 ✗"))
    sys.exit(0 if ok else 1)


# --------------------------------------------------------------------------- #
# dashboard（仅重建页面）
# --------------------------------------------------------------------------- #
def cmd_dashboard(args):
    d = args.analysis_dir
    df = pd.read_csv(os.path.join(d, "data", "gps_time_aligned_samples.csv"))
    seg_index = pd.read_csv(os.path.join(d, "tables", "segment_index.csv"))
    seg_err = pd.read_csv(os.path.join(d, "tables", "segment_error_summary.csv"))
    summary = pd.read_csv(os.path.join(d, "tables", "global_summary.csv")).iloc[0].to_dict()
    quality = pd.read_csv(os.path.join(d, "tables", "gps_sampling_quality.csv")).iloc[0].to_dict()
    # 从 metadata 还原完整 meta，使重建页面的溯源/速度标注与首次一致
    rs_path = os.path.join(d, "metadata", "run_spec_resolved.json")
    meta = {"experiment_id": os.path.basename(d), "out_folder": os.path.basename(d), "gaps": []}
    if os.path.isfile(rs_path):
        with open(rs_path, "r", encoding="utf-8") as f:
            rs = json.load(f)
        vs = rs.get("velocity_source", {})
        meta.update({
            "experiment_id": rs.get("experiment_id", meta["experiment_id"]),
            "flight": rs.get("flight_name", ""), "method": rs.get("method_name", ""),
            "status": rs.get("status", ""), "t0": rs.get("t0"), "t1": rs.get("t1"),
            "alignment": (rs.get("alignment") or {}).get("mode", ""),
            "course_window_s": (rs.get("alignment") or {}).get("course_window_s", ""),
            "velocity_source": vs.get("reference", quality.get("velocity_source", "")),
            "vio_velocity_source": vs.get("vio", ""),
        })
    # 还原 LK 模式（从 LK 报告标题判断是否可用）
    lk_rep = os.path.join(d, "reports", "LK_ONLY_ANALYSIS.md")
    if os.path.isfile(lk_rep):
        txt = open(lk_rep, "r", encoding="utf-8").read()
        avail = "不可用" not in txt and "unavailable" not in txt.lower()
        mode = "LK_ONLY_DIAGNOSTIC" if "LK_ONLY_DIAGNOSTIC" in txt else ("trajectory" if avail else "unavailable")
        meta["lk"] = {"available": avail, "mode": mode}
    # 还原缺口（从采样质量推断不到精确区间时留空，前端按 gap 标记降级）
    payload = dashboard.build_payload(df, seg_index, seg_err, summary, quality, meta)
    p = dashboard.write_dashboard(payload, os.path.join(d, "reports"))
    print(f"[dashboard] 重建 → {p}")


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="OpenVINS 飞行试验评价工具")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("single", help="单条实验完整分析（主功能）")
    p.add_argument("--run-spec", required=True)
    p.add_argument("--out-root", default="C:/Users/baloney/Desktop/实验目录")
    p.add_argument("--force", action="store_true")
    p.add_argument("--no-dashboard", action="store_true")
    p.set_defaults(func=cmd_single)

    p = sub.add_parser("compare", help="多条件对比（只读已完成目录）")
    p.add_argument("--runs", nargs="+", required=True)
    p.add_argument("--out-dir", required=True)
    p.set_defaults(func=cmd_compare)

    p = sub.add_parser("align-time", help="FC/IMU/camera 时间戳对齐")
    p.add_argument("--fc-log", required=True)
    p.add_argument("--imu-csv", required=True)
    p.add_argument("--camera-csv")
    p.add_argument("--offset-range", help="如 -5,5")
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_align_time)

    p = sub.add_parser("build-fc-gps", help="从飞控日志提取 GPS+速度")
    p.add_argument("--fc-log", required=True)
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_build_fc_gps)

    p = sub.add_parser("inspect-run", help="检查实验是否具备分析条件")
    p.add_argument("--run-spec", required=True)
    p.set_defaults(func=cmd_inspect_run)

    p = sub.add_parser("dashboard", help="仅重建离线交互页")
    p.add_argument("--analysis-dir", required=True)
    p.set_defaults(func=cmd_dashboard)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
