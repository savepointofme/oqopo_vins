#!/usr/bin/env python3
"""Generate a self-contained P4-versus-global cross-flight comparison page.

All metrics and time-series inputs come from completed official
``flight_eval_tool.py single`` output directories.  This script only renders
those already-standardized GPS-time/start-heading samples; it does not change
the evaluation alignment or recompute estimator outputs.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


RUN_NAMES = {
    "fly1": {
        "global": "20260717_fly1_historical_global_baseline_oc_stride12_reference",
        "p4": "20260717_fly1_p4_direct_sliding_native_handoff_candidate",
    },
    "fly2": {
        "global": "20260717_fly2_historical_global_baseline_oc_stride12_reference",
        "p4": "20260717_fly2_p4_direct_sliding_native_handoff_candidate",
    },
    "fly3": {
        "global": "20260717_fly3_historical_global_baseline_oc_stride12_reference",
        "p4": "20260717_fly3_p4_direct_sliding_native_handoff_candidate",
    },
}

METRICS = (
    ("xy_rmse_m", "XY RMSE", "m"),
    ("final_xy_error_m", "终点 XY", "m"),
    ("speed_rmse_mps", "速度 RMSE", "m/s"),
    ("vxy_vec_rmse_mps", "水平速度向量 RMSE", "m/s"),
    ("yaw_or_course_rmse_deg", "航向 RMSE", "deg"),
    ("yaw_or_course_final_deg", "终点航向误差", "deg"),
    ("final_vertical_error_m", "终点高度误差", "m"),
)


def read_summary(run_dir: Path) -> dict[str, float]:
    path = run_dir / "tables" / "global_summary.csv"
    row = pd.read_csv(path).iloc[0]
    return {key: float(row[key]) for key, _, _ in METRICS}


def read_samples(run_dir: Path) -> pd.DataFrame:
    path = run_dir / "data" / "gps_time_aligned_samples.csv"
    data = pd.read_csv(path)
    data = data[data["valid"].astype(bool)].copy()
    if data.empty:
        raise RuntimeError(f"no valid samples: {path}")
    data["elapsed_s"] = data["t"] - float(data["t"].iloc[0])
    return data


def plot_flight(flight: str, runs: dict[str, dict], out_dir: Path) -> Path:
    global_data = runs["global"]["samples"]
    p4_data = runs["p4"]["samples"]
    global_summary = runs["global"]["summary"]
    p4_summary = runs["p4"]["summary"]

    fig, axes = plt.subplots(2, 3, figsize=(19, 10.5), constrained_layout=True)
    gps = p4_data
    axes[0, 0].plot(gps["gps_E"].to_numpy(), gps["gps_N"].to_numpy(), color="black", lw=2.2, label="GPS reference")
    axes[0, 0].plot(
        global_data["vio_E"].to_numpy(), global_data["vio_N"].to_numpy(),
        color="#d62728", lw=1.4, label="Global baseline",
    )
    axes[0, 0].plot(
        p4_data["vio_E"].to_numpy(), p4_data["vio_N"].to_numpy(),
        color="#1f77b4", lw=1.4, label="Current P4",
    )
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 0].set_title("Horizontal trajectory (start-heading only)")
    axes[0, 0].set_xlabel("East [m]")
    axes[0, 0].set_ylabel("North [m]")
    axes[0, 0].legend(loc="best")

    panels = (
        ("err_XY", "XY error", "m", axes[0, 1]),
        ("course_err_deg", "Course error", "deg", axes[0, 2]),
        ("err_cross", "Cross-track error", "m", axes[1, 0]),
        ("err_speed_xy", "Horizontal speed error", "m/s", axes[1, 1]),
        ("err_U", "Vertical error", "m", axes[1, 2]),
    )
    for column, title, unit, axis in panels:
        axis.plot(
            global_data["elapsed_s"].to_numpy(), global_data[column].to_numpy(),
            color="#d62728", lw=1.1, label="Global baseline",
        )
        axis.plot(
            p4_data["elapsed_s"].to_numpy(), p4_data[column].to_numpy(),
            color="#1f77b4", lw=1.1, label="Current P4",
        )
        axis.axhline(0.0, color="#777777", lw=0.7, alpha=0.6)
        axis.set_title(title)
        axis.set_xlabel("Elapsed time [s]")
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.22)
        axis.legend(loc="best")

    fig.suptitle(
        f"{flight.upper()} — P4 vs historical global baseline\n"
        f"XY RMSE {p4_summary['xy_rmse_m']:.2f} / {global_summary['xy_rmse_m']:.2f} m, "
        f"final XY {p4_summary['final_xy_error_m']:.2f} / {global_summary['final_xy_error_m']:.2f} m, "
        f"course RMSE {p4_summary['yaw_or_course_rmse_deg']:.2f} / {global_summary['yaw_or_course_rmse_deg']:.2f} deg "
        "(P4 / global)",
        fontsize=15,
    )
    output = out_dir / f"{flight}_p4_vs_global.png"
    fig.savefig(output, dpi=150)
    plt.close(fig)
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval-root", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    payload: dict[str, dict] = {}
    table_rows: list[dict[str, str | float]] = []
    image_paths: dict[str, Path] = {}
    for flight, condition_names in RUN_NAMES.items():
        runs: dict[str, dict] = {}
        payload[flight] = {}
        for condition, run_name in condition_names.items():
            run_dir = args.eval_root / run_name
            if not run_dir.is_dir():
                raise FileNotFoundError(run_dir)
            summary = read_summary(run_dir)
            samples = read_samples(run_dir)
            runs[condition] = {"dir": run_dir, "summary": summary, "samples": samples}
            payload[flight][condition] = {
                "run_name": run_name,
                "summary": summary,
                "t0": float(samples["t"].iloc[0]),
                "t1": float(samples["t"].iloc[-1]),
                "sample_count": int(len(samples)),
            }
        image_paths[flight] = plot_flight(flight, runs, args.out_dir)
        for key, label, unit in METRICS:
            global_value = runs["global"]["summary"][key]
            p4_value = runs["p4"]["summary"][key]
            table_rows.append(
                {
                    "flight": flight,
                    "metric": label,
                    "unit": unit,
                    "global": global_value,
                    "p4": p4_value,
                    "delta": p4_value - global_value,
                }
            )

    csv_path = args.out_dir / "p4_vs_global_metrics.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=table_rows[0].keys())
        writer.writeheader()
        writer.writerows(table_rows)

    rows_html = []
    for row in table_rows:
        delta = float(row["delta"])
        css = "better" if delta < 0 else ("worse" if delta > 0 else "same")
        rows_html.append(
            "<tr>"
            f"<td>{html.escape(str(row['flight']))}</td>"
            f"<td>{html.escape(str(row['metric']))}</td>"
            f"<td>{float(row['global']):.3f}</td>"
            f"<td>{float(row['p4']):.3f}</td>"
            f"<td class='{css}'>{delta:+.3f}</td>"
            f"<td>{html.escape(str(row['unit']))}</td>"
            "</tr>"
        )

    figures_html = []
    for flight in RUN_NAMES:
        p4_run = RUN_NAMES[flight]["p4"]
        global_run = RUN_NAMES[flight]["global"]
        figures_html.append(
            f"<section><h2>{flight.upper()}</h2>"
            f"<img src='{image_paths[flight].name}' alt='{flight} comparison'>"
            "<p>Interactive: "
            f"<a href='../p4_crossflight_v14_20260717/{p4_run}/reports/interactive_dashboard.html'>Current P4</a> · "
            f"<a href='../p4_crossflight_v14_20260717/{global_run}/reports/interactive_dashboard.html'>Global baseline</a>"
            "</p></section>"
        )

    output_html = args.out_dir / "index.html"
    output_html.write_text(
        "<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>P4 cross-flight validation</title>"
        "<style>body{font-family:Segoe UI,Arial,sans-serif;margin:24px;background:#f5f7fa;color:#18202a}"
        "main{max-width:1800px;margin:auto}section,.card{background:white;border:1px solid #dce2e8;border-radius:10px;"
        "padding:18px;margin:18px 0;box-shadow:0 2px 8px #00000012}img{width:100%;height:auto}"
        "table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}th,td{padding:8px 10px;"
        "border-bottom:1px solid #e4e8ed;text-align:right}th:nth-child(-n+2),td:nth-child(-n+2){text-align:left}"
        ".better{color:#087f23;font-weight:600}.worse{color:#b42318;font-weight:600}.same{color:#555}"
        "a{color:#075cab}</style></head><body><main>"
        "<h1>P4 vs historical global baseline — cross-flight validation</h1>"
        "<p>GPS update-time sampling; one start-heading rotation only; no best-fit, scale, translation or SE(3) post-alignment. "
        "The historical baseline retains its original global-yaw-OC runtime path, so the page reports observed end-to-end behavior, not a single-factor causal ablation.</p>"
        "<div class='card'><table><thead><tr><th>Flight</th><th>Metric</th><th>Global</th><th>P4</th>"
        "<th>Δ P4−Global</th><th>Unit</th></tr></thead><tbody>"
        + "".join(rows_html)
        + "</tbody></table></div>"
        + "".join(figures_html)
        + "<script>window.RUN_DATA="
        + json.dumps(payload, ensure_ascii=False, allow_nan=False)
        + ";</script></main></body></html>",
        encoding="utf-8",
    )
    print(output_html)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
