#!/usr/bin/env python3
"""Run evaluate_gps_start_yaw on B0 iwt=2.0 and iwt=5.0 for all 4 flights."""
import sys, argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from evaluate_gps_start_yaw import evaluate_one

CASES = [
    {"fly": "fly1", "gps": "/mnt/d/vscode_dir/20260509_fly1_gps/GPS.csv", "traj": [
        "20260509_fly1/result/baselines_v1/B0_no_gps.txt",
        "20260509_fly1/result/init_window_time_sweep/B0_iwt5.0.txt",
    ]},
    {"fly": "fly2", "gps": "/mnt/d/vscode_dir/20260509_fly2_gps/GPS.csv", "traj": [
        "20260509_fly2/result/baselines_v1/B0_no_gps.txt",
        "20260509_fly2/result/init_window_time_sweep/B0_iwt5.0.txt",
    ]},
    {"fly": "fly3", "gps": "/mnt/d/vscode_dir/20260509_fly3_gps/GPS.csv", "traj": [
        "20260509_fly3/result/baselines_v1/B0_no_gps.txt",
        "20260509_fly3/result/init_window_time_sweep/B0_iwt5.0.txt",
    ]},
    {"fly": "fly4", "gps": "/mnt/d/vscode_dir/20260509_fly4_gps/GPS.csv", "traj": [
        "20260509_fly4/result/stage_a_v2/R0_nogps.txt",
        "20260509_fly4/result/init_window_time_sweep/B0_iwt5.0.txt",
    ]},
]

OUT = ROOT / "evaluation_iwt5_vs_iwt2"
OUT.mkdir(parents=True, exist_ok=True)

args = argparse.Namespace(scan_min=-300, scan_max=300, direction_window_s=30, min_edge_len_m=45)

for case in CASES:
    fly = case["fly"]
    gps_path = Path(case["gps"])
    for traj_rel in case["traj"]:
        traj_path = ROOT / traj_rel
        if not traj_path.exists():
            print(f"SKIP {fly}/{traj_path.name}: not found")
            continue
        print(f"EVAL {fly}: {traj_path.name}")
        try:
            row, edge_rows = evaluate_one(fly, gps_path, traj_path, OUT, args)
        except Exception as e:
            print(f"  FAIL: {e}")
            import traceback; traceback.print_exc()
            continue
        print(f"  offset={row['time_offset_s']:.2f}s yaw={row['yaw_align_deg']:.1f}deg "
              f"ATE2D={row['ate2d_rmse_m']:.1f}m edges={row['edges_detected']}")

print(f"\nDone -> {OUT}")
