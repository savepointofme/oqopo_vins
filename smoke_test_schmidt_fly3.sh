#!/bin/bash
# Smoke test: A = global_yaw_oc_projection vs B = visual_yaw_schmidt_current_gauge
# Dataset: fly3, offset438p0, start=618, until=738 (120s window)
# Binary: ./build_ov_msckf/run_serial_msckf_ros_free  (ENABLE_ROS=OFF, repo-root build)
set -euo pipefail

cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
ROOT=$BASE/schmidt_smoke_test_start618
BIN=./build_ov_msckf/run_serial_msckf_ros_free

START=618
UNTIL=738    # 120s window

run_case() {
  local name="$1"; shift
  local out="$ROOT/$name"
  mkdir -p "$out"
  echo ""
  echo "========================================"
  echo " Running: $name"
  echo "========================================"
  "$BIN" \
    --config "$CONFIG" \
    --dataset "$DATASET" \
    --gps "$GPS" \
    --gps-time-offset 0 \
    --start-time "$START" \
    --until-time "$UNTIL" \
    --init-from-fc "$FC" \
    --init-bg-sigma 0.003 \
    "$@" \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --no-display \
    --diag-csv "$out/diag.csv" \
    --vio-yaw-diag "$out/yaw_update.csv" \
    --output "$out/traj.txt" \
    2>&1 | tee "$out/log.txt"
  local lines=0
  [ -f "$out/traj.txt" ] && lines=$(wc -l < "$out/traj.txt")
  echo "[done] $name  traj_lines=$lines"
}

run_case A_global_oc \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0

run_case B_schmidt \
  --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
  --schmidt-yaw-diag "$ROOT/B_schmidt/schmidt_yaw_update_diag.csv"
