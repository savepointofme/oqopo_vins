#!/bin/bash
# Fly3 two-mode baseline:
#   A) original  — standard EKF + GPS-Z only (no yaw OC, no Schmidt)
#   B) oc_alpha1 — full H-space yaw-OC projection (alpha=1) + GPS-Z
#
# Alpha is binary after the guard added in StateHelper.cpp:
#   0          → standard EKF (no projection)
#   any value > 0 → snapped to 1.0 (full OC projection)
#
# Usage: bash run_fly3_baseline.sh [A|B|both]   (default: both)
set -euo pipefail

cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
ROOT=$BASE/baseline_two_modes_offset438p0_start618
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free

CASE=${1:-both}

run_case() {
  local name="$1"; shift
  local out="$ROOT/$name"
  rm -rf "$out"
  mkdir -p "$out"
  echo "[baseline] running $name ..."
  $BIN \
    --config "$CONFIG" \
    --dataset "$DATASET" \
    --gps "$GPS" \
    --gps-time-offset 0 \
    --start-time 618 \
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
    --viz-fast \
    --dash-every 5 \
    --diag-csv "$out/diag.csv" \
    --diag-events "$out/events.txt" \
    --vio-yaw-diag "$out/yaw.csv" \
    --output "$out/traj.txt" \
    2>&1 | tee "$out/log.txt"
  echo "[baseline] done $name — traj last line:"
  tail -n 1 "$out/traj.txt" 2>/dev/null || echo "(no traj)"
}

if [[ "$CASE" == "A" || "$CASE" == "both" ]]; then
  run_case A_original \
    --vio-yaw-update-mode original
fi

if [[ "$CASE" == "B" || "$CASE" == "both" ]]; then
  run_case B_oc_alpha1 \
    --vio-yaw-update-mode global_yaw_oc_projection \
    --vio-global-yaw-oc-alpha 1.0
fi
