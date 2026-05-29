#!/bin/bash
# Stage validation: A=global_yaw_oc_projection vs B=visual_yaw_schmidt_current_gauge
# Usage: bash run_stage_schmidt.sh <until_time> <stage_label>
# e.g.:  bash run_stage_schmidt.sh 900  stage1
#         bash run_stage_schmidt.sh 1600 stage2
set -euo pipefail

cd /mnt/d/vscode_dir/open_vins

UNTIL="${1:-900}"
LABEL="${2:-stage1}"

DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free

START=618

DIR_A="$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until${UNTIL}"
DIR_B="$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until${UNTIL}"

run_case() {
  local name="$1"; local out="$2"; shift 2
  if [ -f "$out/traj.txt" ]; then
    echo "[skip] $out/traj.txt already exists — not overwriting"
    return 0
  fi
  mkdir -p "$out"
  echo ""
  echo "========================================"
  echo " $name  until=$UNTIL"
  echo " -> $out"
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
    --vio-yaw-diag "$out/yaw_update_diag.csv" \
    --output "$out/traj.txt" \
    2>&1 | tee "$out/log.txt"
  echo "[done] $name  lines=$(wc -l < "$out/traj.txt")"
}

run_case "A_global_oc" "$DIR_A" \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0

run_case "B_fullstate_schmidt" "$DIR_B" \
  --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
  --schmidt-yaw-diag "$DIR_B/schmidt_yaw_update_diag.csv"

echo ""
echo "Both runs complete. Now generating metrics + plots..."
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --until "$UNTIL" \
  --dir-a "$DIR_A" \
  --dir-b "$DIR_B" \
  --gps "$GPS" \
  --imu /mnt/d/vscode_dir/open_vins/imu_window_fly3_full.csv \
  --out "$BASE/stage_schmidt_eval_until${UNTIL}"
