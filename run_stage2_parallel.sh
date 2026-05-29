#!/bin/bash
# Stage 2: A and B in parallel, until=1600s, with --viz-fast
# Mirrors the reference script pattern exactly.
set -uo pipefail   # NOTE: no -e — let each binary run to natural completion

cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free

DIR_A=$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until1600
DIR_B=$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until1600

mkdir -p "$DIR_A" "$DIR_B"

echo "[stage2] Launching A (global_yaw_oc_projection) ..."
"$BIN" \
  --config "$CONFIG" \
  --dataset "$DATASET" \
  --gps "$GPS" \
  --gps-time-offset 0 \
  --start-time 618 \
  --until-time 1600 \
  --init-from-fc "$FC" \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --viz-fast \
  --dash-every 5 \
  --diag-csv "$DIR_A/diag.csv" \
  --vio-yaw-diag "$DIR_A/yaw_update_diag.csv" \
  --output "$DIR_A/traj.txt" \
  > "$DIR_A/log.txt" 2>&1 &
PID_A=$!
echo "[stage2] A PID=$PID_A"

echo "[stage2] Launching B (visual_yaw_schmidt_current_gauge) ..."
"$BIN" \
  --config "$CONFIG" \
  --dataset "$DATASET" \
  --gps "$GPS" \
  --gps-time-offset 0 \
  --start-time 618 \
  --until-time 1600 \
  --init-from-fc "$FC" \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --viz-fast \
  --dash-every 5 \
  --diag-csv "$DIR_B/diag.csv" \
  --vio-yaw-diag "$DIR_B/yaw_update_diag.csv" \
  --schmidt-yaw-diag "$DIR_B/schmidt_yaw_update_diag.csv" \
  --output "$DIR_B/traj.txt" \
  > "$DIR_B/log.txt" 2>&1 &
PID_B=$!
echo "[stage2] B PID=$PID_B"

echo "[stage2] Both running. Waiting..."
wait $PID_A; STATUS_A=$?
wait $PID_B; STATUS_B=$?

echo ""
echo "[stage2] A exit=$STATUS_A  B exit=$STATUS_B"
for TAG_DIR in "A:$DIR_A" "B:$DIR_B"; do
  TAG="${TAG_DIR%%:*}"; D="${TAG_DIR##*:}"
  if [ -f "$D/traj.txt" ]; then
    LINES=$(wc -l < "$D/traj.txt")
    FINAL=$(tail -1 "$D/traj.txt" | awk '{print $1}')
    echo "[stage2] $TAG: lines=$LINES  final_t=$FINAL"
  else
    echo "[stage2] $TAG: traj.txt MISSING"
  fi
done
