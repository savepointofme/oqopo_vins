#!/bin/bash
# global_yaw_oc_fej_prechi2 evaluation on fly1–fly4
# Each flight uses the same flags as its canonical global_oc_alpha1 run
# with only the yaw mode replaced by the FEJ pre-chi2 OC mode.
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins
BIN=./build_ov_msckf/run_serial_msckf_ros_free
CONFIG_FLY12=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
CONFIG_FLY1=/mnt/d/vscode_dir/open_vins/config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml

echo "[all-flights-fej-oc] start $(date)  HEAD=$(git rev-parse HEAD)"

traj_complete() {
  local f="$1/traj.txt" until="$2"
  [ -f "$f" ] || return 1
  local last; last=$(tail -1 "$f" | awk '{print $1}')
  python3 -c "import sys; sys.exit(0 if float('${last:-0}') >= $until - 20 else 1)" 2>/dev/null
}

# ── Fly 1 ─────────────────────────────────────────────────────────────────────
FLY1_DATASET=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
FLY1_BASE=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result
FLY1_GPS=/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv
FLY1_FC=/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_init_state_930.csv
FLY1_START=930; FLY1_UNTIL=1750
DIR_FLY1=$FLY1_BASE/fej_oc_prechi2_start930

if traj_complete "$DIR_FLY1" $FLY1_UNTIL; then
  echo "[skip fly1] complete"
else
  rm -rf "$DIR_FLY1"; mkdir -p "$DIR_FLY1"
  echo "[START fly1]"
  "$BIN" \
    --config "$CONFIG_FLY1" \
    --dataset "$FLY1_DATASET" \
    --gps "$FLY1_GPS" \
    --gps-time-offset 0 \
    --start-time "$FLY1_START" \
    --until-time "$FLY1_UNTIL" \
    --init-from-fc "$FLY1_FC" \
    --init-bg-sigma 0.003 \
    --vio-yaw-update-mode global_yaw_oc_fej_prechi2 \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --viz-fast --dash-every 5 \
    --diag-csv "$DIR_FLY1/diag.csv" \
    --output "$DIR_FLY1/traj.txt" \
    > "$DIR_FLY1/log.txt" 2>&1 &
  PID_FLY1=$!
  echo "[fly1 pid=$PID_FLY1]"
fi

# ── Fly 2 ─────────────────────────────────────────────────────────────────────
FLY2_DATASET=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722
FLY2_BASE=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525
FLY2_GPS=$FLY2_BASE/gps_from_mems_offset450p5_cam_time.csv
FLY2_FC=$FLY2_BASE/canonical_offset450p5_start700/fc_init_state_700_offset450p5.csv
FLY2_START=700; FLY2_UNTIL=2600
DIR_FLY2=$FLY2_BASE/fej_oc_prechi2_start700

if traj_complete "$DIR_FLY2" $FLY2_UNTIL; then
  echo "[skip fly2] complete"
else
  rm -rf "$DIR_FLY2"; mkdir -p "$DIR_FLY2"
  echo "[START fly2]"
  "$BIN" \
    --config "$CONFIG_FLY12" \
    --dataset "$FLY2_DATASET" \
    --gps "$FLY2_GPS" \
    --gps-time-offset 0 \
    --start-time "$FLY2_START" \
    --until-time "$FLY2_UNTIL" \
    --init-from-fc "$FLY2_FC" \
    --init-bg-sigma 0.003 \
    --vio-yaw-update-mode global_yaw_oc_fej_prechi2 \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --viz-fast --dash-every 5 \
    --diag-csv "$DIR_FLY2/diag.csv" \
    --output "$DIR_FLY2/traj.txt" \
    > "$DIR_FLY2/log.txt" 2>&1 &
  PID_FLY2=$!
  echo "[fly2 pid=$PID_FLY2]"
fi

# ── Fly 3 ─────────────────────────────────────────────────────────────────────
FLY3_DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
FLY3_BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
FLY3_GPS=$FLY3_BASE/gps_from_mems_offset438p0_cam_time.csv
FLY3_FC=$FLY3_BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
FLY3_START=618; FLY3_UNTIL=1800
DIR_FLY3=$FLY3_BASE/fej_oc_prechi2_start618

if traj_complete "$DIR_FLY3" $FLY3_UNTIL; then
  echo "[skip fly3] complete"
else
  rm -rf "$DIR_FLY3"; mkdir -p "$DIR_FLY3"
  echo "[START fly3]"
  "$BIN" \
    --config "$CONFIG_FLY12" \
    --dataset "$FLY3_DATASET" \
    --gps "$FLY3_GPS" \
    --gps-time-offset 0 \
    --start-time "$FLY3_START" \
    --until-time "$FLY3_UNTIL" \
    --init-from-fc "$FLY3_FC" \
    --init-bg-sigma 0.003 \
    --vio-yaw-update-mode global_yaw_oc_fej_prechi2 \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --viz-fast --dash-every 5 \
    --diag-csv "$DIR_FLY3/diag.csv" \
    --output "$DIR_FLY3/traj.txt" \
    > "$DIR_FLY3/log.txt" 2>&1 &
  PID_FLY3=$!
  echo "[fly3 pid=$PID_FLY3]"
fi

# ── Fly 4 ─────────────────────────────────────────────────────────────────────
FLY4_DATASET=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549
FLY4_BASE=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528
FLY4_GPS=$FLY4_BASE/gps_from_mems_offsetm202p2_cam_time.csv
FLY4_FC=$FLY4_BASE/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv
FLY4_START=924.4; FLY4_UNTIL=2816
DIR_FLY4=$FLY4_BASE/fej_oc_prechi2_start924p4

if traj_complete "$DIR_FLY4" $FLY4_UNTIL; then
  echo "[skip fly4] complete"
else
  rm -rf "$DIR_FLY4"; mkdir -p "$DIR_FLY4"
  echo "[START fly4]"
  "$BIN" \
    --config "$CONFIG_FLY12" \
    --dataset "$FLY4_DATASET" \
    --gps "$FLY4_GPS" \
    --gps-time-offset 0 \
    --start-time "$FLY4_START" \
    --until-time "$FLY4_UNTIL" \
    --init-from-fc "$FLY4_FC" \
    --init-bg-sigma 0.003 \
    --vio-yaw-update-mode global_yaw_oc_fej_prechi2 \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --viz-fast --dash-every 5 \
    --diag-csv "$DIR_FLY4/diag.csv" \
    --output "$DIR_FLY4/traj.txt" \
    > "$DIR_FLY4/log.txt" 2>&1 &
  PID_FLY4=$!
  echo "[fly4 pid=$PID_FLY4]"
fi

# Wait for any background jobs
for pid in ${PID_FLY1:-} ${PID_FLY2:-} ${PID_FLY3:-} ${PID_FLY4:-}; do
  [ -n "$pid" ] && { wait "$pid" || true; }
done

echo "[all-flights-fej-oc] ALL DONE $(date)"
for label dir in \
    fly1 "$FLY1_BASE/fej_oc_prechi2_start930" \
    fly2 "$FLY2_BASE/fej_oc_prechi2_start700" \
    fly3 "$FLY3_BASE/fej_oc_prechi2_start618" \
    fly4 "$FLY4_BASE/fej_oc_prechi2_start924p4"; do
  f="$dir/traj.txt"
  if [ -f "$f" ]; then
    lines=$(wc -l < "$f")
    last=$(tail -1 "$f" | awk '{print $1}')
    echo "  $label: $lines lines  t=$last"
  else
    echo "  $label: MISSING"
  fi
done
