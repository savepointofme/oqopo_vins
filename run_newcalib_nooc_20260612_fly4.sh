#!/usr/bin/env bash
set -euo pipefail

REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
FLIGHT_ROOT="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4"
DATASET="$FLIGHT_ROOT/d455_20260527_090549"
GPS="$FLIGHT_ROOT/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
FC_INIT="$FLIGHT_ROOT/result/fc_rebuild_20260528/canonical_offsetm202p2_start924p4/fc_init_state_924p4_offsetm202p2.csv"
SOURCE="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_20260612_fly4/config"
OUT="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_NOOC_20260612_fly4"
CFG="$OUT/config"

mkdir -p "$CFG"
cp "$SOURCE/config_new.yaml" "$CFG/config_new.yaml"
cp "$SOURCE/config_old.yaml" "$CFG/config_old.yaml"
cp "$SOURCE/config_diff.txt" "$CFG/config_diff.txt"
cp "$SOURCE/kalibr_imucam_chain.yaml" "$CFG/kalibr_imucam_chain.yaml"
cp "$SOURCE/kalibr_imucam_chain_old.yaml" "$CFG/kalibr_imucam_chain_old.yaml"
cp "$SOURCE/kalibr_imu_chain.yaml" "$CFG/kalibr_imu_chain.yaml"
cp "$SOURCE/kalibr_imu_chain_old.yaml" "$CFG/kalibr_imu_chain_old.yaml"

cat > "$OUT/CONTROLLED_CHANGE.txt" <<'EOF'
Compared with NEWCALIB_20260612_fly4:
  changed: global_yaw_oc_projection alpha=1.0 -> original (OpenVINS FEJ, no OC)
  unchanged: camera/IMU calibration, online camera intrinsics/extrinsics/time-offset
             calibration, GPS-Z, FC initialization, feature settings, clone size,
             update thresholds, IMU noise, and evaluation window.
EOF

cmd=(
  "$BINARY"
  --config "$CFG/config_new.yaml"
  --dataset "$DATASET"
  --gps "$GPS"
  --gps-time-offset 0
  --start-time 924.4
  --init-from-fc "$FC_INIT"
  --vio-yaw-update-mode original
  --init-bg-sigma 0.003
  --gps-alt-update
  --gps-alt-sigma 2.0
  --gps-alt-min-pzz 0.01
  --gps-alt-max-res 80
  --gps-alt-min-t-after-init 10
  --gps-alt-guard-dxy 0.5
  --gps-alt-guard-kxy 5.0
  --diag-csv "$OUT/diag.csv"
  --vio-yaw-diag "$OUT/vio_yaw_diag.csv"
  --viz-fast
  --dash-every 5
  --output "$OUT/traj.txt"
)

printf '%q ' "${cmd[@]}" > "$OUT/command.txt"
printf '\n' >> "$OUT/command.txt"
"${cmd[@]}" > "$OUT/log.txt" 2>&1

python3 "$REPO/eval_stage.py" \
  --t0 924.4 --until 2816 \
  --dir-a "$OUT" --dir-b "$OUT" \
  --gps "$GPS" --imu "$DATASET/imu0/data.csv" \
  --out "$OUT/forensics_valid" \
  --yaw-align-mode start_yaw

python3 "$REPO/analysis/yaw_leak_analysis.py" \
  --run-dir "$OUT" --gps "$GPS" \
  --t0 924.4 --t1 2816 \
  --out "$OUT/leak_analysis"
