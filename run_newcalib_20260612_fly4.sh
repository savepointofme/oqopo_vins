#!/usr/bin/env bash
set -euo pipefail

REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
FLIGHT_ROOT="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4"
DATASET="$FLIGHT_ROOT/d455_20260527_090549"
GPS="$FLIGHT_ROOT/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
FC_INIT="$FLIGHT_ROOT/result/fc_rebuild_20260528/canonical_offsetm202p2_start924p4/fc_init_state_924p4_offsetm202p2.csv"
BASELINE="$FLIGHT_ROOT/result/sigma1_stab_20260610/A_fly4"
OUT="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_20260612_fly4"
CFG="$OUT/config"
NEW_CALIB="/mnt/c/Users/baloney/Desktop/results_20260612"

mkdir -p "$CFG"

cp "$BASELINE/config_snapshot.yaml" "$CFG/config_new.yaml"
sed \
  -e 's/relative_config_imu: "kalibr_imu_chain.yaml"/relative_config_imu: "kalibr_imu_chain_old.yaml"/' \
  -e 's/relative_config_imucam: "kalibr_imucam_chain.yaml"/relative_config_imucam: "kalibr_imucam_chain_old.yaml"/' \
  "$BASELINE/config_snapshot.yaml" > "$CFG/config_old.yaml"

cp "$BASELINE/kalibr_imu_chain.yaml" "$CFG/kalibr_imu_chain_old.yaml"
cp "$BASELINE/kalibr_imucam_chain.yaml" "$CFG/kalibr_imucam_chain_old.yaml"
awk '
  BEGIN { print "%YAML:1.0"; print ""; in_matrix=0 }
  /^  T_cam_imu:/ { in_matrix=1; print; next }
  in_matrix && /^  - / { print "  " $0; next }
  in_matrix && !/^  - / { in_matrix=0 }
  /^  rostopic:/ { print "  rostopic: /camera/color/image_raw"; next }
  { print }
' \
  "$NEW_CALIB/imucam/cam_imu_calib_kalibr_640x480_30hz_imu200hz-camchain-imucam.yaml" \
  > "$CFG/kalibr_imucam_chain.yaml"

awk '
  /^  accelerometer_noise_density:/ { print "  accelerometer_noise_density: 0.009143029881205223"; next }
  /^  accelerometer_random_walk:/   { print "  accelerometer_random_walk: 0.000477273660359181"; next }
  /^  gyroscope_noise_density:/     { print "  gyroscope_noise_density: 0.001284971487348658"; next }
  /^  gyroscope_random_walk:/       { print "  gyroscope_random_walk: 1.9398767374345275e-05"; next }
  { print }
' "$BASELINE/kalibr_imu_chain.yaml" > "$CFG/kalibr_imu_chain.yaml"

{
  echo "=== estimator config ==="
  diff -u "$CFG/config_old.yaml" "$CFG/config_new.yaml" || true
  echo
  echo "=== camera / camera-IMU calibration ==="
  diff -u "$CFG/kalibr_imucam_chain_old.yaml" "$CFG/kalibr_imucam_chain.yaml" || true
  echo
  echo "=== IMU calibration ==="
  diff -u "$CFG/kalibr_imu_chain_old.yaml" "$CFG/kalibr_imu_chain.yaml" || true
} > "$CFG/config_diff.txt"

cmd=(
  "$BINARY"
  --config "$CFG/config_new.yaml"
  --dataset "$DATASET"
  --gps "$GPS"
  --gps-time-offset 0
  --start-time 924.4
  --init-from-fc "$FC_INIT"
  --vio-yaw-update-mode global_yaw_oc_projection
  --vio-global-yaw-oc-alpha 1.0
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
  --t0 924.4 \
  --until 2816 \
  --dir-a "$OUT" \
  --dir-b "$OUT" \
  --gps "$GPS" \
  --imu "$DATASET/imu0/data.csv" \
  --out "$OUT/forensics_valid" \
  --yaw-align-mode start_yaw

python3 "$REPO/analysis/yaw_leak_analysis.py" \
  --run-dir "$OUT" \
  --gps "$GPS" \
  --t0 924.4 \
  --t1 2816 \
  --out "$OUT/leak_analysis"

echo "NEWCALIB_20260612_fly4 complete: $OUT"
