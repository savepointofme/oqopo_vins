#!/bin/bash
set -e
cd /mnt/d/vscode_dir/open_vins
RESULT=20260509_fly4/result/stage_a_v2
BIN=build_ov_msckf/run_serial_msckf_ros_free
CFG=config/user_drone_mono_jc82/estimator_config.yaml
DATA=20260509_fly4/mav0
GPS=20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv

COMMON="--config $CFG --dataset $DATA --no-display --start-time 239 --gps-time-offset 0.0"
GPS_COMMON="--gps $GPS --gps-alt-update --gps-alt-ground-plane --gps-alt-min-pzz 0.01"

echo "=== R1z: sigma=0.3 zonly (matches old baseline) ==="
$BIN $COMMON $GPS_COMMON --gps-alt-sigma 0.3 --gps-alt-zonly \
  --output $RESULT/R1z_gplane_sigma03_zonly.txt > $RESULT/R1z_gplane_sigma03_zonly.log 2>&1
echo "[R1z done]"

echo "=== R2z: sigma=2.0 zonly ==="
$BIN $COMMON $GPS_COMMON --gps-alt-sigma 2.0 --gps-alt-zonly \
  --output $RESULT/R2z_gplane_sigma20_zonly.txt > $RESULT/R2z_gplane_sigma20_zonly.log 2>&1
echo "[R2z done]"

echo "=== R1f: sigma=0.3 full-state (NEW behavior under patch) ==="
$BIN $COMMON $GPS_COMMON --gps-alt-sigma 0.3 \
  --output $RESULT/R1f_gplane_sigma03_full.txt > $RESULT/R1f_gplane_sigma03_full.log 2>&1
echo "[R1f done]"

echo "=== R2f: sigma=2.0 full-state ==="
$BIN $COMMON $GPS_COMMON --gps-alt-sigma 2.0 \
  --output $RESULT/R2f_gplane_sigma20_full.txt > $RESULT/R2f_gplane_sigma20_full.log 2>&1
echo "[R2f done]"

echo "=== ALL DONE ==="
