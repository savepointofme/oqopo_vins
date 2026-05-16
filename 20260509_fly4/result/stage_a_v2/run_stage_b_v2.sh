#!/bin/bash
set -e
cd /mnt/d/vscode_dir/open_vins
RESULT=20260509_fly4/result/stage_a_v2
BIN=build_ov_msckf/run_serial_msckf_ros_free
CFG=config/user_drone_mono_jc82/estimator_config.yaml
DATA=20260509_fly4/mav0
GPS=20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv

COMMON="--config $CFG --dataset $DATA --no-display --start-time 239 --gps-time-offset 0.0"
STAGE_A="--gps $GPS --gps-alt-update --gps-alt-ground-plane --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01"

echo "=== R4a: Stage A + Stage B (pre-MSCKF), sigma_px=3.0 K=5 ==="
$BIN $COMMON $STAGE_A --gplane-feat \
  --output $RESULT/R4a_pre_msckf_default.txt > $RESULT/R4a_pre_msckf_default.log 2>&1
echo "[R4a done]"

echo "=== R4b: Stage A + Stage B (pre-MSCKF), sigma_px=5.0 ==="
$BIN $COMMON $STAGE_A --gplane-feat --gplane-feat-sigma-px 5.0 \
  --output $RESULT/R4b_pre_msckf_loose.txt > $RESULT/R4b_pre_msckf_loose.log 2>&1
echo "[R4b done]"

echo "=== R4c: Stage A + Stage B (pre-MSCKF), K=15 max features ==="
$BIN $COMMON $STAGE_A --gplane-feat --gplane-feat-max 15 \
  --output $RESULT/R4c_pre_msckf_K15.txt > $RESULT/R4c_pre_msckf_K15.log 2>&1
echo "[R4c done]"

echo "=== ALL DONE ==="
