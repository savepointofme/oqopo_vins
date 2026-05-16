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

echo "=== R3a: Stage A best (R2f) + Stage B default (sigma_px=3.0, K=5) ==="
$BIN $COMMON $STAGE_A --gplane-feat \
  --output $RESULT/R3a_stageA_plus_B_default.txt > $RESULT/R3a_stageA_plus_B_default.log 2>&1
echo "[R3a done]"

echo "=== R3b: Stage A best + Stage B loose (sigma_px=5.0) ==="
$BIN $COMMON $STAGE_A --gplane-feat --gplane-feat-sigma-px 5.0 \
  --output $RESULT/R3b_stageA_plus_B_loose.txt > $RESULT/R3b_stageA_plus_B_loose.log 2>&1
echo "[R3b done]"

echo "=== ALL DONE ==="
