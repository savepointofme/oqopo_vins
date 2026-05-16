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

echo "=== R5a: Stage B very loose sigma_px=20.0 K=3 ==="
$BIN $COMMON $STAGE_A --gplane-feat --gplane-feat-sigma-px 20.0 --gplane-feat-max 3 \
  --output $RESULT/R5a_loose20_K3.txt > $RESULT/R5a_loose20_K3.log 2>&1
echo "[R5a done]"

echo "=== R5b: Stage B extreme loose sigma_px=50.0 K=2 ==="
$BIN $COMMON $STAGE_A --gplane-feat --gplane-feat-sigma-px 50.0 --gplane-feat-max 2 \
  --output $RESULT/R5b_extreme_K2.txt > $RESULT/R5b_extreme_K2.log 2>&1
echo "[R5b done]"

echo "=== ALL DONE ==="
