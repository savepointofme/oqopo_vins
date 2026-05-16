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

for delay in 10 20 30 45; do
  echo "=== R6_${delay}s: Stage A best with bootstrap-delay = ${delay}s ==="
  $BIN $COMMON $STAGE_A --gps-alt-min-t-after-init $delay \
    --output $RESULT/R6_delay${delay}s.txt > $RESULT/R6_delay${delay}s.log 2>&1
  echo "[R6_${delay}s done]"
done

echo "=== ALL DONE ==="
