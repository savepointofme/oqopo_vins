#!/bin/bash
set -e
cd /mnt/d/vscode_dir/open_vins
RESULT=20260509_fly4/result/stage_a_v2
BIN=build_ov_msckf/run_serial_msckf_ros_free
CFG=config/user_drone_mono_jc82/estimator_config.yaml
DATA=20260509_fly4/mav0
GPS=20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv

COMMON="--config $CFG --dataset $DATA --no-display --start-time 239 --gps-time-offset 0.0"

echo "=== R0: no-GPS baseline ===" | tee $RESULT/R0_nogps.log
/usr/bin/time -v $BIN $COMMON --output $RESULT/R0_nogps.txt >> $RESULT/R0_nogps.log 2>&1
echo "[R0 done]" >> $RESULT/R0_nogps.log

echo "=== R1: ground-plane sigma=0.3 (patched, full-state, --gps-alt-min-pzz 0.01) ==="
/usr/bin/time -v $BIN $COMMON \
  --gps $GPS --gps-alt-update --gps-alt-ground-plane \
  --gps-alt-sigma 0.3 --gps-alt-min-pzz 0.01 \
  --output $RESULT/R1_gplane_sigma03.txt > $RESULT/R1_gplane_sigma03.log 2>&1
echo "[R1 done]" >> $RESULT/R1_gplane_sigma03.log

echo "=== R2: ground-plane sigma=2.0 (patched, full-state, --gps-alt-min-pzz 0.01) ==="
/usr/bin/time -v $BIN $COMMON \
  --gps $GPS --gps-alt-update --gps-alt-ground-plane \
  --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --output $RESULT/R2_gplane_sigma20.txt > $RESULT/R2_gplane_sigma20.log 2>&1
echo "[R2 done]" >> $RESULT/R2_gplane_sigma20.log

echo "=== ALL DONE ==="
