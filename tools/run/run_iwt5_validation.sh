#!/bin/bash
# iwt=5.0 candidate validation: B1 (GPS-height) and v1_E1 for all 4 flights.
# Uses estimator_config_iwt5_candidate.yaml (init_window_time: 5.0 only change).
#
# GPS references (post-cleanup):
#   fly1: gps_tum_time_alignment_offset_m4p63
#   fly2: gps_tum_time_alignment_vertical   (not corrected/plain — audit finding)
#   fly3: gps_tum_time_alignment
#   fly4: gps_tum_time_alignment
#
# Run from repo root via WSL:
#   bash run_iwt5_validation.sh [b1|v1|both]
# Default: both (B1 first, then v1_E1).

set -e
cd /mnt/d/vscode_dir/open_vins

MODE="${1:-both}"
BIN=build_ov_msckf/run_serial_msckf_ros_free
CFG=config/user_drone_mono_jc82/estimator_config_iwt5_candidate.yaml

declare -A GPS
GPS[1]="20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv"
GPS[2]="20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv"
GPS[3]="20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv"
GPS[4]="20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv"

declare -A START
START[1]=200; START[2]=160; START[3]=180; START[4]=239

# -------------------------------------------------------------------
run_b1() {
  local FLY="$1"
  local OUT="20260509_fly${FLY}/result/init_window_time_sweep"
  mkdir -p "$OUT"
  echo "[B1-iwt5 fly${FLY}] start (start=${START[$FLY]})"
  "$BIN" \
    --config "$CFG" \
    --dataset "20260509_fly${FLY}/mav0" \
    --no-display \
    --start-time "${START[$FLY]}" \
    --gps-time-offset 0.0 \
    --gps "${GPS[$FLY]}" \
    --gps-alt-update --gps-alt-ground-plane \
    --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --output "$OUT/B1_iwt5.0.txt" \
    > "$OUT/B1_iwt5.0.log" 2>&1
  echo "[B1-iwt5 fly${FLY}] done — $(wc -l < $OUT/B1_iwt5.0.txt) lines"
}

run_v1e1() {
  local FLY="$1"
  local OUT="20260509_fly${FLY}/result/init_window_time_sweep"
  mkdir -p "$OUT"
  echo "[v1_E1-iwt5 fly${FLY}] start (start=${START[$FLY]})"
  "$BIN" \
    --config "$CFG" \
    --dataset "20260509_fly${FLY}/mav0" \
    --no-display \
    --start-time "${START[$FLY]}" \
    --gps-time-offset 0.0 \
    --gps "${GPS[$FLY]}" \
    --gps-alt-update --gps-alt-ground-plane \
    --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gplane-feat-v1-update \
    --output "$OUT/v1_E1_iwt5.0.txt" \
    > "$OUT/v1_E1_iwt5.0.log" 2>&1
  echo "[v1_E1-iwt5 fly${FLY}] done — $(wc -l < $OUT/v1_E1_iwt5.0.txt) lines"
}

# -------------------------------------------------------------------
if [[ "$MODE" == "b1" || "$MODE" == "both" ]]; then
  echo "================================================================="
  echo "== Batch 1: B1 (GPS-height, delay=10s) iwt=5.0 — all 4 flights =="
  echo "================================================================="
  run_b1 1 & run_b1 2 & run_b1 3 & run_b1 4 &
  wait
  echo "== B1 batch complete =="
fi

if [[ "$MODE" == "v1" || "$MODE" == "both" ]]; then
  echo "================================================================="
  echo "== Batch 2: v1_E1 (Stage B v1, sigma=50, K=2) iwt=5.0 — all 4 =="
  echo "================================================================="
  run_v1e1 1 & run_v1e1 2 & run_v1e1 3 & run_v1e1 4 &
  wait
  echo "== v1_E1 batch complete =="
fi

echo "================================================================="
echo "All iwt=5.0 validation runs complete."
echo "Output: 20260509_flyN/result/init_window_time_sweep/{B1,v1_E1}_iwt5.0.{txt,log}"
