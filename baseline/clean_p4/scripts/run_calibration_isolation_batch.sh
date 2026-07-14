#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT_ROOT="${OUT_ROOT:-/mnt/c/Users/baloney/Desktop/实验目录/P4_clean_baseline_20260712}"
RUNNER="$REPO_ROOT/build_ov_msckf/run_serial_msckf_ros_free"
FLY=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fly) FLY="${2:?missing --fly value}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:?missing --out-root value}"; shift 2 ;;
    *) echo "Usage: $0 --fly fly1|fly3 [--out-root PATH]" >&2; exit 2 ;;
  esac
done

case "$FLY" in
  fly1)
    DATASET="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"
    GPS="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fc_rebuild_20260614/gps_refined_finealign.csv"
    FC_INIT="$OUT_ROOT/inputs/fly1_fc_init_series_global.csv"
    START_TIME=930.0
    UNTIL_TIME=1010.0
    ;;
  fly3)
    DATASET="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946"
    GPS="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv"
    FC_INIT="$OUT_ROOT/inputs/fly3_fc_init_series_global.csv"
    START_TIME=618.0
    UNTIL_TIME=770.0
    ;;
  *) echo "--fly must be fly1 or fly3" >&2; exit 2 ;;
esac

STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="$OUT_ROOT/calibration_isolation/${STAMP}_${FLY}_C0_C2_short"
CONFIG_ROOT="$BATCH_DIR/generated_configs"
mkdir -p "$BATCH_DIR"
python3 "$REPO_ROOT/baseline/clean_p4/scripts/prepare_calibration_isolation.py" \
  --repo "$REPO_ROOT" --out-dir "$CONFIG_ROOT" > "$BATCH_DIR/config_generation.json"

conditions=(
  C0_june12_ground_kd_locked
  C1_historical_fly3_online_final_kd_locked
  C2_june12_initial_online_kd_only
)

run_one() {
  local condition="$1"
  local run_dir="$BATCH_DIR/${FLY}_${condition}"
  local config="$CONFIG_ROOT/$condition/estimator_config.yaml"
  mkdir -p "$run_dir"
  local cmd=(
    "$RUNNER" --config "$config" --dataset "$DATASET"
    --gps "$GPS" --gps-time-offset 0
    --start-time "$START_TIME" --until-time "$UNTIL_TIME"
    --initialization-mode fc_full_state
    --init-from-fc "$FC_INIT" --init-from-fc-position-frame global_gnav
    --fc-init-level I2 --fc-init-fallback fail_closed
    --init-from-fc-max-bracket-gap 0.35
    --init-window-s 5 --init-window-min-samples 12
    --init-window-max-source-gap 0.35
    --init-window-max-attitude-p95-deg 3
    --init-att-sigma-deg 3 --init-vel-sigma 5 --init-pos-sigma 5
    --init-bg-sigma 0.003 --init-ba-sigma 1
    --yaw-mode baseline
    --gps-alt-update --height-mode guarded --gps-alt-sigma 2
    --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10
    --gps-alt-max-res 80 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5
    --camera-frame-stride 12 --no-display
    --diag-csv "$run_dir/diag.csv" --diag-events "$run_dir/events.txt"
    --output "$run_dir/traj.txt" --output-raw "$run_dir/traj_raw.txt"
    --output-nav "$run_dir/traj_nav.txt"
    --nav-frame-metadata-json "$run_dir/nav_frame_metadata.json"
    --canonical-init-state-json "$run_dir/canonical_init_state.json"
  )
  {
    echo "condition=$condition"
    echo "single_variable=camera_intrinsics_distortion"
    echo "initialization=P4_I2_same"
    echo "T_C_I=june12_same"
    echo "camera_imu_time_offset=june12_same"
    echo "fixed_fc_board_correction=none"
    printf 'command='
    printf '%q ' "${cmd[@]}"
    echo
  } > "$run_dir/run_metadata.txt"
  set +e
  "${cmd[@]}" > "$run_dir/log.txt" 2>&1
  local rc=$?
  set -e
  echo "$rc" > "$run_dir/runner_exit_code.txt"
  [[ "$rc" -eq 0 ]] || return "$rc"
  test -s "$run_dir/traj_nav.txt"
  python3 -m json.tool "$run_dir/canonical_init_state.json" > /dev/null
  python3 "$REPO_ROOT/analysis/validate_frame_contract.py" \
    --raw "$run_dir/traj_raw.txt" \
    --raw-bias "$run_dir/traj.txt.bias" \
    --nav "$run_dir/traj_nav.txt" \
    --metadata "$run_dir/nav_frame_metadata.json" \
    --out "$run_dir/frame_contract_validation.json" \
    > "$run_dir/frame_contract_validation.log" 2>&1
}

pids=()
for condition in "${conditions[@]}"; do
  run_one "$condition" &
  pids+=("$!")
done
batch_rc=0
for pid in "${pids[@]}"; do
  wait "$pid" || batch_rc=1
done
{
  echo "batch_dir=$BATCH_DIR"
  echo "flight=$FLY"
  echo "parallel_job_count=${#pids[@]}"
  echo "conditions=${conditions[*]}"
  echo "batch_exit_code=$batch_rc"
} > "$BATCH_DIR/BATCH_RECEIPT.txt"
echo "$BATCH_DIR"
exit "$batch_rc"
