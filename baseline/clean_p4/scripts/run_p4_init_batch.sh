#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: run_p4_init_batch.sh --fly fly1|fly3 [--scope short|full] [--out-root PATH]

Short scope runs I0/I1/I2; full scope runs I0/I2 concurrently after I2 was
selected as Ibest. The wrapper blocks on the whole batch and never polls the
jobs. The clean profile is fixed: June-12 ground K/D/T_C_I/time, stride12,
locked online camera calibration, GPS XY/course reference-only, guarded GPS-Z,
no FC-board residual correction, and no post alignment.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
P4_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$P4_DIR/../.." && pwd)"

FLY=""
SCOPE="short"
OUT_ROOT="${OUT_ROOT:-/mnt/c/Users/baloney/Desktop/实验目录/P4_clean_baseline_20260712}"
RUNNER="$REPO_ROOT/build_ov_msckf/run_serial_msckf_ros_free"
CONFIG="$P4_DIR/config/estimator_config.yaml"
FRAME_VALIDATOR="$REPO_ROOT/analysis/validate_frame_contract.py"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fly) FLY="${2:?missing --fly value}"; shift 2 ;;
    --scope) SCOPE="${2:?missing --scope value}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:?missing --out-root value}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "$SCOPE" != "short" && "$SCOPE" != "full" ]]; then
  echo "--scope must be short or full" >&2
  exit 2
fi

case "$FLY" in
  fly1)
    DATASET="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"
    GPS="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fc_rebuild_20260614/gps_refined_finealign.csv"
    FC_INIT="$OUT_ROOT/inputs/fly1_fc_init_series_global.csv"
    START_TIME="930.0"
    SHORT_UNTIL="1010.0"
    ;;
  fly3)
    DATASET="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946"
    GPS="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv"
    FC_INIT="$OUT_ROOT/inputs/fly3_fc_init_series_global.csv"
    START_TIME="618.0"
    SHORT_UNTIL="770.0"
    ;;
  *) echo "--fly must be fly1 or fly3" >&2; exit 2 ;;
esac

for required in "$RUNNER" "$CONFIG" "$DATASET" "$GPS" "$FC_INIT"; do
  [[ -e "$required" ]] || { echo "Missing required path: $required" >&2; exit 3; }
done

STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="$OUT_ROOT/runs/${STAMP}_${FLY}_I0_I2_${SCOPE}"
mkdir -p "$BATCH_DIR"

run_one() {
  local level="$1"
  local run_dir="$BATCH_DIR/${FLY}_${level}_${SCOPE}"
  mkdir -p "$run_dir"
  local cmd=(
    "$RUNNER"
    --config "$CONFIG"
    --dataset "$DATASET"
    --gps "$GPS"
    --gps-time-offset 0
    --start-time "$START_TIME"
    --initialization-mode fc_full_state
    --init-from-fc "$FC_INIT"
    --init-from-fc-position-frame global_gnav
    --fc-init-level "$level"
    --fc-init-fallback fail_closed
    --init-from-fc-max-dt 0.25
    --init-from-fc-max-bracket-gap 0.35
    --init-window-s 5.0
    --init-window-min-samples 12
    --init-window-max-source-gap 0.35
    --init-window-max-attitude-p95-deg 3.0
    --init-window-huber-delta-deg 1.5
    --init-att-sigma-deg 3.0
    --init-vel-sigma 5.0
    --init-pos-sigma 5.0
    --init-bg-sigma 0.003
    --init-ba-sigma 1.0
    --yaw-mode baseline
    --gps-alt-update
    --height-mode guarded
    --gps-alt-sigma 2.0
    --gps-alt-min-pzz 0.01
    --gps-alt-min-t-after-init 10
    --gps-alt-max-res 80
    --gps-alt-guard-dxy 0.5
    --gps-alt-guard-kxy 5.0
    --camera-frame-stride 12
    --no-display
    --diag-csv "$run_dir/diag.csv"
    --diag-events "$run_dir/events.txt"
    --camera-stride-audit "$run_dir/stride_audit.csv"
    --output "$run_dir/traj.txt"
    --output-raw "$run_dir/traj_raw.txt"
    --output-nav "$run_dir/traj_nav.txt"
    --nav-frame-metadata-json "$run_dir/nav_frame_metadata.json"
    --canonical-init-state-json "$run_dir/canonical_init_state.json"
  )
  if [[ "$SCOPE" == "short" ]]; then
    cmd+=(--until-time "$SHORT_UNTIL")
  fi
  {
    echo "flight=$FLY"
    echo "scope=$SCOPE"
    echo "initialization_mode=fc_full_state"
    echo "fc_init_level=$level"
    echo "evaluation_primary=absolute_navigation_no_post_alignment"
    echo "evaluation_secondary=relative_drift"
    echo "gps_horizontal_role=reference_only"
    echo "gps_course_role=reference_only"
    echo "fc_attitude_role=reference_only_initialization"
    echo "camera_calibration=june12_ground_k_d_tci_time_locked"
    echo "online_intrinsics=false"
    echo "online_distortion=false"
    echo "online_extrinsic=false"
    echo "online_time_offset=false"
    echo "fixed_fc_board_correction=none"
    echo "T_C_I_time_varying=false"
    echo "camera_stride=12"
    printf 'command='
    printf '%q ' "${cmd[@]}"
    echo
  } > "$run_dir/run_metadata.txt"
  sha256sum "$RUNNER" "$CONFIG" "$P4_DIR/config/kalibr_imucam_chain.yaml" \
    "$FC_INIT" "$GPS" > "$run_dir/input_sha256.txt"
  printf '#!/usr/bin/env bash\n' > "$run_dir/command.sh"
  printf '%q ' "${cmd[@]}" >> "$run_dir/command.sh"
  printf '\n' >> "$run_dir/command.sh"
  chmod +x "$run_dir/command.sh"

  set +e
  "${cmd[@]}" > "$run_dir/log.txt" 2>&1
  local rc=$?
  set -e
  echo "$rc" > "$run_dir/runner_exit_code.txt"
  if [[ "$rc" -ne 0 ]]; then
    return "$rc"
  fi
  test -s "$run_dir/traj_nav.txt"
  test -s "$run_dir/canonical_init_state.json"
  python3 -m json.tool "$run_dir/canonical_init_state.json" > /dev/null
  python3 "$FRAME_VALIDATOR" \
    --raw "$run_dir/traj_raw.txt" \
    --raw-bias "$run_dir/traj.txt.bias" \
    --nav "$run_dir/traj_nav.txt" \
    --metadata "$run_dir/nav_frame_metadata.json" \
    --out "$run_dir/frame_contract_validation.json" \
    > "$run_dir/frame_contract_validation.log" 2>&1
}

if [[ "$SCOPE" == "short" ]]; then
  levels=(I0 I1 I2)
else
  levels=(I0 I2)
fi
pids=()
for level in "${levels[@]}"; do
  run_one "$level" &
  pids+=("$!")
done

batch_rc=0
for pid in "${pids[@]}"; do
  if ! wait "$pid"; then
    batch_rc=1
  fi
done

{
  echo "batch_dir=$BATCH_DIR"
  echo "flight=$FLY"
  echo "scope=$SCOPE"
  echo "parallel_job_count=${#pids[@]}"
  echo "levels=${levels[*]}"
  echo "batch_exit_code=$batch_rc"
} > "$BATCH_DIR/BATCH_RECEIPT.txt"

echo "$BATCH_DIR"
exit "$batch_rc"
