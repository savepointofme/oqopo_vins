#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 [--stride 12] [--yaw-mode baseline] [--height-mode classic]

Options:
  --fly NAME          fly1, fly2, fly3, or fly4
  --stride N          camera-frame stride, default 12
  --yaw-mode MODE     baseline, fej, oc, oc-fej, or none; default baseline
  --height-mode MODE  classic, nasa-lear, nasa-lean, standard, or bounded; default classic
  --dataset PATH      override dataset path
  --gps PATH          override GPS CSV path
  --fc-init PATH      override FC init CSV path
  --start-time SEC    override VIO start time
  --out-root PATH     output root, default $OUT_ROOT or Desktop/openvins_latest_baseline_runs_20260625
  --config PATH       override estimator config
  --runner PATH       override run_serial binary
  --headless          add --no-display
  --rich-diag         write extra visual diagnostic CSV files
  --dry-run           validate and print the exact command without running
  --                  pass remaining args directly to run_serial
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASELINE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$BASELINE_DIR/../.." && pwd)"

FLY=""
STRIDE=12
YAW_MODE="baseline"
HEIGHT_MODE="classic"
DATASET=""
GPS=""
FC_INIT=""
START_TIME=""
OUT_ROOT="${OUT_ROOT:-/mnt/c/Users/baloney/Desktop/openvins_latest_baseline_runs_20260625}"
CONFIG="$BASELINE_DIR/config/estimator_config.yaml"
RUNNER="$REPO_ROOT/build_ov_msckf/run_serial_msckf_ros_free"
HEADLESS=0
RICH_DIAG=0
DRY_RUN=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fly) FLY="${2:?missing --fly value}"; shift 2 ;;
    --stride) STRIDE="${2:?missing --stride value}"; shift 2 ;;
    --yaw-mode) YAW_MODE="${2:?missing --yaw-mode value}"; shift 2 ;;
    --height-mode) HEIGHT_MODE="${2:?missing --height-mode value}"; shift 2 ;;
    --dataset) DATASET="${2:?missing --dataset value}"; shift 2 ;;
    --gps) GPS="${2:?missing --gps value}"; shift 2 ;;
    --fc-init) FC_INIT="${2:?missing --fc-init value}"; shift 2 ;;
    --start-time) START_TIME="${2:?missing --start-time value}"; shift 2 ;;
    --out-root) OUT_ROOT="${2:?missing --out-root value}"; shift 2 ;;
    --config) CONFIG="${2:?missing --config value}"; shift 2 ;;
    --runner) RUNNER="${2:?missing --runner value}"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --rich-diag) RICH_DIAG=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_ARGS+=("$@"); break ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$FLY" in
  fly1)
    DATASET="${DATASET:-/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810}"
    GPS="${GPS:-/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv}"
    FC_INIT="${FC_INIT:-/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_init_state_930.csv}"
    START_TIME="${START_TIME:-930.0}"
    ;;
  fly2)
    DATASET="${DATASET:-/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722}"
    GPS="${GPS:-/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv}"
    FC_INIT="${FC_INIT:-/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv}"
    START_TIME="${START_TIME:-700.0}"
    ;;
  fly3)
    DATASET="${DATASET:-/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946}"
    GPS="${GPS:-/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv}"
    FC_INIT="${FC_INIT:-/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv}"
    START_TIME="${START_TIME:-618.0}"
    ;;
  fly4)
    DATASET="${DATASET:-/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549}"
    GPS="${GPS:-/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv}"
    FC_INIT="${FC_INIT:-/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv}"
    START_TIME="${START_TIME:-924.4}"
    ;;
  "")
    echo "--fly is required" >&2
    usage >&2
    exit 2
    ;;
  *)
    echo "Unsupported fly: $FLY" >&2
    exit 2
    ;;
esac

for path in "$RUNNER" "$CONFIG" "$DATASET" "$GPS" "$FC_INIT"; do
  if [[ ! -e "$path" ]]; then
    echo "Missing required path: $path" >&2
    exit 3
  fi
done

if [[ "$HEADLESS" -eq 0 ]]; then
  export DISPLAY="${DISPLAY:-:0}"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$OUT_ROOT/${FLY}_stride${STRIDE}_${STAMP}"

CMD=(
  "$RUNNER"
  --config "$CONFIG"
  --dataset "$DATASET"
  --gps "$GPS"
  --gps-time-offset 0
  --start-time "$START_TIME"
  --init-from-fc "$FC_INIT"
  --init-att-sigma-deg 3
  --init-pos-sigma 0.05
  --init-bg-sigma 0.003
  --yaw-mode "$YAW_MODE"
  --gps-alt-update
  --height-mode "$HEIGHT_MODE"
  --gps-alt-sigma 2.0
  --gps-alt-min-pzz 0.01
  --gps-alt-min-t-after-init 10
  --gps-alt-max-res 80
  --gps-alt-guard-dxy 0.5
  --gps-alt-guard-kxy 5.0
  --camera-frame-stride "$STRIDE"
  --viz-fast
  --dash-every 5
  --diag-csv "$OUT/diag.csv"
  --diag-events "$OUT/events.txt"
  --vio-yaw-diag "$OUT/yaw_diag.csv"
  --output "$OUT/traj.txt"
)

if [[ "$HEADLESS" -eq 1 ]]; then
  CMD+=(--no-display)
fi

if [[ "$RICH_DIAG" -eq 1 ]]; then
  CMD+=(
    --visual-obs-diag "$OUT/visual_obs_diag.csv"
    --visual-flow-curl-diag "$OUT/visual_flow_curl_diag.csv"
    --visual-residual-frame-summary "$OUT/visual_residual_frame_summary.csv"
    --visual-feature-residual-diag "$OUT/visual_feature_residual_diag.csv"
  )
fi

CMD+=("${EXTRA_ARGS[@]}")

echo "[baseline] output: $OUT"
echo "[baseline] command:"
printf '  %q' "${CMD[@]}"
echo

if [[ "$DRY_RUN" -eq 1 ]]; then
  echo "[baseline] dry-run complete"
  exit 0
fi

mkdir -p "$OUT"
cp -R "$BASELINE_DIR/config" "$OUT/config_snapshot"

{
  echo "#!/usr/bin/env bash"
  printf '%q ' "${CMD[@]}"
  echo
} > "$OUT/command.sh"
chmod +x "$OUT/command.sh"

{
  echo "fly=$FLY"
  echo "stride=$STRIDE"
  echo "yaw_mode=$YAW_MODE"
  echo "height_mode=$HEIGHT_MODE"
  echo "dataset=$DATASET"
  echo "gps=$GPS"
  echo "fc_init=$FC_INIT"
  echo "start_time=$START_TIME"
  echo "config=$CONFIG"
  echo "runner=$RUNNER"
  echo "out=$OUT"
} > "$OUT/run_metadata.txt"

"${CMD[@]}" 2>&1 | tee "$OUT/log.txt"
echo "[baseline] done: $OUT"
tail -n 1 "$OUT/traj.txt" 2>/dev/null || true
