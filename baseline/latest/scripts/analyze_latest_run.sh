#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash baseline/latest/scripts/analyze_latest_run.sh --fly fly3 --run-dir PATH

Options:
  --fly NAME        fly1, fly2, fly3, or fly4
  --run-dir PATH    folder containing traj.txt from run_baseline_fly.sh
  --gps PATH        override GPS CSV path
  --out-dir PATH    analysis output folder, default RUN_DIR/analysis
  --t0 SEC          override analysis start time
  --t1 SEC          override analysis end time
  --method NAME     method label, default latest_baseline
  --notes TEXT      notes passed to the analysis report
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASELINE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$BASELINE_DIR/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

FLY=""
RUN_DIR=""
GPS=""
OUT_DIR=""
T0=""
T1=""
METHOD="latest_baseline"
NOTES=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fly) FLY="${2:?missing --fly value}"; shift 2 ;;
    --run-dir) RUN_DIR="${2:?missing --run-dir value}"; shift 2 ;;
    --gps) GPS="${2:?missing --gps value}"; shift 2 ;;
    --out-dir) OUT_DIR="${2:?missing --out-dir value}"; shift 2 ;;
    --t0) T0="${2:?missing --t0 value}"; shift 2 ;;
    --t1) T1="${2:?missing --t1 value}"; shift 2 ;;
    --method) METHOD="${2:?missing --method value}"; shift 2 ;;
    --notes) NOTES="${2:?missing --notes value}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$FLY" in
  fly1)
    GPS="${GPS:-/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv}"
    T0="${T0:-934.2}"
    T1="${T1:-1644.8}"
    ;;
  fly2)
    GPS="${GPS:-/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv}"
    T0="${T0:-704.2}"
    T1="${T1:-846.8}"
    ;;
  fly3)
    GPS="${GPS:-/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv}"
    T0="${T0:-622.2}"
    T1="${T1:-1406.2}"
    ;;
  fly4)
    GPS="${GPS:-/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv}"
    T0="${T0:-928.6}"
    T1="${T1:-2436.2}"
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

if [[ -z "$RUN_DIR" ]]; then
  echo "--run-dir is required" >&2
  usage >&2
  exit 2
fi

TRAJ="$RUN_DIR/traj.txt"
OUT_DIR="${OUT_DIR:-$RUN_DIR/analysis}"

for path in "$GPS" "$TRAJ"; do
  if [[ ! -e "$path" ]]; then
    echo "Missing required path: $path" >&2
    exit 3
  fi
done

mkdir -p "$OUT_DIR"

CMD=(
  "$PYTHON_BIN" "$REPO_ROOT/analysis/full_flight_error_analysis.py"
  --gps "$GPS"
  --vio-traj "$TRAJ"
  --out-dir "$OUT_DIR"
  --t0 "$T0"
  --t1 "$T1"
  --flight-name "$FLY"
  --method-name "$METHOD"
  --experiment-id "$(basename "$RUN_DIR")"
  --notes "$NOTES"
)

if [[ -e "$RUN_DIR/diag.csv" ]]; then
  CMD+=(--vio-diag "$RUN_DIR/diag.csv")
fi
if [[ -e "$RUN_DIR/yaw_diag.csv" ]]; then
  CMD+=(--vio-yaw-diag "$RUN_DIR/yaw_diag.csv")
fi

echo "[analysis] output: $OUT_DIR"
printf '[analysis] command:'
printf ' %q' "${CMD[@]}"
echo
"${CMD[@]}"
