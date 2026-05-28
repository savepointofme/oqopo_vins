#!/bin/bash
# Stage 3: Full run A/B/C until t=1600, no-overwrite.
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

BIN=./build_ov_msckf/run_serial_msckf_ros_free
DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=./config/d455_fly2/estimator_config_cond_1e5.yaml
ROOT=$BASE/phase6_oc_experiments_offset438p0_start618_until1600
TARGET_T=1600

mkdir -p "$ROOT"

reached_target() {
  python3 -c "import sys; t=float(sys.argv[1] or 0); raise SystemExit(0 if t>=$TARGET_T else 1)" "$1"
}

run_until() {
  local name="$1"; shift
  local out="$ROOT/$name"
  if [ -d "$out" ]; then
    echo "[SKIP] $name — directory already exists (no overwrite)"
    return 0
  fi
  mkdir -p "$out"
  echo "=== START: $name ==="

  $BIN \
    --config "$CONFIG" \
    --dataset "$DATASET" \
    --gps "$GPS" \
    --gps-time-offset 0 \
    --start-time 618 \
    --init-from-fc "$FC" \
    --init-bg-sigma 0.003 \
    "$@" \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --no-display \
    --viz-fast \
    --dash-every 5 \
    --diag-csv "$out/diag.csv" \
    --diag-events "$out/events.txt" \
    --vio-yaw-diag "$out/yaw_update_diag.csv" \
    --visual-obs-diag "$out/visual_observability_diag.csv" \
    --output "$out/traj.txt" \
    > "$out/log.txt" 2>&1 &

  local pid=$!
  local deadline=$((SECONDS + 1800))
  while kill -0 "$pid" 2>/dev/null; do
    local last=0
    [ -s "$out/traj.txt" ] && last=$(tail -n 1 "$out/traj.txt" | awk '{print $1}')
    if reached_target "$last"; then
      echo "[run_until] $name reached t=$last — SIGINT" | tee -a "$out/log.txt"
      kill -INT "$pid" 2>/dev/null || true; break
    fi
    [ "$SECONDS" -gt "$deadline" ] && {
      echo "[run_until] $name TIMEOUT at t=$last — SIGINT" | tee -a "$out/log.txt"
      kill -INT "$pid" 2>/dev/null || true; break
    }
    sleep 5
  done
  wait "$pid" || true
}

report() {
  local name="$1"
  local out="$ROOT/$name"
  echo ""
  echo "--- $name ---"
  local last_t=0 lines=0
  if [ -s "$out/traj.txt" ]; then
    last_t=$(tail -n 1 "$out/traj.txt" | awk '{print $1}')
    lines=$(wc -l < "$out/traj.txt")
  fi
  echo "  traj.txt:                    $([ -s "$out/traj.txt" ] && echo EXISTS || echo MISSING) ($lines lines, last_t=$last_t)"
  echo "  yaw_update_diag.csv:         $([ -s "$out/yaw_update_diag.csv"    ] && echo EXISTS || echo MISSING)"
  echo "  visual_observability_diag:   $([ -s "$out/visual_observability_diag.csv" ] && echo EXISTS || echo MISSING)"
  if [ -s "$out/traj.txt" ]; then
    grep -qiE 'nan|inf' "$out/traj.txt" && echo "  NaN/Inf: YES — FAIL" || echo "  NaN/Inf: clean"
  fi
  grep -q "diagonal at.*is -" "$out/log.txt" 2>/dev/null && \
    echo "  Negative cov diagonal: YES — FAIL" || echo "  Negative cov diagonal: none"
  if [ -s "$out/yaw_update_diag.csv" ]; then
    awk -F, 'NR>1 && $2=="MSCKF" {s+=$17; n++} END {printf "  MSCKF accepted features: mean=%.1f over %d updates\n", s/n, n}' \
      "$out/yaw_update_diag.csv"
  fi
}

# Run A — current global_oc baseline
run_until global_oc_current_late_baseline_offset438p0_until1600 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0

# Run B — 1D FEJ yaw OC before chi2 (no-op: H·n=0 by FEJ)
run_until global_yaw_oc_fej_prechi2_offset438p0_until1600 \
  --vio-yaw-update-mode global_yaw_oc_fej_prechi2

# Run C — 4D FEJ OC before chi2 (no-op: H·n=0 by FEJ)
run_until visual_4d_oc_fej_prechi2_offset438p0_until1600 \
  --vio-yaw-update-mode visual_4d_oc_fej_prechi2

echo ""
echo "========== STAGE 3 REPORT =========="
report global_oc_current_late_baseline_offset438p0_until1600
report global_yaw_oc_fej_prechi2_offset438p0_until1600
report visual_4d_oc_fej_prechi2_offset438p0_until1600
echo ""
echo "Results: $ROOT"
