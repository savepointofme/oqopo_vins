#!/bin/bash
# Phase 6: Visual Observability Policy experiments
# Three mainline candidates:
#   A. global_oc_current_late_baseline   (legacy alpha=1, applied in EKFUpdate)
#   B. global_yaw_oc_fej_prechi2         (1D FEJ yaw OC, applied before chi2)
#   C. visual_4d_oc_fej_prechi2          (4D FEJ OC, applied before chi2)
#
# Fixed conditions: offset=438, start=618, until=1600, FC init, current GPS-alt guard.
# Each experiment writes to a NEW directory. Do NOT overwrite previous results.
# =============================================================================
set -euo pipefail

cd /mnt/d/vscode_dir/open_vins

# ---- Paths (update if your dataset/result root differs) ----
DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
ROOT=$BASE/phase6_oc_experiments_offset438p0_start618_until1600
TARGET_T=1600

BIN=./build_ov_msckf/run_serial_msckf_ros_free

mkdir -p "$ROOT"

# ---- helper: run until TARGET_T, then SIGINT ----
reached_target() {
  python3 - "$1" "$TARGET_T" <<'PY'
import sys
t = float(sys.argv[1] or 0.0)
target = float(sys.argv[2])
raise SystemExit(0 if t >= target else 1)
PY
}

run_until() {
  local name="$1"; shift
  local out="$ROOT/$name"
  if [ -d "$out" ]; then
    echo "[SKIP] $name — directory already exists. Remove it manually to re-run."
    return 0
  fi
  mkdir -p "$out"
  echo "================================================================="
  echo "== EXPERIMENT: $name"
  echo "== out: $out"
  echo "================================================================="

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
    if [ -s "$out/traj.txt" ]; then
      last=$(tail -n 1 "$out/traj.txt" | awk '{print $1}')
    fi
    if reached_target "$last"; then
      echo "[run_until] $name reached t=$last, sending SIGINT" | tee -a "$out/log.txt"
      kill -INT "$pid" 2>/dev/null || true
      break
    fi
    if [ "$SECONDS" -gt "$deadline" ]; then
      echo "[run_until] $name TIMEOUT at t=$last, sending SIGINT" | tee -a "$out/log.txt"
      kill -INT "$pid" 2>/dev/null || true
      break
    fi
    sleep 5
  done
  wait "$pid" || true

  local last_t=0
  if [ -s "$out/traj.txt" ]; then
    last_t=$(tail -n 1 "$out/traj.txt" | awk '{print $1}')
  fi
  echo "[done] $name — last_t=$last_t traj_lines=$(wc -l < "$out/traj.txt" 2>/dev/null || echo 0)"
  if grep -q "StateHelper::EKFUpdate() - diagonal" "$out/log.txt" 2>/dev/null; then
    echo "  WARNING: negative covariance diagonal detected in $name"
  fi
  if grep -q "nan\|inf" "$out/traj.txt" 2>/dev/null; then
    echo "  WARNING: NaN/Inf detected in trajectory for $name"
  fi
}

# ---- Experiment A: current global_oc baseline (legacy, OC in EKFUpdate) ----
run_until global_oc_current_late_baseline_offset438p0_until1600 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0

# ---- Experiment B: 1D FEJ yaw OC, applied before chi2 gating ----
run_until global_yaw_oc_fej_prechi2_offset438p0_until1600 \
  --vio-yaw-update-mode global_yaw_oc_fej_prechi2

# ---- Experiment C: 4D FEJ OC (yaw + translation xyz), before chi2 gating ----
run_until visual_4d_oc_fej_prechi2_offset438p0_until1600 \
  --vio-yaw-update-mode visual_4d_oc_fej_prechi2

echo "================================================================="
echo "Phase 6 experiments complete. Results in: $ROOT"
echo "================================================================="
echo "Traj summary:"
for d in "$ROOT"/*/; do
  name=$(basename "$d")
  last_t=0
  lines=0
  if [ -s "$d/traj.txt" ]; then
    last_t=$(tail -n 1 "$d/traj.txt" | awk '{print $1}')
    lines=$(wc -l < "$d/traj.txt")
  fi
  echo "  $name: last_t=$last_t lines=$lines"
done
