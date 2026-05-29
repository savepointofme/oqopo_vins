#!/bin/bash
# fly4 FEJ-gauge Schmidt evaluation: A vs B_current vs B_FEJ
# RUN_START=904.4 (matches FC init), EVAL_T0=924.4, until=2816
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549
BASE=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528
GPS=$BASE/gps_from_mems_offsetm202p2_cam_time.csv
FC=$BASE/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free
IMU=$DATASET/imu0/data.csv
RUN_START=924.4
EVAL_T0=924.4
UNTIL=2816

# Reuse existing A baseline (already run from 904.4 based on dir name)
DIR_A=$BASE/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1
# Reuse existing B_current (run from 924.4 — note start mismatch vs A; diagnostic only)
DIR_B=$BASE/fly4_schmidt_B_offsetm202p2_start924p4_until2816
# New B_FEJ run (from 904.4 to match A)
DIR_BFEJ=$BASE/fly4_B_fej_gauge_start924p4_until2816

echo "[fly4-fej] start $(date)  HEAD=$(git rev-parse HEAD)"

traj_complete() {
  local f="$1/traj.txt"
  [ -f "$f" ] || return 1
  local last; last=$(tail -1 "$f" | awk '{print $1}')
  python3 -c "import sys; sys.exit(0 if float('${last:-0}') >= $(echo "$UNTIL - 10" | bc) else 1)" 2>/dev/null
}

run_vio() {
  local tag="$1"; local out="$2"; shift 2
  mkdir -p "$out"
  echo "[START $tag -> $out]"
  "$BIN" \
    --config "$CONFIG" \
    --dataset "$DATASET" \
    --gps "$GPS" \
    --gps-time-offset 0 \
    --start-time "$RUN_START" \
    --until-time "$UNTIL" \
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
    --viz-fast \
    --dash-every 5 \
    --diag-csv "$out/diag.csv" \
    --vio-yaw-diag "$out/yaw_update_diag.csv" \
    --output "$out/traj.txt" \
    > "$out/log.txt" 2>&1
  local lines=0; [ -f "$out/traj.txt" ] && lines=$(wc -l < "$out/traj.txt")
  echo "[DONE $tag  lines=$lines  t=$(tail -1 "$out/traj.txt" 2>/dev/null | awk '{print $1}')]"
}

# B_FEJ (run from 904.4 to match A baseline)
if traj_complete "$DIR_BFEJ"; then
  echo "[skip B_FEJ] complete"
else
  rm -f "$DIR_BFEJ/traj.txt" "$DIR_BFEJ/traj.txt.bias" "$DIR_BFEJ/schmidt_yaw_update_diag.csv"
  run_vio B_FEJ "$DIR_BFEJ" \
    --vio-yaw-update-mode visual_yaw_schmidt_fej_gauge \
    --schmidt-yaw-diag "$DIR_BFEJ/schmidt_yaw_update_diag.csv"
fi

# Also run B_current from 904.4 for fair comparison with B_FEJ
DIR_BCUR=$BASE/fly4_B_current_gauge_start924p4_until2816
if traj_complete "$DIR_BCUR"; then
  echo "[skip B_current/904.4] complete"
else
  rm -f "$DIR_BCUR/traj.txt" "$DIR_BCUR/traj.txt.bias" "$DIR_BCUR/schmidt_yaw_update_diag.csv"
  run_vio B_current_904 "$DIR_BCUR" \
    --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
    --schmidt-yaw-diag "$DIR_BCUR/schmidt_yaw_update_diag.csv"
fi

# ── Eval ──────────────────────────────────────────────────────────────────────
EVAL=$BASE/fly4_eval_fej_vs_current

eval_pair() {
  local label="$1"; local da="$2"; local db="$3"; local out="$4"
  if ! [ -f "$da/traj.txt" ] || ! [ -f "$db/traj.txt" ]; then
    echo "[skip eval $label] one or both traj missing"
    return 0
  fi
  echo "[eval] $label"
  rm -rf "$out"; mkdir -p "$out"
  python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
    --t0 "$EVAL_T0" \
    --until "$UNTIL" \
    --dir-a "$da" \
    --dir-b "$db" \
    --gps "$GPS" \
    --imu "$IMU" \
    --out "$out" \
    2>&1 | tee "$out/eval_stdout.txt"
}

# A vs B_FEJ
eval_pair A_vs_BFEJ "$DIR_A" "$DIR_BFEJ" "$EVAL/A_vs_B_fej"

# A vs B_current/904 (fair comparison — same start time as A and B_FEJ)
eval_pair A_vs_Bcurrent904 "$DIR_A" "$DIR_BCUR" "$EVAL/A_vs_B_current_904"

# B_current/904 vs B_FEJ (direct current vs FEJ comparison)
eval_pair Bcurrent904_vs_BFEJ "$DIR_BCUR" "$DIR_BFEJ" "$EVAL/B_current_vs_B_fej"

# Summary table
python3 - <<'PYEOF'
import os

BASE  = '/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528'
UNTIL = 2816.0

DIRS = {
  'A (904.4)':         f'{BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1',
  'B_current_orig':    f'{BASE}/fly4_schmidt_B_offsetm202p2_start924p4_until2816',
  'B_current (924.4)': f'{BASE}/fly4_B_current_gauge_start924p4_until2816',
  'B_FEJ (924.4)':     f'{BASE}/fly4_B_fej_gauge_start924p4_until2816',
}

print(f"{'Run':22s}  {'lines':>7s}  {'last_t':>8s}  {'complete':>10s}")
print('-' * 55)
for name, d in DIRS.items():
  f = f'{d}/traj.txt'
  if not os.path.exists(f):
    print(f'{name:22s}  {"MISSING":>7s}')
    continue
  with open(f) as fp:
    lines = [l for l in fp if l.strip() and not l.startswith('#')]
  last_t = float(lines[-1].split()[0]) if lines else 0
  complete = last_t >= UNTIL - 10
  print(f'{name:22s}  {len(lines):>7d}  {last_t:>8.1f}  {"YES" if complete else "NO":>10s}')
PYEOF

echo ""
echo "[fly4-fej] ALL DONE $(date)"
echo "  eval dir: $EVAL"
