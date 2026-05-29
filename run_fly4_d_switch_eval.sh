#!/bin/bash
# fly4 staged B→A switch experiments: D1–D4
# D1: B[924.4→1289.3) then A; D2: B[924.4→1339.0); D3: B[924.4→1389.0); D4: B[924.4→1440.0)
# Compares each Dx vs pure A baseline on lap/full-flight metrics.
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549
BASE=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528
GPS=$BASE/gps_from_mems_offsetm202p2_cam_time.csv
FC=$BASE/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free
IMU=$DATASET/imu0/data.csv
START=924.4
UNTIL_VAL=2816

# Pure-A baseline (reuse if complete)
DIR_A=$BASE/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1
# Pure-B (reuse if complete)
DIR_B=$BASE/fly4_schmidt_B_offsetm202p2_start924p4_until2816

# D variant output dirs
DIR_D1=$BASE/fly4_D1_switch1289p3_start924p4_until2816
DIR_D2=$BASE/fly4_D2_switch1339p0_start924p4_until2816
DIR_D3=$BASE/fly4_D3_switch1389p0_start924p4_until2816
DIR_D4=$BASE/fly4_D4_switch1440p0_start924p4_until2816

echo "[fly4-D-switch] start $(date)  HEAD=$(git rev-parse HEAD)"

traj_complete() {
  local f="$1/traj.txt"
  [ -f "$f" ] || return 1
  local last; last=$(tail -1 "$f" | awk '{print $1}')
  python3 -c "import sys; sys.exit(0 if float('${last:-0}') >= $(echo "$UNTIL_VAL - 10" | bc) else 1)" 2>/dev/null
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
    --start-time "$START" \
    --until-time "$UNTIL_VAL" \
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

PIDS=()

# Check pure-A baseline exists; warn if missing but don't re-run (it's a long run)
if ! traj_complete "$DIR_A"; then
  echo "[WARN] Pure-A baseline missing at $DIR_A — eval will be skipped for A comparisons"
  echo "       Re-run pure A with: --vio-yaw-update-mode global_yaw_oc_projection --vio-global-yaw-oc-alpha 1.0"
fi

# D1: B until 1289.3, then switch to A (alpha=1.0)
if traj_complete "$DIR_D1"; then
  echo "[skip D1] complete"
else
  rm -f "$DIR_D1/traj.txt" "$DIR_D1/traj.txt.bias" "$DIR_D1/schmidt_yaw_update_diag.csv"
  run_vio D1 "$DIR_D1" \
    --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
    --schmidt-yaw-diag "$DIR_D1/schmidt_yaw_update_diag.csv" \
    --vio-yaw-switch-mode global_yaw_oc_projection \
    --vio-yaw-switch-time 1289.3 \
    --vio-yaw-switch-alpha 1.0 &
  PIDS+=($!)
fi

# D2: B until 1339.0, then A
if traj_complete "$DIR_D2"; then
  echo "[skip D2] complete"
else
  rm -f "$DIR_D2/traj.txt" "$DIR_D2/traj.txt.bias" "$DIR_D2/schmidt_yaw_update_diag.csv"
  run_vio D2 "$DIR_D2" \
    --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
    --schmidt-yaw-diag "$DIR_D2/schmidt_yaw_update_diag.csv" \
    --vio-yaw-switch-mode global_yaw_oc_projection \
    --vio-yaw-switch-time 1339.0 \
    --vio-yaw-switch-alpha 1.0 &
  PIDS+=($!)
fi

# D3: B until 1389.0, then A
if traj_complete "$DIR_D3"; then
  echo "[skip D3] complete"
else
  rm -f "$DIR_D3/traj.txt" "$DIR_D3/traj.txt.bias" "$DIR_D3/schmidt_yaw_update_diag.csv"
  run_vio D3 "$DIR_D3" \
    --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
    --schmidt-yaw-diag "$DIR_D3/schmidt_yaw_update_diag.csv" \
    --vio-yaw-switch-mode global_yaw_oc_projection \
    --vio-yaw-switch-time 1389.0 \
    --vio-yaw-switch-alpha 1.0 &
  PIDS+=($!)
fi

# D4: B until 1440.0, then A
if traj_complete "$DIR_D4"; then
  echo "[skip D4] complete"
else
  rm -f "$DIR_D4/traj.txt" "$DIR_D4/traj.txt.bias" "$DIR_D4/schmidt_yaw_update_diag.csv"
  run_vio D4 "$DIR_D4" \
    --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
    --schmidt-yaw-diag "$DIR_D4/schmidt_yaw_update_diag.csv" \
    --vio-yaw-switch-mode global_yaw_oc_projection \
    --vio-yaw-switch-time 1440.0 \
    --vio-yaw-switch-alpha 1.0 &
  PIDS+=($!)
fi

if [ ${#PIDS[@]} -gt 0 ]; then
  echo "[wait] PIDs=${PIDS[*]}"
  for pid in "${PIDS[@]}"; do wait "$pid" || true; done
  echo "[wait] all D variants done"
fi

# ── Eval D variants vs A and B ────────────────────────────────────────────────
eval_pair() {
  local label="$1"; local da="$2"; local db="$3"; local out="$4"
  if ! [ -f "$da/traj.txt" ] || ! [ -f "$db/traj.txt" ]; then
    echo "[skip eval $label] one or both traj missing"
    return 0
  fi
  echo "[eval] $label"
  rm -rf "$out"; mkdir -p "$out"
  python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
    --t0 "$START" \
    --until "$UNTIL_VAL" \
    --dir-a "$da" \
    --dir-b "$db" \
    --gps "$GPS" \
    --imu "$IMU" \
    --out "$out" \
    2>&1 | tee "$out/eval_stdout.txt"
}

EVAL=$BASE/fly4_eval_D_switch

# A vs each D variant (A=dir-a so A is the "baseline" in plots)
eval_pair A_vs_D1 "$DIR_A" "$DIR_D1" "$EVAL/A_vs_D1"
eval_pair A_vs_D2 "$DIR_A" "$DIR_D2" "$EVAL/A_vs_D2"
eval_pair A_vs_D3 "$DIR_A" "$DIR_D3" "$EVAL/A_vs_D3"
eval_pair A_vs_D4 "$DIR_A" "$DIR_D4" "$EVAL/A_vs_D4"

# B vs each D variant (to see how much D retains B's early advantage)
eval_pair B_vs_D1 "$DIR_B" "$DIR_D1" "$EVAL/B_vs_D1"
eval_pair B_vs_D2 "$DIR_B" "$DIR_D2" "$EVAL/B_vs_D2"
eval_pair B_vs_D3 "$DIR_B" "$DIR_D3" "$EVAL/B_vs_D3"
eval_pair B_vs_D4 "$DIR_B" "$DIR_D4" "$EVAL/B_vs_D4"

# Summary table across all variants
echo "[summary] writing $EVAL/summary.txt"
python3 - <<'PYEOF'
import csv, math, os, sys

BASE  = '/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528'
EVAL  = f'{BASE}/fly4_eval_D_switch'
UNTIL = 2816.0
START = 924.4

DIRS = {
  'A':  f'{BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1',
  'B':  f'{BASE}/fly4_schmidt_B_offsetm202p2_start924p4_until2816',
  'D1': f'{BASE}/fly4_D1_switch1289p3_start924p4_until2816',
  'D2': f'{BASE}/fly4_D2_switch1339p0_start924p4_until2816',
  'D3': f'{BASE}/fly4_D3_switch1389p0_start924p4_until2816',
  'D4': f'{BASE}/fly4_D4_switch1440p0_start924p4_until2816',
}

def traj_stats(d):
  f = f'{d}/traj.txt'
  if not os.path.exists(f):
    return None
  with open(f) as fp:
    lines = [l for l in fp if l.strip() and not l.startswith('#')]
  if not lines:
    return None
  last_t = float(lines[-1].split()[0])
  complete = last_t >= UNTIL - 10
  return {'lines': len(lines), 'last_t': last_t, 'complete': complete}

lines_out = []
lines_out.append(f"{'Run':6s}  {'lines':>7s}  {'last_t':>8s}  {'complete':>10s}")
lines_out.append('-' * 40)
for name, d in DIRS.items():
  s = traj_stats(d)
  if s is None:
    lines_out.append(f"{name:6s}  {'MISSING':>7s}")
  else:
    lines_out.append(f"{name:6s}  {s['lines']:>7d}  {s['last_t']:>8.1f}  {'YES' if s['complete'] else 'NO':>10s}")

out_path = f'{EVAL}/summary.txt'
os.makedirs(EVAL, exist_ok=True)
with open(out_path, 'w') as fp:
  fp.write('\n'.join(lines_out) + '\n')

for l in lines_out:
  print(l)
PYEOF

echo ""
echo "[fly4-D-switch] ALL DONE $(date)"
echo "  eval dir: $EVAL"
