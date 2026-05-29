#!/bin/bash
# bg_z ablation: B_current with 5 visual_bgz_update_scale values + A baseline
# Tests whether visual updates corrupting bg_z is the root cause of B's heading error.
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
UNTIL=2816

DIR_A=$BASE/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1

echo "[bgz-ablation] start $(date)  HEAD=$(git rev-parse HEAD)"

traj_complete() {
  local f="$1/traj.txt"
  [ -f "$f" ] || return 1
  local last; last=$(tail -1 "$f" | awk '{print $1}')
  python3 -c "import sys; sys.exit(0 if float('${last:-0}') >= $(echo "$UNTIL - 10" | bc) else 1)" 2>/dev/null
}

run_bgz() {
  local tag="$1"; local out="$2"; local scale="$3"
  if traj_complete "$out"; then
    echo "[skip $tag] complete"
    return
  fi
  rm -rf "$out"; mkdir -p "$out"
  echo "[START $tag bgz_scale=$scale]"
  "$BIN" \
    --config "$CONFIG" --dataset "$DATASET" \
    --gps "$GPS" --gps-time-offset 0 \
    --start-time "$START" --until-time "$UNTIL" \
    --init-from-fc "$FC" --init-bg-sigma 0.003 \
    --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
    --schmidt-yaw-diag "$out/schmidt_diag.csv" \
    --visual-bgz-update-scale "$scale" \
    --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
    --viz-fast --dash-every 5 \
    --diag-csv "$out/diag.csv" \
    --output "$out/traj.txt" \
    > "$out/log.txt" 2>&1
  local lines=0; [ -f "$out/traj.txt" ] && lines=$(wc -l < "$out/traj.txt")
  echo "[DONE $tag  lines=$lines  t=$(tail -1 "$out/traj.txt" 2>/dev/null | awk '{print $1}')]"
}

PIDS=()

run_bgz B_bgz1p0  "$BASE/fly4_B_bgz1p0_start924p4_until2816"  1.0  &  PIDS+=($!)
run_bgz B_bgz0    "$BASE/fly4_B_bgz0_start924p4_until2816"    0.0  &  PIDS+=($!)
run_bgz B_bgz0p1  "$BASE/fly4_B_bgz0p1_start924p4_until2816"  0.1  &  PIDS+=($!)
run_bgz B_bgz0p25 "$BASE/fly4_B_bgz0p25_start924p4_until2816" 0.25 &  PIDS+=($!)
run_bgz B_bgz0p5  "$BASE/fly4_B_bgz0p5_start924p4_until2816"  0.5  &  PIDS+=($!)

echo "[wait] PIDs=${PIDS[*]}"
for pid in "${PIDS[@]}"; do wait "$pid" || true; done
echo "[wait] all variants done"

# ── Eval each variant vs A ─────────────────────────────────────────────────
EVAL=$BASE/fly4_eval_bgz_ablation

eval_pair() {
  local label="$1"; local da="$2"; local db="$3"; local out="$4"
  if ! [ -f "$da/traj.txt" ] || ! [ -f "$db/traj.txt" ]; then
    echo "[skip eval $label] missing traj"
    return 0
  fi
  echo "[eval] $label"
  rm -rf "$out"; mkdir -p "$out"
  python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
    --t0 "$START" --until "$UNTIL" \
    --dir-a "$da" --dir-b "$db" \
    --gps "$GPS" --imu "$IMU" --out "$out" \
    2>&1 | tee "$out/eval_stdout.txt"
}

eval_pair A_vs_Bbgz1p0  "$DIR_A" "$BASE/fly4_B_bgz1p0_start924p4_until2816"  "$EVAL/A_vs_B_bgz1p0"
eval_pair A_vs_Bbgz0    "$DIR_A" "$BASE/fly4_B_bgz0_start924p4_until2816"    "$EVAL/A_vs_B_bgz0"
eval_pair A_vs_Bbgz0p1  "$DIR_A" "$BASE/fly4_B_bgz0p1_start924p4_until2816"  "$EVAL/A_vs_B_bgz0p1"
eval_pair A_vs_Bbgz0p25 "$DIR_A" "$BASE/fly4_B_bgz0p25_start924p4_until2816" "$EVAL/A_vs_B_bgz0p25"
eval_pair A_vs_Bbgz0p5  "$DIR_A" "$BASE/fly4_B_bgz0p5_start924p4_until2816"  "$EVAL/A_vs_B_bgz0p5"

# ── bg_z timeline summary ──────────────────────────────────────────────────
python3 - <<'PYEOF'
import csv, numpy as np, os

BASE = '/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528'
VARIANTS = {
    'A':         f'{BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/traj.txt.bias',
    'B_bgz1.0':  f'{BASE}/fly4_B_bgz1p0_start924p4_until2816/traj.txt.bias',
    'B_bgz0.5':  f'{BASE}/fly4_B_bgz0p5_start924p4_until2816/traj.txt.bias',
    'B_bgz0.25': f'{BASE}/fly4_B_bgz0p25_start924p4_until2816/traj.txt.bias',
    'B_bgz0.1':  f'{BASE}/fly4_B_bgz0p1_start924p4_until2816/traj.txt.bias',
    'B_bgz0':    f'{BASE}/fly4_B_bgz0_start924p4_until2816/traj.txt.bias',
}

def load_bgz(path):
    rows = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'): continue
                p = line.split()
                if len(p) >= 7:
                    try: rows.append((float(p[0]), float(p[6])))
                    except ValueError: pass
    except FileNotFoundError: pass
    return sorted(rows, key=lambda x: x[0])

data = {name: load_bgz(path) for name, path in VARIANTS.items()}
bins = np.arange(924, 2820, 200)

hdr = f"  {'Bin':11s}" + "".join(f"  {k:>12s}" for k in data)
print("\n" + "="*len(hdr))
print("  bg_z (×1e-4 rad/s) by 200s epoch — all variants")
print("="*len(hdr))
print(hdr)
print("  " + "-"*(len(hdr)-2))
for t0 in bins:
    t1 = t0 + 200
    line = f"  {t0:.0f}-{t1:.0f}     "
    for name, rows in data.items():
        rs = [bgz for t, bgz in rows if t0 <= t < t1]
        line += f"  {np.mean(rs)*1e4:12.4f}" if rs else f"  {'N/A':>12s}"
    print(line)

# trajectory status
print("\n  Run status:")
for name, path in VARIANTS.items():
    traj = path.replace('traj.txt.bias', 'traj.txt')
    if not os.path.exists(traj):
        print(f"    {name:14s}: MISSING")
        continue
    with open(traj) as f:
        lines = [l for l in f if l.strip() and not l.startswith('#')]
    last_t = float(lines[-1].split()[0]) if lines else 0
    complete = last_t >= 2816 - 10
    print(f"    {name:14s}: {len(lines):6d} lines  t={last_t:.1f}  {'OK' if complete else 'INCOMPLETE'}")
PYEOF

echo ""
echo "[bgz-ablation] ALL DONE $(date)"
echo "  eval: $EVAL"
