#!/bin/bash
# Short FEJ diagnostic pass: 924.4 → 985s (~60s, ~1700 visual updates)
# Runs FEJ and current-gauge in parallel, then reports invariants from diag CSVs.
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549
BASE=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528
GPS=$BASE/gps_from_mems_offsetm202p2_cam_time.csv
FC=$BASE/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free
START=924.4
UNTIL=985   # ~60s window

OUT_FEJ=$BASE/fly4_fej_diag_short
OUT_CUR=$BASE/fly4_cur_diag_short

echo "[fej-diag] start $(date)  HEAD=$(git rev-parse HEAD)"
echo "[fej-diag] window: $START → $UNTIL  (~$(echo "$UNTIL - $START" | bc)s)"

common_flags() {
  echo --config "$CONFIG" \
    --dataset "$DATASET" \
    --gps "$GPS" \
    --gps-time-offset 0 \
    --start-time "$START" \
    --until-time "$UNTIL" \
    --init-from-fc "$FC" \
    --init-bg-sigma 0.003 \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 \
    --gps-alt-guard-kxy 5.0 \
    --viz-fast \
    --dash-every 5
}

rm -rf "$OUT_FEJ" "$OUT_CUR"
mkdir -p "$OUT_FEJ" "$OUT_CUR"

echo "[START FEJ]"
$BIN $(common_flags) \
  --vio-yaw-update-mode visual_yaw_schmidt_fej_gauge \
  --schmidt-yaw-diag "$OUT_FEJ/schmidt_diag.csv" \
  --output "$OUT_FEJ/traj.txt" \
  > "$OUT_FEJ/log.txt" 2>&1 &
PID_FEJ=$!

echo "[START CURRENT]"
$BIN $(common_flags) \
  --vio-yaw-update-mode visual_yaw_schmidt_current_gauge \
  --schmidt-yaw-diag "$OUT_CUR/schmidt_diag.csv" \
  --output "$OUT_CUR/traj.txt" \
  > "$OUT_CUR/log.txt" 2>&1 &
PID_CUR=$!

wait $PID_FEJ || true
wait $PID_CUR || true

echo "[DONE FEJ lines=$(wc -l < "$OUT_FEJ/traj.txt" 2>/dev/null || echo 0)  diag=$(wc -l < "$OUT_FEJ/schmidt_diag.csv" 2>/dev/null || echo 0)]"
echo "[DONE CUR lines=$(wc -l < "$OUT_CUR/traj.txt" 2>/dev/null || echo 0)  diag=$(wc -l < "$OUT_CUR/schmidt_diag.csv" 2>/dev/null || echo 0)]"

python3 - "$OUT_FEJ/schmidt_diag.csv" "$OUT_CUR/schmidt_diag.csv" <<'PYEOF'
import sys, csv, math, numpy as np

def load_diag(path):
    rows = []
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(r)
    except FileNotFoundError:
        print(f"  MISSING: {path}")
    return rows

def fv(rows, col):
    out = []
    for r in rows:
        try:
            v = float(r[col])
            if math.isfinite(v):
                out.append(v)
        except (KeyError, ValueError):
            pass
    return np.array(out)

def stats(arr, label, fmt=".6f"):
    if len(arr) == 0:
        print(f"  {label}: NO DATA")
        return
    p95 = np.percentile(arr, 95)
    print(f"  {label}: mean={np.mean(arr):{fmt}}  P95={p95:{fmt}}  max={np.max(arr):{fmt}}")

def abs_stats(arr, label, fmt=".6f"):
    stats(np.abs(arr), label, fmt)

path_fej = sys.argv[1]
path_cur = sys.argv[2]

fej = load_diag(path_fej)
cur = load_diag(path_cur)

for label, rows in [("FEJ", fej), ("CURRENT", cur)]:
    print(f"\n{'='*62}")
    print(f"  MODE: {label}   ({len(rows)} updates)")
    print(f"{'='*62}")

    if not rows:
        print("  No data.")
        continue

    ts = fv(rows, "timestamp")
    print(f"  timestamp range: {ts.min():.3f} → {ts.max():.3f}  (n={len(ts)})")

    # 1. FEJ projection invariant: q_used^T dx_eff
    print("\n1. Schmidt projection invariant  q_used^T dx_eff:")
    abs_stats(fv(rows, "schmidt_dx_s_coeff_after"), "  abs(schmidt_dx_s_coeff_after)", ".3e")

    # 2. FEJ H-nullspace: rel_norm_HQ (= rel_norm_Hq_used)
    print("\n2. H-nullspace of q_used  (rel_norm_HQ = ||H q_used|| / ||H||):")
    stats(fv(rows, "rel_norm_HQ"), "  rel_norm_HQ", ".6f")

    # 3. Current leakage under FEJ projection  q_alt^T dx_eff
    print("\n3. Alt-gauge leakage  q_alt^T dx_eff:")
    abs_stats(fv(rows, "q_alt_dot_dx_eff"), "  abs(q_alt_dot_dx_eff)", ".6f")

    # 4. Current-vs-FEJ gauge angle
    print("\n4. Gauge mismatch  angle(q_used, q_alt):")
    stats(fv(rows, "angle_q_alt_deg"), "  angle_q_alt_deg", ".4f")

    # 5. Alt H-nullspace  rel_norm_Hq_alt
    print("\n5. H-nullspace of q_alt  (rel_norm_Hq_alt = ||H q_alt|| / ||H||):")
    stats(fv(rows, "rel_norm_Hq_alt"), "  rel_norm_Hq_alt", ".6f")

    # 6. q-energy decomposition
    print("\n6. q-energy decomposition (mean Euclidean fractions, ||q||=1):")
    for blk in ["q_energy_imu_ori", "q_energy_imu_pos", "q_energy_imu_vel",
                "q_energy_clone_ori", "q_energy_clone_pos",
                "q_energy_slam", "q_energy_bias_calib"]:
        arr = fv(rows, blk)
        if len(arr):
            print(f"  {blk}: {np.mean(arr):.6f}")

    # 7. Health
    print("\n7. Health:")
    nan_inf = sum(1 for r in rows if r.get("skipped_reason","") in
                  ("dx_eff_nan_inf", "norm_Q_full_below_threshold"))
    neg = int(sum(fv(rows, "neg_diag_clamp_count")))
    print(f"  neg_diag_clamp_count total: {neg}")
    print(f"  NaN/Inf skipped: {nan_inf}")

    # Additional: normal_dx_s_coeff_before (gauge fraction of standard update)
    print("\n8. Normal update gauge fraction  q_used^T dx_normal:")
    abs_stats(fv(rows, "normal_dx_s_coeff_before"), "  abs(normal_dx_s_coeff_before)", ".6f")

print("\n[fej-diag] DONE")
PYEOF

echo "[fej-diag] ALL DONE $(date)"
