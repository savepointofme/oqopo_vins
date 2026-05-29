#!/bin/bash
# Windowed FEJ diagnostic: 924.4 → 1600s
# Compares B_current vs B_FEJ across three windows: 1050-1200, 1200-1400, 1400-1600
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

DATASET=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549
BASE=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528
GPS=$BASE/gps_from_mems_offsetm202p2_cam_time.csv
FC=$BASE/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config_cond_1e5.yaml
BIN=./build_ov_msckf/run_serial_msckf_ros_free
START=924.4
UNTIL=1600

OUT_FEJ=$BASE/fly4_fej_windowed_diag
OUT_CUR=$BASE/fly4_cur_windowed_diag

echo "[windowed-diag] start $(date)  HEAD=$(git rev-parse HEAD)"
echo "[windowed-diag] full window: $START → $UNTIL"

traj_complete() {
  local f="$1/traj.txt"
  [ -f "$f" ] || return 1
  local last; last=$(tail -1 "$f" | awk '{print $1}')
  python3 -c "import sys; sys.exit(0 if float('${last:-0}') >= $(echo "$UNTIL - 10" | bc) else 1)" 2>/dev/null
}

run_mode() {
  local tag="$1"; local out="$2"; local mode="$3"
  if traj_complete "$out"; then
    echo "[skip $tag] already complete"
    return
  fi
  rm -rf "$out"; mkdir -p "$out"
  echo "[START $tag]"
  "$BIN" \
    --config "$CONFIG" --dataset "$DATASET" \
    --gps "$GPS" --gps-time-offset 0 \
    --start-time "$START" --until-time "$UNTIL" \
    --init-from-fc "$FC" --init-bg-sigma 0.003 \
    --vio-yaw-update-mode "$mode" \
    --schmidt-yaw-diag "$out/schmidt_diag.csv" \
    --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 \
    --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
    --viz-fast --dash-every 5 \
    --output "$out/traj.txt" \
    > "$out/log.txt" 2>&1
  local lines=0; [ -f "$out/traj.txt" ] && lines=$(wc -l < "$out/traj.txt")
  echo "[DONE $tag  lines=$lines  t=$(tail -1 "$out/traj.txt" 2>/dev/null | awk '{print $1}')]"
}

run_mode FEJ     "$OUT_FEJ" "visual_yaw_schmidt_fej_gauge"     &
run_mode CURRENT "$OUT_CUR" "visual_yaw_schmidt_current_gauge" &
wait

echo "[analysis] running windowed diagnostics..."

python3 - "$OUT_FEJ/schmidt_diag.csv" "$OUT_CUR/schmidt_diag.csv" <<'PYEOF'
import sys, csv, math, numpy as np

WINDOWS = [
    ("1050-1200s", 1050.0, 1200.0),
    ("1200-1400s", 1200.0, 1400.0),
    ("1400-1600s", 1400.0, 1600.0),
]

def load_diag(path):
    rows = []
    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            for r in reader:
                rows.append(r)
        print(f"  loaded {len(rows)} rows from {path}")
    except FileNotFoundError:
        print(f"  MISSING: {path}")
    return rows

def fv(rows, col):
    out = []
    for r in rows:
        try:
            v = float(r.get(col, "nan"))
            if math.isfinite(v):
                out.append(v)
        except (ValueError, TypeError):
            pass
    return np.array(out)

def stats(arr, label, fmt=".6f"):
    if len(arr) == 0:
        print(f"    {label}: NO DATA")
        return
    p95 = np.percentile(arr, 95)
    print(f"    {label}: mean={np.mean(arr):{fmt}}  P95={p95:{fmt}}  max={np.max(arr):{fmt}}")

def abs_stats(arr, label, fmt=".6f"):
    stats(np.abs(arr), label, fmt)

path_fej = sys.argv[1]
path_cur = sys.argv[2]

print("\n=== Loading diag files ===")
all_fej = load_diag(path_fej)
all_cur = load_diag(path_cur)

def filter_window(rows, t0, t1):
    return [r for r in rows if t0 <= float(r.get("timestamp",0)) < t1]

for win_label, t0, t1 in WINDOWS:
    fej = filter_window(all_fej, t0, t1)
    cur = filter_window(all_cur, t0, t1)

    print(f"\n{'='*68}")
    print(f"  WINDOW: {win_label}   FEJ={len(fej)} updates   CURRENT={len(cur)} updates")
    print(f"{'='*68}")

    for mode_label, rows in [("FEJ", fej), ("CURRENT", cur)]:
        print(f"\n  ── {mode_label} ──")
        if not rows:
            print("    No data in this window.")
            continue

        ts = fv(rows, "timestamp")
        print(f"    updates: {len(ts)}  t=[{ts.min():.1f}, {ts.max():.1f}]")

        # 1. Projection invariant
        print("\n  1. Schmidt invariant  q_used^T dx_eff:")
        abs_stats(fv(rows, "schmidt_dx_s_coeff_after"), "abs(schmidt_dx_s_coeff_after)", ".3e")

        # 2. H-nullspace of q_used
        print("\n  2. H-nullspace q_used  rel_norm_HQ:")
        stats(fv(rows, "rel_norm_HQ"), "rel_norm_HQ", ".3e")

        # 3. Current leakage (q_alt^T dx_eff; in FEJ mode q_alt=q_current, in current q_alt=q_fej)
        print("\n  3. Alt-gauge leakage  q_alt^T dx_eff:")
        abs_stats(fv(rows, "q_alt_dot_dx_eff"), "abs(q_alt_dot_dx_eff)", ".6f")

        # 4. Gauge mismatch  angle(q_cur, q_fej) — explicit field
        print("\n  4. Gauge mismatch  angle(q_current, q_fej):")
        stats(fv(rows, "angle_q_cur_fej_deg"), "angle_q_cur_fej_deg", ".4f")

        # 5. Alt H-nullspace
        print("\n  5. H-nullspace q_alt  rel_norm_Hq_alt:")
        stats(fv(rows, "rel_norm_Hq_alt"), "rel_norm_Hq_alt", ".3e")

        # 6. Normal update gauge fraction
        print("\n  6. Normal update gauge fraction  q_used^T dx_normal:")
        abs_stats(fv(rows, "normal_dx_s_coeff_before"), "abs(normal_dx_s_coeff_before)", ".6f")

        # 7. norm_delta_dx
        print("\n  7. norm_delta_dx  (||dx_normal - dx_eff||):")
        stats(fv(rows, "norm_delta_dx"), "norm_delta_dx", ".6f")

        # 8. Pas_change_norm
        print("\n  8. Pas_change_norm:")
        stats(fv(rows, "Pas_change_norm"), "Pas_change_norm", ".6f")

        # 9. Pss
        pss = fv(rows, "Pss_norm_before")
        if len(pss):
            print(f"\n  9. Pss (q^T P q): mean={np.mean(pss):.2e}  min={np.min(pss):.2e}  max={np.max(pss):.2e}")

        # 10. q-energy decomposition (used gauge)
        print("\n  10. q_used energy decomposition:")
        for blk in ["q_energy_imu_ori", "q_energy_imu_pos", "q_energy_imu_vel",
                    "q_energy_clone_ori", "q_energy_clone_pos",
                    "q_energy_slam", "q_energy_bias_calib"]:
            arr = fv(rows, blk)
            if len(arr):
                print(f"      {blk:30s}: {np.mean(arr):.6f}")

        # 11. q_mix diagnostics
        print("\n  11. q_mixed (IMU-cur + clone-FEJ) diagnostics:")
        stats(fv(rows, "angle_q_cur_mix_deg"), "angle(q_cur, q_mix) deg", ".4f")
        stats(fv(rows, "angle_q_fej_mix_deg"), "angle(q_fej, q_mix) deg", ".4f")
        stats(fv(rows, "q_mix_rel_norm_Hq"), "q_mix rel_norm_Hq", ".3e")
        abs_stats(fv(rows, "q_mix_dot_dx_eff"), "abs(q_mix^T dx_eff)", ".6f")
        print("      q_mix energy:")
        for blk in ["q_mix_energy_imu_ori", "q_mix_energy_imu_pos", "q_mix_energy_imu_vel",
                    "q_mix_energy_clone_ori", "q_mix_energy_clone_pos",
                    "q_mix_energy_slam", "q_mix_energy_bias_calib"]:
            arr = fv(rows, blk)
            if len(arr):
                print(f"        {blk:35s}: {np.mean(arr):.6f}")

        # 12. Health
        neg = int(np.sum(fv(rows, "neg_diag_clamp_count")))
        nan_inf = sum(1 for r in rows if r.get("skipped_reason","") in
                      ("dx_eff_nan_inf", "norm_Q_full_below_threshold"))
        print(f"\n  12. Health: neg_diag_clamp={neg}  NaN/Inf={nan_inf}")

print("\n[windowed-diag] analysis DONE")
PYEOF

echo "[windowed-diag] ALL DONE $(date)"
