#!/bin/bash
# Evaluate global_yaw_oc_fej_prechi2 vs global_oc_alpha1 on all 4 flights.
# Uses eval_stage.py (start+yaw alignment, ATE, yaw error, XY overlay plots).
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

EVAL=./eval_stage.py

FLY4_BASE=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528
FLY3_BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
FLY2_BASE=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525
FLY1_BASE=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result

echo "[eval-all] $(date)  HEAD=$(git rev-parse HEAD)"

run_eval() {
  local label="$1" t0="$2" until="$3"
  local dir_a="$4" dir_b="$5"
  local gps="$6" imu="$7" out="$8"
  if ! [ -f "$dir_a/traj.txt" ] || ! [ -f "$dir_b/traj.txt" ]; then
    echo "[skip $label] missing traj: a=$([ -f "$dir_a/traj.txt" ] && echo ok || echo MISSING)  b=$([ -f "$dir_b/traj.txt" ] && echo ok || echo MISSING)"
    return
  fi
  echo "[eval] $label  t0=$t0  until=$until"
  rm -rf "$out"; mkdir -p "$out"
  python3 "$EVAL" \
    --t0 "$t0" --until "$until" \
    --dir-a "$dir_a" --dir-b "$dir_b" \
    --gps "$gps" --imu "$imu" \
    --out "$out" \
    2>&1 | tee "$out/eval_stdout.txt"
  echo "[done $label]"
}

# ── Fly 1: wrap flat traj.txt into a dir so eval_stage.py can find it ────────
IMU1=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810/imu0/data.csv
GPS1=/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv
FLY1_A1_WRAP=$FLY1_BASE/globaloc_alpha1_wrap_start930
mkdir -p "$FLY1_A1_WRAP"
cp "$FLY1_BASE/A1_fcinit_px2_globaloc1p0_traj.txt"       "$FLY1_A1_WRAP/traj.txt"
cp "$FLY1_BASE/A1_fcinit_px2_globaloc1p0_traj.txt.bias"  "$FLY1_A1_WRAP/traj.txt.bias" 2>/dev/null || true
# dir-a = FEJ-OC (B in eval labels), dir-b = global_oc (A in eval labels)
run_eval fly1 930.0 1744 \
  "$FLY1_A1_WRAP" \
  "$FLY1_BASE/fej_oc_prechi2_start930" \
  "$GPS1" "$IMU1" \
  "$FLY1_BASE/eval_fej_oc_vs_globaloc_start930"

# ── Fly 2: compare over the overlapping window both runs cover ───────────────
IMU2=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722/imu0/data.csv
GPS2=$FLY2_BASE/gps_from_mems_offset450p5_cam_time.csv
run_eval fly2 700.0 1350 \
  "$FLY2_BASE/clean_ablate_oc_alpha10" \
  "$FLY2_BASE/fej_oc_prechi2_start700" \
  "$GPS2" "$IMU2" \
  "$FLY2_BASE/eval_fej_oc_vs_globaloc_start700_until1350"

# ── Fly 3: FEJ-OC vs phase6 global_oc baseline (both ran to ~1600s) ──────────
IMU3=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946/imu0/data.csv
GPS3=$FLY3_BASE/gps_from_mems_offset438p0_cam_time.csv
PHASE6=$FLY3_BASE/phase6_oc_experiments_offset438p0_start618_until1600/global_oc_current_late_baseline_offset438p0_until1600
run_eval fly3 618.0 1600 \
  "$PHASE6" \
  "$FLY3_BASE/fej_oc_prechi2_start618" \
  "$GPS3" "$IMU3" \
  "$FLY3_BASE/eval_fej_oc_vs_globaloc_start618_until1600"

# ── Fly 4: full flight ────────────────────────────────────────────────────────
IMU4=/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549/imu0/data.csv
GPS4=$FLY4_BASE/gps_from_mems_offsetm202p2_cam_time.csv
run_eval fly4 924.4 2816 \
  "$FLY4_BASE/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1" \
  "$FLY4_BASE/fej_oc_prechi2_start924p4" \
  "$GPS4" "$IMU4" \
  "$FLY4_BASE/eval_fej_oc_vs_globaloc_start924p4"

echo ""
echo "[eval-all] DONE $(date)"
