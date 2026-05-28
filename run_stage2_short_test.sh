#!/bin/bash
# Stage 2: Short smoke test — A/B/C until t=760.
# Run from /mnt/d/vscode_dir/open_vins inside WSL.
set -euo pipefail
cd /mnt/d/vscode_dir/open_vins

BIN=./build_ov_msckf/run_serial_msckf_ros_free
DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
GPS=$BASE/gps_from_mems_offset438p0_cam_time.csv
FC=$BASE/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
CONFIG=./config/d455_fly2/estimator_config_cond_1e5.yaml
ROOT=$BASE/stage2_smoke_offset438p0_start618_until760
TARGET_T=760

mkdir -p "$ROOT"

reached_target() {
  python3 -c "import sys; t=float(sys.argv[1] or 0); raise SystemExit(0 if t>=$TARGET_T else 1)" "$1"
}

run_until() {
  local name="$1"; shift
  local out="$ROOT/$name"
  if [ -d "$out" ]; then
    echo "[SKIP] $name — already exists"
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
  local deadline=$((SECONDS + 600))
  while kill -0 "$pid" 2>/dev/null; do
    local last=0
    [ -s "$out/traj.txt" ] && last=$(tail -n 1 "$out/traj.txt" | awk '{print $1}')
    if reached_target "$last"; then
      kill -INT "$pid" 2>/dev/null || true
      break
    fi
    [ "$SECONDS" -gt "$deadline" ] && { kill -INT "$pid" 2>/dev/null || true; break; }
    sleep 3
  done
  wait "$pid" || true
}

report() {
  local name="$1"
  local out="$ROOT/$name"
  echo ""
  echo "--- $name ---"

  # exit / traj
  local last_t=0 lines=0
  if [ -s "$out/traj.txt" ]; then
    last_t=$(tail -n 1 "$out/traj.txt" | awk '{print $1}')
    lines=$(wc -l < "$out/traj.txt")
  fi
  echo "  traj.txt:                    $([ -s "$out/traj.txt" ] && echo EXISTS || echo MISSING) ($lines lines, last_t=$last_t)"
  echo "  yaw_update_diag.csv:         $([ -s "$out/yaw_update_diag.csv" ] && echo EXISTS || echo MISSING)"
  echo "  visual_observability_diag.csv: $([ -s "$out/visual_observability_diag.csv" ] && echo EXISTS || echo MISSING)"

  # NaN/Inf check
  if [ -s "$out/traj.txt" ]; then
    if grep -qiE 'nan|inf' "$out/traj.txt"; then
      echo "  NaN/Inf in traj:             YES — FAIL"
    else
      echo "  NaN/Inf in traj:             clean"
    fi
  fi

  # negative cov check
  if grep -q "diagonal at.*is -" "$out/log.txt" 2>/dev/null; then
    echo "  Negative cov diagonal:       YES — FAIL"
  else
    echo "  Negative cov diagonal:       none"
  fi

  # accepted features
  if [ -s "$out/yaw_update_diag.csv" ]; then
    local msckf_acc
    msckf_acc=$(awk -F, 'NR>1 && $2=="MSCKF" {s+=$17; n++} END {if(n>0) printf "mean=%.1f over %d updates",s/n,n; else print "no MSCKF rows"}' "$out/yaw_update_diag.csv")
    echo "  MSCKF accepted features:     $msckf_acc"
  fi

  # VOP diag (rel_norm_HN before/after)
  if [ -s "$out/visual_observability_diag.csv" ]; then
    python3 - "$out/visual_observability_diag.csv" <<'PY'
import sys, csv, statistics
path = sys.argv[1]
before, after = [], []
with open(path) as f:
    for row in csv.DictReader(f):
        try:
            b = float(row["rel_norm_HN_before"])
            a = float(row["rel_norm_HN_after"])
            before.append(b); after.append(a)
        except:
            pass
if before:
    print(f"  VOP rel_norm_HN_before: mean={statistics.mean(before):.4f} median={statistics.median(before):.4f} (n={len(before)})")
    print(f"  VOP rel_norm_HN_after:  mean={statistics.mean(after):.4f} median={statistics.median(after):.4f}")
    ratio = statistics.mean(after) / max(statistics.mean(before), 1e-12)
    print(f"  VOP after/before ratio: {ratio:.4f} ({'GOOD <0.1' if ratio<0.1 else 'CHECK'})")
else:
    print("  VOP diag: no rows or projection not active")
PY
  fi
}

# Run A
run_until A_global_oc_current_baseline \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0

# Run B
run_until B_global_yaw_oc_fej_prechi2 \
  --vio-yaw-update-mode global_yaw_oc_fej_prechi2

# Run C
run_until C_visual_4d_oc_fej_prechi2 \
  --vio-yaw-update-mode visual_4d_oc_fej_prechi2

echo ""
echo "========== STAGE 2 REPORT =========="
report A_global_oc_current_baseline
report B_global_yaw_oc_fej_prechi2
report C_visual_4d_oc_fej_prechi2
echo ""
echo "Stage 2 complete. Root: $ROOT"
