#!/bin/bash
# EXPERIMENTAL OC modes — NOT FOR BASELINE COMPARISON
#
# These modes are NOT official. Do not use them in:
#   - four-flight comparison tables
#   - condition comparison packages
#   - any result labeled as "baseline" or "official_oc"
#
# Status per fly3 validation (2026-06-07):
#   oc_prechi2_fej_gauge : 638m full RMS — WORSE than openvins_fej (448m)
#   oc_4d_prechi2_fej_gauge : 646m full RMS — WORST of all four modes
#
# These modes are retained for research into pre-chi2 OC methods only.
# Do NOT call them MSCKF2.0. No IJRR 2013 equivalence has been audited.
#
# Usage:
#   bash run_experimental_oc_modes.sh fly3 prechi2   # run oc_prechi2 on fly3
#   bash run_experimental_oc_modes.sh fly3 4d        # run oc_4d on fly3
#   bash run_experimental_oc_modes.sh fly3 both      # run both on fly3
#
set -euo pipefail

FLIGHT=${1:-fly3}
MODE=${2:-both}

BIN=/mnt/d/vscode_dir/open_vins/build_ov_msckf/run_serial_msckf_ros_free

# --- Per-flight paths ---
case "$FLIGHT" in
  fly3)
    DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
    GPS=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
    FC=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
    OUTBASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/experimental
    START=618
    ;;
  *)
    echo "ERROR: Unknown flight '$FLIGHT'. Supported: fly3"
    exit 1
    ;;
esac

CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly2/estimator_config.yaml

echo "[experimental] WARNING: These modes are EXPERIMENTAL_ONLY / NOT_FOR_BASELINE"

run_mode() {
  local label="$1"
  local gauge="$2"
  local out="$OUTBASE/${FLIGHT}_${label}_fi1e4_fcinit"
  mkdir -p "$out"
  echo "[experimental] Running $label on $FLIGHT..."
  "$BIN" \
    --config "$CONFIG" \
    --dataset "$DATASET" \
    --gps "$GPS" \
    --gps-time-offset 0 \
    --start-time "$START" \
    --init-from-fc "$FC" \
    --vio-yaw-gauge-mode "$gauge" \
    --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
    --gps-alt-max-res 80 --gps-alt-min-t-after-init 10 \
    --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
    --no-display --viz-fast --dash-every 5 \
    --output "$out/traj.txt" \
    2>&1 | tee "$out/log.txt"
  echo "[experimental] Done $label"
}

if [[ "$MODE" == "prechi2" || "$MODE" == "both" ]]; then
  run_mode oc_prechi2_fej_gauge oc_prechi2_fej_gauge
fi

if [[ "$MODE" == "4d" || "$MODE" == "both" ]]; then
  run_mode oc_4d_prechi2_fej_gauge oc_4d_prechi2_fej_gauge
fi

echo "[experimental] All done. Results in $OUTBASE/"
echo "[experimental] Remember: do NOT include these in official baseline tables."
