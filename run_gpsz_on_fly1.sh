#!/bin/bash
# GPS-Z ON rerun — Fly1 — global_yaw_oc_projection alpha=1.0 + GPS-Z guard
set -euo pipefail
export DISPLAY=:0

RUNNER=/mnt/d/vscode_dir/open_vins/build_ov_msckf/run_serial_msckf_ros_free
DATASET=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
CONFIG=/mnt/d/vscode_dir/open_vins/config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml
GPS=/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv
FC_INIT=/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_init_state_930.csv
OUT=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/gpsz_on_rerun_fly1_global_oc_guarded

mkdir -p "$OUT"

# Save exact command
cat > "$OUT/cmd.sh" << 'CMDEOF'
#!/bin/bash
# Fly1 GPS-Z ON rerun — exact command
CMDEOF

echo "=== FLY1 GPS-Z ON RERUN ===" | tee "$OUT/log.txt"
echo "Start: $(date)" | tee -a "$OUT/log.txt"
echo "" | tee -a "$OUT/log.txt"

"$RUNNER" \
  --config "$CONFIG" \
  --dataset "$DATASET" \
  --gps "$GPS" \
  --gps-time-offset 0 \
  --start-time 930 \
  --init-from-fc "$FC_INIT" \
  --init-att-sigma-deg 3 \
  --init-pos-sigma 0.05 \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --viz-fast \
  --dash-every 5 \
  --diag-csv "$OUT/diag.csv" \
  --diag-events "$OUT/events.txt" \
  --vio-yaw-diag "$OUT/yaw_diag.csv" \
  --output "$OUT/traj.txt" \
  2>&1 | tee -a "$OUT/log.txt"

echo "" | tee -a "$OUT/log.txt"
echo "Done: $(date)" | tee -a "$OUT/log.txt"
echo "Last line: $(tail -1 "$OUT/traj.txt" 2>/dev/null)" | tee -a "$OUT/log.txt"
