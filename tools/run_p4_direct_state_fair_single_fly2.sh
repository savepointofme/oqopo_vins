#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
OUT_ROOT="${P4_CONTROL_OUT_ROOT:-/mnt/c/Users/baloney/Desktop/P4_direct_state_fair_single_fly2}"
START_TIME="${P4_CONTROL_START_TIME:-700.0}"
UNTIL_TIME="${P4_CONTROL_UNTIL_TIME:-1100.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${OUT_ROOT}/${STAMP}_fly2_single_row_nasa_stride12"
RUNNER="${REPO}/build_p4_sliding_r1/run_serial_msckf_ros_free"
CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"
DATASET=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722
GPS=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260614/gps_refined_finealign.csv
FC_INIT="${P4_CONTROL_FC_INIT:-/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv}"

mkdir -p "${OUT}"
CMD=(
  "${RUNNER}"
  --config "${CONFIG}"
  --dataset "${DATASET}"
  --gps "${GPS}"
  --gps-time-offset 0
  --start-time "${START_TIME}"
  --until-time "${UNTIL_TIME}"
  --initialization-mode fc_full_state
  --init-from-fc "${FC_INIT}"
  --init-from-fc-position-frame global_gnav
  --fc-init-level I0
  --init-att-sigma-deg 3
  --init-pos-sigma 0.05
  --init-bg-sigma 0.003
  --yaw-mode baseline
  --gps-alt-update
  --height-mode nasa_lean
  --gps-alt-sigma 2.0
  --gps-alt-min-pzz 0.01
  --gps-alt-min-t-after-init 10.0
  --gps-alt-max-res 80.0
  --gps-alt-guard-dxy 0.5
  --gps-alt-guard-kxy 5.0
  --gps-alt-cov-psd-check-interval 1.0
  --gps-alt-nasa-beta 0.2
  --gps-alt-nasa-q-threshold 0.0
  --camera-frame-stride 12
  --camera-stride-audit "${OUT}/camera_stride_audit.csv"
  --diag-csv "${OUT}/diag.csv"
  --diag-events "${OUT}/events.txt"
  --output "${OUT}/traj.txt"
  --output-raw "${OUT}/traj_raw.txt"
  --output-nav "${OUT}/traj_nav.txt"
  --nav-frame-metadata-json "${OUT}/nav_frame_metadata.json"
)
if [[ -n "${DISPLAY:-}" ]]; then
  CMD+=(--viz-fast --dash-every 5)
  printf 'visible_dashboard\n' > "${OUT}/visualization_mode.txt"
else
  CMD+=(--no-dashboard)
  printf 'display_unavailable_no_dashboard\n' > "${OUT}/visualization_mode.txt"
fi
printf '%q ' "${CMD[@]}" > "${OUT}/command.txt"
printf '\n' >> "${OUT}/command.txt"
sha256sum "${RUNNER}" "${CONFIG}" > "${OUT}/input_sha256.txt"
date --iso-8601=seconds > "${OUT}/process_start.txt"
/usr/bin/time -v -o "${OUT}/resource_usage.txt" \
  "${CMD[@]}" > "${OUT}/stdout.log" 2> "${OUT}/stderr.log"
printf '0\n' > "${OUT}/exit_code.txt"
date --iso-8601=seconds > "${OUT}/process_end.txt"
printf '%s\n' "${OUT}" > "${OUT_ROOT}/LATEST.txt"
printf 'CONTROL_ROOT=%s\n' "${OUT}"
