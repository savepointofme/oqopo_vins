#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
OUTPUT_PARENT="${P4_SINGLE_ROW_OUTPUT_PARENT:-/mnt/c/Users/baloney/Desktop/实验目录/P4_provisional_gauge_refactor_20260717/current_single_row_focus}"
FLIGHT="${1:?usage: run_current_single_row_focus.sh fly1|fly3}"
BINARY="${REPO}/build_p4_sliding_r1/run_serial_msckf_ros_free"
CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"
HEIGHT_MODE="${P4_HEIGHT_MODE:-guarded}"
ROOT="${OUTPUT_PARENT}/$(date +%Y%m%d_%H%M%S)_${FLIGHT}"
mkdir -p "${ROOT}"

case "${FLIGHT}" in
  fly1)
    DATASET=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
    GPS="${REPO}/config/d455_fly1/fc_gps_cam_time.csv"
    FC="${REPO}/config/d455_fly1/fc_init_state_930.csv"
    START=930.0
    UNTIL="${P4_SINGLE_ROW_FLY1_UNTIL:-1151.0}"
    ;;
  fly3)
    DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
    GPS=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
    FC=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
    START=618.0
    UNTIL="${P4_SINGLE_ROW_FLY3_UNTIL:-947.0}"
    ;;
  *)
    printf 'usage: %s fly1|fly3\n' "$0" >&2
    exit 2
    ;;
esac

cmd=(
  "${BINARY}"
  --config "${CONFIG}"
  --dataset "${DATASET}"
  --gps "${GPS}"
  --gps-time-offset 0
  --start-time "${START}"
  --until-time "${UNTIL}"
  --initialization-mode fc_full_state
  --init-from-fc "${FC}"
  --init-from-fc-position-frame global_gnav
  --init-att-sigma-deg 3.0
  --init-pos-sigma 0.05
  --init-vel-sigma 5.0
  --init-bg-sigma 0.003
  --init-ba-sigma 1.0
  --yaw-mode baseline
  --gps-alt-update
  --height-mode "${HEIGHT_MODE}"
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
  --camera-stride-audit "${ROOT}/camera_stride_audit.csv"
  --diag-csv "${ROOT}/diag.csv"
  --diag-events "${ROOT}/events.txt"
  --output "${ROOT}/traj.txt"
  --output-raw "${ROOT}/traj_raw.txt"
  --output-nav "${ROOT}/traj_nav.txt"
)
if [[ -n "${DISPLAY:-}" ]]; then
  cmd+=(--viz-fast --dash-every 5)
else
  cmd+=(--no-dashboard)
fi
printf '%q ' "${cmd[@]}" > "${ROOT}/command.txt"
printf '\n' >> "${ROOT}/command.txt"
date --iso-8601=seconds > "${ROOT}/process_start.txt"
/usr/bin/time -v -o "${ROOT}/resource_usage.txt" \
  "${cmd[@]}" > "${ROOT}/stdout.log" 2> "${ROOT}/stderr.log"
date --iso-8601=seconds > "${ROOT}/process_end.txt"
printf '0\n' > "${ROOT}/exit_code.txt"
printf '%s\n' "${ROOT}"
