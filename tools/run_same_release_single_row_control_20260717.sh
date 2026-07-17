#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
FLIGHT="${1:?usage: run_same_release_single_row_control_20260717.sh fly1|fly2|fly3 FC_INIT START UNTIL OUTPUT_PARENT}"
FC_INIT="${2:?missing FC init row}"
START_TIME="${3:?missing start time}"
UNTIL_TIME="${4:?missing until time}"
OUTPUT_PARENT="${5:?missing output parent}"
BINARY="${REPO}/build_p4_sliding_r1/run_serial_msckf_ros_free"
CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"

case "${FLIGHT}" in
  fly1)
    DATASET=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
    GPS="${REPO}/config/d455_fly1/fc_gps_cam_time.csv"
    ;;
  fly2)
    DATASET=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722
    GPS=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260614/gps_refined_finealign.csv
    ;;
  fly3)
    DATASET=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
    GPS=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
    ;;
  *)
    printf 'unsupported flight: %s\n' "${FLIGHT}" >&2
    exit 2
    ;;
esac

OUT="${OUTPUT_PARENT}/$(date +%Y%m%d_%H%M%S)_${FLIGHT}_same_release_single_row"
mkdir -p "${OUT}"
CMD=(
  "${BINARY}"
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
  --init-att-sigma-deg 3.0
  --init-pos-sigma 0.05
  --init-vel-sigma 5.0
  --init-bg-sigma 0.003
  --init-ba-sigma 1.0
  --yaw-mode baseline
  --gps-alt-update
  --height-mode guarded
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
sha256sum "${BINARY}" "${CONFIG}" "${FC_INIT}" > "${OUT}/input_sha256.txt"
date --iso-8601=seconds > "${OUT}/process_start.txt"
set +e
/usr/bin/time -v -o "${OUT}/resource_usage.txt" \
  "${CMD[@]}" > "${OUT}/stdout.log" 2> "${OUT}/stderr.log"
code=$?
set -e
printf '%s\n' "${code}" > "${OUT}/exit_code.txt"
date --iso-8601=seconds > "${OUT}/process_end.txt"
printf '%s\n' "${OUT}"
exit "${code}"
