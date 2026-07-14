#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
SOURCE_REPO="${P4_SOURCE_REPO:-/mnt/d/vscode_dir/open_vins}"
GIT_DIR="${P4_R1_GIT_DIR:-${SOURCE_REPO}/.git/worktrees/open_vins_p4_sliding_r1}"
FC_ROOT="${P4_CALIBRATED_FC_ROOT:-${SOURCE_REPO}/readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs}"
OUTPUT_PARENT="${P4_R1_OUTPUT_PARENT:-/mnt/c/Users/baloney/Desktop/P4_sliding_window_r1_20260714}"
ROOT="${OUTPUT_PARENT}/$(date +%Y%m%d_%H%M%S)_short_shadow"
BINARY="${REPO}/build_p4_sliding_r1/run_serial_msckf_ros_free"
TEST_BINARY="${REPO}/build_p4_sliding_r1/test_online_alignment_initializer"
CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"
VALIDATOR="${REPO}/analysis/validate_p4_sliding_window_r1.py"

mkdir -p "${ROOT}"
git --git-dir="${GIT_DIR}" --work-tree="${REPO}" rev-parse HEAD \
  > "${ROOT}/git_head.txt"
git --git-dir="${GIT_DIR}" --work-tree="${REPO}" status --short \
  > "${ROOT}/git_status.txt"
sha256sum "${BINARY}" "${TEST_BINARY}" "${CONFIG}" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.h" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.cpp" \
  "${REPO}/ov_msckf/src/run_serial_msckf_ros_free.cpp" \
  > "${ROOT}/implementation_sha256.txt"
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}" \
  > "${ROOT}/display_environment.txt"
"${TEST_BINARY}" > "${ROOT}/preflight_initializer_test.log" 2>&1

run_one() {
  local flight="$1"
  local dataset gps fc_stream start_time until_time
  case "${flight}" in
    fly1)
      dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
      gps="${REPO}/config/d455_fly1/fc_gps_cam_time.csv"
      fc_stream="${FC_ROOT}/fly1_fc_navigation_online.csv"
      start_time=930.0
      until_time=952.0
      ;;
    fly3)
      dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
      gps=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
      fc_stream="${FC_ROOT}/fly3_fc_navigation_online.csv"
      start_time=618.0
      until_time=640.0
      ;;
  esac

  local out="${ROOT}/${flight}"
  mkdir -p "${out}"
  local cmd=(
    "${BINARY}"
    --config "${CONFIG}"
    --dataset "${dataset}"
    --gps "${gps}"
    --gps-time-offset 0
    --start-time "${start_time}"
    --until-time "${until_time}"
    --initialization-mode online_multisensor_alignment
    --init-from-fc "${fc_stream}"
    --init-from-fc-position-frame global_gnav
    --online-alignment-release-policy practical_navigation_start
    --yaw-mode baseline
    --gps-alt-update
    --height-mode guarded
    --camera-frame-stride 12
    --diag-csv "${out}/diag.csv"
    --diag-events "${out}/events.txt"
    --output "${out}/traj.txt"
    --output-raw "${out}/traj_raw.txt"
    --output-nav "${out}/traj_nav.txt"
    --online-alignment-metadata-json "${out}/online_alignment_metadata.json"
  )
  if [[ -n "${DISPLAY:-}" ]]; then
    cmd+=(--viz-fast --dash-every 5)
    printf 'visible_dashboard\n' > "${out}/visualization_mode.txt"
  else
    cmd+=(--no-dashboard)
    printf 'display_unavailable_no_dashboard\n' > "${out}/visualization_mode.txt"
  fi
  printf '%q ' "${cmd[@]}" > "${out}/command.txt"
  printf '\n' >> "${out}/command.txt"
  date --iso-8601=seconds > "${out}/process_start.txt"
  /usr/bin/time -v -o "${out}/resource_usage.txt" \
    "${cmd[@]}" > "${out}/stdout.log" 2> "${out}/stderr.log"
  printf '0\n' > "${out}/exit_code.txt"
  date --iso-8601=seconds > "${out}/process_end.txt"
  python3 "${VALIDATOR}" \
    --trace "${out}/online_alignment_sliding_windows.csv" \
    --metadata "${out}/online_alignment_metadata.json" \
    --out "${out}/r1_acceptance.json" \
    > "${out}/r1_acceptance.log"
}

printf 'flight,exit_code\n' > "${ROOT}/batch_exit_codes.csv"
for flight in fly1 fly3; do
  code=0
  run_one "${flight}" || code=$?
  printf '%s,%s\n' "${flight}" "${code}" >> "${ROOT}/batch_exit_codes.csv"
done
printf '%s\n' "${ROOT}" > "${OUTPUT_PARENT}/LATEST.txt"
printf 'P4_R1_ROOT=%s\n' "${ROOT}"
cat "${ROOT}/batch_exit_codes.csv"
awk -F, 'NR > 1 && $2 != 0 { failed=1 } END { exit failed }' \
  "${ROOT}/batch_exit_codes.csv"
