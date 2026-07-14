#!/usr/bin/env bash
set -u -o pipefail

REPO=/mnt/d/vscode_dir/open_vins
EXPERIMENT_DIR="${P4_EXPERIMENT_DIR:-$'/mnt/c/Users/baloney/Desktop/\u5b9e\u9a8c\u76ee\u5f55'}"
PARENT="${EXPERIMENT_DIR}/P4_P5_redesign_20260713/runs"
SCOPE="${1:-short}"
VISUALIZATION="${2:-visible}"
MODE="${3:-active}"
CALIBRATION_PROFILE="${4:-global_baseline}"
FLIGHT_SET="${5:-both}"
ROOT="${PARENT}/$(date +%Y%m%d_%H%M%S)_${SCOPE}_${VISUALIZATION}_${MODE}_${CALIBRATION_PROFILE}_${FLIGHT_SET}"
BINARY="${REPO}/build_ov_msckf/run_serial_msckf_ros_free"
TEST_BINARY="${REPO}/build_ov_msckf/test_online_alignment_initializer"
FC_ROOT="${P4_CALIBRATED_FC_ROOT:-${EXPERIMENT_DIR}/P4_online_joint_alignment_20260712/inputs}"
CALIBRATION_ROOT="${P4_CALIBRATION_ROOT:-$(dirname "${FC_ROOT}")}"
PROVENANCE_TOOL="${REPO}/tools/provenance/baseline_provenance.py"
FRAME_VALIDATOR="${REPO}/analysis/validate_frame_contract.py"
PYTHON_BIN="${PYTHON_BIN:-python3}"

case "${CALIBRATION_PROFILE}" in
  global_baseline)
    CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"
    IMU_CONFIG="${REPO}/baseline/latest/config/kalibr_imu_chain.yaml"
    IMUCAM_CONFIG="${REPO}/baseline/latest/config/kalibr_imucam_chain.yaml"
    ;;
  june12_ground)
    CONFIG="${REPO}/baseline/clean_p4/config/estimator_config.yaml"
    IMU_CONFIG="${REPO}/baseline/latest/config/kalibr_imu_chain.yaml"
    IMUCAM_CONFIG="${REPO}/baseline/clean_p4/config/kalibr_imucam_chain.yaml"
    ;;
  *)
    printf 'usage: %s [short|full] [visible|disabled] [active|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
    exit 2
    ;;
esac

if [[ "${SCOPE}" != short && "${SCOPE}" != full ]]; then
  printf 'usage: %s [short|full] [visible|disabled] [active|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
  exit 2
fi
if [[ "${VISUALIZATION}" != visible && "${VISUALIZATION}" != disabled ]]; then
  printf 'usage: %s [short|full] [visible|disabled] [active|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
  exit 2
fi
if [[ "${MODE}" != active && "${MODE}" != p4_only ]]; then
  printf 'usage: %s [short|full] [visible|disabled] [active|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
  exit 2
fi
case "${FLIGHT_SET}" in
  both) flights=(fly1 fly3) ;;
  fly1|fly3) flights=("${FLIGHT_SET}") ;;
  *)
    printf 'usage: %s [short|full] [visible|disabled] [active|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
    exit 2
    ;;
esac

mkdir -p "${ROOT}"
printf '%s\n' "${ROOT}" > "${PARENT}/LATEST_${SCOPE^^}_${MODE^^}_${CALIBRATION_PROFILE^^}.txt"
printf '%s\n' "${CALIBRATION_PROFILE}" > "${ROOT}/calibration_profile.txt"
git -C "${REPO}" rev-parse HEAD > "${ROOT}/git_head.txt"
git -C "${REPO}" status --short > "${ROOT}/git_status.txt"
sha256sum \
  "${BINARY}" "${TEST_BINARY}" "${CONFIG}" "${IMU_CONFIG}" "${IMUCAM_CONFIG}" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.h" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.cpp" \
  "${REPO}/ov_msckf/src/core/VisualCadencePlanner.h" \
  "${REPO}/ov_msckf/src/core/AlignmentFrameSelector.h" \
  "${REPO}/ov_msckf/src/core/BackendUpdateTrigger.h" \
  "${REPO}/ov_msckf/src/core/VioManager.cpp" \
  "${REPO}/ov_msckf/src/core/VioManagerHelper.cpp" \
  "${REPO}/ov_msckf/src/ros_free/AdaptiveStrideController.h" \
  "${REPO}/ov_msckf/src/run_serial_msckf_ros_free.cpp" \
  > "${ROOT}/implementation_sha256.txt"
uname -a > "${ROOT}/host_uname.txt"
lscpu > "${ROOT}/host_lscpu.txt"
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" \
  "${WAYLAND_DISPLAY:-}" > "${ROOT}/display_environment.txt"

preflight_code=0
"${TEST_BINARY}" > "${ROOT}/preflight_test_online_alignment_initializer.log" 2>&1 || preflight_code=$?
printf '%s\n' "${preflight_code}" > "${ROOT}/preflight_test_online_alignment_initializer_exit_code.txt"
if [[ "${preflight_code}" -eq 0 ]]; then
  printf 'nochange_baseline\n' > "${ROOT}/baseline_classification.txt"
else
  printf 'nochange_diagnostic_preflight_failed\n' > "${ROOT}/baseline_classification.txt"
fi

run_one() {
  local flight="$1"
  local dataset gps fc_stream start_time until_time
  case "${flight}" in
    fly1)
      dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
      gps="${REPO}/config/d455_fly1/fc_gps_cam_time.csv"
      fc_stream="${FC_ROOT}/fly1_fc_navigation_online.csv"
      start_time=930.0
      if [[ "${SCOPE}" == short ]]; then until_time=1100.0; else until_time=1938.0; fi
      ;;
    fly3)
      dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
      gps=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
      fc_stream="${FC_ROOT}/fly3_fc_navigation_online.csv"
      start_time=618.0
      if [[ "${SCOPE}" == short ]]; then until_time=920.0; else until_time=1837.0; fi
      ;;
    *) return 2 ;;
  esac

  local out="${ROOT}/${flight}_persistent_p4_${MODE}"
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
    --gps-alt-sigma 2.0
    --gps-alt-min-pzz 0.01
    --gps-alt-min-t-after-init 10
    --gps-alt-max-res 80
    --gps-alt-guard-dxy 0.5
    --gps-alt-guard-kxy 5.0
    --camera-frame-stride 12
    --adaptive-stride-log "${out}/adaptive_stride.csv"
    --visual-flow-curl-diag "${out}/visual_flow_curl_diag.csv"
    --visual-residual-frame-summary "${out}/visual_residual_frame_summary.csv"
    --camera-stride-audit "${out}/stride_audit.csv"
    --state-safety-diag "${out}/state_safety.csv"
    --state-safety-eig-every 100
    --diag-csv "${out}/diag.csv"
    --diag-events "${out}/events.txt"
    --output "${out}/traj.txt"
    --output-raw "${out}/traj_raw.txt"
    --output-nav "${out}/traj_nav.txt"
    --nav-frame-metadata-json "${out}/nav_frame_metadata.json"
    --online-alignment-metadata-json "${out}/online_alignment_metadata.json"
  )
  if [[ "${MODE}" == active ]]; then
    cmd+=(--adaptive-stride)
  fi
  if [[ "${VISUALIZATION}" == visible && -n "${DISPLAY:-}" ]]; then
    cmd+=(--viz-fast --dash-every 5)
    printf 'visible_dashboard\n' > "${out}/visualization_mode.txt"
  else
    cmd+=(--no-dashboard)
    if [[ "${VISUALIZATION}" == disabled ]]; then
      printf 'dashboard_explicitly_disabled\n' > "${out}/visualization_mode.txt"
    else
      printf 'display_unavailable_no_dashboard\n' > "${out}/visualization_mode.txt"
    fi
  fi
  printf '%q ' "${cmd[@]}" > "${out}/command.txt"
  printf '\n' >> "${out}/command.txt"
  mkdir -p "${out}/config_snapshot"
  cp "${CONFIG}" "${IMU_CONFIG}" "${IMUCAM_CONFIG}" "${out}/config_snapshot/"
  local calibration_yaml="${CALIBRATION_ROOT}/full_flight_fc_board_calibration.yaml"
  local provenance_configs=("${CONFIG}" "${IMU_CONFIG}" "${IMUCAM_CONFIG}")
  if [[ -f "${calibration_yaml}" ]]; then
    cp "${calibration_yaml}" "${out}/config_snapshot/"
    provenance_configs+=("${calibration_yaml}")
  fi
  printf -v command_string '%q ' "${cmd[@]}"
  local provenance_cmd=(
    "${PYTHON_BIN}" "${PROVENANCE_TOOL}" capture
    --repo "${REPO}"
    --out "${out}/provenance"
    --runner "${BINARY}"
    --dataset "${dataset}"
    --gps "${gps}"
    --fc-init "${fc_stream}"
    --generated-root "${REPO}/result"
    --generated-root "${REPO}/reports"
    --command "${command_string}"
  )
  local config_input
  for config_input in "${provenance_configs[@]}"; do
    provenance_cmd+=(--config "${config_input}")
  done
  "${provenance_cmd[@]}" > "${out}/provenance_capture.log" 2>&1
  local provenance_code=$?
  if [[ "${provenance_code}" -ne 0 ]]; then
    printf '%s\n' "${provenance_code}" > "${out}/provenance_capture_exit_code.txt"
    return "${provenance_code}"
  fi
  date --iso-8601=seconds > "${out}/process_start.txt"
  date +%s.%N > "${out}/process_start_epoch_s.txt"
  /usr/bin/time -v -o "${out}/resource_usage.txt" \
    "${cmd[@]}" > "${out}/stdout.log" 2> "${out}/stderr.log"
  local code=$?
  date +%s.%N > "${out}/process_end_epoch_s.txt"
  date --iso-8601=seconds > "${out}/process_end.txt"
  printf '%s\n' "${code}" > "${out}/exit_code.txt"
  local frame_code=0
  "${PYTHON_BIN}" "${FRAME_VALIDATOR}" \
    --raw "${out}/traj_raw.txt" \
    --raw-bias "${out}/traj.txt.bias" \
    --nav "${out}/traj_nav.txt" \
    --metadata "${out}/nav_frame_metadata.json" \
    --out "${out}/frame_contract_validation.json" \
    > "${out}/frame_contract_validation.log" 2>&1 || frame_code=$?
  printf '%s\n' "${frame_code}" > "${out}/frame_contract_validation_exit_code.txt"

  local finalize_code=0
  "${PYTHON_BIN}" "${PROVENANCE_TOOL}" finalize \
    --run-dir "${out}" \
    --repo "${REPO}" \
    --runner-exit-code "${code}" \
    --tee-exit-code 0 \
    --frame-validation-exit-code "${frame_code}" \
    --frame-validation-tee-exit-code 0 \
    > "${out}/provenance_finalize.log" 2>&1 || finalize_code=$?
  printf '%s\n' "${finalize_code}" > "${out}/provenance_finalize_exit_code.txt"
  if [[ "${code}" -ne 0 ]]; then return "${code}"; fi
  if [[ "${frame_code}" -ne 0 ]]; then return "${frame_code}"; fi
  return "${finalize_code}"
}

declare -a pids=()
status=0
printf 'flight,exit_code\n' > "${ROOT}/batch_exit_codes.csv"
if [[ "${VISUALIZATION}" == visible ]]; then
  # WSLg/OpenCV windows are interactive only when a single replay owns the
  # event loop. Run visible flights sequentially; non-visual batches remain
  # parallel for throughput.
  for flight in "${flights[@]}"; do
    code=0
    run_one "${flight}" || code=$?
    printf '%s,%s\n' "${flight}" "${code}" >> "${ROOT}/batch_exit_codes.csv"
    if [[ "${code}" -ne 0 ]]; then status=1; fi
  done
else
  for flight in "${flights[@]}"; do
    run_one "${flight}" &
    pids+=("$!")
  done
  for index in "${!pids[@]}"; do
    code=0
    wait "${pids[$index]}" || code=$?
    printf '%s,%s\n' "${flights[$index]}" "${code}" \
      >> "${ROOT}/batch_exit_codes.csv"
    if [[ "${code}" -ne 0 ]]; then status=1; fi
  done
fi
date --iso-8601=seconds > "${ROOT}/batch_end.txt"
printf 'P4_P5_REDESIGN_ROOT=%s\n' "${ROOT}"
cat "${ROOT}/batch_exit_codes.csv"
exit "${status}"
