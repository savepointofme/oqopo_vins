#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_REPO:-/mnt/d/vscode_dir/open_vins}"
if [[ -n "${P4_GIT_DIR:-}" ]]; then
  export GIT_DIR="${P4_GIT_DIR}"
  export GIT_WORK_TREE="${REPO}"
  # The linked worktree was created by Windows Git. Match its checkout EOL
  # semantics so WSL provenance does not report every CRLF file as modified.
  export GIT_CONFIG_COUNT=1
  export GIT_CONFIG_KEY_0=core.autocrlf
  export GIT_CONFIG_VALUE_0=true
fi
EXPERIMENT_DIR="${P4_EXPERIMENT_DIR:-$'/mnt/c/Users/baloney/Desktop/\u5b9e\u9a8c\u76ee\u5f55'}"
PARENT="${EXPERIMENT_DIR}/P4_P5_redesign_20260713/runs"
SCOPE="${1:-short}"
VISUALIZATION="${2:-visible}"
MODE="${3:-active}"
CALIBRATION_PROFILE="${4:-global_baseline}"
FLIGHT_SET="${5:-both}"
ROOT="${PARENT}/$(date +%Y%m%d_%H%M%S)_${SCOPE}_${VISUALIZATION}_${MODE}_${CALIBRATION_PROFILE}_${FLIGHT_SET}"
BINARY="${P4_BINARY:-${REPO}/build_ov_msckf/run_serial_msckf_ros_free}"
TEST_BINARY="${P4_TEST_BINARY:-${REPO}/build_ov_msckf/test_online_alignment_initializer}"
FC_ROOT="${P4_CALIBRATED_FC_ROOT:-${EXPERIMENT_DIR}/P4_online_joint_alignment_20260712/inputs}"
CALIBRATION_ROOT="${P4_CALIBRATION_ROOT:-$(dirname "${FC_ROOT}")}"
FRAME_VALIDATOR="${P4_FRAME_VALIDATOR:-${REPO}/analysis/validate_frame_contract.py}"
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
    printf 'usage: %s [short|full] [visible|disabled] [active|shadow|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
    exit 2
    ;;
esac

if [[ "${SCOPE}" != short && "${SCOPE}" != full ]]; then
  printf 'usage: %s [short|full] [visible|disabled] [active|shadow|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
  exit 2
fi
if [[ "${VISUALIZATION}" != visible && "${VISUALIZATION}" != disabled ]]; then
  printf 'usage: %s [short|full] [visible|disabled] [active|shadow|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
  exit 2
fi
if [[ "${MODE}" != active && "${MODE}" != p4_only && "${MODE}" != shadow ]]; then
  printf 'usage: %s [short|full] [visible|disabled] [active|shadow|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
  exit 2
fi
case "${FLIGHT_SET}" in
  both) flights=(fly1 fly3) ;;
  fly1|fly3) flights=("${FLIGHT_SET}") ;;
  *)
    printf 'usage: %s [short|full] [visible|disabled] [active|shadow|p4_only] [global_baseline|june12_ground] [both|fly1|fly3]\n' "$0" >&2
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
  "${REPO}/ov_msckf/src/core/OnlineAlignmentCandidateFilter.h" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentCandidateFilter.cpp" \
  "${REPO}/ov_msckf/src/core/VisualCadencePlanner.h" \
  "${REPO}/ov_msckf/src/core/AlignmentFrameSelector.h" \
  "${REPO}/ov_msckf/src/core/BackendUpdateTrigger.h" \
  "${REPO}/ov_msckf/src/core/VioManager.cpp" \
  "${REPO}/ov_msckf/src/core/VioManagerHelper.cpp" \
  "${REPO}/ov_msckf/src/ros_free/AdaptiveVisualScheduler.h" \
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
  printf 'current_source_preflight_passed\n' > "${ROOT}/baseline_classification.txt"
else
  printf 'current_source_preflight_failed\n' > "${ROOT}/baseline_classification.txt"
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

  local out="${ROOT}/${flight}_formal_p4_${MODE}"
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
  elif [[ "${MODE}" == shadow ]]; then
    cmd+=(--adaptive-stride-shadow)
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
  # Lightweight run identity only. The former deep provenance helper hashed
  # every image and generated a full dirty-worktree binary patch before each
  # replay; neither operation changes estimator evidence and both dominated
  # runtime. Hash exact executable/config/navigation inputs and all non-image
  # dataset files, while recording only an inventory digest for image payloads.
  {
    sha256sum "${BINARY}" "${TEST_BINARY}" "${gps}" "${fc_stream}"
    local config_input
    for config_input in "${provenance_configs[@]}"; do
      sha256sum "${config_input}"
    done
    while IFS= read -r -d '' input_file; do
      sha256sum "${input_file}"
    done < <(find "${dataset}" \
      -path "${dataset}/cam0/data" -prune -o -type f -print0 | sort -z)
  } > "${out}/input_provenance_sha256.txt"
  # Do not traverse the image payload directory. On drvfs, enumerating tens of
  # thousands of PNG files can cost more wall time than the evidence it adds.
  printf 'lightweight_no_image_inventory_no_git_patch\n' \
    > "${out}/provenance_mode.txt"
  date --iso-8601=seconds > "${out}/process_start.txt"
  date +%s.%N > "${out}/process_start_epoch_s.txt"
  local code=0
  /usr/bin/time -v -o "${out}/resource_usage.txt" \
    "${cmd[@]}" > "${out}/stdout.log" 2> "${out}/stderr.log" || code=$?
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

  # Keep the canonical run-artifact names and explicit feature state.
  cp "${out}/command.txt" "${out}/command.sh"
  cp "${out}/stdout.log" "${out}/log.txt"
  {
    printf 'stride=12\n'
    printf 'baseline_profile=%s\n' "${CALIBRATION_PROFILE}"
    printf 'allow_experimental=false\n'
    if [[ "${MODE}" == active ]]; then
      printf 'adaptive_stride_active=true\n'
      printf 'adaptive_stride_shadow=false\n'
    elif [[ "${MODE}" == shadow ]]; then
      printf 'adaptive_stride_active=false\n'
      printf 'adaptive_stride_shadow=true\n'
    else
      printf 'adaptive_stride_active=false\n'
      printf 'adaptive_stride_shadow=false\n'
    fi
    printf 'camera_frame_adaptive_active=false\n'
    printf 'visual_update_adaptive_active=false\n'
    printf 'pose_repair_sim_gps_active=false\n'
    printf 'restart_supervisor_active=false\n'
    printf 'restart_on_pose_repair_active=false\n'
  } > "${out}/run_metadata.txt"

  printf 'deep provenance finalize disabled; lightweight hashes are stored in the run directory\n' \
    > "${ROOT}/${flight}_provenance_finalize.log"
  printf '0\n' > "${ROOT}/${flight}_provenance_finalize_exit_code.txt"
  if [[ "${code}" -ne 0 ]]; then return "${code}"; fi
  if [[ "${frame_code}" -ne 0 ]]; then return "${frame_code}"; fi
  return 0
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
