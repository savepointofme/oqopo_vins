#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_FORMAL_REPO:-/mnt/d/vscode_dir/open_vins_p4_formal_20260718}"
REPO_WINDOWS="${P4_FORMAL_REPO_WINDOWS:-D:/vscode_dir/open_vins_p4_formal_20260718}"
SOURCE_REPO="${P4_SOURCE_REPO:-/mnt/d/vscode_dir/open_vins}"
EXPERIMENT_DIR="${P4_EXPERIMENT_DIR:-/mnt/c/Users/baloney/Desktop/实验目录}"
OUTPUT_PARENT="${P4_FORMAL_OUTPUT_PARENT:-${EXPERIMENT_DIR}/P4_formal_joint_20260718}"
BUILD_DIR="${P4_FORMAL_BUILD_DIR:-${REPO}/build_p4_formal_20260718}"
BINARY="${P4_FORMAL_BINARY:-${BUILD_DIR}/run_serial_msckf_ros_free}"
INITIALIZER_TEST="${P4_FORMAL_INITIALIZER_TEST:-${BUILD_DIR}/test_online_alignment_initializer}"
P5_TEST="${P4_FORMAL_P5_TEST:-${BUILD_DIR}/test_adaptive_stride}"
CONFIG="${P4_FORMAL_CONFIG:-${REPO}/baseline/latest/config/estimator_config.yaml}"
HEIGHT_CONTRACT="${P4_FORMAL_HEIGHT_CONTRACT:-gps_z_guarded}"
FORMAL_VALIDATOR="${REPO}/analysis/validate_p4_formal_run.py"
FRAME_VALIDATOR="${P4_FORMAL_FRAME_VALIDATOR:-${REPO}/analysis/validate_frame_contract.py}"
if [[ ! -f "${FRAME_VALIDATOR}" ]]; then
  FRAME_VALIDATOR="${SOURCE_REPO}/analysis/validate_frame_contract.py"
fi
FC_ROOT="${P4_FORMAL_FC_ROOT:-${SOURCE_REPO}/readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs}"
FLY2_FC="${P4_FORMAL_FLY2_FC:-${EXPERIMENT_DIR}/P4_fly2_direction_validation_20260716_autonomous/fullflight_calibration/p4_inputs/fly2_fc_navigation_online.csv}"

SCOPE="${1:-focus}"
MODE="${2:-p4_only}"
VISUALIZATION="${3:-visible}"
FLIGHT_SET="${4:-all}"
case "${SCOPE}" in focus|medium|full) ;; *) echo "scope must be focus, medium, or full" >&2; exit 2 ;; esac
case "${MODE}" in p4_only|p5_active) ;; *) echo "mode must be p4_only or p5_active" >&2; exit 2 ;; esac
case "${HEIGHT_CONTRACT}" in
  gps_z_guarded|off|agl_single_reset) ;;
  *) echo "P4_FORMAL_HEIGHT_CONTRACT must be gps_z_guarded, off, or agl_single_reset" >&2; exit 2 ;;
esac
case "${VISUALIZATION}" in visible|disabled) ;; *) echo "visualization must be visible or disabled" >&2; exit 2 ;; esac
case "${FLIGHT_SET}" in
  all) flights=(fly1 fly2 fly3) ;;
  fly1|fly2|fly3) flights=("${FLIGHT_SET}") ;;
  *) echo "flight set must be all, fly1, fly2, or fly3" >&2; exit 2 ;;
esac

ROOT="${OUTPUT_PARENT}/${SCOPE}_runs/$(date +%Y%m%d_%H%M%S)_${MODE}_${FLIGHT_SET}"
mkdir -p "${ROOT}"
git.exe -C "${REPO_WINDOWS}" rev-parse HEAD > "${ROOT}/git_head.txt"
git.exe -C "${REPO_WINDOWS}" status --short > "${ROOT}/git_status.txt"
sha256sum "${BINARY}" "${INITIALIZER_TEST}" "${P5_TEST}" "${CONFIG}" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.cpp" \
  "${REPO}/ov_msckf/src/core/p4/factors/Factor_P4FcTrajectory.cpp" \
  "${REPO}/ov_msckf/src/core/p4/factors/Factor_P4Epipolar.cpp" \
  "${REPO}/ov_msckf/src/core/VisualCadencePlanner.h" \
  "${REPO}/ov_msckf/src/core/BackendUpdateTrigger.h" \
  "${REPO}/ov_msckf/src/ros_free/AdaptiveVisualScheduler.h" \
  "${REPO}/ov_msckf/src/run_serial_msckf_ros_free.cpp" \
  > "${ROOT}/implementation_sha256.txt"
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}" \
  > "${ROOT}/display_environment.txt"
"${INITIALIZER_TEST}" > "${ROOT}/preflight_initializer.log" 2>&1
"${P5_TEST}" > "${ROOT}/preflight_p5.log" 2>&1

run_one() {
  local flight="$1"
  local dataset gps fc_stream start_time until_time
  case "${flight}" in
    fly1)
      dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
      gps=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fc_rebuild_20260614/gps_refined_finealign.csv
      fc_stream="${FC_ROOT}/fly1_fc_navigation_online.csv"
      start_time=930.0
      case "${SCOPE}" in
        focus) until_time=1151.0 ;;
        medium) until_time=1550.0 ;;
        full) until_time=1938.0 ;;
      esac
      ;;
    fly2)
      dataset=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722
      gps=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260614/gps_refined_finealign.csv
      fc_stream="${FLY2_FC}"
      start_time=700.0
      case "${SCOPE}" in
        focus) until_time=900.0 ;;
        medium) until_time=1400.0 ;;
        full) until_time=2500.0 ;;
      esac
      ;;
    fly3)
      dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
      gps=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
      fc_stream="${FC_ROOT}/fly3_fc_navigation_online.csv"
      start_time=618.0
      case "${SCOPE}" in
        focus) until_time=947.0 ;;
        medium) until_time=1400.0 ;;
        full) until_time=1837.0 ;;
      esac
      ;;
  esac

  local out="${ROOT}/${flight}"
  mkdir -p "${out}/config_snapshot"
  cp "${CONFIG}" "${out}/config_snapshot/"
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
    --camera-frame-stride 12
    --adaptive-stride-log "${out}/adaptive_visual.csv"
    --camera-stride-audit "${out}/camera_stride_audit.csv"
    --diag-csv "${out}/diag.csv"
    --diag-events "${out}/events.txt"
    --state-safety-diag "${out}/state_safety.csv"
    --state-safety-eig-every 0
    --output "${out}/traj.txt"
    --output-raw "${out}/traj_raw.txt"
    --output-nav "${out}/traj_nav.txt"
    --nav-frame-metadata-json "${out}/nav_frame_metadata.json"
    --online-alignment-metadata-json "${out}/online_alignment_metadata.json"
  )
  case "${HEIGHT_CONTRACT}" in
    gps_z_guarded)
      cmd+=(--gps-alt-update --height-mode guarded --gps-alt-sigma 2.0
        --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10.0
        --gps-alt-max-res 80.0 --gps-alt-guard-dxy 0.5
        --gps-alt-guard-kxy 5.0 --gps-alt-cov-psd-check-interval 1.0
        --gps-alt-nasa-beta 0.2 --gps-alt-nasa-q-threshold 0.0)
      ;;
    agl_single_reset)
      cmd+=(--agl-scene-scale single_reset
        --agl-scene-scale-diag "${out}/agl_scene_scale.csv")
      ;;
    off) ;;
  esac
  if [[ "${MODE}" == p5_active ]]; then
    cmd+=(--adaptive-stride)
  fi
  if [[ "${VISUALIZATION}" == visible && -n "${DISPLAY:-}" ]]; then
    cmd+=(--viz-fast --dash-every 5)
    echo visible_dashboard > "${out}/visualization_mode.txt"
  else
    cmd+=(--no-dashboard)
    echo dashboard_disabled > "${out}/visualization_mode.txt"
  fi
  printf '%q ' "${cmd[@]}" > "${out}/command.txt"
  printf '\n' >> "${out}/command.txt"
  sha256sum "${BINARY}" "${CONFIG}" "${gps}" "${fc_stream}" \
    > "${out}/input_provenance_sha256.txt"
  date --iso-8601=seconds > "${out}/process_start.txt"
  local code=0
  /usr/bin/time -v -o "${out}/resource_usage.txt" \
    "${cmd[@]}" > "${out}/stdout.log" 2> "${out}/stderr.log" || code=$?
  date --iso-8601=seconds > "${out}/process_end.txt"
  printf '%s\n' "${code}" > "${out}/runner_exit_code.txt"
  if (( code != 0 )); then return "${code}"; fi
  [[ -s "${out}/traj_nav.txt" && -s "${out}/online_alignment_metadata.json" ]] || return 66

  local validator_cmd=(python3 "${FORMAL_VALIDATOR}" \
    --metadata "${out}/online_alignment_metadata.json" \
    --trajectory "${out}/traj_nav.txt" \
    --command "${out}/command.txt" \
    --out "${out}/formal_acceptance.json")
  if [[ "${MODE}" == p5_active ]]; then
    validator_cmd+=(--allow-p5-active)
  fi
  "${validator_cmd[@]}" > "${out}/formal_acceptance.log"
  python3 "${FRAME_VALIDATOR}" \
    --raw "${out}/traj_raw.txt" \
    --raw-bias "${out}/traj.txt.bias" \
    --nav "${out}/traj_nav.txt" \
    --metadata "${out}/nav_frame_metadata.json" \
    --out "${out}/frame_contract_validation.json" \
    > "${out}/frame_contract_validation.log"
}

printf 'flight,exit_code\n' > "${ROOT}/batch_exit_codes.csv"
declare -a pids=()
for flight in "${flights[@]}"; do
  run_one "${flight}" > "${ROOT}/${flight}_launcher.log" 2>&1 &
  pids+=("$!")
done
status=0
for index in "${!pids[@]}"; do
  code=0
  wait "${pids[$index]}" || code=$?
  printf '%s,%s\n' "${flights[$index]}" "${code}" >> "${ROOT}/batch_exit_codes.csv"
  (( code == 0 )) || status=1
done
date --iso-8601=seconds > "${ROOT}/batch_end.txt"
printf '%s\n' "${ROOT}" > "${OUTPUT_PARENT}/LATEST_${SCOPE^^}_${MODE^^}.txt"
printf 'P4_FORMAL_ROOT=%s\n' "${ROOT}"
cat "${ROOT}/batch_exit_codes.csv"
exit "${status}"
