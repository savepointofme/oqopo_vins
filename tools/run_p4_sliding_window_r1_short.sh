#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
SOURCE_REPO="${P4_SOURCE_REPO:-/mnt/d/vscode_dir/open_vins}"
REPO_WINDOWS="${P4_R1_REPO_WINDOWS:-D:/vscode_dir/open_vins_p4_sliding_r1}"
FC_ROOT="${P4_CALIBRATED_FC_ROOT:-${SOURCE_REPO}/readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs}"
OUTPUT_PARENT="${P4_R1_OUTPUT_PARENT:-/mnt/c/Users/baloney/Desktop/P4_sliding_window_r1_20260714}"
RUN_LABEL="${P4_R1_RUN_LABEL:-short_formal}"
ROOT="${OUTPUT_PARENT}/$(date +%Y%m%d_%H%M%S)_${RUN_LABEL}"
FLIGHT_SET="${1:-both}"
BINARY="${REPO}/build_p4_sliding_r1/run_serial_msckf_ros_free"
TEST_BINARY="${REPO}/build_p4_sliding_r1/test_online_alignment_initializer"
CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"
VALIDATOR="${REPO}/analysis/validate_p4_sliding_window_r1.py"
HEIGHT_MODE="${P4_HEIGHT_MODE:-guarded}"

mkdir -p "${ROOT}"
git.exe -C "${REPO_WINDOWS}" rev-parse HEAD > "${ROOT}/git_head.txt"
git.exe -C "${REPO_WINDOWS}" status --short > "${ROOT}/git_status.txt"
sha256sum "${BINARY}" "${TEST_BINARY}" "${CONFIG}" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.h" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.cpp" \
  "${REPO}/ov_msckf/src/run_serial_msckf_ros_free.cpp" \
  > "${ROOT}/implementation_sha256.txt"
printf 'DISPLAY=%s\nWAYLAND_DISPLAY=%s\n' "${DISPLAY:-}" "${WAYLAND_DISPLAY:-}" \
  > "${ROOT}/display_environment.txt"
if [[ "${P4_SKIP_PREFLIGHT:-0}" == "1" ]]; then
  printf 'skipped: current binary already passed in this experiment batch\n' \
    > "${ROOT}/preflight_initializer_test.log"
else
  "${TEST_BINARY}" > "${ROOT}/preflight_initializer_test.log" 2>&1
fi

run_one() {
  local flight="$1"
  local dataset gps fc_stream start_time until_time
  case "${flight}" in
    fly1)
      dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
      gps="${REPO}/config/d455_fly1/fc_gps_cam_time.csv"
      fc_stream="${P4_R1_FLY1_FC_STREAM:-${FC_ROOT}/fly1_fc_navigation_online.csv}"
      start_time="${P4_R1_FLY1_START:-930.0}"
      until_time="${P4_R1_FLY1_UNTIL:-952.0}"
      ;;
    fly3)
      dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
      gps=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
      fc_stream="${P4_R1_FLY3_FC_STREAM:-${FC_ROOT}/fly3_fc_navigation_online.csv}"
      start_time="${P4_R1_FLY3_START:-618.0}"
      until_time="${P4_R1_FLY3_UNTIL:-640.0}"
      ;;
    fly2)
      dataset=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722
      gps=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260614/gps_refined_finealign.csv
      fc_stream="${P4_R1_FLY2_FC_STREAM:-${FC_ROOT}/fly2_fc_navigation_online.csv}"
      start_time="${P4_R1_FLY2_START:-700.0}"
      until_time="${P4_R1_FLY2_UNTIL:-1100.0}"
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
    --adaptive-stride-log "${out}/p4_visual_cadence.csv"
    --camera-stride-audit "${out}/camera_stride_audit.csv"
    --diag-csv "${out}/diag.csv"
    --diag-events "${out}/events.txt"
    --output "${out}/traj.txt"
    --output-raw "${out}/traj_raw.txt"
    --output-nav "${out}/traj_nav.txt"
    --online-alignment-metadata-json "${out}/online_alignment_metadata.json"
  )
  if [[ "${P4_DIAGNOSTIC_NEVER_ANCHOR:-0}" == "1" ]]; then
    cmd+=(--online-alignment-diagnostic-never-anchor)
  fi
  if [[ "${P4_DIAGNOSTIC_RELEASE_ATTITUDE_FROM_FC:-0}" == "1" ]]; then
    cmd+=(--online-alignment-release-attitude-from-fc)
  fi
  if [[ "${P4_DIAGNOSTIC_GRAPH_Q_FC_PV_ZERO_BIAS:-0}" == "1" ]]; then
    cmd+=(
      --online-alignment-diagnostic-graph-q-fc-pv-zero-bias
      --online-alignment-use-cli-init-covariance
      --init-att-sigma-deg 3
      --init-pos-sigma 0.05
      --init-vel-sigma 5
      --init-bg-sigma 0.003
      --init-ba-sigma 1
    )
  elif [[ "${P4_DIAGNOSTIC_CLI_INIT_COVARIANCE:-0}" == "1" ]]; then
    cmd+=(
      --online-alignment-use-cli-init-covariance
      --init-att-sigma-deg 3
      --init-pos-sigma 0.05
      --init-vel-sigma 5
      --init-bg-sigma 0.003
      --init-ba-sigma 1
    )
  fi
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
  set +e
  /usr/bin/time -v -o "${out}/resource_usage.txt" \
    "${cmd[@]}" > "${out}/stdout.log" 2> "${out}/stderr.log"
  local runner_code=$?
  set -e
  printf '%s\n' "${runner_code}" > "${out}/exit_code.txt"
  date --iso-8601=seconds > "${out}/process_end.txt"
  if (( runner_code != 0 )); then
    return "${runner_code}"
  fi
  if [[ ! -s "${out}/online_alignment_sliding_windows.csv" ||
        ! -s "${out}/online_alignment_metadata.json" ||
        ! -s "${out}/traj_nav.txt" ]]; then
    printf 'required P4 output missing after runner success\n' \
      > "${out}/output_contract_error.txt"
    printf '66\n' > "${out}/exit_code.txt"
    return 66
  fi
  if [[ "${P4_DIAGNOSTIC_NEVER_ANCHOR:-0}" != "1" ]]; then
    python3 "${VALIDATOR}" \
      --trace "${out}/online_alignment_sliding_windows.csv" \
      --metadata "${out}/online_alignment_metadata.json" \
      --trajectory "${out}/traj_nav.txt" \
      --out "${out}/r1_acceptance.json" \
      > "${out}/r1_acceptance.log"
  else
    printf 'diagnostic_no_anchor_not_a_release_candidate\n' \
      > "${out}/r1_acceptance.log"
  fi
}

printf 'flight,exit_code\n' > "${ROOT}/batch_exit_codes.csv"
case "${FLIGHT_SET}" in
  both) flights=(fly1 fly3) ;;
  fly1|fly2|fly3) flights=("${FLIGHT_SET}") ;;
  *) printf 'usage: %s [both|fly1|fly2|fly3]\n' "$0" >&2; exit 2 ;;
esac
for flight in "${flights[@]}"; do
  code=0
  run_one "${flight}" || code=$?
  printf '%s,%s\n' "${flight}" "${code}" >> "${ROOT}/batch_exit_codes.csv"
done
printf '%s\n' "${ROOT}" > "${OUTPUT_PARENT}/LATEST.txt"
printf 'P4_R1_ROOT=%s\n' "${ROOT}"
cat "${ROOT}/batch_exit_codes.csv"
awk -F, 'NR > 1 && $2 != 0 { failed=1 } END { exit failed }' \
  "${ROOT}/batch_exit_codes.csv"
