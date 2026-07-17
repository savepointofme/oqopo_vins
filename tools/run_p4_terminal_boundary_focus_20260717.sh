#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
OUTPUT_PARENT="${P4_TERMINAL_BOUNDARY_OUTPUT_PARENT:-/mnt/c/Users/baloney/Desktop/实验目录/P4_terminal_boundary_redesign_20260717/focus_runs}"
FLY2_FC="${P4_R1_FLY2_FC_STREAM:-/mnt/c/Users/baloney/Desktop/实验目录/P4_fly2_direction_validation_20260716_autonomous/fullflight_calibration/p4_inputs/fly2_fc_navigation_online.csv}"
mkdir -p "${OUTPUT_PARENT}"
printf 'flight,exit_code\n' > "${OUTPUT_PARENT}/launcher_exit_codes.csv"

run_flight() {
  local flight="$1"
  local until_time="$2"
  local log="${OUTPUT_PARENT}/${flight}_launcher.log"
  case "${flight}" in
    fly1)
      P4_R1_OUTPUT_PARENT="${OUTPUT_PARENT}" \
      P4_R1_RUN_LABEL="terminal_boundary_${flight}" \
      P4_R1_FLY1_UNTIL="${until_time}" \
        bash "${REPO}/tools/run_p4_sliding_window_r1_short.sh" fly1 \
        > "${log}" 2>&1
      ;;
    fly2)
      P4_R1_OUTPUT_PARENT="${OUTPUT_PARENT}" \
      P4_R1_RUN_LABEL="terminal_boundary_${flight}" \
      P4_R1_FLY2_UNTIL="${until_time}" \
      P4_R1_FLY2_FC_STREAM="${FLY2_FC}" \
        bash "${REPO}/tools/run_p4_sliding_window_r1_short.sh" fly2 \
        > "${log}" 2>&1
      ;;
    fly3)
      P4_R1_OUTPUT_PARENT="${OUTPUT_PARENT}" \
      P4_R1_RUN_LABEL="terminal_boundary_${flight}" \
      P4_R1_FLY3_UNTIL="${until_time}" \
        bash "${REPO}/tools/run_p4_sliding_window_r1_short.sh" fly3 \
        > "${log}" 2>&1
      ;;
  esac
}

run_flight fly1 "${P4_TERMINAL_FLY1_UNTIL:-1151.0}" &
pid_fly1=$!
run_flight fly2 "${P4_TERMINAL_FLY2_UNTIL:-900.0}" &
pid_fly2=$!
run_flight fly3 "${P4_TERMINAL_FLY3_UNTIL:-947.0}" &
pid_fly3=$!

status=0
for entry in "fly1:${pid_fly1}" "fly2:${pid_fly2}" "fly3:${pid_fly3}"; do
  flight="${entry%%:*}"
  pid="${entry##*:}"
  code=0
  wait "${pid}" || code=$?
  printf '%s,%s\n' "${flight}" "${code}" >> "${OUTPUT_PARENT}/launcher_exit_codes.csv"
  if (( code != 0 )); then
    status=1
  fi
done

find "${OUTPUT_PARENT}" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' \
  | sort > "${OUTPUT_PARENT}/focus_run_directories.txt"
exit "${status}"
