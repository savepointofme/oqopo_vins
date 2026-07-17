#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_R1_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
OUTPUT_PARENT="${P4_MOUNT_FOCUS_OUTPUT_PARENT:-/mnt/c/Users/baloney/Desktop/P4_kalibr_mount_injection_20260717/focus_runs_visible_r3}"
NEW_ROOT="${P4_MOUNT_NEW_ROOT:-/mnt/c/Users/baloney/Desktop/P4_kalibr_mount_injection_20260717/inputs}"
FLIGHT="${1:?usage: run_p4_kalibr_mount_focus_three_way_20260717.sh fly1|fly3}"

case "${FLIGHT}" in
  fly1|fly3) ;;
  *) printf 'unsupported flight: %s\n' "${FLIGHT}" >&2; exit 2 ;;
esac

mkdir -p "${OUTPUT_PARENT}"
printf 'variant,flight,exit_code\n' > "${OUTPUT_PARENT}/${FLIGHT}_launcher_exit_codes.csv"

run_job() {
  local variant="$1"
  local input_root="$2"
  local output_root="${OUTPUT_PARENT}/${variant}/${FLIGHT}"
  mkdir -p "${output_root}"
  P4_CALIBRATED_FC_ROOT="${input_root}" \
  P4_R1_OUTPUT_PARENT="${output_root}" \
  P4_R1_RUN_LABEL="${variant}_${FLIGHT}_focus_visible" \
  P4_SKIP_PREFLIGHT="${P4_SKIP_PREFLIGHT:-0}" \
  P4_R1_FLY1_UNTIL="${P4_R1_FLY1_UNTIL:-1151.0}" \
  P4_R1_FLY3_UNTIL="${P4_R1_FLY3_UNTIL:-947.0}" \
    bash "${REPO}/tools/run_p4_sliding_window_r1_short.sh" "${FLIGHT}" \
    > "${output_root}/launcher.log" 2>&1
}

run_job kalibr_full "${NEW_ROOT}/kalibr_full" &
pid_full=$!
run_job kalibr_conservative "${NEW_ROOT}/kalibr_conservative" &
pid_conservative=$!

status=0
for entry in \
  "kalibr_full:${pid_full}" \
  "kalibr_conservative:${pid_conservative}"; do
  variant="${entry%%:*}"
  pid="${entry##*:}"
  code=0
  wait "${pid}" || code=$?
  printf '%s,%s,%s\n' "${variant}" "${FLIGHT}" "${code}" \
    >> "${OUTPUT_PARENT}/${FLIGHT}_launcher_exit_codes.csv"
  if (( code != 0 )); then
    status=1
  fi
done

exit "${status}"
