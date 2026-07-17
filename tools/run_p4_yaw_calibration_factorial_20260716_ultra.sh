#!/usr/bin/env bash
set -uo pipefail

ROOT=$'/mnt/c/Users/baloney/Desktop/\u5b9e\u9a8c\u76ee\u5f55/P4_yaw_calibration_factorial_20260716_ultra'
REPO=/mnt/d/vscode_dir/open_vins_p4_sliding_r1
HELPER="${REPO}/tools/p4_yaw_calibration_factorial_20260716_ultra.py"
BINARY="${ROOT}/provenance/frozen_binary/run_serial_msckf_ros_free"
TEST_BINARY="${ROOT}/provenance/frozen_binary/test_online_alignment_initializer"
FRAME_VALIDATOR="${ROOT}/provenance/analysis_tool_snapshot/analysis/validate_frame_contract.py"

if [[ ! -f "${ROOT}/RUN_MATRIX.csv" || ! -f "${ROOT}/RUN_MATRIX.md" ]]; then
  printf 'RUN_MATRIX files are missing; run the prepare step first\n' >&2
  exit 2
fi

mkdir -p "${ROOT}/provenance/runtime"
date --iso-8601=seconds >"${ROOT}/provenance/runtime/batch_start.txt"
df -h "${ROOT}" >"${ROOT}/provenance/runtime/disk_before.txt"
free -h >"${ROOT}/provenance/runtime/memory_before.txt"
nproc >"${ROOT}/provenance/runtime/nproc.txt"
uname -a >"${ROOT}/provenance/runtime/uname.txt"
python3 --version >"${ROOT}/provenance/runtime/python_version.txt" 2>&1

preexisting="${ROOT}/provenance/runtime/preexisting_openvins_processes.txt"
pgrep -af '[r]un_serial_msckf_ros_free' >"${preexisting}" || true
if [[ -s "${preexisting}" ]]; then
  printf 'Existing OpenVINS runner detected; refusing to overlap:\n' >&2
  cat "${preexisting}" >&2
  exit 3
fi

sha256sum "${BINARY}" "${TEST_BINARY}" >"${ROOT}/provenance/runtime/frozen_binary_sha256_before.txt"
preflight=0
"${TEST_BINARY}" >"${ROOT}/provenance/runtime/preflight.log" 2>&1 || preflight=$?
printf '%s\n' "${preflight}" >"${ROOT}/provenance/runtime/preflight_exit_code.txt"
if (( preflight != 0 )); then
  printf 'P4 preflight failed with exit code %s\n' "${preflight}" >&2
  exit "${preflight}"
fi

printf 'wave,run_id,process_exit,frame_exit,finalize_exit\n' >"${ROOT}/batch_wave_status.csv"

run_one() {
  local wave="$1"
  local run_id="$2"
  local out="${ROOT}/runs/${run_id}"
  local code=0
  local frame_code=0
  local finalize_code=0

  python3 "${HELPER}" record-start "${run_id}" \
    >"${out}/record_start.log" 2>&1 || code=$?
  if (( code == 0 )); then
    date --iso-8601=seconds >"${out}/process_start.txt"
    date +%s.%N >"${out}/process_start_epoch_s.txt"
    /usr/bin/time -v -o "${out}/resource_usage.txt" \
      bash "${out}/command.sh" >"${out}/stdout.log" 2>"${out}/stderr.log" || code=$?
    date +%s.%N >"${out}/process_end_epoch_s.txt"
    date --iso-8601=seconds >"${out}/process_end.txt"
  fi
  printf '%s\n' "${code}" >"${out}/exit_code.txt"

  if (( code == 0 )); then
    python3 "${FRAME_VALIDATOR}" \
      --raw "${out}/traj_raw.txt" \
      --raw-bias "${out}/traj.txt.bias" \
      --nav "${out}/traj_nav.txt" \
      --metadata "${out}/nav_frame_metadata.json" \
      --out "${out}/frame_contract_validation.json" \
      >"${out}/frame_contract_validation.log" 2>&1 || frame_code=$?
  else
    frame_code=99
  fi
  printf '%s\n' "${frame_code}" >"${out}/frame_contract_validation_exit_code.txt"
  if [[ -f "${out}/stdout.log" ]]; then
    cp "${out}/stdout.log" "${out}/log.txt"
  fi

  python3 "${HELPER}" finalize "${run_id}" \
    --exit-code "${code}" --frame-code "${frame_code}" \
    >"${out}/finalize.log" 2>&1 || finalize_code=$?
  printf '%s,%s,%s,%s,%s\n' \
    "${wave}" "${run_id}" "${code}" "${frame_code}" "${finalize_code}" \
    >>"${ROOT}/batch_wave_status.csv"

  if (( code != 0 || frame_code != 0 || finalize_code != 0 )); then
    return 1
  fi
  return 0
}

overall=0
run_wave() {
  local wave="$1"
  shift
  local -a pids=()
  local -a ids=("$@")
  local run_id pid code
  printf 'starting wave %s: %s\n' "${wave}" "${ids[*]}"
  for run_id in "${ids[@]}"; do
    run_one "${wave}" "${run_id}" &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do
    code=0
    wait "${pid}" || code=$?
    if (( code != 0 )); then
      overall=1
    fi
  done
  python3 "${HELPER}" status >"${ROOT}/provenance/runtime/status_after_${wave}.log" 2>&1 || overall=1
}

# C0 is intentionally completed and validated before any candidate wave.
run_wave baseline C0_fly1 C0_fly3
run_wave c1_c2 C1_fly1 C1_fly3 C2_fly1 C2_fly3
run_wave c3_c4 C3_fly1 C3_fly3 C4_fly1 C4_fly3
run_wave c5_c6 C5_fly1 C5_fly3 C6_fly1 C6_fly3
run_wave c7 C7_fly1 C7_fly3

python3 "${HELPER}" status >"${ROOT}/provenance/runtime/final_status.log" 2>&1 || overall=1
sha256sum "${BINARY}" "${TEST_BINARY}" >"${ROOT}/provenance/runtime/frozen_binary_sha256_after.txt"
df -h "${ROOT}" >"${ROOT}/provenance/runtime/disk_after.txt"
free -h >"${ROOT}/provenance/runtime/memory_after.txt"
date --iso-8601=seconds >"${ROOT}/provenance/runtime/batch_end.txt"
exit "${overall}"
