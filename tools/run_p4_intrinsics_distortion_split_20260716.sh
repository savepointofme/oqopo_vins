#!/usr/bin/env bash
set -uo pipefail

REPO=/mnt/d/vscode_dir/open_vins_p4_sliding_r1
SCOPE="${P4_KD_SPLIT_SCOPE:-focus}"
ATTEMPT="${P4_KD_SPLIT_ATTEMPT:-}"
SPLIT_ROOT=/mnt/c/Users/baloney/Desktop/实验目录/P4_yaw_calibration_factorial_20260716_ultra/intrinsics_distortion_split
if [[ -n "${ATTEMPT}" ]]; then
  if [[ ! "${ATTEMPT}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    printf 'invalid P4_KD_SPLIT_ATTEMPT: %s\n' "${ATTEMPT}" >&2
    exit 2
  fi
  ROOT="${SPLIT_ROOT}/attempts/${ATTEMPT}"
elif [[ "${SCOPE}" == focus ]]; then
  ROOT="${SPLIT_ROOT}"
elif [[ "${SCOPE}" == full ]]; then
  ROOT="${SPLIT_ROOT}/full_confirmation"
else
  printf 'invalid P4_KD_SPLIT_SCOPE: %s\n' "${SCOPE}" >&2
  exit 2
fi

HELPER="${REPO}/tools/p4_intrinsics_distortion_split_20260716.py"
BINARY="${ROOT}/provenance/frozen_binary/run_serial_msckf_ros_free"
FRAME_VALIDATOR="${ROOT}/provenance/analysis_tool_snapshot/analysis/validate_frame_contract.py"
EXPECTED="${P4_KD_SPLIT_RUNNER_SHA256:-57657e8ed9d034300a0c041481472808fe951d92cdeaac8f740eb0cd570978fe}"
LIBRARY="${ROOT}/provenance/frozen_binary/libov_msckf_lib.so"
EXPECTED_LIBRARY="${P4_KD_SPLIT_LIBRARY_SHA256:-40ac217cfd15319fb15ba705d44d7781e9b1798170020f21416995e228306080}"
RUN_IDS="${P4_KD_SPLIT_RUN_IDS:-}"
SESSION="${P4_KD_SPLIT_SESSION:-batch}"
if [[ ! "${SESSION}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  printf 'invalid P4_KD_SPLIT_SESSION: %s\n' "${SESSION}" >&2
  exit 2
fi

if [[ ! -f "${ROOT}/RUN_MATRIX.csv" ]]; then
  printf 'missing prepared RUN_MATRIX.csv: %s\n' "${ROOT}" >&2
  exit 2
fi
actual="$(sha256sum "${BINARY}" | awk '{print $1}')"
if [[ "${actual}" != "${EXPECTED}" ]]; then
  printf 'frozen runner hash mismatch: %s\n' "${actual}" >&2
  exit 3
fi
if [[ ! -f "${LIBRARY}" ]]; then
  printf 'missing frozen project library: %s\n' "${LIBRARY}" >&2
  exit 4
fi
actual_library="$(sha256sum "${LIBRARY}" | awk '{print $1}')"
if [[ "${actual_library}" != "${EXPECTED_LIBRARY}" ]]; then
  printf 'frozen project library hash mismatch: %s\n' "${actual_library}" >&2
  exit 5
fi
export LD_LIBRARY_PATH="${ROOT}/provenance/frozen_binary"

mkdir -p "${ROOT}/provenance/runtime"
date --iso-8601=seconds >"${ROOT}/provenance/runtime/batch_start_${SESSION}.txt"
sha256sum "${BINARY}" >"${ROOT}/provenance/runtime/frozen_binary_sha256_before_${SESSION}.txt"
sha256sum "${LIBRARY}" >"${ROOT}/provenance/runtime/frozen_project_library_sha256_before_${SESSION}.txt"
ldd "${BINARY}" >"${ROOT}/provenance/runtime/frozen_binary_ldd_${SESSION}.txt"
resolved_library="$(awk '/libov_msckf_lib[.]so/{print $3; exit}' "${ROOT}/provenance/runtime/frozen_binary_ldd_${SESSION}.txt")"
if [[ -z "${resolved_library}" || "$(readlink -f "${resolved_library}")" != "$(readlink -f "${LIBRARY}")" ]]; then
  printf 'runner did not resolve the frozen project library: %s\n' "${resolved_library}" >&2
  exit 6
fi
sha256sum "${HELPER}" "${REPO}/tools/run_p4_intrinsics_distortion_split_20260716.sh" \
  "${REPO}/tools/analyze_p4_intrinsics_distortion_split_20260716.py" \
  >"${ROOT}/provenance/runtime/orchestration_sha256_before_${SESSION}.txt"
{
  for key in PATH LD_LIBRARY_PATH LANG LANGUAGE LC_ALL LC_CTYPE DISPLAY \
    WAYLAND_DISPLAY XDG_RUNTIME_DIR WSL_DISTRO_NAME WSL_INTEROP WSLENV \
    OMP_NUM_THREADS OPENBLAS_NUM_THREADS MKL_NUM_THREADS NUMEXPR_NUM_THREADS \
    PYTHONPATH P4_KD_SPLIT_SCOPE P4_KD_SPLIT_ATTEMPT P4_KD_FULL_CONDITIONS \
    P4_KD_SPLIT_RUN_IDS P4_KD_SPLIT_LIBRARY_SOURCE P4_KD_SPLIT_LIBRARY_SHA256 \
    P4_KD_SPLIT_RUNNER_SOURCE P4_KD_SPLIT_RUNNER_SHA256 \
    P4_KD_SPLIT_PAIR_PROFILE P4_KD_SPLIT_SESSION; do
    printf '%s=%s\n' "${key}" "${!key-}"
  done
} >"${ROOT}/provenance/runtime/environment_allowlisted_${SESSION}.txt"
if [[ ! -f "${ROOT}/batch_wave_status.csv" ]]; then
  printf 'session,wave,run_id,process_exit,frame_exit,finalize_exit\n' >"${ROOT}/batch_wave_status.csv"
fi

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
  printf '%s\n' "${finalize_code}" >"${out}/finalize_exit_code.txt"
  printf '%s,%s,%s,%s,%s,%s\n' \
    "${SESSION}" "${wave}" "${run_id}" "${code}" "${frame_code}" "${finalize_code}" \
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
  python3 "${HELPER}" status \
    >"${ROOT}/provenance/runtime/status_after_${SESSION}_${wave}.log" 2>&1 || overall=1
}

if [[ -n "${RUN_IDS}" ]]; then
  IFS=',' read -r -a requested_run_ids <<<"${RUN_IDS}"
  validated_run_ids=()
  for run_id in "${requested_run_ids[@]}"; do
    run_id="${run_id//[[:space:]]/}"
    if [[ ! "${run_id}" =~ ^(C0|KONLY|DONLY|KD)_fly(1|3)$ ]]; then
      printf 'invalid P4_KD_SPLIT_RUN_IDS member: %s\n' "${run_id}" >&2
      exit 7
    fi
    validated_run_ids+=("${run_id}")
  done
  run_wave selected_gate "${validated_run_ids[@]}"
elif [[ "${SCOPE}" == focus ]]; then
  run_wave baseline C0_fly1 C0_fly3
  run_wave single_factors KONLY_fly1 KONLY_fly3 DONLY_fly1 DONLY_fly3
  run_wave interaction KD_fly1 KD_fly3
else
  raw_conditions="${P4_KD_FULL_CONDITIONS:-}"
  if [[ -z "${raw_conditions}" ]]; then
    printf 'P4_KD_FULL_CONDITIONS is required for full scope\n' >&2
    exit 2
  fi
  IFS=',' read -r -a requested <<<"${raw_conditions}"
  declare -A seen=()
  conditions=(C0)
  seen[C0]=1
  for item in "${requested[@]}"; do
    cid="${item^^}"
    cid="${cid//[[:space:]]/}"
    [[ -z "${cid}" || -n "${seen[${cid}]:-}" ]] && continue
    conditions+=("${cid}")
    seen["${cid}"]=1
  done
  for cid in "${conditions[@]}"; do
    run_wave "full_${cid}" "${cid}_fly1" "${cid}_fly3"
  done
fi

python3 "${HELPER}" status >"${ROOT}/provenance/runtime/final_status_${SESSION}.log" 2>&1 || overall=1
sha256sum "${BINARY}" >"${ROOT}/provenance/runtime/frozen_binary_sha256_after_${SESSION}.txt"
sha256sum "${LIBRARY}" >"${ROOT}/provenance/runtime/frozen_project_library_sha256_after_${SESSION}.txt"
sha256sum "${HELPER}" "${REPO}/tools/run_p4_intrinsics_distortion_split_20260716.sh" \
  "${REPO}/tools/analyze_p4_intrinsics_distortion_split_20260716.py" \
  >"${ROOT}/provenance/runtime/orchestration_sha256_after_${SESSION}.txt"
df -h "${ROOT}" >"${ROOT}/provenance/runtime/disk_after_${SESSION}.txt"
free -h >"${ROOT}/provenance/runtime/memory_after_${SESSION}.txt"
date --iso-8601=seconds >"${ROOT}/provenance/runtime/batch_end_${SESSION}.txt"
exit "${overall}"
