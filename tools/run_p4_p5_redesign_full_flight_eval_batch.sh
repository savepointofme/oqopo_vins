#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
analysis_script="${P4_ANALYSIS_SCRIPT:-${repo_root}/analysis/full_flight_error_analysis.py}"
common_segment_root="${P4_COMMON_SEGMENT_ROOT:-/mnt/c/Users/baloney/Desktop/实验目录/_OPENVINS_ORGANIZED_20260625/01_final_locked_stride_sweep/ADAPTIVE_STRIDE_VALIDATION_OFFICIAL_20260621}"
run_root="${1:?usage: $0 RUN_ROOT OUT_ROOT [active|p4_only] [short|full]}"
out_root="${2:?usage: $0 RUN_ROOT OUT_ROOT [active|p4_only] [short|full]}"
mode="${3:-active}"
scope="${4:-full}"

case "${mode}" in
  active)
    method_name="formal finite-window P4 plus continuous active P5"
    ;;
  p4_only)
    method_name="formal finite-window P4 plus fixed stride12"
    ;;
  *)
    printf 'unsupported mode: %s (expected active or p4_only)\n' "${mode}" >&2
    exit 2
    ;;
esac

case "${scope}" in
  short)
    evaluation_window_mode="explicit_or_overlap"
    crop_reason="Registered short replay through initialization, first turn, and post-turn stable segment; no full-flight claim."
    ;;
  full)
    evaluation_window_mode="airborne_auto"
    crop_reason="Automatic airborne full-flight window; post-touchdown ground samples excluded from flight-accuracy metrics."
    ;;
  *)
    printf 'unsupported scope: %s (expected short or full)\n' "${scope}" >&2
    exit 2
    ;;
esac

mkdir -p "${out_root}"
sha256sum "${analysis_script}" >"${out_root}/analysis_tool_sha256.txt"

run_one() {
  local flight="$1"
  local gps="$2"
  local run_dir="${run_root}/${flight}_formal_p4_${mode}"
  if [[ ! -d "${run_dir}" ]]; then
    run_dir="${run_root}/${flight}_persistent_p4_${mode}"
  fi
  local out_dir="${out_root}/${flight}"
  local segment_csv="${common_segment_root}/COMMON_${flight}/COMMON_LAP_SEGMENTS.csv"
  if [[ ! -f "${segment_csv}" ]]; then
    printf 'missing shared lap segmentation: %s\n' "${segment_csv}" >&2
    return 2
  fi
  mkdir -p "${out_dir}"
  python3 "${analysis_script}" \
    --gps "${gps}" \
    --vio-traj "${run_dir}/traj_nav.txt" \
    --vio-bias "${run_dir}/traj.txt.bias" \
    --vio-diag "${run_dir}/diag.csv" \
    --out-dir "${out_dir}" \
    --evaluation-window-mode "${evaluation_window_mode}" \
    --segment-index-csv "${segment_csv}" \
    --alignment-mode absolute_navigation_no_post_alignment \
    --flight-name "${flight}" \
    --method-name "${method_name}" \
    --experiment-id "P4_P5_redesign_20260715_${scope}_${mode}" \
    --source-package "${run_dir}" \
    --experiment-config "${run_dir}/command.txt" \
    --run-status success \
    --crop-reason "${crop_reason}" \
    --notes "P4 output is already in G_nav; primary evaluation applies no post position, heading, or SE(3) alignment." \
    >"${out_dir}/evaluation.log" 2>&1
}

run_one fly1 "/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv" &
pid_fly1=$!
run_one fly3 "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv" &
pid_fly3=$!

status_fly1=0
status_fly3=0
wait "${pid_fly1}" || status_fly1=$?
wait "${pid_fly3}" || status_fly3=$?

printf 'fly1,%s\nfly3,%s\n' "${status_fly1}" "${status_fly3}" >"${out_root}/exit_codes.csv"
if (( status_fly1 != 0 || status_fly3 != 0 )); then
  exit 1
fi
