#!/usr/bin/env bash
set -u -o pipefail

REPO="/mnt/d/vscode_dir/open_vins"
EXPERIMENT_DIR=$'/mnt/c/Users/baloney/Desktop/\u5b9e\u9a8c\u76ee\u5f55'
DELIVERY_ROOT="${EXPERIMENT_DIR}/P4_P5_mainline_20260712"
STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_ROOT="${DELIVERY_ROOT}/runs/${STAMP}_fly1_current_source_full_airborne"
BINARY="${REPO}/build_ov_msckf/run_serial_msckf_ros_free"
CONFIG="${REPO}/baseline/clean_p4/config/estimator_config.yaml"
DATASET="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"
GPS="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fc_rebuild_20260614/gps_refined_finealign.csv"
LEGACY_FC="${EXPERIMENT_DIR}/P4_clean_baseline_20260712/inputs/fly1_fc_init_series_global.csv"
ONLINE_FC="${EXPERIMENT_DIR}/P4_online_joint_alignment_20260712/inputs/fly1_fc_navigation_online.csv"

mkdir -p "${BATCH_ROOT}"
printf '%s\n' "${BATCH_ROOT}" > "${DELIVERY_ROOT}/LATEST_P4_FULL_BATCH.txt"

git -C "${REPO}" rev-parse HEAD > "${BATCH_ROOT}/git_head.txt"
git -C "${REPO}" status --short > "${BATCH_ROOT}/git_status.txt"
sha256sum "${BINARY}" "${CONFIG}" "${LEGACY_FC}" "${ONLINE_FC}" > "${BATCH_ROOT}/input_sha256.txt"
cp "${REPO}/docs/official/P4_P5_MAINLINE_CONTRACT_20260712.md" "${BATCH_ROOT}/EXPERIMENT_CONTRACT.md"

common=(
  "${BINARY}"
  --config "${CONFIG}"
  --dataset "${DATASET}"
  --gps "${GPS}"
  --gps-time-offset 0
  --start-time 930.0
  --init-from-fc-position-frame global_gnav
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
  --viz-fast
  --dash-every 5
)

run_one() {
  local name="$1"
  shift
  local out="${BATCH_ROOT}/${name}"
  mkdir -p "${out}"
  local cmd=(
    "${common[@]}"
    "$@"
    --diag-csv "${out}/diag.csv"
    --diag-events "${out}/events.txt"
    --camera-stride-audit "${out}/stride_audit.csv"
    --state-safety-diag "${out}/state_safety.csv"
    --state-safety-eig-every 100
    --output "${out}/traj.txt"
    --output-raw "${out}/traj_raw.txt"
    --output-nav "${out}/traj_nav.txt"
    --nav-frame-metadata-json "${out}/nav_frame_metadata.json"
    --canonical-init-state-json "${out}/canonical_init_state.json"
    --online-alignment-metadata-json "${out}/online_alignment_result.json"
  )
  printf '%q ' "${cmd[@]}" > "${out}/command.txt"
  printf '\n' >> "${out}/command.txt"
  date --iso-8601=seconds > "${out}/process_start.txt"
  "${cmd[@]}" > "${out}/stdout.log" 2> "${out}/stderr.log"
  local code=$?
  printf '%s\n' "${code}" > "${out}/exit_code.txt"
  date --iso-8601=seconds > "${out}/process_end.txt"
  return "${code}"
}

run_one legacy_nearest_fc_row \
  --initialization-mode fc_full_state \
  --init-from-fc "${LEGACY_FC}" \
  --fc-init-level I0 \
  --fc-init-fallback fail_closed \
  --init-from-fc-max-dt 0.25 \
  --init-from-fc-max-bracket-gap 0.35 \
  --init-att-sigma-deg 3.0 \
  --init-vel-sigma 5.0 \
  --init-pos-sigma 5.0 \
  --init-bg-sigma 0.003 \
  --init-ba-sigma 1.0 &
legacy_pid=$!

run_one online_joint_monocular \
  --initialization-mode online_multisensor_alignment \
  --init-from-fc "${ONLINE_FC}" \
  --online-alignment-release-policy practical_navigation_start &
online_pid=$!

legacy_code=0
online_code=0
wait "${legacy_pid}" || legacy_code=$?
wait "${online_pid}" || online_code=$?

printf 'legacy_nearest_fc_row,%s\nonline_joint_monocular,%s\n' \
  "${legacy_code}" "${online_code}" > "${BATCH_ROOT}/batch_exit_codes.csv"
date --iso-8601=seconds > "${BATCH_ROOT}/batch_end.txt"
printf 'BATCH_ROOT=%s\n' "${BATCH_ROOT}"
printf 'legacy_nearest_fc_row=%s online_joint_monocular=%s\n' "${legacy_code}" "${online_code}"

if [[ "${legacy_code}" -ne 0 || "${online_code}" -ne 0 ]]; then
  exit 1
fi
