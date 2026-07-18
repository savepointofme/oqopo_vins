#!/usr/bin/env bash
set -euo pipefail

repo="${P4_FORMAL_REPO:-/mnt/d/vscode_dir/open_vins_p4_formal_20260718}"
experiment_root="${P4_EXPERIMENT_ROOT:-/mnt/c/Users/baloney/Desktop/实验目录}"
output_root="${P4_DIRECT_FC_DIAG_ROOT:?P4_DIRECT_FC_DIAG_ROOT is required}"
binary="${repo}/build_p4_formal_20260718/run_serial_msckf_ros_free"
frozen_root="${experiment_root}/_OPENVINS_ORGANIZED_20260625/01_final_locked_stride_sweep/stable_config_stride_validation_20260622/runs"

run_one() {
  local flight="$1"
  local start_time until_time dataset gps init_state
  case "${flight}" in
    fly1)
      start_time=930
      until_time=936
      dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
      gps=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fc_rebuild_20260614/gps_refined_finealign.csv
      init_state=/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_init_state_930.csv
      ;;
    fly2)
      start_time=700
      until_time=706
      dataset=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722
      gps=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260614/gps_refined_finealign.csv
      init_state=/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv
      ;;
    fly3)
      start_time=618
      until_time=624
      dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
      gps=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
      init_state=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv
      ;;
  esac

  local output="${output_root}/${flight}"
  local config="${frozen_root}/${flight}_oc_stride12/config/config_run.yaml"
  mkdir -p "${output}"
  local command=(
    "${binary}"
    --config "${config}"
    --dataset "${dataset}"
    --gps "${gps}"
    --gps-time-offset 0
    --start-time "${start_time}"
    --until-time "${until_time}"
    --initialization-mode fc_attitude_only
    --init-from-fc "${init_state}"
    --init-from-fc-position-frame local_w0_seed
    --init-bg-sigma 0.003
    --vio-yaw-update-mode global_yaw_oc_projection
    --vio-global-yaw-oc-alpha 1.0
    --init-att-sigma-deg 3.0
    --init-pos-sigma 0.05
    --gps-alt-update
    --gps-alt-sigma 2.0
    --gps-alt-min-pzz 0.01
    --gps-alt-min-t-after-init 10
    --gps-alt-max-res 80
    --gps-alt-guard-dxy 0.5
    --gps-alt-guard-kxy 5.0
    --gps-alt-cov-psd-check-interval 1.0
    --gps-alt-coupled-mode nasa_lean
    --gps-alt-nasa-beta 0.2
    --gps-alt-nasa-q-threshold 0.0
    --camera-frame-stride 12
    --state-safety-diag "${output}/state_safety.csv"
    --state-safety-eig-every 1
    --output "${output}/traj.txt"
    --no-dashboard
  )
  printf '%q ' "${command[@]}" > "${output}/command.txt"
  printf '\n' >> "${output}/command.txt"
  "${command[@]}" > "${output}/stdout.log" 2> "${output}/stderr.log"
}

mkdir -p "${output_root}"
pids=()
for flight in fly1 fly2 fly3; do
  run_one "${flight}" &
  pids+=("$!")
done
status=0
for pid in "${pids[@]}"; do
  wait "${pid}" || status=1
done
exit "${status}"
