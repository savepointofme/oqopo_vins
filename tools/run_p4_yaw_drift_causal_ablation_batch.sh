#!/usr/bin/env bash
set -euo pipefail

REPO="${P4_REPO:-/mnt/d/vscode_dir/open_vins_p4_sliding_r1}"
if [[ -n "${P4_GIT_DIR:-}" ]]; then
  export GIT_DIR="${P4_GIT_DIR}"
  export GIT_WORK_TREE="${REPO}"
  export GIT_CONFIG_COUNT=1
  export GIT_CONFIG_KEY_0=core.autocrlf
  export GIT_CONFIG_VALUE_0=true
fi
EXPERIMENT_DIR="${P4_EXPERIMENT_DIR:-$'/mnt/c/Users/baloney/Desktop/\u5b9e\u9a8c\u76ee\u5f55'}"
PARENT="${EXPERIMENT_DIR}/P4_yaw_drift_causal_ablation_20260715"
BINARY="${P4_BINARY:-${REPO}/build_p4_sliding_r1/run_serial_msckf_ros_free}"
TEST_BINARY="${P4_TEST_BINARY:-${REPO}/build_p4_sliding_r1/test_online_alignment_initializer}"
FC_ROOT="${P4_CALIBRATED_FC_ROOT:-/mnt/d/vscode_dir/open_vins/readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs}"
CONFIG="${REPO}/baseline/latest/config/estimator_config.yaml"
FRAME_VALIDATOR="${P4_FRAME_VALIDATOR:-/mnt/d/vscode_dir/open_vins/analysis/validate_frame_contract.py}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

SCOPE="${1:-focus}"
VARIANT="${2:-control}"
FLIGHT_SET="${3:-both}"
VISUALIZATION="${4:-disabled}"

case "${SCOPE}" in screen|focus|full) ;; *) printf 'scope must be screen, focus, or full\n' >&2; exit 2 ;; esac
case "${VARIANT}" in control|retain_bias|retain_bg|retain_ba|yaw_fej|no_gpsz|gpsz_nasa_lean|mechanism_control|first_error_deep_diag|first_turn_deep_diag|first_error_msckf_yaw_freeze|first_error_all_visual_yaw_gain_zero|no_visual_yaw|no_visual_bgz|gyro_z_plus|gyro_z_minus|gyro_z_observed_plus|gyro_z_observed_minus|gyro_z_scale_up|gyro_z_scale_down|gyro_z_turn_scale_up|gyro_z_turn_scale_down|no_slam|legacy_init_covariance|targeted_slam_freeze|first_error_slam_freeze|gpsz_guard_pz_fallback|fc_attitude_release|fc_yaw_aid|cam_toff_converged|cam_toff_opposite|cam_extrinsic_rot_plus|cam_extrinsic_rot_minus|cam_extrinsic_x_plus|cam_extrinsic_x_minus|cam_extrinsic_y_plus|cam_extrinsic_y_minus|cam_extrinsic_z_plus|cam_extrinsic_z_minus|centered_yaw_oc|local_estimator_origin|full_4d_oc|post_full_4d_oc|adaptive_stride_active|full_rate_post_init|visual_roi_left|visual_roi_center|visual_roi_right|visual_roi_top|visual_roi_bottom|visual_roi_dynamic_turn|agl_scale_shadow|agl_scale_single) ;; *) printf 'unknown variant: %s\n' "${VARIANT}" >&2; exit 2 ;; esac
case "${FLIGHT_SET}" in both) flights=(fly1 fly3) ;; fly1|fly3) flights=("${FLIGHT_SET}") ;; *) exit 2 ;; esac
case "${VISUALIZATION}" in visible|disabled) ;; *) exit 2 ;; esac

ROOT="${PARENT}/$(date +%Y%m%d_%H%M%S)_${SCOPE}_${VARIANT}_${FLIGHT_SET}"
mkdir -p "${ROOT}"
printf '%s\n' "${ROOT}" > "${PARENT}/LATEST_${SCOPE^^}_${VARIANT^^}.txt"
git -C "${REPO}" rev-parse HEAD > "${ROOT}/git_head.txt"
git -C "${REPO}" status --short > "${ROOT}/git_status.txt"
sha256sum "${BINARY}" "${TEST_BINARY}" "${CONFIG}" \
  "${REPO}/ov_msckf/src/run_serial_msckf_ros_free.cpp" \
  "${REPO}/ov_msckf/src/core/VioManager.h" \
  "${REPO}/ov_msckf/src/core/VioManagerHelper.cpp" \
  "${REPO}/ov_msckf/src/core/AglSceneScaleController.h" \
  "${REPO}/ov_msckf/src/core/AglSceneScaleGroundEstimator.h" \
  "${REPO}/ov_msckf/src/state/StateHelper.cpp" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentInitializer.cpp" \
  "${REPO}/ov_msckf/src/core/OnlineAlignmentCandidateFilter.cpp" \
  > "${ROOT}/implementation_sha256.txt"
printf 'scope=%s\nvariant=%s\nflight_set=%s\nvisualization=%s\n' \
  "${SCOPE}" "${VARIANT}" "${FLIGHT_SET}" "${VISUALIZATION}" > "${ROOT}/experiment_contract.txt"

preflight=0
"${TEST_BINARY}" > "${ROOT}/preflight.log" 2>&1 || preflight=$?
printf '%s\n' "${preflight}" > "${ROOT}/preflight_exit_code.txt"
(( preflight == 0 )) || exit "${preflight}"

run_one() {
  local flight="$1" dataset gps fc_stream start_time until_time
  case "${flight}" in
    fly1)
      dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
      gps="${REPO}/config/d455_fly1/fc_gps_cam_time.csv"
      fc_stream="${FC_ROOT}/fly1_fc_navigation_online.csv"
      start_time=930.0
      if [[ "${SCOPE}" == screen ]]; then
        until_time=1151.0
      elif [[ "${SCOPE}" == focus ]]; then
        until_time=1300.0
      else
        until_time=1938.0
      fi
      ;;
    fly3)
      dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
      gps=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
      fc_stream="${FC_ROOT}/fly3_fc_navigation_online.csv"
      start_time=618.0
      if [[ "${SCOPE}" == screen ]]; then
        until_time=947.0
      elif [[ "${SCOPE}" == focus ]]; then
        until_time=1150.0
      else
        until_time=1837.0
      fi
      ;;
  esac

  local out="${ROOT}/${flight}_${VARIANT}"
  mkdir -p "${out}"
  local yaw_mode=baseline
  [[ "${VARIANT}" == yaw_fej ]] && yaw_mode=fej
  local camera_stride=12
  local cmd=(
    "${BINARY}" --config "${CONFIG}" --dataset "${dataset}"
    --gps "${gps}" --gps-time-offset 0 --start-time "${start_time}"
    --until-time "${until_time}"
    --initialization-mode online_multisensor_alignment
    --init-from-fc "${fc_stream}" --init-from-fc-position-frame global_gnav
    --online-alignment-release-policy practical_navigation_start
    --camera-frame-stride "${camera_stride}"
    --adaptive-stride-log "${out}/adaptive_stride.csv"
    --visual-flow-curl-diag "${out}/visual_flow_curl_diag.csv"
    --visual-residual-frame-summary "${out}/visual_residual_frame_summary.csv"
    --camera-stride-audit "${out}/stride_audit.csv"
    --state-safety-diag "${out}/state_safety.csv" --state-safety-eig-every 100
    --diag-csv "${out}/diag.csv" --diag-events "${out}/events.txt"
    --output "${out}/traj.txt" --output-raw "${out}/traj_raw.txt"
    --output-nav "${out}/traj_nav.txt"
    --nav-frame-metadata-json "${out}/nav_frame_metadata.json"
    --online-alignment-metadata-json "${out}/online_alignment_metadata.json"
  )
  if [[ "${VARIANT}" == no_visual_yaw ]]; then
    cmd+=(--no-vio-yaw-update)
  elif [[ "${VARIANT}" == centered_yaw_oc ]]; then
    cmd+=(--vio-yaw-update-mode global_yaw_oc_centered_projection
      --vio-global-yaw-oc-alpha 1.0)
  elif [[ "${VARIANT}" == full_4d_oc ]]; then
    cmd+=(--vio-yaw-update-mode visual_4d_oc_fej_prechi2)
  elif [[ "${VARIANT}" == post_full_4d_oc ]]; then
    cmd+=(--vio-yaw-update-mode global_4d_oc_projection
      --vio-global-yaw-oc-alpha 1.0)
  else
    cmd+=(--yaw-mode "${yaw_mode}")
  fi
  if [[ "${VARIANT}" == no_visual_bgz ]]; then
    cmd+=(--visual-bgz-update-scale 0.0)
  fi
  case "${VARIANT}" in
    first_error_deep_diag|first_turn_deep_diag|first_error_msckf_yaw_freeze|gyro_z_scale_up|gyro_z_scale_down|gyro_z_turn_scale_up|gyro_z_turn_scale_down)
      if [[ "${flight}" == fly1 ]]; then
        cmd+=(--mech-diag-t0 940.5 --mech-diag-t1 1089.793)
      elif [[ "${VARIANT}" == first_turn_deep_diag ]]; then
        cmd+=(--mech-diag-t0 693.2 --mech-diag-t1 740.0)
      elif [[ "${VARIANT}" == first_error_deep_diag ]]; then
        cmd+=(--mech-diag-t0 740.0 --mech-diag-t1 897.0)
      else
        cmd+=(--mech-diag-t0 693.2 --mech-diag-t1 897.0)
      fi
      if [[ "${VARIANT}" == first_error_deep_diag ||
            "${VARIANT}" == first_turn_deep_diag ]]; then
        cmd+=(
          --slam-ekf-leverage-diag "${out}/slam_ekf_leverage_diag.csv"
          --slam-stacked-ekf-diag "${out}/slam_stacked_ekf_diag.csv"
          --slam-landmark-metadata-diag "${out}/slam_landmark_metadata_diag.csv"
        )
      fi
      if [[ "${VARIANT}" == gyro_z_scale_up ]]; then
        cmd+=(--post-alignment-gyro-z-scale 1.0186)
      elif [[ "${VARIANT}" == gyro_z_scale_down ]]; then
        cmd+=(--post-alignment-gyro-z-scale 0.9817)
      elif [[ "${VARIANT}" == gyro_z_turn_scale_up ]]; then
        cmd+=(--post-alignment-gyro-z-scale 1.0186
          --post-alignment-gyro-z-scale-min-abs 0.1)
      elif [[ "${VARIANT}" == gyro_z_turn_scale_down ]]; then
        cmd+=(--post-alignment-gyro-z-scale 0.9817
          --post-alignment-gyro-z-scale-min-abs 0.1)
      elif [[ "${VARIANT}" == first_error_msckf_yaw_freeze ]]; then
        if [[ "${flight}" == fly1 ]]; then
          cmd+=(--msckf-yaw-freeze-window 983.793 1089.793)
        else
          cmd+=(--msckf-yaw-freeze-window 740.0 897.0)
        fi
      fi
      ;;
    gyro_z_plus) cmd+=(--post-alignment-gyro-z-perturbation 0.0001) ;;
    gyro_z_minus) cmd+=(--post-alignment-gyro-z-perturbation -0.0001) ;;
    gyro_z_observed_plus) cmd+=(--post-alignment-gyro-z-perturbation 0.0007) ;;
    gyro_z_observed_minus) cmd+=(--post-alignment-gyro-z-perturbation -0.0007) ;;
    fc_attitude_release) cmd+=(--online-alignment-release-attitude-from-fc) ;;
    fc_yaw_aid) cmd+=(
      --post-alignment-fc-yaw-aid
      --post-alignment-fc-yaw-aid-period 2.0
      --post-alignment-fc-yaw-aid-sigma-deg 2.0
      --post-alignment-fc-yaw-aid-log "${out}/fc_yaw_aid.csv"
    ) ;;
    cam_toff_converged)
      [[ "${flight}" == fly1 ]] && cmd+=(--cam-toff -0.00394) || cmd+=(--cam-toff -0.00152)
      ;;
    cam_toff_opposite)
      [[ "${flight}" == fly1 ]] && cmd+=(--cam-toff 0.00394) || cmd+=(--cam-toff 0.00152)
      ;;
    cam_extrinsic_rot_plus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg 0.75320088 -0.40135798 0.75178641)
      ;;
    cam_extrinsic_rot_minus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg -0.75320088 0.40135798 -0.75178641)
      ;;
    cam_extrinsic_x_plus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg 0.75320088 0.0 0.0)
      ;;
    cam_extrinsic_x_minus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg -0.75320088 0.0 0.0)
      ;;
    cam_extrinsic_y_plus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg 0.0 0.40135798 0.0)
      ;;
    cam_extrinsic_y_minus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg 0.0 -0.40135798 0.0)
      ;;
    cam_extrinsic_z_plus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg 0.0 0.0 0.75178641)
      ;;
    cam_extrinsic_z_minus)
      cmd+=(--post-alignment-camera-extrinsic-left-rotvec-deg 0.0 0.0 -0.75178641)
      ;;
    adaptive_stride_active) cmd+=(--adaptive-stride) ;;
    full_rate_post_init) cmd+=(--post-alignment-camera-frame-stride 1) ;;
    visual_roi_left) cmd+=(--post-alignment-visual-roi left) ;;
    visual_roi_center) cmd+=(--post-alignment-visual-roi center) ;;
    visual_roi_right) cmd+=(--post-alignment-visual-roi right) ;;
    visual_roi_top) cmd+=(--post-alignment-visual-roi top) ;;
    visual_roi_bottom) cmd+=(--post-alignment-visual-roi bottom) ;;
    visual_roi_dynamic_turn) cmd+=(
      --post-alignment-visual-roi dynamic_turn
      --dynamic-turn-roi-diag "${out}/dynamic_turn_roi.csv"
    ) ;;
    agl_scale_shadow) cmd+=(
      --agl-scene-scale shadow
      --agl-scene-scale-diag "${out}/agl_scene_scale.csv"
    ) ;;
    agl_scale_single) cmd+=(
      --agl-scene-scale single_reset
      --agl-scene-scale-diag "${out}/agl_scene_scale.csv"
    ) ;;
    no_slam) cmd+=(--disable-slam-features) ;;
    legacy_init_covariance) cmd+=(
      --online-alignment-use-cli-init-covariance
      --init-att-sigma-deg 3.0 --init-pos-sigma 0.05
      --init-vel-sigma 5.0 --init-bg-sigma 0.003 --init-ba-sigma 1.0
    ) ;;
    local_estimator_origin) cmd+=(--online-alignment-local-estimator-origin) ;;
    targeted_slam_freeze)
      if [[ "${flight}" == fly1 ]]; then
        cmd+=(--slam-update-freeze-window 1089.0 1151.0)
      else
        cmd+=(--slam-update-freeze-window 1000.0 1075.2)
      fi
      ;;
    first_error_slam_freeze)
      if [[ "${flight}" == fly1 ]]; then
        cmd+=(--slam-update-freeze-window 940.5 983.793)
      else
        cmd+=(--slam-update-freeze-window 818.5 897.0)
      fi
      ;;
    first_error_all_visual_yaw_gain_zero)
      if [[ "${flight}" == fly1 ]]; then
        cmd+=(--visual-yaw-gain-zero-window 983.793 1089.793)
      else
        cmd+=(--visual-yaw-gain-zero-window 740.0 897.0)
      fi
      ;;
  esac
  if [[ "${VARIANT}" == gyro_z_scale_up ]]; then
    [[ " ${cmd[*]} " == *" --post-alignment-gyro-z-scale 1.0186 "* ]] || {
      printf 'gyro_z_scale_up command contract missing scale flag\n' >&2
      exit 3
    }
  elif [[ "${VARIANT}" == gyro_z_scale_down ]]; then
    [[ " ${cmd[*]} " == *" --post-alignment-gyro-z-scale 0.9817 "* ]] || {
      printf 'gyro_z_scale_down command contract missing scale flag\n' >&2
      exit 3
    }
  elif [[ "${VARIANT}" == gyro_z_turn_scale_up ]]; then
    [[ " ${cmd[*]} " == *" --post-alignment-gyro-z-scale 1.0186 --post-alignment-gyro-z-scale-min-abs 0.1 "* ]] || {
      printf 'gyro_z_turn_scale_up command contract missing scale gate\n' >&2
      exit 3
    }
  elif [[ "${VARIANT}" == gyro_z_turn_scale_down ]]; then
    [[ " ${cmd[*]} " == *" --post-alignment-gyro-z-scale 0.9817 --post-alignment-gyro-z-scale-min-abs 0.1 "* ]] || {
      printf 'gyro_z_turn_scale_down command contract missing scale gate\n' >&2
      exit 3
    }
  elif [[ "${VARIANT}" == first_error_msckf_yaw_freeze ]]; then
    if [[ "${flight}" == fly1 ]]; then
      expected_msckf_window="--msckf-yaw-freeze-window 983.793 1089.793"
    else
      expected_msckf_window="--msckf-yaw-freeze-window 740.0 897.0"
    fi
    [[ " ${cmd[*]} " == *" ${expected_msckf_window} "* ]] || {
      printf 'first_error_msckf_yaw_freeze command contract missing window\n' >&2
      exit 3
    }
  elif [[ "${VARIANT}" == first_error_all_visual_yaw_gain_zero ]]; then
    if [[ "${flight}" == fly1 ]]; then
      expected_visual_yaw_window="--visual-yaw-gain-zero-window 983.793 1089.793"
    else
      expected_visual_yaw_window="--visual-yaw-gain-zero-window 740.0 897.0"
    fi
    [[ " ${cmd[*]} " == *" ${expected_visual_yaw_window} "* ]] || {
      printf 'first_error_all_visual_yaw_gain_zero command contract missing window\n' >&2
      exit 3
    }
  fi
  if [[ "${VARIANT}" == no_gpsz ||
        "${VARIANT}" == mechanism_control ||
        "${VARIANT}" == first_error_deep_diag ||
        "${VARIANT}" == first_turn_deep_diag ||
        "${VARIANT}" == first_error_msckf_yaw_freeze ||
        "${VARIANT}" == first_error_all_visual_yaw_gain_zero ||
        "${VARIANT}" == no_visual_yaw ||
        "${VARIANT}" == no_visual_bgz ||
        "${VARIANT}" == gyro_z_plus ||
        "${VARIANT}" == gyro_z_minus ||
        "${VARIANT}" == gyro_z_observed_plus ||
        "${VARIANT}" == gyro_z_observed_minus ||
        "${VARIANT}" == gyro_z_scale_up ||
        "${VARIANT}" == gyro_z_scale_down ||
        "${VARIANT}" == gyro_z_turn_scale_up ||
        "${VARIANT}" == gyro_z_turn_scale_down ||
        "${VARIANT}" == no_slam ||
        "${VARIANT}" == legacy_init_covariance ||
        "${VARIANT}" == targeted_slam_freeze ||
        "${VARIANT}" == first_error_slam_freeze ||
        "${VARIANT}" == gpsz_guard_pz_fallback ||
        "${VARIANT}" == fc_attitude_release ||
        "${VARIANT}" == fc_yaw_aid ||
        "${VARIANT}" == cam_toff_converged ||
        "${VARIANT}" == cam_toff_opposite ||
        "${VARIANT}" == cam_extrinsic_rot_plus ||
        "${VARIANT}" == cam_extrinsic_rot_minus ||
        "${VARIANT}" == cam_extrinsic_x_plus ||
        "${VARIANT}" == cam_extrinsic_x_minus ||
        "${VARIANT}" == cam_extrinsic_y_plus ||
        "${VARIANT}" == cam_extrinsic_y_minus ||
        "${VARIANT}" == cam_extrinsic_z_plus ||
        "${VARIANT}" == cam_extrinsic_z_minus ||
        "${VARIANT}" == centered_yaw_oc ||
        "${VARIANT}" == local_estimator_origin ||
        "${VARIANT}" == full_4d_oc ||
        "${VARIANT}" == post_full_4d_oc ||
        "${VARIANT}" == adaptive_stride_active ||
        "${VARIANT}" == full_rate_post_init ||
        "${VARIANT}" == visual_roi_left ||
        "${VARIANT}" == visual_roi_center ||
        "${VARIANT}" == visual_roi_right ||
        "${VARIANT}" == visual_roi_top ||
        "${VARIANT}" == visual_roi_bottom ||
        "${VARIANT}" == visual_roi_dynamic_turn ||
        "${VARIANT}" == agl_scale_shadow ||
        "${VARIANT}" == agl_scale_single ||
        "${VARIANT}" == gpsz_nasa_lean ]]; then
    cmd+=(
      --vio-yaw-diag "${out}/vio_yaw_diag.csv"
      --visual-obs-diag "${out}/visual_observability_diag.csv"
      --yaw-update-mechanism-diag "${out}/yaw_update_mechanism_diag.csv"
      --imu-propagation-yaw-diag "${out}/imu_propagation_yaw_diag.csv"
      --slam-feature-yaw-contrib-diag "${out}/slam_feature_yaw_contrib_diag.csv"
      --visual-feature-residual-diag "${out}/visual_feature_residual_diag.csv"
    )
  fi
  if [[ "${VARIANT}" != no_gpsz &&
        "${VARIANT}" != agl_scale_shadow &&
        "${VARIANT}" != agl_scale_single ]]; then
    local height_mode=guarded
    [[ "${VARIANT}" == gpsz_nasa_lean ]] && height_mode=nasa-lean
    cmd+=(--gps-alt-update --height-mode "${height_mode}" --gps-alt-sigma 2.0
      --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10
      --gps-alt-max-res 80 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0
      --gps-alt-coupled-diag "${out}/gps_alt_coupled_diag.csv")
    if [[ "${VARIANT}" == gpsz_nasa_lean ]]; then
      cmd+=(--gps-alt-nasa-beta 0.2 --gps-alt-nasa-q-threshold 0.0)
    elif [[ "${VARIANT}" == gpsz_guard_pz_fallback ]]; then
      cmd+=(--gps-alt-guard-fallback-joseph-pz)
    fi
  fi
  case "${VARIANT}" in
    retain_bias) cmd+=(--online-alignment-retain-bg-prior --online-alignment-retain-ba-prior) ;;
    retain_bg) cmd+=(--online-alignment-retain-bg-prior) ;;
    retain_ba) cmd+=(--online-alignment-retain-ba-prior) ;;
  esac
  if [[ "${VISUALIZATION}" == visible && -n "${DISPLAY:-}" ]]; then
    cmd+=(--viz-fast --dash-every 5)
    printf 'visible_dashboard\n' > "${out}/visualization_mode.txt"
  else
    cmd+=(--no-dashboard)
    printf 'parallel_causal_ablation_no_dashboard\n' > "${out}/visualization_mode.txt"
  fi
  printf '%q ' "${cmd[@]}" > "${out}/command.txt"; printf '\n' >> "${out}/command.txt"
  sha256sum "${BINARY}" "${CONFIG}" "${gps}" "${fc_stream}" > "${out}/input_identity_sha256.txt"
  date --iso-8601=seconds > "${out}/process_start.txt"
  local code=0
  /usr/bin/time -v -o "${out}/resource_usage.txt" "${cmd[@]}" \
    > "${out}/stdout.log" 2> "${out}/stderr.log" || code=$?
  date --iso-8601=seconds > "${out}/process_end.txt"
  printf '%s\n' "${code}" > "${out}/exit_code.txt"
  local frame_code=0
  if (( code == 0 )); then
    "${PYTHON_BIN}" "${FRAME_VALIDATOR}" --raw "${out}/traj_raw.txt" \
      --raw-bias "${out}/traj.txt.bias" --nav "${out}/traj_nav.txt" \
      --metadata "${out}/nav_frame_metadata.json" \
      --out "${out}/frame_contract_validation.json" \
      > "${out}/frame_contract_validation.log" 2>&1 || frame_code=$?
  fi
  printf '%s\n' "${frame_code}" > "${out}/frame_contract_validation_exit_code.txt"
  (( code == 0 && frame_code == 0 ))
}

printf 'flight,exit_code\n' > "${ROOT}/batch_exit_codes.csv"
status=0
if [[ "${VISUALIZATION}" == visible ]]; then
  for flight in "${flights[@]}"; do
    code=0; run_one "${flight}" || code=$?
    printf '%s,%s\n' "${flight}" "${code}" >> "${ROOT}/batch_exit_codes.csv"
    (( code == 0 )) || status=1
  done
else
  declare -a pids=()
  for flight in "${flights[@]}"; do run_one "${flight}" & pids+=("$!"); done
  for i in "${!flights[@]}"; do
    code=0; wait "${pids[$i]}" || code=$?
    printf '%s,%s\n' "${flights[$i]}" "${code}" >> "${ROOT}/batch_exit_codes.csv"
    (( code == 0 )) || status=1
  done
fi
exit "${status}"
