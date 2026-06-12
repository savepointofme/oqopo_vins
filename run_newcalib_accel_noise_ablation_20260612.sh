#!/usr/bin/env bash
set -euo pipefail

REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
FLIGHT_ROOT="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4"
DATASET="$FLIGHT_ROOT/d455_20260527_090549"
GPS="$FLIGHT_ROOT/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
FC_INIT="$FLIGHT_ROOT/result/fc_rebuild_20260528/canonical_offsetm202p2_start924p4/fc_init_state_924p4_offsetm202p2.csv"
SOURCE="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_FIBASE_SWEEP_ANTIVIB_LOCKED_20260612/fi_dist1200_baseline50/config"
OUT_ROOT="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_ACCEL_NOISE_ABLATION_GPSZ_RERUN_20260612"

declare -A ACCEL_NOISE
ACCEL_NOISE[allan]="0.009143029881205223"
ACCEL_NOISE[mid]="0.05"

prepare_run() {
  local name="$1"
  local accel_noise="${ACCEL_NOISE[$name]}"
  local out="$OUT_ROOT/accel_${name}"
  local cfg="$out/config"

  if [[ -e "$out/traj.txt" || -e "$out/log.txt" ]]; then
    echo "Refusing to overwrite existing run: $out" >&2
    return 1
  fi

  mkdir -p "$cfg"
  cp "$SOURCE/config_new.yaml" "$cfg/config_new.yaml"
  cp "$SOURCE/kalibr_imucam_chain.yaml" "$cfg/kalibr_imucam_chain.yaml"
  cp "$SOURCE/kalibr_imucam_chain_old.yaml" "$cfg/kalibr_imucam_chain_old.yaml"

  awk -v accel_noise="$accel_noise" '
    /^  accelerometer_noise_density:/ {
      print "  accelerometer_noise_density: " accel_noise
      next
    }
    /^  accelerometer_random_walk:/ {
      print "  accelerometer_random_walk: 0.000477273660359181"
      next
    }
    /^  gyroscope_noise_density:/ {
      print "  gyroscope_noise_density: 0.001284971487348658"
      next
    }
    /^  gyroscope_random_walk:/ {
      print "  gyroscope_random_walk: 1.9398767374345275e-05"
      next
    }
    { print }
  ' "$SOURCE/kalibr_imu_chain.yaml" > "$cfg/kalibr_imu_chain.yaml"

  cp "$SOURCE/config_old.yaml" "$cfg/config_old.yaml"
  cp "$SOURCE/kalibr_imu_chain_old.yaml" "$cfg/kalibr_imu_chain_old.yaml"

  {
    echo "=== estimator config ==="
    diff -u "$SOURCE/config_new.yaml" "$cfg/config_new.yaml" || true
    echo
    echo "=== camera calibration ==="
    diff -u "$SOURCE/kalibr_imucam_chain.yaml" "$cfg/kalibr_imucam_chain.yaml" || true
    echo
    echo "=== IMU calibration ==="
    diff -u "$SOURCE/kalibr_imu_chain.yaml" "$cfg/kalibr_imu_chain.yaml" || true
  } > "$cfg/config_diff.txt"

  cat > "$out/CONTROLLED_CHANGE.txt" <<EOF
Only swept parameter:
  accelerometer_noise_density = ${accel_noise}

Fixed:
  accelerometer_random_walk = 0.000477273660359181
  gyroscope_noise_density = 0.001284971487348658
  gyroscope_random_walk = 1.9398767374345275e-05
  camera intrinsics/extrinsics/time-offset online calibration = false
  new camera calibration = fixed
  fi_max_dist = 1200
  fi_max_baseline = 50
  up_msckf_sigma_px = 2
  up_msckf_chi2_multipler = 1
  yaw = global_yaw_oc_projection, alpha = 1
  guarded GPS-Z = enabled
  gps-alt sigma = 2.0
  gps-alt min Pzz = 0.01
  gps-alt max residual = 80
  gps-alt minimum time after init = 10 s
  gps-alt XY guards = dxy 0.5, kxy 5.0
EOF
}

run_one() {
  local name="$1"
  local out="$OUT_ROOT/accel_${name}"
  local cfg="$out/config"

  cmd=(
    "$BINARY"
    --config "$cfg/config_new.yaml"
    --dataset "$DATASET"
    --gps "$GPS"
    --gps-time-offset 0
    --start-time 924.4
    --init-from-fc "$FC_INIT"
    --init-att-sigma-deg 3.0
    --init-pos-sigma 0.05
    --init-bg-sigma 0.003
    --vio-yaw-update-mode global_yaw_oc_projection
    --vio-global-yaw-oc-alpha 1.0
    --gps-alt-update
    --gps-alt-sigma 2.0
    --gps-alt-min-pzz 0.01
    --gps-alt-max-res 80
    --gps-alt-min-t-after-init 10
    --gps-alt-guard-dxy 0.5
    --gps-alt-guard-kxy 5.0
    --diag-csv "$out/diag.csv"
    --vio-yaw-diag "$out/vio_yaw_diag.csv"
    --viz-fast
    --dash-every 5
    --output "$out/traj.txt"
  )

  printf '%q ' "${cmd[@]}" > "$out/command.txt"
  printf '\n' >> "$out/command.txt"
  "${cmd[@]}" > "$out/log.txt" 2>&1
}

mkdir -p "$OUT_ROOT"
prepare_run allan
prepare_run mid

cat > "$OUT_ROOT/ABLATION_PLAN.txt" <<'EOF'
Two simultaneous visual fly4 runs:
  accel_allan: accelerometer_noise_density = 0.009143029881205223
  accel_mid:   accelerometer_noise_density = 0.05

All other calibration, estimator, initialization, yaw, and feature parameters
are identical. Guarded GPS-Z is enabled using the fly4 baseline A flags.
EOF

run_one allan &
pid_allan=$!
run_one mid &
pid_mid=$!

status=0
wait "$pid_allan" || status=1
wait "$pid_mid" || status=1
exit "$status"
