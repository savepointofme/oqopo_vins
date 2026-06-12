#!/usr/bin/env bash
set -euo pipefail

REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
SOURCE_CONFIG="$REPO/config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml"
SOURCE_CALIB="$REPO/config/d455_fly1"

FLIGHT_ROOT="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4"
DATASET="$FLIGHT_ROOT/d455_20260527_090549"
GPS="$FLIGHT_ROOT/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
FC_INIT="$FLIGHT_ROOT/result/fc_rebuild_20260528/canonical_offsetm202p2_start924p4/fc_init_state_924p4_offsetm202p2.csv"
NEW_CALIB_SOURCE="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_20260612_fly4/config"
SWEEP_ROOT="$FLIGHT_ROOT/result/yaw_floor_fix_20260610/NEWCALIB_FIBASE_SWEEP_ANTIVIB_LOCKED_20260612"

BASELINES=(50 200 500 1000)
mkdir -p "$SWEEP_ROOT"

make_config() {
  local baseline="$1"
  local out="$SWEEP_ROOT/fi_dist1200_baseline${baseline}"
  local cfg="$out/config"

  if [[ -e "$out/traj.txt" || -e "$out/log.txt" ]]; then
    echo "Refusing to overwrite existing run: $out" >&2
    return 1
  fi

  mkdir -p "$cfg"

  awk -v baseline="$baseline" '
    /^calib_cam_extrinsics:/ { print "calib_cam_extrinsics: false"; next }
    /^calib_cam_intrinsics:/ { print "calib_cam_intrinsics: false"; next }
    /^calib_cam_timeoffset:/ { print "calib_cam_timeoffset: false"; next }
    /^fi_max_dist:/ {
      print "fi_max_dist: 1200.0"
      print "fi_max_baseline: " baseline ".0"
      next
    }
    { print }
  ' "$SOURCE_CONFIG" > "$cfg/config_new.yaml"

  sed \
    -e 's/relative_config_imu: "kalibr_imu_chain.yaml"/relative_config_imu: "kalibr_imu_chain_old.yaml"/' \
    -e 's/relative_config_imucam: "kalibr_imucam_chain.yaml"/relative_config_imucam: "kalibr_imucam_chain_old.yaml"/' \
    "$cfg/config_new.yaml" > "$cfg/config_old.yaml"

  cp "$SOURCE_CALIB/kalibr_imu_chain.yaml" "$cfg/kalibr_imu_chain_old.yaml"
  cp "$SOURCE_CALIB/kalibr_imucam_chain.yaml" "$cfg/kalibr_imucam_chain_old.yaml"
  cp "$SOURCE_CALIB/kalibr_imu_chain.yaml" "$cfg/kalibr_imu_chain.yaml"
  cp "$NEW_CALIB_SOURCE/kalibr_imucam_chain.yaml" "$cfg/kalibr_imucam_chain.yaml"

  {
    echo "=== estimator config ==="
    diff -u "$cfg/config_old.yaml" "$cfg/config_new.yaml" || true
    echo
    echo "=== camera / camera-IMU calibration ==="
    diff -u "$cfg/kalibr_imucam_chain_old.yaml" "$cfg/kalibr_imucam_chain.yaml" || true
    echo
    echo "=== IMU calibration ==="
    diff -u "$cfg/kalibr_imu_chain_old.yaml" "$cfg/kalibr_imu_chain.yaml" || true
  } > "$cfg/config_diff.txt"

  cat > "$out/CONTROLLED_CHANGE.txt" <<EOF
Mother config:
  config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml

Fixed:
  fi_max_dist = 1200
  fi_max_cond_number = code default 10000
  up_msckf_sigma_px = 2
  up_msckf_chi2_multipler = 1
  yaw mode = global_yaw_oc_projection, alpha = 1
  GPS-Z = disabled
  FC init attitude sigma = 3 deg
  online camera intrinsics/extrinsics/time-offset = disabled
  new camera intrinsics/distortion/extrinsics/time offset = fixed
  anti-vibration IMU noise = accel 0.20, gyro 0.005
  anti-vibration IMU random walks = retained from d455_fly1

Only sweep variable:
  fi_max_baseline = ${baseline}
EOF
}

run_one() {
  local baseline="$1"
  local out="$SWEEP_ROOT/fi_dist1200_baseline${baseline}"
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
    --diag-csv "$out/diag.csv"
    --vio-yaw-diag "$out/vio_yaw_diag.csv"
    --viz-fast
    --dash-every 5
    --output "$out/traj.txt"
  )

  printf '%q ' "${cmd[@]}" > "$out/command.txt"
  printf '\n' >> "$out/command.txt"

  "${cmd[@]}" > "$out/log.txt" 2>&1

  python3 "$REPO/eval_stage.py" \
    --t0 924.4 \
    --until 2816 \
    --dir-a "$out" \
    --dir-b "$out" \
    --gps "$GPS" \
    --imu "$DATASET/imu0/data.csv" \
    --out "$out/forensics_valid" \
    --yaw-align-mode start_yaw \
    > "$out/eval.log" 2>&1
}

for baseline in "${BASELINES[@]}"; do
  make_config "$baseline"
done

cat > "$SWEEP_ROOT/SWEEP_PLAN.txt" <<'EOF'
Four simultaneous fly4 visual runs:
  fi_max_baseline = 50, 200, 500, 1000

All runs use fi_max_dist=1200 and otherwise share the same d455_fly1
high-altitude config, fixed new camera calibration, anti-vibration IMU noise,
global yaw OC, and no GPS-Z.
EOF

declare -a pids=()
for baseline in "${BASELINES[@]}"; do
  run_one "$baseline" &
  pids+=("$!")
done

status=0
for pid in "${pids[@]}"; do
  wait "$pid" || status=1
done

if [[ "$status" -ne 0 ]]; then
  echo "One or more runs failed; inspect per-run log.txt files." >&2
  exit "$status"
fi

echo "Sweep complete: $SWEEP_ROOT"
