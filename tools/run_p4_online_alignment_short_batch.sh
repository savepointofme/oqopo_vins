#!/usr/bin/env bash
set -u

repo=/mnt/d/vscode_dir/open_vins
runner="$repo/build_ov_msckf/run_serial_msckf_ros_free"
config="$repo/baseline/clean_p4/config/estimator_config.yaml"
root=/mnt/c/Users/baloney/Desktop/实验目录/P4_online_joint_alignment_20260712
stamp="$(date +%Y%m%d_%H%M%S)"
batch="$root/runs/${stamp}_causal_monocular_joint_short"
mkdir -p "$batch"

run_one() {
  local name="$1"
  local dataset="$2"
  local fc_stream="$3"
  local start_time="$4"
  local until_time="$5"
  local disable_visual="$6"
  local out="$batch/$name"
  mkdir -p "$out"
  local cmd=(
    "$runner"
    --config "$config"
    --dataset "$dataset"
    --start-time "$start_time"
    --until-time "$until_time"
    --initialization-mode online_multisensor_alignment
    --init-from-fc "$fc_stream"
    --init-from-fc-position-frame global_gnav
    --init-window-s 8.0
    --init-window-min-samples 12
    # The registered FC stream is nominally 5 Hz and occasionally drops one
    # row, producing a measured 0.4000001 s gap in both flights.
    --init-window-max-source-gap 0.45
    --camera-frame-stride 12
    --viz-fast
    --dash-every 5
    --diag-csv "$out/diag.csv"
    --diag-events "$out/events.txt"
    --output "$out/traj.txt"
    --output-raw "$out/traj_raw.txt"
    --output-nav "$out/traj_nav.txt"
    --online-alignment-metadata-json "$out/online_alignment_metadata.json"
  )
  if [[ "$disable_visual" == 1 ]]; then
    cmd+=(--online-alignment-disable-visual)
  fi
  printf '%q ' "${cmd[@]}" >"$out/command.txt"
  printf '\n' >>"$out/command.txt"
  date --iso-8601=seconds >"$out/start_time.txt"
  "${cmd[@]}" >"$out/stdout.log" 2>"$out/stderr.log"
  local code=$?
  printf '%d\n' "$code" >"$out/exit_code.txt"
  date --iso-8601=seconds >"$out/end_time.txt"
  return "$code"
}

if [[ "${P4_FLY3_EXTENDED_ONLY:-0}" == 1 ]]; then
  run_one \
    fly3_real_joint_extended_60s \
    /mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946 \
    "$root/inputs/fly3_fc_navigation_online.csv" \
    613 820 0
  status=$?
  printf '%s\n' "$batch" >"$root/latest_fly3_extended_batch.txt"
  exit "$status"
fi

run_one \
  fly1_real_joint \
  /mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810 \
  "$root/inputs/fly1_fc_navigation_online.csv" \
  925 1040 0 &
pid_fly1=$!

run_one \
  fly3_real_joint \
  /mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946 \
  "$root/inputs/fly3_fc_navigation_online.csv" \
  613 780 0 &
pid_fly3=$!

run_one \
  fly1_no_visual_negative_control \
  /mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810 \
  "$root/inputs/fly1_fc_navigation_online.csv" \
  925 1040 1 &
pid_no_visual=$!

status=0
wait "$pid_fly1" || status=1
wait "$pid_fly3" || status=1
wait "$pid_no_visual" || status=1
printf '%s\n' "$batch" >"$root/latest_short_batch.txt"
exit "$status"
