#!/usr/bin/env bash
set -u

repo=/mnt/d/vscode_dir/open_vins
runner="$repo/build_ov_msckf/run_serial_msckf_ros_free"
config="$repo/baseline/clean_p4/config/estimator_config.yaml"
desktop_root="/mnt/c/Users/baloney/Desktop/实验目录"
delivery_root="$desktop_root/P4_arbitrary_start_20260712"
fc_root="$desktop_root/P4_online_joint_alignment_20260712/inputs"
batch_name="${1:-}"

if [[ "$batch_name" != fly1 && "$batch_name" != fly3 ]]; then
  echo "usage: $0 fly1|fly3" >&2
  exit 2
fi

stamp="$(date +%Y%m%d_%H%M%S)"
batch="$delivery_root/runs/${stamp}_${batch_name}_practical_batch"
mkdir -p "$batch"

run_one() {
  local name="$1"
  local dataset="$2"
  local fc_stream="$3"
  local start_time="$4"
  local until_time="$5"
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
    --online-alignment-release-policy practical_navigation_start
    --camera-frame-stride 12
    --viz-fast
    --dash-every 5
    --diag-csv "$out/diag.csv"
    --diag-events "$out/events.txt"
    --output "$out/traj.txt"
    --output-raw "$out/traj_raw.txt"
    --output-nav "$out/traj_nav.txt"
    --online-alignment-metadata-json "$out/alignment_result.json"
  )
  printf '%q ' "${cmd[@]}" >"$out/command.txt"
  printf '\n' >>"$out/command.txt"
  date --iso-8601=seconds >"$out/process_start.txt"
  "${cmd[@]}" >"$out/stdout.log" 2>"$out/stderr.log"
  local code=$?
  printf '%d\n' "$code" >"$out/exit_code.txt"
  date --iso-8601=seconds >"$out/process_end.txt"
  return "$code"
}

status=0
if [[ "$batch_name" == fly1 ]]; then
  dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
  fc_stream="$fc_root/fly1_fc_navigation_online.csv"
  run_one fly1_straight_start "$dataset" "$fc_stream" 925 995 & pid1=$!
  run_one fly1_weak_motion_start "$dataset" "$fc_stream" 930 1000 & pid2=$!
  run_one fly1_post_turn_start "$dataset" "$fc_stream" 994 1064 & pid3=$!
else
  dataset=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946
  fc_stream="$fc_root/fly3_fc_navigation_online.csv"
  run_one fly3_straight_start "$dataset" "$fc_stream" 613 683 & pid1=$!
  run_one fly3_weak_motion_start "$dataset" "$fc_stream" 682 752 & pid2=$!
  run_one fly3_post_turn_start "$dataset" "$fc_stream" 750 820 & pid3=$!
fi

wait "$pid1" || status=1
wait "$pid2" || status=1
wait "$pid3" || status=1
printf '%s\n' "$batch" >"$delivery_root/latest_${batch_name}_batch.txt"
exit "$status"
