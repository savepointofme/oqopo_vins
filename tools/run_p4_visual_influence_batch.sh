#!/usr/bin/env bash
set -u

repo=/mnt/d/vscode_dir/open_vins
runner="$repo/build_ov_msckf/run_serial_msckf_ros_free"
config="$repo/baseline/clean_p4/config/estimator_config.yaml"
desktop_root="/mnt/c/Users/baloney/Desktop/实验目录"
delivery_root="$desktop_root/P4_arbitrary_start_20260712"
fc_stream="$desktop_root/P4_online_joint_alignment_20260712/inputs/fly1_fc_navigation_online.csv"
dataset=/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810
stamp="$(date +%Y%m%d_%H%M%S)"
batch="$delivery_root/runs/${stamp}_fly1_visual_influence_batch"
mkdir -p "$batch"

run_one() {
  local name="$1"
  shift
  local out="$batch/$name"
  mkdir -p "$out"
  local cmd=(
    "$runner"
    --config "$config"
    --dataset "$dataset"
    --start-time 925
    --until-time "${P4_VISUAL_UNTIL_TIME:-995}"
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
    "$@"
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
if [[ "${P4_V2_SHORT_ONLY:-0}" == 1 ]]; then
  run_one V2_perturbed_monocular_tracks_short_diagnostic \
    --online-alignment-visual-perturbation-px 35 \
    --online-alignment-visual-perturbation-fraction 0.5
  status=$?
  printf '%s\n' "$batch" >"$delivery_root/latest_visual_v2_short_batch.txt"
  exit "$status"
fi

run_one V0_original_monocular_images & pid0=$!
run_one V1_visual_factors_disabled \
  --online-alignment-disable-visual \
  --online-alignment-navigation-allow-without-visual & pid1=$!
run_one V2_perturbed_monocular_tracks \
  --online-alignment-visual-perturbation-px 35 \
  --online-alignment-visual-perturbation-fraction 0.5 & pid2=$!

wait "$pid0" || status=1
wait "$pid1" || status=1
wait "$pid2" || status=1
printf '%s\n' "$batch" >"$delivery_root/latest_visual_influence_batch.txt"
exit "$status"
