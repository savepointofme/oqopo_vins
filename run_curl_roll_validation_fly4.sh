#!/usr/bin/env bash
# run_curl_roll_validation_fly4.sh
#
# Curl correction and extrinsic roll perturbation validation — fly4 only.
#
# Experiments:
#   A  baseline          no correction, standard config
#   B  curl_p159         --curl-correction-rate +159.6 deg/s (CCW feature rotation)
#   C  roll_m1p0         extrinsic Rx(−1.0°) perturbed config
#   D  roll_m0p5         extrinsic Rx(−0.5°) perturbed config
#   E  roll_p0p5         extrinsic Rx(+0.5°) perturbed config
#   F  roll_p1p0         extrinsic Rx(+1.0°) perturbed config
#   G  curl_roll_best    curl+best-roll combined (run last, only if both individually improve)
#
# Fixed across all runs (mainline locked params):
#   sigma_px=2  chi2=1  max_clones=11  fi_max_dist=2000
#   yaw=oc_postchi2_current_gauge  GPS-Z ON  init_bg_sigma=0.003
#   window: start=924.4  eval=[924.4, 2500]
#
# Usage:
#   ./run_curl_roll_validation_fly4.sh            # all experiments
#   ./run_curl_roll_validation_fly4.sh --dry-run  # print commands only
#   ./run_curl_roll_validation_fly4.sh --only A B C  # specific experiments
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
EVAL_SCRIPT="$REPO/eval_stage.py"
PERTURB_SCRIPT="$REPO/analysis/perturb_calib.py"

# ─── Flight config ────────────────────────────────────────────────────────────
DATASET="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549"
BASE_CFG="$REPO/config/d455_fly2/estimator_config.yaml"
BASE_CFG_DIR="$REPO/config/d455_fly2"
GPS_CSV="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
FC_INIT="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/canonical_offsetm202p2_start924p4/fc_init_state_924p4_offsetm202p2.csv"
START_TIME="924.4"
EVAL_T0="924.4"
EVAL_UNTIL="2500"
GPS_OFFSET="0"
OUTPUT_BASE="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/yaw_floor_fix_20260610"

# ─── Fixed VIO flags ──────────────────────────────────────────────────────────
YAW_MODE="oc_postchi2_current_gauge"
BG_SIGMA="0.003"
GPS_Z_FLAGS=(
    --gps-alt-update
    --gps-alt-sigma        2.0
    --gps-alt-min-pzz      0.01
    --gps-alt-max-res      80
    --gps-alt-min-t-after-init 10
    --gps-alt-guard-dxy    0.5
    --gps-alt-guard-kxy    5.0
)

# ─── CLI parsing ──────────────────────────────────────────────────────────────
DRY_RUN=0
ONLY_FILTER=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --only) shift; while [[ $# -gt 0 && "$1" != --* ]]; do ONLY_FILTER+=("$1"); shift; done ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

should_run() {
    local tag="$1"
    [[ ${#ONLY_FILTER[@]} -eq 0 ]] && return 0
    for f in "${ONLY_FILTER[@]}"; do [[ "$f" == "$tag" ]] && return 0; done
    return 1
}

# ─── Config dir creation helpers ──────────────────────────────────────────────
ROLL_CFG_BASE="$OUTPUT_BASE/cfg_roll_perturb"

make_roll_cfg() {
    local tag="$1" roll_deg="$2"
    local cfg_dir="$ROLL_CFG_BASE/cfg_${tag}"
    if [[ ! -d "$cfg_dir" ]]; then
        python3 "$PERTURB_SCRIPT" \
            --cfg-dir "$BASE_CFG_DIR" \
            --out-dir "$cfg_dir" \
            --ext-roll-deg "$roll_deg" \
            --freeze ext
        echo "[cfg] created $cfg_dir (roll=${roll_deg}°)" >&2
    else
        echo "[cfg] $cfg_dir already exists, reusing" >&2
    fi
    echo "$cfg_dir"
}

# ─── Runner ───────────────────────────────────────────────────────────────────
run_experiment() {
    local tag="$1" label="$2" cfg_file="$3"
    shift 3
    local extra_flags=("$@")

    local out_dir="$OUTPUT_BASE/${tag}_fly4"
    if [[ -d "$out_dir/forensics_valid" ]]; then
        echo "[SKIP] $tag already done (forensics_valid exists)"
        return
    fi
    mkdir -p "$out_dir"

    local diag_csv="$out_dir/vio_yaw_diag.csv"
    local cmd=(
        "$BINARY"
        --config "$cfg_file"
        --dataset "$DATASET"
        --gps "$GPS_CSV"
        --gps-time-offset "$GPS_OFFSET"
        --start-time "$START_TIME"
        --init-from-fc "$FC_INIT"
        --vio-yaw-gauge-mode "$YAW_MODE"
        --init-bg-sigma "$BG_SIGMA"
        "${GPS_Z_FLAGS[@]}"
        --viz-fast --dash-every 5
        --output "$out_dir/traj.txt"
        --vio-yaw-diag "$diag_csv"
        "${extra_flags[@]}"
    )

    echo ""
    echo "════════════════════════════════════════════════════"
    echo "  $tag  ($label)"
    echo "════════════════════════════════════════════════════"
    printf '%s \\\n  ' "${cmd[@]}"
    printf '\n'
    printf '%s \\\n  ' "${cmd[@]}" > "$out_dir/command.txt"
    printf '\n' >> "$out_dir/command.txt"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo "[DRY-RUN] skipping execution"
        return
    fi

    "${cmd[@]}" 2>&1 | tee "$out_dir/log.txt"
    echo "[run] $tag complete"

    # Eval
    python3 "$EVAL_SCRIPT" \
        --traj     "$out_dir/traj.txt" \
        --gps      "$GPS_CSV" \
        --fc-init  "$FC_INIT" \
        --t0       "$EVAL_T0" \
        --t-end    "$EVAL_UNTIL" \
        --out-dir  "$out_dir/forensics_valid" \
        --label    "$tag" \
        2>&1 | tee "$out_dir/eval.log" || echo "[warn] eval failed for $tag"
}

# ─── Experiments ──────────────────────────────────────────────────────────────

if should_run A; then
    run_experiment "CURL_BASELINE" "baseline — no correction" "$BASE_CFG"
fi

if should_run B; then
    run_experiment "CURL_CONST_P159" "curl +0.1596 deg/s = +159.6 mdeg/s CCW correction" "$BASE_CFG" \
        --curl-correction-rate 0.1596
fi

# Create roll perturb config dirs (idempotent)
if should_run C || should_run D || should_run E || should_run F || should_run G; then
    mkdir -p "$ROLL_CFG_BASE"
    CFG_ROLL_M1P0=$(make_roll_cfg "roll_m1p0" -1.0)/estimator_config.yaml
    CFG_ROLL_M0P5=$(make_roll_cfg "roll_m0p5" -0.5)/estimator_config.yaml
    CFG_ROLL_P0P5=$(make_roll_cfg "roll_p0p5"  0.5)/estimator_config.yaml
    CFG_ROLL_P1P0=$(make_roll_cfg "roll_p1p0"  1.0)/estimator_config.yaml
fi

if should_run C; then
    run_experiment "EXTR_ROLL_M1P0" "roll −1.0° extrinsic perturbation" "$CFG_ROLL_M1P0"
fi

if should_run D; then
    run_experiment "EXTR_ROLL_M0P5" "roll −0.5° extrinsic perturbation" "$CFG_ROLL_M0P5"
fi

if should_run E; then
    run_experiment "EXTR_ROLL_P0P5" "roll +0.5° extrinsic perturbation" "$CFG_ROLL_P0P5"
fi

if should_run F; then
    run_experiment "EXTR_ROLL_P1P0" "roll +1.0° extrinsic perturbation" "$CFG_ROLL_P1P0"
fi

# G: combined curl+best-roll — create after inspecting C-F results
# Uncomment with the correct roll degree once roll ablation is analysed.
# if should_run G; then
#     BEST_ROLL_DEG="-0.5"  # <-- update after ablation
#     BEST_ROLL_TAG="roll_m0p5"
#     CFG_BEST=$(make_roll_cfg "$BEST_ROLL_TAG" "$BEST_ROLL_DEG")/estimator_config.yaml
#     run_experiment "CURL_ROLL_BEST" "curl +0.1596 deg/s + roll ${BEST_ROLL_DEG}°" "$CFG_BEST" \
#         --curl-correction-rate 0.1596
# fi

echo ""
echo "All requested experiments complete."
echo "Results in: $OUTPUT_BASE"
