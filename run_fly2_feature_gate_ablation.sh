#!/usr/bin/env bash
# run_fly2_feature_gate_ablation.sh
#
# 2D feature-gate ablation: fi_max_cond_number × fi_max_dist
# Fixed:  init_bg_sigma=0.003  yaw=oc_postchi2_current_gauge  GPS-Z ON
# Fly2 dataset, canonical offset447p5 GPS/FC-init, start=700
#
# Usage:
#   bash run_fly2_feature_gate_ablation.sh                     # all 3 batches
#   bash run_fly2_feature_gate_ablation.sh --dry-run           # print commands only
#   bash run_fly2_feature_gate_ablation.sh --single cond1e4_dist1000

set -uo pipefail   # -u: unset var error; -o pipefail; intentionally no -e (parallel)

DRY_RUN=0
MODE="batch"
SINGLE_RUN=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1 ;;
        --single)
            MODE="single"
            SINGLE_RUN="${2:-}"
            [[ -z "$SINGLE_RUN" ]] && { echo "ERROR: --single requires a run name"; exit 1; }
            shift
            ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

# ─── Paths ────────────────────────────────────────────────────────────────────
REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
BASE_CONFIG="$REPO/config/d455_fly2/estimator_config.yaml"
CONFIG_DIR="$REPO/config/d455_fly2"   # for absolute kalibr paths in snapshot
EVAL_SCRIPT="$REPO/eval_stage.py"

DATASET="/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722"
GPS="/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv"
FC_INIT="/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv"
IMU="$DATASET/imu0/data.csv"

OUTPUT_BASE="/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/feature_gate_ablation_20260607"

# ─── Fixed run parameters ─────────────────────────────────────────────────────
YAW_MODE="oc_postchi2_current_gauge"
BG_SIGMA="0.003"
GPS_TIME_OFFSET="0"
START_TIME="700"
EVAL_T0="700"
EVAL_UNTIL="2700"

# ─── Ablation matrix (name:cond:dist) ─────────────────────────────────────────
BATCH1=(
    "cond1e4_dist1000:10000:1000"
    "cond1e4_dist1500:10000:1500"
    "cond1e4_dist2000:10000:2000"
)
BATCH2=(
    "cond3e4_dist1000:30000:1000"
    "cond3e4_dist1500:30000:1500"
    "cond3e4_dist2000:30000:2000"
)
BATCH3=(
    "cond1e5_dist1000:100000:1000"
    "cond1e5_dist1500:100000:1500"
    "cond1e5_dist2000:100000:2000"
)

GIT_COMMIT=$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo "unknown")

# ─── make_config_snapshot ─────────────────────────────────────────────────────
# The binary ALWAYS prepends the config file's directory to relative_config_imu,
# so an absolute path in that field gets double-concatenated and fails.
# Fix: copy the kalibr yamls into the snapshot's directory, then leave the
# relative_config_imu value as "kalibr_imu_chain.yaml" (just the filename).
make_config_snapshot() {
    local cond="$1" dist="$2" dst_dir="$3"
    local cfg="$dst_dir/config_snapshot.yaml"

    # Copy kalibr files so the binary can resolve them relative to snapshot dir
    cp "$CONFIG_DIR/kalibr_imu_chain.yaml"    "$dst_dir/kalibr_imu_chain.yaml"
    cp "$CONFIG_DIR/kalibr_imucam_chain.yaml"  "$dst_dir/kalibr_imucam_chain.yaml"

    # Only patch the fi_ parameters; leave relative_config_* as bare filenames
    sed \
        -e "s/^fi_max_cond_number:[[:space:]]*.*/fi_max_cond_number: ${cond}.0/" \
        -e "s/^fi_max_dist:[[:space:]]*.*/fi_max_dist: ${dist}.0/" \
        "$BASE_CONFIG" > "$cfg"
}

# ─── classify_status ─────────────────────────────────────────────────────────
classify_status() {
    local name="$1" dir="$2" ec="$3"
    local traj="$dir/traj.txt"
    local log="$dir/stdout_stderr.log"

    # Always write exit_code.txt
    echo "$ec" > "$dir/exit_code.txt"

    # CRASH: non-zero exit
    if [[ "$ec" -ne 0 ]]; then
        {
            printf "exit_code=%s\n\n" "$ec"
            printf "=== COMMAND ===\n"; cat "$dir/command.txt" 2>/dev/null
            printf "\n=== FIRST 80 LINES ===\n"; head -80 "$log" 2>/dev/null
            printf "\n=== LAST 80 LINES ===\n";  tail -80 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
        echo "  [$name] STATUS_CRASH  exit=$ec"
        echo "  First error line:"
        grep -m3 -i "error\|unable\|fatal\|assert\|abort" "$log" 2>/dev/null \
            | head -3 | sed 's/^/    /' || true
        return
    fi

    # CRASH: traj.txt missing or empty
    if [[ ! -s "$traj" ]]; then
        {
            printf "exit_code=%s  traj.txt: missing or empty\n\n" "$ec"
            printf "=== FIRST 80 LINES ===\n"; head -80 "$log" 2>/dev/null
            printf "\n=== LAST 80 LINES ===\n";  tail -80 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
        echo "  [$name] STATUS_CRASH  exit=$ec  traj.txt empty/missing"
        echo "  Last log lines:"
        tail -5 "$log" 2>/dev/null | sed 's/^/    /' || true
        return
    fi

    # Parse traj with awk (single pass)
    local parse_result
    parse_result=$(awk '
        /^#/ { next }
        NF >= 3 {
            t  = $1 + 0
            xy = sqrt($2*$2 + $3*$3)
            if (xy > max_xy) max_xy = xy
            if (xy > 10000 && t10k == "") t10k = t
            if (xy > 1000  && t1k  == "") t1k  = t
            final_t = t; n++
        }
        END { printf "final_t=%s max_xy=%.0f t1k=%s t10k=%s n=%d\n",
              final_t, max_xy,
              (t1k==""  ? "none" : t1k),
              (t10k=="" ? "none" : t10k), n }
    ' "$traj")

    local final_t max_xy time_gt_1km time_gt_10km n_lines
    final_t=$(echo   "$parse_result" | grep -o 'final_t=[^ ]*'  | cut -d= -f2)
    max_xy=$(echo    "$parse_result" | grep -o 'max_xy=[^ ]*'   | cut -d= -f2)
    time_gt_1km=$(echo  "$parse_result" | grep -o 't1k=[^ ]*'  | cut -d= -f2)
    time_gt_10km=$(echo "$parse_result" | grep -o 't10k=[^ ]*' | cut -d= -f2)
    n_lines=$(echo   "$parse_result" | grep -o 'n=[^ ]*'        | cut -d= -f2)

    local is_diverged is_partial
    is_diverged=$(python3 -c "print(1 if float('${max_xy:-0}') > 10000 else 0)" 2>/dev/null || echo 0)
    is_partial=$(python3  -c "print(1 if float('${final_t:-0}') < 2500  else 0)" 2>/dev/null || echo 0)

    if [[ "$is_diverged" == "1" ]]; then
        printf "max_xy=%.0fm  final_t=%ss  lines=%s  xy>10km_at=%ss\n" \
               "${max_xy}" "${final_t}" "${n_lines}" "${time_gt_10km}" \
               > "$dir/STATUS_DIVERGED.txt"
        echo "  [$name] STATUS_DIVERGED  max_xy=${max_xy}m  first_10km@t=${time_gt_10km}s  lines=${n_lines}"
    elif [[ "$is_partial" == "1" ]]; then
        printf "final_t=%ss (< 2500 expected)  lines=%s  max_xy=%.0fm\n" \
               "${final_t}" "${n_lines}" "${max_xy}" \
               > "$dir/STATUS_PARTIAL.txt"
        echo "  [$name] STATUS_PARTIAL  final_t=${final_t}s  lines=${n_lines}"
    else
        printf "final_t=%ss  lines=%s  max_xy=%.0fm\n" \
               "${final_t}" "${n_lines}" "${max_xy}" \
               > "$dir/STATUS_OK.txt"
        echo "  [$name] STATUS_OK  final_t=${final_t}s  max_xy=${max_xy}m  lines=${n_lines}"
    fi
}

# ─── run_one ──────────────────────────────────────────────────────────────────
run_one() {
    local entry="$1"
    local name="${entry%%:*}"
    local rest="${entry#*:}"
    local cond="${rest%%:*}"
    local dist="${rest#*:}"
    local out_dir="$OUTPUT_BASE/$name"

    mkdir -p "$out_dir"

    make_config_snapshot "$cond" "$dist" "$out_dir"
    local cfg="$out_dir/config_snapshot.yaml"
    echo "$GIT_COMMIT" > "$out_dir/git_commit.txt"

    local -a cmd=(
        "$BINARY"
        --config          "$cfg"
        --dataset         "$DATASET"
        --gps             "$GPS"
        --gps-time-offset "$GPS_TIME_OFFSET"
        --start-time      "$START_TIME"
        --init-from-fc    "$FC_INIT"
        --vio-yaw-gauge-mode "$YAW_MODE"
        --init-bg-sigma   "$BG_SIGMA"
        --gps-alt-update
        --gps-alt-sigma        2.0
        --gps-alt-min-pzz      0.01
        --gps-alt-max-res      80
        --gps-alt-min-t-after-init 10
        --gps-alt-guard-dxy    0.5
        --gps-alt-guard-kxy    5.0
        --viz-fast
        --dash-every 5
        --output "$out_dir/traj.txt"
    )

    # Record command (one flag per line for readability)
    printf '%s \\\n  ' "${cmd[@]}" > "$out_dir/command.txt"
    printf '\n' >> "$out_dir/command.txt"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo ""
        echo "[DRY-RUN] $name  cond=$cond  dist=$dist"
        echo "  snapshot: $cfg"
        echo "  output:   $out_dir"
        printf '  cmd: '; printf '%s ' "${cmd[@]}"; echo ""
        # Verify snapshot values
        echo "  snapshot fi lines:"
        grep -E "fi_max_cond_number|fi_max_dist|relative_config" "$cfg" | sed 's/^/    /'
        return 0
    fi

    echo ">>> [$(date +%T)] Starting $name  cond=$cond  dist=$dist"
    ec=0
    "${cmd[@]}" > "$out_dir/stdout_stderr.log" 2>&1 || ec=$?
    cp "$out_dir/stdout_stderr.log" "$out_dir/log.txt"
    classify_status "$name" "$out_dir" "$ec"
    echo "    [$(date +%T)] Done $name"
}

# ─── eval_one ─────────────────────────────────────────────────────────────────
eval_one() {
    local entry="$1"
    local name="${entry%%:*}"
    local out_dir="$OUTPUT_BASE/$name"
    local eval_out="$out_dir/eval"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo "  [DRY-RUN] eval $name"
        return 0
    fi

    if [[ ! -s "$out_dir/traj.txt" ]]; then
        echo "  [eval] Skipping $name — traj.txt missing or empty"
        return 0
    fi

    mkdir -p "$eval_out"
    echo "  [eval] $name"
    python3 "$EVAL_SCRIPT" \
        --t0    "$EVAL_T0"    \
        --until "$EVAL_UNTIL" \
        --dir-a "$out_dir"    \
        --dir-b "$out_dir"    \
        --gps   "$GPS"        \
        --imu   "$IMU"        \
        --out   "$eval_out"   \
        --yaw-align-mode start_yaw \
        2>&1 | tail -6
}

# ─── run_batch ────────────────────────────────────────────────────────────────
# Returns 1 (fail-fast) if every run in the batch crashed (no traj.txt).
run_batch() {
    local batch_label="$1"; shift
    echo ""
    echo "══════════ $batch_label ══════════"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        for entry in "$@"; do run_one  "$entry"; done
        echo ""
        for entry in "$@"; do eval_one "$entry"; done
        return 0
    fi

    # Launch all 3 in parallel
    local pids=()
    for entry in "$@"; do
        run_one "$entry" &
        pids+=($!)
    done

    # Wait for all; collect per-pid exit codes
    local batch_ec=0
    for pid in "${pids[@]}"; do
        wait "$pid" || batch_ec=1
    done
    echo "  ── $batch_label runs finished ──"

    # Fail-fast: if every run in this batch produced no traj.txt, stop here
    local n_traj=0
    for entry in "$@"; do
        local name="${entry%%:*}"
        [[ -s "$OUTPUT_BASE/$name/traj.txt" ]] && n_traj=$((n_traj+1))
    done
    if [[ "$n_traj" -eq 0 ]]; then
        echo ""
        echo "FATAL: All ${#@} runs in '$batch_label' produced no traj.txt."
        echo "  Check STATUS_CRASH.txt and stdout_stderr.log in each run dir."
        echo "  First crash log:"
        local first_name="${1%%:*}"
        head -30 "$OUTPUT_BASE/$first_name/STATUS_CRASH.txt" 2>/dev/null | sed 's/^/  /' || true
        echo ""
        echo "Stopping — fix the error before proceeding to the next batch."
        exit 1
    fi

    for entry in "$@"; do
        eval_one "$entry"
    done
}

# ─── generate_summary ─────────────────────────────────────────────────────────
generate_summary() {
    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo ""; echo "[DRY-RUN] Would write: $OUTPUT_BASE/summary.csv"; return 0
    fi
    echo ""; echo "══════════ Generating summary.csv ══════════"

    python3 - <<'PYEOF'
import sys, os, csv, math, re

base = "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/feature_gate_ablation_20260607"
RUNS = [
    ("cond1e4_dist1000",  10000, 1000),
    ("cond1e4_dist1500",  10000, 1500),
    ("cond1e4_dist2000",  10000, 2000),
    ("cond3e4_dist1000",  30000, 1000),
    ("cond3e4_dist1500",  30000, 1500),
    ("cond3e4_dist2000",  30000, 2000),
    ("cond1e5_dist1000", 100000, 1000),
    ("cond1e5_dist1500", 100000, 1500),
    ("cond1e5_dist2000", 100000, 2000),
]
FIELDS = [
    "run_name","fi_max_cond_number","fi_max_dist","status",
    "final_t","traj_lines",
    "gps_z_accept_count","gps_z_reject_count","last_gps_z_accept_t",
    "xy_full_rms_m","xy_early_rms_m","xy_late_rms_m",
    "xy_max_m","xy_final_m",
    "yaw_rms_deg","yaw_max_deg",
    "nan_inf","neg_cov",
    "time_xy_gt_1km","time_xy_gt_10km",
]

def get_status(d):
    for s in ("STATUS_OK","STATUS_PARTIAL","STATUS_DIVERGED","STATUS_CRASH"):
        if os.path.exists(f"{d}/{s}.txt"): return s
    return "UNKNOWN"

def parse_traj(d):
    p = f"{d}/traj.txt"
    if not os.path.exists(p): return {}
    rows = [l.split() for l in open(p) if not l.startswith('#') and l.strip()]
    if not rows: return {}
    try: final_t = float(rows[-1][0])
    except: return {}
    max_xy=0.0; final_xy=0.0; t1k=None; t10k=None; has_nan=False
    for r in rows:
        if len(r)<3: continue
        try: t,x,y = float(r[0]),float(r[1]),float(r[2])
        except: has_nan=True; continue
        if not (math.isfinite(x) and math.isfinite(y)): has_nan=True; continue
        xy=math.sqrt(x*x+y*y)
        if xy>max_xy: max_xy=xy
        if t1k  is None and xy>1000:  t1k=t
        if t10k is None and xy>10000: t10k=t
    try: final_xy=math.sqrt(float(rows[-1][1])**2+float(rows[-1][2])**2)
    except: pass
    return dict(n=len(rows),final_t=final_t,max_xy=max_xy,final_xy=final_xy,
                t1k=t1k,t10k=t10k,has_nan=has_nan)

def parse_gpsz(d):
    logp=f"{d}/log.txt"
    if not os.path.exists(logp): return 0,0,""
    acc=0; rej=0; last=""
    for line in open(logp,encoding='utf-8',errors='ignore'):
        m=re.search(r'GPS-ALT-EVAL.*status=(\w+).*t_cam=([\d.]+)',line)
        if not m: continue
        if m.group(1)=='ACC': acc+=1; last=m.group(2)
        else: rej+=1
    return acc,rej,last

def parse_eval(d):
    p=f"{d}/eval/metrics_summary.csv"
    if not os.path.exists(p): return {}
    out={}
    for row in csv.reader(open(p)):
        if len(row)<2: continue
        try: out[row[0].strip()]=float(row[1])
        except: out[row[0].strip()]=row[1].strip()
    return out

rows_out=[]
for (name,cond,dist) in RUNS:
    d=f"{base}/{name}"
    status=get_status(d); ti=parse_traj(d); acc,rej,last=parse_gpsz(d); em=parse_eval(d)
    rows_out.append({
        "run_name":name,"fi_max_cond_number":cond,"fi_max_dist":dist,"status":status,
        "final_t":ti.get("final_t",""),"traj_lines":ti.get("n",""),
        "gps_z_accept_count":acc,"gps_z_reject_count":rej,"last_gps_z_accept_t":last,
        "xy_full_rms_m":em.get("xy_ate_rms_m",""),
        "xy_early_rms_m":"",
        "xy_late_rms_m":em.get("xy_late_rms_m",""),
        "xy_max_m":ti.get("max_xy",em.get("xy_ate_max_m","")),
        "xy_final_m":ti.get("final_xy",em.get("xy_ate_final_m","")),
        "yaw_rms_deg":em.get("yaw_err_rms_deg",""),"yaw_max_deg":em.get("yaw_err_max_deg",""),
        "nan_inf":1 if ti.get("has_nan") else 0,"neg_cov":em.get("neg_cov_warn",""),
        "time_xy_gt_1km":ti.get("t1k",""),"time_xy_gt_10km":ti.get("t10k",""),
    })

out_csv=f"{base}/summary.csv"
with open(out_csv,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=FIELDS); w.writeheader(); w.writerows(rows_out)

print(f"Written: {out_csv}\n")
print(f"{'run_name':<25} {'status':<16} {'final_t':>9} {'max_xy_m':>12} {'yaw_rms':>9}")
print("-"*75)
for r in rows_out:
    print(f"{r['run_name']:<25} {r['status']:<16} "
          f"{str(r['final_t']):>9} {str(r['xy_max_m']):>12} {str(r['yaw_rms_deg']):>9}")
PYEOF
}

# ─── Main ─────────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════════"
echo "  Fly2 Feature-Gate Ablation: fi_max_cond_number × fi_max_dist"
echo "  Fixed: init_bg_sigma=$BG_SIGMA  mode=$YAW_MODE  GPS-Z ON"
echo "  Binary: $BINARY"
echo "  Commit: $GIT_COMMIT"
echo "  Output: $OUTPUT_BASE"
[[ "$DRY_RUN"  -eq 1 ]] && echo "  *** DRY-RUN MODE ***"
[[ "$MODE" == "single" ]] && echo "  *** SINGLE-RUN MODE: $SINGLE_RUN ***"
echo "══════════════════════════════════════════════════════════════════"
echo ""

# Pre-flight checks
if [[ "$DRY_RUN" -eq 0 ]]; then
    [[ -x "$BINARY"      ]] || { echo "ERROR: binary missing: $BINARY";      exit 1; }
    [[ -f "$BASE_CONFIG" ]] || { echo "ERROR: config missing: $BASE_CONFIG"; exit 1; }
    [[ -d "$DATASET"     ]] || { echo "ERROR: dataset missing: $DATASET";   exit 1; }
    [[ -f "$GPS"         ]] || { echo "ERROR: GPS missing: $GPS";           exit 1; }
    [[ -f "$FC_INIT"     ]] || { echo "ERROR: FC init missing: $FC_INIT";   exit 1; }
    [[ -f "$CONFIG_DIR/kalibr_imu_chain.yaml"    ]] \
        || { echo "ERROR: kalibr_imu_chain.yaml missing in $CONFIG_DIR"; exit 1; }
    [[ -f "$CONFIG_DIR/kalibr_imucam_chain.yaml" ]] \
        || { echo "ERROR: kalibr_imucam_chain.yaml missing in $CONFIG_DIR"; exit 1; }
    mkdir -p "$OUTPUT_BASE"
    echo "Pre-flight checks passed."
fi

if [[ "$MODE" == "single" ]]; then
    # Find the matching entry in the matrix
    all_entries=("${BATCH1[@]}" "${BATCH2[@]}" "${BATCH3[@]}")
    matched=""
    for entry in "${all_entries[@]}"; do
        [[ "${entry%%:*}" == "$SINGLE_RUN" ]] && matched="$entry" && break
    done
    if [[ -z "$matched" ]]; then
        echo "ERROR: unknown run name '$SINGLE_RUN'"
        echo "Valid names:"
        for e in "${all_entries[@]}"; do echo "  ${e%%:*}"; done
        exit 1
    fi
    [[ "$DRY_RUN" -eq 0 ]] && mkdir -p "$OUTPUT_BASE"
    run_one  "$matched"
    eval_one "$matched"
    echo ""
    echo "Single-run complete: $SINGLE_RUN"
    exit 0
fi

# Full matrix: 3 batches sequential, 3 parallel each
run_batch "Batch 1 (cond=1e4)" "${BATCH1[@]}"
run_batch "Batch 2 (cond=3e4)" "${BATCH2[@]}"
run_batch "Batch 3 (cond=1e5)" "${BATCH3[@]}"
generate_summary

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  Ablation complete.  Results: $OUTPUT_BASE"
echo "══════════════════════════════════════════════════════════════════"
