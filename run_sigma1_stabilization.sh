#!/usr/bin/env bash
# run_sigma1_stabilization.sh
#
# Sigma=1 stabilisation ablation — fly1 / fly3 / fly4
# Purpose: determine whether sigma_px=1 can be stabilised without per-flight tuning.
#
# Groups (all × 3 flights unless --flight filter):
#   A  baseline     sigma=2 chi2=1 dist=2000 clones=11 (shared baseline)
#   B  raw_sigma1   sigma=1 chi2=1 dist=2000 clones=11
#   C  chi2comp3    sigma=1 chi2=3 dist=2000 clones=11
#   D  chi2comp5    sigma=1 chi2=5 dist=2000 clones=11  [diagnostic]
#   E  altdist      sigma=1 chi2=1 dist=ALT  clones=11
#   F  altdist_c3   sigma=1 chi2=3 dist=ALT  clones=11
#   G  morefeat     sigma=1 chi2=1 dist=2000 clones=11  num_pts=600 fast=15 minpx=12
#   H  morefeat_c3  sigma=1 chi2=3 dist=ALT  clones=11  num_pts=600 fast=15 minpx=12
#   I  clones15     sigma=1 chi2=3 dist=ALT  clones=15  num_pts=600 fast=15 minpx=12
#   J  base_cl15    sigma=2 chi2=1 dist=2000 clones=15  [optional secondary]
#
# Fixed across all runs:
#   yaw = oc_postchi2_current_gauge   fi_max_cond = 1e4 (YAML default)
#   init_bg_sigma = 0.003             gyro_nd = 0.005 (YAML default)
#   GPS-Z ON with guard flags
#
# ALT-based fi_max_dist rule: max(500, 2.5 * cruise_altitude_m)
#   fly1 ~200 m  -> 500
#   fly3 ~200 m  -> 500
#   fly4 ~400 m  -> 1000
#
# Per-run diagnostics: --diag-csv-path writes per-frame chi2_acc / chi2_rej counts.
#
# Usage:
#   bash run_sigma1_stabilization.sh              # all groups, all flights
#   bash run_sigma1_stabilization.sh --dry-run
#   bash run_sigma1_stabilization.sh --group A
#   bash run_sigma1_stabilization.sh --group B
#   bash run_sigma1_stabilization.sh --flight fly3
#   bash run_sigma1_stabilization.sh --no-optional   # skip I / J
#   bash run_sigma1_stabilization.sh --only-optional # run only I / J

set -uo pipefail

# ─── CLI ──────────────────────────────────────────────────────────────────────
DRY_RUN=0
GROUP_FILTER=""
FLIGHT_FILTER=""
NO_OPTIONAL=0
ONLY_OPTIONAL=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)       DRY_RUN=1 ;;
        --group)         GROUP_FILTER="${2:-}"; shift ;;
        --flight)        FLIGHT_FILTER="${2:-}"; shift ;;
        --no-optional)   NO_OPTIONAL=1 ;;
        --only-optional) ONLY_OPTIONAL=1 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

[[ -n "$GROUP_FILTER"  ]] && [[ ! "$GROUP_FILTER"  =~ ^[A-J]$ ]] && {
    echo "ERROR: --group must be A-J"; exit 1; }
[[ -n "$FLIGHT_FILTER" ]] && [[ ! "$FLIGHT_FILTER" =~ ^(fly1|fly3|fly4)$ ]] && {
    echo "ERROR: --flight must be fly1|fly3|fly4"; exit 1; }

# ─── Paths and constants ───────────────────────────────────────────────────────
REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
EVAL_SCRIPT="$REPO/eval_stage.py"
YAW_MODE="oc_postchi2_current_gauge"
BG_SIGMA="0.003"
RUN_TAG="sigma1_stab_20260610"
GIT_COMMIT=$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo "unknown")

GPS_Z_FLAGS=(
    --gps-alt-update
    --gps-alt-sigma        2.0
    --gps-alt-min-pzz      0.01
    --gps-alt-max-res      80
    --gps-alt-min-t-after-init 10
    --gps-alt-guard-dxy    0.5
    --gps-alt-guard-kxy    5.0
)

# ─── Per-flight altitude-based fi_max_dist ────────────────────────────────────
# Rule: max(500, 2.5 * cruise_altitude_m)
declare -A F_ALT_FIDIST
F_ALT_FIDIST[fly1]="500.0"    # ~200 m cruise
F_ALT_FIDIST[fly3]="500.0"    # ~200 m cruise
F_ALT_FIDIST[fly4]="1000.0"   # ~400 m cruise

# ─── Per-flight parameters ────────────────────────────────────────────────────
declare -A F_DATASET F_CONFIG F_CFG_DIR F_GPS F_FC_INIT
declare -A F_START F_EVAL_T0 F_EVAL_UNTIL F_GPS_OFFSET F_OUTPUT_BASE F_IMU

F_DATASET[fly1]="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"
F_CONFIG[fly1]="$REPO/config/d455_fly1/estimator_config.yaml"
F_CFG_DIR[fly1]="$REPO/config/d455_fly1"
F_GPS[fly1]="$REPO/config/d455_fly1/fc_gps_cam_time.csv"
F_FC_INIT[fly1]="$REPO/config/d455_fly1/fc_init_state_930.csv"
F_START[fly1]="930"
F_EVAL_T0[fly1]="930"
F_EVAL_UNTIL[fly1]="1740"
F_GPS_OFFSET[fly1]="0"
F_OUTPUT_BASE[fly1]="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/$RUN_TAG"
F_IMU[fly1]="${F_DATASET[fly1]}/imu0/data.csv"

F_DATASET[fly3]="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946"
F_CONFIG[fly3]="$REPO/config/d455_fly2/estimator_config.yaml"
F_CFG_DIR[fly3]="$REPO/config/d455_fly2"
F_GPS[fly3]="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv"
F_FC_INIT[fly3]="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv"
F_START[fly3]="618"
F_EVAL_T0[fly3]="618"
F_EVAL_UNTIL[fly3]="1600"
F_GPS_OFFSET[fly3]="0"
F_OUTPUT_BASE[fly3]="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/$RUN_TAG"
F_IMU[fly3]="${F_DATASET[fly3]}/imu0/data.csv"

F_DATASET[fly4]="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549"
F_CONFIG[fly4]="$REPO/config/d455_fly2/estimator_config.yaml"
F_CFG_DIR[fly4]="$REPO/config/d455_fly2"
F_GPS[fly4]="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
F_FC_INIT[fly4]="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/canonical_offsetm202p2_start924p4/fc_init_state_924p4_offsetm202p2.csv"
F_START[fly4]="924.4"
F_EVAL_T0[fly4]="924.4"
F_EVAL_UNTIL[fly4]="2816"
F_GPS_OFFSET[fly4]="0"
F_OUTPUT_BASE[fly4]="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/$RUN_TAG"
F_IMU[fly4]="${F_DATASET[fly4]}/imu0/data.csv"

# ─── make_config_snapshot ─────────────────────────────────────────────────────
# Args: flight dst_dir sigma_px chi2_mult fi_max_dist max_clones num_pts fast_threshold min_px_dist
# Pass "" to keep the config file's original value for that field.
make_config_snapshot() {
    local flight="$1" dst_dir="$2"
    local sigma_px="$3" chi2_mult="$4" fi_max_dist="$5" max_clones_v="$6"
    local num_pts_v="$7" fast_thr="$8" min_px_v="$9"
    local base_config="${F_CONFIG[$flight]}"
    local cfg_dir="${F_CFG_DIR[$flight]}"
    local cfg="$dst_dir/config_snapshot.yaml"

    cp "$cfg_dir/kalibr_imu_chain.yaml"   "$dst_dir/"
    cp "$cfg_dir/kalibr_imucam_chain.yaml" "$dst_dir/"

    awk \
        -v sigma="$sigma_px" \
        -v chi2="$chi2_mult" \
        -v fidist="$fi_max_dist" \
        -v clones="$max_clones_v" \
        -v npts="$num_pts_v" \
        -v fast="$fast_thr" \
        -v minpx="$min_px_v" \
        '
        /^up_msckf_sigma_px:/       { if (sigma  != "") { print "up_msckf_sigma_px: "       sigma;   next } }
        /^up_msckf_chi2_multipler:/ { if (chi2   != "") { print "up_msckf_chi2_multipler: " chi2;    next } }
        /^fi_max_dist:/             { if (fidist != "") { print "fi_max_dist: "             fidist;  next } }
        /^max_clones:/              { if (clones != "") { print "max_clones: "              clones;  next } }
        /^num_pts:/                 { if (npts   != "") { print "num_pts: "                 npts;    next } }
        /^fast_threshold:/          { if (fast   != "") { print "fast_threshold: "          fast;    next } }
        /^min_px_dist:/             { if (minpx  != "") { print "min_px_dist: "             minpx;   next } }
        { print }
        ' "$base_config" > "$cfg"
}

# ─── classify_status ──────────────────────────────────────────────────────────
classify_status() {
    local name="$1" dir="$2" ec="$3"
    local traj="$dir/traj.txt" log="$dir/log.txt"
    echo "$ec" > "$dir/exit_code.txt"
    rm -f "$dir"/STATUS_*.txt

    if [[ "$ec" -ne 0 ]]; then
        { printf "exit_code=%s\n\n=== FIRST 80 LINES ===\n" "$ec"
          head -80 "$log" 2>/dev/null
          printf "\n=== LAST 80 LINES ===\n"
          tail -80 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
        echo "  [$name] CRASH  exit=$ec"
        return
    fi
    if [[ ! -s "$traj" ]]; then
        printf "exit_code=0  traj missing\n" > "$dir/STATUS_CRASH.txt"
        tail -20 "$log" 2>/dev/null >> "$dir/STATUS_CRASH.txt"
        echo "  [$name] CRASH  traj missing"
        return
    fi

    local parse
    parse=$(awk 'NF>=3 && !/^#/ {
        x=$2+0; y=$3+0; t=$1+0
        xy=sqrt(x*x+y*y)
        if(xy>max_xy) max_xy=xy
        if(xy>10000 && t10k=="") t10k=t
        final_t=t; n++
    } END { printf "n=%d final_t=%.3f max_xy=%.1f t10k=%s\n",
            n, final_t, max_xy, (t10k==""?"none":t10k) }' "$traj")

    local n final_t max_xy t10k
    n=$(      echo "$parse" | grep -o 'n=[^ ]*'       | cut -d= -f2)
    final_t=$(echo "$parse" | grep -o 'final_t=[^ ]*' | cut -d= -f2)
    max_xy=$( echo "$parse" | grep -o 'max_xy=[^ ]*'  | cut -d= -f2)
    t10k=$(   echo "$parse" | grep -o 't10k=[^ ]*'    | cut -d= -f2)

    local is_div; is_div=$(python3 -c "print(1 if ${max_xy:-0} > 10000 else 0)" 2>/dev/null || echo 0)
    if [[ "$is_div" == "1" ]]; then
        printf "max_xy=%.0fm final_t=%ss lines=%s first_10km@t=%s\n" \
               "$max_xy" "$final_t" "$n" "$t10k" > "$dir/STATUS_DIVERGED.txt"
        echo "  [$name] DIVERGED  max_xy=${max_xy}m  first_10km@t=${t10k}s"
    else
        printf "max_xy=%.0fm final_t=%ss lines=%s\n" "$max_xy" "$final_t" "$n" \
               > "$dir/STATUS_OK.txt"
        echo "  [$name] OK  max_xy=${max_xy}m  final_t=${final_t}s  poses=$n"
    fi
}

# ─── run_one ──────────────────────────────────────────────────────────────────
# Args: flight run_name sigma chi2 fidist clones npts fast minpx group_id
run_one() {
    local flight="$1" name="$2"
    local sigma="$3" chi2="$4" fidist="$5" clones="$6"
    local npts="$7" fast="$8" minpx="$9" group="${10}"
    local out_dir="${F_OUTPUT_BASE[$flight]}/$name"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo ""
        echo "  [DRY-RUN] $flight/$name  (group $group)"
        printf "    sigma=%-4s chi2=%-2s fidist=%-7s clones=%-3s npts=%-5s fast=%-4s minpx=%s\n" \
               "${sigma:-(cfg)}" "${chi2:-(cfg)}" "${fidist:-(cfg)}" \
               "${clones:-(cfg)}" "${npts:-(cfg)}" "${fast:-(cfg)}" "${minpx:-(cfg)}"
        # Show what the config snapshot would contain
        mkdir -p "$out_dir"
        make_config_snapshot "$flight" "$out_dir" \
            "$sigma" "$chi2" "$fidist" "$clones" "$npts" "$fast" "$minpx"
        grep -E "up_msckf_sigma_px|up_msckf_chi2|fi_max_dist|max_clones|num_pts|fast_threshold|min_px_dist" \
             "$out_dir/config_snapshot.yaml" | sed 's/^/    /'
        return 0
    fi

    mkdir -p "$out_dir"
    make_config_snapshot "$flight" "$out_dir" \
        "$sigma" "$chi2" "$fidist" "$clones" "$npts" "$fast" "$minpx"
    local cfg="$out_dir/config_snapshot.yaml"
    echo "$GIT_COMMIT" > "$out_dir/git_commit.txt"
    echo "$group" > "$out_dir/group_id.txt"

    local -a cmd=(
        "$BINARY"
        --config    "$cfg"
        --dataset   "${F_DATASET[$flight]}"
        --gps       "${F_GPS[$flight]}"
        --gps-time-offset "${F_GPS_OFFSET[$flight]}"
        --start-time "${F_START[$flight]}"
        --init-from-fc "${F_FC_INIT[$flight]}"
        --vio-yaw-gauge-mode "$YAW_MODE"
        --init-bg-sigma "$BG_SIGMA"
        "${GPS_Z_FLAGS[@]}"
        --diag-csv "$out_dir/diag.csv"
        --viz-fast --dash-every 5
        --output "$out_dir/traj.txt"
    )

    printf '%s \\\n  ' "${cmd[@]}" > "$out_dir/command.txt"
    printf '\n' >> "$out_dir/command.txt"

    echo ">>> [$(date +%T)] $flight/$name"
    local ec=0
    "${cmd[@]}" > "$out_dir/log.txt" 2>&1 || ec=$?
    classify_status "$name" "$out_dir" "$ec"
    echo "    [$(date +%T)] done $flight/$name"
}

# ─── eval_one ─────────────────────────────────────────────────────────────────
eval_one() {
    local flight="$1" name="$2"
    local out_dir="${F_OUTPUT_BASE[$flight]}/$name"
    local eval_out="$out_dir/eval"
    [[ "$DRY_RUN" -eq 1 ]] && { echo "  [DRY-RUN] eval $flight/$name"; return 0; }
    [[ ! -s "$out_dir/traj.txt" ]] && { echo "  [eval] skip $flight/$name (no traj)"; return 0; }

    mkdir -p "$eval_out"
    python3 "$EVAL_SCRIPT" \
        --t0    "${F_EVAL_T0[$flight]}"    \
        --until "${F_EVAL_UNTIL[$flight]}" \
        --dir-a "$out_dir"                 \
        --dir-b "$out_dir"                 \
        --gps   "${F_GPS[$flight]}"        \
        --imu   "${F_IMU[$flight]}"        \
        --out   "$eval_out"                \
        --yaw-align-mode start_yaw         \
        2>&1 | tail -6 || true
}

# ─── run_batch ────────────────────────────────────────────────────────────────
# Up to 4 parallel runs, then eval each sequentially.
# Entries: "flight:name:sigma:chi2:fidist:clones:npts:fast:minpx:group"
run_batch() {
    local label="$1"; shift
    local entries=("$@")
    echo ""
    echo "  ════ Batch: $label (${#entries[@]} runs) ════"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        for entry in "${entries[@]}"; do
            IFS=: read -r _fl _nm _sig _chi _fd _cl _np _fa _mp _grp <<< "$entry"
            run_one "$_fl" "$_nm" "$_sig" "$_chi" "$_fd" "$_cl" "$_np" "$_fa" "$_mp" "$_grp"
        done
    else
        local pids=()
        for entry in "${entries[@]}"; do
            IFS=: read -r _fl _nm _sig _chi _fd _cl _np _fa _mp _grp <<< "$entry"
            run_one "$_fl" "$_nm" "$_sig" "$_chi" "$_fd" "$_cl" "$_np" "$_fa" "$_mp" "$_grp" &
            pids+=($!)
        done
        for pid in "${pids[@]}"; do wait "$pid" || true; done
    fi

    for entry in "${entries[@]}"; do
        IFS=: read -r _fl _nm _sig _chi _fd _cl _np _fa _mp _grp <<< "$entry"
        eval_one "$_fl" "$_nm"
    done
}

# ─── group / flight filters ───────────────────────────────────────────────────
group_applies() { [[ -z "$GROUP_FILTER" || "$GROUP_FILTER" == "$1" ]]; }
flight_applies() { [[ -z "$FLIGHT_FILTER" || "$FLIGHT_FILTER" == "$1" ]]; }

is_optional() {
    [[ "$1" == "I" || "$1" == "J" ]]
}

should_run_group() {
    local g="$1"
    is_optional "$g" && [[ "$NO_OPTIONAL" -eq 1 ]] && return 1
    ! is_optional "$g" && [[ "$ONLY_OPTIONAL" -eq 1 ]] && return 1
    group_applies "$g"
}

# ─── build_batch_for_group ────────────────────────────────────────────────────
# Emit run entries for a given group across the requested flights.
# Args: group_id sigma chi2 fidist_key clones npts fast minpx
#   fidist_key: "2000" or "ALT" (substituted per-flight)
build_batch() {
    local g="$1" sigma="$2" chi2="$3" fidist_key="$4" clones="$5"
    local npts="$6" fast="$7" minpx="$8"

    should_run_group "$g" || return 0

    local entries=()
    for flight in fly1 fly3 fly4; do
        flight_applies "$flight" || continue
        local fidist
        if [[ "$fidist_key" == "ALT" ]]; then
            fidist="${F_ALT_FIDIST[$flight]}"
        else
            fidist="$fidist_key"
        fi
        local run_name="${g}_${flight}"
        entries+=("${flight}:${run_name}:${sigma}:${chi2}:${fidist}:${clones}:${npts}:${fast}:${minpx}:${g}")
    done

    [[ "${#entries[@]}" -gt 0 ]] && run_batch "Group $g" "${entries[@]}"
}

# ─── Pre-flight checks ────────────────────────────────────────────────────────
preflight_check() {
    [[ -x "$BINARY" ]] || { echo "ERROR: binary not found: $BINARY"; exit 1; }
    for f in fly1 fly3 fly4; do
        flight_applies "$f" || continue
        [[ -d "${F_DATASET[$f]}"    ]] || { echo "ERROR: dataset missing: ${F_DATASET[$f]}";   exit 1; }
        [[ -f "${F_CONFIG[$f]}"     ]] || { echo "ERROR: config missing: ${F_CONFIG[$f]}";     exit 1; }
        [[ -f "${F_GPS[$f]}"        ]] || { echo "ERROR: GPS missing: ${F_GPS[$f]}";           exit 1; }
        [[ -f "${F_FC_INIT[$f]}"    ]] || { echo "ERROR: FC init missing: ${F_FC_INIT[$f]}";   exit 1; }
        [[ -f "${F_CFG_DIR[$f]}/kalibr_imu_chain.yaml"    ]] \
            || { echo "ERROR: kalibr_imu missing in ${F_CFG_DIR[$f]}"; exit 1; }
        [[ -f "${F_CFG_DIR[$f]}/kalibr_imucam_chain.yaml" ]] \
            || { echo "ERROR: kalibr_imucam missing in ${F_CFG_DIR[$f]}"; exit 1; }
    done
    echo "Pre-flight checks passed."
}

# ─── generate_summary ─────────────────────────────────────────────────────────
generate_summary() {
    [[ "$DRY_RUN" -eq 1 ]] && { echo "[DRY-RUN] summary skipped"; return; }

    python3 - <<'PYEOF'
import csv, math, os, json

flights_cfg = {
    "fly1": "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/sigma1_stab_20260610",
    "fly3": "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/sigma1_stab_20260610",
    "fly4": "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/sigma1_stab_20260610",
}

GROUPS = ["A","B","C","D","E","F","G","H","I","J"]

def get_status(d):
    for s in ("STATUS_OK","STATUS_DIVERGED","STATUS_PARTIAL","STATUS_CRASH"):
        if os.path.exists(f"{d}/{s}.txt"): return s[7:]
    return "NOT_RUN"

def parse_eval(d):
    p = f"{d}/eval/metrics_summary.csv"
    if not os.path.exists(p): return {}
    out = {}
    for row in csv.reader(open(p)):
        if len(row) >= 2:
            try: out[row[0].strip()] = float(row[1])
            except: out[row[0].strip()] = row[1].strip()
    return out

def parse_snapshot(d):
    out = {}
    cfg = f"{d}/config_snapshot.yaml"
    if not os.path.exists(cfg): return out
    for l in open(cfg):
        for key in ("up_msckf_sigma_px","up_msckf_chi2_multipler","fi_max_dist",
                    "max_clones","num_pts","fast_threshold","min_px_dist"):
            if l.strip().startswith(key+":"):
                try: out[key] = float(l.split(":",1)[1].strip())
                except: pass
    return out

def parse_diag(d):
    """Extract chi2 stats from diag.csv: mean accepted, mean rejected, rejection rate,
    starvation events (chi2_acc < 3), first starvation timestamp."""
    p = f"{d}/diag.csv"
    if not os.path.exists(p): return {}
    rows = list(csv.DictReader(open(p)))
    if not rows: return {}
    # Find columns — handle both n_acc and chi2_acc naming
    acc_col = "n_acc" if "n_acc" in rows[0] else ("chi2_acc" if "chi2_acc" in rows[0] else None)
    rej_col = "chi2_rej" if "chi2_rej" in rows[0] else None
    t_col   = "t"       if "t"       in rows[0] else None
    if not acc_col or not rej_col: return {}

    accs, rejs, ts = [], [], []
    for r in rows:
        try:
            acc = float(r[acc_col]); rej = float(r[rej_col])
            accs.append(acc); rejs.append(rej)
            if t_col and r[t_col]: ts.append(float(r[t_col]))
        except: pass

    if not accs: return {}
    total    = [a+b for a,b in zip(accs,rejs)]
    rej_rate = [b/t if t > 0 else 0 for b,t in zip(rejs,total)]
    stv_idx  = [i for i,a in enumerate(accs) if a < 3]  # starvation: <3 accepted
    first_stv = ts[stv_idx[0]] if stv_idx and ts else (None if not ts else float('nan'))
    return {
        "mean_acc":      sum(accs)/len(accs),
        "mean_rej":      sum(rejs)/len(rejs),
        "mean_rej_rate": sum(rej_rate)/len(rej_rate),
        "min_acc":       min(accs),
        "n_starvation":  len(stv_idx),
        "first_stv_t":   first_stv if stv_idx else None,
        "n_frames":      len(accs),
    }

FIELDS = ["flight","group","run_name",
          "sigma_px","chi2_mult","fi_max_dist","max_clones","num_pts","fast_thr","min_px",
          "status",
          "xy_ate_rms_m","xy_ate_final_m",
          "yaw_err_rms_deg","yaw_err_p95_deg","yaw_err_final_deg",
          "mean_acc","mean_rej","mean_rej_rate","min_acc","n_starvation","first_stv_t"]

rows = []
for flight, base in sorted(flights_cfg.items()):
    if not os.path.exists(base): continue
    for run_name in sorted(os.listdir(base)):
        d = f"{base}/{run_name}"
        if not os.path.isdir(d): continue
        group_id = run_name.split("_")[0] if "_" in run_name else "?"
        st = get_status(d)
        em = parse_eval(d)
        pr = parse_snapshot(d)
        dg = parse_diag(d)
        rows.append({
            "flight": flight, "group": group_id, "run_name": run_name,
            "sigma_px":   pr.get("up_msckf_sigma_px",  ""),
            "chi2_mult":  pr.get("up_msckf_chi2_multipler", ""),
            "fi_max_dist":pr.get("fi_max_dist",          ""),
            "max_clones": pr.get("max_clones",            ""),
            "num_pts":    pr.get("num_pts",               ""),
            "fast_thr":   pr.get("fast_threshold",        ""),
            "min_px":     pr.get("min_px_dist",           ""),
            "status": st,
            "xy_ate_rms_m":      em.get("xy_ate_rms_m",""),
            "xy_ate_final_m":    em.get("xy_ate_final_m",""),
            "yaw_err_rms_deg":   em.get("yaw_err_rms_deg",""),
            "yaw_err_p95_deg":   em.get("yaw_err_p95_deg",""),
            "yaw_err_final_deg": em.get("yaw_err_final_deg",""),
            "mean_acc":     dg.get("mean_acc",""),
            "mean_rej":     dg.get("mean_rej",""),
            "mean_rej_rate":dg.get("mean_rej_rate",""),
            "min_acc":      dg.get("min_acc",""),
            "n_starvation": dg.get("n_starvation",""),
            "first_stv_t":  dg.get("first_stv_t",""),
        })

# Combined CSV
combo = "/mnt/d/vscode_dir/open_vins/docs/reports/sigma1_stab_summary_20260610.csv"
os.makedirs(os.path.dirname(combo), exist_ok=True)
with open(combo,"w",newline="") as f:
    w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(rows)
print(f"Written: {combo}")

# Per-flight CSVs
for flight, base in sorted(flights_cfg.items()):
    if not os.path.exists(base): continue
    out_csv = f"{base}/summary_{flight}.csv"
    frows = [r for r in rows if r["flight"] == flight]
    with open(out_csv,"w",newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(frows)
    print(f"Written: {out_csv}")

# Console table
def fmt(v, w=7, d=1):
    return f"{v:{w}.{d}f}" if isinstance(v, float) else f"{str(v):>{w}}"

print(f"\n{'flight':<5} {'grp':<4} {'σ':>3} {'χ²':>3} {'dist':>6} {'cl':>3} "
      f"{'status':<10} {'ATE_rms':>8} {'yaw_rms':>8} {'acc/upd':>7} {'rej_rt':>7} {'stv_n':>5}")
print("-"*90)
for r in rows:
    sig  = f"{r['sigma_px']:.0f}"    if isinstance(r['sigma_px'],  float) else "-"
    chi2 = f"{r['chi2_mult']:.0f}"   if isinstance(r['chi2_mult'], float) else "-"
    dist = f"{r['fi_max_dist']:.0f}" if isinstance(r['fi_max_dist'],float) else "-"
    cl   = f"{r['max_clones']:.0f}"  if isinstance(r['max_clones'],float) else "-"
    ate  = fmt(r['xy_ate_rms_m'])
    yrms = fmt(r['yaw_err_rms_deg'])
    acc  = f"{r['mean_acc']:.1f}"    if isinstance(r['mean_acc'], float) else "-"
    rrt  = f"{r['mean_rej_rate']:.2f}" if isinstance(r['mean_rej_rate'],float) else "-"
    stv  = str(r['n_starvation'])    if r['n_starvation'] != "" else "-"
    print(f"  {r['flight']:<5} {r['group']:<4} {sig:>3} {chi2:>3} {dist:>6} {cl:>3} "
          f"  {r['status']:<10} {ate} {yrms} {acc:>7} {rrt:>7} {stv:>5}")
PYEOF
}

# ─── Header ───────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════════════"
echo "  Sigma=1 stabilisation ablation — fly1 / fly3 / fly4"
echo "  Commit: $GIT_COMMIT"
echo "  Tag:    $RUN_TAG"
echo "  YAW:    $YAW_MODE    BG_SIGMA: $BG_SIGMA"
[[ "$DRY_RUN"       -eq 1 ]] && echo "  *** DRY-RUN ***"
[[ -n "$GROUP_FILTER"     ]] && echo "  *** GROUP FILTER: $GROUP_FILTER ***"
[[ -n "$FLIGHT_FILTER"    ]] && echo "  *** FLIGHT FILTER: $FLIGHT_FILTER ***"
[[ "$NO_OPTIONAL"   -eq 1 ]] && echo "  *** SKIPPING optional groups I/J ***"
[[ "$ONLY_OPTIONAL" -eq 1 ]] && echo "  *** ONLY optional groups I/J ***"
echo "══════════════════════════════════════════════════════════════════════"

[[ "$DRY_RUN" -eq 0 ]] && preflight_check

if [[ "$DRY_RUN" -eq 0 ]]; then
    for f in fly1 fly3 fly4; do
        flight_applies "$f" && mkdir -p "${F_OUTPUT_BASE[$f]}"
    done
fi

# ─── Groups ───────────────────────────────────────────────────────────────────
# build_batch  GRP  sigma  chi2  fidist  clones  npts   fast  minpx
build_batch    A    2      1     2000    11       ""     ""    ""
build_batch    B    1      1     2000    11       ""     ""    ""
build_batch    C    1      3     2000    11       ""     ""    ""
build_batch    D    1      5     2000    11       ""     ""    ""
build_batch    E    1      1     ALT     11       ""     ""    ""
build_batch    F    1      3     ALT     11       ""     ""    ""
build_batch    G    1      1     2000    11       600    15    12
build_batch    H    1      3     ALT     11       600    15    12
build_batch    I    1      3     ALT     15       600    15    12
build_batch    J    2      1     2000    15       ""     ""    ""

echo ""
echo "All runs complete. Generating summary..."
generate_summary

echo ""
echo "══════════════════════════════════════════════════════════════════════"
echo "  Sigma=1 stabilisation ablation complete."
echo "  Summary: docs/reports/sigma1_stab_summary_20260610.csv"
echo "  Analysis: python3 analysis/sigma1_analysis.py"
echo "══════════════════════════════════════════════════════════════════════"
