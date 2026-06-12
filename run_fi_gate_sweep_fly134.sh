#!/usr/bin/env bash
# run_fi_gate_sweep_fly134.sh
#
# Feature-gate threshold sweep: tighter-than-1e4 condition numbers for low/mid-altitude flights.
# Flights: fly1 (~200m), fly3 (~200m), fly4 (~400m)
# Fixed:   yaw=oc_postchi2_current_gauge  GPS-Z ON  init_bg_sigma=0.003
#
# Ablation axes:
#   fi_max_cond_number ∈ {5000, 3000, 1000}
#   fi_max_dist:  fly1/fly3 ∈ {200, 300, 500, 800}
#                 fly4      ∈ {300, 500, 800, 1000}
#
# Usage:
#   bash run_fi_gate_sweep_fly134.sh                         # all 3 flights, all cond levels
#   bash run_fi_gate_sweep_fly134.sh --dry-run               # print commands only
#   bash run_fi_gate_sweep_fly134.sh --single fly1 cond5e3_dist500
#   bash run_fi_gate_sweep_fly134.sh --flight fly1           # one flight only
#   bash run_fi_gate_sweep_fly134.sh --flight fly3
#   bash run_fi_gate_sweep_fly134.sh --flight fly4

set -uo pipefail

DRY_RUN=0
MODE="all"
SINGLE_FLIGHT=""
SINGLE_RUN=""
FLIGHT_FILTER=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1 ;;
        --single)
            MODE="single"
            SINGLE_FLIGHT="${2:-}"; SINGLE_RUN="${3:-}"
            [[ -z "$SINGLE_FLIGHT" || -z "$SINGLE_RUN" ]] && {
                echo "Usage: --single <fly1|fly3|fly4> <run_name>"; exit 1; }
            shift 2 ;;
        --flight)
            FLIGHT_FILTER="${2:-}"
            [[ "$FLIGHT_FILTER" =~ ^(fly1|fly3|fly4)$ ]] || {
                echo "ERROR: --flight must be fly1, fly3, or fly4"; exit 1; }
            shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

# ─── Common ───────────────────────────────────────────────────────────────────
REPO="/mnt/d/vscode_dir/open_vins"
BINARY="$REPO/build_ov_msckf/run_serial_msckf_ros_free"
EVAL_SCRIPT="$REPO/eval_stage.py"
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
GIT_COMMIT=$(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo "unknown")

# ─── Per-flight configuration ─────────────────────────────────────────────────
declare -A F_DATASET F_CONFIG F_CFG_DIR F_GPS F_FC_INIT F_START F_EVAL_T0 F_EVAL_UNTIL \
           F_GPS_OFFSET F_OUTPUT_BASE F_IMU

F_DATASET[fly1]="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810"
F_CONFIG[fly1]="$REPO/config/d455_fly1/estimator_config.yaml"
F_CFG_DIR[fly1]="$REPO/config/d455_fly1"
F_GPS[fly1]="$REPO/config/d455_fly1/fc_gps_cam_time.csv"
F_FC_INIT[fly1]="$REPO/config/d455_fly1/fc_init_state_930.csv"
F_START[fly1]="930"
F_EVAL_T0[fly1]="930"
F_EVAL_UNTIL[fly1]="1740"
F_GPS_OFFSET[fly1]="0"
F_OUTPUT_BASE[fly1]="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fi_gate_sweep_20260608"
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
F_OUTPUT_BASE[fly3]="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fi_gate_sweep_20260608"
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
F_OUTPUT_BASE[fly4]="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fi_gate_sweep_20260608"
F_IMU[fly4]="${F_DATASET[fly4]}/imu0/data.csv"

# ─── Ablation matrices ────────────────────────────────────────────────────────
# Format: "name:cond:dist"
make_matrix_fly1() {
    local cond="$1"
    local tag; tag=$(cond_tag "$cond")
    echo "${tag}_dist200:${cond}:200"
    echo "${tag}_dist300:${cond}:300"
    echo "${tag}_dist500:${cond}:500"
    echo "${tag}_dist800:${cond}:800"
}
make_matrix_fly3() { make_matrix_fly1 "$1"; }

make_matrix_fly4() {
    local cond="$1"
    local tag; tag=$(cond_tag "$cond")
    echo "${tag}_dist300:${cond}:300"
    echo "${tag}_dist500:${cond}:500"
    echo "${tag}_dist800:${cond}:800"
    echo "${tag}_dist1000:${cond}:1000"
}

cond_tag() {
    case "$1" in
        5000)   echo "cond5e3" ;;
        3000)   echo "cond3e3" ;;
        1000)   echo "cond1e3" ;;
        *)      echo "cond${1}" ;;
    esac
}

# ─── make_config_snapshot ─────────────────────────────────────────────────────
# Copies kalibr yamls to snapshot dir so relative paths resolve correctly.
# Handles fly1 (no fi_max_cond_number line) by inserting after fi_max_baseline,
# and fly2/3/4 (has the line) by replacing it — using two separate awk branches.
make_config_snapshot() {
    local cond="$1" dist="$2" dst_dir="$3" base_config="$4" cfg_dir="$5"
    local cfg="$dst_dir/config_snapshot.yaml"

    cp "$cfg_dir/kalibr_imu_chain.yaml"    "$dst_dir/"
    cp "$cfg_dir/kalibr_imucam_chain.yaml"  "$dst_dir/"

    if grep -q "^fi_max_cond_number:" "$base_config"; then
        # Config already has the line: replace it (and replace fi_max_dist)
        awk -v cond="${cond}.0" -v dist="${dist}.0" '
            /^fi_max_cond_number:[[:space:]]/ { print "fi_max_cond_number: " cond; next }
            /^fi_max_dist:[[:space:]]/        { print "fi_max_dist: " dist; next }
            { print }
        ' "$base_config" > "$cfg"
    else
        # Config has no fi_max_cond_number: insert it after fi_max_baseline
        awk -v cond="${cond}.0" -v dist="${dist}.0" '
            /^fi_max_dist:[[:space:]]/    { print "fi_max_dist: " dist; next }
            /^fi_max_baseline:[[:space:]]/ { print; print "fi_max_cond_number: " cond; next }
            { print }
        ' "$base_config" > "$cfg"
    fi
}

# ─── classify_status ──────────────────────────────────────────────────────────
classify_status() {
    local name="$1" dir="$2" ec="$3"
    local traj="$dir/traj.txt"
    local log="$dir/stdout_stderr.log"

    echo "$ec" > "$dir/exit_code.txt"
    # Remove stale status files from any previous run
    rm -f "$dir"/STATUS_*.txt

    if [[ "$ec" -ne 0 ]]; then
        {
            printf "exit_code=%s\n\n=== COMMAND ===\n" "$ec"
            cat "$dir/command.txt" 2>/dev/null
            printf "\n=== FIRST 80 LINES ===\n";  head -80 "$log" 2>/dev/null
            printf "\n=== LAST 80 LINES ===\n";   tail -80 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
        echo "  [$name] CRASH  exit=$ec"
        grep -m3 -i "error\|unable\|fatal\|assert\|abort" "$log" 2>/dev/null \
            | head -3 | sed 's/^/    /' || true
        return
    fi

    if [[ ! -s "$traj" ]]; then
        {
            printf "exit_code=%s  traj.txt: missing/empty\n\n" "$ec"
            printf "=== FIRST 80 LINES ===\n"; head -80 "$log" 2>/dev/null
            printf "\n=== LAST 80 LINES ===\n"; tail -80 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
        echo "  [$name] CRASH  exit=0  traj.txt missing/empty"
        tail -5 "$log" 2>/dev/null | sed 's/^/    /' || true
        return
    fi

    local parse
    parse=$(awk '
        /^#/ { next }
        NF >= 3 {
            t=$1+0; x=$2+0; y=$3+0
            if (!(x==x) || !(y==y)) { has_nan=1; next }
            xy=sqrt(x*x+y*y)
            if (xy>max_xy) max_xy=xy
            if (xy>10000 && t10k=="") t10k=t
            if (xy>1000  && t1k =="") t1k=t
            final_t=t; final_x=x; final_y=y; n++
        }
        END {
            final_xy = sqrt(final_x*final_x + final_y*final_y)
            printf "n=%d final_t=%.3f max_xy=%.1f final_xy=%.1f t1k=%s t10k=%s nan=%d\n",
                   n, final_t, max_xy, final_xy,
                   (t1k==""?"none":t1k), (t10k==""?"none":t10k), has_nan+0
        }
    ' "$traj")

    local n final_t max_xy final_xy t1k t10k has_nan
    n=$(       echo "$parse" | grep -o 'n=[^ ]*'        | cut -d= -f2)
    final_t=$( echo "$parse" | grep -o 'final_t=[^ ]*'  | cut -d= -f2)
    max_xy=$(  echo "$parse" | grep -o 'max_xy=[^ ]*'   | cut -d= -f2)
    final_xy=$(echo "$parse" | grep -o 'final_xy=[^ ]*' | cut -d= -f2)
    t1k=$(     echo "$parse" | grep -o 't1k=[^ ]*'      | cut -d= -f2)
    t10k=$(    echo "$parse" | grep -o 't10k=[^ ]*'     | cut -d= -f2)
    has_nan=$( echo "$parse" | grep -o 'nan=[^ ]*'      | cut -d= -f2)

    local is_diverged is_partial
    is_diverged=$(python3 -c "print(1 if ${max_xy:-0} > 10000 else 0)" 2>/dev/null || echo 0)
    is_partial=$(python3  -c "print(1 if ${n:-0} < 1000 else 0)"       2>/dev/null || echo 0)

    if   [[ "$is_diverged" == "1" ]]; then
        printf "max_xy=%.0fm final_xy=%.0fm final_t=%ss first_10km@t=%s lines=%s\n" \
               "$max_xy" "$final_xy" "$final_t" "$t10k" "$n" > "$dir/STATUS_DIVERGED.txt"
        echo "  [$name] DIVERGED  max_xy=${max_xy}m  first_10km@t=${t10k}s  final_t=${final_t}s"
    elif [[ "$is_partial"  == "1" ]]; then
        printf "n=%s (< 1000)  final_t=%ss  max_xy=%.0fm\n" "$n" "$final_t" "$max_xy" \
               > "$dir/STATUS_PARTIAL.txt"
        echo "  [$name] PARTIAL  n=${n}  final_t=${final_t}s"
    else
        printf "max_xy=%.0fm final_xy=%.0fm final_t=%ss lines=%s\n" \
               "$max_xy" "$final_xy" "$final_t" "$n" > "$dir/STATUS_OK.txt"
        echo "  [$name] OK  max_xy=${max_xy}m  final_xy=${final_xy}m  final_t=${final_t}s"
    fi
}

# ─── run_one ──────────────────────────────────────────────────────────────────
run_one() {
    local flight="$1" entry="$2"
    local name="${entry%%:*}"
    local rest="${entry#*:}"; local cond="${rest%%:*}"; local dist="${rest#*:}"
    local out_dir="${F_OUTPUT_BASE[$flight]}/$name"

    mkdir -p "$out_dir"
    make_config_snapshot "$cond" "$dist" "$out_dir" "${F_CONFIG[$flight]}" "${F_CFG_DIR[$flight]}"
    local cfg="$out_dir/config_snapshot.yaml"
    echo "$GIT_COMMIT" > "$out_dir/git_commit.txt"

    local -a cmd=(
        "$BINARY"
        --config "$cfg"
        --dataset "${F_DATASET[$flight]}"
        --gps "${F_GPS[$flight]}"
        --gps-time-offset "${F_GPS_OFFSET[$flight]}"
        --start-time "${F_START[$flight]}"
        --init-from-fc "${F_FC_INIT[$flight]}"
        --vio-yaw-gauge-mode "$YAW_MODE"
        --init-bg-sigma "$BG_SIGMA"
        "${GPS_Z_FLAGS[@]}"
        --viz-fast --dash-every 5
        --output "$out_dir/traj.txt"
    )

    printf '%s \\\n  ' "${cmd[@]}" > "$out_dir/command.txt"
    printf '\n' >> "$out_dir/command.txt"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        echo ""
        echo "[DRY-RUN] $flight/$name  cond=$cond  dist=$dist"
        echo "  output: $out_dir"
        echo "  snapshot fi lines:"
        grep -E "fi_max_cond_number|fi_max_dist|relative_config" "$cfg" | sed 's/^/    /'
        return 0
    fi

    echo ">>> [$(date +%T)] $flight/$name  cond=$cond  dist=$dist"
    local ec=0
    "${cmd[@]}" > "$out_dir/stdout_stderr.log" 2>&1 || ec=$?
    cp "$out_dir/stdout_stderr.log" "$out_dir/log.txt"
    classify_status "$name" "$out_dir" "$ec"
    echo "    [$(date +%T)] Done $flight/$name"
}

# ─── eval_one ─────────────────────────────────────────────────────────────────
eval_one() {
    local flight="$1" entry="$2"
    local name="${entry%%:*}"
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
        2>&1 | tail -4 || true
}

# ─── run_flight_cond_batch ────────────────────────────────────────────────────
# Runs all dist values for one (flight, cond) in parallel, then evals.
run_flight_cond_batch() {
    local flight="$1" cond="$2"
    local label="$flight cond=$(cond_tag $cond)"
    echo "  ── $label (parallel dist sweep) ──"

    local entries=()
    if [[ "$flight" == "fly4" ]]; then
        while IFS= read -r e; do entries+=("$e"); done < <(make_matrix_fly4 "$cond")
    else
        while IFS= read -r e; do entries+=("$e"); done < <(make_matrix_fly1 "$cond")
    fi

    if [[ "$DRY_RUN" -eq 1 ]]; then
        for e in "${entries[@]}"; do run_one "$flight" "$e"; done
    else
        local pids=()
        for e in "${entries[@]}"; do run_one "$flight" "$e" & pids+=($!); done
        for pid in "${pids[@]}"; do wait "$pid" || true; done
    fi

    # Fail-fast: all crashed?
    local n_traj=0
    for e in "${entries[@]}"; do
        local nm="${e%%:*}"
        [[ -s "${F_OUTPUT_BASE[$flight]}/$nm/traj.txt" ]] && n_traj=$((n_traj+1))
    done
    if [[ "$DRY_RUN" -eq 0 && "$n_traj" -eq 0 ]]; then
        echo "FATAL: all runs in $label produced no traj.txt — stopping."
        local first_nm="${entries[0]%%:*}"
        head -20 "${F_OUTPUT_BASE[$flight]}/$first_nm/STATUS_CRASH.txt" 2>/dev/null | sed 's/^/  /' || true
        exit 1
    fi

    for e in "${entries[@]}"; do eval_one "$flight" "$e"; done
}

# ─── generate_flight_summary ──────────────────────────────────────────────────
generate_flight_summary() {
    local flight="$1"
    local out_base="${F_OUTPUT_BASE[$flight]}"
    [[ "$DRY_RUN" -eq 1 ]] && { echo "  [DRY-RUN] summary for $flight"; return 0; }

    python3 - <<PYEOF
import csv, math, os, re, sys

flight  = "$flight"
base    = "$out_base"

if flight in ("fly1", "fly3"):
    dist_list = [200, 300, 500, 800]
else:
    dist_list = [300, 500, 800, 1000]
cond_list = [5000, 3000, 1000]
cond_tags = {5000:"cond5e3", 3000:"cond3e3", 1000:"cond1e3"}

FIELDS = ["flight","run_name","fi_max_cond_number","fi_max_dist","status",
          "final_t","traj_lines","max_xy_m","final_xy_m",
          "gps_ate_rms_m","gps_ate_max_m","gps_ate_final_m",
          "yaw_rms_deg","yaw_max_deg","gps_z_accept","gps_z_reject"]

def get_status(d):
    for s in ("STATUS_OK","STATUS_DIVERGED","STATUS_PARTIAL","STATUS_CRASH"):
        if os.path.exists(f"{d}/{s}.txt"): return s[7:]
    return "UNKNOWN"

def parse_traj(d):
    p = f"{d}/traj.txt"
    if not os.path.exists(p): return {}
    rows = [l.split() for l in open(p) if not l.startswith('#') and l.strip()]
    if not rows: return {}
    n=len(rows); max_xy=0.0; final_t=final_xy=0.0
    for r in rows:
        if len(r) < 3: continue
        try:
            x,y = float(r[1]),float(r[2]); t=float(r[0])
        except: continue
        if not (math.isfinite(x) and math.isfinite(y)): continue
        xy=math.sqrt(x*x+y*y)
        if xy>max_xy: max_xy=xy
        final_t=t
    if rows and len(rows[-1])>=3:
        try: final_xy=math.sqrt(float(rows[-1][1])**2+float(rows[-1][2])**2)
        except: pass
    return dict(n=n, final_t=final_t, max_xy=max_xy, final_xy=final_xy)

def parse_gpsz(d):
    logp = f"{d}/log.txt"
    if not os.path.exists(logp): return 0,0
    acc=rej=0
    for l in open(logp, encoding='utf-8', errors='ignore'):
        m = re.search(r'GPS-ALT-EVAL.*status=(\w+)', l)
        if m:
            if m.group(1)=='ACC': acc+=1
            else: rej+=1
    return acc,rej

def parse_eval(d):
    p=f"{d}/eval/metrics_summary.csv"
    if not os.path.exists(p): return {}
    out={}
    for row in csv.reader(open(p)):
        if len(row)>=2:
            try: out[row[0].strip()]=float(row[1])
            except: out[row[0].strip()]=row[1].strip()
    return out

rows_out=[]
for cond in cond_list:
    for dist in dist_list:
        name = f"{cond_tags[cond]}_dist{dist}"
        d = f"{base}/{name}"
        st = get_status(d); ti = parse_traj(d); acc,rej = parse_gpsz(d); em = parse_eval(d)
        rows_out.append({
            "flight": flight, "run_name": name,
            "fi_max_cond_number": cond, "fi_max_dist": dist,
            "status": st,
            "final_t":    ti.get("final_t",""),
            "traj_lines": ti.get("n",""),
            "max_xy_m":   ti.get("max_xy",""),
            "final_xy_m": ti.get("final_xy",""),
            "gps_ate_rms_m":   em.get("xy_ate_rms_m",   em.get("A_xy_ate_rms_m","")),
            "gps_ate_max_m":   em.get("xy_ate_max_m",   em.get("A_xy_ate_max_m","")),
            "gps_ate_final_m": em.get("xy_ate_final_m", em.get("A_xy_ate_final_m","")),
            "yaw_rms_deg": em.get("yaw_err_rms_deg",""), "yaw_max_deg": em.get("yaw_err_max_deg",""),
            "gps_z_accept": acc, "gps_z_reject": rej,
        })

out_csv = f"{base}/summary_{flight}.csv"
with open(out_csv,"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=FIELDS); w.writeheader(); w.writerows(rows_out)

print(f"\n{flight} summary written: {out_csv}")
print(f"{'run_name':<22} {'status':<12} {'final_t':>9} {'max_xy':>10} {'ate_rms':>9} {'yaw_rms':>9}")
print("-"*75)
for r in rows_out:
    print(f"{r['run_name']:<22} {r['status']:<12} "
          f"{str(r['final_t']):>9} {str(r['max_xy_m']):>10} "
          f"{str(r['gps_ate_rms_m']):>9} {str(r['yaw_rms_deg']):>9}")
PYEOF
}

# ─── run_flight ───────────────────────────────────────────────────────────────
run_flight() {
    local flight="$1"
    local ob="${F_OUTPUT_BASE[$flight]}"
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  FLIGHT: $flight"
    echo "  Output: $ob"
    echo "════════════════════════════════════════════════════════"

    [[ "$DRY_RUN" -eq 0 ]] && mkdir -p "$ob"

    for cond in 5000 3000 1000; do
        run_flight_cond_batch "$flight" "$cond"
    done
    generate_flight_summary "$flight"
}

# ─── Pre-flight checks ────────────────────────────────────────────────────────
preflight_check() {
    [[ -x "$BINARY" ]]         || { echo "ERROR: binary not found: $BINARY";  exit 1; }
    local f
    for f in fly1 fly3 fly4; do
        [[ "$FLIGHT_FILTER" != "" && "$f" != "$FLIGHT_FILTER" && "$f" != "$SINGLE_FLIGHT" ]] && continue
        [[ -d "${F_DATASET[$f]}"  ]] || { echo "ERROR: dataset missing: ${F_DATASET[$f]}";  exit 1; }
        [[ -f "${F_CONFIG[$f]}"   ]] || { echo "ERROR: config missing: ${F_CONFIG[$f]}";    exit 1; }
        [[ -f "${F_GPS[$f]}"      ]] || { echo "ERROR: GPS missing: ${F_GPS[$f]}";          exit 1; }
        [[ -f "${F_FC_INIT[$f]}"  ]] || { echo "ERROR: FC init missing: ${F_FC_INIT[$f]}";  exit 1; }
        [[ -f "${F_CFG_DIR[$f]}/kalibr_imu_chain.yaml"    ]] \
            || { echo "ERROR: kalibr_imu_chain missing in ${F_CFG_DIR[$f]}"; exit 1; }
        [[ -f "${F_CFG_DIR[$f]}/kalibr_imucam_chain.yaml" ]] \
            || { echo "ERROR: kalibr_imucam_chain missing in ${F_CFG_DIR[$f]}"; exit 1; }
    done
    echo "Pre-flight checks passed."
}

# ─── Main ─────────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════════"
echo "  fi_max_cond × fi_max_dist sweep — fly1 / fly3 / fly4"
echo "  Fixed: yaw=$YAW_MODE  bg_sigma=$BG_SIGMA  GPS-Z ON"
echo "  Commit: $GIT_COMMIT"
[[ "$DRY_RUN" -eq 1  ]] && echo "  *** DRY-RUN ***"
[[ "$MODE" == "single" ]] && echo "  *** SINGLE: $SINGLE_FLIGHT / $SINGLE_RUN ***"
[[ -n "$FLIGHT_FILTER" ]] && echo "  *** FLIGHT FILTER: $FLIGHT_FILTER ***"
echo "══════════════════════════════════════════════════════════════════"
echo ""

[[ "$DRY_RUN" -eq 0 ]] && preflight_check

if [[ "$MODE" == "single" ]]; then
    # Find the matching entry
    local_entry=""
    for cond in 5000 3000 1000; do
        if [[ "$SINGLE_FLIGHT" == "fly4" ]]; then
            while IFS= read -r e; do
                [[ "${e%%:*}" == "$SINGLE_RUN" ]] && local_entry="$e" && break
            done < <(make_matrix_fly4 "$cond")
        else
            while IFS= read -r e; do
                [[ "${e%%:*}" == "$SINGLE_RUN" ]] && local_entry="$e" && break
            done < <(make_matrix_fly1 "$cond")
        fi
        [[ -n "$local_entry" ]] && break
    done
    if [[ -z "$local_entry" ]]; then
        echo "ERROR: run '$SINGLE_RUN' not found for flight '$SINGLE_FLIGHT'"
        echo "Valid names for $SINGLE_FLIGHT:"
        for cond in 5000 3000 1000; do
            if [[ "$SINGLE_FLIGHT" == "fly4" ]]; then
                make_matrix_fly4 "$cond" | while IFS= read -r e; do echo "  ${e%%:*}"; done
            else
                make_matrix_fly1 "$cond" | while IFS= read -r e; do echo "  ${e%%:*}"; done
            fi
        done
        exit 1
    fi
    [[ "$DRY_RUN" -eq 0 ]] && mkdir -p "${F_OUTPUT_BASE[$SINGLE_FLIGHT]}"
    run_one  "$SINGLE_FLIGHT" "$local_entry"
    eval_one "$SINGLE_FLIGHT" "$local_entry"
    echo ""
    echo "Single-run complete: $SINGLE_FLIGHT / $SINGLE_RUN"
    exit 0
fi

# Full sweep: flights in sequence
for flight in fly1 fly3 fly4; do
    [[ -n "$FLIGHT_FILTER" && "$flight" != "$FLIGHT_FILTER" ]] && continue
    run_flight "$flight"
done

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  All sweeps complete."
echo "══════════════════════════════════════════════════════════════════"
