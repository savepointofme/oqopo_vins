#!/usr/bin/env bash
# run_yaw_ablation.sh
#
# Targeted yaw-drift parameter ablation: one parameter at a time.
# Flights: fly1 (~200 m), fly3 (~200 m), fly4 (~400 m)
# Fixed:   yaw=oc_postchi2_current_gauge  GPS-Z ON  init_bg_sigma=0.003
#          fi_max_cond_number=1e4 (config default, no override)
#
# Groups:
#   baseline  — unmodified config per flight
#   P1        — up_msckf_sigma_px ∈ {1, 2, 3}   (fly1 baseline=1; fly3/fly4 baseline=2)
#   P2        — up_msckf_chi2_multipler ∈ {1, 2, 5}  (all baseline=1)
#   P3        — max_clones ∈ {11, 15, 20}   (fly3 only)
#   P4        — gyroscope_noise_density ∈ {0.002, 0.005, 0.010}  (fly3 only)
#
# Usage:
#   bash run_yaw_ablation.sh                       # full sweep all flights
#   bash run_yaw_ablation.sh --group baseline
#   bash run_yaw_ablation.sh --group P1
#   bash run_yaw_ablation.sh --group P2
#   bash run_yaw_ablation.sh --group P3
#   bash run_yaw_ablation.sh --group P4
#   bash run_yaw_ablation.sh --flight fly3
#   bash run_yaw_ablation.sh --single fly1 baseline_fly1
#   bash run_yaw_ablation.sh --dry-run

set -uo pipefail

DRY_RUN=0
MODE="all"
GROUP_FILTER=""
FLIGHT_FILTER=""
SINGLE_FLIGHT=""
SINGLE_RUN=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1 ;;
        --group)
            GROUP_FILTER="${2:-}"
            [[ "$GROUP_FILTER" =~ ^(baseline|P1|P2|P3|P4)$ ]] || {
                echo "ERROR: --group must be baseline|P1|P2|P3|P4"; exit 1; }
            shift ;;
        --flight)
            FLIGHT_FILTER="${2:-}"
            [[ "$FLIGHT_FILTER" =~ ^(fly1|fly3|fly4)$ ]] || {
                echo "ERROR: --flight must be fly1|fly3|fly4"; exit 1; }
            shift ;;
        --single)
            MODE="single"
            SINGLE_FLIGHT="${2:-}"; SINGLE_RUN="${3:-}"
            [[ -z "$SINGLE_FLIGHT" || -z "$SINGLE_RUN" ]] && {
                echo "Usage: --single <fly1|fly3|fly4> <run_name>"; exit 1; }
            shift 2 ;;
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
F_OUTPUT_BASE[fly1]="/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/yaw_ablation_20260608"
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
F_OUTPUT_BASE[fly3]="/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/yaw_ablation_20260608"
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
F_OUTPUT_BASE[fly4]="/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/yaw_ablation_20260608"
F_IMU[fly4]="${F_DATASET[fly4]}/imu0/data.csv"

# ─── make_config_snapshot ─────────────────────────────────────────────────────
# Args: flight dst_dir sigma_px chi2_mult max_clones gyro_nd
# Pass "" (empty string) to keep the config file's original value for that field.
make_config_snapshot() {
    local flight="$1" dst_dir="$2"
    local sigma_px="$3" chi2_mult="$4" max_clones_v="$5" gyro_nd="$6"
    local base_config="${F_CONFIG[$flight]}"
    local cfg_dir="${F_CFG_DIR[$flight]}"
    local cfg="$dst_dir/config_snapshot.yaml"

    cp "$cfg_dir/kalibr_imu_chain.yaml"    "$dst_dir/"
    cp "$cfg_dir/kalibr_imucam_chain.yaml"  "$dst_dir/"

    awk \
        -v sigma="$sigma_px" \
        -v chi2="$chi2_mult" \
        -v clones="$max_clones_v" \
        '
        /^up_msckf_sigma_px:/ {
            if (sigma != "") { print "up_msckf_sigma_px: " sigma; next }
        }
        /^up_msckf_chi2_multipler:/ {
            if (chi2 != "") { print "up_msckf_chi2_multipler: " chi2; next }
        }
        /^max_clones:/ {
            if (clones != "") { print "max_clones: " clones; next }
        }
        { print }
        ' "$base_config" > "$cfg"

    if [[ -n "$gyro_nd" ]]; then
        sed -i "s/^  gyroscope_noise_density:.*$/  gyroscope_noise_density: ${gyro_nd}/" \
            "$dst_dir/kalibr_imu_chain.yaml"
    fi
}

# ─── classify_status ──────────────────────────────────────────────────────────
classify_status() {
    local name="$1" dir="$2" ec="$3"
    local traj="$dir/traj.txt" log="$dir/stdout_stderr.log"
    echo "$ec" > "$dir/exit_code.txt"
    rm -f "$dir"/STATUS_*.txt

    if [[ "$ec" -ne 0 ]]; then
        { printf "exit_code=%s\n\n=== COMMAND ===\n" "$ec"
          cat "$dir/command.txt" 2>/dev/null
          printf "\n=== FIRST 80 LINES ===\n"; head -80 "$log" 2>/dev/null
          printf "\n=== LAST 80 LINES ===\n";  tail -80 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
        echo "  [$name] CRASH  exit=$ec"
        return
    fi
    if [[ ! -s "$traj" ]]; then
        { printf "exit_code=0  traj missing\n"; tail -20 "$log" 2>/dev/null
        } > "$dir/STATUS_CRASH.txt"
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
        echo "  [$name] DIVERGED  max_xy=${max_xy}m  t10k=${t10k}s"
    else
        printf "max_xy=%.0fm final_t=%ss lines=%s\n" "$max_xy" "$final_t" "$n" \
               > "$dir/STATUS_OK.txt"
        echo "  [$name] OK  max_xy=${max_xy}m  final_t=${final_t}s"
    fi
}

# ─── run_one ──────────────────────────────────────────────────────────────────
# Args: flight run_name sigma_px chi2_mult max_clones gyro_nd
run_one() {
    local flight="$1" name="$2"
    local sigma_px="$3" chi2_mult="$4" max_clones_v="$5" gyro_nd="$6"
    local out_dir="${F_OUTPUT_BASE[$flight]}/$name"

    mkdir -p "$out_dir"
    make_config_snapshot "$flight" "$out_dir" "$sigma_px" "$chi2_mult" "$max_clones_v" "$gyro_nd"
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
        echo "[DRY-RUN] $flight/$name"
        printf "  sigma_px=%-3s  chi2_mult=%-3s  max_clones=%-3s  gyro_nd=%s\n" \
            "${sigma_px:-(cfg)}" "${chi2_mult:-(cfg)}" "${max_clones_v:-(cfg)}" "${gyro_nd:-(cfg)}"
        grep -E "up_msckf_sigma_px|up_msckf_chi2_multipler|max_clones" "$cfg" | sed 's/^/    /'
        if [[ -n "$gyro_nd" ]]; then
            grep "gyroscope_noise_density" "$out_dir/kalibr_imu_chain.yaml" | sed 's/^/    /'
        fi
        return 0
    fi

    echo ">>> [$(date +%T)] $flight/$name"
    local ec=0
    "${cmd[@]}" > "$out_dir/stdout_stderr.log" 2>&1 || ec=$?
    cp "$out_dir/stdout_stderr.log" "$out_dir/log.txt"
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
        2>&1 | tail -4 || true
}

# ─── run_group ────────────────────────────────────────────────────────────────
# Runs up to 4 entries in parallel, then evals sequentially.
# Entry format: "flight:name:sigma:chi2:clones:gyro"
run_group() {
    local label="$1"; shift
    local entries=("$@")
    echo ""
    echo "  ════ Group: $label (${#entries[@]} runs) ════"

    if [[ "$DRY_RUN" -eq 1 ]]; then
        for entry in "${entries[@]}"; do
            IFS=: read -r _fl _nm _sig _chi _cl _gn <<< "$entry"
            run_one "$_fl" "$_nm" "$_sig" "$_chi" "$_cl" "$_gn"
        done
    else
        local pids=()
        for entry in "${entries[@]}"; do
            IFS=: read -r _fl _nm _sig _chi _cl _gn <<< "$entry"
            run_one "$_fl" "$_nm" "$_sig" "$_chi" "$_cl" "$_gn" &
            pids+=($!)
        done
        for pid in "${pids[@]}"; do wait "$pid" || true; done
    fi

    for entry in "${entries[@]}"; do
        IFS=: read -r _fl _nm _sig _chi _cl _gn <<< "$entry"
        eval_one "$_fl" "$_nm"
    done
}

# ─── generate_summary ─────────────────────────────────────────────────────────
generate_summary() {
    [[ "$DRY_RUN" -eq 1 ]] && { echo "[DRY-RUN] summary skipped"; return; }

    python3 - <<'PYEOF'
import csv, math, os, re

flights_cfg = {
    "fly1": "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/yaw_ablation_20260608",
    "fly3": "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/yaw_ablation_20260608",
    "fly4": "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/yaw_ablation_20260608",
}

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

def parse_traj_max_xy(d):
    p = f"{d}/traj.txt"
    if not os.path.exists(p): return 0.0
    mx = 0.0
    for l in open(p):
        if l.startswith('#') or not l.strip(): continue
        r = l.split()
        if len(r) < 3: continue
        try:
            x,y = float(r[1]),float(r[2])
            mx = max(mx, math.sqrt(x*x+y*y))
        except: pass
    return mx

def parse_snapshot(d):
    out = {}
    cfg = f"{d}/config_snapshot.yaml"
    if os.path.exists(cfg):
        for l in open(cfg):
            for key in ("up_msckf_sigma_px","up_msckf_chi2_multipler","max_clones"):
                if l.strip().startswith(key+":"):
                    try: out[key] = float(l.split(":",1)[1].strip())
                    except: pass
    imu = f"{d}/kalibr_imu_chain.yaml"
    if os.path.exists(imu):
        for l in open(imu):
            if "gyroscope_noise_density:" in l:
                try: out["gyro_nd"] = float(l.split(":",1)[1].strip())
                except: pass
    return out

FIELDS = ["flight","run_name","group","sigma_px","chi2_mult","max_clones","gyro_nd",
          "status","max_xy_m",
          "xy_ate_rms_m","xy_ate_final_m",
          "yaw_err_rms_deg","yaw_err_max_deg","yaw_err_p95_deg","yaw_err_final_deg"]

rows = []
for flight, base in sorted(flights_cfg.items()):
    if not os.path.exists(base): continue
    for run_name in sorted(os.listdir(base)):
        d = f"{base}/{run_name}"
        if not os.path.isdir(d): continue
        st = get_status(d)
        em = parse_eval(d)
        pr = parse_snapshot(d)
        group = ("baseline" if "baseline" in run_name
                 else "P1" if "sigma"  in run_name
                 else "P2" if "chi2"   in run_name
                 else "P3" if "clones" in run_name
                 else "P4" if "gyro"   in run_name
                 else "other")
        rows.append({
            "flight": flight, "run_name": run_name, "group": group,
            "sigma_px":   pr.get("up_msckf_sigma_px",  ""),
            "chi2_mult":  pr.get("up_msckf_chi2_multipler", ""),
            "max_clones": pr.get("max_clones",          ""),
            "gyro_nd":    pr.get("gyro_nd",             ""),
            "status": st,
            "max_xy_m": f"{parse_traj_max_xy(d):.0f}" if st not in ("NOT_RUN",) else "",
            "xy_ate_rms_m":      em.get("xy_ate_rms_m",""),
            "xy_ate_final_m":    em.get("xy_ate_final_m",""),
            "yaw_err_rms_deg":   em.get("yaw_err_rms_deg",""),
            "yaw_err_max_deg":   em.get("yaw_err_max_deg",""),
            "yaw_err_p95_deg":   em.get("yaw_err_p95_deg",""),
            "yaw_err_final_deg": em.get("yaw_err_final_deg",""),
        })

# Per-flight CSV
for flight, base in sorted(flights_cfg.items()):
    if not os.path.exists(base): continue
    out_csv = f"{base}/summary_yaw_ablation_{flight}.csv"
    frows = [r for r in rows if r["flight"] == flight]
    with open(out_csv,"w",newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(frows)
    print(f"Written: {out_csv}")

# Print table
def fmt(v, w=8, d=2):
    return f"{v:{w}.{d}f}" if isinstance(v, float) else f"{str(v):>{w}}"

print(f"\n{'flight':<6} {'run_name':<28} {'grp':<9} {'σ':>3} {'χ²':>3} {'cl':>3} {'gyro':>7}  {'status':<10} {'ATE_rms':>8} {'yaw_rms':>8} {'yaw_p95':>8} {'yaw_fin':>8}")
print("-"*115)
for r in rows:
    sig  = f"{r['sigma_px']:.0f}"   if isinstance(r['sigma_px'],  float) else "-"
    chi2 = f"{r['chi2_mult']:.0f}"  if isinstance(r['chi2_mult'], float) else "-"
    cl   = f"{r['max_clones']:.0f}" if isinstance(r['max_clones'],float) else "-"
    gnd  = f"{r['gyro_nd']:.3f}"    if isinstance(r['gyro_nd'],   float) else "-"
    ate  = fmt(r['xy_ate_rms_m'])
    yrms = fmt(r['yaw_err_rms_deg'])
    yp95 = fmt(r['yaw_err_p95_deg'])
    yfin = fmt(r['yaw_err_final_deg'])
    print(f"  {r['flight']:<5} {r['run_name']:<28} {r['group']:<9} {sig:>3} {chi2:>3} {cl:>3} {gnd:>7}  {r['status']:<10} {ate} {yrms} {yp95} {yfin}")
PYEOF
}

# ─── Pre-flight checks ────────────────────────────────────────────────────────
preflight_check() {
    [[ -x "$BINARY" ]] || { echo "ERROR: binary not found: $BINARY"; exit 1; }
    for f in fly1 fly3 fly4; do
        [[ -n "$FLIGHT_FILTER" && "$f" != "$FLIGHT_FILTER" ]] && continue
        [[ -d "${F_DATASET[$f]}"  ]] || { echo "ERROR: dataset missing: ${F_DATASET[$f]}";   exit 1; }
        [[ -f "${F_CONFIG[$f]}"   ]] || { echo "ERROR: config missing: ${F_CONFIG[$f]}";     exit 1; }
        [[ -f "${F_GPS[$f]}"      ]] || { echo "ERROR: GPS missing: ${F_GPS[$f]}";           exit 1; }
        [[ -f "${F_FC_INIT[$f]}"  ]] || { echo "ERROR: FC init missing: ${F_FC_INIT[$f]}";   exit 1; }
        [[ -f "${F_CFG_DIR[$f]}/kalibr_imu_chain.yaml"    ]] \
            || { echo "ERROR: kalibr_imu missing in ${F_CFG_DIR[$f]}"; exit 1; }
        [[ -f "${F_CFG_DIR[$f]}/kalibr_imucam_chain.yaml" ]] \
            || { echo "ERROR: kalibr_imucam missing in ${F_CFG_DIR[$f]}"; exit 1; }
    done
    echo "Pre-flight checks passed."
}

# ─── group_applies ────────────────────────────────────────────────────────────
group_applies() {
    local group="$1"
    [[ -z "$GROUP_FILTER" || "$GROUP_FILTER" == "$group" ]]
}

flight_applies() {
    local flight="$1"
    [[ -z "$FLIGHT_FILTER" || "$FLIGHT_FILTER" == "$flight" ]]
}

# ─── Main ─────────────────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════════"
echo "  Yaw-drift parameter ablation — fly1 / fly3 / fly4"
echo "  Fixed: yaw=$YAW_MODE  bg_sigma=$BG_SIGMA  GPS-Z ON"
echo "  Commit: $GIT_COMMIT"
[[ "$DRY_RUN" -eq 1 ]]      && echo "  *** DRY-RUN ***"
[[ -n "$GROUP_FILTER" ]]     && echo "  *** GROUP FILTER: $GROUP_FILTER ***"
[[ -n "$FLIGHT_FILTER" ]]    && echo "  *** FLIGHT FILTER: $FLIGHT_FILTER ***"
[[ "$MODE" == "single" ]]    && echo "  *** SINGLE: $SINGLE_FLIGHT / $SINGLE_RUN ***"
echo "══════════════════════════════════════════════════════════════════"

[[ "$DRY_RUN" -eq 0 ]] && preflight_check

# Create output dirs
if [[ "$DRY_RUN" -eq 0 ]]; then
    for f in fly1 fly3 fly4; do
        flight_applies "$f" && mkdir -p "${F_OUTPUT_BASE[$f]}"
    done
fi

# ─── Single-run mode ──────────────────────────────────────────────────────────
if [[ "$MODE" == "single" ]]; then
    [[ "$DRY_RUN" -eq 0 ]] && mkdir -p "${F_OUTPUT_BASE[$SINGLE_FLIGHT]}"
    # Find the run's params by iterating all defined groups
    # We'll just use empty overrides (user can re-specify if needed);
    # the snapshot is created from config defaults when all params are "".
    echo "ERROR: --single not yet implemented; use --group + --flight filters"
    exit 1
fi

# ─── baseline group ───────────────────────────────────────────────────────────
if group_applies "baseline"; then
    g_runs=()
    flight_applies "fly1" && g_runs+=("fly1:baseline_fly1::::")
    flight_applies "fly3" && g_runs+=("fly3:baseline_fly3::::")
    flight_applies "fly4" && g_runs+=("fly4:baseline_fly4::::")
    [[ "${#g_runs[@]}" -gt 0 ]] && run_group "baseline" "${g_runs[@]}"
fi

# ─── P1: sigma_px ─────────────────────────────────────────────────────────────
# fly1 baseline=1 → test 2, 3
# fly3 baseline=2 → test 1, 3
# fly4 baseline=2 → test 1, 3
if group_applies "P1"; then
    # sigma=2 batch (fly1)
    g_runs=()
    flight_applies "fly1" && g_runs+=("fly1:P1_sigma2_fly1:2:::")
    [[ "${#g_runs[@]}" -gt 0 ]] && run_group "P1 sigma=2" "${g_runs[@]}"

    # sigma=3 batch (fly1, fly3, fly4)
    g_runs=()
    flight_applies "fly1" && g_runs+=("fly1:P1_sigma3_fly1:3:::")
    flight_applies "fly3" && g_runs+=("fly3:P1_sigma3_fly3:3:::")
    flight_applies "fly4" && g_runs+=("fly4:P1_sigma3_fly4:3:::")
    [[ "${#g_runs[@]}" -gt 0 ]] && run_group "P1 sigma=3" "${g_runs[@]}"

    # sigma=1 batch (fly3, fly4)
    g_runs=()
    flight_applies "fly3" && g_runs+=("fly3:P1_sigma1_fly3:1:::")
    flight_applies "fly4" && g_runs+=("fly4:P1_sigma1_fly4:1:::")
    [[ "${#g_runs[@]}" -gt 0 ]] && run_group "P1 sigma=1" "${g_runs[@]}"
fi

# ─── P2: chi2_multipler ───────────────────────────────────────────────────────
# fly1 and fly3 only; baseline=1 → test 2, 5
if group_applies "P2"; then
    g_runs=()
    flight_applies "fly1" && g_runs+=("fly1:P2_chi2m2_fly1::2::")
    flight_applies "fly3" && g_runs+=("fly3:P2_chi2m2_fly3::2::")
    [[ "${#g_runs[@]}" -gt 0 ]] && run_group "P2 chi2=2" "${g_runs[@]}"

    g_runs=()
    flight_applies "fly1" && g_runs+=("fly1:P2_chi2m5_fly1::5::")
    flight_applies "fly3" && g_runs+=("fly3:P2_chi2m5_fly3::5::")
    [[ "${#g_runs[@]}" -gt 0 ]] && run_group "P2 chi2=5" "${g_runs[@]}"
fi

# ─── P3: max_clones (fly3 only) ───────────────────────────────────────────────
if group_applies "P3" && flight_applies "fly3"; then
    run_group "P3 clones=15" "fly3:P3_clones15_fly3:::15:"
    run_group "P3 clones=20" "fly3:P3_clones20_fly3:::20:"
fi

# ─── P4: gyroscope_noise_density (fly3 only) ──────────────────────────────────
if group_applies "P4" && flight_applies "fly3"; then
    run_group "P4 gyro=0.002" "fly3:P4_gyro002_fly3::::0.002"
    run_group "P4 gyro=0.010" "fly3:P4_gyro010_fly3::::0.010"
fi

echo ""
echo "All runs complete. Generating summary..."
generate_summary

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "  Yaw ablation complete."
echo "══════════════════════════════════════════════════════════════════"
