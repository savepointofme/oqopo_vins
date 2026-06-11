#!/usr/bin/env python3
"""
analysis/sigma1_analysis.py

Post-run analysis for the sigma=1 stabilisation ablation.

Produces:
  1. Ablation summary table (ATE, chi2 rejection stats, convergence)
  2. fly3 first-turn analysis
       - XY overlay (GPS truth vs each group)
       - Along-track error in turn window
       - Cross-track error in turn window
       - Local scale ratio in turn window
       - Yaw error in turn window
  3. fly1 stability analysis (chi2 rejection rate over time)
  4. fly4 impact analysis (ATE and yaw drift comparison)
  5. Decision table (reject / diagnostic / promising / mainline candidate)

Usage:
  python3 analysis/sigma1_analysis.py
  python3 analysis/sigma1_analysis.py --out /path/to/output
  python3 analysis/sigma1_analysis.py --flight fly3
"""

import argparse, csv, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# ── CLI ───────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--out",    default="/mnt/c/Users/baloney/Desktop/sigma1_analysis_20260610")
parser.add_argument("--flight", default="")  # filter to one flight for quick plots
parser.add_argument("--tag",    default="sigma1_stab_20260610")
args = parser.parse_args()

os.makedirs(args.out, exist_ok=True)
TAG = args.tag

# ── Per-flight configuration ──────────────────────────────────────────────────
FLIGHTS = {
    "fly1": {
        "base":   f"/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/{TAG}",
        "gps":    "/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv",
        "t0":     930.0,
        "until":  1740.0,
        "label":  "fly1 (~200 m)",
    },
    "fly3": {
        "base":   f"/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/{TAG}",
        "gps":    "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv",
        "t0":     618.0,
        "until":  1600.0,
        "label":  "fly3 (~200 m)",
    },
    "fly4": {
        "base":   f"/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/{TAG}",
        "gps":    "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv",
        "t0":     924.4,
        "until":  2816.0,
        "label":  "fly4 (~400 m)",
    },
}

# GPS WGS84 reference per flight (from eval_stage.py)
GPS_REF = {
    "fly1": (38.4975415, 103.2091577, 1399.505),   # same sensor area
    "fly3": (38.4975415, 103.2091577, 1399.505),
    "fly4": (38.4975415, 103.2091577, 1399.505),
}

GROUP_ORDER = list("ABCDEFGHIJ")
GROUP_COLOR = {
    "A": "#2196F3",  # blue  (baseline σ=2)
    "B": "#F44336",  # red   (raw σ=1)
    "C": "#FF9800",  # orange (σ=1 chi2=3)
    "D": "#9C27B0",  # purple (σ=1 chi2=5 diag)
    "E": "#4CAF50",  # green (σ=1 alt-dist)
    "F": "#009688",  # teal  (σ=1 chi2=3 alt-dist)
    "G": "#795548",  # brown (σ=1 more-feat)
    "H": "#607D8B",  # grey  (σ=1 chi2=3 alt-dist more-feat)
    "I": "#E91E63",  # pink  (σ=1 clones=15)
    "J": "#3F51B5",  # indigo (σ=2 clones=15)
}
GROUP_DESC = {
    "A": "σ=2 χ²=1 baseline",
    "B": "σ=1 χ²=1 raw",
    "C": "σ=1 χ²=3",
    "D": "σ=1 χ²=5 [diag]",
    "E": "σ=1 χ²=1 alt-dist",
    "F": "σ=1 χ²=3 alt-dist",
    "G": "σ=1 χ²=1 more-feat",
    "H": "σ=1 χ²=3 alt-dist +feat",
    "I": "σ=1 χ²=3 alt-dist +feat cl=15",
    "J": "σ=2 χ²=1 cl=15",
}

# ── helpers ───────────────────────────────────────────────────────────────────
RAD = math.pi / 180.0

def wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0):
    dN = (lat - lat0) * 111320.0
    dE = (lon - lon0) * 111320.0 * math.cos(lat0 * RAD)
    return dE, dN, alt - alt0

def rms(v):
    a = np.asarray(v, float)
    return float(np.sqrt(np.mean(a**2))) if len(a) else float("nan")

def pct(v, p):
    a = np.asarray(v, float)
    return float(np.percentile(np.abs(a), p)) if len(a) else float("nan")

def interp1(xs, ys, x):
    xs = np.asarray(xs); ys = np.asarray(ys)
    idx = np.searchsorted(xs, x)
    if idx == 0:       return float(ys[0])
    if idx >= len(xs): return float(ys[-1])
    a = (x - xs[idx-1]) / (xs[idx] - xs[idx-1])
    return float(ys[idx-1] + a*(ys[idx]-ys[idx-1]))

def wrap180(a):
    while a >  180: a -= 360
    while a < -180: a += 360
    return a

def quat_to_yaw_deg(qx, qy, qz, qw):
    y_rad = math.atan2(-2.0*qw*qz + 2.0*qx*qy, 1.0 - 2.0*(qy**2 + qz**2))
    return math.degrees(y_rad)

# ── GPS loader ────────────────────────────────────────────────────────────────
def load_gps(path, lat0, lon0, alt0):
    """Return (t_s, E, N, U) arrays. Handles WGS84 or ENU CSV."""
    ts, Es, Ns, Us = [], [], [], []
    rows = list(csv.DictReader(open(path)))
    if not rows: return None
    keys = rows[0].keys()
    is_wgs = "lat" in keys or "latitude" in keys
    t_key = "ts_ns" if "ts_ns" in keys else ("timestamp" if "timestamp" in keys else list(keys)[0])
    for r in rows:
        try:
            t = float(r[t_key])
            t_s = t / 1e9 if abs(t) > 1e7 else t  # ns → s if large
            if is_wgs:
                lat = float(r.get("lat", r.get("latitude", 0)))
                lon = float(r.get("lon", r.get("longitude", 0)))
                alt = float(r.get("alt", r.get("altitude", 0)))
                E, N, U = wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0)
            else:
                E = float(r.get("x", r.get("E", 0)))
                N = float(r.get("y", r.get("N", 0)))
                U = float(r.get("z", r.get("U", 0)))
            ts.append(t_s); Es.append(E); Ns.append(N); Us.append(U)
        except: pass
    return np.array(ts), np.array(Es), np.array(Ns), np.array(Us)

# ── trajectory loader ─────────────────────────────────────────────────────────
def load_traj(path):
    rows = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith("#"): continue
        c = s.split()
        if len(c) >= 8: rows.append([float(x) for x in c[:8]])
    if not rows: return None
    return np.array(rows)   # columns: t x y z qx qy qz qw

# ── diag.csv loader ───────────────────────────────────────────────────────────
def load_diag(path):
    if not os.path.exists(path): return None
    rows = list(csv.DictReader(open(path)))
    if not rows: return None
    keys = rows[0].keys()
    acc_col = "n_acc" if "n_acc" in keys else ("chi2_acc" if "chi2_acc" in keys else None)
    rej_col = "chi2_rej" if "chi2_rej" in keys else None
    t_col   = "t" if "t" in keys else None
    if not acc_col or not rej_col: return None
    ts, accs, rejs = [], [], []
    for r in rows:
        try:
            ts.append(float(r[t_col]) if t_col else 0)
            accs.append(float(r[acc_col]))
            rejs.append(float(r[rej_col]))
        except: pass
    if not ts: return None
    return {"t": np.array(ts), "acc": np.array(accs), "rej": np.array(rejs)}

# ── eval metric loader ────────────────────────────────────────────────────────
def load_eval(d):
    p = f"{d}/eval/metrics_summary.csv"
    if not os.path.exists(p): return {}
    out = {}
    for row in csv.reader(open(p)):
        if len(row) >= 2:
            try: out[row[0].strip()] = float(row[1])
            except: out[row[0].strip()] = row[1].strip()
    return out

def get_status(d):
    for s in ("STATUS_OK","STATUS_DIVERGED","STATUS_PARTIAL","STATUS_CRASH"):
        if os.path.exists(f"{d}/{s}.txt"): return s[7:]
    return "NOT_RUN"

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

# ── collect all runs ──────────────────────────────────────────────────────────
def collect_runs(flight_filter=""):
    runs = {}  # (flight, group_id) -> dict
    for flight, cfg in FLIGHTS.items():
        if flight_filter and flight != flight_filter: continue
        base = cfg["base"]
        if not os.path.isdir(base): continue
        for run_name in sorted(os.listdir(base)):
            d = f"{base}/{run_name}"
            if not os.path.isdir(d): continue
            grp_id = run_name.split("_")[0] if "_" in run_name else "?"
            if grp_id not in GROUP_ORDER: continue
            traj_path = f"{d}/traj.txt"
            traj = load_traj(traj_path) if os.path.exists(traj_path) else None
            diag = load_diag(f"{d}/diag.csv")
            em   = load_eval(d)
            pr   = parse_snapshot(d)
            st   = get_status(d)
            key  = (flight, grp_id)

            # Chi2 stats from diag
            if diag is not None:
                total = diag["acc"] + diag["rej"]
                rej_rate = np.where(total > 0, diag["rej"] / total, 0.0)
                stv_mask = diag["acc"] < 3
                first_stv = diag["t"][np.argmax(stv_mask)] if stv_mask.any() else None
                chi_stats = {
                    "mean_acc":      float(np.mean(diag["acc"])),
                    "mean_rej":      float(np.mean(diag["rej"])),
                    "mean_rej_rate": float(np.mean(rej_rate)),
                    "min_acc":       float(np.min(diag["acc"])),
                    "n_starvation":  int(stv_mask.sum()),
                    "first_stv_t":   float(first_stv) if first_stv is not None else None,
                    "diag":          diag,
                }
            else:
                chi_stats = {}

            runs[key] = {
                "flight": flight, "group": grp_id, "run_name": run_name,
                "dir": d, "status": st, "traj": traj, "eval": em, "snap": pr,
                **chi_stats,
            }
    return runs

# ── first-turn detection ──────────────────────────────────────────────────────
def detect_first_turn(gps_t, gps_E, gps_N, t_start, min_speed=10.0,
                      heading_change_deg=30.0, window_s=60.0, smooth_s=5.0):
    """Find the time of the first major heading change after t_start.

    Returns (t_turn_start, t_turn_peak, t_turn_end, heading_before, heading_after).
    heading_before/after in degrees.
    """
    # Keep only t >= t_start and above min_speed
    mask = gps_t >= t_start
    ts = gps_t[mask]; Es = gps_E[mask]; Ns = gps_N[mask]
    if len(ts) < 10: return None

    # Compute instantaneous headings from finite differences
    dt = np.diff(ts)
    dE = np.diff(Es); dN = np.diff(Ns)
    spd = np.sqrt((dE/dt)**2 + (dN/dt)**2)
    hdg = np.degrees(np.arctan2(dE, dN))  # compass bearing (N=0, E=90)
    t_mid = (ts[:-1] + ts[1:]) / 2

    # Smooth heading (box filter over smooth_s seconds)
    w = max(1, int(smooth_s / (dt.mean() + 1e-9)))
    kernel = np.ones(w) / w
    if len(hdg) > w:
        hdg_s = np.convolve(np.unwrap(np.radians(hdg)), kernel, mode="same")
        hdg_s = np.degrees(hdg_s)
    else:
        hdg_s = hdg

    # Find first point where heading change over window exceeds threshold
    for i in range(len(t_mid)):
        t_look = t_mid[i] + window_s
        j = np.searchsorted(t_mid, t_look)
        j = min(j, len(hdg_s)-1)
        if j <= i: continue
        delta = abs(wrap180(hdg_s[j] - hdg_s[i]))
        if delta >= heading_change_deg:
            t_turn_start = t_mid[i]
            t_turn_peak  = t_mid[i + (j-i)//2]
            t_turn_end   = t_mid[j]
            hdg_before   = hdg_s[i]
            hdg_after    = hdg_s[j]
            return (t_turn_start, t_turn_peak, t_turn_end, hdg_before, hdg_after)
    return None

# ── along/cross-track errors ──────────────────────────────────────────────────
def track_errors(vio_t, vio_E, vio_N, gps_t, gps_E, gps_N, t_win_start, t_win_end):
    """Compute along-track and cross-track errors in [t_win_start, t_win_end].

    For each VIO timestamp in window:
      - Interpolate GPS position
      - Find GPS track direction at that timestamp
      - Project error into along/cross-track components

    Returns (t, along_err, cross_err, total_dist).
    """
    mask = (vio_t >= t_win_start) & (vio_t <= t_win_end)
    ts = vio_t[mask]; vE = vio_E[mask]; vN = vio_N[mask]
    if len(ts) < 2: return None

    # GPS track direction: smooth derivative
    dt_g = np.diff(gps_t)
    dE_g = np.diff(gps_E); dN_g = np.diff(gps_N)
    spd_g = np.sqrt((dE_g/dt_g)**2 + (dN_g/dt_g)**2)
    t_g_mid = (gps_t[:-1] + gps_t[1:]) / 2

    along, cross, tdist = [], [], []
    for i, t in enumerate(ts):
        gE = interp1(gps_t, gps_E, t)
        gN = interp1(gps_t, gps_N, t)
        err_E = vE[i] - gE
        err_N = vN[i] - gN
        # Track direction at t
        spd = interp1(t_g_mid, spd_g, t)
        if spd > 1.0:
            tE = interp1(t_g_mid, dE_g/dt_g, t)
            tN = interp1(t_g_mid, dN_g/dt_g, t)
            tnorm = math.sqrt(tE**2 + tN**2)
            if tnorm > 0.1:
                tE /= tnorm; tN /= tnorm
                aE = err_E*tE + err_N*tN   # along track
                cE = err_E*(-tN) + err_N*tE # cross track
                along.append(aE); cross.append(cE)
                tdist.append(math.sqrt(err_E**2 + err_N**2))
                continue
        along.append(float("nan")); cross.append(float("nan"))
        tdist.append(float("nan"))

    return ts, np.array(along), np.array(cross), np.array(tdist)

# ── local scale ratio ─────────────────────────────────────────────────────────
def local_scale_ratio(vio_t, vio_E, vio_N, gps_t, gps_E, gps_N,
                      t_win_start, t_win_end, half_win=30.0):
    """Cumulative arc-length ratio VIO/GPS in rolling window."""
    mask = (vio_t >= t_win_start) & (vio_t <= t_win_end)
    ts = vio_t[mask]
    ratios = []
    for t in ts:
        t0 = t - half_win; t1 = t + half_win
        # GPS arc
        gm = (gps_t >= t0) & (gps_t <= t1)
        if gm.sum() < 2:
            ratios.append(float("nan")); continue
        ge = gps_E[gm]; gn = gps_N[gm]
        gps_arc = float(np.sum(np.sqrt(np.diff(ge)**2 + np.diff(gn)**2)))
        # VIO arc
        vm = (vio_t >= t0) & (vio_t <= t1)
        if vm.sum() < 2:
            ratios.append(float("nan")); continue
        ve = vio_E[vm]; vn = vio_N[vm]
        vio_arc = float(np.sum(np.sqrt(np.diff(ve)**2 + np.diff(vn)**2)))
        ratios.append(vio_arc / gps_arc if gps_arc > 1.0 else float("nan"))
    return ts, np.array(ratios)

# ─────────────────────────────────────────────────────────────────────────────
# MAIN ANALYSIS
# ─────────────────────────────────────────────────────────────────────────────
runs = collect_runs(args.flight)

if not runs:
    print(f"ERROR: No runs found under {list(FLIGHTS.values())[0]['base']}.")
    print("Have the experiments finished? Run bash run_sigma1_stabilization.sh first.")
    sys.exit(1)

# ── Print ablation table ──────────────────────────────────────────────────────
print("\n" + "═"*110)
print("  SIGMA=1 STABILISATION ABLATION — SUMMARY TABLE")
print("═"*110)
hdr = (f"{'flight':<5} {'grp':<4} {'σ':>3} {'χ²':>3} {'dist':>6} {'cl':>3} {'npts':>5}"
       f"  {'status':<10} {'ATE_rms':>8} {'yaw_rms':>8} {'yaw_p95':>8}"
       f"  {'acc/f':>6} {'rej_rt':>6} {'stv_n':>5} {'stv@t':>7}")
print(hdr)
print("-"*110)

summary_rows = []
for grp in GROUP_ORDER:
    for flight in ["fly1","fly3","fly4"]:
        if args.flight and flight != args.flight: continue
        key = (flight, grp)
        if key not in runs: continue
        r = runs[key]
        pr = r["snap"]
        em = r["eval"]
        sig  = f"{pr.get('up_msckf_sigma_px',''):>3.0f}"  if isinstance(pr.get("up_msckf_sigma_px"), float) else "  -"
        chi2 = f"{pr.get('up_msckf_chi2_multipler',''):>3.0f}" if isinstance(pr.get("up_msckf_chi2_multipler"), float) else "  -"
        dist = f"{pr.get('fi_max_dist',''):>6.0f}" if isinstance(pr.get("fi_max_dist"), float) else "     -"
        cl   = f"{pr.get('max_clones',''):>3.0f}" if isinstance(pr.get("max_clones"), float) else "  -"
        npts = f"{pr.get('num_pts',''):>5.0f}" if isinstance(pr.get("num_pts"), float) else "    -"
        ate  = f"{em['xy_ate_rms_m']:>8.1f}" if isinstance(em.get("xy_ate_rms_m"), float) else "       -"
        yrms = f"{em['yaw_err_rms_deg']:>8.2f}" if isinstance(em.get("yaw_err_rms_deg"), float) else "       -"
        yp95 = f"{em['yaw_err_p95_deg']:>8.2f}" if isinstance(em.get("yaw_err_p95_deg"), float) else "       -"
        acc  = f"{r['mean_acc']:>6.1f}" if "mean_acc" in r else "     -"
        rrt  = f"{r['mean_rej_rate']:>6.2f}" if "mean_rej_rate" in r else "     -"
        stv  = f"{r['n_starvation']:>5d}" if "n_starvation" in r else "    -"
        stv_t= f"{r['first_stv_t']:>7.1f}" if r.get("first_stv_t") is not None else "      -"
        print(f"  {flight:<5} {grp:<4} {sig} {chi2} {dist} {cl} {npts}  {r['status']:<10} {ate} {yrms} {yp95}  {acc} {rrt} {stv} {stv_t}")
        summary_rows.append((flight, grp, r))

print("═"*110)

# ── fly3 first-turn analysis ──────────────────────────────────────────────────
print("\n" + "─"*80)
print("  FLY3 FIRST-TURN ANALYSIS")
print("─"*80)

if "fly3" not in FLIGHTS or (args.flight and args.flight != "fly3"):
    print("  (skipped — flight filter excludes fly3)")
else:
    fcfg = FLIGHTS["fly3"]
    lat0, lon0, alt0 = GPS_REF["fly3"]
    gps_data = load_gps(fcfg["gps"], lat0, lon0, alt0)
    if gps_data is None:
        print("  ERROR: could not load fly3 GPS")
    else:
        gps_t, gps_E, gps_N, gps_U = gps_data
        # Restrict to eval window
        gm = (gps_t >= fcfg["t0"]) & (gps_t <= fcfg["until"])
        g_t = gps_t[gm]; g_E = gps_E[gm]; g_N = gps_N[gm]

        # Detect first turn
        turn = detect_first_turn(g_t, g_E, g_N, fcfg["t0"],
                                  min_speed=10.0, heading_change_deg=30.0,
                                  window_s=60.0, smooth_s=5.0)
        if turn is None:
            print("  No major turn detected in fly3 eval window. Using fallback window t=[618,750].")
            t_turn_start, t_turn_peak, t_turn_end = fcfg["t0"], fcfg["t0"]+65, fcfg["t0"]+130
            hdg_before, hdg_after = float("nan"), float("nan")
        else:
            t_turn_start, t_turn_peak, t_turn_end, hdg_before, hdg_after = turn
            print(f"  First turn detected:")
            print(f"    start={t_turn_start:.1f}s  peak={t_turn_peak:.1f}s  end={t_turn_end:.1f}s")
            print(f"    heading: {hdg_before:.1f}° → {hdg_after:.1f}°  (Δ={wrap180(hdg_after-hdg_before):.1f}°)")

        # Turn analysis window: extend slightly beyond detected bounds
        tw0 = max(fcfg["t0"], t_turn_start - 20.0)
        tw1 = min(fcfg["until"], t_turn_end + 40.0)
        print(f"  Analysis window: [{tw0:.1f}, {tw1:.1f}] s")

        # ── 1. XY overlay — first turn ────────────────────────────────────────
        fig_xy, ax_xy = plt.subplots(figsize=(8, 8))
        # GPS truth in window
        gm_w = (g_t >= tw0) & (g_t <= tw1)
        ax_xy.plot(g_E[gm_w], g_N[gm_w], "k-", lw=3, label="GPS truth", zorder=10)
        ax_xy.plot(g_E[gm_w][0], g_N[gm_w][0], "ks", ms=8, zorder=11)

        turn_entries = []
        for grp in GROUP_ORDER:
            key = ("fly3", grp)
            if key not in runs: continue
            r = runs[key]
            if r["traj"] is None or r["status"] not in ("OK","PARTIAL"): continue
            traj = r["traj"]
            # Apply same yaw alignment as eval_stage: anchor at t0
            vio_t = traj[:,0]; vio_E = traj[:,1]; vio_N = traj[:,2]
            # Shift origin to GPS position at t0
            gE0 = interp1(g_t, g_E, fcfg["t0"]); gN0 = interp1(g_t, g_N, fcfg["t0"])
            vE0 = interp1(vio_t, vio_E, fcfg["t0"]); vN0 = interp1(vio_t, vio_N, fcfg["t0"])
            # Yaw from start velocity over first 15s
            t_end_yaw = fcfg["t0"] + 15.0
            gm_yaw = (g_t >= fcfg["t0"]) & (g_t <= t_end_yaw)
            vm_yaw = (vio_t >= fcfg["t0"]) & (vio_t <= t_end_yaw)
            yaw_gps_rad = (math.atan2(g_E[gm_yaw][-1]-g_E[gm_yaw][0], g_N[gm_yaw][-1]-g_N[gm_yaw][0])
                           if gm_yaw.sum() > 1 else 0.0)
            yaw_vio_rad = (math.atan2(vio_E[vm_yaw][-1]-vio_E[vm_yaw][0],
                                       vio_N[vm_yaw][-1]-vio_N[vm_yaw][0])
                           if vm_yaw.sum() > 1 else 0.0)
            delta = yaw_gps_rad - yaw_vio_rad
            cos_d, sin_d = math.cos(delta), math.sin(delta)

            wm = (vio_t >= tw0) & (vio_t <= tw1)
            ve = vio_E[wm] - vE0; vn = vio_N[wm] - vN0
            ve_aln =  ve*cos_d - vn*sin_d + gE0
            vn_aln =  ve*sin_d + vn*cos_d + gN0
            c = GROUP_COLOR.get(grp, "grey")
            ax_xy.plot(ve_aln, vn_aln, "-", color=c, lw=1.5,
                       alpha=0.85, label=f"{grp}: {GROUP_DESC[grp]}")
            ate_val = r["eval"].get("xy_ate_rms_m", float("nan"))
            turn_entries.append((grp, ate_val, c))

        ax_xy.set_xlabel("East (m)"); ax_xy.set_ylabel("North (m)")
        ax_xy.set_title(f"fly3 First-Turn XY — GPS vs VIO (all groups)\n"
                        f"window [{tw0:.0f}–{tw1:.0f}] s, start_yaw aligned")
        ax_xy.legend(fontsize=7, loc="best")
        ax_xy.set_aspect("equal", "box")
        ax_xy.grid(True, alpha=0.3)
        fig_xy.savefig(f"{args.out}/fly3_first_turn_xy.png", dpi=150, bbox_inches="tight")
        plt.close(fig_xy)
        print(f"  Saved: fly3_first_turn_xy.png")

        # ── 2. Along/cross-track + scale + yaw in turn window ─────────────────
        fig_at, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
        ax_at, ax_ct, ax_sc, ax_yaw = axes

        for grp in GROUP_ORDER:
            key = ("fly3", grp)
            if key not in runs: continue
            r = runs[key]
            if r["traj"] is None or r["status"] not in ("OK","PARTIAL"): continue
            traj = r["traj"]
            vio_t = traj[:,0]; vio_E = traj[:,1]; vio_N = traj[:,2]
            # Apply same alignment
            gE0 = interp1(g_t, g_E, fcfg["t0"]); gN0 = interp1(g_t, g_N, fcfg["t0"])
            vE0 = interp1(vio_t, vio_E, fcfg["t0"]); vN0 = interp1(vio_t, vio_N, fcfg["t0"])
            t_end_yaw = fcfg["t0"] + 15.0
            gm_yaw = (g_t >= fcfg["t0"]) & (g_t <= t_end_yaw)
            vm_yaw = (vio_t >= fcfg["t0"]) & (vio_t <= t_end_yaw)
            yaw_gps_rad = (math.atan2(g_E[gm_yaw][-1]-g_E[gm_yaw][0], g_N[gm_yaw][-1]-g_N[gm_yaw][0])
                           if gm_yaw.sum() > 1 else 0.0)
            yaw_vio_rad = (math.atan2(vio_E[vm_yaw][-1]-vio_E[vm_yaw][0],
                                       vio_N[vm_yaw][-1]-vio_N[vm_yaw][0])
                           if vm_yaw.sum() > 1 else 0.0)
            delta = yaw_gps_rad - yaw_vio_rad
            cos_d, sin_d = math.cos(delta), math.sin(delta)
            # Aligned trajectory
            all_ve = (vio_E - vE0)*cos_d - (vio_N - vN0)*sin_d + gE0
            all_vn = (vio_E - vE0)*sin_d + (vio_N - vN0)*cos_d + gN0

            c = GROUP_COLOR.get(grp, "grey")
            label = f"{grp}: {GROUP_DESC[grp]}"

            res = track_errors(vio_t, all_ve, all_vn, g_t, g_E, g_N, tw0, tw1)
            if res:
                t_e, al, cr, td = res
                ax_at.plot(t_e - fcfg["t0"], al, "-", color=c, lw=1.3, alpha=0.8, label=label)
                ax_ct.plot(t_e - fcfg["t0"], cr, "-", color=c, lw=1.3, alpha=0.8, label=label)

            sc_res = local_scale_ratio(vio_t, all_ve, all_vn, g_t, g_E, g_N, tw0, tw1)
            if sc_res[0] is not None and len(sc_res[0]) > 0:
                ax_sc.plot(sc_res[0] - fcfg["t0"], sc_res[1], "-", color=c, lw=1.3, alpha=0.8)

            # Yaw error in window
            vm_w = (vio_t >= tw0) & (vio_t <= tw1)
            if vm_w.sum() > 0:
                vio_yaw_win = np.array([quat_to_yaw_deg(*traj[i,4:8]) for i in np.where(vm_w)[0]])
                vio_t_win = vio_t[vm_w]
                # GPS course heading
                gps_hdg_win = []
                for t in vio_t_win:
                    idx = np.searchsorted(g_t, t)
                    if idx == 0 or idx >= len(g_t):
                        gps_hdg_win.append(float("nan")); continue
                    dE_g_ = g_E[idx] - g_E[idx-1]; dN_g_ = g_N[idx] - g_N[idx-1]
                    if math.sqrt(dE_g_**2+dN_g_**2) < 0.1:
                        gps_hdg_win.append(float("nan")); continue
                    gps_hdg_win.append(math.degrees(math.atan2(dE_g_, dN_g_)) - math.degrees(delta))
                gps_hdg_arr = np.array(gps_hdg_win)
                yaw_err = np.array([wrap180(v-g) for v,g in zip(vio_yaw_win, gps_hdg_arr)
                                    if not math.isnan(g)])
                if len(yaw_err) > 0:
                    t_yaw = vio_t_win[:len(yaw_err)]
                    ax_yaw.plot(t_yaw - fcfg["t0"], yaw_err, "-", color=c, lw=1.3, alpha=0.8)

        for ax, ylabel, title in [
            (ax_at, "Along-track error (m)", "Along-track error in turn window"),
            (ax_ct, "Cross-track error (m)", "Cross-track error (positive = right)"),
            (ax_sc, "VIO/GPS arc ratio",     "Local scale ratio (1.0 = perfect)"),
            (ax_yaw,"Yaw error (°)",         "Yaw error vs GPS course heading"),
        ]:
            ax.set_ylabel(ylabel); ax.set_title(title)
            ax.axhline(0, color="black", lw=0.5, ls="--")
            ax.grid(True, alpha=0.3)
        ax_sc.axhline(1.0, color="black", lw=0.5, ls="--")
        ax_yaw.set_xlabel(f"Time since t={fcfg['t0']:.0f} s  (seconds)")
        # Add vertical line at turn start/end
        t_ts = t_turn_start - fcfg["t0"]; t_te = t_turn_end - fcfg["t0"]
        for ax in axes:
            ax.axvspan(t_ts, t_te, alpha=0.07, color="yellow", label="turn window" if ax==ax_at else "")
        # Legend once
        handles, labels = ax_at.get_legend_handles_labels()
        fig_at.legend(handles, labels, loc="upper right", fontsize=7, ncol=2)
        fig_at.suptitle(f"fly3 First-Turn Detail — sigma=1 ablation\nwindow [{tw0:.0f}–{tw1:.0f}] s",
                        fontsize=11)
        fig_at.tight_layout(rect=[0,0,1,0.96])
        fig_at.savefig(f"{args.out}/fly3_first_turn_detail.png", dpi=150, bbox_inches="tight")
        plt.close(fig_at)
        print(f"  Saved: fly3_first_turn_detail.png")

        # ── Qualitative assessment ────────────────────────────────────────────
        print(f"\n  Qualitative first-turn assessment:")
        for grp in GROUP_ORDER:
            key = ("fly3", grp)
            if key not in runs: continue
            r = runs[key]
            if r["traj"] is None: continue
            ate = r["eval"].get("xy_ate_rms_m", float("nan"))
            st  = r["status"]
            print(f"    {grp} ({GROUP_DESC[grp]:30s}): status={st:10s}  full-win ATE={ate:.1f} m")

# ── Chi2 rejection analysis (fly1 focus) ─────────────────────────────────────
print("\n" + "─"*80)
print("  FLY1 CHI2 REJECTION / STARVATION ANALYSIS")
print("─"*80)

fig_chi, ax_chi = plt.subplots(figsize=(14, 6))
fig_acc, ax_acc = plt.subplots(figsize=(14, 6))

for grp in GROUP_ORDER:
    key = ("fly1", grp)
    if key not in runs: continue
    r = runs[key]
    if "diag" not in r: continue
    diag = r["diag"]
    c = GROUP_COLOR.get(grp, "grey")
    label = f"{grp}: {GROUP_DESC[grp]} ({r['status']})"
    t = diag["t"]
    total = diag["acc"] + diag["rej"]
    rej_rate = np.where(total > 0, diag["rej"] / total, 0.0)
    ax_chi.plot(t, rej_rate, "-", color=c, lw=1.2, alpha=0.8, label=label)
    ax_acc.plot(t, diag["acc"], "-", color=c, lw=1.2, alpha=0.8, label=label)
    stv = (diag["acc"] < 3).sum()
    mean_acc = float(np.mean(diag["acc"]))
    rej_mean = float(np.mean(rej_rate))
    print(f"  {grp} ({GROUP_DESC[grp]:30s}): status={r['status']:10s}  "
          f"mean_acc={mean_acc:.1f}  rej_rate={rej_mean:.2f}  starvation_frames={stv}")

ax_chi.set_xlabel("Timestamp (s)"); ax_chi.set_ylabel("Chi2 rejection rate")
ax_chi.set_title("fly1 — Chi2 rejection rate over time (sigma=1 ablation)")
ax_chi.axhline(0.5, color="red", lw=0.8, ls="--", label="50% rejection")
ax_chi.set_ylim(0, 1.05); ax_chi.legend(fontsize=7); ax_chi.grid(True, alpha=0.3)
fig_chi.savefig(f"{args.out}/fly1_chi2_rejection_rate.png", dpi=150, bbox_inches="tight")
plt.close(fig_chi)

ax_acc.set_xlabel("Timestamp (s)"); ax_acc.set_ylabel("Accepted MSCKF features / update")
ax_acc.set_title("fly1 — Accepted MSCKF features per update (sigma=1 ablation)")
ax_acc.axhline(3, color="red", lw=0.8, ls="--", label="Starvation threshold (acc<3)")
ax_acc.legend(fontsize=7); ax_acc.grid(True, alpha=0.3)
fig_acc.savefig(f"{args.out}/fly1_accepted_features.png", dpi=150, bbox_inches="tight")
plt.close(fig_acc)
print(f"  Saved: fly1_chi2_rejection_rate.png, fly1_accepted_features.png")

# ── fly4 impact ───────────────────────────────────────────────────────────────
print("\n" + "─"*80)
print("  FLY4 IMPACT ANALYSIS (high-altitude robustness check)")
print("─"*80)

for grp in GROUP_ORDER:
    key = ("fly4", grp)
    if key not in runs: continue
    r = runs[key]
    ate  = r["eval"].get("xy_ate_rms_m", float("nan"))
    yrms = r["eval"].get("yaw_err_rms_deg", float("nan"))
    yp95 = r["eval"].get("yaw_err_p95_deg", float("nan"))
    acc  = r.get("mean_acc", float("nan"))
    rrt  = r.get("mean_rej_rate", float("nan"))
    ref_ate = runs.get(("fly4","A"), {}).get("eval", {}).get("xy_ate_rms_m", float("nan"))
    delta = ate - ref_ate if (not math.isnan(ate) and not math.isnan(ref_ate)) else float("nan")
    delta_str = f"{delta:+.0f} m vs A" if not math.isnan(delta) else ""
    print(f"  {grp} ({GROUP_DESC[grp]:30s}): status={r['status']:10s}  "
          f"ATE={ate:.1f} m  {delta_str}  yaw_rms={yrms:.1f}°  yaw_p95={yp95:.1f}°"
          f"  acc={acc:.1f}  rej_rt={rrt:.2f}")

# ── Full XY trajectories ──────────────────────────────────────────────────────
for flight in (["fly3"] if args.flight else ["fly1","fly3","fly4"]):
    if args.flight and flight != args.flight: continue
    fcfg = FLIGHTS[flight]
    lat0, lon0, alt0 = GPS_REF[flight]
    gps_data = load_gps(fcfg["gps"], lat0, lon0, alt0)
    if gps_data is None: continue
    gps_t, gps_E, gps_N, _ = gps_data
    gm = (gps_t >= fcfg["t0"]) & (gps_t <= fcfg["until"])
    g_t = gps_t[gm]; g_E = gps_E[gm]; g_N = gps_N[gm]

    fig, ax = plt.subplots(figsize=(9,9))
    ax.plot(g_E, g_N, "k-", lw=2.5, label="GPS truth", zorder=10)
    ax.plot(g_E[0], g_N[0], "ks", ms=8, zorder=11)

    for grp in GROUP_ORDER:
        key = (flight, grp)
        if key not in runs: continue
        r = runs[key]
        if r["traj"] is None or r["status"] not in ("OK","PARTIAL"): continue
        traj = r["traj"]
        vio_t = traj[:,0]; vio_E = traj[:,1]; vio_N = traj[:,2]
        # Anchor at t0
        gE0 = interp1(g_t, g_E, fcfg["t0"]); gN0 = interp1(g_t, g_N, fcfg["t0"])
        vE0 = interp1(vio_t, vio_E, fcfg["t0"]); vN0 = interp1(vio_t, vio_N, fcfg["t0"])
        t_ey = fcfg["t0"] + 15.0
        gm_y = (g_t >= fcfg["t0"]) & (g_t <= t_ey)
        vm_y = (vio_t >= fcfg["t0"]) & (vio_t <= t_ey)
        yaw_g = math.atan2(g_E[gm_y][-1]-g_E[gm_y][0], g_N[gm_y][-1]-g_N[gm_y][0]) if gm_y.sum()>1 else 0
        yaw_v = math.atan2(vio_E[vm_y][-1]-vio_E[vm_y][0], vio_N[vm_y][-1]-vio_N[vm_y][0]) if vm_y.sum()>1 else 0
        delta = yaw_g - yaw_v
        cos_d, sin_d = math.cos(delta), math.sin(delta)
        vm = (vio_t >= fcfg["t0"]) & (vio_t <= fcfg["until"])
        ve = (vio_E[vm]-vE0)*cos_d - (vio_N[vm]-vN0)*sin_d + gE0
        vn = (vio_E[vm]-vE0)*sin_d + (vio_N[vm]-vN0)*cos_d + gN0
        ate = r["eval"].get("xy_ate_rms_m", float("nan"))
        ax.plot(ve, vn, "-", color=GROUP_COLOR.get(grp,"grey"), lw=1.3, alpha=0.8,
                label=f"{grp}: {GROUP_DESC[grp]}  ATE={ate:.0f}m ({r['status']})")

    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"{flight} full-window XY (all groups, start_yaw aligned)\n"
                 f"eval [{fcfg['t0']:.0f}–{fcfg['until']:.0f}] s")
    ax.legend(fontsize=7, loc="best"); ax.set_aspect("equal","box"); ax.grid(True, alpha=0.3)
    out_path = f"{args.out}/{flight}_full_xy.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {flight}_full_xy.png")

# ── Decision table ────────────────────────────────────────────────────────────
print("\n" + "═"*80)
print("  DECISION TABLE")
print("═"*80)
print(f"  {'Group':<4}  {'fly1':^14}  {'fly3':^14}  {'fly4':^14}  {'Decision'}")
print("  " + "-"*76)

def decision_tag(runs, grp):
    """Classify a group based on convergence and ATE across primary flights."""
    statuses = []
    ates = []
    for f in ["fly1","fly3","fly4"]:
        k = (f,grp)
        if k not in runs:
            statuses.append("NOT_RUN"); ates.append(float("nan")); continue
        statuses.append(runs[k]["status"])
        ates.append(runs[k]["eval"].get("xy_ate_rms_m", float("nan")))
    baseline_ates = [runs.get(("fly1","A"),{}).get("eval",{}).get("xy_ate_rms_m",float("nan")),
                     runs.get(("fly3","A"),{}).get("eval",{}).get("xy_ate_rms_m",float("nan")),
                     runs.get(("fly4","A"),{}).get("eval",{}).get("xy_ate_rms_m",float("nan"))]

    n_ok       = sum(1 for s in statuses if s == "OK")
    n_div      = sum(1 for s in statuses if s == "DIVERGED")
    n_not_run  = sum(1 for s in statuses if s in ("NOT_RUN",""))

    if n_not_run == 3: return "NOT_RUN"
    if n_div >= 2:    return "REJECT — diverges on ≥2 flights"
    if n_div == 1:    return "REJECT — diverges on 1 flight (not deployable)"

    # All converged — check ATE vs baseline
    improvements = []
    for ate, b_ate in zip(ates, baseline_ates):
        if math.isnan(ate) or math.isnan(b_ate): continue
        improvements.append(ate - b_ate)  # negative = improvement

    avg_delta = sum(improvements)/len(improvements) if improvements else float("nan")
    fly1_delta = (ates[0]-baseline_ates[0]) if not (math.isnan(ates[0]) or math.isnan(baseline_ates[0])) else float("nan")
    fly4_delta = (ates[2]-baseline_ates[2]) if not (math.isnan(ates[2]) or math.isnan(baseline_ates[2])) else float("nan")

    if grp == "D": return "DIAGNOSTIC — σ=1+χ²=5 too loose; starvation workaround not clean"
    if grp == "J": return "SECONDARY — σ=2 + clones=15; baseline style with more context"
    if not math.isnan(fly4_delta) and fly4_delta > 150:
        return "REJECT — fly4 degrades by >150 m (too expensive for high-altitude)"
    if not math.isnan(avg_delta) and avg_delta < -30:
        return "CANDIDATE — avg improvement >30 m across primary group"
    if not math.isnan(avg_delta) and avg_delta < 0:
        return "PROMISING — modest improvement; cross-flight consistent"
    return "NEUTRAL — no clear improvement vs baseline"

for grp in GROUP_ORDER:
    cells = []
    for f in ["fly1","fly3","fly4"]:
        k = (f,grp)
        if k not in runs:
            cells.append(f"{'n/a':^14}"); continue
        r = runs[k]
        ate = r["eval"].get("xy_ate_rms_m", float("nan"))
        st  = r["status"][:3]
        cells.append(f"{ate:5.0f}m {st:^4}   " if not math.isnan(ate) else f"{'---':^5}  {st:^5}  ")
    dec = decision_tag(runs, grp)
    print(f"  {grp:<4}  {cells[0]:^14}  {cells[1]:^14}  {cells[2]:^14}  {dec}")

# ── Final recommendation ──────────────────────────────────────────────────────
print("\n" + "═"*80)
print("  FINAL RECOMMENDATION ANSWERS")
print("═"*80)

answers = {
    "A. Can σ=1 be made non-divergent on fly1?": None,
    "B. Does stable σ=1 improve fly3 first-turn?":  None,
    "C. Does it hurt fly4?":                        None,
    "D. Is the fix deployable as shared setting?":  None,
    "E. If not deployable, what next?":             None,
    "F. Exact configuration for next test?":        None,
}

# Auto-fill based on data
b_run = runs.get(("fly1","B"), {})
c_run = runs.get(("fly1","C"), {})
f_run = runs.get(("fly1","F"), {})
h_run = runs.get(("fly1","H"), {})

fly1_A_ate = runs.get(("fly1","A"),{}).get("eval",{}).get("xy_ate_rms_m", float("nan"))
fly3_A_ate = runs.get(("fly3","A"),{}).get("eval",{}).get("xy_ate_rms_m", float("nan"))
fly4_A_ate = runs.get(("fly4","A"),{}).get("eval",{}).get("xy_ate_rms_m", float("nan"))

# Q-A: Can σ=1 be non-divergent on fly1?
candidates_ok = []
for grp in ["C","E","F","G","H","I"]:
    k = ("fly1",grp)
    if k in runs and runs[k]["status"] == "OK":
        candidates_ok.append(f"{grp}({GROUP_DESC[grp]})")
if candidates_ok:
    answers["A. Can σ=1 be made non-divergent on fly1?"] = \
        f"YES — converges with: {', '.join(candidates_ok)}"
elif b_run.get("status") == "DIVERGED":
    answers["A. Can σ=1 be made non-divergent on fly1?"] = \
        "PARTIAL — raw σ=1 diverges; check C/E/F/G/H results above"
else:
    answers["A. Can σ=1 be made non-divergent on fly1?"] = "INCONCLUSIVE — runs not yet complete"

for q, a in answers.items():
    if a is None: a = "(evaluate from table above)"
    print(f"  {q}")
    print(f"    {a}")
    print()

# ── Save CSV summary ──────────────────────────────────────────────────────────
out_csv = f"{args.out}/sigma1_analysis_summary.csv"
fields = ["flight","group","status","sigma_px","chi2_mult","fi_max_dist","max_clones",
          "num_pts","xy_ate_rms_m","xy_ate_final_m","yaw_err_rms_deg","yaw_err_p95_deg",
          "yaw_err_final_deg","mean_acc","mean_rej","mean_rej_rate","min_acc","n_starvation","first_stv_t"]
with open(out_csv,"w",newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    w.writeheader()
    for _, grp, r in summary_rows:
        em = r["eval"]; pr = r["snap"]
        row = {"flight": r["flight"], "group": grp, "status": r["status"],
               "sigma_px":    pr.get("up_msckf_sigma_px",""),
               "chi2_mult":   pr.get("up_msckf_chi2_multipler",""),
               "fi_max_dist": pr.get("fi_max_dist",""),
               "max_clones":  pr.get("max_clones",""),
               "num_pts":     pr.get("num_pts",""),
               "xy_ate_rms_m":     em.get("xy_ate_rms_m",""),
               "xy_ate_final_m":   em.get("xy_ate_final_m",""),
               "yaw_err_rms_deg":  em.get("yaw_err_rms_deg",""),
               "yaw_err_p95_deg":  em.get("yaw_err_p95_deg",""),
               "yaw_err_final_deg":em.get("yaw_err_final_deg",""),
               "mean_acc":      r.get("mean_acc",""),
               "mean_rej":      r.get("mean_rej",""),
               "mean_rej_rate": r.get("mean_rej_rate",""),
               "min_acc":       r.get("min_acc",""),
               "n_starvation":  r.get("n_starvation",""),
               "first_stv_t":   r.get("first_stv_t",""),
               }
        w.writerow(row)
print(f"Summary CSV: {out_csv}")
print(f"Plots dir:   {args.out}/")
