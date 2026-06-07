#!/usr/bin/env python3
"""
tools/diag/eval_modes_vs_gps.py — Canonical multi-flight, multi-mode GPS evaluator.

Alignment (per-run, fully independent — no cross-mode fitting):
  p_aligned(t) = R(yaw_gps0 - yaw_vio_quat0) × (p_vio(t) − p_vio(t0)) + p_gps(t0)

  yaw_gps0       : circular-mean GPS course in [t0, t0+5 s], speed-gated ≥ 2 m/s
                   GPS course = atan2(dE, dN) — bearing conv. (N=0°, E=+90°)
  yaw_vio_quat0  : VIO quaternion yaw at t0  ← PRIMARY alignment yaw
  yaw_vio_vel0   : atan2(vx, vy) from .bias sidecar at t0  ← SANITY CHECK only

No Umeyama, no SE(2)/Sim(2), no scale fitting, no pairwise A→B alignment.

Validity checks:
  run_complete    : final_t ≥ until − 60 s
  gps_yaw_n_samp  : number of speed-gated GPS samples used for yaw_gps0
  rms_30s         : XY RMS in first 30 s — direct alignment quality probe

Outputs (all in OUT_DIR):
  alignment_debug.txt  — per (flight, mode) alignment table
  bgz_summary.csv      — bg_z stats per run
  metrics_summary.csv  — XY ATE + yaw metrics per run
  bgz_all_flights.png  — bg_z timelines (4 flights, modes overlaid)
  traj_all_flights.png — 4 rows (modes) × 4 cols (flights), GPS + one VIO each

Run from WSL:
  python3 tools/diag/eval_modes_vs_gps.py
"""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Output directory ──────────────────────────────────────────────────────────
OUT_DIR = "/mnt/c/Users/baloney/Desktop/eval_modes_out"
os.makedirs(OUT_DIR, exist_ok=True)

# ── Constants ─────────────────────────────────────────────────────────────────
SPEED_GATE = 2.0   # m/s
YAW_WIN    = 5.0   # s — GPS initial-yaw window [t0, t0+YAW_WIN]
EARLY_END  = 60.0  # s — flag invalid if final_t < until − EARLY_END

# ── Mode display colours (consistent across all plots) ────────────────────────
MODE_COLORS = {
    "global_oc_alpha1":          "#e05a2b",
    "global_yaw_oc_fej_prechi2": "#2b6de0",
    "B_current":                 "#2ca02c",
    "hard_gyro_yaw":             "#9467bd",
}
ALL_MODES = list(MODE_COLORS)  # display row order

# ── Base paths (WSL) ──────────────────────────────────────────────────────────
FLY1 = "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result"
FLY2 = "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525"
FLY3 = "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527"
FLY4 = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528"
GPS1 = "/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv"

def _bp(t, b=None):
    """Build {"traj": t, "bias": b} dict; infer bias path as traj+'.bias' if omitted."""
    if b is None and t is not None:
        b = t + ".bias"
    return {"traj": t, "bias": b}

# ── Flight + mode configuration ───────────────────────────────────────────────
# until = evaluation end time (seconds, camera time).
# Trajectory data beyond 'until' is ignored in metric windows.
FLIGHTS = [
    {
        "name":  "fly1",
        "label": "Fly 1  (t0=930 s, until≈1750 s)",
        "gps":   GPS1,
        "t0":    930.0,
        "until": 1750.0,
        "modes": {
            # bare file — bias sidecar at same path + '.bias'
            "global_oc_alpha1":          _bp(f"{FLY1}/A1_fcinit_px2_globaloc1p0_traj.txt"),
            "global_yaw_oc_fej_prechi2": _bp(f"{FLY1}/fej_oc_prechi2_start930/traj.txt"),
            "B_current":                 _bp(None),
            "hard_gyro_yaw":             _bp(None),
        },
    },
    {
        "name":  "fly2",
        "label": "Fly 2  (t0=700 s, until≈2600 s)",
        "gps":   f"{FLY2}/gps_from_mems_offset450p5_cam_time.csv",
        "t0":    700.0,
        "until": 2600.0,
        "modes": {
            # KNOWN EARLY TERMINATION at t≈1352 s — will be flagged INVALID
            "global_oc_alpha1":          _bp(f"{FLY2}/clean_ablate_oc_alpha10/traj.txt"),
            "global_yaw_oc_fej_prechi2": _bp(f"{FLY2}/fej_oc_prechi2_start700/traj.txt"),
            "B_current":                 _bp(None),
            "hard_gyro_yaw":             _bp(None),
        },
    },
    {
        "name":  "fly3",
        "label": "Fly 3  (t0=618 s, until≈1600 s)",
        "gps":   f"{FLY3}/gps_from_mems_offset438p0_cam_time.csv",
        "t0":    618.0,
        "until": 1600.0,
        "modes": {
            # stage_schmidt_A is the canonical global_oc baseline for fly3 eval
            "global_oc_alpha1":          _bp(f"{FLY3}/stage_schmidt_A_global_oc_offset438p0_start618_until1600/traj.txt"),
            "global_yaw_oc_fej_prechi2": _bp(f"{FLY3}/fej_oc_prechi2_start618/traj.txt"),
            "B_current":                 _bp(f"{FLY3}/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until1600/traj.txt"),
            "hard_gyro_yaw":             _bp(f"{FLY3}/yaw_method_viz_offset438p0_start618/hard_gyro_yaw_gpsz_viz/traj.txt"),
        },
    },
    {
        "name":  "fly4",
        "label": "Fly 4  (t0=924 s, until≈2816 s)",
        "gps":   f"{FLY4}/gps_from_mems_offsetm202p2_cam_time.csv",
        "t0":    924.4,
        "until": 2816.0,
        "modes": {
            # global_oc run started at t=904.4; we evaluate from t0=924.4
            "global_oc_alpha1":          _bp(f"{FLY4}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/traj.txt"),
            "global_yaw_oc_fej_prechi2": _bp(f"{FLY4}/fej_oc_prechi2_start924p4/traj.txt"),
            "B_current":                 _bp(f"{FLY4}/fly4_B_bgz1p0_start924p4_until2816/traj.txt"),
            "hard_gyro_yaw":             _bp(None),
        },
    },
]

# ── Helpers ───────────────────────────────────────────────────────────────────

def wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0):
    RAD = math.pi / 180.0
    dN = (lat - lat0) * 111320.0
    dE = (lon - lon0) * 111320.0 * math.cos(lat0 * RAD)
    return dE, dN, alt - alt0

def wrap180(a):
    a = float(a)
    while a >  180: a -= 360
    while a < -180: a += 360
    return a

def quat_to_yaw(qx, qy, qz, qw):
    """OpenVINS JPL q_GtoI → IMU heading in global ENU.
    Bearing convention: N=0°, E=+90°.
    Derivation: conjugate to get q_ItoG, extract yaw around z-axis."""
    return math.atan2(-2.0*qw*qz + 2.0*qx*qy,
                       1.0 - 2.0*(qy**2 + qz**2))

def interp1(xs, ys, x):
    xs = np.asarray(xs, float); ys = np.asarray(ys, float)
    if len(xs) == 0: return float('nan')
    i = np.searchsorted(xs, x)
    if i == 0:         return float(ys[0])
    if i >= len(xs):   return float(ys[-1])
    a = (x - xs[i-1]) / (xs[i] - xs[i-1])
    return float(ys[i-1] + a*(ys[i] - ys[i-1]))

def rms(v):
    a = np.asarray(v, float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a**2))) if len(a) else float('nan')

def p95(v):
    a = np.asarray(v, float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, 95)) if len(a) else float('nan')

def fmt(x, d=2):
    return f"{x:.{d}f}" if math.isfinite(x) else "nan"

def load_traj(path):
    """TUM format → N×8 [t, x, y, z, qx, qy, qz, qw]. Returns empty if missing."""
    if not path or not os.path.exists(path):
        return np.zeros((0, 8))
    rows = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'): continue
            c = s.split()
            if len(c) >= 8:
                try: rows.append([float(x) for x in c[:8]])
                except ValueError: pass
    return np.array(rows) if rows else np.zeros((0, 8))

def load_bias(path):
    """Bias sidecar → N×10 [t, vx, vy, vz, bgx, bgy, bgz, ...]. Returns empty if missing."""
    if not path or not os.path.exists(path):
        return np.zeros((0, 10))
    rows = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith('#'): continue
            c = s.split()
            if len(c) >= 7:
                try:
                    r = [float(x) for x in c[:min(len(c), 10)]]
                    rows.append(r + [0.0]*(10-len(r)))
                except ValueError: pass
    return np.array(rows) if rows else np.zeros((0, 10))

def load_gps(path, t0, until):
    """Load GPS CSV (ts_ns, lat, lon, alt) → ENU arrays anchored near t0.
    Returns (t_s, E, N, U, (lat0, lon0, alt0)) or empty arrays if missing."""
    if not path or not os.path.exists(path):
        return np.array([]), np.array([]), np.array([]), np.array([]), None
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                t = float(r.get('ts_ns') or list(r.values())[0])
                if t > 1e9: t /= 1e9
                rows.append((t, float(r['lat']), float(r['lon']), float(r.get('alt', 0.0))))
            except (ValueError, KeyError): pass
    if not rows: return np.array([]), np.array([]), np.array([]), np.array([]), None
    rows.sort(key=lambda x: x[0])
    # ENU reference: first GPS within ±30 s of t0
    lat0 = lon0 = alt0 = None
    for t, lat, lon, alt in rows:
        if t0 - 30 <= t <= t0 + 30:
            lat0, lon0, alt0 = lat, lon, alt
            break
    if lat0 is None:
        lat0, lon0, alt0 = rows[0][1], rows[0][2], rows[0][3]
    ts, Es, Ns, Us = [], [], [], []
    for t, lat, lon, alt in rows:
        if t0 - 30 <= t <= until + 120:
            e, n, u = wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0)
            ts.append(t); Es.append(e); Ns.append(n); Us.append(u)
    return (np.array(ts), np.array(Es), np.array(Ns), np.array(Us),
            (lat0, lon0, alt0))

# ── Main evaluation loop ──────────────────────────────────────────────────────

results    = []   # one dict per (flight, mode) — in FLIGHTS × ALL_MODES order
debug_rows = []

for flight in FLIGHTS:
    fname = flight["name"]
    t0    = flight["t0"]
    until = flight["until"]

    gps_t, gps_E, gps_N, gps_U, gps_ref = load_gps(flight["gps"], t0, until)
    gps_ok = len(gps_t) >= 10

    # GPS start position at t0
    p_gps_t0 = np.array([interp1(gps_t, gps_E, t0),
                          interp1(gps_t, gps_N, t0)]) if gps_ok else np.array([0.0, 0.0])

    # GPS course time-series (atan2(dE,dN)) — bearing convention
    gc_t, gc_v, gc_spd = [], [], []
    if gps_ok:
        for i in range(1, len(gps_t)):
            dt = gps_t[i] - gps_t[i-1]
            if dt <= 0: continue
            dE = gps_E[i] - gps_E[i-1]; dN = gps_N[i] - gps_N[i-1]
            spd = math.sqrt(dE**2 + dN**2) / dt
            gc_t.append(0.5*(gps_t[i]+gps_t[i-1]))
            gc_v.append(math.atan2(dE, dN))
            gc_spd.append(spd)
    gc_t   = np.array(gc_t);   gc_v   = np.array(gc_v)
    gc_spd = np.array(gc_spd); speed_mask = gc_spd >= SPEED_GATE

    # GPS yaw at t0: circular mean in speed-gated [t0, t0+YAW_WIN]
    yaw_win = speed_mask & (gc_t >= t0) & (gc_t <= t0 + YAW_WIN)
    n_yaw   = int(yaw_win.sum())
    if n_yaw > 0:
        sin_s = np.sum(np.sin(gc_v[yaw_win])); cos_s = np.sum(np.cos(gc_v[yaw_win]))
        yaw_gps0 = math.atan2(sin_s, cos_s)
    elif speed_mask.sum() > 0:
        yaw_gps0 = float(gc_v[np.argmax(speed_mask)]); n_yaw = 1
    else:
        yaw_gps0 = 0.0; n_yaw = 0

    for mname in ALL_MODES:
        mcfg = flight["modes"].get(mname, _bp(None))

        traj = load_traj(mcfg["traj"])
        bias = load_bias(mcfg["bias"])
        has_traj = len(traj) >= 10
        has_bias = len(bias) >= 10

        # Status
        if mcfg["traj"] is None:
            status = "NOT_RUN"
        elif not os.path.exists(mcfg["traj"]):
            status = "MISSING"
        elif not has_traj:
            status = "EMPTY"
        else:
            status = "OK"

        # VIO quaternion yaw at t0 (body heading — differs from track by sideslip)
        quat_yaws = np.array([]) if not has_traj else \
            np.array([quat_to_yaw(*traj[i, 4:8]) for i in range(len(traj))])
        yaw_vio_quat0 = interp1(traj[:, 0], quat_yaws, t0) if has_traj else float('nan')

        # VIO velocity yaw at t0 (kinematic heading = GPS course — PRIMARY for alignment).
        # atan2(vx, vy) matches GPS atan2(dE, dN) because vx=East vel, vy=North vel.
        # ~0.03 deg residual vs ~5.5 deg for quaternion (sideslip angle).
        if has_bias:
            vx0 = interp1(bias[:, 0], bias[:, 1], t0)
            vy0 = interp1(bias[:, 0], bias[:, 2], t0)
            spd0 = math.sqrt(vx0**2 + vy0**2) if math.isfinite(vx0+vy0) else 0.0
            yaw_vio_vel0 = math.atan2(vx0, vy0) if spd0 > 1.0 else float('nan')
        else:
            vx0 = vy0 = spd0 = float('nan')
            yaw_vio_vel0 = float('nan')

        # Choose primary alignment yaw: velocity if available (speed-gated), else quaternion
        if math.isfinite(yaw_vio_vel0):
            yaw_vio_primary = yaw_vio_vel0
            primary_src = 'vel'
        else:
            yaw_vio_primary = yaw_vio_quat0
            primary_src = 'quat(fallback)'

        # Delta yaws (primary and sanity)
        d_primary = (yaw_gps0 - yaw_vio_primary)  if math.isfinite(yaw_vio_primary) else float('nan')
        dq        = (yaw_gps0 - yaw_vio_quat0)    if math.isfinite(yaw_vio_quat0)   else float('nan')
        dv        = (yaw_gps0 - yaw_vio_vel0)      if math.isfinite(yaw_vio_vel0)    else float('nan')

        # Alignment: R(d_primary) * (p_vio - p_vio(t0)) + p_gps(t0)
        p_vio_t0 = np.array([float('nan')]*2)
        p_aln     = None
        yaw_vio_aln = None
        if has_traj and math.isfinite(d_primary) and gps_ok:
            ts = traj[:, 0]
            p_vio_t0 = np.array([interp1(ts, traj[:, 1], t0),
                                  interp1(ts, traj[:, 2], t0)])
            R = np.array([[math.cos(d_primary), -math.sin(d_primary)],
                          [math.sin(d_primary),  math.cos(d_primary)]])
            p_rel = np.stack([traj[:, 1] - p_vio_t0[0],
                              traj[:, 2] - p_vio_t0[1]], axis=1)
            p_aln = (R @ p_rel.T).T + p_gps_t0   # N×2, ENU
            # Body heading in GPS frame uses the quaternion offset from the primary
            yaw_vio_aln = quat_yaws + d_primary

        # XY ATE — first 30 s (alignment sanity)
        rms30 = float('nan')
        if p_aln is not None:
            ts = traj[:, 0]
            m30 = (ts >= t0) & (ts <= t0 + 30)
            if m30.sum() >= 2:
                gE30 = np.array([interp1(gps_t, gps_E, t) for t in ts[m30]])
                gN30 = np.array([interp1(gps_t, gps_N, t) for t in ts[m30]])
                rms30 = rms(np.sqrt((p_aln[m30, 0]-gE30)**2 + (p_aln[m30, 1]-gN30)**2))

        # XY ATE — full window [t0, until]
        ate_rms = ate_max = ate_p95 = ate_final = float('nan')
        if p_aln is not None:
            ts   = traj[:, 0]
            wmask = (ts >= t0) & (ts <= until)
            if wmask.sum() >= 2:
                gEw = np.array([interp1(gps_t, gps_E, t) for t in ts[wmask]])
                gNw = np.array([interp1(gps_t, gps_N, t) for t in ts[wmask]])
                errs = np.sqrt((p_aln[wmask, 0]-gEw)**2 + (p_aln[wmask, 1]-gNw)**2)
                ate_rms  = rms(errs)
                ate_max  = float(np.max(errs))
                ate_p95  = p95(errs)
                ate_final = float(errs[-1])

        # Yaw error vs GPS course — speed-gated, full [t0, until]
        yaw_err_rms = yaw_err_max = yaw_err_final = float('nan')
        if yaw_vio_aln is not None:
            ts   = traj[:, 0]
            gated = speed_mask & (gc_t >= t0) & (gc_t <= until)
            if gated.sum() >= 2:
                vio_at_gc = np.array([interp1(ts, yaw_vio_aln, t) for t in gc_t[gated]])
                ye = np.array([wrap180(math.degrees(float(v)-float(g)))
                                for v, g in zip(vio_at_gc, gc_v[gated])])
                yaw_err_rms  = rms(ye)
                yaw_err_max  = float(np.max(np.abs(ye)))
                yaw_err_final = float(ye[-1])

        # bg_z stats — window [t0, until]
        bgz_mean = bgz_final = bgz_min = bgz_max = float('nan')
        bgz_ts_full = bgz_arr_full = None
        if has_bias:
            bts  = bias[:, 0];  bgz = bias[:, 6]
            wm   = (bts >= t0) & (bts <= until)
            if wm.sum() >= 2:
                bgz_w    = bgz[wm]
                bgz_mean  = float(np.mean(bgz_w))
                bgz_final = float(bgz_w[-1])
                bgz_min   = float(np.min(bgz_w))
                bgz_max   = float(np.max(bgz_w))
            bgz_ts_full  = bts;  bgz_arr_full = bgz  # for plotting

        # Validity
        final_t      = float(traj[-1, 0]) if has_traj else float('nan')
        run_complete = has_traj and (final_t >= until - EARLY_END)
        nan_count    = int(np.sum(~np.isfinite(traj))) if has_traj else 0

        # ── Debug row ──
        debug_rows.append({
            "flight":              fname,
            "mode":                mname,
            "t0":                  t0,
            "until":               until,
            "gps_start_EN_m":      f"({fmt(p_gps_t0[0])},{fmt(p_gps_t0[1])})" if gps_ok else "N/A",
            "vio_start_XY_m":      f"({fmt(p_vio_t0[0])},{fmt(p_vio_t0[1])})" if has_traj and math.isfinite(p_vio_t0[0]) else "N/A",
            "yaw_gps0_deg":        fmt(math.degrees(yaw_gps0)),
            "yaw_vio_quat0_deg":   fmt(math.degrees(yaw_vio_quat0)) if math.isfinite(yaw_vio_quat0) else "nan",
            "yaw_vio_vel0_deg":    fmt(math.degrees(yaw_vio_vel0))  if math.isfinite(yaw_vio_vel0)  else "nan",
            "primary_src":         primary_src,
            "delta_yaw_vel_deg":   fmt(math.degrees(dv)) if math.isfinite(dv) else "nan",
            "delta_yaw_quat_deg":  fmt(math.degrees(dq)) if math.isfinite(dq) else "nan",
            "sideslip_deg":        fmt(math.degrees(dq - dv)) if (math.isfinite(dq) and math.isfinite(dv)) else "nan",
            "rms_30s_m":           fmt(rms30),
            "gps_yaw_n_samp":      n_yaw,
            "status":              status,
            "run_complete":        run_complete,
            "final_t":             fmt(final_t, 1),
        })

        results.append({
            "flight": fname, "label": flight["label"], "mode": mname,
            "status": status, "run_complete": run_complete,
            "t0": t0, "until": until, "final_t": final_t,
            "ate_rms": ate_rms, "ate_max": ate_max, "ate_p95": ate_p95, "ate_final": ate_final,
            "yaw_rms": yaw_err_rms, "yaw_max": yaw_err_max, "yaw_final": yaw_err_final,
            "bgz_mean": bgz_mean, "bgz_final": bgz_final, "bgz_min": bgz_min, "bgz_max": bgz_max,
            "nan_count": nan_count,
            # arrays for plotting
            "gps_t": gps_t, "gps_E": gps_E, "gps_N": gps_N,
            "traj_ts": traj[:, 0] if has_traj else np.array([]),
            "p_aln": p_aln,
            "bgz_ts": bgz_ts_full,
            "bgz_arr": bgz_arr_full,
        })

# ── Print alignment debug table ───────────────────────────────────────────────
hdr = (f"{'flight':<6} {'mode':<30} {'src':<5} {'yaw_gps0':>9} {'vel0':>7} {'quat0':>7} "
       f"{'d_vel':>6} {'d_quat':>7} {'sideslip':>8} {'rms30s':>7} {'n':>3} {'status':<8} {'done'}")
SEP = "─" * 115
print("\n" + "═"*115)
print("  ALIGNMENT DEBUG TABLE")
print("  PRIMARY: velocity yaw atan2(vx,vy) — matches GPS course (kinematic heading).")
print("  sideslip = quat_heading − vel_heading (body vs track; expected 3-8° at cruise).")
print("  If primary=quat(fallback) and sideslip is large → alignment may show directional scale diff.")
print("═"*115)
print(hdr); print(SEP)
for d in debug_rows:
    run_str = "YES" if d["run_complete"] else ("NO" if d["status"]=="OK" else "—")
    print(f"{d['flight']:<6} {d['mode']:<30} {d.get('primary_src','?'):<5} "
          f"{d['yaw_gps0_deg']:>9} {d['yaw_vio_vel0_deg']:>7} {d['yaw_vio_quat0_deg']:>7} "
          f"{d['delta_yaw_vel_deg']:>6} {d['delta_yaw_quat_deg']:>7} "
          f"{d.get('sideslip_deg','nan'):>8} "
          f"{d['rms_30s_m']:>7} {d['gps_yaw_n_samp']:>3} {d['status']:<8} {run_str}")

with open(os.path.join(OUT_DIR, "alignment_debug.txt"), "w") as f:
    f.write("ALIGNMENT DEBUG TABLE\n")
    f.write("Primary: velocity yaw atan2(vx,vy). Sideslip = quat-vel (body vs track angle).\n\n")
    f.write(hdr + "\n" + SEP + "\n")
    for d in debug_rows:
        run_str = "YES" if d["run_complete"] else ("NO" if d["status"]=="OK" else "—")
        f.write(f"{d['flight']:<6} {d['mode']:<30} {d.get('primary_src','?'):<5} "
                f"{d['yaw_gps0_deg']:>9} {d['yaw_vio_vel0_deg']:>7} {d['yaw_vio_quat0_deg']:>7} "
                f"{d['delta_yaw_vel_deg']:>6} {d['delta_yaw_quat_deg']:>7} "
                f"{d.get('sideslip_deg','nan'):>8} "
                f"{d['rms_30s_m']:>7} {d['gps_yaw_n_samp']:>3} {d['status']:<8} {run_str}\n")
print(f"Wrote: {OUT_DIR}/alignment_debug.txt")

# ── bgz_summary.csv ───────────────────────────────────────────────────────────
bgz_rows = [["flight","mode","status","run_complete","final_t","until",
             "bgz_mean","bgz_final","bgz_min","bgz_max",
             "bgz_final_x1e4","bgz_mean_x1e4","nan_count"]]
for r in results:
    def fs(x, d=6): return fmt(x, d) if math.isfinite(x) else "nan"
    bgz_rows.append([
        r["flight"], r["mode"], r["status"], int(r["run_complete"]),
        fmt(r["final_t"], 1), fmt(r["until"], 1),
        fs(r["bgz_mean"]), fs(r["bgz_final"]), fs(r["bgz_min"]), fs(r["bgz_max"]),
        fs(r["bgz_final"]*1e4, 4) if math.isfinite(r["bgz_final"]) else "nan",
        fs(r["bgz_mean"]*1e4,  4) if math.isfinite(r["bgz_mean"])  else "nan",
        r["nan_count"],
    ])
with open(os.path.join(OUT_DIR, "bgz_summary.csv"), "w", newline="") as f:
    csv.writer(f).writerows(bgz_rows)
print(f"Wrote: {OUT_DIR}/bgz_summary.csv")

# ── metrics_summary.csv ───────────────────────────────────────────────────────
met_rows = [["flight","mode","status","run_complete","final_t","until",
             "ate_rms_m","ate_max_m","ate_p95_m","ate_final_m",
             "yaw_err_rms_deg","yaw_err_max_deg","yaw_err_final_deg",
             "bgz_final","bgz_mean"]]
for r in results:
    def fs(x, d=2): return fmt(x, d) if math.isfinite(x) else "nan"
    met_rows.append([
        r["flight"], r["mode"], r["status"], int(r["run_complete"]),
        fmt(r["final_t"], 1), fmt(r["until"], 1),
        fs(r["ate_rms"]), fs(r["ate_max"]), fs(r["ate_p95"]), fs(r["ate_final"]),
        fs(r["yaw_rms"]), fs(r["yaw_max"]), fs(r["yaw_final"]),
        fs(r["bgz_final"], 6), fs(r["bgz_mean"], 6),
    ])
with open(os.path.join(OUT_DIR, "metrics_summary.csv"), "w", newline="") as f:
    csv.writer(f).writerows(met_rows)
print(f"Wrote: {OUT_DIR}/metrics_summary.csv")

# ── bgz_all_flights.png ───────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(14, 13), squeeze=True)
for ax, flight in zip(axes, FLIGHTS):
    fname = flight["name"];  t0 = flight["t0"];  until = flight["until"]
    ax.set_title(flight["label"], fontsize=9, fontweight="bold")
    ax.axhline(0, color="k", lw=0.5, ls="--", alpha=0.35)
    plotted = False
    for mname in ALL_MODES:
        r = next((x for x in results if x["flight"]==fname and x["mode"]==mname), None)
        if r is None or r["bgz_ts"] is None: continue
        bts = r["bgz_ts"]; bgz = r["bgz_arr"]
        wm  = (bts >= t0 - 5) & (bts <= until + 5)
        if wm.sum() < 2: continue
        lbl = f"{mname}" + ("  [INVALID]" if not r["run_complete"] and r["status"]=="OK" else "")
        ax.plot(bts[wm], bgz[wm], "-", color=MODE_COLORS[mname],
                lw=0.85, alpha=0.85, label=lbl)
        plotted = True
    if plotted:
        ax.legend(fontsize=7, loc="upper right", framealpha=0.85)
    else:
        ax.text(0.5, 0.5, "no bias data", transform=ax.transAxes,
                ha="center", va="center", color="gray", fontsize=9)
    ax.set_xlim(t0 - 10, until + 10)
    ax.set_ylabel("bg_z (rad/s)", fontsize=8)
    ax.grid(True, alpha=0.25)
    ax.tick_params(labelsize=8)
axes[-1].set_xlabel("t (s, camera time)", fontsize=8)
fig.suptitle("bg_z (gyro-z bias) timelines — all flights, all modes\n"
             "(alignment: quat-primary start+yaw only)", fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "bgz_all_flights.png"), dpi=140, bbox_inches="tight")
plt.close(fig)
print(f"Wrote: {OUT_DIR}/bgz_all_flights.png")

# ── traj_all_flights.png  (rows=modes, cols=flights) ─────────────────────────
n_modes   = len(ALL_MODES)
n_flights = len(FLIGHTS)
fig, axes = plt.subplots(n_modes, n_flights,
                         figsize=(5.5*n_flights, 4.5*n_modes),
                         squeeze=False)

for ri, mname in enumerate(ALL_MODES):
    for ci, flight in enumerate(FLIGHTS):
        ax    = axes[ri][ci]
        fname = flight["name"];  t0 = flight["t0"];  until = flight["until"]
        r = next((x for x in results if x["flight"]==fname and x["mode"]==mname), None)

        # GPS reference
        gps_t_ = r["gps_t"] if r else np.array([])
        gps_E_ = r["gps_E"] if r else np.array([])
        gps_N_ = r["gps_N"] if r else np.array([])
        if len(gps_t_) > 0:
            gm = (gps_t_ >= t0) & (gps_t_ <= until)
            if gm.sum() > 0:
                step = max(1, int(gm.sum()) // 500)
                ax.plot(gps_E_[gm][::step], gps_N_[gm][::step],
                        color="k", lw=1.1, alpha=0.35, label="GPS", zorder=1)
                ax.plot(gps_E_[gm][0],  gps_N_[gm][0],  "k^", ms=5, zorder=5)
                ax.plot(gps_E_[gm][-1], gps_N_[gm][-1], "ks", ms=5, zorder=5)

        # VIO trajectory
        clr = MODE_COLORS.get(mname, "#888")
        if r and r["status"] == "OK" and r["p_aln"] is not None:
            ts_  = r["traj_ts"]; p_  = r["p_aln"]
            vm   = (ts_ >= t0) & (ts_ <= until)
            if vm.sum() > 0:
                step = max(1, int(vm.sum()) // 800)
                ax.plot(p_[vm][::step, 0], p_[vm][::step, 1],
                        "-", color=clr, lw=1.0, alpha=0.85, zorder=3)
                ax.plot(p_[vm][0, 0], p_[vm][0, 1], "o", color=clr, ms=4, zorder=6)
            # ATE / validity annotation
            valid_str = "VALID" if r["run_complete"] else "INVALID(early)"
            ate_str   = f"ATE {fmt(r['ate_rms'])} m" if math.isfinite(r["ate_rms"]) else "ATE nan"
            yaw_str   = f"yaw {fmt(r['yaw_rms'])}°"  if math.isfinite(r["yaw_rms"]) else "yaw nan"
            ax.set_xlabel(f"{valid_str}\n{ate_str}  {yaw_str}", fontsize=7)
        elif r and r["status"] == "MISSING":
            ax.text(0.5, 0.5, "MISSING\nfile", transform=ax.transAxes,
                    ha="center", va="center", color="red", fontsize=9)
        else:
            ax.text(0.5, 0.5, r["status"] if r else "not configured",
                    transform=ax.transAxes, ha="center", va="center",
                    color="gray", fontsize=8)

        # Row/column labels
        if ci == 0:
            ax.set_ylabel(mname.replace("_", "\n"), fontsize=7, labelpad=2)
        if ri == 0:
            ax.set_title(flight["label"].split("(")[0].strip(), fontsize=9, fontweight="bold")

        ax.set_aspect("equal")
        ax.grid(True, alpha=0.2)
        ax.tick_params(labelsize=7)

fig.suptitle("GPS-aligned XY trajectories — start+yaw only, quat primary, no global fit\n"
             "Black: GPS reference  |  Colour: VIO aligned  |  △=start  □=end",
             fontsize=11)
fig.tight_layout()
fig.savefig(os.path.join(OUT_DIR, "traj_all_flights.png"), dpi=130, bbox_inches="tight")
plt.close(fig)
print(f"Wrote: {OUT_DIR}/traj_all_flights.png")

# ── Validity summary ──────────────────────────────────────────────────────────
print("\n" + "═"*75)
print("  VALIDITY SUMMARY")
print("═"*75)
for r in results:
    if r["status"] == "NOT_RUN": continue
    tag = "VALID  " if r["run_complete"] and r["status"]=="OK" else "INVALID"
    print(f"  [{tag}]  {r['flight']}  {r['mode']:<30}  "
          f"final_t={fmt(r['final_t'],1)}  until={fmt(r['until'],1)}")

print(f"\nAll outputs in: {OUT_DIR}")
