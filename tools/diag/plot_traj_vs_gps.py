#!/usr/bin/env python3
"""
2x4 grid: rows = method, cols = flight.
Each cell: GPS reference (black) + single VIO trajectory (colour),
aligned by start-position + velocity-heading.
"""
import csv, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = "/mnt/c/Users/baloney/Desktop/traj_vs_gps_all_flights.png"

FLY4_BASE = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528"
FLY3_BASE = "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527"
FLY2_BASE = "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525"
FLY1_BASE = "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result"

FLIGHTS = [
    dict(
        label="Fly 1",
        t0=930.0,
        gps="/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv",
        fej=f"{FLY1_BASE}/fej_oc_prechi2_start930",
        base=f"{FLY1_BASE}/globaloc_alpha1_wrap_start930",
    ),
    dict(
        label="Fly 2",
        t0=700.0,
        gps=f"{FLY2_BASE}/gps_from_mems_offset450p5_cam_time.csv",
        fej=f"{FLY2_BASE}/fej_oc_prechi2_start700",
        base=f"{FLY2_BASE}/clean_ablate_oc_alpha10",
    ),
    dict(
        label="Fly 3",
        t0=618.0,
        gps=f"{FLY3_BASE}/gps_from_mems_offset438p0_cam_time.csv",
        fej=f"{FLY3_BASE}/fej_oc_prechi2_start618",
        base=f"{FLY3_BASE}/phase6_oc_experiments_offset438p0_start618_until1600/global_oc_current_late_baseline_offset438p0_until1600",
    ),
    dict(
        label="Fly 4",
        t0=924.4,
        gps=f"{FLY4_BASE}/gps_from_mems_offsetm202p2_cam_time.csv",
        fej=f"{FLY4_BASE}/fej_oc_prechi2_start924p4",
        base=f"{FLY4_BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1",
    ),
]

# ── Loaders ───────────────────────────────────────────────────────────────────

def load_tum(path):
    ts, xs, ys = [], [], []
    if not os.path.exists(path): return np.array([]), np.array([]), np.array([])
    with open(path) as f:
        for ln in f:
            if not ln.strip() or ln.startswith('#'): continue
            p = ln.split()
            if len(p) >= 4:
                try: ts.append(float(p[0])); xs.append(float(p[1])); ys.append(float(p[2]))
                except ValueError: pass
    return np.array(ts), np.array(xs), np.array(ys)

def load_bias_vel(path):
    """Load (t, vx, vy) from traj.txt.bias  [col 0=t, 1=vx, 2=vy]."""
    ts, vxs, vys = [], [], []
    if not os.path.exists(path): return np.array([]), np.array([]), np.array([])
    with open(path) as f:
        for ln in f:
            if not ln.strip() or ln.startswith('#'): continue
            p = ln.split()
            if len(p) >= 3:
                try: ts.append(float(p[0])); vxs.append(float(p[1])); vys.append(float(p[2]))
                except ValueError: pass
    return np.array(ts), np.array(vxs), np.array(vys)

def load_gps_enu(path, t0):
    """Return (t_s, east, north) centred at GPS position nearest to t0."""
    rows = []
    if not os.path.exists(path): return np.array([]), np.array([]), np.array([])
    with open(path) as f:
        rd = csv.reader(f)
        hdr = next(rd, None)
        for row in rd:
            if not row: continue
            try:
                t = float(row[0])
                if t > 1e9: t /= 1e9
                lat, lon = float(row[1]), float(row[2])
                rows.append((t, lat, lon))
            except (ValueError, IndexError): pass
    if not rows: return np.array([]), np.array([]), np.array([])
    rows.sort(key=lambda x: x[0])
    # flat-earth ENU anchored at first sample
    lat0 = math.radians(rows[0][1]); lon0 = math.radians(rows[0][2]); R = 6371000.0
    ts = np.array([r[0] for r in rows])
    es = np.array([R * math.cos(lat0) * (math.radians(r[2]) - lon0) for r in rows])
    ns = np.array([R * (math.radians(r[1]) - lat0)                   for r in rows])
    # anchor at the sample closest to t0
    idx0 = int(np.argmin(np.abs(ts - t0)))
    es -= es[idx0]; ns -= ns[idx0]
    return ts, es, ns

def vel_heading(ts_bias, vxs, vys, t0, win=5.0, speed_gate=2.0):
    """Heading from velocity vector averaged over [t0, t0+win]."""
    mask = (ts_bias >= t0) & (ts_bias <= t0 + win)
    if mask.sum() < 2:
        return None
    vx = vxs[mask]; vy = vys[mask]
    spd = np.sqrt(vx**2 + vy**2)
    fast = spd > speed_gate
    if fast.sum() == 0: fast = np.ones(len(vx), dtype=bool)
    return math.atan2(float(np.mean(vy[fast])), float(np.mean(vx[fast])))

def gps_heading(ts_gps, es, ns, t0, win=5.0, speed_gate=2.0):
    """Course-over-ground heading from GPS centred at t0."""
    mask = (ts_gps >= t0) & (ts_gps <= t0 + win)
    if mask.sum() < 2: return 0.0
    t_w = ts_gps[mask]; e_w = es[mask]; n_w = ns[mask]
    dt = np.diff(t_w); de = np.diff(e_w); dn = np.diff(n_w)
    spd = np.sqrt(de**2 + dn**2) / np.maximum(dt, 1e-6)
    fast = spd > speed_gate
    if fast.sum() == 0: fast = np.ones(len(de), dtype=bool)
    return math.atan2(float(np.sum(de[fast])), float(np.sum(dn[fast])))

def align(ts_traj, xs, ys, ts_bias, vxs, vys, ts_gps, es_gps, ns_gps, t0):
    """Translate + rotate VIO to match GPS at t0."""
    # VIO start position at t0
    idx_v = int(np.argmin(np.abs(ts_traj - t0)))
    x0, y0 = xs[idx_v], ys[idx_v]

    # headings
    yaw_vio = vel_heading(ts_bias, vxs, vys, t0)
    yaw_gps_val = gps_heading(ts_gps, es_gps, ns_gps, t0)
    if yaw_vio is None: yaw_vio = yaw_gps_val

    dth = yaw_gps_val - yaw_vio
    c, s = math.cos(dth), math.sin(dth)
    dx = xs - x0; dy = ys - y0
    xa = c*dx - s*dy
    ya = s*dx + c*dy
    return xa, ya   # centred at origin = GPS anchor at t0

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(2, 4, figsize=(22, 11))
fig.suptitle(
    "XY Trajectory vs GPS reference — start+velocity-heading alignment\n"
    "Top row: FEJ-OC-prechi2   |   Bottom row: global_oc_alpha1",
    fontsize=12, fontweight="bold", y=1.01,
)

ROWS = [("FEJ-OC-prechi2",   "fej",  "C0"),
        ("global_oc_alpha1", "base", "C2")]

for col, flt in enumerate(FLIGHTS):
    t0    = flt["t0"]
    ts_g, es_g, ns_g = load_gps_enu(flt["gps"], t0)
    # Decimate GPS for visual clarity
    gps_step = max(1, len(ts_g) // 600)

    for row, (mode_label, key, color) in enumerate(ROWS):
        ax = axes[row, col]
        traj_dir = flt[key]
        traj_path = os.path.join(traj_dir, "traj.txt")
        bias_path = os.path.join(traj_dir, "traj.txt.bias")

        # Draw GPS
        if len(ts_g):
            mask = ts_g >= t0
            ax.plot(es_g[mask][::gps_step], ns_g[mask][::gps_step],
                    color="k", lw=1.0, alpha=0.4, zorder=1, label="GPS")
            ax.plot(0, 0, "k^", ms=6, zorder=5)   # GPS start marker

        # Load and align VIO
        ts_t, xs_t, ys_t = load_tum(traj_path)
        ts_b, vxs_b, vys_b = load_bias_vel(bias_path)

        if len(ts_t) and len(ts_b) and len(ts_g):
            try:
                xa, ya = align(ts_t, xs_t, ys_t, ts_b, vxs_b, vys_b,
                               ts_g, es_g, ns_g, t0)
                step = max(1, len(ts_t) // 1000)
                ax.plot(xa[::step], ya[::step], color=color, lw=1.6,
                        alpha=0.85, zorder=2, label=mode_label)
                # Start marker
                idx0 = int(np.argmin(np.abs(ts_t - t0)))
                ax.plot(xa[idx0], ya[idx0], "o", color=color, ms=5, zorder=6)
            except Exception as e:
                ax.text(0.5, 0.5, f"align error\n{e}", ha="center", va="center",
                        transform=ax.transAxes, fontsize=7)
        else:
            ax.text(0.5, 0.5, "MISSING", ha="center", va="center",
                    transform=ax.transAxes, fontsize=8, color="red")

        title = flt["label"] if row == 0 else ""
        if title:
            ax.set_title(f"{flt['label']}  (t₀={t0:.0f} s)", fontsize=9, pad=3)
        ax.set_xlabel("East (m)", fontsize=7); ax.set_ylabel("North (m)", fontsize=7)
        ax.set_aspect("equal"); ax.grid(True, alpha=0.18, lw=0.4)
        ax.tick_params(labelsize=7)
        ax.legend(fontsize=7, loc="best", framealpha=0.8)

        # Row label on leftmost column
        if col == 0:
            ax.set_ylabel(f"{mode_label}\nNorth (m)", fontsize=7)

plt.tight_layout()
plt.savefig(OUT, dpi=130, bbox_inches="tight")
print(f"Saved: {OUT}")
