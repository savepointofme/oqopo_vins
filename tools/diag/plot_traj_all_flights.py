#!/usr/bin/env python3
"""XY trajectory comparison for fly1-fly4: FEJ-OC-prechi2 vs global_oc_alpha1 + GPS."""
import os, csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = "/mnt/c/Users/baloney/Desktop/traj_all_flights.png"

# ── Flight definitions ────────────────────────────────────────────────────────
FLY4_BASE = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528"
FLY3_BASE = "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527"
FLY2_BASE = "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525"
FLY1_BASE = "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result"

FLIGHTS = {
    "fly1": {
        "gps":   "/mnt/d/vscode_dir/open_vins/config/d455_fly1/fc_gps_cam_time.csv",
        "t0":    930.0,
        "trajs": [
            ("FEJ-OC-prechi2",   f"{FLY1_BASE}/fej_oc_prechi2_start930/traj.txt",   "C0", 1.8, "-"),
            ("global_oc_alpha1", f"{FLY1_BASE}/A1_fcinit_px2_globaloc1p0_traj.txt", "C2", 1.5, "--"),
        ],
    },
    "fly2": {
        "gps":   f"{FLY2_BASE}/gps_from_mems_offset450p5_cam_time.csv",
        "t0":    700.0,
        "trajs": [
            ("FEJ-OC-prechi2",   f"{FLY2_BASE}/fej_oc_prechi2_start700/traj.txt",       "C0", 1.8, "-"),
            ("global_oc_alpha1", f"{FLY2_BASE}/clean_ablate_oc_alpha10/traj.txt",        "C2", 1.5, "--"),
        ],
    },
    "fly3": {
        "gps":   f"{FLY3_BASE}/gps_from_mems_offset438p0_cam_time.csv",
        "t0":    618.0,
        "trajs": [
            ("FEJ-OC-prechi2",   f"{FLY3_BASE}/fej_oc_prechi2_start618/traj.txt",  "C0", 1.8, "-"),
            ("global_oc_alpha1", f"{FLY3_BASE}/phase6_oc_experiments_offset438p0_start618_until1600/global_oc_current_late_baseline_offset438p0_until1600/traj.txt", "C2", 1.5, "--"),
        ],
    },
    "fly4": {
        "gps":   f"{FLY4_BASE}/gps_from_mems_offsetm202p2_cam_time.csv",
        "t0":    924.4,
        "trajs": [
            ("FEJ-OC-prechi2",   f"{FLY4_BASE}/fej_oc_prechi2_start924p4/traj.txt",  "C0", 1.8, "-"),
            ("global_oc_alpha1", f"{FLY4_BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/traj.txt", "C2", 1.5, "--"),
            ("B_current",        f"{FLY4_BASE}/fly4_B_bgz1p0_start924p4_until2816/traj.txt", "C3", 1.2, ":"),
        ],
    },
}

FLIGHT_LABELS = {
    "fly1": "Fly 1  (t₀=930 s, until=1750 s)",
    "fly2": "Fly 2  (t₀=700 s, until=2600 s)",
    "fly3": "Fly 3  (t₀=618 s, until=1800 s)",
    "fly4": "Fly 4  (t₀=924 s, until=2816 s)",
}

# ── Loaders ───────────────────────────────────────────────────────────────────

def load_tum(path):
    """Load TUM traj: returns arrays t, x, y, z."""
    ts, xs, ys, zs = [], [], [], []
    if not os.path.exists(path):
        return np.array([]), np.array([]), np.array([]), np.array([])
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith('#'): continue
            p = line.split()
            if len(p) >= 4:
                try:
                    ts.append(float(p[0])); xs.append(float(p[1]))
                    ys.append(float(p[2])); zs.append(float(p[3]))
                except ValueError: pass
    return np.array(ts), np.array(xs), np.array(ys), np.array(zs)

def load_gps_enu(path):
    """Load GPS CSV (ts_ns, lat, lon, alt) → (t_s, east, north) in ENU."""
    rows = []
    if not os.path.exists(path): return np.array([]), np.array([]), np.array([])
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                t = float(list(r.values())[0])
                if t > 1e9: t /= 1e9
                lat = float(r.get('lat', list(r.values())[1]))
                lon = float(r.get('lon', list(r.values())[2]))
                rows.append((t, lat, lon))
            except (ValueError, IndexError): pass
    if not rows: return np.array([]), np.array([]), np.array([])
    rows.sort(key=lambda x: x[0])
    lat0 = math.radians(rows[0][1]); lon0 = math.radians(rows[0][2])
    R = 6371000.0
    ts, es, ns = [], [], []
    for t, lat, lon in rows:
        dlat = math.radians(lat) - lat0
        dlon = math.radians(lon) - lon0
        ts.append(t)
        es.append(R * math.cos(lat0) * dlon)
        ns.append(R * dlat)
    return np.array(ts), np.array(es), np.array(ns)

def yaw_from_vel(ts, xs, ys, t0, win=5.0, speed_gate=2.0):
    """Estimate heading at t0 from velocity over [t0, t0+win]."""
    mask = (ts >= t0) & (ts <= t0 + win)
    if mask.sum() < 2: return 0.0
    dx = np.diff(xs[mask]); dy = np.diff(ys[mask])
    dt = np.diff(ts[mask])
    speeds = np.sqrt(dx**2 + dy**2) / np.maximum(dt, 1e-6)
    fast = speeds > speed_gate
    if fast.sum() == 0: return math.atan2(np.mean(dy), np.mean(dx))
    return math.atan2(np.sum(dy[fast]), np.sum(dx[fast]))

def align_start_yaw(ts, xs, ys, t0, yaw_gps, yaw_vio):
    """Align VIO to GPS: rotate + translate to match start position and heading."""
    # Find start index
    idx = np.searchsorted(ts, t0)
    if idx >= len(ts): return xs, ys
    x0, y0 = xs[idx], ys[idx]
    dtheta = yaw_gps - yaw_vio
    c, s = math.cos(dtheta), math.sin(dtheta)
    # Centre on start, rotate, re-anchor at GPS start
    dx = xs - x0; dy = ys - y0
    xa = c*dx - s*dy; ya = s*dx + c*dy
    return xa, ya

# ── Plot ──────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(2, 2, figsize=(16, 13))
fig.suptitle("XY Trajectory — FEJ-OC-prechi2 vs global_oc_alpha1  (GPS = ground reference)",
             fontsize=13, fontweight="bold", y=0.99)

axes_flat = [axes[0,0], axes[0,1], axes[1,0], axes[1,1]]
for ax, (fname, cfg) in zip(axes_flat, FLIGHTS.items()):
    t0 = cfg["t0"]

    # Load GPS
    gps_t, gps_e, gps_n = load_gps_enu(cfg["gps"])

    # Filter GPS to flight window
    if len(gps_t):
        mask = gps_t >= t0
        gt, ge, gn = gps_t[mask], gps_e[mask], gps_n[mask]
        # Compute GPS speed and heading at t0
        spd_mask = (gt >= t0) & (gt <= t0 + 5.0)
        if spd_mask.sum() >= 2:
            de = np.diff(ge[spd_mask]); dn = np.diff(gn[spd_mask])
            dt_ = np.diff(gt[spd_mask])
            spds = np.sqrt(de**2 + dn**2) / np.maximum(dt_, 1e-6)
            fast = spds > 2.0
            yaw_gps_start = math.atan2(np.sum(de[fast]) if fast.sum() else de.sum(),
                                        np.sum(dn[fast]) if fast.sum() else dn.sum())
        else:
            yaw_gps_start = 0.0
        # Anchor GPS at origin for the start of the eval window
        idx0 = np.searchsorted(gt, t0)
        ge = ge - ge[idx0]
        gn = gn - gn[idx0]
        # Decimate GPS for plotting
        step = max(1, len(gt) // 800)
        ax.plot(ge[::step], gn[::step], color="k", lw=1.0, alpha=0.35,
                zorder=1, label="GPS (reference)")
    else:
        yaw_gps_start = 0.0

    # Plot VIO trajectories
    for label, traj_path, color, lw, ls in cfg["trajs"]:
        ts, xs, ys, _ = load_tum(traj_path)
        if len(ts) == 0: continue

        # Compute VIO yaw at t0 from velocity
        yaw_vio_start = yaw_from_vel(ts, xs, ys, t0, win=5.0)

        # Align
        xa, ya = align_start_yaw(ts, xs, ys, t0, yaw_gps_start, yaw_vio_start)

        # Decimate
        step = max(1, len(ts) // 1500)
        ax.plot(xa[::step], ya[::step], color=color, lw=lw, ls=ls,
                alpha=0.85, zorder=2+lw, label=label)

        # Mark start
        idx = np.searchsorted(ts, t0)
        if idx < len(ts):
            ax.plot(xa[idx], ya[idx], "o", color=color, ms=5, zorder=5)

    ax.set_title(FLIGHT_LABELS[fname], fontsize=10, pad=4)
    ax.set_xlabel("East (m)", fontsize=8)
    ax.set_ylabel("North (m)", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, lw=0.5)
    ax.legend(fontsize=7.5, loc="best", framealpha=0.85)
    ax.tick_params(labelsize=8)

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(OUT, dpi=140, bbox_inches="tight")
print(f"Saved: {OUT}")
