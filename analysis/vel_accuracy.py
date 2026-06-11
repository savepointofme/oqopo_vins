#!/usr/bin/env python3
"""
vel_accuracy.py — OC velocity accuracy analysis for fly1–fly4.

For each flight: load the best existing OC traj.txt, load GPS CSV, derive
velocities via Savitzky-Golay-smoothed finite differences, align frames via
start-window yaw rotation, compute metrics, save plots.
"""
import os
import math
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

# ─── Configuration ────────────────────────────────────────────────────────────
BASE_DESK = "/mnt/c/Users/baloney/Desktop"
REPO      = "/mnt/d/vscode_dir/open_vins"

FLIGHTS = {
    "fly1": dict(
        traj  = f"{BASE_DESK}/20260517_gsmq_d455_fly1/result/gpsz_on_yawcmp_fly1_oc_fej_prechi2/traj.txt",
        gps   = f"{REPO}/config/d455_fly1/fc_gps_cam_time.csv",
        t0    = 930.0,
        until = 1740.0,
        label = "Fly1 (~200m alt)",
    ),
    "fly2": dict(
        traj  = f"{BASE_DESK}/20260518_gsmq_d455_fly2/result/feature_gate_ablation_20260607/cond1e5_dist1000/traj.txt",
        gps   = f"{BASE_DESK}/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv",
        t0    = 700.0,
        until = 2700.0,
        label = "Fly2 (~500m alt)",
    ),
    "fly3": dict(
        traj  = f"{BASE_DESK}/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/start618_oc1_gpsalt_guard_offset438p0_viz/traj.txt",
        gps   = f"{BASE_DESK}/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv",
        t0    = 618.0,
        until = 1600.0,
        label = "Fly3 (~200m alt)",
    ),
    "fly4": dict(
        traj  = f"{BASE_DESK}/20260528_gsmq_d455_fly4/result/gpsz_on_yawcmp_fly4_oc_fej_prechi2/traj.txt",
        gps   = f"{BASE_DESK}/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv",
        t0    = 924.4,
        until = 2816.0,
        label = "Fly4 (~400m alt)",
    ),
}

OUT_DIR = f"{BASE_DESK}/vel_accuracy_analysis_20260608"
SG_WINDOW   = 31    # Savitzky-Golay window (samples) for VIO smoothing
SG_POLY     = 3     # SG polynomial order
GPS_SG_WIN  = 7     # smaller window for GPS (lower rate)
YAW_ALIGN_T = 60.0  # seconds of overlap used for yaw alignment

# ─── Coordinate utilities ─────────────────────────────────────────────────────
def wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Simple WGS84 → ENU (valid for small areas, <100 km)."""
    a  = 6378137.0
    b  = 6356752.3142
    e2 = 1.0 - (b/a)**2
    N  = a / math.sqrt(1 - e2 * math.sin(math.radians(lat0))**2)
    dx = math.radians(lat  - lat0) * (a * (1 - e2) / (1 - e2 * math.sin(math.radians(lat0))**2)**1.5)
    dy = math.radians(lon  - lon0) * N * math.cos(math.radians(lat0))
    dz = alt - alt0
    return dy, dx, dz   # East, North, Up

# ─── I/O ──────────────────────────────────────────────────────────────────────
def load_traj(path):
    """Load TUM-format traj.txt → (t, x, y, z) arrays."""
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            if len(parts) >= 4:
                rows.append([float(parts[0]), float(parts[1]),
                              float(parts[2]), float(parts[3])])
    arr = np.array(rows)
    return arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]  # t, x, y, z


def load_gps(path):
    """
    Load GPS CSV.  Accepts two formats:
      ts_ns,lat,lon,alt    (WGS84)
      ts_ns,x,y,z          (ENU already, detected by |lat|>90 check)
    Returns t_s, lat_or_x, lon_or_y, alt_or_z, is_wgs84.
    Removes duplicate / non-monotone timestamps.
    """
    rows = []
    with open(path) as f:
        f.readline()  # skip header
        for line in f:
            if not line.strip():
                continue
            parts = line.split(",")
            if len(parts) >= 4:
                try:
                    rows.append([float(p) for p in parts[:4]])
                except ValueError:
                    continue
    arr = np.array(rows)
    t_s = arr[:, 0] / 1e9
    c1, c2, c3 = arr[:, 1], arr[:, 2], arr[:, 3]
    # Remove duplicate / non-monotone timestamps
    keep = np.concatenate([[True], np.diff(t_s) > 1e-6])
    t_s, c1, c2, c3 = t_s[keep], c1[keep], c2[keep], c3[keep]
    # Detect WGS84: latitude in [-90, 90], longitude in [-180, 180]
    is_wgs84 = (np.abs(c1) <= 90).all() and (np.abs(c2) <= 180).all()
    return t_s, c1, c2, c3, is_wgs84


def gps_to_enu_array(t, c1, c2, c3, is_wgs84):
    """Convert GPS array to ENU numpy arrays (E, N, U)."""
    if not is_wgs84:
        return c1.copy(), c2.copy(), c3.copy()
    lat0, lon0, alt0 = c1[0], c2[0], c3[0]
    E = np.empty_like(c1)
    N = np.empty_like(c1)
    U = np.empty_like(c1)
    for i, (la, lo, al) in enumerate(zip(c1, c2, c3)):
        e, n, u = wgs84_to_enu(la, lo, al, lat0, lon0, alt0)
        E[i] = e; N[i] = n; U[i] = u
    return E, N, U

# ─── Velocity derivation ──────────────────────────────────────────────────────
def sg_velocity(t, x, y, z, sg_win, sg_poly):
    """
    Savitzky-Golay smooth positions, then central finite differences for velocity.
    Returns vx, vy, vz at the same time grid as input.
    """
    n = len(t)
    # SG requires odd window >= poly+2, and < n
    sg_win = min(sg_win, n if n % 2 == 1 else n - 1)
    sg_win = max(sg_poly + 2, sg_win)
    if sg_win % 2 == 0:
        sg_win -= 1
    sg_poly = min(sg_poly, sg_win - 1)

    xs = savgol_filter(x, sg_win, sg_poly)
    ys = savgol_filter(y, sg_win, sg_poly)
    zs = savgol_filter(z, sg_win, sg_poly)

    # Use explicit central differences with non-uniform spacing to avoid div-by-zero
    dt = np.diff(t)
    dt = np.where(dt < 1e-9, 1e-9, dt)   # guard zero intervals

    def deriv(pos):
        dp = np.diff(pos)
        raw = dp / dt
        # Central: average adjacent forward differences
        v = np.empty(n)
        v[0]    = raw[0]
        v[-1]   = raw[-1]
        v[1:-1] = 0.5 * (raw[:-1] + raw[1:])
        return v

    return deriv(xs), deriv(ys), deriv(zs)

# ─── Yaw alignment ───────────────────────────────────────────────────────────
def yaw_align_velocity(t_vio, vx_vio, vy_vio,
                       t_gps, vx_gps, vy_gps,
                       t_start, align_window=YAW_ALIGN_T):
    """
    Estimate yaw offset between VIO and GPS frames using the horizontal velocity
    direction in the first `align_window` seconds after t_start.
    Returns (yaw_deg, vx_aligned, vy_aligned).
    """
    t_end = t_start + align_window
    mask_v = (t_vio >= t_start) & (t_vio <= t_end)
    mask_g = (t_gps >= t_start) & (t_gps <= t_end)
    if mask_v.sum() < 5 or mask_g.sum() < 5:
        return 0.0, vx_vio, vy_vio

    # Mean heading in each frame
    spd_v = np.sqrt(vx_vio[mask_v]**2 + vy_vio[mask_v]**2)
    spd_g = np.sqrt(vx_gps[mask_g]**2 + vy_gps[mask_g]**2)
    # Weight by speed to avoid low-speed noise
    wv = spd_v + 1e-6; wg = spd_g + 1e-6
    hdg_v = np.average(np.arctan2(vy_vio[mask_v], vx_vio[mask_v]), weights=wv)
    hdg_g = np.average(np.arctan2(vy_gps[mask_g], vx_gps[mask_g]), weights=wg)

    dyaw = hdg_g - hdg_v          # rotation to apply to VIO
    c, s = math.cos(dyaw), math.sin(dyaw)
    vx_a = c * vx_vio - s * vy_vio
    vy_a = s * vx_vio + c * vy_vio
    return math.degrees(dyaw), vx_a, vy_a

# ─── Metrics ─────────────────────────────────────────────────────────────────
def metrics(err):
    """Compute summary statistics for an error array (NaN-safe)."""
    err = np.asarray(err, dtype=float)
    err = err[np.isfinite(err)]
    if len(err) == 0:
        nan = float("nan")
        return dict(rmse=nan, median=nan, p95=nan, bias=nan, std=nan)
    ae = np.abs(err)
    return dict(
        rmse   = float(np.sqrt(np.mean(err**2))),
        median = float(np.median(ae)),
        p95    = float(np.percentile(ae, 95)),
        bias   = float(np.mean(err)),
        std    = float(np.std(err)),
    )

# ─── Per-flight analysis ──────────────────────────────────────────────────────
def analyse_flight(name, cfg, out_dir):
    print(f"\n{'='*60}")
    print(f"  {name.upper()} — {cfg['label']}")
    print(f"{'='*60}")

    # Load trajectory
    t_v, x_v, y_v, z_v = load_traj(cfg["traj"])
    # Clip to eval window
    mask = (t_v >= cfg["t0"]) & (t_v <= cfg["until"])
    t_v, x_v, y_v, z_v = t_v[mask], x_v[mask], y_v[mask], z_v[mask]
    print(f"  VIO: {len(t_v)} samples  t=[{t_v[0]:.1f}, {t_v[-1]:.1f}]s")

    # Load GPS
    t_g, c1, c2, c3, is_wgs84 = load_gps(cfg["gps"])
    E_g, N_g, U_g = gps_to_enu_array(t_g, c1, c2, c3, is_wgs84)
    mask_g = (t_g >= cfg["t0"]) & (t_g <= cfg["until"])
    t_g, E_g, N_g, U_g = t_g[mask_g], E_g[mask_g], N_g[mask_g], U_g[mask_g]
    print(f"  GPS: {len(t_g)} samples  t=[{t_g[0]:.1f}, {t_g[-1]:.1f}]s  wgs84={is_wgs84}")

    if len(t_g) < 5:
        print("  ERROR: too few GPS samples in window — skipping")
        return None

    # Derive VIO velocities (high-rate)
    vx_v, vy_v, vz_v = sg_velocity(t_v, x_v, y_v, z_v, SG_WINDOW, SG_POLY)

    # Derive GPS velocities (low-rate, smaller SG window)
    win_g = min(GPS_SG_WIN, (len(t_g)//2)*2 - 1)
    win_g = max(3, win_g if win_g % 2 == 1 else win_g - 1)
    vx_g, vy_g, vz_g = sg_velocity(t_g, E_g, N_g, U_g, win_g, min(SG_POLY, win_g-1))

    # Yaw-align VIO horizontal velocity to GPS frame
    yaw_deg, vx_va, vy_va = yaw_align_velocity(
        t_v, vx_v, vy_v, t_g, vx_g, vy_g, cfg["t0"])
    print(f"  Yaw alignment: {yaw_deg:+.2f} deg")

    # Interpolate aligned VIO velocity onto GPS timestamps
    def interp_v(t_src, v_src, t_dst):
        f = interp1d(t_src, v_src, kind="linear", bounds_error=False,
                     fill_value=(v_src[0], v_src[-1]))
        return f(t_dst)

    vx_at_gps  = interp_v(t_v, vx_va,  t_g)
    vy_at_gps  = interp_v(t_v, vy_va,  t_g)
    vz_at_gps  = interp_v(t_v, vz_v,   t_g)
    spd_vio    = np.sqrt(vx_at_gps**2 + vy_at_gps**2 + vz_at_gps**2)
    spd_gps    = np.sqrt(vx_g**2 + vy_g**2 + vz_g**2)
    spd_h_vio  = np.sqrt(vx_at_gps**2 + vy_at_gps**2)
    spd_h_gps  = np.sqrt(vx_g**2 + vy_g**2)

    err_vx   = vx_at_gps - vx_g
    err_vy   = vy_at_gps - vy_g
    err_vz   = vz_at_gps - vz_g
    err_spd  = spd_vio - spd_gps
    err_spdh = spd_h_vio - spd_h_gps
    err_3d   = np.sqrt(err_vx**2 + err_vy**2 + err_vz**2)

    hdg_vio = np.degrees(np.arctan2(vy_at_gps, vx_at_gps))
    hdg_gps = np.degrees(np.arctan2(vy_g, vx_g))
    # Wrap heading error to [-180, 180]
    err_hdg = ((hdg_vio - hdg_gps + 180) % 360) - 180
    # Only count heading when GPS speed > 1 m/s (avoid noise at low speed)
    hdg_mask = spd_gps > 1.0
    err_hdg_valid = err_hdg[hdg_mask] if hdg_mask.sum() > 10 else err_hdg

    m_vx    = metrics(err_vx)
    m_vy    = metrics(err_vy)
    m_vz    = metrics(err_vz)
    m_spd   = metrics(err_spd)
    m_spdh  = metrics(err_spdh)
    m_3d    = metrics(err_3d)
    m_hdg   = metrics(err_hdg_valid)

    print(f"\n  {'Metric':<30} {'vx':>9} {'vy':>9} {'vz':>9} {'|v|3D':>9} {'|v|H':>9}")
    print(f"  {'-'*75}")
    for k in ("rmse", "median", "p95", "bias", "std"):
        print(f"  {k:<30} {m_vx[k]:>9.3f} {m_vy[k]:>9.3f} {m_vz[k]:>9.3f} "
              f"{m_spd[k]:>9.3f} {m_spdh[k]:>9.3f}  m/s")
    print(f"\n  3D velocity RMSE: {m_3d['rmse']:.3f} m/s  (median: {m_3d['median']:.3f})")
    print(f"  Heading error (spd>1m/s): RMSE={m_hdg['rmse']:.2f}°  bias={m_hdg['bias']:+.2f}°  "
          f"P95={m_hdg['p95']:.2f}°")

    result = dict(
        name=name, label=cfg["label"],
        yaw_align_deg=yaw_deg,
        vx=m_vx, vy=m_vy, vz=m_vz,
        spd3d=m_spd, spdh=m_spdh, v3d_err=m_3d, hdg=m_hdg,
        t_g=t_g, spd_gps=spd_gps, spd_vio=spd_vio,
        err_spd=err_spd, err_vx=err_vx, err_vy=err_vy, err_vz=err_vz, err_3d=err_3d,
        vx_g=vx_g, vy_g=vy_g, vx_vio=vx_at_gps, vy_vio=vy_at_gps,
    )

    # ─── Plots ────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(f"{cfg['label']} — OC velocity vs GPS truth", fontsize=13)

    ax = axes[0]
    ax.plot(t_g, spd_gps,  label="GPS speed", color="tab:blue",  lw=1.5)
    ax.plot(t_g, spd_vio,  label="OC speed",  color="tab:orange", lw=1.0, alpha=0.8)
    ax.set_ylabel("|v| m/s")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t_g, vx_g,       label="GPS vE",  color="tab:blue",   lw=1.5)
    ax.plot(t_g, vx_at_gps,  label="OC vE",   color="tab:orange", lw=1.0, alpha=0.8)
    ax.plot(t_g, vy_g,       label="GPS vN",  color="tab:green",  lw=1.5, ls="--")
    ax.plot(t_g, vy_at_gps,  label="OC vN",   color="tab:red",    lw=1.0, alpha=0.8, ls="--")
    ax.set_ylabel("vE, vN  m/s")
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(t_g, err_spd, label="|v| error",  color="tab:purple", lw=1.2)
    ax.plot(t_g, err_3d,  label="3D |Δv|",    color="tab:red",    lw=0.8, alpha=0.7)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("error  m/s"); ax.set_xlabel("t (s)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title(f"RMSE(spd)={m_spd['rmse']:.3f}  RMSE(3D)={m_3d['rmse']:.3f}  "
                 f"bias={m_spd['bias']:+.3f}  P95={m_spd['p95']:.3f} m/s", fontsize=9)

    plt.tight_layout()
    fig.savefig(f"{out_dir}/{name}_vel_vs_time.png", dpi=150)
    plt.close(fig)

    # Scatter: OC speed vs GPS speed
    fig2, ax2 = plt.subplots(figsize=(6, 6))
    valid = np.isfinite(spd_gps) & np.isfinite(spd_vio)
    ax2.scatter(spd_gps[valid], spd_vio[valid], s=2, alpha=0.3, color="tab:blue")
    lim_val = max(np.nanmax(spd_gps), np.nanmax(spd_vio))
    lim = lim_val * 1.05 if np.isfinite(lim_val) else 30.0
    ax2.plot([0, lim], [0, lim], "k--", lw=1, label="ideal")
    ax2.set_xlabel("GPS speed (m/s)"); ax2.set_ylabel("OC speed (m/s)")
    ax2.set_title(f"{cfg['label']} — speed scatter")
    ax2.set_xlim(0, lim); ax2.set_ylim(0, lim)
    ax2.legend(); ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(f"{out_dir}/{name}_speed_scatter.png", dpi=150)
    plt.close(fig2)

    print(f"  Plots saved: {out_dir}/{name}_*.png")
    return result

# ─── Cross-flight summary ─────────────────────────────────────────────────────
def print_summary(results):
    print(f"\n{'='*80}")
    print("  CROSS-FLIGHT SUMMARY")
    print(f"{'='*80}")
    cols = ["name", "3D RMSE", "Spd RMSE", "SpdH RMSE", "Hdg RMSE", "vz RMSE", "Spd bias", "Yaw align"]
    hdr  = f"{'Flight':<8} {'3D RMSE':>9} {'Spd RMSE':>9} {'SpdH':>9} {'Hdg RMSE':>10} {'vz RMSE':>9} {'Spd bias':>9} {'Yaw°':>8}"
    print(f"\n  {hdr}")
    print(f"  {'-'*75}")
    for r in results:
        print(f"  {r['name']:<8} "
              f"{r['v3d_err']['rmse']:>9.3f} "
              f"{r['spd3d']['rmse']:>9.3f} "
              f"{r['spdh']['rmse']:>9.3f} "
              f"{r['hdg']['rmse']:>10.2f}° "
              f"{r['vz']['rmse']:>9.3f} "
              f"{r['spd3d']['bias']:>+9.3f} "
              f"{r['yaw_align_deg']:>+8.2f}°")

    rmses = [r["spd3d"]["rmse"] for r in results]
    best  = results[int(np.argmin(rmses))]
    worst = results[int(np.argmax(rmses))]
    print(f"\n  Best  velocity accuracy: {best['name']} ({best['label']})  RMSE={best['spd3d']['rmse']:.3f} m/s")
    print(f"  Worst velocity accuracy: {worst['name']} ({worst['label']})  RMSE={worst['spd3d']['rmse']:.3f} m/s")

    print(f"\n  Interpretation guide:")
    for r in results:
        spd_rmse = r["spd3d"]["rmse"]
        bias     = r["spd3d"]["bias"]
        hdg_rmse = r["hdg"]["rmse"]
        usable   = "YES" if spd_rmse < 1.5 else "MARGINAL" if spd_rmse < 3.0 else "NO"
        print(f"  {r['name']}: spd_rmse={spd_rmse:.2f} m/s  bias={bias:+.2f}  "
              f"hdg_rmse={hdg_rmse:.1f}°  usable_as_vel_source={usable}")


def save_summary_csv(results, out_dir):
    import csv
    path = f"{out_dir}/vel_accuracy_summary.csv"
    fields = ["flight","label","yaw_align_deg",
              "spd3d_rmse","spd3d_median","spd3d_p95","spd3d_bias",
              "spdh_rmse","spdh_median","spdh_p95",
              "v3d_rmse","vx_rmse","vy_rmse","vz_rmse",
              "hdg_rmse","hdg_p95","hdg_bias"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in results:
            w.writerow({
                "flight": r["name"], "label": r["label"],
                "yaw_align_deg": f"{r['yaw_align_deg']:.2f}",
                "spd3d_rmse":   f"{r['spd3d']['rmse']:.4f}",
                "spd3d_median": f"{r['spd3d']['median']:.4f}",
                "spd3d_p95":    f"{r['spd3d']['p95']:.4f}",
                "spd3d_bias":   f"{r['spd3d']['bias']:.4f}",
                "spdh_rmse":    f"{r['spdh']['rmse']:.4f}",
                "spdh_median":  f"{r['spdh']['median']:.4f}",
                "spdh_p95":     f"{r['spdh']['p95']:.4f}",
                "v3d_rmse":     f"{r['v3d_err']['rmse']:.4f}",
                "vx_rmse":      f"{r['vx']['rmse']:.4f}",
                "vy_rmse":      f"{r['vy']['rmse']:.4f}",
                "vz_rmse":      f"{r['vz']['rmse']:.4f}",
                "hdg_rmse":     f"{r['hdg']['rmse']:.4f}",
                "hdg_p95":      f"{r['hdg']['p95']:.4f}",
                "hdg_bias":     f"{r['hdg']['bias']:.4f}",
            })
    print(f"\n  Summary CSV: {path}")


def save_comparison_plot(results, out_dir):
    n = len(results)
    fig, axes = plt.subplots(2, n, figsize=(5*n, 8))
    fig.suptitle("OC velocity error — fly1 to fly4", fontsize=13)

    for i, r in enumerate(results):
        # Speed error over time
        ax = axes[0, i]
        ax.plot(r["t_g"], r["err_spd"],  label="|v| err", lw=0.8, color="tab:blue")
        ax.plot(r["t_g"], r["err_3d"],   label="3D err",  lw=0.8, color="tab:orange", alpha=0.7)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_title(f"{r['name']}\nRMSE={r['spd3d']['rmse']:.2f} m/s", fontsize=9)
        ax.set_ylabel("error m/s"); ax.set_xlabel("t (s)")
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
        ax.set_ylim(-15, 15)

        # Speed scatter
        ax = axes[1, i]
        sg, sv = r["spd_gps"], r["spd_vio"]
        valid = np.isfinite(sg) & np.isfinite(sv)
        ax.scatter(sg[valid], sv[valid], s=1, alpha=0.2, color="tab:blue")
        lim_val = max(np.nanmax(sg), np.nanmax(sv))
        lim = lim_val * 1.05 if np.isfinite(lim_val) else 30.0
        ax.plot([0, lim], [0, lim], "k--", lw=1)
        ax.set_xlabel("GPS |v| m/s"); ax.set_ylabel("OC |v| m/s")
        ax.set_xlim(0, lim); ax.set_ylim(0, lim)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(f"{out_dir}/all_flights_vel_comparison.png", dpi=150)
    plt.close(fig)
    print(f"  Comparison plot: {out_dir}/all_flights_vel_comparison.png")


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Output directory: {OUT_DIR}")

    results = []
    for name, cfg in FLIGHTS.items():
        if not os.path.exists(cfg["traj"]):
            print(f"\n[SKIP] {name}: traj not found: {cfg['traj']}")
            continue
        if not os.path.exists(cfg["gps"]):
            print(f"\n[SKIP] {name}: GPS not found: {cfg['gps']}")
            continue
        r = analyse_flight(name, cfg, OUT_DIR)
        if r is not None:
            results.append(r)

    if results:
        print_summary(results)
        save_summary_csv(results, OUT_DIR)
        save_comparison_plot(results, OUT_DIR)

    print(f"\nDone. All outputs in: {OUT_DIR}")


if __name__ == "__main__":
    main()
