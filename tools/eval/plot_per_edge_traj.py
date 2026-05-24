#!/usr/bin/env python3
"""Plot XY trajectories with edge segments annotated — iwt=2.0 vs iwt=5.0."""

import math, os, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import matplotlib.patheffects as pe

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, 'comparison_plots', 'per_edge_scale')
os.makedirs(OUT_DIR, exist_ok=True)

FLIGHTS = [
    dict(flight=1,
         b0_iwt2='20260509_fly1/result/baselines_v1/B0_no_gps.txt',
         b0_iwt5='20260509_fly1/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv'),
    dict(flight=2,
         b0_iwt2='20260509_fly2/result/baselines_v1/B0_no_gps.txt',
         b0_iwt5='20260509_fly2/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv'),
    dict(flight=3,
         b0_iwt2='20260509_fly3/result/baselines_v1/B0_no_gps.txt',
         b0_iwt5='20260509_fly3/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
    dict(flight=4,
         b0_iwt2='20260509_fly4/result/stage_a_v2/R0_nogps.txt',
         b0_iwt5='20260509_fly4/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
]

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

def load_b0(p):
    rows, lt = [], -math.inf
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            try:
                t, x, y, z = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
            except ValueError: continue
            if t <= lt: continue
            rows.append((t, x, y, z)); lt = t
    return np.asarray(rows)

def load_gps(p):
    rows = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try: t, lat, lon, alt = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
            except (ValueError, IndexError): continue
            rows.append((t, lat, lon, alt))
    a = np.asarray(rows)
    if a.size and a[0, 0] > 1e11: a[:, 0] *= 1e-9
    return a[np.argsort(a[:, 0])]

def wgs84_to_ecef(lat, lon, alt):
    s, c = np.sin(np.deg2rad(lat)), np.cos(np.deg2rad(lat))
    so, co = np.sin(np.deg2rad(lon)), np.cos(np.deg2rad(lon))
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * s * s)
    return np.column_stack([(N + alt) * c * co, (N + alt) * c * so,
                            (N * (1 - WGS84_E2) + alt) * s])

def ecef_to_enu_R(lat0, lon0):
    sl, cl = np.sin(np.deg2rad(lat0)), np.cos(np.deg2rad(lat0))
    so, co = np.sin(np.deg2rad(lon0)), np.cos(np.deg2rad(lon0))
    return np.array([[-so, co, 0], [-sl*co, -sl*so, cl], [cl*co, cl*so, sl]])

def gps_to_enu(g):
    lat0, lon0, alt0 = g[0, 1], g[0, 2], g[0, 3]
    e0 = wgs84_to_ecef(lat0, lon0, alt0).reshape(1, 3)
    R = ecef_to_enu_R(lat0, lon0)
    e = wgs84_to_ecef(g[:, 1], g[:, 2], g[:, 3])
    enu = (e - e0) @ R.T
    return enu[:, :2], enu[:, 2], g[:, 0]

def segment_edges(gps_t, gps_xy, min_edge_len=30.0):
    if len(gps_xy) < 3: return []
    dxy = np.diff(gps_xy, axis=0)
    headings = np.arctan2(dxy[:, 1], dxy[:, 0])
    headings = np.unwrap(headings)
    win = min(5, len(headings)//2)
    if win >= 2:
        hs = np.convolve(headings, np.ones(win)/win, mode='same')
    else:
        hs = headings.copy()
    turn_window = 10
    cum_turn = np.zeros(len(gps_xy))
    for i in range(1, len(cum_turn)):
        j = max(0, i - turn_window)
        cum_turn[i] = abs(hs[i-1] - hs[j]) if i > 0 else 0
    turn_rad = np.deg2rad(30.0)
    splits = [0]
    i = turn_window
    while i < len(cum_turn) - turn_window:
        for j in range(i, len(cum_turn)):
            if cum_turn[j] > turn_rad:
                if j - splits[-1] >= 3: splits.append(j)
                i = j + turn_window
                break
        else: i += 1
    splits.append(len(gps_xy)-1)
    edges = []
    for k in range(len(splits)-1):
        i0, i1 = splits[k], splits[k+1]
        if i1 - i0 < 3: continue
        t0, t1 = gps_t[i0], gps_t[i1]
        path = np.sum(np.linalg.norm(np.diff(gps_xy[i0:i1+1], axis=0), axis=1))
        if path >= min_edge_len:
            edges.append((t0, t1, len(edges)))
    return edges

def edge_scale_ratio(b0_t, b0_xy, gps_t, gps_xy, t0, t1):
    mb = (b0_t >= t0) & (b0_t <= t1)
    mg = (gps_t >= t0) & (gps_t <= t1)
    if mb.sum() < 3 or mg.sum() < 3: return None, None, None
    b_len = float(np.sum(np.linalg.norm(np.diff(b0_xy[mb], axis=0), axis=1)))
    g_len = float(np.sum(np.linalg.norm(np.diff(gps_xy[mg], axis=0), axis=1)))
    if g_len < 1.0: return None, None, None
    return b_len / g_len, b_len, g_len

def detect_descent_start(gps_alt, gps_t):
    dz = np.diff(gps_alt)
    win = min(20, len(dz)//3)
    if win < 2: return None
    dz_sm = np.convolve(dz, np.ones(win)/win, mode='same')
    for i in range(len(dz_sm)-win, win, -1):
        if np.all(dz_sm[i-win:i] < -3.0/win): return gps_t[max(0,i-win)]
    return None

def detect_takeoff(gps_t, gps_xy, min_dist=3.0):
    if len(gps_xy) < 5: return None
    d = np.sqrt(np.sum((gps_xy - gps_xy[0])**2, axis=1))
    cr = np.where(d >= min_dist)[0]
    return float(gps_t[int(cr[0])]) if cr.size else None

# ---- plot ----

def umeyama_align(vio_t, vio_xy, gps_t, gps_xy, t_lo, t_hi):
    """Umeyama SE(2) alignment: rotation + translation (no scale).

    Aligns VIO XY to GPS XY using data in [t_lo, t_hi].
    Returns aligned VIO xy for the ENTIRE trajectory.
    """
    mv = (vio_t >= t_lo) & (vio_t <= t_hi)
    mg = (gps_t >= t_lo) & (gps_t <= t_hi)
    if mv.sum() < 5 or mg.sum() < 5:
        return vio_xy

    # Interpolate GPS to VIO timestamps in alignment window
    gps_at_vio = np.column_stack([
        np.interp(vio_t[mv], gps_t, gps_xy[:, 0]),
        np.interp(vio_t[mv], gps_t, gps_xy[:, 1])
    ])
    vio_align = vio_xy[mv]

    # Umeyama: find R, t that minimizes ||R*vio + t - gps||^2
    mu_v = np.mean(vio_align, axis=0)
    mu_g = np.mean(gps_at_vio, axis=0)
    v_centered = vio_align - mu_v
    g_centered = gps_at_vio - mu_g
    H = v_centered.T @ g_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # Ensure proper rotation (det = +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Apply rotation + translation to entire VIO trajectory
    aligned = (vio_xy - mu_v) @ R.T + mu_g
    return aligned

# Load all data first
all_data = []
for cfg in FLIGHTS:
    fly = cfg['flight']
    root = ROOT
    b0_2 = load_b0(os.path.join(root, cfg['b0_iwt2']))
    b0_5 = load_b0(os.path.join(root, cfg['b0_iwt5']))
    gps = load_gps(os.path.join(root, cfg['gps']))
    gps_xy, gps_alt, gps_t = gps_to_enu(gps)
    all_data.append((fly, b0_2, b0_5, gps_xy, gps_alt, gps_t))

# ===== FIGURE 1: 4-panel, per-flight trajectory + edge annotations =====
fig, axes = plt.subplots(2, 2, figsize=(22, 20))
axes = axes.flatten()

EDGE_COLORS = plt.cm.tab10(np.linspace(0, 1, 10))

for idx, (fly, b0_2, b0_5, gps_xy, gps_alt, gps_t) in enumerate(all_data):
    ax = axes[idx]

    t_takeoff = detect_takeoff(gps_t, gps_xy)
    t_descent = detect_descent_start(gps_alt, gps_t)
    if t_takeoff is None:
        ax.set_title(f'fly{fly}: no takeoff detected')
        continue

    t_start = t_takeoff
    t_end = t_descent if t_descent is not None else gps_t[-1]

    # Crop GPS
    mg = (gps_t >= t_start) & (gps_t <= t_end)
    gt = gps_t[mg]; gxy = gps_xy[mg]

    edges = segment_edges(gt, gxy, min_edge_len=30.0)

    # Align VIO to GPS using the first edge window
    if edges:
        align_lo, align_hi = edges[0][0], edges[0][1]
    else:
        align_lo, align_hi = t_start, t_start + 30

    b2_aligned = umeyama_align(b0_2[:, 0], b0_2[:, 1:3], gps_t, gps_xy, align_lo, align_hi)
    b5_aligned = umeyama_align(b0_5[:, 0], b0_5[:, 1:3], gps_t, gps_xy, align_lo, align_hi)

    # Plot trajectories (aligned)
    ax.plot(b2_aligned[:, 0], b2_aligned[:, 1], color='#3498db', alpha=0.6, linewidth=1.0, label='B0 iwt=2.0')
    ax.plot(b5_aligned[:, 0], b5_aligned[:, 1], color='#e74c3c', alpha=0.6, linewidth=1.0, label='B0 iwt=5.0')
    # GPS
    ax.plot(gps_xy[:, 0], gps_xy[:, 1], 'gray', alpha=0.5, linewidth=1.5, label='GPS ground track')

    # Mark takeoff and descent
    if t_takeoff:
        i_tk = np.argmin(np.abs(gps_t - t_takeoff))
        ax.plot(gps_xy[i_tk, 0], gps_xy[i_tk, 1], 'go', markersize=10, zorder=5,
                label=f'takeoff')
    if t_descent:
        i_ds = np.argmin(np.abs(gps_t - t_descent))
        ax.plot(gps_xy[i_ds, 0], gps_xy[i_ds, 1], 'rs', markersize=10, zorder=5,
                label=f'descent start')

    # Draw edges on GPS track and annotate scale
    for eid, (t0, t1, _) in enumerate(edges):
        color = EDGE_COLORS[eid % len(EDGE_COLORS)]
        mg_e = (gps_t >= t0) & (gps_t <= t1)
        ax.plot(gps_xy[mg_e, 0], gps_xy[mg_e, 1], color=color, linewidth=3.5, alpha=0.7)

        # Scale ratios
        r2, b2_len, g2_len = edge_scale_ratio(b0_2[:, 0], b0_2[:, 1:3], gps_t, gps_xy, t0, t1)
        r5, b5_len, g5_len = edge_scale_ratio(b0_5[:, 0], b0_5[:, 1:3], gps_t, gps_xy, t0, t1)

        # Annotate at midpoint of edge
        mid = np.argmin(np.abs(gps_t - (t0 + t1) / 2))
        mx, my = gps_xy[mid, 0], gps_xy[mid, 1]

        if r2 is not None and r5 is not None:
            d2 = abs(r2 - 1); d5 = abs(r5 - 1)
            if d5 < d2:
                tag = f'E{eid}: {r2:.3f}→{r5:.3f} ✓'
                tcolor = 'darkgreen'
            else:
                tag = f'E{eid}: {r2:.3f}→{r5:.3f} ✗'
                tcolor = 'darkred'
        elif r2 is not None:
            tag = f'E{eid}: {r2:.3f}→N/A'
            tcolor = 'gray'
        else:
            tag = f'E{eid}'
            tcolor = 'gray'

        ax.annotate(tag, (mx, my), fontsize=6.5, color=tcolor, fontweight='bold',
                    ha='center', va='bottom',
                    path_effects=[pe.withStroke(linewidth=2, foreground='white')])

    # Start/end markers
    ax.plot(gps_xy[0, 0], gps_xy[0, 1], 'ko', markersize=6, label='start')
    ax.plot(gps_xy[-1, 0], gps_xy[-1, 1], 'kx', markersize=8, label='end')

    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title(f'fly{fly} — XY trajectory with edge segments\n'
                 f'iwt=2.0 (blue) vs iwt=5.0 (red) | colored segments = edges | ✓=iwt5 better, ✗=iwt5 worse',
                 fontsize=10)
    ax.legend(loc='upper left', fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

# Hide unused
for idx in range(len(all_data), len(axes)):
    axes[idx].set_visible(False)

fig.suptitle('Per-Edge Scale: Trajectory Overlay (iwt=2.0 vs iwt=5.0)\n'
             'Edge annotations show scale ratio: iwt2.0→iwt5.0   ✓=better   ✗=worse',
             fontsize=14, fontweight='bold', y=0.99)
plt.tight_layout(rect=[0, 0, 1, 0.96])
p1 = os.path.join(OUT_DIR, 'trajectory_with_edges.png')
fig.savefig(p1, dpi=150, bbox_inches='tight')
plt.close(fig)
print(f'[saved] {p1}')

# ===== FIGURE 2: Side-by-side VIO overlays with edge bands =====
fig2, axes2 = plt.subplots(4, 1, figsize=(22, 28))

for idx, (fly, b0_2, b0_5, gps_xy, gps_alt, gps_t) in enumerate(all_data):
    ax = axes2[idx]

    t_takeoff = detect_takeoff(gps_t, gps_xy)
    t_descent = detect_descent_start(gps_alt, gps_t)
    if t_takeoff is None:
        ax.set_title(f'fly{fly}: no takeoff')
        continue
    t_start = t_takeoff
    t_end = t_descent if t_descent is not None else gps_t[-1]

    mg = (gps_t >= t_start) & (gps_t <= t_end)
    gt = gps_t[mg]; gxy = gps_xy[mg]
    edges = segment_edges(gt, gxy, min_edge_len=30.0)

    # Align
    if edges: align_lo, align_hi = edges[0][0], edges[0][1]
    else: align_lo, align_hi = t_start, t_start + 30
    b2_aligned = umeyama_align(b0_2[:, 0], b0_2[:, 1:3], gps_t, gps_xy, align_lo, align_hi)
    b5_aligned = umeyama_align(b0_5[:, 0], b0_5[:, 1:3], gps_t, gps_xy, align_lo, align_hi)

    # Compute per-frame scale ratio (30s sliding window)
    WIN = 30.0; STEP = 5.0
    ratios2, ratios5, centers = [], [], []
    t = 0.0
    t0 = t_start
    dur = t_end - t_start
    while t + WIN <= dur:
        lo, hi = t0 + t, t0 + t + WIN
        # iwt2
        mb2 = (b0_2[:, 0] >= lo) & (b0_2[:, 0] <= hi)
        mg2 = (gps_t >= lo) & (gps_t <= hi)
        if mb2.sum() >= 5 and mg2.sum() >= 5:
            b2l = float(np.sum(np.linalg.norm(np.diff(b2_aligned[mb2], axis=0), axis=1)))
            g2l = float(np.sum(np.linalg.norm(np.diff(gps_xy[mg2], axis=0), axis=1)))
        else: b2l = g2l = 0
        # iwt5
        mb5 = (b0_5[:, 0] >= lo) & (b0_5[:, 0] <= hi)
        mg5 = (gps_t >= lo) & (gps_t <= hi)
        if mb5.sum() >= 5 and mg5.sum() >= 5:
            b5l = float(np.sum(np.linalg.norm(np.diff(b5_aligned[mb5], axis=0), axis=1)))
            g5l = float(np.sum(np.linalg.norm(np.diff(gps_xy[mg5], axis=0), axis=1)))
        else: b5l = g5l = 0

        r2 = b2l / g2l if g2l > 5 else float('nan')
        r5 = b5l / g5l if g5l > 5 else float('nan')
        ratios2.append(r2); ratios5.append(r5)
        centers.append(t + WIN/2)
        t += STEP

    centers = np.array(centers)
    ratios2 = np.array(ratios2)
    ratios5 = np.array(ratios5)

    valid = ~np.isnan(ratios2) & ~np.isnan(ratios5)
    ax.plot(centers[valid], ratios2[valid], 'b-o', lw=1.0, ms=3, alpha=0.7, label='iwt=2.0')
    ax.plot(centers[valid], ratios5[valid], 'r-s', lw=1.0, ms=3, alpha=0.7, label='iwt=5.0')
    ax.axhline(1.0, color='k', lw=0.6, alpha=0.5)
    ax.axhspan(0.85, 1.15, alpha=0.08, color='green')
    ax.axhspan(0.70, 0.85, alpha=0.08, color='orange')
    ax.axhspan(1.15, 1.30, alpha=0.08, color='orange')

    # Edge bands
    for eid, (e_t0, e_t1, _) in enumerate(edges):
        e0_rel = e_t0 - t0; e1_rel = e_t1 - t0
        color = EDGE_COLORS[eid % len(EDGE_COLORS)]
        ax.axvspan(e0_rel, e1_rel, alpha=0.12, color=color)
        r2_e, _, _ = edge_scale_ratio(b0_2[:, 0], b0_2[:, 1:3], gps_t, gps_xy, e_t0, e_t1)
        r5_e, _, _ = edge_scale_ratio(b0_5[:, 0], b0_5[:, 1:3], gps_t, gps_xy, e_t0, e_t1)
        if r2_e is not None and r5_e is not None:
            d2, d5 = abs(r2_e-1), abs(r5_e-1)
            better = d5 < d2
            ax.text((e0_rel+e1_rel)/2, 0.05, f'E{eid}\n{r2_e:.3f}→{r5_e:.3f}',
                    ha='center', fontsize=6.5, color='darkgreen' if better else 'darkred',
                    fontweight='bold')

    ax.set_xlabel('time rel to takeoff (s)')
    ax.set_ylabel('scale ratio (VIO/GPS)')
    ax.set_title(f'fly{fly} — sliding-window scale ratio (30s win, 5s step) | '
                 f'green=±15% band | colored=edges')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0.5, 1.5)

fig2.suptitle('Per-Edge Scale: Sliding-Window Ratio with Edge Bands\n'
              'Blue=iwt2.0  Red=iwt5.0  Edge annotations: iwt2.0→iwt5.0 scale',
              fontsize=14, fontweight='bold')
plt.tight_layout(rect=[0, 0, 1, 0.97])
p2 = os.path.join(OUT_DIR, 'sliding_window_with_edges.png')
fig2.savefig(p2, dpi=150, bbox_inches='tight')
plt.close(fig2)
print(f'[saved] {p2}')

# ===== FIGURE 3: Z comparison (height vs time) =====
fig3, axes3 = plt.subplots(4, 1, figsize=(22, 24))

for idx, (fly, b0_2, b0_5, gps_xy, gps_alt, gps_t) in enumerate(all_data):
    ax = axes3[idx]
    ax.plot(gps_t, gps_alt, 'gray', alpha=0.5, linewidth=1.5, label='GPS altitude')

    # Interpolate VIO z to GPS timestamps
    z2_interp = np.interp(gps_t, b0_2[:, 0], b0_2[:, 3], left=np.nan, right=np.nan)
    z5_interp = np.interp(gps_t, b0_5[:, 0], b0_5[:, 3], left=np.nan, right=np.nan)

    # Align z offset using first 30s
    valid2 = ~np.isnan(z2_interp)
    valid5 = ~np.isnan(z5_interp)
    if valid2.sum() > 10:
        # Use first 30s to align
        t_align = gps_t[0] + 30
        m_align = gps_t < t_align
        if m_align.sum() > 5:
            dz2 = np.nanmean(gps_alt[m_align] - z2_interp[m_align])
            dz5 = np.nanmean(gps_alt[m_align] - z5_interp[m_align])
            z2_aligned = z2_interp + dz2
            z5_aligned = z5_interp + dz5
        else:
            z2_aligned = z2_interp; z5_aligned = z5_interp
    else:
        z2_aligned = z2_interp; z5_aligned = z5_interp

    ax.plot(gps_t, z2_aligned, 'blue', alpha=0.7, linewidth=1.0, label='B0 iwt=2.0 z')
    ax.plot(gps_t, z5_aligned, 'red', alpha=0.7, linewidth=1.0, label='B0 iwt=5.0 z')

    t_takeoff = detect_takeoff(gps_t, gps_xy)
    t_descent = detect_descent_start(gps_alt, gps_t)
    if t_takeoff:
        ax.axvline(t_takeoff, color='green', lw=1.5, ls='--', alpha=0.7, label='takeoff')
    if t_descent:
        ax.axvline(t_descent, color='purple', lw=1.5, ls='--', alpha=0.7, label='descent')

    ax.set_xlabel('time (s)')
    ax.set_ylabel('altitude (m)')
    ax.set_title(f'fly{fly} — height vs time (VIO z aligned to GPS altitude over first 30s)')
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)

fig3.suptitle('Height Comparison: iwt=2.0 vs iwt=5.0 (vs GPS altitude)',
              fontsize=14, fontweight='bold')
plt.tight_layout(rect=[0, 0, 1, 0.97])
p3 = os.path.join(OUT_DIR, 'height_comparison.png')
fig3.savefig(p3, dpi=150, bbox_inches='tight')
plt.close(fig3)
print(f'[saved] {p3}')

print("\nDone. All plots in", OUT_DIR)
