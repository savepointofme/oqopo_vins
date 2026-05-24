#!/usr/bin/env python3
"""
Per-edge scale comparison: iwt=2.0 vs iwt=5.0 for all 4 flights.

Segments the GPS ground track into "edges" (straight-ish horizontal segments
between turns), then computes VIO_path / GPS_path for each edge.

Final descent is excluded (uninteresting).
"""

from __future__ import annotations
import math, os, sys, json
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:
    _HAVE_MPL = False

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, 'comparison_plots', 'per_edge_scale')
os.makedirs(OUT_DIR, exist_ok=True)

# ---- config ----

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

# ---- data loading ----

def load_b0(p):
    rows = []
    t_last = -math.inf
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            ps = ln.split()
            try:
                t, x, y, z = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
            except ValueError:
                continue
            if t <= t_last:
                continue
            rows.append((t, x, y, z))
            t_last = t
    return np.asarray(rows)

def load_gps(p):
    rows = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            ps = ln.replace(',', ' ').split()
            try:
                t, lat, lon, alt = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
            except (ValueError, IndexError):
                continue
            rows.append((t, lat, lon, alt))
    a = np.asarray(rows)
    if a.size and a[0, 0] > 1e11:
        a[:, 0] *= 1e-9
    return a[np.argsort(a[:, 0])]

# ---- geodetic ----

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

def wgs84_to_ecef(lat, lon, alt):
    s, c = np.sin(np.deg2rad(lat)), np.cos(np.deg2rad(lat))
    so, co = np.sin(np.deg2rad(lon)), np.cos(np.deg2rad(lon))
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * s * s)
    return np.column_stack([(N + alt) * c * co,
                            (N + alt) * c * so,
                            (N * (1 - WGS84_E2) + alt) * s])

def ecef_to_enu_R(lat0, lon0):
    sl, cl = np.sin(np.deg2rad(lat0)), np.cos(np.deg2rad(lat0))
    so, co = np.sin(np.deg2rad(lon0)), np.cos(np.deg2rad(lon0))
    return np.array([[-so, co, 0],
                     [-sl*co, -sl*so, cl],
                     [cl*co, cl*so, sl]])

def gps_to_enu(g):
    lat0, lon0, alt0 = g[0, 1], g[0, 2], g[0, 3]
    e0 = wgs84_to_ecef(lat0, lon0, alt0).reshape(1, 3)
    R = ecef_to_enu_R(lat0, lon0)
    e = wgs84_to_ecef(g[:, 1], g[:, 2], g[:, 3])
    enu = (e - e0) @ R.T
    return enu[:, :2], enu[:, 2], g[:, 0]

# ---- edge segmentation ----

def segment_edges(gps_t, gps_xy, min_edge_len=30.0, turn_angle_deg=30.0):
    """Segment GPS ground track into edges by detecting turns.

    For each consecutive pair of GPS points, compute the heading.
    When the cumulative heading change over a sliding window exceeds
    turn_angle_deg, insert a split.

    Returns list of (t_start, t_end, edge_idx)
    """
    if len(gps_xy) < 3:
        return []

    # Compute heading at each GPS point
    dxy = np.diff(gps_xy, axis=0)
    headings = np.arctan2(dxy[:, 1], dxy[:, 0])  # -pi to pi
    headings = np.unwrap(headings)  # unwrap to avoid 2pi jumps

    # Smooth heading with a short window
    win = min(5, len(headings) // 2)
    if win >= 2:
        kernel = np.ones(win) / win
        headings_sm = np.convolve(headings, kernel, mode='same')
    else:
        headings_sm = headings.copy()

    # Cumulative heading change
    turn_window = 10  # points
    cum_turn = np.zeros(len(gps_xy))
    cum_turn[0] = 0
    for i in range(1, len(cum_turn)):
        if i < turn_window:
            cum_turn[i] = abs(headings_sm[i-1] - headings_sm[0])
        else:
            cum_turn[i] = abs(headings_sm[i-1] - headings_sm[i-turn_window])

    # Detect turn peaks
    turn_rad = np.deg2rad(turn_angle_deg)
    splits = [0]

    # Find places where heading changes significantly AND cumulatively
    # over a window exceed threshold
    i = turn_window
    while i < len(cum_turn) - turn_window:
        # Look ahead: find the next significant turn
        for j in range(i, len(cum_turn)):
            if cum_turn[j] > turn_rad:
                # Found a turn. Split at the midpoint between this and last split.
                # Check edge length.
                if j - splits[-1] >= 3:  # at least 3 GPS points
                    splits.append(j)
                i = j + turn_window
                break
        else:
            i += 1

    splits.append(len(gps_xy) - 1)

    # Convert splits to (t_start, t_end) and filter short edges
    edges = []
    for k in range(len(splits) - 1):
        i0, i1 = splits[k], splits[k+1]
        if i1 - i0 < 3:
            continue
        t0, t1 = gps_t[i0], gps_t[i1]
        path = np.sum(np.linalg.norm(np.diff(gps_xy[i0:i1+1], axis=0), axis=1))
        if path >= min_edge_len:
            edges.append((t0, t1, len(edges)))

    return edges


def edge_scale_ratio(b0_t, b0_xy, gps_t, gps_xy, t0, t1):
    """Compute VIO_path / GPS_path over [t0, t1]."""
    mb = (b0_t >= t0) & (b0_t <= t1)
    mg = (gps_t >= t0) & (gps_t <= t1)
    if mb.sum() < 3 or mg.sum() < 3:
        return None, None, None
    b_xy = b0_xy[mb]
    g_xy = gps_xy[mg]
    b_len = float(np.sum(np.linalg.norm(np.diff(b_xy, axis=0), axis=1)))
    g_len = float(np.sum(np.linalg.norm(np.diff(g_xy, axis=0), axis=1)))
    if g_len < 1.0:
        return None, None, None
    return b_len / g_len, b_len, g_len


def detect_descent_start(gps_alt, gps_t, min_descent=-3.0):
    """Find where altitude starts consistently dropping (final descent)."""
    dz = np.diff(gps_alt)
    # Smooth
    win = min(20, len(dz)//3)
    if win < 2:
        return None
    kernel = np.ones(win) / win
    dz_sm = np.convolve(dz, kernel, mode='same')
    # Look for sustained negative dz near the end
    for i in range(len(dz_sm) - win, win, -1):
        if np.all(dz_sm[i-win:i] < min_descent / win):
            return gps_t[max(0, i - win)]
    return None


def detect_takeoff(gps_t, gps_xy, min_dist=3.0):
    """Detect when drone first moves min_dist from origin."""
    if len(gps_xy) < 5:
        return None
    d = np.sqrt(np.sum((gps_xy - gps_xy[0])**2, axis=1))
    cr = np.where(d >= min_dist)[0]
    return float(gps_t[int(cr[0])]) if cr.size else None


# ---- main ----

def main():
    all_results = []

    for cfg in FLIGHTS:
        fly = cfg['flight']
        print(f"\n{'='*80}")
        print(f" fly{fly} per-edge scale analysis")
        print(f"{'='*80}")

        b0_iwt2_path = os.path.join(ROOT, cfg['b0_iwt2'])
        b0_iwt5_path = os.path.join(ROOT, cfg['b0_iwt5'])
        gps_path = os.path.join(ROOT, cfg['gps'])

        # Load
        b0_2 = load_b0(b0_iwt2_path)
        b0_5 = load_b0(b0_iwt5_path)
        gps = load_gps(gps_path)

        if b0_2.size == 0 or b0_5.size == 0 or gps.size == 0:
            print(f" [SKIP] missing data")
            continue

        gps_xy, gps_alt, gps_t = gps_to_enu(gps)

        # Detect takeoff and descent
        t_takeoff = detect_takeoff(gps_t, gps_xy)
        t_descent = detect_descent_start(gps_alt, gps_t)

        if t_takeoff is None:
            print(" [SKIP] no takeoff detected")
            continue

        # Focus on post-takeoff, pre-descent
        t_start = t_takeoff
        t_end = t_descent if t_descent is not None else gps_t[-1]

        print(f"  takeoff @ t={t_start:.1f}  descent @ t={t_end:.1f}  "
              f"analysis window = {t_end-t_start:.1f}s")

        # Crop GPS to analysis window
        m_g = (gps_t >= t_start) & (gps_t <= t_end)
        gps_t_crop = gps_t[m_g]
        gps_xy_crop = gps_xy[m_g]

        if len(gps_t_crop) < 10:
            print(" [SKIP] too few GPS points in analysis window")
            continue

        # Segment into edges
        edges = segment_edges(gps_t_crop, gps_xy_crop,
                              min_edge_len=30.0, turn_angle_deg=30.0)

        print(f"  found {len(edges)} edges:")

        # Compute scale per edge for both iwt versions
        edge_results = []
        for t0, t1, eid in edges:
            r2, b2, g2 = edge_scale_ratio(b0_2[:, 0], b0_2[:, 1:3],
                                           gps_t_crop, gps_xy_crop, t0, t1)
            r5, b5, g5 = edge_scale_ratio(b0_5[:, 0], b0_5[:, 1:3],
                                           gps_t_crop, gps_xy_crop, t0, t1)

            edge_label = f"E{eid}"
            if r2 is not None and r5 is not None:
                delta = r5 - r2
                direction = "better" if abs(r5 - 1) < abs(r2 - 1) else "worse"
                print(f"    {edge_label}: t=[{t0-t_start:.0f},{t1-t_start:.0f}]s  "
                      f"GPS={g2:.1f}m  "
                      f"iwt2.0={r2:.3f}  iwt5.0={r5:.3f}  "
                      f"Δ={delta:+.3f} ({direction})")
            elif r2 is not None:
                print(f"    {edge_label}: t=[{t0-t_start:.0f},{t1-t_start:.0f}]s  "
                      f"GPS={g2:.1f}m  iwt2.0={r2:.3f}  iwt5.0=N/A")
            elif r5 is not None:
                print(f"    {edge_label}: t=[{t0-t_start:.0f},{t1-t_start:.0f}]s  "
                      f"GPS={g5:.1f}m  iwt2.0=N/A  iwt5.0={r5:.3f}")

            edge_results.append(dict(
                edge=eid, t0_rel=float(t0 - t_start),
                t1_rel=float(t1 - t_start),
                gps_len=float(g2) if g2 is not None else float(g5),
                r_iwt2=r2, b0_iwt2_len=b2, gps_iwt2_len=g2,
                r_iwt5=r5, b0_iwt5_len=b5, gps_iwt5_len=g5,
            ))

        # Overlap-edge aggregation (edges where both iwt have data)
        valid_edges = [e for e in edge_results
                       if e['r_iwt2'] is not None and e['r_iwt5'] is not None]

        if valid_edges:
            mean_r2 = np.mean([e['r_iwt2'] for e in valid_edges])
            mean_r5 = np.mean([e['r_iwt5'] for e in valid_edges])
            n_better = sum(1 for e in valid_edges
                          if abs(e['r_iwt5'] - 1) < abs(e['r_iwt2'] - 1))
            n_worse = len(valid_edges) - n_better
            print(f"\n  SUMMARY: {len(valid_edges)} edges")
            print(f"    iwt2.0 mean = {mean_r2:.3f}  (err={abs(mean_r2-1)*100:.1f}%)")
            print(f"    iwt5.0 mean = {mean_r5:.3f}  (err={abs(mean_r5-1)*100:.1f}%)")
            print(f"    better: {n_better} edges  worse: {n_worse} edges")

        all_results.append(dict(
            flight=fly,
            t_takeoff=float(t_start),
            t_descent=float(t_end) if t_descent else None,
            edges=edge_results,
            n_edges=len(valid_edges),
            n_better=n_better if valid_edges else 0,
            n_worse=n_worse if valid_edges else 0,
        ))

    # ---- plot ----
    if not _HAVE_MPL:
        print("\nno matplotlib, skipping plots")
        return

    n_flights = len(all_results)
    if n_flights == 0:
        print("no results to plot")
        return

    fig, axes = plt.subplots(2, 2, figsize=(18, 12))
    axes = axes.flatten()

    colors = {'iwt2.0': '#3498db', 'iwt5.0': '#e74c3c'}
    bar_colors_2 = '#3498db'
    bar_colors_5 = '#e74c3c'

    for idx, res in enumerate(all_results):
        fly = res['flight']
        ax = axes[idx]

        edges = res['edges']
        if not edges:
            ax.text(0.5, 0.5, f'fly{fly}: no edges', ha='center', va='center',
                    transform=ax.transAxes, fontsize=14, color='gray')
            ax.set_title(f'fly{fly}')
            continue

        valid = [e for e in edges
                 if e['r_iwt2'] is not None and e['r_iwt5'] is not None]

        if not valid:
            ax.text(0.5, 0.5, f'fly{fly}: no valid edges', ha='center', va='center',
                    transform=ax.transAxes, fontsize=14, color='gray')
            ax.set_title(f'fly{fly}')
            continue

        n = len(valid)
        x = np.arange(n)
        width = 0.35

        r2_vals = [e['r_iwt2'] for e in valid]
        r5_vals = [e['r_iwt5'] for e in valid]
        labels = [f"E{e['edge']}" for e in valid]
        gps_lens = [e['gps_len'] for e in valid]

        bars2 = ax.bar(x - width/2, r2_vals, width, color=bar_colors_2,
                       alpha=0.85, label='iwt=2.0', edgecolor='white', linewidth=0.5)
        bars5 = ax.bar(x + width/2, r5_vals, width, color=bar_colors_5,
                       alpha=0.85, label='iwt=5.0', edgecolor='white', linewidth=0.5)

        # Annotate values on bars
        for bar, val in zip(bars2, r2_vals):
            y_pos = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, y_pos + 0.02,
                    f'{val:.3f}', ha='center', va='bottom', fontsize=7, color=bar_colors_2)
        for bar, val in zip(bars5, r5_vals):
            y_pos = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, y_pos + 0.02,
                    f'{val:.3f}', ha='center', va='bottom', fontsize=7, color=bar_colors_5)

        # Add GPS length subtitle per edge
        gps_text = "  ".join([f"{l}:{g:.0f}m" for l, g in zip(labels, gps_lens)])

        ax.axhline(1.0, color='black', linewidth=0.8, alpha=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=9)
        ax.set_ylabel('scale ratio (VIO/GPS)')
        ax.set_title(f'fly{fly} per-edge scale: iwt=2.0 vs iwt=5.0')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3, axis='y')

        # Color bars: green if closer to 1, red if farther
        for i in range(n):
            d2 = abs(r2_vals[i] - 1)
            d5 = abs(r5_vals[i] - 1)
            if d5 < d2:
                bars5[i].set_facecolor('#27ae60')  # better
                bars2[i].set_facecolor('#e74c3c')  # worse
            elif d2 < d5:
                bars2[i].set_facecolor('#27ae60')  # better
                bars5[i].set_facecolor('#e74c3c')  # worse

        # Annotation at bottom
        n_better = res.get('n_better', 0)
        n_worse = res.get('n_worse', 0)
        ax.text(0.5, -0.18, f'iwt=5.0: {n_better} better, {n_worse} worse',
                transform=ax.transAxes, ha='center', fontsize=10,
                fontweight='bold',
                color='green' if n_better >= n_worse else 'red')

    # Hide unused subplot if 3 flights or fewer
    for idx in range(len(all_results), len(axes)):
        axes[idx].set_visible(False)

    fig.suptitle('Per-Edge Scale Comparison: iwt=2.0 vs iwt=5.0\n'
                 'green = closer to 1.0 (better)   red = farther from 1.0 (worse)',
                 fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0.02, 1, 0.95])

    png_path = os.path.join(OUT_DIR, 'per_edge_scale_comparison.png')
    fig.savefig(png_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"\n[saved] {png_path}")

    # ---- summary bar chart: per-flight win/loss ----
    fig2, ax2 = plt.subplots(figsize=(10, 5))
    flights = [r['flight'] for r in all_results]
    better = [r['n_better'] for r in all_results]
    worse = [r['n_worse'] for r in all_results]
    x = np.arange(len(flights))
    w = 0.3
    ax2.bar(x - w/2, better, w, color='#27ae60', label='iwt5.0 better', alpha=0.85)
    ax2.bar(x + w/2, worse, w, color='#e74c3c', label='iwt5.0 worse', alpha=0.85)
    ax2.set_xticks(x)
    ax2.set_xticklabels([f'fly{f}' for f in flights], fontsize=11)
    ax2.set_ylabel('number of edges')
    ax2.set_title('Per-Edge Scale: iwt=5.0 vs iwt=2.0 (win/loss per flight)')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3, axis='y')

    for i in range(len(flights)):
        ax2.text(x[i] - w/2, better[i] + 0.1, str(better[i]), ha='center', fontsize=10)
        ax2.text(x[i] + w/2, worse[i] + 0.1, str(worse[i]), ha='center', fontsize=10)

    png2 = os.path.join(OUT_DIR, 'per_edge_win_loss.png')
    fig2.tight_layout()
    fig2.savefig(png2, dpi=150)
    plt.close(fig2)
    print(f"[saved] {png2}")

    # ---- combined edge-by-edge bar chart (all flights) ----
    fig3, ax3 = plt.subplots(figsize=(20, 6))

    all_labels = []
    all_r2 = []
    all_r5 = []
    bar_x = []
    x_pos = 0
    flight_boundaries = [0]

    for res in all_results:
        valid = [e for e in res['edges']
                 if e['r_iwt2'] is not None and e['r_iwt5'] is not None]
        for e in valid:
            all_labels.append(f"F{res['flight']}E{e['edge']}")
            all_r2.append(e['r_iwt2'])
            all_r5.append(e['r_iwt5'])
            bar_x.append(x_pos)
            x_pos += 1
        flight_boundaries.append(x_pos)

    w2 = 0.35
    ax3.bar([x - w2/2 for x in bar_x], all_r2, w2, color=bar_colors_2, alpha=0.85,
            label='iwt=2.0')
    ax3.bar([x + w2/2 for x in bar_x], all_r5, w2, color=bar_colors_5, alpha=0.85,
            label='iwt=5.0')

    # Color individual bars
    for i in range(len(all_r2)):
        d2 = abs(all_r2[i] - 1)
        d5 = abs(all_r5[i] - 1)
        # Access bars by index
        pass  # simpler to re-plot below

    ax3.axhline(1.0, color='black', linewidth=0.8, alpha=0.5)
    ax3.set_xticks(bar_x)
    ax3.set_xticklabels(all_labels, fontsize=7, rotation=45)
    ax3.set_ylabel('scale ratio (VIO/GPS)')
    ax3.set_title('All flights per-edge scale: iwt=2.0 vs iwt=5.0')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3, axis='y')

    # Flight boundaries
    for b in flight_boundaries[1:-1]:
        ax3.axvline(b - 0.5, color='gray', linewidth=1, linestyle='--', alpha=0.5)

    png3 = os.path.join(OUT_DIR, 'per_edge_all_flights.png')
    fig3.tight_layout()
    fig3.savefig(png3, dpi=150)
    plt.close(fig3)
    print(f"[saved] {png3}")

    # Save JSON
    json_path = os.path.join(OUT_DIR, 'per_edge_scale.json')
    with open(json_path, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)
    print(f"[saved] {json_path}")

    print("\nDone.")


if __name__ == "__main__":
    main()
