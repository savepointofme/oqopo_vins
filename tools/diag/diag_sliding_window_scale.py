#!/usr/bin/env python3
"""Sliding-window scale localization (B0 vs aligned GPS).

For each flight:
  1. Restrict to GPS-overlap interval only (eliminate post-GPS divergence).
  2. Compute a 30-s sliding-window scale_ratio = pathlen(B0_xy) / pathlen(GPS_xy)
     stepped every 5 s.
  3. Identify the first sustained interval where ratio leaves [0.85, 1.15].
  4. Separate (a) first-edge scale at takeoff vs (b) late-flight divergence.
  5. Report when the under-/over-scale signature first appears: at init,
     during vertical_climb, or only after horizontal_motion begins.

This is OFFLINE EVALUATION ONLY. GPS is not fused into the EKF.
"""
from __future__ import annotations
import math, os, json, sys
import numpy as np
try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception: pass
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:
    _HAVE_MPL = False

FLIGHTS = [
    dict(flight=1,
         b0='20260509_fly1/result/baselines_v1/B0_no_gps.txt',
         b0_iwt5='20260509_fly1/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv'),
    dict(flight=2,
         b0='20260509_fly2/result/baselines_v1/B0_no_gps.txt',
         b0_iwt5='20260509_fly2/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv'),
    dict(flight=3,
         b0='20260509_fly3/result/baselines_v1/B0_no_gps.txt',
         b0_iwt5='20260509_fly3/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
    dict(flight=4,
         b0='20260509_fly4/result/stage_a_v2/R0_nogps.txt',
         b0_iwt5='20260509_fly4/result/init_window_time_sweep/B0_iwt5.0.txt',
         gps='20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
]

OUT = 'comparison_plots/sliding_window_scale'
os.makedirs(OUT, exist_ok=True)

def load_b0(p):
    rows, lt = [], -math.inf
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            try:
                ps = ln.split()
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
            try:
                t, lat, lon, alt = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
            except (ValueError, IndexError): continue
            rows.append((t, lat, lon, alt))
    a = np.asarray(rows)
    if a.size and a[0, 0] > 1e11: a[:, 0] *= 1e-9
    return a[np.argsort(a[:, 0])]

def wgs84_to_ecef(lat, lon, alt):
    a = 6378137.0; e2 = 6.69437999014e-3
    s, c = np.sin(np.deg2rad(lat)), np.cos(np.deg2rad(lat))
    so, co = np.sin(np.deg2rad(lon)), np.cos(np.deg2rad(lon))
    N = a / np.sqrt(1 - e2 * s * s)
    return np.stack([(N + alt) * c * co, (N + alt) * c * so,
                     (N * (1 - e2) + alt) * s], axis=-1)

def ecef_to_enu_R(lat0, lon0):
    sl, cl = np.sin(np.deg2rad(lat0)), np.cos(np.deg2rad(lat0))
    so, co = np.sin(np.deg2rad(lon0)), np.cos(np.deg2rad(lon0))
    return np.array([[-so, co, 0], [-sl*co, -sl*so, cl], [cl*co, cl*so, sl]])

def gps_to_enu_xy(g):
    lat0, lon0, alt0 = g[0, 1], g[0, 2], g[0, 3]
    e0 = wgs84_to_ecef(lat0, lon0, alt0); R = ecef_to_enu_R(lat0, lon0)
    e = wgs84_to_ecef(g[:, 1], g[:, 2], g[:, 3])
    enu = (e - e0) @ R.T
    return enu[:, :2], g[:, 3]   # xy, alt

def path_len_xy(xy):
    if len(xy) < 2: return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))

def detect_takeoff(t_rel, xy, min_dist=2.0):
    if len(t_rel) < 5: return None
    d = np.sqrt((xy[:, 0] - xy[0, 0]) ** 2 + (xy[:, 1] - xy[0, 1]) ** 2)
    cr = np.where(d >= min_dist)[0]
    return float(t_rel[int(cr[0])]) if cr.size else None

def detect_first_alt_rise(t_rel, alt, min_rise=2.0):
    if len(t_rel) < 5: return None, float(np.median(alt[:50]))
    base = float(np.median(alt[: max(5, len(alt) // 50)]))
    cr = np.where(alt - base >= min_rise)[0]
    return (float(t_rel[int(cr[0])]) if cr.size else None), base

# --------- per-flight ---------

results = []
for cfg in FLIGHTS:
    fly = cfg['flight']
    print(f'\n{"="*78}\n  fly{fly} sliding-window scale localization\n{"="*78}')
    b0 = load_b0(cfg['b0'])
    g  = load_gps(cfg['gps'])
    if b0.size == 0 or g.size == 0:
        print(' [SKIP]'); continue

    g_xy, g_alt = gps_to_enu_xy(g)
    g_t = g[:, 0]

    # OVERLAP-ONLY interval
    t_lo = max(b0[0, 0], g_t[0])
    t_hi = min(b0[-1, 0], g_t[-1])
    t0 = t_lo
    overlap_dur = t_hi - t_lo

    # B0 clipped to overlap
    m_b0 = (b0[:, 0] >= t_lo) & (b0[:, 0] <= t_hi)
    b0_t = b0[m_b0, 0]; b0_xy = b0[m_b0, 1:3]; b0_z = b0[m_b0, 3]
    # GPS clipped to overlap
    m_g = (g_t >= t_lo) & (g_t <= t_hi)
    gpst = g_t[m_g]; gpsxy = g_xy[m_g]; gpsalt = g_alt[m_g]
    g_rel = gpst - t0

    # Detect takeoff + alt-rise within the overlap
    t_takeoff_rel = detect_takeoff(g_rel, gpsxy)
    t_alt_rise_rel, ground_alt = detect_first_alt_rise(g_rel, gpsalt)

    print(f'  overlap interval = [{t_lo:.3f}, {t_hi:.3f}]  dur={overlap_dur:.1f}s')
    print(f'  t_first_alt_rise = {t_alt_rise_rel}s   t_xy_takeoff = {t_takeoff_rel}s')
    print(f'  ground_alt       = {ground_alt:.2f}m')

    # SLIDING WINDOW: 30s window, 5s step, evaluated entirely within overlap
    WIN, STEP = 30.0, 5.0
    centers, ratios, b0_lens, gps_lens, b0_dz, gps_dz = [], [], [], [], [], []
    t = 0.0
    while t + WIN <= overlap_dur:
        lo, hi = t, t + WIN
        m_bb = (b0_t - t0 >= lo) & (b0_t - t0 <= hi)
        m_gg = (g_rel    >= lo) & (g_rel    <= hi)
        if m_bb.sum() >= 5 and m_gg.sum() >= 5:
            l_b = path_len_xy(b0_xy[m_bb])
            l_g = path_len_xy(gpsxy[m_gg])
            r = (l_b / l_g) if l_g > 0 else float('nan')
            dz_b = float(b0_z[m_bb].max() - b0_z[m_bb].min())
            dz_g = float(gpsalt[m_gg].max() - gpsalt[m_gg].min())
        else:
            l_b = l_g = r = dz_b = dz_g = float('nan')
        centers.append(t + WIN / 2)
        ratios.append(r); b0_lens.append(l_b); gps_lens.append(l_g)
        b0_dz.append(dz_b); gps_dz.append(dz_g)
        t += STEP
    centers = np.asarray(centers); ratios = np.asarray(ratios)
    b0_lens = np.asarray(b0_lens); gps_lens = np.asarray(gps_lens)

    # Mask: only consider windows where GPS_len >= 5 m (window contains real motion)
    motion_mask = gps_lens >= 5.0
    # First sustained divergence: first window where |ratio - 1| > 0.15 for 3 windows in a row
    diverge_idx = None
    if motion_mask.sum() >= 5:
        ok_band = np.abs(ratios - 1.0) <= 0.15
        for i in range(len(ratios) - 2):
            if (motion_mask[i] and motion_mask[i+1] and motion_mask[i+2]
                and not ok_band[i] and not ok_band[i+1] and not ok_band[i+2]):
                diverge_idx = i
                break
    diverge_t = float(centers[diverge_idx]) if diverge_idx is not None else None

    # Catastrophic divergence: where ratio > 3 sustained for 2 windows
    catastrophic_idx = None
    for i in range(len(ratios) - 1):
        if (motion_mask[i] and motion_mask[i+1]
            and ratios[i] > 3.0 and ratios[i+1] > 3.0):
            catastrophic_idx = i; break
    catastrophic_t = float(centers[catastrophic_idx]) if catastrophic_idx is not None else None

    # Phase-anchored summary
    def phase_of(t_rel):
        if t_takeoff_rel is None: return 'unknown'
        if t_alt_rise_rel is not None and t_rel < t_alt_rise_rel: return 'ground_hover'
        if t_rel < t_takeoff_rel: return 'vertical_climb'
        return 'horizontal_motion'

    # First-edge scale = mean ratio over [takeoff, takeoff+80]
    if t_takeoff_rel is not None:
        fe_mask = (centers >= t_takeoff_rel) & (centers <= t_takeoff_rel + 80) & motion_mask
        first_edge_r = float(np.nanmean(ratios[fe_mask])) if fe_mask.any() else float('nan')
        # Window-by-window from takeoff for next 80s
        print(f'\n  FIRST 80s POST-TAKEOFF (window-centered):')
        print(f'  {"t_rel":>7s} {"phase":>17s} {"B0_xy":>8s} {"GPS_xy":>8s} {"ratio":>7s} '
              f'{"B0_dz":>6s} {"GPS_dz":>7s}')
        for i in range(len(centers)):
            if not motion_mask[i]: continue
            if t_takeoff_rel <= centers[i] <= t_takeoff_rel + 80:
                ph = phase_of(centers[i])
                print(f'  {centers[i]:>6.1f}s {ph:>17s} '
                      f'{b0_lens[i]:>8.2f} {gps_lens[i]:>8.2f} {ratios[i]:>7.3f} '
                      f'{b0_dz[i]:>6.2f} {gps_dz[i]:>7.2f}')
    else:
        first_edge_r = float('nan')

    # Window-by-window for ENTIRE overlap (compact)
    print(f'\n  FULL OVERLAP WINDOWS (compact, motion-only):')
    print(f'  {"t_rel":>7s} {"phase":>17s} {"B0_xy":>9s} {"GPS_xy":>9s} {"ratio":>7s}')
    for i in range(len(centers)):
        if not motion_mask[i]: continue
        ph = phase_of(centers[i])
        marker = ''
        if diverge_idx is not None and i == diverge_idx: marker = '  <-- DIVERGE'
        if catastrophic_idx is not None and i == catastrophic_idx: marker += '  <-- CATASTROPHIC'
        print(f'  {centers[i]:>6.1f}s {ph:>17s} '
              f'{b0_lens[i]:>9.2f} {gps_lens[i]:>9.2f} {ratios[i]:>7.3f}{marker}')

    # Phase-aggregated scale
    def phase_agg(phase_label):
        mask = motion_mask.copy()
        for i in range(len(centers)):
            if phase_of(centers[i]) != phase_label: mask[i] = False
        if mask.sum() == 0:
            return float('nan'), 0, 0, 0
        return (float(np.nansum(b0_lens[mask]) / np.nansum(gps_lens[mask])),
                int(mask.sum()),
                float(np.nansum(b0_lens[mask])),
                float(np.nansum(gps_lens[mask])))

    r_gh, n_gh, b_gh, g_gh = phase_agg('ground_hover')
    r_vc, n_vc, b_vc, g_vc = phase_agg('vertical_climb')
    r_hm, n_hm, b_hm, g_hm = phase_agg('horizontal_motion')

    print(f'\n  PHASE-AGGREGATED SCALE (sum_B0_motion / sum_GPS_motion over phase\'s windows):')
    print(f'    ground_hover      : n_win={n_gh}  B0_total={b_gh:>9.2f}  GPS_total={g_gh:>9.2f}  ratio={r_gh:.3f}')
    print(f'    vertical_climb    : n_win={n_vc}  B0_total={b_vc:>9.2f}  GPS_total={g_vc:>9.2f}  ratio={r_vc:.3f}')
    print(f'    horizontal_motion : n_win={n_hm}  B0_total={b_hm:>9.2f}  GPS_total={g_hm:>9.2f}  ratio={r_hm:.3f}')

    print(f'\n  divergence_onset_t_rel    = {diverge_t}')
    print(f'  catastrophic_onset_t_rel  = {catastrophic_t}')
    print(f'  first_edge_scale (mean ratio over takeoff..+80s windows) = {first_edge_r}')

    summary = dict(
        flight=fly,
        overlap_t_lo=float(t_lo), overlap_t_hi=float(t_hi),
        overlap_dur_s=float(overlap_dur),
        t_first_alt_rise_rel=t_alt_rise_rel,
        t_xy_takeoff_rel=t_takeoff_rel,
        ground_alt_m=float(ground_alt),
        sliding_window={
            'WIN': WIN, 'STEP': STEP,
            'centers_rel': centers.tolist(),
            'ratios': ratios.tolist(),
            'b0_xy_path': b0_lens.tolist(),
            'gps_xy_path': gps_lens.tolist(),
            'motion_mask': motion_mask.tolist(),
        },
        first_edge_scale_mean=first_edge_r,
        phase_scale=dict(
            ground_hover=dict(n=n_gh, B0=b_gh, GPS=g_gh, ratio=r_gh),
            vertical_climb=dict(n=n_vc, B0=b_vc, GPS=g_vc, ratio=r_vc),
            horizontal_motion=dict(n=n_hm, B0=b_hm, GPS=g_hm, ratio=r_hm),
        ),
        diverge_onset_rel=diverge_t,
        catastrophic_onset_rel=catastrophic_t,
    )
    results.append(summary)

    # plot
    if _HAVE_MPL:
        fig, ax = plt.subplots(1, 1, figsize=(13, 5))
        # color-code by motion_mask
        valid = motion_mask
        ax.plot(centers[valid], ratios[valid], 'b-o', lw=1.2, ms=3,
                label='scale_ratio (B0_xy_path / GPS_xy_path, 30s win/5s step)')
        # Annotate phases as vertical bands
        if t_alt_rise_rel is not None:
            ax.axvline(t_alt_rise_rel, color='gray', lw=1.0, ls=':',
                       label=f'GPS alt rise @ {t_alt_rise_rel:.1f}s')
        if t_takeoff_rel is not None:
            ax.axvline(t_takeoff_rel, color='orange', lw=1.4, ls='--',
                       label=f'GPS xy takeoff @ {t_takeoff_rel:.1f}s')
        if diverge_t is not None:
            ax.axvline(diverge_t, color='red', lw=1.4, ls='-',
                       label=f'divergence onset @ {diverge_t:.1f}s')
        if catastrophic_t is not None:
            ax.axvline(catastrophic_t, color='purple', lw=2.0, ls='-',
                       label=f'catastrophic onset @ {catastrophic_t:.1f}s')
        ax.axhspan(0.85, 1.15, alpha=0.10, color='green', label='in-band [0.85, 1.15]')
        ax.axhline(1.0, color='k', lw=0.6, alpha=0.5)
        ax.set_yscale('symlog', linthresh=1.0)
        ax.set_xlabel('window center (s rel to overlap start)')
        ax.set_ylabel('scale_ratio (B0 / GPS)  [symlog]')
        ax.set_title(f'fly{fly} sliding-window scale (overlap-only, 30s/5s step)')
        ax.grid(alpha=0.3, which='both')
        ax.legend(loc='best', fontsize=8)
        p = os.path.join(OUT, f'fly{fly}_sliding_scale.png')
        fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
        print(f'\n  [saved] {p}')

# Summary across flights
print(f'\n{"="*100}')
print(f'  CROSS-FLIGHT SUMMARY')
print(f'{"="*100}')
hdr = (f'{"fly":>3s} {"overlap":>8s} {"alt_rise":>8s} {"takeoff":>8s} '
       f'{"r_gh":>6s} {"r_vc":>6s} {"r_hm":>6s} {"first_edge":>10s} '
       f'{"diverge":>8s} {"catastr":>8s}')
print(hdr); print('-' * len(hdr))
for r in results:
    print(f'{r["flight"]:>3d} {r["overlap_dur_s"]:>7.1f}s '
          f'{(r["t_first_alt_rise_rel"] or float("nan")):>7.1f}s '
          f'{(r["t_xy_takeoff_rel"] or float("nan")):>7.1f}s '
          f'{r["phase_scale"]["ground_hover"]["ratio"]:>6.3f} '
          f'{r["phase_scale"]["vertical_climb"]["ratio"]:>6.3f} '
          f'{r["phase_scale"]["horizontal_motion"]["ratio"]:>6.3f} '
          f'{r["first_edge_scale_mean"]:>10.3f} '
          f'{(r["diverge_onset_rel"] or float("nan")):>7.1f}s '
          f'{(r["catastrophic_onset_rel"] or float("nan")):>7.1f}s')

with open(os.path.join(OUT, 'summary.json'), 'w') as f:
    json.dump(results, f, indent=2, default=str)
print(f'\n[saved] {OUT}/summary.json')
