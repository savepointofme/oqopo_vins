#!/usr/bin/env python3
"""Hover / takeoff / multi-window scale diagnostic across fly1-fly4.

For each pure-monocular B0 run:
  A. Hover-drift quantification (between init and GPS takeoff)
  B. Slow-vertical-takeoff diagnostic (first 30 s post-takeoff)
  C. Multi-window scale (hover-only, vertical-takeoff, first horizontal edge,
     full GPS-covered motion, fly4 late southwest segment)
  D. Init/bias correlation table

Inputs:
  - 20260509_flyN/result/baselines_v1/B0_no_gps.txt        TUM trajectory
  - 20260509_flyN/result/baselines_v1/B0_no_gps.txt.bias   per-frame (vx vy vz bg_xyz ba_xyz)
  - 20260509_flyN/result/baselines_v1/B0_no_gps.log        run log (init mode/bias)
  - 20260509_flyN/result/gps_tum_time_alignment*/aligned_gps_cam_time.csv

GPS is ONLY used as an offline evaluation reference (no fusion).
"""
from __future__ import annotations
import math, os, re, json, sys
import numpy as np
# Force UTF-8 stdout so unicode (m/s², °) prints on Windows GBK shells.
try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass
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
         bias='20260509_fly1/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly1/result/baselines_v1/B0_no_gps.log',
         gps='20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv'),
    dict(flight=2,
         b0='20260509_fly2/result/baselines_v1/B0_no_gps.txt',
         bias='20260509_fly2/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly2/result/baselines_v1/B0_no_gps.log',
         gps='20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv'),
    dict(flight=3,
         b0='20260509_fly3/result/baselines_v1/B0_no_gps.txt',
         bias='20260509_fly3/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly3/result/baselines_v1/B0_no_gps.log',
         gps='20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
    dict(flight=4,
         b0='20260509_fly4/result/stage_a_v2/R0_nogps.txt',
         bias='20260509_fly4/result/stage_a_v2/R0_nogps.txt.bias',
         log='20260509_fly4/result/stage_a_v2/R0_nogps.log',
         gps='20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
]

OUT_DIR = 'comparison_plots/hover_takeoff_scale'
os.makedirs(OUT_DIR, exist_ok=True)

# ---------------- I/O helpers ----------------

def load_b0(p):
    rows, last_t = [], -math.inf
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            try:
                t, x, y, z = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
            except (ValueError, IndexError):
                continue
            if t <= last_t: continue
            rows.append((t, x, y, z)); last_t = t
    return np.asarray(rows)

def load_bias(p):
    """Returns (t, vx, vy, vz, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z)."""
    rows, last_t = [], -math.inf
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            if len(ps) < 10: continue
            try:
                t = float(ps[0])
                if t <= last_t: continue
                rows.append([t] + [float(x) for x in ps[1:10]]); last_t = t
            except ValueError: continue
    return np.asarray(rows)

def load_gps_llz(p):
    rows = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try:
                t, lat, lon, alt = float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])
                rows.append((t, lat, lon, alt))
            except (ValueError, IndexError): continue
    a = np.asarray(rows)
    if a.size and a[0, 0] > 1e11: a[:, 0] = a[:, 0] * 1e-9
    return a[np.argsort(a[:, 0])]

def parse_init_log(p):
    """Parse [init]: orientation/bias gyro/bias accel/disparity from log."""
    out = dict(init_mode='?', init_disparity=None, init_bg=None, init_ba=None,
               cam_imu_timeoffset=None)
    if not os.path.isfile(p): return out
    # Strip ANSI; track the LAST disparity printed before success.
    last_disp = None
    with open(p, errors='replace') as f:
        for ln in f:
            ln = re.sub(r'\x1b\[[0-9;]*m', '', ln)
            md = re.search(r'\[init\]: disparity is ([\d.]+),([\d.]+) \(([\d.]+) thresh\)', ln)
            if md: last_disp = (float(md.group(1)), float(md.group(2)),
                                 float(md.group(3))); continue
            if '[init]: successful initialization' in ln:
                out['init_disparity'] = last_disp
                # 2026-05-17 corrected: with try_zupt=false, wait_for_jerk=true,
                # the selector picks StaticInitializer when older-half disparity
                # is below init_max_disparity. All four flights took the static path.
                out['init_mode'] = 'static (via wait_for_jerk; check selector logic)'
            mbg = re.search(r'\[init\]: bias gyro = ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
            if mbg: out['init_bg'] = (float(mbg.group(1)), float(mbg.group(2)),
                                       float(mbg.group(3)))
            mba = re.search(r'\[init\]: bias accel = ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
            if mba: out['init_ba'] = (float(mba.group(1)), float(mba.group(2)),
                                       float(mba.group(3)))
            mto = re.search(r'camera-imu timeoffset = ([\d.]+)', ln)
            if mto: out['cam_imu_timeoffset'] = float(mto.group(1))
    return out

# WGS84 -> ENU
def wgs84_to_ecef(lat_d, lon_d, alt):
    a = 6378137.0; e2 = 6.69437999014e-3
    lat = np.deg2rad(lat_d); lon = np.deg2rad(lon_d)
    s = np.sin(lat); c = np.cos(lat)
    N = a / np.sqrt(1 - e2 * s * s)
    x = (N + alt) * c * np.cos(lon)
    y = (N + alt) * c * np.sin(lon)
    z = (N * (1 - e2) + alt) * s
    return np.stack([x, y, z], axis=-1)

def ecef_to_enu_R(lat0_d, lon0_d):
    lat = np.deg2rad(lat0_d); lon = np.deg2rad(lon0_d)
    sl, cl = np.sin(lat), np.cos(lat); so, co = np.sin(lon), np.cos(lon)
    return np.array([[-so, co, 0], [-sl*co, -sl*so, cl], [cl*co, cl*so, sl]])

def gps_to_enu(g):
    lat0, lon0, alt0 = g[0, 1], g[0, 2], g[0, 3]
    e0 = wgs84_to_ecef(lat0, lon0, alt0); R = ecef_to_enu_R(lat0, lon0)
    e = wgs84_to_ecef(g[:, 1], g[:, 2], g[:, 3]); enu = (e - e0) @ R.T
    out = np.zeros((len(g), 5))
    out[:, 0] = g[:, 0]; out[:, 1:4] = enu; out[:, 4] = g[:, 3]
    return out, (lat0, lon0, alt0)

def path_len_xy(xy):
    if len(xy) < 2: return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))

def chord_xy(xy):
    if len(xy) < 2: return 0.0
    return float(np.linalg.norm(xy[-1] - xy[0]))

def detect_gps_takeoff(t_rel, xy, alt, min_dist_m=2.0):
    if len(t_rel) < 5: return None
    d = np.sqrt((xy[:, 0] - xy[0, 0]) ** 2 + (xy[:, 1] - xy[0, 1]) ** 2)
    cr = np.where(d >= min_dist_m)[0]
    if cr.size == 0: return None
    return float(t_rel[int(cr[0])])

def detect_gps_first_alt_rise(t_rel, alt, min_rise_m=2.0):
    """When does GPS altitude first rise by min_rise_m from its early-mean?"""
    if len(t_rel) < 5: return None
    early = alt[: max(5, len(alt) // 50)]
    base = float(np.median(early))
    cr = np.where(alt - base >= min_rise_m)[0]
    if cr.size == 0: return None
    return float(t_rel[int(cr[0])]), base

def yaw_align_xy(b0_xy, gps_xy):
    """2D Procrustes rotation, then re-anchor at first sample to match GPS start."""
    b = b0_xy - b0_xy[0]; g = gps_xy - gps_xy[0]
    v = ~np.isnan(g[:, 0]) & ~np.isnan(g[:, 1])
    if v.sum() < 5: return 0.0, b0_xy
    bv = b[v]; gv = g[v]
    s = np.sum(bv[:, 0] * gv[:, 1] - bv[:, 1] * gv[:, 0])
    c = np.sum(bv[:, 0] * gv[:, 0] + bv[:, 1] * gv[:, 1])
    theta = float(np.arctan2(s, c))
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    rot = (R @ b0_xy.T).T
    return theta, rot - rot[0] + gps_xy[0]

# ---------------- main per-flight analysis ----------------

results = []
for cfg in FLIGHTS:
    fly = cfg['flight']
    print(f'\n{"="*78}\n  fly{fly}\n{"="*78}')
    b0   = load_b0(cfg['b0'])
    bias = load_bias(cfg['bias'])
    gps  = load_gps_llz(cfg['gps'])
    log  = parse_init_log(cfg['log'])
    if b0.size == 0 or gps.size == 0:
        print(' [SKIP] missing inputs'); continue

    gps_enu5, anchor = gps_to_enu(gps)
    gps_t   = gps_enu5[:, 0]
    gps_xy  = gps_enu5[:, 1:3]
    gps_alt = gps_enu5[:, 4]

    t_lo = max(b0[0, 0], gps_t[0])
    t_hi = min(b0[-1, 0], gps_t[-1])
    t0 = t_lo
    b0_in = (b0[:, 0] >= t_lo) & (b0[:, 0] <= t_hi)
    b0_t = b0[b0_in, 0]; b0_xyz = b0[b0_in, 1:4]
    bi_in = (bias[:, 0] >= t_lo) & (bias[:, 0] <= t_hi)
    b_t = bias[bi_in, 0]; b_v = bias[bi_in, 1:4]
    b_bg = bias[bi_in, 4:7]; b_ba = bias[bi_in, 7:10]

    # Overlap-aligned GPS subset
    g_in = (gps_t >= t_lo) & (gps_t <= t_hi)
    g_t = gps_t[g_in]; g_xy = gps_xy[g_in]; g_alt = gps_alt[g_in]
    g_rel = g_t - t0
    if g_rel.size < 10:
        print(' [SKIP] GPS overlap too short'); continue

    # Detect takeoff & first alt rise from GPS
    t_takeoff = detect_gps_takeoff(g_rel, g_xy, g_alt, min_dist_m=2.0)
    alt_rise = detect_gps_first_alt_rise(g_rel, g_alt, min_rise_m=2.0)
    t_alt_rise = alt_rise[0] if alt_rise else None
    gps_ground_alt = alt_rise[1] if alt_rise else float(np.median(g_alt[:50]))

    # ---- A. PHASE DECOMPOSITION ------------------------------------------
    # Three phases between init and horizontal takeoff:
    #   ground_hover  : [0, t_first_alt_rise]      drone on ground, no motion
    #   vertical_climb: [t_first_alt_rise, t_xy_takeoff]
    #                                              drone airborne but no
    #                                              significant horizontal motion
    #   horizontal    : [t_xy_takeoff, end]        normal flight
    # If t_first_alt_rise is missing fall back to t_xy_takeoff for ground_hover
    # end; if both are missing, treat everything as "ground_hover".
    phase_bounds = dict(
        ground_hover_end   = t_alt_rise if t_alt_rise is not None else t_takeoff,
        vertical_climb_end = t_takeoff,
    )
    # Build per-phase masks (B0, bias, GPS) on rel-time
    def make_mask(t_arr_rel, lo, hi):
        return (t_arr_rel >= lo) & (t_arr_rel <= hi)

    b0_rel = b0_t - t0; bi_rel = b_t - t0
    phases = {}
    # ground_hover
    if phase_bounds['ground_hover_end'] is not None:
        lo, hi = 0.0, max(1.0, phase_bounds['ground_hover_end'])
    else:
        lo = hi = 0.0
    phases['ground_hover'] = (lo, hi)
    # vertical_climb
    lo_v = phases['ground_hover'][1]
    hi_v = phase_bounds['vertical_climb_end'] if phase_bounds['vertical_climb_end'] is not None else lo_v
    if hi_v < lo_v: hi_v = lo_v
    phases['vertical_climb'] = (lo_v, hi_v)

    def phase_stats(label, lo, hi):
        m_b0 = make_mask(b0_rel, lo, hi)
        m_bi = make_mask(bi_rel, lo, hi)
        m_g  = make_mask(g_rel, lo, hi)
        n_b0 = int(m_b0.sum())
        if n_b0 < 3:
            return dict(label=label, span=[lo, hi], n_b0=n_b0)
        xy = b0_xyz[m_b0, :2]; z = b0_xyz[m_b0, 2]
        xy_chord = float(np.linalg.norm(xy[-1] - xy[0]))
        xy_path  = path_len_xy(xy)
        z_drift  = float(z[-1] - z[0])
        z_path   = float(np.sum(np.abs(np.diff(z))))
        if m_bi.sum() >= 3:
            v_norm = np.linalg.norm(b_v[m_bi], axis=1)
            v_mean = float(v_norm.mean()); v_max = float(v_norm.max())
            ba_p = b_ba[m_bi]; bg_p = b_bg[m_bi]
            d_ba = (ba_p[-1] - ba_p[0]).tolist()
            d_bg = (bg_p[-1] - bg_p[0]).tolist()
            ba_start = ba_p[0].tolist(); ba_end = ba_p[-1].tolist()
            bg_start = bg_p[0].tolist(); bg_end = bg_p[-1].tolist()
        else:
            v_mean = v_max = float('nan')
            d_ba = d_bg = [float('nan')] * 3
            ba_start = ba_end = bg_start = bg_end = [float('nan')] * 3
        # GPS stats over same window
        if m_g.sum() >= 3:
            gxy = g_xy[m_g]; galt = g_alt[m_g]
            g_xy_chord = float(np.linalg.norm(gxy[-1] - gxy[0]))
            g_xy_path  = path_len_xy(gxy)
            g_z_drift  = float(galt[-1] - galt[0])
        else:
            g_xy_chord = g_xy_path = g_z_drift = float('nan')
        return dict(label=label, span=[lo, hi], n_b0=n_b0,
                    B0_xy_chord=xy_chord, B0_xy_path=xy_path,
                    B0_z_drift=z_drift, B0_z_path=z_path,
                    B0_v_mean=v_mean, B0_v_max=v_max,
                    GPS_xy_chord=g_xy_chord, GPS_xy_path=g_xy_path,
                    GPS_z_drift=g_z_drift,
                    ba_start=ba_start, ba_end=ba_end, d_ba=d_ba,
                    bg_start=bg_start, bg_end=bg_end, d_bg=d_bg)

    ph_gh = phase_stats('ground_hover',   *phases['ground_hover'])
    ph_vc = phase_stats('vertical_climb', *phases['vertical_climb'])

    print(f'  init  ba = {log.get("init_ba")}   bg = {log.get("init_bg")}')
    print(f'  init  disparity={log.get("init_disparity")}   '
          f'cam_imu_timeoffset={log.get("cam_imu_timeoffset")}')
    print(f'  GPS_takeoff (XY+2m)      = {t_takeoff}s')
    print(f'  GPS_first_alt_rise (+2m) = {t_alt_rise}s  ground_alt={gps_ground_alt:.2f}m')
    for ph in (ph_gh, ph_vc):
        if 'B0_xy_chord' not in ph:
            print(f'  [{ph["label"]:<14}] [{ph["span"][0]:>5.1f},{ph["span"][1]:>5.1f}]s  '
                  f'n={ph["n_b0"]} (too short)')
            continue
        print(f'  [{ph["label"]:<14}] [{ph["span"][0]:>5.1f},{ph["span"][1]:>5.1f}]s  '
              f'n={ph["n_b0"]}')
        print(f'     B0  : xy_chord={ph["B0_xy_chord"]:>5.2f}m  xy_path={ph["B0_xy_path"]:>6.2f}m  '
              f'z_drift={ph["B0_z_drift"]:>+6.2f}m  |v|_mean={ph["B0_v_mean"]:>5.3f}  '
              f'max={ph["B0_v_max"]:>5.3f} m/s')
        print(f'     GPS : xy_chord={ph["GPS_xy_chord"]:>5.2f}m  xy_path={ph["GPS_xy_path"]:>6.2f}m  '
              f'z_drift={ph["GPS_z_drift"]:>+6.2f}m')
        print(f'     ba  : start=({ph["ba_start"][0]:+.5f},{ph["ba_start"][1]:+.5f},{ph["ba_start"][2]:+.5f})  '
              f'end=({ph["ba_end"][0]:+.5f},{ph["ba_end"][1]:+.5f},{ph["ba_end"][2]:+.5f})  '
              f'd=({ph["d_ba"][0]:+.5f},{ph["d_ba"][1]:+.5f},{ph["d_ba"][2]:+.5f})')

    # Variables expected downstream
    if 'ba_end' in ph_vc and isinstance(ph_vc.get('ba_end'), list):
        ba_at_takeoff = ph_vc['ba_end']
        bg_at_takeoff = ph_vc['bg_end']
    elif 'ba_end' in ph_gh and isinstance(ph_gh.get('ba_end'), list):
        ba_at_takeoff = ph_gh['ba_end']
        bg_at_takeoff = ph_gh['bg_end']
    else:
        ba_at_takeoff = bg_at_takeoff = [float('nan')] * 3
    ba_at_init = ph_gh.get('ba_start', [float('nan')] * 3)
    bg_at_init = ph_gh.get('bg_start', [float('nan')] * 3)
    # Backward-compat aliases used later for plots / multi-window section
    hover_lo, hover_hi = phases['ground_hover']
    hov_b0_xy = b0_xyz[make_mask(b0_rel, *phases['ground_hover']), :2]
    hov_b0_z  = b0_xyz[make_mask(b0_rel, *phases['ground_hover']), 2]
    hov_t     = b0_t  [make_mask(b0_rel, *phases['ground_hover'])]
    m_g_hov   = make_mask(g_rel, *phases['ground_hover'])
    m_bi_hov  = make_mask(bi_rel, *phases['ground_hover'])
    hover_xy_drift_chord = ph_gh.get('B0_xy_chord', float('nan'))
    hover_xy_path        = ph_gh.get('B0_xy_path',  float('nan'))
    hover_z_drift        = ph_gh.get('B0_z_drift',  float('nan'))
    hover_v_mean         = ph_gh.get('B0_v_mean',   float('nan'))
    hover_v_max          = ph_gh.get('B0_v_max',    float('nan'))
    gps_hov_xy_drift     = ph_gh.get('GPS_xy_chord', float('nan'))
    gps_hov_z_drift      = ph_gh.get('GPS_z_drift',  float('nan'))
    d_ba_hover           = ph_gh.get('d_ba', [float('nan')] * 3)
    d_bg_hover           = ph_gh.get('d_bg', [float('nan')] * 3)
    hover_n              = ph_gh.get('n_b0', 0)

    # (per-phase block above already printed ba/bg/d_ba/d_bg)

    # ---- B. SLOW VERTICAL TAKEOFF (first 30 s post takeoff) ---------------
    if t_takeoff is not None:
        tk_lo = t_takeoff; tk_hi = min(t_takeoff + 30.0, t_hi - t0 - 1.0)
        m_b0_tk = (b0_t - t0 >= tk_lo) & (b0_t - t0 <= tk_hi)
        m_bi_tk = (b_t  - t0 >= tk_lo) & (b_t  - t0 <= tk_hi)
        m_g_tk  = (g_rel >= tk_lo) & (g_rel <= tk_hi)
        tk_b0_xy = b0_xyz[m_b0_tk, :2]; tk_b0_z = b0_xyz[m_b0_tk, 2]
        tk_g_xy = g_xy[m_g_tk]; tk_g_alt = g_alt[m_g_tk]
        # Verticality ratio: |delta_z| / max(|delta_xy|, 1e-3)
        gps_dz = float(np.max(tk_g_alt) - gps_ground_alt)
        gps_dxy = path_len_xy(tk_g_xy)
        gps_vertical_ratio = gps_dz / max(gps_dxy, 1e-3)
        b0_dz = float(np.max(tk_b0_z) - tk_b0_z[0])
        b0_dxy = path_len_xy(tk_b0_xy)
        b0_vertical_ratio = b0_dz / max(b0_dxy, 1e-3)
        # Mean horizontal speed during takeoff
        if m_bi_tk.sum() >= 5:
            tk_speed_xy = np.linalg.norm(b_v[m_bi_tk, :2], axis=1)
            tk_speed_z = np.abs(b_v[m_bi_tk, 2])
            tk_mean_xy_speed = float(tk_speed_xy.mean())
            tk_mean_z_speed = float(tk_speed_z.mean())
        else:
            tk_mean_xy_speed = tk_mean_z_speed = float('nan')
        print(f'\n  TAKEOFF window  = [{tk_lo:.1f}, {tk_hi:.1f}]s  '
              f'(first 30s of motion)')
        print(f'  GPS  dz={gps_dz:.2f}m  dxy={gps_dxy:.2f}m  '
              f'vertical_ratio dz/dxy = {gps_vertical_ratio:.3f}  '
              f'(>1 = mostly vertical)')
        print(f'  B0   dz={b0_dz:.2f}m  dxy={b0_dxy:.2f}m  '
              f'vertical_ratio = {b0_vertical_ratio:.3f}')
        print(f'  B0   mean speeds: |v_xy|={tk_mean_xy_speed:.3f} m/s   '
              f'|v_z|={tk_mean_z_speed:.3f} m/s')
    else:
        gps_dz = gps_dxy = gps_vertical_ratio = float('nan')
        b0_dz = b0_dxy = b0_vertical_ratio = float('nan')
        tk_mean_xy_speed = tk_mean_z_speed = float('nan')
        tk_lo = tk_hi = float('nan')

    # ---- D. MULTI-WINDOW SCALE ------------------------------------------
    # 1) hover window
    if hover_n >= 5 and m_g_hov.sum() >= 5:
        win_hov_b0 = path_len_xy(hov_b0_xy)
        win_hov_gps = path_len_xy(g_xy[m_g_hov])
    else:
        win_hov_b0 = win_hov_gps = float('nan')

    # 2) takeoff first 30s
    if t_takeoff is not None:
        win_tk_b0 = b0_dxy; win_tk_gps = gps_dxy
    else:
        win_tk_b0 = win_tk_gps = float('nan')

    # 3) first clean horizontal edge (post-takeoff, 80s of motion)
    if t_takeoff is not None:
        edge_lo = t_takeoff; edge_hi = min(t_takeoff + 80.0, t_hi - t0 - 1.0)
        m_b0_eg = (b0_t - t0 >= edge_lo) & (b0_t - t0 <= edge_hi)
        m_g_eg  = (g_rel >= edge_lo) & (g_rel <= edge_hi)
        win_edge_b0 = path_len_xy(b0_xyz[m_b0_eg, :2]) if m_b0_eg.sum() else float('nan')
        win_edge_gps = path_len_xy(g_xy[m_g_eg]) if m_g_eg.sum() else float('nan')
    else:
        edge_lo = edge_hi = float('nan')
        win_edge_b0 = win_edge_gps = float('nan')

    # 4) full GPS-covered motion (from takeoff to end of overlap)
    if t_takeoff is not None:
        full_lo = t_takeoff; full_hi = t_hi - t0
        m_b0_fl = (b0_t - t0 >= full_lo) & (b0_t - t0 <= full_hi)
        m_g_fl  = (g_rel >= full_lo) & (g_rel <= full_hi)
        win_full_b0 = path_len_xy(b0_xyz[m_b0_fl, :2]) if m_b0_fl.sum() else float('nan')
        win_full_gps = path_len_xy(g_xy[m_g_fl]) if m_g_fl.sum() else float('nan')
    else:
        full_lo = full_hi = float('nan')
        win_full_b0 = win_full_gps = float('nan')

    # 5) fly4-specific: late-half motion only (capture SW segment vs early)
    win_late_b0 = win_late_gps = float('nan')
    late_lo = late_hi = float('nan')
    if t_takeoff is not None:
        late_lo = t_takeoff + (full_hi - t_takeoff) * 0.5
        late_hi = full_hi
        m_b0_lt = (b0_t - t0 >= late_lo) & (b0_t - t0 <= late_hi)
        m_g_lt  = (g_rel >= late_lo) & (g_rel <= late_hi)
        if m_b0_lt.sum() and m_g_lt.sum():
            win_late_b0 = path_len_xy(b0_xyz[m_b0_lt, :2])
            win_late_gps = path_len_xy(g_xy[m_g_lt])
    # Early half also
    win_early_b0 = win_early_gps = float('nan')
    early_lo = early_hi = float('nan')
    if t_takeoff is not None:
        early_lo = t_takeoff
        early_hi = t_takeoff + (full_hi - t_takeoff) * 0.5
        m_b0_er = (b0_t - t0 >= early_lo) & (b0_t - t0 <= early_hi)
        m_g_er  = (g_rel >= early_lo) & (g_rel <= early_hi)
        if m_b0_er.sum() and m_g_er.sum():
            win_early_b0 = path_len_xy(b0_xyz[m_b0_er, :2])
            win_early_gps = path_len_xy(g_xy[m_g_er])

    def ratio(b, g): return float(b / g) if g > 0 else float('nan')
    r_hov = ratio(win_hov_b0, win_hov_gps)
    r_tk  = ratio(win_tk_b0,  win_tk_gps)
    r_edge = ratio(win_edge_b0, win_edge_gps)
    r_full = ratio(win_full_b0, win_full_gps)
    r_late = ratio(win_late_b0, win_late_gps)
    r_early = ratio(win_early_b0, win_early_gps)

    print(f'\n  MULTI-WINDOW SCALE (B0_len / GPS_len)')
    print(f'    1. hover_only         [{hover_lo:>5.1f},{hover_hi:>5.1f}]s '
          f'B0={win_hov_b0:>6.2f}  GPS={win_hov_gps:>6.2f}  ratio={r_hov}  '
          f'(low GPS => drift only, not scale)')
    print(f'    2. takeoff_first_30s  [{tk_lo:>5.1f},{tk_hi:>5.1f}]s '
          f'B0={win_tk_b0:>6.2f}  GPS={win_tk_gps:>6.2f}  ratio={r_tk:.3f}')
    print(f'    3. first_edge_80s     [{edge_lo:>5.1f},{edge_hi:>5.1f}]s '
          f'B0={win_edge_b0:>6.2f}  GPS={win_edge_gps:>6.2f}  ratio={r_edge:.3f}')
    print(f'    4. full_GPS_motion    [{full_lo:>5.1f},{full_hi:>5.1f}]s '
          f'B0={win_full_b0:>6.2f}  GPS={win_full_gps:>6.2f}  ratio={r_full:.3f}')
    print(f'    5. early_half         [{early_lo:>5.1f},{early_hi:>5.1f}]s '
          f'B0={win_early_b0:>6.2f}  GPS={win_early_gps:>6.2f}  ratio={r_early:.3f}')
    print(f'    6. late_half          [{late_lo:>5.1f},{late_hi:>5.1f}]s '
          f'B0={win_late_b0:>6.2f}  GPS={win_late_gps:>6.2f}  ratio={r_late:.3f}')

    summary = dict(
        flight=fly,
        init_mode=log.get('init_mode'),
        init_disparity=log.get('init_disparity'),
        init_bg=log.get('init_bg'),
        init_ba=log.get('init_ba'),
        cam_imu_timeoffset=log.get('cam_imu_timeoffset'),
        t_init_abs=float(b0[0, 0]),
        t0_overlap_abs=float(t0),
        t_takeoff_rel=t_takeoff,
        t_first_alt_rise_rel=t_alt_rise,
        hover_dur_s=float(hover_hi - hover_lo),
        hover_B0_xy_chord_drift_m=hover_xy_drift_chord,
        hover_B0_xy_path_drift_m=hover_xy_path,
        hover_B0_z_drift_m=hover_z_drift,
        hover_B0_v_mean_mps=hover_v_mean,
        hover_B0_v_max_mps=hover_v_max,
        hover_GPS_xy_drift_m=gps_hov_xy_drift,
        hover_GPS_z_drift_m=gps_hov_z_drift,
        ba_init=ba_at_init, bg_init=bg_at_init,
        ba_takeoff=ba_at_takeoff, bg_takeoff=bg_at_takeoff,
        d_ba_hover=d_ba_hover, d_bg_hover=d_bg_hover,
        takeoff_GPS_dz_m=gps_dz, takeoff_GPS_dxy_m=gps_dxy,
        takeoff_GPS_vertical_ratio=gps_vertical_ratio,
        takeoff_B0_dz_m=b0_dz,  takeoff_B0_dxy_m=b0_dxy,
        takeoff_B0_vertical_ratio=b0_vertical_ratio,
        takeoff_B0_mean_xy_speed=tk_mean_xy_speed,
        takeoff_B0_mean_z_speed=tk_mean_z_speed,
        windows=dict(
            hover_only=dict(span=[hover_lo, hover_hi],
                            B0=win_hov_b0, GPS=win_hov_gps, ratio=r_hov),
            takeoff_first_30s=dict(span=[tk_lo, tk_hi],
                                    B0=win_tk_b0, GPS=win_tk_gps, ratio=r_tk),
            first_edge_80s=dict(span=[edge_lo, edge_hi],
                                B0=win_edge_b0, GPS=win_edge_gps, ratio=r_edge),
            full_GPS_motion=dict(span=[full_lo, full_hi],
                                 B0=win_full_b0, GPS=win_full_gps, ratio=r_full),
            early_half=dict(span=[early_lo, early_hi],
                            B0=win_early_b0, GPS=win_early_gps, ratio=r_early),
            late_half=dict(span=[late_lo, late_hi],
                           B0=win_late_b0, GPS=win_late_gps, ratio=r_late),
        ),
    )
    results.append(summary)

    # ---------------- plots ----------------
    if not _HAVE_MPL: continue
    pfx = os.path.join(OUT_DIR, f'fly{fly}')
    os.makedirs(pfx, exist_ok=True)

    # plot 1: hover XY drift (B0 in arbitrary VIO frame; do not yaw-align hover
    # because GPS chord is essentially 0)
    fig, ax = plt.subplots(1, 1, figsize=(7, 7))
    ax.plot(hov_b0_xy[:, 0], hov_b0_xy[:, 1], 'b-', lw=1.2,
            label=f'B0 hover XY  (path={hover_xy_path:.2f}m, '
                  f'chord_drift={hover_xy_drift_chord:.2f}m)')
    ax.scatter([hov_b0_xy[0, 0]], [hov_b0_xy[0, 1]], s=80, c='g', marker='o',
               label='hover start')
    ax.scatter([hov_b0_xy[-1, 0]], [hov_b0_xy[-1, 1]], s=80, c='r', marker='x',
               label='hover end (~takeoff)')
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=0.3)
    ax.set_title(f'fly{fly} hover drift in B0 (pure mono, VIO frame)\n'
                 f'hover dur={hover_hi-hover_lo:.1f}s | GPS chord drift '
                 f'{gps_hov_xy_drift:.2f}m (reality)')
    ax.set_xlabel('X_VIO (m)'); ax.set_ylabel('Y_VIO (m)')
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout(); fig.savefig(f'{pfx}/A_hover_xy.png', dpi=120); plt.close(fig)

    # plot 2: hover Z drift
    fig, ax = plt.subplots(1, 1, figsize=(11, 4))
    ax.plot(hov_t - t0, hov_b0_z, 'b-', lw=1.2,
            label=f'B0 z (drift = {hover_z_drift:+.2f}m)')
    if m_g_hov.sum():
        ax.plot(g_rel[m_g_hov], g_alt[m_g_hov] - gps_ground_alt, 'k:', lw=1.4,
                label=f'GPS alt - ground (drift = {gps_hov_z_drift:+.2f}m)')
    ax.set_xlabel('t_rel (s)'); ax.set_ylabel('z (m)')
    ax.set_title(f'fly{fly} hover Z: B0 in VIO frame vs GPS (anchored to ground level)')
    ax.legend(loc='best', fontsize=9); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f'{pfx}/A_hover_z.png', dpi=120); plt.close(fig)

    # plot 3: bias_accel over time (full overlap + hover/takeoff markers)
    if b_t.size:
        fig, axs = plt.subplots(2, 1, figsize=(13, 6), sharex=True)
        axs[0].plot(b_t - t0, b_ba[:, 0], 'r-', lw=1.0, label='ba_x')
        axs[0].plot(b_t - t0, b_ba[:, 1], 'g-', lw=1.0, label='ba_y')
        axs[0].plot(b_t - t0, b_ba[:, 2], 'b-', lw=1.2, label='ba_z')
        if t_takeoff is not None:
            for ax in axs:
                ax.axvline(t_takeoff, color='orange', lw=1.4, ls='--',
                           label=f'GPS takeoff @ {t_takeoff:.1f}s')
        axs[0].axhline(0, color='k', lw=0.6, alpha=0.4)
        axs[0].set_ylabel('bias_a (m/s²)'); axs[0].grid(alpha=0.3)
        axs[0].legend(loc='best', fontsize=8)
        axs[0].set_title(f'fly{fly} bias trajectories (B0 pure mono)  |  init_ba={log.get("init_ba")}')
        axs[1].plot(b_t - t0, b_bg[:, 0], 'r-', lw=1.0, label='bg_x')
        axs[1].plot(b_t - t0, b_bg[:, 1], 'g-', lw=1.0, label='bg_y')
        axs[1].plot(b_t - t0, b_bg[:, 2], 'b-', lw=1.2, label='bg_z')
        axs[1].axhline(0, color='k', lw=0.6, alpha=0.4)
        axs[1].set_xlabel('t_rel (s)'); axs[1].set_ylabel('bias_g (rad/s)')
        axs[1].grid(alpha=0.3); axs[1].legend(loc='best', fontsize=8)
        fig.tight_layout(); fig.savefig(f'{pfx}/A_bias_trajectories.png', dpi=120)
        plt.close(fig)

    # plot 4: takeoff vertical profile
    if t_takeoff is not None:
        m_b0_show = (b0_t - t0 >= 0) & (b0_t - t0 <= min(t_takeoff + 60, t_hi - t0))
        m_g_show  = (g_rel  >= 0) & (g_rel  <= min(t_takeoff + 60, t_hi - t0))
        fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        axs[0].plot(g_rel[m_g_show], g_alt[m_g_show] - gps_ground_alt, 'k:', lw=1.4,
                    label='GPS alt - ground')
        axs[0].plot(b0_t[m_b0_show] - t0, b0_xyz[m_b0_show, 2], 'b-', lw=1.2,
                    label='B0 z')
        axs[0].axvline(t_takeoff, color='orange', lw=1.4, ls='--',
                       label=f'takeoff @ {t_takeoff:.1f}s')
        if t_alt_rise:
            axs[0].axvline(t_alt_rise, color='red', lw=1.0, ls=':',
                           label=f'GPS alt+2m @ {t_alt_rise:.1f}s')
        axs[0].set_ylabel('z (m)  (GPS=alt-ground)'); axs[0].grid(alpha=0.3)
        axs[0].legend(loc='best', fontsize=8)
        axs[0].set_title(f'fly{fly} altitude pre/post takeoff (first 60s after takeoff)')

        if m_bi_tk.sum() >= 5:
            mvel = (b_t - t0 >= 0) & (b_t - t0 <= min(t_takeoff + 60, t_hi - t0))
            axs[1].plot(b_t[mvel] - t0, np.linalg.norm(b_v[mvel, :2], axis=1),
                        'm-', lw=1.0, label='|v_xy| (B0)')
            axs[1].plot(b_t[mvel] - t0, np.abs(b_v[mvel, 2]),
                        'b-', lw=1.0, label='|v_z| (B0)')
            axs[1].axvline(t_takeoff, color='orange', lw=1.4, ls='--')
            axs[1].set_xlabel('t_rel (s)'); axs[1].set_ylabel('speed (m/s)')
            axs[1].grid(alpha=0.3); axs[1].legend(loc='best', fontsize=8)
        fig.tight_layout(); fig.savefig(f'{pfx}/B_takeoff_z_speed.png', dpi=120)
        plt.close(fig)

    # plot 5: full XY trajectory (yaw-aligned) with hover, takeoff, edge, early/late windows
    if t_takeoff is not None:
        # yaw align over the FULL motion window
        m_b0_yaw = (b0_t - t0 >= full_lo) & (b0_t - t0 <= full_hi)
        gps_at_b0 = np.column_stack([
            np.interp(b0_t[m_b0_yaw], gps_t, gps_xy[:, 0], left=np.nan, right=np.nan),
            np.interp(b0_t[m_b0_yaw], gps_t, gps_xy[:, 1], left=np.nan, right=np.nan),
        ])
        theta, b0_xy_y = yaw_align_xy(b0_xyz[m_b0_yaw, :2], gps_at_b0)
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        ax.plot(g_xy[:, 0], g_xy[:, 1], 'k:', lw=1.4,
                label=f'GPS full track ({len(g_xy)} pts)')
        ax.plot(b0_xy_y[:, 0], b0_xy_y[:, 1], 'b-', lw=1.0,
                label=f'B0 motion (yaw={np.rad2deg(theta):+.1f}°, '
                      f'ratio_full={r_full:.3f}, ratio_early={r_early:.3f}, '
                      f'ratio_late={r_late:.3f})')
        # mark divisions
        m_b0_early = (b0_t[m_b0_yaw] - t0 >= early_lo) & (b0_t[m_b0_yaw] - t0 <= early_hi)
        m_b0_late  = (b0_t[m_b0_yaw] - t0 >= late_lo)  & (b0_t[m_b0_yaw] - t0 <= late_hi)
        ax.plot(b0_xy_y[m_b0_early, 0], b0_xy_y[m_b0_early, 1], 'g-', lw=2.0,
                alpha=0.6, label='early half (B0)')
        ax.plot(b0_xy_y[m_b0_late, 0], b0_xy_y[m_b0_late, 1], 'r-', lw=2.0,
                alpha=0.6, label='late half (B0)')
        ax.scatter([g_xy[0, 0]], [g_xy[0, 1]], s=80, c='k', marker='o', label='GPS t0')
        ax.scatter([g_xy[-1, 0]], [g_xy[-1, 1]], s=80, c='k', marker='x', label='GPS end')
        ax.set_xlabel('E (m)'); ax.set_ylabel('N (m)')
        ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=0.3)
        ax.set_title(f'fly{fly} full XY  |  B0 yaw-aligned (NO scale align) vs GPS')
        ax.legend(loc='best', fontsize=8)
        fig.tight_layout(); fig.savefig(f'{pfx}/D_xy_full_yaw_aligned.png', dpi=120)
        plt.close(fig)

# ---------------- E. correlation table ----------------
print(f'\n{"="*120}')
print(f'  INIT / HOVER / SCALE CORRELATION TABLE')
print(f'{"="*120}')
hdr = (f'{"fly":>3s} {"takeoff":>7s} {"hov_dur":>7s} '
       f'{"hov_xy_drft":>11s} {"hov_z_drft":>10s} {"hov_|v|":>7s} '
       f'{"ba_init_z":>9s} {"ba_takeoff_z":>12s} {"d_ba_z_hov":>10s} '
       f'{"tk_dz/dxy":>10s} {"r_first_edge":>12s} {"r_full":>8s} {"r_late":>8s}')
print(hdr); print('-' * len(hdr))
for r in results:
    fly = r['flight']; tk = r['t_takeoff_rel']
    print(f'{fly:>3d} {tk if tk is None else f"{tk:>5.1f}s":>7s} '
          f'{r["hover_dur_s"]:>6.1f}s '
          f'{r["hover_B0_xy_chord_drift_m"]:>10.2f}m '
          f'{r["hover_B0_z_drift_m"]:>+9.2f}m '
          f'{r["hover_B0_v_mean_mps"]:>7.3f} '
          f'{r["ba_init"][2]:>+9.5f} {r["ba_takeoff"][2]:>+12.5f} '
          f'{r["d_ba_hover"][2]:>+10.5f} '
          f'{r["takeoff_GPS_vertical_ratio"]:>10.3f} '
          f'{r["windows"]["first_edge_80s"]["ratio"]:>12.3f} '
          f'{r["windows"]["full_GPS_motion"]["ratio"]:>8.3f} '
          f'{r["windows"]["late_half"]["ratio"]:>8.3f}')

with open(os.path.join(OUT_DIR, 'summary.json'), 'w') as f:
    json.dump(results, f, indent=2, default=str)
print(f'\n[saved] {OUT_DIR}/summary.json')
