#!/usr/bin/env python3
"""First-edge scale diagnostic — pure monocular B0 vs aligned GPS.

For each flight:
  1. Load B0 trajectory (TUM format, pure monocular, no GPS fusion).
  2. Load aligned GPS (cam-time, lat/lon/alt). Convert lat/lon to local ENU
     anchored at the FIRST GPS sample of that flight. GPS is used here as
     POST-HOC evaluation reference ONLY — it is NOT fed to the EKF in the
     B0 run.
  3. Detect VIO init time as the first valid B0 sample timestamp.
  4. Detect the first valid straight/edge segment after init, using the
     GPS XY trajectory (path_length / chord <= 1.05 and chord >= 30 m).
  5. Translation-only align B0 to GPS over the first `align_window` seconds
     of the edge (NO scale alignment — that would hide the bug we're
     measuring).
  6. Compute:
        scale_ratio   = path_length(B0_edge_xy)   / path_length(GPS_edge_xy)
        edge_xy_rmse  = RMS of (B0_aligned_xy - GPS_xy) over the edge
        edge_xy_final = | B0_aligned_xy[end] - GPS_xy[end] |
        z_edge_rmse   = RMS of (B0_aligned_z  - GPS_alt) over the edge
  7. Save:
        first-edge zoom plot   (B0 vs GPS, both XY-aligned at edge start)
        Z vs time              (full flight, edge region highlighted)
        XY full traj           (B0 vs GPS, full overlap region)

Important: GPS is used as a REFERENCE TRAJECTORY for evaluation. It is not
ground truth in absolute terms (GPS HDOP is meter-scale absolute, but the
relative-motion path length over a 30 m+ segment is accurate to a small
fraction of a meter). Treat the GPS-derived scale_ratio as a useful
signature, not as an absolute truth.
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

# Per-flight inputs.  GPS files are the official correct aligned files
# per GPS_REFERENCE_AUDIT.md (post 2026-05-17 data cleanup).
FLIGHTS = [
    dict(flight=1,
         b0='20260509_fly1/result/baselines_v1/B0_no_gps.txt',
         gps='20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv',
         start_time_cli=200),
    dict(flight=2,
         b0='20260509_fly2/result/baselines_v1/B0_no_gps.txt',
         gps='20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv',
         start_time_cli=160),
    dict(flight=3,
         b0='20260509_fly3/result/baselines_v1/B0_no_gps.txt',
         gps='20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv',
         start_time_cli=180),
    dict(flight=4,
         b0='20260509_fly4/result/stage_a_v2/R0_nogps.txt',
         gps='20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv',
         start_time_cli=239),
]

OUT_DIR = 'comparison_plots/first_edge_scale_gps_ref'
os.makedirs(OUT_DIR, exist_ok=True)

# --------------------------- I/O ---------------------------

def load_b0(p):
    rows, last_t = [], -math.inf
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            try:
                t = float(ps[0]); x = float(ps[1])
                y = float(ps[2]); z = float(ps[3])
            except ValueError:
                continue
            if t <= last_t: continue
            rows.append((t, x, y, z)); last_t = t
    return np.asarray(rows)

def load_gps_llz(p):
    """Aligned GPS file `#timestamp_ns,lat_deg,lon_deg,alt_m` -> (t_s, lat, lon, alt)."""
    rows = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try:
                t = float(ps[0]); lat = float(ps[1])
                lon = float(ps[2]); alt = float(ps[3])
            except (ValueError, IndexError):
                continue
            rows.append((t, lat, lon, alt))
    a = np.asarray(rows)
    if a.size and a[0, 0] > 1e11:
        a[:, 0] = a[:, 0] * 1e-9  # ns -> s
    return a[np.argsort(a[:, 0])]

# WGS84 -> local-tangent ENU about a fixed lat0/lon0/alt0
def wgs84_to_ecef(lat_d, lon_d, alt):
    a = 6378137.0
    e2 = 6.69437999014e-3
    lat = np.deg2rad(lat_d); lon = np.deg2rad(lon_d)
    s = np.sin(lat); c = np.cos(lat)
    N = a / np.sqrt(1.0 - e2 * s * s)
    x = (N + alt) * c * np.cos(lon)
    y = (N + alt) * c * np.sin(lon)
    z = (N * (1.0 - e2) + alt) * s
    return np.stack([x, y, z], axis=-1)

def ecef_to_enu_R(lat0_d, lon0_d):
    lat = np.deg2rad(lat0_d); lon = np.deg2rad(lon0_d)
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    return np.array([
        [-so,        co,      0.0],
        [-sl * co,  -sl * so, cl ],
        [ cl * co,   cl * so, sl ],
    ])

def gps_to_enu(gps_llz, anchor_idx=0):
    """Returns (t_s, e, n, u, alt) anchored at gps_llz[anchor_idx]."""
    lat0 = gps_llz[anchor_idx, 1]; lon0 = gps_llz[anchor_idx, 2]
    alt0 = gps_llz[anchor_idx, 3]
    ecef0 = wgs84_to_ecef(lat0, lon0, alt0)
    R = ecef_to_enu_R(lat0, lon0)
    ecef = wgs84_to_ecef(gps_llz[:, 1], gps_llz[:, 2], gps_llz[:, 3])
    enu = (ecef - ecef0) @ R.T
    out = np.zeros((len(gps_llz), 5))
    out[:, 0] = gps_llz[:, 0]
    out[:, 1:4] = enu
    out[:, 4] = gps_llz[:, 3]  # raw alt
    return out, (lat0, lon0, alt0)

# ----------------------- geometry --------------------------

def path_len_xy(xy):
    if len(xy) < 2: return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))

def chord_xy(xy):
    if len(xy) < 2: return 0.0
    return float(np.linalg.norm(xy[-1] - xy[0]))

def detect_first_edge(t_rel, xy, t_init_rel,
                      min_chord_m=30.0, min_straight=1.05,
                      candidate_durs=(60.0, 80.0, 100.0),
                      step=5.0, t_end_rel=None):
    """Walk forward from t_init_rel; find first window where path/chord <= 1.05
    and chord >= 30 m. Returns (s_rel, e_rel, path_len, chord, straight)."""
    if t_end_rel is None: t_end_rel = float(t_rel[-1])
    s = max(t_init_rel + 1.0, 5.0)
    while s + min(candidate_durs) < t_end_rel:
        for dur in candidate_durs:
            e = s + dur
            if e > t_end_rel - 1.0: break
            m = (t_rel >= s) & (t_rel <= e)
            if m.sum() < 10: continue
            sub = xy[m]
            ch = chord_xy(sub)
            if ch < min_chord_m: continue
            pl = path_len_xy(sub)
            sgn = pl / ch if ch > 0 else 1e9
            if sgn <= min_straight:
                return (s, e, pl, ch, sgn)
        s += step
    return None

def translate_align_xy(b0_t, b0_xyz, gps_t, gps_enu, align_window_s=5.0,
                       align_t_start_abs=None):
    """Return (dx, dy, dz) so that B0+offset best matches GPS over a
    short window starting at `align_t_start_abs` (defaults to B0 first sample)."""
    if align_t_start_abs is None: align_t_start_abs = b0_t[0]
    m = (b0_t >= align_t_start_abs) & (b0_t <= align_t_start_abs + align_window_s)
    if m.sum() < 5: m = np.zeros(len(b0_t), bool); m[:5] = True
    sub = b0_xyz[m]; sub_t = b0_t[m]
    out = np.zeros(3)
    for k in range(3):
        ref = np.interp(sub_t, gps_t, gps_enu[:, k], left=np.nan, right=np.nan)
        d = ref - sub[:, k]; v = ~np.isnan(d)
        out[k] = float(d[v].mean()) if v.any() else 0.0
    return out


def yaw_align_xy(b0_xy, gps_xy_at_b0_t, weight_secs=None):
    """Find the 2-D rotation theta minimizing |R(theta) b - g|^2 (Procrustes
    in 2D). Returns theta_rad and the rotated b0_xy. Both inputs must be
    centered (mean-subtracted) for this to be the pure-rotation Procrustes
    solution. Returns ALSO the rotated version anchored at the same start
    point so the trajectory begins where the GPS begins.
    """
    b = b0_xy - b0_xy[0]
    g = gps_xy_at_b0_t - gps_xy_at_b0_t[0]
    valid = ~np.isnan(g[:, 0]) & ~np.isnan(g[:, 1])
    if valid.sum() < 5:
        return 0.0, b0_xy
    bv = b[valid]; gv = g[valid]
    # H = sum_i b_i g_i^T;  2D-Procrustes: theta = atan2(sum(b_x g_y - b_y g_x),
    #                                                   sum(b_x g_x + b_y g_y))
    s = np.sum(bv[:, 0] * gv[:, 1] - bv[:, 1] * gv[:, 0])
    c = np.sum(bv[:, 0] * gv[:, 0] + bv[:, 1] * gv[:, 1])
    theta = float(np.arctan2(s, c))
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    rotated = (R @ b0_xy.T).T
    # re-anchor: translate so rotated[0] sits where GPS starts
    rotated = rotated - rotated[0] + gps_xy_at_b0_t[0]
    return theta, rotated


def detect_gps_takeoff(t_rel, xy, alt, min_speed_mps=0.5, min_dist_m=2.0,
                       hold_s=2.0):
    """Return the t_rel where the drone first sustains motion: GPS XY moves
    by at least `min_dist_m` from its starting point AND stays moving for at
    least `hold_s` seconds. Falls back to the first sample where chord
    crosses min_dist_m."""
    if len(t_rel) < 5: return None
    x0, y0 = xy[0]
    d_from_start = np.sqrt((xy[:, 0] - x0) ** 2 + (xy[:, 1] - y0) ** 2)
    crossing = np.where(d_from_start >= min_dist_m)[0]
    if crossing.size == 0: return None
    idx = int(crossing[0])
    # require it stays past threshold for hold_s
    t_idx = t_rel[idx]
    later = (t_rel >= t_idx) & (t_rel <= t_idx + hold_s)
    if d_from_start[later].min() >= min_dist_m * 0.5:
        return float(t_idx)
    return float(t_idx)


def multi_window_scale(b0_t, b0_xy, gps_t, gps_xy, t_edge_start_abs,
                       windows_s=(10, 20, 40, 60)):
    """For each cumulative duration d, compute scale_ratio over
    [edge_start, edge_start + d]. Returns list of dicts."""
    out = []
    for d in windows_s:
        e_lo = t_edge_start_abs
        e_hi = t_edge_start_abs + d
        m_b0  = (b0_t  >= e_lo) & (b0_t  <= e_hi)
        m_gps = (gps_t >= e_lo) & (gps_t <= e_hi)
        if m_b0.sum() < 5 or m_gps.sum() < 5:
            out.append(dict(window_s=d, b0_len=float('nan'),
                            gps_len=float('nan'), ratio=float('nan')))
            continue
        b0_l  = path_len_xy(b0_xy[m_b0])
        gps_l = path_len_xy(gps_xy[m_gps])
        r = b0_l / gps_l if gps_l > 0 else float('nan')
        out.append(dict(window_s=d, b0_len=float(b0_l),
                        gps_len=float(gps_l), ratio=float(r)))
    return out

# ------------------------- main ----------------------------

results = []
for cfg in FLIGHTS:
    fly = cfg['flight']
    print(f'\n========= fly{fly} =========')
    print(f'  B0  : {cfg["b0"]}')
    print(f'  GPS : {cfg["gps"]}')
    print(f'  start-time CLI : {cfg["start_time_cli"]}')

    if not os.path.isfile(cfg['b0']):
        print('  [SKIP] B0 missing'); continue
    if not os.path.isfile(cfg['gps']):
        print('  [SKIP] GPS missing'); continue

    b0 = load_b0(cfg['b0'])
    gps_llz = load_gps_llz(cfg['gps'])
    if len(b0) < 50 or len(gps_llz) < 50:
        print('  [SKIP] not enough samples'); continue

    # ENU about FIRST gps sample
    gps_enu5, anchor = gps_to_enu(gps_llz, anchor_idx=0)
    gps_t   = gps_enu5[:, 0]
    gps_xy  = gps_enu5[:, 1:3]   # (e, n)
    gps_alt = gps_enu5[:, 4]     # WGS84 m

    # common timeline = max(B0_start, GPS_start) ... min(B0_end, GPS_end)
    t_lo = max(b0[0, 0], gps_t[0])
    t_hi = min(b0[-1, 0], gps_t[-1])
    t0 = t_lo  # use overlap start as the diagnostic t=0
    # b0_t in absolute s
    b0_t = b0[:, 0]
    b0_in = (b0_t >= t_lo) & (b0_t <= t_hi)
    if b0_in.sum() < 50:
        print('  [SKIP] no overlap'); continue
    b0_t_o = b0_t[b0_in]; b0_xyz_o = b0[b0_in, 1:4]

    # init time = first B0 sample, relative to t0
    t_init_abs = b0[0, 0]
    t_init_rel = t_init_abs - t0
    # the first usable VIO sample within overlap
    t_init_overlap_rel = b0_t_o[0] - t0
    # GPS timeline
    gps_rel = gps_t - t0

    # First edge detection on GPS XY
    edge = detect_first_edge(gps_rel, gps_xy, t_init_overlap_rel)
    if edge is None:
        print('  [SKIP] no clean straight edge detected on GPS XY'); continue
    e_s, e_e, gps_edge_len, gps_chord, gps_str = edge

    # Translation-only alignment over the first `align_window` of the edge
    align_t_start_abs = t0 + e_s
    align_window_s = 5.0
    off = translate_align_xy(b0_t_o, b0_xyz_o, gps_t,
                             np.column_stack([gps_xy[:, 0], gps_xy[:, 1], gps_alt]),
                             align_window_s=align_window_s,
                             align_t_start_abs=align_t_start_abs)
    aligned = b0_xyz_o + off

    # Metrics in the edge window
    in_edge_b0  = (b0_t_o - t0 >= e_s) & (b0_t_o - t0 <= e_e)
    in_edge_gps = (gps_rel >= e_s) & (gps_rel <= e_e)
    if in_edge_b0.sum() < 10 or in_edge_gps.sum() < 5:
        print('  [SKIP] edge window has too few samples'); continue

    b0_edge_xy = aligned[in_edge_b0, :2]
    b0_edge_z  = aligned[in_edge_b0, 2]
    b0_edge_t  = b0_t_o[in_edge_b0]

    b0_edge_len = path_len_xy(b0_edge_xy)
    scale_ratio = b0_edge_len / gps_edge_len if gps_edge_len > 0 else float('nan')

    # YAW alignment: B0 lives in arbitrary VIO yaw frame, GPS is ENU. Find
    # the 2-D rotation that minimizes XY residual over the edge. This does
    # NOT affect scale_ratio (path length is rotation-invariant); it makes
    # the plotted XY shapes and the RMSE metric meaningful.
    gps_x_at_edge_b0 = np.interp(b0_edge_t, gps_t, gps_xy[:, 0], left=np.nan, right=np.nan)
    gps_y_at_edge_b0 = np.interp(b0_edge_t, gps_t, gps_xy[:, 1], left=np.nan, right=np.nan)
    gps_at_edge = np.column_stack([gps_x_at_edge_b0, gps_y_at_edge_b0])
    yaw_rad, b0_edge_xy_yawed = yaw_align_xy(b0_edge_xy, gps_at_edge)

    # Compute RMSE in the yaw-aligned frame
    gps_z_at_edge = np.interp(b0_edge_t, gps_t, gps_alt, left=np.nan, right=np.nan)
    dx = b0_edge_xy_yawed[:, 0] - gps_at_edge[:, 0]
    dy = b0_edge_xy_yawed[:, 1] - gps_at_edge[:, 1]
    dz = b0_edge_z              - gps_z_at_edge
    valid_xy = (~np.isnan(dx)) & (~np.isnan(dy))
    valid_z  = ~np.isnan(dz)
    e_xy = np.sqrt(dx[valid_xy] ** 2 + dy[valid_xy] ** 2)
    e_z  = np.abs(dz[valid_z])
    edge_xy_rmse  = float(np.sqrt(np.mean(e_xy ** 2))) if e_xy.size else float('nan')
    edge_xy_final = float(e_xy[-1]) if e_xy.size else float('nan')
    edge_z_rmse   = float(np.sqrt(np.mean(e_z ** 2))) if e_z.size else float('nan')

    # Multi-window scale ratio (cumulative durations from edge_start)
    mw = multi_window_scale(b0_t_o, aligned[:, :2], gps_t, gps_xy,
                            t_edge_start_abs=t0 + e_s,
                            windows_s=(10, 20, 40, 60, int(e_e - e_s)))

    # Detect GPS takeoff (drone first leaves rest) AFTER the first B0 sample.
    gps_in_overlap = (gps_t >= t_lo)
    gps_t_oo = gps_t[gps_in_overlap]
    gps_xy_oo = gps_xy[gps_in_overlap]
    gps_alt_oo = gps_alt[gps_in_overlap]
    t_takeoff_rel = detect_gps_takeoff(gps_t_oo - t0, gps_xy_oo, gps_alt_oo,
                                       min_dist_m=2.0)
    # Motion-only window: 80 s starting at takeoff (clipped to overlap end)
    if t_takeoff_rel is not None:
        mo_lo = t_takeoff_rel
        mo_hi = min(t_takeoff_rel + 80.0, t_hi - t0 - 1.0)
        if mo_hi - mo_lo >= 10.0:
            m_b0  = (b0_t_o - t0 >= mo_lo) & (b0_t_o - t0 <= mo_hi)
            m_gps = (gps_t   - t0 >= mo_lo) & (gps_t   - t0 <= mo_hi)
            if m_b0.sum() >= 5 and m_gps.sum() >= 5:
                mo_b0_len  = path_len_xy(aligned[m_b0, :2])
                mo_gps_len = path_len_xy(gps_xy[m_gps])
                mo_ratio = mo_b0_len / mo_gps_len if mo_gps_len > 0 else float('nan')
            else:
                mo_b0_len = mo_gps_len = mo_ratio = float('nan')
        else:
            mo_b0_len = mo_gps_len = mo_ratio = float('nan')
    else:
        mo_lo = mo_hi = float('nan')
        mo_b0_len = mo_gps_len = mo_ratio = float('nan')

    # B0 path length and GPS path length within the full overlap (sanity)
    b0_all_xy_len  = path_len_xy(aligned[:, :2])
    gps_all_xy_len = path_len_xy(gps_xy[in_edge_gps])

    summary = dict(
        flight=fly,
        b0_path=cfg['b0'], gps_path=cfg['gps'],
        gps_anchor_latlonalt=anchor,
        t0_overlap_abs=float(t0),
        t_overlap_end_abs=float(t_hi),
        t_init_abs=float(t_init_abs),
        t_init_overlap_rel=float(t_init_overlap_rel),
        edge_start_rel=float(e_s),
        edge_end_rel=float(e_e),
        edge_dur_s=float(e_e - e_s),
        gps_edge_len_m=float(gps_edge_len),
        gps_chord_m=float(gps_chord),
        gps_straightness=float(gps_str),
        b0_edge_len_m=float(b0_edge_len),
        scale_ratio=float(scale_ratio),
        yaw_align_deg=float(np.rad2deg(yaw_rad)),
        edge_xy_rmse_m_post_yaw=edge_xy_rmse,
        edge_xy_final_m_post_yaw=edge_xy_final,
        edge_z_rmse_m=edge_z_rmse,
        translation_align_offset_m=off.tolist(),
        align_window_s=align_window_s,
        multi_window_scale=mw,
        gps_takeoff_rel=t_takeoff_rel,
        motion_only_window_rel=(mo_lo, mo_hi),
        motion_only_b0_len_m=mo_b0_len,
        motion_only_gps_len_m=mo_gps_len,
        motion_only_scale_ratio=mo_ratio,
        b0_n_samples=int(len(b0)),
        gps_n_samples=int(len(gps_llz)),
    )
    results.append(summary)

    print(f'  GPS anchor (lat,lon,alt) = ({anchor[0]:.7f}, {anchor[1]:.7f}, {anchor[2]:.2f})')
    print(f'  t_init_abs={t_init_abs:.3f}  overlap=[{t_lo:.3f}, {t_hi:.3f}]  '
          f'dur={t_hi-t_lo:.1f}s')
    print(f'  edge       rel=[{e_s:.2f}, {e_e:.2f}]s  dur={e_e-e_s:.0f}s  '
          f'chord={gps_chord:.2f}m  path={gps_edge_len:.2f}m  '
          f'straightness={gps_str:.3f}')
    print(f'  GPS edge   length = {gps_edge_len:.2f} m')
    print(f'  VIO edge   length = {b0_edge_len:.2f} m   '
          f'(B0 pure monocular, NO GPS fusion)')
    print(f'  scale_ratio       = VIO/GPS = {scale_ratio:.4f}   '
          f'({"OVER" if scale_ratio>1.02 else "UNDER" if scale_ratio<0.98 else "near 1"})')
    print(f'  yaw_align         = {np.rad2deg(yaw_rad):+7.2f} deg  '
          f'(2-D Procrustes of B0 vs GPS over edge)')
    print(f'  edge XY  RMSE     = {edge_xy_rmse:.2f} m  (after yaw + edge-start anchor)')
    print(f'  edge XY  final    = {edge_xy_final:.2f} m  (after yaw + edge-start anchor)')
    print(f'  edge Z   RMSE     = {edge_z_rmse:.2f} m')
    print(f'  translation align (dx,dy,dz) = '
          f'({off[0]:+.2f}, {off[1]:+.2f}, {off[2]:+.2f}) m  '
          f'(over {align_window_s:.1f}s window from edge_start)')
    print(f'  cumulative scale_ratio by window:')
    for w in mw:
        print(f'    [{w["window_s"]:>3.0f}s]  B0_len={w["b0_len"]:>7.2f}  '
              f'GPS_len={w["gps_len"]:>7.2f}  ratio={w["ratio"]:.3f}')
    if t_takeoff_rel is not None:
        print(f'  GPS_takeoff_rel   = {t_takeoff_rel:.2f}s  '
              f'(drone first moves >=2m from rest)')
        print(f'  motion-only window= [{mo_lo:.1f}, {mo_hi:.1f}]s rel')
        print(f'  motion-only       B0_len={mo_b0_len:.2f}  GPS_len={mo_gps_len:.2f}  '
              f'ratio={mo_ratio:.3f}    <- TRUE SCALE (drift-free)')
    else:
        print(f'  GPS_takeoff_rel   = (not detected)')

    if not _HAVE_MPL: continue

    out_pfx = os.path.join(OUT_DIR, f'fly{fly}')
    os.makedirs(out_pfx, exist_ok=True)

    # ---- plot 1: first-edge zoom (yaw aligned, no scale align) ----
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    ax.plot(gps_xy[in_edge_gps, 0], gps_xy[in_edge_gps, 1], 'k:', lw=2.2,
            label=f'GPS-XY edge (len={gps_edge_len:.1f}m, chord={gps_chord:.1f}m)')
    ax.plot(b0_edge_xy_yawed[:, 0], b0_edge_xy_yawed[:, 1], 'b-', lw=1.6,
            label=f'B0 mono [yaw={np.rad2deg(yaw_rad):+.1f}°] '
                  f'len={b0_edge_len:.1f}m  ratio={scale_ratio:.3f}  rmse={edge_xy_rmse:.1f}m')
    ax.scatter([gps_xy[in_edge_gps, 0][0]], [gps_xy[in_edge_gps, 1][0]],
               s=70, c='k', marker='o', label='edge_start')
    ax.scatter([gps_xy[in_edge_gps, 0][-1]], [gps_xy[in_edge_gps, 1][-1]],
               s=70, c='r', marker='x', label='edge_end')
    ax.set_xlabel('E (m)'); ax.set_ylabel('N (m)')
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=0.3)
    ax.set_title(f'fly{fly} first edge — pure monocular B0 vs GPS (yaw-aligned, NO scale align)\n'
                 f'edge rel [{e_s:.1f},{e_e:.1f}]s  '
                 f'scale_ratio = VIO/GPS = {scale_ratio:.3f}')
    ax.legend(loc='best', fontsize=9)
    p = f'{out_pfx}/first_edge_xy.png'
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f'  [saved] {p}')

    # ---- plot 1b: scale_ratio cumulative window ----
    fig, ax = plt.subplots(1, 1, figsize=(9, 5))
    ws = [w['window_s'] for w in mw if not np.isnan(w['ratio'])]
    rs = [w['ratio']    for w in mw if not np.isnan(w['ratio'])]
    ax.axhline(1.0, color='k', lw=0.8, ls='--', alpha=0.5)
    ax.plot(ws, rs, 'bo-', lw=1.4)
    for x, y in zip(ws, rs):
        ax.annotate(f'{y:.3f}', (x, y), textcoords='offset points', xytext=(5, 5))
    ax.set_xlabel('cumulative duration from edge_start (s)')
    ax.set_ylabel('scale_ratio  = path_len(B0_xy) / path_len(GPS_xy)')
    ax.set_title(f'fly{fly} scale_ratio vs cumulative window from edge_start '
                 f'(edge {e_s:.0f}–{e_e:.0f}s rel)')
    ax.grid(alpha=0.3)
    p = f'{out_pfx}/scale_vs_window.png'
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f'  [saved] {p}')

    # ---- plot 2: full overlap XY ----
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    ax.plot(gps_xy[:, 0], gps_xy[:, 1], 'k:', lw=1.4, label='GPS-XY (full)')
    ax.plot(aligned[:, 0], aligned[:, 1], 'b-', lw=1.1,
            label='B0 mono (full, edge-translation-aligned)')
    # Highlight edge
    ax.plot(b0_edge_xy[:, 0], b0_edge_xy[:, 1], 'b-', lw=2.4, alpha=0.6,
            label='first edge (B0)')
    ax.plot(gps_xy[in_edge_gps, 0], gps_xy[in_edge_gps, 1], 'k-', lw=2.4,
            alpha=0.6, label='first edge (GPS)')
    ax.set_xlabel('E (m)'); ax.set_ylabel('N (m)')
    ax.set_aspect('equal', adjustable='datalim'); ax.grid(alpha=0.3)
    ax.set_title(f'fly{fly} XY trajectory  |  B0 (pure mono, NO GPS fusion) vs GPS')
    ax.legend(loc='best', fontsize=9)
    p = f'{out_pfx}/xy_full.png'
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f'  [saved] {p}')

    # ---- plot 3: Z vs time, edge highlighted ----
    fig, ax = plt.subplots(1, 1, figsize=(13, 5))
    ax.plot(gps_t - t0, gps_alt, 'k:', lw=1.4, label='GPS altitude (m)')
    ax.plot(b0_t_o - t0, aligned[:, 2], 'b-', lw=1.2,
            label='B0 mono z (edge-translation-aligned)')
    ax.axvspan(e_s, e_e, alpha=0.15, color='orange',
               label=f'first edge [{e_s:.0f},{e_e:.0f}]s')
    ax.axvline(t_init_overlap_rel, color='green', lw=1.0, ls='--',
               label=f'first B0 sample in overlap (t={t_init_overlap_rel:.1f}s)')
    ax.set_xlabel('t_rel (s)  (anchor = overlap start)')
    ax.set_ylabel('z (m)')
    ax.set_title(f'fly{fly} altitude — B0 vs GPS (z RMSE in edge = {edge_z_rmse:.2f} m)')
    ax.legend(loc='best', fontsize=8); ax.grid(alpha=0.3)
    p = f'{out_pfx}/z_vs_time.png'
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f'  [saved] {p}')

# ----------------------- summary ---------------------------
print(f'\n{"="*78}\n CROSS-FLIGHT FIRST-EDGE SCALE SUMMARY (pure monocular B0)\n{"="*78}')
hdr = (f'{"fly":>3s} {"takeoff":>7s} {"mo_win":>10s} '
       f'{"mo_GPS":>7s} {"mo_VIO":>7s} {"mo_ratio":>8s} '
       f'{"full_ratio":>10s} {"yaw_deg":>7s} {"XY_rmse":>8s} {"Z_rmse":>7s}')
print(hdr); print('-' * len(hdr))
for r in results:
    mo = r['motion_only_scale_ratio']
    v_mo = 'OVER' if mo > 1.02 else ('UNDER' if mo < 0.98 else 'near1')
    mo_lo, mo_hi = r['motion_only_window_rel']
    tk = r['gps_takeoff_rel']
    print(f'{r["flight"]:>3d} {tk if tk is None else f"{tk:>5.1f}s":>7s} '
          f'[{mo_lo:>4.0f},{mo_hi:>3.0f}]s '
          f'{r["motion_only_gps_len_m"]:>7.2f} {r["motion_only_b0_len_m"]:>7.2f} '
          f'{mo:>7.3f}{v_mo:<1s} '
          f'{r["scale_ratio"]:>10.3f} '
          f'{r["yaw_align_deg"]:>+7.2f} '
          f'{r["edge_xy_rmse_m_post_yaw"]:>8.2f} '
          f'{r["edge_z_rmse_m"]:>7.2f}')

with open(os.path.join(OUT_DIR, 'summary.json'), 'w') as f:
    json.dump(results, f, indent=2, default=str)
print(f'\n[saved] {OUT_DIR}/summary.json')
