#!/usr/bin/env python3
"""Cross-flight init_window_time validation.

Tests init_window_time = 2.0 (baseline), 4.0, 5.0 on all 4 flights.
Extracts init diagnostics from logs + raw IMU, and first-edge scale from trajectories.
"""

import json, os, re, sys
import numpy as np

# ---------------------------------------------------------------------------
# Flight configs: (flight_num, baseline_log, baseline_traj, gps_csv, start_time, imu_csv)
# ---------------------------------------------------------------------------
FLIGHTS = [
    dict(flight=1, start_time=200.0,
         imu='20260509_fly1/mav0/imu0/data.csv',
         gps='20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv',
         bl_log='20260509_fly1/result/baselines_v1/B0_no_gps.log',
         bl_traj='20260509_fly1/result/baselines_v1/B0_no_gps.txt',
         sweep_dir='20260509_fly1/result/init_window_time_sweep'),
    dict(flight=2, start_time=160.0,
         imu='20260509_fly2/mav0/imu0/data.csv',
         gps='20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv',
         bl_log='20260509_fly2/result/baselines_v1/B0_no_gps.log',
         bl_traj='20260509_fly2/result/baselines_v1/B0_no_gps.txt',
         sweep_dir='20260509_fly2/result/init_window_time_sweep'),
    dict(flight=3, start_time=180.0,
         imu='20260509_fly3/mav0/imu0/data.csv',
         gps='20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv',
         bl_log='20260509_fly3/result/baselines_v1/B0_no_gps.log',
         bl_traj='20260509_fly3/result/baselines_v1/B0_no_gps.txt',
         sweep_dir='20260509_fly3/result/init_window_time_sweep'),
    dict(flight=4, start_time=239.0,
         imu='20260509_fly4/mav0/imu0/data.csv',
         gps='20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv',
         bl_log='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.log',
         bl_traj='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.txt',
         sweep_dir='20260509_fly4/result/init_window_time_sweep'),
]

OUT_JSON = 'comparison_plots/stage_b_diag/cross_flight_iwt_sweep.json'

INIT_IMU_THRESH = 1.5
GRAVITY = 9.81

# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------
def load_imu_region(path, t_lo_s, t_hi_s):
    rows = []
    t_lo_ns, t_hi_ns = t_lo_s * 1e9, t_hi_s * 1e9
    with open(path) as f:
        f.readline()
        for ln in f:
            ln = ln.strip()
            if not ln: continue
            ps = ln.split(',')
            try: t_ns = float(ps[0])
            except ValueError: continue
            if t_ns < t_lo_ns: continue
            if t_ns >= t_hi_ns: break
            rows.append([t_ns * 1e-9] + [float(x) for x in ps[1:7]])
    a = np.array(rows)
    if a.size == 0:
        return dict(t=np.array([]), w=np.empty((0,3)), a=np.empty((0,3)))
    return dict(t=a[:,0], w=a[:,1:4], a=a[:,4:7])


def slice_window(imu, t_lo, t_hi):
    mask = (imu['t'] > t_lo) & (imu['t'] <= t_hi)
    return dict(t=imu['t'][mask], w=imu['w'][mask], a=imu['a'][mask])


def compute_var(a_samples):
    n = len(a_samples)
    if n < 2: return float('inf')
    a_avg = a_samples.mean(axis=0)
    s = sum((ai - a_avg).dot(ai - a_avg) for ai in a_samples)
    return float(np.sqrt(s / (n - 1)))


def gram_schmidt(z_axis):
    z = z_axis / np.linalg.norm(z_axis)
    e1 = np.array([1.0, 0.0, 0.0])
    x = np.cross(e1, z)
    if np.linalg.norm(x) < 1e-12:
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rot2quat(R):
    qw = np.sqrt(max(0.0, 1.0 + R[0,0] + R[1,1] + R[2,2])) / 2.0
    if qw > 1e-8:
        qx = (R[2,1] - R[1,2]) / (4.0 * qw)
        qy = (R[0,2] - R[2,0]) / (4.0 * qw)
        qz = (R[1,0] - R[0,1]) / (4.0 * qw)
    else:
        qx = np.sqrt(max(0.0, 1.0 + R[0,0] - R[1,1] - R[2,2])) / 2.0
        qy = (R[1,0] + R[0,1]) / (4.0 * max(qx, 1e-12))
        qz = (R[2,0] + R[0,2]) / (4.0 * max(qx, 1e-12))
        qw = (R[2,1] - R[1,2]) / (4.0 * max(qx, 1e-12))
    return np.array([qx, qy, qz, qw])


def recompute_from_older(imu_older):
    a_avg = imu_older['a'].mean(axis=0)
    w_avg = imu_older['w'].mean(axis=0)
    gravity_dir = a_avg / np.linalg.norm(a_avg)
    R_GtoI = gram_schmidt(gravity_dir)
    bg = w_avg
    g_inI = R_GtoI @ np.array([0.0, 0.0, GRAVITY])
    ba = a_avg - g_inI
    q = rot2quat(R_GtoI)
    a_var = compute_var(imu_older['a'])
    w_var = compute_var(imu_older['w'])
    tilt_deg = float(np.rad2deg(np.arccos(np.clip(gravity_dir[2], -1, 1))))
    a_horiz = float(np.sqrt(a_avg[0]**2 + a_avg[1]**2))
    return dict(ba=ba, bg=bg, q=q, a_avg=a_avg, w_avg=w_avg,
                a_var=a_var, w_var=w_var, tilt_deg=tilt_deg, a_horiz=a_horiz)


def scan_windows(imu_region, init_window_time):
    t = imu_region['t']
    if len(t) < 200: return []
    half = init_window_time / 2.0
    results = []
    t_lo = float(t[0] + init_window_time + 0.5)
    t_hi = float(t[-1] - half - 0.5)
    for s in np.arange(t_lo, t_hi, 0.02):
        older = slice_window(imu_region, s - init_window_time, s - half)
        newer = slice_window(imu_region, s - half, s)
        if len(older['t']) < 20 or len(newer['t']) < 20: continue
        a_var_old = compute_var(older['a'])
        a_var_new = compute_var(newer['a'])
        if a_var_old < INIT_IMU_THRESH and a_var_new > INIT_IMU_THRESH:
            rec = recompute_from_older(older)
            results.append(dict(
                t_split=float(s),
                older_start=float(s - init_window_time),
                older_end=float(s - half),
                newer_start=float(s - half),
                newer_end=float(s),
                init_window_time=init_window_time,
                a_var_old=float(a_var_old), a_var_new=float(a_var_new),
                ba=list(rec['ba']), bg=list(rec['bg']), q=list(rec['q']),
                ba_z=float(rec['ba'][2]),
                bg_x=float(rec['bg'][0]), bg_y=float(rec['bg'][1]), bg_z=float(rec['bg'][2]),
                n_older=len(older['t']), n_newer=len(newer['t']),
                tilt_deg=rec['tilt_deg'], a_horiz=rec['a_horiz'],
                a_avg=list(rec['a_avg']), w_avg=list(rec['w_avg']),
            ))
    return results


# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------
def parse_log(path):
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    out = dict(success=False)
    with open(path, errors='replace') as f:
        lines = f.readlines()
    for ln in lines:
        ln = ansi.sub('', ln)
        m = re.search(r'\[init\]: bias gyro = ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
        if m: out['bg'] = np.array([float(m.group(i)) for i in range(1,4)])
        m = re.search(r'\[init\]: bias accel = ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
        if m: out['ba'] = np.array([float(m.group(i)) for i in range(1,4)])
        m = re.search(r'\[init\]: orientation = ([-\d.]+), ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
        if m: out['q'] = np.array([float(m.group(i)) for i in range(1,5)])
        if 'successful initialization' in ln:
            out['success'] = True
    to_vals = []
    for ln in lines:
        ln = ansi.sub('', ln)
        m = re.search(r'camera-imu timeoffset = ([\-\d.]+)', ln)
        if m: to_vals.append(float(m.group(1)))
    out['timeoffset_initial'] = to_vals[0] if to_vals else None
    out['timeoffset_final'] = to_vals[-1] if to_vals else None
    # Get frame count
    m = re.search(r'frames=(\d+)', ansi.sub('', lines[-1] if lines else ''))
    out['frames'] = int(m.group(1)) if m else None
    return out


def parse_disparity_at_success(path):
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    with open(path, errors='replace') as f:
        lines = f.readlines()
    last_disp = None
    for ln in lines:
        ln = ansi.sub('', ln)
        m = re.search(r'disparity is ([\d.]+),([\d.]+) \(([\d.]+) thresh\)', ln)
        if m: last_disp = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
        if 'successful initialization' in ln:
            return last_disp
    return None


# ---------------------------------------------------------------------------
# GPS / first-edge scale
# ---------------------------------------------------------------------------
def load_tum_traj(path):
    rows = []
    if not os.path.exists(path): return np.empty((0,8))
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            if len(ps) < 8: continue
            try: rows.append([float(x) for x in ps[:8]])
            except ValueError: continue
    a = np.array(rows)
    if a.size == 0: return np.empty((0,8))
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
    return a[a[:, 0].argsort()][keep]


def load_gps_llz(path):
    rows = []
    if not os.path.exists(path): return np.empty((0,4))
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try: rows.append((float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])))
            except (ValueError, IndexError): continue
    a = np.array(rows)
    if a.size and a[0,0] > 1e11: a[:,0] *= 1e-9
    return a[np.argsort(a[:,0])]


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
    lat0, lon0, alt0 = g[0,1], g[0,2], g[0,3]
    e0 = wgs84_to_ecef(lat0, lon0, alt0)
    R = ecef_to_enu_R(lat0, lon0)
    e = wgs84_to_ecef(g[:,1], g[:,2], g[:,3])
    enu = (e - e0) @ R.T
    out = np.zeros((len(g), 4))
    out[:,0] = g[:,0]; out[:,1:4] = enu
    return out


def compute_first_edge_scale(traj_tum, gps_csv, win_s=30.0, step_s=5.0,
                              motion_thresh_m=5.0, edge_dur_s=80.0):
    traj = load_tum_traj(traj_tum)
    gps_raw = load_gps_llz(gps_csv)
    if traj.size == 0 or gps_raw.size == 0:
        return None

    gps = gps_to_enu(gps_raw)
    t_lo = max(traj[0,0], gps[0,0])
    t_hi = min(traj[-1,0], gps[-1,0])
    if t_hi <= t_lo: return None

    gps_t = gps[:,0]; gps_xy = gps[:,1:3]
    gps_x_interp = np.interp(traj[:,0], gps_t, gps_xy[:,0])
    gps_y_interp = np.interp(traj[:,0], gps_t, gps_xy[:,1])
    traj_xy = traj[:,1:3]; traj_t = traj[:,0]
    t0 = traj_t[0]

    gps_d = np.sqrt(np.diff(gps_x_interp)**2 + np.diff(gps_y_interp)**2)
    gps_cum = np.cumsum(np.concatenate([[0], gps_d]))
    tk_mask = gps_cum >= motion_thresh_m
    if not tk_mask.any(): return None
    t_takeoff = traj_t[np.where(tk_mask)[0][0]]
    t_takeoff_rel = t_takeoff - t0

    centers, ratios, motion_mask = [], [], []
    c = max(t_takeoff_rel, win_s/2)
    while c + win_s/2 <= traj_t[-1] - t0:
        mask = (traj_t >= t0 + c - win_s/2) & (traj_t <= t0 + c + win_s/2)
        if mask.sum() < 5:
            centers.append(c); ratios.append(None); motion_mask.append(False)
            c += step_s; continue
        b0_path = np.sum(np.sqrt(np.diff(traj_xy[mask,0])**2 + np.diff(traj_xy[mask,1])**2))
        gps_path = np.sum(np.sqrt(np.diff(gps_x_interp[mask])**2 + np.diff(gps_y_interp[mask])**2))
        centers.append(c); ratios.append(float(b0_path / max(gps_path, 1e-3)))
        motion_mask.append(bool(gps_path >= motion_thresh_m))
        c += step_s

    centers = np.array(centers); ratios = np.array(ratios)
    motion_mask = np.array(motion_mask, dtype=bool)
    edge_mask = (centers >= t_takeoff_rel) & (centers <= t_takeoff_rel + edge_dur_s) & motion_mask
    first_edge_mean = float(np.mean(ratios[edge_mask])) if edge_mask.sum() > 0 else None

    post_tk = np.where((centers >= t_takeoff_rel) & motion_mask)[0][:6]
    first_windows = [dict(center=float(centers[j]), ratio=float(ratios[j]) if not np.isnan(ratios[j]) else None)
                     for j in post_tk]

    return dict(first_edge_mean=first_edge_mean, first_windows=first_windows,
                t_takeoff_rel=float(t_takeoff_rel),
                centers=centers.tolist(), ratios=[float(x) if x is not None and not np.isnan(x) else None for x in ratios],
                motion_mask=[bool(x) for x in motion_mask])


# ---------------------------------------------------------------------------
def process_run(cfg, iwt, log_path, traj_path):
    """Process one run: parse log, match IMU window, compute scale."""
    result = dict(flight=cfg['flight'], iwt=iwt, success=False)

    if not os.path.exists(log_path):
        result['error'] = 'no_log'
        return result

    info = parse_log(log_path)
    result['logged_ba_z'] = float(info['ba'][2]) if info.get('ba') is not None else None
    result['logged_ba'] = info['ba'].tolist() if info.get('ba') is not None else None
    result['logged_bg'] = info['bg'].tolist() if info.get('bg') is not None else None
    result['timeoffset_initial'] = info.get('timeoffset_initial')
    result['timeoffset_final'] = info.get('timeoffset_final')
    result['frames'] = info.get('frames')
    result['success'] = info.get('success', False)
    result['disparity'] = parse_disparity_at_success(log_path)

    if not result['success']:
        return result

    # Raw IMU matching
    with open(cfg['imu']) as f:
        f.readline()
        t0_imu = float(f.readline().split(',')[0]) * 1e-9
    t_eff = t0_imu + cfg['start_time']
    imu = load_imu_region(cfg['imu'], t_eff, t_eff + 60.0)
    windows = scan_windows(imu, iwt)

    matched = None
    if info.get('ba') is not None:
        for w in windows:
            if np.linalg.norm(w['ba'] - info['ba']) < 1e-4 and np.linalg.norm(w['bg'] - info['bg']) < 1e-4:
                matched = w
                break

    if matched:
        result['raw_imu'] = dict(
            t_split=matched['t_split'],
            older_start=matched['older_start'],
            older_end=matched['older_end'],
            newer_start=matched['newer_start'],
            newer_end=matched['newer_end'],
            a_var_old=matched['a_var_old'],
            a_var_new=matched['a_var_new'],
            recomputed_ba_z=matched['ba_z'],
            recomputed_bg_x=matched['bg_x'],
            recomputed_bg_y=matched['bg_y'],
            recomputed_bg_z=matched['bg_z'],
            n_older=matched['n_older'],
            tilt_deg=matched['tilt_deg'],
            a_horiz=matched['a_horiz'],
            ba=list(matched['ba']),
            bg=list(matched['bg']),
        )

    # First-edge scale
    scale = compute_first_edge_scale(traj_path, cfg['gps'])
    if scale:
        result['first_edge_mean'] = scale['first_edge_mean']
        result['first_windows'] = scale['first_windows']
        result['t_takeoff_rel'] = scale['t_takeoff_rel']

    return result


# ---------------------------------------------------------------------------
def main():
    all_results = []

    for cfg in FLIGHTS:
        fly = cfg['flight']
        print(f"\n{'='*70}\n  FLY{fly}\n{'='*70}")

        # Baseline (iwt=2.0)
        bl = process_run(cfg, 2.0, cfg['bl_log'], cfg['bl_traj'])
        bl_ok = 'OK' if bl['success'] else 'FAILED'
        bl_fe = f"{bl.get('first_edge_mean'):.4f}" if bl.get('first_edge_mean') else '?'
        bl_ba = f"{bl.get('logged_ba_z'):+.5f}" if bl.get('logged_ba_z') is not None else '?'
        print(f"  iwt=2.0: {bl_ok}  ba_z={bl_ba}  fe={bl_fe}  to_init={bl.get('timeoffset_initial')}")
        all_results.append(bl)

        # Sweep (iwt=4.0, 5.0)
        for iwt in [4.0, 5.0]:
            log_path = os.path.join(cfg['sweep_dir'], f'B0_iwt{iwt}.log')
            traj_path = os.path.join(cfg['sweep_dir'], f'B0_iwt{iwt}.txt')
            r = process_run(cfg, iwt, log_path, traj_path)
            r_ok = 'OK' if r['success'] else 'FAILED'
            r_fe = f"{r.get('first_edge_mean'):.4f}" if r.get('first_edge_mean') else '?'
            r_ba = f"{r.get('logged_ba_z'):+.5f}" if r.get('logged_ba_z') is not None else '?'
            delta = ''
            if r.get('first_edge_mean') and bl.get('first_edge_mean'):
                delta = f"Δ={r['first_edge_mean'] - bl['first_edge_mean']:+.4f}"
            print(f"  iwt={iwt}: {r_ok}  ba_z={r_ba}  fe={r_fe}  {delta}  to_init={r.get('timeoffset_initial')}")
            all_results.append(r)

    # ---- Cross-flight summary ----
    print(f"\n\n{'='*100}")
    print(f"  CROSS-FLIGHT INIT_WINDOW_TIME SUMMARY")
    print(f"{'='*100}")

    for iwt in [2.0, 4.0, 5.0]:
        print(f"\n  --- iwt={iwt} ---")
        print(f"  {'flight':>8s}  {'success':>8s}  {'ba_z':>10s}  {'a_var_old':>10s}  {'older_disp':>10s}  "
              f"{'to_init':>8s}  {'first_edge':>12s}  {'err_vs_1.0':>10s}")
        print(f"  {'-'*8}  {'-'*8}  {'-'*10}  {'-'*10}  {'-'*10}  {'-'*8}  {'-'*12}  {'-'*10}")
        for cfg in FLIGHTS:
            fly = cfg['flight']
            matches = [r for r in all_results if r['flight'] == fly and abs(r['iwt'] - iwt) < 0.01]
            if not matches:
                print(f"  {fly:>8d}  {'?':>8s}")
                continue
            r = matches[0]
            ok = 'OK' if r['success'] else 'FAIL'
            ba = f"{r['logged_ba_z']:+.5f}" if r.get('logged_ba_z') is not None else '?'
            ri = r.get('raw_imu', {}) or {}
            av = f"{ri.get('a_var_old', float('nan')):.4f}" if ri else '?'
            disp = f"{r['disparity'][0]:.3f}" if r.get('disparity') else '?'
            to_i = f"{r.get('timeoffset_initial', '?')}"
            fe = f"{r['first_edge_mean']:.4f}" if r.get('first_edge_mean') else '?'
            err = f"{r['first_edge_mean'] - 1.0:+.4f}" if r.get('first_edge_mean') else '?'
            print(f"  {fly:>8d}  {ok:>8s}  {ba:>10s}  {av:>10s}  {disp:>10s}  "
                  f"{to_i:>8s}  {fe:>12s}  {err:>10s}")

    # ---- Answers to A-E ----
    print(f"\n\n{'='*70}")
    print(f"  ANSWERS TO KEY QUESTIONS")
    print(f"{'='*70}")

    # Organize data
    by_flight_iwt = {}
    for r in all_results:
        by_flight_iwt[(r['flight'], r['iwt'])] = r

    # A. Does iwt=4.0 improve fly2/fly3 under-scale?
    print(f"\n  A. Does iwt=4.0 improve fly2/fly3 under-scale?")
    for fly in [2, 3]:
        bl_r = by_flight_iwt.get((fly, 2.0))
        iw4_r = by_flight_iwt.get((fly, 4.0))
        if bl_r and iw4_r and bl_r.get('first_edge_mean') and iw4_r.get('first_edge_mean'):
            delta = iw4_r['first_edge_mean'] - bl_r['first_edge_mean']
            closer = abs(iw4_r['first_edge_mean'] - 1.0) < abs(bl_r['first_edge_mean'] - 1.0)
            print(f"    fly{fly}: {bl_r['first_edge_mean']:.4f} → {iw4_r['first_edge_mean']:.4f}  "
                  f"Δ={delta:+.4f}  closer_to_1.0={'YES' if closer else 'NO'}")

    # B. Does it preserve fly1's already-good scale?
    print(f"\n  B. Does iwt=4.0 preserve fly1's already-good scale?")
    for fly in [1]:
        bl_r = by_flight_iwt.get((fly, 2.0))
        iw4_r = by_flight_iwt.get((fly, 4.0))
        if bl_r and iw4_r and bl_r.get('first_edge_mean') and iw4_r.get('first_edge_mean'):
            delta = iw4_r['first_edge_mean'] - bl_r['first_edge_mean']
            worse = abs(iw4_r['first_edge_mean'] - 1.0) > abs(bl_r['first_edge_mean'] - 1.0)
            print(f"    fly{fly}: {bl_r['first_edge_mean']:.4f} → {iw4_r['first_edge_mean']:.4f}  "
                  f"Δ={delta:+.4f}  regressed={'YES' if worse else 'NO'}")

    # C. Does it reduce or worsen fly4's transition overshoot?
    print(f"\n  C. Does iwt=4.0 reduce or worsen fly4's transition overshoot?")
    for fly in [4]:
        bl_r = by_flight_iwt.get((fly, 2.0))
        iw4_r = by_flight_iwt.get((fly, 4.0))
        if bl_r and iw4_r:
            bl_fw = bl_r.get('first_windows', [])
            iw4_fw = iw4_r.get('first_windows', [])
            if bl_fw:
                print(f"    fly4 baseline first SW: {bl_fw[0]['ratio']:.3f}" if bl_fw[0].get('ratio') else "    fly4 baseline: ?")
            if iw4_fw:
                print(f"    fly4 iwt=4.0 first SW: {iw4_fw[0]['ratio']:.3f}" if iw4_fw[0].get('ratio') else "    fly4 iwt=4.0: ?")
            if bl_r.get('first_edge_mean') and iw4_r.get('first_edge_mean'):
                delta = iw4_r['first_edge_mean'] - bl_r['first_edge_mean']
                print(f"    first_edge: {bl_r['first_edge_mean']:.4f} → {iw4_r['first_edge_mean']:.4f}  Δ={delta:+.4f}")

    # D. Is 4.0 generally better?
    print(f"\n  D. Is iwt=4.0 generally better than iwt=2.0?")
    improvements = []
    regressions = []
    for fly in [1,2,3,4]:
        bl_r = by_flight_iwt.get((fly, 2.0))
        iw4_r = by_flight_iwt.get((fly, 4.0))
        if bl_r and iw4_r and bl_r.get('first_edge_mean') and iw4_r.get('first_edge_mean'):
            bl_err = abs(bl_r['first_edge_mean'] - 1.0)
            iw4_err = abs(iw4_r['first_edge_mean'] - 1.0)
            if iw4_err < bl_err:
                improvements.append((fly, bl_err, iw4_err))
            else:
                regressions.append((fly, bl_err, iw4_err))
    print(f"    Improved: {len(improvements)} flights {[f for f,_,_ in improvements]}")
    print(f"    Regressed: {len(regressions)} flights {[f for f,_,_ in regressions]}")
    if len(improvements) >= 3:
        print(f"    VERDICT: iwt=4.0 is a candidate default improvement")
    elif len(improvements) >= 2 and len(regressions) <= 1:
        print(f"    VERDICT: iwt=4.0 helps most flights, check regressions")
    else:
        print(f"    VERDICT: iwt=4.0 is NOT a general improvement")

    # E. What changed besides ba_z?
    print(f"\n  E. What init quantity changed enough to explain scale improvement?")
    for fly in [1,2,3,4]:
        bl_r = by_flight_iwt.get((fly, 2.0))
        iw4_r = by_flight_iwt.get((fly, 4.0))
        if not bl_r or not iw4_r: continue
        bl_ri = bl_r.get('raw_imu', {}) or {}
        iw4_ri = iw4_r.get('raw_imu', {}) or {}
        d_ba = iw4_r.get('logged_ba_z', 0) - bl_r.get('logged_ba_z', 0) if bl_r.get('logged_ba_z') and iw4_r.get('logged_ba_z') else 0
        d_av = iw4_ri.get('a_var_old', 0) - bl_ri.get('a_var_old', 0) if bl_ri and iw4_ri else 0
        d_to = (iw4_r.get('timeoffset_initial', 0) or 0) - (bl_r.get('timeoffset_initial', 0) or 0)
        d_disp = (iw4_r.get('disparity', (0,))[0] - bl_r.get('disparity', (0,))[0]) if bl_r.get('disparity') and iw4_r.get('disparity') else 0
        print(f"    fly{fly}: Δba_z={d_ba:+.5f}  Δa_var={d_av:+.4f}  Δto_init={d_to:+.5f}  Δolder_disp={d_disp:+.3f}")

    # ---- Write JSON ----
    class NpEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, (np.bool_,)): return bool(obj)
            if isinstance(obj, (np.integer,)): return int(obj)
            if isinstance(obj, (np.floating,)): return float(obj)
            if isinstance(obj, np.ndarray): return obj.tolist()
            return super().default(obj)

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    json.dump(all_results, open(OUT_JSON, 'w'), indent=2, cls=NpEncoder)
    print(f"\n[saved] {OUT_JSON}")


if __name__ == '__main__':
    main()
