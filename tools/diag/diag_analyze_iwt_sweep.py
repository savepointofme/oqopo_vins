#!/usr/bin/env python3
"""Analyze fly3 init_window_time sweep results.

For each init_window_time value (2.0 baseline, 3.0, 4.0, 5.0):
  - Parse init success/failure from log
  - Scan raw IMU to find matching init window
  - Recompute ba_z, bg, q from raw IMU
  - Compute first-edge scale from trajectory
"""

import json, os, re, sys, glob
import numpy as np

# ---------------------------------------------------------------------------
IMU_CSV = '20260509_fly3/mav0/imu0/data.csv'
GPS_CSV = '20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'
BASELINE_LOG = '20260509_fly3/result/baselines_v1/B0_no_gps.log'
BASELINE_TRAJ = '20260509_fly3/result/baselines_v1/B0_no_gps.txt'
SWEEP_DIR = '20260509_fly3/result/init_window_time_sweep'
OUT_JSON = 'comparison_plots/stage_b_diag/fly3_iwt_sweep.json'

INIT_IMU_THRESH = 1.5
GRAVITY = 9.81
SCAN_RANGE = 30.0

# ---------------------------------------------------------------------------
def load_imu_region(path, t_lo_s, t_hi_s):
    rows = []
    t_lo_ns = t_lo_s * 1e9; t_hi_ns = t_hi_s * 1e9
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
    tilt_rad = np.arccos(np.clip(gravity_dir[2], -1, 1))
    a_horiz = np.sqrt(a_avg[0]**2 + a_avg[1]**2)
    return dict(ba=ba, bg=bg, q=q, a_avg=a_avg, w_avg=w_avg,
                a_var=a_var, w_var=w_var, tilt_deg=np.rad2deg(tilt_rad),
                a_horiz=a_horiz)


def scan_windows(imu_region, init_window_time):
    """Find all windows satisfying StaticInitializer criteria with given window_time."""
    t = imu_region['t']
    if len(t) < 200: return []
    half = init_window_time / 2.0
    results = []
    t_lo = t[0] + init_window_time + 0.5
    t_hi = t[-1] - half - 0.5
    step = 0.01
    for s in np.arange(t_lo, t_hi, step):
        older = slice_window(imu_region, s - init_window_time, s - half)
        newer = slice_window(imu_region, s - half, s)
        if len(older['t']) < 20 or len(newer['t']) < 20:
            continue
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
                a_var_old=float(a_var_old),
                a_var_new=float(a_var_new),
                ba=rec['ba'], bg=rec['bg'], q=rec['q'],
                ba_z=float(rec['ba'][2]),
                bg_x=float(rec['bg'][0]), bg_y=float(rec['bg'][1]), bg_z=float(rec['bg'][2]),
                n_older=len(older['t']),
                n_newer=len(newer['t']),
                tilt_deg=rec['tilt_deg'],
                a_horiz=float(rec['a_horiz']),
                a_avg=list(rec['a_avg']),
                w_avg=list(rec['w_avg']),
                a_norm_mean=float(np.mean(np.linalg.norm(older['a'], axis=1))),
                w_norm_mean=float(np.mean(np.linalg.norm(older['w'], axis=1))),
            ))
    return results


# ---------------------------------------------------------------------------
def parse_log(path):
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    out = {}
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
        m = re.search(r'start-time=([\d.]+)s', ln)
        if m: out['start_time'] = float(m.group(1))
        if 'successful initialization' in ln:
            out['success'] = True
    to_vals = []
    for ln in lines:
        ln = ansi.sub('', ln)
        m = re.search(r'camera-imu timeoffset = ([\-\d.]+)', ln)
        if m: to_vals.append(float(m.group(1)))
    out['timeoffset_initial'] = to_vals[0] if to_vals else None
    out['timeoffset_final'] = to_vals[-1] if to_vals else None
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
# GPS / trajectory / first-edge scale
# ---------------------------------------------------------------------------
def load_tum_traj(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            if len(ps) < 8: continue
            try: rows.append([float(x) for x in ps[:8]])
            except ValueError: continue
    a = np.array(rows)
    if a.size == 0: return np.empty((0, 8))
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
    return a[a[:, 0].argsort()][keep]


def load_gps_llz(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try: rows.append((float(ps[0]), float(ps[1]), float(ps[2]), float(ps[3])))
            except (ValueError, IndexError): continue
    a = np.array(rows)
    if a.size and a[0, 0] > 1e11: a[:, 0] *= 1e-9
    return a[np.argsort(a[:, 0])]


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
    e0 = wgs84_to_ecef(lat0, lon0, alt0)
    R = ecef_to_enu_R(lat0, lon0)
    e = wgs84_to_ecef(g[:, 1], g[:, 2], g[:, 3])
    enu = (e - e0) @ R.T
    out = np.zeros((len(g), 4))
    out[:, 0] = g[:, 0]; out[:, 1:4] = enu
    return out


def compute_first_edge_scale(traj_tum, gps_csv, win_s=30.0, step_s=5.0,
                              motion_thresh_m=5.0, edge_dur_s=80.0):
    traj = load_tum_traj(traj_tum)
    gps_raw = load_gps_llz(gps_csv)
    if traj.size == 0 or gps_raw.size == 0:
        return None

    gps = gps_to_enu(gps_raw)
    t_lo = max(traj[0, 0], gps[0, 0])
    t_hi = min(traj[-1, 0], gps[-1, 0])
    if t_hi <= t_lo: return None

    gps_t = gps[:, 0]; gps_xy = gps[:, 1:3]
    gps_x_interp = np.interp(traj[:, 0], gps_t, gps_xy[:, 0])
    gps_y_interp = np.interp(traj[:, 0], gps_t, gps_xy[:, 1])
    traj_xy = traj[:, 1:3]; traj_t = traj[:, 0]
    t0 = traj_t[0]

    gps_d = np.sqrt(np.diff(gps_x_interp)**2 + np.diff(gps_y_interp)**2)
    gps_cum = np.cumsum(np.concatenate([[0], gps_d]))
    tk_mask = gps_cum >= motion_thresh_m
    if not tk_mask.any(): return None
    t_takeoff = traj_t[np.where(tk_mask)[0][0]]
    t_takeoff_rel = t_takeoff - t0

    centers, ratios, motion_mask = [], [], []
    c = max(t_takeoff_rel, win_s / 2)
    while c + win_s / 2 <= traj_t[-1] - t0:
        mask = (traj_t >= t0 + c - win_s/2) & (traj_t <= t0 + c + win_s/2)
        if mask.sum() < 5:
            centers.append(c); ratios.append(np.nan); motion_mask.append(False)
            c += step_s; continue
        b0_path = np.sum(np.sqrt(np.diff(traj_xy[mask, 0])**2 + np.diff(traj_xy[mask, 1])**2))
        gps_path = np.sum(np.sqrt(np.diff(gps_x_interp[mask])**2 + np.diff(gps_y_interp[mask])**2))
        centers.append(c); ratios.append(b0_path / max(gps_path, 1e-3))
        motion_mask.append(gps_path >= motion_thresh_m)
        c += step_s

    centers = np.array(centers); ratios = np.array(ratios)
    motion_mask = np.array(motion_mask, dtype=bool)
    edge_mask = (centers >= t_takeoff_rel) & (centers <= t_takeoff_rel + edge_dur_s) & motion_mask
    first_edge_mean = float(np.mean(ratios[edge_mask])) if edge_mask.sum() > 0 else None

    post_tk = np.where((centers >= t_takeoff_rel) & motion_mask)[0][:5]
    first_windows = [dict(center=float(centers[j]), ratio=float(ratios[j])) for j in post_tk]

    return dict(first_edge_mean=first_edge_mean, first_windows=first_windows,
                t_takeoff_rel=float(t_takeoff_rel))


# ---------------------------------------------------------------------------
def main():
    with open(IMU_CSV) as f:
        f.readline()
        t0_imu = float(f.readline().split(',')[0]) * 1e-9
    print(f"t0_imu = {t0_imu:.6f}")

    all_results = []
    START_TIME = 180.0
    t_eff = t0_imu + START_TIME

    # Process baseline (init_window_time=2.0)
    print(f"\n{'='*70}\n  BASELINE (iwt=2.0, st=180.0)\n{'='*70}")
    bl_log = parse_log(BASELINE_LOG)
    bl_disp = parse_disparity_at_success(BASELINE_LOG)
    print(f"  Logged ba_z: {bl_log['ba'][2]:+.5f}" if bl_log.get('ba') is not None else "  INIT FAILED")
    print(f"  Disparity: {bl_disp}")
    print(f"  to_initial: {bl_log.get('timeoffset_initial')}  to_final: {bl_log.get('timeoffset_final')}")

    imu_bl = load_imu_region(IMU_CSV, t_eff, t_eff + 60.0)
    windows_bl = scan_windows(imu_bl, 2.0)
    print(f"  Found {len(windows_bl)} valid windows (iwt=2.0)")

    matched_bl = None
    if bl_log.get('ba') is not None:
        for w in windows_bl:
            if np.linalg.norm(w['ba'] - bl_log['ba']) < 1e-4 and np.linalg.norm(w['bg'] - bl_log['bg']) < 1e-4:
                matched_bl = w
                print(f"  MATCHED: t_split={w['t_split']:.6f}  ba_z={w['ba_z']:+.5f}  "
                      f"a_var_old={w['a_var_old']:.4f}")
                break

    bl_scale = compute_first_edge_scale(BASELINE_TRAJ, GPS_CSV)
    bl_fe = bl_scale['first_edge_mean'] if bl_scale else None
    print(f"  first_edge_scale: {bl_fe:.4f}" if bl_fe else "  first_edge_scale: FAILED")

    # Process sweep runs
    for iwt in [3.0, 4.0, 5.0]:
        log_path = os.path.join(SWEEP_DIR, f'B0_iwt{iwt}.log')
        traj_path = os.path.join(SWEEP_DIR, f'B0_iwt{iwt}.txt')

        print(f"\n{'='*70}\n  iwt={iwt} (st=180.0)\n{'='*70}")

        if not os.path.exists(log_path):
            print(f"  Log file not found: {log_path}")
            all_results.append(dict(iwt=iwt, success=False, reason='no_log'))
            continue

        info = parse_log(log_path)
        info_disp = parse_disparity_at_success(log_path)

        if info.get('ba') is None:
            print(f"  INIT FAILED")
            # Check why
            with open(log_path, errors='replace') as f:
                content = f.read()
            if 'no accel jerk detected' in content:
                print(f"  Reason: no accel jerk detected")
            elif 'disparity' in content:
                # find last disparity
                matches = re.findall(r'disparity is ([\d.]+),([\d.]+)', content)
                if matches:
                    last = matches[-1]
                    print(f"  Last disparity: {last[0]}, {last[1]}")
            all_results.append(dict(iwt=iwt, success=False, reason='init_failed'))
            continue

        print(f"  Logged ba_z: {info['ba'][2]:+.5f}  ba: {info['ba']}")
        print(f"  Logged bg: {info['bg']}")
        print(f"  Disparity: {info_disp}")
        print(f"  to_initial: {info.get('timeoffset_initial')}  to_final: {info.get('timeoffset_final')}")

        imu = load_imu_region(IMU_CSV, t_eff, t_eff + 60.0)
        windows = scan_windows(imu, iwt)
        print(f"  Found {len(windows)} valid windows (iwt={iwt})")

        matched = None
        for w in windows:
            if np.linalg.norm(w['ba'] - info['ba']) < 1e-4 and np.linalg.norm(w['bg'] - info['bg']) < 1e-4:
                matched = w
                print(f"  MATCHED: t_split={w['t_split']:.6f}  ba_z={w['ba_z']:+.5f}  "
                      f"a_var_old={w['a_var_old']:.4f}  n_older={w['n_older']}")
                break

        if matched is None and len(windows) > 0:
            print(f"  NO EXACT MATCH. Top candidates:")
            for w in sorted(windows, key=lambda w: np.linalg.norm(w['ba'] - info['ba']))[:3]:
                d = np.linalg.norm(w['ba'] - info['ba'])
                print(f"    t_split={w['t_split']:.6f}  ba_z={w['ba_z']:+.5f}  "
                      f"ba_diff={d:.2e}  a_var_old={w['a_var_old']:.4f}")

        scale_info = compute_first_edge_scale(traj_path, GPS_CSV) if os.path.exists(traj_path) else None
        fe = scale_info['first_edge_mean'] if scale_info else None
        print(f"  first_edge_scale: {fe:.4f}" if fe else "  first_edge_scale: FAILED")

        all_results.append(dict(
            iwt=iwt, success=True,
            logged_ba_z=float(info['ba'][2]),
            logged_ba=info['ba'].tolist(),
            logged_bg=info['bg'].tolist(),
            disparity=info_disp,
            timeoffset_initial=info.get('timeoffset_initial'),
            timeoffset_final=info.get('timeoffset_final'),
            matched_window=matched,
            first_edge_mean=fe,
            first_windows=scale_info.get('first_windows') if scale_info else None,
        ))

    # ---- Summary ----
    print(f"\n\n{'='*90}")
    print(f"  INIT_WINDOW_TIME SWEEP SUMMARY — fly3, start-time=180.0")
    print(f"{'='*90}")

    header = f"  {'iwt':>6s}  {'ba_z':>10s}  {'a_var_old':>10s}  {'to_init':>8s}  {'older_disp':>10s}  {'first_edge':>12s}  {'delta_vs_bl':>12s}"
    print(header)
    print(f"  {'-'*6}  {'-'*10}  {'-'*10}  {'-'*8}  {'-'*10}  {'-'*12}  {'-'*12}")

    # Baseline row
    bl_row = f"  {2.0:>6.1f}  {bl_log['ba'][2]:+10.5f}" if bl_log.get('ba') is not None else f"  {2.0:>6.1f}  {'FAILED':>10s}"
    if matched_bl:
        bl_row += f"  {matched_bl['a_var_old']:>10.4f}"
    else:
        bl_row += f"  {'?':>10s}"
    to_init_bl = bl_log.get('timeoffset_initial')
    bl_row += f"  {str(to_init_bl):>8s}"
    bl_row += f"  {str(bl_disp[0] if bl_disp else '?'):>10s}"
    bl_row += f"  {bl_fe:>12.4f}" if bl_fe else f"  {'?':>12s}"
    bl_row += f"  {'--':>12s}"
    print(bl_row)

    for r in all_results:
        iwt = r['iwt']
        if not r['success']:
            print(f"  {iwt:>6.1f}  {'FAILED':>10s}")
            continue
        ba_str = f"{r['logged_ba_z']:+10.5f}"
        mw = r.get('matched_window')
        av_str = f"{mw['a_var_old']:>10.4f}" if mw else f"{'?':>10s}"
        to_str = f"{str(r['timeoffset_initial']):>8s}" if r['timeoffset_initial'] is not None else f"{'?':>8s}"
        disp_str = f"{r['disparity'][0]:>10.3f}" if r['disparity'] else f"{'?':>10s}"
        fe_str = f"{r['first_edge_mean']:>12.4f}" if r['first_edge_mean'] else f"{'?':>12s}"
        delta = f"{r['first_edge_mean'] - bl_fe:>+12.4f}" if (r['first_edge_mean'] and bl_fe) else f"{'?':>12s}"
        print(f"  {iwt:>6.1f}  {ba_str}  {av_str}  {to_str}  {disp_str}  {fe_str}  {delta}")

    # ---- Assessment ----
    print(f"\n{'='*70}")
    print(f"  ASSESSMENT")
    print(f"{'='*70}")
    successful = [r for r in all_results if r['success']]
    if len(successful) > 0:
        print(f"  {len(successful)}/{len(all_results)} non-baseline runs succeeded")
        for r in successful:
            mw = r.get('matched_window')
            if mw:
                print(f"  iwt={r['iwt']}: older_dur={mw['init_window_time']/2:.1f}s  "
                      f"n_older={mw['n_older']}  a_var_old={mw['a_var_old']:.4f}  "
                      f"ba_z={r['logged_ba_z']:+.5f}")
        # Check if any improved over baseline
        improved = [r for r in successful if r['first_edge_mean'] and bl_fe
                    and abs(r['first_edge_mean'] - 1.0) < abs(bl_fe - 1.0)]
        if improved:
            print(f"\n  Improved first-edge scale (closer to 1.0):")
            for r in improved:
                print(f"    iwt={r['iwt']}: {r['first_edge_mean']:.4f} (baseline={bl_fe:.4f})")
        else:
            print(f"\n  No improvement over baseline first-edge scale")
    else:
        print(f"  All non-baseline runs failed init")

    # Write JSON
    class NpEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.bool_): return bool(obj)
            if isinstance(obj, np.integer): return int(obj)
            if isinstance(obj, np.floating): return float(obj)
            if isinstance(obj, np.ndarray): return obj.tolist()
            return super().default(obj)

    output = dict(
        flight=3, start_time=START_TIME, t0_imu=t0_imu,
        baseline=dict(iwt=2.0,
                       logged_ba_z=float(bl_log['ba'][2]) if bl_log.get('ba') is not None else None,
                       logged_ba=bl_log['ba'].tolist() if bl_log.get('ba') is not None else None,
                       logged_bg=bl_log['bg'].tolist() if bl_log.get('bg') is not None else None,
                       disparity=bl_disp,
                       timeoffset_initial=bl_log.get('timeoffset_initial'),
                       timeoffset_final=bl_log.get('timeoffset_final'),
                       matched_window=matched_bl,
                       first_edge_mean=bl_fe),
        sweep_results=all_results,
    )
    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    json.dump(output, open(OUT_JSON, 'w'), indent=2, cls=NpEncoder)
    print(f"\n[saved] {OUT_JSON}")


if __name__ == '__main__':
    main()
