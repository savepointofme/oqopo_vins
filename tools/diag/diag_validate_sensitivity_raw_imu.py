#!/usr/bin/env python3
"""Validate fly3 sensitivity re-runs (st170, st188) from raw IMU data.

For each successful re-run:
  - Scan raw IMU around effective start to find all valid init windows
  - Match the window whose recomputed ba/bg matches the logged values
  - Report full window details: bounds, a_var, disparities, ba_z, etc.
  - Also report timeoffset initial/final for confound documentation
"""

import json, os, re, sys
import numpy as np

# ---------------------------------------------------------------------------
IMU_CSV = '20260509_fly3/mav0/imu0/data.csv'
BASELINE_LOG = '20260509_fly3/result/baselines_v1/B0_no_gps.log'
SENS_DIR = '20260509_fly3/result/init_window_sensitivity'
OUT_JSON = 'comparison_plots/stage_b_diag/fly3_sensitivity_raw_imu.json'

INIT_WINDOW_TIME = 2.0
INIT_IMU_THRESH = 1.5
GRAVITY = 9.81
SCAN_RANGE = 30.0  # wide scan after effective start

# ---------------------------------------------------------------------------
def load_imu_region(path, t_lo_s, t_hi_s):
    rows = []
    t_lo_ns = t_lo_s * 1e9
    t_hi_ns = t_hi_s * 1e9
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
                a_horiz=a_horiz, gravity_dir=gravity_dir, R_GtoI=R_GtoI)


def scan_windows(imu_region, init_window_time=2.0):
    """Find all windows satisfying StaticInitializer criteria."""
    t = imu_region['t']
    if len(t) < 200: return []
    half = init_window_time / 2.0
    results = []
    t_lo = t[0] + init_window_time
    t_hi = t[-1] - half
    for s in np.arange(t_lo, t_hi, 0.01):
        older = slice_window(imu_region, s - init_window_time, s - half)
        newer = slice_window(imu_region, s - half, s)
        if len(older['t']) < 20 or len(newer['t']) < 20:
            continue
        a_var_old = compute_var(older['a'])
        a_var_new = compute_var(newer['a'])
        if a_var_old < INIT_IMU_THRESH and a_var_new > INIT_IMU_THRESH:
            rec = recompute_from_older(older)
            results.append(dict(
                t_split=s,
                older_start=s - init_window_time,
                older_end=s - half,
                newer_start=s - half,
                newer_end=s,
                a_var_old=float(a_var_old),
                a_var_new=float(a_var_new),
                ba=rec['ba'], bg=rec['bg'], q=rec['q'],
                ba_z=float(rec['ba'][2]),
                n_older=len(older['t']),
                n_newer=len(newer['t']),
                tilt_deg=rec['tilt_deg'],
                a_horiz=rec['a_horiz'],
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
        m = re.search(r'start-time=([\d.]+)s', ln)
        if m: out['start_time'] = float(m.group(1))
    # Get first and last timeoffset
    to_vals = []
    for ln in lines:
        ln = ansi.sub('', ln)
        m = re.search(r'camera-imu timeoffset = ([\-\d.]+)', ln)
        if m: to_vals.append(float(m.group(1)))
    out['timeoffset_initial'] = to_vals[0] if to_vals else None
    out['timeoffset_final'] = to_vals[-1] if to_vals else None
    out['timeoffset_vals'] = to_vals
    return out


def parse_disparities_at_success(path):
    """Find disparities near init success."""
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    with open(path, errors='replace') as f:
        lines = f.readlines()
    last_disp = None
    for i, ln in enumerate(lines):
        ln = ansi.sub('', ln)
        m = re.search(r'disparity is ([\d.]+),([\d.]+) \(([\d.]+) thresh\)', ln)
        if m: last_disp = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
        if 'successful initialization' in ln:
            return last_disp
    return None


# ---------------------------------------------------------------------------
def main():
    # Read first IMU time
    with open(IMU_CSV) as f:
        f.readline()
        t0_imu = float(f.readline().split(',')[0]) * 1e-9
    print(f"t0_imu = {t0_imu:.6f}")

    all_runs = []

    # Process baseline
    print(f"\n{'='*70}\n  BASELINE (start-time=180.0)\n{'='*70}")
    bl = parse_log(BASELINE_LOG)
    bl_disp = parse_disparities_at_success(BASELINE_LOG)
    print(f"  Logged ba: {bl['ba']}  ba_z: {bl['ba'][2]:+.5f}")
    print(f"  Logged bg: {bl['bg']}")
    print(f"  Disparity at success: {bl_disp}")
    print(f"  start_time: {bl.get('start_time')}")
    print(f"  timeoffset initial: {bl.get('timeoffset_initial')}  final: {bl.get('timeoffset_final')}")

    # Scan IMU from effective start
    t_eff_bl = t0_imu + 180.0
    imu_bl = load_imu_region(IMU_CSV, t_eff_bl, t_eff_bl + 60.0)
    print(f"  Scanning IMU from t={t_eff_bl:.3f} to {t_eff_bl+60:.3f}")
    windows_bl = scan_windows(imu_bl)
    print(f"  Found {len(windows_bl)} valid init windows")

    # Match baseline
    matched_bl = None
    for w in windows_bl:
        ba_diff = np.linalg.norm(w['ba'] - bl['ba'])
        bg_diff = np.linalg.norm(w['bg'] - bl['bg'])
        if ba_diff < 1e-4 and bg_diff < 1e-4:
            matched_bl = w
            print(f"  MATCHED window: t_split={w['t_split']:.6f}  ba_z={w['ba_z']:+.5f}  "
                  f"a_var_old={w['a_var_old']:.4f}  a_var_new={w['a_var_new']:.4f}")
            break
    if matched_bl is None:
        print(f"  NO MATCH FOUND for baseline!")

    all_runs.append(dict(name='baseline_st180',
                         start_time=180.0,
                         t_eff=t_eff_bl,
                         logged_ba_z=float(bl['ba'][2]),
                         logged_ba=bl['ba'].tolist(),
                         logged_bg=bl['bg'].tolist(),
                         disparity=bl_disp,
                         timeoffset_initial=bl.get('timeoffset_initial'),
                         timeoffset_final=bl.get('timeoffset_final'),
                         matched_window=matched_bl))

    # Process sensitivity runs
    for st_val in [170.0, 188.0]:
        log_path = os.path.join(SENS_DIR, f'B0_st{int(st_val)}.log')
        if not os.path.exists(log_path):
            print(f"\n  SKIP: {log_path} not found")
            continue

        print(f"\n{'='*70}\n  st{int(st_val)} (start-time={st_val})\n{'='*70}")
        info = parse_log(log_path)
        info_disp = parse_disparities_at_success(log_path)

        if info.get('ba') is None:
            print(f"  INIT FAILED")
            all_runs.append(dict(name=f'st{int(st_val)}', start_time=st_val,
                                 t_eff=t0_imu + st_val, success=False))
            continue

        print(f"  Logged ba: {info['ba']}  ba_z: {info['ba'][2]:+.5f}")
        print(f"  Logged bg: {info['bg']}")
        print(f"  Disparity at success: {info_disp}")
        print(f"  timeoffset initial: {info.get('timeoffset_initial')}  final: {info.get('timeoffset_final')}")

        t_eff = t0_imu + st_val
        imu = load_imu_region(IMU_CSV, t_eff, t_eff + 60.0)
        print(f"  Scanning IMU from t={t_eff:.3f} to {t_eff+60:.3f}")
        windows = scan_windows(imu)
        print(f"  Found {len(windows)} valid init windows")

        matched = None
        for w in windows:
            ba_diff = np.linalg.norm(w['ba'] - info['ba'])
            bg_diff = np.linalg.norm(w['bg'] - info['bg'])
            if ba_diff < 1e-4 and bg_diff < 1e-4:
                matched = w
                print(f"  MATCHED window: t_split={w['t_split']:.6f}  ba_z={w['ba_z']:+.5f}  "
                      f"a_var_old={w['a_var_old']:.4f}  a_var_new={w['a_var_new']:.4f}")
                break

        if matched is None:
            print(f"  NO MATCH! Listing top candidates:")
            for w in sorted(windows, key=lambda w: np.linalg.norm(w['ba'] - info['ba']))[:3]:
                d = np.linalg.norm(w['ba'] - info['ba'])
                print(f"    t_split={w['t_split']:.6f}  ba_z={w['ba_z']:+.5f}  "
                      f"ba_diff={d:.2e}  a_var_old={w['a_var_old']:.4f}")

        all_runs.append(dict(name=f'st{int(st_val)}',
                             start_time=st_val,
                             t_eff=t_eff,
                             success=True,
                             logged_ba_z=float(info['ba'][2]),
                             logged_ba=info['ba'].tolist(),
                             logged_bg=info['bg'].tolist(),
                             disparity=info_disp,
                             timeoffset_initial=info.get('timeoffset_initial'),
                             timeoffset_final=info.get('timeoffset_final'),
                             matched_window=matched))

    # ---- Summary ----
    print(f"\n\n{'='*90}")
    print(f"  RAW-IMU VALIDATION SUMMARY")
    print(f"{'='*90}")

    for r in all_runs:
        name = r['name']
        if not r.get('success'):
            print(f"  {name}: INIT FAILED")
            continue
        mw = r.get('matched_window')
        if mw is None:
            print(f"  {name}: NO RAW-IMU MATCH")
            continue

        print(f"\n  {name}:")
        print(f"    IMU window:")
        print(f"      older: ({mw['older_start']:.6f}, {mw['older_end']:.6f}]  n={mw['n_older']}")
        print(f"      newer: ({mw['newer_start']:.6f}, {mw['newer_end']:.6f}]  n={mw['n_newer']}")
        print(f"    a_var_old={mw['a_var_old']:.4f}  a_var_new={mw['a_var_new']:.4f}")
        print(f"    recomputed ba_z = {mw['ba_z']:+.5f}  (logged = {r['logged_ba_z']:+.5f})")
        print(f"    recomputed bg = [{mw['bg'][0]:+.5f},{mw['bg'][1]:+.5f},{mw['bg'][2]:+.5f}]")
        print(f"    tilt={mw['tilt_deg']:.1f}deg  a_horiz={mw['a_horiz']:.4f}")
        print(f"    logged disparity: {r['disparity']}")
        print(f"    timeoffset: initial={r['timeoffset_initial']}  final={r['timeoffset_final']}")

        # Compare with baseline
        if name != 'baseline_st180':
            bl_w = all_runs[0].get('matched_window')
            if bl_w:
                dt_split = mw['t_split'] - bl_w['t_split']
                print(f"    vs baseline: dt_split={dt_split:+.3f}s  d_ba_z={mw['ba_z'] - bl_w['ba_z']:+.5f}")

    # Document timeoffset confound
    print(f"\n\n{'='*70}")
    print(f"  TIMEOFFSET CONFOUND CHECK")
    print(f"{'='*70}")
    for r in all_runs:
        if not r.get('success'): continue
        print(f"  {r['name']}: to_initial={r['timeoffset_initial']}  to_final={r['timeoffset_final']}")
    print(f"\n  Note: timeoffset differs between baseline ({all_runs[0]['timeoffset_initial']})")
    print(f"  and st170/st188. The scale change may be partially mediated")
    print(f"  through timeoffset differences, not purely through ba_z.")

    # Write JSON
    class NpEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.bool_): return bool(obj)
            if isinstance(obj, np.integer): return int(obj)
            if isinstance(obj, np.floating): return float(obj)
            if isinstance(obj, np.ndarray): return obj.tolist()
            return super().default(obj)

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    json.dump(all_runs, open(OUT_JSON, 'w'), indent=2, cls=NpEncoder)
    print(f"\n[saved] {OUT_JSON}")


if __name__ == '__main__':
    main()
