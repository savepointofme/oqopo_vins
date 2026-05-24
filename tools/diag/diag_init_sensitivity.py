#!/usr/bin/env python3
"""Process fly3 init-window sensitivity re-runs.

For each re-run (different --start-time):
  - Parse init success from log (ba, bg, disparities)
  - Find exact init window from raw IMU
  - Compute first-edge scale from trajectory + GPS
  - Compare with baseline
"""

import json, os, re, sys, glob
import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
FLIGHT = 3
IMU_CSV = f'20260509_fly{FLIGHT}/mav0/imu0/data.csv'
GPS_CSV = f'20260509_fly{FLIGHT}/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'
BASELINE_DIR = f'20260509_fly{FLIGHT}/result/baselines_v1'
BASELINE_TRAJ = f'{BASELINE_DIR}/B0_no_gps.txt'
BASELINE_LOG = f'{BASELINE_DIR}/B0_no_gps.log'
BASELINE_BIAS = f'{BASELINE_DIR}/B0_no_gps.txt.bias'
SENS_DIR = f'20260509_fly{FLIGHT}/result/init_window_sensitivity'
OUT_JSON = f'comparison_plots/stage_b_diag/fly{FLIGHT}_sensitivity.json'

# StaticInitializer parameters
INIT_WINDOW_TIME = 2.0
INIT_IMU_THRESH = 1.5
GRAVITY = 9.81

# ---------------------------------------------------------------------------
# IMU utilities (from diag_init_window_audit.py)
# ---------------------------------------------------------------------------

def load_imu_region(path, t_lo, t_hi):
    """Load IMU samples in [t_lo, t_hi]."""
    rows = []
    with open(path) as f:
        f.readline()  # skip header
        for ln in f:
            ln = ln.strip()
            if not ln: continue
            ps = ln.split(',')
            try: ts = float(ps[0]) * 1e-9
            except ValueError: continue
            if ts < t_lo: continue
            if ts > t_hi: break
            rows.append([ts] + [float(x) for x in ps[1:7]])
    return np.array(rows)


def compute_var(data):
    """Per-axis variance of Nx3 data."""
    return np.var(data, axis=0).sum()


def gram_schmidt(v):
    """GS orthonormalization of 3x3 matrix columns (v is 3-vector for z-axis)."""
    z = v / np.linalg.norm(v)
    # arbitrary x not parallel to z
    x = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(x, z)) > 0.99:
        x = np.array([0.0, 1.0, 0.0])
    x = x - np.dot(x, z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rot2quat(R):
    """Rotation matrix to JPL quaternion [x,y,z,w]."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return np.array([qx, qy, qz, qw])


def recompute_bias(imu_older):
    """Recompute bg, ba, q_GtoI from older-half IMU data."""
    a_avg = imu_older[:, 4:7].mean(axis=0)
    w_avg = imu_older[:, 1:4].mean(axis=0)
    a_norm = np.linalg.norm(a_avg)
    gravity_dir = a_avg / a_norm  # in IMU frame, points "up" (opposite gravity)
    R_GtoI = gram_schmidt(gravity_dir)
    bg = w_avg
    g_inG = np.array([0.0, 0.0, GRAVITY])
    g_inI = R_GtoI @ g_inG
    ba = a_avg - g_inI
    q_GtoI = rot2quat(R_GtoI)
    a_var = compute_var(imu_older[:, 4:7])
    w_var = compute_var(imu_older[:, 1:4])
    tilt = np.arccos(np.clip(gravity_dir[2], -1, 1))
    a_horiz = np.sqrt(a_avg[0]**2 + a_avg[1]**2)
    return dict(ba=ba, bg=bg, q_GtoI=q_GtoI, a_avg=a_avg, w_avg=w_avg,
                a_var=a_var, w_var=w_var, tilt_rad=tilt, a_horiz=a_horiz)


def find_init_window_from_split(imu_path, t_split, window_time=2.0):
    """Given a split timestamp, extract older and newer halves from raw IMU."""
    older = load_imu_region(imu_path, t_split - window_time, t_split)
    newer = load_imu_region(imu_path, t_split, t_split + window_time / 2)
    # actually newer is (newesttime - 1.0, newesttime], and t_split = newesttime - 1.0
    # so newer = (t_split, t_split + 1.0]
    newer = load_imu_region(imu_path, t_split, t_split + 1.0)
    return older, newer


# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------

def parse_init_from_log(log_path):
    """Extract init ba_z, bg, disparities from B0 log."""
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    result = dict(success=False, disparity=None, ba=None, bg=None,
                  ba_z=None, timeoffset_initial=None,
                  q_GtoI_str=None, success_line=None)

    with open(log_path, errors='replace') as f:
        lines = f.readlines()

    for i, ln in enumerate(lines):
        ln_clean = ansi.sub('', ln)

        # Check for disparity right before success
        m = re.search(r'disparity is ([\d.]+),([\d.]+) \(([\d.]+) thresh\)', ln_clean)
        if m:
            result['disparity'] = (float(m.group(1)), float(m.group(2)), float(m.group(3)))

        # Check for success
        if 'successful initialization' in ln_clean:
            result['success'] = True
            result['success_line'] = i
            result['disparity_at_success'] = result.get('disparity')

            # Look for bias accel in next few lines
            for j in range(i, min(i + 20, len(lines))):
                lj = ansi.sub('', lines[j])
                m_ba = re.search(r'bias accel = ([\-\d.]+), ([\-\d.]+), ([\-\d.]+)', lj)
                if m_ba:
                    result['ba'] = np.array([float(m_ba.group(1)), float(m_ba.group(2)), float(m_ba.group(3))])
                    result['ba_z'] = result['ba'][2]
                    break
                m_bg = re.search(r'bias gyro = ([\-\d.]+), ([\-\d.]+), ([\-\d.]+)', lj)
                if m_bg:
                    result['bg'] = np.array([float(m_bg.group(1)), float(m_bg.group(2)), float(m_bg.group(3))])

            # Get q_GtoI
            for j in range(i, min(i + 5, len(lines))):
                lj = ansi.sub('', lines[j])
                m_q = re.search(r'orientation = ([\-\d.]+), ([\-\d.]+), ([\-\d.]+), ([\-\d.]+)', lj)
                if m_q:
                    result['q_GtoI_str'] = f"{float(m_q.group(1)):.4f},{float(m_q.group(2)):.4f},{float(m_q.group(3)):.4f},{float(m_q.group(4)):.4f}"
                    break

            # Get initial timeoffset
            for j in range(i, min(i + 20, len(lines))):
                lj = ansi.sub('', lines[j])
                m_to = re.search(r'camera-imu timeoffset = ([\-\d.]+)', lj)
                if m_to:
                    result['timeoffset_initial'] = float(m_to.group(1))
                    break

            break

    return result


def parse_start_time_from_log(log_path):
    """Extract --start-time value from log."""
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    with open(log_path, errors='replace') as f:
        for ln in f:
            ln_clean = ansi.sub('', ln)
            m = re.search(r'start-time=([\d.]+)s', ln_clean)
            if m:
                return float(m.group(1))
    return None


# ---------------------------------------------------------------------------
# GPS / trajectory utilities
# ---------------------------------------------------------------------------

def load_tum_traj(path):
    """Load TUM-format trajectory: ts tx ty tz qx qy qz qw"""
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


# ---------------------------------------------------------------------------
# First-edge scale computation
# ---------------------------------------------------------------------------

def compute_first_edge_scale(traj_tum, gps_csv, win_s=30.0, step_s=5.0,
                              motion_thresh_m=5.0, edge_dur_s=80.0):
    """Compute first-edge scale from trajectory and GPS."""
    traj = load_tum_traj(traj_tum)
    gps_raw = load_gps_llz(gps_csv)
    if traj.size == 0 or gps_raw.size == 0:
        return None, None

    gps = gps_to_enu(gps_raw)

    # Overlap time range
    t_lo = max(traj[0, 0], gps[0, 0])
    t_hi = min(traj[-1, 0], gps[-1, 0])
    if t_hi <= t_lo:
        return None, None

    # Interpolate GPS to traj timestamps
    gps_t = gps[:, 0]; gps_xy = gps[:, 1:3]
    gps_x_interp = np.interp(traj[:, 0], gps_t, gps_xy[:, 0])
    gps_y_interp = np.interp(traj[:, 0], gps_t, gps_xy[:, 1])

    # Trajectory XY
    traj_xy = traj[:, 1:3]
    traj_t = traj[:, 0]

    t0 = traj_t[0]

    # Find takeoff: first time cumulative GPS XY path > motion_thresh
    gps_d = np.sqrt(np.diff(gps_x_interp)**2 + np.diff(gps_y_interp)**2)
    gps_cum = np.cumsum(np.concatenate([[0], gps_d]))
    tk_mask = gps_cum >= motion_thresh_m
    if not tk_mask.any():
        return None, None
    t_takeoff = traj_t[np.where(tk_mask)[0][0]]
    t_takeoff_rel = t_takeoff - t0

    # Sliding windows from takeoff
    centers = []
    ratios = []
    motion_mask = []

    c = max(t_takeoff_rel, win_s / 2)
    while c + win_s / 2 <= traj_t[-1] - t0:
        t_lo_w = c - win_s / 2
        t_hi_w = c + win_s / 2
        mask = (traj_t >= t0 + t_lo_w) & (traj_t <= t0 + t_hi_w)
        if mask.sum() < 5:
            centers.append(c)
            ratios.append(np.nan)
            motion_mask.append(False)
            c += step_s
            continue

        b0_d = np.sqrt(np.diff(traj_xy[mask, 0])**2 + np.diff(traj_xy[mask, 1])**2)
        b0_path = np.sum(b0_d)
        gps_d_w = np.sqrt(np.diff(gps_x_interp[mask])**2 + np.diff(gps_y_interp[mask])**2)
        gps_path_w = np.sum(gps_d_w)

        centers.append(c)
        ratios.append(b0_path / max(gps_path_w, 1e-3))
        motion_mask.append(gps_path_w >= motion_thresh_m)
        c += step_s

    centers = np.array(centers)
    ratios = np.array(ratios)
    motion_mask = np.array(motion_mask, dtype=bool)

    # First edge: first 80s of horizontal motion after takeoff
    edge_end = t_takeoff_rel + edge_dur_s
    edge_mask = (centers >= t_takeoff_rel) & (centers <= edge_end) & motion_mask
    if edge_mask.sum() > 0:
        first_edge_mean = float(np.mean(ratios[edge_mask]))
    else:
        first_edge_mean = None

    # Also get the first 5 motion-masked windows
    post_tk_mask = (centers >= t_takeoff_rel) & motion_mask
    first_windows = []
    if post_tk_mask.sum() > 0:
        idxs = np.where(post_tk_mask)[0][:5]
        for j in idxs:
            first_windows.append({
                'center': float(centers[j]),
                'ratio': float(ratios[j]),
                't_takeoff_rel': float(t_takeoff_rel),
            })

    return dict(
        first_edge_mean=first_edge_mean,
        first_windows=first_windows,
        t_takeoff_rel=float(t_takeoff_rel),
        centers=centers.tolist(),
        ratios=[float(x) if not np.isnan(x) else None for x in ratios],
        motion_mask=motion_mask.tolist(),
    ), t_takeoff_rel


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    all_results = []

    # Process baseline first
    print("=" * 70)
    print("Processing BASELINE (start-time=180.0)")
    print("=" * 70)

    baseline_init = parse_init_from_log(BASELINE_LOG)
    baseline_st = parse_start_time_from_log(BASELINE_LOG)
    print(f"  start-time: {baseline_st}")
    print(f"  success: {baseline_init['success']}")
    print(f"  ba: {baseline_init.get('ba')}")
    print(f"  ba_z: {baseline_init.get('ba_z'):+.5f}" if baseline_init.get('ba_z') else "  ba_z: None")
    print(f"  disparity: {baseline_init.get('disparity_at_success')}")
    print(f"  timeoffset_initial: {baseline_init.get('timeoffset_initial')}")

    # Compute first-edge scale for baseline
    baseline_scale, baseline_tk = compute_first_edge_scale(BASELINE_TRAJ, GPS_CSV)
    if baseline_scale:
        print(f"  first_edge_scale: {baseline_scale['first_edge_mean']:.4f}" if baseline_scale['first_edge_mean'] else "  first_edge_scale: None")
        print(f"  t_takeoff_rel: {baseline_scale.get('t_takeoff_rel')}")
        print(f"  first_windows: {baseline_scale.get('first_windows')}")

    # Now process each re-run
    pattern = os.path.join(SENS_DIR, 'B0_st*.log')
    log_files = sorted(glob.glob(pattern))
    print(f"\nFound {len(log_files)} sensitivity re-run logs")

    for log_path in log_files:
        fn = os.path.basename(log_path)
        print(f"\n{'='*70}")
        print(f"Processing {fn}")
        print(f"{'='*70}")

        init_info = parse_init_from_log(log_path)
        st = parse_start_time_from_log(log_path)

        print(f"  start-time: {st}")
        print(f"  success: {init_info['success']}")

        if not init_info['success']:
            print(f"  ** INIT FAILED **")
            all_results.append(dict(
                start_time=st, log=fn, success=False,
                ba_z=None, disparity=None, init_same_as_baseline=None,
                first_edge_mean=None,
            ))
            continue

        print(f"  ba: {init_info.get('ba')}")
        ba_z_val = init_info.get('ba_z')
        print(f"  ba_z: {ba_z_val:+.5f}" if ba_z_val is not None else "  ba_z: None")
        print(f"  disparity: {init_info.get('disparity_at_success')}")
        print(f"  timeoffset_initial: {init_info.get('timeoffset_initial')}")

        # Check if init is same as baseline
        init_same = False
        if baseline_init.get('ba') is not None and init_info.get('ba') is not None:
            ba_diff = np.linalg.norm(baseline_init['ba'] - init_info['ba'])
            init_same = ba_diff < 1e-5
        print(f"  init_same_as_baseline: {init_same} (ba diff={ba_diff:.2e})" if 'ba_diff' in dir() else f"  init_same_as_baseline: {init_same}")

        # Compute first-edge scale
        traj_path = log_path.replace('.log', '.txt')
        if os.path.exists(traj_path):
            scale_info, tk = compute_first_edge_scale(traj_path, GPS_CSV)
            if scale_info:
                fem = scale_info['first_edge_mean']
                print(f"  first_edge_scale: {fem:.4f}" if fem else "  first_edge_scale: None")
                print(f"  t_takeoff_rel: {scale_info.get('t_takeoff_rel')}")
                print(f"  first_windows: {scale_info.get('first_windows')}")
            else:
                fem = None
                print(f"  first_edge_scale: FAILED (no overlap)")
        else:
            fem = None
            print(f"  trajectory file not found: {traj_path}")

        info_str = "SAME_AS_BASELINE" if init_same else "DIFFERENT"

        all_results.append(dict(
            start_time=st, log=fn, success=True,
            ba_z=float(ba_z_val) if ba_z_val is not None else None,
            ba=list(init_info['ba']) if init_info.get('ba') is not None else None,
            bg=list(init_info['bg']) if init_info.get('bg') is not None else None,
            disparity=list(init_info.get('disparity_at_success', [])),
            timeoffset_initial=init_info.get('timeoffset_initial'),
            init_same_as_baseline=init_same,
            first_edge_mean=fem,
            first_windows=scale_info.get('first_windows') if scale_info else None,
            t_takeoff_rel=scale_info.get('t_takeoff_rel') if scale_info else None,
            init_window_info=info_str,
        ))

    # Summary table
    print(f"\n\n{'='*90}")
    print(f"  SUMMARY: fly3 init-window sensitivity")
    print(f"{'='*90}")
    print(f"  {'start_time':>10s}  {'ba_z':>10s}  {'init_vs_bl':>14s}  {'first_edge':>12s}  {'delta':>8s}")
    print(f"  {'-'*10}  {'-'*10}  {'-'*14}  {'-'*12}  {'-'*8}")

    bl_ba_z = baseline_init.get('ba_z')
    bl_fe = baseline_scale['first_edge_mean'] if baseline_scale else None

    for r in all_results:
        st_str = f"{r['start_time']:.1f}" if r['start_time'] else "?"
        if r['success']:
            ba_str = f"{r['ba_z']:+.5f}" if r['ba_z'] is not None else "?"
            fe_str = f"{r['first_edge_mean']:.4f}" if r['first_edge_mean'] else "?"
            if r['init_same_as_baseline']:
                info_str = "SAME_AS_BL"
            else:
                info_str = "DIFFERENT"
            delta = ""
            if r['first_edge_mean'] and bl_fe:
                delta = f"{r['first_edge_mean'] - bl_fe:+.4f}"
        else:
            ba_str = "FAILED"
            fe_str = "FAILED"
            info_str = "N/A"
            delta = ""
        print(f"  {st_str:>10s}  {ba_str:>10s}  {info_str:>14s}  {fe_str:>12s}  {delta:>8s}")

    # Write JSON
    output = dict(
        flight=FLIGHT,
        baseline=dict(
            start_time=baseline_st,
            ba_z=float(bl_ba_z) if bl_ba_z is not None else None,
            ba=list(baseline_init['ba']) if baseline_init.get('ba') is not None else None,
            bg=list(baseline_init['bg']) if baseline_init.get('bg') is not None else None,
            disparity=list(baseline_init.get('disparity_at_success', [])),
            first_edge_mean=bl_fe,
            first_windows=baseline_scale.get('first_windows') if baseline_scale else None,
        ),
        reruns=all_results,
    )
    class NpEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, (np.bool_,)): return bool(obj)
            if isinstance(obj, (np.integer,)): return int(obj)
            if isinstance(obj, (np.floating,)): return float(obj)
            if isinstance(obj, np.ndarray): return obj.tolist()
            return super().default(obj)
    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    json.dump(output, open(OUT_JSON, 'w'), indent=2, cls=NpEncoder)
    print(f"\n[saved] {OUT_JSON}")

    # Hypothesis assessment
    print(f"\n{'='*70}")
    print(f"  HYPOTHESIS ASSESSMENT")
    print(f"{'='*70}")

    different_runs = [r for r in all_results if r['success'] and not r['init_same_as_baseline']]
    same_runs = [r for r in all_results if r['success'] and r['init_same_as_baseline']]
    failed_runs = [r for r in all_results if not r['success']]

    print(f"  Runs with DIFFERENT init window: {len(different_runs)}")
    print(f"  Runs with SAME init window (not informative): {len(same_runs)}")
    print(f"  Runs with FAILED init: {len(failed_runs)}")

    if len(different_runs) > 0:
        print(f"\n  Different-init results:")
        for r in different_runs:
            print(f"    st={r['start_time']:.1f}: ba_z={r['ba_z']:+.5f}  first_edge={r['first_edge_mean']:.4f}")

        # Check for ba_z sign change
        bl_sign = np.sign(bl_ba_z) if bl_ba_z else 0
        any_sign_change = any(np.sign(r['ba_z']) != bl_sign for r in different_runs if r['ba_z'] is not None)
        if any_sign_change:
            print(f"\n  ** ba_z SIGN CHANGED — strong test of direction hypothesis **")
        else:
            print(f"\n  ba_z sign unchanged from baseline {bl_sign:+}")

        # Check for scale change
        if bl_fe:
            max_delta = max(abs(r['first_edge_mean'] - bl_fe) for r in different_runs if r['first_edge_mean'] is not None)
            print(f"  Max |delta first_edge|: {max_delta:.4f}")
            if max_delta > 0.03:
                print(f"  ** First-edge scale changed measurably **")
            else:
                print(f"  First-edge scale change is small (<0.03)")
    else:
        print(f"\n  No runs produced a different init window.")
        print(f"  The init window appears insensitive to --start-time changes")
        print(f"  in the tested range, or all attempted shifts failed.")

    print(f"\n[done]")


if __name__ == '__main__':
    main()
