#!/usr/bin/env python3
"""INIT_WINDOW_AUDIT: Recompute StaticInitializer quantities from raw IMU data.

For each flight, scan raw IMU around the known init time to find the exact
2-second window that matches the StaticInitializer criteria:
  - older half (1s): a_var < init_imu_thresh (1.5)
  - newer half (1s): a_var > init_imu_thresh (jerk detected)

Then recompute init quantities from the older half and compare with logs.
"""

import json, os, re, sys
import numpy as np

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

# ---------------------------------------------------------------------------
FLIGHTS = [
    dict(flight=1,
         imu='20260509_fly1/mav0/imu0/data.csv',
         log='20260509_fly1/result/baselines_v1/B0_no_gps.log'),
    dict(flight=2,
         imu='20260509_fly2/mav0/imu0/data.csv',
         log='20260509_fly2/result/baselines_v1/B0_no_gps.log'),
    dict(flight=3,
         imu='20260509_fly3/mav0/imu0/data.csv',
         log='20260509_fly3/result/baselines_v1/B0_no_gps.log'),
    dict(flight=4,
         imu='20260509_fly4/mav0/imu0/data.csv',
         log='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.log'),
]

HT_JSON = 'comparison_plots/hover_takeoff_scale/summary.json'
SW_JSON = 'comparison_plots/sliding_window_scale/summary.json'
OUT_DIR = 'comparison_plots/stage_b_diag'
os.makedirs(OUT_DIR, exist_ok=True)

INIT_WINDOW_TIME = 2.0
INIT_IMU_THRESH = 1.5
GRAVITY_MAG = 9.81
SCAN_RANGE = 6.0   # seconds around t_init_abs to scan for the window

# ---------------------------------------------------------------------------
# IMU loader — loads a wider region, returns arrays
# ---------------------------------------------------------------------------

def load_imu_region(path, t_mid_s, half_width_s=5.0):
    """Load IMU samples in [t_mid - half_width, t_mid + half_width]."""
    t_lo = (t_mid_s - half_width_s) * 1e9
    t_hi = (t_mid_s + half_width_s) * 1e9
    ts, wx, wy, wz, ax, ay, az = [], [], [], [], [], [], []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            ps = ln.split(',')
            try:
                t_ns = float(ps[0])
            except ValueError:
                continue
            if t_ns < t_lo:
                continue
            if t_ns >= t_hi:
                break
            ts.append(t_ns * 1e-9)
            wx.append(float(ps[1])); wy.append(float(ps[2])); wz.append(float(ps[3]))
            ax.append(float(ps[4])); ay.append(float(ps[5])); az.append(float(ps[6]))
    return dict(
        t=np.array(ts),
        w=np.column_stack([wx, wy, wz]) if ts else np.empty((0, 3)),
        a=np.column_stack([ax, ay, az]) if ts else np.empty((0, 3)),
    )


def slice_window(imu, t_lo_s, t_hi_s):
    """Extract IMU samples with timestamps in (t_lo_s, t_hi_s]."""
    mask = (imu['t'] > t_lo_s) & (imu['t'] <= t_hi_s)
    return dict(t=imu['t'][mask], w=imu['w'][mask], a=imu['a'][mask])


def compute_var(a_samples):
    """Compute same variance metric as StaticInitializer: sqrt(1/(N-1) * sum||a_i - a_avg||²)."""
    n = len(a_samples)
    if n < 2:
        return float('inf')
    a_avg = a_samples.mean(axis=0)
    s = sum((ai - a_avg).dot(ai - a_avg) for ai in a_samples)
    return float(np.sqrt(s / (n - 1)))


# ---------------------------------------------------------------------------
# Log parsing
# ---------------------------------------------------------------------------

def parse_init_log(path):
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    out = {}
    with open(path, errors='replace') as f:
        for ln in f:
            ln = ansi.sub('', ln)
            m = re.search(r'\[init\]: orientation = ([-\d.]+), ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
            if m: out['q_GtoI'] = np.array([float(m.group(i)) for i in range(1, 5)])
            m = re.search(r'\[init\]: bias gyro = ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
            if m: out['bg'] = np.array([float(m.group(i)) for i in range(1, 4)])
            m = re.search(r'\[init\]: bias accel = ([-\d.]+), ([-\d.]+), ([-\d.]+)', ln)
            if m: out['ba'] = np.array([float(m.group(i)) for i in range(1, 4)])
    return out


# ---------------------------------------------------------------------------
# Quaternion math (JPL convention, w-last)
# ---------------------------------------------------------------------------

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
    qw = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if qw > 1e-8:
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
    else:
        qx = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
        qy = (R[1, 0] + R[0, 1]) / (4.0 * max(qx, 1e-12))
        qz = (R[2, 0] + R[0, 2]) / (4.0 * max(qx, 1e-12))
        qw = (R[2, 1] - R[1, 2]) / (4.0 * max(qx, 1e-12))
    return np.array([qx, qy, qz, qw])


# ---------------------------------------------------------------------------
# Window scanner
# ---------------------------------------------------------------------------

def find_init_window(imu_region, log_ba, log_bg, t_center):
    """Scan IMU data to find the 2s window matching StaticInitializer criteria.

    The init timestamp from code is: window_2to1.at(window_2to1.size()-1).timestamp
    which is the LAST IMU sample in the older half = the split point between older and newer.

    We scan for the split point 's' such that:
    - older half = (s - 1.0, s] has a_var < 1.5
    - newer half = (s, s + 1.0] has a_var > 1.5

    Then verify the recomputed ba/bg match the log.
    """
    t = imu_region['t']
    if len(t) < 200:
        return None

    best = None
    best_score = float('inf')

    # Scan with step = 0.01s (~1 IMU sample at 200Hz)
    t_lo = t_center - SCAN_RANGE
    t_hi = t_center + SCAN_RANGE

    for split_t in np.arange(t_lo + 1.1, t_hi - 1.1, 0.01):
        old = slice_window(imu_region, split_t - 1.0, split_t)
        new = slice_window(imu_region, split_t, split_t + 1.0)
        if len(old['a']) < 10 or len(new['a']) < 10:
            continue

        var_old = compute_var(old['a'])
        var_new = compute_var(new['a'])

        # Must satisfy: var_old < 1.5 AND var_new > 1.5
        if not (var_old < INIT_IMU_THRESH and var_new > INIT_IMU_THRESH):
            continue

        # Compute init quantities from older half
        a_avg = old['a'].mean(axis=0)
        w_avg = old['w'].mean(axis=0)
        z_axis = a_avg / np.linalg.norm(a_avg)
        R = gram_schmidt(z_axis)
        gravity_inG = np.array([0.0, 0.0, GRAVITY_MAG])
        ba = a_avg - R @ gravity_inG
        bg = w_avg

        # Score: how well does this match the logged values?
        score = (np.linalg.norm(ba - log_ba) * 100
                 + np.linalg.norm(bg - log_bg) * 100
                 + var_old * 0.1)
        if score < best_score:
            best_score = score
            best = dict(split_t=split_t, old=old, new=new,
                        var_old=var_old, var_new=var_new,
                        a_avg=a_avg, w_avg=w_avg, ba=ba, bg=bg,
                        R=R, z_axis=z_axis, score=score)

    return best


# ---------------------------------------------------------------------------
# Full recomputation + characterization
# ---------------------------------------------------------------------------

def characterize(imu_old, imu_new, a_avg, w_avg, R, ba, bg):
    n_old = len(imu_old['a']); n_new = len(imu_new['a'])

    a_old = imu_old['a']; w_old = imu_old['w']
    a_new = imu_new['a']; w_new = imu_new['w']

    var_old = compute_var(a_old)
    w_var_old = 0.0
    if n_old >= 2:
        w_var_old = float(np.sqrt(sum((wi - w_avg).dot(wi - w_avg) for wi in w_old) / (n_old - 1)))

    var_new = compute_var(a_new)
    w_var_new = 0.0
    a_avg_new = a_new.mean(axis=0)
    w_avg_new = w_new.mean(axis=0)
    if n_new >= 2:
        w_var_new = float(np.sqrt(sum((wi - w_avg_new).dot(wi - w_avg_new) for wi in w_new) / (n_new - 1)))

    at_norm = np.linalg.norm(a_old, axis=1)
    wt_norm = np.linalg.norm(w_old, axis=1)

    z_axis = a_avg / np.linalg.norm(a_avg)
    a_horiz_old = float(np.linalg.norm(a_avg - z_axis * np.dot(a_avg, z_axis)))

    # q_GtoI from R
    q_GtoI = rot2quat(R)

    # Tilt
    tilt_cos = abs(z_axis[2])
    tilt_deg = float(np.arccos(np.clip(tilt_cos, 0, 1)) * 180 / np.pi)

    # Roll/pitch from R_GtoI = R^T (since R = [x_G_in_I | y_G_in_I | z_G_in_I])
    # R_GtoI = R.T
    R_GI = R.T
    roll  = float(np.arctan2(R_GI[2, 1], R_GI[2, 2]) * 180 / np.pi)
    pitch = float(-np.arcsin(np.clip(R_GI[2, 0], -1, 1)) * 180 / np.pi)

    return dict(
        n_old=n_old, n_new=n_new,
        a_avg_old=a_avg, w_avg_old=w_avg,
        a_var_old=var_old, w_var_old=w_var_old,
        a_norm_old_mean=float(at_norm.mean()), a_norm_old_std=float(at_norm.std()),
        w_norm_old_mean=float(wt_norm.mean()), w_norm_old_std=float(wt_norm.std()),
        a_avg_new=a_avg_new, w_avg_new=w_avg_new,
        a_var_new=var_new, w_var_new=w_var_new,
        a_norm_new_mean=float(np.linalg.norm(a_new, axis=1).mean()),
        a_norm_new_std=float(np.linalg.norm(a_new, axis=1).std()),
        w_norm_new_mean=float(np.linalg.norm(w_new, axis=1).mean()),
        w_norm_new_std=float(np.linalg.norm(w_new, axis=1).std()),
        q_GtoI=q_GtoI, R_GtoI=R,
        bg=bg, ba=ba,
        gravity_dir=z_axis,
        tilt_deg=tilt_deg, roll=roll, pitch=pitch,
        a_horiz_old=a_horiz_old,
        a_norm_old_vs_g=float(at_norm.mean() - GRAVITY_MAG),
    )


def classify_contamination(rec, older_disp):
    reasons = []
    score = 0
    if older_disp is not None:
        if older_disp < 0.1: pass
        elif older_disp < 0.35: score += 1; reasons.append(f'older_disp={older_disp:.3f}')
        else: score += 2; reasons.append(f'older_disp={older_disp:.3f} (high)')
    av = rec['a_var_old']
    if av > 1.0: score += 2; reasons.append(f'a_var_old={av:.3f} (large)')
    elif av > 0.3: score += 1; reasons.append(f'a_var_old={av:.3f} (moderate)')
    ang = abs(rec['a_norm_old_vs_g'])
    if ang > 1.0: score += 2; reasons.append(f'|a_norm-g|={ang:.2f}')
    elif ang > 0.3: score += 1; reasons.append(f'|a_norm-g|={ang:.2f}')
    wm = rec['w_norm_old_mean']
    if wm > 0.1: score += 2; reasons.append(f'|w|_mean={wm:.4f}')
    elif wm > 0.03: score += 1; reasons.append(f'|w|_mean={wm:.4f}')
    if score <= 1: label = 'clean static'
    elif score <= 3: label = 'mildly contaminated'
    else: label = 'clearly contaminated'
    return label, reasons


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ht_all = json.load(open(HT_JSON))
    sw_all = json.load(open(SW_JSON))
    ht_by_fly = {r['flight']: r for r in ht_all}
    sw_by_fly = {r['flight']: r for r in sw_all}

    results = []

    for cfg in FLIGHTS:
        fly = cfg['flight']
        ht = ht_by_fly[fly]
        sw = sw_by_fly[fly]
        log_vals = parse_init_log(cfg['log'])
        t_center = ht['t_init_abs']  # approximate — scan around this

        older_disp = ht['init_disparity'][0] if ht.get('init_disparity') else None
        newer_disp = ht['init_disparity'][1] if ht.get('init_disparity') else None

        log_ba = log_vals.get('ba')
        log_bg = log_vals.get('bg')
        log_q  = log_vals.get('q_GtoI')

        if log_ba is None or log_bg is None:
            print(f'fly{fly}: missing logged init values, skipping')
            continue

        print(f'\n{"="*90}')
        print(f'  FLIGHT {fly}')
        print(f'{"="*90}')
        print(f'  Scan center (t_init_abs from traj): {t_center:.6f}')
        print(f'  Scanning ±{SCAN_RANGE}s for matching 2s window...')

        # Load IMU region
        imu_region = load_imu_region(cfg['imu'], t_center, SCAN_RANGE + 2.0)

        # Find best window
        best = find_init_window(imu_region, log_ba, log_bg, t_center)

        if best is None:
            print(f'  [FAIL] No window found meeting a_var criteria '
                  f'(old<{INIT_IMU_THRESH}, new>{INIT_IMU_THRESH}) near t_center')
            continue

        split_t = best['split_t']
        rec = characterize(best['old'], best['new'], best['a_avg'], best['w_avg'],
                          best['R'], best['ba'], best['bg'])

        # Compare
        bg_err = np.linalg.norm(best['bg'] - log_bg)
        ba_err = np.linalg.norm(best['ba'] - log_ba)
        if log_q is not None:
            q_err = min(np.linalg.norm(rec['q_GtoI'] - log_q),
                        np.linalg.norm(rec['q_GtoI'] + log_q))
        else:
            q_err = float('nan')

        print(f'  Found window: split={split_t:.6f}  '
              f'var_old={best["var_old"]:.4f}  var_new={best["var_new"]:.4f}  '
              f'score={best["score"]:.4f}')
        print(f'  older_half: [{split_t - 1.0:.6f}, {split_t:.6f}]  n={rec["n_old"]}')
        print(f'  newer_half: [{split_t:.6f}, {split_t + 1.0:.6f}]  n={rec["n_new"]}')
        print(f'  Disparity: older={older_disp:.3f}  newer={newer_disp:.3f}')

        print(f'\n  --- Older half ("static") IMU ---')
        print(f'  a_avg = [{rec["a_avg_old"][0]:+.4f}, {rec["a_avg_old"][1]:+.4f}, {rec["a_avg_old"][2]:+.4f}]')
        print(f'  w_avg = [{rec["w_avg_old"][0]:+.4f}, {rec["w_avg_old"][1]:+.4f}, {rec["w_avg_old"][2]:+.4f}]')
        print(f'  a_var = {rec["a_var_old"]:.4f}  w_var = {rec["w_var_old"]:.4f}')
        print(f'  |a|   = {rec["a_norm_old_mean"]:.4f} ± {rec["a_norm_old_std"]:.4f}')
        print(f'  |w|   = {rec["w_norm_old_mean"]:.4f} ± {rec["w_norm_old_std"]:.4f}')
        print(f'  a_horiz = {rec["a_horiz_old"]:.4f}  ||a||-g = {rec["a_norm_old_vs_g"]:+.4f}')

        print(f'\n  --- Newer half ("motion") IMU ---')
        print(f'  a_var = {rec["a_var_new"]:.4f}  w_var = {rec["w_var_new"]:.4f}')
        print(f'  |a|   = {rec["a_norm_new_mean"]:.4f} ± {rec["a_norm_new_std"]:.4f}')

        print(f'\n  --- Reconstructed vs Logged ---')
        print(f'  bg  recomputed = {best["bg"]}  logged = {log_bg}  err={bg_err:.1e}')
        print(f'  ba  recomputed = {best["ba"]}  logged = {log_ba}  err={ba_err:.1e}')
        if log_q is not None:
            print(f'  q   recomputed = {rec["q_GtoI"]}  logged = {log_q}  err={q_err:.1e}')

        if bg_err > 1e-3 or ba_err > 1e-3:
            print(f'  ⚠ SIGNIFICANT MISMATCH! Check window alignment.')
        else:
            print(f'  ✓ Matches logged values (err < 1e-3)')

        contam_label, contam_reasons = classify_contamination(rec, older_disp)
        first_edge = sw['first_edge_scale_mean']
        scale_err = first_edge - 1.0

        print(f'\n  --- Contamination ---')
        print(f'  Classification: {contam_label}')
        for r in contam_reasons: print(f'    - {r}')

        print(f'\n  --- ba_z sign convention ---')
        _print_ba_z_convention(rec)

        results.append(dict(
            flight=fly,
            t_center=t_center, split_t=split_t,
            t_older_lo=split_t - 1.0, t_older_hi=split_t,
            t_newer_lo=split_t, t_newer_hi=split_t + 1.0,
            older_disp=older_disp, newer_disp=newer_disp,
            n_imu_old=rec['n_old'], n_imu_new=rec['n_new'],
            a_avg_old=rec['a_avg_old'], w_avg_old=rec['w_avg_old'],
            a_var_old=rec['a_var_old'], w_var_old=rec['w_var_old'],
            a_norm_old_mean=rec['a_norm_old_mean'], a_norm_old_std=rec['a_norm_old_std'],
            w_norm_old_mean=rec['w_norm_old_mean'], w_norm_old_std=rec['w_norm_old_std'],
            a_var_new=rec['a_var_new'], w_var_new=rec['w_var_new'],
            a_avg_new=rec['a_avg_new'], w_avg_new=rec['w_avg_new'],
            a_norm_new_mean=rec['a_norm_new_mean'], a_norm_new_std=rec['a_norm_new_std'],
            w_norm_new_mean=rec['w_norm_new_mean'], w_norm_new_std=rec['w_norm_new_std'],
            q_recomp=rec['q_GtoI'], q_log=log_q if log_q is not None else np.zeros(4),
            bg_recomp=best['bg'], bg_log=log_bg,
            ba_recomp=best['ba'], ba_log=log_ba,
            bg_err=bg_err, ba_err=ba_err, q_err=q_err,
            gravity_dir=rec['gravity_dir'],
            tilt_deg=rec['tilt_deg'], roll=rec['roll'], pitch=rec['pitch'],
            a_horiz_old=rec['a_horiz_old'],
            a_norm_old_vs_g=rec['a_norm_old_vs_g'],
            contam_label=contam_label,
            contam_reasons=contam_reasons,
            first_edge_scale=first_edge,
            scale_err=scale_err,
            R_GtoI=rec['R_GtoI'],
        ))

    # ---- Cross-flight summary ----
    if results:
        _print_summary(results)
        _write_audit_md(results)


def _print_ba_z_convention(rec):
    a_avg = rec['a_avg_old']
    R = rec['R_GtoI']
    g_inG = np.array([0.0, 0.0, GRAVITY_MAG])
    g_inI = R @ g_inG
    ba = rec['ba']
    z_axis = rec['gravity_dir']

    print(f'  gravity_inG = (0, 0, +{GRAVITY_MAG})')
    print(f'  gravity_inI = [{g_inI[0]:+.4f}, {g_inI[1]:+.4f}, {g_inI[2]:+.4f}]')
    print(f'  a_avg_old   = [{a_avg[0]:+.4f}, {a_avg[1]:+.4f}, {a_avg[2]:+.4f}]')
    print(f'  ba          = a_avg - grav_inI = [{ba[0]:+.5f}, {ba[1]:+.5f}, {ba[2]:+.5f}]')
    print(f'  IMU z direction in world: [{z_axis[0]:+.3f}, {z_axis[1]:+.3f}, {z_axis[2]:+.3f}]')
    print(f'  If IMU z ≈ world +z (drone inverted): gravity_inI_z ≈ +9.81')
    print(f'  If IMU z ≈ world -z (drone upright):  gravity_inI_z ≈ -9.81')
    print(f'  Actual gravity_inI_z = {g_inI[2]:+.4f}')
    print(f'')
    print(f'  ba_z = {ba[2]:+.5f}: ', end='')
    if abs(ba[2]) < 0.001:
        print(f'effectively zero — no net vertical bias')
    elif ba[2] > 0:
        print(f'EKF thinks IMU measures MORE +z accel than gravity → subtracts positive')
        print(f'  → downward velocity bias → "features appear faster" → OVER-scale expected')
    else:
        print(f'EKF thinks IMU measures LESS +z accel than gravity → subtracts negative')
        print(f'  → upward velocity bias → "features appear slower" → UNDER-scale expected')


def _print_summary(results):
    print(f'\n\n{"="*120}')
    print(f'  CROSS-FLIGHT SUMMARY')
    print(f'{"="*120}')
    hdr = (f'{"fly":>4s} {"older_disp":>10s} {"a_var_old":>10s} {"w_var_old":>10s} '
           f'{"ba_z":>10s} {"bg_norm":>9s} {"tilt":>5s} {"a_horiz":>8s} '
           f'{"contam":>22s} {"scale":>7s} {"err_sign":>8s}')
    print(hdr); print('-' * len(hdr))
    for r in results:
        bg_norm = float(np.linalg.norm(r['bg_recomp']))
        print(f'{r["flight"]:>4d} {r["older_disp"]:>10.3f} {r["a_var_old"]:>10.4f} '
              f'{r["w_var_old"]:>10.4f} {r["ba_recomp"][2]:>+10.5f} {bg_norm:>9.5f} '
              f'{r["tilt_deg"]:>4.1f}° {r["a_horiz_old"]:>8.4f} '
              f'{r["contam_label"]:>22s} {r["first_edge_scale"]:>7.3f} '
              f'{"+" if r["scale_err"] > 0 else "-":>7s}')

    print(f'\n  H1 check — older_disp vs |scale_err|:')
    for r in sorted(results, key=lambda x: abs(x['scale_err'])):
        print(f'    fly{r["flight"]}: older_disp={r["older_disp"]:.3f}  '
              f'a_var={r["a_var_old"]:.4f}  |scale_err|={abs(r["scale_err"]):.3f}  '
              f'contam={r["contam_label"]}')

    print(f'\n  H3 check — ba_z sign vs scale_err sign:')
    for r in results:
        ba_sign = '+' if r['ba_recomp'][2] > 0 else '-'
        sc_sign = '+' if r['scale_err'] > 0 else '-'
        match_mark = '✓' if ba_sign == sc_sign else '✗'
        print(f'    fly{r["flight"]}: ba_z={r["ba_recomp"][2]:+.5f} ({ba_sign})  '
              f'scale_err={r["scale_err"]:+.3f} ({sc_sign})  {match_mark}')

    print(f'\n  Combined H1+H3:')
    for r in results:
        ba_z = r['ba_recomp'][2]
        print(f'    fly{r["flight"]}: disp={r["older_disp"]:.3f}  '
              f'a_var={r["a_var_old"]:.4f}  '
              f'ba_z={ba_z:+.5f}  tilt={r["tilt_deg"]:.1f}°  '
              f'a_horiz={r["a_horiz_old"]:.4f}  '
              f'scale_err={r["scale_err"]:+.3f}  '
              f'{r["contam_label"]}')


def _write_audit_md(results):
    L = []
    def a(s): L.append(s)

    a('# INIT_WINDOW_AUDIT.md')
    a('')
    a('## 1. Exact Init Windows')
    a('')
    a('Window definitions (from StaticInitializer.cpp + InertialInitializer.cpp):')
    a('- **StaticInitializer older half**: `(newesttime - 2.0, newesttime - 1.0]`')
    a('- **StaticInitializer newer half**: `(newesttime - 1.0, newesttime]`')
    a('- **Init timestamp**: end of older half = `newesttime - 1.0`')
    a('- **Disparity older**: camera frames `(newest_cam_time - 2.1, newest_cam_time - 1.0]`')
    a('- **Disparity newer**: camera frames `(newest_cam_time - 1.0, newest_cam_time]`')
    a('')
    a(f'Parameters: `init_window_time={INIT_WINDOW_TIME}s`, `init_imu_thresh={INIT_IMU_THRESH}`, `init_max_disparity=0.7`')
    a('')
    a('| flight | older_half_start | older_half_end (split) | newer_half_end | older_disp | newer_disp | n_IMU_old | n_IMU_new |')
    a('|--------|-----------------|----------------------|---------------|------------|------------|-----------|-----------|')
    for r in results:
        a(f'| {r["flight"]} | {r["t_older_lo"]:.6f} | {r["t_older_hi"]:.6f} | '
          f'{r["t_newer_hi"]:.6f} | {r["older_disp"]:.3f} | {r["newer_disp"]:.3f} | '
          f'{r["n_imu_old"]} | {r["n_imu_new"]} |')
    a('')

    a('## 2. Raw IMU Re-computation')
    a('')
    a('### Older Half (used for gravity/bias estimation)')
    a('')
    a('| flight | a_avg [x,y,z] | w_avg [x,y,z] | a_var | w_var | \\|a\\| mean±std | \\|w\\| mean±std | \\|a\\|-g | a_horiz |')
    a('|--------|--------------|--------------|-------|-------|------------|------------|--------|---------|')
    for r in results:
        av = r['a_avg_old']; wv = r['w_avg_old']
        a(f'| {r["flight"]} | [{av[0]:+.4f},{av[1]:+.4f},{av[2]:+.4f}] | '
          f'[{wv[0]:+.4f},{wv[1]:+.4f},{wv[2]:+.4f}] | {r["a_var_old"]:.4f} | {r["w_var_old"]:.4f} | '
          f'{r["a_norm_old_mean"]:.3f}±{r["a_norm_old_std"]:.3f} | '
          f'{r["w_norm_old_mean"]:.4f}±{r["w_norm_old_std"]:.4f} | '
          f'{r["a_norm_old_vs_g"]:+.3f} | {r["a_horiz_old"]:.4f} |')
    a('')

    a('### Newer Half (jerk detection)')
    a('')
    a('| flight | a_avg [x,y,z] | a_var | w_var | \\|a\\| mean±std |')
    a('|--------|--------------|-------|-------|------------|')
    for r in results:
        av = r['a_avg_new']
        a(f'| {r["flight"]} | [{av[0]:+.4f},{av[1]:+.4f},{av[2]:+.4f}] | '
          f'{r["a_var_new"]:.4f} | {r["w_var_new"]:.4f} | '
          f'{r["a_norm_new_mean"]:.3f}±{r["a_norm_new_std"]:.3f} |')
    a('')

    a('## 3. Comparison with Logged Init Values')
    a('')
    a('| flight | bg recomp | bg logged | err | ba recomp | ba logged | err | q err |')
    a('|--------|----------|-----------|-----|----------|-----------|-----|-------|')
    for r in results:
        a(f'| {r["flight"]} | [{r["bg_recomp"][0]:+.5f},{r["bg_recomp"][1]:+.5f},{r["bg_recomp"][2]:+.5f}] | '
          f'[{r["bg_log"][0]:+.5f},{r["bg_log"][1]:+.5f},{r["bg_log"][2]:+.5f}] | {r["bg_err"]:.1e} | '
          f'[{r["ba_recomp"][0]:+.5f},{r["ba_recomp"][1]:+.5f},{r["ba_recomp"][2]:+.5f}] | '
          f'[{r["ba_log"][0]:+.5f},{r["ba_log"][1]:+.5f},{r["ba_log"][2]:+.5f}] | {r["ba_err"]:.1e} | '
          f'{r["q_err"]:.1e} |')
    a('')

    a('## 4. Static-Window Contamination Score')
    a('')
    a('| flight | older_disp | a_var_old | \\|w\\| mean | \\|a\\|-g | a_horiz | tilt° | classification | reasons |')
    a('|--------|-----------|-----------|----------|--------|---------|-------|----------------|---------|')
    for r in results:
        reasons_str = '; '.join(r['contam_reasons']) if r['contam_reasons'] else '(clean)'
        a(f'| {r["flight"]} | {r["older_disp"]:.3f} | {r["a_var_old"]:.4f} | '
          f'{r["w_norm_old_mean"]:.4f} | {r["a_norm_old_vs_g"]:+.3f} | '
          f'{r["a_horiz_old"]:.4f} | {r["tilt_deg"]:.1f} | '
          f'**{r["contam_label"]}** | {reasons_str} |')
    a('')

    a('## 5. ba_z Sign Convention')
    a('')
    a('```')
    a('ba = a_avg_old - R_GtoI @ (0, 0, +g)')
    a('')
    a('where:')
    a('  a_avg_old = mean raw accelerometer reading during older 1s [IMU frame]')
    a('  R_GtoI    = rotation from global to IMU, estimated from gravity direction')
    a('  g_inG     = (0, 0, +9.81)')
    a('')
    a('Physical interpretation:')
    a('  - If drone is perfectly static and upright: a_avg ≈ R_GtoI @ g_inG → ba ≈ 0')
    a('  - ba_z > 0: IMU measures MORE +z accel than gravity → EKF subtracts positive bias')
    a('    → velocity biased DOWNWARD → features appear faster → OVER-scale')
    a('  - ba_z < 0: IMU measures LESS +z accel than gravity → EKF subtracts negative bias')
    a('    → velocity biased UPWARD → features appear slower → UNDER-scale')
    a('```')
    a('')
    a('| flight | gravity_inI_z | a_avg_z | ba_z | scale_err | predicted_sign | actual_sign | match |')
    a('|--------|--------------|---------|------|-----------|---------------|-------------|-------|')
    for r in results:
        R = r['R_GtoI']
        g_inI_z = float((R @ np.array([0, 0, GRAVITY_MAG]))[2])
        az = float(r['a_avg_old'][2])
        ba_z = float(r['ba_recomp'][2])
        pred = '+' if ba_z > 0 else ('0' if abs(ba_z) < 0.001 else '-')
        actual = '+' if r['scale_err'] > 0 else '-'
        match = '✓' if pred == actual else '✗'
        a(f'| {r["flight"]} | {g_inI_z:+.4f} | {az:+.4f} | {ba_z:+.5f} | '
          f'{r["scale_err"]:+.3f} | {pred} | {actual} | {match} |')
    a('')

    a('## 6. Relation to First-Edge Scale')
    a('')
    a('| flight | older_disp | a_var_old | ba_z | tilt° | a_horiz | contam | first_edge | scale_err |')
    a('|--------|-----------|-----------|------|-------|---------|--------|-----------|-----------|')
    for r in results:
        a(f'| {r["flight"]} | {r["older_disp"]:.3f} | {r["a_var_old"]:.4f} | '
          f'{r["ba_recomp"][2]:+.5f} | {r["tilt_deg"]:.1f} | '
          f'{r["a_horiz_old"]:.4f} | {r["contam_label"]} | '
          f'{r["first_edge_scale"]:.3f} | {r["scale_err"]:+.3f} |')
    a('')

    a('## 7. Conclusions')
    a('')
    a('### Q1: Does older-half contamination correlate with |scale_err|?')
    for r in sorted(results, key=lambda x: abs(x['scale_err'])):
        a(f'- fly{r["flight"]}: |err|={abs(r["scale_err"]):.3f}, '
          f'older_disp={r["older_disp"]:.3f}, a_var={r["a_var_old"]:.4f}, '
          f'{r["contam_label"]}')
    a('')

    a('### Q2: Does ba_init sign correlate with scale error sign?')
    sign_matches = [(r['ba_recomp'][2] > 0) == (r['scale_err'] > 0) for r in results]
    all_match = all(sign_matches) if sign_matches else False
    a(f'Match {sum(sign_matches)}/{len(results)}: **{"YES" if all_match else "NO"}**')
    for i, r in enumerate(results):
        ba_sign = '+' if r['ba_recomp'][2] > 0 else '-'
        sc_sign = '+' if r['scale_err'] > 0 else '-'
        a(f'- fly{r["flight"]}: ba_z={ba_sign}, scale_err={sc_sign}  '
          f'{"✓" if sign_matches[i] else "✗"}')
    a('')

    a('### Q3: Is fly4 still an outlier?')
    fly2_list = [r for r in results if r['flight'] == 2]
    fly4_list = [r for r in results if r['flight'] == 4]
    if fly2_list and fly4_list:
        f2, f4 = fly2_list[0], fly4_list[0]
        a(f'- fly4 older_disp={f4["older_disp"]:.3f} vs fly2={f2["older_disp"]:.3f}')
        a(f'- fly4 a_var_old={f4["a_var_old"]:.4f} vs fly2={f2["a_var_old"]:.4f}')
        a(f'- fly4 a_horiz={f4["a_horiz_old"]:.4f} vs fly2={f2["a_horiz_old"]:.4f}')
        a(f'- fly4 ba_z={f4["ba_recomp"][2]:+.5f} vs fly2={f2["ba_recomp"][2]:+.5f}')
        a(f'- fly4 tilt={f4["tilt_deg"]:.1f}° vs fly2={f2["tilt_deg"]:.1f}°')
        a(f'- fly4 contamination={f4["contam_label"]} vs fly2={f2["contam_label"]}')
    a('')

    a('### Q4: Remaining uncertainty')
    a('- The ba_z ↔ scale sign correlation is observed across available flights, not proven causal.')
    a('- The StaticInitializer uses the older 1s IMU window to estimate gravity direction.')
    a('  Any contamination (motion, vibration, tilt) in that window biases ba_z.')
    a('- Camera-IMU extrinsic (R_ItoC) affects how IMU errors project into feature tracks.')
    a('- These results support H1+H3 as leading hypotheses but do not rule out')
    a('  contributions from vertical-climb observability (H2) or timeoffset (H5).')

    out_path = f'{OUT_DIR}/INIT_WINDOW_AUDIT.md'
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    print(f'\n[saved] {out_path}')


if __name__ == '__main__':
    main()
