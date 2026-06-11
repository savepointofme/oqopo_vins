#!/usr/bin/env python3
"""
lk_flow_calib_warp.py — offline calibration-warp LK validation.

For candidate extrinsic perturbations (roll, pitch), generate a static
undistort-rectify remap, warp every frame, mask invalid border pixels,
and re-run the raw LK yaw diagnostic on the warped images.

Outputs a comparison table of LK-minus-gyro/GPS metrics per candidate.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yaw_drift_forensics import read_gps_enu, unwrap_with_gaps, boxcar


# -----------------------------------------------------------------------------
# Same ground radtan calibration as lk_flow_yaw_diag.py
K = np.array([[367.4567959235411, 0, 328.6368136147717],
              [0, 367.1397460471364, 233.58213701688157],
              [0, 0, 1.0]])
D = np.array([-0.059513850515088895, 0.034926517536683285,
              -0.0027539606930595275, 7.51756448944016e-05])
W, H = 640, 480


# -----------------------------------------------------------------------------
def load_cam_index(dataset):
    ts, files = [], []
    root = os.path.join(dataset, 'cam0')
    for line in open(os.path.join(root, 'data.csv')):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        c = s.split(',')
        if len(c) < 5:
            continue
        ts.append(float(c[0]))
        files.append(os.path.join(root, 'data', c[4]))
    return np.array(ts), files


def load_imu_wz(dataset):
    t, wz = [], []
    for line in open(os.path.join(dataset, 'imu0', 'data.csv')):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        c = s.split(',')
        if len(c) < 11:
            continue
        try:
            t.append(float(c[0]))
            wz.append(float(c[7]))
        except ValueError:
            continue
    return np.array(t), np.array(wz)


def rot_from_2x2(M):
    u, _, vt = np.linalg.svd(M)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def build_R(roll_deg, pitch_deg, yaw_deg=0.0):
    """Return R = Rz(yaw) @ Ry(pitch) @ Rx(roll)  (perturb_calib order)."""
    y, p, r = math.radians(yaw_deg), math.radians(pitch_deg), math.radians(roll_deg)
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


# -----------------------------------------------------------------------------
def run_lk_for_config(cam_t, cam_files, imu_t, imu_wz, gps, t0, t1,
                      roll_deg, pitch_deg, pair_dt, stride, max_feats):
    """Run LK diagnostic on images warped by candidate (roll, pitch)."""
    R = build_R(roll_deg, pitch_deg)
    map1, map2 = cv2.initUndistortRectifyMap(K, D, R, K, (W, H), cv2.CV_32FC1)
    # valid region: source pixel inside original image bounds
    valid_mask = (map1 >= 0) & (map1 < W) & (map2 >= 0) & (map2 < H)
    valid_ratio = float(valid_mask.mean())

    # GPS course rate (same as yaw_diag)
    tg = np.arange(t0, t1, 1.0)
    gE = np.interp(tg, gps[:, 0], gps[:, 1])
    gN = np.interp(tg, gps[:, 0], gps[:, 2])
    dE = np.gradient(gE, tg)
    dN = np.gradient(gN, tg)
    cg = np.degrees(np.arctan2(dN, dE))
    spd = np.hypot(dE, dN)
    valid = spd >= 2.0
    th_g = unwrap_with_gaps(cg, valid)
    omega_g = boxcar(np.gradient(th_g, tg), 15)

    lk_params = dict(winSize=(21, 21), maxLevel=4,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    rows = []
    t_next = t0
    n_done = 0
    while t_next < t1 - pair_dt:
        i0 = int(np.searchsorted(cam_t, t_next))
        i1 = int(np.searchsorted(cam_t, cam_t[i0] + pair_dt))
        if i1 >= len(cam_t):
            break
        t_a, t_b = cam_t[i0], cam_t[i1]
        dt = t_b - t_a
        t_next = t_a + stride
        if dt <= 0.5 * pair_dt or dt > 2.0 * pair_dt:
            continue
        im_a_raw = cv2.imread(cam_files[i0], cv2.IMREAD_GRAYSCALE)
        im_b_raw = cv2.imread(cam_files[i1], cv2.IMREAD_GRAYSCALE)
        if im_a_raw is None or im_b_raw is None:
            continue
        im_a = cv2.remap(im_a_raw, map1, map2, cv2.INTER_LINEAR)
        im_b = cv2.remap(im_b_raw, map1, map2, cv2.INTER_LINEAR)

        p0 = cv2.goodFeaturesToTrack(im_a, max_feats, 0.01, 12)
        if p0 is None or len(p0) < 30:
            rows.append([t_a, dt, 0, 0, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])
            continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(im_a, im_b, p0, None, **lk_params)
        p0b, stb, _ = cv2.calcOpticalFlowPyrLK(im_b, im_a, p1, None, **lk_params)
        fb = np.linalg.norm(p0 - p0b, axis=2).ravel()
        good = (st.ravel() == 1) & (stb.ravel() == 1) & (fb < 0.5)
        n_track = int(good.sum())
        if n_track < 30:
            rows.append([t_a, dt, int(len(p0)), n_track, np.nan, np.nan, np.nan,
                         np.nan, np.nan, np.nan])
            continue

        q0 = cv2.undistortPoints(p0[good], K, D).reshape(-1, 2) * K[0, 0]
        q1 = cv2.undistortPoints(p1[good], K, D).reshape(-1, 2) * K[0, 0]
        med_flow = float(np.median(np.linalg.norm(q1 - q0, axis=1)))

        M_sim, inl = cv2.estimateAffinePartial2D(q0, q1, method=cv2.RANSAC,
                                                 ransacReprojThreshold=1.5)
        if M_sim is None:
            rows.append([t_a, dt, int(len(p0)), n_track, np.nan, np.nan, np.nan,
                         med_flow, np.nan, np.nan])
            continue
        n_inl = int(inl.sum())
        rot_sim = float(np.degrees(np.arctan2(M_sim[1, 0], M_sim[0, 0])))
        q0i = q0[inl.ravel() == 1]
        q1i = q1[inl.ravel() == 1]
        pred = q0i @ M_sim[:, :2].T + M_sim[:, 2]
        resid = float(np.median(np.linalg.norm(pred - q1i, axis=1)))

        M_aff, _ = cv2.estimateAffine2D(q0, q1, method=cv2.RANSAC,
                                        ransacReprojThreshold=1.5)
        rot_aff = rot_from_2x2(M_aff[:, :2]) if M_aff is not None else np.nan
        Hmat, _ = cv2.findHomography(q0, q1, cv2.RANSAC, 1.5)
        rot_hom = rot_from_2x2(Hmat[:2, :2]) if Hmat is not None else np.nan

        rows.append([t_a, dt, int(len(p0)), n_inl, rot_sim, rot_aff, rot_hom,
                     med_flow, resid, n_track])
        n_done += 1
        if n_done % 200 == 0:
            print(f'  {n_done} pairs, t={t_a:.0f}', flush=True)

    A = np.array([[r[0], r[1], r[2], r[3],
                   r[4] if r[4] is not None else np.nan,
                   r[5], r[6], r[7], r[8] if len(r) > 8 else np.nan]
                  for r in rows], dtype=float)
    t_p, dt_p, nfeat, ninl = A[:, 0], A[:, 1], A[:, 2], A[:, 3]
    rot_sim, rot_aff, rot_hom = A[:, 4], A[:, 5], A[:, 6]
    medfl, resid = A[:, 7], A[:, 8]
    ok = ~np.isnan(rot_sim) & (ninl >= 30)
    rate_lk = np.where(ok, rot_sim / dt_p, np.nan)

    # references at pair midpoints
    om_p = np.interp(t_p + dt_p / 2, tg, omega_g)
    omu_p = (np.interp(t_p + dt_p, tg, th_g) - np.interp(t_p, tg, th_g)) / dt_p
    crs_p = np.interp(t_p, tg, cg)
    wz_p = np.full(len(t_p), np.nan)
    for i in range(len(t_p)):
        m = (imu_t >= t_p[i]) & (imu_t <= t_p[i] + dt_p[i])
        if m.sum() > 3:
            wz_p[i] = np.degrees(imu_wz[m].mean())

    # sign/gain calibration against GPS course rate
    mfit = ok & ~np.isnan(om_p) & (np.abs(om_p) > 0.5)
    k_lk = np.polyfit(om_p[mfit], rate_lk[mfit], 1)[0]
    mfg = ~np.isnan(wz_p) & (np.abs(om_p) > 0.5)
    k_gz = np.polyfit(om_p[mfg], wz_p[mfg], 1)[0]
    s_lk = np.sign(k_lk)
    s_gz = np.sign(k_gz)
    rate_lk_h = s_lk * rate_lk
    wz_h = s_gz * wz_p

    # straight/turn conditioning
    p90 = np.percentile(np.abs(omega_g[valid]), 90)
    str_thr = max(0.3, 0.15 * p90)
    trn_thr = 0.5 * p90
    m_str = ok & (np.abs(om_p) < str_thr)
    m_trn = ok & (np.abs(om_p) > trn_thr)

    # aggregate stats
    stats = {
        'roll_deg': roll_deg,
        'pitch_deg': pitch_deg,
        'valid_ratio': valid_ratio,
        'pairs_total': len(t_p),
        'pairs_usable': int(ok.sum()),
        'inlier_median': float(np.nanmedian(ninl[ok])),
        'resid_median': float(np.nanmedian(resid[ok])),
        'med_flow_median': float(np.nanmedian(medfl[ok])),
        'lk_gain': float(k_lk),
        'gz_gain': float(k_gz),
        'lk_sign': int(s_lk),
        'gz_sign': int(s_gz),
    }

    def add_seg(label, m):
        d_lk = (rate_lk_h - omu_p)[m]
        d_gz = (wz_h - omu_p)[m & ~np.isnan(wz_h)]
        m_lg = m & ~np.isnan(wz_h)
        d_lg = (rate_lk_h - wz_h)[m_lg]
        n = int(m.sum())
        stats[f'{label}_n'] = n
        stats[f'{label}_lk_gps_mdeg'] = float(np.nanmean(d_lk) * 1000)
        stats[f'{label}_lk_gps_std_mdeg'] = float(np.nanstd(d_lk) * 1000)
        stats[f'{label}_gz_gps_mdeg'] = float(np.nanmean(d_gz) * 1000)
        stats[f'{label}_lk_gz_mdeg'] = float(np.nanmean(d_lg) * 1000)
        stats[f'{label}_lk_gz_std_mdeg'] = float(np.nanstd(d_lg) * 1000)

    add_seg('straight', m_str)
    add_seg('turn', m_trn)

    # E/W sector residual on straights (course: 0=E, 90=N, 180=W, -90=S)
    d_str = rate_lk_h - wz_h
    crs_wrap = (crs_p + 180) % 360 - 180   # wrap to [-180,180]
    mm_E = m_str & ~np.isnan(d_str) & (np.abs(crs_wrap) < 45)          # E ±45°
    mm_W = m_str & ~np.isnan(d_str) & (np.abs(crs_wrap) > 135)        # W ±45°
    stats['ew_E_mdeg'] = float(np.nanmean(d_str[mm_E]) * 1000) if mm_E.sum() > 40 else np.nan
    stats['ew_W_mdeg'] = float(np.nanmean(d_str[mm_W]) * 1000) if mm_W.sum() > 40 else np.nan
    stats['ew_spread_mdeg'] = stats['ew_W_mdeg'] - stats['ew_E_mdeg']

    # first / second half trend
    tm = (t_p[m_str].min() + t_p[m_str].max()) / 2
    h1 = m_str & (t_p < tm) & ~np.isnan(d_str)
    h2 = m_str & (t_p >= tm) & ~np.isnan(d_str)
    stats['first_half_mdeg'] = float(np.nanmean(d_str[h1]) * 1000) if h1.sum() > 20 else np.nan
    stats['second_half_mdeg'] = float(np.nanmean(d_str[h2]) * 1000) if h2.sum() > 20 else np.nan

    return stats


# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--gps', required=True)
    ap.add_argument('--t0', type=float, default=924.4)
    ap.add_argument('--t1', type=float, default=2500.0)
    ap.add_argument('--roll-degs', default='-1.0,-0.5,0.0,0.5,1.0')
    ap.add_argument('--pitch-degs', default='0.0')
    ap.add_argument('--pair-dt', type=float, default=0.5)
    ap.add_argument('--stride', type=float, default=1.0)
    ap.add_argument('--max-feats', type=int, default=400)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cam_t, cam_files = load_cam_index(args.dataset)
    imu_t, imu_wz = load_imu_wz(args.dataset)
    gps = read_gps_enu(args.gps, args.t0)

    roll_vals = [float(x) for x in args.roll_degs.split(',') if x.strip()]
    pitch_vals = [float(x) for x in args.pitch_degs.split(',') if x.strip()]

    results = []
    for pitch_deg in pitch_vals:
        for roll_deg in roll_vals:
            label = f'roll_{roll_deg:+.1f}_pitch_{pitch_deg:+.1f}'
            print(f'\n=== {label} ===')
            stats = run_lk_for_config(
                cam_t, cam_files, imu_t, imu_wz, gps,
                args.t0, args.t1, roll_deg, pitch_deg,
                args.pair_dt, args.stride, args.max_feats)
            stats['label'] = label
            results.append(stats)

    # -------------------------------------------------------------------------
    # Summary CSV
    csv_path = os.path.join(args.out, 'lk_calib_warp_summary.csv')
    fieldnames = [
        'label', 'roll_deg', 'pitch_deg', 'valid_ratio',
        'pairs_total', 'pairs_usable', 'inlier_median', 'resid_median',
        'med_flow_median', 'lk_sign', 'gz_sign', 'lk_gain', 'gz_gain',
        'straight_n', 'straight_lk_gps_mdeg', 'straight_lk_gps_std_mdeg',
        'straight_gz_gps_mdeg', 'straight_lk_gz_mdeg', 'straight_lk_gz_std_mdeg',
        'turn_n', 'turn_lk_gps_mdeg', 'turn_lk_gz_mdeg',
        'ew_E_mdeg', 'ew_W_mdeg', 'ew_spread_mdeg',
        'first_half_mdeg', 'second_half_mdeg',
    ]
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v)
                        for k, v in r.items() if k in fieldnames})
    print(f'\nWrote summary: {csv_path}')

    # -------------------------------------------------------------------------
    # Markdown report
    md_path = os.path.join(args.out, 'lk_calib_warp_report.md')
    with open(md_path, 'w') as f:
        f.write('# Offline Calibration-Warp LK Validation\n\n')
        f.write(f'**Dataset:** {args.dataset}  \n')
        f.write(f'**Window:** [{args.t0}, {args.t1}]  \n')
        f.write(f'**Pair dt:** {args.pair_dt}s, **stride:** {args.stride}s\n\n')

        f.write('## Comparison table\n\n')
        f.write('| roll° | pitch° | valid% | usable pairs | inlier med | resid px | '
                'LK−gyro mdeg/s | LK−GPS mdeg/s | gyro−GPS mdeg/s | E/W spread | 1st half | 2nd half |\n')
        f.write('|------:|-------:|-------:|-------------:|-----------:|---------:|'
                '---------------:|--------------:|----------------:|----------:|---------:|---------:|\n')
        for r in results:
            f.write(f"| {r['roll_deg']:+.1f} | {r['pitch_deg']:+.1f} | "
                    f"{r['valid_ratio']*100:.1f} | {r['pairs_usable']} | "
                    f"{r['inlier_median']:.0f} | {r['resid_median']:.2f} | "
                    f"{r['straight_lk_gz_mdeg']:+.1f} ± {r['straight_lk_gz_std_mdeg']:.1f} | "
                    f"{r['straight_lk_gps_mdeg']:+.1f} ± {r['straight_lk_gps_std_mdeg']:.1f} | "
                    f"{r['straight_gz_gps_mdeg']:+.1f} | "
                    f"{r['ew_spread_mdeg']:.1f} | "
                    f"{r['first_half_mdeg']:.1f} | {r['second_half_mdeg']:.1f} |\n")
        f.write('\n')

        # Sector detail for each config
        for r in results:
            f.write(f"## {r['label']}\n\n")
            f.write(f"- valid mask ratio: {r['valid_ratio']*100:.1f}%\n")
            f.write(f"- usable pairs: {r['pairs_usable']} / {r['pairs_total']}\n")
            f.write(f"- inliers median: {r['inlier_median']:.0f}, resid median: {r['resid_median']:.2f}px\n")
            f.write(f"- straight LK−gyro: {r['straight_lk_gz_mdeg']:+.1f} ± {r['straight_lk_gz_std_mdeg']:.1f} mdeg/s\n")
            f.write(f"- straight LK−GPS: {r['straight_lk_gps_mdeg']:+.1f} ± {r['straight_lk_gps_std_mdeg']:.1f} mdeg/s\n")
            f.write(f"- E sector: {r['ew_E_mdeg']:.1f}, W sector: {r['ew_W_mdeg']:.1f}, spread: {r['ew_spread_mdeg']:.1f} mdeg/s\n")
            f.write(f"- first half: {r['first_half_mdeg']:.1f}, second half: {r['second_half_mdeg']:.1f} mdeg/s\n\n")

    print(f'Wrote report: {md_path}')


if __name__ == '__main__':
    main()
