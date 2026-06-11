#!/usr/bin/env python3
"""
lk_flow_yaw_diag.py — raw LK optical-flow yaw diagnostic, independent of the VIO.

For sampled frame pairs of the downward camera: GFTT features -> pyramidal LK
(forward-backward check) -> undistort points (ground radtan calibration) ->
RANSAC 2D motion fits (similarity / full affine / homography) -> in-plane
rotation as an image-yaw-rate proxy.

The proxy is compared against three independent references over the same window:
  - GPS course rate (truth for heading rate, speed-gated)
  - raw IMU gyro z mean over each pair interval
  - the VIO baseline run's course-yaw drift + bg_z (read from its forensics CSV)

Sign convention: the LK and gyro signals are mapped onto "GPS course rate" sign
by correlation over the full window (turns dominate the variance); the fitted
sign/gain is reported, not assumed from extrinsics.

This measures RELATIVE rotation only; it is a raw visual-motion diagnostic,
not an absolute yaw sensor.

Outputs in --out: lk_flow_yaw_diag.csv, lk_flow_yaw_diag.png, summary printed.
"""
import argparse
import csv
import os
import sys

import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yaw_drift_forensics import read_gps_enu, unwrap_with_gaps, boxcar


def load_cam_index(dataset):
    """custom csv: t_rel_s, t_device_s, t_ns, domain, filename -> (t[], path[])"""
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
    """custom csv: t_rel_s,...,wx,wy,wz,ax,ay,az at cols 0,5..10"""
    t, wz = [], []
    for line in open(os.path.join(dataset, 'imu0', 'data.csv')):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        c = s.split(',')
        if len(c) < 11:
            continue
        try:
            t.append(float(c[0])); wz.append(float(c[7]))
        except ValueError:
            continue
    return np.array(t), np.array(wz)


def rot_from_2x2(M):
    """rotation angle (deg) of the closest rotation to 2x2 matrix M (polar decomp)."""
    u, _, vt = np.linalg.svd(M)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--gps', required=True)
    ap.add_argument('--ref-forensics', default=None,
                    help='yaw_forensics.csv of a VIO run for drift/bg_z overlay')
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--t1', type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--pair-dt', type=float, default=0.5, help='frame pair baseline (s)')
    ap.add_argument('--stride', type=float, default=1.0, help='pair sampling interval (s)')
    ap.add_argument('--max-feats', type=int, default=400)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    # ground radtan calibration (config/d455_fly2 kalibr_imucam_chain.yaml)
    K = np.array([[367.4567959235411, 0, 328.6368136147717],
                  [0, 367.1397460471364, 233.58213701688157],
                  [0, 0, 1.0]])
    D = np.array([-0.059513850515088895, 0.034926517536683285,
                  -0.0027539606930595275, 7.51756448944016e-05])

    cam_t, cam_files = load_cam_index(args.dataset)
    imu_t, imu_wz = load_imu_wz(args.dataset)

    # GPS course rate on 1 Hz grid (same math as forensics)
    gps = read_gps_enu(args.gps, args.t0)
    tg = np.arange(args.t0, args.t1, 1.0)
    gE = np.interp(tg, gps[:, 0], gps[:, 1]); gN = np.interp(tg, gps[:, 0], gps[:, 2])
    dE = np.gradient(gE, tg); dN = np.gradient(gN, tg)
    cg = np.degrees(np.arctan2(dN, dE)); spd = np.hypot(dE, dN)
    valid = spd >= 2.0
    th_g = unwrap_with_gaps(cg, valid)
    omega_g = boxcar(np.gradient(th_g, tg), 15)

    # ---- frame pair loop -----------------------------------------------------
    lk_params = dict(winSize=(21, 21), maxLevel=4,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    rows = []
    t_next = args.t0
    n_done = 0
    while t_next < args.t1 - args.pair_dt:
        i0 = int(np.searchsorted(cam_t, t_next))
        i1 = int(np.searchsorted(cam_t, cam_t[i0] + args.pair_dt))
        if i1 >= len(cam_t):
            break
        t_a, t_b = cam_t[i0], cam_t[i1]
        dt = t_b - t_a
        t_next = t_a + args.stride
        if dt <= 0.5 * args.pair_dt or dt > 2.0 * args.pair_dt:
            continue
        im_a = cv2.imread(cam_files[i0], cv2.IMREAD_GRAYSCALE)
        im_b = cv2.imread(cam_files[i1], cv2.IMREAD_GRAYSCALE)
        if im_a is None or im_b is None:
            continue
        p0 = cv2.goodFeaturesToTrack(im_a, args.max_feats, 0.01, 12)
        if p0 is None or len(p0) < 30:
            rows.append([t_a, dt, 0, 0, np.nan, np.nan, np.nan, np.nan,
                         np.nan, np.nan])
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
        # similarity (rotation+scale+translation), RANSAC
        M_sim, inl = cv2.estimateAffinePartial2D(q0, q1, method=cv2.RANSAC,
                                                 ransacReprojThreshold=1.5)
        if M_sim is None:
            rows.append([t_a, dt, int(len(p0)), n_track, np.nan, np.nan, np.nan,
                         med_flow, np.nan, np.nan])
            continue
        n_inl = int(inl.sum())
        rot_sim = float(np.degrees(np.arctan2(M_sim[1, 0], M_sim[0, 0])))
        # inlier residual
        q0i = q0[inl.ravel() == 1]; q1i = q1[inl.ravel() == 1]
        pred = q0i @ M_sim[:, :2].T + M_sim[:, 2]
        resid = float(np.median(np.linalg.norm(pred - q1i, axis=1)))
        # full affine
        M_aff, _ = cv2.estimateAffine2D(q0, q1, method=cv2.RANSAC,
                                        ransacReprojThreshold=1.5)
        rot_aff = rot_from_2x2(M_aff[:, :2]) if M_aff is not None else np.nan
        # homography
        H, _ = cv2.findHomography(q0, q1, cv2.RANSAC, 1.5)
        rot_hom = rot_from_2x2(H[:2, :2]) if H is not None else np.nan

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
    rate_lk = np.where(ok, rot_sim / dt_p, np.nan)        # deg/s, image frame

    # references at pair midpoints
    om_p = np.interp(t_p + dt_p / 2, tg, omega_g)         # smoothed (gate/plots only)
    # unsmoothed course rate over the exact pair interval (fly4 lesson: the 15s
    # boxcar smears turn rate into the straight gate, biasing X-minus-GPS means)
    omu_p = (np.interp(t_p + dt_p, tg, th_g) - np.interp(t_p, tg, th_g)) / dt_p
    crs_p = np.interp(t_p, tg, cg)                        # course angle for leg split
    wz_p = np.full(len(t_p), np.nan)                      # mean gyro z over pair
    for i in range(len(t_p)):
        m = (imu_t >= t_p[i]) & (imu_t <= t_p[i] + dt_p[i])
        if m.sum() > 3:
            wz_p[i] = np.degrees(imu_wz[m].mean())

    # ---- sign/gain calibration of LK and gyro against GPS course rate --------
    mfit = ok & ~np.isnan(om_p) & (np.abs(om_p) > 0.5)
    k_lk = np.polyfit(om_p[mfit], rate_lk[mfit], 1)[0]
    mfg = ~np.isnan(wz_p) & (np.abs(om_p) > 0.5)
    k_gz = np.polyfit(om_p[mfg], wz_p[mfg], 1)[0]
    s_lk = np.sign(k_lk); s_gz = np.sign(k_gz)
    rate_lk_h = s_lk * rate_lk                            # heading-sign LK rate
    wz_h = s_gz * wz_p

    lines = []
    def say(s=''):
        lines.append(s); print(s)

    say('=' * 78)
    say(f'LK flow yaw diagnostic  dataset={args.dataset}')
    say(f'window=[{args.t0},{args.t1}]  pairs={len(t_p)}  usable={int(ok.sum())} '
        f'({100*ok.mean():.0f}%)  pair_dt~{np.nanmedian(dt_p):.2f}s stride={args.stride}s')
    say(f'tracking health: inliers median={np.nanmedian(ninl[ok]):.0f}  '
        f'med_flow median={np.nanmedian(medfl[ok]):.1f}px  '
        f'RANSAC resid median={np.nanmedian(resid[ok]):.2f}px')
    say('')
    say(f'sign/gain vs GPS course rate (|omega|>0.5 deg/s): '
        f'LK gain={k_lk:+.3f} (sign {s_lk:+.0f}), gyro-z gain={k_gz:+.3f} (sign {s_gz:+.0f})')
    say('(gain magnitude < 1 expected: cos(tilt), model leakage)')
    say('')

    # ---- straight/turn conditioning (same defn as forensics) ------------------
    p90 = np.percentile(np.abs(omega_g[valid]), 90)
    str_thr = max(0.3, 0.15 * p90); trn_thr = 0.5 * p90
    m_str = ok & (np.abs(om_p) < str_thr)
    m_trn = ok & (np.abs(om_p) > trn_thr)

    def seg_stats(label, m):
        # GPS comparisons use the UNSMOOTHED course rate (gate stays on smoothed)
        d_lk = (rate_lk_h - omu_p)[m]
        d_gz = (wz_h - omu_p)[m & ~np.isnan(wz_h)]
        # vision-vs-gyro differential: real rotation (incl. airframe wobble) is
        # common-mode and cancels -> per-pair noise drops ~50x vs the GPS refs
        m_lg = m & ~np.isnan(wz_h)
        d_lg = (rate_lk_h - wz_h)[m_lg]
        n = int(m.sum())
        mu = np.nanmean(d_lk); se = np.nanstd(d_lk) / max(1, np.sqrt(n))
        say(f'{label}: n={n}')
        say(f'  LK-rate minus GPS course rate (unsmoothed): mean={mu*1000:+8.2f} mdeg/s  '
            f'(SE {se*1000:.2f}, t={mu/max(se,1e-12):+.1f})  median={np.nanmedian(d_lk)*1000:+.2f}')
        mug = np.nanmean(d_gz); seg = np.nanstd(d_gz) / max(1, np.sqrt(len(d_gz)))
        say(f'  gyro-z minus GPS course rate (unsmoothed):  mean={mug*1000:+8.2f} mdeg/s  '
            f'(SE {seg*1000:.2f}, t={mug/max(seg,1e-12):+.1f})')
        mul = np.nanmean(d_lg); sel = np.nanstd(d_lg) / max(1, np.sqrt(len(d_lg)))
        say(f'  LK-rate minus gyro-z (wobble-cancelled): mean={mul*1000:+8.2f} mdeg/s  '
            f'(SE {sel*1000:.2f}, t={mul/max(sel,1e-12):+.1f})  std={np.nanstd(d_lg)*1000:.1f}')
        return mu

    def leg_and_trend(m):
        d = rate_lk_h - wz_h
        say('opposite-leg sign check (LK - gyro by GPS course sector, straights):')
        for c0 in range(-180, 180, 45):
            mm = m & ~np.isnan(d) & (((crs_p - c0 + 180) % 360 - 180 >= 0) &
                                     ((crs_p - c0 + 180) % 360 - 180 < 45))
            if mm.sum() > 40:
                mu_ = np.nanmean(d[mm]); se_ = np.nanstd(d[mm]) / np.sqrt(mm.sum())
                say(f'  course {c0:+4d}..{c0+45:+4d}: n={int(mm.sum()):4d}  '
                    f'{mu_*1000:+8.1f} +/- {se_*1000:.1f} mdeg/s')
        tm = (t_p[m].min() + t_p[m].max()) / 2
        h1 = m & (t_p < tm) & ~np.isnan(d); h2 = m & (t_p >= tm) & ~np.isnan(d)
        if h1.sum() > 20 and h2.sum() > 20:
            say(f'time trend (LK - gyro, straights): first half {np.nanmean(d[h1])*1000:+.1f}, '
                f'second half {np.nanmean(d[h2])*1000:+.1f} mdeg/s')

    say(f'straight/turn thresholds: straight |om|<{str_thr:.2f}, turn |om|>{trn_thr:.2f} deg/s')
    mu_str = seg_stats('STRAIGHT segments', m_str)
    seg_stats('TURN segments', m_trn)
    say('')
    leg_and_trend(m_str)
    say('')
    say('reference values (fly4 valid window): VIO A-baseline drift = +12.9 mdeg/s on')
    say('straights; floor across interventions ~ +17 mdeg/s; stationary bg_z ~ +1e-4')
    say(f'rad/s = {np.degrees(1e-4)*1000:.1f} mdeg/s equivalent.')
    say('')

    # time trend of the LK-vs-GPS straight-segment residual
    if m_str.sum() > 100:
        c = np.polyfit(t_p[m_str] - args.t0, (rate_lk_h - om_p)[m_str], 1)
        say(f'straight-segment LK-residual time trend: {c[0]*1000*1000:+.3f} mdeg/s per 1000s '
            f'(start {np.polyval(c,0)*1000:+.1f} -> end {np.polyval(c,args.t1-args.t0)*1000:+.1f} mdeg/s)')
    say('=' * 78)

    # ---- CSV -------------------------------------------------------------------
    csv_path = os.path.join(args.out, 'lk_flow_yaw_diag.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'dt', 'n_feat', 'n_inliers', 'rot_sim_deg', 'rot_affine_deg',
                    'rot_homog_deg', 'med_flow_px', 'ransac_resid_px',
                    'rate_lk_heading_degps', 'omega_gps_degps',
                    'omega_gps_unsmoothed_degps', 'gyro_z_heading_degps'])
        for i in range(len(t_p)):
            w.writerow([f'{t_p[i]:.3f}', f'{dt_p[i]:.3f}', int(nfeat[i]), int(ninl[i]),
                        f'{rot_sim[i]:.5f}', f'{rot_aff[i]:.5f}', f'{rot_hom[i]:.5f}',
                        f'{medfl[i]:.2f}', f'{resid[i]:.3f}',
                        f'{rate_lk_h[i]:.5f}', f'{om_p[i]:.4f}', f'{omu_p[i]:.4f}',
                        f'{wz_h[i]:.5f}'])

    # ---- plots -------------------------------------------------------------------
    fig, ax = plt.subplots(3, 2, figsize=(15, 13))
    a = ax[0, 0]
    a.plot(t_p[ok], rate_lk_h[ok], '.', ms=2, alpha=0.4, label='LK rate (heading sign)')
    a.plot(tg, omega_g, 'r-', lw=0.7, alpha=0.7, label='GPS course rate')
    a.set_title('LK image-rotation rate vs GPS course rate')
    a.set_xlabel('t [s]'); a.set_ylabel('deg/s'); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = ax[0, 1]
    # accumulated proxies: integrate (rate - unsmoothed GPS course rate)
    d_lk = np.where(ok, rate_lk_h - omu_p, 0.0)
    acc_lk = np.cumsum(d_lk * args.stride)
    d_gz = np.where(~np.isnan(wz_h), wz_h - omu_p, 0.0)
    acc_gz = np.cumsum(d_gz * args.stride)
    a.plot(t_p, acc_lk, lw=1.2, label='accum (LK - GPS) [deg]')
    a.plot(t_p, acc_gz, lw=1.2, label='accum (gyro_z - GPS) [deg]')
    if args.ref_forensics and os.path.exists(args.ref_forensics):
        rt, re = [], []
        for r in csv.DictReader(open(args.ref_forensics)):
            try:
                rt.append(float(r['t'])); re.append(float(r['yaw_err_deg']))
            except (ValueError, KeyError):
                pass
        a.plot(rt, np.array(re) - re[0], 'r--', lw=1.2, label='VIO course-yaw err (A)')
    a.set_title('accumulated LK yaw proxy vs VIO course-yaw error')
    a.set_xlabel('t [s]'); a.set_ylabel('deg'); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = ax[1, 0]
    a.plot(t_p, ninl, '.', ms=2)
    a.set_title('RANSAC inlier count'); a.set_xlabel('t [s]'); a.grid(alpha=0.3)

    a = ax[1, 1]
    a.plot(t_p[ok], medfl[ok], '.', ms=2)
    a.set_title('median flow magnitude [px]'); a.set_xlabel('t [s]'); a.grid(alpha=0.3)

    a = ax[2, 0]
    a.plot(t_p[ok], resid[ok], '.', ms=2)
    a.set_title('RANSAC median inlier residual [px]'); a.set_xlabel('t [s]'); a.grid(alpha=0.3)

    a = ax[2, 1]
    d = (rate_lk_h - omu_p)[m_str] * 1000
    a.hist(d[~np.isnan(d)], bins=80, range=(-400, 400))
    a.axvline(0, color='k', lw=0.8)
    a.axvline(np.nanmean(d), color='r', lw=1.2,
              label=f'mean {np.nanmean(d):+.1f} mdeg/s')
    a.set_title('straight-segment (LK - GPS) rate histogram')
    a.set_xlabel('mdeg/s'); a.legend(); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'lk_flow_yaw_diag.png'), dpi=140)

    with open(os.path.join(args.out, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'\nWrote: {csv_path}, lk_flow_yaw_diag.png, summary.txt')


if __name__ == '__main__':
    main()
