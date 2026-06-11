#!/usr/bin/env python3
"""
yaw_drift_forensics.py — separate yaw-drift mechanisms for one VIO run vs GPS.

Core idea: different biased error sources leave different signatures in the
yaw-error time series
    yaw_err(t) = course_VIO(t) - course_GPS(t)   (velocity-direction comparison,
                                                  so sideslip/crab cancels)

  * drift rate proportional to TIME (grows on straights too)
        -> gyro z bias (bg_z) mis-estimation
  * drift rate proportional to TURN RATE (staircase: flat on straights,
    ramps during turns; total drift proportional to cumulative heading turned)
        -> gyro z scale-factor error / cross-axis coupling /
           cam-IMU time-offset interacting with turns

The script fits  yaw_err(t) ~ c0 + c1*(t-t0) + c2*Theta(t)
where Theta(t) = cumulative signed heading turned (from GPS course), and also
measures the drift rate on straight segments only (bias-like) and the drift
per degree turned inside turns only (scale-like).  A per-lap table reports
(VIO heading turned)/(GPS heading turned) - 1, which is a direct estimate of a
scale-type error (e.g. BMI055 gyro sensitivity tolerance is ±1% -> 3.6°/lap).

Also: segment-wise yaw re-alignment ATE (how much of XY ATE is explained by
heading drift) and bg_z overlay from the runner's traj.txt.bias side file.

Inputs
  --traj  traj.txt  (t x y z ...; .bias side file auto-detected)
  --gps   GPS csv with header ts_ns,lat,lon,alt (cam-time aligned)
  --t0 --t1  evaluation window (same time base as traj/gps)
Outputs in --out
  yaw_forensics.png, yaw_forensics.csv, summary.txt (also printed)
"""
import argparse
import csv
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def boxcar(x, n):
    n = max(1, int(n))
    k = np.ones(n) / n
    return np.convolve(x, k, mode='same')


def read_traj_xyz(path):
    rows = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        c = s.split()
        if len(c) >= 4:
            rows.append([float(x) for x in c[:4]])
    return np.array(rows)


def read_bias(path):
    """traj.txt.bias: t vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z -> (t, bgx, bgy, bgz)"""
    rows = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        c = s.split()
        if len(c) >= 7:
            rows.append([float(c[0]), float(c[4]), float(c[5]), float(c[6])])
    return np.array(rows)


def read_gps_enu(path, t0):
    """GPS csv (header ts_ns,lat,lon,alt) -> Nx4 [t E N U], ENU origin near t0."""
    with open(path) as f:
        rdr = csv.DictReader(f)
        names = rdr.fieldnames or []
        rows = list(rdr)
    if 'lat' in names and 'ts_ns' in names:
        pts = [(float(r['ts_ns']) * 1e-9, float(r['lat']), float(r['lon']),
                float(r['alt'])) for r in rows]
        ref = next(((la, lo, al) for (t, la, lo, al) in pts if abs(t - t0) <= 10),
                   pts[0][1:])
        la0, lo0, al0 = ref
        rad = math.pi / 180.0
        return np.array([(t, (lo - lo0) * 111320.0 * math.cos(la0 * rad),
                          (la - la0) * 111320.0, al - al0)
                         for (t, la, lo, al) in pts])
    # fallback: numeric csv t_ns,x,y,z (ENU)
    arr = np.genfromtxt(path, delimiter=',', comments='#')
    arr = arr[~np.isnan(arr[:, 0])]
    return np.column_stack([arr[:, 0] * 1e-9, arr[:, 1], arr[:, 2], arr[:, 3]])


def unwrap_with_gaps(course_deg, valid):
    """Unwrap course over valid samples only; forward-fill across gaps."""
    th = np.full(course_deg.shape, np.nan)
    acc, prev = 0.0, None
    for i in np.where(valid)[0]:
        if prev is not None:
            acc += float(wrap180(course_deg[i] - course_deg[prev]))
        th[i] = acc
        prev = i
    # forward/backward fill so gradients and plots are defined everywhere
    idx = np.where(~np.isnan(th))[0]
    if len(idx) == 0:
        return th
    th[:idx[0]] = th[idx[0]]
    for a, b in zip(idx[:-1], idx[1:]):
        th[a + 1:b] = th[a]
    th[idx[-1] + 1:] = th[idx[-1]]
    return th


def fit_se2(P, G):
    """2D rotation+translation fit minimising |R p + t - g|^2. Returns (theta_deg, rms)."""
    pc, gc = P.mean(axis=0), G.mean(axis=0)
    p, g = P - pc, G - gc
    th = math.atan2(np.sum(p[:, 0] * g[:, 1] - p[:, 1] * g[:, 0]),
                    np.sum(p[:, 0] * g[:, 0] + p[:, 1] * g[:, 1]))
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    res = (p @ R.T) - g
    return math.degrees(th), float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--traj', required=True)
    ap.add_argument('--gps', required=True)
    ap.add_argument('--imu', default=None, help='(unused placeholder, kept for symmetry)')
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--t1', type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--speed-gate', type=float, default=2.0)
    ap.add_argument('--yaw-win', type=float, default=11.0, help='circular smoothing window (s)')
    ap.add_argument('--seg', type=float, default=100.0, help='segment length for yaw re-alignment ATE (s)')
    ap.add_argument('--align-win', type=float, default=60.0, help='global alignment fit window (s)')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    traj = read_traj_xyz(args.traj)
    gps = read_gps_enu(args.gps, args.t0)
    bias_path = args.traj + '.bias'
    bias = read_bias(bias_path) if os.path.exists(bias_path) else np.zeros((0, 4))

    lo = max(args.t0, traj[0, 0], gps[0, 0])
    hi = min(args.t1, traj[-1, 0], gps[-1, 0])
    tg = np.arange(lo, hi, 1.0)                       # 1 Hz analysis grid
    vE = np.interp(tg, traj[:, 0], traj[:, 1]); vN = np.interp(tg, traj[:, 0], traj[:, 2])
    gE = np.interp(tg, gps[:, 0], gps[:, 1]);   gN = np.interp(tg, gps[:, 0], gps[:, 2])

    def course_speed(E, N):
        dE = np.gradient(E, tg); dN = np.gradient(N, tg)
        return np.degrees(np.arctan2(dN, dE)), np.hypot(dE, dN)

    cg, sg = course_speed(gE, gN)
    cv, sv = course_speed(vE, vN)
    valid = (sg >= args.speed_gate) & (sv >= 0.5)

    # smoothed yaw error (circular)
    z = np.exp(1j * np.radians(wrap180(cv - cg))) * valid
    w = max(3, int(args.yaw_win))
    num, den = boxcar(z.real, w) + 1j * boxcar(z.imag, w), boxcar(valid.astype(float), w)
    ok = den > 0.3
    yaw_err = np.full(tg.shape, np.nan)
    yaw_err[ok] = np.degrees(np.angle(num[ok] / den[ok]))
    first = np.where(~np.isnan(yaw_err))[0]
    if len(first) == 0:
        raise SystemExit('no valid yaw-error samples (speed gate too high?)')
    ref = np.nanmean(yaw_err[first[0]:first[0] + 30])
    yaw_err = wrap180(yaw_err - ref)

    # cumulative turned heading + turn rate
    th_g = unwrap_with_gaps(cg, valid); th_g -= th_g[first[0]]
    th_v = unwrap_with_gaps(cv, valid); th_v -= th_v[first[0]]
    omega = boxcar(np.gradient(th_g, tg), 15)         # deg/s, GPS-derived heading rate

    yaw_f = np.copy(yaw_err)
    nanm = np.isnan(yaw_f)
    yaw_f[nanm] = np.interp(tg[nanm], tg[~nanm], yaw_f[~nanm])
    dyaw = boxcar(np.gradient(boxcar(yaw_f, 21), tg), 21)   # deg/s, heavily smoothed

    lines = []
    def say(s=''):
        lines.append(s); print(s)

    say('=' * 78)
    say(f'yaw_drift_forensics  traj={args.traj}')
    say(f'window=[{lo:.1f},{hi:.1f}]s  n_grid={len(tg)}  valid={int(valid.sum())}')
    say('')

    # ---- loop direction / laps ------------------------------------------------
    total_turn = th_g[-1]
    say(f'GPS total signed heading turned: {total_turn:+.0f} deg '
        f'({"CCW" if total_turn > 0 else "CW"} loops, ~{abs(total_turn)/360.0:.1f} laps)')
    say(f'final yaw_err: {yaw_f[-1]:+.2f} deg   (positive = VIO rotated CCW of GPS)')
    ye = yaw_err[~np.isnan(yaw_err)]
    say(f'course-yaw RMS: {float(np.sqrt(np.mean(ye**2))):.2f} deg   '
        f'p95: {float(np.percentile(np.abs(ye), 95)):.2f} deg   '
        f'(offset-removed, first-30s ref)')
    say('')

    # ---- regression: time-proportional vs turn-proportional -------------------
    m = ~np.isnan(yaw_err)
    A = np.column_stack([np.ones(m.sum()), tg[m] - tg[0], th_g[m]])
    y = yaw_err[m]
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ coef
    dof = max(1, m.sum() - 3)
    cov = float(resid @ resid) / dof * np.linalg.inv(A.T @ A)
    se = np.sqrt(np.diag(cov))
    r2 = 1 - float(resid @ resid) / float(((y - y.mean()) ** 2).sum())
    cc = np.corrcoef(A[:, 1], A[:, 2])[0, 1]
    say('joint fit  yaw_err ~ c0 + c1*(t-t0) + c2*Theta_gps(t):')
    say(f'  c1 (time-proportional, bias-like)  = {coef[1]*1000:+.3f} ± {se[1]*1000:.3f} mdeg/s'
        f'  ({coef[1]*3600:+.2f} deg/hr)')
    say(f'  c2 (turn-proportional, scale-like) = {coef[2]*100:+.3f} ± {se[2]*100:.3f} %'
        f'  ({coef[2]*360:+.2f} deg/lap)')
    say(f'  R2={r2:.3f}   corr(t, Theta)={cc:+.3f}'
        + ('   [WARNING: regressors collinear, prefer straight/turn split below]'
           if abs(cc) > 0.97 else ''))
    say('  (std errors assume white residuals -> optimistic; trust signs/magnitudes)')
    say('')

    # ---- straight vs turn split (the clean discriminator) ---------------------
    p90 = np.percentile(np.abs(omega[m]), 90) if m.sum() else 1.0
    straight = m & (np.abs(omega) < max(0.3, 0.15 * p90))
    turning = m & (np.abs(omega) > 0.5 * p90)
    say(f'straight/turn split  (|omega| p90 = {p90:.2f} deg/s; '
        f'straight n={int(straight.sum())}, turn n={int(turning.sum())}):')
    if straight.sum() > 30:
        cs = np.polyfit(tg[straight] - tg[0], yaw_f[straight], 1)
        say(f'  STRAIGHT segments: d(yaw_err)/dt = {cs[0]*1000:+.3f} mdeg/s '
            f'({cs[0]*3600:+.2f} deg/hr)   -> bias-like (bg_z) component')
    if turning.sum() > 30:
        ct = np.polyfit(th_g[turning], yaw_f[turning], 1)
        say(f'  TURN segments:     d(yaw_err)/dTheta = {ct[0]*100:+.3f} % '
            f'({ct[0]*360:+.2f} deg/lap)        -> scale-like component')
    say('')

    # ---- per-lap table ---------------------------------------------------------
    say('per-lap heading audit (VIO turned / GPS turned):')
    say('  lap    t_range          dTheta_gps  dTheta_vio   ratio-1      dYaw_err')
    sgn = 1.0 if total_turn >= 0 else -1.0
    k, i_prev, rows_lap = 1, first[0], []
    for i in range(first[0], len(tg)):
        if sgn * th_g[i] >= 360.0 * k:
            dg, dv = th_g[i] - th_g[i_prev], th_v[i] - th_v[i_prev]
            de = yaw_f[i] - yaw_f[i_prev]
            ratio = (dv / dg - 1.0) if abs(dg) > 1 else float('nan')
            say(f'  {k:3d}  [{tg[i_prev]:7.1f},{tg[i]:7.1f}]  {dg:+9.1f}  {dv:+9.1f}'
                f'   {ratio*100:+7.3f} %   {de:+7.2f} deg')
            rows_lap.append(ratio)
            k += 1; i_prev = i
    if rows_lap:
        say(f'  median scale estimate: {np.nanmedian(rows_lap)*100:+.3f} % '
            f'(BMI055 gyro sensitivity tolerance spec is +/-1 %)')
    say('')

    # ---- segment-wise yaw re-alignment ATE -------------------------------------
    P, G = np.column_stack([vE, vN]), np.column_stack([gE, gN])
    w0 = tg <= lo + args.align_win
    th0, _ = fit_se2(P[w0], G[w0])
    c0_, s0_ = math.cos(math.radians(th0)), math.sin(math.radians(th0))
    R0 = np.array([[c0_, -s0_], [s0_, c0_]])
    pc, gc_ = P[w0].mean(axis=0), G[w0].mean(axis=0)
    P_glob = (P - pc) @ R0.T + gc_
    glob_rms_all = float(np.sqrt(np.mean(np.sum((P_glob - G) ** 2, axis=1))))

    seg_edges = np.arange(lo, hi, args.seg)
    seg_rows, seg_sq, n_seg_pts = [], 0.0, 0
    for s0 in seg_edges:
        sm = (tg >= s0) & (tg < s0 + args.seg)
        if sm.sum() < 10:
            continue
        th_s, rms_s = fit_se2(P[sm], G[sm])
        g_rms = float(np.sqrt(np.mean(np.sum((P_glob[sm] - G[sm]) ** 2, axis=1))))
        seg_rows.append((s0, th_s, rms_s, g_rms))
        seg_sq += rms_s ** 2 * sm.sum(); n_seg_pts += int(sm.sum())
    seg_rms_all = math.sqrt(seg_sq / max(1, n_seg_pts))
    say(f'XY ATE, start-window global alignment ({args.align_win:.0f}s fit): '
        f'{glob_rms_all:.1f} m rms')
    say(f'XY ATE, per-{args.seg:.0f}s segment re-alignment (rot+trans):       '
        f'{seg_rms_all:.1f} m rms')
    expl = 100.0 * (1.0 - seg_rms_all / glob_rms_all) if glob_rms_all > 0 else 0.0
    say(f'-> heading/position drift explains ~{expl:.0f} % of global XY ATE '
        f'(residual {seg_rms_all:.1f} m is local shape/scale error)')
    say('')

    # ---- bg_z ------------------------------------------------------------------
    bgz_g = np.full(tg.shape, np.nan)
    if len(bias):
        bgz_g = np.interp(tg, bias[:, 0], bias[:, 3])
        say(f'bg_z trace ({bias_path}), values at WINDOW start/end:')
        say(f'  start {bgz_g[0]:+.3e}  end {bgz_g[-1]:+.3e}  '
            f'delta {bgz_g[-1]-bgz_g[0]:+.3e} rad/s'
            f'   (full file: {bias[0, 3]:+.3e} -> {bias[-1, 3]:+.3e})')
        say(f'  NOTE: 1e-4 rad/s of UNCORRECTED bg_z error = {math.degrees(1e-4)*3600:.1f} deg/hr '
            f'= {math.degrees(1e-4)*(hi-lo):.2f} deg over this window')
    else:
        say(f'(no .bias side file at {bias_path})')
    say('=' * 78)

    with open(os.path.join(args.out, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    with open(os.path.join(args.out, 'yaw_forensics.csv'), 'w', newline='') as f:
        wcsv = csv.writer(f)
        wcsv.writerow(['t', 'yaw_err_deg', 'dyaw_dt_degps', 'theta_gps_deg',
                       'theta_vio_deg', 'omega_gps_degps', 'bg_z_radps'])
        for i in range(len(tg)):
            wcsv.writerow([f'{tg[i]:.1f}', f'{yaw_err[i]:.4f}', f'{dyaw[i]:.6f}',
                           f'{th_g[i]:.2f}', f'{th_v[i]:.2f}', f'{omega[i]:.4f}',
                           f'{bgz_g[i]:.6e}' if not np.isnan(bgz_g[i]) else ''])

    # ---- plots -----------------------------------------------------------------
    fig, ax = plt.subplots(3, 2, figsize=(15, 13))
    a = ax[0, 0]
    a.plot(tg, yaw_err, lw=0.9)
    k, i_prev = 1, first[0]
    for i in range(first[0], len(tg)):
        if sgn * th_g[i] >= 360.0 * k:
            a.axvline(tg[i], color='gray', lw=0.5, ls=':')
            k += 1
    a.set_title('yaw_err vs time (dotted = lap boundaries)')
    a.set_xlabel('t [s]'); a.set_ylabel('deg'); a.grid(alpha=0.3)

    a = ax[0, 1]
    a.plot(th_g, yaw_f, lw=0.9)
    a.plot(th_g[m], coef[0] + coef[1] * (tg[m] - tg[0]) + coef[2] * th_g[m],
           'r--', lw=0.8, label='joint fit')
    a.set_title(f'yaw_err vs cumulative turned heading (c2={coef[2]*100:+.2f}%)')
    a.set_xlabel('Theta_gps [deg]'); a.set_ylabel('deg'); a.legend(); a.grid(alpha=0.3)

    a = ax[1, 0]
    a.plot(tg, dyaw * 1000, lw=0.9, label='d(yaw_err)/dt [mdeg/s]')
    a2 = a.twinx(); a2.plot(tg, omega, lw=0.6, color='orange', alpha=0.6)
    a2.set_ylabel('omega_gps [deg/s]', color='orange')
    a.set_title('yaw-error rate (blue) vs turn rate (orange)')
    a.set_xlabel('t [s]'); a.grid(alpha=0.3)

    a = ax[1, 1]
    a.plot(omega[m], dyaw[m] * 1000, '.', ms=2, alpha=0.4)
    if m.sum() > 10:
        cf = np.polyfit(omega[m], dyaw[m] * 1000, 1)
        xs = np.linspace(omega[m].min(), omega[m].max(), 10)
        a.plot(xs, np.polyval(cf, xs), 'r-',
               label=f'slope={cf[0]:.3f} mdeg/s per deg/s = {cf[0]/10:.4f}% scale')
        a.legend()
    a.set_title('drift rate vs turn rate (slope => scale-like error)')
    a.set_xlabel('omega_gps [deg/s]'); a.set_ylabel('mdeg/s'); a.grid(alpha=0.3)

    a = ax[2, 0]
    if len(bias):
        a.plot(tg, bgz_g * 1e3, lw=0.9, color='green', label='bg_z [mrad/s]')
        a.legend(loc='upper left')
    a2 = a.twinx(); a2.plot(tg, yaw_err, lw=0.6, color='C0', alpha=0.5)
    a2.set_ylabel('yaw_err [deg]', color='C0')
    a.set_title('bg_z (green) vs yaw_err (blue)')
    a.set_xlabel('t [s]'); a.set_ylabel('mrad/s'); a.grid(alpha=0.3)

    a = ax[2, 1]
    if seg_rows:
        ts_ = [r[0] for r in seg_rows]
        a.bar(ts_, [r[3] for r in seg_rows], width=args.seg * 0.42, label='global-aligned rms')
        a.bar([t + args.seg * 0.45 for t in ts_], [r[2] for r in seg_rows],
              width=args.seg * 0.42, label='segment re-aligned rms')
        a.legend()
    a.set_title(f'XY rms per {args.seg:.0f}s segment')
    a.set_xlabel('t [s]'); a.set_ylabel('m'); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'yaw_forensics.png'), dpi=140)
    print(f'\nWrote: {args.out}/yaw_forensics.png  .csv  summary.txt')


if __name__ == '__main__':
    main()
