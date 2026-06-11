#!/usr/bin/env python3
"""
yaw_leak_analysis.py — dissect the per-update yaw leak from --vio-yaw-diag.

Decomposition: body_yaw(t) evolves through (a) propagation (gyro - bg) and
(b) EKF update jumps. --vio-yaw-diag logs every update's actual yaw change
(delta_yaw_update_deg), so over the analysis window

    total course-yaw drift  =  [sum of update deltas]  +  [propagation drift]
                               (direct injection)         (bg_z error channel)

This answers whether the drift floor is injected by visual updates (OC/FEJ
leak through clone orientations) or accumulated between updates (bg error).

Also: per-update histogram & sign consistency, leak vs time bins, straights
vs turns, correlation with feature statistics (vio_yaw_diag + visual_obs_diag
+ per-frame diag.csv), FEJ-basis residual yaw content in H over time, and a
top-contributor table.

Inputs: --run-dir with vio_yaw_diag.csv (+optional visual_obs_diag.csv,
diag.csv, traj.txt), --gps, --t0, --t1, --out.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yaw_drift_forensics import (read_gps_enu, wrap180, unwrap_with_gaps, boxcar)


def read_csv_dict(path):
    if not os.path.exists(path):
        return []
    return list(csv.DictReader(open(path)))


def fnum(r, k, default=np.nan):
    v = r.get(k, '')
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--gps', required=True)
    ap.add_argument('--t0', type=float, required=True)
    ap.add_argument('--t1', type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--bin', type=float, default=100.0)
    ap.add_argument('--speed-gate', type=float, default=2.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rd = args.run_dir

    # ---- course-yaw drift reference (same math as yaw_drift_forensics) ------
    traj = []
    for line in open(os.path.join(rd, 'traj.txt')):
        s = line.strip()
        if s and not s.startswith('#'):
            c = s.split()
            if len(c) >= 4:
                traj.append([float(x) for x in c[:4]])
    traj = np.array(traj)
    gps = read_gps_enu(args.gps, args.t0)
    lo = max(args.t0, traj[0, 0], gps[0, 0])
    hi = min(args.t1, traj[-1, 0], gps[-1, 0])
    tg = np.arange(lo, hi, 1.0)
    vE = np.interp(tg, traj[:, 0], traj[:, 1]); vN = np.interp(tg, traj[:, 0], traj[:, 2])
    gE = np.interp(tg, gps[:, 0], gps[:, 1]);   gN = np.interp(tg, gps[:, 0], gps[:, 2])

    def course_speed(E, N):
        dE = np.gradient(E, tg); dN = np.gradient(N, tg)
        return np.degrees(np.arctan2(dN, dE)), np.hypot(dE, dN)

    cg, sg = course_speed(gE, gN)
    cv, sv = course_speed(vE, vN)
    valid = (sg >= args.speed_gate) & (sv >= 0.5)
    th_g = unwrap_with_gaps(cg, valid)
    th_v = unwrap_with_gaps(cv, valid)
    yaw_err = (th_v - th_v[0]) - (th_g - th_g[0])       # drift, offset-free
    omega = boxcar(np.gradient(th_g, tg), 15)            # deg/s turn rate
    drift_total = float(yaw_err[-1] - yaw_err[0])

    # ---- vio_yaw_diag rows ---------------------------------------------------
    rows = read_csv_dict(os.path.join(rd, 'vio_yaw_diag.csv'))
    rows = [r for r in rows if lo <= fnum(r, 'timestamp') <= hi]
    if not rows:
        raise SystemExit('no vio_yaw_diag rows in window')
    t_u = np.array([fnum(r, 'timestamp') for r in rows])
    dy = np.array([fnum(r, 'delta_yaw_update_deg') for r in rows])
    typ = np.array([r['update_type'] for r in rows])
    nfeat = np.array([fnum(r, 'num_features') for r in rows])
    nacc = np.array([fnum(r, 'accepted') for r in rows])
    nrej = np.array([fnum(r, 'rejected') for r in rows])
    bgz_u = np.array([fnum(r, 'bg_z') for r in rows])
    dxb = np.array([fnum(r, 'dx_yaw_before_projection_deg') for r in rows])
    dxa = np.array([fnum(r, 'dx_yaw_after_projection_deg') for r in rows])
    om_u = np.interp(t_u, tg, omega)
    types = sorted(set(typ))

    lines = []
    def say(s=''):
        lines.append(s); print(s)

    say('=' * 78)
    say(f'yaw_leak_analysis  run={rd}')
    say(f'window=[{lo:.1f},{hi:.1f}]s  updates={len(rows)}  types={types}')
    say('')
    say(f'course-yaw drift over window (truth-referenced): {drift_total:+.2f} deg')
    inj_all = float(np.nansum(dy))
    say(f'sum of ALL update yaw deltas (direct injection): {inj_all:+.2f} deg')
    say(f'residual (propagation = gyro-bg channel):        {drift_total - inj_all:+.2f} deg')
    say('')

    # ---- per-type stats --------------------------------------------------------
    say('per-update-type injection stats (window):')
    say('  type            n      sum_deg   mean_mdeg  median_mdeg  std_mdeg   t-stat')
    type_sums = {}
    for tp in types:
        m = typ == tp
        d = dy[m] * 1000.0
        ts_ = float(np.mean(d) / (np.std(d) / math.sqrt(max(1, len(d))))) if len(d) > 1 and np.std(d) > 0 else 0.0
        type_sums[tp] = float(np.sum(d) / 1000.0)
        say(f'  {tp:<14}{m.sum():6d}   {np.sum(d)/1000.0:+8.2f}   {np.mean(d):+9.4f}  '
            f'{np.median(d):+10.4f}  {np.std(d):8.3f}  {ts_:+7.1f}')
    say('')
    say('dx yaw projection columns (MSCKF rows): mean |before|->|after| per update')
    m = typ == 'MSCKF'
    if m.sum():
        say(f'  |dx_yaw_before|={np.nanmean(np.abs(dxb[m]))*1000:.4f} mdeg  '
            f'|dx_yaw_after|={np.nanmean(np.abs(dxa[m]))*1000:.4f} mdeg  '
            f'(equal => projection happened in H-space upstream, dx columns see applied dx)')
    say('')

    # ---- time-binned rates -----------------------------------------------------
    bins = np.arange(lo, hi, args.bin)
    rate_rows = []
    say(f'injection rate per {args.bin:.0f}s bin (mdeg/s), vs course drift rate:')
    hdr = '  t_bin      ' + ''.join(f'{tp:<10}' for tp in types) + 'total_inj  drift_rate'
    say(hdr)
    for b in bins:
        mb = (t_u >= b) & (t_u < b + args.bin)
        mg = (tg >= b) & (tg < b + args.bin)
        if mg.sum() < 10:
            continue
        dr = np.polyfit(tg[mg] - b, yaw_err[mg], 1)[0] * 1000
        cells, tot = [], 0.0
        for tp in types:
            s_ = float(np.nansum(dy[mb & (typ == tp)])) / args.bin * 1000
            cells.append(s_); tot += s_
        rate_rows.append([b] + cells + [tot, dr])
        say(f'  {int(b):6d}     ' + ''.join(f'{c:+8.3f}  ' for c in cells)
            + f'{tot:+8.3f}   {dr:+8.3f}')
    say('')

    # ---- straights vs turns ------------------------------------------------------
    p90 = np.percentile(np.abs(om_u), 90)
    s_m = np.abs(om_u) < max(0.3, 0.15 * p90)
    t_m = np.abs(om_u) > 0.5 * p90
    # time spent in each condition (from 1 Hz grid)
    g_s = np.abs(omega) < max(0.3, 0.15 * p90)
    g_t = np.abs(omega) > 0.5 * p90
    say('straights vs turns (MSCKF injection rate normalised by time-in-condition):')
    for lbl, mu, mg_ in [('STRAIGHT', s_m, g_s), ('TURN', t_m, g_t)]:
        dt_cond = float(mg_.sum())  # seconds at 1 Hz
        for tp in types:
            s_ = float(np.nansum(dy[mu & (typ == tp)]))
            say(f'  {lbl:<9} {tp:<14} sum={s_:+7.3f} deg over {dt_cond:6.0f}s '
                f'-> {s_/max(1,dt_cond)*1000:+7.3f} mdeg/s')
    say('')

    # ---- correlations with feature stats (MSCKF) ---------------------------------
    say('correlation of MSCKF delta_yaw with statistics (Pearson r):')
    mm = typ == 'MSCKF'
    cand = {'num_features': nfeat, 'accepted': nacc, 'rejected': nrej,
            'rej_rate': nrej / np.maximum(1, nfeat), 'omega_gps': om_u,
            'abs_omega': np.abs(om_u), 'bg_z': bgz_u, 't': t_u}
    obs = read_csv_dict(os.path.join(rd, 'visual_obs_diag.csv'))
    obs = [r for r in obs if lo <= fnum(r, 'timestamp') <= hi]
    if obs:
        t_o = np.array([fnum(r, 'timestamp') for r in obs])
        for k in ['norm_HN_before', 'norm_HN_after', 'rel_norm_HN_before',
                  'rel_norm_HN_after', 'condition_N', 'chi2_mean_before_oc',
                  'chi2_mean_after_oc']:
            v_o = np.array([fnum(r, k) for r in obs])
            # join on nearest timestamp
            idx = np.searchsorted(t_o, t_u[mm])
            idx = np.clip(idx, 0, len(t_o) - 1)
            cand['obs:' + k] = np.full(len(t_u), np.nan)
            cand['obs:' + k][mm] = v_o[idx]
    dvals = dy[mm]
    for k, v in cand.items():
        vv = v[mm]
        ok = ~np.isnan(vv) & ~np.isnan(dvals)
        if ok.sum() > 20 and np.std(vv[ok]) > 0:
            r_ = float(np.corrcoef(vv[ok], dvals[ok])[0, 1])
            r_abs = float(np.corrcoef(vv[ok], np.abs(dvals[ok]))[0, 1])
            say(f'  {k:<26} r(delta)={r_:+.3f}   r(|delta|)={r_abs:+.3f}')
    say('')

    # ---- FEJ basis residual yaw content in H over time -----------------------------
    if obs:
        rel_after = np.array([fnum(r, 'rel_norm_HN_after') for r in obs])
        ok = ~np.isnan(rel_after)
        if ok.sum() > 20:
            cf = np.polyfit(t_o[ok] - lo, rel_after[ok], 1)
            say(f'residual yaw content in H after OC projection (rel_norm_HN_after):')
            say(f'  mean={np.nanmean(rel_after):.3e}  median={np.nanmedian(rel_after):.3e}  '
                f'slope={cf[0]:+.3e}/s ({"GROWS" if cf[0] > 0 else "shrinks"} over flight)')
            say('')

    # ---- top contributors -------------------------------------------------------
    order = np.argsort(-np.abs(dy))
    say('top-15 single updates by |delta_yaw|:')
    say('  t          type      delta_mdeg  nfeat acc rej   omega   cum_share_of_gross')
    gross = float(np.sum(np.abs(dy))) or 1.0
    cum = 0.0
    top_rows = []
    for i in order[:15]:
        cum += abs(dy[i])
        say(f'  {t_u[i]:9.2f}  {typ[i]:<8} {dy[i]*1000:+9.3f}  {int(nfeat[i]):5d} '
            f'{int(nacc[i]):3d} {int(nrej[i]):3d}  {om_u[i]:+6.2f}   {cum/gross*100:5.1f}%')
        top_rows.append([t_u[i], typ[i], dy[i], int(nfeat[i]), int(nacc[i]),
                         int(nrej[i]), om_u[i]])
    n1 = max(1, len(dy) // 100)
    say(f'  top 1% of updates ({n1}) carry {np.sum(np.abs(dy)[order[:n1]])/gross*100:.1f}% '
        f'of gross |injection|; net of top 1% = {np.sum(dy[order[:n1]]):+.2f} deg')
    say('=' * 78)

    with open(os.path.join(args.out, 'leak_summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    with open(os.path.join(args.out, 'top_contributors.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'type', 'delta_yaw_deg', 'num_features', 'accepted',
                    'rejected', 'omega_gps'])
        w.writerows(top_rows)
    with open(os.path.join(args.out, 'binned_rates.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_bin'] + [f'inj_{tp}_mdegps' for tp in types]
                   + ['inj_total_mdegps', 'drift_rate_mdegps'])
        w.writerows(rate_rows)

    # ---- plots --------------------------------------------------------------------
    fig, ax = plt.subplots(3, 2, figsize=(15, 13))
    a = ax[0, 0]
    for tp in types:
        m_ = typ == tp
        a.plot(t_u[m_], np.cumsum(np.nan_to_num(dy[m_])), lw=1.0, label=f'inj {tp}')
    a.plot(t_u, np.cumsum(np.nan_to_num(dy)), 'k--', lw=1.2, label='inj TOTAL')
    a.plot(tg, yaw_err - yaw_err[0], 'r', lw=1.2, label='course drift (truth-ref)')
    prop = np.interp(t_u, tg, yaw_err - yaw_err[0]) - np.cumsum(np.nan_to_num(dy))
    a.plot(t_u, prop, color='gray', lw=1.0, label='residual = propagation')
    a.set_title('cumulative yaw: update injection vs course drift')
    a.set_xlabel('t [s]'); a.set_ylabel('deg'); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = ax[0, 1]
    for tp in types:
        d = dy[typ == tp] * 1000
        d = d[~np.isnan(d)]
        if len(d):
            a.hist(d, bins=80, range=(-20, 20), alpha=0.55, label=f'{tp} (mean {np.mean(d):+.3f})')
    a.set_title('per-update yaw delta histogram')
    a.set_xlabel('mdeg'); a.set_ylabel('count'); a.legend(fontsize=8); a.grid(alpha=0.3)
    a.set_yscale('log')

    a = ax[1, 0]
    if rate_rows:
        rr = np.array([[r[0], r[-2], r[-1]] for r in rate_rows], float)
        a.plot(rr[:, 0], rr[:, 1], 'o-', ms=3, label='total injection rate')
        a.plot(rr[:, 0], rr[:, 2], 's-', ms=3, label='course drift rate')
        a.plot(rr[:, 0], rr[:, 2] - rr[:, 1], '^-', ms=3, label='propagation rate')
    a.set_title(f'rates per {args.bin:.0f}s bin')
    a.set_xlabel('t [s]'); a.set_ylabel('mdeg/s'); a.legend(fontsize=8); a.grid(alpha=0.3)

    a = ax[1, 1]
    a.plot(om_u[mm], dy[mm] * 1000, '.', ms=2, alpha=0.3)
    a.set_title('MSCKF delta_yaw vs turn rate')
    a.set_xlabel('omega_gps [deg/s]'); a.set_ylabel('mdeg'); a.grid(alpha=0.3)

    a = ax[2, 0]
    if obs:
        a.plot(t_o, rel_after, '.', ms=2, alpha=0.4)
        a.set_title('rel_norm_HN_after (residual yaw in H post-OC) vs time')
        a.set_xlabel('t [s]'); a.grid(alpha=0.3)
    else:
        a.set_title('(no visual_obs_diag.csv)')

    a = ax[2, 1]
    a.plot(t_u, bgz_u * 1e4, lw=0.8, label='bg_z [1e-4 rad/s]')
    a2 = a.twinx()
    a2.plot(tg, yaw_err - yaw_err[0], 'r', lw=0.8, alpha=0.6)
    a2.set_ylabel('course drift [deg]', color='r')
    a.set_title('bg_z (at updates) vs course drift')
    a.set_xlabel('t [s]'); a.legend(fontsize=8); a.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'yaw_leak.png'), dpi=140)
    print(f'\nWrote: {args.out}/yaw_leak.png  leak_summary.txt  top_contributors.csv  binned_rates.csv')


if __name__ == '__main__':
    main()
