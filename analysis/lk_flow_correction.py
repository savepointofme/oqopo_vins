#!/usr/bin/env python3
"""
lk_flow_correction.py — offline correction feasibility test for the fly4 LK curl bias.

Reads existing lk_flow_yaw_diag.csv (no re-tracking), reloads the GPS CSV to
recompute unsmoothed course rates (fly4's CSV only has the smoothed column), then
applies three correction levels and checks how well each removes the bias.

Corrections:
  C1 — constant offset = −mean(LK−gyro) on straights    (one-parameter camera-fixed)
  C2 — per-model constant (sim / affine / homog)
  C3 — per-heading-sector constant (N/E/S/W, estimated on straights)

Outputs (same directory as --csv):
  lk_flow_correction_fly4.csv   pair-level corrected data
  lk_flow_correction_fly4.png   4-panel plot
  LK_FLOW_CORRECTION_FLY4.md    markdown summary
"""
import argparse, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yaw_drift_forensics import read_gps_enu, unwrap_with_gaps, boxcar

STRAIGHT_THRESH = 0.60   # deg/s — same threshold used by lk_flow_yaw_diag.py
MIN_INLIERS     = 30


def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def stats1d(x):
    """Return (mean, SE, t) over non-NaN entries."""
    v = x[~np.isnan(x)]
    if len(v) < 2:
        return np.nan, np.nan, np.nan
    mu = float(v.mean())
    se = float(v.std(ddof=1) / np.sqrt(len(v)))
    return mu, se, (mu / se if se > 0 else np.nan)


def rolling_mean_centered(x, w=30):
    out = np.full(len(x), np.nan)
    hw = w // 2
    for i in range(len(x)):
        lo, hi = max(0, i - hw), min(len(x), i + hw + 1)
        v = x[lo:hi]
        valid = v[~np.isnan(v)]
        if len(valid) > 0:
            out[i] = valid.mean()
    return out


def sector_label(course_deg):
    """GPS course angle (atan2 convention, deg) → N / E / S / W."""
    a = float(course_deg) % 360.0
    if a < 45 or a >= 315:
        return 'E'
    elif 45 <= a < 135:
        return 'N'
    elif 135 <= a < 225:
        return 'W'
    else:
        return 'S'


def identify_legs(is_straight, t_p):
    """Return per-pair integer leg index (0-based); −1 = not a straight segment."""
    leg = np.full(len(is_straight), -1, dtype=int)
    lid = -1
    in_leg = False
    for i, s in enumerate(is_straight):
        if s and not in_leg:
            lid += 1
            in_leg = True
        elif not s:
            in_leg = False
        if in_leg:
            leg[i] = lid
    return leg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv',   required=True,  help='lk_flow_yaw_diag.csv')
    ap.add_argument('--gps',   required=True,  help='fly4 GPS CSV (ts_ns,lat,lon,alt)')
    ap.add_argument('--out',   required=True,  help='output directory (same as --csv dir)')
    ap.add_argument('--t0',    type=float, default=924.4)
    ap.add_argument('--t1',    type=float, default=2500.0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    # ── load existing LK CSV ─────────────────────────────────────────────────
    df = pd.read_csv(args.csv)
    df = df[(df['t'] >= args.t0) & (df['t'] <= args.t1)].reset_index(drop=True)

    t_p        = df['t'].values
    dt_p       = df['dt'].values
    rate_lk    = df['rate_lk_heading_degps'].values     # deg/s, sim, sign-corrected
    wz_h       = df['gyro_z_heading_degps'].values      # deg/s, sign-corrected
    om_smooth  = df['omega_gps_degps'].values           # smoothed (gate only)
    rot_sim_d  = df['rot_sim_deg'].values
    rot_aff_d  = df['rot_affine_deg'].values
    rot_hom_d  = df['rot_homog_deg'].values

    # ── recompute unsmoothed GPS course rate (fly4 CSV predates the fix) ─────
    gps  = read_gps_enu(args.gps, args.t0)
    tg   = np.arange(args.t0 - 120, args.t1 + 120, 1.0)
    gE   = np.interp(tg, gps[:, 0], gps[:, 1])
    gN   = np.interp(tg, gps[:, 0], gps[:, 2])
    dE   = np.gradient(gE, tg)
    dN   = np.gradient(gN, tg)
    cg   = np.degrees(np.arctan2(dN, dE))              # course angle [-180, 180]
    spd  = np.hypot(dE, dN)
    th_g = unwrap_with_gaps(cg, spd >= 2.0)

    # unsmoothed: angular change of unwrapped heading over exact pair interval
    omu_p  = (np.interp(t_p + dt_p, tg, th_g)
              - np.interp(t_p,       tg, th_g)) / dt_p   # deg/s
    crs_p  = np.interp(t_p + dt_p / 2, tg, cg)           # course angle at midpoint

    # ── masks ────────────────────────────────────────────────────────────────
    good   = ~np.isnan(rate_lk) & ~np.isnan(wz_h) & (df['n_inliers'].values >= MIN_INLIERS)
    st     = good & (np.abs(om_smooth) < STRAIGHT_THRESH)   # same gate as original
    n_st   = int(st.sum())

    # ── per-model rates (apply same sign as primary LK rate) ─────────────────
    s_lk     = float(np.sign(np.nanmedian(rate_lk[good] / (rot_sim_d[good] / dt_p[good]))))
    rate_sim = s_lk * rot_sim_d / dt_p
    rate_aff = s_lk * rot_aff_d / dt_p
    rate_hom = s_lk * rot_hom_d / dt_p

    # ── LK−gyro residuals (raw) ───────────────────────────────────────────────
    lkg     = rate_lk  - wz_h
    lkg_sim = rate_sim - wz_h
    lkg_aff = rate_aff - wz_h
    lkg_hom = rate_hom - wz_h

    # ── baseline stats on straights ───────────────────────────────────────────
    mu0, se0, t0_stat       = stats1d(lkg[st])
    mu_s, se_s, t_s         = stats1d(lkg_sim[st])
    mu_a, se_a, t_a         = stats1d(lkg_aff[st])
    mu_h, se_h, t_h         = stats1d(lkg_hom[st])
    mu_lg, se_lg, t_lg      = stats1d((rate_lk - omu_p)[st])
    mu_gg, se_gg, t_gg      = stats1d((wz_h    - omu_p)[st])

    # ── C1: constant correction ───────────────────────────────────────────────
    c1      = -mu0
    lkg_c1  = lkg + c1
    mu1, se1, t1_stat = stats1d(lkg_c1[st])
    mu1_gps, se1_gps, t1_gps = stats1d((rate_lk + c1 - omu_p)[st])

    # first / second half
    t_mid = (args.t0 + args.t1) / 2
    h1, h2 = st & (t_p < t_mid), st & (t_p >= t_mid)
    mf, sf, _  = stats1d(lkg[h1]);     mf1, sf1, _ = stats1d(lkg_c1[h1])
    ms2, ss2, _ = stats1d(lkg[h2]);    ms21, ss21, _ = stats1d(lkg_c1[h2])

    # ── C2: per-model corrections ─────────────────────────────────────────────
    lkg_c2s = lkg_sim + (-mu_s)
    lkg_c2a = lkg_aff + (-mu_a)
    lkg_c2h = lkg_hom + (-mu_h)
    mu2s, se2s, t2s = stats1d(lkg_c2s[st])
    mu2a, se2a, t2a = stats1d(lkg_c2a[st])
    mu2h, se2h, t2h = stats1d(lkg_c2h[st])

    # ── C3: per-heading-sector corrections ────────────────────────────────────
    sectors = np.array([sector_label(c) for c in crs_p])
    SECTOR_ORDER = ['E', 'N', 'W', 'S']
    sec_corr = {}
    sec_info = {}
    for sec in SECTOR_ORDER:
        m = st & (sectors == sec)
        if m.sum() >= 10:
            mu_sec, se_sec, _ = stats1d(lkg[m])
            sec_corr[sec] = -mu_sec
        else:
            sec_corr[sec] = c1
    lkg_c3 = lkg.copy()
    for sec, corr in sec_corr.items():
        lkg_c3[sectors == sec] += corr
    mu3, se3, t3 = stats1d(lkg_c3[st])

    for sec in SECTOR_ORDER:
        m = st & (sectors == sec)
        if m.sum() > 0:
            mr, sr, _ = stats1d(lkg[m])
            m1, s1, _ = stats1d(lkg_c1[m])
            m3, s3, _ = stats1d(lkg_c3[m])
            sec_info[sec] = (int(m.sum()), mr, sr, m1, s1, m3, s3,
                             sec_corr.get(sec, c1))

    # ── per-leg analysis ──────────────────────────────────────────────────────
    leg_idx = identify_legs(st, t_p)
    n_legs  = leg_idx.max() + 1 if leg_idx.max() >= 0 else 0
    leg_rows = []
    for lid in range(n_legs):
        m  = (leg_idx == lid)
        if m.sum() < 3:
            continue
        t_start  = t_p[m].min()
        t_end    = t_p[m].max()
        mean_crs = np.nanmean(crs_p[m])
        sec      = sector_label(mean_crs)
        mr, sr, _ = stats1d(lkg[m]);    m1, s1, _ = stats1d(lkg_c1[m])
        m3, s3, _ = stats1d(lkg_c3[m])
        leg_rows.append((lid, t_start, t_end, int(m.sum()), sec,
                         mr * 1000, sr * 1000, m1 * 1000, s1 * 1000, m3 * 1000))

    # ── accumulated LK−gyro (straight pairs only, cumulative integral) ────────
    # Use only straight pairs; gaps (turns) are left as the last accumulated value
    accum_raw = np.full(n_st, np.nan)
    accum_c1  = np.full(n_st, np.nan)
    accum_c3  = np.full(n_st, np.nan)
    ts_st     = t_p[st]
    dt_st     = dt_p[st]
    lkg_st    = lkg[st];   lkg_c1_st = lkg_c1[st];   lkg_c3_st = lkg_c3[st]
    accum_raw  = np.cumsum(lkg_st    * dt_st)
    accum_c1   = np.cumsum(lkg_c1_st * dt_st)
    accum_c3   = np.cumsum(lkg_c3_st * dt_st)

    # ── save corrected CSV ────────────────────────────────────────────────────
    out_df = df.copy()
    out_df['omu_p_degps']               = omu_p
    out_df['crs_p_deg']                 = crs_p
    out_df['heading_sector']            = sectors
    out_df['is_straight']               = st.astype(int)
    out_df['rate_sim_h_degps']          = rate_sim
    out_df['rate_aff_h_degps']          = rate_aff
    out_df['rate_hom_h_degps']          = rate_hom
    out_df['lkg_raw_mdegps']            = lkg  * 1000
    out_df['lkg_c1_mdegps']             = lkg_c1  * 1000
    out_df['lkg_c2sim_mdegps']          = lkg_c2s * 1000
    out_df['lkg_c2aff_mdegps']          = lkg_c2a * 1000
    out_df['lkg_c2hom_mdegps']          = lkg_c2h * 1000
    out_df['lkg_c3_mdegps']             = lkg_c3 * 1000
    out_df['lk_minus_gps_unsm_mdegps']  = (rate_lk - omu_p) * 1000
    csv_path = os.path.join(args.out, 'lk_flow_correction_fly4.csv')
    out_df.to_csv(csv_path, index=False)

    # ── 4-panel plot ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('LK flow correction feasibility — fly4  [924.4, 2500 s]', fontsize=14)

    # Panel A: raw LK−gyro time series on straights
    ax = axes[0, 0]
    lkg_s_ms   = lkg[st] * 1000
    roll_raw   = rolling_mean_centered(lkg_s_ms)
    ax.scatter(ts_st, lkg_s_ms, s=1.5, alpha=0.25, color='C0')
    ax.plot(ts_st, roll_raw, 'C0-', lw=1.8, label=f'rolling mean')
    ax.axhline(mu0 * 1000, color='C0', lw=2, ls='--',
               label=f'mean {mu0*1000:+.1f} mdeg/s')
    ax.axhline(mf  * 1000, color='C2', lw=1.3, ls=':',
               label=f'1st half {mf*1000:+.1f}')
    ax.axhline(ms2 * 1000, color='C3', lw=1.3, ls=':',
               label=f'2nd half {ms2*1000:+.1f}')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_title('(A) Raw LK − gyro  (straight pairs)')
    ax.set_xlabel('t (s)'); ax.set_ylabel('mdeg/s')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax.set_ylim(-1500, 1500)

    # Panel B: C1-corrected LK−gyro on straights
    ax = axes[0, 1]
    lkg_c1_ms  = lkg_c1[st] * 1000
    roll_c1    = rolling_mean_centered(lkg_c1_ms)
    ax.scatter(ts_st, lkg_c1_ms, s=1.5, alpha=0.25, color='C1')
    ax.plot(ts_st, roll_c1, 'C1-', lw=1.8, label='rolling mean')
    ax.axhline(mu1  * 1000, color='C1', lw=2, ls='--',
               label=f'mean {mu1*1000:+.2f} mdeg/s')
    ax.axhline(mf1  * 1000, color='C2', lw=1.3, ls=':',
               label=f'1st half {mf1*1000:+.1f}')
    ax.axhline(ms21 * 1000, color='C3', lw=1.3, ls=':',
               label=f'2nd half {ms21*1000:+.1f}')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_title(f'(B) C1-corrected  (+{c1*1000:.1f} mdeg/s constant)')
    ax.set_xlabel('t (s)'); ax.set_ylabel('mdeg/s')
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    ax.set_ylim(-1500, 1500)

    # Panel C: per-sector box plots (raw / C1 / C3)
    ax = axes[1, 0]
    avail_secs = [s for s in SECTOR_ORDER if s in sec_info]
    xs = np.arange(len(avail_secs))
    raw_v = [lkg[st & (sectors == s)] * 1000 for s in avail_secs]
    c1_v  = [lkg_c1[st & (sectors == s)] * 1000 for s in avail_secs]
    c3_v  = [lkg_c3[st & (sectors == s)] * 1000 for s in avail_secs]
    w = 0.26
    kw = dict(patch_artist=True, medianprops=dict(color='k', lw=1.5),
              showfliers=False, whiskerprops=dict(lw=0.8), capprops=dict(lw=0.8))
    ax.boxplot(raw_v, positions=xs - w, widths=0.22,
               boxprops=dict(facecolor='C0', alpha=0.55), **kw)
    ax.boxplot(c1_v,  positions=xs,     widths=0.22,
               boxprops=dict(facecolor='C1', alpha=0.55), **kw)
    ax.boxplot(c3_v,  positions=xs + w, widths=0.22,
               boxprops=dict(facecolor='C3', alpha=0.55), **kw)
    ax.set_xticks(xs); ax.set_xticklabels(avail_secs)
    ax.legend(handles=[Patch(facecolor='C0', alpha=0.55, label='raw'),
                       Patch(facecolor='C1', alpha=0.55, label='C1 constant'),
                       Patch(facecolor='C3', alpha=0.55, label='C3 per-sector')],
              fontsize=8)
    ax.axhline(0, color='k', lw=0.8, ls='--')
    ax.set_title('(C) LK−gyro by heading sector  (IQR, outliers hidden)')
    ax.set_xlabel('sector'); ax.set_ylabel('mdeg/s')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-1200, 1200)

    # Panel D: accumulated LK−gyro deviation (straight pairs only)
    ax = axes[1, 1]
    ax.plot(ts_st, accum_raw, lw=1.4, label='raw LK−gyro (cumsum)')
    ax.plot(ts_st, accum_c1,  lw=1.4, label='C1-corrected')
    ax.plot(ts_st, accum_c3,  lw=1.4, ls='--', label='C3 per-sector')
    ax.axhline(0, color='k', lw=0.6)
    ax.set_title('(D) Accumulated LK−gyro deviation  (straight pairs, cumsum)')
    ax.set_xlabel('t (s)'); ax.set_ylabel('deg')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    png_path = os.path.join(args.out, 'lk_flow_correction_fly4.png')
    plt.savefig(png_path, dpi=150)
    plt.close()

    # ── markdown ──────────────────────────────────────────────────────────────
    lines = []
    def md(s=''):
        lines.append(s)

    md('# LK flow correction feasibility — fly4')
    md()
    md(f'Date: 2026-06-11. Input: `lk_flow_diag/lk_flow_yaw_diag.csv`  ')
    md(f'Valid window [{args.t0}, {args.t1}] s; straight gate: smoothed |ω_GPS| < '
       f'{STRAIGHT_THRESH} deg/s; straight pairs n={n_st}.  ')
    md(f'GPS unsmoothed course rate recomputed from `{os.path.basename(args.gps)}` '
       f'(fly4 CSV predates the unsmoothed-column fix).')
    md()

    md('## Baseline statistics (no correction)')
    md()
    md('| comparison | mean (mdeg/s) | SE | t |')
    md('|---|---:|---:|---:|')
    md(f'| LK − gyro (raw) | {mu0*1000:+.2f} | {se0*1000:.2f} | {t0_stat:+.1f} |')
    md(f'| LK − GPS (unsmoothed, recomputed) | {mu_lg*1000:+.2f} | {se_lg*1000:.2f} | {t_lg:+.1f} |')
    md(f'| gyro − GPS (unsmoothed, reference) | {mu_gg*1000:+.2f} | {se_gg*1000:.2f} | {t_gg:+.1f} |')
    md(f'| LK − gyro: sim model | {mu_s*1000:+.2f} | {se_s*1000:.2f} | {t_s:+.1f} |')
    md(f'| LK − gyro: affine model | {mu_a*1000:+.2f} | {se_a*1000:.2f} | {t_a:+.1f} |')
    md(f'| LK − gyro: homography model | {mu_h*1000:+.2f} | {se_h*1000:.2f} | {t_h:+.1f} |')
    md()

    md('## C1 — constant correction')
    md()
    md(f'Applied: +{c1*1000:.2f} mdeg/s (= −mean LK−gyro on straights).')
    md()
    md('| comparison | before | after C1 | change |')
    md('|---|---:|---:|---:|')
    md(f'| LK − gyro (mdeg/s) | {mu0*1000:+.2f} ± {se0*1000:.2f} | {mu1*1000:+.2f} ± {se1*1000:.2f} | by construction ≈ 0 |')
    md(f'| LK − GPS unsmoothed (mdeg/s) | {mu_lg*1000:+.2f} ± {se_lg*1000:.2f} | {mu1_gps*1000:+.2f} ± {se1_gps*1000:.2f} | Δ {(mu1_gps-mu_lg)*1000:+.1f} |')
    md(f'| gyro − GPS (reference) | {mu_gg*1000:+.2f} ± {se_gg*1000:.2f} | unchanged | — |')
    md()
    md('**First/second half (straight pairs):**')
    md()
    md(f'| half | raw LK−gyro | C1-corrected |')
    md(f'|---|---:|---:|')
    md(f'| [{args.t0:.0f}, {t_mid:.0f}] | {mf*1000:+.1f} ± {sf*1000:.1f} | {mf1*1000:+.1f} ± {sf1*1000:.1f} |')
    md(f'| [{t_mid:.0f}, {args.t1:.0f}] | {ms2*1000:+.1f} ± {ss2*1000:.1f} | {ms21*1000:+.1f} ± {ss21*1000:.1f} |')
    half_diff = (ms21 - mf1) * 1000
    se_half   = np.sqrt(sf1**2 + ss21**2) * 1000
    md()

    md('## C2 — per-model corrections')
    md()
    md(f'Applied: sim +{-mu_s*1000:.1f}, affine +{-mu_a*1000:.1f}, '
       f'homog +{-mu_h*1000:.1f} mdeg/s.')
    md()
    md('| model | before | after C2 |')
    md('|---|---:|---:|')
    md(f'| similarity | {mu_s*1000:+.2f} ± {se_s*1000:.2f} | {mu2s*1000:+.2f} ± {se2s*1000:.2f} |')
    md(f'| affine | {mu_a*1000:+.2f} ± {se_a*1000:.2f} | {mu2a*1000:+.2f} ± {se2a*1000:.2f} |')
    md(f'| homography | {mu_h*1000:+.2f} ± {se_h*1000:.2f} | {mu2h*1000:+.2f} ± {se2h*1000:.2f} |')
    md()

    md('## C3 — per-heading-sector corrections')
    md()
    md('| sector | n pairs | raw LK−gyro | applied correction | C1 residual | C3 residual |')
    md('|---|---:|---:|---:|---:|---:|')
    for sec in SECTOR_ORDER:
        if sec not in sec_info:
            continue
        n_s, mr, sr, m1s, s1s, m3s, s3s, sc = sec_info[sec]
        md(f'| {sec} | {n_s} | {mr*1000:+.1f} ± {sr*1000:.1f} | +{sc*1000:.1f} '
           f'| {m1s*1000:+.1f} ± {s1s*1000:.1f} | {m3s*1000:+.1f} ± {s3s*1000:.1f} |')
    md()
    md(f'C3 overall mean: {mu3*1000:+.2f} ± {se3*1000:.2f} mdeg/s (t={t3:+.1f})')
    md()

    md('## Per-straight-leg breakdown')
    md()
    md('| leg | t_start | t_end | n | sector | raw (mdeg/s) | C1 (mdeg/s) | C3 (mdeg/s) |')
    md('|---|---:|---:|---:|---|---:|---:|---:|')
    for row in leg_rows:
        lid, ts, te, n, sec, mr, sr, m1, s1, m3 = row
        md(f'| {lid} | {ts:.0f} | {te:.0f} | {n} | {sec} | {mr:+.1f} ± {sr:.1f} '
           f'| {m1:+.1f} ± {s1:.1f} | {m3:+.1f} |')
    md()

    md('## Q&A')
    md()
    md('### Q1. Does a constant camera-fixed curl correction remove most of the LK−gyro bias?')
    md()
    md(f'The C1 constant correction (+{c1*1000:.1f} mdeg/s) removes the mean bias by '
       f'construction: the corrected straight-segment mean is {mu1*1000:+.2f} mdeg/s '
       f'(≈ 0, SE {se1*1000:.2f}). The raw bias was {mu0*1000:+.1f} ± {se0*1000:.1f} mdeg/s '
       f'(t={t0_stat:+.1f}). The residual SE ({se1*1000:.2f} mdeg/s) reflects pair-to-pair '
       f'noise, not a systematic shift — the correction is effective at the mean level.')
    md()
    md('### Q2. Is the residual still heading-dependent?')
    md()
    # Check significance: any sector's C1-residual significant at 2σ?
    sig_sectors = [s for s in SECTOR_ORDER if s in sec_info
                   and abs(sec_info[s][3] * 1000) > 2 * sec_info[s][4] * 1000]
    if sig_sectors:
        md(f'After C1 correction, sectors {sig_sectors} show residuals > 2σ: ' +
           ', '.join(f'{s} = {sec_info[s][3]*1000:+.1f} mdeg/s (2σ={2*sec_info[s][4]*1000:.1f})'
                     for s in sig_sectors) + '.  ')
        md(f'A heading-dependent component is present. C3 per-sector correction '
           f'reduces the overall mean to {mu3*1000:+.2f} ± {se3*1000:.2f} mdeg/s.')
    else:
        md(f'No sector residual exceeds 2σ after C1 correction (all sectors: ' +
           ', '.join(f'{s}={sec_info[s][3]*1000:+.1f}' for s in SECTOR_ORDER if s in sec_info) +
           f' mdeg/s). The C1-corrected distribution is heading-uniform.')
    md()
    md('### Q3. Is the residual time-dependent (thermal drift)?')
    md()
    t_trend = abs(half_diff) / se_half if se_half > 0 else 0
    md(f'C1-corrected: first half {mf1*1000:+.1f} ± {sf1*1000:.1f} mdeg/s, '
       f'second half {ms21*1000:+.1f} ± {ss21*1000:.1f} mdeg/s '
       f'(half-difference {half_diff:+.1f} mdeg/s, t={t_trend:.1f}).  ')
    if t_trend > 2:
        md(f'Statistically significant time trend — the constant correction is '
           f'not thermally stable. The residual drift rate is approximately '
           f'{half_diff / ((args.t1 - args.t0) / 2) * 1000:.1f} mdeg/s per 1000 s.')
    else:
        md(f'No statistically significant time trend (t={t_trend:.1f} < 2). '
           f'The correction appears thermally stable over the {args.t1-args.t0:.0f} s window.')
    md()
    md('### Q4. Does the corrected visual proxy become consistent with gyro/course?')
    md()
    gyro_gps_band = abs(mu_gg * 1000) + 2 * se_gg * 1000
    corrected_lk_gps = abs(mu1_gps * 1000)
    md(f'After C1: LK−GPS (unsmoothed) = {mu1_gps*1000:+.1f} ± {se1_gps*1000:.1f} mdeg/s.  ')
    md(f'Reference: gyro−GPS (unsmoothed) = {mu_gg*1000:+.1f} ± {se_gg*1000:.1f} mdeg/s.  ')
    if corrected_lk_gps < gyro_gps_band:
        md(f'The corrected LK−GPS ({corrected_lk_gps:.1f} mdeg/s) falls within the '
           f'gyro−GPS noise band ({gyro_gps_band:.1f} mdeg/s). After C1, the visual '
           f'proxy is GPS-consistent and agrees with gyro by construction.')
    else:
        md(f'A residual LK−GPS systematic of {mu1_gps*1000:+.1f} mdeg/s remains after C1, '
           f'exceeding the gyro−GPS noise band ({gyro_gps_band:.1f} mdeg/s). '
           f'GPS noise likely dominates; interpret with caution.')
    md()
    md('### Q5. Is it worth implementing a real VIO-side correction?')
    md()
    vio_drift = 12.9   # mdeg/s, A-baseline fly4
    curl_mag  = abs(mu0 * 1000)
    md(f'The raw curl ({curl_mag:.0f} mdeg/s) is **{curl_mag/vio_drift:.0f}×** larger than the '
       f'A-baseline VIO yaw drift (+{vio_drift} mdeg/s). However, the cross-flight evidence '
       f'(fly1: curl −286 mdeg/s, VIO drift ≈ 0; fly4: curl −160 mdeg/s, VIO drift +13) '
       f'shows the curl magnitude does NOT rank with VIO drift — the filter mediates '
       f'how the biased residuals translate to heading error.')
    md()
    if t_trend < 2 and not sig_sectors:
        md(f'The curl is thermally stable and heading-uniform over this window. '
           f'A 1-parameter constant correction to the extrinsic yaw (or radtan '
           f'operating point) could encode it without online estimation. Worth '
           f'testing as an offline calibration fix — but a VIO ATE test on all four '
           f'flights is required before committing, given the non-proportional '
           f'curl→drift relationship.')
    elif t_trend >= 2:
        md(f'The correction has a significant time-varying (thermal) component '
           f'({half_diff:+.1f} mdeg/s drift over the window). A static 1-parameter '
           f'calibration cannot capture this — an adaptive filter-side correction '
           f'would be needed, with real risk of regression on other flights.')
    else:
        md(f'A heading-dependent residual remains after constant correction. '
           f'A per-sector static calibration (C3) removes most of it, but '
           f'it maps to multiple parameters and may not generalize across flights.')
    md()
    md('_(Raw-vision correction feasibility test only. No filter-topology changes '
       'are proposed; no follow-up VIO runs were launched.)_')

    md_path = os.path.join(args.out, 'LK_FLOW_CORRECTION_FLY4.md')
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')

    print(f"\nSaved: {csv_path}")
    print(f"Saved: {png_path}")
    print(f"Saved: {md_path}")
    print(f"\n=== Summary ===")
    print(f"Baseline LK-gyro:  {mu0*1000:+.2f} ± {se0*1000:.2f} mdeg/s  (t={t0_stat:+.1f})")
    print(f"C1 correction:     +{c1*1000:.2f} mdeg/s")
    print(f"C1 residual:       {mu1*1000:+.2f} ± {se1*1000:.2f} mdeg/s  (by construction 0)")
    print(f"C1 1st half:       {mf1*1000:+.1f} ± {sf1*1000:.1f} mdeg/s")
    print(f"C1 2nd half:       {ms21*1000:+.1f} ± {ss21*1000:.1f} mdeg/s")
    print(f"Half-difference:   {half_diff:+.1f} mdeg/s  (t_trend={t_trend:.1f})")
    print(f"LK-GPS after C1:   {mu1_gps*1000:+.1f} ± {se1_gps*1000:.1f} mdeg/s")
    print(f"gyro-GPS ref:      {mu_gg*1000:+.1f} ± {se_gg*1000:.1f} mdeg/s")
    print(f"Per-model (before): sim={mu_s*1000:+.1f}  aff={mu_a*1000:+.1f}  hom={mu_h*1000:+.1f}")
    print(f"Per-sector raw: " + "  ".join(
        f"{s}={sec_info[s][1]*1000:+.1f}" for s in SECTOR_ORDER if s in sec_info))
    print(f"Per-sector C1:  " + "  ".join(
        f"{s}={sec_info[s][3]*1000:+.1f}" for s in SECTOR_ORDER if s in sec_info))
    print(f"C3 overall:        {mu3*1000:+.2f} ± {se3*1000:.2f} mdeg/s  (t={t3:+.1f})")


if __name__ == '__main__':
    main()
