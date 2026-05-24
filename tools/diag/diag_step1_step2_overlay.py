#!/usr/bin/env python3
"""Step 1 + Step 2: bias_a_z(t) overlay vs sliding-window scale, plus
cam_imu_timeoffset(t) and bg(t) trajectories.

Step 1 (test H3): overlay bias_a_z(t) with sliding-window scale_ratio(t).
  We want to answer: does the sign and magnitude of residual bias_a_z after
  vertical-climb completion predict the scale-error sign?

Step 2 (test H5): overlay cam_imu_timeoffset(t) and bg(t) with phase markers.
  We want to answer: does per-flight timeoffset divergence or gyro bias
  behavior correlate with first-edge scale error?

Inputs:
  - 20260509_flyN/result/baselines_v1/B0_no_gps.txt.bias   (fly1-3)
  - 20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.txt.bias (fly4)
  - comparison_plots/sliding_window_scale/summary.json
  - comparison_plots/hover_takeoff_scale/summary.json
  - 20260509_fly*/result/baselines_v1/B0_no_gps.log (fly1-3) / stage_a_v2 (fly4)
"""

import json, os, re, sys
import numpy as np

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:
    _HAVE_MPL = False

# ---------------------------------------------------------------------------
# Config: per-flight inputs
# ---------------------------------------------------------------------------
FLIGHTS = [
    dict(flight=1,
         bias='20260509_fly1/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly1/result/baselines_v1/B0_no_gps.log'),
    dict(flight=2,
         bias='20260509_fly2/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly2/result/baselines_v1/B0_no_gps.log'),
    dict(flight=3,
         bias='20260509_fly3/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly3/result/baselines_v1/B0_no_gps.log'),
    dict(flight=4,
         bias='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.txt.bias',
         log='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.log'),
]

SW_JSON = 'comparison_plots/sliding_window_scale/summary.json'
HT_JSON = 'comparison_plots/hover_takeoff_scale/summary.json'
OUT_DIR = 'comparison_plots/stage_b_diag'
os.makedirs(OUT_DIR, exist_ok=True)

# Phase-marker colours for consistency across plots
PHASE_COLORS = dict(alt_rise='red', xy_takeoff='orange', diverge='purple')

# ---------------------------------------------------------------------------
# Data loaders
# ---------------------------------------------------------------------------

def load_bias(path):
    """Returns (t, ba_x, ba_y, ba_z, bg_x, bg_y, bg_z, vx, vy, vz)."""
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'):
                continue
            ps = ln.split()
            if len(ps) < 10:
                continue
            try:
                rows.append([float(x) for x in ps[:10]])
            except ValueError:
                continue
    a = np.array(rows)
    # deduplicate by strictly increasing t
    if a.size:
        keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
        a = a[keep]
    return a  # [N, 10]: t, vx, vy, vz, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z


def load_sw_summary(path):
    with open(path) as f:
        return json.load(f)


def load_ht_summary(path):
    with open(path) as f:
        return json.load(f)


def parse_timeoffset_log(log_path):
    """Parse all 'camera-imu timeoffset = X' lines from the log.

    The log has no per-line timestamps, but timeoffset is printed once per
    camera frame, at the same rate as the .bias file.  We return the values
    in order so they can be paired with bias timestamps.
    """
    vals = []
    with open(log_path, errors='replace') as f:
        for ln in f:
            ln = re.sub(r'\x1b\[[0-9;]*m', '', ln)
            m = re.search(r'camera-imu timeoffset = ([-\d.]+)', ln)
            if m:
                vals.append(float(m.group(1)))
    return np.array(vals)


# ---------------------------------------------------------------------------
# Per-flight analysis
# ---------------------------------------------------------------------------

def analyze_flight(cfg, sw_rec, ht_rec):
    fly = cfg['flight']
    print(f'\n{"="*70}\n  fly{fly}\n{"="*70}')

    # -- bias data --
    bias = load_bias(cfg['bias'])
    if bias.size == 0:
        print(' [SKIP] no bias data'); return
    b_t = bias[:, 0]
    b_ba_z = bias[:, 9]   # column 9 = ba_z
    b_bg_x = bias[:, 4]   # columns 4-6 = bg_x, bg_y, bg_z
    b_bg_y = bias[:, 5]
    b_bg_z = bias[:, 6]

    # -- timeoffset --
    to_vals = parse_timeoffset_log(cfg['log'])
    # Pair with bias timestamps (same per-frame rate)
    n_to = len(to_vals)
    n_bias = len(b_t)
    if n_to > 0 and abs(n_to - n_bias) <= 5:
        # Truncate to min length
        n = min(n_to, n_bias)
        to_t = b_t[:n]
        to_vals = to_vals[:n]
    elif n_to > 0:
        # Interpolate onto bias timestamps
        print(f'  timeoffset count {n_to} vs bias rows {n_bias} — interpolating')
        to_x = np.linspace(b_t[0], b_t[-1], n_to)
        to_vals = np.interp(b_t, to_x, to_vals)
        to_t = b_t
    else:
        to_t = np.array([])
        to_vals = np.array([])
        print('  [WARN] no timeoffset lines in log')

    # -- sliding-window scale data --
    centers = np.array(sw_rec['sliding_window']['centers_rel'])
    ratios = np.array(sw_rec['sliding_window']['ratios'])
    motion_mask = np.array(sw_rec['sliding_window']['motion_mask'], dtype=bool)

    # -- phase markers (from hover_takeoff, but assume same per flight) --
    t_alt_rise = ht_rec.get('t_first_alt_rise_rel')
    t_takeoff = ht_rec.get('t_takeoff_rel')
    first_edge_mean = sw_rec.get('first_edge_scale_mean', None)
    init_disparity = ht_rec.get('init_disparity')
    diverge_onset = sw_rec.get('diverge_onset_rel')

    t0 = b_t[0]
    b_t_rel = b_t - t0

    # -- compute bias_a_z delta over vertical-climb phase --
    # vertical_climb = [t_first_alt_rise, t_xy_takeoff]
    if t_alt_rise is not None and t_takeoff is not None:
        vc_mask = (b_t_rel >= t_alt_rise) & (b_t_rel <= t_takeoff)
        if vc_mask.sum() >= 3:
            d_ba_z_vc = float(b_ba_z[vc_mask][-1] - b_ba_z[vc_mask][0])
            ba_z_end_vc = float(b_ba_z[vc_mask][-1])
        else:
            d_ba_z_vc = float('nan')
            ba_z_end_vc = float('nan')
    else:
        d_ba_z_vc = float('nan')
        ba_z_end_vc = float('nan')

    # -- compute bias_a_z delta over first 30s post-takeoff --
    if t_takeoff is not None:
        tk30_mask = (b_t_rel >= t_takeoff) & (b_t_rel <= t_takeoff + 30.0)
        if tk30_mask.sum() >= 3:
            d_ba_z_tk30 = float(b_ba_z[tk30_mask][-1] - b_ba_z[tk30_mask][0])
        else:
            d_ba_z_tk30 = float('nan')
    else:
        d_ba_z_tk30 = float('nan')

    # Print summary
    print(f'  t_alt_rise={t_alt_rise}  t_takeoff={t_takeoff}  '
          f'diverge_onset={diverge_onset}')
    print(f'  init_disparity = {init_disparity}')
    print(f'  first_edge_scale_mean = {first_edge_mean}')
    print(f'  ba_z @ init          = {b_ba_z[0]:+.6f} m/s²')
    print(f'  ba_z @ takeoff       = {ba_z_end_vc:+.6f} m/s²  '
          f'(Δ over vert-climb = {d_ba_z_vc:+.6f})')
    print(f'  Δ ba_z first 30s tk  = {d_ba_z_tk30:+.6f} m/s²')
    if len(to_vals):
        print(f'  timeoffset range     = [{to_vals.min():.5f}, {to_vals.max():.5f}]  '
              f'Δ = {to_vals[-1] - to_vals[0]:+.5f}')

    # -- return data for downstream summary --
    return dict(
        flight=fly,
        b_t_rel=b_t_rel,
        b_ba_z=b_ba_z,
        b_bg_x=b_bg_x, b_bg_y=b_bg_y, b_bg_z=b_bg_z,
        to_t_rel=(to_t - t0) if len(to_t) else np.array([]),
        to_vals=to_vals,
        centers=centers,
        ratios=ratios,
        motion_mask=motion_mask,
        t_alt_rise=t_alt_rise,
        t_takeoff=t_takeoff,
        diverge_onset=diverge_onset,
        first_edge_mean=first_edge_mean,
        init_disparity=init_disparity,
        d_ba_z_vc=d_ba_z_vc,
        d_ba_z_tk30=d_ba_z_tk30,
        ba_z_init=float(b_ba_z[0]),
        ba_z_takeoff=ba_z_end_vc,
        t0=t0,
    )


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def make_step1_plot(d, out_prefix):
    """One figure per flight: bias_a_z and scale_ratio overlay."""
    if not _HAVE_MPL:
        return

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 1, hspace=0.35,
                          height_ratios=[1.0, 1.0, 1.2])

    # -- Panel A: bias_a_z(t) --
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(d['b_t_rel'], d['b_ba_z'], 'b-', lw=0.8, alpha=0.8,
             label='bias_a_z (m/s²)')
    ax1.axhline(0, color='k', lw=0.5, alpha=0.3)
    _add_phase_lines(ax1, d)
    ax1.set_ylabel('bias_a_z (m/s²)')
    ax1.grid(alpha=0.3)
    ax1.legend(loc='best', fontsize=8)
    ax1.set_title(f'fly{d["flight"]} — Step 1: bias_a_z(t) & sliding-window scale_ratio(t)\n'
                  f'init disparity={d["init_disparity"]}  '
                  f'first-edge scale mean={d["first_edge_mean"]:.3f}  '
                  f'Δba_z(vert-climb)={d["d_ba_z_vc"]:+.5f}')

    # -- Panel B: sliding-window scale ratio --
    ax2 = fig.add_subplot(gs[1])
    _plot_sw_scale(ax2, d)
    _add_phase_lines(ax2, d)
    ax2.set_ylabel('scale ratio (B0/GPS)')
    ax2.grid(alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    # -- Panel C: combined twin-axis view --
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(d['b_t_rel'], d['b_ba_z'], 'b-', lw=0.8, alpha=0.7,
             label='bias_a_z (m/s²)')
    ax3.axhline(0, color='b', lw=0.5, alpha=0.2)
    ax3.set_xlabel('t_rel (s)')
    ax3.set_ylabel('bias_a_z (m/s²)', color='b')
    ax3.tick_params(axis='y', labelcolor='b')

    ax3b = ax3.twinx()
    _plot_sw_scale(ax3b, d, color='r')
    ax3b.set_ylabel('scale ratio (B0/GPS)', color='r')
    ax3b.tick_params(axis='y', labelcolor='r')
    ax3b.axhline(1.0, color='r', lw=0.8, alpha=0.3, ls='-', label='scale=1.0')

    _add_phase_lines(ax3, d)
    ax3.grid(alpha=0.2)
    lines1, labels1 = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3b.get_legend_handles_labels()
    ax3.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)

    fig.tight_layout()
    fig.savefig(f'{out_prefix}_step1_bias_vs_scale.png', dpi=120)
    plt.close(fig)
    print(f'  [saved] {out_prefix}_step1_bias_vs_scale.png')


def make_step2_plot(d, out_prefix):
    """One figure per flight: cam_imu_timeoffset(t) and bg(t)."""
    if not _HAVE_MPL:
        return
    if len(d['to_vals']) == 0:
        print(f'  [SKIP step2] fly{d["flight"]}: no timeoffset data')
        return

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 1, hspace=0.35)

    # -- Panel A: timeoffset(t) --
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(d['to_t_rel'], d['to_vals'], 'g-', lw=1.0,
             label='cam_imu_timeoffset (s)')
    _add_phase_lines(ax1, d)
    ax1.set_ylabel('timeoffset (s)')
    ax1.grid(alpha=0.3)
    ax1.legend(loc='best', fontsize=8)
    to_delta = float(d['to_vals'][-1] - d['to_vals'][0])
    to_range = (float(d['to_vals'].min()), float(d['to_vals'].max()))
    ax1.set_title(f'fly{d["flight"]} — Step 2: cam_imu_timeoffset(t) & bg(t)\n'
                  f'timeoffset range=[{to_range[0]:.5f}, {to_range[1]:.5f}]  '
                  f'Δ={to_delta:+.5f}  init disparity={d["init_disparity"]}')

    # -- Panel B: bg(t) --
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(d['b_t_rel'], d['b_bg_x'], 'r-', lw=0.8, alpha=0.8, label='bg_x')
    ax2.plot(d['b_t_rel'], d['b_bg_y'], 'g-', lw=0.8, alpha=0.8, label='bg_y')
    ax2.plot(d['b_t_rel'], d['b_bg_z'], 'b-', lw=1.0, alpha=0.8, label='bg_z')
    ax2.axhline(0, color='k', lw=0.5, alpha=0.3)
    _add_phase_lines(ax2, d)
    ax2.set_xlabel('t_rel (s)')
    ax2.set_ylabel('bias_g (rad/s)')
    ax2.grid(alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    fig.tight_layout()
    fig.savefig(f'{out_prefix}_step2_timeoffset_bg.png', dpi=120)
    plt.close(fig)
    print(f'  [saved] {out_prefix}_step2_timeoffset_bg.png')


def make_combined_xy_plots(all_data):
    """Cross-flight summary: normalised view overlay."""
    if not _HAVE_MPL or len(all_data) < 4:
        return

    # -- bias_a_z overlay, all flights on one plot --
    fig, axs = plt.subplots(4, 1, figsize=(16, 12), sharex=False)
    for i, d in enumerate(all_data):
        ax = axs[i]
        ax.plot(d['b_t_rel'], d['b_ba_z'], 'b-', lw=0.6, alpha=0.8)
        ax.axhline(0, color='k', lw=0.4, alpha=0.3)
        _add_phase_lines(ax, d)
        ax.set_ylabel('ba_z')
        ax.grid(alpha=0.2)
        ax.set_title(f'fly{d["flight"]}  '
                     f'first-edge scale={d["first_edge_mean"]:.3f}  '
                     f'Δba_z(vc)={d["d_ba_z_vc"]:+.5f}  '
                     f'Δba_z(tk30)={d["d_ba_z_tk30"]:+.5f}',
                     fontsize=9)
    axs[-1].set_xlabel('t_rel (s)')
    fig.suptitle('Step 1 combined: bias_a_z(t) across all flights with phase markers',
                 fontsize=12, y=0.995)
    fig.tight_layout()
    fig.savefig(f'{OUT_DIR}/combined_step1_bias_a_z.png', dpi=120)
    plt.close(fig)
    print(f'\n  [saved] {OUT_DIR}/combined_step1_bias_a_z.png')

    # -- timeoffset overlay, all flights --
    fig, axs = plt.subplots(4, 1, figsize=(16, 10), sharex=False)
    for i, d in enumerate(all_data):
        ax = axs[i]
        if len(d['to_vals']):
            ax.plot(d['to_t_rel'], d['to_vals'], 'g-', lw=0.8)
            _add_phase_lines(ax, d)
            ax.set_ylabel('to (s)')
            ax.grid(alpha=0.2)
            d_to = float(d['to_vals'][-1] - d['to_vals'][0])
            ax.set_title(f'fly{d["flight"]}  cam_imu_timeoffset  '
                         f'Δ={d_to:+.5f}s  first-edge scale={d["first_edge_mean"]:.3f}',
                         fontsize=9)
        else:
            ax.text(0.5, 0.5, 'no data', ha='center', va='center',
                    transform=ax.transAxes)
            ax.set_title(f'fly{d["flight"]} — no timeoffset data', fontsize=9)
    axs[-1].set_xlabel('t_rel (s)')
    fig.suptitle('Step 2 combined: cam_imu_timeoffset(t) across all flights',
                 fontsize=12, y=0.995)
    fig.tight_layout()
    fig.savefig(f'{OUT_DIR}/combined_step2_timeoffset.png', dpi=120)
    plt.close(fig)
    print(f'  [saved] {OUT_DIR}/combined_step2_timeoffset.png')

    # -- bg norm overlay --
    fig, axs = plt.subplots(4, 1, figsize=(16, 10), sharex=False)
    for i, d in enumerate(all_data):
        ax = axs[i]
        ax.plot(d['b_t_rel'], d['b_bg_x'], 'r-', lw=0.6, alpha=0.7, label='bg_x')
        ax.plot(d['b_t_rel'], d['b_bg_y'], 'g-', lw=0.6, alpha=0.7, label='bg_y')
        ax.plot(d['b_t_rel'], d['b_bg_z'], 'b-', lw=0.8, alpha=0.7, label='bg_z')
        ax.axhline(0, color='k', lw=0.4, alpha=0.3)
        _add_phase_lines(ax, d)
        ax.set_ylabel('rad/s')
        ax.grid(alpha=0.2)
        ax.legend(loc='best', fontsize=7)
        ax.set_title(f'fly{d["flight"]}  first-edge scale={d["first_edge_mean"]:.3f}',
                     fontsize=9)
    axs[-1].set_xlabel('t_rel (s)')
    fig.suptitle('Step 2 combined: gyro bias bg(t) across all flights',
                 fontsize=12, y=0.995)
    fig.tight_layout()
    fig.savefig(f'{OUT_DIR}/combined_step2_bg.png', dpi=120)
    plt.close(fig)
    print(f'  [saved] {OUT_DIR}/combined_step2_bg.png')


def _add_phase_lines(ax, d):
    """Add vertical phase markers."""
    if d['t_alt_rise'] is not None:
        ax.axvline(d['t_alt_rise'], color=PHASE_COLORS['alt_rise'],
                   lw=1.0, ls=':', alpha=0.8,
                   label=f'alt_rise @ {d["t_alt_rise"]:.1f}s')
    if d['t_takeoff'] is not None:
        ax.axvline(d['t_takeoff'], color=PHASE_COLORS['xy_takeoff'],
                   lw=1.2, ls='--', alpha=0.8,
                   label=f'xy_takeoff @ {d["t_takeoff"]:.1f}s')
    if d['diverge_onset'] is not None and not np.isnan(d['diverge_onset']):
        ax.axvline(d['diverge_onset'], color=PHASE_COLORS['diverge'],
                   lw=1.0, ls='-.', alpha=0.7,
                   label=f'diverge @ {d["diverge_onset"]:.0f}s')


def _plot_sw_scale(ax, d, color='r'):
    """Plot sliding-window scale ratio, masking low-GPS-motion windows."""
    mask = d['motion_mask']
    centers = d['centers']
    ratios = d['ratios']
    if mask.sum() > 0:
        ax.plot(centers[mask], ratios[mask], '-', color=color, lw=1.5,
                label=f'scale_ratio (motion-masked, n={mask.sum()})')
        ax.scatter(centers[mask], ratios[mask], c=color, s=8, alpha=0.5)
    # Show unmasked too as faint dots
    if (~mask).sum() > 0:
        ax.scatter(centers[~mask], ratios[~mask], c='gray', s=4, alpha=0.3,
                   label=f'low-GPS windows (n={(~mask).sum()})')
    ax.axhline(1.0, color='k', lw=0.8, alpha=0.3, ls='-', label='scale=1.0')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    sw_all = load_sw_summary(SW_JSON)
    ht_all = load_ht_summary(HT_JSON)

    # Index by flight
    sw_by_fly = {r['flight']: r for r in sw_all}
    ht_by_fly = {r['flight']: r for r in ht_all}

    all_data = []
    for cfg in FLIGHTS:
        fly = cfg['flight']
        sw_rec = sw_by_fly.get(fly)
        ht_rec = ht_by_fly.get(fly)
        if sw_rec is None or ht_rec is None:
            print(f'fly{fly}: missing summary data, skipping')
            continue

        d = analyze_flight(cfg, sw_rec, ht_rec)
        if d is None:
            continue
        all_data.append(d)

        # Per-flight output dir
        pfx = f'{OUT_DIR}/fly{fly}'
        os.makedirs(pfx, exist_ok=True)

        make_step1_plot(d, pfx)
        make_step2_plot(d, pfx)

    # Cross-flight combined plots
    make_combined_xy_plots(all_data)

    # -- Summary table --
    print(f'\n{"="*110}')
    print(f'  SUMMARY: bias vs first-edge scale')
    print(f'{"="*110}')
    hdr = (f'{"fly":>4s} {"first_edge":>10s} {"disp_older":>10s} '
           f'{"ba_z_init":>12s} {"ba_z_takeoff":>13s} {"d_ba_z_vc":>10s} '
           f'{"d_ba_z_tk30":>11s} {"to_delta":>10s}')
    print(hdr)
    print('-' * len(hdr))
    for d in all_data:
        disp = d['init_disparity']
        disp_str = f'{disp[0]:.3f}' if disp else '?'
        to_d = float(d['to_vals'][-1] - d['to_vals'][0]) if len(d['to_vals']) > 0 else float('nan')
        print(f'{d["flight"]:>4d} {d["first_edge_mean"]:>10.3f} {disp_str:>10s} '
              f'{d["ba_z_init"]:>+12.6f} {d["ba_z_takeoff"]:>+13.6f} '
              f'{d["d_ba_z_vc"]:>+10.6f} {d["d_ba_z_tk30"]:>+11.6f} '
              f'{to_d:>+10.5f}')

    # -- Hypothesis-relevant observations --
    print(f'\n{"="*70}')
    print(f'  HYPOTHESIS-RELEVANT PATTERNS')
    print(f'{"="*70}')
    # Sort by first-edge scale error (distance from 1.0)
    sorted_by_error = sorted(all_data,
                             key=lambda d: abs(d['first_edge_mean'] - 1.0)
                             if d['first_edge_mean'] is not None else 0)
    print(f'  Sorted by |scale - 1.0| (smallest error first):')
    for d in sorted_by_error:
        fe = d['first_edge_mean']
        err = fe - 1.0
        print(f'    fly{d["flight"]}: scale={fe:.3f} (err={err:+.3f})  '
              f'ba_z_init={d["ba_z_init"]:+.5f}  '
              f'ba_z_takeoff={d["ba_z_takeoff"]:+.5f}  '
              f'd_ba_z_vc={d["d_ba_z_vc"]:+.5f}  '
              f'd_ba_z_tk30={d["d_ba_z_tk30"]:+.5f}')
    # ba_z_init vs scale error
    print(f'\n  H3 check — bias_a_z at init vs scale error:')
    for d in all_data:
        print(f'    fly{d["flight"]}: ba_z_init={d["ba_z_init"]:+.5f}  '
              f'scale_err={d["first_edge_mean"] - 1.0:+.3f}')
    print(f'  H3 check — bias_a_z at takeoff vs scale error:')
    for d in all_data:
        print(f'    fly{d["flight"]}: ba_z_takeoff={d["ba_z_takeoff"]:+.5f}  '
              f'scale_err={d["first_edge_mean"] - 1.0:+.3f}')
    print(f'  H3 check — Δba_z over vertical climb vs scale error:')
    for d in all_data:
        print(f'    fly{d["flight"]}: d_ba_z_vc={d["d_ba_z_vc"]:+.5f}  '
              f'scale_err={d["first_edge_mean"] - 1.0:+.3f}')
    print(f'  H5 check — timeoffset delta vs scale error:')
    for d in all_data:
        if len(d['to_vals']) > 0:
            to_d = float(d['to_vals'][-1] - d['to_vals'][0])
            print(f'    fly{d["flight"]}: to_delta={to_d:+.5f}  '
                  f'scale_err={d["first_edge_mean"] - 1.0:+.3f}')
        else:
            print(f'    fly{d["flight"]}: no timeoffset data')
    print(f'  H1 check — init disparity (older half) vs scale error:')
    for d in all_data:
        disp = d['init_disparity']
        if disp:
            print(f'    fly{d["flight"]}: older_disp={disp[0]:.3f}  '
                  f'scale_err={d["first_edge_mean"] - 1.0:+.3f}')
    print(f'\n  [done] outputs in {OUT_DIR}/')


if __name__ == '__main__':
    main()
