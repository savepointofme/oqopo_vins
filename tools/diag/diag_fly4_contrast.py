#!/usr/bin/env python3
"""fly4 contrast diagnostic: overlay scale_ratio with ba_z(t), v_z(t),
timeoffset(t), and GPS metrics for all four flights.

Answers:
  - When does the scale error appear? (during vertical climb, at transition, or later)
  - Is ba_z sign-consistent through the vertical-to-horizontal transition?
  - Is vertical-climb observability different for fly2/fly3 vs fly1/fly4?
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
FLIGHTS = [
    dict(flight=1,
         bias='20260509_fly1/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly1/result/baselines_v1/B0_no_gps.log',
         gps='20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv'),
    dict(flight=2,
         bias='20260509_fly2/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly2/result/baselines_v1/B0_no_gps.log',
         gps='20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv'),
    dict(flight=3,
         bias='20260509_fly3/result/baselines_v1/B0_no_gps.txt.bias',
         log='20260509_fly3/result/baselines_v1/B0_no_gps.log',
         gps='20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
    dict(flight=4,
         bias='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.txt.bias',
         log='20260509_fly4/result/stage_a_v2/R0_fresh_no_gps.log',
         gps='20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
]

SW_JSON = 'comparison_plots/sliding_window_scale/summary.json'
HT_JSON = 'comparison_plots/hover_takeoff_scale/summary.json'
INIT_AUDIT_JSON = 'comparison_plots/stage_b_diag/init_window_audit.json'
OUT_DIR = 'comparison_plots/stage_b_diag'
os.makedirs(OUT_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

def load_bias(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            if len(ps) < 10: continue
            try: rows.append([float(x) for x in ps[:10]])
            except ValueError: continue
    a = np.array(rows)
    if a.size:
        keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
        a = a[keep]
    return a  # [N,10]: t, vx, vy, vz, bg_x, bg_y, bg_z, ba_x, ba_y, ba_z


def parse_timeoffset_log(log_path):
    vals = []
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    with open(log_path, errors='replace') as f:
        for ln in f:
            ln = ansi.sub('', ln)
            m = re.search(r'camera-imu timeoffset = ([-\d.]+)', ln)
            if m: vals.append(float(m.group(1)))
    return np.array(vals)


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


def load_gps_enu_window(path, t_lo, t_hi):
    g = load_gps_llz(path)
    if g.size == 0: return np.empty((0, 4))
    mask = (g[:, 0] >= t_lo) & (g[:, 0] <= t_hi)
    g = g[mask]
    if g.size < 5: return np.empty((0, 4))
    return gps_to_enu(g)


# ---------------------------------------------------------------------------
# Per-flight analysis + plot
# ---------------------------------------------------------------------------

def analyze_flight(cfg, sw_rec, ht_rec, init_split=None):
    fly = cfg['flight']
    print(f'\n{"="*70}\n  fly{fly}\n{"="*70}')

    bias = load_bias(cfg['bias'])
    if bias.size == 0: return None
    b_t = bias[:, 0]
    b_ba_z = bias[:, 9]
    b_vx = bias[:, 1]; b_vy = bias[:, 2]; b_vz = bias[:, 3]
    b_v_xy = np.sqrt(b_vx**2 + b_vy**2)

    to_vals = parse_timeoffset_log(cfg['log'])
    n_to, n_b = len(to_vals), len(b_t)
    if n_to > 0 and abs(n_to - n_b) <= 10:
        n = min(n_to, n_b)
        to_t = b_t[:n]; to_vals = to_vals[:n]
    elif n_to > 0:
        to_x = np.linspace(b_t[0], b_t[-1], n_to)
        to_vals = np.interp(b_t, to_x, to_vals)
        to_t = b_t
    else:
        to_t = np.array([]); to_vals = np.array([])

    t0 = b_t[0]
    b_t_rel = b_t - t0
    if len(to_t): to_rel = to_t - t0

    # Sliding window scale
    centers = np.array(sw_rec['sliding_window']['centers_rel'])
    ratios = np.array(sw_rec['sliding_window']['ratios'])
    motion_mask = np.array(sw_rec['sliding_window']['motion_mask'], dtype=bool)
    sw_b0_path = np.array(sw_rec['sliding_window']['b0_xy_path'])
    sw_gps_path = np.array(sw_rec['sliding_window']['gps_xy_path'])

    # Phase markers
    t_alt_rise = ht_rec.get('t_first_alt_rise_rel')
    t_takeoff = ht_rec.get('t_takeoff_rel')
    diverge_onset = sw_rec.get('diverge_onset_rel')
    first_edge_mean = sw_rec.get('first_edge_scale_mean')

    # Init split point (from audit)
    init_rel = None
    if init_split is not None:
        init_rel = init_split - t0

    # GPS data
    t_hi_abs = sw_rec['overlap_t_hi']
    gps_win = load_gps_enu_window(cfg['gps'], t0, t_hi_abs + 60)
    if gps_win.size:
        gps_t_rel = gps_win[:, 0] - t0
        gps_z = gps_win[:, 3]
        gps_xy = gps_win[:, 1:3]
        # Cumulative GPS path from takeoff
        if t_takeoff is not None and gps_win.size > 5:
            gps_tk = gps_t_rel >= t_takeoff
            if gps_tk.sum() > 1:
                gps_tk_idx = np.where(gps_tk)[0]
                gps_cum_xy = np.cumsum(np.concatenate([[0], np.sqrt(
                    np.diff(gps_xy[gps_tk_idx, 0])**2 +
                    np.diff(gps_xy[gps_tk_idx, 1])**2)]))
                gps_cum_z = np.cumsum(np.concatenate([[0], np.abs(
                    np.diff(gps_z[gps_tk_idx]))]))
                gps_tk_t = gps_t_rel[gps_tk_idx]
            else:
                gps_cum_xy = gps_cum_z = gps_tk_t = np.array([])
        else:
            gps_cum_xy = gps_cum_z = gps_tk_t = np.array([])
    else:
        gps_t_rel = gps_z = gps_cum_xy = gps_cum_z = gps_tk_t = np.array([])

    # ---- Numerical answers to timing questions ----
    # 1. Scale ratio in first few sliding windows after takeoff
    print(f'  Phase markers: alt_rise={t_alt_rise}s  takeoff={t_takeoff}s  diverge={diverge_onset}s')
    if init_rel is not None:
        print(f'  Init split point (from raw IMU): t_rel={init_rel:.3f}s  (offset from bias start: {init_rel:.3f}s)')

    if t_takeoff is not None:
        # First few SW windows after takeoff
        post_tk = centers >= t_takeoff
        if post_tk.sum() > 0:
            idx = np.where(post_tk)[0]
            print(f'  Scale ratios in sliding windows after takeoff:')
            for j in idx[:8]:
                mask_str = '✓' if motion_mask[j] else '✗'
                print(f'    SW center={centers[j]:>6.1f}s  ratio={ratios[j]:>8.3f}  '
                      f'motion_mask={mask_str}')
        # ba_z at takeoff
        if len(b_t_rel) > 0:
            tk_pos = np.argmin(np.abs(b_t_rel - t_takeoff))
            ba_z_tk = b_ba_z[tk_pos]
            print(f'  ba_z @ takeoff = {ba_z_tk:+.5f}')
            ba_z_20s_later = b_ba_z[np.argmin(np.abs(b_t_rel - (t_takeoff + 20.0)))]
            print(f'  ba_z @ takeoff+20s = {ba_z_20s_later:+.5f}')
            # ba_z sign consistency through transition
            vc_mask = (b_t_rel >= (t_alt_rise or 0)) & (b_t_rel <= t_takeoff + 30)
            if vc_mask.sum() > 5:
                ba_z_vc = b_ba_z[vc_mask]
                ba_z_sign_flips = int(np.sum(np.abs(np.diff(np.sign(ba_z_vc))) > 0))
                print(f'  ba_z sign consistency (alt_rise..takeoff+30s): '
                      f'mean={ba_z_vc.mean():+.5f}  sign_flips={ba_z_sign_flips}')

    # ---- GPS vertical ratio ----
    if t_takeoff is not None and gps_win.size:
        gps_tk30 = (gps_t_rel >= t_takeoff) & (gps_t_rel <= t_takeoff + 30)
        if gps_tk30.sum() >= 3:
            dz30 = float(np.max(gps_z[gps_tk30]) - gps_z[gps_tk30][0])
            dxy30 = float(np.sum(np.sqrt(np.diff(gps_xy[gps_tk30, 0])**2 + np.diff(gps_xy[gps_tk30, 1])**2)))
            ratio30 = dz30 / max(dxy30, 1e-3)
            print(f'  GPS first 30s post-takeoff: dz={dz30:.1f}m  dxy={dxy30:.1f}m  '
                  f'vertical_ratio={ratio30:.3f}')

        # Vertical ratio during vertical_climb phase
        if t_alt_rise is not None:
            gps_vc = (gps_t_rel >= t_alt_rise) & (gps_t_rel <= t_takeoff)
            if gps_vc.sum() >= 3:
                dz_vc = float(np.max(gps_z[gps_vc]) - gps_z[gps_vc][0])
                dxy_vc = float(np.sum(np.sqrt(np.diff(gps_xy[gps_vc, 0])**2 + np.diff(gps_xy[gps_vc, 1])**2)))
                vc_ratio = dz_vc / max(dxy_vc, 1e-3)
                vc_dur = t_takeoff - t_alt_rise
                print(f'  GPS vertical-climb phase ({vc_dur:.1f}s): dz={dz_vc:.1f}m  '
                      f'dxy={dxy_vc:.1f}m  vertical_ratio={vc_ratio:.3f}')

    return dict(
        flight=fly, t0=t0,
        b_t_rel=b_t_rel, b_ba_z=b_ba_z, b_vz=b_vz, b_v_xy=b_v_xy,
        to_t_rel=(to_t - t0) if len(to_t) else np.array([]),
        to_vals=to_vals,
        centers=centers, ratios=ratios, motion_mask=motion_mask,
        sw_b0_path=sw_b0_path, sw_gps_path=sw_gps_path,
        t_alt_rise=t_alt_rise, t_takeoff=t_takeoff,
        diverge_onset=diverge_onset, init_rel=init_rel,
        first_edge_mean=first_edge_mean,
        gps_t_rel=gps_t_rel, gps_z=gps_z,
        gps_cum_xy=gps_cum_xy, gps_cum_z=gps_cum_z, gps_tk_t=gps_tk_t,
    )


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def make_overlay_plot(d, out_pfx):
    if not _HAVE_MPL: return
    fly = d['flight']

    fig, axs = plt.subplots(4, 1, figsize=(16, 14), sharex=True)
    _add_phase_lines_all(axs, d)

    # --- Panel A: ba_z(t) + scale_ratio ---
    ax = axs[0]
    ax.plot(d['b_t_rel'], d['b_ba_z'], 'b-', lw=0.6, alpha=0.8, label='ba_z')
    ax.axhline(0, color='b', lw=0.4, alpha=0.3)
    ax.set_ylabel('ba_z (m/s²)', color='b')
    ax.tick_params(axis='y', labelcolor='b')
    axb = ax.twinx()
    _plot_sw(axb, d)
    axb.set_ylabel('scale ratio', color='r')
    axb.tick_params(axis='y', labelcolor='r')
    ax.grid(alpha=0.2)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = axb.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)
    ax.set_title(f'fly{fly} — ba_z(t) & scale  |  first-edge scale={d["first_edge_mean"]:.3f}')

    # --- Panel B: v_z(t), v_xy(t) + scale_ratio ---
    ax = axs[1]
    ax.plot(d['b_t_rel'], d['b_vz'], 'b-', lw=0.6, alpha=0.7, label='v_z (B0)')
    ax.plot(d['b_t_rel'], d['b_v_xy'], 'm-', lw=0.6, alpha=0.7, label='|v_xy| (B0)')
    ax.axhline(0, color='k', lw=0.4, alpha=0.3)
    ax.set_ylabel('velocity (m/s)')
    axb = ax.twinx()
    _plot_sw(axb, d)
    axb.set_ylabel('scale ratio', color='r')
    axb.tick_params(axis='y', labelcolor='r')
    ax.grid(alpha=0.2)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = axb.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)

    # --- Panel C: timeoffset(t) + scale_ratio ---
    ax = axs[2]
    if len(d['to_vals']):
        ax.plot(d['to_t_rel'], d['to_vals'], 'g-', lw=1.0, alpha=0.8, label='timeoffset')
        ax.set_ylabel('timeoffset (s)', color='g')
        ax.tick_params(axis='y', labelcolor='g')
    axb = ax.twinx()
    _plot_sw(axb, d)
    axb.set_ylabel('scale ratio', color='r')
    axb.tick_params(axis='y', labelcolor='r')
    ax.grid(alpha=0.2)
    if len(d['to_vals']):
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = axb.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)

    # --- Panel D: GPS cumulative XY and Z from takeoff ---
    ax = axs[3]
    if len(d['gps_tk_t']) > 0:
        ax.plot(d['gps_tk_t'], d['gps_cum_xy'], 'k-', lw=1.0, label='GPS cum XY path (m)')
        ax.plot(d['gps_tk_t'], d['gps_cum_z'], 'k:', lw=1.0, label='GPS cum |Z| path (m)')
        ax.set_ylabel('GPS path (m)')
        ax.legend(loc='upper left', fontsize=8)
    ax.set_xlabel('t_rel (s)')
    ax.grid(alpha=0.2)

    fig.tight_layout()
    fig.savefig(f'{out_pfx}_contrast_overlay.png', dpi=120)
    plt.close(fig)
    print(f'  [saved] {out_pfx}_contrast_overlay.png')


def make_combined_figure(all_data):
    """2x2 comparison: fly2 vs fly3 vs fly4 (fly1 as reference)."""
    if not _HAVE_MPL or len(all_data) < 4: return
    # We want to compare the early phase only (0..200s rel)
    fly_map = {d['flight']: d for d in all_data}

    fig, axs = plt.subplots(4, 1, figsize=(18, 14), sharex=True)
    colors = {1: 'blue', 2: 'orange', 3: 'red', 4: 'green'}

    # Panel A: ba_z overlay all flights
    ax = axs[0]
    for fly in [1, 2, 3, 4]:
        d = fly_map[fly]
        ax.plot(d['b_t_rel'], d['b_ba_z'], color=colors[fly], lw=0.6, alpha=0.7,
                label=f'fly{fly} ba_z (1st-edge scale={d["first_edge_mean"]:.3f})')
    ax.axhline(0, color='k', lw=0.4, alpha=0.3)
    ax.set_ylabel('ba_z (m/s²)'); ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set_title('ba_z(t) — all flights')
    # Phase lines for fly1
    _add_phase_lines_single(ax, fly_map[1])

    # Panel B: scale_ratio overlay (motion-masked)
    ax = axs[1]
    for fly in [1, 2, 3, 4]:
        d = fly_map[fly]
        mask = d['motion_mask']
        ax.plot(d['centers'][mask], d['ratios'][mask], '-', color=colors[fly], lw=1.2,
                label=f'fly{fly} scale')
        ax.scatter(d['centers'][mask], d['ratios'][mask], c=colors[fly], s=8, alpha=0.4)
    ax.axhline(1.0, color='k', lw=0.6, alpha=0.3)
    ax.set_ylabel('scale ratio'); ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set_title('Sliding-window scale ratio (motion-masked) — all flights')

    # Panel C: v_z overlay
    ax = axs[2]
    for fly in [1, 2, 3, 4]:
        d = fly_map[fly]
        ax.plot(d['b_t_rel'], d['b_vz'], color=colors[fly], lw=0.5, alpha=0.6,
                label=f'fly{fly} v_z')
    ax.axhline(0, color='k', lw=0.4, alpha=0.3)
    ax.set_ylabel('v_z (m/s)'); ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set_title('v_z(t) — all flights')

    # Panel D: GPS cum XY
    ax = axs[3]
    for fly in [1, 2, 3, 4]:
        d = fly_map[fly]
        if len(d['gps_tk_t']) > 0:
            ax.plot(d['gps_tk_t'], d['gps_cum_xy'], color=colors[fly], lw=1.2,
                    label=f'fly{fly} GPS cum XY')
    ax.set_xlabel('t_rel (s)'); ax.set_ylabel('GPS XY path (m)')
    ax.grid(alpha=0.2); ax.legend(fontsize=8)
    ax.set_title('GPS cumulative XY path from takeoff — all flights')

    fig.tight_layout()
    fig.savefig(f'{OUT_DIR}/combined_contrast.png', dpi=120)
    plt.close(fig)
    print(f'\n  [saved] {OUT_DIR}/combined_contrast.png')


def _add_phase_lines_all(axs, d):
    for ax in axs[:-1]:
        _add_phase_lines_single(ax, d)
    _add_phase_lines_single(axs[-1], d)


def _add_phase_lines_single(ax, d):
    if d['init_rel'] is not None:
        ax.axvline(d['init_rel'], color='green', lw=1.2, ls='-', alpha=0.7,
                   label=f'init @ {d["init_rel"]:.1f}s')
    if d['t_alt_rise'] is not None:
        ax.axvline(d['t_alt_rise'], color='red', lw=1.0, ls=':', alpha=0.7,
                   label=f'alt_rise @ {d["t_alt_rise"]:.1f}s')
    if d['t_takeoff'] is not None:
        ax.axvline(d['t_takeoff'], color='orange', lw=1.2, ls='--', alpha=0.7,
                   label=f'xy_takeoff @ {d["t_takeoff"]:.1f}s')
    if d['diverge_onset'] is not None and not np.isnan(d['diverge_onset']):
        ax.axvline(d['diverge_onset'], color='purple', lw=1.0, ls='-.', alpha=0.7,
                   label=f'diverge @ {d["diverge_onset"]:.0f}s')


def _plot_sw(ax, d):
    mask = d['motion_mask']
    if mask.sum() > 0:
        ax.plot(d['centers'][mask], d['ratios'][mask], 'r-', lw=1.2,
                label=f'scale (masked)')
        ax.scatter(d['centers'][mask], d['ratios'][mask], c='r', s=6, alpha=0.4)
    if (~mask).sum() > 0:
        ax.scatter(d['centers'][~mask], d['ratios'][~mask], c='gray', s=3, alpha=0.2,
                   label='low-GPS')
    ax.axhline(1.0, color='k', lw=0.6, alpha=0.3, ls='-')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    sw_all = json.load(open(SW_JSON))
    ht_all = json.load(open(HT_JSON))
    sw_by_fly = {r['flight']: r for r in sw_all}
    ht_by_fly = {r['flight']: r for r in ht_all}

    # Load init split points from audit (we computed these)
    # For now, compute them on-the-fly from the .bias file + logged ba/bg
    # Actually, let me load the init split from the audit JSON if available,
    # otherwise use t_init_abs - 1.0 as approximation.
    # From the audit run:
    init_splits = {
        1: 1774463632.600000,
        2: 1774463583.099999,
        3: 1774464295.550002,
        4: 1774463625.700000,
    }

    all_data = []
    for cfg in FLIGHTS:
        fly = cfg['flight']
        sw_rec = sw_by_fly.get(fly)
        ht_rec = ht_by_fly.get(fly)
        if sw_rec is None or ht_rec is None: continue
        d = analyze_flight(cfg, sw_rec, ht_rec, init_splits.get(fly))
        if d is None: continue
        all_data.append(d)
        pfx = os.path.join(OUT_DIR, f'fly{fly}')
        os.makedirs(pfx, exist_ok=True)
        make_overlay_plot(d, pfx)

    make_combined_figure(all_data)

    # ---- Timing summary ----
    print(f'\n\n{"="*80}')
    print(f'  TIMING ANALYSIS: When does scale error appear?')
    print(f'{"="*80}')
    for d in all_data:
        fly = d['flight']
        tk = d['t_takeoff']
        if tk is None: continue
        # First 5 SW windows with motion after takeoff
        mask = d['motion_mask']
        post_tk_mask = d['centers'] >= tk
        post_tk = np.where(post_tk_mask & mask)[0]
        print(f'\n  fly{fly} (takeoff={tk:.1f}s, first_edge_mean={d["first_edge_mean"]:.3f}):')
        print(f'    init_rel={d["init_rel"]:.1f}s  alt_rise={d["t_alt_rise"]}s')
        for j in post_tk[:6]:
            win_lo = d['centers'][j] - 15
            win_hi = d['centers'][j] + 15
            b0_path = d['sw_b0_path'][j]
            gps_path = d['sw_gps_path'][j]
            print(f'    SW [{win_lo:.1f},{win_hi:.1f}]s  '
                  f'ratio={d["ratios"][j]:.4f}  '
                  f'B0_path={b0_path:.1f}m  GPS_path={gps_path:.1f}m')

    # ---- Specific questions ----
    print(f'\n\n{"="*80}')
    print(f'  SPECIFIC QUESTIONS')
    print(f'{"="*80}')

    for d in all_data:
        fly = d['flight']
        print(f'\n--- fly{fly} ---')
        tk = d['t_takeoff']
        if tk is None:
            print('  No takeoff detected'); continue

        # Q: When does scale error appear?
        mask = d['motion_mask']
        post_tk = np.where((d['centers'] >= tk) & mask)[0]
        if len(post_tk) > 0:
            j = post_tk[0]
            first_sw_center = d['centers'][j]
            first_sw_ratio = d['ratios'][j]
            # Was this window during vertical climb or horizontal?
            phase = 'vertical climb' if first_sw_center - 15 < tk else 'horizontal motion'
            print(f'  First motion-masked SW after takeoff: center={first_sw_center:.1f}s  '
                  f'ratio={first_sw_ratio:.3f}  ({phase})')

            # First 3 windows
            for k in post_tk[:3]:
                jj = k
                c = d['centers'][jj]
                print(f'    SW[{c-15:.0f},{c+15:.0f}]s  ratio={d["ratios"][jj]:.4f}')

        # Q: Is ba_z sign-consistent through transition?
        if d['t_alt_rise'] is not None:
            lo = d['t_alt_rise']
        else:
            lo = 0.0
        hi = min(tk + 60, d['b_t_rel'][-1]) if tk is not None else 60
        vc = (d['b_t_rel'] >= lo) & (d['b_t_rel'] <= hi)
        if vc.sum() > 5:
            ba_slice = d['b_ba_z'][vc]
            t_slice = d['b_t_rel'][vc]
            crosses = np.where(np.diff(np.sign(ba_slice)) != 0)[0]
            print(f'  ba_z over [{lo:.1f}, {hi:.1f}]s: mean={ba_slice.mean():+.5f}  '
                  f'min={ba_slice.min():+.5f}  max={ba_slice.max():+.5f}  '
                  f'sign_changes={len(crosses)}')

        # Q: Vertical-climb duration and GPS ratio
        ar = d['t_alt_rise']
        if ar is not None and tk is not None:
            vc_dur = tk - ar
            print(f'  Vertical-climb duration: {vc_dur:.1f}s')
        print(f'  Diverge onset: {d["diverge_onset"]}s')
        print(f'  First-edge scale mean: {d["first_edge_mean"]:.3f}')

    print(f'\n[done] outputs in {OUT_DIR}/fly*/_contrast_overlay.png + combined_contrast.png')


if __name__ == '__main__':
    main()
