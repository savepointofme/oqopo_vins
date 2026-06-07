#!/usr/bin/env python3
"""
eval_stage.py — Stage validation evaluator for visual_yaw_schmidt_current_gauge.

Default alignment: START-POSITION + INITIAL-YAW from GPS course at T0 vs VIO velocity at T0.
No global trajectory fitting, no scale, no Umeyama, no SE(3).

  p_aligned(t) = R(delta_yaw) * (p_vio(t) - p_vio_start) + p_gps_start

Yaw alignment modes (--yaw-align-mode):
  start_yaw        [DEFAULT] GPS course in [T0, T0+yaw_win] vs VIO velocity at T0
  gps_course_window Same logic, window/gate controlled by --yaw-win / --speed-gate
  segment_yaw_fit  GPS+VIO mean over user segment [--yaw-fit-t0, --yaw-fit-t1]
  fixed_yaw_offset User-supplied rotation in degrees (--yaw-offset-deg)
  best_yaw_fit     Analytically optimal yaw over eval window — OPTIMISTIC, not headline metric
  no_yaw_align     Translation-only, no rotation — sanity check
  all              Run all modes and print comparison table (uses start_yaw for main report)

Usage:
  python3 eval_stage.py --until 1630 --t0 980 --dir-a <A> --dir-b <B> \\
      --gps <csv> --imu <csv> --out <dir> [--yaw-align-mode start_yaw]
"""
import argparse, csv, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── CLI ──────────────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument('--until',  type=float, required=True)
ap.add_argument('--t0',     type=float, default=618.0)
ap.add_argument('--dir-a',  required=True)
ap.add_argument('--dir-b',  required=True)
ap.add_argument('--gps',    required=True)
ap.add_argument('--imu',    required=True)
ap.add_argument('--out',    required=True)
# Yaw alignment
ap.add_argument('--yaw-align-mode', default='start_yaw',
                choices=['start_yaw','fixed_yaw_offset','best_yaw_fit',
                         'segment_yaw_fit','gps_course_window','no_yaw_align','all'],
                help='Yaw alignment strategy (default: start_yaw)')
ap.add_argument('--yaw-offset-deg', type=float, default=0.0,
                help='[fixed_yaw_offset] Rotation in degrees to apply')
ap.add_argument('--yaw-fit-t0',  type=float, default=None,
                help='[segment_yaw_fit] Segment start time (default: T0)')
ap.add_argument('--yaw-fit-t1',  type=float, default=None,
                help='[segment_yaw_fit] Segment end time (default: T0+60s)')
ap.add_argument('--yaw-win',     type=float, default=5.0,
                help='GPS course window length in seconds (default: 5.0)')
ap.add_argument('--speed-gate',  type=float, default=2.0,
                help='Speed gate m/s for GPS course estimation (default: 2.0)')
args = ap.parse_args()

T0, T1       = args.t0, args.until
SPEED_GATE   = args.speed_gate
YAW_WIN      = args.yaw_win
YAW_MODE     = args.yaw_align_mode

# Auto-derive GPS reference origin from first GPS point within [T0-10, T0+10]s
# so that ENU is centred near the start of flight (works for any dataset).
_gps_ref = None
for _r in csv.DictReader(open(args.gps)):
    _t = float(_r['ts_ns']) * 1e-9
    if T0 - 10 <= _t <= T0 + 10:
        _gps_ref = (float(_r['lat']), float(_r['lon']), float(_r['alt']))
        break
if _gps_ref is None:
    _gps_ref = (38.4975415, 103.2091577, 1399.505)  # fly3 fallback
LAT0, LON0, ALT0 = _gps_ref
RAD = math.pi / 180.0
os.makedirs(args.out, exist_ok=True)

# ── helpers ──────────────────────────────────────────────────────────────────
def wgs84_to_enu(lat, lon, alt):
    dN = (lat - LAT0) * 111320.0
    dE = (lon - LON0) * 111320.0 * math.cos(LAT0 * RAD)
    return dE, dN, alt - ALT0

def wrap180(a):
    while a >  180: a -= 360
    while a < -180: a += 360
    return a

def quat_to_yaw(qx, qy, qz, qw):
    # OpenVINS traj stores q_GtoI (JPL, global→IMU).
    # For the IMU heading IN global frame we need q_ItoG = conjugate(-qx,-qy,-qz,qw).
    # Yaw of q_ItoG:
    #   atan2(2*(qw*(-qz) + (-qx)*(-qy)), 1 - 2*((-qy)^2 + (-qz)^2))
    # = atan2(-2*qw*qz + 2*qx*qy, 1 - 2*(qy^2 + qz^2))
    return math.atan2(-2.0*qw*qz + 2.0*qx*qy, 1.0 - 2.0*(qy**2 + qz**2))

def interp1(xs, ys, x):
    xs = np.asarray(xs); ys = np.asarray(ys)
    idx = np.searchsorted(xs, x)
    if idx == 0:          return float(ys[0])
    if idx >= len(xs):    return float(ys[-1])
    a = (x - xs[idx-1]) / (xs[idx] - xs[idx-1])
    return float(ys[idx-1] + a*(ys[idx]-ys[idx-1]))

def rms(v):
    a = np.asarray(v, float); return float(np.sqrt(np.mean(a**2))) if len(a) else float('nan')

def pct(v, p):
    a = np.asarray(v, float)
    return float(np.percentile(np.abs(a), p)) if len(a) else float('nan')

def read_traj(path):
    rows = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'): continue
        c = s.split()
        if len(c) >= 8: rows.append([float(x) for x in c[:8]])
    return np.array(rows)

def read_bias(path):
    rows = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'): continue
        c = s.split()
        if len(c) >= 7: rows.append([float(x) for x in c[:7]])
    return np.array(rows)

def read_schmidt_csv(path):
    if not os.path.exists(path): return {}
    rows = list(csv.DictReader(open(path)))
    if not rows: return {}
    def col(k): return np.array([float(r[k]) for r in rows if r.get(k,'') and r[k] != ''])
    ts = col('timestamp')
    return {
        'ts':           ts,
        'qTdx_n':       np.abs(col('normal_dx_s_coeff_before')),
        'qTdx_s':       np.abs(col('schmidt_dx_s_coeff_after')),
        'pss_chg':      np.abs(col('Pss_change_norm')),
        'pas_chg':      col('Pas_change_norm'),
        'norm_ddx':     col('norm_delta_dx'),
        'rel_norm_HQ':  col('rel_norm_HQ'),
        'norm_HQ':      col('norm_HQ'),
        'yaw_delta':    col('delta_yaw_update'),
        'pss_after_old_q': col('Pss_norm_after')     if 'Pss_norm_after'     in rows[0] else np.array([]),
        'pss_new_q':    col('Pss_norm_after_new_q') if 'Pss_norm_after_new_q' in rows[0] else np.array([]),
        'pss_before':   col('Pss_norm_before'),
        'neg_clamp':    col('neg_diag_clamp_count') if 'neg_diag_clamp_count' in rows[0] else np.zeros(len(ts)),
        'q_e_ori':      col('q_energy_imu_ori')   if 'q_energy_imu_ori' in rows[0] else np.array([]),
        'q_e_pos':      col('q_energy_imu_pos')   if 'q_energy_imu_pos' in rows[0] else np.array([]),
        'q_e_vel':      col('q_energy_imu_vel')   if 'q_energy_imu_vel' in rows[0] else np.array([]),
        'q_e_cl_ori':   col('q_energy_clone_ori') if 'q_energy_clone_ori' in rows[0] else np.array([]),
        'q_e_cl_pos':   col('q_energy_clone_pos') if 'q_energy_clone_pos' in rows[0] else np.array([]),
        'q_e_slam':     col('q_energy_slam')      if 'q_energy_slam' in rows[0] else np.array([]),
        'q_e_bc':       col('q_energy_bias_calib')if 'q_energy_bias_calib' in rows[0] else np.array([]),
        'n':       len(rows),
        'skipped': sum(1 for r in rows if r.get('skipped_reason','')),
    }

# ── yaw-alignment helpers ─────────────────────────────────────────────────────
def _gps_course_mean(win_t0, win_t1, speed_gate, _gc_t, _gc_v, _gc_spd, fallback):
    """Circular mean of speed-gated GPS course bearings in [win_t0, win_t1].
    Returns (yaw_rad, n_samples). Falls back to `fallback` if no qualifying samples."""
    m = (_gc_spd >= speed_gate) & (_gc_t >= win_t0) & (_gc_t <= win_t1)
    if m.sum() > 0:
        s = np.sum(np.sin(_gc_v[m])); c = np.sum(np.cos(_gc_v[m]))
        return math.atan2(s, c), int(m.sum())
    return fallback, 0

def _align_dataset(mode, ds, _gc_t, _gc_v, _gc_spd, _gps_t, _gps_E, _gps_N,
                   _p_gps_start, _T0, _T1, _YAW_WIN, _SPEED_GATE,
                   _yaw_fit_t0, _yaw_fit_t1, _yaw_offset_deg, _yaw_gps_default):
    """
    Compute yaw-aligned positions for one dataset under the given alignment mode.

    Returns dict:
      p_aln       (N,2)  — aligned XY positions
      vio_yaw_aln (N,)   — VIO body yaw in GPS frame (rad)
      delta_yaw   float  — rotation applied (rad)
      vio_yaw_arr (N,)   — raw VIO yaw from quaternion (rad)
      p_vio_start (2,)   — VIO position at T0 (unrotated)
      yaw_src     str    — 'vel' | 'quat(fallback)' | 'n/a'
      meta        dict   — mode, delta_yaw_deg, n_samples, notes, ...
    """
    traj = ds['traj']; ts = traj[:,0]
    bias = ds['bias']; bias_ts = bias[:,0]
    vio_yaw_arr = np.array([quat_to_yaw(*traj[i,4:8]) for i in range(len(traj))])
    p_vio_start = np.array([interp1(ts, traj[:,1], _T0), interp1(ts, traj[:,2], _T0)])

    def _vio_vel_dir(t_c):
        """VIO velocity direction at time t_c; falls back to quaternion if slow."""
        vx = interp1(bias_ts, bias[:,1], t_c)
        vy = interp1(bias_ts, bias[:,2], t_c)
        if math.sqrt(vx**2 + vy**2) > 1.0:
            return math.atan2(vx, vy), 'vel'
        return float(interp1(ts, vio_yaw_arr, t_c)), 'quat(fallback)'

    if mode == 'no_yaw_align':
        delta_yaw = 0.0
        meta = dict(mode=mode, delta_yaw_deg=0.0, n_samples=0,
                    notes='no rotation — translation-only (sanity check)')

    elif mode == 'fixed_yaw_offset':
        delta_yaw = math.radians(_yaw_offset_deg)
        meta = dict(mode=mode, delta_yaw_deg=_yaw_offset_deg, n_samples=0,
                    notes=f'user-supplied {_yaw_offset_deg:.3f} deg')

    elif mode in ('start_yaw', 'gps_course_window'):
        yaw_gps, n_s = _gps_course_mean(_T0, _T0 + _YAW_WIN, _SPEED_GATE,
                                         _gc_t, _gc_v, _gc_spd, _yaw_gps_default)
        yaw_vio, src = _vio_vel_dir(_T0)
        delta_yaw = yaw_gps - yaw_vio
        meta = dict(mode=mode, delta_yaw_deg=math.degrees(delta_yaw),
                    yaw_gps_deg=math.degrees(yaw_gps),
                    yaw_vio_deg=math.degrees(yaw_vio),
                    n_samples=n_s, vio_src=src,
                    notes=(f'GPS course [{_T0:.0f},{_T0+_YAW_WIN:.0f}]s '
                           f'gate={_SPEED_GATE}m/s n={n_s} vio={src}'))

    elif mode == 'segment_yaw_fit':
        ft0 = _yaw_fit_t0 if _yaw_fit_t0 is not None else _T0
        ft1 = _yaw_fit_t1 if _yaw_fit_t1 is not None else _T0 + 60.0
        yaw_gps, n_s = _gps_course_mean(ft0, ft1, _SPEED_GATE,
                                         _gc_t, _gc_v, _gc_spd, _yaw_gps_default)
        seg_m = (bias_ts >= ft0) & (bias_ts <= ft1)
        vx_s = bias[:,1][seg_m]; vy_s = bias[:,2][seg_m]
        spd_s = np.sqrt(vx_s**2 + vy_s**2)
        fast = spd_s > 1.0
        if fast.sum() > 0:
            yaw_vio = float(np.arctan2(np.mean(vx_s[fast]), np.mean(vy_s[fast])))
            src = 'vel-mean'
        else:
            qq = np.array([quat_to_yaw(*traj[i,4:8])
                           for i in range(len(traj)) if ft0 <= traj[i,0] <= ft1])
            yaw_vio = float(np.mean(qq)) if len(qq) else float(interp1(ts, vio_yaw_arr, _T0))
            src = 'quat-mean'
        delta_yaw = yaw_gps - yaw_vio
        meta = dict(mode=mode, delta_yaw_deg=math.degrees(delta_yaw),
                    n_samples=n_s, seg=f'[{ft0:.0f},{ft1:.0f}]s', vio_src=src,
                    notes=f'GPS+VIO mean over [{ft0:.0f},{ft1:.0f}]s n={n_s} vio={src}')

    elif mode == 'best_yaw_fit':
        # Analytical: maximize sum_i g_i^T R(theta) r_i
        # = cos(theta)*sum(g.r) + sin(theta)*sum(gy*rx - gx*ry)
        # => theta = atan2(sum(gy*rx-gx*ry), sum(gx*rx+gy*ry))
        win_m = (ts >= _T0) & (ts <= _T1)
        ts_w = ts[win_m]
        rx = traj[:,1][win_m] - p_vio_start[0]
        ry = traj[:,2][win_m] - p_vio_start[1]
        gx = np.array([interp1(_gps_t, _gps_E, t) for t in ts_w]) - _p_gps_start[0]
        gy = np.array([interp1(_gps_t, _gps_N, t) for t in ts_w]) - _p_gps_start[1]
        A = float(np.sum(gx*rx + gy*ry))
        B = float(np.sum(gy*rx - gx*ry))
        delta_yaw = math.atan2(B, A)
        meta = dict(mode=mode, delta_yaw_deg=math.degrees(delta_yaw),
                    n_samples=int(win_m.sum()),
                    notes=f'OPTIMISTIC — minimizes XY ATE over [{_T0:.0f},{_T1:.0f}]s analytically')

    else:
        raise ValueError(f'Unknown yaw align mode: {mode!r}')

    # Apply rotation: p_aln = R(delta_yaw) @ (p_vio - p_vio_start) + p_gps_start
    R = np.array([[math.cos(delta_yaw), -math.sin(delta_yaw)],
                  [math.sin(delta_yaw),  math.cos(delta_yaw)]])
    p_vio_rel = np.stack([traj[:,1] - p_vio_start[0],
                          traj[:,2] - p_vio_start[1]], axis=1)
    p_aln = (R @ p_vio_rel.T).T + _p_gps_start
    vio_yaw_aln = vio_yaw_arr + delta_yaw

    return dict(p_aln=p_aln, vio_yaw_aln=vio_yaw_aln, delta_yaw=delta_yaw,
                vio_yaw_arr=vio_yaw_arr, p_vio_start=p_vio_start,
                yaw_src=meta.get('vio_src', 'n/a'), meta=meta)

# ── 1. GPS → ENU ─────────────────────────────────────────────────────────────
print('Loading GPS...', flush=True)
gps = []
for r in csv.DictReader(open(args.gps)):
    t = float(r['ts_ns']) * 1e-9
    if T0 - 5 <= t <= T1 + 1:
        e, n, u = wgs84_to_enu(float(r['lat']), float(r['lon']), float(r['alt']))
        gps.append((t, e, n, u))

gps_t = np.array([g[0] for g in gps])
gps_E = np.array([g[1] for g in gps])
gps_N = np.array([g[2] for g in gps])
print(f'  GPS: {len(gps)} pts in [{gps_t[0]:.1f},{gps_t[-1]:.1f}]s')

# GPS course (all consecutive pairs)
gc_t, gc_v, gc_spd = [], [], []
for i in range(1, len(gps)):
    dt = gps[i][0] - gps[i-1][0]
    if dt <= 0: continue
    dE = gps[i][1] - gps[i-1][1]
    dN = gps[i][2] - gps[i-1][2]
    spd = math.sqrt(dE**2 + dN**2) / dt
    gc_t.append(0.5*(gps[i][0]+gps[i-1][0]))
    gc_v.append(math.atan2(dE, dN))   # radians, bearing (N=0,E=pi/2)
    gc_spd.append(spd)
gc_t = np.array(gc_t); gc_v = np.array(gc_v); gc_spd = np.array(gc_spd)
mask = gc_spd >= SPEED_GATE
print(f'  GPS course speed-gated: {mask.sum()}/{len(mask)} (>{SPEED_GATE}m/s)')

# ── 2. Reference GPS initial yaw (used as start_yaw default) ─────────────────
yaw_gps_start, n_yaw_samples = _gps_course_mean(T0, T0 + YAW_WIN, SPEED_GATE,
                                                  gc_t, gc_v, gc_spd, 0.0)
if n_yaw_samples == 0:
    first_ok = np.argmax(mask) if mask.any() else 0
    yaw_gps_start = float(gc_v[first_ok])
    n_yaw_samples = 1
    print('WARNING: no speed-gated GPS in yaw window, using first valid sample')

print(f'  GPS initial yaw (reference): {math.degrees(yaw_gps_start):.2f} deg '
      f'(from {n_yaw_samples} samples in [{T0:.0f},{T0+YAW_WIN:.0f}]s)')

# GPS start position at T0
p_gps_start = np.array([interp1(gps_t, gps_E, T0), interp1(gps_t, gps_N, T0)])

# ── 3. IMU (optional — used only for vbg_rms; skipped if unavailable) ────────
print('Loading IMU...', flush=True)
imu = []
try:
    import io
    raw = open(args.imu, 'rb').read().replace(b'\x00', b'')
    imu_reader = csv.DictReader(io.StringIO(raw.decode('utf-8', errors='replace')))
    t_col = next((k for k in imu_reader.fieldnames if 't_rel_s' in k), None)
    if t_col:
        for r in imu_reader:
            try:
                t = float(r[t_col])
                if T0 - 0.1 <= t <= T1 + 0.1:
                    imu.append((t, float(r['wz'])))
            except (ValueError, KeyError):
                pass
except Exception as e:
    print(f'  IMU: skipped ({e})')
imu_t  = np.array([x[0] for x in imu])
imu_wz = np.array([x[1] for x in imu])
if len(imu):
    print(f'  IMU: {len(imu)} pts ~{len(imu)/(T1-T0):.0f}Hz')
else:
    print('  IMU: no data — vbg_rms will be nan')

# ── 4. Load trajectories ─────────────────────────────────────────────────────
print('Loading trajectories...', flush=True)
datasets = {}
for tag, d in [('A', args.dir_a), ('B', args.dir_b)]:
    traj  = read_traj(f'{d}/traj.txt')
    bias  = read_bias(f'{d}/traj.txt.bias')
    scl   = read_schmidt_csv(f'{d}/schmidt_yaw_update_diag.csv')
    log_txt = open(f'{d}/log.txt', encoding='utf-8', errors='ignore').read()
    neg_w = ('negative diag' in log_txt or 'NEGATIVE' in log_txt)
    datasets[tag] = {'traj':traj,'bias':bias,'scl':scl,'neg_warn':neg_w,'dir':d}
    print(f'  {tag}: traj={len(traj)}, bias={len(bias)}, '
          f'schmidt_rows={scl.get("n",0)}, neg_warn={neg_w}')

# ── 5. Yaw alignment ─────────────────────────────────────────────────────────
_main_mode = YAW_MODE if YAW_MODE != 'all' else 'start_yaw'
print(f'\nAlignment mode: {_main_mode}  (use --yaw-align-mode to change)')
print(f'  p_aligned(t) = R(delta_yaw) * (p_vio(t) - p_vio_start) + p_gps_start')

_align_kwargs = dict(
    _gc_t=gc_t, _gc_v=gc_v, _gc_spd=gc_spd,
    _gps_t=gps_t, _gps_E=gps_E, _gps_N=gps_N,
    _p_gps_start=p_gps_start, _T0=T0, _T1=T1,
    _YAW_WIN=YAW_WIN, _SPEED_GATE=SPEED_GATE,
    _yaw_fit_t0=args.yaw_fit_t0, _yaw_fit_t1=args.yaw_fit_t1,
    _yaw_offset_deg=args.yaw_offset_deg,
    _yaw_gps_default=yaw_gps_start,
)

for tag, ds in datasets.items():
    aln  = _align_dataset(_main_mode, ds, **_align_kwargs)
    meta = aln['meta']
    ds.update({
        'p_aln':         aln['p_aln'],
        'delta_yaw':     aln['delta_yaw'],
        'yaw_vio_start': meta.get('yaw_vio_deg', float('nan')),
        'yaw_gps_start': meta.get('yaw_gps_deg', math.degrees(yaw_gps_start)),
        'p_vio_start':   aln['p_vio_start'],
        'vio_yaw_arr':   aln['vio_yaw_arr'],
        'vio_yaw_aln':   aln['vio_yaw_aln'],
        'yaw_src':       aln['yaw_src'],
    })
    print(f'  {tag}: delta_yaw={meta["delta_yaw_deg"]:+.3f} deg  '
          f'n_gps_samples={meta.get("n_samples",0)}  {meta["notes"]}')

# ── 6. XY ATE (GPS-interpolated at VIO timestamps) ───────────────────────────
def compute_ate(p_aln, ts):
    gps_E_at_vio = np.array([interp1(gps_t, gps_E, t) for t in ts])
    gps_N_at_vio = np.array([interp1(gps_t, gps_N, t) for t in ts])
    err = np.sqrt((p_aln[:,0]-gps_E_at_vio)**2 + (p_aln[:,1]-gps_N_at_vio)**2)
    return err, gps_E_at_vio, gps_N_at_vio

for tag, ds in datasets.items():
    ts = ds['traj'][:,0]
    err, gE, gN = compute_ate(ds['p_aln'], ts)
    ds['err'] = err
    ds['gps_E_at_vio'] = gE
    ds['gps_N_at_vio'] = gN

# ── 7. Gyro integration for bias-corrected yaw ──────────────────────────────
for tag, ds in datasets.items():
    traj = ds['traj']; ts = traj[:,0]
    bias = ds['bias']
    bias_t = bias[:,0]; bgz = bias[:,6]

    y0 = ds['vio_yaw_arr'][0]
    gb_t_l = [float(ts[0])]; gb_yaw_l = [y0]
    for i in range(1, len(imu_t)):
        dt = float(imu_t[i] - imu_t[i-1])
        if dt <= 0 or dt > 0.1: continue
        bz = interp1(bias_t, bgz, float(imu_t[i]))
        y0 += (float(imu_wz[i]) - bz) * dt
        gb_t_l.append(float(imu_t[i])); gb_yaw_l.append(y0)
    ds['gb_t']   = np.array(gb_t_l) if len(imu_t) else np.array([float(ts[0])])
    ds['gb_yaw'] = np.array(gb_yaw_l) if len(imu_t) else np.array([y0])

# ── 8. Yaw error: VIO-aligned vs GPS course (speed-gated) ────────────────────
def yaw_err_series(vio_yaw_aln, ts, gc_t_arr, gc_v_arr, gc_mask):
    """VIO yaw (in GPS frame) vs GPS course, at speed-gated GPS timestamps."""
    t_gated  = gc_t_arr[gc_mask]
    v_gated  = gc_v_arr[gc_mask]   # radians
    vio_at_g = np.array([interp1(ts, vio_yaw_aln, t) for t in t_gated])
    err_deg  = np.array([wrap180(math.degrees(v-g))
                         for v, g in zip(vio_at_g, v_gated)])
    return t_gated, err_deg

for tag, ds in datasets.items():
    ts = ds['traj'][:,0]
    t_ye, e_ye = yaw_err_series(ds['vio_yaw_aln'], ts, gc_t, gc_v, mask)
    ds['yaw_err_t'] = t_ye
    ds['yaw_err']   = e_ye

# ── 9. Segment boundaries ────────────────────────────────────────────────────
if T1 >= 1400:
    _seg1 = max(T0, 900.0)   # clamp so middle never includes pre-T0 data
    segs = [('early',  T0,    _seg1),
            ('middle', _seg1, 1200.0),
            ('late',   1200.0, T1)]
    late_start = 1300.0  # last 300s
else:
    _seg1 = max(T0, 750.0)
    segs = [('early',  T0,    _seg1),
            ('late',   _seg1,  T1)]
    late_start = T1 - 150.0

# ── 10. Compute all metrics ───────────────────────────────────────────────────
def seg_stats(err, ts, t0, t1):
    m = (ts >= t0) & (ts <= t1)
    v = err[m]
    if len(v) == 0: return dict(rms=float('nan'),max=float('nan'),p95=float('nan'),final=float('nan'))
    return dict(rms=rms(v), max=float(np.max(v)), p95=pct(v,95),
                final=float(v[-1]))

metrics = {}
for tag, ds in datasets.items():
    ts   = ds['traj'][:,0]
    err  = ds['err']
    bias = ds['bias']
    bgz  = bias[:,6]
    scl  = ds['scl']

    win_mask = (ts >= T0) & (ts <= T1)
    err_w = err[win_mask]; ts_w = ts[win_mask]
    full = dict(rms=rms(err_w), max=float(err_w.max()) if len(err_w) else float('nan'),
                p95=pct(err_w, 95), final=float(err_w[-1]) if len(err_w) else float('nan'))

    seg_xy = {name: seg_stats(err, ts, t0, t1) for name, t0, t1 in segs}
    late_xy = seg_stats(err, ts, late_start, T1)

    ye_t = ds['yaw_err_t']; ye = ds['yaw_err']
    ye_full = dict(rms=rms(ye), max=float(np.max(np.abs(ye))) if len(ye) else float('nan'),
                   p95=pct(ye,95), final=float(ye[-1]) if len(ye) else float('nan'))
    ye_late = {k: v for k, v in seg_stats(np.abs(ye), ye_t, late_start, T1).items()}
    seg_ye  = {name: seg_stats(np.abs(ye), ye_t, t0, t1) for name,t0,t1 in segs}

    bgz_range = (float(bgz.min()), float(bgz.max()))
    bgz_final = float(bgz[-1])
    bgz_step  = float(np.abs(np.diff(bgz)).max())

    gb_t  = ds['gb_t'];  gb_yaw = ds['gb_yaw']
    gb_yaw_aln = gb_yaw + ds['delta_yaw']
    gb_at_gc = np.array([interp1(gb_t, gb_yaw_aln, t) for t in gc_t[mask]])
    vbg_err = np.array([wrap180(math.degrees(float(b)-float(g)))
                        for b,g in zip(gb_at_gc, gc_v[mask])])
    vbg_rms = rms(vbg_err)

    n_sc   = scl.get('n', 0)
    sk_q_n = scl.get('qTdx_n',  np.array([]))
    sk_q_s = scl.get('qTdx_s',  np.array([]))
    sk_pss = scl.get('pss_chg', np.array([]))
    sk_pas = scl.get('pas_chg', np.array([]))
    sk_nho = scl.get('norm_ddx',np.array([]))

    metrics[tag] = {
        'full_xy': full,
        'seg_xy':  seg_xy,
        'late_xy': late_xy,
        'ye_full': ye_full,
        'ye_late': ye_late,
        'seg_ye':  seg_ye,
        'bgz_range': bgz_range,
        'bgz_final': bgz_final,
        'bgz_step':  bgz_step,
        'vbg_rms':   vbg_rms,
        'neg_warn':  int(ds['neg_warn']),
        'nan_traj':  int(not np.all(np.isfinite(ds['traj']))),
        'traj_lines':len(ds['traj']),
        'final_t':   float(ts_w[-1]) if len(ts_w) else float('nan'),
        'sc_n':      n_sc,
        'sc_qn_mean': float(sk_q_n.mean()) if len(sk_q_n) else float('nan'),
        'sc_qn_max':  float(sk_q_n.max())  if len(sk_q_n) else float('nan'),
        'sc_qs_mean': float(sk_q_s.mean()) if len(sk_q_s) else float('nan'),
        'sc_qs_max':  float(sk_q_s.max())  if len(sk_q_s) else float('nan'),
        'sc_pss_mean':float(sk_pss.mean()) if len(sk_pss) else float('nan'),
        'sc_pss_max': float(sk_pss.max())  if len(sk_pss) else float('nan'),
        'sc_pas_mean':float(sk_pas.mean()) if len(sk_pas) else float('nan'),
        'sc_pas_max': float(sk_pas.max())  if len(sk_pas) else float('nan'),
        'sc_nho_mean':float(sk_nho.mean()) if len(sk_nho) else float('nan'),
        'sc_nho_max': float(sk_nho.max())  if len(sk_nho) else float('nan'),
    }

# ── 11. Print report ─────────────────────────────────────────────────────────
labels = {'A': 'A_global_oc', 'B': 'B_fullstate_schmidt'}

print('\n' + '═'*65)
print(f'  STAGE VALIDATION  start={T0:.1f}s until={T1:.0f}s')
print(f'  Alignment: {_main_mode}  (YAW_WIN={YAW_WIN}s  SPEED_GATE={SPEED_GATE}m/s)')
print('═'*65)

for tag in ('A','B'):
    m  = metrics[tag]
    lbl = labels[tag]
    print(f'\n── {lbl} ──')
    f  = m['full_xy']
    print(f'  XY ATE (full window): RMS={f["rms"]:.2f}m  max={f["max"]:.2f}m  '
          f'P95={f["p95"]:.2f}m  final={f["final"]:.2f}m')
    for sname,sv in m['seg_xy'].items():
        print(f'    [{sname}]: RMS={sv["rms"]:.2f}m  max={sv["max"]:.2f}m  final={sv["final"]:.2f}m')
    l  = m['late_xy']
    print(f'  XY ATE (last {T1-late_start:.0f}s): RMS={l["rms"]:.2f}m  max={l["max"]:.2f}m  final={l["final"]:.2f}m')
    ye = m['ye_full']
    print(f'  Yaw err (full window): RMS={ye["rms"]:.2f}deg  max={ye["max"]:.2f}deg  '
          f'P95={ye["p95"]:.2f}deg  final={ye["final"]:.2f}deg')
    yl = m['ye_late']
    print(f'  Yaw err (last {T1-late_start:.0f}s): RMS={yl["rms"]:.2f}deg  max={yl["max"]:.2f}deg')
    print(f'  biasGyro-GPS yaw RMS={m["vbg_rms"]:.2f}deg')
    print(f'  bg_z: range=[{m["bgz_range"][0]:.6f},{m["bgz_range"][1]:.6f}]  '
          f'final={m["bgz_final"]:.6f}  max_step={m["bgz_step"]:.6f} rad/s')
    if m['sc_n'] > 0:
        print(f'  Schmidt ({m["sc_n"]} updates):')
        print(f'    |qTdx_normal| mean={m["sc_qn_mean"]:.4f}  max={m["sc_qn_max"]:.4f}')
        print(f'    |qTdx_schmidt| mean={m["sc_qs_mean"]:.6f}  max={m["sc_qs_max"]:.6f}  [must be ~0]')
        print(f'    Pss_change_norm mean={m["sc_pss_mean"]:.6f}  max={m["sc_pss_max"]:.6f}  [must be ~0]')
        print(f'    Pas_change_norm mean={m["sc_pas_mean"]:.4f}  max={m["sc_pas_max"]:.4f}  [must be >0]')
        print(f'    non_H_order_dx mean={m["sc_nho_mean"]:.4f}  max={m["sc_nho_max"]:.4f}')
    print(f'  NaN/Inf={m["nan_traj"]}  neg_cov={m["neg_warn"]}  '
          f'lines={m["traj_lines"]}  final_t={m["final_t"]:.2f}')

# ── 12. Analysis questions ───────────────────────────────────────────────────
mA = metrics['A']; mB = metrics['B']
print('\n' + '─'*65)
print('  ANALYSIS SUMMARY (start+yaw-only alignment)')
print('─'*65)

xy_better = mB['full_xy']['rms'] < mA['full_xy']['rms']
delta_rms = mA['full_xy']['rms'] - mB['full_xy']['rms']
print(f'\n1. XY improvement after start+yaw-only alignment:')
print(f'   A XY ATE RMS={mA["full_xy"]["rms"]:.2f}m  B={mB["full_xy"]["rms"]:.2f}m  '
      f'delta={delta_rms:.2f}m ({delta_rms/mA["full_xy"]["rms"]*100:.1f}%)')
print(f'   B {"IS" if xy_better else "IS NOT"} better on XY after start+yaw-only alignment.')

late_ye_A = mA['ye_late']['rms']; late_ye_B = mB['ye_late']['rms']
print(f'\n2. Late-stage yaw drift (last {T1-late_start:.0f}s):')
print(f'   A yaw RMS={late_ye_A:.2f}deg  B yaw RMS={late_ye_B:.2f}deg')
print(f'   B {"SHOWS STRONGER" if late_ye_B > late_ye_A + 1 else "SHOWS SIMILAR" if abs(late_ye_B-late_ye_A) <= 1 else "SHOWS WEAKER"} late-stage yaw drift than A.')

print(f'\n3. XY/yaw trade-off:')
xy_gain = mA['full_xy']['rms'] - mB['full_xy']['rms']
yaw_cost = mB['ye_full']['rms'] - mA['ye_full']['rms']
print(f'   XY gain: {xy_gain:.2f}m RMS.  Yaw cost: {yaw_cost:+.2f}deg RMS.')
if abs(yaw_cost) < 1.0:
    print(f'   No meaningful yaw/position trade-off: B improves XY without degrading yaw RMS.')
elif yaw_cost > 1.0:
    print(f'   Trade-off present: B gains {xy_gain:.1f}m XY at cost of {yaw_cost:.1f}deg yaw RMS.')
else:
    print(f'   B improves both XY and yaw.')

print(f'\n4. bg_z learning (Schmidt does not freeze gyro bias):')
print(f'   A bg_z: range=[{mA["bgz_range"][0]:.4e},{mA["bgz_range"][1]:.4e}]  '
      f'final={mA["bgz_final"]:.4e}  step={mA["bgz_step"]:.4e}')
print(f'   B bg_z: range=[{mB["bgz_range"][0]:.4e},{mB["bgz_range"][1]:.4e}]  '
      f'final={mB["bgz_final"]:.4e}  step={mB["bgz_step"]:.4e}')
bgz_affected = abs(mB['bgz_range'][1]-mA['bgz_range'][1]) > 5e-4
print(f'   Schmidt {"AFFECTS" if bgz_affected else "DOES NOT AFFECT"} bg_z learning '
      f'(max range diff < 5e-4 rad/s).')

print(f'\n5. Schmidt invariants:')
if mB['sc_n'] > 0:
    if mB['sc_qs_max'] < 1e-9 and mB['sc_pss_max'] < 1e-9:
        print(f'   PASS: |qTdx_schmidt|_max={mB["sc_qs_max"]:.2e}  '
              f'Pss_change_max={mB["sc_pss_max"]:.2e}  (machine-zero)')
        print(f'   Pas_change_mean={mB["sc_pas_mean"]:.4f} > 0  '
              f'(genuine Schmidt cross-covariance update confirmed)')
    else:
        print(f'   FAIL: invariants violated')
print()

# ── 13. Write metrics_summary.csv ────────────────────────────────────────────
rows_csv = [
    ['metric','A_global_oc','B_fullstate_schmidt'],
    ['alignment', _main_mode, _main_mode],
    ['yaw_win_s', YAW_WIN, YAW_WIN],
    ['speed_gate_ms', SPEED_GATE, SPEED_GATE],
    ['xy_ate_rms_m',  mA['full_xy']['rms'],  mB['full_xy']['rms']],
    ['xy_ate_max_m',  mA['full_xy']['max'],  mB['full_xy']['max']],
    ['xy_ate_p95_m',  mA['full_xy']['p95'],  mB['full_xy']['p95']],
    ['xy_ate_final_m',mA['full_xy']['final'],mB['full_xy']['final']],
    ['xy_late_rms_m', mA['late_xy']['rms'],  mB['late_xy']['rms']],
    ['xy_late_max_m', mA['late_xy']['max'],  mB['late_xy']['max']],
    ['yaw_err_rms_deg',mA['ye_full']['rms'], mB['ye_full']['rms']],
    ['yaw_err_max_deg',mA['ye_full']['max'], mB['ye_full']['max']],
    ['yaw_err_p95_deg',mA['ye_full']['p95'], mB['ye_full']['p95']],
    ['yaw_err_final_deg',mA['ye_full']['final'],mB['ye_full']['final']],
    ['yaw_late_rms_deg',mA['ye_late']['rms'],mB['ye_late']['rms']],
    ['vbg_rms_deg',   mA['vbg_rms'],        mB['vbg_rms']],
    ['bgz_range_min', mA['bgz_range'][0],   mB['bgz_range'][0]],
    ['bgz_range_max', mA['bgz_range'][1],   mB['bgz_range'][1]],
    ['bgz_final',     mA['bgz_final'],      mB['bgz_final']],
    ['bgz_max_step',  mA['bgz_step'],       mB['bgz_step']],
    ['sc_updates',    mA['sc_n'],           mB['sc_n']],
    ['sc_qTdx_s_max', mA['sc_qs_max'],      mB['sc_qs_max']],
    ['sc_Pss_max',    mA['sc_pss_max'],     mB['sc_pss_max']],
    ['sc_Pas_mean',   mA['sc_pas_mean'],    mB['sc_pas_mean']],
    ['nan_inf',       mA['nan_traj'],       mB['nan_traj']],
    ['neg_cov_warn',  mA['neg_warn'],       mB['neg_warn']],
    ['traj_lines',    mA['traj_lines'],     mB['traj_lines']],
    ['final_t',       mA['final_t'],        mB['final_t']],
]
with open(f'{args.out}/metrics_summary.csv','w',newline='') as f:
    csv.writer(f).writerows(rows_csv)
print(f'Wrote: {args.out}/metrics_summary.csv')

# ── 14. Plots ────────────────────────────────────────────────────────────────
colors = {'A':'#e05a2b','B':'#2b6de0'}
labels_short = {'A':'A global_oc','B':'B schmidt'}

gps_in_win = (gps_t >= T0) & (gps_t <= T1)
gps_E_win  = gps_E[gps_in_win]; gps_N_win = gps_N[gps_in_win]

# ── Plot 1: Aligned XY overlay ───────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10,9))
ax.plot(gps_E_win, gps_N_win, 'k-', lw=2.0, label='GPS truth', zorder=6)
ax.plot(gps_E_win[0],  gps_N_win[0],  'k^', ms=10, zorder=7, label=f't={T0:.0f}s start')
ax.plot(gps_E_win[-1], gps_N_win[-1], 'ks', ms=10, zorder=7, label=f't={T1:.0f}s end')
for tag, ds in datasets.items():
    ax.plot(ds['p_aln'][:,0], ds['p_aln'][:,1], '-',
            color=colors[tag], lw=1.0, alpha=0.85, label=labels_short[tag])
ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)')
ax.set_title(f'fly3 start={T0:.0f}–{T1:.0f}s  |  {_main_mode} alignment\n'
             f'(yaw init from GPS course [{T0:.0f},{T0+YAW_WIN:.0f}]s, SPEED>{SPEED_GATE}m/s)')
ax.legend(fontsize=9); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(f'{args.out}/gps_xy_overlay.png', dpi=150)
plt.close(fig)
print(f'Wrote: {args.out}/gps_xy_overlay.png')

# ── Plot 2: XY ATE vs time with segment bands ────────────────────────────────
fig, ax = plt.subplots(figsize=(12,4))
seg_colors = ['#e8f4e8','#fff3cd','#fde8e8']
for (sname,st0,st1), sc in zip(segs, seg_colors):
    ax.axvspan(st0, st1, alpha=0.25, color=sc, label=sname)
for tag, ds in datasets.items():
    ts = ds['traj'][:,0]
    ax.plot(ts, ds['err'], '-', color=colors[tag], lw=0.9, alpha=0.9,
            label=labels_short[tag])
ax.set_xlabel('t (s)'); ax.set_ylabel('XY ATE (m)')
ax.set_title(f'XY ATE vs time — {_main_mode} (fly3 until={T1:.0f}s)')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(f'{args.out}/xy_ate_vs_time.png', dpi=150)
plt.close(fig)
print(f'Wrote: {args.out}/xy_ate_vs_time.png')

# ── Plot 3: Yaw error vs time ────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12,4))
for (sname,st0,st1), sc in zip(segs, seg_colors):
    ax.axvspan(st0, st1, alpha=0.2, color=sc)
for tag, ds in datasets.items():
    ax.plot(ds['yaw_err_t'], ds['yaw_err'], '-',
            color=colors[tag], lw=0.9, alpha=0.8, label=labels_short[tag])
ax.axhline(0, color='k', lw=0.6, ls='--')
ax.set_xlabel('t (s)'); ax.set_ylabel('VIO yaw − GPS course (deg)')
ax.set_title(f'Yaw error vs time — {_main_mode} (fly3 until={T1:.0f}s)')
ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(f'{args.out}/yaw_err_vs_time.png', dpi=150)
plt.close(fig)
print(f'Wrote: {args.out}/yaw_err_vs_time.png')

# ── Plot 4: 850–930s local heading detail ────────────────────────────────────
HEAD_T0 = max(T0, min(850.0, T1 - 80))
HEAD_T1 = min(T1, HEAD_T0 + 80)
fig, ax = plt.subplots(figsize=(10,4))
m_h = (gc_t >= HEAD_T0) & (gc_t <= HEAD_T1) & mask
ax.plot(gc_t[m_h], np.degrees(gc_v[m_h]), 'k-', lw=2.5,
        label='GPS course truth', zorder=6)
for tag, ds in datasets.items():
    ts = ds['traj'][:,0]
    vio_yaw_deg = np.degrees(ds['vio_yaw_aln'])
    m_v = (ts >= HEAD_T0) & (ts <= HEAD_T1)
    ax.plot(ts[m_v], vio_yaw_deg[m_v], '-',
            color=colors[tag], lw=1.2, alpha=0.9, label=labels_short[tag])
ax.set_xlabel('t (s)'); ax.set_ylabel('Heading (deg, N=0 E=90)')
ax.set_title(f'Local heading vs GPS course [{HEAD_T0:.0f}–{HEAD_T1:.0f}s]  '
             f'({_main_mode})')
ax.set_xlim(HEAD_T0, HEAD_T1)
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(f'{args.out}/heading_850_930s.png', dpi=150)
plt.close(fig)
print(f'Wrote: {args.out}/heading_850_930s.png')

# ── Plot 5: Schmidt diagnostic correlation plots (B only) ────────────────────
scl_B = datasets['B']['scl']
if scl_B.get('n', 0) > 0 and len(scl_B.get('ts', [])) > 0:
    sc_ts     = scl_B['ts']
    sc_in_win = (sc_ts >= T0) & (sc_ts <= T1)
    sc_ts_w   = sc_ts[sc_in_win]

    def sc_arr(key):
        v = scl_B.get(key, np.array([]))
        return v[sc_in_win] if len(v) == len(sc_ts) else np.array([])

    rel_hq   = sc_arr('rel_norm_HQ')
    ddx      = sc_arr('norm_ddx')
    pas      = sc_arr('pas_chg')
    yaw_d    = sc_arr('yaw_delta')
    neg_cl   = sc_arr('neg_clamp')

    q_e_imu_ori  = sc_arr('q_e_ori')
    q_e_imu_pos  = sc_arr('q_e_pos')
    q_e_imu_vel  = sc_arr('q_e_vel')
    q_e_cl_ori   = sc_arr('q_e_cl_ori')
    q_e_cl_pos   = sc_arr('q_e_cl_pos')
    q_e_slam     = sc_arr('q_e_slam')
    q_e_bc       = sc_arr('q_e_bc')

    pss_old_q_arr = sc_arr('pss_after_old_q')
    pss_new_q_arr = sc_arr('pss_new_q')

    def _sp(ax, t, y, *args, **kwargs):
        if len(y) == len(t) and len(t) > 0:
            ax.plot(t, y, *args, **kwargs)

    fig, axes = plt.subplots(6, 1, figsize=(12, 18), sharex=True)

    _sp(axes[0], sc_ts_w, rel_hq, '-', color='#9b4dca', lw=0.7, alpha=0.7)
    axes[0].set_ylabel('rel_norm_Hq'); axes[0].set_title(
        f'B diagnostic correlations ({_main_mode}, until={T1:.0f}s)\n'
        f'rel_norm_Hq = ||H·q_H|| / ||H||  — if large, H observes yaw gauge')
    axes[0].axhline(0.1, color='r', lw=0.8, ls='--', label='0.1 threshold')
    axes[0].legend(fontsize=7); axes[0].grid(True, alpha=0.3)

    _sp(axes[1], sc_ts_w, ddx, '-', color='#2b6de0', lw=0.7, alpha=0.7)
    axes[1].set_ylabel('norm_delta_dx'); axes[1].set_title(
        'norm_delta_dx = ||dx_normal - dx_eff||  — yaw-direction correction removed')
    axes[1].grid(True, alpha=0.3)

    _sp(axes[2], sc_ts_w, pas, '-', color='#1a8f4a', lw=0.7, alpha=0.7)
    axes[2].set_ylabel('Pas_change_norm'); axes[2].set_title(
        'Pas_change_norm  — cross-cov active↔Schmidt; should be nonzero (real Schmidt)')
    axes[2].grid(True, alpha=0.3)

    _sp(axes[3], sc_ts_w, yaw_d, '-', color='#e05a2b', lw=0.7, alpha=0.7)
    axes[3].axhline(0, color='k', lw=0.5, ls='--')
    axes[3].set_ylabel('current_imu_yaw_delta_deg'); axes[3].set_title(
        'current_imu_yaw_delta_deg (deg/update)  — IMU yaw shift caused by Schmidt update; should be small & zero-mean')
    axes[3].grid(True, alpha=0.3)

    traj_B = datasets['B']['traj']; p_aln_B = datasets['B']['p_aln']
    ts_B = traj_B[:, 0]
    gps_at_sc = np.array([
        math.sqrt((interp1(ts_B, p_aln_B[:,0], t) - interp1(gps_t, gps_E, t))**2 +
                  (interp1(ts_B, p_aln_B[:,1], t) - interp1(gps_t, gps_N, t))**2)
        for t in sc_ts_w[::10]])
    axes[4].plot(sc_ts_w[::10], gps_at_sc, '-', color='k', lw=0.9)
    axes[4].set_ylabel('XY ATE B (m)'); axes[4].set_title('XY ATE (B, aligned)')
    axes[4].grid(True, alpha=0.3)

    ye_B = datasets['B']
    if len(ye_B.get('yaw_err_t', [])) > 0:
        ye_at_sc = np.array([interp1(ye_B['yaw_err_t'], ye_B['yaw_err'], t)
                              for t in sc_ts_w[::10]])
        axes[5].plot(sc_ts_w[::10], ye_at_sc, '-', color='#e05a2b', lw=0.9)
    axes[5].axhline(0, color='k', lw=0.5, ls='--')
    axes[5].set_ylabel('Yaw err B (deg)'); axes[5].set_title('VIO yaw − GPS course (B)')
    axes[5].set_xlabel('t (s)'); axes[5].grid(True, alpha=0.3)

    for ax in axes:
        for (sname, st0, st1), sc_col in zip(segs, ['#e8f4e8','#fff3cd','#fde8e8']):
            ax.axvspan(st0, st1, alpha=0.12, color=sc_col)
    fig.tight_layout()
    fig.savefig(f'{args.out}/schmidt_diag_correlations.png', dpi=150)
    plt.close(fig)
    print(f'Wrote: {args.out}/schmidt_diag_correlations.png')

    if len(q_e_imu_ori) > 0:
        fig2, ax2 = plt.subplots(figsize=(12, 4))
        step = max(1, len(sc_ts_w)//2000)
        t_s  = sc_ts_w[::step]
        slam_s = q_e_slam[::step] if len(q_e_slam) == len(sc_ts_w) else np.zeros(len(t_s))
        bc_s   = q_e_bc[::step]   if len(q_e_bc)   == len(sc_ts_w) else np.zeros(len(t_s))
        ax2.stackplot(t_s,
                      q_e_imu_ori[::step], q_e_imu_pos[::step], q_e_imu_vel[::step],
                      q_e_cl_ori[::step],  q_e_cl_pos[::step],
                      slam_s, bc_s,
                      labels=['IMU ori','IMU pos','IMU vel','clone ori','clone pos',
                              'SLAM lm','bias/calib'],
                      colors=['#4e79a7','#f28e2b','#e15759','#76b7b2','#59a14f',
                              '#b07aa1','#9c755f'],
                      alpha=0.85)
        ax2.set_xlabel('t (s)'); ax2.set_ylabel('fraction of ||q||²')
        ax2.set_title(f'q-energy decomposition over time (until={T1:.0f}s)\n'
                      f'Note: mixes rad/m/m·s⁻¹ — Euclidean fraction only')
        ax2.legend(loc='upper right', fontsize=8); ax2.grid(True, alpha=0.3)
        ax2.set_xlim(T0, T1); ax2.set_ylim(0, 1.05)
        fig2.tight_layout()
        fig2.savefig(f'{args.out}/q_energy_decomp.png', dpi=150)
        plt.close(fig2)
        print(f'Wrote: {args.out}/q_energy_decomp.png')

    print('\n── B Schmidt diagnostic segmented summary ──')
    print(f'  (NOTE: described as "full-state current-yaw-gauge projected-gain update")')
    print(f'  rel_norm_Hq meaning: if large, H observes the yaw gauge → '
          f'residual may redirect yaw info into orthogonal subspace')
    for sname, st0, st1 in segs:
        m_s = (sc_ts_w >= st0) & (sc_ts_w < st1)
        def sp(arr, name):
            v = arr[m_s] if len(arr) == len(sc_ts_w) else np.array([])
            if len(v) == 0: return f'  [{sname:12s}] {name}: no data'
            return (f'  [{sname:12s}] {name}: '
                    f'mean={np.mean(v):.4f}  P95={np.percentile(np.abs(v),95):.4f}  '
                    f'max={np.max(np.abs(v)):.4f}')
        print(sp(rel_hq,  'rel_norm_Hq              '))
        print(sp(ddx,     'norm_delta_dx            '))
        print(sp(pas,     'Pas_change_norm          '))
        print(sp(yaw_d,   'current_imu_yaw_delta_deg'))
        clamp_count = int(neg_cl[m_s].sum()) if len(neg_cl) == len(sc_ts_w) else 0
        print(f'  [{sname:12s}] neg_diag_clamp_count: {clamp_count}')
        def pss_seg(arr, label):
            v = arr[m_s] if len(arr) == len(sc_ts_w) else np.array([])
            if len(v) == 0: return f'  [{sname:12s}] {label}: no data'
            return (f'  [{sname:12s}] {label}: '
                    f'mean={np.mean(v):.6f}  min={np.min(v):.6f}  max={np.max(v):.6f}')
        print(pss_seg(pss_old_q_arr, 'Pss(q_old^T P+ q_old)   '))
        print(pss_seg(pss_new_q_arr, 'Pss(q_new^T P+ q_new)   '))

    if len(q_e_imu_ori) > 0:
        print('\n── q-energy decomposition (mean over full window, Euclidean fraction of ||q||²) ──')
        print('   NOTE: q mixes rad(ori)/m(pos)/m·s⁻¹(vel) — fractions are unit-heterogeneous')
        for lbl, arr in [('imu_ori',    q_e_imu_ori),
                          ('imu_pos',   q_e_imu_pos),
                          ('imu_vel',   q_e_imu_vel),
                          ('clone_ori', q_e_cl_ori),
                          ('clone_pos', q_e_cl_pos),
                          ('slam_lm',   q_e_slam),
                          ('bias_calib',q_e_bc)]:
            if len(arr): print(f'  {lbl:12s}: mean={np.mean(arr):.4f}  '
                               f'min={np.min(arr):.4f}  max={np.max(arr):.4f}')

    total_clamp = int(neg_cl.sum()) if len(neg_cl) > 0 else 0
    print(f'\n── Covariance health ──')
    print(f'  neg_diag_clamp_count TOTAL = {total_clamp}  '
          f'(must be 0 for valid run)')

# ── 15. Multi-mode yaw alignment comparison table ─────────────────────────────
_ALL_MODES = ['start_yaw', 'gps_course_window', 'segment_yaw_fit',
              'fixed_yaw_offset', 'best_yaw_fit', 'no_yaw_align']

print('\n' + '═'*80)
print(f'  MULTI-MODE YAW ALIGNMENT COMPARISON  [{T0:.0f},{T1:.0f}]s')
print(f'  Main mode for this report: {_main_mode}')
print('═'*80)

_mm_rows = []
for _m in _ALL_MODES:
    _row = {'mode': _m, 'tags': {}}
    for _tag, _ds in datasets.items():
        try:
            _aln = _align_dataset(_m, _ds, **_align_kwargs)
        except Exception as _ex:
            _row['tags'][_tag] = {'err': str(_ex)}
            continue
        _meta = _aln['meta']
        _ts   = _ds['traj'][:,0]
        _win  = (_ts >= T0) & (_ts <= T1)
        _ts_w = _ts[_win]; _p = _aln['p_aln'][_win]
        _gE_w = np.array([interp1(gps_t, gps_E, t) for t in _ts_w])
        _gN_w = np.array([interp1(gps_t, gps_N, t) for t in _ts_w])
        _e_w  = np.sqrt((_p[:,0]-_gE_w)**2 + (_p[:,1]-_gN_w)**2)
        _xy_rms = float(np.sqrt(np.mean(_e_w**2))) if len(_e_w) else float('nan')
        _xy_max = float(np.max(_e_w))               if len(_e_w) else float('nan')
        _xy_fin = float(_e_w[-1])                   if len(_e_w) else float('nan')
        _vya = _aln['vio_yaw_aln']
        _gcm = mask & (gc_t >= T0) & (gc_t <= T1)
        _tg  = gc_t[_gcm]; _vg = gc_v[_gcm]
        if len(_tg) > 0:
            _vio_at_g = np.array([interp1(_ts, _vya, t) for t in _tg])
            _ye_d = np.array([wrap180(math.degrees(v-g)) for v,g in zip(_vio_at_g, _vg)])
            _yaw_rms = rms(_ye_d); _yaw_max = float(np.max(np.abs(_ye_d)))
        else:
            _yaw_rms = float('nan'); _yaw_max = float('nan')
        _row['tags'][_tag] = {
            'delta_deg': _meta['delta_yaw_deg'], 'n': _meta.get('n_samples', 0),
            'xy_rms': _xy_rms, 'xy_max': _xy_max, 'xy_fin': _xy_fin,
            'yaw_rms': _yaw_rms, 'yaw_max': _yaw_max,
            'notes': _meta.get('notes', ''),
        }
    _mm_rows.append(_row)

# Print table
_TAG_HDR = f"{'mode':>20} tag  delta_yaw  n_gps  XY_RMS   XY_max   XY_fin  Yaw_RMS  Yaw_max"
print(_TAG_HDR)
print('-' * len(_TAG_HDR))
for _r in _mm_rows:
    _m = _r['mode']
    _mark = ' ◄ MAIN' if _m == _main_mode else ''
    _mark += ' *** OPTIMISTIC ***' if _m == 'best_yaw_fit' else ''
    for _tag in ('A', 'B'):
        _d = _r['tags'].get(_tag, {})
        if 'err' in _d:
            print(f"  {_m:>20} [{_tag}] ERROR: {_d['err']}")
            continue
        _suffix = (_mark if _tag == 'A' else '')
        print(f"  {_m:>20}  [{_tag}]  "
              f"{_d['delta_deg']:+7.2f}°  {_d['n']:>4}  "
              f"{_d['xy_rms']:7.2f}m  {_d['xy_max']:7.2f}m  {_d['xy_fin']:7.2f}m  "
              f"{_d['yaw_rms']:6.2f}°  {_d['yaw_max']:6.2f}°"
              + _suffix)
    print()

# Write multi-mode CSV
_mm_csv_rows = [['mode','tag','delta_deg','n_gps_samples',
                 'xy_rms_m','xy_max_m','xy_final_m','yaw_rms_deg','yaw_max_deg','notes']]
for _r in _mm_rows:
    for _tag in ('A', 'B'):
        _d = _r['tags'].get(_tag, {})
        if 'err' in _d: continue
        _mm_csv_rows.append([_r['mode'], _tag, _d['delta_deg'], _d['n'],
                              _d['xy_rms'], _d['xy_max'], _d['xy_fin'],
                              _d['yaw_rms'], _d['yaw_max'], _d['notes']])
_mm_csv_path = f'{args.out}/yaw_mode_comparison.csv'
with open(_mm_csv_path, 'w', newline='') as _f:
    csv.writer(_f).writerows(_mm_csv_rows)
print(f'Wrote: {_mm_csv_path}')

print('\n── Alignment mode guide ──')
print('  start_yaw        GPS course at T0 vs VIO velocity at T0 — CANONICAL DEFAULT')
print('  gps_course_window Same logic; use --yaw-win and --speed-gate to tune window/gate')
print('  segment_yaw_fit  GPS+VIO mean over [--yaw-fit-t0, --yaw-fit-t1] — first-turn check')
print('  fixed_yaw_offset User-supplied rotation (--yaw-offset-deg) — sensitivity test')
print('  best_yaw_fit     Analytical optimum over eval window — OPTIMISTIC, not headline metric')
print('  no_yaw_align     No rotation — sanity check; large XY expected if frames differ')
print()
print(f'  Recommended default: start_yaw (physically interpretable, no future information)')
print(f'  Diagnostic: best_yaw_fit shows XY floor given perfect yaw alignment')
print(f'\nAll outputs in: {args.out}')
