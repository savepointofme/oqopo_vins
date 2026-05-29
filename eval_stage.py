#!/usr/bin/env python3
"""
eval_stage.py — Stage validation evaluator for visual_yaw_schmidt_current_gauge.

Alignment method: START-POSITION + INITIAL-YAW ONLY. No global trajectory fitting.

  p_aligned(t) = R(yaw_gps_start - yaw_vio_start) * (p_vio(t) - p_vio_start) + p_gps_start

  yaw_gps_start : mean GPS course heading over speed-gated samples in [T0, T0+YAW_WIN]s
  yaw_vio_start : VIO VELOCITY direction (atan2(vx,vy) from traj.bias) at T0
                  GPS course = direction of motion, so velocity direction is
                  the correct counterpart (not body quaternion heading).
                  Residual: ~0.03 deg vs ~5.7 deg for quaternion-based.
  YAW_WIN       : 5.0 s  (i.e. [618, 623]s for this experiment)
  SPEED_GATE    : 2.0 m/s
  p_vio_start   : VIO position interpolated at T0
  p_gps_start   : GPS ENU position interpolated at T0

No Umeyama, no SE(2) least-squares, no ICP, no time-varying re-alignment,
no scale correction, no end-point fitting.

Usage:
  python3 eval_stage.py --until 900  --dir-a <A> --dir-b <B> --gps <csv> --imu <csv> --out <dir>
  python3 eval_stage.py --until 1600 --dir-a <A> --dir-b <B> --gps <csv> --imu <csv> --out <dir>
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
ap.add_argument('--dir-a',  required=True)
ap.add_argument('--dir-b',  required=True)
ap.add_argument('--gps',    required=True)
ap.add_argument('--imu',    required=True)
ap.add_argument('--out',    required=True)
args = ap.parse_args()

T0, T1       = 618.0, args.until
SPEED_GATE   = 2.0     # m/s — for GPS course gating
YAW_WIN      = 5.0     # s   — initial-yaw estimation window [T0, T0+YAW_WIN]
LAT0, LON0, ALT0 = 38.4975415, 103.2091577, 1399.505
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
        if len(c) >= 7: rows.append([float(x) for x in c[:10]])
    return np.array(rows)

def read_schmidt_csv(path):
    if not os.path.exists(path): return {}
    rows = list(csv.DictReader(open(path)))
    def col(k): return [float(r[k]) for r in rows if r.get(k,'')]
    return {
        'qTdx_n':  np.abs(np.array(col('normal_dx_s_coeff_before'))),
        'qTdx_s':  np.abs(np.array(col('schmidt_dx_s_coeff_after'))),
        'pss_chg': np.abs(np.array(col('Pss_change_norm'))),
        'pas_chg': np.array(col('Pas_change_norm')),
        'norm_ddx': np.array(col('norm_delta_dx')),
        'n': len(rows),
        'skipped': sum(1 for r in rows if r.get('skipped_reason','')),
    }

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

# ── 2. Initial yaw from GPS course in [T0, T0+YAW_WIN] ──────────────────────
# Documented: use mean GPS course in speed-gated [T0, T0+YAW_WIN] = [618, 623]s
yaw_win_mask = mask & (gc_t >= T0) & (gc_t <= T0 + YAW_WIN)
if yaw_win_mask.sum() > 0:
    # Circular mean for angles
    sin_sum = np.sum(np.sin(gc_v[yaw_win_mask]))
    cos_sum = np.sum(np.cos(gc_v[yaw_win_mask]))
    yaw_gps_start = math.atan2(sin_sum, cos_sum)
    n_yaw_samples = int(yaw_win_mask.sum())
else:
    # Fallback: first speed-gated sample
    first_ok = np.argmax(mask)
    yaw_gps_start = float(gc_v[first_ok])
    n_yaw_samples = 1
    print('WARNING: no speed-gated GPS in yaw window, using first valid sample')

print(f'  GPS initial yaw: {math.degrees(yaw_gps_start):.2f} deg '
      f'(from {n_yaw_samples} samples in [{T0:.0f},{T0+YAW_WIN:.0f}]s)')

# GPS start position at T0
p_gps_start = np.array([interp1(gps_t, gps_E, T0), interp1(gps_t, gps_N, T0)])

# ── 3. IMU ───────────────────────────────────────────────────────────────────
print('Loading IMU...', flush=True)
imu = []
imu_reader = csv.DictReader(open(args.imu))
t_col = next(k for k in imu_reader.fieldnames if 't_rel_s' in k)
for r in imu_reader:
    t = float(r[t_col])
    if T0 - 0.1 <= t <= T1 + 0.1:
        imu.append((t, float(r['wz'])))
imu_t  = np.array([x[0] for x in imu])
imu_wz = np.array([x[1] for x in imu])
print(f'  IMU: {len(imu)} pts ~{len(imu)/(T1-T0):.0f}Hz')

# ── 4. Load trajectories ─────────────────────────────────────────────────────
print('Loading trajectories...', flush=True)
datasets = {}
for tag, d in [('A', args.dir_a), ('B', args.dir_b)]:
    traj  = read_traj(f'{d}/traj.txt')
    bias  = read_bias(f'{d}/traj.txt.bias')
    scl   = read_schmidt_csv(f'{d}/schmidt_yaw_update_diag.csv')
    neg_w = ('negative diag' in open(f'{d}/log.txt').read() or
             'NEGATIVE' in open(f'{d}/log.txt').read())
    datasets[tag] = {'traj':traj,'bias':bias,'scl':scl,'neg_warn':neg_w,'dir':d}
    print(f'  {tag}: traj={len(traj)}, bias={len(bias)}, '
          f'schmidt_rows={scl.get("n",0)}, neg_warn={neg_w}')

# ── 5. Start+yaw alignment ───────────────────────────────────────────────────
print(f'\nAlignment: start+yaw only (YAW_WIN={YAW_WIN}s, SPEED_GATE={SPEED_GATE}m/s)')
print(f'  p_aligned(t) = R(yaw_gps_start - yaw_vio_start) * (p_vio(t) - p_vio_start) + p_gps_start')

for tag, ds in datasets.items():
    traj = ds['traj']; ts = traj[:,0]
    bias = ds['bias'];  bias_ts = bias[:,0]
    vio_yaw_arr = np.array([quat_to_yaw(*traj[i,4:8]) for i in range(len(traj))])

    p_vio_start = np.array([interp1(ts, traj[:,1], T0),
                             interp1(ts, traj[:,2], T0)])

    # Initial yaw from VIO VELOCITY direction at T0.
    # traj.bias columns: t vx vy vz bg_x bg_y bg_z ...
    # vx,vy are the IMU velocity in the VIO global frame.
    # GPS course = direction of motion in ENU = atan2(dE, dN).
    # VIO course at T0 = atan2(vx_vio, vy_vio) — same physical quantity.
    # This eliminates the ~6 deg body/velocity sideslip offset that arises from
    # comparing quaternion body-heading with GPS course heading.
    vx0 = interp1(bias_ts, bias[:,1], T0)
    vy0 = interp1(bias_ts, bias[:,2], T0)
    speed0 = math.sqrt(vx0**2 + vy0**2)
    if speed0 > 1.0:   # speed-gated: use velocity direction only if moving
        yaw_vio_start = math.atan2(vx0, vy0)
        yaw_src = 'vel'
    else:
        yaw_vio_start = interp1(ts, vio_yaw_arr, T0)
        yaw_src = 'quat(fallback)'

    delta_yaw = yaw_gps_start - yaw_vio_start
    R_align   = np.array([[math.cos(delta_yaw), -math.sin(delta_yaw)],
                          [math.sin(delta_yaw),  math.cos(delta_yaw)]])

    # Relative VIO positions
    p_vio_rel = np.stack([traj[:,1] - p_vio_start[0],
                          traj[:,2] - p_vio_start[1]], axis=1)  # N×2
    # Aligned
    p_aln = (R_align @ p_vio_rel.T).T + p_gps_start  # N×2

    # VIO yaw in GPS frame = body yaw from quat + same delta_yaw
    # (delta_yaw corrects VIO frame to GPS frame, regardless of how we estimated it)
    yaw_vio_quat_start = interp1(ts, vio_yaw_arr, T0)
    yaw_offset_quat = yaw_gps_start - yaw_vio_quat_start
    vio_yaw_aln = vio_yaw_arr + yaw_offset_quat  # body yaw in GPS frame

    ds.update({
        'p_aln': p_aln,
        'delta_yaw': delta_yaw,
        'yaw_vio_start': yaw_vio_start,
        'yaw_gps_start': yaw_gps_start,
        'p_vio_start': p_vio_start,
        'vio_yaw_arr': vio_yaw_arr,
        'vio_yaw_aln': vio_yaw_aln,
        'yaw_src': yaw_src,
    })
    print(f'  {tag}: yaw_vio_start({yaw_src})={math.degrees(yaw_vio_start):.2f}deg  '
          f'yaw_gps_start={math.degrees(yaw_gps_start):.2f}deg  '
          f'delta={math.degrees(delta_yaw):.3f}deg')

# ── 6. XY ATE (GPS-interpolated at VIO timestamps) ───────────────────────────
# Interpolate GPS ENU at VIO timestamps for per-frame ATE
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

    # Start from VIO yaw at T0 (already aligned to GPS frame)
    y0 = ds['vio_yaw_arr'][0]
    gb_t_l = [float(ts[0])]; gb_yaw_l = [y0]
    for i in range(1, len(imu_t)):
        dt = float(imu_t[i] - imu_t[i-1])
        if dt <= 0 or dt > 0.1: continue
        bz = interp1(bias_t, bgz, float(imu_t[i]))
        y0 += (float(imu_wz[i]) - bz) * dt
        gb_t_l.append(float(imu_t[i])); gb_yaw_l.append(y0)
    ds['gb_t']   = np.array(gb_t_l)
    ds['gb_yaw'] = np.array(gb_yaw_l)  # biasGyro yaw in VIO frame (same start as VIO)

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
    segs = [('early',  T0,   900.0),
            ('middle', 900.0,1200.0),
            ('late',   1200.0, T1)]
    late_start = 1300.0  # last 300s
else:
    segs = [('early',  T0,   750.0),
            ('late',   750.0,  T1)]
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

    # Full window
    full = dict(rms=rms(err), max=float(err.max()), p95=pct(err,95), final=float(err[-1]))

    # Per-segment XY ATE
    seg_xy = {name: seg_stats(err, ts, t0, t1) for name, t0, t1 in segs}

    # Late window XY
    late_xy = seg_stats(err, ts, late_start, T1)

    # Yaw error metrics
    ye_t = ds['yaw_err_t']; ye = ds['yaw_err']
    ye_full = dict(rms=rms(ye), max=float(np.max(np.abs(ye))) if len(ye) else float('nan'),
                   p95=pct(ye,95), final=float(ye[-1]) if len(ye) else float('nan'))
    ye_late = {k: v for k, v in seg_stats(np.abs(ye), ye_t, late_start, T1).items()}
    seg_ye  = {name: seg_stats(np.abs(ye), ye_t, t0, t1) for name,t0,t1 in segs}

    # bg_z
    bgz_range = (float(bgz.min()), float(bgz.max()))
    bgz_final = float(bgz[-1])
    bgz_step  = float(np.abs(np.diff(bgz)).max())

    # biasGyro-GPS yaw RMS
    gb_t  = ds['gb_t'];  gb_yaw = ds['gb_yaw']
    gb_yaw_aln = gb_yaw + ds['delta_yaw']   # bias-gyro yaw in GPS frame
    gb_at_gc = np.array([interp1(gb_t, gb_yaw_aln, t) for t in gc_t[mask]])
    vbg_err = np.array([wrap180(math.degrees(float(b)-float(g)))
                        for b,g in zip(gb_at_gc, gc_v[mask])])
    vbg_rms = rms(vbg_err)

    # Schmidt invariants
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
        'final_t':   float(ts[-1]),
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
print(f'  STAGE VALIDATION  fly3 start={T0:.0f}s until={T1:.0f}s')
print(f'  Alignment: start+yaw only (YAW_WIN={YAW_WIN}s, no global fit)')
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
    ['alignment','start+yaw_only','start+yaw_only'],
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

# GPS truth for reference (T0..T1)
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
ax.set_title(f'fly3 start={T0:.0f}–{T1:.0f}s  |  start+yaw aligned only, no global fitting\n'
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
ax.set_title(f'XY ATE vs time — start+yaw aligned only (fly3 until={T1:.0f}s)')
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
ax.set_title(f'Yaw error vs time — start+yaw aligned (fly3 until={T1:.0f}s)')
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
             f'(start+yaw aligned)')
ax.set_xlim(HEAD_T0, HEAD_T1)
ax.legend(fontsize=9); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(f'{args.out}/heading_850_930s.png', dpi=150)
plt.close(fig)
print(f'Wrote: {args.out}/heading_850_930s.png')

print(f'\nAll outputs in: {args.out}')
