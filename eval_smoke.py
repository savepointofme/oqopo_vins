#!/usr/bin/env python3
"""
Full smoke-test evaluation: A vs B, GPS-referenced metrics.
Run:  python3 eval_smoke.py
"""
import csv, math, sys
import numpy as np

ROOT  = '/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527'
GPS_CSV = ROOT + '/gps_from_mems_offset438p0_cam_time.csv'
IMU_CSV = '/mnt/d/vscode_dir/open_vins/imu_window_fly3.csv'  # pre-filtered
SMOKE = ROOT + '/schmidt_smoke_test_start618'
T0, T1 = 618.0, 738.0

LAT0, LON0, ALT0 = 38.4975415, 103.2091577, 1399.505  # ENU anchor
RAD = math.pi / 180.0
SPEED_GATE = 2.0   # m/s for speed-gating

# ── helpers ──────────────────────────────────────────────────────────────────

def wgs84_to_enu(lat, lon, alt):
    dN = (lat - LAT0) * 111320.0
    dE = (lon - LON0) * 111320.0 * math.cos(LAT0 * RAD)
    dU = alt - ALT0
    return dE, dN, dU

def wrap180(a):
    while a >  180: a -= 360
    while a < -180: a += 360
    return a

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy**2 + qz**2))

def interp1(ts, vals, t):
    idx = np.searchsorted(ts, t)
    if idx == 0:     return vals[0]
    if idx >= len(ts): return vals[-1]
    alpha = (t - ts[idx-1]) / (ts[idx] - ts[idx-1])
    return vals[idx-1] + alpha * (vals[idx] - vals[idx-1])

def interp_xy(ts, xs, ys, t):
    return interp1(ts, xs, t), interp1(ts, ys, t)

def rms(arr):
    a = np.asarray(arr, dtype=float)
    return float(np.sqrt(np.mean(a**2))) if len(a) > 0 else float('nan')

# ── 1. GPS ───────────────────────────────────────────────────────────────────

gps = []
for r in csv.DictReader(open(GPS_CSV)):
    t = float(r['ts_ns']) * 1e-9
    if T0 - 1 <= t <= T1 + 1:
        e, n, u = wgs84_to_enu(float(r['lat']), float(r['lon']), float(r['alt']))
        gps.append((t, e, n, u))

gps_t  = np.array([g[0] for g in gps])
gps_E  = np.array([g[1] for g in gps])
gps_N  = np.array([g[2] for g in gps])
print(f'GPS: {len(gps)} samples in [{gps_t[0]:.1f}, {gps_t[-1]:.1f}]s')

# GPS course + speed (between consecutive samples, speed-gated)
gps_course_t, gps_course, gps_spd = [], [], []
for i in range(1, len(gps)):
    dt = gps[i][0] - gps[i-1][0]
    if dt <= 0: continue
    dE = gps[i][1] - gps[i-1][1]
    dN = gps[i][2] - gps[i-1][2]
    spd = math.sqrt(dE**2 + dN**2) / dt
    course = math.degrees(math.atan2(dE, dN))  # bearing: N=0, E=90
    gps_course_t.append(0.5*(gps[i][0]+gps[i-1][0]))
    gps_course.append(course)
    gps_spd.append(spd)

gated_mask = [s >= SPEED_GATE for s in gps_spd]
print(f'GPS course: {sum(gated_mask)}/{len(gps_course)} speed-gated (>{SPEED_GATE} m/s)')

# ── 2. IMU ───────────────────────────────────────────────────────────────────

print(f'Loading IMU...', flush=True)
imu = []
imu_reader = csv.DictReader(open(IMU_CSV))
t_col = next(k for k in imu_reader.fieldnames if 't_rel_s' in k)  # handles '#t_rel_s'
for r in imu_reader:
    t = float(r[t_col])
    if T0 - 0.1 <= t <= T1 + 0.1:
        imu.append((t, float(r['wx']), float(r['wy']), float(r['wz'])))

imu_t  = np.array([x[0] for x in imu])
imu_wz = np.array([x[3] for x in imu])   # raw gyro Z (rad/s, body frame)
print(f'IMU: {len(imu)} samples in [{imu_t[0]:.2f}, {imu_t[-1]:.2f}]s  (~{len(imu)/(T1-T0):.0f} Hz)')

# ── 3. Trajectories + bias files ─────────────────────────────────────────────

def read_traj(path):
    pts = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'): continue
        c = s.split()
        if len(c) >= 8:
            pts.append([float(x) for x in c[:8]])
    return np.array(pts)

def read_bias(path):
    # t vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z
    pts = []
    for line in open(path):
        s = line.strip()
        if not s or s.startswith('#'): continue
        c = s.split()
        if len(c) >= 7:
            pts.append([float(x) for x in c[:10]])
    return np.array(pts)

trajs, biases = {}, {}
for name in ('A_global_oc', 'B_schmidt'):
    trajs[name]  = read_traj(f'{SMOKE}/{name}/traj.txt')
    biases[name] = read_bias(f'{SMOKE}/{name}/traj.txt.bias')
    print(f'{name}: traj={len(trajs[name])} pts, bias={len(biases[name])} pts')

# ── 4. SE(2) alignment (yaw + translation) for each method ───────────────────

def align_vio_to_gps(vio_EN, gps_EN):
    """Umeyama 2-D: minimise ||G - (R*V + t)||^2 over R,t (no scale)."""
    v_mu = vio_EN.mean(0); g_mu = gps_EN.mean(0)
    V = vio_EN - v_mu;     G = gps_EN - g_mu
    H = V.T @ G
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1; R = Vt.T @ U.T
    t = g_mu - R @ v_mu
    return R, t

# Sample VIO at GPS timestamps for alignment
def vio_at_gps(traj, gps_t_arr):
    ts = traj[:, 0]
    out_x = np.array([interp1(ts, traj[:,1], t) for t in gps_t_arr])
    out_y = np.array([interp1(ts, traj[:,2], t) for t in gps_t_arr])
    return np.stack([out_x, out_y], axis=1)

alignments = {}
for name, traj in trajs.items():
    vio_xy = vio_at_gps(traj, gps_t)
    gps_xy = np.stack([gps_E, gps_N], axis=1)
    R, tv  = align_vio_to_gps(vio_xy, gps_xy)
    alignments[name] = (R, tv)

# ── 5. Per-method metrics ─────────────────────────────────────────────────────

print()
for name in ('A_global_oc', 'B_schmidt'):
    traj  = trajs[name]
    bias  = biases[name]
    R, tv = alignments[name]
    traj_t = traj[:,0]

    # Aligned VIO at GPS timestamps
    vio_xy   = vio_at_gps(traj, gps_t)
    vio_xy_a = (R @ vio_xy.T).T + tv       # aligned to GPS frame

    # XY ATE (over full GPS window)
    gps_xy = np.stack([gps_E, gps_N], axis=1)
    err_xy = np.sqrt(((vio_xy_a - gps_xy)**2).sum(1))
    ate_rms   = rms(err_xy)
    ate_max   = float(err_xy.max())
    ate_final = float(err_xy[-1])

    # VIO yaw from quaternion (in aligned frame)
    yaw_off = math.atan2(R[1,0], R[0,0])   # frame rotation angle
    vio_yaw_raw  = np.array([quat_to_yaw(*traj[i,4:8]) for i in range(len(traj))])
    vio_yaw_aln  = vio_yaw_raw + yaw_off    # VIO yaw in GPS/ENU frame

    # GPS course yaw at gated timestamps
    gps_c_t = np.array([gps_course_t[i] for i,m in enumerate(gated_mask) if m])
    gps_c_v = np.array([gps_course[i]   for i,m in enumerate(gated_mask) if m])

    # VIO course yaw at gated timestamps (from aligned position differences)
    vio_course_at_gps = []
    for tc in gps_c_t:
        dt_half = 0.6
        x0,y0 = interp_xy(traj_t, traj[:,1], traj[:,2], tc - dt_half)
        x1,y1 = interp_xy(traj_t, traj[:,1], traj[:,2], tc + dt_half)
        dx,dy = x1-x0, y1-y0
        if math.sqrt(dx**2+dy**2) < 0.01:
            vio_course_at_gps.append(float('nan'))
            continue
        # rotate displacement to GPS frame
        rxy = R @ np.array([dx, dy])
        vio_course_at_gps.append(math.degrees(math.atan2(rxy[0], rxy[1])))

    # VIO-GPS course yaw error (speed-gated)
    crs_err = [wrap180(vc - gc) for vc, gc in zip(vio_course_at_gps, gps_c_v)
               if math.isfinite(vc)]
    vio_gps_yaw_rms = rms(crs_err)
    vio_gps_yaw_max = max(abs(e) for e in crs_err) if crs_err else float('nan')

    # Gyro-integrated yaw: integrate IMU gyro Z in body frame
    # Use bias from traj.bias at each IMU step
    bias_t  = bias[:,0]
    bias_bgz= bias[:,6]

    yaw_raw_int  = math.degrees(quat_to_yaw(*traj[0,4:8]))   # start from VIO initial yaw
    yaw_bias_int = yaw_raw_int
    gyro_raw_t,  gyro_raw_yaw  = [traj[0,0]], [yaw_raw_int]
    gyro_bias_t, gyro_bias_yaw = [traj[0,0]], [yaw_bias_int]

    for i in range(1, len(imu_t)):
        dt = float(imu_t[i] - imu_t[i-1])
        if dt <= 0 or dt > 0.1: continue
        wz_raw  = float(imu_wz[i])
        bg_z    = float(interp1(bias_t, bias_bgz, float(imu_t[i])))
        wz_corr = wz_raw - bg_z
        # Project gyro Z (body) to global yaw rate:
        # For small roll/pitch, dpsi/dt ≈ wz / cos(pitch).
        # Approximation: dpsi_global ≈ wz_body (for near-level flight)
        yaw_raw_int  += math.degrees(wz_raw  * dt)
        yaw_bias_int += math.degrees(wz_corr * dt)
        gyro_raw_t.append(float(imu_t[i]))
        gyro_raw_yaw.append(yaw_raw_int)
        gyro_bias_t.append(float(imu_t[i]))
        gyro_bias_yaw.append(yaw_bias_int)

    gyro_raw_t  = np.array(gyro_raw_t)
    gyro_raw_yaw= np.array(gyro_raw_yaw)
    gyro_bias_t = np.array(gyro_bias_t)
    gyro_bias_yaw=np.array(gyro_bias_yaw)

    # rawGyro-GPS course yaw RMS (speed-gated)
    raw_course_at_gps = [math.degrees(interp1(gyro_raw_t, np.deg2rad(gyro_raw_yaw), tc))
                         for tc in gps_c_t]
    raw_gps_err = [wrap180(rc - gc) for rc, gc in zip(raw_course_at_gps, gps_c_v)
                   if math.isfinite(rc)]
    raw_gps_yaw_rms = rms(raw_gps_err)

    # biasGyro-GPS course yaw RMS (speed-gated)
    bias_course_at_gps = [math.degrees(interp1(gyro_bias_t, np.deg2rad(gyro_bias_yaw), tc))
                          for tc in gps_c_t]
    bias_gps_err = [wrap180(bc - gc) for bc, gc in zip(bias_course_at_gps, gps_c_v)
                    if math.isfinite(bc)]
    bias_gps_yaw_rms = rms(bias_gps_err)

    # VIO - biasGyro yaw RMS
    vio_at_bias_t = np.array([math.degrees(interp1(traj_t, vio_yaw_aln, t))
                               for t in gyro_bias_t])
    vio_bias_err = [wrap180(v - b) for v, b in zip(vio_at_bias_t, gyro_bias_yaw)]
    vio_bias_yaw_rms = rms(vio_bias_err)

    # bg_z stats
    bg_z_vals = bias[:,6]
    bg_z_steps = np.abs(np.diff(bg_z_vals))

    print(f'══════════  {name}  ══════════')
    print(f'  XY ATE RMS       = {ate_rms:.2f} m')
    print(f'  XY ATE max       = {ate_max:.2f} m')
    print(f'  XY ATE final     = {ate_final:.2f} m')
    print(f'  VIO-GPS yaw RMS  = {vio_gps_yaw_rms:.2f} deg  (n={len(crs_err)}, speed>{SPEED_GATE}m/s)')
    print(f'  VIO-GPS yaw max  = {vio_gps_yaw_max:.2f} deg')
    print(f'  rawGyro-GPS yaw  = {raw_gps_yaw_rms:.2f} deg')
    print(f'  biasGyro-GPS yaw = {bias_gps_yaw_rms:.2f} deg')
    print(f'  VIO-biasGyro RMS = {vio_bias_yaw_rms:.2f} deg')
    print(f'  bg_z range       = [{bg_z_vals.min():.6f}, {bg_z_vals.max():.6f}] rad/s')
    print(f'  bg_z final       = {bg_z_vals[-1]:.6f} rad/s')
    print(f'  bg_z max step    = {bg_z_steps.max():.6f} rad/s')
    print()
