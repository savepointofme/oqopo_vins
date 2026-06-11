#!/usr/bin/env python3
"""Side-by-side fly4 trajectory comparison: A (clones=11) vs J (clones=15)."""

import csv, math, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── paths ─────────────────────────────────────────────────────────────────────
BASE_A = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/sigma1_stab_20260610/A_fly4"
BASE_J = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/sigma1_stab_20260610/J_fly4"
GPS_CSV = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
OUT_DIR = "/mnt/c/Users/baloney/Desktop/sigma1_analysis_20260610/fly4_comparison"
os.makedirs(OUT_DIR, exist_ok=True)

T0, T1 = 924.4, 2815.99
YAW_WIN, SPEED_GATE = 5.0, 2.0
RAD = math.pi / 180.0

# ── helpers ───────────────────────────────────────────────────────────────────
def load_traj(path):
    data = []
    with open(path) as f:
        for line in f:
            if line.startswith('#'): continue
            p = line.strip().split()
            if len(p) < 8: continue
            data.append([float(x) for x in p[:8]])
    return np.array(data)

def load_gps(path):
    rows = []
    with open(path) as f:
        r = csv.reader(f)
        header = next(r)
        for row in r:
            if len(row) < 4: continue
            try:
                t = float(row[0]) / 1e9
                lat = float(row[1])
                lon = float(row[2])
                alt = float(row[3])
                rows.append((t, lat, lon, alt))
            except: pass
    return rows

def wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0):
    dN = (lat - lat0) * 111320.0
    dE = (lon - lon0) * 111320.0 * math.cos(lat0 * RAD)
    return dE, dN, alt - alt0

def quat_to_yaw(qx, qy, qz, qw):
    # JPL quaternion
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

def interp1(x, y, xi):
    if xi <= x[0]: return y[0]
    if xi >= x[-1]: return y[-1]
    i = np.searchsorted(x, xi)
    if i == 0: return y[0]
    t = (xi - x[i-1]) / (x[i] - x[i-1])
    return y[i-1] + t*(y[i] - y[i-1])

def gps_course_mean(gt, gv, gspd, t0, t1, gate):
    m = (gspd >= gate) & (gt >= t0) & (gt <= t1)
    if m.sum() > 0:
        s, c = np.sum(np.sin(gv[m])), np.sum(np.cos(gv[m]))
        return math.atan2(s, c), int(m.sum())
    return None, 0

def align_start_yaw(traj, bias, gt, gv, gspd, gps_E, gps_N):
    ts = traj[:,0]
    bias_ts = bias[:,0]
    vio_yaw_arr = np.array([quat_to_yaw(*traj[i,4:8]) for i in range(len(traj))])
    p_vio_start = np.array([interp1(ts, traj[:,1], T0), interp1(ts, traj[:,2], T0)])

    yaw_gps, n_s = gps_course_mean(gt, gv, gspd, T0, T0+YAW_WIN, SPEED_GATE)
    if yaw_gps is None:
        # fallback: use first two GPS points
        yaw_gps = math.atan2(gps_E[1]-gps_E[0], gps_N[1]-gps_N[0])

    vx = interp1(bias_ts, bias[:,1], T0)
    vy = interp1(bias_ts, bias[:,2], T0)
    if math.sqrt(vx**2 + vy**2) > 1.0:
        yaw_vio = math.atan2(vx, vy)
    else:
        yaw_vio = float(interp1(ts, vio_yaw_arr, T0))

    delta_yaw = yaw_gps - yaw_vio
    p_gps_start = np.array([interp1(gt, gps_E, T0), interp1(gt, gps_N, T0)])

    cos_d, sin_d = math.cos(delta_yaw), math.sin(delta_yaw)
    R = np.array([[cos_d, -sin_d], [sin_d, cos_d]])

    p_aln = np.zeros((len(traj), 2))
    for i in range(len(traj)):
        p_local = traj[i,1:3] - p_vio_start
        p_aln[i] = R @ p_local + p_gps_start

    vio_yaw_aln = vio_yaw_arr + delta_yaw
    return p_aln, vio_yaw_aln, delta_yaw, p_vio_start, p_gps_start, vio_yaw_arr

# ── load GPS ──────────────────────────────────────────────────────────────────
print("Loading GPS...")
gps_rows = load_gps(GPS_CSV)
# Reference from first GPS sample
lat0, lon0, alt0 = gps_rows[0][1], gps_rows[0][2], gps_rows[0][3]
gt, gps_E, gps_N, gps_alt = [], [], [], []
for t, lat, lon, alt in gps_rows:
    e, n, u = wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0)
    gt.append(t); gps_E.append(e); gps_N.append(n); gps_alt.append(u)
gt = np.array(gt); gps_E = np.array(gps_E); gps_N = np.array(gps_N); gps_alt = np.array(gps_alt)

# GPS course bearings
gv = np.arctan2(np.diff(gps_E, prepend=gps_E[0]), np.diff(gps_N, prepend=gps_N[0]))
dt_gps = np.diff(gt, prepend=gt[0]+0.001)
gspd = np.sqrt(np.diff(gps_E, prepend=gps_E[0])**2 + np.diff(gps_N, prepend=gps_N[0])**2) / dt_gps

# ── load trajectories ─────────────────────────────────────────────────────────
print("Loading trajectories...")
traj_A = load_traj(f"{BASE_A}/traj.txt")
traj_J = load_traj(f"{BASE_J}/traj.txt")
bias_A = np.loadtxt(f"{BASE_A}/traj.txt.bias", skiprows=1)
bias_J = np.loadtxt(f"{BASE_J}/traj.txt.bias", skiprows=1)

print("Aligning A...")
p_aln_A, vya_A, dy_A, pvs_A, pgs_A, vraw_A = align_start_yaw(traj_A, bias_A, gt, gv, gspd, gps_E, gps_N)
print("Aligning J...")
p_aln_J, vya_J, dy_J, pvs_J, pgs_J, vraw_J = align_start_yaw(traj_J, bias_J, gt, gv, gspd, gps_E, gps_N)

# ── common time grid ──────────────────────────────────────────────────────────
ts_common = np.linspace(max(T0, traj_A[0,0], traj_J[0,0]),
                         min(T1, traj_A[-1,0], traj_J[-1,0]), 5000)

def sample_traj(traj, p_aln, vya, ts):
    px = np.array([interp1(traj[:,0], p_aln[:,0], t) for t in ts])
    py = np.array([interp1(traj[:,0], p_aln[:,1], t) for t in ts])
    pyaw = np.array([interp1(traj[:,0], vya, t) for t in ts])
    return px, py, pyaw

def sample_gps(ts):
    ex = np.array([interp1(gt, gps_E, t) for t in ts])
    ny = np.array([interp1(gt, gps_N, t) for t in ts])
    gz = np.array([interp1(gt, gv, t) for t in ts])
    alt = np.array([interp1(gt, gps_alt, t) for t in ts])
    return ex, ny, gz, alt

px_A, py_A, pyaw_A = sample_traj(traj_A, p_aln_A, vya_A, ts_common)
px_J, py_J, pyaw_J = sample_traj(traj_J, p_aln_J, vya_J, ts_common)
ex, ny, gc, galt = sample_gps(ts_common)

err_yaw_A = np.degrees(np.arctan2(np.sin(pyaw_A - gc), np.cos(pyaw_A - gc)))
err_yaw_J = np.degrees(np.arctan2(np.sin(pyaw_J - gc), np.cos(pyaw_J - gc)))
err_xy_A = np.hypot(px_A - ex, py_A - ny)
err_xy_J = np.hypot(px_J - ex, py_J - ny)

# ── plot 1: top-down XY ───────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 10))
ax.plot(gps_E, gps_N, 'k-', lw=1.5, label='GPS reference')
ax.plot(p_aln_A[:,0], p_aln_A[:,1], 'b-', lw=1.2, alpha=0.7, label='A: clones=11')
ax.plot(p_aln_J[:,0], p_aln_J[:,1], 'r-', lw=1.2, alpha=0.7, label='J: clones=15')
for p_aln, color, name in [(p_aln_A, 'blue', 'A'), (p_aln_J, 'red', 'J')]:
    ax.scatter(p_aln[0,0], p_aln[0,1], c=color, s=60, marker='o', zorder=5)
    ax.scatter(p_aln[-1,0], p_aln[-1,1], c=color, s=60, marker='x', zorder=5)
ax.set_aspect('equal')
ax.set_xlabel('East [m]')
ax.set_ylabel('North [m]')
ax.set_title('fly4: XY trajectory (start_yaw aligned)')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(f"{OUT_DIR}/01_xy_topdown.png", dpi=150)
plt.close()
print("Saved 01_xy_topdown.png")

# ── plot 2: XY error vs time ──────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 5))
ax.plot(ts_common, err_xy_A, 'b-', lw=1.0, alpha=0.7, label='A: clones=11')
ax.plot(ts_common, err_xy_J, 'r-', lw=1.0, alpha=0.7, label='J: clones=15')
ratio = err_xy_J / np.maximum(err_xy_A, 1.0)
over_idx = np.where(ratio > 1.5)[0]
if len(over_idx):
    ax.axvline(ts_common[over_idx[0]], color='orange', ls='--', lw=1.5,
               label=f'J > 1.5× A  @ t={ts_common[over_idx[0]]:.0f}s')
ax.set_xlabel('Time [s]')
ax.set_ylabel('XY error [m]')
ax.set_title('fly4: XY error vs time')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(f"{OUT_DIR}/02_xy_error_vs_time.png", dpi=150)
plt.close()
print("Saved 02_xy_error_vs_time.png")

# ── plot 3: yaw error vs time ─────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(14, 5))
ax.plot(ts_common, err_yaw_A, 'b-', lw=1.0, alpha=0.7, label='A: clones=11')
ax.plot(ts_common, err_yaw_J, 'r-', lw=1.0, alpha=0.7, label='J: clones=15')
dyaw_diff = np.abs(err_yaw_J - err_yaw_A)
big_idx = np.where(dyaw_diff > 10)[0]
if len(big_idx):
    ax.axvline(ts_common[big_idx[0]], color='purple', ls='--', lw=1.5,
               label=f'|Δyaw| > 10°  @ t={ts_common[big_idx[0]]:.0f}s')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Yaw error [deg]')
ax.set_title('fly4: Yaw error vs time')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(f"{OUT_DIR}/03_yaw_error_vs_time.png", dpi=150)
plt.close()
print("Saved 03_yaw_error_vs_time.png")

# ── plot 4: time-colored segments ─────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(16, 8))

def plot_segments(ax, traj, p_aln, title):
    t = traj[:,0]
    tmin, tmax = t.min(), t.max()
    thirds = np.linspace(tmin, tmax, 4)
    colors = ['green', 'blue', 'red']
    labels = ['early', 'middle', 'late']
    for i in range(3):
        m = (t >= thirds[i]) & (t <= thirds[i+1])
        ax.plot(p_aln[m,0], p_aln[m,1], color=colors[i], lw=1.5, alpha=0.8, label=labels[i])
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.set_xlabel('East [m]')
    ax.set_ylabel('North [m]')
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

plot_segments(axes[0], traj_A, p_aln_A, 'A: clones=11 (time segments)')
plot_segments(axes[1], traj_J, p_aln_J, 'J: clones=15 (time segments)')
plt.tight_layout()
plt.savefig(f"{OUT_DIR}/04_time_segments.png", dpi=150)
plt.close()
print("Saved 04_time_segments.png")

# ── summary ───────────────────────────────────────────────────────────────────
print("\n=== fly4 A vs J summary ===")
mask = (ts_common >= T0) & (ts_common <= T1)
print(f"XY ATE  A={np.mean(err_xy_A[mask]):.1f}m  J={np.mean(err_xy_J[mask]):.1f}m")
print(f"Yaw RMS A={np.sqrt(np.mean(err_yaw_A[mask]**2)):.1f}°  J={np.sqrt(np.mean(err_yaw_J[mask]**2)):.1f}°")
if len(over_idx):
    print(f"J exceeds 1.5× A XY error first at t={ts_common[over_idx[0]]:.1f}s")
if len(big_idx):
    print(f"Yaw difference >10° first at t={ts_common[big_idx[0]]:.1f}s")
print(f"Plots saved to: {OUT_DIR}")
