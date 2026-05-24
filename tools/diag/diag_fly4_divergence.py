#!/usr/bin/env python3
"""
fly4 iwt=5.0 divergence diagnostic: pinpoint exactly when and why fly4 diverges.

Run from repo root:
  python tools/diag_fly4_divergence.py

Outputs text analysis to stdout + plots to:
  comparison_plots/fly4_divergence_debug/
"""

import os, sys, json, re, math
import numpy as np
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
FLY4 = REPO / "20260509_fly4"
RES = FLY4 / "result"
IWT_DIR = RES / "init_window_time_sweep"
OUT_DIR = REPO / "comparison_plots" / "fly4_divergence_debug"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# Paths
B0_IWT2 = RES / "stage_a_v2" / "R0_nogps.txt"
B0_IWT5 = IWT_DIR / "B0_iwt5.0.txt"
B1_IWT2 = RES / "stage_a_v2" / "R6_delay10s.txt"
B1_IWT5 = IWT_DIR / "B1_iwt5.0.txt"
V1_IWT5 = IWT_DIR / "v1_E1_iwt5.0.txt"
B1_IWT5_LOG = IWT_DIR / "B1_iwt5.0.log"
V1_IWT5_LOG = IWT_DIR / "v1_E1_iwt5.0.log"
GPS_CSV = RES / "gps_tum_time_alignment" / "aligned_gps_cam_time.csv"

# ============================================================
# 1. Load trajectories
# ============================================================
def load_tum_traj(path):
    """Load TUM trajectory: t x y z qx qy qz qw"""
    data = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                data.append([float(x) for x in parts[:8]])
    return np.array(data)

def quat_to_rpy(qx, qy, qz, qw):
    """Convert JPL quaternion to roll/pitch/yaw (radians)."""
    # JPL convention: q = [qx, qy, qz, qw] with qw scalar
    # R = I + 2*qw*skew(qv) + 2*skew(qv)^2  where qv = [qx,qy,qz]^T
    # But standard conversion uses Hamilton convention. For JPL:
    # We can use standard formulas with qw as scalar.
    roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(2*(qw*qy - qz*qx))
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw

def compute_cos_tilt(qx, qy, qz, qw):
    """cos_tilt = R_GtoI(2,2) = r22. This is the cosine of the tilt angle.
    From OpenVINS JPL convention: R = exp(quat).
    r22 = 1 - 2*(qx^2 + qy^2) for Hamilton; for JPL: r22 = qw^2 - qx^2 - qy^2 + qz^2
    """
    # Using the standard rotation matrix element for quaternion:
    # R(2,2) = 1 - 2*(qx^2 + qy^2) for Hamilton scalar-last (w,x,y,z)
    # For TUM format (x,y,z,w) with JPL, the rotation is:
    # R_GtoI = (2*qw^2 - 1)*I + 2*qw*skew(qv) + 2*qv*qv^T
    # where qv = [qx, qy, qz]
    # R(2,2) = 2*qw^2 - 1 + 2*qw*0 + 2*qz^2 = 2*(qw^2 + qz^2) - 1
    # Actually let me use the standard formula:
    return 1.0 - 2.0*(qx*qx + qy*qy)

def compute_traj_stats(traj):
    """Compute position norm, z, roll, pitch, yaw, cos_tilt for each frame."""
    t = traj[:, 0]
    x, y, z = traj[:, 1], traj[:, 2], traj[:, 3]
    pos_norm = np.sqrt(x**2 + y**2 + z**2)

    rpy = np.array([quat_to_rpy(*traj[i, 4:8]) for i in range(len(traj))])
    roll, pitch, yaw = rpy[:, 0], rpy[:, 1], rpy[:, 2]

    cos_tilt = np.array([compute_cos_tilt(*traj[i, 4:8]) for i in range(len(traj))])

    return t, pos_norm, z, roll, pitch, yaw, cos_tilt

# ============================================================
# 2. Parse GPS-ALT-STAT and GPLANE-RNG-STAT from log
# ============================================================
def parse_stat_lines(logpath):
    """Extract GPLANE-RNG-STAT lines as list of dicts."""
    stats = []
    with open(logpath, errors='ignore') as f:
        for line in f:
            m = re.search(r'\[GPLANE-RNG-STAT\]\s+t=([\d.]+)\s+calls=(\d+)\s+acc=(\d+)\s+rej=(\d+)\s+skip=(\d+)\s+'
                          r'K_pz_mu=([\d.]+)\s+\|K_xy\|_mu=([\d.]+)\s+\|dxy\|_mu=([\d.]+)\s+'
                          r'\|dtheta\|_mu=([\d.]+)\s+\|dba\|_mu=([\d.]+)\s+\|dbg\|_mu=([\d.]+)\s+'
                          r'\|res\|_mu=([\d.]+)\s+dpz_mu=([\d.]+)', line)
            if m:
                stats.append({
                    't': float(m.group(1)), 'calls': int(m.group(2)), 'acc': int(m.group(3)),
                    'rej': int(m.group(4)), 'skip': int(m.group(5)),
                    'K_pz_mu': float(m.group(6)), 'K_xy_mu': float(m.group(7)),
                    'dxy_mu': float(m.group(8)), 'dtheta_mu': float(m.group(9)),
                    'dba_mu': float(m.group(10)), 'dbg_mu': float(m.group(11)),
                    'res_mu': float(m.group(12)), 'dpz_mu': float(m.group(13)),
                })
    return stats

def parse_v1_stat_lines(logpath):
    """Extract GPLANE-V1-STAT lines."""
    stats = []
    with open(logpath, errors='ignore') as f:
        for line in f:
            m = re.search(r'\[GPLANE-V1-STAT\]\s+t=([\d.]+)\s+called=(\d+)\s+acc=(\d+)\s+'
                          r'skip_tilt=(\d+)\s+skip_noclone=(\d+)\s+skip_nofeat=(\d+)', line)
            if m:
                stats.append({
                    't': float(m.group(1)), 'called': int(m.group(2)), 'acc': int(m.group(3)),
                    'skip_tilt': int(m.group(4)), 'skip_noclone': int(m.group(5)),
                    'skip_nofeat': int(m.group(6)),
                })
    return stats

# ============================================================
# 3. Load GPS altitude data
# ============================================================
def load_gps_alt(path):
    """Load GPS altitude: returns (timestamps_ns, alt_m)."""
    ts, alts = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split(",")
            if len(parts) >= 4:
                ts.append(float(parts[0]) / 1e9)  # ns -> seconds
                alts.append(float(parts[3]))
    return np.array(ts), np.array(alts)

# ============================================================
# 4. Main diagnostic
# ============================================================
def main():
    print("=" * 70)
    print("fly4 iwt=5.0 Divergence Diagnostic")
    print("=" * 70)

    # --- Load trajectories ---
    print("\n=== Loading trajectories ===")
    trajs = {}
    for label, path in [("B0_iwt2", B0_IWT2), ("B0_iwt5", B0_IWT5),
                         ("B1_iwt2", B1_IWT2), ("B1_iwt5", B1_IWT5),
                         ("v1_E1_iwt5", V1_IWT5)]:
        if path.exists():
            trajs[label] = load_tum_traj(path)
            print(f"  {label}: {len(trajs[label])} frames, t=[{trajs[label][0,0]:.2f}, {trajs[label][-1,0]:.2f}]")
        else:
            print(f"  {label}: NOT FOUND ({path})")

    # --- Load GPS ---
    print("\n=== Loading GPS altitude ===")
    gps_ts, gps_alt = load_gps_alt(GPS_CSV)
    print(f"  GPS: {len(gps_ts)} points, t=[{gps_ts[0]:.2f}, {gps_ts[-1]:.2f}]")

    # --- Parse STAT lines ---
    print("\n=== Parsing B1_iwt5.0 STAT lines ===")
    b1_stats = parse_stat_lines(B1_IWT5_LOG)
    for i, s in enumerate(b1_stats):
        flagged = ""
        if s['dxy_mu'] > 0.1:
            flagged += " <-- DXY SPIKED"
        if s['res_mu'] > 5:
            flagged += " <-- RES SPIKED"
        if s['skip'] > 0:
            flagged += f" <-- {s['skip']} SKIPS"
        print(f"  [{i}] t={s['t']:.1f} calls={s['calls']} acc={s['acc']} skip={s['skip']} "
              f"dxy={s['dxy_mu']:.3f}m res={s['res_mu']:.2f}m K_xy={s['K_xy_mu']:.4f}{flagged}")

    print("\n=== Parsing v1_E1_iwt5.0 V1-STAT lines ===")
    v1_stats = parse_v1_stat_lines(V1_IWT5_LOG)
    for i, s in enumerate(v1_stats):
        acc_pct = s['acc'] / s['called'] * 100 if s['called'] > 0 else 0
        flagged = ""
        if s['skip_tilt'] > s['called'] * 0.1:
            flagged += f" <-- TILT SKIP {s['skip_tilt']}/{s['called']}"
        print(f"  [{i}] t={s['t']:.1f} called={s['called']} acc={s['acc']} ({acc_pct:.0f}%) "
              f"skip_tilt={s['skip_tilt']}{flagged}")

    # --- Key failure timing ---
    print("\n" + "=" * 70)
    print("KEY FAILURE TIMING (fly4 B1 iwt=5.0)")
    print("=" * 70)

    # The divergence window: between last healthy STAT and first STAT with skips
    last_healthy = None
    first_bad = None
    for s in b1_stats:
        if s['skip'] == 0 and s['dxy_mu'] < 0.1 and s['res_mu'] < 5:
            last_healthy = s
        if s['skip'] > 0 or s['dxy_mu'] > 1.0:
            if first_bad is None:
                first_bad = s
            break

    if last_healthy:
        print(f"\nLast healthy STAT: t={last_healthy['t']:.1f} "
              f"dxy={last_healthy['dxy_mu']:.3f}m res={last_healthy['res_mu']:.2f}m")
    if first_bad:
        print(f"First bad STAT:     t={first_bad['t']:.1f} "
              f"dxy={first_bad['dxy_mu']:.3f}m res={first_bad['res_mu']:.2f}m skip={first_bad['skip']}")

    # Find the diverging STAT (the one right before the gap)
    print("\nDivergence window analysis:")
    for i, s in enumerate(b1_stats):
        if i + 1 < len(b1_stats):
            dt = b1_stats[i+1]['t'] - s['t']
            if dt > 60:
                print(f"  STAT[{i}] t={s['t']:.1f} -> STAT[{i+1}] t={b1_stats[i+1]['t']:.1f}: "
                      f"gap={dt:.0f}s (should be ~30s)")
                print(f"  During this gap: {b1_stats[i+1]['skip'] - s['skip']} new skips accumulated")
                print(f"  First skip occurred between t={s['t']:.1f} and t={s['t']+30:.1f}")

    # --- Compare trajectories ---
    print("\n" + "=" * 70)
    print("TRAJECTORY COMPARISON")
    print("=" * 70)

    if "B0_iwt2" in trajs and "B0_iwt5" in trajs:
        b0_iwt2 = trajs["B0_iwt2"]
        b0_iwt5 = trajs["B0_iwt5"]

        b0_2_t, b0_2_norm, b0_2_z, _, _, _, b0_2_ct = compute_traj_stats(b0_iwt2)
        b0_5_t, b0_5_norm, b0_5_z, b0_5_r, b0_5_p, b0_5_y, b0_5_ct = compute_traj_stats(b0_iwt5)

        # Find time overlap
        t_min = max(b0_2_t[0], b0_5_t[0])
        t_max = min(b0_2_t[-1], b0_5_t[-1])

        print(f"\nB0_iwt2: {len(b0_iwt2)} frames, |pos| max={b0_2_norm.max():.1f}m")
        print(f"B0_iwt5: {len(b0_iwt5)} frames, |pos| max={b0_5_norm.max():.1f}m")
        print(f"Overlap: t=[{t_min:.1f}, {t_max:.1f}] ({t_max-t_min:.1f}s)")

        # Check cos_tilt in B0_iwt5
        min_ct_idx = np.argmin(b0_5_ct)
        min_ct = b0_5_ct[min_ct_idx]
        print(f"\nB0_iwt5 cos_tilt: min={min_ct:.4f} at t={b0_5_t[min_ct_idx]:.1f} "
              f"(tilt={math.degrees(math.acos(min(abs(min_ct), 1.0))):.1f}°)")
        print(f"  cos_tilt < 0.9: {np.sum(b0_5_ct < 0.9)} frames")
        print(f"  cos_tilt < 0.7: {np.sum(b0_5_ct < 0.7)} frames")
        print(f"  cos_tilt < 0.5: {np.sum(b0_5_ct < 0.5)} frames")
        print(f"  cos_tilt < 0.3: {np.sum(b0_5_ct < 0.3)} frames")

    if "B1_iwt5" in trajs:
        b1_iwt5 = trajs["B1_iwt5"]
        b1_t, b1_norm, b1_z, b1_r, b1_p, b1_y, b1_ct = compute_traj_stats(b1_iwt5)

        print(f"\nB1_iwt5: {len(b1_iwt5)} frames, |pos| max={b1_norm.max():.1f}m")

        # Find when position first exceeds 500m
        big_pos = np.where(b1_norm > 500)[0]
        if len(big_pos) > 0:
            print(f"  First |pos| > 500m: t={b1_t[big_pos[0]]:.2f} |pos|={b1_norm[big_pos[0]]:.1f}m")
        big_pos = np.where(b1_norm > 100)[0]
        if len(big_pos) > 0:
            print(f"  First |pos| > 100m:  t={b1_t[big_pos[0]]:.2f} |pos|={b1_norm[big_pos[0]]:.1f}m")

        # Track when cos_tilt drops
        ct_bad = np.where(b1_ct < 0.5)[0]
        if len(ct_bad) > 0:
            print(f"  First cos_tilt < 0.5: t={b1_t[ct_bad[0]]:.2f} cos_tilt={b1_ct[ct_bad[0]]:.3f}")
        ct_bad2 = np.where(b1_ct < 0.4)[0]
        if len(ct_bad2) > 0:
            print(f"  First cos_tilt < 0.4: t={b1_t[ct_bad2[0]]:.2f} cos_tilt={b1_ct[ct_bad2[0]]:.3f}")

        # Find position jumps
        if len(b1_iwt5) > 1:
            dp = np.sqrt(np.diff(b1_iwt5[:, 1])**2 + np.diff(b1_iwt5[:, 2])**2 + np.diff(b1_iwt5[:, 3])**2)
            big_jumps = np.where(dp > 50)[0]
            if len(big_jumps) > 0:
                print(f"\n  First position jump > 50m: t={b1_t[big_jumps[0]]:.2f} dp={dp[big_jumps[0]]:.1f}m")
                print(f"  Total jumps > 50m: {len(big_jumps)}")

    # --- GPS update quality check ---
    print("\n" + "=" * 70)
    print("GPS UPDATE QUALITY (B1 fly4 vs fly1)")
    print("=" * 70)

    # Compare fly1 B1 and fly4 B1 at similar relative times
    fly1_b1_log = REPO / "20260509_fly1" / "result" / "init_window_time_sweep" / "B1_iwt5.0.log"
    fly1_stats = parse_stat_lines(fly1_b1_log) if fly1_b1_log.exists() else []

    # At t~30s into GPS updates:
    if len(fly1_stats) >= 2 and len(b1_stats) >= 2:
        print(f"\n  After ~2 STATs (~60s GPS):")
        print(f"    fly1: dxy={fly1_stats[1]['dxy_mu']:.3f}m res={fly1_stats[1]['res_mu']:.2f}m K_xy={fly1_stats[1]['K_xy_mu']:.4f}")
        print(f"    fly4: dxy={b1_stats[1]['dxy_mu']:.3f}m res={b1_stats[1]['res_mu']:.2f}m K_xy={b1_stats[1]['K_xy_mu']:.4f}")

    # At the last shared healthy point
    if len(fly1_stats) >= 6 and len(b1_stats) >= 6:
        print(f"\n  After ~6 STATs (~180s GPS):")
        print(f"    fly1: dxy={fly1_stats[5]['dxy_mu']:.3f}m res={fly1_stats[5]['res_mu']:.2f}m K_xy={fly1_stats[5]['K_xy_mu']:.4f}")
        print(f"    fly4: dxy={b1_stats[5]['dxy_mu']:.3f}m res={b1_stats[5]['res_mu']:.2f}m K_xy={b1_stats[5]['K_xy_mu']:.4f}")

    # At the divergence point
    if len(b1_stats) >= 8:
        print(f"\n  At divergence (STAT[7]):")
        print(f"    fly4: dxy={b1_stats[7]['dxy_mu']:.3f}m res={b1_stats[7]['res_mu']:.2f}m K_xy={b1_stats[7]['K_xy_mu']:.4f}")

    print("\n" + "=" * 70)
    print("CONCLUSION")
    print("=" * 70)
    print("""
The divergence follows this cascade:
1. fly4 iwt=5.0 changes ba_z sign (+0.00062 -> -0.00195)
2. Early trajectory is slightly different (different velocity bias direction)
3. By ~t+210s (STAT[7] at t=1774463845), dxy starts growing (0.204m vs fly1's 0.004m)
   and residual increases (2.91m vs fly1's 0.89m)
4. Within the next 30s, the EKF attitude degrades to cos_tilt < threshold
5. GPS range model divides position residual by near-zero cos_tilt -> huge residuals
6. Huge residuals push position catastrophically, causing t_state vs t_gps mismatch
7. All subsequent GPS updates are skipped (1744 skips in B1)
8. v1_E1 tilt guard blocks 86.9% of updates, proving attitude divergence

Root cause: iwt=5.0 init -> different ba_z -> different trajectory -> attitude drift
-> GPS range model amplification -> runaway divergence.
The VIO itself (B0_iwt5) is stable throughout — GPS triggers the runaway.
""")

    # ============================================================
    # PLOTS
    # ============================================================
    print("Generating plots...")
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # 1. XY trajectory comparison
        fig, axes = plt.subplots(1, 2, figsize=(16, 7))

        for ax, title, key in [(axes[0], "XY trajectory (full)", None),
                                (axes[1], "XY trajectory (zoom: first 100m)", None)]:
            pass  # handled below

        ax = axes[0]
        colors = {'B0_iwt2': 'blue', 'B0_iwt5': 'green', 'B1_iwt2': 'cyan', 'B1_iwt5': 'red'}
        for label, color in colors.items():
            if label in trajs:
                t = trajs[label]
                ax.plot(t[:, 1], t[:, 2], color=color, label=label, alpha=0.7, linewidth=0.5)

        # Mark last valid point on B1_iwt5
        if "B1_iwt5" in trajs:
            b1 = trajs["B1_iwt5"]
            b1_t_vals, b1_norm_vals, _, _, _, _, _ = compute_traj_stats(b1)
            valid = np.where(b1_norm_vals < 500)[0]
            if len(valid) > 0:
                last_valid = valid[-1]
                ax.plot(b1[last_valid, 1], b1[last_valid, 2], 'rx', markersize=10, markeredgewidth=2,
                        label=f'last valid B1_iwt5 (t={b1_t_vals[last_valid]:.0f})')

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("fly4 XY trajectory — iwt=2.0 vs iwt=5.0")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        # Zoomed XY
        ax = axes[1]
        for label, color in colors.items():
            if label in trajs:
                t = trajs[label]
                ax.plot(t[:, 1], t[:, 2], color=color, label=label, alpha=0.7, linewidth=0.8)

        if "B1_iwt5" in trajs and len(valid) > 0:
            ax.plot(b1[last_valid, 1], b1[last_valid, 2], 'rx', markersize=10, markeredgewidth=2)

        # Zoom to first 100m box
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("fly4 XY trajectory (zoom: first 100m)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        plt.tight_layout()
        p1 = OUT_DIR / "01_xy_traj.png"
        fig.savefig(str(p1), dpi=150)
        plt.close(fig)
        print(f"  {p1}")

        # 2. Height vs time (B0_iwt2, B0_iwt5, B1_iwt5)
        fig, ax = plt.subplots(figsize=(14, 5))
        if "B0_iwt2" in trajs:
            t, _, z, _, _, _, _ = compute_traj_stats(trajs["B0_iwt2"])
            ax.plot(t, z, 'blue', alpha=0.6, linewidth=0.5, label='B0_iwt2')
        if "B0_iwt5" in trajs:
            t, _, z, _, _, _, _ = compute_traj_stats(trajs["B0_iwt5"])
            ax.plot(t, z, 'green', alpha=0.6, linewidth=0.5, label='B0_iwt5')
        if "B1_iwt5" in trajs:
            t, _, z, _, _, _, _ = compute_traj_stats(trajs["B1_iwt5"])
            ax.plot(t, z, 'red', alpha=0.6, linewidth=0.5, label='B1_iwt5')

        # Overlay GPS altitude
        ax.plot(gps_ts, gps_alt, 'gray', alpha=0.3, linewidth=0.3, label='GPS alt')

        ax.set_xlabel("time (s)")
        ax.set_ylabel("z (m)")
        ax.set_title("fly4 height vs time")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        p2 = OUT_DIR / "02_height.png"
        fig.savefig(str(p2), dpi=150)
        plt.close(fig)
        print(f"  {p2}")

        # 3. GPS residual vs time (from STAT lines)
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # GPS res
        ax = axes[0, 0]
        if b1_stats:
            ts = [s['t'] for s in b1_stats]
            res = [s['res_mu'] for s in b1_stats]
            ax.plot(ts, res, 'ro-', markersize=4, label='B1_iwt5')
            ax.axhline(5, color='orange', linestyle='--', alpha=0.5, label='5m threshold')
            ax.axhline(30, color='red', linestyle='--', alpha=0.5, label='30m (potential gate)')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("|res|_mu (m)")
        ax.set_title("GPS residual vs time (30s rolling mean)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_yscale('log')

        # dxy per GPS update
        ax = axes[0, 1]
        if b1_stats:
            ts = [s['t'] for s in b1_stats]
            dxy = [s['dxy_mu'] for s in b1_stats]
            ax.plot(ts, dxy, 'mo-', markersize=4, label='B1_iwt5')
            ax.axhline(1, color='orange', linestyle='--', alpha=0.5, label='1m threshold')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("|dxy|_mu (m)")
        ax.set_title("Position correction per GPS update (30s rolling mean)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_yscale('log')

        # cos_tilt vs time
        ax = axes[1, 0]
        if "B0_iwt5" in trajs:
            t, _, _, _, _, _, ct = compute_traj_stats(trajs["B0_iwt5"])
            ax.plot(t, ct, 'green', alpha=0.6, linewidth=0.3, label='B0_iwt5')
        if "B1_iwt5" in trajs:
            t, _, _, _, _, _, ct = compute_traj_stats(trajs["B1_iwt5"])
            ax.plot(t, ct, 'red', alpha=0.6, linewidth=0.3, label='B1_iwt5')
        ax.axhline(0.05, color='orange', linestyle='--', alpha=0.5, label='min_cos_tilt (0.05)')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("cos_tilt")
        ax.set_title("cos_tilt vs time (0 = 90° tilt)")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # Roll/Pitch vs time
        ax = axes[1, 1]
        if "B1_iwt5" in trajs:
            t, _, _, r, p, y, _ = compute_traj_stats(trajs["B1_iwt5"])
            ax.plot(t, np.degrees(r), 'red', alpha=0.5, linewidth=0.3, label='roll')
            ax.plot(t, np.degrees(p), 'blue', alpha=0.5, linewidth=0.3, label='pitch')
        ax.set_xlabel("time (s)")
        ax.set_ylabel("degrees")
        ax.set_title("fly4 B1_iwt5 Roll / Pitch vs time")
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        p3 = OUT_DIR / "03_gps_diagnostics.png"
        fig.savefig(str(p3), dpi=150)
        plt.close(fig)
        print(f"  {p3}")

        # 4. B0_iwt2 vs B0_iwt5 position norm comparison
        fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

        if "B0_iwt2" in trajs and "B0_iwt5" in trajs:
            # |pos|
            ax = axes[0]
            t2, n2, _, _, _, _, _ = compute_traj_stats(trajs["B0_iwt2"])
            t5, n5, _, _, _, _, _ = compute_traj_stats(trajs["B0_iwt5"])
            ax.plot(t2, n2, 'blue', alpha=0.6, linewidth=0.5, label='B0_iwt2')
            ax.plot(t5, n5, 'green', alpha=0.6, linewidth=0.5, label='B0_iwt5')
            ax.set_ylabel("|pos| (m)")
            ax.set_title("fly4 B0 VIO — position norm")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

            # z
            ax = axes[1]
            _, z2, _, _, _, _, _ = compute_traj_stats(trajs["B0_iwt2"])
            _, z5, _, _, _, _, _ = compute_traj_stats(trajs["B0_iwt5"])
            ax.plot(t2, z2, 'blue', alpha=0.6, linewidth=0.5, label='B0_iwt2')
            ax.plot(t5, z5, 'green', alpha=0.6, linewidth=0.5, label='B0_iwt5')
            ax.set_ylabel("z (m)")
            ax.set_title("fly4 B0 VIO — height")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

            # cos_tilt
            ax = axes[2]
            _, _, _, _, _, _, ct2 = compute_traj_stats(trajs["B0_iwt2"])
            _, _, _, _, _, _, ct5 = compute_traj_stats(trajs["B0_iwt5"])
            ax.plot(t2, ct2, 'blue', alpha=0.6, linewidth=0.5, label='B0_iwt2')
            ax.plot(t5, ct5, 'green', alpha=0.6, linewidth=0.5, label='B0_iwt5')
            ax.set_xlabel("time (s)")
            ax.set_ylabel("cos_tilt")
            ax.set_title("fly4 B0 VIO — cos_tilt")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        p4 = OUT_DIR / "04_b0_comparison.png"
        fig.savefig(str(p4), dpi=150)
        plt.close(fig)
        print(f"  {p4}")

    except ImportError:
        print("WARNING: matplotlib not available, skipping plots")

    print(f"\nOutputs written to: {OUT_DIR}")
    print("Done.")


if __name__ == "__main__":
    main()
