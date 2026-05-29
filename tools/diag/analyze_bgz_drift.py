#!/usr/bin/env python3
"""Analyse bg_z drift and gauge pressure in B_current vs A.

Bias file format: # t_cam vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z
                    0    1  2  3   4    5    6    7    8    9   (0-indexed)
"""
import csv, math, numpy as np

DIAG_B  = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/fly4_schmidt_B_offsetm202p2_start924p4_until2816/schmidt_yaw_update_diag.csv"
A_BIAS  = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/traj.txt.bias"
B_BIAS  = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/fly4_schmidt_B_offsetm202p2_start924p4_until2816/traj.txt.bias"

# Load B_current Schmidt diag
print("Loading B_current Schmidt diag...")
diag = []
with open(DIAG_B) as f:
    for r in csv.DictReader(f):
        try:
            diag.append({
                't':   float(r['timestamp']),
                'bgz': float(r['bg_z_before']),
                'Pss': float(r['Pss_norm_before']),
                'ndx': abs(float(r['normal_dx_s_coeff_before'])),
                'Pas': float(r['Pas_change_norm']),
            })
        except (ValueError, KeyError):
            pass
diag.sort(key=lambda x: x['t'])
print(f"  {len(diag)} updates  t=[{diag[0]['t']:.1f}, {diag[-1]['t']:.1f}]")
print(f"  first bg_z_before: {diag[0]['bgz']:.6e}  last: {diag[-1]['bgz']:.6e}")

# Load bias files — bg_z at column 6 (0-indexed); skip comment lines
def load_bias_bgz(path):
    rows = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) >= 7:
                    try:
                        rows.append((float(parts[0]), float(parts[6])))  # t, bg_z
                    except ValueError:
                        pass
    except FileNotFoundError:
        print(f"  MISSING: {path}")
    return sorted(rows, key=lambda x: x[0])

print("Loading bias files (bg_z col 6)...")
a_bias = load_bias_bgz(A_BIAS)
b_bias = load_bias_bgz(B_BIAS)
print(f"  A: {len(a_bias)} rows  [{a_bias[0][0]:.1f}, {a_bias[-1][0]:.1f}]  bg_z first={a_bias[0][1]:.4e}")
print(f"  B: {len(b_bias)} rows  [{b_bias[0][0]:.1f}, {b_bias[-1][0]:.1f}]  bg_z first={b_bias[0][1]:.4e}")

# ── Table 1: B bg_z + Pss + gauge pressure (100s bins) ────────────────────
print()
print("=" * 100)
print("  B_current: bg_z (diag), Pss, gauge pressure — 100s bins")
print("=" * 100)
print(f"  {'Bin':11s}  {'bgz×1e4':>9s}  {'Δbgz×1e5':>10s}  {'Pss×1e-6':>9s}  "
      f"{'ndx_mean':>9s}  {'ndx_P95':>8s}  {'ndx_max':>8s}  {'Pas_mean':>9s}  {'n_ndx>1':>7s}")
print(f"  {'-'*97}")
prev_bgz = None
for t0 in np.arange(924, 2820, 100):
    t1 = t0 + 100
    rows = [d for d in diag if t0 <= d['t'] < t1]
    if not rows:
        continue
    bgz  = float(np.mean([r['bgz'] for r in rows]))
    Pss  = float(np.mean([r['Pss'] for r in rows]))
    ndxs = np.array([r['ndx'] for r in rows])
    Pas  = float(np.mean([r['Pas'] for r in rows]))
    n1   = int(np.sum(ndxs > 1.0))
    d_bgz = (bgz - prev_bgz) * 1e5 if prev_bgz is not None else float('nan')
    d_s = f"{d_bgz:10.3f}" if math.isfinite(d_bgz) else " " * 10
    print(f"  {t0:.0f}-{t1:.0f}    {bgz*1e4:9.4f}  {d_s}  {Pss/1e6:9.3f}  "
          f"{float(np.mean(ndxs)):9.4f}  {float(np.percentile(ndxs,95)):8.4f}  "
          f"{float(np.max(ndxs)):8.4f}  {Pas:9.4f}  {n1:7d}")
    prev_bgz = bgz

# ── Table 2: A vs B bg_z from bias files ──────────────────────────────────
print()
print("=" * 64)
print("  A vs B: bg_z from traj.txt.bias (col 6) — 100s bins")
print("=" * 64)
print(f"  {'Bin':11s}  {'A_bgz×1e4':>11s}  {'B_bgz×1e4':>11s}  {'B-A×1e4':>10s}")
print(f"  {'-'*50}")
for t0 in np.arange(924, 2820, 100):
    t1 = t0 + 100
    a_rows = [bgz for t, bgz in a_bias if t0 <= t < t1]
    b_rows = [bgz for t, bgz in b_bias if t0 <= t < t1]
    if not a_rows or not b_rows:
        continue
    am = float(np.mean(a_rows))
    bm = float(np.mean(b_rows))
    print(f"  {t0:.0f}-{t1:.0f}    {am*1e4:11.4f}  {bm*1e4:11.4f}  {(bm-am)*1e4:10.4f}")

# ── Integrated heading error from bg_z ────────────────────────────────────
print()
print("=" * 64)
print("  Integrated heading from bg_z — running integral (degrees)")
print("  (= cumulative heading drift from gyro Z-bias alone)")
print("=" * 64)
print(f"  {'t':>6s}  {'A_bgz×1e4':>11s}  {'B_bgz×1e4':>11s}  {'A_∫bgz_deg':>12s}  {'B_∫bgz_deg':>12s}")
print(f"  {'-'*60}")
a_integ = 0.0
b_integ = 0.0
a_prev_t = None
b_prev_t = None
a_ptr = 0
b_ptr = 0
for t_snap in np.arange(924, 2820, 100):
    # advance to nearest sample
    while a_ptr + 1 < len(a_bias) and a_bias[a_ptr+1][0] <= t_snap:
        dt = a_bias[a_ptr+1][0] - a_bias[a_ptr][0]
        a_integ += a_bias[a_ptr][1] * dt * (180.0/math.pi)
        a_ptr += 1
    while b_ptr + 1 < len(b_bias) and b_bias[b_ptr+1][0] <= t_snap:
        dt = b_bias[b_ptr+1][0] - b_bias[b_ptr][0]
        b_integ += b_bias[b_ptr][1] * dt * (180.0/math.pi)
        b_ptr += 1
    am = a_bias[a_ptr][1] if a_ptr < len(a_bias) else 0
    bm = b_bias[b_ptr][1] if b_ptr < len(b_bias) else 0
    print(f"  {t_snap:6.0f}  {am*1e4:11.4f}  {bm*1e4:11.4f}  {a_integ:12.2f}  {b_integ:12.2f}")

print("\nDONE")
