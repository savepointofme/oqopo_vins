#!/usr/bin/env python3
"""Compare bg_z timelines between FEJ-OC-prechi2 and global_oc_alpha1."""
import numpy as np, os

BASE = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528"

files = {
    "FEJ-OC-prechi2 (start 924.4)":
        f"{BASE}/fej_oc_prechi2_start924p4/traj.txt.bias",
    "Global-OC-alpha1 (start 904.4)":
        f"{BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/traj.txt.bias",
    "B_current (start 924.4)":
        f"{BASE}/fly4_B_bgz1p0_start924p4_until2816/traj.txt.bias",
}

def load(path):
    rows = []
    try:
        with open(path) as f:
            for line in f:
                if line.startswith('#') or not line.strip(): continue
                p = line.split()
                if len(p) >= 7:
                    rows.append((float(p[0]), float(p[6])))
    except: pass
    return sorted(rows, key=lambda x: x[0])

data = {k: load(v) for k, v in files.items()}
for k, rows in data.items():
    if rows:
        print(f"{k}: {len(rows)} rows  t=[{rows[0][0]:.1f}, {rows[-1][0]:.1f}]  bg_z_0={rows[0][1]:.3e}  bg_z_final={rows[-1][1]:.3e}")
    else:
        print(f"{k}: MISSING {files[k]}")

print()
print(f"{'Epoch':11s}", end="")
for k in data: print(f"  {k[:20]:>22s}", end="")
print()
print("-" * 90)

for t0 in np.arange(925, 2820, 200):
    t1 = t0 + 200
    print(f"{t0:.0f}-{t1:.0f}   ", end="")
    for rows in data.values():
        rs = [bgz for t, bgz in rows if t0 <= t < t1]
        if rs:
            print(f"  {np.mean(rs)*1e4:22.4f}", end="")
        else:
            print(f"  {'N/A':>22s}", end="")
    print()
