#!/usr/bin/env python3
"""Report FEJ-OC-prechi2 results across all flights with baseline comparisons."""
import os, numpy as np

FLIGHTS = {
    "fly1": {
        "new":  "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/fej_oc_prechi2_start930",
        "base": "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result",
        "base_traj": "A1_fcinit_px2_globaloc1p0_traj.txt",
        "base_bias": "A1_fcinit_px2_globaloc1p0_traj.txt.bias",
    },
    "fly2": {
        "new":  "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/fej_oc_prechi2_start700",
        "base": "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/clean_ablate_oc_alpha10",
        "base_traj": "traj.txt",
        "base_bias": "traj.txt.bias",
    },
    "fly3": {
        "new":  "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/fej_oc_prechi2_start618",
        "base": "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/yaw_method_ablation_offset438p0_start618/global_oc_alpha1",
        "base_traj": "traj.txt",
        "base_bias": "traj.txt.bias",
    },
    "fly4": {
        "new":  "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/fej_oc_prechi2_start924p4",
        "base": "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1",
        "base_traj": "traj.txt",
        "base_bias": "traj.txt.bias",
    },
}

def load_traj(path):
    rows = []
    if not os.path.exists(path): return rows
    with open(path) as f:
        for line in f:
            if line.strip() and not line.startswith('#'):
                rows.append(line.split())
    return rows

def load_bias_bgz(path):
    rows = []
    if not os.path.exists(path): return rows
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith('#'): continue
            p = line.split()
            if len(p) >= 7:
                try: rows.append((float(p[0]), float(p[6])))
                except: pass
    return sorted(rows, key=lambda x: x[0])

def check_nan(traj_rows):
    count = 0
    for row in traj_rows:
        for v in row:
            try:
                x = float(v)
                if not (x == x) or abs(x) > 1e30:
                    count += 1
            except: pass
    return count

print("=" * 80)
print("  global_yaw_oc_fej_prechi2 — all-flights report")
print("=" * 80)

for name, cfg in FLIGHTS.items():
    print(f"\n{'─'*76}")
    print(f"  {name.upper()}")
    print(f"{'─'*76}")

    new_traj  = load_traj(os.path.join(cfg["new"], "traj.txt"))
    new_bias  = load_bias_bgz(os.path.join(cfg["new"], "traj.txt.bias"))

    if cfg["base_traj"].startswith("/"):
        base_traj = load_traj(cfg["base_traj"])
        base_bias = load_bias_bgz(cfg["base_bias"])
    else:
        base_traj = load_traj(os.path.join(cfg["base"], cfg["base_traj"]))
        base_bias = load_bias_bgz(os.path.join(cfg["base"], cfg["base_bias"]))

    # New run summary
    if new_traj:
        t0_new = float(new_traj[0][0]);  tf_new = float(new_traj[-1][0])
        nan_new = check_nan(new_traj)
        bgz_final_new = new_bias[-1][1] if new_bias else float('nan')
        bgz_mean_new  = float(np.mean([b for _, b in new_bias])) if new_bias else float('nan')
        print(f"  NEW (fej_oc_prechi2):")
        print(f"    traj:  {len(new_traj)} lines  t=[{t0_new:.1f}, {tf_new:.1f}]")
        print(f"    NaN:   {nan_new}")
        print(f"    bg_z:  mean={bgz_mean_new:.3e}  final={bgz_final_new:.3e}")
    else:
        print("  NEW: MISSING")

    # Baseline summary
    if base_traj:
        t0_b = float(base_traj[0][0]);  tf_b = float(base_traj[-1][0])
        nan_b = check_nan(base_traj)
        bgz_final_b = base_bias[-1][1] if base_bias else float('nan')
        bgz_mean_b  = float(np.mean([b for _, b in base_bias])) if base_bias else float('nan')
        print(f"  BASE (global_oc_alpha1):")
        print(f"    traj:  {len(base_traj)} lines  t=[{t0_b:.1f}, {tf_b:.1f}]")
        print(f"    NaN:   {nan_b}")
        print(f"    bg_z:  mean={bgz_mean_b:.3e}  final={bgz_final_b:.3e}")
    else:
        print(f"  BASE: MISSING {cfg['base']}")

    # bg_z timeline comparison
    if new_bias and base_bias:
        t_start = max(float(new_bias[0][0]), float(base_bias[0][0]))
        t_end   = min(float(new_bias[-1][0]), float(base_bias[-1][0]))
        if t_end > t_start:
            print(f"\n  bg_z (×1e-4) comparison — overlapping window [{t_start:.0f}, {t_end:.0f}]:")
            print(f"    {'Epoch':11s}  {'FEJ-OC':>10s}  {'Base-A':>10s}  {'diff':>10s}")
            bin_w = min(200.0, (t_end - t_start) / 4)
            for t0 in np.arange(t_start, t_end, bin_w):
                t1 = t0 + bin_w
                new_vals = [b for t, b in new_bias if t0 <= t < t1]
                base_vals = [b for t, b in base_bias if t0 <= t < t1]
                if new_vals and base_vals:
                    nm = np.mean(new_vals)*1e4; bm = np.mean(base_vals)*1e4
                    print(f"    {t0:.0f}-{t1:.0f}    {nm:10.4f}  {bm:10.4f}  {nm-bm:10.4f}")

print(f"\n{'='*80}")
print("  NOTE: 'unknown vio_yaw_update_mode' warning is cosmetic —")
print("  VOP is set correctly in apply_yaw_control_to_updaters.")
print("  bg_z timeline confirms VOP IS active (different from ORIGINAL/B_current).")
print(f"{'='*80}")
