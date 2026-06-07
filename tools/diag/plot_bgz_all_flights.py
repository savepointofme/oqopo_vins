#!/usr/bin/env python3
"""Plot bg_z timelines for FEJ-OC-prechi2, global_oc_alpha1, and B_current across all flights."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

OUT = "/mnt/c/Users/baloney/Desktop/bgz_all_flights.png"

# ── Data definitions ──────────────────────────────────────────────────────────

FLY4_BASE = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528"
FLY3_BASE = "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527"
FLY2_BASE = "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525"
FLY1_BASE = "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result"

SERIES = {
    "fly4": [
        ("FEJ-OC-prechi2",   f"{FLY4_BASE}/fej_oc_prechi2_start924p4/traj.txt.bias",               "C0", "-"),
        ("global_oc_alpha1", f"{FLY4_BASE}/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/traj.txt.bias", "C2", "--"),
        ("B_current",        f"{FLY4_BASE}/fly4_B_bgz1p0_start924p4_until2816/traj.txt.bias",       "C3", ":"),
        ("B_bgz0.25",        f"{FLY4_BASE}/fly4_B_bgz0p25_start924p4_until2816/traj.txt.bias",      "C1", "-."),
    ],
    "fly3": [
        ("FEJ-OC-prechi2",   f"{FLY3_BASE}/fej_oc_prechi2_start618/traj.txt.bias",                 "C0", "-"),
        ("global_oc_alpha1", f"{FLY3_BASE}/yaw_method_ablation_offset438p0_start618/global_oc_alpha1/traj.txt.bias", "C2", "--"),
    ],
    "fly2": [
        ("FEJ-OC-prechi2",   f"{FLY2_BASE}/fej_oc_prechi2_start700/traj.txt.bias",                 "C0", "-"),
        ("global_oc_alpha1", f"{FLY2_BASE}/clean_ablate_oc_alpha10/traj.txt.bias",                  "C2", "--"),
    ],
    "fly1": [
        ("FEJ-OC-prechi2",   f"{FLY1_BASE}/fej_oc_prechi2_start930/traj.txt.bias",                 "C0", "-"),
        ("global_oc_alpha1", f"{FLY1_BASE}/A1_fcinit_px2_globaloc1p0_traj.txt.bias",               "C2", "--"),
    ],
}

FLIGHT_LABELS = {
    "fly1": "Fly 1  (start 930 s)",
    "fly2": "Fly 2  (start 700 s)",
    "fly3": "Fly 3  (start 618 s)",
    "fly4": "Fly 4  (start 924 s)",
}

# ── Loader ────────────────────────────────────────────────────────────────────

def load_bgz(path):
    """Load (t, bg_z) from traj.txt.bias  [col 0 = t, col 6 = bg_z]."""
    ts, bgz = [], []
    if not os.path.exists(path):
        return np.array([]), np.array([])
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) >= 7:
                try:
                    ts.append(float(p[0]))
                    bgz.append(float(p[6]) * 1e4)   # convert to ×1e-4 rad/s
                except ValueError:
                    pass
    return np.array(ts), np.array(bgz)

def smooth(ts, vals, bin_s=30.0):
    """Bin-average with bin_s seconds."""
    if len(ts) == 0:
        return np.array([]), np.array([])
    t0, t1 = ts[0], ts[-1]
    bins = np.arange(t0, t1 + bin_s, bin_s)
    t_out, v_out = [], []
    for a, b in zip(bins[:-1], bins[1:]):
        mask = (ts >= a) & (ts < b)
        if mask.sum() > 0:
            t_out.append(0.5 * (a + b))
            v_out.append(np.mean(vals[mask]))
    return np.array(t_out), np.array(v_out)

# ── Plot ──────────────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(16, 10))
fig.suptitle("bg_z trajectory (gyro Z-bias, ×10⁻⁴ rad/s) — FEJ-OC-prechi2 vs global_oc_alpha1",
             fontsize=13, fontweight="bold", y=0.98)

gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.38, wspace=0.28)

axes_order = [("fly1", gs[0, 0]), ("fly2", gs[0, 1]),
              ("fly3", gs[1, 0]), ("fly4", gs[1, 1])]

for flight, pos in axes_order:
    ax = fig.add_subplot(pos)
    ax.set_title(FLIGHT_LABELS[flight], fontsize=11, pad=4)
    ax.set_xlabel("Time (s)", fontsize=9)
    ax.set_ylabel("bg_z  (×10⁻⁴ rad/s)", fontsize=9)
    ax.axhline(0, color="k", lw=0.5, ls="-", alpha=0.3)
    ax.tick_params(labelsize=8)

    any_plotted = False
    for label, path, color, ls in SERIES.get(flight, []):
        ts, bgz = load_bgz(path)
        if len(ts) == 0:
            continue
        t_s, v_s = smooth(ts, bgz, bin_s=30.0)
        lw = 2.0 if label == "FEJ-OC-prechi2" else 1.5
        alpha = 1.0 if label in ("FEJ-OC-prechi2", "global_oc_alpha1") else 0.75
        ax.plot(t_s, v_s, color=color, ls=ls, lw=lw, alpha=alpha, label=label)
        any_plotted = True

    if any_plotted:
        ax.legend(fontsize=7.5, loc="lower left", framealpha=0.8)

    ax.grid(True, alpha=0.25, lw=0.5)

# ── Annotation ────────────────────────────────────────────────────────────────
note = (
    "bg_z correctly estimated ≈ 4–6×10⁻⁴ (positive)  |  "
    "Negative drift → heading under-compensated  |  "
    "30-second bin average"
)
fig.text(0.5, 0.01, note, ha="center", fontsize=8, color=(0.27, 0.27, 0.27), style="italic")

plt.savefig(OUT, dpi=140, bbox_inches="tight")
print(f"Saved: {OUT}")
