#!/usr/bin/env python3
"""
Per-fly comparison plots: altitude curve + XY-plane trajectory.

For each fly (1..4), overlays:
  - no-GPS monocular baseline
  - current best GPS-aided run
  - one clearly divergent reference run

Divergence rule:
  A sample is considered "diverged" if either
    (a) max(|x|, |y|, |z|) > POS_LIMIT_M (default 500 m), or
    (b) consecutive-pose displacement > JUMP_LIMIT_M (default 50 m).
  The curve is truncated at the LAST good sample, marked with a large 'X'.
"""

import os
import sys
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = "/mnt/d/vscode_dir/open_vins" if os.path.isdir("/mnt/d") else "d:/vscode_dir/open_vins"
OUT_DIR = os.path.join(ROOT, "comparison_plots")

POS_LIMIT_M = 500.0   # absolute position threshold for divergence
JUMP_LIMIT_M = 50.0   # consecutive-pose jump threshold

# ----------------------------------------------------------------------------
# Run selection per fly.  Each "run" entry = (label, traj_path, style)
# Truth path is loaded separately (ASL CSV with x/y/z columns).
# ----------------------------------------------------------------------------
RUNS = {
    "fly1": {
        "truth": "20260509_fly1/result/gps_tum_time_alignment/truth_asl_cam_time.csv",
        "runs": [
            ("no-GPS baseline (pr20_mono_nogps)",
             "20260509_fly1/result/pr20_mono_nogps.txt",
             dict(color="tab:gray", lw=1.5, ls="-")),
            ("GPS best (pr20_robust_gps_rel)",
             "20260509_fly1/result/pr20_robust_gps_rel_fly1.txt",
             dict(color="tab:blue", lw=2.0, ls="-")),
            ("GPS divergent (pr20_fixed_rel)",
             "20260509_fly1/result/pr20_fixed_rel_fly1.txt",
             dict(color="tab:red", lw=1.2, ls="--")),
        ],
        "gps_csv": "20260509_fly1/result/gps_tum_time_alignment/aligned_gps_cam_time.csv",
    },
    "fly2": {
        "truth": "20260509_fly2/result/gps_tum_time_alignment/truth_asl_cam_time.csv",
        "runs": [
            ("no-GPS baseline (pr17_mono_phase0)",
             "20260509_fly2/result/pr17_mono_phase0_tum.txt",
             dict(color="tab:gray", lw=1.5, ls="-")),
            ("GPS best (pr20_fixed_abs σ=0.3)",
             "20260509_fly2/result/pr20_fixed_abs_sigma0.3.txt",
             dict(color="tab:blue", lw=2.0, ls="-")),
            ("GPS divergent (pr20_fixed_abs σ=2.0)",
             "20260509_fly2/result/pr20_fixed_abs_sigma2.0.txt",
             dict(color="tab:red", lw=1.2, ls="--")),
        ],
        "gps_csv": "20260509_fly2/result/gps_tum_time_alignment/aligned_gps_cam_time.csv",
    },
    "fly3": {
        "truth": "20260509_fly3/result/gps_tum_time_alignment/truth_asl_cam_time.csv",
        "runs": [
            ("no-GPS baseline (mono start185)",
             "20260509_fly3/result/no_gps_baseline/traj_mono_nogps_start185.txt",
             dict(color="tab:gray", lw=1.5, ls="-")),
            ("GPS best (pr20_s0.3_st182)",
             "20260509_fly3/result/pr20_s0.3_st182.txt",
             dict(color="tab:blue", lw=2.0, ls="-")),
            ("GPS divergent (pr20_fixed_abs)",
             "20260509_fly3/result/pr20_fixed_abs_fly3.txt",
             dict(color="tab:red", lw=1.2, ls="--")),
        ],
        "gps_csv": "20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv",
    },
    "fly4": {
        "truth": "20260509_fly4/result/gps_tum_time_alignment/truth_asl_cam_time.csv",
        "runs": [
            ("no-GPS baseline (R0)",
             "20260509_fly4/result/stage_a_v2/R0_nogps.txt",
             dict(color="tab:gray", lw=1.5, ls="-")),
            ("Stage A best (R2f, σ=2.0 full-state)",
             "20260509_fly4/result/stage_a_v2/R2f_gplane_sigma20_full.txt",
             dict(color="tab:blue", lw=2.0, ls="-")),
            ("Stage A divergent (R1z, σ=0.3 zonly)",
             "20260509_fly4/result/stage_a_v2/R1z_gplane_sigma03_zonly.txt",
             dict(color="tab:red", lw=1.2, ls="--")),
        ],
        "gps_csv": "20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv",
    },
}


def abspath(rel):
    return os.path.join(ROOT, rel)


def load_traj(path):
    """Load TUM trajectory (t x y z ...).  Returns Nx4 numpy [t,x,y,z]."""
    rows = []
    last_t = -math.inf
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split()
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0]); x = float(parts[1]); y = float(parts[2]); z = float(parts[3])
            except ValueError:
                continue
            if t <= last_t:  # drop out-of-order tail glitches
                continue
            rows.append((t, x, y, z))
            last_t = t
    if not rows:
        return None
    return np.asarray(rows)


def load_truth(path):
    """Load ASL ground-truth CSV (ts_ns, x, y, z, ...).  Returns Nx4 [t_s,x,y,z]."""
    if not os.path.isfile(path):
        return None
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split(",")
            if len(parts) < 4:
                continue
            try:
                ts = float(parts[0]) * 1e-9
                x = float(parts[1]); y = float(parts[2]); z = float(parts[3])
                rows.append((ts, x, y, z))
            except ValueError:
                continue
    if not rows:
        return None
    arr = np.asarray(rows)
    arr = arr[np.argsort(arr[:, 0])]
    return arr


def load_gps_last_ts(path):
    """Return last GPS timestamp (seconds) — for the vertical 'GPS ends' marker."""
    if not os.path.isfile(path):
        return None
    last = None
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split(",")
            try:
                last = float(parts[0]) * 1e-9
            except (ValueError, IndexError):
                pass
    return last


def find_divergence(traj):
    """
    Return (n_good, reason) where n_good = number of samples to KEEP (first
    n_good samples are valid).  reason is a short string for legend / debug.
    """
    if traj is None or traj.shape[0] == 0:
        return 0, "empty"
    n = traj.shape[0]
    xyz = traj[:, 1:4]
    # rule (a): absolute position limit
    abs_bad = np.where(np.max(np.abs(xyz), axis=1) > POS_LIMIT_M)[0]
    cut_a = abs_bad[0] if len(abs_bad) else n
    # rule (b): consecutive jump
    if n >= 2:
        diff = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
        jump_bad = np.where(diff > JUMP_LIMIT_M)[0]
        cut_b = jump_bad[0] + 1 if len(jump_bad) else n
    else:
        cut_b = n
    n_good = min(cut_a, cut_b)
    if n_good == n:
        return n, "no divergence"
    if n_good == cut_a:
        return n_good, f"|xyz|>{POS_LIMIT_M:.0f}m"
    return n_good, f"jump>{JUMP_LIMIT_M:.0f}m"


def plot_fly(name, cfg, save_path=None, fig=None, axes=None):
    """Render the 2-panel comparison for one fly."""
    truth = load_truth(abspath(cfg["truth"]))
    gps_end_ts = load_gps_last_ts(abspath(cfg["gps_csv"])) if cfg.get("gps_csv") else None

    if fig is None:
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    ax_z, ax_xy = axes

    # Truth — used as time origin & visual reference
    t0_ref = None
    if truth is not None:
        t0_ref = truth[0, 0]
        ax_z.plot(truth[:, 0] - t0_ref, truth[:, 3],
                  color="black", lw=1.4, ls=":", alpha=0.85,
                  label="truth (ASL gt)")
        ax_xy.plot(truth[:, 1], truth[:, 2],
                   color="black", lw=1.4, ls=":", alpha=0.85,
                   label="truth (ASL gt)")

    legend_notes = []
    plotted_anything = False

    for label, rel_path, style in cfg["runs"]:
        path = abspath(rel_path)
        if not os.path.isfile(path):
            legend_notes.append(f"{label}: FILE MISSING")
            continue
        traj = load_traj(path)
        if traj is None or traj.shape[0] < 2:
            legend_notes.append(f"{label}: empty trajectory")
            continue
        n_good, reason = find_divergence(traj)
        plotted_anything = True

        if t0_ref is None:
            t0_ref = traj[0, 0]

        t_rel = traj[:, 0] - t0_ref
        kept = traj[:n_good]
        kept_t = t_rel[:n_good]

        label_full = label
        if n_good < traj.shape[0]:
            label_full = f"{label} — truncated ({reason})"

        ax_z.plot(kept_t, kept[:, 3], label=label_full, **style)
        ax_xy.plot(kept[:, 1], kept[:, 2], label=label_full, **style)

        if n_good < traj.shape[0]:
            ax_z.plot(kept_t[-1], kept[-1, 3], "X",
                      color=style.get("color", "k"), markersize=14, mec="black", mew=1.5)
            ax_xy.plot(kept[-1, 1], kept[-1, 2], "X",
                       color=style.get("color", "k"), markersize=14, mec="black", mew=1.5)

    # GPS-end vertical line on height plot
    if gps_end_ts is not None and t0_ref is not None:
        ax_z.axvline(gps_end_ts - t0_ref, color="purple", lw=1.0, ls="--", alpha=0.6,
                     label=f"last GPS (t={gps_end_ts - t0_ref:.0f}s)")

    ax_z.set_xlabel("time since first truth sample (s)", fontsize=12)
    ax_z.set_ylabel("z / altitude (m)", fontsize=12)
    ax_z.set_title(f"{name}: altitude vs time", fontsize=13)
    ax_z.grid(True, alpha=0.3)
    ax_z.legend(loc="best", fontsize=8)

    ax_xy.set_xlabel("X (m)", fontsize=12)
    ax_xy.set_ylabel("Y (m)", fontsize=12)
    ax_xy.set_title(f"{name}: XY-plane trajectory", fontsize=13)
    ax_xy.grid(True, alpha=0.3)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(loc="best", fontsize=8)

    if save_path is not None:
        fig.suptitle(f"{name}  (× = divergence cutoff, |xyz|>{POS_LIMIT_M:.0f}m or jump>{JUMP_LIMIT_M:.0f}m)",
                     fontsize=14)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        fig.savefig(save_path, dpi=120)
        plt.close(fig)
        print(f"[saved] {save_path}")
    if legend_notes:
        for n in legend_notes:
            print(f"  NOTE [{name}]: {n}")


def make_summary(out_path):
    fig, axes = plt.subplots(len(RUNS), 2, figsize=(15, 4.6 * len(RUNS)))
    if len(RUNS) == 1:
        axes = np.array([axes])
    for row, (name, cfg) in enumerate(RUNS.items()):
        plot_fly(name, cfg, save_path=None, fig=fig, axes=axes[row])
    fig.suptitle(f"All-fly comparison  (× = divergence cutoff, |xyz|>{POS_LIMIT_M:.0f}m or jump>{JUMP_LIMIT_M:.0f}m)",
                 fontsize=15)
    fig.tight_layout(rect=[0, 0, 1, 0.985])
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f"[saved] {out_path}")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, cfg in RUNS.items():
        plot_fly(name, cfg, save_path=os.path.join(OUT_DIR, f"comparison_{name}.png"))
    make_summary(os.path.join(OUT_DIR, "comparison_summary.png"))


if __name__ == "__main__":
    sys.exit(main())
