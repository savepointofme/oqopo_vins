#!/usr/bin/env python3
"""
Phase-0 Stage B diagnostic plots + metrics for fly4.

Generates:
  - XY trajectory overlay (with GPS-end marker + divergence ×)
  - Z curve vs time
  - XY error vs time
  - Z error vs time
  - Post-GPS XY error vs time-since-cutoff (drift-rate plot, with linear fit)
  - Running (cumulative) post-GPS XY RMSE

Reports per-run table with:
  - during/post GPS XY & Z RMSE
  - post-GPS max / final XY error, max / final Z error
  - drift rate (slope) and final-minus-initial error in post window
  - time-to-cross XY error thresholds (50/100/200 m)
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = "/mnt/d/vscode_dir/open_vins" if os.path.isdir("/mnt/d") else "d:/vscode_dir/open_vins"
RESULT_DIR = os.path.join(ROOT, "20260509_fly4/result/stage_a_v2")
OUT_DIR = os.path.join(ROOT, "comparison_plots/stage_b_diag")
TRUTH_CSV = os.path.join(ROOT, "20260509_fly4/result/gps_tum_time_alignment/truth_asl_cam_time.csv")
GPS_CSV = os.path.join(ROOT, "20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv")

POS_LIMIT_M = 500.0
JUMP_LIMIT_M = 50.0
ALIGN_WINDOW_SEC = 5.0      # seconds at startup used to compute translation offset

RUNS = [
    ("R0_nogps",     "R0 no-GPS baseline",            "R0_nogps.txt",
        dict(color="tab:gray",   lw=1.6, ls="-")),
    ("R2f_stageA",   "R2f Stage-A best (σ=2.0, full)",  "R2f_gplane_sigma20_full.txt",
        dict(color="tab:blue",   lw=2.0, ls="-")),
    ("R5b_stageB",   "R5b Stage-B (σ_px=50, K=2)",     "R5b_extreme_K2.txt",
        dict(color="tab:green",  lw=2.0, ls="-")),
    ("R4b_stageB",   "R4b Stage-B (σ_px=5, K=5)",      "R4b_pre_msckf_loose.txt",
        dict(color="tab:orange", lw=1.4, ls="--")),
    ("R5a_stageB",   "R5a Stage-B (σ_px=20, K=3)",     "R5a_loose20_K3.txt",
        dict(color="tab:red",    lw=1.4, ls="--")),
]


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------
def load_traj(path):
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
            if t <= last_t:
                continue
            rows.append((t, x, y, z))
            last_t = t
    return np.asarray(rows) if rows else np.zeros((0, 4))


def load_truth(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split(",")
            try:
                ts = float(parts[0]) * 1e-9
                x = float(parts[1]); y = float(parts[2]); z = float(parts[3])
                rows.append((ts, x, y, z))
            except (ValueError, IndexError):
                continue
    arr = np.asarray(rows)
    return arr[np.argsort(arr[:, 0])]


def load_gps_last_t(path):
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


# ---------------------------------------------------------------------------
# Divergence detection
# ---------------------------------------------------------------------------
def find_divergence(traj):
    if traj.shape[0] == 0:
        return 0, "empty"
    xyz = traj[:, 1:4]
    abs_bad = np.where(np.max(np.abs(xyz), axis=1) > POS_LIMIT_M)[0]
    cut_a = abs_bad[0] if len(abs_bad) else traj.shape[0]
    if traj.shape[0] >= 2:
        diff = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
        jump_bad = np.where(diff > JUMP_LIMIT_M)[0]
        cut_b = jump_bad[0] + 1 if len(jump_bad) else traj.shape[0]
    else:
        cut_b = traj.shape[0]
    n_good = min(cut_a, cut_b)
    if n_good == traj.shape[0]:
        return n_good, "no divergence"
    return n_good, ("|xyz|>%.0fm" % POS_LIMIT_M if n_good == cut_a else "jump>%.0fm" % JUMP_LIMIT_M)


# ---------------------------------------------------------------------------
# Truth interpolation + translation alignment
# ---------------------------------------------------------------------------
def interp_truth(truth, t_query):
    """Linear-interpolate truth XYZ at t_query.  NaN if outside truth time range."""
    if truth.shape[0] == 0:
        return np.full((len(t_query), 3), np.nan)
    t_truth = truth[:, 0]
    out = np.full((len(t_query), 3), np.nan)
    for axis in range(3):
        out[:, axis] = np.interp(t_query, t_truth, truth[:, 1 + axis],
                                 left=np.nan, right=np.nan)
    return out


def translation_offset(vio, truth, align_window_sec):
    """Compute mean(truth - vio) over first align_window_sec of VIO samples."""
    if vio.shape[0] == 0:
        return np.zeros(3)
    t0 = vio[0, 0]
    mask = vio[:, 0] - t0 < align_window_sec
    sub = vio[mask]
    if sub.shape[0] < 5:
        sub = vio[:5]
    truth_at = interp_truth(truth, sub[:, 0])
    diff = truth_at - sub[:, 1:4]
    valid = ~np.any(np.isnan(diff), axis=1)
    if not np.any(valid):
        return np.zeros(3)
    return np.mean(diff[valid], axis=0)


# ---------------------------------------------------------------------------
# Per-run computation
# ---------------------------------------------------------------------------
def process_run(slug, label, traj_path, truth, t_split):
    traj = load_traj(traj_path)
    if traj.shape[0] == 0:
        return None
    n_good, reason = find_divergence(traj)
    traj_kept = traj[:n_good]

    offset = translation_offset(traj_kept, truth, ALIGN_WINDOW_SEC)
    truth_at = interp_truth(truth, traj_kept[:, 0])
    vio_xyz_aligned = traj_kept[:, 1:4] + offset
    err = vio_xyz_aligned - truth_at  # NaN where truth missing
    e_xy = np.sqrt(err[:, 0] ** 2 + err[:, 1] ** 2)
    e_z = np.abs(err[:, 2])

    t = traj_kept[:, 0]
    valid = ~(np.isnan(e_xy) | np.isnan(e_z))
    t_v = t[valid]
    e_xy_v = e_xy[valid]
    e_z_v = e_z[valid]
    vio_z_v = vio_xyz_aligned[valid, 2]
    truth_z_v = truth_at[valid, 2]

    during = t_v <= t_split
    post = t_v > t_split

    def rmse(a):
        return float(np.sqrt(np.mean(a ** 2))) if a.size else float("nan")

    metrics = {
        "slug": slug,
        "label": label,
        "diverged_at_t": (None if n_good == traj.shape[0]
                          else float(traj[n_good - 1, 0])),
        "div_reason": reason,
        "n_traj": traj.shape[0],
        "n_kept": n_good,
        "offset_xyz": offset.tolist(),
        "during_n":   int(np.sum(during)),
        "during_xy_rmse": rmse(e_xy_v[during]),
        "during_z_rmse":  rmse(e_z_v[during]),
        "post_n":     int(np.sum(post)),
        "post_xy_rmse": rmse(e_xy_v[post]),
        "post_z_rmse":  rmse(e_z_v[post]),
        "post_xy_max":  float(e_xy_v[post].max()) if np.any(post) else float("nan"),
        "post_z_max":   float(e_z_v[post].max())  if np.any(post) else float("nan"),
        "post_xy_final": float(e_xy_v[post][-1])  if np.any(post) else float("nan"),
        "post_z_final":  float(e_z_v[post][-1])   if np.any(post) else float("nan"),
        "post_xy_initial": float(e_xy_v[post][0]) if np.any(post) else float("nan"),
        "post_z_initial":  float(e_z_v[post][0])  if np.any(post) else float("nan"),
        "drift_rate_xy_per_s": float("nan"),
    }
    if np.any(post) and np.sum(post) >= 5:
        t_post = t_v[post] - t_split
        # OLS slope (force intercept ~ initial error so the slope reflects growth)
        slope, _ = np.polyfit(t_post, e_xy_v[post], 1)
        metrics["drift_rate_xy_per_s"] = float(slope)
    # time-to-threshold (50/100/200 m XY)
    for thresh in [50.0, 100.0, 200.0]:
        cross = np.where(e_xy_v >= thresh)[0]
        if cross.size:
            metrics[f"t_xy_ge_{int(thresh)}m"] = float(t_v[cross[0]] - traj[0, 0])
            metrics[f"in_window_xy_ge_{int(thresh)}m"] = (
                "during" if t_v[cross[0]] <= t_split else "post")
        else:
            metrics[f"t_xy_ge_{int(thresh)}m"] = None
            metrics[f"in_window_xy_ge_{int(thresh)}m"] = "never"

    series = {
        "t": t_v,
        "e_xy": e_xy_v,
        "e_z": e_z_v,
        "vio_z": vio_z_v,
        "truth_z": truth_z_v,
        "xy_aligned": vio_xyz_aligned[valid, :2],
    }
    return metrics, series, traj_kept, offset, n_good < traj.shape[0]


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_xy_traj(processed, truth, t_split, out_path):
    fig, ax = plt.subplots(1, 1, figsize=(11, 9))
    # truth (only span where we have data)
    ax.plot(truth[:, 1], truth[:, 2], color="black", lw=1.6, ls=":", label="truth")
    for slug, label, traj_kept, offset, was_truncated, m, s, style in processed:
        ax.plot(s["xy_aligned"][:, 0], s["xy_aligned"][:, 1],
                label=label + (" [trunc]" if was_truncated else ""),
                **style)
        if was_truncated:
            ax.plot(s["xy_aligned"][-1, 0], s["xy_aligned"][-1, 1], "X",
                    color=style["color"], markersize=14, mec="black", mew=1.5)
        # mark sample at t_split (GPS-end)
        idx = np.searchsorted(s["t"], t_split)
        if 0 < idx < len(s["t"]):
            ax.plot(s["xy_aligned"][idx, 0], s["xy_aligned"][idx, 1], "o",
                    color=style["color"], markersize=9, mec="black", mew=1.0)
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title("fly4 XY trajectory (translation-aligned to truth over first %.0fs).\n"
                 "Circle = GPS cutoff sample; × = divergence-truncation point."
                 % ALIGN_WINDOW_SEC,
                 fontsize=12)
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[saved] {out_path}")


def plot_z_curve(processed, truth, t_split, out_path):
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    t0 = truth[0, 0]
    ax.plot(truth[:, 0] - t0, truth[:, 3], color="black", lw=1.4, ls=":", label="truth")
    for slug, label, traj_kept, offset, was_truncated, m, s, style in processed:
        ax.plot(s["t"] - t0, s["vio_z"],
                label=label + (" [trunc]" if was_truncated else ""),
                **style)
        if was_truncated:
            ax.plot(s["t"][-1] - t0, s["vio_z"][-1], "X",
                    color=style["color"], markersize=12, mec="black", mew=1.2)
    ax.axvline(t_split - t0, color="purple", lw=1.0, ls="--", alpha=0.7,
               label="last GPS sample")
    ax.set_xlabel("time since first truth sample (s)", fontsize=12)
    ax.set_ylabel("z (m)", fontsize=12)
    ax.set_title("fly4 altitude vs time (truth-aligned)", fontsize=12)
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[saved] {out_path}")


def plot_err_vs_time(processed, t_split, out_path, kind):
    """kind: 'xy' or 'z'."""
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    t0 = min(s["t"][0] for *_, s, _ in [(*x[:6], x[6], x[7]) for x in processed] if s["t"].size)
    for slug, label, traj_kept, offset, was_truncated, m, s, style in processed:
        y = s["e_xy"] if kind == "xy" else s["e_z"]
        ax.plot(s["t"] - t0, y,
                label=label + (" [trunc]" if was_truncated else ""),
                **style)
        if was_truncated:
            ax.plot(s["t"][-1] - t0, y[-1], "X", color=style["color"],
                    markersize=12, mec="black", mew=1.2)
    ax.axvline(t_split - t0, color="purple", lw=1.0, ls="--", alpha=0.7,
               label="last GPS sample")
    if kind == "xy":
        for thresh in [50, 100, 200]:
            ax.axhline(thresh, color="gray", lw=0.7, ls=":", alpha=0.5)
        ax.set_ylabel(r"$e_{xy}(t) = \sqrt{(x_e-x_t)^2 + (y_e-y_t)^2}$  (m)", fontsize=12)
        ax.set_title("fly4 XY error vs time", fontsize=12)
    else:
        ax.set_ylabel(r"$|e_z(t)| = |z_e - z_t|$  (m)", fontsize=12)
        ax.set_title("fly4 Z error vs time", fontsize=12)
    ax.set_xlabel("time since first truth sample (s)", fontsize=12)
    ax.set_yscale("symlog", linthresh=1.0)
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[saved] {out_path}")


def plot_post_drift(processed, t_split, out_path):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    for slug, label, traj_kept, offset, was_truncated, m, s, style in processed:
        mask = s["t"] > t_split
        if not np.any(mask):
            continue
        tau = s["t"][mask] - t_split
        e_xy_post = s["e_xy"][mask]
        ax1.plot(tau, e_xy_post,
                 label=label + (" [trunc]" if was_truncated else ""),
                 **style)
        # linear-fit overlay
        if tau.size >= 5:
            slope, intercept = np.polyfit(tau, e_xy_post, 1)
            fit_lbl = f"  fit slope = {slope:+.2f} m/s"
            ax1.plot(tau, slope * tau + intercept,
                     color=style["color"], lw=0.8, ls=":", alpha=0.7,
                     label=fit_lbl)
        # running RMSE (right panel)
        running_rmse = np.sqrt(np.cumsum(e_xy_post ** 2) /
                               np.arange(1, len(e_xy_post) + 1))
        ax2.plot(tau, running_rmse, label=label, **style)
    for ax in (ax1, ax2):
        ax.set_xlabel("time since GPS cutoff (s)", fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    ax1.set_ylabel("XY error (m)", fontsize=12)
    ax1.set_title("Post-GPS XY error vs Δt  (dotted = linear fit)", fontsize=12)
    ax2.set_ylabel("running XY RMSE (m)", fontsize=12)
    ax2.set_title("Post-GPS cumulative XY RMSE", fontsize=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[saved] {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def fmt(v, w=8, p=2, dash="    --"):
    if v is None or (isinstance(v, float) and (np.isnan(v) or not np.isfinite(v))):
        return dash
    return f"{v:{w}.{p}f}"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    truth = load_truth(TRUTH_CSV)
    t_split = load_gps_last_t(GPS_CSV)
    print(f"truth: {truth.shape[0]} samples; t_split = {t_split:.3f}")

    processed = []
    for slug, label, fname, style in RUNS:
        p = os.path.join(RESULT_DIR, fname)
        if not os.path.isfile(p):
            print(f"  [skip] missing {p}")
            continue
        result = process_run(slug, label, p, truth, t_split)
        if result is None:
            continue
        m, s, traj_kept, offset, was_truncated = result
        processed.append((slug, label, traj_kept, offset, was_truncated, m, s, style))
        print(f"  loaded {slug}: kept {m['n_kept']}/{m['n_traj']} samples; "
              f"offset={offset.round(2).tolist()}; "
              f"div={m['div_reason']}")

    # --- plots ---
    plot_xy_traj(processed, truth, t_split,
                 os.path.join(OUT_DIR, "01_xy_traj.png"))
    plot_z_curve(processed, truth, t_split,
                 os.path.join(OUT_DIR, "02_z_curve.png"))
    plot_err_vs_time(processed, t_split,
                     os.path.join(OUT_DIR, "03_xy_error_vs_time.png"), "xy")
    plot_err_vs_time(processed, t_split,
                     os.path.join(OUT_DIR, "04_z_error_vs_time.png"), "z")
    plot_post_drift(processed, t_split,
                    os.path.join(OUT_DIR, "05_post_gps_drift.png"))

    # --- metrics table ---
    table = []
    header = (f"{'run':32s}  {'during':>7s}  {'  ':>7s}  "
              f"{'post':>7s} {'post':>7s} {'post-XY':>8s} {'post-XY':>8s} "
              f"{'post-Z':>7s} {'post-Z':>7s} {'drift':>9s}  "
              f"{'t>50m':>7s} {'t>100m':>7s} {'t>200m':>7s}")
    header2 = (f"{'':32s}  {'XY rms':>7s}  {'Z rms':>7s}  "
               f"{'XY rms':>7s} {'Z rms':>7s} {'max':>8s} {'final':>8s} "
               f"{'max':>7s} {'final':>7s} {'(m/s)':>9s}  "
               f"{'(s)':>7s} {'(s)':>7s} {'(s)':>7s}")
    table.append(header)
    table.append(header2)
    table.append("-" * len(header))
    for slug, label, traj_kept, offset, was_trunc, m, s, style in processed:
        row = (f"{slug:32s}  "
               f"{fmt(m['during_xy_rmse'])}  "
               f"{fmt(m['during_z_rmse'])}  "
               f"{fmt(m['post_xy_rmse'])} "
               f"{fmt(m['post_z_rmse'])} "
               f"{fmt(m['post_xy_max'])} "
               f"{fmt(m['post_xy_final'])} "
               f"{fmt(m['post_z_max'])} "
               f"{fmt(m['post_z_final'])} "
               f"{fmt(m['drift_rate_xy_per_s'], w=9, p=3)}  "
               f"{fmt(m['t_xy_ge_50m'], w=7, p=1)} "
               f"{fmt(m['t_xy_ge_100m'], w=7, p=1)} "
               f"{fmt(m['t_xy_ge_200m'], w=7, p=1)}")
        table.append(row)

    txt = "\n".join(table)
    print("\n" + txt + "\n")
    with open(os.path.join(OUT_DIR, "metrics_table.txt"), "w") as f:
        f.write(txt + "\n")
    print(f"[saved] {os.path.join(OUT_DIR, 'metrics_table.txt')}")


if __name__ == "__main__":
    main()
