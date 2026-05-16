#!/usr/bin/env python3
"""
Re-evaluation with eval_end = final descent start (detected from truth z).

Detection rule:
  smooth truth z with a 5-sample window, then walk BACKWARD from the last
  sample.  The "final descent" is the trailing block of samples whose
  smoothed z monotonically descends to landing.  eval_end is set to the
  FIRST sample of that block — i.e. the latest sample at which the drone
  has NOT yet started the persistent descent that ends in landing.
"""

import os, math
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
ALIGN_WINDOW_SEC = 5.0

RUNS = [
    ("R0_nogps",   "R0 no-GPS baseline",            "R0_nogps.txt",
        dict(color="tab:gray",  lw=1.6, ls="-")),
    ("R2f_stageA", "R2f Stage-A best (σ=2.0, full)", "R2f_gplane_sigma20_full.txt",
        dict(color="tab:blue",  lw=2.0, ls="-")),
    ("R5b_stageB", "R5b Stage-B (σ_px=50, K=2)",     "R5b_extreme_K2.txt",
        dict(color="tab:green", lw=2.0, ls="-")),
]


# ---------------------------------------------------------------------------
def load_traj(path):
    rows, last_t = [], -math.inf
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"): continue
            parts = ln.split()
            if len(parts) < 4: continue
            try:
                t, x, y, z = (float(parts[i]) for i in range(4))
            except ValueError:
                continue
            if t <= last_t: continue
            rows.append((t, x, y, z))
            last_t = t
    return np.asarray(rows) if rows else np.zeros((0, 4))


def load_truth(path):
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"): continue
            parts = ln.split(",")
            try:
                ts = float(parts[0]) * 1e-9
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
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
            if not ln or ln.startswith("#"): continue
            parts = ln.split(",")
            try:
                last = float(parts[0]) * 1e-9
            except (ValueError, IndexError):
                pass
    return last


# ---------------------------------------------------------------------------
def find_final_descent_start(truth, alt_frac=0.9):
    """
    Detect the final descent start using a z-altitude threshold.

    Rule: eval_end is the LAST sample where truth z is still >= alt_frac * z_peak.
    After this sample, z monotonically descends below the threshold and never
    returns (final descent to landing).

    For fly4 with z_peak ≈ 88.5 m and alt_frac=0.9, this gives ~ 79.6 m as the
    cutoff and eval_end ≈ t_rel = 432 s (well before the GPS cutoff at 613.5 s
    and the landing at ~600 s).
    """
    z = truth[:, 3]
    z_peak = float(z.max())
    cutoff = alt_frac * z_peak
    mask = z >= cutoff
    if not mask.any():
        return float(truth[-1, 0])
    return float(truth[np.where(mask)[0][-1], 0])


def find_divergence(traj):
    if traj.shape[0] == 0: return 0, "empty"
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
    return n_good, ("ok" if n_good == traj.shape[0]
                    else ("|xyz|>%.0fm" % POS_LIMIT_M if n_good == cut_a
                          else "jump>%.0fm" % JUMP_LIMIT_M))


def interp_truth(truth, t_query):
    out = np.full((len(t_query), 3), np.nan)
    for axis in range(3):
        out[:, axis] = np.interp(t_query, truth[:, 0], truth[:, 1 + axis],
                                 left=np.nan, right=np.nan)
    return out


def translation_offset(vio, truth, window_sec):
    t0 = vio[0, 0]
    mask = vio[:, 0] - t0 < window_sec
    sub = vio[mask] if mask.sum() >= 5 else vio[:5]
    truth_at = interp_truth(truth, sub[:, 0])
    diff = truth_at - sub[:, 1:4]
    valid = ~np.any(np.isnan(diff), axis=1)
    return np.mean(diff[valid], axis=0) if valid.any() else np.zeros(3)


# ---------------------------------------------------------------------------
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    truth = load_truth(TRUTH_CSV)
    t_gps_end = load_gps_last_t(GPS_CSV)
    t_truth0 = truth[0, 0]
    t_descent = find_final_descent_start(truth)
    print(f"truth: {len(truth)} samples, t0 = {t_truth0:.3f}")
    print(f"GPS last sample t       = {t_gps_end:.3f}  ({t_gps_end - t_truth0:+.1f}s rel)")
    print(f"Final descent starts at t = {t_descent:.3f}  ({t_descent - t_truth0:+.1f}s rel)")
    print(f"Pre-descent eval window: [{t_truth0:.3f}, {t_descent:.3f}]  span = {t_descent - t_truth0:.1f}s")
    eval_end = t_descent

    rows = []
    series = []
    for slug, label, fname, style in RUNS:
        traj = load_traj(os.path.join(RESULT_DIR, fname))
        n_good, reason = find_divergence(traj)
        traj_kept = traj[:n_good]
        offset = translation_offset(traj_kept, truth, ALIGN_WINDOW_SEC)
        truth_at = interp_truth(truth, traj_kept[:, 0])
        vio_aligned = traj_kept[:, 1:4] + offset
        err = vio_aligned - truth_at
        e_xy = np.sqrt(err[:, 0] ** 2 + err[:, 1] ** 2)
        e_z = np.abs(err[:, 2])

        t = traj_kept[:, 0]
        valid_truth = ~np.isnan(e_xy)
        in_window = valid_truth & (t <= eval_end)

        if not in_window.any():
            print(f"  {slug}: no samples in window")
            continue

        e_xy_w = e_xy[in_window]
        e_z_w = e_z[in_window]

        rmse = lambda a: float(np.sqrt(np.mean(a ** 2)))
        rows.append({
            "slug": slug, "label": label,
            "n": int(in_window.sum()),
            "xy_rmse": rmse(e_xy_w),
            "z_rmse":  rmse(e_z_w),
            "xy_max":  float(e_xy_w.max()),
            "xy_final": float(e_xy_w[-1]),
            "z_max":   float(e_z_w.max()),
            "z_final": float(e_z_w[-1]),
            "div_reason": reason,
        })

        series.append((slug, label, style, t, vio_aligned, e_xy, e_z, in_window))

    # ---- print table ----
    print()
    hdr = f"{'run':14s} {'XY RMSE':>9s} {'Z RMSE':>8s} {'XY max':>8s} {'XY final':>9s} {'Z max':>7s} {'Z final':>8s}"
    print(hdr); print("-" * len(hdr))
    for r in rows:
        print(f"{r['slug']:14s} {r['xy_rmse']:9.3f} {r['z_rmse']:8.3f} "
              f"{r['xy_max']:8.2f} {r['xy_final']:9.2f} {r['z_max']:7.2f} {r['z_final']:8.2f}")

    txt = "Pre-descent eval window  [t = {:.3f} ⟶ {:.3f}]   span = {:.1f}s\n".format(
        truth[0, 0], eval_end, eval_end - truth[0, 0])
    txt += f"GPS cutoff at t = {t_gps_end:.3f}  ({t_gps_end - truth[0, 0]:+.1f}s rel).  "
    txt += f"Descent starts BEFORE GPS cutoff by {t_gps_end - eval_end:.1f}s.\n\n"
    txt += hdr + "\n" + "-" * len(hdr) + "\n"
    for r in rows:
        txt += (f"{r['slug']:14s} {r['xy_rmse']:9.3f} {r['z_rmse']:8.3f} "
                f"{r['xy_max']:8.2f} {r['xy_final']:9.2f} "
                f"{r['z_max']:7.2f} {r['z_final']:8.2f}\n")
    with open(os.path.join(OUT_DIR, "metrics_pre_descent.txt"), "w") as f:
        f.write(txt)
    print(f"\n[saved] {os.path.join(OUT_DIR, 'metrics_pre_descent.txt')}")

    # ---- plot XY traj ----
    fig, ax = plt.subplots(1, 1, figsize=(10, 9))
    mask_truth = truth[:, 0] <= eval_end
    ax.plot(truth[mask_truth, 1], truth[mask_truth, 2],
            color="black", lw=1.6, ls=":", label="truth (pre-descent)")
    ax.plot(truth[~mask_truth, 1], truth[~mask_truth, 2],
            color="black", lw=0.8, ls=":", alpha=0.3, label="truth (descent — excluded)")
    for slug, label, style, t, vio_aligned, e_xy, e_z, in_win in series:
        ax.plot(vio_aligned[in_win, 0], vio_aligned[in_win, 1],
                label=f"{label}", **style)
        # endpoint of pre-descent window
        last_i = np.where(in_win)[0][-1]
        ax.plot(vio_aligned[last_i, 0], vio_aligned[last_i, 1], "s",
                color=style["color"], markersize=10, mec="black", mew=1.0)
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title(f"fly4 XY trajectory in pre-descent window  [t ≤ {eval_end - truth[0,0]:.0f}s rel]\n"
                 "Square = pre-descent endpoint; descent/landing excluded",
                 fontsize=12)
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    p = os.path.join(OUT_DIR, "06_xy_traj_pre_descent.png")
    fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # ---- plot z curve ----
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    t0 = truth[0, 0]
    ax.plot(truth[:, 0] - t0, truth[:, 3], color="black", lw=1.3, ls=":", label="truth")
    for slug, label, style, t, vio_aligned, e_xy, e_z, in_win in series:
        ax.plot(t - t0, vio_aligned[:, 2], **style, label=label)
    ax.axvline(eval_end - t0, color="red", lw=1.2, ls="-", alpha=0.8,
               label=f"eval_end (descent start)  t={eval_end - t0:.0f}s")
    ax.axvline(t_gps_end - t0, color="purple", lw=0.8, ls="--", alpha=0.6,
               label=f"GPS cutoff  t={t_gps_end - t0:.0f}s")
    ax.set_xlabel("time since first truth sample (s)", fontsize=12)
    ax.set_ylabel("z (m)", fontsize=12)
    ax.set_title("fly4 altitude vs time  (eval_end = final descent start)", fontsize=12)
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    p = os.path.join(OUT_DIR, "07_z_curve_pre_descent.png")
    fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # ---- plot e_xy(t) ----
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    for slug, label, style, t, vio_aligned, e_xy, e_z, in_win in series:
        ax.plot(t - t0, e_xy, **style, label=label)
    ax.axvline(eval_end - t0, color="red", lw=1.2, ls="-", alpha=0.8,
               label=f"eval_end  t={eval_end - t0:.0f}s")
    ax.axvline(t_gps_end - t0, color="purple", lw=0.8, ls="--", alpha=0.6,
               label=f"GPS cutoff  t={t_gps_end - t0:.0f}s")
    for thresh in [50, 100, 200]:
        ax.axhline(thresh, color="gray", lw=0.6, ls=":", alpha=0.5)
    ax.set_xlabel("time since first truth sample (s)", fontsize=12)
    ax.set_ylabel(r"$e_{xy}(t)$  (m)", fontsize=12)
    ax.set_title("fly4 XY error vs time  (vertical red = pre-descent eval_end)", fontsize=12)
    ax.set_yscale("symlog", linthresh=1.0)
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    p = os.path.join(OUT_DIR, "08_xy_error_pre_descent.png")
    fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")


if __name__ == "__main__":
    main()
