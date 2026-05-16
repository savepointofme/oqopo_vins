#!/usr/bin/env python3
"""
First-edge scale diagnostic for fly4.

First straight edge of truth: t_rel ∈ [48.2, 116.4] s.  ~400 m straight south
after takeoff.  For each run we measure:
  - path length over this window
  - scale_ratio = est_length / truth_length  (1.0 = ideal)
  - e_xy at edge end
  - peak e_xy across edge
  - e_xy(t) curve

Runs compared:
  R0 no-GPS, R2f Stage A best, R5b Stage B v0.
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

ALIGN_WINDOW_SEC = 5.0
EDGE_T_REL_START = 48.2
EDGE_T_REL_END   = 116.4

RUNS = [
    ("R0_nogps",   "R0 no-GPS baseline",            "R0_nogps.txt",
        dict(color="tab:gray",  lw=1.6, ls="-")),
    ("R2f_stageA", "R2f Stage-A best (σ=2.0, full)", "R2f_gplane_sigma20_full.txt",
        dict(color="tab:blue",  lw=2.0, ls="-")),
    ("R5b_stageB_v0", "R5b Stage-B v0 (σ_px=50, K=2)", "R5b_extreme_K2.txt",
        dict(color="tab:green", lw=2.0, ls="-")),
]


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


def path_length(xy):
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def main():
    truth = load_truth(TRUTH_CSV)
    t0 = truth[0, 0]
    t_e_start = t0 + EDGE_T_REL_START
    t_e_end   = t0 + EDGE_T_REL_END

    # truth XY in edge window
    mask_truth = (truth[:, 0] >= t_e_start) & (truth[:, 0] <= t_e_end)
    truth_edge = truth[mask_truth]
    truth_xy = truth_edge[:, 1:3]
    truth_len = path_length(truth_xy)
    truth_straight = float(np.linalg.norm(truth_xy[-1] - truth_xy[0]))
    print(f"Truth first edge: span {EDGE_T_REL_END - EDGE_T_REL_START:.1f}s, "
          f"path = {truth_len:.2f} m, straight-dist = {truth_straight:.2f} m, "
          f"straightness = {truth_straight / truth_len:.3f}")

    rows = []
    fig_traj, ax_traj = plt.subplots(1, 1, figsize=(11, 9))
    fig_err, ax_err = plt.subplots(1, 1, figsize=(13, 5.5))

    ax_traj.plot(truth_xy[:, 0], truth_xy[:, 1],
                 color="black", lw=1.6, ls=":", label=f"truth ({truth_len:.1f} m, ratio 1.000)")
    ax_traj.plot(truth_xy[0, 0],  truth_xy[0, 1],  "o", color="black", markersize=10, label="edge start")
    ax_traj.plot(truth_xy[-1, 0], truth_xy[-1, 1], "s", color="black", markersize=10, label="edge end")

    for slug, label, fname, style in RUNS:
        traj = load_traj(os.path.join(RESULT_DIR, fname))
        if traj.shape[0] == 0:
            continue
        offset = translation_offset(traj, truth, ALIGN_WINDOW_SEC)
        aligned_xyz = traj[:, 1:4] + offset
        mask_run = (traj[:, 0] >= t_e_start) & (traj[:, 0] <= t_e_end)
        if not mask_run.any():
            continue
        xy = aligned_xyz[mask_run, :2]
        t = traj[mask_run, 0]

        # truth-interpolated to this run's timestamps
        truth_at = interp_truth(truth, t)
        valid = ~np.isnan(truth_at[:, 0])
        e_xy = np.sqrt(np.sum((xy[valid] - truth_at[valid, :2]) ** 2, axis=1))

        est_len = path_length(xy)
        scale_ratio = est_len / truth_len if truth_len > 0 else float("nan")
        final_e = float(e_xy[-1]) if len(e_xy) else float("nan")
        peak_e = float(e_xy.max()) if len(e_xy) else float("nan")
        rmse_e = float(np.sqrt(np.mean(e_xy ** 2))) if len(e_xy) else float("nan")

        rows.append({
            "slug": slug, "label": label,
            "est_length": est_len, "scale_ratio": scale_ratio,
            "first_edge_xy_rmse": rmse_e,
            "first_edge_xy_max": peak_e,
            "first_edge_xy_final": final_e,
            "offset_xyz": offset.tolist(),
        })

        ax_traj.plot(xy[:, 0], xy[:, 1],
                     label=f"{label}  (len={est_len:.1f} m, ratio={scale_ratio:.3f})",
                     **style)
        ax_traj.plot(xy[0, 0], xy[0, 1], "o", color=style["color"], markersize=8, mec="black")
        ax_traj.plot(xy[-1, 0], xy[-1, 1], "s", color=style["color"], markersize=8, mec="black")

        ax_err.plot(t - t0, e_xy, label=label, **style)

    ax_traj.set_xlabel("X (m)", fontsize=12); ax_traj.set_ylabel("Y (m)", fontsize=12)
    ax_traj.set_aspect("equal", adjustable="datalim")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.set_title(
        f"fly4 first straight edge  (t_rel ∈ [{EDGE_T_REL_START:.1f}, {EDGE_T_REL_END:.1f}] s, ~400 m southbound)\n"
        f"circle = edge start, square = edge end; legend shows path length and scale_ratio",
        fontsize=11)
    ax_traj.legend(loc="best", fontsize=9)

    ax_err.axvline(EDGE_T_REL_START, color="black", lw=0.8, ls=":", alpha=0.6)
    ax_err.axvline(EDGE_T_REL_END,   color="black", lw=0.8, ls=":", alpha=0.6)
    ax_err.set_xlim(EDGE_T_REL_START - 5, EDGE_T_REL_END + 5)
    ax_err.set_xlabel("time since first truth sample (s)", fontsize=12)
    ax_err.set_ylabel(r"$e_{xy}(t)$  (m)", fontsize=12)
    ax_err.set_title("fly4 first-edge XY error vs time", fontsize=11)
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="best", fontsize=9)

    p1 = os.path.join(OUT_DIR, "09_first_edge_traj.png")
    p2 = os.path.join(OUT_DIR, "10_first_edge_xy_error.png")
    fig_traj.tight_layout(); fig_traj.savefig(p1, dpi=120); plt.close(fig_traj)
    fig_err.tight_layout();  fig_err.savefig(p2, dpi=120);  plt.close(fig_err)
    print(f"[saved] {p1}")
    print(f"[saved] {p2}")

    print()
    hdr = (f"{'run':16s} {'est len (m)':>11s} {'scale_ratio':>11s} "
           f"{'XY rmse':>9s} {'XY max':>8s} {'XY final':>9s}")
    print(hdr); print("-" * len(hdr))
    for r in rows:
        print(f"{r['slug']:16s} {r['est_length']:11.2f} {r['scale_ratio']:11.3f} "
              f"{r['first_edge_xy_rmse']:9.2f} {r['first_edge_xy_max']:8.2f} "
              f"{r['first_edge_xy_final']:9.2f}")

    txt = f"Truth first-edge: path_length = {truth_len:.2f} m, straightness = {truth_straight / truth_len:.3f}\n"
    txt += f"Window: t_rel ∈ [{EDGE_T_REL_START:.1f}, {EDGE_T_REL_END:.1f}] s  (span {EDGE_T_REL_END - EDGE_T_REL_START:.1f}s, ~400 m southbound)\n\n"
    txt += hdr + "\n" + "-" * len(hdr) + "\n"
    for r in rows:
        txt += (f"{r['slug']:16s} {r['est_length']:11.2f} {r['scale_ratio']:11.3f} "
                f"{r['first_edge_xy_rmse']:9.2f} {r['first_edge_xy_max']:8.2f} "
                f"{r['first_edge_xy_final']:9.2f}\n")
    with open(os.path.join(OUT_DIR, "metrics_first_edge.txt"), "w") as f:
        f.write(txt)
    print(f"\n[saved] {os.path.join(OUT_DIR, 'metrics_first_edge.txt')}")


if __name__ == "__main__":
    main()
