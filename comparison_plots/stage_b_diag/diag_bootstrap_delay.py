#!/usr/bin/env python3
"""
Stage A z_ground bootstrap-delay ablation analysis (fly4).

Evaluates each run on three windows:
  - first edge:   t_rel ∈ [48.2, 116.4] s
  - pre-descent:  t_rel ∈ [0, 432.5] s
  - descent:      t_rel ∈ [432.5, end]  (appendix)

For each run/window we report:
  - path length (first edge) + scale_ratio (first edge)
  - XY RMSE / Z RMSE
  - max XY error / final XY error
  - max Z error  / final Z error
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

ALIGN_WINDOW_SEC = 5.0
EDGE_T_REL_START = 48.2
EDGE_T_REL_END   = 116.4
PRE_DESCENT_END  = 432.5

RUNS = [
    ("R0_nogps",   "R0 no-GPS baseline",            "R0_nogps.txt",
        dict(color="tab:gray",    lw=1.4, ls="-")),
    ("R2f_stageA", "R2f Stage-A (delay=0)",          "R2f_gplane_sigma20_full.txt",
        dict(color="tab:blue",    lw=2.0, ls="-")),
    ("R6_d10",     "Stage-A delay=10s",              "R6_delay10s.txt",
        dict(color="tab:cyan",    lw=1.6, ls="-")),
    ("R6_d20",     "Stage-A delay=20s",              "R6_delay20s.txt",
        dict(color="tab:olive",   lw=1.6, ls="-")),
    ("R6_d30",     "Stage-A delay=30s",              "R6_delay30s.txt",
        dict(color="tab:orange",  lw=1.6, ls="-")),
    ("R6_d45",     "Stage-A delay=45s",              "R6_delay45s.txt",
        dict(color="tab:purple",  lw=1.6, ls="-")),
    ("R5b_stageB_v0", "R5b Stage-B v0 (delay=0)",     "R5b_extreme_K2.txt",
        dict(color="tab:green",   lw=1.6, ls="--")),
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
    if len(xy) < 2: return 0.0
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def window_metrics(t, vio_xyz, truth_at, t_lo, t_hi):
    in_w = (t >= t_lo) & (t <= t_hi)
    if not in_w.any(): return None
    err = vio_xyz[in_w] - truth_at[in_w]
    valid = ~np.any(np.isnan(err), axis=1)
    if not valid.any(): return None
    e = err[valid]
    e_xy = np.sqrt(e[:, 0] ** 2 + e[:, 1] ** 2)
    e_z = np.abs(e[:, 2])
    return {
        "n": int(valid.sum()),
        "xy_rmse": float(np.sqrt(np.mean(e_xy ** 2))),
        "z_rmse":  float(np.sqrt(np.mean(e_z ** 2))),
        "xy_max":  float(e_xy.max()),
        "z_max":   float(e_z.max()),
        "xy_final": float(e_xy[-1]),
        "z_final":  float(e_z[-1]),
    }


def main():
    truth = load_truth(TRUTH_CSV)
    t0 = truth[0, 0]

    # truth first-edge path length
    mask_truth_edge = (truth[:, 0] >= t0 + EDGE_T_REL_START) & (truth[:, 0] <= t0 + EDGE_T_REL_END)
    truth_edge_xy = truth[mask_truth_edge, 1:3]
    truth_edge_len = path_length(truth_edge_xy)

    rows = []
    series = []
    for slug, label, fname, style in RUNS:
        traj = load_traj(os.path.join(RESULT_DIR, fname))
        if traj.shape[0] == 0:
            print(f"  [skip] {slug}: empty"); continue
        offset = translation_offset(traj, truth, ALIGN_WINDOW_SEC)
        aligned = traj[:, 1:4] + offset
        truth_at = interp_truth(truth, traj[:, 0])

        # First-edge metrics
        in_edge = (traj[:, 0] >= t0 + EDGE_T_REL_START) & (traj[:, 0] <= t0 + EDGE_T_REL_END)
        edge_xy = aligned[in_edge, :2]
        est_len = path_length(edge_xy)
        scale_ratio = est_len / truth_edge_len if truth_edge_len > 0 else float("nan")

        m_edge  = window_metrics(traj[:, 0], aligned, truth_at,
                                  t0 + EDGE_T_REL_START, t0 + EDGE_T_REL_END)
        m_pre   = window_metrics(traj[:, 0], aligned, truth_at,
                                  t0, t0 + PRE_DESCENT_END)
        m_post  = window_metrics(traj[:, 0], aligned, truth_at,
                                  t0 + PRE_DESCENT_END, traj[-1, 0])

        rows.append({
            "slug": slug, "label": label,
            "scale_ratio": scale_ratio,
            "edge_len": est_len,
            "edge_xy_rmse":  m_edge["xy_rmse"]  if m_edge else float("nan"),
            "edge_xy_final": m_edge["xy_final"] if m_edge else float("nan"),
            "pre_xy_rmse":   m_pre["xy_rmse"]   if m_pre  else float("nan"),
            "pre_z_rmse":    m_pre["z_rmse"]    if m_pre  else float("nan"),
            "pre_xy_max":    m_pre["xy_max"]    if m_pre  else float("nan"),
            "pre_xy_final":  m_pre["xy_final"]  if m_pre  else float("nan"),
            "post_xy_rmse":  m_post["xy_rmse"]  if m_post else float("nan"),
            "post_z_rmse":   m_post["z_rmse"]   if m_post else float("nan"),
        })
        series.append((slug, label, style, traj[:, 0], aligned))

    # ---- print table ----
    hdr = (f"{'run':22s} {'edge len':>9s} {'ratio':>7s} {'edge rmse':>10s} {'edge final':>11s}  "
           f"{'pre XY':>8s} {'pre Z':>7s} {'pre max':>8s} {'pre final':>10s}  "
           f"{'post XY':>9s} {'post Z':>8s}")
    line = "-" * len(hdr)
    print()
    print("Stage A bootstrap-delay ablation (fly4, truth-aligned)\n")
    print(f"Truth first-edge length = {truth_edge_len:.2f} m, window [{EDGE_T_REL_START:.1f}, {EDGE_T_REL_END:.1f}] s")
    print(f"Pre-descent window = [0, {PRE_DESCENT_END:.1f}] s; post-descent window = ({PRE_DESCENT_END:.1f}, end] s\n")
    print(hdr); print(line)
    out_txt = [hdr, line]
    for r in rows:
        ln = (f"{r['slug']:22s} {r['edge_len']:9.2f} {r['scale_ratio']:7.3f} "
              f"{r['edge_xy_rmse']:10.2f} {r['edge_xy_final']:11.2f}  "
              f"{r['pre_xy_rmse']:8.2f} {r['pre_z_rmse']:7.2f} {r['pre_xy_max']:8.2f} {r['pre_xy_final']:10.2f}  "
              f"{r['post_xy_rmse']:9.2f} {r['post_z_rmse']:8.2f}")
        print(ln); out_txt.append(ln)
    with open(os.path.join(OUT_DIR, "metrics_bootstrap_delay.txt"), "w") as f:
        f.write("\n".join(out_txt) + "\n")
    print(f"\n[saved] {os.path.join(OUT_DIR, 'metrics_bootstrap_delay.txt')}")

    # ---- plot full XY traj ----
    fig, ax = plt.subplots(1, 1, figsize=(11, 10))
    mask_truth_pre = truth[:, 0] <= t0 + PRE_DESCENT_END
    ax.plot(truth[mask_truth_pre, 1], truth[mask_truth_pre, 2],
            color="black", lw=1.4, ls=":", label="truth (pre-descent)")
    for slug, label, style, t, aligned in series:
        in_pre = (t >= t0) & (t <= t0 + PRE_DESCENT_END)
        ax.plot(aligned[in_pre, 0], aligned[in_pre, 1], label=label, **style)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_title("fly4 XY trajectory (pre-descent window) — Stage A bootstrap-delay ablation", fontsize=12)
    ax.legend(loc="best", fontsize=9)
    p = os.path.join(OUT_DIR, "11_bootstrap_delay_xy_traj.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # ---- first-edge zoom ----
    fig, ax = plt.subplots(1, 1, figsize=(10, 9))
    ax.plot(truth_edge_xy[:, 0], truth_edge_xy[:, 1],
            color="black", lw=1.6, ls=":",
            label=f"truth ({truth_edge_len:.1f} m, ratio 1.000)")
    for r, (slug, label, style, t, aligned) in zip(rows, series):
        in_edge = (t >= t0 + EDGE_T_REL_START) & (t <= t0 + EDGE_T_REL_END)
        xy = aligned[in_edge, :2]
        ax.plot(xy[:, 0], xy[:, 1],
                label=f"{label}  (len={r['edge_len']:.1f}, ratio={r['scale_ratio']:.3f})",
                **style)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"fly4 first-edge zoom (t_rel ∈ [{EDGE_T_REL_START:.1f}, {EDGE_T_REL_END:.1f}] s)",
                 fontsize=12)
    ax.legend(loc="best", fontsize=8)
    p = os.path.join(OUT_DIR, "12_bootstrap_delay_first_edge.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # ---- e_xy(t) ----
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    for slug, label, style, t, aligned in series:
        truth_at = interp_truth(truth, t)
        e_xy = np.sqrt((aligned[:, 0] - truth_at[:, 0]) ** 2 +
                       (aligned[:, 1] - truth_at[:, 1]) ** 2)
        ax.plot(t - t0, e_xy, label=label, **style)
    ax.axvline(EDGE_T_REL_END, color="black", lw=0.6, ls=":", alpha=0.6)
    ax.axvline(EDGE_T_REL_START, color="black", lw=0.6, ls=":", alpha=0.6)
    ax.axvline(PRE_DESCENT_END, color="red", lw=1.2, ls="-", alpha=0.7, label="descent start (eval_end)")
    ax.set_xlim(0, PRE_DESCENT_END + 10)
    ax.set_xlabel("time since first truth sample (s)")
    ax.set_ylabel(r"$e_{xy}(t)$  (m)")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_title("fly4 XY error vs time — bootstrap-delay ablation (pre-descent)", fontsize=12)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    p = os.path.join(OUT_DIR, "13_bootstrap_delay_xy_err.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")


if __name__ == "__main__":
    main()
