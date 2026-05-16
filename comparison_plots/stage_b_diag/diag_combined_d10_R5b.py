#!/usr/bin/env python3
"""
Combined experiment analysis: Stage A delay=10s + R5b Stage B v0 (R7) vs the
reference set:
  - R0  no-GPS
  - R2f Stage A (delay=0)
  - R6_d10 Stage A (delay=10s, no Stage B)
  - R5b Stage B v0 (delay=0, sigma_px=50, K=2)
  - R7  Stage A delay=10s + Stage B v0 R5b config  (the new combined run)

Same windows as before:
  first edge   [48.2, 116.4] s
  pre-descent  [0, 432.5] s
  descent      (432.5, end] s   (appendix)
"""

import os, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = "/mnt/d/vscode_dir/open_vins" if os.path.isdir("/mnt/d") else "d:/vscode_dir/open_vins"
RESULT_DIR = os.path.join(ROOT, "20260509_fly4/result/stage_a_v2")
RESULT_PARENT = os.path.join(ROOT, "20260509_fly4/result")
OUT_DIR = os.path.join(ROOT, "comparison_plots/stage_b_diag")
TRUTH_CSV = os.path.join(ROOT, "20260509_fly4/result/gps_tum_time_alignment/truth_asl_cam_time.csv")

ALIGN_WINDOW_SEC = 5.0
EDGE_T_REL_START = 48.2
EDGE_T_REL_END   = 116.4
PRE_DESCENT_END  = 432.5

# (slug, label, path_relative_to_RESULT_DIR_or_absolute, style)
RUNS = [
    ("PR17_old_nogps",  "PR17 old no-GPS",                    os.path.join(RESULT_PARENT, "pr17_mono_tum.txt"),
        dict(color="tab:olive",  lw=1.4, ls="-")),
    ("R0_nogps",        "R0 no-GPS",                          "R0_nogps.txt",
        dict(color="tab:gray",   lw=1.4, ls="-")),
    ("R0_fresh_nogps",  "R0_fresh no-GPS (rerun)",            "R0_fresh_no_gps.txt",
        dict(color="black",      lw=1.0, ls=":")),
    ("R2f_stageA",      "R2f Stage-A (delay=0)",              "R2f_gplane_sigma20_full.txt",
        dict(color="tab:blue",   lw=1.6, ls="-")),
    ("R6_d10",          "R6 Stage-A delay=10s",               "R6_delay10s.txt",
        dict(color="tab:cyan",   lw=1.6, ls="-")),
    ("R5b_stageB_v0",   "R5b Stage-B v0 (delay=0)",            "R5b_extreme_K2.txt",
        dict(color="tab:green",  lw=1.6, ls="--")),
    ("R7_combined",     "R7 Stage-A delay=10s + Stage-B v0",   "R7_d10_plus_R5b.txt",
        dict(color="tab:red",    lw=2.2, ls="-")),
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
            except ValueError: continue
            if t <= last_t: continue
            rows.append((t, x, y, z)); last_t = t
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
                rows.append((ts, float(parts[1]), float(parts[2]), float(parts[3])))
            except (ValueError, IndexError): continue
    arr = np.asarray(rows); return arr[np.argsort(arr[:, 0])]


def interp_truth(truth, t_query):
    out = np.full((len(t_query), 3), np.nan)
    for axis in range(3):
        out[:, axis] = np.interp(t_query, truth[:, 0], truth[:, 1 + axis], left=np.nan, right=np.nan)
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
    e_xy = np.sqrt(e[:, 0] ** 2 + e[:, 1] ** 2); e_z = np.abs(e[:, 2])
    return {"n": int(valid.sum()),
            "xy_rmse": float(np.sqrt(np.mean(e_xy ** 2))),
            "z_rmse":  float(np.sqrt(np.mean(e_z ** 2))),
            "xy_max":  float(e_xy.max()), "z_max":   float(e_z.max()),
            "xy_final": float(e_xy[-1]),  "z_final":  float(e_z[-1])}


def main():
    truth = load_truth(TRUTH_CSV)
    t0 = truth[0, 0]
    mask_t_edge = (truth[:, 0] >= t0 + EDGE_T_REL_START) & (truth[:, 0] <= t0 + EDGE_T_REL_END)
    truth_edge_xy = truth[mask_t_edge, 1:3]
    truth_edge_len = path_length(truth_edge_xy)
    print(f"\nTruth first-edge length = {truth_edge_len:.2f} m, "
          f"window [{EDGE_T_REL_START:.1f}, {EDGE_T_REL_END:.1f}] s")
    print(f"Pre-descent window = [0, {PRE_DESCENT_END:.1f}] s\n")

    rows = []; series = []
    for slug, label, fname, style in RUNS:
        path = fname if os.path.isabs(fname) else os.path.join(RESULT_DIR, fname)
        if not os.path.isfile(path):
            print(f"  [skip] missing {path}"); continue
        traj = load_traj(path)
        if len(traj) < 10:
            print(f"  [skip] {slug}: only {len(traj)} samples in {path}"); continue
        offset = translation_offset(traj, truth, ALIGN_WINDOW_SEC)
        aligned = traj[:, 1:4] + offset
        truth_at = interp_truth(truth, traj[:, 0])

        in_edge = (traj[:, 0] >= t0 + EDGE_T_REL_START) & (traj[:, 0] <= t0 + EDGE_T_REL_END)
        edge_xy = aligned[in_edge, :2]
        est_len = path_length(edge_xy)
        ratio = est_len / truth_edge_len if truth_edge_len > 0 else float("nan")

        m_edge = window_metrics(traj[:, 0], aligned, truth_at, t0 + EDGE_T_REL_START, t0 + EDGE_T_REL_END)
        m_pre  = window_metrics(traj[:, 0], aligned, truth_at, t0, t0 + PRE_DESCENT_END)
        m_post = window_metrics(traj[:, 0], aligned, truth_at, t0 + PRE_DESCENT_END, traj[-1, 0])

        rows.append({
            "slug": slug, "label": label,
            "edge_len": est_len, "ratio": ratio,
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

    hdr = (f"{'run':22s} {'edge len':>9s} {'ratio':>7s} {'edge rmse':>10s} {'edge final':>11s}  "
           f"{'pre XY':>8s} {'pre Z':>7s} {'pre max':>8s} {'pre final':>10s}  "
           f"{'post XY':>9s} {'post Z':>8s}")
    line = "-" * len(hdr)
    print(hdr); print(line)
    out_lines = [hdr, line]
    for r in rows:
        ln = (f"{r['slug']:22s} {r['edge_len']:9.2f} {r['ratio']:7.3f} "
              f"{r['edge_xy_rmse']:10.2f} {r['edge_xy_final']:11.2f}  "
              f"{r['pre_xy_rmse']:8.2f} {r['pre_z_rmse']:7.2f} "
              f"{r['pre_xy_max']:8.2f} {r['pre_xy_final']:10.2f}  "
              f"{r['post_xy_rmse']:9.2f} {r['post_z_rmse']:8.2f}")
        print(ln); out_lines.append(ln)
    with open(os.path.join(OUT_DIR, "metrics_combined_d10_R5b.txt"), "w") as f:
        f.write("\n".join(out_lines) + "\n")

    # plots
    # 1) full XY (pre-descent)
    fig, ax = plt.subplots(1, 1, figsize=(10, 9))
    mask_truth_pre = truth[:, 0] <= t0 + PRE_DESCENT_END
    ax.plot(truth[mask_truth_pre, 1], truth[mask_truth_pre, 2],
            color="black", lw=1.4, ls=":", label="truth")
    for slug, label, style, t, aligned in series:
        in_pre = t <= t0 + PRE_DESCENT_END
        ax.plot(aligned[in_pre, 0], aligned[in_pre, 1], label=label, **style)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=9)
    ax.set_title("fly4 XY trajectory (pre-descent) — combined run R7", fontsize=12)
    p = os.path.join(OUT_DIR, "14_combined_xy_traj.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # 2) first-edge zoom
    fig, ax = plt.subplots(1, 1, figsize=(10, 9))
    ax.plot(truth_edge_xy[:, 0], truth_edge_xy[:, 1],
            color="black", lw=1.6, ls=":",
            label=f"truth ({truth_edge_len:.1f} m, ratio 1.000)")
    for r, (slug, label, style, t, aligned) in zip(rows, series):
        in_edge = (t >= t0 + EDGE_T_REL_START) & (t <= t0 + EDGE_T_REL_END)
        ax.plot(aligned[in_edge, 0], aligned[in_edge, 1],
                label=f"{label}  (len={r['edge_len']:.1f}, ratio={r['ratio']:.3f})",
                **style)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="datalim"); ax.grid(True, alpha=0.3)
    ax.set_title(f"fly4 first-edge zoom — combined run R7", fontsize=12)
    ax.legend(loc="best", fontsize=8)
    p = os.path.join(OUT_DIR, "15_combined_first_edge.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # 3) e_xy(t)
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    for slug, label, style, t, aligned in series:
        truth_at = interp_truth(truth, t)
        e_xy = np.sqrt((aligned[:, 0] - truth_at[:, 0]) ** 2 +
                       (aligned[:, 1] - truth_at[:, 1]) ** 2)
        ax.plot(t - t0, e_xy, label=label, **style)
    ax.axvline(EDGE_T_REL_START, color="black", lw=0.6, ls=":", alpha=0.6)
    ax.axvline(EDGE_T_REL_END,   color="black", lw=0.6, ls=":", alpha=0.6)
    ax.axvline(PRE_DESCENT_END,  color="red",   lw=1.2, ls="-", alpha=0.7,
               label="descent start (eval_end)")
    ax.set_xlim(0, PRE_DESCENT_END + 10)
    ax.set_xlabel("time since first truth sample (s)")
    ax.set_ylabel(r"$e_{xy}(t)$  (m)")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_title("fly4 XY error vs time — combined run R7", fontsize=12)
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    p = os.path.join(OUT_DIR, "16_combined_xy_err.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")

    # 4) z curve
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    ax.plot(truth[:, 0] - t0, truth[:, 3], color="black", lw=1.3, ls=":", label="truth")
    for slug, label, style, t, aligned in series:
        ax.plot(t - t0, aligned[:, 2], label=label, **style)
    ax.axvline(PRE_DESCENT_END, color="red", lw=1.2, ls="-", alpha=0.7, label="descent start")
    ax.set_xlabel("time since first truth sample (s)")
    ax.set_ylabel("z (m)")
    ax.set_title("fly4 altitude vs time — combined run R7", fontsize=12)
    ax.legend(loc="best", fontsize=9); ax.grid(True, alpha=0.3)
    p = os.path.join(OUT_DIR, "17_combined_z.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig); print(f"[saved] {p}")


if __name__ == "__main__":
    main()
