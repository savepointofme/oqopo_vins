#!/usr/bin/env python3
"""
Unified windowed evaluator for the frozen baselines (BASELINES.md).

For each flight, loads any set of trajectories (B0, B1, plus optional refs
like R5b/R7) and the truth CSV, then:

  - prints the exact timestamps required by BASELINES.md
  - auto-detects descent_start from truth z(t)
  - computes metrics on the pre-descent window [eval_start, descent_start]
  - computes a descent-appendix table on (descent_start, traj_end]
  - when two runs have different lengths, computes on the common overlap
    and labels the comparison "common-window"
  - saves XY traj, XY error, Z plots per flight

Usage:
    python tools/eval_baselines.py --flight 1 \
        --runs B0=20260509_fly1/result/baselines_v1/B0_no_gps.txt \
               B1=20260509_fly1/result/baselines_v1/B1_gps_height_d10.txt \
        --out comparison_plots/baselines_v1/fly1

The truth CSV is auto-located at
    20260509_flyN/result/gps_tum_time_alignment/truth_asl_cam_time.csv
"""

from __future__ import annotations
import argparse
import json
import math
import os
import sys
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except Exception:
    _HAVE_MPL = False


# ---------- loaders ---------------------------------------------------------

def load_traj(path):
    rows, last_t = [], -math.inf
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split()
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0]); x = float(parts[1])
                y = float(parts[2]); z = float(parts[3])
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
                rows.append((ts, float(parts[1]),
                             float(parts[2]), float(parts[3])))
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


# ---------- descent auto-detect --------------------------------------------

def detect_descent_start(truth):
    """Return descent_start (absolute), or None if no clean descent.

    Rule (mirrors BASELINES.md):
      window = [10%, 90%] of duration
      z_max  = max(z) in that window
      z_end  = z at last sample
      thresh = z_max - 0.5 * (z_max - z_end)
      descent_start = first t after argmax(z) where z<thresh and stays below
                      for the rest of the trajectory.
    """
    if len(truth) < 50:
        return None
    t = truth[:, 0]; z = truth[:, 3]
    dur = t[-1] - t[0]
    if dur <= 0:
        return None
    lo = t[0] + 0.10 * dur
    hi = t[0] + 0.90 * dur
    mask = (t >= lo) & (t <= hi)
    if mask.sum() < 5:
        return None
    z_max = float(z[mask].max())
    z_end = float(z[-1])
    if z_max - z_end < 1.0:           # no real descent (<1 m drop)
        return None
    thresh = z_max - 0.5 * (z_max - z_end)
    i_peak = int(np.argmax(z * mask.astype(float)))
    # walk forward from peak; require z<thresh and "stays below" until end
    for i in range(i_peak + 1, len(z)):
        if z[i] >= thresh:
            continue
        # candidate: every sample from i onward stays below thresh?
        if np.all(z[i:] < thresh):
            return float(t[i])
    return None


# ---------- metrics ---------------------------------------------------------

def translation_offset(vio, truth, window_sec=5.0):
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


def window_metrics(t, vio_xyz_aligned, truth_at, t_lo, t_hi):
    in_w = (t >= t_lo) & (t <= t_hi)
    if not in_w.any():
        return None
    err = vio_xyz_aligned[in_w] - truth_at[in_w]
    valid = ~np.any(np.isnan(err), axis=1)
    if not valid.any():
        return None
    e = err[valid]
    e_xy = np.sqrt(e[:, 0] ** 2 + e[:, 1] ** 2)
    e_z = np.abs(e[:, 2])
    return dict(
        n=int(valid.sum()),
        xy_rmse=float(np.sqrt(np.mean(e_xy ** 2))),
        z_rmse=float(np.sqrt(np.mean(e_z ** 2))),
        xy_max=float(e_xy.max()), z_max=float(e_z.max()),
        xy_final=float(e_xy[-1]), z_final=float(e_z[-1]),
    )


# ---------- main ------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight", type=int, required=True)
    ap.add_argument("--runs", nargs="+", required=True,
                    help="label=path entries (e.g. B0=..../B0.txt)")
    ap.add_argument("--truth", default=None,
                    help="override truth CSV path")
    ap.add_argument("--out", default=None,
                    help="output directory; default comparison_plots/baselines_v1/flyN")
    ap.add_argument("--first-edge", nargs=2, type=float, default=None,
                    metavar=("T_LO", "T_HI"),
                    help="optional first-edge window (relative seconds)")
    ap.add_argument("--align-seconds", type=float, default=5.0)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fly = args.flight
    truth_csv = args.truth or os.path.join(
        root, f"20260509_fly{fly}/result/gps_tum_time_alignment/"
        f"truth_asl_cam_time.csv")
    out_dir = args.out or os.path.join(
        root, f"comparison_plots/baselines_v1/fly{fly}")
    os.makedirs(out_dir, exist_ok=True)

    truth = load_truth(truth_csv)
    if len(truth) < 10:
        print(f"[error] truth too short or missing: {truth_csv}",
              file=sys.stderr)
        sys.exit(2)
    t0 = truth[0, 0]
    t_truth_end = truth[-1, 0]
    descent_abs = detect_descent_start(truth)
    descent_rel = descent_abs - t0 if descent_abs is not None else None

    runs = []
    for entry in args.runs:
        if "=" not in entry:
            print(f"[error] bad --runs entry '{entry}', want label=path",
                  file=sys.stderr); sys.exit(2)
        label, path = entry.split("=", 1)
        if not os.path.isfile(path):
            print(f"[warn] missing run {label}: {path}"); continue
        traj = load_traj(path)
        if len(traj) < 10:
            print(f"[warn] {label}: only {len(traj)} samples"); continue
        runs.append((label, path, traj))

    if not runs:
        print("[error] no runs loaded", file=sys.stderr); sys.exit(2)

    # ---- timestamp report ----
    print(f"\n===== fly{fly} =====")
    print(f"truth_csv          = {truth_csv}")
    print(f"t_truth_start_abs  = {t0:.6f}")
    print(f"t_truth_end_abs    = {t_truth_end:.6f}  (rel {t_truth_end - t0:.2f}s)")
    if descent_abs is not None:
        print(f"descent_start_abs  = {descent_abs:.6f}  "
              f"(rel {descent_rel:.2f}s)  [auto-detected]")
    else:
        print(f"descent_start      = NOT DETECTED  (no clean descent in truth)")
    print()
    per_run = []
    common_lo = -math.inf
    common_hi = math.inf
    for label, path, traj in runs:
        t_traj_start = traj[0, 0]
        t_traj_end = traj[-1, 0]
        print(f"run {label}")
        print(f"  path           = {path}")
        print(f"  t_traj_start   = {t_traj_start:.6f}  (rel {t_traj_start - t0:.2f}s)")
        print(f"  t_traj_end     = {t_traj_end:.6f}  (rel {t_traj_end - t0:.2f}s)")
        common_lo = max(common_lo, t_traj_start)
        common_hi = min(common_hi, t_traj_end)
        per_run.append(dict(label=label, path=path, traj=traj,
                            t_traj_start=t_traj_start, t_traj_end=t_traj_end))
    print()
    print(f"common_overlap     = [{common_lo:.6f}, {common_hi:.6f}]  "
          f"(rel [{common_lo - t0:.2f}, {common_hi - t0:.2f}]s)")

    # ---- eval window ----
    eval_start_abs = common_lo
    eval_end_abs = descent_abs if descent_abs is not None else common_hi
    eval_end_abs = min(eval_end_abs, common_hi)
    print(f"eval_window        = [{eval_start_abs:.6f}, {eval_end_abs:.6f}]  "
          f"(rel [{eval_start_abs - t0:.2f}, {eval_end_abs - t0:.2f}]s)")
    if descent_abs is None:
        print("  (no descent detected -> eval_end = common_overlap end)")
    else:
        print("  (eval_end = descent_start; descent segment is appendix only)")
    print()

    # ---- align + metrics ----
    metrics_lines = []
    if args.first_edge is not None:
        edge_lo, edge_hi = args.first_edge
        mask_t_edge = ((truth[:, 0] >= t0 + edge_lo) &
                       (truth[:, 0] <= t0 + edge_hi))
        edge_truth_xy = truth[mask_t_edge, 1:3]
        edge_truth_len = path_length(edge_truth_xy)
    else:
        edge_lo = edge_hi = None
        edge_truth_xy = np.zeros((0, 2))
        edge_truth_len = 0.0

    series = []
    rows = []
    for r in per_run:
        traj = r["traj"]
        offset = translation_offset(traj, truth, args.align_seconds)
        aligned = traj[:, 1:4] + offset
        truth_at = interp_truth(truth, traj[:, 0])

        # pre-descent (main window)
        m_pre = window_metrics(traj[:, 0], aligned, truth_at,
                               eval_start_abs, eval_end_abs)
        # descent appendix
        if descent_abs is not None and r["t_traj_end"] > descent_abs:
            m_post = window_metrics(traj[:, 0], aligned, truth_at,
                                    descent_abs, r["t_traj_end"])
        else:
            m_post = None
        # first-edge
        if args.first_edge is not None:
            in_edge = ((traj[:, 0] >= t0 + edge_lo) &
                       (traj[:, 0] <= t0 + edge_hi))
            edge_xy = aligned[in_edge, :2]
            est_len = path_length(edge_xy)
            ratio = est_len / edge_truth_len if edge_truth_len > 0 else float("nan")
            m_edge = window_metrics(traj[:, 0], aligned, truth_at,
                                    t0 + edge_lo, t0 + edge_hi)
        else:
            est_len = float("nan"); ratio = float("nan"); m_edge = None

        row = dict(
            label=r["label"],
            t_traj_start_rel=r["t_traj_start"] - t0,
            t_traj_end_rel=r["t_traj_end"] - t0,
            edge_len=est_len, ratio=ratio,
            edge_xy_rmse=(m_edge["xy_rmse"] if m_edge else float("nan")),
            edge_xy_final=(m_edge["xy_final"] if m_edge else float("nan")),
            pre_xy_rmse=(m_pre["xy_rmse"] if m_pre else float("nan")),
            pre_z_rmse=(m_pre["z_rmse"] if m_pre else float("nan")),
            pre_xy_max=(m_pre["xy_max"] if m_pre else float("nan")),
            pre_xy_final=(m_pre["xy_final"] if m_pre else float("nan")),
            post_xy_rmse=(m_post["xy_rmse"] if m_post else float("nan")),
            post_z_rmse=(m_post["z_rmse"] if m_post else float("nan")),
        )
        rows.append(row)
        series.append((r["label"], traj[:, 0], aligned))

    # ---- table ----
    hdr = (f"{'run':22s} {'t_start':>8s} {'t_end':>8s} "
           f"{'edge_len':>9s} {'ratio':>6s} {'edge_rmse':>10s} "
           f"{'pre_XY':>8s} {'pre_Z':>7s} {'pre_max':>8s} {'pre_final':>10s}  "
           f"{'post_XY':>9s} {'post_Z':>8s}")
    sep = "-" * len(hdr)
    print(hdr); print(sep)
    metrics_lines.extend([hdr, sep])
    for r in rows:
        ln = (f"{r['label']:22s} {r['t_traj_start_rel']:8.2f} "
              f"{r['t_traj_end_rel']:8.2f} {r['edge_len']:9.2f} "
              f"{r['ratio']:6.3f} {r['edge_xy_rmse']:10.2f} "
              f"{r['pre_xy_rmse']:8.2f} {r['pre_z_rmse']:7.2f} "
              f"{r['pre_xy_max']:8.2f} {r['pre_xy_final']:10.2f}  "
              f"{r['post_xy_rmse']:9.2f} {r['post_z_rmse']:8.2f}")
        print(ln); metrics_lines.append(ln)

    # ---- write metrics file ----
    meta = dict(
        flight=fly, truth_csv=truth_csv, t_truth_start=t0,
        t_truth_end=t_truth_end,
        descent_start_abs=descent_abs,
        descent_start_rel=descent_rel,
        eval_window_rel=(eval_start_abs - t0, eval_end_abs - t0),
        common_overlap_rel=(common_lo - t0, common_hi - t0),
        first_edge_rel=tuple(args.first_edge) if args.first_edge else None,
    )
    with open(os.path.join(out_dir, "metrics.txt"), "w") as f:
        for k, v in meta.items(): f.write(f"# {k} = {v}\n")
        f.write("\n".join(metrics_lines) + "\n")
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump({"meta": meta, "rows": rows}, f, indent=2, default=str)

    # ---- plots ----
    if not _HAVE_MPL:
        print("[note] matplotlib unavailable; skipping plots")
        return

    pre_lo, pre_hi = eval_start_abs, eval_end_abs

    # XY trajectory (pre-descent)
    fig, ax = plt.subplots(1, 1, figsize=(10, 9))
    mt = (truth[:, 0] >= pre_lo) & (truth[:, 0] <= pre_hi)
    ax.plot(truth[mt, 1], truth[mt, 2], color="black", lw=1.4,
            ls=":", label="truth")
    for label, t, aligned in series:
        in_pre = (t >= pre_lo) & (t <= pre_hi)
        ax.plot(aligned[in_pre, 0], aligned[in_pre, 1], lw=1.5, label=label)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=9)
    ax.set_title(
        f"fly{fly} XY trajectory (pre-descent)\n"
        f"eval [{pre_lo - t0:.1f}, {pre_hi - t0:.1f}]s"
        + ("" if descent_abs is None
           else f"  |  descent_start={descent_rel:.1f}s rel"))
    p = os.path.join(out_dir, "01_xy_traj_pre_descent.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f"[saved] {p}")

    # XY error vs time
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    for label, t, aligned in series:
        truth_at = interp_truth(truth, t)
        e_xy = np.sqrt((aligned[:, 0] - truth_at[:, 0]) ** 2 +
                       (aligned[:, 1] - truth_at[:, 1]) ** 2)
        ax.plot(t - t0, e_xy, lw=1.4, label=label)
    ax.axvline(pre_lo - t0, color="black", lw=0.6, ls=":", alpha=0.6)
    if descent_abs is not None:
        ax.axvline(descent_rel, color="red", lw=1.2, ls="-", alpha=0.7,
                   label=f"descent_start={descent_rel:.1f}s")
    ax.set_xlabel("t_rel (s)"); ax.set_ylabel(r"$e_{xy}(t)$ (m)")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.set_title(f"fly{fly} XY error vs time")
    ax.legend(loc="best", fontsize=9); ax.grid(True, alpha=0.3, which="both")
    p = os.path.join(out_dir, "02_xy_err_vs_time.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f"[saved] {p}")

    # Z vs time
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    ax.plot(truth[:, 0] - t0, truth[:, 3], color="black",
            lw=1.3, ls=":", label="truth")
    for label, t, aligned in series:
        ax.plot(t - t0, aligned[:, 2], lw=1.4, label=label)
    if descent_abs is not None:
        ax.axvline(descent_rel, color="red", lw=1.2, ls="-", alpha=0.7,
                   label=f"descent_start={descent_rel:.1f}s")
    ax.set_xlabel("t_rel (s)"); ax.set_ylabel("z (m)")
    ax.set_title(f"fly{fly} altitude vs time")
    ax.legend(loc="best", fontsize=9); ax.grid(True, alpha=0.3)
    p = os.path.join(out_dir, "03_z_vs_time.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f"[saved] {p}")


if __name__ == "__main__":
    main()
