#!/usr/bin/env python3
"""
Unified windowed evaluator for the frozen baselines (BASELINES.md).

Reference-mode discipline (post 2026-05-17 data cleanup):
  This evaluator NO LONGER silently uses `truth_asl_cam_time.csv`. That file
  is derived from a stereo VIO trajectory and is NOT real ground truth. You
  MUST pass `--reference-mode` explicitly:

    --reference-mode stereo_pseudo_ref
        Use `truth_asl_cam_time.csv` (or `--truth` override). All XY/Z
        metrics will be labeled vs `stereo_pseudo_ref` in plots and JSON,
        and a warning banner is printed. Do not cite these numbers as
        official ground-truth RMSE. Stable-segment comparison only.

    --reference-mode gps_z
        Z-only evaluation against an aligned GPS altitude CSV (pass via
        `--gps-alt-csv`). XY metrics are skipped because GPS horizontal is
        not high-precision ground truth.

For each flight, loads any set of trajectories (B0, B1, plus optional refs
like R5b/R7) and the chosen reference, then:

  - prints the exact timestamps required by BASELINES.md
  - auto-detects descent_start from the chosen reference's z(t)
  - computes metrics on the pre-descent window [eval_start, descent_start]
  - computes a descent-appendix table on (descent_start, traj_end]
  - when two runs have different lengths, computes on the common overlap
    and labels the comparison "common-window"
  - saves XY traj, XY error, Z plots per flight (with the reference clearly
    labeled stereo_pseudo_ref or gps_alt)

Usage examples:
    # Stereo pseudo-ref (XY/Z, debug only — NOT ground truth):
    python tools/eval_baselines.py --flight 1 --reference-mode stereo_pseudo_ref \
        --runs B0=.../B0_no_gps.txt B1=.../B1_gps_height_d10.txt \
        --out comparison_plots/baselines_v1/fly1_stereoref

    # GPS-altitude-only (official Z metric):
    python tools/eval_baselines.py --flight 1 --reference-mode gps_z \
        --gps-alt-csv 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv \
        --runs B0=.../B0_no_gps.txt B1=.../B1_gps_height_d10.txt \
        --out comparison_plots/baselines_v1/fly1_gpsz
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
    """Load stereo_pseudo_ref (truth_asl_*) — 4 cols (ts_ns, x, y, z)."""
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


def load_gps_alt(path):
    """Load aligned GPS altitude CSV `#timestamp_ns,lat,lon,alt_m`.

    Returns array of shape (N, 4) with columns (t_s, NaN, NaN, alt_m). The
    NaN x/y placeholders keep the downstream array shape identical to the
    stereo_pseudo_ref load, but XY metrics will be NaN and must be skipped.
    """
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.replace(",", " ").split()
            try:
                t = float(parts[0])
                alt = float(parts[3])
                rows.append((t, alt))
            except (ValueError, IndexError):
                continue
    arr = np.asarray(rows)
    if arr.size and arr[0, 0] > 1e11:
        arr[:, 0] = arr[:, 0] * 1e-9  # ns -> s
    out = np.full((len(arr), 4), np.nan)
    out[:, 0] = arr[:, 0]
    out[:, 3] = arr[:, 1]
    return out[np.argsort(out[:, 0])]


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
    """Per-axis metrics — XY and Z are computed independently so that a
    reference with valid Z but NaN XY (gps_z mode) still yields Z metrics."""
    in_w = (t >= t_lo) & (t <= t_hi)
    if not in_w.any():
        return None
    err = vio_xyz_aligned[in_w] - truth_at[in_w]
    # XY block valid when both x and y are finite
    valid_xy = (~np.isnan(err[:, 0])) & (~np.isnan(err[:, 1]))
    valid_z = ~np.isnan(err[:, 2])
    if not (valid_xy.any() or valid_z.any()):
        return None
    out = dict(n=int(in_w.sum()))
    if valid_xy.any():
        e_xy = np.sqrt(err[valid_xy, 0] ** 2 + err[valid_xy, 1] ** 2)
        out.update(
            xy_rmse=float(np.sqrt(np.mean(e_xy ** 2))),
            xy_max=float(e_xy.max()),
            xy_final=float(e_xy[-1]),
        )
    else:
        out.update(xy_rmse=float("nan"), xy_max=float("nan"),
                   xy_final=float("nan"))
    if valid_z.any():
        e_z = np.abs(err[valid_z, 2])
        out.update(
            z_rmse=float(np.sqrt(np.mean(e_z ** 2))),
            z_max=float(e_z.max()),
            z_final=float(e_z[-1]),
        )
    else:
        out.update(z_rmse=float("nan"), z_max=float("nan"),
                   z_final=float("nan"))
    return out


# ---------- main ------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight", type=int, required=True)
    ap.add_argument("--reference-mode",
                    choices=["stereo_pseudo_ref", "gps_z"],
                    required=True,
                    help="Which reference to compare against. "
                         "'stereo_pseudo_ref' uses truth_asl_* (stereo VIO; "
                         "NOT ground truth, debug only). 'gps_z' compares Z "
                         "only against aligned GPS altitude.")
    ap.add_argument("--runs", nargs="+", required=True,
                    help="label=path entries (e.g. B0=..../B0.txt)")
    ap.add_argument("--truth", default=None,
                    help="(stereo_pseudo_ref mode) override stereo_pseudo_ref "
                         "CSV path")
    ap.add_argument("--gps-alt-csv", default=None,
                    help="(gps_z mode) aligned GPS altitude CSV")
    ap.add_argument("--out", default=None,
                    help="output directory; default comparison_plots/baselines_v1/flyN_<mode>")
    ap.add_argument("--first-edge", nargs=2, type=float, default=None,
                    metavar=("T_LO", "T_HI"),
                    help="optional first-edge window (relative seconds)")
    ap.add_argument("--align-seconds", type=float, default=5.0)
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fly = args.flight
    mode = args.reference_mode

    # Resolve the reference file based on the explicit mode.
    if mode == "stereo_pseudo_ref":
        ref_csv = args.truth or os.path.join(
            root, f"20260509_fly{fly}/result/gps_tum_time_alignment/"
            f"truth_asl_cam_time.csv")
        ref_label = "stereo_pseudo_ref"
        print("=" * 72)
        print(" WARNING: reference-mode=stereo_pseudo_ref")
        print("   The 'reference' below is a STEREO VIO trajectory estimate,")
        print("   NOT real ground truth. Use these XY/Z metrics ONLY as a")
        print("   debug / pseudo-reference comparison on stable segments.")
        print("   Do not cite as 'truth RMSE' in official results.")
        print("=" * 72)
        truth = load_truth(ref_csv)
    elif mode == "gps_z":
        if not args.gps_alt_csv:
            print("[error] --reference-mode gps_z requires --gps-alt-csv",
                  file=sys.stderr)
            sys.exit(2)
        ref_csv = args.gps_alt_csv
        ref_label = "gps_alt"
        print("=" * 72)
        print(" reference-mode=gps_z")
        print("   Z-only evaluation against aligned GPS altitude.")
        print("   XY metrics are SKIPPED (GPS horizontal is not GT-quality).")
        print("=" * 72)
        truth = load_gps_alt(ref_csv)
    else:
        print(f"[error] unknown reference-mode {mode}", file=sys.stderr)
        sys.exit(2)

    if args.out is None:
        out_dir = os.path.join(
            root, f"comparison_plots/baselines_v1/fly{fly}_{mode}")
    else:
        out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)

    if len(truth) < 10:
        print(f"[error] reference too short or missing: {ref_csv}",
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
    print(f"\n===== fly{fly} (reference={ref_label}) =====")
    print(f"reference_csv      = {ref_csv}")
    print(f"t_ref_start_abs    = {t0:.6f}")
    print(f"t_ref_end_abs      = {t_truth_end:.6f}  (rel {t_truth_end - t0:.2f}s)")
    if descent_abs is not None:
        src = "GPS-Z" if mode == "gps_z" else "stereo_pseudo_ref Z"
        print(f"descent_start_abs  = {descent_abs:.6f}  "
              f"(rel {descent_rel:.2f}s)  [auto-detected from {src}]")
        if mode == "stereo_pseudo_ref":
            print("  (descent detected from stereo VIO Z — debug only;")
            print("   do not use as official descent_start)")
    else:
        print(f"descent_start      = NOT DETECTED  (no clean descent in reference)")
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
    xy_valid = (mode == "stereo_pseudo_ref")  # only stereo ref has XY data
    if args.first_edge is not None:
        edge_lo, edge_hi = args.first_edge
        mask_t_edge = ((truth[:, 0] >= t0 + edge_lo) &
                       (truth[:, 0] <= t0 + edge_hi))
        if xy_valid:
            edge_truth_xy = truth[mask_t_edge, 1:3]
            edge_truth_len = path_length(edge_truth_xy)
        else:
            edge_truth_xy = np.zeros((0, 2))
            edge_truth_len = 0.0
    else:
        edge_lo = edge_hi = None
        edge_truth_xy = np.zeros((0, 2))
        edge_truth_len = 0.0
    if args.first_edge is not None and not xy_valid:
        print("[note] --first-edge ignored: gps_z mode has no XY reference")

    series = []
    rows = []
    for r in per_run:
        traj = r["traj"]
        if xy_valid:
            offset = translation_offset(traj, truth, args.align_seconds)
        else:
            # gps_z: align only Z. XY of the VIO is reported as-is (no shift).
            t0_v = traj[0, 0]
            mask = traj[:, 0] - t0_v < args.align_seconds
            sub = traj[mask] if mask.sum() >= 5 else traj[:5]
            z_ref = np.interp(sub[:, 0], truth[:, 0], truth[:, 3],
                              left=np.nan, right=np.nan)
            dz = z_ref - sub[:, 3]
            valid = ~np.isnan(dz)
            offset = np.zeros(3)
            offset[2] = float(dz[valid].mean()) if valid.any() else 0.0
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
        # first-edge (only meaningful when xy_valid)
        if args.first_edge is not None and xy_valid:
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
    if xy_valid:
        xy_note = f"XY/Z vs {ref_label} (XY is debug-only — not GT)"
    else:
        xy_note = f"Z only vs {ref_label}; XY columns are NaN (no XY reference)"
    print(f"[reference] {xy_note}")
    metrics_lines.append(f"# reference: {xy_note}")
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
        flight=fly,
        reference_mode=mode,
        reference_label=ref_label,
        reference_csv=ref_csv,
        xy_metrics_are_official=False,
        xy_metrics_note=(
            "XY metrics are vs stereo_pseudo_ref (NOT ground truth) — debug only"
            if mode == "stereo_pseudo_ref"
            else "XY metrics not computed; gps_z mode is Z-only"),
        t_ref_start=t0,
        t_ref_end=t_truth_end,
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
    ref_legend = ref_label  # "stereo_pseudo_ref" or "gps_alt"
    title_suffix = f"  |  ref={ref_label}"
    if mode == "stereo_pseudo_ref":
        title_suffix += "  (NOT ground truth — debug only)"

    if xy_valid:
        # XY trajectory (pre-descent)
        fig, ax = plt.subplots(1, 1, figsize=(10, 9))
        mt = (truth[:, 0] >= pre_lo) & (truth[:, 0] <= pre_hi)
        ax.plot(truth[mt, 1], truth[mt, 2], color="black", lw=1.4,
                ls=":", label=ref_legend)
        for label, t, aligned in series:
            in_pre = (t >= pre_lo) & (t <= pre_hi)
            ax.plot(aligned[in_pre, 0], aligned[in_pre, 1], lw=1.5, label=label)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=9)
        ax.set_title(
            f"fly{fly} XY trajectory (pre-descent){title_suffix}\n"
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
        ax.set_xlabel("t_rel (s)")
        ax.set_ylabel(rf"$e_{{xy}}(t)$ vs {ref_legend} (m)")
        ax.set_yscale("symlog", linthresh=1.0)
        ax.set_title(f"fly{fly} XY error vs time{title_suffix}")
        ax.legend(loc="best", fontsize=9); ax.grid(True, alpha=0.3, which="both")
        p = os.path.join(out_dir, "02_xy_err_vs_time.png")
        fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
        print(f"[saved] {p}")

    # Z vs time (always emitted — Z is meaningful in both modes)
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    ax.plot(truth[:, 0] - t0, truth[:, 3], color="black",
            lw=1.3, ls=":", label=ref_legend)
    for label, t, aligned in series:
        ax.plot(t - t0, aligned[:, 2], lw=1.4, label=label)
    if descent_abs is not None:
        ax.axvline(descent_rel, color="red", lw=1.2, ls="-", alpha=0.7,
                   label=f"descent_start={descent_rel:.1f}s")
    ax.set_xlabel("t_rel (s)"); ax.set_ylabel("z (m)")
    ax.set_title(f"fly{fly} altitude vs time{title_suffix}")
    ax.legend(loc="best", fontsize=9); ax.grid(True, alpha=0.3)
    p = os.path.join(out_dir, "03_z_vs_time.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f"[saved] {p}")

    # Z error vs time — the official Z metric in gps_z mode
    fig, ax = plt.subplots(1, 1, figsize=(13, 6))
    for label, t, aligned in series:
        z_ref = np.interp(t, truth[:, 0], truth[:, 3],
                          left=np.nan, right=np.nan)
        e_z = np.abs(aligned[:, 2] - z_ref)
        ax.plot(t - t0, e_z, lw=1.4, label=label)
    ax.axvline(pre_lo - t0, color="black", lw=0.6, ls=":", alpha=0.6)
    if descent_abs is not None:
        ax.axvline(descent_rel, color="red", lw=1.2, ls="-", alpha=0.7,
                   label=f"descent_start={descent_rel:.1f}s")
    ax.set_xlabel("t_rel (s)")
    ax.set_ylabel(rf"$|e_z|$ vs {ref_legend} (m)")
    ax.set_yscale("symlog", linthresh=0.5)
    ax.set_title(f"fly{fly} Z error vs time{title_suffix}")
    ax.legend(loc="best", fontsize=9); ax.grid(True, alpha=0.3, which="both")
    p = os.path.join(out_dir, "04_z_err_vs_time.png")
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f"[saved] {p}")


if __name__ == "__main__":
    main()
