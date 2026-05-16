#!/usr/bin/env python3
"""Stage A v2 analysis: split during/post GPS for fly4 runs."""

import math
import os
import re
import sys

import numpy as np

RESULT_DIR = "/mnt/d/vscode_dir/open_vins/20260509_fly4/result/stage_a_v2"
GPS_CSV = "/mnt/d/vscode_dir/open_vins/20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv"

# WGS84 -> ECEF -> ENU using first GPS sample as origin (matches DatasetReaderEuroc)
A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2 - F)


def wgs84_to_ecef(lat_deg, lon_deg, alt):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    N = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
    return np.array([
        (N + alt) * math.cos(lat) * math.cos(lon),
        (N + alt) * math.cos(lat) * math.sin(lon),
        (N * (1 - E2) + alt) * math.sin(lat),
    ])


def ecef_to_enu_rot(lat_deg, lon_deg):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    return np.array([
        [-so, co, 0.0],
        [-sl * co, -sl * so, cl],
        [cl * co, cl * so, sl],
    ])


def load_gps(path):
    """Load GPS CSV (header + ts_ns,lat,lon,alt) into [(t_s, x, y, z)] ENU."""
    samples = []
    with open(path) as f:
        rows = [ln.strip() for ln in f if ln.strip() and not ln.startswith("#")]
    parsed = [list(map(float, ln.split(","))) for ln in rows]
    origin = wgs84_to_ecef(parsed[0][1], parsed[0][2], parsed[0][3])
    R = ecef_to_enu_rot(parsed[0][1], parsed[0][2])
    for r in parsed:
        ecef = wgs84_to_ecef(r[1], r[2], r[3])
        enu = R @ (ecef - origin)
        samples.append((r[0] * 1e-9, enu[0], enu[1], enu[2]))
    return samples


def load_traj(path):
    """Load TUM trajectory: list of (t, x, y, z). Strictly increasing time only."""
    out = []
    prev_t = -1.0
    with open(path) as f:
        for ln in f:
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split()
            if len(parts) < 4:
                continue
            try:
                t, x, y, z = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            if t <= prev_t:
                continue  # drop out-of-order entries
            out.append((t, x, y, z))
            prev_t = t
    return out


def parse_log(path):
    """Extract per-update GPLANE-RNG stats: list of dicts and a final summary dict."""
    if not os.path.isfile(path):
        return [], {}
    updates = []
    final = {}
    rx_status = re.compile(
        r"\[GPLANE-RNG\] status=(?P<dec>\w+)\s+t=(?P<t>[\d.]+).*?"
        r"rho_meas=(?P<rm>[-\d.]+)\s+rho_hat=(?P<rh>[-\d.]+)\s+res=(?P<res>[+-][\d.]+).*?"
        r"K_pz=(?P<kpz>[-\d.]+)\s+\|K_xy\|=(?P<kxy>[-\d.]+)\s+\|dxy\|_pred=(?P<dxy>[-\d.]+)"
    )
    rx_pz = re.compile(r"pz=(?P<a>[-\d.]+)->(?P<b>[-\d.]+)")
    rx_predxy = re.compile(r"")  # we already have |dxy|_pred
    rx_final_counts = re.compile(r"calls=(\d+)\s+acc=(\d+)\s+rej=(\d+)\s+skip=(\d+)")
    rx_final_kmu = re.compile(r"K_pz_mu=([-\d.]+)\s+\|K_xy\|_mu=([-\d.]+)\s+\|dxy\|_mu=([-\d.]+)")
    rx_final_meta = re.compile(r"\|res\|_mu=([-\d.]+)\s+\|dpz\|_mu=([-\d.]+)")
    rx_skip_tilt = re.compile(r"\[GPLANE-RNG\] skip: cos_tilt")
    n_skip_tilt = 0
    with open(path, errors="replace") as f:
        for ln in f:
            m = rx_status.search(ln)
            if m:
                d = m.groupdict()
                pm = rx_pz.search(ln)
                pz_before = float(pm.group("a")) if pm else 0.0
                pz_after = float(pm.group("b")) if pm else 0.0
                updates.append({
                    "t": float(d["t"]),
                    "decision": d["dec"],
                    "rm": float(d["rm"]),
                    "rh": float(d["rh"]),
                    "res": float(d["res"]),
                    "K_pz": float(d["kpz"]),
                    "K_xy": float(d["kxy"]),
                    "dxy_pred": float(d["dxy"]),
                    "pz_before": pz_before,
                    "pz_after": pz_after,
                })
                continue
            if rx_skip_tilt.search(ln):
                n_skip_tilt += 1
                continue
            m = rx_final_counts.search(ln)
            if m:
                final["calls"] = int(m.group(1))
                final["acc"] = int(m.group(2))
                final["rej"] = int(m.group(3))
                final["skip"] = int(m.group(4))
            m = rx_final_kmu.search(ln)
            if m:
                final["K_pz_mu"] = float(m.group(1))
                final["K_xy_mu"] = float(m.group(2))
                final["dxy_mu"] = float(m.group(3))
            m = rx_final_meta.search(ln)
            if m:
                final["res_mu"] = float(m.group(1))
                final["dpz_mu"] = float(m.group(2))
    final["skip_tilt_lines"] = n_skip_tilt
    return updates, final


def metrics(traj, gps, t_lo, t_hi, label):
    """Compute ATE, XY RMSE, Z RMSE, max |pz| for traj samples in [t_lo, t_hi]."""
    seg = [p for p in traj if t_lo <= p[0] <= t_hi]
    if not seg:
        return {"label": label, "n_traj": 0}
    # Nearest-time GPS match per traj sample (within ±0.2s)
    gps_t = np.array([g[0] for g in gps])
    gps_xyz = np.array([(g[1], g[2], g[3]) for g in gps])
    errs_xyz = []
    matched = 0
    for t, x, y, z in seg:
        idx = int(np.searchsorted(gps_t, t))
        cand = []
        if idx < len(gps_t):
            cand.append(idx)
        if idx > 0:
            cand.append(idx - 1)
        if not cand:
            continue
        best = min(cand, key=lambda i: abs(gps_t[i] - t))
        if abs(gps_t[best] - t) > 0.2:
            continue
        dx = x - gps_xyz[best, 0]
        dy = y - gps_xyz[best, 1]
        dz = z - gps_xyz[best, 2]
        errs_xyz.append((dx, dy, dz))
        matched += 1
    pz = np.array([abs(p[3]) for p in seg])
    if not errs_xyz:
        return {
            "label": label,
            "n_traj": len(seg),
            "matched": 0,
            "max_pz": float(pz.max()),
            "ATE": None,
            "XY_RMSE": None,
            "Z_RMSE": None,
        }
    e = np.array(errs_xyz)
    ate = float(np.sqrt((e ** 2).sum(axis=1).mean()))
    xy = float(np.sqrt((e[:, :2] ** 2).sum(axis=1).mean()))
    zr = float(np.sqrt((e[:, 2] ** 2).mean()))
    return {
        "label": label,
        "n_traj": len(seg),
        "matched": matched,
        "max_pz": float(pz.max()),
        "ATE": ate,
        "XY_RMSE": xy,
        "Z_RMSE": zr,
    }


def log_window_stats(updates, t_lo, t_hi):
    """Stats from log-derived per-update rows within [t_lo, t_hi]."""
    seg = [u for u in updates if t_lo <= u["t"] <= t_hi]
    if not seg:
        return {"n_updates": 0}
    kpz = np.array([u["K_pz"] for u in seg])
    kxy = np.array([u["K_xy"] for u in seg])
    ratio = np.where(np.abs(kpz) > 1e-9, kxy / np.abs(kpz), 0.0)
    cum_dxy = float(sum(u["dxy_pred"] for u in seg))
    cum_dpz = float(sum(u["pz_after"] - u["pz_before"] for u in seg))
    abs_dpz = float(sum(abs(u["pz_after"] - u["pz_before"]) for u in seg))
    return {
        "n_updates": len(seg),
        "K_pz_mu": float(kpz.mean()),
        "K_xy/K_pz_mu": float(ratio.mean()),
        "cum_|dxy|_pred": cum_dxy,
        "cum_dpz_signed": cum_dpz,
        "cum_|dpz|": abs_dpz,
    }


def report_one(run, traj_path, log_path, gps, t_split):
    traj = load_traj(traj_path)
    updates, final = parse_log(log_path)
    if not traj:
        print(f"### {run}: trajectory file empty or missing ({traj_path})")
        return
    t_start, t_end = traj[0][0], traj[-1][0]
    m_during = metrics(traj, gps, t_start, t_split, "during-GPS")
    m_post = metrics(traj, gps, t_split + 1e-6, t_end, "post-GPS")
    s_during = log_window_stats(updates, t_start, t_split)
    s_post = log_window_stats(updates, t_split + 1e-6, t_end)
    print(f"\n### {run}")
    print(f"  traj: {traj_path}")
    print(f"  log:  {log_path}")
    print(f"  trajectory t-range: [{t_start:.3f}, {t_end:.3f}] ({t_end-t_start:.1f}s, {len(traj)} samples)")
    print(f"  final summary: {final or '(no GPLANE-RNG-FINAL block found)'}")
    for tag, m, s in [("A. during-GPS", m_during, s_during),
                      ("B. post-GPS  ", m_post, s_post)]:
        print(f"  {tag}: traj_samples={m['n_traj']:5d}  gps_matched={m.get('matched',0):4d}  "
              f"max|pz|={m['max_pz']:.2f}m  "
              f"ATE={fmt(m.get('ATE'))}  XY_RMSE={fmt(m.get('XY_RMSE'))}  Z_RMSE={fmt(m.get('Z_RMSE'))}")
        if s.get("n_updates", 0) > 0:
            print(f"      log_updates={s['n_updates']}  K_pz_mu={s['K_pz_mu']:.4f}  "
                  f"K_xy/K_pz_mu={s['K_xy/K_pz_mu']:.4f}  "
                  f"cum_|dxy|_pred={s['cum_|dxy|_pred']:.2f}m  "
                  f"cum_dpz_signed={s['cum_dpz_signed']:+.2f}m  "
                  f"cum_|dpz|={s['cum_|dpz|']:.2f}m")
        else:
            print(f"      log_updates=0  (no GPS feeds in this window)")


def fmt(v):
    return "  None" if v is None else f"{v:7.3f}m"


def main():
    gps = load_gps(GPS_CSV)
    print(f"GPS samples loaded: {len(gps)}  (t: {gps[0][0]:.3f} -> {gps[-1][0]:.3f}, span {gps[-1][0]-gps[0][0]:.1f}s)")
    t_split = gps[-1][0]
    print(f"Split timestamp t_split (last GPS): {t_split:.3f}")

    runs = [
        ("R0_nogps                  ", "R0_nogps.txt", "R0_nogps.log"),
        ("R2f Stage-A best          ", "R2f_gplane_sigma20_full.txt", "R2f_gplane_sigma20_full.log"),
        ("R4a Stage-B sigma3 K5     ", "R4a_pre_msckf_default.txt", "R4a_pre_msckf_default.log"),
        ("R4b Stage-B sigma5 K5     ", "R4b_pre_msckf_loose.txt", "R4b_pre_msckf_loose.log"),
        ("R4c Stage-B sigma3 K15    ", "R4c_pre_msckf_K15.txt", "R4c_pre_msckf_K15.log"),
        ("R5a Stage-B sigma20 K3    ", "R5a_loose20_K3.txt", "R5a_loose20_K3.log"),
        ("R5b Stage-B sigma50 K2    ", "R5b_extreme_K2.txt", "R5b_extreme_K2.log"),
    ]
    for name, t, l in runs:
        report_one(name, os.path.join(RESULT_DIR, t), os.path.join(RESULT_DIR, l), gps, t_split)


if __name__ == "__main__":
    main()
