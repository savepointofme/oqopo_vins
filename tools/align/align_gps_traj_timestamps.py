#!/usr/bin/env python3
"""Align a GPS CSV and a TUM trajectory by timestamp.

GPS input:
  #timestamp_ns,lat_deg,lon_deg,alt_m

TUM input:
  t x y z qx qy qz qw

Output rows are GPS samples paired with the nearest TUM pose after applying an
optional constant timestamp offset to TUM time.
"""

import argparse
import bisect
import csv
from pathlib import Path


def read_gps(path: Path):
    rows = []
    with path.open(newline="") as f:
        for raw in f:
            raw = raw.strip()
            if not raw or raw.startswith("#"):
                continue
            parts = [p.strip() for p in raw.split(",")]
            if len(parts) < 4:
                continue
            ts_ns = int(parts[0])
            rows.append(
                {
                    "gps_t_ns": ts_ns,
                    "gps_t": ts_ns / 1e9,
                    "lat": float(parts[1]),
                    "lon": float(parts[2]),
                    "alt": float(parts[3]),
                }
            )
    return rows


def read_tum(path: Path):
    rows = []
    with path.open() as f:
        for raw in f:
            raw = raw.strip()
            if not raw or raw.startswith("#"):
                continue
            parts = raw.split()
            if len(parts) < 8:
                continue
            rows.append(
                {
                    "tum_t": float(parts[0]),
                    "x": float(parts[1]),
                    "y": float(parts[2]),
                    "z": float(parts[3]),
                    "qx": float(parts[4]),
                    "qy": float(parts[5]),
                    "qz": float(parts[6]),
                    "qw": float(parts[7]),
                }
            )
    return rows


def nearest_index(times, target):
    pos = bisect.bisect_left(times, target)
    if pos == 0:
        return 0
    if pos == len(times):
        return len(times) - 1
    before = pos - 1
    return before if abs(times[before] - target) <= abs(times[pos] - target) else pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gps", type=Path, required=True)
    ap.add_argument("--tum", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument(
        "--offset-mode",
        choices=["none", "start"],
        default="none",
        help="none: use absolute timestamps; start: shift TUM so first TUM time equals first GPS time",
    )
    ap.add_argument(
        "--time-offset",
        type=float,
        default=None,
        help="manual TUM offset in seconds: tum_time_aligned = tum_time + offset",
    )
    ap.add_argument("--max-dt", type=float, default=0.06, help="maximum nearest timestamp gap in seconds")
    args = ap.parse_args()

    gps = read_gps(args.gps)
    tum = read_tum(args.tum)
    if not gps:
        raise SystemExit(f"no GPS rows: {args.gps}")
    if not tum:
        raise SystemExit(f"no TUM rows: {args.tum}")

    if args.time_offset is not None:
        offset = args.time_offset
    elif args.offset_mode == "start":
        offset = gps[0]["gps_t"] - tum[0]["tum_t"]
    else:
        offset = 0.0

    tum_times = [r["tum_t"] + offset for r in tum]
    matched = []
    for g in gps:
        i = nearest_index(tum_times, g["gps_t"])
        tr = tum[i]
        tum_t_aligned = tum_times[i]
        dt = g["gps_t"] - tum_t_aligned
        if abs(dt) <= args.max_dt:
            matched.append((g, tr, tum_t_aligned, dt))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "gps_t_ns",
                "gps_t_sec",
                "tum_t_sec",
                "dt_gps_minus_tum_sec",
                "lat_deg",
                "lon_deg",
                "alt_m",
                "vio_x",
                "vio_y",
                "vio_z",
                "vio_qx",
                "vio_qy",
                "vio_qz",
                "vio_qw",
            ]
        )
        for g, tr, tum_t_aligned, dt in matched:
            w.writerow(
                [
                    g["gps_t_ns"],
                    f"{g['gps_t']:.9f}",
                    f"{tum_t_aligned:.9f}",
                    f"{dt:.9f}",
                    f"{g['lat']:.10f}",
                    f"{g['lon']:.10f}",
                    f"{g['alt']:.3f}",
                    f"{tr['x']:.9f}",
                    f"{tr['y']:.9f}",
                    f"{tr['z']:.9f}",
                    f"{tr['qx']:.9f}",
                    f"{tr['qy']:.9f}",
                    f"{tr['qz']:.9f}",
                    f"{tr['qw']:.9f}",
                ]
            )

    gps_span = gps[-1]["gps_t"] - gps[0]["gps_t"]
    tum_span = tum_times[-1] - tum_times[0]
    overlap = max(0.0, min(gps[-1]["gps_t"], tum_times[-1]) - max(gps[0]["gps_t"], tum_times[0]))
    print(f"gps rows: {len(gps)}, span: {gps_span:.3f}s")
    print(f"tum rows: {len(tum)}, span: {tum_span:.3f}s")
    print(f"applied tum offset: {offset:.9f}s")
    print(f"timestamp overlap: {overlap:.3f}s")
    print(f"matched rows: {len(matched)} with max_dt={args.max_dt:.3f}s")
    print(f"wrote: {args.out}")


if __name__ == "__main__":
    main()
