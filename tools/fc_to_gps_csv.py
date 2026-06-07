#!/usr/bin/env python3
"""Convert the GSMQ FC CSV into a ros-free OpenVINS GPS comparison CSV.

Output format:
  ts_ns,lat,lon,alt

`ts_ns` is camera-relative dataset time in nanoseconds, matching the D455
`imu0/data.csv` / `cam0/data.csv` time base. This file is intended for
`run_serial_msckf_ros_free --gps` visualization/evaluation only. It does not
enable GPS fusion unless the runner is also given explicit GPS update flags.
"""

import argparse
import csv
import datetime as dt
import os


def parse_fc_time_utc(text):
    text = text.strip()
    if "_" in text:
        base, ms = text.rsplit("_", 1)
        micros = int(ms.ljust(3, "0")[:3]) * 1000
    else:
        base = text
        micros = 0
    stamp = dt.datetime.strptime(base, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(microsecond=micros, tzinfo=dt.timezone.utc).timestamp()


def iter_clean_csv_rows(path):
    with open(path, "rb") as f:
        for raw in f:
            raw = raw.replace(b"\x00", b"").strip()
            if not raw or raw.startswith(b"#"):
                continue
            yield raw.decode("utf-8-sig", "replace")


def camera_epoch_unix(dataset_dir):
    imu_path = os.path.join(dataset_dir, "imu0", "data.csv")
    for line in iter_clean_csv_rows(imu_path):
        row = next(csv.reader([line]))
        return float(row[1]) - float(row[0])
    raise RuntimeError("imu0/data.csv had no data rows")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fc", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--fc-rel-to-cam-offset",
        type=float,
        default=None,
        help="Use manual timing t_cam = (fc_time - first_fc_time) - OFFSET instead of absolute UTC.",
    )
    args = parser.parse_args()

    cam_epoch = None if args.fc_rel_to_cam_offset is not None else camera_epoch_unix(args.dataset)
    rows = []
    with open(args.fc, newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        next(reader, None)
        fc0_unix = None
        for raw in reader:
            if len(raw) < 8:
                continue
            try:
                unix = parse_fc_time_utc(raw[3])
                if fc0_unix is None:
                    fc0_unix = unix
                if args.fc_rel_to_cam_offset is None:
                    t_s = unix - cam_epoch
                else:
                    t_s = (unix - fc0_unix) - args.fc_rel_to_cam_offset
                lat = float(raw[5])
                lon = float(raw[6])
                alt = float(raw[7])
            except Exception:
                continue
            rows.append((int(round(t_s * 1e9)), lat, lon, alt))

    if not rows:
        raise RuntimeError("No GPS rows parsed from FC CSV")
    rows.sort(key=lambda r: r[0])

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["ts_ns", "lat", "lon", "alt"])
        writer.writerows(rows)

    print("wrote", args.output)
    print("rows", len(rows))
    if args.fc_rel_to_cam_offset is None:
        print("time mode absolute UTC")
    else:
        print("time mode relative offset %.6f" % args.fc_rel_to_cam_offset)
    print("time_s range %.3f %.3f" % (rows[0][0] * 1e-9, rows[-1][0] * 1e-9))
    print("first lat/lon/alt %.8f %.8f %.3f" % (rows[0][1], rows[0][2], rows[0][3]))
    print("last  lat/lon/alt %.8f %.8f %.3f" % (rows[-1][1], rows[-1][2], rows[-1][3]))


if __name__ == "__main__":
    main()
