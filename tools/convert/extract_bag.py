#!/usr/bin/env python3
"""
Extract a ROS1 / ROS2 bag into EuRoC-style mav0/ layout for OpenVINS.

Usage (basic):
    python3 extract_bag.py <bag.bag> <out_dir>
        # auto-detects IMU / image / GPS topics

Usage (override):
    python3 extract_bag.py <bag.bag> <out_dir> \
        --imu /jc82_imu/data_raw \
        --cam0 /fisheye/bright/image_raw/compressed \
        --cam1 /fisheye/right/image_raw/compressed \
        --gps  /jc82_gps/fix      # optional (NavSatFix); skip if no GPS

Layout produced:
    <out_dir>/mav0/
       imu0/data.csv          (#timestamp [ns], wx, wy, wz, ax, ay, az)
       cam0/data/<ns>.png
       cam0/data.csv          (#timestamp [ns], filename)
       cam1/data/<ns>.png     (only if --cam1 / detected)
       cam1/data.csv
       gps0/data.csv          (ts_ns, lat, lon, alt)  -- only if GPS

Requirements:
    pip install rosbags opencv-python numpy
"""
import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


def stamp_ns(hdr):
    """Support both ROS1 Header.stamp.{secs,nsecs} and ROS2 Header.stamp.{sec,nanosec}."""
    stamp = hdr.stamp
    sec = getattr(stamp, "sec", None)
    if sec is None:
        sec = getattr(stamp, "secs", None)
    nsec = getattr(stamp, "nanosec", None)
    if nsec is None:
        nsec = getattr(stamp, "nsecs", None)
    if sec is None or nsec is None:
        raise AttributeError("header.stamp does not have sec/secs and nanosec/nsecs fields")
    return int(sec) * 1_000_000_000 + int(nsec)


def list_topics(bag_path: Path) -> Dict[str, tuple]:
    """Print every topic + msgtype + count, then return mapping topic->(msgtype, msgcount)."""
    info = {}
    with AnyReader([bag_path]) as reader:
        print(f"\n=== topics in {bag_path.name} ===")
        for c in reader.connections:
            msgcount = getattr(c, "msgcount", None)
            info[c.topic] = (c.msgtype, msgcount)
            print(f"  {c.topic:50s} {c.msgtype:35s}  msgs={msgcount}")
    return info


def auto_pick(
    info: Dict,
    *,
    name: str,
    kinds: Sequence[str],
    prefer_keywords: Sequence[str] = (),
) -> Optional[str]:
    """Pick first topic whose msgtype matches any of `kinds`.

    Prefer those whose topic contains any keyword in prefer_keywords.
    Returns None if no match.
    """
    candidates = [t for t, (m, _) in info.items() if any(k in m for k in kinds)]
    for kw in prefer_keywords:
        for t in candidates:
            if kw in t.lower():
                return t
    return candidates[0] if candidates else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--imu", type=str, default=None)
    ap.add_argument("--cam0", type=str, default=None)
    ap.add_argument("--cam1", type=str, default=None, help="empty / 'none' to skip stereo")
    ap.add_argument("--gps", type=str, default=None, help="empty / 'none' to skip GPS")
    ap.add_argument("--list-only", action="store_true", help="just print topics and exit")
    args = ap.parse_args()

    info = list_topics(args.bag)
    if args.list_only:
        return

    # Auto-detect missing
    imu = args.imu or auto_pick(
        info,
        name="imu",
        kinds=["sensor_msgs/Imu", "sensor_msgs/msg/Imu"],
        prefer_keywords=["jc82", "data_raw", "imu"],
    )
    cam0 = args.cam0 or auto_pick(
        info,
        name="cam0",
        kinds=[
            "sensor_msgs/Image",
            "sensor_msgs/msg/Image",
            "sensor_msgs/CompressedImage",
            "sensor_msgs/msg/CompressedImage",
        ],
        prefer_keywords=["bright", "left", "cam0"],
    )
    cam1 = args.cam1
    if cam1 is None:
        cam1 = auto_pick(
            {t: v for t, v in info.items() if t != cam0},
            name="cam1",
            kinds=[
                "sensor_msgs/Image",
                "sensor_msgs/msg/Image",
                "sensor_msgs/CompressedImage",
                "sensor_msgs/msg/CompressedImage",
            ],
            prefer_keywords=["right", "cam1"],
        )
    gps = args.gps
    if gps is None:
        gps = auto_pick(
            info,
            name="gps",
            kinds=["sensor_msgs/NavSatFix", "sensor_msgs/msg/NavSatFix"],
            prefer_keywords=["gps", "fix", "navsat"],
        )

    # treat 'none' / '' as disabled
    if cam1 in ("", "none", "None"):
        cam1 = None
    if gps in ("", "none", "None"):
        gps = None

    print("\n=== picked topics ===")
    print(f"  IMU :  {imu}")
    print(f"  cam0:  {cam0}")
    print(f"  cam1:  {cam1 or '(skip - mono only)'}")
    print(f"  GPS :  {gps or '(skip - no GPS)'}")
    if not imu or not cam0:
        print("\nERROR: need at least IMU + cam0. Pass --imu / --cam0 explicitly.")
        sys.exit(2)

    out_root = args.out_dir / "mav0"
    (out_root / "cam0" / "data").mkdir(parents=True, exist_ok=True)
    (out_root / "imu0").mkdir(parents=True, exist_ok=True)
    if cam1:
        (out_root / "cam1" / "data").mkdir(parents=True, exist_ok=True)
    if gps:
        (out_root / "gps0").mkdir(parents=True, exist_ok=True)

    imu_rows = []
    cam0_rows = []
    cam1_rows = []
    gps_rows = []

    pick = {imu, cam0}
    if cam1:
        pick.add(cam1)
    if gps:
        pick.add(gps)

    with AnyReader([args.bag]) as reader:
        conns = [c for c in reader.connections if c.topic in pick]
        n_imu = n_c0 = n_c1 = n_g = 0
        for conn, ts_bag, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = stamp_ns(msg.header) if hasattr(msg, "header") else int(ts_bag)

            if conn.topic == imu:
                w = msg.angular_velocity
                a = msg.linear_acceleration
                imu_rows.append((t, w.x, w.y, w.z, a.x, a.y, a.z))
                n_imu += 1
            elif conn.topic == cam0 or conn.topic == cam1:
                # decode (handles both Image and CompressedImage)
                if "Compressed" in conn.msgtype:
                    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                    im = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
                else:
                    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                    h, w_ = int(msg.height), int(msg.width)
                    if "mono8" in msg.encoding:
                        im = arr.reshape(h, w_)
                    elif "bgr8" in msg.encoding or "rgb8" in msg.encoding:
                        im = arr.reshape(h, w_, 3)
                        im = cv2.cvtColor(
                            im,
                            cv2.COLOR_BGR2GRAY if "bgr8" in msg.encoding else cv2.COLOR_RGB2GRAY,
                        )
                    else:
                        print(f"WARN unhandled encoding {msg.encoding}, skipping frame")
                        continue
                if im is None:
                    continue
                target = out_root / ("cam0" if conn.topic == cam0 else "cam1") / "data" / f"{t}.png"
                cv2.imwrite(str(target), im)
                if conn.topic == cam0:
                    cam0_rows.append((t, f"{t}.png"))
                    n_c0 += 1
                else:
                    cam1_rows.append((t, f"{t}.png"))
                    n_c1 += 1
            elif conn.topic == gps:
                # NavSatFix
                gps_rows.append((t, float(msg.latitude), float(msg.longitude), float(msg.altitude)))
                n_g += 1

            if (n_imu + n_c0 + n_c1 + n_g) % 5000 == 0:
                print(f"  imu={n_imu} c0={n_c0} c1={n_c1} gps={n_g}")

    # sort + write
    imu_rows.sort()
    cam0_rows.sort()
    cam1_rows.sort()
    gps_rows.sort()

    with open(out_root / "imu0" / "data.csv", "w", newline="") as f:
        wcsv = csv.writer(f)
        wcsv.writerow([
            "#timestamp [ns]",
            "w_RS_S_x [rad s^-1]",
            "w_RS_S_y [rad s^-1]",
            "w_RS_S_z [rad s^-1]",
            "a_RS_S_x [m s^-2]",
            "a_RS_S_y [m s^-2]",
            "a_RS_S_z [m s^-2]",
        ])
        wcsv.writerows(imu_rows)

    for name, rows in [("cam0", cam0_rows)] + ([("cam1", cam1_rows)] if cam1 else []):
        with open(out_root / name / "data.csv", "w", newline="") as f:
            wcsv = csv.writer(f)
            wcsv.writerow(["#timestamp [ns]", "filename"])
            wcsv.writerows(rows)

    if gps:
        with open(out_root / "gps0" / "data.csv", "w", newline="") as f:
            wcsv = csv.writer(f)
            wcsv.writerow(["ts_ns", "lat", "lon", "alt"])
            wcsv.writerows(gps_rows)

    print(f"\nDone. imu={n_imu} cam0={n_c0} cam1={n_c1} gps={n_g}")
    if imu_rows:
        print(f"  imu  span: {(imu_rows[-1][0] - imu_rows[0][0]) / 1e9:.2f}s")
    if cam0_rows:
        span = (cam0_rows[-1][0] - cam0_rows[0][0]) / 1e9
        fps = n_c0 / span if span > 0 else 0.0
        print(f"  cam0 span: {span:.2f}s, fps≈{fps:.1f}")
    print(f"  output:    {out_root}")


if __name__ == "__main__":
    main()
