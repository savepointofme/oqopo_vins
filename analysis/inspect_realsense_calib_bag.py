#!/usr/bin/env python3
"""Inspect RealSense ROS1 bags for Allan and Kalibr suitability."""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict

import numpy as np
import rosbag


TOPICS = {
    "image": "/device_0/sensor_1/Color_0/image/data",
    "camera_info": "/device_0/sensor_1/Color_0/info/camera_info",
    "accel": "/device_0/sensor_2/Accel_0/imu/data",
    "gyro": "/device_0/sensor_2/Gyro_0/imu/data",
}


def stamp_sec(msg, bag_time) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    value = stamp.to_sec() if stamp is not None else 0.0
    return value if value > 0.0 else bag_time.to_sec()


def summarize_times(values: list[float]) -> dict:
    a = np.asarray(values, dtype=float)
    if len(a) < 2:
        return {"count": int(len(a))}
    dt = np.diff(a)
    positive = dt[dt > 0]
    median = float(np.median(positive)) if len(positive) else math.nan
    return {
        "count": int(len(a)),
        "start_s": float(a[0]),
        "end_s": float(a[-1]),
        "duration_s": float(a[-1] - a[0]),
        "median_hz": float(1.0 / median) if median > 0 else None,
        "median_dt_s": median,
        "p99_dt_s": float(np.percentile(positive, 99)) if len(positive) else None,
        "max_dt_s": float(np.max(positive)) if len(positive) else None,
        "nonpositive_dt_count": int(np.sum(dt <= 0)),
        "gap_gt_2p5x_median_count": (
            int(np.sum(positive > 2.5 * median)) if median > 0 else None
        ),
    }


def inspect(path: str) -> dict:
    times = defaultdict(list)
    vectors = defaultdict(list)
    image = {}
    camera_info = {}
    with rosbag.Bag(path, "r") as bag:
        selected = list(TOPICS.values())
        for topic, msg, bag_time in bag.read_messages(topics=selected):
            key = next(name for name, value in TOPICS.items() if value == topic)
            times[key].append(stamp_sec(msg, bag_time))
            if key == "image" and not image:
                image = {
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": msg.encoding,
                    "step": int(msg.step),
                    "is_bigendian": int(msg.is_bigendian),
                    "frame_id": msg.header.frame_id,
                }
            elif key == "camera_info" and not camera_info:
                camera_info = {
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "distortion_model": msg.distortion_model,
                    "D": list(msg.D),
                    "K": list(msg.K),
                }
            elif key == "accel":
                vectors[key].append([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ])
            elif key == "gyro":
                vectors[key].append([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ])

    out = {"path": path, "streams": {}, "image": image, "camera_info": camera_info}
    for key, values in times.items():
        out["streams"][key] = summarize_times(values)
    for key, values in vectors.items():
        a = np.asarray(values, dtype=float)
        out["streams"][key]["axis_mean"] = np.mean(a, axis=0).tolist()
        out["streams"][key]["axis_std"] = np.std(a, axis=0).tolist()
        out["streams"][key]["norm_mean"] = float(np.mean(np.linalg.norm(a, axis=1)))
        out["streams"][key]["norm_std"] = float(np.std(np.linalg.norm(a, axis=1)))
    if times["image"] and times["gyro"]:
        out["cross_stream"] = {
            "image_minus_nearest_gyro_abs_median_s": float(np.median([
                np.min(np.abs(np.asarray(times["gyro"]) - t))
                for t in times["image"]
            ])),
            "image_minus_nearest_gyro_abs_max_s": float(np.max([
                np.min(np.abs(np.asarray(times["gyro"]) - t))
                for t in times["image"]
            ])),
        }
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--output")
    args = parser.parse_args()
    result = [inspect(path) for path in args.bags]
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as file:
            file.write(text + "\n")


if __name__ == "__main__":
    main()
