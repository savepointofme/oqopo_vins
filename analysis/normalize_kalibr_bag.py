#!/usr/bin/env python3
"""Normalize standard camera/IMU topics and trim IMU to camera coverage."""
from __future__ import annotations

import argparse
import copy
import json
import os

import rosbag
import rospy


def stamp_sec(msg, bag_time) -> float:
    value = msg.header.stamp.to_sec()
    return value if value > 0.0 else bag_time.to_sec()


def camera_bounds(path: str, image_topic: str) -> tuple[float, float]:
    first = None
    last = None
    with rosbag.Bag(path, "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[image_topic]):
            value = stamp_sec(msg, bag_time)
            first = value if first is None else first
            last = value
    if first is None or last is None:
        raise ValueError(f"no image messages on {image_topic}")
    return first, last


def normalize(input_path: str, output_path: str, image_topic: str,
              imu_topic: str | None, compression: str) -> dict:
    start, end = camera_bounds(input_path, image_topic)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    counts = {"images": 0, "imu": 0, "imu_trimmed": 0}
    topics = [image_topic] + ([imu_topic] if imu_topic else [])
    with rosbag.Bag(input_path, "r") as source, rosbag.Bag(
        output_path, "w", compression=compression, chunk_threshold=768 * 1024
    ) as target:
        for topic, msg, bag_time in source.read_messages(topics=topics):
            value = stamp_sec(msg, bag_time)
            if topic == image_topic:
                out = copy.deepcopy(msg)
                out.header.stamp = rospy.Time.from_sec(value)
                out.header.frame_id = "cam0"
                target.write("/cam0/image_raw", out, out.header.stamp)
                counts["images"] += 1
            elif imu_topic and topic == imu_topic:
                if value < start - 0.05 or value > end + 0.05:
                    counts["imu_trimmed"] += 1
                    continue
                out = copy.deepcopy(msg)
                out.header.stamp = rospy.Time.from_sec(value)
                out.header.frame_id = "imu0"
                target.write("/imu0", out, out.header.stamp)
                counts["imu"] += 1

    return {
        "input": input_path,
        "output": output_path,
        "compression": compression,
        "camera_start_s": start,
        "camera_end_s": end,
        "duration_s": end - start,
        "counts": counts,
        "output_size_bytes": os.path.getsize(output_path),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--image-topic", required=True)
    parser.add_argument("--imu-topic")
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    parser.add_argument("--summary", required=True)
    args = parser.parse_args()
    result = normalize(
        args.input, args.output, args.image_topic, args.imu_topic, args.compression
    )
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    with open(args.summary, "w", encoding="utf-8") as file:
        file.write(text + "\n")


if __name__ == "__main__":
    main()
