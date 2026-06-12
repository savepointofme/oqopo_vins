#!/usr/bin/env python3
"""Convert RealSense ROS1 bags into compact Allan/Kalibr input bags."""
from __future__ import annotations

import argparse
import copy
import json
import os
from collections import deque

import rosbag
import rospy
from sensor_msgs.msg import Imu


IMAGE_IN = "/device_0/sensor_1/Color_0/image/data"
ACCEL_IN = "/device_0/sensor_2/Accel_0/imu/data"
GYRO_IN = "/device_0/sensor_2/Gyro_0/imu/data"


def stamp_sec(msg, bag_time) -> float:
    value = msg.header.stamp.to_sec()
    return value if value > 0.0 else bag_time.to_sec()


def ros_time(value: float) -> rospy.Time:
    return rospy.Time.from_sec(value)


def interpolate_accel(left, right, timestamp: float):
    t0 = left[0]
    t1 = right[0]
    alpha = 0.0 if t1 <= t0 else min(1.0, max(0.0, (timestamp - t0) / (t1 - t0)))
    a0 = left[1].linear_acceleration
    a1 = right[1].linear_acceleration
    return (
        a0.x + alpha * (a1.x - a0.x),
        a0.y + alpha * (a1.y - a0.y),
        a0.z + alpha * (a1.z - a0.z),
    )


def make_imu(gyro_msg, accel_xyz, stamp: float, seq: int, accel_covariance) -> Imu:
    out = Imu()
    out.header.seq = seq
    out.header.stamp = ros_time(stamp)
    out.header.frame_id = "imu0"
    out.orientation_covariance[0] = -1.0
    out.angular_velocity = copy.deepcopy(gyro_msg.angular_velocity)
    out.angular_velocity_covariance = list(gyro_msg.angular_velocity_covariance)
    out.linear_acceleration.x = accel_xyz[0]
    out.linear_acceleration.y = accel_xyz[1]
    out.linear_acceleration.z = accel_xyz[2]
    out.linear_acceleration_covariance = list(accel_covariance)
    return out


def convert(input_path: str, output_path: str, mode: str, compression: str) -> dict:
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    selected = [IMAGE_IN] if mode == "camera" else [ACCEL_IN, GYRO_IN]
    if mode == "imucam":
        selected.append(IMAGE_IN)

    counts = {"images": 0, "imu": 0, "skipped_gyro_before_accel": 0}
    first_stamp = None
    last_stamp = None
    prev_accel = None
    pending_gyro = deque()
    seq = 0

    def record_stamp(value: float) -> None:
        nonlocal first_stamp, last_stamp
        first_stamp = value if first_stamp is None else min(first_stamp, value)
        last_stamp = value if last_stamp is None else max(last_stamp, value)

    with rosbag.Bag(input_path, "r") as source, rosbag.Bag(
        output_path, "w", compression=compression, chunk_threshold=768 * 1024
    ) as target:
        for topic, msg, bag_time in source.read_messages(topics=selected):
            timestamp = stamp_sec(msg, bag_time)
            if topic == IMAGE_IN:
                image = copy.deepcopy(msg)
                image.header.stamp = ros_time(timestamp)
                image.header.frame_id = "cam0"
                target.write("/cam0/image_raw", image, image.header.stamp)
                counts["images"] += 1
                record_stamp(timestamp)
                continue

            if topic == GYRO_IN:
                pending_gyro.append((timestamp, copy.deepcopy(msg)))
                continue

            current_accel = (timestamp, copy.deepcopy(msg))
            if prev_accel is None:
                while pending_gyro and pending_gyro[0][0] < timestamp:
                    pending_gyro.popleft()
                    counts["skipped_gyro_before_accel"] += 1
                prev_accel = current_accel
                continue

            # Keep the native gyro time axis (~200 Hz) and linearly interpolate
            # the native accelerometer stream (~100 Hz) onto those timestamps.
            while pending_gyro and pending_gyro[0][0] <= timestamp:
                gyro_stamp, gyro_msg = pending_gyro.popleft()
                accel_xyz = interpolate_accel(prev_accel, current_accel, gyro_stamp)
                imu = make_imu(
                    gyro_msg, accel_xyz, gyro_stamp, seq,
                    current_accel[1].linear_acceleration_covariance,
                )
                target.write("/imu0", imu, imu.header.stamp)
                counts["imu"] += 1
                seq += 1
                record_stamp(gyro_stamp)
            prev_accel = current_accel

        # Keep the final gyro samples with zero-order-held acceleration.
        if prev_accel is not None:
            accel = prev_accel[1].linear_acceleration
            accel_xyz = (accel.x, accel.y, accel.z)
            while pending_gyro:
                gyro_stamp, gyro_msg = pending_gyro.popleft()
                imu = make_imu(
                    gyro_msg, accel_xyz, gyro_stamp, seq,
                    prev_accel[1].linear_acceleration_covariance,
                )
                target.write("/imu0", imu, imu.header.stamp)
                counts["imu"] += 1
                seq += 1
                record_stamp(gyro_stamp)

    return {
        "input": input_path,
        "output": output_path,
        "mode": mode,
        "compression": compression,
        "counts": counts,
        "start_s": first_stamp,
        "end_s": last_stamp,
        "duration_s": (
            last_stamp - first_stamp
            if first_stamp is not None and last_stamp is not None else 0.0
        ),
        "output_size_bytes": os.path.getsize(output_path),
        "topics": (
            ["/cam0/image_raw"] if mode == "camera"
            else ["/imu0"] if mode == "allan"
            else ["/cam0/image_raw", "/imu0"]
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--mode", choices=("allan", "camera", "imucam"), required=True)
    parser.add_argument("--compression", choices=("none", "bz2", "lz4"), default="lz4")
    parser.add_argument("--summary")
    args = parser.parse_args()
    summary = convert(args.input, args.output, args.mode, args.compression)
    text = json.dumps(summary, ensure_ascii=False, indent=2)
    print(text)
    if args.summary:
        os.makedirs(os.path.dirname(args.summary), exist_ok=True)
        with open(args.summary, "w", encoding="utf-8") as file:
            file.write(text + "\n")


if __name__ == "__main__":
    main()
