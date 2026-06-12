#!/usr/bin/env python3
"""Inspect standard ROS1 camera and IMU calibration bags."""
from __future__ import annotations

import argparse
import json
import os

import cv2
import numpy as np
import rosbag


def stamp(msg, bag_time) -> float:
    value = msg.header.stamp.to_sec()
    return value if value > 0.0 else bag_time.to_sec()


def stats(times: list[float]) -> dict:
    a = np.asarray(times, dtype=float)
    if len(a) < 2:
        return {"count": int(len(a))}
    dt = np.diff(a)
    positive = dt[dt > 0]
    median = float(np.median(positive))
    return {
        "count": int(len(a)),
        "start_s": float(a[0]),
        "end_s": float(a[-1]),
        "duration_s": float(a[-1] - a[0]),
        "median_hz": float(1.0 / median),
        "p99_dt_s": float(np.percentile(positive, 99)),
        "max_dt_s": float(np.max(positive)),
        "nonpositive_dt_count": int(np.sum(dt <= 0)),
        "gaps_gt_2p5x_median": int(np.sum(positive > 2.5 * median)),
        "largest_gap_after_index": int(np.argmax(dt)),
    }


def image_array(msg) -> np.ndarray:
    channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def inspect(path: str, image_topic: str | None, imu_topic: str | None,
            preview_dir: str | None) -> dict:
    image_times = []
    imu_times = []
    imu_vectors = []
    first_image = None
    images = []
    with rosbag.Bag(path, "r") as bag:
        total_images = bag.get_message_count(image_topic) if image_topic else 0
        preview_indices = set(np.linspace(
            0, max(0, total_images - 1), min(12, total_images), dtype=int
        ).tolist()) if total_images else set()
        image_index = 0
        topics = ([image_topic] if image_topic else []) + ([imu_topic] if imu_topic else [])
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            if topic == image_topic:
                image_times.append(stamp(msg, bag_time))
                if first_image is None:
                    first_image = {
                        "width": int(msg.width),
                        "height": int(msg.height),
                        "encoding": msg.encoding,
                        "step": int(msg.step),
                        "frame_id": msg.header.frame_id,
                    }
                if image_index in preview_indices:
                    images.append((image_index, image_array(msg)))
                image_index += 1
            elif imu_topic and topic == imu_topic:
                imu_times.append(stamp(msg, bag_time))
                imu_vectors.append([
                    msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z, msg.linear_acceleration.x,
                    msg.linear_acceleration.y, msg.linear_acceleration.z,
                ])

    if preview_dir and images:
        os.makedirs(preview_dir, exist_ok=True)
        thumbs = []
        for index, image in images:
            thumb = cv2.resize(image, (480, 270), interpolation=cv2.INTER_AREA)
            cv2.putText(thumb, f"frame {index}", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, .75, (0, 255, 0), 2)
            thumbs.append(thumb)
        cols = 3
        rows = int(np.ceil(len(thumbs) / cols))
        blank = np.zeros_like(thumbs[0])
        while len(thumbs) < rows * cols:
            thumbs.append(blank)
        mosaic = np.vstack([
            np.hstack(thumbs[row * cols:(row + 1) * cols])
            for row in range(rows)
        ])
        cv2.imwrite(os.path.join(preview_dir, "image_preview_mosaic.jpg"), mosaic)

    result = {
        "path": path,
        "image_topic": image_topic,
        "image": first_image,
        "image_timing": stats(image_times),
    }
    if imu_topic:
        vectors = np.asarray(imu_vectors, dtype=float)
        result.update({
            "imu_topic": imu_topic,
            "imu_timing": stats(imu_times),
            "gyro_axis_std": np.std(vectors[:, :3], axis=0).tolist(),
            "accel_axis_std": np.std(vectors[:, 3:], axis=0).tolist(),
            "accel_norm_mean": float(np.mean(np.linalg.norm(vectors[:, 3:], axis=1))),
        })
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--image-topic")
    parser.add_argument("--imu-topic")
    parser.add_argument("--preview-dir")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = inspect(
        args.bag, args.image_topic, args.imu_topic, args.preview_dir
    )
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as file:
        file.write(text + "\n")


if __name__ == "__main__":
    main()
