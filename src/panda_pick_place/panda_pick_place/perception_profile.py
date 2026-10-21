"""Measure image topic rates and header ages without decoding image pixels."""

import argparse
import json
import statistics
import struct
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


TOPICS = [
    '/camera0/camera/color/image_raw',
    '/camera0/camera/depth/image_rect_raw',
    '/camera0/segmentation/image_resized',
    '/camera0/segmentation/image_bgr',
    '/camera0/segmentation/people_mask',
    '/camera0/unet/colored_segmentation_mask',
    '/nvblox_node/dynamic_color_frame_overlay',
    '/nvblox_node/dynamic_depth_frame_overlay',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--warmup', type=float, default=3.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--topic', action='append', help='Repeat to override default topics')
    args = parser.parse_args()
    if args.duration <= 0 or args.warmup < 0:
        parser.error('duration must be positive and warmup must be nonnegative')

    rclpy.init()
    node = Node('perception_profile')
    topics = args.topic or TOPICS
    records = {name: {'times': [], 'stamps': set(), 'ages': [], 'image': None}
               for name in topics}
    subscriptions = []
    start = time.monotonic() + args.warmup
    end = start + args.duration

    for name in topics:
        def callback(raw, topic=name):
            now = time.monotonic()
            record = records[topic]
            if record['image'] is None:
                message = deserialize_message(raw, Image)
                record['image'] = {
                    'width': message.width, 'height': message.height,
                    'encoding': message.encoding,
                }
            if not start <= now < end:
                return
            record['times'].append(now)
            # Image starts with Header. The 4-byte CDR encapsulation precedes
            # Header's signed seconds and unsigned nanoseconds.
            endian = '<' if raw[1] & 1 else '>'
            seconds, nanoseconds = struct.unpack_from(endian + 'iI', raw, 4)
            stamp = seconds * 1_000_000_000 + nanoseconds
            if stamp > 0:
                record['stamps'].add(stamp)
                age_ms = (time.time_ns() - stamp) / 1_000_000
                if 0 <= age_ms < 10_000:
                    record['ages'].append(age_ms)

        subscriptions.append(node.create_subscription(
            Image, name, callback, qos_profile_sensor_data, raw=True))

    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = {
            'time': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'duration_seconds': args.duration,
            'qos': 'best_effort',
            'note': 'Observer subscriptions can add ROS serialization work; '
                    'header age includes all upstream processing, not stage-only latency.',
            'topics': {},
        }
        for name, record in records.items():
            times = record['times']
            ages = sorted(record['ages'])
            result['topics'][name] = {
                'frames': len(times),
                'fps': round(len(times) / args.duration, 2),
                'unique_stamps': len(record['stamps']),
                'header_age_samples': len(ages),
                'header_age_median_ms': round(statistics.median(ages), 1) if ages else None,
                'header_age_p95_ms': round(ages[int((len(ages) - 1) * .95)], 1)
                    if ages else None,
                'image': record['image'],
            }
        output = json.dumps(result, indent=2)
        if args.output:
            args.output.write_text(output + '\n')
        print(output, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
