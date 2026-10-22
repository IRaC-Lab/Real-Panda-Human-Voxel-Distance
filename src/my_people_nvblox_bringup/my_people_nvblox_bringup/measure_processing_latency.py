"""Measure the latest RGB-D contribution to a distance result, before output throttling.

Map voxels can persist across frames; these ages describe the latest integrated
RGB-D pair in each snapshot, not the capture time of every cached voxel.
"""
import argparse
from collections import Counter, OrderedDict
import json
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


def summarize(values):
    if not values:
        return {'count': 0}
    ordered = sorted(values)
    return {'count': len(values), 'mean_ms': statistics.fmean(values),
            'median_ms': statistics.median(values),
            'p95_ms': ordered[math.ceil(len(values) * .95) - 1],
            'max_ms': ordered[-1], 'negative_samples': sum(value < 0 for value in values)}


class DetailedTimings:
    """Match frame stamps at ROS topic boundaries; includes transport/scheduling.

    Tensor observations use standard ROS messages and can add serialization work.
    These are observed node-to-node intervals, not CUDA kernel-only durations.
    """

    def __init__(self):
        self.checkpoints = OrderedDict()
        self.layers = OrderedDict()
        self.samples = {name: [] for name in (
            'people_segmentation', 'tensorrt_interval', 'mask_decode_interval', 'inference_decode_interval',
            'nvblox_interval', 'voxel_preparation', 'human_components',
            'robot_exclusion', 'nearest_pair', 'distance_filter')}
        self.missing = Counter()
        self.negative = Counter()

    @staticmethod
    def remember(buffer, key, value):
        buffer[key] = value
        while len(buffer) > 600:
            buffer.popitem(last=False)

    def checkpoint(self, stage, key, received_ns):
        points = self.checkpoints.setdefault(key, {})
        points.setdefault(stage, received_ns)
        while len(self.checkpoints) > 600:
            self.checkpoints.popitem(last=False)

    def interval(self, name, start, end):
        if start is None or end is None:
            self.missing[name] += 1
        elif end < start:
            self.negative[name] += 1
        else:
            self.samples[name].append((end - start) / 1e6)

    def computation(self, sample):
        stages = sample.get('stage_ms', {})
        for name in ('voxel_preparation', 'human_components', 'robot_exclusion',
                     'nearest_pair', 'distance_filter'):
            value = stages.get(name)
            if isinstance(value, (float, int)) and math.isfinite(value) and value >= 0:
                self.samples[name].append(value)
            else:
                self.missing[name] += 1

    def frame(self, sample, image_arrivals):
        key = int(sample['rgb_source_ns'])
        points = self.checkpoints.get(key, {})
        self.interval('people_segmentation', image_arrivals['rgb'].get(key),
                      points.get('mask_native'))
        self.interval('tensorrt_interval', points.get('trt_input'), points.get('trt_output'))
        self.interval('mask_decode_interval', points.get('trt_output'), points.get('mask_decoded'))
        self.interval('inference_decode_interval', points.get('trt_input'), points.get('mask_decoded'))
        self.interval('nvblox_interval', points.get('mask_native'),
                      self.layers.get(int(sample['layer_stamp_ns'])))

    def report(self):
        return {'method': 'Matched ROS topic receipt times; tensor intervals include transport and observer overhead. Distance substeps use in-node monotonic timestamps.',
                'latency': {name: summarize(values) for name, values in self.samples.items()},
                'missing_pairs': dict(self.missing), 'negative_pairs': dict(self.negative)}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=180.0)
    parser.add_argument('--detailed', action='store_true', help='Observe segmentation tensors and distance substeps; adds ROS serialization work')
    options, ros_args = parser.parse_known_args(args)
    if not math.isfinite(options.duration) or options.duration <= 0:
        parser.error('--duration must be finite and positive')
    rclpy.init(args=ros_args)
    node = Node('distance_processing_measurement')
    arrivals = {'rgb': OrderedDict(), 'depth': OrderedDict()}
    image_counts = Counter()
    counters = Counter()
    statuses = Counter()
    seen_pairs = set()
    details = DetailedTimings() if options.detailed else None
    # ponytail: memory grows with measurement duration; use streaming stats for continuous monitoring.
    samples = {name: [] for name in ('compute_ms', 'depth_to_compute_ms', 'rgb_to_compute_ms',
                                    'depth_received_to_compute_ms', 'rgb_received_to_compute_ms')}
    start = None

    def active(now):
        return start is not None and start <= now < start + options.duration

    def on_image(stream, msg):
        now_ns = time.monotonic_ns()
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        arrivals[stream][key] = now_ns
        while len(arrivals[stream]) > 600:
            arrivals[stream].popitem(last=False)
        if active(now_ns / 1e9):
            image_counts[stream] += 1

    def on_timing(msg):
        if not active(time.monotonic()):
            return
        try:
            sample = json.loads(msg.data)
            duration = float(sample['compute_ms'])
            pair = (int(sample['depth_source_ns']), int(sample['rgb_source_ns']))
            ended = int(sample['compute_finished_mono_ns'])
            if not math.isfinite(duration) or duration < 0 or min(pair) <= 0:
                raise ValueError('invalid timing sample')
            counters['calculations'] += 1
            statuses[sample['filter_status']] += 1
            samples['compute_ms'].append(duration)
            if details is not None:
                details.computation(sample)
            if sample['filter_status'] != 'VALID' or pair in seen_pairs:
                return
            seen_pairs.add(pair)
            if details is not None:
                details.frame(sample, arrivals)
            for stream, key in zip(('depth', 'rgb'), pair):
                samples[stream + '_to_compute_ms'].append(float(sample[stream + '_to_compute_ms']))
                if key in arrivals[stream]:
                    samples[stream + '_received_to_compute_ms'].append((ended - arrivals[stream][key]) / 1e6)
        except (ValueError, TypeError, KeyError):
            counters['invalid_timing_messages'] += 1

    def on_distance(msg):
        if active(time.monotonic()):
            counters['publications'] += 1
            counters['finite_publications' if math.isfinite(msg.data) else 'invalid_publications'] += 1

    subscriptions = [
        node.create_subscription(Image, '/camera0/camera/color/image_raw',
                                 lambda msg: on_image('rgb', msg), qos_profile_sensor_data),
        node.create_subscription(Image, '/camera0/camera/depth/image_rect_raw',
                                 lambda msg: on_image('depth', msg), qos_profile_sensor_data),
        node.create_subscription(String, '/closest_panda_human/processing_timing', on_timing, 10),
        node.create_subscription(Float32, '/closest_panda_human/distance', on_distance, qos_profile_sensor_data),
    ]
    if details is not None:
        from isaac_ros_tensor_list_interfaces.msg import TensorList
        from nvblox_msgs.msg import VoxelBlockLayer

        def on_checkpoint(stage, msg):
            key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            details.checkpoint(stage, key, time.monotonic_ns())

        for stage, topic, kind in (
                ('trt_input', '/camera0/segmentation/tensor_input', TensorList),
                ('trt_output', '/camera0/segmentation/tensor_output', TensorList),
                ('mask_decoded', '/camera0/segmentation/people_mask', Image),
                ('mask_native', '/camera0/segmentation/people_mask_native', Image)):
            subscriptions.append(node.create_subscription(
                kind, topic, lambda msg, stage=stage: on_checkpoint(stage, msg),
                qos_profile_sensor_data))

        def on_layer(msg):
            key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            details.remember(details.layers, key, time.monotonic_ns())

        subscriptions.append(node.create_subscription(
            VoxelBlockLayer, '/nvblox_node/dynamic_occupancy_layer', on_layer,
            qos_profile_sensor_data))

    try:
        ready = time.monotonic() + 3
        while rclpy.ok() and time.monotonic() < ready:
            rclpy.spin_once(node, timeout_sec=.05)
        start = time.monotonic()
        next_progress = 30.0
        print(f'Measuring for {options.duration:g}s; no robot commands are sent.', flush=True)
        while rclpy.ok() and time.monotonic() - start < options.duration:
            rclpy.spin_once(node, timeout_sec=max(0.0, min(.05, options.duration - (time.monotonic() - start))))
            elapsed = time.monotonic() - start
            if elapsed >= next_progress:
                print(f'{elapsed:.0f}s: calculations={counters["calculations"]/elapsed:.2f} Hz, '
                      f'valid frame pairs={len(seen_pairs)/elapsed:.2f} Hz, '
                      f'finite outputs={counters["finite_publications"]/elapsed:.2f} Hz', flush=True)
                next_progress += 30
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = min(options.duration, time.monotonic() - start) if start is not None else 0
        result = {
            'duration_s': elapsed,
            'scope': 'Latest integrated RGB-D pair to distance calculation completion; excludes 5 Hz output waiting and GUI rendering.',
            'calculation_hz': counters['calculations'] / elapsed if elapsed else 0,
            'valid_unique_frame_pair_hz': len(seen_pairs) / elapsed if elapsed else 0,
            'output_hz': counters['publications'] / elapsed if elapsed else 0,
            'finite_output_hz': counters['finite_publications'] / elapsed if elapsed else 0,
            'invalid_outputs': counters['invalid_publications'],
            'camera_rgb_hz': image_counts['rgb'] / elapsed if elapsed else 0,
            'camera_depth_hz': image_counts['depth'] / elapsed if elapsed else 0,
            'filter_status_counts': dict(statuses),
            'invalid_timing_messages': counters['invalid_timing_messages'],
            'latency': {name: summarize(values) for name, values in samples.items()},
        }
        if details is not None:
            result['detailed'] = details.report()
        print(json.dumps(result, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
