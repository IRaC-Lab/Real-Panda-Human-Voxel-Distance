#!/usr/bin/env python3

"""Find and visualize the closest observed Panda/human voxel pair."""

import copy
from collections import OrderedDict
import json
import time
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from geometry_msgs.msg import Point
from nvblox_msgs.msg import VoxelBlockLayer
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, TimeReference
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


BlockKey = Tuple[int, int, int]
SphereSpec = Tuple[str, np.ndarray, float]


class DistanceFilter:
    """Reject implausible near-zero jumps using elapsed time, independent of rate."""

    def __init__(self, near_zero_threshold_m: float, max_relative_speed_mps: float,
                 distance_noise_margin_m: float, rejected_hold_timeout_s: float) -> None:
        values = (near_zero_threshold_m, max_relative_speed_mps,
                  distance_noise_margin_m, rejected_hold_timeout_s)
        if (not all(math.isfinite(value) for value in values) or
                min(values[:3]) < 0.0 or rejected_hold_timeout_s <= 0.0):
            raise ValueError('filter limits must be finite and non-negative; timeout must be positive')
        self.near_zero_threshold_m = near_zero_threshold_m
        self.max_relative_speed_mps = max_relative_speed_mps
        self.distance_noise_margin_m = distance_noise_margin_m
        self.rejected_hold_timeout_s = rejected_hold_timeout_s
        self.reset()

    def reset(self) -> None:
        self.filtered_distance = None
        self.last_update_time = None
        self.last_valid_time = None
        self.last_valid_at_edge = False

    def rejects_candidate(self, raw_distance: float, now=None) -> bool:
        if not math.isfinite(raw_distance) or raw_distance < 0.0:
            return True
        if self.filtered_distance is None:
            return raw_distance <= self.near_zero_threshold_m
        now = time.monotonic() if now is None else now
        elapsed = max(0.0, now - self.last_update_time)
        max_drop = self.max_relative_speed_mps * elapsed + self.distance_noise_margin_m
        return (raw_distance <= self.near_zero_threshold_m and
                self.filtered_distance - raw_distance > max_drop)

    def update(self, raw_distance: float, measurement_at_edge: bool = False,
               now=None) -> Tuple[Optional[float], str]:
        now = time.monotonic() if now is None else now
        rejected = self.rejects_candidate(raw_distance, now)
        self.last_update_time = now
        if not math.isfinite(raw_distance) or raw_distance < 0.0:
            return None, 'INVALID'
        if self.filtered_distance is None and raw_distance <= self.near_zero_threshold_m:
            return None, 'NO_DETECTION'
        # ponytail: d-only heuristic; use tracked person IDs when identity switches matter.
        if rejected:
            if self.last_valid_at_edge:
                self.filtered_distance = None
                return None, 'EXPIRED_AT_IMAGE_EDGE'
            if now - self.last_valid_time >= self.rejected_hold_timeout_s:
                self.filtered_distance = None
                return None, 'EXPIRED_TIMEOUT'
            return self.filtered_distance, 'REJECTED_NEAR_ZERO_JUMP'
        self.filtered_distance = raw_distance
        self.last_valid_time = now
        self.last_valid_at_edge = measurement_at_edge
        return self.filtered_distance, 'VALID'


class ClosestPandaHumanVoxels(Node):
    """Compute the exact closest pair between two cached voxel layers."""

    def __init__(self) -> None:
        super().__init__('closest_panda_human_voxels')

        self.declare_parameter(
            'human_topic', '/nvblox_node/dynamic_occupancy_layer')
        self.declare_parameter('panda_topic', '/panda/voxel_layer')
        self.declare_parameter('sphere_topic', '/panda/collision_spheres')
        self.declare_parameter(
            'expanded_sphere_topic', '/panda/human_exclusion_spheres')
        self.declare_parameter(
            'camera_info_topic', '/front_stereo_camera/left/camera_info')
        # Only publication is rate-limited; calculation/filtering follow voxel updates.
        self.declare_parameter('update_rate_hz', 5.0)
        self.declare_parameter('input_timeout_s', 0.5)
        self.declare_parameter('comparison_chunk_size', 512)
        self.declare_parameter('use_spatial_human_filter', False)
        self.declare_parameter('robot_component_overlap_ratio', 0.3)
        self.declare_parameter('panda_classification_sphere_margin_m', 0.025)
        self.declare_parameter('human_exclusion_additional_margin_m', 0.05)
        self.declare_parameter('near_zero_rejection_threshold_m', 0.3)
        self.declare_parameter('max_relative_speed_mps', 3.0)
        self.declare_parameter('distance_noise_margin_m', 0.1)
        self.declare_parameter('rejected_hold_timeout_s', 2.0)
        self.declare_parameter('image_edge_margin_ratio', 0.1)
        self.declare_parameter('marker_size_m', 0.065)
        self.declare_parameter('marker_lifetime_s', 0.5)

        human_topic = str(self.get_parameter('human_topic').value)
        panda_topic = str(self.get_parameter('panda_topic').value)
        sphere_topic = str(self.get_parameter('sphere_topic').value)
        expanded_sphere_topic = str(
            self.get_parameter('expanded_sphere_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.input_timeout_s = float(self.get_parameter('input_timeout_s').value)
        self.chunk_size = int(
            self.get_parameter('comparison_chunk_size').value)
        self.use_spatial_human_filter = bool(
            self.get_parameter('use_spatial_human_filter').value)
        self.robot_component_overlap_ratio = float(
            self.get_parameter('robot_component_overlap_ratio').value)
        self.panda_classification_sphere_margin_m = float(
            self.get_parameter('panda_classification_sphere_margin_m').value)
        self.human_exclusion_additional_margin_m = float(
            self.get_parameter('human_exclusion_additional_margin_m').value)
        near_zero_rejection_threshold_m = float(
            self.get_parameter('near_zero_rejection_threshold_m').value)
        max_relative_speed_mps = float(
            self.get_parameter('max_relative_speed_mps').value)
        distance_noise_margin_m = float(
            self.get_parameter('distance_noise_margin_m').value)
        rejected_hold_timeout_s = float(
            self.get_parameter('rejected_hold_timeout_s').value)
        self.image_edge_margin_ratio = float(
            self.get_parameter('image_edge_margin_ratio').value)
        self.marker_size_m = float(
            self.get_parameter('marker_size_m').value)
        lifetime_s = float(
            self.get_parameter('marker_lifetime_s').value)
        if not math.isfinite(self.input_timeout_s) or self.input_timeout_s <= 0.0:
            raise ValueError('input_timeout_s must be finite and positive')
        if not math.isfinite(update_rate_hz) or update_rate_hz <= 0.0 or self.chunk_size <= 0:
            raise ValueError('update_rate_hz and comparison_chunk_size must be positive')
        if max_relative_speed_mps < 0.0 or distance_noise_margin_m < 0.0:
            raise ValueError('distance filter calibration must be non-negative')
        if rejected_hold_timeout_s <= 0.0:
            raise ValueError('rejected_hold_timeout_s must be positive')
        if not 0.0 <= self.image_edge_margin_ratio < 0.5:
            raise ValueError('image_edge_margin_ratio must be in [0, 0.5)')
        if (self.panda_classification_sphere_margin_m < 0.0 or
                self.human_exclusion_additional_margin_m < 0.0):
            raise ValueError('sphere margins must be non-negative')
        if not 0.0 < self.robot_component_overlap_ratio <= 1.0:
            raise ValueError('robot_component_overlap_ratio must be in (0, 1]')
        if self.marker_size_m <= 0.0 or lifetime_s <= 0.0:
            raise ValueError('marker size and lifetime must be positive')
        self.marker_lifetime = Duration(seconds=lifetime_s).to_msg()
        self.distance_filter = DistanceFilter(
            near_zero_rejection_threshold_m, max_relative_speed_mps,
            distance_noise_margin_m, rejected_hold_timeout_s)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.human_subscription = self.create_subscription(
            VoxelBlockLayer, human_topic, self.human_callback, qos)
        self.panda_subscription = self.create_subscription(
            VoxelBlockLayer, panda_topic, self.panda_callback, qos)
        self.sphere_subscription = self.create_subscription(
            MarkerArray, sphere_topic, self.sphere_callback, static_qos)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback,
            qos_profile_sensor_data)

        self.distance_publisher = self.create_publisher(
            Float32, '/closest_panda_human/distance', 10)
        self.raw_distance_publisher = self.create_publisher(
            Float32, '/closest_panda_human/raw_distance', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/closest_panda_human/markers', 10)
        self.expanded_sphere_publisher = self.create_publisher(
            MarkerArray, expanded_sphere_topic, static_qos)

        self.human_blocks: Dict[BlockKey, np.ndarray] = {}
        self.panda_blocks: Dict[BlockKey, np.ndarray] = {}
        self.human_frame: Optional[str] = None
        self.panda_frame: Optional[str] = None
        self.human_voxel_size_m = 0.05
        self.sphere_specs: List[SphereSpec] = []
        self.camera_info: Optional[CameraInfo] = None
        self.last_valid_human_point: Optional[np.ndarray] = None
        self.last_valid_panda_point: Optional[np.ndarray] = None
        self.last_status = ''
        self.latest_result = None
        self.last_human_received = None
        self.last_panda_received = None
        self.human_layer_stamp_ns = 0
        self.panda_layer_stamp_ns = 0
        self.layer_sources = OrderedDict()
        self.timing_publisher = self.create_publisher(
            String, '/closest_panda_human/processing_timing', 10)
        source_prefix = human_topic.rsplit('/', 1)[0]
        self.depth_reference_subscription = self.create_subscription(
            TimeReference, source_prefix + '/layer_depth_reference',
            lambda msg: self.source_reference_callback(msg, 'depth'), 10)
        self.rgb_reference_subscription = self.create_subscription(
            TimeReference, source_prefix + '/layer_mask_reference',
            lambda msg: self.source_reference_callback(msg, 'rgb'), 10)
        # Coalesce cache updates; compute without waiting for the publication timer.
        self.compute_guard = self.create_guard_condition(self.compute_distance)
        self.timer = self.create_timer(1.0 / update_rate_hz, self.publish_latest)
        self.get_logger().info(
            f'Finding the closest pair between {human_topic} and '
            f'{panda_topic}; geometry=on voxel updates, output={update_rate_hz:.1f} Hz; '
            f'rejecting d <= {near_zero_rejection_threshold_m:.3f} m only '
            f'max relative speed={max_relative_speed_mps:.3f} m/s; '
            f'hold timeout={rejected_hold_timeout_s:.1f} s; '
            f'image edge margin={self.image_edge_margin_ratio:.0%}; '
            f'spatial human filter={self.use_spatial_human_filter}')

    @property
    def total_sphere_margin_m(self) -> float:
        return (
            self.panda_classification_sphere_margin_m +
            self.human_exclusion_additional_margin_m)

    def human_callback(self, msg: VoxelBlockLayer) -> None:
        if msg.voxel_size_m > 0.0:
            self.human_voxel_size_m = float(msg.voxel_size_m)
        self.human_frame = self.update_cache(
            msg, self.human_frame, self.human_blocks, 'human')
        if msg.header.frame_id:
            self.last_human_received = time.monotonic()
            self.human_layer_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if not self.human_blocks:
            self.latest_result = None
        self.compute_guard.trigger()

    def panda_callback(self, msg: VoxelBlockLayer) -> None:
        self.panda_frame = self.update_cache(
            msg, self.panda_frame, self.panda_blocks, 'Panda')
        if msg.header.frame_id:
            self.last_panda_received = time.monotonic()
            self.panda_layer_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if not self.panda_blocks:
            self.latest_result = None
        self.compute_guard.trigger()

    def sphere_callback(self, msg: MarkerArray) -> None:
        sphere_specs = []
        expanded_markers = MarkerArray()
        for marker in msg.markers:
            radius = 0.5 * max(
                float(marker.scale.x), float(marker.scale.y),
                float(marker.scale.z))
            if (marker.type != Marker.SPHERE or
                    marker.action != Marker.ADD or
                    not marker.header.frame_id or radius <= 0.0):
                continue
            offset = np.asarray([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            ], dtype=np.float64)
            sphere_specs.append((marker.header.frame_id, offset, radius))
            expanded = copy.deepcopy(marker)
            expanded.ns = 'panda_human_exclusion_spheres'
            diameter_increase = 2.0 * self.total_sphere_margin_m
            expanded.scale.x += diameter_increase
            expanded.scale.y += diameter_increase
            expanded.scale.z += diameter_increase
            expanded.color.r = 1.0
            expanded.color.g = 0.35
            expanded.color.b = 0.02
            expanded.color.a = 0.16
            expanded_markers.markers.append(expanded)
        self.sphere_specs = sphere_specs
        self.expanded_sphere_publisher.publish(expanded_markers)
        self.compute_guard.trigger()

    def source_reference_callback(self, msg: TimeReference, stream: str) -> None:
        key = Time.from_msg(msg.header.stamp).nanoseconds
        source = self.layer_sources.setdefault(key, {})
        source[stream] = Time.from_msg(msg.time_ref).nanoseconds
        while len(self.layer_sources) > 256:
            self.layer_sources.popitem(last=False)
        if key == self.human_layer_stamp_ns and len(source) == 2:
            self.compute_guard.trigger()

    def inputs_fresh(self, sources) -> bool:
        now = time.monotonic()
        if not self.human_blocks or not self.panda_blocks:
            return False
        if any(stamp is None or now - stamp > self.input_timeout_s
               for stamp in (self.last_human_received, self.last_panda_received)):
            return False
        now_ns = self.get_clock().now().nanoseconds
        return all(stamp > 0 and abs(now_ns - stamp) <= self.input_timeout_s * 1e9
                   for stamp in sources.values())

    def invalidate_result(self, status: str) -> None:
        self.latest_result = None
        self.distance_filter.reset()
        self.last_valid_human_point = self.last_valid_panda_point = None
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def human_points_touch_image_edge(self, points: np.ndarray) -> bool:
        """Return whether observed human points reach the camera image edge."""
        info = self.camera_info
        if (info is None or not info.header.frame_id or
                self.human_frame is None):
            return False
        if self.human_frame != info.header.frame_id:
            try:
                transform = self.tf_buffer.lookup_transform(
                    info.header.frame_id, self.human_frame, Time(),
                    timeout=Duration(seconds=0.05))
            except TransformException:
                return False
            points = self.transform_points(points, transform.transform)

        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        if info.width <= 0 or info.height <= 0 or fx <= 0.0 or fy <= 0.0:
            return False
        visible = points[:, 2] > 0.0
        if not np.any(visible):
            return False
        points = points[visible]
        u = fx * points[:, 0] / points[:, 2] + cx
        v = fy * points[:, 1] / points[:, 2] + cy
        margin_x = self.image_edge_margin_ratio * info.width
        margin_y = self.image_edge_margin_ratio * info.height
        return bool(np.any(
            (u <= margin_x) | (u >= info.width - margin_x) |
            (v <= margin_y) | (v >= info.height - margin_y)))

    def update_cache(
        self,
        msg: VoxelBlockLayer,
        old_frame: Optional[str],
        blocks: Dict[BlockKey, np.ndarray],
        label: str,
    ) -> Optional[str]:
        if not msg.header.frame_id:
            self.get_logger().warning(f'Received {label} layer without frame_id')
            return old_frame
        if old_frame is not None and old_frame != msg.header.frame_id:
            self.get_logger().warning(
                f'{label} frame changed from {old_frame} to '
                f'{msg.header.frame_id}; clearing its cache')
            blocks.clear()
            self.invalidate_result('voxel frame changed')
        if msg.clear:
            blocks.clear()
        for block_index, block in zip(msg.block_indices, msg.blocks):
            key = (block_index.x, block_index.y, block_index.z)
            if not block.centers:
                blocks.pop(key, None)
                continue
            points = np.asarray(
                [(center.x, center.y, center.z) for center in block.centers],
                dtype=np.float64,
            )
            points = points[np.isfinite(points).all(axis=1)]
            if len(points):
                blocks[key] = points
            else:
                blocks.pop(key, None)
        return msg.header.frame_id

    @staticmethod
    def transform_points(points: np.ndarray, transform) -> np.ndarray:
        q = transform.rotation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm == 0.0:
            raise ValueError('Received a zero-length transform quaternion')
        x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
        rotation = np.asarray([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ])
        translation = np.asarray([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ])
        return points @ rotation.T + translation

    def compute_distance(self) -> None:
        """Calculate and filter a fresh map snapshot before publication throttling."""
        started_mono_ns = time.monotonic_ns()
        started_ros_ns = self.get_clock().now().nanoseconds
        pair = self.layer_sources.get(self.human_layer_stamp_ns)
        if pair is None or 'depth' not in pair or 'rgb' not in pair:
            return
        sources = dict(pair, panda=self.panda_layer_stamp_ns)
        if not self.inputs_fresh(sources):
            self.invalidate_result('waiting for fresh RGB-D and Panda voxel inputs')
            return
        if not self.human_blocks or self.human_frame is None:
            status = 'waiting for human voxel data'
            self.invalidate_result(status)
            return

        human_points = np.concatenate(tuple(self.human_blocks.values()))
        stamp = self.get_clock().now().to_msg()

        waiting_for_spheres = (
            self.use_spatial_human_filter and not self.sphere_specs)
        if (not self.panda_blocks or self.panda_frame is None or
                waiting_for_spheres):
            status = 'waiting for Panda voxel data'
            if waiting_for_spheres:
                status += ' and collision-sphere data'
            self.invalidate_result(status)
            return

        panda_points = np.concatenate(tuple(self.panda_blocks.values()))
        if self.panda_frame != self.human_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.human_frame, self.panda_frame, Time(),
                    timeout=Duration(seconds=0.1))
            except TransformException as exception:
                status = f'TF unavailable: {self.panda_frame} -> {self.human_frame}'
                self.invalidate_result(f'{status}: {exception}')
                return
            panda_points = self.transform_points(
                panda_points, transform.transform)
            stamp = transform.header.stamp

        prepared_mono_ns = time.monotonic_ns()
        components = self.connected_component_indices(human_points, self.human_voxel_size_m)
        components_done_mono_ns = time.monotonic_ns()
        ignored_count = 0
        if self.use_spatial_human_filter:
            transformed_spheres = self.spheres_in_human_frame()
            if transformed_spheres is None:
                self.invalidate_result('waiting for fresh collision-sphere transforms')
                return
            sphere_centers, sphere_radii = transformed_spheres
            components, ignored_count = self.exclude_robot_components(
                human_points, sphere_centers, sphere_radii,
                self.human_voxel_size_m,
                self.robot_component_overlap_ratio, components)
            if not components:
                status = 'all human voxels were removed by the spatial filter'
                self.invalidate_result(status)
                return

        exclusion_done_mono_ns = time.monotonic_ns()
        panda_tree = cKDTree(panda_points)
        candidates = sorted(
            (
                (*self.closest_pair(human_points[component], panda_points, panda_tree),
                 human_points[component])
                for component in components
            ),
            key=lambda candidate: candidate[2],
        )
        nearest_done_mono_ns = time.monotonic_ns()
        filter_time = time.monotonic()
        _, _, raw_distance, _ = candidates[0]
        selected = self.first_accepted_candidate(
            candidates, self.distance_filter, filter_time)
        human_point, panda_point, selected_distance, selected_points = selected
        used_fallback = selected is not candidates[0]
        filtered_distance, filter_status = self.distance_filter.update(
            selected_distance,
            self.human_points_touch_image_edge(selected_points), now=filter_time)
        if filter_status == 'VALID':
            self.last_valid_human_point = human_point.copy()
            self.last_valid_panda_point = panda_point.copy()
            display_human_point = human_point
            display_panda_point = panda_point
        elif filter_status == 'REJECTED_NEAR_ZERO_JUMP':
            if (self.last_valid_human_point is None or
                    self.last_valid_panda_point is None):
                filtered_distance = None
            display_human_point = self.last_valid_human_point
            display_panda_point = self.last_valid_panda_point
        else:
            display_human_point = human_point
            display_panda_point = panda_point

        finished_mono_ns = time.monotonic_ns()
        finished_ros_ns = self.get_clock().now().nanoseconds
        if not self.inputs_fresh(sources):
            self.invalidate_result('RGB-D or Panda inputs expired during distance calculation')
            return
        self.latest_result = {
            'raw_distance': raw_distance, 'filtered_distance': filtered_distance,
            'filter_status': filter_status, 'human_point': display_human_point,
            'panda_point': display_panda_point, 'stamp': stamp,
            'sources': sources, 'layer_stamp_ns': self.human_layer_stamp_ns,
            'ignored_voxels': ignored_count, 'fallback_component': used_fallback,
        }
        self.last_status = filter_status
        if self.timing_publisher.get_subscription_count() > 0:
            timing = {
                'layer_stamp_ns': self.human_layer_stamp_ns,
                'depth_source_ns': sources['depth'], 'rgb_source_ns': sources['rgb'],
                'panda_source_ns': sources['panda'],
                'compute_started_ns': started_ros_ns, 'compute_finished_ns': finished_ros_ns,
                'compute_started_mono_ns': started_mono_ns,
                'compute_finished_mono_ns': finished_mono_ns,
                'compute_ms': (finished_mono_ns - started_mono_ns) / 1e6,
                'stage_ms': {
                    'voxel_preparation': (prepared_mono_ns - started_mono_ns) / 1e6,
                    'human_components': (components_done_mono_ns - prepared_mono_ns) / 1e6,
                    'robot_exclusion': (exclusion_done_mono_ns - components_done_mono_ns) / 1e6,
                    'nearest_pair': (nearest_done_mono_ns - exclusion_done_mono_ns) / 1e6,
                    'distance_filter': (finished_mono_ns - nearest_done_mono_ns) / 1e6,
                },
                'depth_to_compute_ms': (finished_ros_ns - sources['depth']) / 1e6,
                'rgb_to_compute_ms': (finished_ros_ns - sources['rgb']) / 1e6,
                'raw_distance_m': raw_distance,
                'filtered_distance_m': filtered_distance,
                'filter_status': filter_status,
                'ignored_voxels': ignored_count, 'fallback_component': used_fallback,
            }
            self.timing_publisher.publish(String(data=json.dumps(timing, allow_nan=False)))

    def publish_latest(self) -> None:
        """Publish the cached result; never calculate or advance the filter here."""
        result = self.latest_result
        if result is None or not self.inputs_fresh(result['sources']):
            self.invalidate_result('waiting for fresh RGB-D and Panda voxel inputs')
            self.raw_distance_publisher.publish(Float32(data=math.nan))
            self.distance_publisher.publish(Float32(data=math.nan))
            self.marker_publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            return
        raw_distance = result['raw_distance']
        filtered_distance = result['filtered_distance']
        self.raw_distance_publisher.publish(Float32(data=float(raw_distance)))
        self.distance_publisher.publish(Float32(
            data=float(filtered_distance) if filtered_distance is not None else math.nan))
        if filtered_distance is None:
            self.marker_publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        else:
            self.publish_markers(
                result['human_point'], result['panda_point'], raw_distance,
                filtered_distance, result['filter_status'], result['stamp'])
        self.get_logger().info(
            f'd_raw={raw_distance:.3f} m, '
            f'd_filtered={filtered_distance if filtered_distance is not None else math.nan:.3f} m, '
            f'filter={result["filter_status"]}, ignored={result["ignored_voxels"]}')

    def spheres_in_human_frame(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Transform collision spheres and enlarge them for human filtering."""
        if self.human_frame is None:
            return None
        centers = []
        radii = []
        transforms = {}
        for frame, offset, radius in self.sphere_specs:
            if frame == self.human_frame:
                center = offset
            else:
                try:
                    if frame not in transforms:
                        transforms[frame] = self.tf_buffer.lookup_transform(
                            self.human_frame, frame, Time(),
                            timeout=Duration(seconds=0.05))
                    center = self.transform_points(
                        offset[None, :], transforms[frame].transform)[0]
                except TransformException as exception:
                    status = f'TF unavailable: {frame} -> {self.human_frame}'
                    if status != self.last_status:
                        self.get_logger().warning(f'{status}: {exception}')
                        self.last_status = status
                    return None
            centers.append(center)
            radii.append(radius + self.total_sphere_margin_m + 1e-6)
        return np.asarray(centers), np.asarray(radii)

    @staticmethod
    def first_accepted_candidate(candidates, distance_filter, now=None):
        """Prefer the nearest candidate that the temporal filter accepts."""
        return next(
            (candidate for candidate in candidates
             if not distance_filter.rejects_candidate(candidate[2], now)),
            candidates[0],
        )

    @staticmethod
    def connected_component_indices(
        points: np.ndarray, voxel_size_m: float,
    ) -> List[np.ndarray]:
        """Return 26-connected components as point-index arrays."""
        if not len(points):
            return []
        indices = np.floor(points / voxel_size_m).astype(np.int64)
        cells, inverse = np.unique(indices, axis=0, return_inverse=True)
        pairs = cKDTree(cells).query_pairs(1, p=np.inf, output_type='ndarray')
        graph = coo_matrix(
            (np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
            shape=(len(cells), len(cells)))
        _, labels = connected_components(graph, directed=False)
        point_labels = labels[inverse]
        order = np.argsort(point_labels, kind='stable')
        boundaries = np.flatnonzero(np.diff(point_labels[order])) + 1
        return list(np.split(order, boundaries))

    @staticmethod
    def exclude_robot_components(
        human_points: np.ndarray,
        sphere_centers: np.ndarray,
        sphere_radii: np.ndarray,
        voxel_size_m: float,
        overlap_ratio: float,
        components: Optional[List[np.ndarray]] = None,
    ) -> Tuple[List[np.ndarray], int]:
        """Remove connected human components mostly overlapping Panda."""
        squared_radii = sphere_radii * sphere_radii
        difference = np.maximum(
            np.abs(human_points[:, None, :] - sphere_centers[None, :, :]) -
            0.5 * voxel_size_m,
            0.0,
        )
        overlaps_robot = np.any(
            np.einsum('ijk,ijk->ij', difference, difference) <=
            squared_radii[None, :],
            axis=1,
        )

        if components is None:
            components = ClosestPandaHumanVoxels.connected_component_indices(human_points, voxel_size_m)
        retained = [component for component in components
                    if np.mean(overlaps_robot[component]) < overlap_ratio]
        removed = len(human_points) - sum(len(component) for component in retained)
        return retained, removed

    def closest_pair(
        self, human_points: np.ndarray, panda_points: np.ndarray,
        panda_tree=None,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        tree = panda_tree if panda_tree is not None else cKDTree(panda_points)
        best_distance = math.inf
        best_human = human_points[0]
        best_panda = panda_points[0]
        for start in range(0, len(human_points), self.chunk_size):
            humans = human_points[start:start + self.chunk_size]
            distances, indices = tree.query(humans, k=1)
            closest = int(np.argmin(distances))
            if distances[closest] < best_distance:
                best_distance = float(distances[closest])
                best_human = humans[closest].copy()
                best_panda = panda_points[indices[closest]].copy()
        return best_human, best_panda, best_distance

    def publish_markers(
        self,
        human_point: np.ndarray,
        panda_point: np.ndarray,
        raw_distance: float,
        filtered_distance: float,
        filter_status: str,
        stamp,
    ) -> None:
        markers = MarkerArray()
        markers.markers.append(self.cube_marker(
            0, 'closest_human_voxel', human_point, (0.0, 1.0, 0.0, 1.0), stamp))
        markers.markers.append(self.cube_marker(
            1, 'closest_panda_voxel', panda_point, (1.0, 0.05, 0.55, 1.0), stamp))

        line = self.base_marker(2, 'minimum_distance', Marker.LINE_LIST, stamp)
        line.scale.x = 0.012
        line.color.r, line.color.g, line.color.b, line.color.a = (
            1.0, 1.0, 0.0, 1.0)
        line.points = [
            Point(x=float(human_point[0]), y=float(human_point[1]),
                  z=float(human_point[2])),
            Point(x=float(panda_point[0]), y=float(panda_point[1]),
                  z=float(panda_point[2])),
        ]
        markers.markers.append(line)

        midpoint = 0.5 * (human_point + panda_point)
        text_lines = (
            (f'd_raw={raw_distance:.3f}m', 0.26),
            (f'd_filtered={filtered_distance:.3f}m', 0.12),
            (filter_status, -0.02),
        )
        for marker_id, (caption, z_offset) in enumerate(text_lines, start=3):
            text = self.base_marker(
                marker_id, 'minimum_distance', Marker.TEXT_VIEW_FACING,
                stamp)
            text.pose.position = Point(
                x=float(midpoint[0]),
                y=float(midpoint[1]),
                z=float(midpoint[2] + z_offset),
            )
            text.pose.orientation.w = 1.0
            text.scale.z = 0.12
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = caption
            markers.markers.append(text)
        self.marker_publisher.publish(markers)

    def base_marker(self, marker_id: int, namespace: str, marker_type: int, stamp):
        marker = Marker()
        marker.header.frame_id = self.human_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.lifetime = self.marker_lifetime
        return marker

    def cube_marker(self, marker_id, namespace, point, color, stamp):
        marker = self.base_marker(marker_id, namespace, Marker.CUBE, stamp)
        marker.pose.position = Point(
            x=float(point[0]), y=float(point[1]), z=float(point[2]))
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_size_m
        marker.scale.y = self.marker_size_m
        marker.scale.z = self.marker_size_m
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClosestPandaHumanVoxels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
