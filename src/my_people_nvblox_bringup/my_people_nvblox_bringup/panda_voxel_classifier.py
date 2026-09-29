#!/usr/bin/env python3

"""Publish precomputed Panda collision-mesh voxels at the latest link TFs."""

import copy
import math
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Point32
from nvblox_msgs.msg import Index3D, VoxelBlock, VoxelBlockLayer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray, Marker


BlockKey = Tuple[int, int, int]


class PandaVoxelClassifier(Node):
    """Publish Panda collision-mesh voxels transformed by link TFs."""

    def __init__(self) -> None:
        super().__init__('panda_voxel_classifier')

        self.declare_parameter('voxel_frame', '')
        self.declare_parameter(
            'realtime_panda_layer_topic', '/panda/voxel_layer')
        self.declare_parameter('sphere_topic', '/panda/collision_spheres')
        self.declare_parameter('realtime_update_rate_hz', 20.0)
        self.declare_parameter('margin_m', 0.025)
        self.declare_parameter('sphere_alpha', 0.18)
        self.declare_parameter('voxel_template_path', '')
        self.declare_parameter('robot_description_path', '')
        self.declare_parameter(
            'robot_description_topic', '/panda/robot_description')
        self.declare_parameter('sphere_links', ['panda_link0'])
        self.declare_parameter('sphere_offsets', [0.0, 0.0, 0.05])
        self.declare_parameter('sphere_radii', [0.10])

        configured_voxel_frame = str(
            self.get_parameter('voxel_frame').value).strip()
        self.margin_m = float(self.get_parameter('margin_m').value)
        self.sphere_alpha = float(self.get_parameter('sphere_alpha').value)
        self.block_size_m = 0.4
        self.voxel_size_m = 0.05
        self.layer_type = 2  # nvblox::LayerType::kColor
        self.voxel_templates = self.load_voxel_templates()
        realtime_update_rate_hz = float(
            self.get_parameter('realtime_update_rate_hz').value)
        links = [
            str(value) for value in
            self.get_parameter('sphere_links').value]
        offsets = [
            float(value) for value in
            self.get_parameter('sphere_offsets').value]
        radii = [
            float(value) for value in
            self.get_parameter('sphere_radii').value]

        if realtime_update_rate_hz <= 0.0:
            raise ValueError(
                'realtime_update_rate_hz must be greater than zero')
        if not configured_voxel_frame:
            raise ValueError('voxel_frame is required')
        if len(offsets) != 3 * len(links) or len(radii) != len(links):
            raise ValueError(
                'sphere_offsets needs 3 values per sphere_links entry, and '
                'sphere_radii must have the same length as sphere_links')
        if any(radius <= 0.0 for radius in radii):
            raise ValueError('All sphere radii must be greater than zero')

        parameter_spheres = [
            (link, tuple(offsets[3 * index:3 * index + 3]), radii[index])
            for index, link in enumerate(links)
        ]
        self.spheres = parameter_spheres

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.realtime_panda_layer_publisher = self.create_publisher(
            VoxelBlockLayer,
            str(self.get_parameter('realtime_panda_layer_topic').value),
            qos,
        )
        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sphere_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('sphere_topic').value),
            static_qos,
        )
        self.description_publisher = self.create_publisher(
            String,
            str(self.get_parameter('robot_description_topic').value),
            static_qos,
        )

        self.voxel_frame: Optional[str] = configured_voxel_frame or None
        self.last_status = ''
        self.realtime_callback_group = MutuallyExclusiveCallbackGroup()
        self.realtime_timer = self.create_timer(
            1.0 / realtime_update_rate_hz,
            self.publish_realtime_voxels,
            callback_group=self.realtime_callback_group,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.publish_robot_description()
        self.publish_spheres()
        self.get_logger().info(
            f'Loaded {sum(map(len, self.voxel_templates.values()))} '
            f'collision-mesh voxels across '
            f'{len(self.voxel_templates)} links in '
            f'{self.voxel_frame}')

    def load_voxel_templates(self) -> Dict[str, np.ndarray]:
        template_path = str(
            self.get_parameter('voxel_template_path').value).strip()
        if not template_path:
            raise ValueError('voxel_template_path is required')
        try:
            with np.load(template_path, allow_pickle=False) as archive:
                self.voxel_size_m = float(archive['voxel_size_m'][0])
                inflation_m = float(archive['inflation_m'][0])
                templates = {
                    key: np.asarray(archive[key], dtype=np.float64)
                    for key in archive.files
                    if key not in ('voxel_size_m', 'inflation_m')
                }
        except (OSError, KeyError, ValueError) as exception:
            raise ValueError(
                f'Could not load Panda voxel templates from {template_path}: '
                f'{exception}') from exception
        if not templates or any(
                points.ndim != 2 or points.shape[1] != 3
                for points in templates.values()):
            raise ValueError(f'Invalid Panda voxel templates: {template_path}')
        if not math.isclose(inflation_m, self.margin_m, abs_tol=1e-6):
            self.get_logger().warning(
                f'Voxel template inflation ({inflation_m:.3f} m) differs from '
                f'margin_m ({self.margin_m:.3f} m)')
        return templates

    def publish_robot_description(self) -> None:
        """Publish the Panda URDF for the sphere-placement debug RViz."""
        description_path = str(
            self.get_parameter('robot_description_path').value)
        if not description_path:
            self.get_logger().warning(
                'robot_description_path is empty; '
                'RobotModel will not be shown')
            return
        try:
            with open(description_path, 'r', encoding='utf-8') as urdf_file:
                description = urdf_file.read()
        except OSError as exception:
            self.get_logger().warning(
                f'Could not read Panda URDF {description_path}: {exception}')
            return

        package_root = description_path.rsplit('/robots/', 1)[0]
        description = description.replace(
            'package://franka_description/', f'file://{package_root}/')
        self.description_publisher.publish(String(data=description))

    def link_transforms_in_voxel_frame(self):
        if self.voxel_frame is None:
            return None
        transforms = {}
        links = set(self.voxel_templates)
        links.update(link for link, _, _ in self.spheres)
        for link in links:
            try:
                transforms[link] = self.tf_buffer.lookup_transform(
                    self.voxel_frame, link, Time(),
                    timeout=Duration(seconds=0.05)).transform
            except TransformException as exception:
                status = f'TF unavailable: {link} -> {self.voxel_frame}'
                if status != self.last_status:
                    self.get_logger().warning(f'{status}: {exception}')
                    self.last_status = status
                return None
        return transforms

    @staticmethod
    def voxel_indices_from_templates(templates, transforms, voxel_size_m):
        indices = []
        for link, local_points in templates.items():
            transform = transforms[link]
            q = transform.rotation
            norm = math.sqrt(
                q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if norm == 0.0:
                raise ValueError('Received a zero-length transform quaternion')
            x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            rotation = np.asarray([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w),
                 1 - 2 * (x * x + y * y)],
            ])
            translation = np.asarray([
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ])
            world_points = local_points @ rotation.T + translation
            indices.append(
                np.floor(world_points / voxel_size_m).astype(np.int64))
        return np.unique(np.concatenate(indices), axis=0)

    def publish_realtime_voxels(self) -> None:
        """Transform precomputed link voxels using the latest TFs."""
        if self.voxel_frame is None:
            return
        transforms = self.link_transforms_in_voxel_frame()
        if transforms is None:
            return

        stamp = self.get_clock().now().to_msg()
        blocks: Dict[BlockKey, VoxelBlock] = {}
        panda_color = ColorRGBA(r=0.02, g=0.12, b=1.0, a=1.0)
        for voxel_index in self.voxel_indices_from_templates(
                self.voxel_templates, transforms, self.voxel_size_m):
            point = tuple(
                (index + 0.5) * self.voxel_size_m
                for index in voxel_index
            )
            block_key = tuple(
                math.floor(coordinate / self.block_size_m)
                for coordinate in point
            )
            block = blocks.setdefault(block_key, VoxelBlock())
            block.centers.append(
                Point32(x=point[0], y=point[1], z=point[2]))
            block.colors.append(copy.deepcopy(panda_color))

        msg = self.make_layer_message(stamp, True)
        for key in sorted(blocks):
            msg.block_indices.append(
                Index3D(x=key[0], y=key[1], z=key[2]))
            msg.blocks.append(blocks[key])
        self.realtime_panda_layer_publisher.publish(msg)

    def make_layer_message(self, stamp, clear: bool) -> VoxelBlockLayer:
        """Create a derived layer message with source-layer metadata."""
        msg = VoxelBlockLayer()
        msg.header.frame_id = self.voxel_frame
        msg.header.stamp = stamp
        msg.block_size_m = self.block_size_m
        msg.voxel_size_m = self.voxel_size_m
        msg.clear = clear
        msg.layer_type = self.layer_type
        return msg

    def publish_spheres(self) -> None:
        marker_array = MarkerArray()
        for index, (link, offset, radius) in enumerate(self.spheres):
            marker = Marker()
            marker.header.frame_id = link
            marker.ns = 'panda_collision_spheres'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.frame_locked = True
            marker.pose.position = Point(
                x=offset[0], y=offset[1], z=offset[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * radius
            marker.scale.y = 2.0 * radius
            marker.scale.z = 2.0 * radius
            marker.color.r = 0.1
            marker.color.g = 0.55
            marker.color.b = 1.0
            marker.color.a = self.sphere_alpha
            marker.text = link
            marker_array.markers.append(marker)
        self.sphere_publisher.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PandaVoxelClassifier()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
