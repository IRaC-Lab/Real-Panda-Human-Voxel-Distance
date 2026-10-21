"""Feed latest camera frames while bounding outstanding segmentation work."""

from collections import OrderedDict
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def stamp_key(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class SegmentationInputNode(Node):
    def __init__(self):
        super().__init__('segmentation_input_node')
        self.declare_parameter('max_in_flight', 3)
        self.declare_parameter('feedback_timeout_sec', 1.0)
        self.limit = int(self.get_parameter('max_in_flight').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout_sec').value)
        if self.limit < 1:
            raise ValueError('max_in_flight must be positive')
        if self.feedback_timeout <= 0:
            raise ValueError('feedback_timeout_sec must be positive')
        self.infos = OrderedDict()
        self.in_flight = OrderedDict()
        self.latest = None
        self.last_sent = -1
        self.ready_after = None
        self.has_result = False
        self.image_pub = self.create_publisher(
            Image, '/camera0/segmentation/input/image', QoSProfile(depth=5))
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera0/segmentation/input/camera_info', QoSProfile(depth=5))
        self.create_subscription(Image, '/camera0/camera/color/image_raw',
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera0/camera/color/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera0/segmentation/people_mask',
                                 self.on_result, QoSProfile(depth=10))
        self.create_timer(0.1, self.try_publish)

    def on_image(self, image):
        self.latest = image
        self.try_publish()

    def on_info(self, info):
        self.infos[stamp_key(info.header)] = info
        while len(self.infos) > 120:
            self.infos.popitem(last=False)
        self.try_publish()

    def on_result(self, mask):
        key = stamp_key(mask.header)
        if key not in self.in_flight:
            return
        self.has_result = True
        for pending in list(self.in_flight):
            if pending <= key:
                self.in_flight.pop(pending)
        self.try_publish()

    def try_publish(self):
        now = time.monotonic()
        if not self.image_pub.get_subscription_count() or not self.info_pub.get_subscription_count():
            self.ready_after = None
            return
        if self.ready_after is None:
            # NITROS creates ROS subscribers before its inference graph is ready.
            self.ready_after = now + 3.0
        if now < self.ready_after:
            return
        for key, sent_at in list(self.in_flight.items()):
            if now - sent_at > self.feedback_timeout:
                self.in_flight.pop(key)
                if self.has_result:
                    self.get_logger().warning('segmentation result timed out; recovering input slot',
                                              throttle_duration_sec=5)
        if self.latest is None or len(self.in_flight) >= self.limit:
            return
        key = stamp_key(self.latest.header)
        if key <= self.last_sent or key not in self.infos:
            return
        info = self.infos[key]
        if info.header.frame_id != self.latest.header.frame_id:
            return
        self.in_flight[key] = now
        self.last_sent = key
        self.info_pub.publish(info)
        self.image_pub.publish(self.latest)


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationInputNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
