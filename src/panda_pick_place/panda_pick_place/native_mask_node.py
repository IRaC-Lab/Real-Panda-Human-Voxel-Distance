"""Filter a model mask and pair it with calibration at camera resolution."""

from collections import OrderedDict
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def prepare_native_mask(mask, width, height, component_threshold=2000, component_subsampling=2):
    """Filter four-connected components, then retain any foreground coverage.

    nvblox filters four-connected components on a 2x subsampled mask using
    threshold // 4, then repeats each pixel twice in both dimensions. Doing
    component_subsampling=2 reproduces that legacy filter. A stride of 1
    filters at full resolution to preserve thin attached foreground and gaps.
    The ROS node defaults to 1; this helper defaults to 2 for compatibility.
    Doing this once before mapping avoids running the filter twice.
    Area reduction with a >0 threshold preserves foreground at camera pixels
    touched by the cleaned full-resolution mask, including narrow edges.
    """
    if mask.ndim != 2 or not 0 < width <= mask.shape[1] or not 0 < height <= mask.shape[0]:
        raise ValueError('native mapping requires camera dimensions no larger than the model mask')
    if component_subsampling not in (1, 2):
        raise ValueError('component_subsampling must be 1 or 2')
    if component_threshold > 0:
        stride = component_subsampling
        if mask.shape[0] % stride or mask.shape[1] % stride:
            raise ValueError('component filtering requires dimensions divisible by its subsampling')
        small = np.ascontiguousarray(mask[::stride, ::stride] > 0, dtype=np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(small, connectivity=4)
        keep = np.zeros(count, dtype=np.uint8)
        keep[1:] = (stats[1:, cv2.CC_STAT_AREA] >= max(1, component_threshold // (stride * stride)))
        clean = keep[labels]
        if stride > 1:
            clean = np.repeat(np.repeat(clean, stride, axis=0), stride, axis=1)
    else:
        clean = (mask > 0).astype(np.uint8)
    if clean.shape == (height, width):
        return np.ascontiguousarray(clean * 255, dtype=np.uint8)
    coverage = cv2.resize(clean.astype(np.float32), (width, height), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray((coverage > 0).astype(np.uint8) * 255)


def stamp_key(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class NativeMaskNode(Node):
    def __init__(self):
        super().__init__('native_mask_node')
        cv2.setNumThreads(1)
        self.declare_parameter('component_threshold', 2000)
        self.declare_parameter('component_subsampling', 1)
        self.threshold = self.get_parameter('component_threshold').value
        self.subsampling = self.get_parameter('component_subsampling').value
        if self.subsampling not in (1, 2):
            raise ValueError('component_subsampling must be 1 or 2')
        self.infos = OrderedDict()
        self.pending = OrderedDict()
        self.mask_pub = self.create_publisher(
            Image, '/camera0/segmentation/people_mask_native', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera0/segmentation/camera_info_native', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera0/camera/color/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera0/segmentation/people_mask',
                                 self.on_mask, QoSProfile(depth=10))

    def on_info(self, info):
        key = stamp_key(info.header)
        self.infos[key] = info
        while len(self.infos) > 120:
            self.infos.popitem(last=False)
        mask = self.pending.pop(key, None)
        if mask is not None:
            self.publish(mask, info)

    def on_mask(self, mask):
        key = stamp_key(mask.header)
        info = self.infos.get(key)
        if info is not None:
            self.publish(mask, info)
        else:
            self.pending[key] = mask
            while len(self.pending) > 5:
                self.pending.popitem(last=False)

    def publish(self, mask, info):
        if mask.encoding not in ('mono8', '8UC1') or mask.header.frame_id != info.header.frame_id:
            self.get_logger().error('mask encoding or camera frame mismatch', throttle_duration_sec=5)
            return
        pixels = np.frombuffer(mask.data, dtype=np.uint8).reshape(mask.height, mask.step)[:, :mask.width]
        try:
            native = prepare_native_mask(pixels, info.width, info.height,
                                         self.threshold, self.subsampling)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)
            return
        result = Image()
        result.header = deepcopy(mask.header)
        result.height, result.width = native.shape
        result.encoding = 'mono8'
        result.step = result.width
        result.data = native.tobytes()
        calibration = deepcopy(info)
        calibration.header = deepcopy(mask.header)
        self.info_pub.publish(calibration)
        self.mask_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = NativeMaskNode()
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
