#!/usr/bin/env python3
"""Align the RealSense camera frame with the Panda base frame (panda_link0).

Ports the calibration approach from the lab's franka-open-mp-integration
repo (mp_with_camera/scripts/aruco.py) to ROS2:

  1. Detect a single fixed ArUco marker in the color image and compute
     T_marker_from_camera (transforms a point expressed in the camera frame
     into the marker frame).
  2. Combine it with T_robot, the fixed marker-to-panda_link0 transform that
     was measured once for the lab's marker jig (rotation about X + a fixed
     translation offset).
  3. Broadcast the result as a TF into the robot base frame.

Note on fidelity to the lab script: ~num_samples:=1 reproduces the same
formula as a single-shot lock (grab one detection, no averaging), not an
identical result -- the lab script ran the camera at 1280x720 directly via
pyrealsense2, while this node's default launch profile is 640x480 (to fit
inside a USB2 link). Lower resolution means fewer pixels across the marker's
edges for cv2.aruco's corner refinement, so the pose estimate is noisier at
640x480 than the original's 1280x720 even with the same formula. Raise
~marker_length's supporting resolution back up once on USB3.

Two modes (~mode):
  - 'static' (default): average ~num_samples detections, lock, and publish
    once via a StaticTransformBroadcaster. This is what you want once the
    camera is actually bolted down -- a static transform never goes stale,
    so there is nothing for 'dynamic' mode to buy you in that case.
  - 'dynamic': recompute the transform whenever the marker is visible, with
    light smoothing, for live feedback while you are still adjusting the
    physical camera/marker mount. It also keeps re-publishing the last known
    transform through occlusion instead of letting the TF go stale the
    moment detection drops out (echoing "3.3 Collision-Risk Assessment Under
    ArUco Marker Occlusion" in the reference paper) -- but note this node
    only keeps the camera->base TF alive, not a downstream quantity like a
    human-robot distance signal; a consumer would still compute that itself
    from whatever it's tracking, using this TF to get it into the robot
    frame.

TF frame ownership: this node's own extrinsic is only valid at the camera's
*optical* frame (cv2.aruco's pose is defined in that convention), but
realsense2_camera already publishes its own static tree rooted at
~camera_mount_frame (default 'camera_link') down to that optical frame, e.g.
camera_link -> camera_color_frame -> camera_color_optical_frame. Publishing
straight to the optical frame as this node's child would give that frame two
different parents (this node's and realsense2_camera's), which tf2 cannot
represent consistently. So when ~camera_mount_frame is set, this node looks
up the camera driver's own (mount -> optical) static transform via tf2 and
republishes robot_base_frame -> camera_mount_frame instead, leaving
realsense2_camera's own subtree under camera_mount_frame untouched:
  panda_link0 -> camera_link -> camera_color_frame -> camera_color_optical_frame
Leave ~camera_mount_frame empty only for standalone use with no other TF
publisher owning the optical frame.

If your physical marker mount differs from the lab jig, override
~rotation_deg / ~translation via parameters instead of editing the code.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformListener,
)

# Same marker convention as the lab's aruco.py.
ARUCO_DICT_NAME = 'DICT_6X6_250'
DEFAULT_MARKER_ID = 30
DEFAULT_MARKER_LENGTH_M = 0.05

# Fixed marker(panda_link0)-frame <- marker-frame transform measured for the
# lab's jig: 90 deg rotation about X plus a fixed translation offset.
DEFAULT_ROTATION_DEG = 90.0
DEFAULT_TRANSLATION = [0.0, -0.25, -0.02]


def make_transform(r: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Pack a 3x3 rotation and translation into a 4x4 homogeneous transform."""
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def average_transforms(transforms, weights=None) -> np.ndarray:
    """(Weighted) average of a list of 4x4 transforms.

    Rotations are averaged on SO(3) (scipy re-orthonormalizes internally, so
    this also covers a single already-noisy input). weights=None means a
    uniform mean; used for the 'static' batch lock (uniform) and the
    'dynamic' EMA blend (weights=[1 - alpha, alpha]) alike.
    """
    rotations = Rotation.from_matrix(np.stack([t[:3, :3] for t in transforms]))
    avg_rotation = rotations.mean(weights=weights).as_matrix()
    avg_translation = np.average(
        np.stack([t[:3, 3] for t in transforms]), axis=0, weights=weights)
    return make_transform(avg_rotation, avg_translation)


def invert_rigid_transform(t: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid (rotation + translation) transform."""
    r = t[:3, :3]
    p = t[:3, 3]
    return make_transform(r.T, -r.T @ p)


def transform_msg_to_matrix(msg: TransformStamped) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4 homogeneous transform."""
    tr = msg.transform.translation
    q = msg.transform.rotation
    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    return make_transform(r, np.array([tr.x, tr.y, tr.z]))


class ArucoCameraAligner(Node):

    def __init__(self):
        super().__init__('aruco_camera_align')

        self.declare_parameter('image_topic', '/camera0/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera0/camera/color/camera_info')
        self.declare_parameter('marker_id', DEFAULT_MARKER_ID)
        self.declare_parameter('marker_length', DEFAULT_MARKER_LENGTH_M)
        self.declare_parameter('robot_base_frame', 'panda_link0')
        self.declare_parameter('camera_frame', '')  # '' -> use camera_info.header.frame_id
        # Root of the camera driver's own static TF subtree (its
        # base_frame_id, e.g. '<camera_name>_link'). When set, this node
        # publishes robot_base_frame -> camera_mount_frame (looked up and
        # composed against the driver's own mount->optical static
        # transform) instead of claiming the optical frame directly and
        # conflicting with the driver's own broadcast of it. See module
        # docstring "TF frame ownership".
        self.declare_parameter('camera_mount_frame', 'camera_link')
        self.declare_parameter('rotation_deg', DEFAULT_ROTATION_DEG)
        self.declare_parameter('translation', DEFAULT_TRANSLATION)
        # Frames to average before locking the calibration ('static' mode
        # only). 1 replicates the lab script exactly (single-shot lock).
        self.declare_parameter('num_samples', 15)
        self.declare_parameter('show_image', True)
        # 'static': lock once after num_samples and stop. 'dynamic': keep
        # detecting and re-publishing a live (smoothed) TF every frame.
        self.declare_parameter('mode', 'static')
        # EMA smoothing factor for 'dynamic' mode: 1.0 = no smoothing (use
        # the raw per-frame detection), smaller = smoother but laggier.
        self.declare_parameter('smoothing_alpha', 0.3)
        # 'dynamic' mode: rate at which the held transform is re-published,
        # independent of whether the marker is currently visible. Keeps the
        # TF fresh (non-stale) through occlusion instead of only publishing
        # on detection.
        self.declare_parameter('publish_rate_hz', 15.0)
        # 'dynamic' mode: log a reminder at most this often while occluded.
        self.declare_parameter('occlusion_warn_period_sec', 5.0)

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_length = self.get_parameter('marker_length').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.camera_frame_override = self.get_parameter('camera_frame').value
        self.num_samples = max(1, int(self.get_parameter('num_samples').value))
        self.show_image = self.get_parameter('show_image').value
        self.mode = self.get_parameter('mode').value
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.occlusion_warn_period_sec = float(
            self.get_parameter('occlusion_warn_period_sec').value)
        self.camera_mount_frame = self.get_parameter('camera_mount_frame').value
        if self.mode not in ('static', 'dynamic'):
            raise ValueError(f"~mode must be 'static' or 'dynamic', got {self.mode!r}")
        if self.mode == 'dynamic' and self.publish_rate_hz <= 0.0:
            raise ValueError(
                f'~publish_rate_hz must be > 0 in dynamic mode, got {self.publish_rate_hz}')
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError(f'~smoothing_alpha must be in (0, 1], got {self.smoothing_alpha}')

        rotation_deg = self.get_parameter('rotation_deg').value
        translation = np.array(self.get_parameter('translation').value, dtype=float)
        self.t_robot = make_transform(
            Rotation.from_euler('x', np.deg2rad(rotation_deg)).as_matrix(), translation)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, ARUCO_DICT_NAME))
        # cv2 4.5.x does not have the DetectorParameters() constructor used
        # by the lab script (that's a 4.7+ API) -- use the matching old API.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = self.camera_frame_override
        self.samples = []
        self.smoothed_t = None
        # Occlusion bookkeeping ('dynamic' mode). Marker is considered
        # visible iff occluded_since is None -- kept as the single source of
        # truth instead of a separate "visible" flag.
        self.occluded_since = None
        self.last_occlusion_warn = None
        # Static-mode: an averaged transform that's locked but not yet
        # broadcast because the camera driver's mount->optical static TF
        # wasn't available yet -- retried by static_retry_timer instead of
        # being silently dropped.
        self.pending_static_transform = None
        self.static_retry_timer = None
        self.last_mount_wait_warn = None

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        # Cache of the camera driver's own static (camera_mount_frame ->
        # camera_frame) transform, looked up lazily once camera_frame is
        # known and the driver's static TF has actually been received.
        self.t_mount_from_optical = None
        self.tf_buffer = None
        self.tf_listener = None
        if self.camera_mount_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().warn(
                '~camera_mount_frame is empty -- publishing directly to the optical frame. '
                'This will conflict with realsense2_camera\'s own static TF tree if anything '
                'else already publishes that frame as a child (e.g. camera_link -> ... -> '
                'optical). Set ~camera_mount_frame unless you know this node is the sole '
                'publisher of that frame.')

        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.create_subscription(
            CameraInfo, info_topic, self.camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic, self.image_cb, qos_profile_sensor_data)

        if self.mode == 'dynamic':
            self.create_timer(1.0 / self.publish_rate_hz, self.publish_timer_cb)

        self.get_logger().info(
            f'[{self.mode}] Waiting for marker id={self.marker_id} ({ARUCO_DICT_NAME}, '
            f'{self.marker_length} m) on "{image_topic}" / "{info_topic}"...')

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=float)
            if not self.camera_frame_override:
                self.camera_frame = msg.header.frame_id
            self.get_logger().info(
                f'Resolved camera_frame="{self.camera_frame}" from camera_info. '
                f'TF will be published as {self.robot_base_frame} -> {self.camera_frame}.')

    def image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return
        if self.mode == 'static' and len(self.samples) >= self.num_samples:
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(
            image, self.aruco_dict, parameters=self.aruco_params)

        detected = ids is not None and self.marker_id in ids.flatten()

        if detected:
            idx = np.where(ids.flatten() == self.marker_id)[0][0]
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[idx]], self.marker_length, self.camera_matrix, self.dist_coeffs)
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]

            r, _ = cv2.Rodrigues(rvec)
            r_inv = r.T
            t_inv = -r_inv @ tvec
            t_marker_from_camera = make_transform(r_inv, t_inv)

            if self.show_image:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                cv2.drawFrameAxes(
                    image, self.camera_matrix, self.dist_coeffs, rvec, tvec,
                    self.marker_length * 0.5)

            if self.mode == 'static':
                self.samples.append(t_marker_from_camera)
                self.get_logger().info(
                    f'Sample {len(self.samples)}/{self.num_samples} captured.')
                if len(self.samples) >= self.num_samples:
                    self.lock_calibration()
            else:
                self.update_dynamic(t_marker_from_camera)
        elif self.mode == 'dynamic':
            self.note_occlusion()

        if self.show_image:
            cv2.imshow('aruco_align', image)
            cv2.waitKey(1)

    def lock_calibration(self):
        t_marker_from_camera = average_transforms(self.samples)
        t_base_from_camera = self.t_robot @ t_marker_from_camera

        np.set_printoptions(precision=6, suppress=True)
        self.get_logger().info(
            f'Locked calibration. {self.robot_base_frame} <- {self.camera_frame} (optical):\n'
            f'{t_base_from_camera}')

        # broadcast_tf can fail here if ~camera_mount_frame is set but the
        # camera driver hasn't published its static (mount -> optical)
        # transform yet -- retry instead of silently losing the lock.
        self.pending_static_transform = t_base_from_camera
        self.try_publish_pending_static()
        if self.pending_static_transform is not None and self.static_retry_timer is None:
            self.static_retry_timer = self.create_timer(0.5, self.try_publish_pending_static)

    def try_publish_pending_static(self):
        if self.pending_static_transform is None:
            return
        if self.broadcast_tf(
                self.static_tf_broadcaster, self.pending_static_transform,
                self.get_clock().now().to_msg()):
            self.get_logger().info(
                f'Broadcast static TF {self.robot_base_frame} -> '
                f'{self.camera_mount_frame or self.camera_frame}.')
            self.pending_static_transform = None
            if self.static_retry_timer is not None:
                self.static_retry_timer.cancel()
                self.static_retry_timer = None

    def update_dynamic(self, t_marker_from_camera: np.ndarray):
        """Update the held transform from a fresh detection.

        Only called when the marker is actually visible -- occlusion simply
        means this is not called, so publish_timer_cb keeps re-broadcasting
        whatever was last computed here (paper section 3.3: reuse the stored
        marker-based transformation while the marker is occluded).
        """
        t_base_from_camera = self.t_robot @ t_marker_from_camera

        if self.smoothed_t is None:
            self.smoothed_t = t_base_from_camera
        else:
            alpha = self.smoothing_alpha
            self.smoothed_t = average_transforms(
                [self.smoothed_t, t_base_from_camera], weights=[1 - alpha, alpha])

        if self.occluded_since is not None:
            occluded_for = (self.get_clock().now() - self.occluded_since).nanoseconds / 1e9
            self.get_logger().info(f'Marker reacquired after {occluded_for:.1f}s occlusion.')
        self.occluded_since = None

    def note_occlusion(self):
        """Record that the marker was not found this frame ('dynamic' only).

        Doesn't touch smoothed_t -- just tracks/logs the occlusion so
        publish_timer_cb keeps holding the last known transform instead of
        the TF going stale.
        """
        if self.smoothed_t is None:
            return  # never got an initial fix yet, nothing to hold onto
        now = self.get_clock().now()
        if self.occluded_since is None:
            self.occluded_since = now
            self.last_occlusion_warn = now
            self.get_logger().warn(
                'Marker occluded -- holding last known transform until it reappears.')
        elif self.occlusion_warn_period_sec > 0:
            since_warn = (now - self.last_occlusion_warn).nanoseconds / 1e9
            if since_warn >= self.occlusion_warn_period_sec:
                occluded_for = (now - self.occluded_since).nanoseconds / 1e9
                self.get_logger().warn(
                    f'Marker still occluded ({occluded_for:.0f}s) -- still holding last '
                    'known transform.')
                self.last_occlusion_warn = now

    def publish_timer_cb(self):
        if self.smoothed_t is not None:
            self.broadcast_tf(
                self.dynamic_tf_broadcaster, self.smoothed_t, self.get_clock().now().to_msg())

    def resolve_publish_target(self, t_base_from_optical: np.ndarray):
        """Return (child_frame, matrix) to publish.

        Returns (None, None) if the camera driver's own static extrinsic
        isn't available yet (only relevant when ~camera_mount_frame is set
        -- see the module docstring's "TF frame ownership" section).
        """
        if not self.camera_mount_frame:
            return self.camera_frame, t_base_from_optical

        if self.t_mount_from_optical is None:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.camera_mount_frame, self.camera_frame, Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as exc:
                now = self.get_clock().now()
                if (self.last_mount_wait_warn is None
                        or (now - self.last_mount_wait_warn).nanoseconds / 1e9 >= 5.0):
                    self.get_logger().warn(
                        f'Waiting for the camera driver\'s static transform '
                        f'{self.camera_mount_frame} -> {self.camera_frame} ({exc}); not '
                        'publishing this node\'s TF yet.')
                    self.last_mount_wait_warn = now
                return None, None
            self.t_mount_from_optical = transform_msg_to_matrix(tf_msg)
            self.get_logger().info(
                f'Cached camera driver static transform {self.camera_mount_frame} -> '
                f'{self.camera_frame}. Publishing {self.robot_base_frame} -> '
                f'{self.camera_mount_frame} from here on.')

        t_base_from_mount = t_base_from_optical @ invert_rigid_transform(self.t_mount_from_optical)
        return self.camera_mount_frame, t_base_from_mount

    def broadcast_tf(self, broadcaster, t_base_from_optical: np.ndarray, stamp) -> bool:
        child_frame, t = self.resolve_publish_target(t_base_from_optical)
        if child_frame is None:
            return False

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.robot_base_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(t[0, 3])
        tf_msg.transform.translation.y = float(t[1, 3])
        tf_msg.transform.translation.z = float(t[2, 3])
        qx, qy, qz, qw = Rotation.from_matrix(t[:3, :3]).as_quat()
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        broadcaster.sendTransform(tf_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCameraAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
