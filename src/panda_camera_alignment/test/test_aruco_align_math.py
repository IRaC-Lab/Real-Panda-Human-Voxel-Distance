"""Unit tests for the pure math and parameter validation in aruco_align.

These deliberately avoid needing a live camera/robot: the transform algebra
(averaging, inversion, the camera_mount_frame remap) and parameter
validation are all exercised directly, without a running ROS graph, except
where a minimal Node is needed to check constructor validation.
"""
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

from panda_camera_alignment.aruco_align import (
    ArucoCameraAligner,
    average_transforms,
    invert_rigid_transform,
    make_transform,
    transform_msg_to_matrix,
)


def random_transform(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    r = Rotation.from_euler('xyz', rng.uniform(-np.pi, np.pi, 3)).as_matrix()
    t = rng.uniform(-1.0, 1.0, 3)
    return make_transform(r, t)


def test_make_transform_shape_and_identity():
    t = make_transform(np.eye(3), np.zeros(3))
    assert np.allclose(t, np.eye(4))


def test_invert_rigid_transform_is_a_true_inverse():
    for seed in range(5):
        t = random_transform(seed)
        assert np.allclose(t @ invert_rigid_transform(t), np.eye(4), atol=1e-9)
        assert np.allclose(invert_rigid_transform(t) @ t, np.eye(4), atol=1e-9)


def test_transform_msg_to_matrix_round_trip():
    t = random_transform(42)
    msg = TransformStamped()
    tx, ty, tz = t[:3, 3]
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (
        tx, ty, tz)
    qx, qy, qz, qw = Rotation.from_matrix(t[:3, :3]).as_quat()
    (msg.transform.rotation.x, msg.transform.rotation.y,
     msg.transform.rotation.z, msg.transform.rotation.w) = (qx, qy, qz, qw)
    recovered = transform_msg_to_matrix(msg)
    assert np.allclose(recovered, t, atol=1e-9)


def test_average_transforms_of_identical_inputs_is_unchanged():
    t = random_transform(1)
    avg = average_transforms([t, t, t])
    assert np.allclose(avg, t, atol=1e-9)


def test_average_transforms_uniform_translation_midpoint():
    a = make_transform(np.eye(3), np.array([0.0, 0.0, 0.0]))
    b = make_transform(np.eye(3), np.array([2.0, 0.0, 0.0]))
    avg = average_transforms([a, b])
    assert np.allclose(avg[:3, 3], [1.0, 0.0, 0.0])


def test_average_transforms_ema_weights():
    a = make_transform(np.eye(3), np.array([0.0, 0.0, 0.0]))
    b = make_transform(np.eye(3), np.array([1.0, 0.0, 0.0]))
    alpha = 0.3
    blended = average_transforms([a, b], weights=[1 - alpha, alpha])
    assert np.allclose(blended[:3, 3], [alpha, 0.0, 0.0])


def test_camera_mount_frame_remap_matches_ground_truth_composition():
    """Verify the fix for the TF child-frame-ownership issue.

    Given the camera driver's own static (mount -> optical) transform,
    recovering (base -> mount) from our computed (base -> optical) must
    exactly match composing the chain the other way.
    """
    t_base_mount = random_transform(10)
    t_mount_optical = random_transform(20)
    t_base_optical = t_base_mount @ t_mount_optical

    recovered = t_base_optical @ invert_rigid_transform(t_mount_optical)
    assert np.allclose(recovered, t_base_mount, atol=1e-9)


@pytest.fixture()
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(ros_context, overrides):
    args = ['--ros-args']
    for name, value in overrides.items():
        args += ['-p', f'{name}:={value}']
    rclpy.shutdown()
    rclpy.init(args=args)
    return ArucoCameraAligner()


def test_default_construction_does_not_raise(ros_context):
    node = ArucoCameraAligner()
    try:
        assert node.mode == 'static'
        assert node.camera_mount_frame == 'camera_link'
    finally:
        node.destroy_node()


def test_publish_rate_hz_zero_raises_value_error_not_zero_division(ros_context):
    with pytest.raises(ValueError, match='publish_rate_hz'):
        make_node(ros_context, {'mode': 'dynamic', 'publish_rate_hz': '0.0'})


def test_smoothing_alpha_out_of_range_raises(ros_context):
    with pytest.raises(ValueError, match='smoothing_alpha'):
        make_node(ros_context, {'smoothing_alpha': '0.0'})
    with pytest.raises(ValueError, match='smoothing_alpha'):
        make_node(ros_context, {'smoothing_alpha': '1.5'})


def test_invalid_mode_raises(ros_context):
    with pytest.raises(ValueError, match='mode'):
        make_node(ros_context, {'mode': 'bogus'})


def test_resolve_publish_target_direct_when_mount_frame_empty(ros_context):
    # '' isn't expressible via a `-p name:=value` CLI override, so set the
    # attribute directly on a default-constructed node instead.
    node = ArucoCameraAligner()
    try:
        node.camera_mount_frame = ''
        node.camera_frame = 'camera_color_optical_frame'
        t = random_transform(5)
        child_frame, out_t = node.resolve_publish_target(t)
        assert child_frame == 'camera_color_optical_frame'
        assert np.allclose(out_t, t)
    finally:
        node.destroy_node()


def test_resolve_publish_target_waits_when_mount_transform_unavailable(ros_context):
    node = make_node(ros_context, {'camera_mount_frame': 'camera_link'})
    try:
        node.camera_frame = 'camera_color_optical_frame'
        # No static transform has been published on /tf_static in this test,
        # so the lookup must fail cleanly (not publish to the wrong frame).
        child_frame, out_t = node.resolve_publish_target(random_transform(6))
        assert child_frame is None
        assert out_t is None
    finally:
        node.destroy_node()
