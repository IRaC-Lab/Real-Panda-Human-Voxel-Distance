"""Small behavioral checks for the Panda/human distance filter."""

from types import SimpleNamespace

from builtin_interfaces.msg import Duration, Time
import numpy as np
from sensor_msgs.msg import CameraInfo

from my_people_nvblox_bringup.closest_panda_human_voxels import (
    ClosestPandaHumanVoxels, DistanceFilter)


def test_distance_filter():
    distance_filter = DistanceFilter(0.3, 0.7, 3)

    assert distance_filter.update(1.5) == (1.5, 'VALID')
    assert distance_filter.update(0.9) == (0.9, 'VALID')
    assert distance_filter.update(0.25) == (0.25, 'VALID')
    assert distance_filter.update(1.5) == (1.5, 'VALID')
    assert distance_filter.update(0.02) == (
        1.5, 'REJECTED_NEAR_ZERO_JUMP')
    assert distance_filter.rejects_candidate(0.02)
    assert not distance_filter.rejects_candidate(1.6)

    near_false_positive = (None, None, 0.02, None)
    real_person = (None, None, 1.6, None)
    assert ClosestPandaHumanVoxels.first_accepted_candidate(
        [near_false_positive, real_person], distance_filter) is real_person


def test_rejected_hold_expires_and_stays_empty():
    distance_filter = DistanceFilter(0.3, 0.7, 2)

    assert distance_filter.update(1.5) == (1.5, 'VALID')
    assert distance_filter.update(0.02) == (
        1.5, 'REJECTED_NEAR_ZERO_JUMP')
    assert distance_filter.update(0.02) == (None, 'EXPIRED_TIMEOUT')
    assert distance_filter.update(0.02) == (None, 'NO_DETECTION')


def test_edge_exit_expires_immediately():
    distance_filter = DistanceFilter(0.3, 0.7, 10)

    assert distance_filter.update(1.5, measurement_at_edge=True) == (
        1.5, 'VALID')
    assert distance_filter.update(0.02) == (
        None, 'EXPIRED_AT_IMAGE_EDGE')


def test_human_points_touch_image_edge():
    node = ClosestPandaHumanVoxels.__new__(ClosestPandaHumanVoxels)
    node.human_frame = 'camera'
    node.image_edge_margin_ratio = 0.1
    node.camera_info = CameraInfo()
    node.camera_info.header.frame_id = 'camera'
    node.camera_info.width = 640
    node.camera_info.height = 480
    node.camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0,
                          0.0, 0.0, 1.0]

    assert not node.human_points_touch_image_edge(
        np.asarray([[0.0, 0.0, 2.0]]))
    assert node.human_points_touch_image_edge(
        np.asarray([[1.2, 0.0, 2.0]]))


def test_rejected_measurement_keeps_filtered_line():
    published = []
    node = ClosestPandaHumanVoxels.__new__(ClosestPandaHumanVoxels)
    node.human_frame = 'odom'
    node.marker_size_m = 0.05
    node.marker_lifetime = Duration()
    node.marker_publisher = SimpleNamespace(publish=published.append)

    node.publish_markers(
        np.asarray([0.0, 0.0, 0.0]), np.asarray([1.5, 0.0, 0.0]),
        0.02, 1.5, 'REJECTED_NEAR_ZERO_JUMP', Time())

    line = published[0].markers[2]
    assert line.type == line.LINE_LIST
    assert line.color.r == 1.0 and line.color.g == 1.0
    assert [(point.x, point.y, point.z) for point in line.points] == [
        (0.0, 0.0, 0.0), (1.5, 0.0, 0.0)]


def test_robot_component_filter_keeps_separate_person():
    robot_false_positive = np.asarray([
        [0.025, 0.025, 0.025],
        [0.075, 0.025, 0.025],
        [0.125, 0.025, 0.025],
    ])
    person = np.asarray([
        [1.025, 0.025, 0.025],
        [1.075, 0.025, 0.025],
        [1.125, 0.025, 0.025],
    ])

    filtered, removed = ClosestPandaHumanVoxels.exclude_robot_components(
        np.concatenate([robot_false_positive, person]),
        np.asarray([[0.075, 0.025, 0.025]]), np.asarray([0.15]),
        0.05, 0.3)

    assert removed == 3
    assert np.allclose(filtered, person)


def test_robot_component_filter_keeps_partial_contact():
    person_touching_robot = np.asarray([
        [0.125, 0.025, 0.025],
        [0.175, 0.025, 0.025],
        [0.225, 0.025, 0.025],
        [0.275, 0.025, 0.025],
    ])

    filtered, removed = ClosestPandaHumanVoxels.exclude_robot_components(
        person_touching_robot,
        np.asarray([[0.075, 0.025, 0.025]]), np.asarray([0.04]),
        0.05, 0.3)

    assert removed == 0
    assert np.allclose(filtered, person_touching_robot)
