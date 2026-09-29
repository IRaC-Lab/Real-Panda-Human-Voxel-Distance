from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import Transform

from my_people_nvblox_bringup.panda_voxel_classifier import (
    PandaVoxelClassifier,
)


def test_voxel_templates_follow_link_transform_and_deduplicate():
    transform = Transform()
    transform.translation.x = 0.05
    transform.rotation.w = 1.0
    indices = PandaVoxelClassifier.voxel_indices_from_templates(
        {'link': np.asarray([[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]])},
        {'link': transform}, 0.05)

    assert indices.tolist() == [[1, 0, 0]]


def test_static_sphere_markers_follow_link_frames():
    published = []
    node = PandaVoxelClassifier.__new__(PandaVoxelClassifier)
    node.spheres = [('panda_link1', (0.0, 0.0, 0.0), 0.1)]
    node.sphere_alpha = 0.18
    node.sphere_publisher = SimpleNamespace(publish=published.append)

    node.publish_spheres()

    marker = published[0].markers[0]
    assert marker.header.frame_id == 'panda_link1'
    assert marker.frame_locked
    assert marker.scale.x == 0.2
