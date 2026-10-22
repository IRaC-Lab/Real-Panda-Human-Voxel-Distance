"""Run with the workspace sourced: python3 test/check_perception_speed.py."""
from types import SimpleNamespace as NS
from collections import OrderedDict
import json
import math
import time

import numpy as np
from geometry_msgs.msg import Point32
from nvblox_msgs.msg import VoxelBlock, VoxelBlockLayer, Index3D
from scipy.spatial import cKDTree
from rclpy.time import Time
from sensor_msgs.msg import TimeReference

from my_people_nvblox_bringup.closest_panda_human_voxels import (
    ClosestPandaHumanVoxels, DistanceFilter)
from panda_pick_place.native_mask_node import prepare_native_mask


def check_exact_nearest_pair():
    node = NS(chunk_size=23)
    for count in (1, 17, 600):
        rng = np.random.default_rng(count)
        humans = rng.normal(size=(count, 3))
        pandas = rng.normal(size=(389, 3))
        expected = np.linalg.norm(humans[:, None] - pandas[None, :], axis=2).min()
        for tree in (None, cKDTree(pandas)):
            human, panda, distance = ClosestPandaHumanVoxels.closest_pair(node, humans, pandas, tree)
            assert np.isclose(distance, expected, rtol=1e-13, atol=1e-13)
            assert np.isclose(np.linalg.norm(human - panda), expected)
    point = np.asarray([[0.0, 0.0, 0.0]])
    assert ClosestPandaHumanVoxels.closest_pair(node, point, point)[2] == 0


def make_node():
    node = ClosestPandaHumanVoxels.__new__(ClosestPandaHumanVoxels)
    node.human_blocks = {(0, 0, 0): np.asarray([[1.025, .025, .025]])}
    node.panda_blocks = {(0, 0, 0): np.asarray([[.025, .025, .025]])}
    node.human_frame = node.panda_frame = 'world'
    node.human_voxel_size_m = .05
    node.use_spatial_human_filter = False
    node.chunk_size = 512
    node.last_status = ''
    node.latest_result = None
    clock = [1.0]
    node.get_clock = lambda: NS(now=lambda: Time(seconds=clock[0]))
    node.get_logger = lambda: NS(info=lambda *a: None, warning=lambda *a: None)
    node.distance_filter = DistanceFilter(.3, 3.0, .1, 2.0)
    node.last_valid_human_point = node.last_valid_panda_point = None
    node.camera_info = None
    node.input_timeout_s = .5
    node.last_human_received = node.last_panda_received = time.monotonic()
    node.human_layer_stamp_ns = node.panda_layer_stamp_ns = 1_000_000_000
    node.layer_sources = OrderedDict({1_000_000_000: {'depth': 1_000_000_000, 'rgb': 1_000_000_000}})
    raw, filtered, markers, timings = [], [], [], []
    node.raw_distance_publisher = NS(publish=lambda msg: raw.append(msg.data))
    node.distance_publisher = NS(publish=lambda msg: filtered.append(msg.data))
    node.marker_publisher = NS(publish=markers.append)
    node.publish_markers = lambda *args: markers.append(args)
    node.timing_publisher = NS(get_subscription_count=lambda: 1,
                               publish=lambda msg: timings.append(json.loads(msg.data)))
    node.compute_guard = NS(trigger=lambda: None)
    return node, clock, raw, filtered, markers, timings


def check_calculation_and_publication_are_independent():
    node, clock, raw, filtered, markers, timings = make_node()
    for x in np.linspace(1.0, 2.0, 30):
        node.human_blocks[(0, 0, 0)][0, 0] = x
        node.compute_distance()
    assert not raw and not filtered and not markers
    assert np.isclose(node.distance_filter.filtered_distance, 1.975)
    assert len(timings) == 30  # Final filtered results exist before any 5 Hz output.
    last_update = node.distance_filter.last_update_time
    node.publish_latest()
    node.publish_latest()
    assert len(raw) == len(filtered) == len(markers) == 2
    assert np.isclose(raw[-1], 1.975)
    assert node.distance_filter.last_update_time == last_update
    sample = timings[-1]
    assert sample['depth_source_ns'] == sample['rgb_source_ns'] == 1_000_000_000
    assert sample['compute_finished_mono_ns'] >= sample['compute_started_mono_ns']
    assert sample['compute_ms'] >= 0
    assert all(value >= 0 for value in sample['stage_ms'].values())
    assert math.isclose(sum(sample['stage_ms'].values()), sample['compute_ms'], abs_tol=1e-9)
    node.human_blocks.clear()
    node.compute_distance()
    node.publish_latest()
    assert math.isnan(filtered[-1]) and node.distance_filter.filtered_distance is None


def check_stale_sources_cannot_be_kept_alive_by_fresh_map_messages():
    node, clock, raw, filtered, markers, timings = make_node()
    node.compute_distance()
    clock[0] = 1.6  # Mapping still publishes, but original RGB-D frames stopped.
    node.last_human_received = node.last_panda_received = time.monotonic()
    node.publish_latest()
    assert math.isnan(raw[-1]) and math.isnan(filtered[-1])
    assert node.latest_result is None and node.distance_filter.filtered_distance is None
    assert markers[-1].markers[0].action == markers[-1].markers[0].DELETEALL
    node, clock, raw, filtered, markers, timings = make_node()
    node.compute_distance()
    node.last_panda_received -= 1.0
    node.publish_latest()
    assert math.isnan(filtered[-1])


def check_reference_pairing_and_single_component_pass():
    node, clock, raw, filtered, markers, timings = make_node()
    node.layer_sources.clear()
    node.compute_distance()
    assert node.latest_result is None
    reference = TimeReference()
    reference.header.stamp = Time(seconds=1.0).to_msg()
    reference.time_ref = Time(seconds=1.0).to_msg()
    node.source_reference_callback(reference, 'rgb')
    node.compute_distance()
    assert node.latest_result is None
    node.source_reference_callback(reference, 'depth')
    node.use_spatial_human_filter = True
    node.robot_component_overlap_ratio = .3
    node.sphere_specs = [('world', (9., 9., 9.), .1)]
    node.spheres_in_human_frame = lambda: (np.array([[9., 9., 9.]]), np.array([.1]))
    original = ClosestPandaHumanVoxels.connected_component_indices
    calls = []
    def counted(*args):
        calls.append(True)
        return original(*args)
    ClosestPandaHumanVoxels.connected_component_indices = staticmethod(counted)
    try:
        node.compute_distance()
    finally:
        ClosestPandaHumanVoxels.connected_component_indices = staticmethod(original)
    assert node.latest_result is not None and len(calls) == 1
    for index in range(300):
        reference.header.stamp = Time(seconds=2.0 + index).to_msg()
        node.source_reference_callback(reference, 'depth')
    assert len(node.layer_sources) == 256


def check_incremental_updates_and_invalid_centers():
    node, clock, raw, filtered, markers, timings = make_node()
    node.human_blocks.clear()
    triggers = []
    node.compute_guard = NS(trigger=lambda: triggers.append(True))
    msg = VoxelBlockLayer()
    msg.header.frame_id = 'world'
    msg.header.stamp = Time(seconds=1.0).to_msg()
    msg.voxel_size_m = .05
    msg.block_indices = [Index3D()]
    block = VoxelBlock()
    block.centers = [Point32(x=1.0), Point32(x=float('nan'))]
    msg.blocks = [block]
    node.human_callback(msg)
    assert len(triggers) == 1
    assert node.human_blocks[(0, 0, 0)].tolist() == [[1.0, 0.0, 0.0]]
    node.compute_distance()
    node.panda_frame = 'world'
    msg.clear = True
    node.panda_callback(msg)
    assert node.latest_result is not None  # A normal full snapshot does not reset the filter.
    block.centers = []
    msg.blocks = [block]
    node.human_callback(msg)
    assert not node.human_blocks and node.latest_result is None
    msg.header.frame_id = 'new_world'
    block.centers = [Point32(x=2.0)]
    msg.blocks = [block]
    node.human_callback(msg)
    assert node.latest_result is None and node.human_frame == 'new_world'
    assert node.distance_filter.filtered_distance is None


def check_mask_edges_and_component_filter():
    mask = np.zeros((16, 16), dtype=np.uint8)
    mask[2:10, 2:10] = 255
    mask[15, 15] = 255
    expected = mask.copy()
    expected[15, 15] = 0
    for stride in (1, 2):
        actual = prepare_native_mask(mask, 16, 16, 16, stride)
        assert np.array_equal(actual, expected)
        assert actual.dtype == np.uint8 and actual.flags.c_contiguous
    edge = np.zeros((16, 16), dtype=np.uint8)
    edge[1, 1] = 255
    reduced = prepare_native_mask(edge, 8, 8, 0, 1)
    assert reduced.sum() == 255 and reduced[0, 0] == 255



def check_components_preserve_diagonals_duplicates_and_negative_cells():
    points = np.asarray([[-.05, -.05, -.05], [.05, .05, .05],
                         [1.05, 1.05, 1.05], [1.06, 1.06, 1.06],
                         [2.05, 2.05, 2.05]])
    components = ClosestPandaHumanVoxels.connected_component_indices(points, 1.0)
    assert sorted(sorted(indices.tolist()) for indices in components) == [[0, 1, 2, 3, 4]]
    components = ClosestPandaHumanVoxels.connected_component_indices(points, .05)
    assert sorted(sorted(indices.tolist()) for indices in components) == [[0], [1], [2, 3], [4]]
    assert ClosestPandaHumanVoxels.connected_component_indices(np.empty((0, 3)), .05) == []


def check_detail_timing_pairing():
    from my_people_nvblox_bringup.measure_processing_latency import DetailedTimings
    details = DetailedTimings()
    key = 100
    for stage, received in [('trt_output', 5_000_000), ('trt_input', 2_000_000),
                            ('mask_decoded', 6_000_000), ('mask_native', 7_000_000)]:
        details.checkpoint(stage, key, received)
    details.remember(details.layers, 200, 12_000_000)
    sample = {'rgb_source_ns': key, 'layer_stamp_ns': 200,
              'stage_ms': dict.fromkeys(('voxel_preparation', 'human_components',
                                        'robot_exclusion', 'nearest_pair', 'distance_filter'), .2)}
    details.computation(sample)
    details.frame(sample, {'rgb': {key: 1_000_000}})
    assert details.samples['people_segmentation'] == [6.0]
    assert details.samples['tensorrt_interval'] == [3.0]
    assert details.samples['mask_decode_interval'] == [1.0]
    assert details.samples['inference_decode_interval'] == [4.0]
    assert details.samples['nvblox_interval'] == [5.0]
    details.interval('tensorrt_interval', 5, 4)
    details.interval('nvblox_interval', None, 4)
    assert details.negative['tensorrt_interval'] == details.missing['nvblox_interval'] == 1
    for index in range(1000):
        details.checkpoint('mask_native', index, index)
        details.remember(details.layers, index, index)
    assert len(details.layers) == len(details.checkpoints) == 600
    assert details.report()['latency']['human_components']['mean_ms'] == .2


def check_timing_summary():
    from my_people_nvblox_bringup.measure_processing_latency import summarize
    assert summarize([]) == {'count': 0}
    result = summarize([-1.0, 1.0, 3.0])
    assert result['mean_ms'] == result['median_ms'] == 1.0
    assert result['p95_ms'] == result['max_ms'] == 3.0
    assert result['negative_samples'] == 1


if __name__ == '__main__':
    check_timing_summary()
    check_detail_timing_pairing()
    check_exact_nearest_pair()
    check_calculation_and_publication_are_independent()
    check_incremental_updates_and_invalid_centers()
    check_stale_sources_cannot_be_kept_alive_by_fresh_map_messages()
    check_reference_pairing_and_single_component_pass()
    check_mask_edges_and_component_filter()
    check_components_preserve_diagonals_duplicates_and_negative_cells()
    print('PASS: exact distances, fast filtering, 5 Hz publication, stale inputs, provenance, one component pass, mask edges')
