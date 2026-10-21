import time
import unittest
from collections import OrderedDict
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from panda_pick_place.native_mask_node import prepare_native_mask
from panda_pick_place.segmentation_input_node import SegmentationInputNode, stamp_key


class MaskTests(unittest.TestCase):
    def test_original_component_threshold_boundary(self):
        mask = np.zeros((544, 960), dtype=np.uint8)
        mask[20:60, 20:70] = 1  # 2000 original pixels: keep.
        mask[100:140, 100:148] = 1  # 1920 pixels: remove.
        result = prepare_native_mask(mask, 960, 544)
        self.assertTrue(np.all(result[20:60, 20:70] == 255))
        self.assertFalse(result[100:140, 100:148].any())
        self.assertEqual(np.count_nonzero(result), 2000)

    def test_reduction_preserves_narrow_foreground_coverage(self):
        mask = np.zeros((544, 960), dtype=np.uint8)
        mask[111, 222] = 1
        result = prepare_native_mask(mask, 424, 240, component_threshold=0)
        self.assertEqual(result.shape, (240, 424))
        self.assertTrue(result.any())
        self.assertEqual(set(np.unique(result)), {0, 255})

    def test_native_resolution_component_threshold(self):
        mask = np.zeros((240, 424), dtype=np.uint8)
        mask[20:34, 20:48] = 1  # 392 pixels: keep at scaled threshold 390.
        mask[60:72, 60:92] = 1  # 384 pixels: remove.
        result = prepare_native_mask(mask, 424, 240, component_threshold=390)
        self.assertEqual(result.shape, (240, 424))
        self.assertTrue(np.all(result[20:34, 20:48] == 255))
        self.assertFalse(result[60:72, 60:92].any())
        self.assertEqual(np.count_nonzero(result), 392)

    def test_full_resolution_filter_preserves_thin_attached_foreground_and_gap(self):
        mask = np.zeros((240, 424), dtype=np.uint8)
        mask[40:80, 40:80] = 1
        mask[21:40, 51] = 1
        mask[21:40, 53] = 1
        mask[120:130, 120:130] = 1  # Separate small noise should still be removed.
        result = prepare_native_mask(mask, 424, 240, 390, component_subsampling=1)
        self.assertTrue(np.all(result[21:40, 51] == 255))
        self.assertTrue(np.all(result[21:40, 53] == 255))
        self.assertFalse(result[21:40, 52].any())
        self.assertFalse(result[120:130, 120:130].any())
        legacy = prepare_native_mask(mask, 424, 240, 390, component_subsampling=2)
        self.assertFalse(legacy[21:40].any())

    def test_unsupported_upscaling_is_rejected(self):
        with self.assertRaises(ValueError):
            prepare_native_mask(np.zeros((544, 960), dtype=np.uint8), 1280, 720)


class Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)


class InputTests(unittest.TestCase):
    def make_gate(self):
        gate = SimpleNamespace(
            infos=OrderedDict(), in_flight=OrderedDict(), latest=None,
            last_sent=-1, ready_after=time.monotonic() - 1, has_result=False,
            limit=2, feedback_timeout=1.0, image_pub=Publisher(), info_pub=Publisher(),
        )
        gate.try_publish = lambda: SegmentationInputNode.try_publish(gate)
        return gate

    def pair(self, index):
        image, info = Image(), CameraInfo()
        for message in (image, info):
            message.header.stamp.sec = index
            message.header.frame_id = 'camera0_color_optical_frame'
        return image, info

    def test_two_in_flight_and_latest_paired_frame(self):
        gate = self.make_gate()
        for index in range(1, 5):
            image, info = self.pair(index)
            SegmentationInputNode.on_image(gate, image)
            SegmentationInputNode.on_info(gate, info)
            self.assertLessEqual(len(gate.in_flight), 2)
        self.assertEqual([m.header.stamp.sec for m in gate.image_pub.messages], [1, 2])
        result, _ = self.pair(1)
        SegmentationInputNode.on_result(gate, result)
        self.assertEqual([m.header.stamp.sec for m in gate.image_pub.messages], [1, 2, 4])
        self.assertEqual([stamp_key(m.header) for m in gate.image_pub.messages],
                         [stamp_key(m.header) for m in gate.info_pub.messages])
        self.assertEqual(len(gate.in_flight), 2)

    def test_frame_mismatch_is_not_forwarded(self):
        gate = self.make_gate()
        image, info = self.pair(1)
        info.header.frame_id = 'different_camera'
        SegmentationInputNode.on_image(gate, image)
        SegmentationInputNode.on_info(gate, info)
        self.assertFalse(gate.image_pub.messages)

    def test_unrelated_result_cannot_release_input_slots(self):
        gate = self.make_gate()
        for index in (1, 2):
            image, info = self.pair(index)
            SegmentationInputNode.on_image(gate, image)
            SegmentationInputNode.on_info(gate, info)
        unrelated, _ = self.pair(100)
        SegmentationInputNode.on_result(gate, unrelated)
        self.assertEqual(len(gate.in_flight), 2)


if __name__ == '__main__':
    unittest.main()
