"""YOLOX tensor/depth/tracking checks using synthetic data, without model/server."""

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.rear_perception import CameraPedestrian
from town10_adas.rear_rgb import RearRGBDetector, _PersonTracker, decode_yolox_persons, pedestrian_from_depth

try:
    import numpy as np
except ImportError:
    np = None
try:
    import cv2
except ImportError:
    cv2 = None


def encoded_person(output, index=0, box=(100.25, 60.25, 140.25, 180.25), score=0.9, category=0):
    """Encode a known box independently in the first stride-eight output grid."""
    x0, y0, x1, y1 = box
    grid_x, grid_y = index % 80, index // 80
    output[0, index, :4] = [(x0 + x1) / 16.0 - grid_x,
                           (y0 + y1) / 16.0 - grid_y,
                           math.log((x1 - x0) / 8.0), math.log((y1 - y0) / 8.0)]
    output[0, index, 4] = score
    output[0, index, 5 + category] = 0.9


@unittest.skipIf(np is None, "NumPy is not installed")
class YOLOXDecodeTests(unittest.TestCase):
    def test_strided_centers_exponential_sizes_and_scale_decode(self):
        output = np.zeros((1, 8400, 85), dtype=np.float32)
        encoded_person(output, index=81)
        boxes = decode_yolox_persons(output, 320, 180, scale=2.0)
        self.assertEqual(len(boxes), 1)
        self.assertEqual(boxes[0][0], (50, 30, 71, 91))
        self.assertAlmostEqual(boxes[0][1], 0.81, places=6)
        self.assertAlmostEqual(output[0, 81, 0], (100.25 + 140.25) / 16 - 1)

    def test_person_class_filter_confidence_product_and_duplicate_suppression(self):
        output = np.zeros((1, 8400, 85), dtype=np.float32)
        encoded_person(output, index=0)
        encoded_person(output, index=1, score=0.8)  # Same box, lower confidence.
        encoded_person(output, index=2, box=(250.25, 60.25, 290.25, 180.25), category=2)
        encoded_person(output, index=3, box=(350.25, 60.25, 390.25, 180.25), score=0.3)
        encoded_person(output, index=4, box=(450.25, 60.25, 490.25, 180.25))
        boxes = decode_yolox_persons(output, 640, 360, scale=1.0)
        self.assertEqual(len(boxes), 2)
        self.assertEqual([box[0][0] for box in boxes], [100, 450])

    def test_person_runner_up_to_other_class_is_not_a_person_detection(self):
        output = np.zeros((1, 8400, 85), dtype=np.float32)
        encoded_person(output)
        output[0, 0, 7] = 0.95
        self.assertEqual(decode_yolox_persons(output, 640, 360, 1.0), [])

    def test_padding_detections_and_nonfinite_predictions_are_rejected(self):
        output = np.zeros((1, 8400, 85), dtype=np.float32)
        encoded_person(output, index=0, box=(100, 400, 140, 500))
        encoded_person(output, index=1)
        output[0, 1, 0] = math.nan
        self.assertEqual(decode_yolox_persons(output, 640, 360, 1.0), [])

    def test_wrong_model_tensor_layout_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "expected YOLOX"):
            decode_yolox_persons(np.zeros((1, 85, 8400)), 640, 360, 1.0)


@unittest.skipIf(np is None, "NumPy is not installed")
class RGBDepthTests(unittest.TestCase):
    def estimate(self, depth, box=(40, 10, 60, 110)):
        return pedestrian_from_depth(box, depth, fov_deg=90.0,
                                      camera_to_bumper_m=0.05, max_range_m=15.0)

    def test_body_foreground_rejects_far_background_and_ground_below_feet(self):
        depth = np.full((120, 160), 12.0)
        depth[30:75, 47:53] = 4.0
        depth[90:110, 40:60] = 1.0  # Ground below feet must not set person range.
        person = self.estimate(depth)
        self.assertIsNotNone(person)
        self.assertAlmostEqual(person.clearance_m, 4.05)
        self.assertAlmostEqual(person.lateral_m, 1.5)
        self.assertAlmostEqual(person.width_m, 1.0)
        self.assertEqual(person.pixel_count, 45 * 6)

    def test_sparse_invalid_depth_does_not_manufacture_person_geometry(self):
        for fill in (0.0, math.nan, math.inf, 1000.0):
            with self.subTest(fill=fill):
                depth = np.full((120, 160), fill)
                depth[50, 50] = 4.0
                self.assertIsNone(self.estimate(depth))

    def test_small_nearer_occluder_does_not_replace_body_cluster(self):
        depth = np.full((120, 160), 4.0)
        depth[40:42, 45:47] = 1.0
        person = self.estimate(depth)
        self.assertAlmostEqual(person.clearance_m, 4.05)

    def test_box_must_be_inside_depth_image(self):
        with self.assertRaises(ValueError):
            self.estimate(np.ones((120, 160)), box=(-1, 10, 60, 110))


class AssociationTests(unittest.TestCase):
    def person(self, lateral, x):
        return CameraPedestrian(0, 4.0, lateral, 0.4, (x, 40, x + 20, 120), 40)

    def test_two_people_keep_separate_ids_when_detection_order_changes(self):
        tracker = _PersonTracker()
        left, right = self.person(-1.5, 400), self.person(1.5, 200)
        first = tracker.assign([left, right], 10)
        by_side = {person.lateral_m: person.instance_id for person in first}
        second = tracker.assign([replace(right, lateral_m=1.4), replace(left, lateral_m=-1.4)], 11)
        self.assertEqual(second[0].instance_id, by_side[1.5])
        self.assertEqual(second[1].instance_id, by_side[-1.5])
        self.assertNotEqual(second[0].instance_id, second[1].instance_id)

    def test_one_track_cannot_be_assigned_to_two_detections(self):
        tracker = _PersonTracker()
        old = tracker.assign([self.person(0.0, 300)], 1)[0]
        current = tracker.assign([self.person(0.1, 301), self.person(-0.2, 298)], 2)
        self.assertEqual(len({person.instance_id for person in current}), 2)
        self.assertEqual(sum(person.instance_id == old.instance_id for person in current), 1)

    def test_long_gap_and_unrelated_positions_create_new_tracks(self):
        tracker = _PersonTracker()
        old = tracker.assign([self.person(0.0, 300)], 1)[0]
        new = tracker.assign([self.person(0.0, 300)], 12)[0]
        self.assertNotEqual(old.instance_id, new.instance_id)
        distant = tracker.assign([self.person(8.0, 20)], 13)[0]
        self.assertNotEqual(new.instance_id, distant.instance_id)


@unittest.skipIf(np is None or cv2 is None, "NumPy/OpenCV optional dependencies are missing")
class RGBPipelineTests(unittest.TestCase):
    def detector_and_images(self):
        # A fake network isolates image plumbing from weights or model accuracy.
        output = np.zeros((1, 8400, 85), dtype=np.float32)
        encoded_person(output, box=(200.25, 60.25, 250.25, 220.25))
        net = SimpleNamespace(forward=lambda: output)
        net.setInput = lambda blob: setattr(net, "blob", blob.copy())
        detector = RearRGBDetector.__new__(RearRGBDetector)
        detector.net = net
        detector.fov_deg = 90.0
        detector.camera_to_bumper_m = 0.05
        detector.max_range_m = 15.0
        detector.confidence = 0.35
        detector.tracker = _PersonTracker()
        detector.last_frame = None
        rgb = np.full((360, 640, 4), [11, 22, 33, 255], dtype=np.uint8)
        encoded = round(4.0 / 1000.0 * 16777215)
        depth = np.full_like(rgb, [(encoded >> 16) & 255, (encoded >> 8) & 255, encoded & 255, 255])
        images = [SimpleNamespace(raw_data=array.tobytes(), width=640, height=360,
                                  frame=7, timestamp=0.35, fov=90.0) for array in (rgb, depth)]
        return detector, images

    def test_true_rgb_unnormalized_chw_letterbox_and_raw_depth_pipeline(self):
        detector, images = self.detector_and_images()
        people = detector.detect(*images)
        self.assertEqual(detector.net.blob.shape, (1, 3, 640, 640))
        np.testing.assert_array_equal(detector.net.blob[0, :, 10, 10], [33, 22, 11])
        np.testing.assert_array_equal(detector.net.blob[0, :, 500, 10], [114, 114, 114])
        self.assertEqual(detector.net.blob.dtype, np.float32)
        self.assertEqual(len(people), 1)
        self.assertAlmostEqual(people[0].clearance_m, 4.05, places=4)
        self.assertGreater(people[0].lateral_m, 0.0)

    def test_misaligned_frames_dimensions_timestamps_fov_and_payload_raise(self):
        for field, value in (("frame", 8), ("width", 641), ("timestamp", 0.4),
                             ("fov", 110.0), ("raw_data", b"bad")):
            with self.subTest(field=field):
                detector, images = self.detector_and_images()
                setattr(images[1], field, value)
                with self.assertRaises(ValueError):
                    detector.detect(*images)

    def test_repeated_frame_does_not_update_tracking(self):
        detector, images = self.detector_and_images()
        detector.detect(*images)
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            detector.detect(*images)


if __name__ == "__main__":
    unittest.main()
