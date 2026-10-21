"""Synthetic pixel fixtures exercise decoding; these are not CARLA road tests."""

from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.rear_perception import CameraPedestrian, RearAEB, detect_pedestrians

try:
    import numpy as np
except ImportError:
    np = None


def pedestrian(clearance=2.5, lateral=0.0, instance_id=42):
    return CameraPedestrian(instance_id, clearance, lateral, 0.4)


@unittest.skipIf(np is None, "NumPy optional camera dependency is not installed")
class RearCameraTests(unittest.TestCase):
    def images(self, *, tag=12, depth_m=4.0, frame=7):
        labels = np.zeros((8, 12, 4), dtype=np.uint8)
        labels[2:6, 1:3, :] = [0x34, 0x12, tag, 255]
        depth = np.zeros_like(labels)
        encoded = round(depth_m / 1000.0 * 16777215)
        depth[:, :, :] = [(encoded >> 16) & 255, (encoded >> 8) & 255, encoded & 255, 255]
        return tuple(SimpleNamespace(raw_data=array.tobytes(), width=12, height=8,
                                     frame=frame, timestamp=frame * 0.05, fov=90.0)
                     for array in (labels, depth))

    def test_bgra_semantic_instance_and_axial_depth_decode(self):
        detections = detect_pedestrians(*self.images(), 90.0)
        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection.instance_id, 0x1234)
        self.assertEqual(detection.bbox_xyxy, (1, 2, 3, 6))
        self.assertEqual(detection.pixel_count, 8)
        self.assertAlmostEqual(detection.clearance_m, 4.05, places=4)
        # Camera-left means ego-right. Off-center axial depth is still 4 m.
        self.assertAlmostEqual(detection.lateral_m, 4.0 * 4.0 / 6.0, places=4)
        self.assertAlmostEqual(detection.width_m, 2.0 * 4.0 / 6.0, places=4)

    def test_current_pedestrian_tag_and_explicit_legacy_override(self):
        self.assertEqual(detect_pedestrians(*self.images(tag=4), 90.0), [])
        self.assertEqual(len(detect_pedestrians(*self.images(tag=4), 90.0, pedestrian_tag=4)), 1)

    def test_separate_instances_with_same_semantic_tag(self):
        instance, depth = self.images()
        labels = np.frombuffer(instance.raw_data, dtype=np.uint8).copy().reshape(8, 12, 4)
        labels[2:6, 8:10, :] = [0x35, 0x12, 12, 255]
        instance.raw_data = labels.tobytes()
        detections = detect_pedestrians(instance, depth, 90.0)
        self.assertEqual({item.instance_id for item in detections}, {0x1234, 0x1235})
        self.assertLess(next(item.lateral_m for item in detections if item.instance_id == 0x1235), 0)

    def test_mismatched_frames_dimensions_and_timestamps_are_rejected(self):
        for field, value in (("frame", 8), ("width", 11), ("timestamp", 0.8), ("fov", 110.0)):
            with self.subTest(field=field):
                instance, depth = self.images()
                setattr(depth, field, value)
                with self.assertRaises(ValueError):
                    detect_pedestrians(instance, depth, 90.0)

    def test_malformed_image_metadata_and_payload_are_rejected(self):
        for field, value in (("frame", float("nan")), ("width", 0), ("height", 8.5),
                             ("raw_data", b"too short")):
            with self.subTest(field=field):
                instance, depth = self.images()
                setattr(instance, field, value)
                with self.assertRaises(ValueError):
                    detect_pedestrians(instance, depth, 90.0)

    def test_zero_far_and_insufficient_pixel_depth_are_filtered(self):
        for depth_m in (0.0, 20.0, 1000.0):
            with self.subTest(depth_m=depth_m):
                self.assertEqual(detect_pedestrians(*self.images(depth_m=depth_m), 90.0), [])
        self.assertEqual(detect_pedestrians(*self.images(), 90.0, min_pixels=9), [])

    def test_camera_calibration_must_be_finite_and_physical(self):
        for kwargs in ({"fov_deg": math.nan}, {"fov_deg": 180},
                       {"fov_deg": 90, "max_range_m": math.inf},
                       {"fov_deg": 90, "camera_to_bumper_m": -0.1},
                       {"fov_deg": 90, "min_pixels": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                detect_pedestrians(*self.images(), **kwargs)


class RearAEBTests(unittest.TestCase):
    def test_child_enters_path_predictively_and_brake_latches(self):
        aeb = RearAEB()
        first = aeb.update([pedestrian(lateral=2.8)], speed_mps=2.0, reverse=True, time_s=0.0)
        self.assertFalse(first.active)
        crossing = aeb.update([pedestrian(lateral=2.6)], speed_mps=2.0, reverse=True, time_s=0.1)
        self.assertTrue(crossing.active)
        self.assertEqual(crossing.reason, "predicted_crossing")
        self.assertEqual(crossing.target_instance_id, 42)
        self.assertEqual(crossing.brake, 1.0)
        # Losing the person or stopping must not release emergency braking.
        stopped = aeb.update([], speed_mps=0.0, reverse=True, time_s=0.2)
        self.assertTrue(stopped.active)
        self.assertEqual(stopped.reason, "latched")
        aeb.reset()
        self.assertFalse(aeb.update([], speed_mps=0.0, reverse=True, time_s=0.0).active)

    def test_front_side_and_forward_motion_do_not_trigger(self):
        for detection, reverse in ((pedestrian(clearance=-2.0), True),
                                    (pedestrian(lateral=3.0), True),
                                    (pedestrian(clearance=0.1), False)):
            with self.subTest(detection=detection, reverse=reverse):
                decision = RearAEB().update([detection], speed_mps=2.0,
                                            reverse=reverse, time_s=0.0)
                self.assertFalse(decision.active)
                self.assertEqual(decision.brake, 0.0)

    def test_moving_away_from_reverse_corridor_does_not_trigger(self):
        aeb = RearAEB()
        aeb.update([pedestrian(lateral=2.5)], speed_mps=2.0, reverse=True, time_s=0.0)
        result = aeb.update([pedestrian(lateral=2.7)], speed_mps=2.0, reverse=True, time_s=0.1)
        self.assertFalse(result.active)

    def test_near_corridor_first_observation_uses_small_safety_expansion(self):
        result = RearAEB().update([pedestrian(lateral=1.6)], speed_mps=2.0,
                                  reverse=True, time_s=0.0)
        self.assertTrue(result.active)
        self.assertEqual(result.reason, "near_corridor")

    def test_new_instance_does_not_inherit_other_person_velocity(self):
        aeb = RearAEB()
        aeb.update([pedestrian(lateral=3.0)], speed_mps=2.0, reverse=True, time_s=0.0)
        result = aeb.update([pedestrian(lateral=2.5, instance_id=99)], speed_mps=2.0,
                            reverse=True, time_s=0.1)
        self.assertFalse(result.active)

    def test_stale_track_does_not_produce_crossing_velocity(self):
        aeb = RearAEB()
        aeb.update([pedestrian(lateral=3.0)], speed_mps=2.0, reverse=True, time_s=0.0)
        result = aeb.update([pedestrian(lateral=2.0)], speed_mps=2.0, reverse=True, time_s=1.0)
        self.assertFalse(result.active)

    def test_dynamic_stop_distance_can_trigger_independently_of_ttc(self):
        aeb = RearAEB(ttc_threshold_s=0.1)
        result = aeb.update([pedestrian(clearance=2.0)], speed_mps=3.0,
                            reverse=True, time_s=0.0)
        self.assertAlmostEqual(result.stopping_distance_m, 0.8 + 3 * 0.25 + 9 / 8)
        self.assertTrue(result.active)
        self.assertGreater(result.ttc_s, 0.1)

    def test_ttc_can_trigger_independently_of_stopping_distance(self):
        result = RearAEB().update([pedestrian(clearance=3.0)], speed_mps=2.0,
                                  reverse=True, time_s=0.0)
        self.assertGreater(result.clearance_m, result.stopping_distance_m)
        self.assertEqual(result.ttc_s, 1.5)
        self.assertTrue(result.active)

    def test_stationary_vehicle_only_holds_close_reverse_threat(self):
        for clearance, expected in ((4.0, False), (0.2, True)):
            with self.subTest(clearance=clearance):
                result = RearAEB().update([pedestrian(clearance=clearance)], speed_mps=0.0,
                                          reverse=True, time_s=0.0)
                self.assertIsNone(result.ttc_s)
                self.assertEqual(result.active, expected)

    def test_reverse_gate_suppresses_latch_but_does_not_clear_it(self):
        aeb = RearAEB()
        aeb.update([pedestrian()], speed_mps=2.0, reverse=True, time_s=0.0)
        forward = aeb.update([], speed_mps=2.0, reverse=False, time_s=0.1)
        self.assertFalse(forward.active)
        self.assertEqual(forward.brake, 0.0)
        self.assertTrue(aeb.latched)
        self.assertTrue(aeb.update([], speed_mps=0.0, reverse=True, time_s=0.2).active)

    def test_nonfinite_negative_and_duplicate_inputs_are_rejected(self):
        for kwargs in ({"speed_mps": math.nan}, {"speed_mps": -1.0},
                       {"time_s": math.inf}, {"time_s": -0.1}, {"reverse": 1}):
            values = dict(speed_mps=1.0, reverse=True, time_s=0.0)
            values.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RearAEB().update([], **values)
        for field in ("clearance_m", "lateral_m", "width_m"):
            with self.subTest(field=field), self.assertRaises(ValueError):
                RearAEB().update([replace(pedestrian(), **{field: math.nan})],
                                 speed_mps=1.0, reverse=True, time_s=0.0)
        with self.assertRaises(ValueError):
            RearAEB().update([pedestrian(), pedestrian()], speed_mps=1.0, reverse=True, time_s=0.0)
        aeb = RearAEB()
        aeb.update([], speed_mps=1.0, reverse=True, time_s=1.0)
        with self.assertRaises(ValueError):
            aeb.update([], speed_mps=1.0, reverse=True, time_s=1.0)

    def test_nonfinite_configuration_is_rejected(self):
        for kwargs in ({"braking_decel_mps2": 0}, {"stop_margin_m": math.nan},
                       {"reaction_s": -1}, {"prediction_horizon_s": math.inf}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RearAEB(**kwargs)


if __name__ == "__main__":
    unittest.main()
