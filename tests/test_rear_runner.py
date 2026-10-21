"""Runner checks with camera queues and simulator fakes; no server is used."""

from copy import deepcopy
from dataclasses import dataclass
import io
import json
import math
from pathlib import Path
import queue
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.rear_runner import _ground_truth, _spawn_camera, frame_from_queue, load_config, run


def scenario_config():
    return json.loads((ROOT / "config/town10_rear_aeb.json").read_text())


class RearConfigTests(unittest.TestCase):
    def load(self, config):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scenario.json"
            path.write_text(json.dumps(config))
            return load_config(path)

    def test_default_scenario_is_a_low_speed_town10_reverse_run(self):
        config = self.load(scenario_config())
        self.assertIn(config["map"], {"Town10HD", "Town10HD_Opt"})
        self.assertLess(config["crossing_trigger_distance_m"], config["maximum_reverse_distance_m"])
        self.assertLessEqual(config["reverse_speed_mps"], 2.0)

    def test_wrong_map_cannot_reuse_town10_parking_coordinates(self):
        config = scenario_config()
        config["map"] = "Town03"
        with self.assertRaises(ValueError):
            self.load(config)

    def test_trigger_must_be_reachable_before_travel_limit(self):
        config = scenario_config()
        config["crossing_trigger_distance_m"] = config["maximum_reverse_distance_m"]
        with self.assertRaises(ValueError):
            self.load(config)

    def test_nonfinite_nonpositive_and_boolean_dynamics_are_rejected(self):
        for field in ("fixed_delta_seconds", "reverse_speed_mps", "child_speed_mps",
                      "crossing_trigger_distance_m", "maximum_reverse_distance_m",
                      "timeout_s", "stop_speed_mps", "stop_hold_s"):
            for value in (math.nan, math.inf, 0, -1, True):
                with self.subTest(field=field, value=value):
                    config = scenario_config()
                    config[field] = value
                    with self.assertRaises(ValueError):
                        self.load(config)

    def test_invalid_camera_calibration_is_rejected_before_connection(self):
        cases = (("width", 63), ("height", 121), ("width", True),
                 ("fov_deg", 180), ("height_m", math.nan), ("bumper_offset_m", -0.1),
                 ("max_range_m", 1000), ("max_range_m", 0.01))
        for field, value in cases:
            with self.subTest(field=field, value=value):
                config = scenario_config()
                config["camera"][field] = value
                with self.assertRaises(ValueError):
                    self.load(config)

    def test_nonfinite_placement_and_invalid_deceleration_are_rejected(self):
        for actor in ("ego", "child", "child_end"):
            with self.subTest(actor=actor):
                config = scenario_config()
                config["parking_spot"][actor]["x"] = math.inf
                with self.assertRaises(ValueError):
                    self.load(config)
        config = scenario_config()
        config["aeb"]["braking_decel_mps2"] = 0.0
        with self.assertRaises(ValueError):
            self.load(config)


class CameraQueueTests(unittest.TestCase):
    def test_old_images_are_discarded_but_exact_requested_frame_is_returned(self):
        frames = queue.Queue()
        for frame in (5, 6, 7, 8):
            frames.put(SimpleNamespace(frame=frame))
        self.assertEqual(frame_from_queue(frames, 7).frame, 7)
        self.assertEqual(frames.get_nowait().frame, 8)

    def test_future_image_is_rejected_instead_of_pairing_wrong_frames(self):
        frames = queue.Queue()
        frames.put(SimpleNamespace(frame=8))
        with self.assertRaisesRegex(RuntimeError, "skipped frame 7"):
            frame_from_queue(frames, 7)

    def test_missing_frame_times_out(self):
        with self.assertRaisesRegex(TimeoutError, "frame 7"):
            frame_from_queue(queue.Queue(), 7, timeout_s=0.001)

    def test_camera_is_calibrated_rigidly_and_overflow_retains_recent_frames(self):
        attributes = {}
        blueprint = SimpleNamespace(set_attribute=attributes.__setitem__)
        sensor = SimpleNamespace(listen=lambda callback: setattr(sensor, "callback", callback))
        spawn_calls = []

        def spawn(bp, transform, **kwargs):
            spawn_calls.append((bp, transform, kwargs))
            return sensor

        world = SimpleNamespace(get_blueprint_library=lambda: SimpleNamespace(find=lambda kind: blueprint),
                                spawn_actor=spawn)
        carla = SimpleNamespace(AttachmentType=SimpleNamespace(Rigid="rigid"))
        ego, transform, sensors = object(), object(), []
        frames = _spawn_camera(world, carla, "sensor.camera.depth", transform,
                               scenario_config()["camera"], sensors, ego=ego)
        self.assertEqual(sensors, [sensor])
        self.assertEqual(spawn_calls[0][2], {"attach_to": ego, "attachment_type": "rigid"})
        self.assertEqual(attributes["lens_k"], "0.0")
        self.assertEqual(attributes["lens_kcube"], "0.0")
        self.assertEqual(attributes["sensor_tick"], "0.0")
        for frame in range(32):
            sensor.callback(SimpleNamespace(frame=frame))
        self.assertEqual(frames.qsize(), 16)
        self.assertEqual(frames.get_nowait().frame, 16)
        self.assertEqual(frame_from_queue(frames, 31).frame, 31)

    def test_sensor_is_registered_for_cleanup_before_listen_can_fail(self):
        blueprint = SimpleNamespace(set_attribute=lambda *args: None)
        sensor = SimpleNamespace(listen=lambda callback: (_ for _ in ()).throw(RuntimeError("listen failed")))
        world = SimpleNamespace(get_blueprint_library=lambda: SimpleNamespace(find=lambda kind: blueprint),
                                spawn_actor=lambda *args, **kwargs: sensor)
        sensors = []
        with self.assertRaisesRegex(RuntimeError, "listen failed"):
            _spawn_camera(world, None, "sensor.camera.rgb", None,
                          scenario_config()["camera"], sensors)
        self.assertEqual(sensors, [sensor])


@dataclass
class Vector:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z


class Transform:
    def __init__(self, x, y, yaw):
        self.location = Vector(x, y)
        self.rotation = SimpleNamespace(yaw=yaw)
        self.yaw = math.radians(yaw)

    def transform(self, point):
        return Vector(self.location.x + point.x * math.cos(self.yaw) - point.y * math.sin(self.yaw),
                      self.location.y + point.x * math.sin(self.yaw) + point.y * math.cos(self.yaw),
                      point.z)

    def get_forward_vector(self):
        return Vector(math.cos(self.yaw), math.sin(self.yaw))

    def get_right_vector(self):
        return Vector(-math.sin(self.yaw), math.cos(self.yaw))


def actor(transform, extent, center=None):
    return SimpleNamespace(get_transform=lambda: transform,
                           bounding_box=SimpleNamespace(extent=extent, location=center or Vector()))


class GroundTruthGeometryTests(unittest.TestCase):
    def test_clearance_accounts_for_box_center_and_rotated_child_extent(self):
        ego = actor(Transform(0, 0, 0), Vector(2, 1), Vector(0.25, 0))
        child = actor(Transform(-8, 3, 90), Vector(0.3, 0.2), Vector(0.1, 0))
        clearance, lateral = _ground_truth(ego, child)
        self.assertAlmostEqual(clearance, 6.05)
        self.assertAlmostEqual(lateral, 3.1)

    def test_world_rotation_does_not_change_rear_clearance_or_ego_lateral(self):
        ego = actor(Transform(10, 20, 90), Vector(2, 1), Vector(0.25, 0))
        child = actor(Transform(7, 12, 180), Vector(0.3, 0.2), Vector(0.1, 0))
        clearance, lateral = _ground_truth(ego, child)
        self.assertAlmostEqual(clearance, 6.05)
        self.assertAlmostEqual(lateral, 3.1)


class RunnerFailureLifecycleTests(unittest.TestCase):
    def test_setup_failure_restores_world_and_records_failed_outcome(self):
        original = SimpleNamespace(synchronous_mode=False, fixed_delta_seconds=None,
                                   no_rendering_mode=True, substepping=False,
                                   max_substep_delta_time=0.02, max_substeps=4)
        world = SimpleNamespace(settings=deepcopy(original), weather="original weather")
        applied = []

        def apply_settings(settings):
            applied.append(deepcopy(settings))
            world.settings = deepcopy(settings)

        def missing_blueprint(kind):
            raise RuntimeError("child blueprint is unavailable")

        world.get_actors = lambda: []
        world.get_map = lambda: SimpleNamespace(name="/Game/Carla/Maps/Town10HD_Opt")
        world.get_settings = lambda: deepcopy(world.settings)
        world.get_weather = lambda: world.weather
        world.apply_settings = apply_settings
        world.set_weather = lambda weather: setattr(world, "weather", weather)
        world.get_blueprint_library = lambda: SimpleNamespace(find=missing_blueprint)
        client = SimpleNamespace(set_timeout=lambda value: None, get_world=lambda: world,
                                 get_client_version=lambda: "test", get_server_version=lambda: "test")
        carla = SimpleNamespace(Client=lambda host, port: client,
                                WeatherParameters=SimpleNamespace(ClearNoon="clear noon"))
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            config = scenario_config()
            # Exercise spawn cleanup without a downloaded model or OpenCV.
            config["perception"] = "instance_depth"
            with patch.dict(sys.modules, {"carla": carla}), patch("sys.stdout", new=io.StringIO()):
                status = run(config, output, video=False)
            metadata = json.loads((output / "metadata.json").read_text())
        self.assertEqual(status, 1)
        self.assertEqual(metadata["outcome"], "error")
        self.assertFalse(metadata["completed"])
        self.assertIn("child blueprint is unavailable", metadata["error"])
        self.assertEqual(metadata["cleanup_errors"], [])
        self.assertEqual(world.weather, "original weather")
        self.assertEqual(vars(world.settings), vars(original))
        self.assertTrue(applied[0].synchronous_mode)
        self.assertFalse(applied[-1].synchronous_mode)

    def test_active_server_is_rejected_before_changing_world(self):
        world = SimpleNamespace(get_actors=lambda: [SimpleNamespace(type_id="vehicle.lincoln.mkz")])
        client = SimpleNamespace(set_timeout=lambda value: None, get_world=lambda: world)
        carla = SimpleNamespace(Client=lambda host, port: client)
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(sys.modules, {"carla": carla}), self.assertRaisesRegex(RuntimeError, "idle server"):
                run(scenario_config(), Path(directory) / "run", video=False)


if __name__ == "__main__":
    unittest.main()
