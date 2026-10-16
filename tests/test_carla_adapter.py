"""CARLA adapter/provenance checks with fake actors; no simulator required."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.config import load_config
from town10_adas.carla_runner import _route_lateral_error, _spawn_scenario_actors, run
from town10_adas.lateral import RoutePoint
from town10_adas.provenance import write_carla_metadata


@dataclass
class FakeConfig:
    dt: float = 0.05


def evidence():
    return {"completed": True, "outcome": "COMPLETE", "collision_count": 0,
            "collision_events": [], "collision_sensor_attached": True,
            "route": {"spawn_index": 0}, "spawns": {"ego": {"x": 0}, "lead": {"x": 80}},
            "map": "Town10HD_Opt", "versions": {"client": "test", "server": "test"}}


class CarlaAdapterTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "run.csv"
        self.path.write_text("time_s,phase\n0,HOLD\n")

    def write(self, data=None, controller="pid"):
        path = write_carla_metadata(self.path, FakeConfig(), controller, evidence() if data is None else data)
        return json.loads(path.read_text())

    def test_metadata_binds_csv_and_settings(self):
        pid, mpc = self.write(), self.write(controller="mpc")
        self.assertEqual(pid["source"], "carla")
        self.assertEqual(pid["csv_sha256"], hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(pid["shared_settings"], mpc["shared_settings"])
        self.assertEqual(pid["source_sha256"], mpc["source_sha256"])
        self.assertEqual(pid["row_count"], 1)

    def test_spawn_changes_invalidate_shared_conditions(self):
        before = self.write()
        data = evidence()
        data["spawns"]["ego"]["x"] = 1
        self.assertNotEqual(before["shared_settings"], self.write(data)["shared_settings"])

    def test_incomplete_run_preserves_unknown_contact(self):
        data = {"completed": False, "outcome": "exception", "collision_count": None,
                "failure": "spawn failed"}
        result = self.write(data)
        self.assertFalse(result["completed"])
        self.assertIsNone(result["collision_count"])
        self.assertEqual(result["evidence"]["failure"], "spawn failed")

    def test_reject_incomplete_evidence_and_nonfinite_values(self):
        for key in ("route", "spawns", "map", "versions", "collision_sensor_attached"):
            data = evidence()
            data.pop(key)
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.write(data)
        data = evidence()
        data["spawns"]["ego"]["x"] = float("nan")
        with self.assertRaises(ValueError):
            self.write(data)

    def test_reject_header_only_and_wrong_collision_count(self):
        data = evidence()
        data["collision_count"] = 1
        with self.assertRaises(ValueError):
            self.write(data)
        self.path.write_text("time_s,phase\n")
        with self.assertRaises(ValueError):
            self.write()

    def test_failed_fixed_spawn_never_tries_another_location(self):
        world = MagicMock()
        world.get_map.return_value.get_spawn_points.return_value = [
            SimpleNamespace(location="first"), SimpleNamespace(location="second")]
        ego = MagicMock()
        ego.is_alive = True
        config = load_config(ROOT / "config/town10_adas.json")
        with patch("town10_adas.carla_runner.build_fixed_lane_route", return_value=[]), \
             patch("town10_adas.carla_runner._waypoint_at_distance"), \
             patch("town10_adas.carla_runner._spawn_vehicle", side_effect=[ego, RuntimeError("occupied")]) as spawn:
            with self.assertRaises(RuntimeError):
                _spawn_scenario_actors(world, config, 0)
            self.assertEqual(spawn.call_count, 2)
            ego.destroy.assert_called_once()
            world.get_map.return_value.get_waypoint.assert_called_once()

    def test_invalid_spawn_index_is_rejected(self):
        world = MagicMock()
        world.get_map.return_value.get_spawn_points.return_value = [object()]
        with self.assertRaises(ValueError):
            _spawn_scenario_actors(world, load_config(ROOT / "config/town10_adas.json"), -1)

    def test_lane_error_uses_segments_and_clamps_their_endpoints(self):
        route = [RoutePoint(0.0, 0.0), RoutePoint(10.0, 0.0), RoutePoint(10.0, 10.0)]
        self.assertAlmostEqual(_route_lateral_error(SimpleNamespace(x=5.0, y=1.0), route), 1.0)
        self.assertAlmostEqual(_route_lateral_error(SimpleNamespace(x=11.0, y=5.0), route), 1.0)
        self.assertAlmostEqual(_route_lateral_error(SimpleNamespace(x=-3.0, y=-4.0), route), 5.0)

    def test_video_rerun_cannot_truncate_existing_csv_before_connecting(self):
        original = self.path.read_bytes()
        with self.assertRaises(FileExistsError):
            run(load_config(ROOT / "config/town10_adas.json"), "pid", self.path,
                video_path=self.path.with_suffix(".mp4"))
        self.assertEqual(self.path.read_bytes(), original)
