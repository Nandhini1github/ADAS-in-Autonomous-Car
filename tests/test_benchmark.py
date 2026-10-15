import csv
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.benchmark import DEFAULT_CASES, ModelCase, integrate_motion, run_model
from town10_adas.config import load_config
from town10_adas.safety import SafetySupervisor
from town10_adas.aeb import AEBMPC


class ModelBenchmarkTests(unittest.TestCase):
    def test_exact_stop_distance(self):
        speed, distance = integrate_motion(1.0, -4.0, 0.5)
        self.assertEqual(speed, 0.0)
        self.assertAlmostEqual(distance, 0.125)

    def test_overlap_triggers_emergency(self):
        for gap in (0, -1):
            assessment = SafetySupervisor().evaluate(0, 0, gap)
            self.assertTrue(assessment.aeb)
            self.assertEqual(assessment.ttc_s, 0)

    def test_invalid_inputs_rejected(self):
        with self.assertRaises(ValueError):
            SafetySupervisor().evaluate(float("nan"), 0, 10)
        with self.assertRaises(ValueError):
            AEBMPC(step_s=0)
        with self.assertRaises(ValueError):
            ModelCase("bad", 10, 15)

    def test_repeatable_and_matched_until_emergency(self):
        config = load_config(ROOT / "config/town10_adas.json")
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / name for name in ("pid.csv", "mpc.csv", "repeat.csv")]
            for controller, path in zip(("pid", "mpc", "pid"), paths):
                metadata = run_model(config, DEFAULT_CASES[0], controller, path)
                self.assertEqual(metadata["source"], "deterministic_1d_model")
                self.assertEqual(metadata["outcome"], "COMPLETE")
            self.assertEqual(paths[0].read_bytes(), paths[2].read_bytes())
            rows = []
            for path in paths[:2]:
                with path.open() as stream:
                    rows.append(list(csv.DictReader(stream)))
            event_indices = [next(i for i, row in enumerate(run) if row["aeb_active"] == "1") for run in rows]
            self.assertEqual(event_indices[0], event_indices[1])
            self.assertEqual(rows[0][:event_indices[0]], rows[1][:event_indices[1]])
            for run in rows:
                self.assertEqual(run[-1]["phase"], "COMPLETE")
                self.assertGreater(min(float(row["distance_m"]) for row in run), 0)

    def test_reduced_grip_failure_is_retained(self):
        config = load_config(ROOT / "config/town10_adas.json")
        with tempfile.TemporaryDirectory() as directory:
            for controller in ("pid", "mpc"):
                path = Path(directory) / f"{controller}.csv"
                metadata = run_model(config, DEFAULT_CASES[4], controller, path)
                self.assertEqual(metadata["outcome"], "ABORTED")
                self.assertTrue(metadata["overlap_detected"])
                with path.open() as stream:
                    last = list(csv.DictReader(stream))[-1]
                self.assertEqual(last["phase"], "ABORTED")
                self.assertLessEqual(float(last["distance_m"]), 0)


if __name__ == "__main__":
    unittest.main()
