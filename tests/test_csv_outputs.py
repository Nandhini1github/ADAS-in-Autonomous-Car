import csv
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.carla_runner import default_output_path
from town10_adas.config import load_config
from town10_adas.csv_logger import FIELDNAMES, ScenarioCsvLogger


class CsvOutputTests(unittest.TestCase):
    def test_pid_and_mpc_have_one_stable_output_each(self) -> None:
        config = load_config(ROOT / "config" / "town10_adas.json")
        self.assertEqual(default_output_path(config, "pid"), Path("results/town10_aeb_pid.csv"))
        self.assertEqual(default_output_path(config, "mpc"), Path("results/town10_aeb_mpc.csv"))

    def test_logger_matches_the_town10_reference_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "town10_aeb_pid.csv"
            with ScenarioCsvLogger(output) as logger:
                logger.write(
                    time_s="0.050",
                    phase="ACCELERATE",
                    ego_speed_mph="1.000",
                    target_speed_mph="0.500",
                    distance_m="80.000",
                    closing_speed_mph="0.500",
                    ttc_s="160.000",
                    fcw=0,
                    aeb_active=0,
                    brake_percent="0.00",
                    ego_steer="0.0000",
                    ego_accel_mps2="0.1000",
                )
            with output.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                rows = list(reader)
            self.assertEqual(reader.fieldnames, FIELDNAMES)
            self.assertEqual(len(rows), 1)


if __name__ == "__main__":
    unittest.main()
