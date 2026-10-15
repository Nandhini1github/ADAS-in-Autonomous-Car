import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.csv_logger import FIELDNAMES
from town10_adas.metrics import compare_runs, load_csv, load_metadata, summarize_run, write_report


def example_rows():
    result = []
    for time, speed, phase, gap in [(0, 10, "HOLD", 20), (1, 10, "AEB", 18),
                                   (2, 5, "AEB", 16), (3, 0, "COMPLETE", 15)]:
        result.append(dict(time_s=time, phase=phase, ego_speed_mph=speed, target_speed_mph=0,
                           distance_m=gap, closing_speed_mph=speed, ttc_s=4 if speed else float("inf"),
                           fcw=int(time > 0), aeb_active=int(time > 0), brake_percent=80 if time else 0,
                           ego_steer=0, ego_accel_mps2=-2 if time > 1 else 0))
    return result


PROVENANCE = {"source": "deterministic_1d_model", "shared_settings": {"dt_s": 1},
              "controllers": {"pid": "pid", "mpc": "mpc"}}


class MetricsTests(unittest.TestCase):
    def test_stop_distance_uses_trapezoids_from_activation(self):
        result = summarize_run(example_rows())
        self.assertTrue(result["valid_for_comparison"])
        self.assertAlmostEqual(result["stop_distance_from_aeb_m"], 4.4704)
        self.assertEqual(result["stop_time_from_aeb_s"], 2)
        self.assertAlmostEqual(result["peak_deceleration_fd_mps2"], 2.2352)
        self.assertAlmostEqual(result["peak_absolute_jerk_fd_mps3"], 2.2352)
        self.assertIsNone(result["controller_runtime_ms"])

    def test_nonuniform_time_derivatives(self):
        rows = example_rows()
        rows[2]["time_s"], rows[3]["time_s"] = 3, 5
        result = summarize_run(rows)
        self.assertAlmostEqual(result["stop_distance_from_aeb_m"], 8.9408)
        self.assertAlmostEqual(result["peak_deceleration_fd_mps2"], 1.1176)
        self.assertAlmostEqual(result["peak_absolute_jerk_fd_mps3"], 1.1176 / 1.5)

    def test_missing_aeb_and_terminal_are_diagnostics(self):
        rows = example_rows()
        for row in rows:
            row.update(aeb_active=0, phase="CRUISE")
        result = summarize_run(rows)
        self.assertFalse(result["valid_for_comparison"])
        self.assertIn("no_aeb_activation", result["invalid_reasons"])
        self.assertIn("missing_terminal_phase", result["invalid_reasons"])
        self.assertIsNone(result["stop_distance_from_aeb_m"])

    def test_overlap_never_valid_even_if_marked_complete(self):
        for phase in ("ABORTED", "COMPLETE"):
            rows = example_rows()
            rows[-1].update(phase=phase, distance_m=-0.1)
            result = compare_runs(rows, rows, provenance=PROVENANCE)
            self.assertFalse(result["comparison_valid"])
            self.assertTrue(result["runs"]["pid"]["gap_overlap_proxy"])
            self.assertIsNone(result["runs"]["pid"]["collision_sensor_contact"])
            self.assertTrue(all(v is None for v in result["mpc_minus_pid"].values()))

    def test_bad_numbers_flags_and_time_rejected(self):
        for field, value in [("time_s", float("nan")), ("distance_m", float("inf")),
                             ("ttc_s", -float("inf")), ("ego_speed_mph", -1),
                             ("fcw", 2), ("aeb_active", 0.5), ("brake_percent", 101), ("ego_steer", 2)]:
            with self.subTest(field=field):
                rows = example_rows()
                rows[1][field] = value
                with self.assertRaises(ValueError):
                    summarize_run(rows)
        rows = example_rows()
        rows[2]["time_s"] = rows[1]["time_s"]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            summarize_run(rows)

    def test_positive_infinite_ttc_is_json_safe(self):
        rows = example_rows()
        for row in rows:
            row["ttc_s"] = float("inf")
        result = summarize_run(rows)
        self.assertIsNone(result["minimum_finite_ttc_s"])
        json.dumps(result, allow_nan=False)

    def test_pre_aeb_mismatch_disables_comparison(self):
        pid, mpc = example_rows(), example_rows()
        mpc[0]["distance_m"] += 0.1
        result = compare_runs(pid, mpc, provenance=PROVENANCE)
        self.assertFalse(result["comparison_valid"])
        self.assertFalse(result["pre_aeb_traces_match"])

    def test_provenance_required_override_diagnostic(self):
        with self.assertRaisesRegex(ValueError, "provenance"):
            compare_runs(example_rows(), example_rows())
        result = compare_runs(example_rows(), example_rows(), allow_unverified=True)
        self.assertFalse(result["comparison_valid"])
        paired = {"pid": {"source": "model", "controller": "pid", "shared_settings": {"dt": .05}},
                  "mpc": {"source": "model", "controller": "mpc", "shared_settings": {"dt": .1}}}
        with self.assertRaisesRegex(ValueError, "settings"):
            compare_runs(example_rows(), example_rows(), provenance=paired)

    def test_header_only_duplicate_headers_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.csv"
            path.write_text(",".join(FIELDNAMES) + "\n")
            with self.assertRaisesRegex(ValueError, "no samples"):
                load_csv(path)
            path.write_text(",".join(FIELDNAMES + ["phase"]) + "\n")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                load_csv(path)

    def test_metadata_hash_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pid.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
                writer.writeheader()
                writer.writerows(example_rows())
            path.with_suffix(".metadata.json").write_text(json.dumps(
                {"csv_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}))
            self.assertIsNotNone(load_metadata(path))
            rows = load_csv(path)
            report = compare_runs(rows, rows, provenance=PROVENANCE)
            paths = write_report(report, rows, rows, Path(directory) / "report", plots=False)
            self.assertTrue(json.loads(Path(paths["json"]).read_text())["comparison_valid"])
            path.write_text(path.read_text() + "\n")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                load_metadata(path)

    def test_stop_threshold_and_rebound(self):
        for threshold in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                summarize_run(example_rows(), threshold)
        rows = example_rows()
        rows[2]["ego_speed_mph"], rows[-1]["ego_speed_mph"] = 0.5, 2
        self.assertIn("speed_rebounded_after_stop_threshold", summarize_run(rows)["invalid_reasons"])
