#!/usr/bin/env python3
"""Exercise real DDS replay/controller/recorder, both controllers, and bad input."""
import argparse
import copy
import csv
import json
import math
from pathlib import Path
import tempfile
import time
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from town10_adas_ros.aeb_shadow import AebShadow
from town10_adas_ros.csv_replay import CsvReplay
from town10_adas_ros.metrics_recorder import MetricsRecorder


def overrides(**values):
    return [Parameter(name, value=value) for name, value in values.items()]


def spin_until(executor, predicate, timeout):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "ROS pipeline timed out"


def exercise(input_path, output_path, mode):
    suffix = uuid.uuid4().hex
    telemetry, command = f"/adas_smoke_{suffix}/telemetry", f"/adas_smoke_{suffix}/shadow"
    executor, nodes = SingleThreadedExecutor(), []
    try:
        recorder = MetricsRecorder(parameter_overrides=overrides(
            output_csv=str(output_path), command_topic=command))
        nodes.append(recorder)
        shadow = AebShadow(parameter_overrides=overrides(
            controller=mode, telemetry_topic=telemetry, command_topic=command, stale_timeout_s=1.0))
        nodes.append(shadow)
        replay = CsvReplay(parameter_overrides=overrides(
            input_csv=str(input_path), telemetry_topic=telemetry, rate=10.0, start_delay_s=0.5))
        nodes.append(replay)
        for node in nodes:
            executor.add_node(node)
        expected = len(replay.samples)
        duration = replay.samples[-1].source_time_s - replay.samples[0].source_time_s
        spin_until(executor, lambda: recorder.valid_count == expected, max(30, duration / 10 + 10))
        spin_until(executor, lambda: recorder.invalid_count >= 1, 3)
        assert replay.complete
        duplicate = copy.deepcopy(replay.samples[-1])
        duplicate.header.stamp = replay.get_clock().now().to_msg()
        replay.publisher.publish(duplicate)
        spin_until(executor, lambda: recorder.invalid_count >= 2, 3)
        stale = copy.deepcopy(duplicate)
        stale.header.stamp.sec = 1
        replay.publisher.publish(stale)
        spin_until(executor, lambda: recorder.invalid_count >= 3, 3)
        malformed = copy.deepcopy(duplicate)
        malformed.header.stamp = replay.get_clock().now().to_msg()
        malformed.ego_speed_mps = math.nan
        replay.publisher.publish(malformed)
        spin_until(executor, lambda: recorder.invalid_count >= 4, 3)
        assert recorder.valid_count == expected
        recovery = copy.deepcopy(replay.samples[0])
        recovery.run_id = uuid.uuid4().hex
        recovery.header.stamp = replay.get_clock().now().to_msg()
        replay.publisher.publish(recovery)
        spin_until(executor, lambda: recorder.valid_count == expected + 1, 3)
        original_run = replay.samples[0].run_id
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
    with input_path.open(newline="") as stream:
        source_rows = list(csv.DictReader(stream))
    with output_path.open(newline="") as stream:
        recorded_rows = list(csv.DictReader(stream))
    valid = [row for row in recorded_rows if row["source_valid"] == "1" and row["run_id"] == original_run]
    assert len(valid) == len(source_rows)
    assert [int(row["sequence"]) for row in valid] == list(range(expected))
    for source, recorded in zip(source_rows, valid):
        for field, value in source.items():
            if field not in recorded:
                continue
            if field == "phase":
                assert value == recorded[field]
            else:
                assert math.isclose(float(value), float(recorded[field]), rel_tol=1e-9, abs_tol=1e-9), field
        assert 0 <= float(recorded["shadow_brake_percent"]) <= 100
        assert recorded["controller"] == mode
    assert any(float(row["shadow_brake_percent"]) > 0 for row in valid)
    invalid = [row for row in recorded_rows if row["source_valid"] == "0"]
    for reason in ("timeout", "out-of-order", "timestamp", "non-finite"):
        assert any(reason in row["reason"] for row in invalid), reason
    assert all(float(row["shadow_brake_percent"]) == 0 for row in invalid)
    summary = json.loads(output_path.with_suffix(".summary.json").read_text())
    assert summary["valid_samples"] == expected + 1
    print(f"PASS {mode}: {expected} rows preserved; stale, duplicate, NaN, timeout and new-run recovery checked")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    args = parser.parse_args()
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory(prefix="adas-ros-smoke-") as temporary:
            for mode in ("pid", "mpc"):
                exercise(args.input_csv.resolve(), Path(temporary) / f"{mode}.csv", mode)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
