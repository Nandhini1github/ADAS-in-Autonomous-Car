"""Record observed motion and shadow output from the same embedded sample."""
import csv
import json
from pathlib import Path
from rclpy.node import Node
from town10_adas.csv_logger import FIELDNAMES
from town10_adas.units import mps_to_mph
from town10_adas_interfaces.msg import ShadowCommand
from .common import parameter, run_node

EXTRA_FIELDS = [
    "run_id", "sequence", "controller", "source_valid", "reason",
    "publication_time_s", "received_time_s", "shadow_fcw", "shadow_aeb_active",
    "shadow_ttc_s", "shadow_brake_percent", "shadow_stopping_distance_m", "processing_ms",
]


class MetricsRecorder(Node):
    def __init__(self, **kwargs):
        super().__init__("metrics_recorder", **kwargs)
        output = parameter(self, "output_csv", "")
        if not output:
            raise ValueError("output_csv is required")
        self.path = Path(output).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w" if parameter(self, "overwrite", False) else "x",
                                     newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDNAMES + EXTRA_FIELDS)
        self.writer.writeheader()
        self.stream.flush()
        self.valid_count = self.invalid_count = 0
        self.processing_total = self.processing_max = 0.0
        self.runs, self.controllers = set(), set()
        self.subscription = self.create_subscription(
            ShadowCommand, parameter(self, "command_topic", "/adas/shadow_command"), self.on_command, 100)
        self.get_logger().info(f"Recording shadow output to {self.path}")

    def on_command(self, msg):
        source = msg.source
        self.writer.writerow({
            "time_s": source.source_time_s, "phase": source.phase,
            "ego_speed_mph": mps_to_mph(source.ego_speed_mps),
            "target_speed_mph": mps_to_mph(source.target_speed_mps),
            "distance_m": source.distance_m, "closing_speed_mph": mps_to_mph(source.closing_speed_mps),
            "ttc_s": source.observed_ttc_s, "fcw": int(source.observed_fcw),
            "aeb_active": int(source.observed_aeb_active), "brake_percent": source.observed_brake_percent,
            "ego_steer": source.ego_steer, "ego_accel_mps2": source.ego_accel_mps2,
            "run_id": source.run_id, "sequence": source.sequence, "controller": msg.controller,
            "source_valid": int(msg.source_valid), "reason": msg.reason,
            "publication_time_s": source.header.stamp.sec + source.header.stamp.nanosec * 1e-9,
            "received_time_s": self.get_clock().now().nanoseconds * 1e-9,
            "shadow_fcw": int(msg.fcw), "shadow_aeb_active": int(msg.aeb_active),
            "shadow_ttc_s": msg.ttc_s, "shadow_brake_percent": msg.brake_percent,
            "shadow_stopping_distance_m": msg.stopping_distance_m, "processing_ms": msg.processing_ms,
        })
        self.stream.flush()
        if msg.source_valid:
            self.valid_count += 1
            self.processing_total += msg.processing_ms
            self.processing_max = max(self.processing_max, msg.processing_ms)
            self.runs.add(source.run_id)
            self.controllers.add(msg.controller)
        else:
            self.invalid_count += 1

    def destroy_node(self):
        if not self.stream.closed:
            self.stream.close()
            summary = {
                "evaluation": "shadow replay; source vehicle motion is unchanged",
                "valid_samples": self.valid_count, "invalid_samples": self.invalid_count,
                "runs": sorted(self.runs), "controllers": sorted(self.controllers),
                "mean_processing_ms": self.processing_total / self.valid_count if self.valid_count else None,
                "max_processing_ms": self.processing_max if self.valid_count else None,
                "note": "Replay cannot establish stopping-distance improvements.",
            }
            self.path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        return super().destroy_node()


def main(args=None):
    run_node(MetricsRecorder, args)
