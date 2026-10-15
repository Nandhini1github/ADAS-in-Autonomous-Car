"""Replay scenario CSV observations onto typed SI telemetry."""
import time
import uuid
from rclpy.node import Node
from town10_adas.metrics import load_csv
from town10_adas.units import mph_to_mps
from town10_adas_interfaces.msg import Telemetry
from .common import parameter, positive, run_node, telemetry_qos, validate_telemetry


def read_samples(path, run_id):
    samples = []
    for index, row in enumerate(load_csv(path)):
        msg = Telemetry()
        msg.run_id, msg.sequence = run_id, index
        msg.source_time_s = row["time_s"]
        msg.phase = row["phase"]
        msg.ego_speed_mps = mph_to_mps(row["ego_speed_mph"])
        msg.target_speed_mps = mph_to_mps(row["target_speed_mph"])
        msg.distance_m = row["distance_m"]
        msg.closing_speed_mps = mph_to_mps(row["closing_speed_mph"])
        msg.observed_ttc_s = row["ttc_s"]
        msg.observed_fcw = bool(row["fcw"])
        msg.observed_aeb_active = bool(row["aeb_active"])
        msg.observed_brake_percent = row["brake_percent"]
        msg.ego_steer = row["ego_steer"]
        msg.ego_accel_mps2 = row["ego_accel_mps2"]
        validate_telemetry(msg)
        samples.append(msg)
    return samples


class CsvReplay(Node):
    def __init__(self, **kwargs):
        super().__init__("csv_replay", **kwargs)
        path = parameter(self, "input_csv", "")
        if not path:
            raise ValueError("input_csv is required")
        self.rate = positive(parameter(self, "rate", 1.0), "rate")
        self.delay = positive(parameter(self, "start_delay_s", 1.0), "start_delay_s")
        self.frame = parameter(self, "frame_id", "base_link")
        self.samples = read_samples(path, uuid.uuid4().hex)
        self.publisher = self.create_publisher(
            Telemetry, parameter(self, "telemetry_topic", "/adas/telemetry"), telemetry_qos(self))
        self.index, self.started, self.complete = 0, None, False
        self.timer = self.create_timer(0.005, self.tick)
        self.get_logger().info(f"Loaded {len(self.samples)} rows; waiting for subscriber")

    def tick(self):
        if self.started is None:
            if self.publisher.get_subscription_count() == 0:
                return
            self.started = time.monotonic() + self.delay
        elapsed = (time.monotonic() - self.started) * self.rate
        published = 0
        while self.index < len(self.samples) and published < 100:
            msg = self.samples[self.index]
            if msg.source_time_s - self.samples[0].source_time_s > elapsed:
                break
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            self.publisher.publish(msg)
            self.index += 1
            published += 1
        if self.index == len(self.samples):
            self.timer.cancel()
            self.complete = True
            self.get_logger().info(f"Replay complete: {self.index} rows (Ctrl-C to stop)")


def main(args=None):
    run_node(CsvReplay, args)
