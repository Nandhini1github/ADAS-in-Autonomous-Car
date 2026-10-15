"""Shared PID/MPC evaluation; publishes no vehicle actuation commands."""
import math
import time
from rclpy.node import Node
from town10_adas.aeb import AEBMPC, AEBPID
from town10_adas.safety import SafetySupervisor
from town10_adas_interfaces.msg import ShadowCommand, Telemetry
from .common import parameter, positive, run_node, telemetry_qos, validate_telemetry


class AebShadow(Node):
    def __init__(self, **kwargs):
        super().__init__("aeb_shadow", **kwargs)
        self.controller_name = parameter(self, "controller", "pid")
        if self.controller_name not in ("pid", "mpc"):
            raise ValueError("controller must be pid or mpc")
        self.stale_timeout = positive(parameter(self, "stale_timeout_s", 0.5), "stale_timeout_s")
        self.default_dt = positive(parameter(self, "default_dt_s", 0.05), "default_dt_s")
        max_decel = positive(parameter(self, "max_decel_mps2", 8.0), "max_decel_mps2")
        fcw = positive(parameter(self, "fcw_ttc_s", 3.0), "fcw_ttc_s")
        aeb = positive(parameter(self, "aeb_ttc_s", 2.0), "aeb_ttc_s")
        margin = parameter(self, "distance_margin_m", 5.0)
        self.supervisor = SafetySupervisor(fcw, aeb, margin, max_decel)
        gains = [parameter(self, key, value) for key, value in
                 (("pid_kp", 0.45), ("pid_ki", 0.07), ("pid_kd", 0.02))]
        target = positive(parameter(self, "pid_target_decel_mps2", 6.5), "pid_target_decel_mps2")
        horizon = positive(parameter(self, "mpc_horizon_s", 2.5), "mpc_horizon_s")
        step = positive(parameter(self, "mpc_step_s", 0.05), "mpc_step_s")
        self.target_decel = positive(parameter(self, "target_decel_mps2", 7.0), "target_decel_mps2")
        self.emergency = (AEBPID(*gains, target, max_decel) if self.controller_name == "pid"
                          else AEBMPC(max_decel, horizon, step))
        self.publisher = self.create_publisher(
            ShadowCommand, parameter(self, "command_topic", "/adas/shadow_command"), 100)
        self.subscription = self.create_subscription(
            Telemetry, parameter(self, "telemetry_topic", "/adas/telemetry"),
            self.on_telemetry, telemetry_qos(self))
        self.last_source, self.last_received = None, None
        self.stale_reported = False
        self.timer = self.create_timer(min(0.1, self.stale_timeout / 2), self.watchdog)
        self.get_logger().info(f"{self.controller_name.upper()} SHADOW mode: recommendations only")

    def invalid(self, source, reason):
        output = ShadowCommand()
        output.header.stamp = self.get_clock().now().to_msg()
        if source is not None:
            output.source = source
            output.header.frame_id = source.header.frame_id
        output.controller = self.controller_name
        output.source_valid = False
        output.reason = reason
        output.ttc_s = math.inf
        self.publisher.publish(output)
        self.get_logger().warning(reason)

    def on_telemetry(self, source):
        start = time.perf_counter()
        try:
            validate_telemetry(source)
            stamp = source.header.stamp.sec + source.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if age > self.stale_timeout or age < -self.stale_timeout:
                raise ValueError("stale or future telemetry publication timestamp")
            previous = self.last_source
            new_run = previous is None or source.run_id != previous.run_id
            if not new_run and (source.sequence <= previous.sequence or
                                source.source_time_s <= previous.source_time_s):
                raise ValueError("out-of-order or duplicate telemetry")
            dt = self.default_dt if new_run else source.source_time_s - previous.source_time_s
            if new_run:
                self.supervisor.reset()
                self.emergency.reset()
            assessment = self.supervisor.evaluate(
                source.ego_speed_mps, source.target_speed_mps, source.distance_m)
            brake = 0.0
            if assessment.aeb:
                brake = self.emergency.compute(
                    actual_accel_mps2=source.ego_accel_mps2, dt_s=dt,
                    ego_speed_mps=source.ego_speed_mps, target_speed_mps=source.target_speed_mps,
                    gap_m=source.distance_m, target_decel_mps2=self.target_decel)
            output = ShadowCommand()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = source.header.frame_id
            output.source = source
            output.controller, output.source_valid, output.reason = self.controller_name, True, "ok"
            output.fcw, output.aeb_active = assessment.fcw, assessment.aeb
            output.closing_speed_mps = assessment.closing_speed_mps
            output.ttc_s, output.stopping_distance_m = assessment.ttc_s, assessment.stopping_distance_m
            output.brake_percent = 100.0 * brake
            output.processing_ms = (time.perf_counter() - start) * 1000
            self.publisher.publish(output)
            self.last_source, self.last_received, self.stale_reported = source, time.monotonic(), False
        except (ValueError, OverflowError) as error:
            self.invalid(source, str(error))

    def watchdog(self):
        if (self.last_received is not None and not self.stale_reported
                and time.monotonic() - self.last_received > self.stale_timeout):
            self.invalid(self.last_source, "telemetry timeout; recommendation invalid")
            self.stale_reported = True


def main(args=None):
    run_node(AebShadow, args)
