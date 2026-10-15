"""Shared parameter, telemetry validation and shutdown helpers."""
import math
import signal
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def parameter(node, name, default):
    return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value


def positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def telemetry_qos(node):
    reliability = parameter(node, "qos_reliability", "reliable")
    if reliability not in ("reliable", "best_effort"):
        raise ValueError("qos_reliability must be reliable or best_effort")
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                      reliability=(ReliabilityPolicy.RELIABLE if reliability == "reliable"
                                   else ReliabilityPolicy.BEST_EFFORT))


def validate_telemetry(msg):
    fields = ("source_time_s", "ego_speed_mps", "target_speed_mps", "distance_m",
              "closing_speed_mps", "observed_brake_percent", "ego_steer", "ego_accel_mps2")
    if not all(math.isfinite(getattr(msg, field)) for field in fields):
        raise ValueError("non-finite telemetry")
    if not msg.run_id or msg.phase not in {"ACCELERATE", "HOLD", "THREAT", "AEB", "COMPLETE", "ABORTED", "CRUISE"}:
        raise ValueError("run_id is empty or phase is unknown")
    if min(msg.source_time_s, msg.ego_speed_mps, msg.target_speed_mps, msg.closing_speed_mps) < 0:
        raise ValueError("time and speed fields must be nonnegative")
    if not 0 <= msg.observed_brake_percent <= 100 or abs(msg.ego_steer) > 1:
        raise ValueError("invalid brake percent or normalized steer")
    if math.isnan(msg.observed_ttc_s) or msg.observed_ttc_s < 0:
        raise ValueError("TTC must be nonnegative or positive infinity")


def run_node(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal/process-group interrupt and launch's forwarded interrupt can
        # both arrive. Let cleanup finish and the recorder flush its summary.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
