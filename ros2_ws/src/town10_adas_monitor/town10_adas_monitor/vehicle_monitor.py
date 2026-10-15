"""Observe Autoware vehicle velocity without taking command authority."""

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

MPS_TO_MPH = 2.2369362921


class VehicleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("town10_adas_vehicle_monitor")
        topic = self.declare_parameter("velocity_topic", "/vehicle/status/velocity_status").value
        self.speed_mph = None
        self.create_subscription(
            VelocityReport,
            topic,
            self.on_velocity,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self.log_velocity)
        self.get_logger().info(f"Monitoring {topic} (read-only, sensor QoS)")

    def on_velocity(self, message: VelocityReport) -> None:
        self.speed_mph = message.longitudinal_velocity * MPS_TO_MPH

    def log_velocity(self) -> None:
        if self.speed_mph is not None:
            self.get_logger().info(f"Last Autoware ego speed: {self.speed_mph:.2f} mph")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
