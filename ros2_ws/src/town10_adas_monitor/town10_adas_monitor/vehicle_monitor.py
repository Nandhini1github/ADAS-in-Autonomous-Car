"""Observe Autoware vehicle velocity without taking command authority."""

import rclpy
from autoware_vehicle_msgs.msg import VelocityReport
from rclpy.node import Node

MPS_TO_MPH = 2.2369362921


class VehicleMonitor(Node):
    def __init__(self) -> None:
        super().__init__("town10_adas_vehicle_monitor")
        self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.on_velocity,
            10,
        )
        self.get_logger().info("Monitoring /vehicle/status/velocity_status (read-only)")

    def on_velocity(self, message: VelocityReport) -> None:
        speed_mph = message.longitudinal_velocity * MPS_TO_MPH
        self.get_logger().info(f"Autoware ego speed: {speed_mph:.2f} mph")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
