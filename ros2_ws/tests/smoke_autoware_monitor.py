#!/usr/bin/env python3
"""Verify Autoware VelocityReport reaches the optional monitor over DDS."""
import math
import time
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from autoware_vehicle_msgs.msg import VelocityReport
from town10_adas_monitor.vehicle_monitor import VehicleMonitor


def main():
    rclpy.init(args=["--ros-args", "-p", f"velocity_topic:=/adas_monitor_smoke_{uuid.uuid4().hex}"])
    monitor, sender = VehicleMonitor(), Node("adas_monitor_test_sender")
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(sender)
    publisher = sender.create_publisher(
        VelocityReport, monitor.get_parameter("velocity_topic").value, qos_profile_sensor_data)
    message = VelocityReport()
    message.longitudinal_velocity = 4.4704
    try:
        deadline = time.monotonic() + 5
        while monitor.speed_mph is None and time.monotonic() < deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
        assert monitor.speed_mph is not None, "Autoware velocity message was not received"
        assert math.isclose(monitor.speed_mph, 10.0, abs_tol=1e-5)
        print("PASS Autoware VelocityReport -> monitor: 4.4704 m/s = 10 mph")
    finally:
        executor.remove_node(monitor)
        executor.remove_node(sender)
        monitor.destroy_node()
        sender.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
