#!/usr/bin/env python3
"""Read-only checks for Ubuntu 24.04, ROS Jazzy, Autoware and Docker CARLA."""
import argparse
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import sys


def module_available(name):
    try:
        importlib.import_module(name)
        return True
    except (ImportError, OSError):
        return False


def collect_report(host, port):
    release = platform.freedesktop_os_release()
    kernel = platform.release()
    try:
        with socket.create_connection((host, port), timeout=1.0):
            reachable = True
    except OSError:
        reachable = False
    docker = {"available": shutil.which("docker") is not None}
    if docker["available"]:
        try:
            result = subprocess.run(["docker", "version", "--format", "{{.Server.Version}}"],
                                    capture_output=True, text=True, timeout=5)
            docker["server_version"] = result.stdout.strip() if result.returncode == 0 else None
        except (OSError, subprocess.TimeoutExpired):
            docker["server_version"] = None
    return {
        "os": release.get("PRETTY_NAME"), "ubuntu_24_04": (
            release.get("ID") == "ubuntu" and release.get("VERSION_ID") == "24.04"),
        "kernel": kernel, "wsl_detected": "microsoft" in kernel.lower(),
        "python": platform.python_version(), "python_executable": sys.executable,
        "python_3_12": sys.version_info[:2] == (3, 12),
        "ros_distro": os.environ.get("ROS_DISTRO"), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "ROS default"),
        "rclpy": module_available("rclpy"), "colcon": shutil.which("colcon") is not None,
        "autoware_vehicle_msgs": module_available("autoware_vehicle_msgs.msg"),
        "carla_python_api": module_available("carla"), "carla_host": host, "carla_rpc_port": port,
        "carla_tcp_reachable": reachable, "docker": docker,
        "note": "TCP reachability is not a CARLA protocol/version check; use check_carla_integration.py.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-wsl", action="store_true")
    parser.add_argument("--require-autoware", action="store_true")
    parser.add_argument("--require-carla", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    report = collect_report(args.host, args.port)
    required = ["ubuntu_24_04", "python_3_12", "rclpy", "colcon"]
    if args.require_wsl:
        required.append("wsl_detected")
    if args.require_autoware:
        required.append("autoware_vehicle_msgs")
    if args.require_carla:
        required += ["carla_python_api", "carla_tcp_reachable"]
    failures = [key for key in required if not report[key]]
    if report["ros_distro"] != "jazzy":
        failures.append("ros_distro must be jazzy; source the matching underlay")
    report["failed_checks"] = failures
    print(json.dumps(report, indent=2))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
