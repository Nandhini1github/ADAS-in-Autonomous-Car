"""Replay recorded motion through a selectable shadow controller."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = str(Path(get_package_share_directory("town10_adas_ros")) / "config/defaults.yaml")
    params = LaunchConfiguration("params_file")
    recorder = Node(package="town10_adas_ros", executable="metrics_recorder", output="screen",
                    parameters=[params, {"output_csv": LaunchConfiguration("output_csv")}])
    controller = Node(package="town10_adas_ros", executable="aeb_shadow", output="screen",
                      parameters=[params, {"controller": LaunchConfiguration("controller")}])
    replay = Node(package="town10_adas_ros", executable="csv_replay", output="screen",
                  parameters=[params, {"input_csv": LaunchConfiguration("input_csv"),
                      "rate": ParameterValue(LaunchConfiguration("rate"), value_type=float)}])
    arguments = [
        DeclareLaunchArgument("input_csv", description="Absolute path to full scenario CSV"),
        DeclareLaunchArgument("output_csv", description="New CSV file for matched shadow output"),
        DeclareLaunchArgument("controller", default_value="pid", choices=["pid", "mpc"]),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("params_file", default_value=defaults),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=[EmitEvent(event=Shutdown(reason="ADAS node exited"))]))
        for node in (recorder, controller, replay)]
    return LaunchDescription(arguments + handlers + [recorder, controller, replay])
