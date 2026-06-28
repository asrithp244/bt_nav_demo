"""
bt_nav_eval — launch file
Usage:
  ros2 launch bt_nav_eval bt_nav_eval.launch.py
  ros2 launch bt_nav_eval bt_nav_eval.launch.py scenarios_file:=/path/to/custom.yaml
  ros2 launch bt_nav_eval bt_nav_eval.launch.py output_dir:=/tmp/my_results
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("bt_nav_eval")
    default_scenarios = os.path.join(pkg_dir, "config", "scenarios.yaml")
    default_output    = "/tmp/bt_nav_eval_results"

    # ── Declare launch arguments ────────────────────────────────────────────
    scenarios_arg = DeclareLaunchArgument(
        "scenarios_file",
        default_value=default_scenarios,
        description="Path to scenarios.yaml"
    )

    output_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=default_output,
        description="Directory to write metrics.json and evaluation_report.md"
    )

    nav_server_arg = DeclareLaunchArgument(
        "nav_action_server",
        default_value="/navigate_to_pose",
        description="Nav2 NavigateToPose action server name"
    )

    inter_delay_arg = DeclareLaunchArgument(
        "inter_scenario_delay_s",
        default_value="2.0",
        description="Pause between scenarios (seconds) to let Nav2 / costmap settle"
    )

    # ── scenario_runner_node ────────────────────────────────────────────────
    runner_node = Node(
        package    = "bt_nav_eval",
        executable = "scenario_runner_node",
        name       = "scenario_runner",
        output     = "screen",
        parameters = [
            {"scenarios_file":         LaunchConfiguration("scenarios_file")},
            {"output_dir":             LaunchConfiguration("output_dir")},
            {"nav_action_server":      LaunchConfiguration("nav_action_server")},
            {"inter_scenario_delay_s": LaunchConfiguration("inter_scenario_delay_s")},
        ],
    )

    return LaunchDescription([
        scenarios_arg,
        output_arg,
        nav_server_arg,
        inter_delay_arg,
        runner_node,
    ])
