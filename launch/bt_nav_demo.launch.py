"""
Launch file for bt_nav_demo.

Usage:
    ros2 launch bt_nav_demo bt_nav_demo.launch.py
    ros2 launch bt_nav_demo bt_nav_demo.launch.py bt_xml:=/path/to/custom_tree.xml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("bt_nav_demo")
    default_bt_xml = os.path.join(pkg_share, "config", "nav_mission.xml")

    bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value=default_bt_xml,
        description="Path to the BehaviorTree XML file",
    )

    bt_nav_demo_node = Node(
        package="bt_nav_demo",
        executable="bt_nav_demo_node",
        name="bt_nav_demo_node",
        output="screen",
        parameters=[
            {"bt_xml": LaunchConfiguration("bt_xml")},
        ],
    )

    return LaunchDescription([
        bt_xml_arg,
        bt_nav_demo_node,
    ])
