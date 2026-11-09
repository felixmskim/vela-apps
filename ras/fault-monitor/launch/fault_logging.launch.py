# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Start kernel_monitor and fault_logger together.

    ros2 launch ras_fault_monitor fault_logging.launch.py
    ros2 launch ras_fault_monitor fault_logging.launch.py fault_topic:=/ras/events

The `fault_topic` default is `/fault_log`, from p.13 of the ETRI deck.
The test plan's TC.04.011 (ROS2 Alarm Notification) asks for `/ras/events`, so when
running that test, override the argument as shown above (both nodes share the argument).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    fault_topic = LaunchConfiguration("fault_topic")
    log_dir = LaunchConfiguration("log_dir")
    from_beginning = LaunchConfiguration("from_beginning")

    return LaunchDescription([
        DeclareLaunchArgument("fault_topic", default_value="/fault_log"),
        DeclareLaunchArgument("log_dir", default_value="~/fault_logs"),
        DeclareLaunchArgument(
            "from_beginning", default_value="false",
            description="when true, sweep the whole kernel log from boot",
        ),
        Node(
            package="ras_fault_monitor",
            executable="kernel_monitor",
            name="kernel_monitor",
            output="screen",
            parameters=[{
                "fault_topic": fault_topic,
                "from_beginning": from_beginning,
            }],
        ),
        Node(
            package="ras_fault_monitor",
            executable="fault_logger",
            name="fault_logger",
            output="screen",
            parameters=[{
                "fault_topic": fault_topic,
                "log_dir": log_dir,
            }],
        ),
    ])
