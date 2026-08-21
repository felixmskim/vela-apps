# Milestone A launch — five lifecycle nodes, the loopback kinematic simulator,
# and one static transform.
#
# Deliberately NOT nav2_bringup's tb3_loopback_simulation.launch.py, for four
# reasons documented in loopback_nav.yaml and here:
#
#   1. use_rviz defaults True there; ros-jazzy-rviz2 is not packaged for riscv64,
#      so the launch would fail outright.
#   2. It brings up robot_state_publisher with the TurtleBot3 URDF from
#      nav2_minimal_tb3_sim, whose apt closure includes ros-gz-bridge, ros-gz-image
#      and ros-gz-sim. The URDF contributes exactly two fixed joints we care about;
#      one static_transform_publisher replaces them.
#   3. It runs the full 10-node navigation_launch.py. Of those, collision_monitor
#      (source_timeout 1.0) silently zeroes /cmd_vel when TCG cannot deliver a scan
#      in time, and velocity_smoother/docking_server add 20-50 Hz timers. Five
#      nodes are enough for NavigateToPose.
#   4. It sets use_sim_time True and relays /clock from the simulator, which makes
#      one Python timer a global point of failure. See loopback_nav.yaml.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
DEFAULT_MAP = os.path.join(
    HOME, 'nav2_data', 'opt', 'ros', 'jazzy', 'share', 'nav2_bringup',
    'maps', 'tb3_sandbox.yaml')
DEFAULT_PARAMS = os.path.join(HOME, 'nav2_tcg', 'loopback_nav.yaml')

LIFECYCLE = [
    ('nav2_map_server',        'map_server',        'map_server'),
    ('nav2_planner',           'planner_server',    'planner_server'),
    ('nav2_controller',        'controller_server', 'controller_server'),
    ('nav2_behaviors',         'behavior_server',   'behavior_server'),
    ('nav2_bt_navigator',      'bt_navigator',      'bt_navigator'),
]


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    nodes = [
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP),

        # loopback_simulator calls getBaseToLaserTf() and needs this at startup.
        # Offsets are the TurtleBot3 Waffle values from the URDF we are not loading.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_scan_tf', output='log',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_scan',
                        '--x', '-0.064', '--z', '0.122']),

        Node(package='nav2_loopback_sim', executable='loopback_simulator',
             name='loopback_simulator', output='screen', parameters=[params]),
    ]

    for pkg, exe, name in LIFECYCLE:
        extra = [{'yaml_filename': map_yaml}] if name == 'map_server' else []
        nodes.append(Node(package=pkg, executable=exe, name=name,
                          output='screen', parameters=[params] + extra))

    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen', parameters=[params]))

    return LaunchDescription(nodes)
