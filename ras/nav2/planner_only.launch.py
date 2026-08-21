# Milestone B launch — map_server + planner_server + lifecycle_manager, plus one
# fake TF. No robot, no simulator, no AMCL.
#
# The static_transform_publisher is not cosmetic: Costmap2DROS::activate() blocks
# waiting for a map -> base_link transform, and with no robot nothing publishes
# one. Without it the costmap never activates and bringup hangs.
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
DEFAULT_PARAMS = os.path.join(HOME, 'nav2_tcg', 'planner_only.yaml')


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP),
        # Verified free cells on tb3_sandbox: (-1.88, 0.08) and (1.83, 0.08) are
        # in the same 4-connected free component (7895 cells) and the straight
        # line between them is blocked by several lethal bands.
        DeclareLaunchArgument('start_x', default_value='-1.88'),
        DeclareLaunchArgument('start_y', default_value='0.08'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='fake_robot_tf', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_link',
                        '--x', start_x, '--y', start_y]),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[params, {'yaml_filename': map_yaml}]),

        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen', parameters=[params]),
    ])
