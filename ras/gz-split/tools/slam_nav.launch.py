# Guest side of the split topology: Nav2 plus slam_toolbox, nothing else.
#
# What is deliberately absent, and why:
#
#   map_server         -- slam_toolbox publishes /map. Two publishers on a
#                         TRANSIENT_LOCAL topic give the static layer whichever
#                         one it latched first.
#   amcl               -- slam_toolbox owns map->odom in mapping mode.
#   loopback_simulator -- Gazebo is the simulator now. Note that if it were ever
#                         run alongside, publish_map_odom_tf must be false
#                         (declared loopback_simulator.py:84, gated :283) or two
#                         broadcasters fight over map->odom.
#   seed_pose          -- the /initialpose seeding that broke the Milestone A
#                         bringup deadlock is unnecessary: Gazebo publishes the
#                         odom transform from t=0, so the costmaps' wait for a
#                         map->base chain resolves as soon as slam_toolbox emits
#                         its first map->odom.
#   collision_monitor,
#   velocity_smoother  -- collision_monitor zeroes /cmd_vel when a scan misses
#                         its source_timeout, which under TCG is routine.
#   rviz2              -- not packaged for riscv64.
#
# The static-TF strategy differs per robot, because the two robots differ in what
# crosses the bridge. Gazebo's /tf carries only what its own plugins publish --
# the DiffDrive odometry transform -- never the model's fixed joints.
#
#   tb3  odom->base_footprint from Gazebo (gz_waffle sets
#        child_frame_id=base_footprint). The two fixed joints we need are
#        published here by hand, because installing nav2_minimal_tb3_sim in the
#        guest would drag in 602 packages for two numbers.
#
#   tb4  odom->base_link from Gazebo (create3 sets child_frame_id=base_link).
#        The chain base_link->shell_link->rplidar_link plus base_link->
#        base_footprint has more links and a pi/2 yaw, so robot_state_publisher
#        reads them from the URDF instead. Only the expanded .urdf file is
#        needed, not the description package -- robot_state_publisher never
#        loads the meshes it references.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Beside-this-file defaults serve the GUEST, where everything is scp'd into one
# flat directory (~/gz_split). In the container the repo layout splits tools/
# from config/, so these defaults do not resolve there -- every containerized
# caller (run_container_loop.sh) passes the paths explicitly.
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_NAV_PARAMS = os.path.join(HERE, 'nav2_split.yaml')
DEFAULT_SLAM_PARAMS = os.path.join(HERE, 'mapper_params_tcg.yaml')
DEFAULT_URDF = os.path.join(HERE, 'robot.urdf')

# Nav2 nodes, in the order lifecycle_manager must transition them.
LIFECYCLE = [
    ('nav2_planner', 'planner_server', 'planner_server'),
    ('nav2_controller', 'controller_server', 'controller_server'),
    ('nav2_behaviors', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
]

# TurtleBot3 Waffle fixed joints, from gz_waffle.sdf.xacro:
#   base_joint   base_footprint -> base_link   z 0.010
#   lidar_joint  base_link      -> base_scan   x -0.064, z 0.122
TB3_STATIC_TF = [
    ('footprint_to_base_tf', ['--frame-id', 'base_footprint',
                              '--child-frame-id', 'base_link', '--z', '0.010']),
    ('base_to_scan_tf', ['--frame-id', 'base_link',
                         '--child-frame-id', 'base_scan',
                         '--x', '-0.064', '--z', '0.122']),
]


def robot_frames(context, *_args, **_kwargs):
    """Emit the per-robot TF publishers once the launch arguments are resolved."""
    robot = LaunchConfiguration('robot').perform(context)
    urdf = LaunchConfiguration('urdf_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if robot == 'tb3':
        return [Node(package='tf2_ros', executable='static_transform_publisher',
                     name=name, output='log', arguments=args)
                for name, args in TB3_STATIC_TF]

    if not os.path.isfile(urdf):
        raise RuntimeError(
            f'robot=tb4 needs the expanded URDF at {urdf}. Generate it on the '
            'host with "python3 make_world.py --robot tb4" and copy robot.urdf '
            'into this directory.')

    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='log',
                 parameters=[{'robot_description': open(urdf).read(),
                              'use_sim_time': use_sim_time.lower() == 'true'}])]


def generate_launch_description():
    nav_params = LaunchConfiguration('nav_params_file')
    slam_params = LaunchConfiguration('slam_params_file')

    nodes = [
        DeclareLaunchArgument('robot', default_value='tb3',
                              description='tb3 or tb4'),
        DeclareLaunchArgument('urdf_file', default_value=DEFAULT_URDF,
                              description='expanded URDF, tb4 only'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav_params_file', default_value=DEFAULT_NAV_PARAMS),
        DeclareLaunchArgument('slam_params_file', default_value=DEFAULT_SLAM_PARAMS),

        OpaqueFunction(function=robot_frames),

        # slam_toolbox first: the costmaps block in activation until map->odom
        # exists, and this is what publishes it.
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[slam_params]),
    ]

    for pkg, exe, name in LIFECYCLE:
        nodes.append(Node(package=pkg, executable=exe, name=name,
                          output='screen', parameters=[nav_params]))

    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_nav', output='screen', parameters=[nav_params]))

    return LaunchDescription(nodes)
