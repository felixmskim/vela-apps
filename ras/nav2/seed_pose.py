#!/usr/bin/env python3
"""Publish /initialpose repeatedly until killed.

Exists to break a circular dependency in bringup:

    global_costmap activation  waits for  map -> base_link
    loopback_simulator         publishes  map -> odom  only after /initialpose
    /initialpose               came from  a client that ran after activation

so nothing ever started. Seeding from launch time onward breaks the cycle, which
is also what a real system does — rviz's "2D Pose Estimate" is available before
the stack is up, not after.

MUST be stopped once the stack is active. Every /initialpose after the first one
re-derives map->odom so that map->base_link equals the seeded pose, which during
navigation teleports the robot back to the start.

    seed_pose.py X Y [RATE_HZ]

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class Seeder(Node):
    def __init__(self, x, y, rate):
        super().__init__('initialpose_seeder')
        self.x, self.y = x, y
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.n = 0

    def tick(self):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = 0.25
        m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.068
        self.pub.publish(m)
        self.n += 1
        if self.n % 10 == 1:
            self.get_logger().info(f'seeded /initialpose ({self.x}, {self.y}) x{self.n}')


def main():
    x = float(sys.argv[1]) if len(sys.argv) > 1 else -1.88
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.08
    rate = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    rclpy.init()
    try:
        rclpy.spin(Seeder(x, y, rate))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
