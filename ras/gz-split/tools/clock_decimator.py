#!/usr/bin/env python3
"""Republish /clock at a lower rate, so the guest's nodes can keep up with it.

Runs on the host, between the bridge and the TCP link.

Why this exists. In the split topology every guest node that sets use_sim_time
carries its own /clock subscription, and Gazebo's clock rate is its physics rate:
max_step_size 0.01 gives 100 Hz. Seven guest nodes against 100 Hz is roughly 700
clock callbacks per second inside a TCG-emulated CPU, on top of /scan, /odom and
/tf. Measured 2026-08-17, that is more than the guest can service: bt_navigator's
own clock ran 65.6 s behind the TF stamps arriving at the same node --

    Extrapolation Error looking up target frame: Lookup would require
    extrapolation into the past. Requested time 489.820000 but the earliest data
    is at time 555.400000, when looking up transform from frame [base_footprint]
    to frame [map]
    [bt_navigator]: Initial robot pose is not available.

-- and every goal aborted on the missing initial pose, with nothing wrong with
the map or the planner. Note what that pair of numbers rules out: the TF stamps
were current, so the shared TCP link was not backlogged. The clock alone was
behind, which points at callback throughput and not at bandwidth.

Nav2 and slam_toolbox need a clock that is monotonic and roughly current. They do
not need 100 Hz.

Decimation is by SIM TIME, not by message count: republish once the incoming clock
has advanced at least `period`. Counting messages would silently change the output
rate whenever the simulator is throttled, which is exactly the G2b configuration.

use_sim_time must be FALSE here, and the node asserts it. This node is what
publishes /clock; taking its own time from /clock would be circular, and the
symptom would be a clock that never starts.

Usage (host container, alongside the bridge):
    python3 clock_decimator.py --in /clock_gz --out /clock --hz 20 \\
        --ros-args -p use_sim_time:=false
"""

import argparse
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock


def stamp_seconds(msg):
    return msg.clock.sec + msg.clock.nanosec * 1e-9


class ClockDecimator(Node):

    def __init__(self, a):
        super().__init__('clock_decimator')
        self.period = 1.0 / a.hz
        self.last_out = None
        self.n_in = 0
        self.n_out = 0

        # rclcpp::ClockQoS on the subscribing side is KEEP_LAST 1 / BEST_EFFORT /
        # VOLATILE, so match it: a deeper queue here would only let stale clock
        # samples pile up, which is the failure being fixed.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Clock, a.out, qos)
        self.create_subscription(Clock, a.inp, self._on_clock, qos)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'{a.inp} -> {a.out}, republishing when sim time advances '
            f'{self.period * 1000:.0f} ms ({a.hz:g} Hz)')

    def _on_clock(self, msg):
        self.n_in += 1
        t = stamp_seconds(msg)
        # Forward the first sample immediately: a guest node that has not yet seen
        # any /clock blocks on time, so the first one must not wait for a period.
        # Also forward on a backwards jump, which is a restarted simulator rather
        # than a late sample -- holding the old value would wedge every consumer.
        if self.last_out is None or t < self.last_out or t - self.last_out >= self.period:
            self.last_out = t
            self.pub.publish(msg)
            self.n_out += 1

    def _report(self):
        if not self.n_in:
            self.get_logger().warn('no clock in yet on the input topic')
            return
        self.get_logger().info(
            f'in {self.n_in}, out {self.n_out} '
            f'({100.0 * self.n_out / self.n_in:.0f}% forwarded)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='inp', default='/clock_gz',
                    help='raw clock from the bridge')
    ap.add_argument('--out', default='/clock', help='decimated clock for the guest')
    ap.add_argument('--hz', type=float, default=20.0)
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    if a.hz <= 0:
        print('FAIL: --hz must be positive', file=sys.stderr)
        return 2
    if a.inp == a.out:
        print(f'FAIL: --in and --out are both {a.inp!r}, which would feed this '
              f'node its own output', file=sys.stderr)
        return 2

    rclpy.init()
    node = ClockDecimator(a)
    if node.get_parameter('use_sim_time').value:
        node.get_logger().error(
            'use_sim_time is true. This node publishes /clock, so taking its own '
            'time from /clock is circular -- run it with '
            '--ros-args -p use_sim_time:=false')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:                           # noqa: BLE001
        print(f'spin ended early: {type(exc).__name__}: {exc}')
    try:
        node.destroy_node()
    except Exception:                                  # noqa: BLE001, S110
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
