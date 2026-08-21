#!/usr/bin/env python3
"""Stop the simulated robot when /cmd_vel goes quiet. Runs on the host.

gz-sim-diff-drive-system latches the last commanded velocity indefinitely: it
has no command timeout of its own. In the split topology that turns a guest-side
stall into a robot that keeps driving. Two consequences, both bad:

  - the robot leaves the mapped area and the SLAM graph diverges, so a
    RERI-induced pause shows up as a mapping failure instead of a pause;
  - any downtime measurement becomes fiction, because the observable behaviour
    of "workload stopped" is identical to "workload running steadily".

This node republishes a zero Twist after `timeout` seconds without a command,
so a stalled controller produces a stationary robot.

It publishes to the same topic it subscribes to. That is intentional -- the
bridge only forwards ROS -> gz for /cmd_vel, so the zero it publishes reaches
Gazebo, and the self-receipt merely refreshes the timer with a zero command,
which is idempotent.

Uses sim time: it must expire in the simulator's frame, not the host's, or the
timeout would scale with the real-time factor.

Usage (host container):
    python3 cmd_vel_watchdog.py --ros-args -p use_sim_time:=true
"""

import sys

import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', 2.0)
        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('check_period', 0.25)

        self.timeout = self.get_parameter('timeout').value
        topic = self.get_parameter('topic').value
        period = self.get_parameter('check_period').value

        self.pub = self.create_publisher(Twist, topic, 1)
        self.create_subscription(Twist, topic, self._on_cmd, 10)
        self.create_timer(period, self._check)

        self.last = self.get_clock().now()
        self.stopped = True
        # Inter-arrival intervals on CLOCK_MONOTONIC, wall seconds, kept alongside
        # the deadman logic rather than in a second process.
        #
        # This is the /cmd_vel half of the TSP's topic figure, and the ONLY half of
        # it that is measurable for this topic: geometry_msgs/Twist has no Header,
        # so there is no publish timestamp to subtract, and the publisher is in the
        # guest while this node is on the host, so the DDS stamps carry an
        # unsynchronised clock offset. Arrival interval in one process on one clock
        # is what the correction request (CR-11) proposes for exactly this case.
        #
        # This node is the right place for it because it is already the host-side
        # consumer of /cmd_vel and host CPU is free. It measures nothing useful
        # unless it owns a TCP acceptor -- on the bridge's port it saw no guest
        # traffic at all -- so this arrived with the move to port 42102.
        #
        # self.last is ROS time (sim) because the deadman must be; these are wall.
        self.intervals = []
        self._prev_wall = None
        self.get_logger().info(
            f'watching {topic}, zeroing after {self.timeout:.1f}s of silence')

    def _on_cmd(self, msg):
        self.last = self.get_clock().now()
        moving = any((msg.linear.x, msg.linear.y, msg.linear.z,
                      msg.angular.x, msg.angular.y, msg.angular.z))
        if moving:
            self.stopped = False
        # Nonzero commands only, and that is a correctness fix rather than a filter.
        # This node PUBLISHES a zero Twist to the same topic it subscribes to, so it
        # receives its own deadman messages: counting them made the reported maximum
        # interval the watchdog's own `timeout` instead of anything the link did. A
        # genuine zero from Nav2 at goal completion is excluded too, which costs one
        # sample per leg and is the right trade for a statistic about delivery.
        if not moving:
            return
        now = time.monotonic()
        if self._prev_wall is not None:
            self.intervals.append(now - self._prev_wall)
        self._prev_wall = now

    def _check(self):
        # Already zeroed: nothing to do until a real command arrives.
        if self.stopped:
            return
        age = (self.get_clock().now() - self.last).nanoseconds / 1e9
        if age >= self.timeout:
            self.pub.publish(Twist())
            self.stopped = True
            self.get_logger().warn(
                f'no /cmd_vel for {age:.1f}s (sim) -- published zero velocity')


    def report(self):
        vals = sorted(v * 1000.0 for v in self.intervals)
        if not vals:
            print('\n/cmd_vel inter-arrival: NO MESSAGES SEEN.')
            print('  In a split run that means this node has no TCP acceptor and is')
            print('  blind: check artifacts/watchdog-*.log for "Error binding at'
                  ' port".')
            return

        def pct(q):
            return vals[min(len(vals) - 1, max(0, int(round(q * len(vals))) - 1))]

        print(f'\n/cmd_vel inter-arrival at the host consumer, wall ms, '
              f'n={len(vals)} (nonzero commands only; this node\'s own zero-velocity '
              f'publications are excluded)')
        print(f'  p50 {pct(0.50):.1f}   p99 {pct(0.99):.1f}   max {vals[-1]:.1f}')
        # Not gated here. controller_frequency is 5.0 wall Hz, so ~200 ms is the
        # expected interval and a long one can be Nav2 thinking rather than the
        # link stalling. The correction request asks for non-degradation against a
        # baseline (+20% on p99 and max) rather than an absolute bound, and that
        # comparison belongs to whoever holds the baseline, not to this node.


def main():
    rclpy.init()
    n = CmdVelWatchdog()

    def _bye(_sig, _frm):
        raise KeyboardInterrupt

    # docker exec -d plus teardown means SIGTERM is how this node normally ends,
    # and without this handler the interval report would never be printed.
    signal.signal(signal.SIGTERM, _bye)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.report()
        finally:
            n.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
