#!/usr/bin/env python3
"""Measure how well ONE guest node's notion of sim time tracks the host's clock.

Why this exists. In the split topology every guest node gets its time from
/clock across the TCP link. Measured 2026-08-17, bt_navigator's clock ran at
0.21x wall while the simulation ran at 0.73x, so it fell steadily further behind
the TF stamps arriving beside it and aborted every goal with

    Requested time 307.130000 but the earliest data is at time 341.600000
    [bt_navigator]: Initial robot pose is not available.

`ros2 topic hz /clock` cannot see that: it says samples are arriving at 5 Hz,
which they are. The failure is on the consuming side of the subscription.

What it measured, and what that ruled out. Run on the guest 2026-08-17 with the
decimator at 2 Hz:

    idle              sim_rate 1.00x, received 2.00/s, gap 0.00 -- perfect
    after --block 15  clock frozen at 296.00 during the block, then 296 -> 316
                      in the next 4.9 s (sim_rate 4.08x) while received rose by
                      only 10, i.e. at the ordinary 2.04/s publish rate

That second line is the useful one. The clock came back to exactly the current
sim time without consuming a backlog, so the ~30 samples published during the
block were DROPPED: KEEP_LAST(1) behaves as documented even across a TCP
transport, and one clock callback is enough to become current however far behind
the node was.

So a blocking operation leaves no lasting debt, and these two explanations for a
node whose clock lags are dead:

    "samples queue up and are consumed one per executor pass, so the clock
     advances only one decimation period at a time and can never catch up"
        Refuted above. Lowering the decimator's --hz does not address this,
        because this is not what happens.

    "the link is too slow to carry the clock"
        Refuted by the idle line: 100% delivery at the published rate.

What survives is starvation of the consumer. A node whose clock reads N seconds
old has not run its clock callback for N seconds of sim time, and on this guest
(4 TCG-emulated cores, ~8 ROS processes, Ceres among them) the scarce resource is
CPU, not messages. rclcpp gives TimeSource its own callback group and its own
thread precisely so a busy executor cannot do this, which points at the scheduler
rather than at any Nav2 parameter.

use_sim_time must be TRUE here -- the whole point is to reproduce the condition
bt_navigator runs under. That is the opposite of clock_decimator.py, which
defines sim time and must not consume it.

    ros2 run ... python3 clock_lag_probe.py --ros-args -p use_sim_time:=true

--load N adds N no-op timer callbacks at 100 Hz to the same single-threaded
executor, which is what a busy node looks like from the clock's point of view.

--block SEC blocks that executor once, for SEC seconds, imitating what
bt_navigator does on this guest, where parsing one behaviour-tree XML takes ~9 s
during which nothing else in the node runs. It is the flag that produced the
measurement above. Read the first REPORT after the block, not the unblock line
itself, which prints before anything has been consumed.

Both flags turn this into a reproducer that runs in a minute instead of a
7-minute G5.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock

# rclpy has no ClockQoS constant, and the profile is not a detail: rclcpp's
# TimeSource subscribes with KEEP_LAST(1)/BEST_EFFORT/VOLATILE, so a probe on
# any other profile would measure a different subscription than the one whose
# behaviour is in question.
CLOCK_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def stamp_seconds(msg):
    return msg.clock.sec + msg.clock.nanosec * 1e-9


class ClockLagProbe(Node):
    def __init__(self, args):
        super().__init__('clock_lag_probe')
        if not self.get_parameter('use_sim_time').value:
            raise SystemExit('clock_lag_probe: use_sim_time must be true -- with it '
                             'false get_clock() is wall time and there is nothing to '
                             'measure. Pass --ros-args -p use_sim_time:=true')
        self.received = 0
        self.raw_latest = None
        # Steady, not time(): a wall reference that a clock change cannot move.
        self.t0_wall = time.monotonic()
        self.last_wall = self.t0_wall
        self.last_received = 0
        self.last_node_clock = None
        self.warned = False

        self.create_subscription(Clock, args.topic, self._on_clock, CLOCK_QOS)
        # create_timer under use_sim_time is a ROS timer driven by /clock, which
        # would stop reporting exactly when the clock stalls -- the one case this
        # has to report. create_timer with a steady clock keeps ticking.
        self.create_timer(args.every, self._report,
                          clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
        for _ in range(args.load):
            self.create_timer(0.01, lambda: None,
                              clock=rclpy.clock.Clock(
                                  clock_type=rclpy.clock.ClockType.STEADY_TIME))
        if args.load:
            self.get_logger().info(f'{args.load} competing 100 Hz callbacks in this executor')
        self.block = args.block
        if self.block:
            self.block_timer = self.create_timer(
                args.block_at, self._block_once,
                clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
            self.get_logger().info(f'will block this executor for {self.block}s, once, '
                                   f'{args.block_at}s from now')
        self.get_logger().info(f'clock_lag_probe on {args.topic}, reporting every '
                               f'{args.every}s (use_sim_time=true)')

    def _block_once(self):
        self.block_timer.cancel()
        before = self.node_clock_seconds()
        print(f'--- blocking the executor for {self.block}s '
              f'(node_clock {before:.2f} on entry)', flush=True)
        # time.sleep and not a busy loop: the point is to hold the executor
        # thread, not to burn a core the rest of the guest needs.
        time.sleep(self.block)
        print(f'--- unblocked. node_clock is now {self.node_clock_seconds():.2f}; it was '
              f'{before:.2f}. Read the next report before drawing a conclusion -- nothing '
              f'has been consumed yet at this instant.', flush=True)

    def _on_clock(self, msg):
        self.received += 1
        self.raw_latest = stamp_seconds(msg)

    def _report(self):
        now_wall = time.monotonic()
        d_wall = now_wall - self.last_wall
        if d_wall <= 0:
            return
        # get_clock() here is the TimeSource's clock -- the same object every
        # rclcpp/rclpy node uses for now() and for TF lookups.
        node_clock = self.node_clock_seconds()
        d_recv = self.received - self.last_received
        rate = d_recv / d_wall
        if self.last_node_clock is None:
            slope = float('nan')
        else:
            slope = (node_clock - self.last_node_clock) / d_wall
        # raw_latest comes from a subscription in this same executor, so it is
        # starved alongside the clock and this gap is normally ~0. It is printed
        # because a NONZERO gap would mean the two subscriptions are being served
        # unequally, which is a different fault from the one described above.
        gap = (self.raw_latest - node_clock) if self.raw_latest is not None else float('nan')
        print(f'wall={now_wall - self.t0_wall:7.1f}s  node_clock={node_clock:9.2f}  '
              f'sim_rate={slope:5.2f}x  received={self.received:6d} ({rate:5.2f}/s)  '
              f'raw-node={gap:+6.2f}', flush=True)
        # A clock slower than the simulation never recovers on its own, so say so
        # once rather than leaving it to be read out of a column of numbers.
        if not self.warned and self.last_node_clock is not None and slope < 0.5:
            self.warned = True
            print(f'WARNING: this node\'s clock is advancing at {slope:.2f}x wall. If the '
                  f'simulation is faster than that, the gap to the TF stamps grows without '
                  f'bound and every TF lookup at now() will fail.', flush=True)
        self.last_wall = now_wall
        self.last_received = self.received
        self.last_node_clock = node_clock

    def node_clock_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/clock')
    ap.add_argument('--every', type=float, default=5.0,
                    help='wall seconds between reports')
    ap.add_argument('--load', type=int, default=0,
                    help='competing 100 Hz no-op callbacks, to imitate a busy node')
    ap.add_argument('--block', type=float, default=0.0,
                    help='block the executor once for this many seconds, imitating '
                         'a long parse or plugin load')
    ap.add_argument('--block-at', type=float, default=30.0,
                    help='wall seconds from start at which to block')
    # argparse must not see --ros-args; remove_ros_args strips it and everything
    # it owns. Same reason as pose_truth_probe.py.
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = ClockLagProbe(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(f'total received {node.received}, final node_clock '
              f'{node.node_clock_seconds():.2f}', flush=True)
        node.destroy_node()
        # rclpy.shutdown() raises if the signal handler already ran, which it has
        # whenever this was stopped with SIGINT.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
