#!/usr/bin/env python3
"""Milestone A acceptance test — end-to-end NavigateToPose on riscv64/TCG.

Exit 0 = PASS.

Sequence, and why each step exists:

  1. Publish /initialpose, repeatedly, until map->base_link appears in TF.
     loopback_simulator ignores every /cmd_vel while initial_pose is None and does
     not even create its integration timer until the first /initialpose arrives
     (initialPoseCallback -> create_timer). Publishing once and hoping the
     subscriber was already matched is the most likely way for this test to hang.
  2. Send a NavigateToPose goal and poll feedback, so a slow run is visibly
     progressing rather than indistinguishable from a hang.
  3. Verify the ACTUAL final pose from TF, not just the action status. A
     behaviour tree can report SUCCEEDED on a goal checker that was satisfied for
     the wrong reason; the transform is ground truth.

As in check_plan.py, nav2_simple_commander.BasicNavigator is avoided: its
constructor creates ~70 DDS endpoints and the resulting discovery burst starves
the servers' executors under TCG. It also defaults to localizer='amcl', which
never activates here.

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

START = (-1.88, 0.08)
GOAL = (1.83, 0.08)
XY_TOL = 0.25          # must match general_goal_checker.xy_goal_tolerance

SRV_TIMEOUT = 240.0
TF_TIMEOUT = 240.0
NAV_TIMEOUT = 900.0


def say(m):
    print(f'[{time.strftime("%H:%M:%S")}] {m}', flush=True)


class NavChecker(Node):
    def __init__(self):
        super().__init__('nav_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def stamped(self, x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        return p

    def publish_initial_pose(self, x, y):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance[0] = 0.25
        m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.068
        self.initpose_pub.publish(m)

    def robot_xy(self):
        """map -> base_link translation, or None."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def seed_pose_until_tf(self, x, y, timeout=TF_TIMEOUT):
        """Wait for map->base_link, publishing /initialpose only if it is absent.

        run_milestone_a.sh normally seeds during bringup and stops before this
        runs, so the transform is already there and nothing is published. Checking
        first matters: an /initialpose sent after motion has begun re-derives
        map->odom and teleports the robot back to the seeded pose.
        """
        end = time.time() + timeout
        n = 0
        while time.time() < end:
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            xy = self.robot_xy()
            if xy is not None:
                say(f'  map->base_link present ({n} /initialpose sent from here)')
                return xy
            self.publish_initial_pose(x, y)
            n += 1
        return None


def main():
    # --dump FILE records the ground-truth trajectory as t,x,y — the robot's
    # actual TF over time, not the plan it was following.
    dump = None
    if '--dump' in sys.argv:
        dump = sys.argv[sys.argv.index('--dump') + 1]

    rclpy.init()
    n = NavChecker()
    fails = []

    say(f'waiting for /navigate_to_pose action server (up to {SRV_TIMEOUT:.0f}s)...')
    if not n.cli.wait_for_server(timeout_sec=SRV_TIMEOUT):
        say('FAIL: bt_navigator action server never appeared')
        rclpy.shutdown()
        return 1
    say('  action server up')

    say(f'seeding /initialpose at {START} until the simulator starts publishing TF...')
    xy = n.seed_pose_until_tf(*START)
    if xy is None:
        say('FAIL: map->base_link never appeared — loopback_simulator did not start')
        say('      it ignores /cmd_vel and creates no timer until /initialpose arrives')
        rclpy.shutdown()
        return 1
    say(f'  robot starts at ({xy[0]:.2f}, {xy[1]:.2f})')
    if math.dist(xy, START) > 0.30:
        fails.append(f'initial pose {xy} is not at {START}')

    say(f'sending NavigateToPose goal {GOAL}')
    g = NavigateToPose.Goal()
    g.pose = n.stamped(*GOAL)
    t0 = time.time()
    fut = n.cli.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=120.0)
    handle = fut.result() if fut.done() else None
    if handle is None or not handle.accepted:
        say('FAIL: goal rejected or never acknowledged')
        rclpy.shutdown()
        return 1
    say('  goal accepted')

    rfut = handle.get_result_async()
    last = 0.0
    track = []                                    # (t, x, y) ground-truth trajectory
    while not rfut.done() and time.time() - t0 < NAV_TIMEOUT:
        rclpy.spin_once(n, timeout_sec=0.2)
        cur = n.robot_xy()
        if cur:
            track.append((time.time() - t0, cur[0], cur[1]))
        if time.time() - last >= 15.0:            # heartbeat, so progress is visible
            last = time.time()
            if cur:
                say(f'  t+{time.time()-t0:5.0f}s  robot=({cur[0]:.2f},{cur[1]:.2f})  '
                    f'remaining={math.dist(cur, GOAL):.2f}m')
    dt = time.time() - t0

    if dump and track:
        with open(dump, 'w') as fh:
            fh.write('t,x,y\n')
            for t, x, y in track:
                fh.write(f'{t:.2f},{x:.4f},{y:.4f}\n')
        say(f'  trajectory ({len(track)} samples) written to {dump}')

    if not rfut.done():
        fails.append(f'navigation did not finish within {NAV_TIMEOUT:.0f}s')
        say(f'FAIL: timeout after {dt:.0f}s')
    else:
        status = rfut.result().status
        ok = status == GoalStatus.STATUS_SUCCEEDED
        say(f'  action finished in {dt:.0f}s, status={status} '
            f'({"SUCCEEDED" if ok else "not SUCCEEDED"})')
        if not ok:
            fails.append(f'action status {status}, expected SUCCEEDED (4)')

    # Ground truth: where did the robot actually end up?
    for _ in range(30):
        rclpy.spin_once(n, timeout_sec=0.1)
    final = n.robot_xy()
    if final is None:
        fails.append('final map->base_link lookup failed')
    else:
        err = math.dist(final, GOAL)
        say(f'  final pose ({final[0]:.2f}, {final[1]:.2f}), '
            f'goal error {err:.3f} m (tolerance {XY_TOL})')
        if err > XY_TOL:
            fails.append(f'final pose is {err:.3f} m from the goal, tolerance {XY_TOL}')

    print()
    if fails:
        say('RESULT: FAIL')
        for f in fails:
            print('   - ' + f)
    else:
        say('RESULT: PASS')
    rclpy.shutdown()
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
