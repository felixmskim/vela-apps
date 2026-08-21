#!/usr/bin/env python3
"""Milestone B acceptance test — static-map global path planning on riscv64/TCG.

Exit 0 = PASS.

WHY THIS DOES NOT USE nav2_simple_commander.BasicNavigator
----------------------------------------------------------
BasicNavigator's constructor builds 12 action clients plus subscriptions,
publishers and service clients — roughly 70 DDS endpoints. Under TCG every one of
those has to be discovered and matched by every other participant, on their
executors. Observed on this guest: bringing up BasicNavigator starved
planner_server's executor long enough that its bond heartbeat lapsed for >60 s and
lifecycle_manager declared "CRITICAL FAILURE: SERVER planner_server IS DOWN" and
tore the stack down. The plan itself was never attempted.

This node opens exactly one action client and one subscription (~7 endpoints).

Every wait has an explicit timeout and prints before it blocks, so a slow step is
always distinguishable from a hung one.

The load-bearing assertion is the LETHAL-CELL CHECK, not path length. If the
static layer silently fails to load, every cell reads free and navfn returns a
clean straight line whose length and pose count both look healthy. Only comparing
the path against the occupancy grid catches that.

Deliberately NOT asserted: "a goal inside a wall must fail". tb3_sandbox walls are
1-2 px (max 0.10 m) thick, so with the default 0.5 m tolerance the planner
correctly satisfies such a goal from a free cell beside it. Asserting failure
there would report a defect that is not one.

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

START = (-1.88, 0.08)
GOAL = (1.83, 0.08)
STRAIGHT = math.dist(START, GOAL)      # 3.71 m
LETHAL = 90                            # trinary map_server emits 100 for occupied

SRV_TIMEOUT = 180.0                    # action server discovery, TCG-generous
PLAN_TIMEOUT = 240.0                   # one ComputePathToPose round trip
MAP_TIMEOUT = 180.0


def say(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


class PlanChecker(Node):
    def __init__(self):
        super().__init__('plan_checker')
        self.grid = None
        qos = QoSProfile(depth=1,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # transient_local is mandatory: map_server publishes /map with
        # KeepLast(1).transient_local().reliable(). A volatile subscriber never
        # matches, which is indistinguishable from "the map failed to load".
        self.create_subscription(OccupancyGrid, 'map', self._on_map, qos)
        self.cli = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def _on_map(self, msg):
        if self.grid is None:
            self.grid = msg

    def pose(self, x, y):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.w = 1.0
        return p

    def wait_map(self, timeout=MAP_TIMEOUT):
        say(f'waiting for /map (transient_local, up to {timeout:.0f}s)...')
        end = time.time() + timeout
        while self.grid is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=1.0)
        return self.grid

    def cost_at(self, x, y):
        i = self.grid.info
        cx = int((x - i.origin.position.x) / i.resolution)
        cy = int((y - i.origin.position.y) / i.resolution)
        if not (0 <= cx < i.width and 0 <= cy < i.height):
            return None
        return self.grid.data[cy * i.width + cx]

    def plan(self, start, goal, timeout=PLAN_TIMEOUT):
        """Return (path_or_None, reason)."""
        g = ComputePathToPose.Goal()
        g.start = self.pose(*start)
        g.goal = self.pose(*goal)
        g.planner_id = 'GridBased'
        g.use_start = True

        fut = self.cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return None, 'goal was never accepted or rejected (timeout)'
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None, 'goal rejected by planner_server'

        rfut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=timeout)
        if not rfut.done():
            return None, 'no result within timeout'
        wrapped = rfut.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return None, f'action status {wrapped.status} (not SUCCEEDED)'
        return wrapped.result.path, 'ok'


def path_len(path):
    q = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(math.dist(q[i], q[i + 1]) for i in range(len(q) - 1))


def main():
    # --dump FILE writes the positive path as CSV (x,y per line) so a run can be
    # inspected or drawn afterwards. Evidence, not decoration.
    dump = None
    if '--dump' in sys.argv:
        dump = sys.argv[sys.argv.index('--dump') + 1]

    rclpy.init()
    n = PlanChecker()
    fails = []

    say(f'waiting for /compute_path_to_pose action server (up to {SRV_TIMEOUT:.0f}s)...')
    if not n.cli.wait_for_server(timeout_sec=SRV_TIMEOUT):
        say('FAIL: planner_server action server never appeared')
        say('      check the launch terminal: is planner_server active?')
        rclpy.shutdown()
        return 1
    say('  action server up')

    if n.wait_map() is None:
        say('FAIL: /map never arrived')
        rclpy.shutdown()
        return 1
    gi = n.grid.info
    say(f'  map {gi.width}x{gi.height} res={gi.resolution:.3f} '
        f'origin=({gi.origin.position.x:.1f},{gi.origin.position.y:.1f})')
    if (gi.width, gi.height) != (384, 384):
        fails.append(f'map is {gi.width}x{gi.height}, expected 384x384')
    for name, (x, y) in (('start', START), ('goal', GOAL)):
        c = n.cost_at(x, y)
        say(f'  {name} ({x}, {y}) -> occupancy {c}')
        if c is None or c >= LETHAL:
            fails.append(f'{name} is not free space (occupancy={c})')

    # ---- 1. positive --------------------------------------------------------
    say('plan 1/3  positive: START -> GOAL across the obstacle bands')
    t0 = time.time()
    path, why = n.plan(START, GOAL)
    dt = time.time() - t0
    if path is None or not path.poses:
        fails.append(f'positive: no path ({why})')
        say(f'  FAILED after {dt:.1f}s: {why}')
    else:
        L = path_len(path)
        a, b = path.poses[0].pose.position, path.poses[-1].pose.position
        bad = []
        for p in path.poses:
            c = n.cost_at(p.pose.position.x, p.pose.position.y)
            if c is not None and c >= LETHAL:
                bad.append((round(p.pose.position.x, 2), round(p.pose.position.y, 2)))
        say(f'  poses={len(path.poses)} length={L:.2f}m (straight={STRAIGHT:.2f}m) '
            f'wall={dt:.1f}s')
        say(f'  first=({a.x:.2f},{a.y:.2f}) last=({b.x:.2f},{b.y:.2f}) '
            f'lethal_waypoints={len(bad)}')
        if dump:
            with open(dump, 'w') as fh:
                fh.write('x,y\n')
                for p in path.poses:
                    fh.write(f'{p.pose.position.x:.4f},{p.pose.position.y:.4f}\n')
            say(f'  path written to {dump}')
        if bad:
            fails.append(f'positive: {len(bad)} waypoints inside lethal cells '
                         f'(first {bad[0]}) -> static layer not applied to the costmap')
        if len(path.poses) < 20:
            fails.append(f'positive: only {len(path.poses)} poses')
        if not (STRAIGHT - 0.05 <= L <= 20.0):
            fails.append(f'positive: length {L:.2f}m outside [{STRAIGHT - 0.05:.2f}, 20.0]')
        if math.dist((a.x, a.y), START) > 0.30:
            fails.append('positive: path does not start at the requested pose')
        if math.dist((b.x, b.y), GOAL) > 0.60:      # GridBased.tolerance is 0.5
            fails.append('positive: path does not end near the goal')

    # ---- 2. negatives: costmap covers x,y in [-10.0, 9.2] -------------------
    for k, (gx, gy) in enumerate(((100.0, 100.0), (-50.0, 0.0)), start=2):
        say(f'plan {k}/3  negative: off-map goal ({gx}, {gy}) must be refused')
        r, why = n.plan(START, (gx, gy))
        if r is not None and r.poses:
            fails.append(f'negative: planner returned a path to off-map goal ({gx},{gy})')
            say('  UNEXPECTED: a path was returned')
        else:
            say(f'  correctly refused ({why})')

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
