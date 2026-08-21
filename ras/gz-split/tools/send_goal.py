#!/usr/bin/env python3
"""Send NavigateToPose goals and record where the robot actually went.

Why this is not nav2_simple_commander.BasicNavigator:

    BasicNavigator's constructor opens twelve action clients, which is roughly
    seventy DDS endpoints created at once. Measured on this guest, that
    discovery burst starves planner_server's executor long enough that its bond
    heartbeat lapses for over 60 s, lifecycle_manager declares
    "CRITICAL FAILURE: SERVER planner_server IS DOWN" and tears the stack down.
    The plan is never attempted. This node opens one action client, one
    subscription and one TF listener: about seven endpoints.

Ground truth is the TF chain, not the action result. A goal can report SUCCEEDED
while the robot sits still if the goal checker is reading a stale transform, so
the final position is measured independently and the exit code reflects that.

Usage (in the guest, after slam_nav.launch.py is up):
    python3 send_goal.py --waypoints "1.8,0.5 1.8,-1.7 -1.8,-1.7 -1.8,0.5"
"""

import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

# Must match general_goal_checker.xy_goal_tolerance in nav2_split.yaml.
XY_TOL = 0.25

SRV_TIMEOUT = 300.0   # action server availability; bringup is slow under TCG
TF_TIMEOUT = 300.0    # map->base_footprint appearing, i.e. first SLAM map->odom
NAV_TIMEOUT = 900.0   # one goal


class GoalRunner(Node):

    def __init__(self, base_frame='base_footprint', world_offset=(0.0, 0.0)):
        super().__init__('send_goal')
        # tb3 roots at base_footprint; tb4's DiffDrive publishes odom->base_link
        # and hangs base_footprint off it. Both are valid targets, so it is a
        # parameter rather than a constant.
        self.base_frame = base_frame
        # Waypoints are given in world coordinates so they stay comparable to the
        # reference map and to the earlier Milestone runs. slam_toolbox puts the
        # map frame's origin wherever the robot started, so a goal in the map
        # frame is world minus the spawn pose.
        self.world_offset = world_offset
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # slam_toolbox publishes /map TRANSIENT_LOCAL + RELIABLE. A volatile
        # subscriber never matches, and the callback simply never fires.
        self.map_msg = None
        self.create_subscription(
            OccupancyGrid, 'map', self._on_map,
            QoSProfile(depth=1,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.track = []

    def _on_map(self, msg):
        self.map_msg = msg

    def say(self, m):
        self.get_logger().info(m)

    def wait_for_server(self):
        self.say(f'waiting for navigate_to_pose (up to {SRV_TIMEOUT:.0f}s)')
        t0 = time.monotonic()
        while time.monotonic() - t0 < SRV_TIMEOUT:
            if self.ac.wait_for_server(timeout_sec=2.0):
                self.say(f'  server up after {time.monotonic() - t0:.0f}s')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_for_tf(self):
        self.say(f'waiting for map->{self.base_frame} (up to {TF_TIMEOUT:.0f}s)')
        t0 = time.monotonic()
        while time.monotonic() - t0 < TF_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.tf_buffer.can_transform('map', self.base_frame,
                                            rclpy.time.Time(),
                                            timeout=Duration(seconds=0.0)):
                self.say(f'  tf up after {time.monotonic() - t0:.0f}s')
                return True
        return False

    def pose_now(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', self.base_frame,
                                                 rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.0))
        except Exception:
            return None
        t = tr.transform.translation
        return (t.x, t.y)

    def navigate(self, x, y, index):
        # Convert before building the pose, not after. Waypoints are given in
        # world coordinates so they stay comparable across runs, but the SLAM map
        # frame has its origin at the robot's spawn pose, so the goal has to be
        # shifted by it.
        gx = x - self.world_offset[0]
        gy = y - self.world_offset[1]
        self.say(f'[wp{index}] goal world ({x:.2f}, {y:.2f}) '
                 f'-> map ({gx:.2f}, {gy:.2f})')

        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(gx)
        p.pose.position.y = float(gy)
        p.pose.orientation.w = 1.0
        goal.pose = p
        send = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.say(f'[wp{index}] goal rejected')
            return None

        result_fut = handle.get_result_async()
        t0 = time.monotonic()
        last_beat = t0
        while not result_fut.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            pos = self.pose_now()
            if pos:
                self.track.append((now - t0, pos[0], pos[1]))
            if now - last_beat >= 15.0:
                last_beat = now
                if pos:
                    rem = math.dist(pos, (gx, gy))
                    self.say(f'[wp{index}]   t+{now - t0:5.0f}s '
                             f'robot=({pos[0]:5.2f},{pos[1]:5.2f}) remaining={rem:.2f}m')
            if now - t0 > NAV_TIMEOUT:
                self.say(f'[wp{index}] NAV_TIMEOUT')
                return None

        status = result_fut.result().status
        elapsed = time.monotonic() - t0
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.say(f'[wp{index}] finished in {elapsed:.0f}s status={status} '
                 f'({"SUCCEEDED" if ok else "not SUCCEEDED"})')
        return status

    def settle(self, max_probes):
        """Send zero-distance goals until bt_navigator serves one, then start.

        bt_navigator's own node clock falls behind its TF buffer during bringup,
        pre-drive holds and DDS discovery bursts (a viewer, a monitor, any late
        participant), and a goal sent while it is behind aborts with "Initial
        robot pose is not available" -- takes 5, 7 and 8 all died this way, at
        16-25 s behind. A fixed pre-drive sleep cannot fix it, because how much
        debt there is depends on what attached and when.

        So probe instead of sleeping: a goal at the robot's CURRENT pose does
        not move it, but bt_navigator must still resolve its pose to serve it --
        which is exactly the operation that fails while the clock lags. Each
        served abort also advances recovery (~1.5-2 sim-s per goal, measured on
        take 8: 16.6 s behind at wp1, 2.7 s by wp8), so the probes pump the
        very clock they are testing. The tour starts only after bt_navigator
        has demonstrably succeeded at one goal.

        Returns True when a probe SUCCEEDED, False when max_probes ran out --
        the caller drives the tour either way; this is a ramp, not a gate.
        """
        for i in range(1, max_probes + 1):
            pos = self.pose_now()
            if pos is None:
                rclpy.spin_once(self, timeout_sec=2.0)
                continue
            # navigate() takes world coordinates; the current map-frame pose
            # converts back through the same offset it will subtract.
            wx = pos[0] + self.world_offset[0]
            wy = pos[1] + self.world_offset[1]
            status = self.navigate(wx, wy, f'settle-{i}')
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.say(f'settle: bt_navigator healthy after {i} probe(s)')
                return True
        self.say(f'settle: NOT healthy after {max_probes} probes -- '
                 'driving anyway so the failure is on the record')
        return False


def parse_waypoints(spec):
    out = []
    for token in spec.split():
        x, y = token.split(',')
        out.append((float(x), float(y)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--waypoints', required=True,
                    help='space-separated x,y pairs in the map frame')
    ap.add_argument('--base-frame', default='base_footprint',
                    help="robot base frame; base_footprint for tb3 and tb4")
    ap.add_argument('--world-offset', default='0,0',
                    help='the robot spawn pose as x,y. Waypoints are world '
                         'coordinates; map-frame goals are world minus this. '
                         'Read it from robot.env (SPAWN_X, SPAWN_Y).')
    ap.add_argument('--track-csv', default='track_split.csv')
    ap.add_argument('--settle', type=int, default=0, metavar='N',
                    help='before the tour, send up to N zero-distance goals '
                         'until bt_navigator serves one -- see settle(). The '
                         'probes do not move the robot and are not scored')
    a = ap.parse_args()

    wps = parse_waypoints(a.waypoints)
    a.world_offset = tuple(float(v) for v in a.world_offset.split(','))

    rclpy.init()
    n = GoalRunner(a.base_frame, a.world_offset)
    rc = 0
    try:
        if not n.wait_for_server():
            n.say('FAIL: action server never appeared')
            return 1
        if not n.wait_for_tf():
            n.say(f'FAIL: map->{a.base_frame} never appeared '
                  '(slam_toolbox published no map->odom)')
            return 1
        if n.map_msg is None:
            # Not fatal for navigation, but it means /map never arrived, which
            # makes the saved-map scoring step meaningless.
            n.say('WARNING: no /map received yet')
        else:
            m = n.map_msg.info
            n.say(f'map {m.width}x{m.height} res={m.resolution:.3f} '
                  f'origin=({m.origin.position.x:.1f},{m.origin.position.y:.1f})')

        if a.settle > 0:
            n.settle(a.settle)

        reached = 0
        for i, (x, y) in enumerate(wps, start=1):
            status = n.navigate(x, y, i)
            final = n.pose_now()
            if final is None:
                n.say(f'[wp{i}] FAIL: no transform to score against')
                rc = 1
                continue
            err = math.dist(final, (x - a.world_offset[0], y - a.world_offset[1]))
            # The action server's SUCCEEDED is the authority on whether the goal
            # was reached -- its goal checker owns xy_goal_tolerance, and it
            # evaluates at the moment of arrival. This measurement is sampled
            # afterwards, so the robot has coasted and map->odom has been corrected
            # since. Requiring the same 0.25 m here turned a genuine SUCCEEDED into
            # "MISSED" over 19 mm and failed the whole run.
            #
            # So the distance stays as a cross-check for a server that claims
            # success without arriving, with a 2x margin, and it can only ever
            # contradict SUCCEEDED by a wide margin, not by a rounding error.
            slack = 2.0 * XY_TOL
            ok = status == GoalStatus.STATUS_SUCCEEDED and err <= slack
            if not ok:
                verdict = 'MISSED'
            elif err <= XY_TOL:
                verdict = 'ok'
            else:
                verdict = 'ok (server SUCCEEDED; sampled after arrival)'
            n.say(f'[wp{i}] final ({final[0]:.2f}, {final[1]:.2f}) '
                  f'error {err:.3f} m (goal tolerance {XY_TOL}, '
                  f'cross-check limit {slack}) -> {verdict}')
            if ok:
                reached += 1
            else:
                rc = 1

        with open(a.track_csv, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['t', 'x', 'y'])
            w.writerows(n.track)
        n.say(f'wrote {a.track_csv} ({len(n.track)} samples)')
        n.say(f'RESULT: {"PASS" if rc == 0 else "FAIL"} '
              f'({reached}/{len(wps)} waypoints reached)')
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
