#!/usr/bin/env python3
"""Log the simulator's TRUE robot pose beside the SLAM estimate and wheel odometry.

Diagnosis tool for the open G2b failure (RUNLOG.md Step 6): at RTF 0.5 the SLAM
map comes out drawn several times over at rotating angles, and the SLAM pose takes
11-14 discontinuous corrections totalling 0.70-1.05 m, against 2 totalling 0.06 m
at RTF 1.0. Two candidate causes, and they need different fixes:

  wheel odometry is wrong    the robot slipped or collided, DiffDrive integrated
                             motion that did not happen, and slam_toolbox is
                             correcting real odometry error. Look for odom drifting
                             away from truth while SLAM stays close to it.
  scan matching is wrong     odometry is fine and the matcher is placing scans at
                             the wrong pose. Look for odom tracking truth while
                             SLAM departs from it.

Three series, all stamped in sim time:

  truth  gz PosePublisher, bridged to TFMessage. Requires
         `make_world.py --truth-pose`; the measured configuration has no such
         publisher, so this is never a measurement run.
  slam   the map -> base_frame transform, i.e. what Nav2 acts on.
  odom   /odom, i.e. what DiffDrive integrated from the wheels.

truth and slam live in different frames: the SLAM frame's origin is the robot's
start pose, so --world-offset (the spawn pose) converts SLAM to world, exactly as
in send_goal.py and score_map.py. Pass it as --world-offset=-2.0,-0.5.

Usage:
    python3 pose_truth_probe.py --truth-topic /gz_truth --model turtlebot3_waffle \\
        --base-frame base_footprint --world-offset=-2.0,-0.5 \\
        --duration 120 --csv /work/artifacts/truth-tb3.csv
"""

import argparse
import csv
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
    QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    """Yaw from a quaternion, the only component a planar robot needs."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Probe(Node):
    def __init__(self, a):
        super().__init__('pose_truth_probe')
        self.a = a
        self.dx, self.dy = (float(v) for v in a.world_offset.split(','))
        self.rows = []
        self.truth = None
        self.odom = None
        self.warned = set()

        # Sensor-data QoS: the bridge republishes with the gz side's settings and a
        # RELIABLE subscription on a BEST_EFFORT publisher matches nothing at all,
        # which looks exactly like a silent topic.
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(TFMessage, a.truth_topic, self.on_truth, qos)
        self.create_subscription(Odometry, a.odom_topic, self.on_odom, qos)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_timer(1.0 / a.rate, self.sample)

    def on_truth(self, msg):
        for tr in msg.transforms:
            if self.a.model in (tr.child_frame_id, tr.header.frame_id):
                t = tr.transform.translation
                self.truth = (t.x, t.y, yaw_of(tr.transform.rotation))
                return
        # Fall back to the only entry rather than losing a three-minute run to a
        # name mismatch -- the plugin names its topic and its frames after the
        # ENTITY, which the spawn command renames away from the SDF's model name.
        if len(msg.transforms) == 1:
            tr = msg.transforms[0]
            if 'fallback' not in self.warned:
                self.warned.add('fallback')
                self.get_logger().warn(
                    f'{self.a.model!r} not in this message; using the only entry '
                    f'{tr.child_frame_id!r}')
            t = tr.transform.translation
            self.truth = (t.x, t.y, yaw_of(tr.transform.rotation))
            return
        if 'model' not in self.warned:
            self.warned.add('model')
            names = [t.child_frame_id for t in msg.transforms]
            self.get_logger().warn(
                f'{self.a.model!r} not among {names} on {self.a.truth_topic}')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def sample(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        if self.truth is None or self.odom is None:
            return
        try:
            tf = self.buf.lookup_transform('map', self.a.base_frame,
                                           rclpy.time.Time())
        except Exception as exc:                       # noqa: BLE001 - any TF error
            if 'tf' not in self.warned:
                self.warned.add('tf')
                self.get_logger().warn(f'map->{self.a.base_frame} not ready: {exc}')
            return

        tr = tf.transform.translation
        # SLAM frame -> world, so the three series are comparable.
        sx = tr.x + self.dx
        sy = tr.y + self.dy
        syaw = yaw_of(tf.transform.rotation)
        tx, ty, tyaw = self.truth
        # Odometry starts at the spawn pose too, and has no absolute reference.
        ox = self.odom[0] + self.dx
        oy = self.odom[1] + self.dy
        oyaw = self.odom[2]

        self.rows.append({
            't': f'{t:.4f}',
            'truth_x': f'{tx:.4f}', 'truth_y': f'{ty:.4f}',
            'truth_yaw': f'{tyaw:.4f}',
            'slam_x': f'{sx:.4f}', 'slam_y': f'{sy:.4f}', 'slam_yaw': f'{syaw:.4f}',
            'odom_x': f'{ox:.4f}', 'odom_y': f'{oy:.4f}', 'odom_yaw': f'{oyaw:.4f}',
            'slam_err': f'{math.hypot(sx - tx, sy - ty):.4f}',
            'odom_err': f'{math.hypot(ox - tx, oy - ty):.4f}',
            'slam_yaw_err': f'{wrap(syaw - tyaw):.4f}',
            'odom_yaw_err': f'{wrap(oyaw - tyaw):.4f}',
        })


FIELDS = ['t', 'truth_x', 'truth_y', 'truth_yaw', 'slam_x', 'slam_y', 'slam_yaw',
          'odom_x', 'odom_y', 'odom_yaw', 'slam_err', 'odom_err',
          'slam_yaw_err', 'odom_yaw_err']


def report(rows):
    """Print the one comparison the whole tool exists for."""
    if not rows:
        print('RESULT: FAIL (no samples -- truth topic, /odom or map TF never came)')
        return 1

    def col(k):
        return [abs(float(r[k])) for r in rows]

    se, oe = col('slam_err'), col('odom_err')
    sy, oy = col('slam_yaw_err'), col('odom_yaw_err')
    span = float(rows[-1]['t']) - float(rows[0]['t'])
    print(f'\nsamples {len(rows)} over {span:.1f} s of sim time')
    print(f'{"series":<26} {"mean":>8} {"p95":>8} {"max":>8}')
    for tag, v in (('position error, slam', se), ('position error, odom', oe)):
        s = sorted(v)
        print(f'{tag:<26} {sum(v)/len(v):8.3f} {s[int(.95*(len(s)-1))]:8.3f} '
              f'{max(v):8.3f}   m')
    for tag, v in (('yaw error, slam', sy), ('yaw error, odom', oy)):
        s = sorted(v)
        print(f'{tag:<26} {math.degrees(sum(v)/len(v)):8.2f} '
              f'{math.degrees(s[int(.95*(len(s)-1))]):8.2f} '
              f'{math.degrees(max(v)):8.2f}   deg')

    # The verdict this tool is for. Thresholds are deliberately loose: the point is
    # which series is worse, not whether either passes a gate.
    print()
    if max(oe) > 2 * max(se) and max(oe) > 0.10:
        print('READS AS: wheel odometry drifted from the simulator while SLAM stayed '
              'closer.\n          Look for slip or contact, not for the scan matcher.')
    elif max(se) > 2 * max(oe) and max(se) > 0.10:
        print('READS AS: odometry tracked the simulator and the SLAM estimate left '
              'it.\n          Look at scan matching and the map, not at the wheels.')
    else:
        print('READS AS: neither series is clearly worse. Compare the two RTFs '
              'before concluding.')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--truth-topic', default='/gz_truth')
    ap.add_argument('--model', default='turtlebot3_waffle')
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--world-offset', default='0,0',
                    help='spawn pose, added to SLAM/odom coords to reach world '
                         'coords. Pass as --world-offset=-2.0,-0.5: argparse reads '
                         'a bare leading-dash token as an option name')
    ap.add_argument('--rate', type=float, default=10.0, help='sample rate, sim Hz')
    ap.add_argument('--duration', type=float, default=180.0, help='sim seconds')
    ap.add_argument('--csv', default='')
    # This node is launched with `--ros-args -p use_sim_time:=true`, and argparse
    # sees those tokens before rclpy does. Strip them, or the process exits 2 with
    # "unrecognized arguments" and the run yields no CSV at all.
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Probe(a)
    # use_sim_time has to be set before the first get_clock() read, and the runner
    # passes it on the command line. Fail loudly rather than silently timing the
    # wall clock, which is the whole bug under investigation.
    if not node.get_parameter('use_sim_time').value:
        node.get_logger().error('use_sim_time is false -- run with '
                                '--ros-args -p use_sim_time:=true')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    start = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            t = node.get_clock().now().nanoseconds / 1e9
            if start is None and node.rows:
                start = t
            if start is not None and t - start >= a.duration:
                break
    # The runner's cleanup pkills this node, so a shutdown mid-spin is the normal
    # ending, not an error -- swallow it and still write what was collected. rclpy
    # signals it two different ways depending on where the spin was: an
    # ExternalShutdownException, or an RCLError from the wait set when the context
    # has already gone. Losing the CSV to either one wastes the whole run.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:                           # noqa: BLE001
        print(f'spin ended early: {type(exc).__name__}: {exc}')

    rows = node.rows
    if a.csv:
        with open(a.csv, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(rows)
        print(f'wrote {a.csv} ({len(rows)} samples)')
    rc = report(rows)
    # Both of these throw if the context is already down, which is exactly the
    # normal ending here, and an exception now would lose the report just printed.
    try:
        node.destroy_node()
    except Exception:                                  # noqa: BLE001, S110
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
