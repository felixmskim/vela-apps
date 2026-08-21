#!/usr/bin/env python3
"""Workload-level outage detector for the guest. Records gaps, does not act.

Nav2 ships its own liveliness mechanism: each lifecycle node holds a bond with
lifecycle_manager, and a lapsed heartbeat tears the stack down. Under TCG that
fired constantly on healthy nodes, so ras/nav2/loopback_nav.yaml sets
bond_timeout and bond_heartbeat_period to 0.0 -- measured, because 60.0 did not
work either. That removes the false positives and the detector with them.

TC.10.007 needs to know when the workload stopped delivering, so the signal has
to be rebuilt outside Nav2. This node watches the topics that constitute
"delivering" and records every gap longer than a threshold.

It also reports the full inter-arrival distribution per topic, because that is the
only defensible way this rig can answer the TSP's topic-latency requirement
("/scan, /odom 토픽이 중단 없이 유지된다 (지연 <= 500ms)", 4th year; 1,000 ms in the
3rd). Publish-to-subscribe latency is NOT measurable across this split and the
reason is worth stating so nobody rebuilds the wrong instrument:

  Every topic the TSP names crosses the machine boundary -- /scan and /odom are
  published by the host bridge and consumed here; /cmd_vel goes the other way --
  and the host and guest clocks are not synchronised. DDS source_timestamp minus
  received_timestamp therefore carries the clock offset (see latency_probe.py,
  which says so and is only valid same-machine).

  Using header.stamp against this node's now() does not rescue it either, and this
  is the trap worth naming. Both sides do share one sim clock by value, so the
  arithmetic looks sound -- but the guest's now() is a sample-and-hold of the last
  /clock sample the guest actually SERVICED, and /clock arrives over the very link
  being measured into a CPU-starved node. What such a probe reports is

      now()_guest - stamp  =  RTF * L_transport  -  LAG_sim

  and LAG_sim, the guest clock's staleness, is what step 7 of run_split.sh gates
  on: measured 0.5-3.1 s in passing runs and up to 6.5 s in failing ones, against
  a wanted signal of 5-25 ms. The instrument would report the negative of the
  clock lag, 20x to 600x the signal, with the wrong sign.

So: inter-arrival gaps here, measured in ONE process on ONE monotonic clock, which
is exactly what "중단 없이 유지" asks about; and rtt_probe.py's round trip (G4,
p95 12.1 ms, one clock, guest->host->guest) for the crossing cost. The correction
request already proposes arrival interval as the substitute for headerless topics
(CR-11); this extends it to the stamped ones for a different and better reason.

Two deliberate choices:

  use_sim_time = false. This is the only node in the guest stack on the wall
  clock. An outage has to be measured against real elapsed time; if this node
  used /clock it would be blind to exactly the failure it exists to catch --
  Gazebo's clock stopping.

  CLOCK_MONOTONIC via time.monotonic(), not the guest's wall clock. The guest
  runs UTC while the host runs KST, and the RAS event timestamps come from a
  third place; a monotonic per-observation delta avoids all of that. Correlate
  by duration, not by absolute time.

Usage (guest):
    python3 liveliness_watchdog.py --out ~/liveliness.csv
"""

import argparse
import csv
import math
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy,
                       QoSDurabilityPolicy, QoSHistoryPolicy)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

# topic -> (msg type, expected period in seconds, qos)
# Periods come from the SDF and from nav2_split.yaml, times a slack factor of 4
# because TCG jitter alone routinely doubles an interval.
# topic -> (type, nominal period in SIM seconds, qos, budget-gated?)
#
# The fourth field decides whether --budget-ms may fail the run on this topic, and
# only two qualify. Measured on the tb4 run of 2026-08-19 at RTF 0.5, wall ms:
#
#   /scan     p50  399.9   = 5 Hz of sim, exactly     GATED
#   /odom     p50  199.9   = 10 Hz of sim, exactly    GATED
#   /cmd_vel  p50  208.5   p99 1614   max 1993        reported, not gated
#   /map      p50 4978.8   p99 6395   max 7668        reported, not gated
#
# /scan and /odom are driven by the simulator at a fixed rate, so a long interval
# means the link or the consumer stalled -- which is what the TSP requirement is
# about, and what these two are gated on.
#
# /cmd_vel is published by controller_server only while a goal is active. The gaps
# between legs, and the whole of an aborted leg, are Nav2 deciding rather than
# anything stalling; gating them would fail a healthy run for finishing a waypoint.
# This is exactly why the correction request asks for /cmd_vel to be judged on
# non-degradation against a baseline rather than an absolute bound.
#
# /map is republished every map_update_interval, 5.0 s -- a 1 s budget on it is a
# statement about slam_toolbox's configuration, not about delivery.
#
# The periods are SIM seconds and the intervals measured here are WALL seconds, so
# every threshold below is divided by --rtf. Getting this wrong is not academic:
# /odom's period was 0.05 (20 Hz) while run_split.sh forces odom_publish_frequency
# to 10, i.e. 0.1 s of sim, which at RTF 0.5 is 199.9 ms of wall. With SLACK 4 the
# threshold was 0.05*4 = 200 ms against a 199.9 ms median, so about half of all
# HEALTHY /odom arrivals were logged as outages.
WATCH = {
    '/scan':     (LaserScan, 0.2,  'sensor',  True),
    # 0.1 s, not 0.05: run_split.sh defaults ODOM_HZ=10 and passes it to
    # make_world.py as odom_publish_frequency. If ODOM_HZ is ever changed, change
    # this with it -- there is no way for this node to read it.
    '/odom':     (Odometry,  0.1,  'sensor',  True),
    '/cmd_vel':  (Twist,     0.2,  'default', False),
    '/map':      (OccupancyGrid, 10.0, 'latched', False),
}
SLACK = 4.0

TRANSIENT_LOCAL = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Liveliness(Node):

    def __init__(self, out_path, budget_ms=None, rtf=1.0):
        # use_sim_time is left at its default false on purpose. See the module
        # docstring: this node must not follow Gazebo's clock.
        super().__init__('liveliness_watchdog')
        self.out_path = out_path
        self.budget_ms = budget_ms
        # Sim seconds per wall second. Thresholds are stated in sim time and
        # measured in wall time, so they scale by 1/rtf.
        self.rtf = rtf if rtf and rtf > 0 else 1.0
        self.last = {}
        self.gaps = []
        # Every inter-arrival interval, per topic, not just the over-budget ones.
        # One float per message: /scan at 5 Hz sim and /odom at 10 Hz sim over a
        # 10 minute run is a few thousand appends, which is nothing beside the
        # rclpy dispatch this callback already costs. Without the full
        # distribution the only reportable figure is "no gap exceeded the budget",
        # which cannot show a link that degraded without ever breaching it.
        self.intervals = {}
        # A SECOND arrival-time series, for the interval distribution only, and it
        # is never re-stamped. _check() deliberately re-stamps self.last during a
        # silence so one long outage is logged as consecutive chunks -- correct for
        # the gap log, and fatal for the distribution: it capped the largest
        # interval this node could ever report at about 2*period*SLACK + one 0.5 s
        # tick. For /odom that ceiling was ~0.9 s, BELOW the 1000 ms budget, so the
        # --budget-ms gate on /odom could not fire however badly the link stalled.
        self.arrival = {}
        self.started = time.monotonic()

        for topic, (msg_type, period, qos_kind, _gated) in WATCH.items():
            if qos_kind == 'sensor':
                qos = qos_profile_sensor_data
            elif qos_kind == 'latched':
                qos = TRANSIENT_LOCAL
            else:
                qos = 10
            # raw=True is not an optimisation here, it is a correctness fix.
            # This node reads no field of any message -- it records arrival times
            # and nothing else -- so deserialising is pure waste, and in the guest
            # that waste is enormous: /scan is 360 floats, /map is 11322 cells, and
            # rclpy turns each into Python objects on an emulated core. Measured
            # 2026-08-17 in the split run, this process averaged 93% of a core, the
            # largest single consumer in a guest already at loadavg 23 on 4 cores,
            # and Nav2's own nodes were starved of the CPU they needed to service
            # /clock. An instrument that changes what it measures is worse than no
            # instrument. raw=True hands the callback the bytes and skips all of it.
            self.create_subscription(
                msg_type, topic,
                lambda _m, t=topic: self._on_msg(t), qos, raw=True)
            self.last[topic] = None
            self.arrival[topic] = None
            self.intervals[topic] = []

        self.create_timer(0.5, self._check)
        self.get_logger().info(
            f'watching (rtf {self.rtf:g}, thresholds in wall s) ' + ', '.join(
                f'{t} (>{p * SLACK / self.rtf:.1f}s)'
                for t, (_, p, _q, _g) in WATCH.items()))

    def _on_msg(self, topic):
        now = time.monotonic()
        prev = self.last[topic]
        self.last[topic] = now
        if prev is None:
            self.get_logger().info(f'{topic}: first message at '
                                   f't+{now - self.started:.1f}s')
            return
        # From the un-re-stamped series, so a real 30 s outage is one 30 s interval.
        if self.arrival[topic] is not None:
            self.intervals[topic].append(now - self.arrival[topic])
        self.arrival[topic] = now
        budget = WATCH[topic][1] * SLACK / self.rtf
        gap = now - prev
        if gap > budget:
            self.gaps.append((topic, round(prev - self.started, 3),
                              round(now - self.started, 3), round(gap, 3)))
            self.get_logger().warn(f'{topic}: gap {gap:.2f}s '
                                   f'(budget {budget:.2f}s)')

    def _check(self):
        # Catch a topic that has gone silent and stays silent: _on_msg cannot
        # fire, so the gap would never be recorded without this.
        now = time.monotonic()
        for topic, (_t, period, _q, _g) in WATCH.items():
            prev = self.last[topic]
            if prev is None:
                continue
            budget = period * SLACK / self.rtf
            if now - prev > budget * 2:
                # Re-stamp so one long outage is logged as consecutive chunks
                # rather than suppressed entirely.
                self.gaps.append((topic, round(prev - self.started, 3),
                                  round(now - self.started, 3),
                                  round(now - prev, 3)))
                self.last[topic] = now
                self.get_logger().error(f'{topic}: SILENT for {now - prev:.1f}s')

    def dump(self):
        with open(self.out_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['topic', 't_start', 't_end', 'gap_s'])
            w.writerows(self.gaps)
        # The full distribution goes to its own file so the summary below stays
        # readable and the raw numbers stay auditable.
        iv_path = self.out_path.replace('.csv', '') + '-intervals.csv'
        with open(iv_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['topic', 'interval_ms'])
            for topic, vals in self.intervals.items():
                for v in vals:
                    w.writerow([topic, round(v * 1000.0, 3)])
        total = {}
        for topic, _s, _e, gap in self.gaps:
            total[topic] = total.get(topic, 0.0) + gap
        dur = time.monotonic() - self.started
        print(f'\nobserved {dur:.1f}s, {len(self.gaps)} gap(s)')
        for topic in WATCH:
            print(f'  {topic:10s} cumulative gap {total.get(topic, 0.0):7.2f}s')

        # Inter-arrival distribution, WALL milliseconds. The budget is the TSP's
        # topic figure read the only way this rig can answer it: as the longest
        # interruption a consumer saw. See the module docstring for why
        # publish-to-subscribe latency is not available here.
        print(f'\ninter-arrival at the consumer, wall ms'
              f'{"" if self.budget_ms is None else f" (budget {self.budget_ms:.0f} ms on max)"}')
        print(f'  {"topic":10s} {"n":>6} {"p50":>8} {"p99":>8} {"max":>8}')
        verdict = 0
        for topic in WATCH:
            vals = sorted(v * 1000.0 for v in self.intervals[topic])
            if not vals:
                # /cmd_vel legitimately has none if no goal ever ran; a gated
                # sensor topic with none is a dead link.
                print(f'  {topic:10s} {"0":>6}   NO MESSAGES'
                      f'{" -- gated topic, this is a failure" if WATCH[topic][3] else ""}')
                if WATCH[topic][3]:
                    verdict = 1
                continue

            def pct(q):
                # Nearest-rank: ceil, not round. With round() a p99 over 854 samples
                # returns rank 845 instead of 846 -- one sample optimistic, and on
                # a small n the error is a whole percentile band. math.ceil keeps
                # every printed number an interval that was actually observed.
                return vals[min(len(vals) - 1, max(0, math.ceil(q * len(vals)) - 1))]

            mx = vals[-1]
            gated = WATCH[topic][3]
            flag = ''
            if self.budget_ms is None:
                pass
            elif not gated:
                flag = '  not gated'
            else:
                over = mx > self.budget_ms
                flag = '  OVER' if over else '  ok'
                if over:
                    verdict = 1
            print(f'  {topic:10s} {len(vals):>6} {pct(0.50):>8.1f} '
                  f'{pct(0.99):>8.1f} {mx:>8.1f}{flag}')
        print(f'wrote {self.out_path} and {iv_path}')
        return verdict


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='liveliness.csv')
    # WALL milliseconds, applied to the MAXIMUM inter-arrival interval per topic.
    # The TSP's figures are 1,000 ms (3rd year) and 500 ms (4th). Off by default:
    # a budget that fires turns this node's exit status into a gate, and every
    # existing caller treats a nonzero exit as an instrument failure.
    ap.add_argument('--budget-ms', type=float, default=None)
    # Sim seconds per wall second, so the sim-time periods above can be compared
    # against wall-time arrivals. run_split.sh passes RTF_LIMIT.
    ap.add_argument('--rtf', type=float, default=1.0)
    a = ap.parse_args()

    rclpy.init()
    n = Liveliness(a.out, a.budget_ms, a.rtf)

    def _bye(_sig, _frm):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _bye)
    # dump() stays in a finally: SIGTERM at teardown is the normal way this node
    # ends, and the distribution it just spent the whole run collecting is the
    # deliverable. An unexpected exception must not take it with it.
    rc = 0
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy installs its own SIGINT handler and raises ExternalShutdownException
        # rather than KeyboardInterrupt. Catching only the latter let the exception
        # escape through the finally, so dump()'s verdict was discarded and the
        # process exited nonzero for a clean shutdown.
        pass
    finally:
        try:
            rc = n.dump()
        finally:
            n.destroy_node()
            # Already shut down if rclpy handled the signal itself; shutting down
            # twice raises RCLError and would lose the verdict computed above.
            if rclpy.ok():
                rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
