#!/usr/bin/env python3
"""Measure publish-to-subscribe latency for a message that has no Header.

This exists to settle a specific claim. Defect D-15 as filed against the TSP
states that /cmd_vel latency is "unmeasurable in principle" because
geometry_msgs/Twist carries no Header and therefore no timestamp. That is wrong,
and the plan depends on knowing it is wrong.

The DDS middleware stamps every sample independently of the message body. rclpy
exposes those stamps: a subscription callback taking two parameters is
auto-detected as CallbackType.WithMessageInfo (rclpy/subscription.py:105-120,
dispatched at rclpy/executors.py:474), and the second argument is a dict with

    source_timestamp        ns, written by the publisher's middleware
    received_timestamp      ns, written by the subscriber's middleware
    publication_sequence_number
    reception_sequence_number

Verified on this guest, 2026-08-13, with an unmodified geometry_msgs/Twist:
30/30 samples carried a nonzero source_timestamp, and
received_timestamp - source_timestamp gave 0.227 / 0.228 / 17.605 ms
for min / median / max.

Scope, stated because it decides which tool to use:

  * Both endpoints on the SAME machine -- the figure is exact. Use this.
  * Endpoints on DIFFERENT machines (host container <-> guest) -- the difference
    also contains the clock offset between the two systems, which is not
    synchronised here. Use rtt_probe.py for that, which needs one clock only.

So D-15 should be rewritten as "the test procedure omits wall-clock
instrumentation", not "measurement is impossible".

Usage (guest, while the split stack is running):
    python3 latency_probe.py --topic /cmd_vel --type geometry_msgs/msg/Twist -n 300
    python3 latency_probe.py --topic /scan --type sensor_msgs/msg/LaserScan -n 100
"""

import argparse
import csv
import importlib
import signal
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def import_msg(spec):
    """'geometry_msgs/msg/Twist' -> the Twist class."""
    pkg, kind, name = spec.split('/')
    mod = importlib.import_module(f'{pkg}.{kind}')
    return getattr(mod, name)


def pct(values, p):
    s = sorted(values)
    k = min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1))))
    return s[k]


class LatencyProbe(Node):

    def __init__(self, topic, msg_type, count, sensor_qos):
        # Wall clock deliberately: a latency budget is a wall-clock budget, and
        # the middleware stamps are wall-clock nanoseconds regardless of what
        # use_sim_time is set to elsewhere in the stack.
        super().__init__('latency_probe')
        self.count = count
        self.samples = []
        self.zero_stamps = 0

        qos = qos_profile_sensor_data if sensor_qos else 10
        # Two parameters: this is what selects WithMessageInfo. Do not "tidy"
        # it to one parameter -- the info dict is the entire point.
        self.create_subscription(msg_type, topic, self._on_msg, qos)
        self.get_logger().info(f'listening on {topic} for {count} samples '
                               f'(qos={"sensor_data" if sensor_qos else "depth 10"})')

    def _on_msg(self, _msg, info):
        src = info.get('source_timestamp', 0)
        recv = info.get('received_timestamp', 0)
        if not src or not recv:
            self.zero_stamps += 1
            return
        self.samples.append({
            'pub_seq': info.get('publication_sequence_number', 0),
            'recv_seq': info.get('reception_sequence_number', 0),
            'source_ns': src,
            'received_ns': recv,
            'latency_ms': (recv - src) / 1e6,
        })
        n = len(self.samples)
        if n % 50 == 0:
            self.get_logger().info(f'  {n}/{self.count}')

    def done(self):
        return len(self.samples) >= self.count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', required=True)
    ap.add_argument('--type', required=True,
                    help="e.g. geometry_msgs/msg/Twist")
    ap.add_argument('-n', '--count', type=int, default=300)
    ap.add_argument('--sensor-qos', action='store_true',
                    help='use qos_profile_sensor_data (needed for /scan, /odom)')
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--csv', default=None)
    ap.add_argument('--budget-ms', type=float, default=None,
                    help='if given, exit nonzero when p95 exceeds it')
    a = ap.parse_args()

    msg_type = import_msg(a.type)

    rclpy.init()
    n = LatencyProbe(a.topic, msg_type, a.count, a.sensor_qos)

    def _bye(_s, _f):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _bye)

    rc = 0
    try:
        t0 = time.monotonic()
        while not n.done() and time.monotonic() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.2)

        lat = [s['latency_ms'] for s in n.samples]
        if not lat:
            print(f'RESULT: FAIL (no samples with usable stamps on {a.topic}; '
                  f'{n.zero_stamps} had zero stamps)')
            return 1

        # Sequence gaps mean the middleware dropped samples, which would bias
        # every percentile downward if not reported.
        seqs = [s['pub_seq'] for s in n.samples]
        gaps = sum(1 for x, y in zip(seqs, seqs[1:]) if y - x > 1)

        print(f'\n{a.topic}  ({a.type})')
        print(f'  samples      {len(lat)}')
        print(f'  seq gaps     {gaps}')
        print(f'  zero stamps  {n.zero_stamps}')
        print(f'  min   {min(lat):9.3f} ms')
        print(f'  p50   {pct(lat, 50):9.3f} ms')
        print(f'  p95   {pct(lat, 95):9.3f} ms')
        print(f'  p99   {pct(lat, 99):9.3f} ms')
        print(f'  max   {max(lat):9.3f} ms')
        print(f'  mean  {statistics.fmean(lat):9.3f} ms')
        print('  NOTE: valid as an absolute latency only when publisher and '
              'subscriber share a clock. Across host/guest use rtt_probe.py.')

        if a.csv:
            with open(a.csv, 'w', newline='') as fh:
                w = csv.DictWriter(fh, fieldnames=list(n.samples[0].keys()))
                w.writeheader()
                w.writerows(n.samples)
            print(f'  wrote {a.csv}')

        if a.budget_ms is not None:
            ok = pct(lat, 95) < a.budget_ms
            rc = 0 if ok else 1
            print(f'RESULT: {"PASS" if ok else "FAIL"} '
                  f'(p95 {pct(lat, 95):.3f} ms vs budget {a.budget_ms:.0f} ms)')
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
