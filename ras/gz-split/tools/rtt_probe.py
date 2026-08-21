#!/usr/bin/env python3
"""Measure guest -> host -> guest round-trip time over the Fast DDS TCP link.

Gate G4. Runs in two halves:

    host container:  python3 rtt_probe.py --role echo
    guest:           python3 rtt_probe.py --role ping -n 200

Why round trip rather than one way: the guest runs UTC, the host runs KST, and
neither is NTP-synchronised to the other. A one-way figure would be an offset
measurement pretending to be a latency measurement. The round trip is stamped
and read by the same clock in the same process, so no alignment is needed.

Why time.monotonic() rather than the ROS clock: use_sim_time is true across this
stack, and sim time advances at the real-time factor. A latency budget is a
wall-clock budget.

The payload is a std_msgs/Header, whose stamp field is used as an opaque
correlation id (sec = sequence number), not as a timestamp. This deliberately
avoids depending on any clock the other side owns.
"""

import argparse
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Header

PING_TOPIC = 'rtt_ping'
PONG_TOPIC = 'rtt_pong'

# RELIABLE with a real depth: a dropped sample would be scored as a timeout and
# reported as latency, which would be wrong.
QOS = QoSProfile(depth=50,
                 history=QoSHistoryPolicy.KEEP_LAST,
                 reliability=QoSReliabilityPolicy.RELIABLE)


class Echo(Node):
    """Host side: reflect every ping straight back, unmodified."""

    def __init__(self):
        super().__init__('rtt_echo')
        self.pub = self.create_publisher(Header, PONG_TOPIC, QOS)
        self.create_subscription(Header, PING_TOPIC, self._on_ping, QOS)
        self.n = 0
        self.get_logger().info(f'echoing {PING_TOPIC} -> {PONG_TOPIC}')

    def _on_ping(self, msg):
        self.pub.publish(msg)
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(f'echoed {self.n}')


class Ping(Node):
    """Guest side: stamp, send, wait for the matching id, record the delta."""

    def __init__(self, count, interval, timeout):
        super().__init__('rtt_ping')
        self.pub = self.create_publisher(Header, PING_TOPIC, QOS)
        self.create_subscription(Header, PONG_TOPIC, self._on_pong, QOS)
        self.count = count
        self.interval = interval
        self.timeout = timeout
        self.sent = {}
        self.rtts = []
        self.lost = 0

    def _on_pong(self, msg):
        seq = msg.stamp.sec
        t0 = self.sent.pop(seq, None)
        if t0 is not None:
            self.rtts.append((time.monotonic() - t0) * 1000.0)

    def wait_for_echo(self, deadline):
        self.get_logger().info(f'waiting for the echo node (up to {deadline:.0f}s)')
        t0 = time.monotonic()
        while time.monotonic() - t0 < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.pub.get_subscription_count() > 0:
                self.get_logger().info(
                    f'  echo node visible after {time.monotonic() - t0:.0f}s')
                return True
        return False

    def run(self):
        for seq in range(1, self.count + 1):
            h = Header()
            h.frame_id = 'rtt'
            h.stamp.sec = seq
            h.stamp.nanosec = 0
            self.sent[seq] = time.monotonic()
            self.pub.publish(h)

            # Drain replies while pacing the next send.
            t_end = time.monotonic() + self.interval
            while time.monotonic() < t_end:
                rclpy.spin_once(self, timeout_sec=0.01)

            if seq % 50 == 0:
                self.get_logger().info(f'sent {seq}/{self.count}, '
                                       f'{len(self.rtts)} replies')

        # Give late replies a chance before declaring loss.
        t_end = time.monotonic() + self.timeout
        while time.monotonic() < t_end and self.sent:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.lost = len(self.sent)


def pct(values, p):
    if not values:
        return float('nan')
    s = sorted(values)
    k = min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1))))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--role', choices=['ping', 'echo'], required=True)
    ap.add_argument('-n', '--count', type=int, default=200)
    ap.add_argument('--interval', type=float, default=0.1)
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--budget-ms', type=float, default=500.0,
                    help='G4 pass threshold on p95')
    a = ap.parse_args()

    rclpy.init()
    rc = 0
    if a.role == 'echo':
        n = Echo()
        try:
            rclpy.spin(n)
        except KeyboardInterrupt:
            pass
        finally:
            n.destroy_node()
            rclpy.shutdown()
        return 0

    n = Ping(a.count, a.interval, a.timeout)
    try:
        if not n.wait_for_echo(120.0):
            n.get_logger().error('FAIL: echo node never discovered '
                                 '(G3 has to pass before G4)')
            return 1
        n.run()
        r = n.rtts
        if not r:
            print('RESULT: FAIL (no replies at all)')
            return 1
        p50, p95, p99 = pct(r, 50), pct(r, 95), pct(r, 99)
        print(f'\nsamples {len(r)}/{a.count}, lost {n.lost}')
        print(f'  min  {min(r):8.1f} ms')
        print(f'  p50  {p50:8.1f} ms')
        print(f'  p95  {p95:8.1f} ms   <- G4')
        print(f'  p99  {p99:8.1f} ms')
        print(f'  max  {max(r):8.1f} ms')
        print(f'  mean {statistics.fmean(r):8.1f} ms')
        ok = p95 < a.budget_ms and n.lost == 0
        rc = 0 if ok else 1
        print(f'RESULT: {"PASS" if ok else "FAIL"} '
              f'(p95 {p95:.1f} ms vs budget {a.budget_ms:.0f} ms, lost {n.lost})')
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
