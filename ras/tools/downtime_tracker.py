#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Record per-topic message gaps so downtime can be measured, not guessed.

The test procedure defines downtime as the time during which the ROS 2 workload is
not delivering: any interval in which a monitored topic went longer than
--threshold-ms without a message. This subscribes with `raw=True`, so it needs no
message class and works for any topic, including custom .msg types.

Each gap is written as one CSV row the moment it closes, so a kill -9 still leaves
every gap observed up to that point on disk.

    python3 downtime_tracker.py --topics /scan /odom --threshold-ms 1000 \
        --out gaps.csv --duration 3600

Run it as the same user as the publishers: cross-user DDS discovery succeeds while
data delivery silently does not.
"""
import argparse
import csv
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message


class Tracker(Node):
    def __init__(self, topics, threshold_s, writer, fp, grace_s=10.0):
        super().__init__("downtime_tracker")
        # A topic may carry its own threshold as "/name:ms" — an inference topic
        # that legitimately publishes every few seconds cannot share the 1 s
        # threshold of a laser scan without every normal interval counting as a gap.
        self.threshold = {}
        parsed = []
        for t in topics:
            if ":" in t and t.rsplit(":", 1)[1].isdigit():
                name, ms = t.rsplit(":", 1)
                self.threshold[name] = int(ms) / 1000.0
                parsed.append(name)
            else:
                self.threshold[t] = threshold_s
                parsed.append(t)
        topics = parsed
        self.grace = grace_s
        self.writer = writer
        self.fp = fp
        self.last = {}          # topic -> wall time of last message
        self.counts = {}        # topic -> messages seen
        self.gaps = {}          # topic -> [total_s, count]
        self.no_publisher = {}  # topic -> gaps that happened with no publisher left
        self.started = time.time()
        self.pending = list(topics)
        for t in topics:
            self.counts[t] = 0
            self.gaps[t] = [0.0, 0]
            self.no_publisher[t] = 0
            # a topic that never delivers is downtime from the start, not silence
            self.last[t] = self.started
        self._attach()
        # retry discovery, and close out gaps that are still open
        self.create_timer(2.0, self._attach)
        self.create_timer(1.0, self._sweep)

    def _attach(self):
        """Subscribe to whatever is discoverable now.

        raw=True still matches by type name, so passing a placeholder type means DDS
        never matches the publisher and the tracker sits at zero messages while
        reporting perfect availability. Resolve the real type from the graph.
        """
        if not self.pending:
            return
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        graph = dict(self.get_topic_names_and_types())
        for t in list(self.pending):
            types = graph.get(t)
            if not types:
                continue
            try:
                msg_type = get_message(types[0])
            except Exception as exc:
                self.get_logger().error(f"{t}: cannot resolve {types[0]}: {exc}")
                continue
            self.create_subscription(msg_type, t, self._make_cb(t), qos, raw=True)
            self.pending.remove(t)
            self.get_logger().info(f"watching {t} ({types[0]})")

    def _make_cb(self, topic):
        def cb(_msg):
            now = time.time()
            prev = self.last.get(topic)
            if prev is not None:
                gap = now - prev
                if self.counts[topic] == 0 and now - self.started < self.grace:
                    gap = 0.0          # 첫 메시지까지의 대기는 discovery 지연
                if gap > self.threshold[topic]:
                    self._record(topic, prev, now, gap, "gap")
            else:
                self.last[topic] = now
            self.last[topic] = now
            self.counts[topic] += 1
        return cb

    def _sweep(self):
        """Close out a gap that is still open, so a dead publisher is not invisible."""
        now = time.time()
        for topic, prev in list(self.last.items()):
            if now - prev > self.threshold[topic] * 3:
                # 시험 시작 직후 discovery 가 끝나기 전의 공백은 다운타임이 아니다.
                # 단 유예가 지나도 한 건도 못 받았으면 그때부터는 진짜 다운타임이다.
                if self.counts[topic] == 0 and now - self.started < self.grace:
                    self.last[topic] = now
                    continue
                # 퍼블리셔가 사라졌는지 먼저 본다. 워크로드가 스스로 끝난 것과
                # 고장으로 전달이 끊긴 것은 토픽 쪽에서 똑같이 보이는데, 둘을 같은
                # 다운타임으로 세면 하드웨어 고장이 없어도 가용성이 미달로 나온다.
                if self.count_publishers(topic) == 0:
                    kind = "no_publisher"
                    self.no_publisher[topic] += 1
                else:
                    kind = "never" if self.counts[topic] == 0 else "ongoing"
                self._record(topic, prev, now, now - prev, kind)
                self.last[topic] = now

    def _record(self, topic, start, end, gap, kind):
        # no_publisher 구간은 CSV 에 남기되 다운타임 합계에는 넣지 않는다.
        # 측정이 무효라는 사실이 가용성 수치로 둔갑하면 안 된다.
        if kind != "no_publisher":
            self.gaps[topic][0] += gap
        self.gaps[topic][1] += 1
        self.writer.writerow([f"{start:.3f}", f"{end:.3f}", topic,
                              f"{gap:.3f}", kind])
        self.fp.flush()
        self.get_logger().warn(f"{topic}: {gap:.3f}s gap ({kind})")

    def summary(self):
        total = sum(v[0] for v in self.gaps.values())
        silent = [t for t, n in self.counts.items() if n == 0]
        lost = [t for t, n in self.no_publisher.items() if n]
        return {
            "elapsed_s": time.time() - self.started,
            "total_downtime_s": total,
            "topics_never_seen": silent,
            # 퍼블리셔가 중간에 사라진 토픽. 비어 있지 않으면 이 창의 가용성 수치는
            # 고장을 잰 것이 아니므로 그대로 보고하면 안 된다.
            "topics_lost_publisher": lost,
            "measurement_valid": not lost,
            "subscribed": [t for t in self.counts if t not in self.pending],
            "per_topic": {t: {"messages": self.counts[t],
                              "gap_count": self.gaps[t][1],
                              "gap_total_s": round(self.gaps[t][0], 3),
                              "no_publisher_gaps": self.no_publisher[t]}
                          for t in self.counts},
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topics", nargs="+", required=True)
    ap.add_argument("--threshold-ms", type=float, default=1000.0)
    ap.add_argument("--duration", type=float, default=3600.0)
    ap.add_argument("--out", default="gaps.csv")
    ap.add_argument("--summary-out", default="")
    ap.add_argument("--grace-s", type=float, default=10.0,
                    help="시작 직후 이 시간 안의 첫 수신 지연은 다운타임으로 세지 않는다")
    args = ap.parse_args()

    fp = open(args.out, "w", newline="", buffering=1)
    writer = csv.writer(fp)
    writer.writerow(["gap_start", "gap_end", "topic", "gap_s", "kind"])

    rclpy.init()
    node = Tracker(args.topics, args.threshold_ms / 1000.0, writer, fp, args.grace_s)

    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("now", True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("now", True))

    deadline = time.time() + args.duration
    try:
        while rclpy.ok() and not stop["now"] and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        import json
        s = node.summary()
        text = json.dumps(s, indent=2)
        print(text)
        if args.summary_out:
            open(args.summary_out, "w").write(text + "\n")
        fp.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
