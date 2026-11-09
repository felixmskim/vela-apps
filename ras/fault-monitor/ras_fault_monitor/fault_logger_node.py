# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The `fault_logger` node — subscribes to `/fault_log`, **writes it to a file**, and
computes statistics.

It corresponds to `fault_logger` (JSON log storage / statistics) on p.13 of the ETRI
deck.

The file format is **JSONL** (one line = one JSON record). Writing a single JSON array
means that an abnormal process exit leaves no closing `]` and **the whole file becomes
unparseable**. Fault injection deliberately kills the system, so the format has to stay
readable up to the last instant.
"""

from __future__ import annotations

import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .fault import Fault
from .kernel_monitor_node import KernelMonitor, fault_qos

#: cap on stored dedup keys (keeps memory from growing without bound)
_SEEN_CAP = 20000


class FaultLogger(Node):

    def __init__(self):
        super().__init__("fault_logger")

        self.declare_parameter("fault_topic", "/fault_log")
        self.declare_parameter("log_dir", os.path.expanduser("~/fault_logs"))
        self.declare_parameter("file_prefix", "fault")
        self.declare_parameter("stats_period", 10.0)
        self.declare_parameter("echo_console", True)
        self.declare_parameter("durability", "transient_local")
        self.declare_parameter("dedupe", True)

        topic = self.get_parameter("fault_topic").value
        log_dir = os.path.expanduser(str(self.get_parameter("log_dir").value))
        prefix = str(self.get_parameter("file_prefix").value)
        self.echo_console = bool(self.get_parameter("echo_console").value)

        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"{prefix}_{stamp}.jsonl")
        self.summary_path = os.path.join(log_dir, f"{prefix}_{stamp}.summary.json")
        # line buffering (=1): each record reaches the file as it is written, so a sudden
        # death still leaves everything up to that point
        self._fp = open(self.log_path, "a", encoding="utf-8", buffering=1)

        self.total = 0
        self.by_type: dict[str, int] = {}
        self.by_severity: dict[str, int] = {}
        self.first_ts: float | None = None
        self.last_ts: float | None = None
        self.duplicates = 0
        self.dedupe = bool(self.get_parameter("dedupe").value)
        self._seen: set = set()

        durability = str(self.get_parameter("durability").value)
        self.subscription = self.create_subscription(
            String, topic, self._on_fault, fault_qos(durability)
        )
        period = float(self.get_parameter("stats_period").value)
        self.stats_timer = self.create_timer(period, self._print_stats) if period > 0 else None

        self.get_logger().info(f"Fault Logger started, logging to {self.log_path}")

    # -- callbacks ----------------------------------------------------------
    def _on_fault(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"JSON parse failed (ignored): {exc}")
            return

        fault = Fault.from_dict(data)

        # Being TRANSIENT_LOCAL, a logger restart replays past events. The kernel record
        # sequence (kmsg.seq) is unique per event, so it serves as the dedup key.
        key = (fault.fault_type, fault.kmsg.get("seq", -1), fault.timestamp)
        if self.dedupe and key in self._seen:
            self.duplicates += 1
            return
        if self.dedupe:
            if len(self._seen) >= _SEEN_CAP:
                self._seen.clear()
            self._seen.add(key)

        self._fp.write(msg.data + "\n")
        self.total += 1
        self.by_type[fault.fault_type] = self.by_type.get(fault.fault_type, 0) + 1
        self.by_severity[fault.severity] = self.by_severity.get(fault.severity, 0) + 1
        if self.first_ts is None:
            self.first_ts = fault.timestamp
        self.last_ts = fault.timestamp

        if self.echo_console:
            # the call site must differ per severity — see the KernelMonitor._echo_impl note
            KernelMonitor._echo_impl(
                self.get_logger(),
                fault.severity,
                f"[{fault.severity}] {fault.fault_type} | {fault.node_name} | {fault.description}",
            )

    def _print_stats(self) -> None:
        dup = f" (dup {self.duplicates})" if self.duplicates else ""
        self.get_logger().info(f"=== Fault Stats (total: {self.total}){dup} ===")
        for fault_type, count in sorted(self.by_type.items()):
            self.get_logger().info(f"   {fault_type}: {count}")

    # -- shutdown -----------------------------------------------------------
    def shutdown(self) -> None:
        summary = {
            "log_file": self.log_path,
            "total": self.total,
            "duplicates_dropped": self.duplicates,
            "by_type": self.by_type,
            "by_severity": self.by_severity,
            "first_event_time": self.first_ts,
            "last_event_time": self.last_ts,
            "closed_at": datetime.now().isoformat(timespec="seconds"),
        }
        with open(self.summary_path, "w", encoding="utf-8") as fp:
            json.dump(summary, fp, ensure_ascii=False, indent=2)
        self._fp.close()
        text = (
            f"Fault Logger stopped — total={self.total} duplicates={self.duplicates} "
            f"by_type={self.by_type}\n"
            f"  log:     {self.log_path}\n"
            f"  summary: {self.summary_path}"
        )
        # after SIGINT, publishing to rosout fails (see the KernelMonitor._final note)
        if rclpy.ok():
            self.get_logger().info(text)
        else:
            print(text, flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
