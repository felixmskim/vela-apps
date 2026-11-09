# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The topic detection bridge — the one piece that depends on rclpy.

Its role stays **thin**: discover topics, subscribe, flatten a message into a dict, and
hand it to :mod:`~.topiccheck`. The verdict logic lives over there so it can be tested on
the host without QEMU. If this node dies the daemon core keeps running, and the core's
`ros2app` observer records that death.

How the "which app will run is unknown" problem is handled
----------------------------------------------------------
* Topics are **discovered at runtime** via `get_topic_names_and_types()`. No list has to
  be known in advance.
* The type string is resolved to a class dynamically with
  `rosidl_runtime_py.utilities.get_message()`.
* A type whose load fails (an unbuilt custom `.msg`) is **not dropped**: it is subscribed
  with `raw=True` — period and size are observable from the CDR bytes alone (Tier 0).
* Field traversal goes through `get_fields_and_field_types()`, so no per-type code is
  needed.

Topics this node creates itself are not subscribed, to avoid self-observation through
overlapping namespaces.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from .topiccheck import TopicMonitor

#: topics that are not watched — its own and the ROS internals.
SKIP_PREFIXES = ("/rosout", "/parameter_events", "/fault_log", "/ras/")

#: flattening stops at this depth (keeps nested arrays like PointCloud2.fields from
#: exploding).
MAX_DEPTH = 3


def flatten(msg, prefix: str = "", depth: int = 0) -> dict:
    """A message -> `{"linear_acceleration.z": 9.8, ...}`.

    A variable-length array is **not stored in full** — the length (`_len`) plus a few
    elements are kept. Putting a PointCloud2 `data` into the dict verbatim would make the
    log megabytes long.
    """
    out: dict = {}
    getter = getattr(msg, "get_fields_and_field_types", None)
    if getter is None or depth > MAX_DEPTH:
        return out
    for name, type_str in getter().items():
        try:
            value = getattr(msg, name)
        except AttributeError:
            continue
        key = f"{prefix}{name}"
        if isinstance(value, (bytes, bytearray, memoryview)):
            out[f"{key}_len"] = len(value)
        elif isinstance(value, (list, tuple)) or type_str.startswith("sequence"):
            try:
                seq = list(value)
            except TypeError:
                continue
            out[f"{key}_len"] = len(seq)
            if seq and isinstance(seq[0], (int, float)):
                out[key] = seq          # numeric arrays are needed for the verdicts
        elif hasattr(value, "get_fields_and_field_types"):
            out.update(flatten(value, prefix=f"{key}.", depth=depth + 1))
        else:
            out[key] = value
    # the timestamp is used often, so it is also stored under a fixed name
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None and not prefix:
        out["header.stamp_sec"] = getattr(stamp, "sec", None)
        out["header.stamp_nanosec"] = getattr(stamp, "nanosec", None)
    return out


class TopicBridge:
    """Discovers topics, subscribes, and streams the verdicts to a sink."""

    def __init__(self, monitor: TopicMonitor, node, emit, raw_fallback: bool = True):
        self.monitor = monitor
        self.node = node
        self.emit = emit
        self.raw_fallback = raw_fallback
        self.subscribed: dict[str, str] = {}      # topic -> 'typed' | 'raw'
        self.unsupported: dict[str, str] = {}     # topic -> reason

    def discover(self) -> list[str]:
        """Subscribe to newly appeared topics, skipping those already subscribed."""
        added = []
        try:
            listing = self.node.get_topic_names_and_types()
        except Exception:
            return added
        for topic, types in listing:
            if topic in self.subscribed or topic in self.unsupported:
                continue
            if topic.startswith(SKIP_PREFIXES) or not types:
                continue
            if self._subscribe(topic, types[0]):
                added.append(topic)
        return added

    def _subscribe(self, topic: str, type_name: str) -> bool:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        # An observer does not impose its QoS on publishers — attaching as best-effort is
        # what lets it match sensors, which are usually best-effort.
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        try:
            from rosidl_runtime_py.utilities import get_message
            msg_cls = get_message(type_name)
        except Exception as exc:
            if not self.raw_fallback:
                self.unsupported[topic] = f"type load failed: {exc}"
                return False
            # period and size are observable even with an unknown type (Tier 0)
            try:
                from std_msgs.msg import ByteMultiArray
                self.node.create_subscription(
                    ByteMultiArray, topic,
                    lambda data, t=topic, n=type_name: self._on_raw(t, n, data),
                    qos, raw=True)
            except Exception as exc2:
                self.unsupported[topic] = f"raw subscription also failed: {exc2}"
                return False
            self.subscribed[topic] = "raw"
            return True

        try:
            self.node.create_subscription(
                msg_cls, topic,
                lambda msg, t=topic, n=type_name: self._on_typed(t, n, msg), qos)
        except Exception as exc:
            self.unsupported[topic] = f"subscription failed: {exc}"
            return False
        self.subscribed[topic] = "typed"
        return True

    # -- callbacks ----------------------------------------------------------
    def _on_typed(self, topic: str, type_name: str, msg) -> None:
        fields = flatten(msg)
        size = self._estimate_size(fields)
        for fault in self.monitor.on_message(topic, type_name, time.time(),
                                            size, fields):
            self.emit(fault)

    def _on_raw(self, topic: str, type_name: str, data) -> None:
        size = len(data) if isinstance(data, (bytes, bytearray)) else 0
        # fields=None skips the Tier 1 checks: no verdict on absent evidence.
        for fault in self.monitor.on_message(topic, type_name, time.time(),
                                             size, None):
            self.emit(fault)

    @staticmethod
    def _estimate_size(fields: dict) -> int:
        """Sum of variable-array lengths plus scalar count. Not exact bytes — it exists
        **to detect change**."""
        total = 0
        for key, value in fields.items():
            if key.endswith("_len") and isinstance(value, int):
                total += value
            elif isinstance(value, (int, float)):
                total += 8
        return total or len(fields)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="ras_fault_monitor.ros2topic_node",
        description="topic anomaly detection bridge (requires rclpy)")
    p.add_argument("--out", default="", help="JSONL log path")
    p.add_argument("--stdout", action="store_true")
    p.add_argument("--duration", type=float, default=0.0)
    p.add_argument("--warmup", type=int, default=20,
                   help="messages required before a baseline locks")
    p.add_argument("--deadline-factor", type=float, default=5.0,
                   help="how many times the baseline period to wait")
    p.add_argument("--discover-interval", type=float, default=5.0)
    a = p.parse_args(argv)

    import rclpy
    from rclpy.node import Node

    from .daemon import JsonlSink

    sink = JsonlSink(a.out, a.stdout)
    monitor = TopicMonitor(node_name="topic_bridge", warmup_n=a.warmup,
                           deadline_factor=a.deadline_factor)

    rclpy.init()
    node = Node("ras_topic_bridge")

    def emit(fault) -> None:
        sink.write(fault.to_dict(), fault.one_line())

    bridge = TopicBridge(monitor, node, emit)
    sink.write({"kind": "bridge_start", "timestamp": time.time()})

    started = time.time()
    next_discover = 0.0
    try:
        while rclpy.ok():
            now = time.time()
            if now >= next_discover:
                next_discover = now + a.discover_interval
                added = bridge.discover()
                if added:
                    sink.write({"kind": "topics_added", "timestamp": now,
                                "topics": added,
                                "mode": {t: bridge.subscribed[t] for t in added}})
                    if a.stdout:
                        print("  subscribed:", added, flush=True)
            rclpy.spin_once(node, timeout_sec=0.2)
            for fault in monitor.check_deadlines(time.time()):
                emit(fault)
            if a.duration and time.time() - started >= a.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        summary = {"kind": "bridge_stop", "timestamp": time.time(),
                   "topics": monitor.status(),
                   "unsupported": bridge.unsupported}
        sink.write(summary)
        if a.stdout:
            print("=== bridge stop ===",
                  json.dumps(summary, ensure_ascii=False), flush=True)
        # On SIGTERM/SIGINT rclpy's own handler has already shut the context down, and
        # calling shutdown again raises `rcl_shutdown already called` — the traceback
        # made every clean stop exit 1, so `systemctl restart` looked like a crash and
        # ate into StartLimitBurst.
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        sink.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
