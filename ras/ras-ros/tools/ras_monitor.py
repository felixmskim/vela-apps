#!/usr/bin/env python3
"""Publish kernel RAS events onto ROS 2, and write them where the TSP says to.

    kernel                    ras_monitor                 consumers
  ras:non_standard_event ──┐
  /dev/kmsg              ──┼──►  decode, stamp, count ──► /ras/events
  rasdaemon (sqlite)     ──┘                          ──► /diagnostics
                                                      ──► /var/log/ras/events.log

Three design points that are not obvious, each with a reason:

**Its own tracefs instance.** Reading `trace_pipe` CONSUMES events. rasdaemon is
already running on this guest and already has its own instance
(`/sys/kernel/tracing/instances/rasdaemon`), so a monitor reading the top-level
pipe would be fine today -- and would silently start stealing rasdaemon's events
the day someone reconfigures it. Instances have independent ring buffers, so this
takes its own and neither side can starve the other. `scripts/install_guest.sh`
creates it.

**No custom message type.** `diagnostic_msgs/DiagnosticArray` is already on the
guest, is what `rqt_robot_monitor` and `diagnostic_aggregator` consume, and needs
no `rosidl` build -- which on an emulated riscv64 guest is hours, and which would
also introduce a type hash the x86 host container does not have. The RAS-specific
fields ride in `KeyValue` pairs. A dedicated `vela_ras_msgs/RasEvent` is a later
upgrade, not a prerequisite.

**Corrected errors are WARN, not ERROR.** TC.10.007 injects a *corrected* CPU
error. By definition the workload does not stop, so an alarm level that says
"failure" would be wrong, and a downtime measured from it would be fiction.
`ras_decode.GHES_SEV_TO_DIAG` is the mapping and README.md argues it.

Runs as an ordinary user once `install_guest.sh` has put the tracefs group in
place; it does not need root, and deliberately does not want it -- a root-owned
Fast DDS participant does not share `/dev/shm` cleanly with the Nav2 nodes.

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import argparse
import json
import os
import pathlib
import socket
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import ras_decode  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402

DEFAULT_INSTANCE = "/sys/kernel/tracing/instances/ras_ros"
DEFAULT_DB = "/var/lib/rasdaemon/ras-mc_event.db"

# rasdaemon writes non-standard records here. Confirmed against the schema
# rasdaemon 0.8.5 creates with --enable-non-standard; the query degrades to a
# disabled source rather than an exception if the table is not there.
RASDAEMON_TABLES = ("non_standard_event", "non_standard_events")


def mono():
    return time.clock_gettime(time.CLOCK_MONOTONIC)


class Source(threading.Thread):
    """A reader thread that never lets its own failure take the node down.

    A RAS monitor that dies quietly is worse than no RAS monitor: the test it
    serves would report "no errors detected" and pass. So every source records
    its own health, the node publishes that health on the heartbeat, and one
    dead source does not stop the others.
    """

    def __init__(self, name, node):
        super().__init__(name=name, daemon=True)
        self.node = node
        self.count = 0
        self.ok = False
        self.error = "not started"
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()


class TraceSource(Source):
    """Blocking reads on a tracefs trace_pipe. Costs nothing while idle."""

    def __init__(self, node, path):
        super().__init__("trace", node)
        self.path = path

    def run(self):
        try:
            fd = open(self.path, "rb", buffering=0)
        except OSError as exc:
            self.error = f"cannot open {self.path}: {exc}"
            self.node.get_logger().error(
                f"[trace] {self.error} -- run scripts/install_guest.sh")
            return
        self.ok, self.error = True, ""
        self.node.get_logger().info(f"[trace] reading {self.path}")
        buf = b""
        with fd:
            while not self._stop.is_set():
                try:
                    chunk = fd.read(4096)
                except OSError as exc:
                    self.error = f"read failed: {exc}"
                    self.ok = False
                    return
                if not chunk:
                    # A real trace_pipe blocks and never returns empty. A plain
                    # file does, and that is the replay path the acceptance test
                    # uses -- so this is EOF on a fixture, not an error.
                    time.sleep(0.2)
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._line(line.decode("utf-8", "replace"))

    def _line(self, line):
        seen = mono()
        ev = ras_decode.parse_trace_line(line)
        if ev is None:
            return
        self.count += 1
        self.node.emit(ev, seen)


class KmsgSource(Source):
    """/dev/kmsg, seeked to the end so only new records arrive.

    One hardware error is a dozen printk lines sharing a {N} id, so they are
    reassembled before publishing. Without the timed flush the last error of a
    run would be held forever -- nothing follows it on a healthy system.
    """

    def __init__(self, node, path, quiet_s):
        super().__init__("kmsg", node)
        self.path = path
        self.asm = ras_decode.KmsgAssembler(quiet_s=quiet_s)

    def run(self):
        try:
            fd = os.open(self.path, os.O_RDONLY)
            os.lseek(fd, 0, os.SEEK_END)
        except OSError as exc:
            self.error = f"cannot open {self.path}: {exc}"
            self.node.get_logger().warning(f"[kmsg] {self.error}")
            return
        self.ok, self.error = True, ""
        self.node.get_logger().info(f"[kmsg] reading {self.path}")
        try:
            while not self._stop.is_set():
                try:
                    rec = os.read(fd, 8192)
                except OSError as exc:
                    # EPIPE means the ring wrapped past our position while we
                    # were busy. The record is gone; the reader is still valid.
                    if getattr(exc, "errno", None) == 32:
                        continue
                    self.error = f"read failed: {exc}"
                    self.ok = False
                    return
                if not rec:
                    time.sleep(0.2)
                    continue
                seen = mono()
                for line in rec.decode("utf-8", "replace").splitlines():
                    parsed = ras_decode.parse_kmsg_line(line)
                    if parsed:
                        self.asm.add(parsed[0], parsed[1], seen)
                self._flush(seen)
        finally:
            os.close(fd)

    def _flush(self, now):
        for ev in self.asm.due(now):
            self.count += 1
            self.node.emit(ev, now)

    def tick(self):
        """Called from the heartbeat so a quiet group still gets flushed."""
        if self.ok:
            self._flush(mono())


class RasdaemonSource(Source):
    """Count rows in rasdaemon's database. A cross-check, not an event stream.

    This exists for CR-4: the correction request asks for the `EDAC ce_count`
    criterion -- which cannot work on this kernel, `CONFIG_EDAC` is unset -- to be
    replaced by agreement between the tracepoint, rasdaemon and dmesg. Counting
    here is what makes that check something a script can assert on.
    """

    def __init__(self, node, db, period):
        super().__init__("rasdaemon", node)
        self.db = db
        self.period = period
        self.baseline = None

    def _count(self):
        import sqlite3
        # Read-only URI: never let a monitor write to the daemon's database.
        con = sqlite3.connect(f"file:{self.db}?mode=ro", uri=True, timeout=5.0)
        try:
            names = {r[0] for r in con.execute(
                "SELECT name FROM sqlite_master WHERE type='table'")}
            table = next((t for t in RASDAEMON_TABLES if t in names), None)
            if table is None:
                raise RuntimeError(
                    f"no non-standard table in {self.db}; rasdaemon was built "
                    "without --enable-non-standard (see ../README.md)")
            return con.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]
        finally:
            con.close()

    def run(self):
        while not self._stop.is_set():
            try:
                total = self._count()
                if self.baseline is None:
                    self.baseline = total
                    self.node.get_logger().info(
                        f"[rasdaemon] {self.db}: {total} rows at start")
                self.count = total - self.baseline
                self.ok, self.error = True, ""
            except Exception as exc:            # sqlite, permissions, schema
                if self.ok or self.error == "not started":
                    self.node.get_logger().warning(f"[rasdaemon] {exc}")
                self.ok, self.error = False, str(exc)
            self._stop.wait(self.period)


class RasMonitor(Node):

    def __init__(self, args):
        super().__init__("ras_monitor")
        self.args = args
        self.hardware_id = args.hardware_id or socket.gethostname()
        self.seq = 0
        self.started = mono()
        self._pub_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=args.depth,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_events = self.create_publisher(DiagnosticArray, args.events_topic, qos)
        self.pub_diag = self.create_publisher(DiagnosticArray, args.diagnostics_topic, qos)

        self.log_fh = None
        if args.log_file:
            self._open_log(args.log_file)

        self.clock_comparable, self.trace_clock = self._check_trace_clock()

        self.sources = []
        if not args.no_trace:
            self.sources.append(TraceSource(self, args.trace_path))
        if not args.no_kmsg:
            self.sources.append(KmsgSource(self, args.kmsg_path, args.kmsg_quiet))
        if not args.no_rasdaemon:
            self.sources.append(RasdaemonSource(self, args.rasdaemon_db, args.rasdaemon_period))
        for s in self.sources:
            s.start()

        self.create_timer(args.heartbeat, self._heartbeat)
        self.get_logger().info(
            f"publishing {args.events_topic} and {args.diagnostics_topic}; "
            f"hardware_id={self.hardware_id}; trace_clock={self.trace_clock}")

    # -- plumbing ------------------------------------------------------------

    def _open_log(self, path):
        """Open the JSONL sink, creating the directory the TSP names.

        TSP §4.3.1 lists `truncate -s0 /var/log/ras/events.log` as a
        precondition, and nothing on the system has ever written that file --
        filed as D-10, whose sharpest point is that the precondition CREATES it,
        so the run looks prepared while the evidence file stays empty. This is
        the producer that defect asked for.
        """
        p = pathlib.Path(path)
        try:
            p.parent.mkdir(parents=True, exist_ok=True)
            self.log_fh = open(p, "a", encoding="utf-8")
            self.get_logger().info(f"appending events to {p}")
        except OSError as exc:
            self.get_logger().error(f"cannot open {p}: {exc} -- continuing without a log file")

    def _check_trace_clock(self):
        """Is the trace timestamp comparable with CLOCK_MONOTONIC?

        Detection latency is `read time - event time`, and subtracting two
        different clock domains produces a number that looks fine and means
        nothing. tracefs defaults to `local`, which is per-CPU and not
        CLOCK_MONOTONIC, so this is checked rather than assumed -- CR-9.
        """
        path = pathlib.Path(self.args.trace_path).parent / "trace_clock"
        try:
            active = ""
            for token in path.read_text().split():
                if token.startswith("[") and token.endswith("]"):
                    active = token.strip("[]")
            return active == "mono", active or "unreadable"
        except OSError:
            return False, "unreadable"

    # -- publishing ----------------------------------------------------------

    def emit(self, event, seen_mono):
        """Publish one RAS event. Called from reader threads."""
        self.seq += 1
        event = dict(event)
        event["seq"] = self.seq
        event["detect_mono"] = round(seen_mono, 6)
        event["hardware_id"] = self.hardware_id
        if self.clock_comparable and event.get("trace_ts") is not None:
            event["detect_latency_ms"] = round((seen_mono - event["trace_ts"]) * 1000.0, 3)
        else:
            event["detect_latency_note"] = (
                f"trace_clock is {self.trace_clock}, not mono -- not comparable "
                "with CLOCK_MONOTONIC")

        status = DiagnosticStatus()
        status.level = bytes([ras_decode.diag_level(event)])
        status.name = f"ras/{event.get('source', 'unknown')}"
        status.message = ras_decode.summary(event)
        status.hardware_id = self.hardware_id
        status.values = [KeyValue(key=str(k), value=str(v))
                         for k, v in sorted(event.items()) if v is not None]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        with self._pub_lock:
            self.pub_events.publish(msg)
            self.pub_diag.publish(msg)
            if self.log_fh:
                self.log_fh.write(json.dumps(event, sort_keys=True, default=str) + "\n")
                self.log_fh.flush()

        self.get_logger().warning(f"RAS event #{self.seq} [{status.name}] {status.message}")
        if self.args.exit_after and self.seq >= self.args.exit_after:
            self.get_logger().info(f"--exit-after {self.args.exit_after} reached")
            # One last heartbeat before going. The final counts are the whole
            # point of the cross-check, and a monitor that exits between beats
            # leaves its last events uncounted -- which is how the acceptance
            # test caught this: 2 events delivered, count_trace still 0.
            self._heartbeat()
            raise SystemExit(0)

    def _heartbeat(self):
        """Liveness plus the three-way count comparison.

        Publishing nothing while healthy would make a dead monitor and a quiet
        machine look identical, and "탐지 누락률 = 0%" cannot be asserted from an
        absence. The heartbeat is what makes silence meaningful.
        """
        for s in self.sources:
            if isinstance(s, KmsgSource):
                s.tick()

        counts = {s.name: s.count for s in self.sources}
        dead = [s.name for s in self.sources if not s.ok]
        live = {s.name: s.count for s in self.sources if s.ok}
        agree = len(set(live.values())) <= 1

        status = DiagnosticStatus()
        status.name = "ras/monitor"
        status.hardware_id = self.hardware_id
        if dead:
            status.level = bytes([1])
            status.message = f"{len(dead)} source(s) unavailable: {', '.join(dead)}"
        elif not agree:
            status.level = bytes([1])
            status.message = "sources disagree on the event count: " + \
                ", ".join(f"{k}={v}" for k, v in sorted(live.items()))
        else:
            status.level = bytes([0])
            status.message = f"alive, {self.seq} event(s)"

        values = {
            "uptime_s": round(mono() - self.started, 1),
            "events_published": self.seq,
            "sources_ok": ",".join(sorted(s.name for s in self.sources if s.ok)) or "none",
            "sources_failed": ",".join(sorted(dead)) or "none",
            # CR-4: the criterion that replaces `EDAC ce_count = 4`.
            "count_agreement": str(agree).lower(),
            "clock_comparable": str(self.clock_comparable).lower(),
            "trace_clock": self.trace_clock,
            "log_file": self.args.log_file or "none",
        }
        values.update({f"count_{k}": v for k, v in counts.items()})
        for s in self.sources:
            if s.error:
                values[f"error_{s.name}"] = s.error
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in sorted(values.items())]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        with self._pub_lock:
            self.pub_diag.publish(msg)

    def shutdown(self):
        for s in self.sources:
            s.stop()
        if self.log_fh:
            self.log_fh.close()


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--trace-path", default=f"{DEFAULT_INSTANCE}/trace_pipe",
                    help="tracefs trace_pipe to read. Point it at a FIFO or a "
                         "recorded file to exercise the same path without RERI")
    ap.add_argument("--kmsg-path", default="/dev/kmsg")
    ap.add_argument("--rasdaemon-db", default=DEFAULT_DB)
    ap.add_argument("--no-trace", action="store_true")
    ap.add_argument("--no-kmsg", action="store_true")
    ap.add_argument("--no-rasdaemon", action="store_true")
    ap.add_argument("--log-file", default="",
                    help="append events as JSON lines, e.g. /var/log/ras/events.log")
    ap.add_argument("--events-topic", default="/ras/events")
    ap.add_argument("--diagnostics-topic", default="/diagnostics")
    ap.add_argument("--heartbeat", type=float, default=10.0, metavar="S")
    ap.add_argument("--kmsg-quiet", type=float, default=2.0, metavar="S",
                    help="how long a kmsg group must be idle before it is published")
    ap.add_argument("--rasdaemon-period", type=float, default=30.0, metavar="S")
    ap.add_argument("--depth", type=int, default=100)
    ap.add_argument("--hardware-id", default="")
    ap.add_argument("--exit-after", type=int, default=0, metavar="N",
                    help="exit once N events have been published (acceptance tests)")
    return ap


def main(argv=None):
    argv = [a for a in (argv if argv is not None else sys.argv[1:]) if a != "--ros-args"]
    args, unknown = build_parser().parse_known_args(argv)
    rclpy.init(args=None)
    node = RasMonitor(args)
    rc = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        rc = getattr(exc, "code", 0) or 0
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
