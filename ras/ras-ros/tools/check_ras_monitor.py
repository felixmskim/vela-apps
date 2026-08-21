#!/usr/bin/env python3
"""Acceptance test for ras_monitor: does a kernel RAS record reach a ROS 2 topic?

The guest this runs on is Config A -- no `reri=true`, no pflash pair -- so no
hardware error can be injected and none ever will be until Config C exists. The
test therefore feeds the monitor recorded records through a FIFO placed where
`trace_pipe` would be, next to a `trace_clock` file that says what tracefs would
say. Everything downstream of the read is the production path: the same parser,
the same decode, the same publisher, the same log writer.

What that does and does not prove:

  proven here    parsing, decoding, severity mapping, topic delivery, QoS,
                 the JSON log, the heartbeat, and the monitor's own detection
                 latency in milliseconds
  NOT proven     that the kernel emits the event -- that needs Config C, and
                 the only change is dropping --trace-path

Timestamps in the fixtures are rewritten to the current CLOCK_MONOTONIC before
they are fed in. Without that the recorded 414.59 s would be compared against a
guest that has been up for hours and the latency figure would be nonsense.

    python3 check_ras_monitor.py            # -> RESULT: PASS, exit 0

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import argparse
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray

HERE = pathlib.Path(__file__).resolve().parent


def find_samples():
    """samples/ sits beside this file in the guest and one level up in the repo.

    install_guest.sh flattens tools/ into ~/ras_ros/ and puts samples/ under it,
    so `HERE.parent/samples` resolves to /home/vela/samples and does not exist.
    Both layouts are real; neither is the odd one out.
    """
    for cand in (HERE / "samples", HERE.parent / "samples"):
        if (cand / "non_standard_event.trace").is_file():
            return cand
    return HERE / "samples"


SAMPLES = find_samples()
TS_RE = re.compile(r"^(\s*\S.*?-\d+\s+\[\d+\]\s+\S+\s+)(\d+\.\d+)(:)")

# What tracefs prints when the mono clock is selected. install_guest.sh makes the
# real instance say this; the test writes it so _check_trace_clock() takes the
# same branch it will take in production.
TRACE_CLOCK_TEXT = "local global counter uptime perf [mono] mono_raw boot tai\n"


def restamp(line, now):
    m = TS_RE.match(line)
    return TS_RE.sub(lambda _: f"{m.group(1)}{now:.6f}{m.group(3)}", line, count=1) if m else line


class Collector(Node):

    def __init__(self):
        super().__init__("check_ras_monitor")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=100,
                         durability=DurabilityPolicy.VOLATILE)
        self.events = []
        self.heartbeats = []
        self.create_subscription(DiagnosticArray, "/ras/events", self._on_event, qos)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, qos)

    @staticmethod
    def _kv(status):
        return {kv.key: kv.value for kv in status.values}

    def _on_event(self, msg):
        for s in msg.status:
            self.events.append((s.name, int.from_bytes(s.level, "big"), s.message, self._kv(s)))

    def _on_diag(self, msg):
        for s in msg.status:
            if s.name == "ras/monitor":
                self.heartbeats.append((int.from_bytes(s.level, "big"), s.message, self._kv(s)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--max-latency-ms", type=float, default=5000.0,
                    help="ceiling for the monitor's own read-to-publish latency. "
                         "Generous on purpose: it is a smoke bound, and under TCG "
                         "an absolute millisecond figure is a property of the host")
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = ap.parse_args()

    results = []

    def check(label, ok, detail=""):
        results.append((label, bool(ok), detail))
        print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f": {detail}" if detail else ""))

    tmp = pathlib.Path(tempfile.mkdtemp(prefix="ras-monitor-check-"))
    fifo = tmp / "trace_pipe"
    logfile = tmp / "events.log"
    os.mkfifo(fifo)
    (tmp / "trace_clock").write_text(TRACE_CLOCK_TEXT)

    fixtures = [l for l in (SAMPLES / "non_standard_event.trace").read_text().splitlines()
                if l.strip() and not l.startswith("#")]
    if len(fixtures) < 2:
        print(f"FAIL: need at least 2 fixture lines in {SAMPLES}")
        return 1

    rclpy.init(args=None)
    collector = Collector()

    proc = subprocess.Popen(
        [sys.executable, str(HERE / "ras_monitor.py"),
         "--trace-path", str(fifo), "--no-kmsg", "--no-rasdaemon",
         "--log-file", str(logfile), "--heartbeat", "3.0",
         "--exit-after", str(len(fixtures))],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    monitor_out = []
    threading.Thread(target=lambda: [monitor_out.append(l) for l in proc.stdout],
                     daemon=True).start()

    # Opening the FIFO for writing blocks until the monitor opens the read end,
    # which is exactly the handshake we want -- it proves the reader started.
    writer = None
    try:
        deadline = time.time() + args.timeout
        opener = {}

        def _open():
            try:
                opener["fh"] = open(fifo, "w", buffering=1)
            except OSError as exc:
                opener["err"] = exc

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=30.0)
        writer = opener.get("fh")
        check("monitor opened the trace pipe", writer is not None,
              opener.get("err", "") and str(opener["err"]))
        if writer is None:
            raise SystemExit(1)

        # Let discovery settle before the first event, or a RELIABLE publisher
        # with no matched subscriber yet simply drops it.
        settle_end = time.time() + 20.0
        while time.time() < settle_end:
            rclpy.spin_once(collector, timeout_sec=0.2)
            if collector.heartbeats:
                break

        for line in fixtures:
            writer.write(restamp(line, time.clock_gettime(time.CLOCK_MONOTONIC)) + "\n")
            writer.flush()
            time.sleep(1.0)

        # Waiting for "any heartbeat" is not enough: the settle loop above
        # already saw one, and it was published before the first event, so the
        # loop would exit with the final counts still in flight. What is wanted
        # is a heartbeat that has SEEN every event -- which is also the thing
        # being asserted.
        def counted_beat():
            return next((h for h in reversed(collector.heartbeats)
                         if h[2].get("count_trace") == str(len(fixtures))), None)

        while time.time() < deadline and (
                len(collector.events) < len(fixtures) or counted_beat() is None):
            rclpy.spin_once(collector, timeout_sec=0.2)
    finally:
        if writer:
            writer.close()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()

    print()
    check(f"{len(fixtures)} events published on /ras/events",
          len(collector.events) == len(fixtures), f"got {len(collector.events)}")

    by_type = {kv.get("error_type_name"): (name, level, msg, kv)
               for name, level, msg, kv in collector.events}

    tlb = by_type.get("TLB error")
    check("TLB record decoded and delivered", tlb is not None)
    if tlb:
        check("  TLB severity is ERROR (recoverable)", tlb[1] == 2, f"level={tlb[1]}")
        check("  TLB names the ISA", tlb[3].get("proc_isa_name") == "RISCV64",
              tlb[3].get("proc_isa_name", ""))
        check("  TLB record is whole", tlb[3].get("truncated") == "False",
              f"decoded_len={tlb[3].get('decoded_len')}")

    ce = by_type.get("micro-architectural error")
    check("corrected CPU record delivered (what TC.10.007 injects)", ce is not None)
    if ce:
        check("  corrected error is WARN, not ERROR", ce[1] == 1, f"level={ce[1]}")
        check("  operation decoded", ce[3].get("operation_name") == "data write",
              ce[3].get("operation_name", ""))

    lats = [float(kv["detect_latency_ms"]) for _, _, _, kv in collector.events
            if "detect_latency_ms" in kv]
    check("detection latency measured against CLOCK_MONOTONIC",
          len(lats) == len(collector.events), f"{len(lats)}/{len(collector.events)}")
    if lats:
        worst = max(lats)
        check(f"  worst detection latency under {args.max_latency_ms:.0f} ms",
              worst < args.max_latency_ms, f"{worst:.1f} ms (all: "
              + ", ".join(f"{x:.1f}" for x in lats) + ")")

    seqs = sorted(int(kv["seq"]) for _, _, _, kv in collector.events if "seq" in kv)
    check("sequence numbers are complete and gapless",
          seqs == list(range(1, len(fixtures) + 1)), str(seqs))

    lines = [json.loads(l) for l in logfile.read_text().splitlines() if l.strip()] \
        if logfile.exists() else []
    check(f"JSON log written ({logfile.name})", len(lines) == len(fixtures),
          f"{len(lines)} line(s)")
    if lines:
        check("  log records carry the decoded fields",
              all("error_type_name" in r and "ghes_severity_name" in r for r in lines))

    check("heartbeat published on /diagnostics", len(collector.heartbeats) >= 1,
          f"{len(collector.heartbeats)} beat(s)")
    if collector.heartbeats:
        beat = next((h for h in reversed(collector.heartbeats)
                     if h[2].get("count_trace") == str(len(fixtures))), None)
        check("  a heartbeat accounts for every event", beat is not None,
              "counts seen: " + ", ".join(
                  h[2].get("count_trace", "?") for h in collector.heartbeats))
        last = beat or collector.heartbeats[-1]
        check("  heartbeat reports the clock as comparable",
              last[2].get("clock_comparable") == "true", last[2].get("trace_clock", ""))
        check("  no source reported as failed", last[2].get("sources_failed") == "none",
              last[2].get("sources_failed", "?"))

    collector.destroy_node()
    rclpy.shutdown()

    failed = [l for l, ok, _ in results if not ok]
    print()
    if failed:
        print("monitor output:")
        for line in monitor_out[-25:]:
            print("   ", line.rstrip())
    if not args.keep:
        for p in (fifo, logfile, tmp / "trace_clock"):
            p.unlink(missing_ok=True)
        tmp.rmdir()
    else:
        print(f"scratch kept in {tmp}")

    print(f"RESULT: {'PASS' if not failed else 'FAIL'} "
          f"({len(results) - len(failed)}/{len(results)} checks)")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
