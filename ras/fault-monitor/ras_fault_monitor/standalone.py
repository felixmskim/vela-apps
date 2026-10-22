# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""A runner that works without ROS, for two purposes.

1. **Host verification (replay)**: feed a captured kmsg sample and check the parser.
   No QEMU is started, so the edit-verify round trip is one second.
       python3 -m ras_fault_monitor.standalone --replay test/fixtures/reri_sample.kmsg --stdout

2. **A guest fallback path**: the fault log has to survive even when rclpy misbehaves
   (the P-014 family).
       sudo python3 -m ras_fault_monitor.standalone --out ~/fault_logs/kernel.jsonl

Example:
    python3 -m ras_fault_monitor.standalone --duration 120 --out /tmp/faults.jsonl --stdout
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

from .pipeline import FaultPipeline, _uptime_now, make_reader
from .kmsg import ReplayReader


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="ras_fault_monitor.standalone",
        description="/dev/kmsg fault reader (independent of ROS)",
    )
    p.add_argument("--source", default="/dev/kmsg",
                   help="what to read (default /dev/kmsg)")
    p.add_argument("--replay", metavar="FILE",
                   help="replay a file as if it were kmsg (host verification)")
    p.add_argument("--out", metavar="FILE", help="JSONL log output path")
    p.add_argument("--stdout", action="store_true",
                   help="also print the classification to the screen")
    p.add_argument("--json", action="store_true",
                   help="print raw JSON to the screen")
    p.add_argument("--duration", type=float, default=0.0,
                   help="run time in seconds (0 = forever)")
    p.add_argument("--poll", type=float, default=0.2,
                   help="poll period in seconds")
    p.add_argument("--from-beginning", action="store_true",
                   help="read the whole kernel log from boot (default: new messages only)")
    p.add_argument("--ghes-timeout", type=float, default=0.5,
                   help="seconds to wait before closing a GHES block")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    source = args.replay or args.source
    reader = ReplayReader(source) if args.replay else make_reader(
        source, from_beginning=args.from_beginning
    )
    replay_mode = isinstance(reader, ReplayReader)
    pipeline = FaultPipeline(
        node_name="standalone", ghes_timeout=args.ghes_timeout, live=not replay_mode
    )

    out_fp = None
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        out_fp = open(args.out, "a", encoding="utf-8", buffering=1)

    try:
        reader.open()
    except PermissionError:
        print(
            f"error: cannot open {source} (CAP_SYSLOG required).\n"
            "  sudo sysctl -w kernel.dmesg_restrict=0   or run under sudo",
            file=sys.stderr,
        )
        return 13

    started = time.time()

    def emit(faults) -> None:
        for fault in faults:
            line = fault.to_json()
            if out_fp:
                out_fp.write(line + "\n")
            if args.stdout or replay_mode:
                print(line if args.json else fault.one_line())

    try:
        while True:
            emit(pipeline.process(reader.read_available()))
            if not replay_mode:
                # in replay mode the host clock must not decide (its axis differs from
                # the times inside the file)
                emit(pipeline.tick(_uptime_now()))

            if replay_mode and reader.exhausted:
                emit(pipeline.flush())
                break
            if args.duration and time.time() - started >= args.duration:
                emit(pipeline.flush())
                break
            time.sleep(args.poll)
    except KeyboardInterrupt:
        emit(pipeline.flush())
    finally:
        reader.close()
        if out_fp:
            out_fp.close()

    stats = pipeline.stats
    summary = {
        "records_read": stats.records_read,
        "faults_emitted": stats.faults_emitted,
        "ghes_events": stats.ghes_events,
        "lost_chunks": getattr(reader, "lost_chunks", 0),
        "by_type": stats.by_type,
        "out": args.out,
    }
    print("=== summary ===", json.dumps(summary, ensure_ascii=False), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
