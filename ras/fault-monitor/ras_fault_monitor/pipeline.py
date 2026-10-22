# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Read -> reassemble -> classify pipeline (no ROS dependency).

The ROS 2 nodes and the standalone runner use **exactly this code**, so
  * parser verification can run on the host without QEMU, and
  * log output continues even when rclpy misbehaves.

On hart correlation
-------------------
The RERI device is per-hart (`0x4010000 + 0x1000 * hart`, measured), so the injecting
side always knows which hart it targeted. What the kernel reports has changed over
time: the core-identification patch now puts the hart in `processor_id`, and
`CONFIG_PRINTK_CALLER` adds `caller=C<n>` to the kmsg metadata. Both are handled in
:func:`~.classify.classify_ghes`.

The injection marker (`RASINJ hart=N ...`, written to `/dev/kmsg` just before
injecting) is therefore no longer the only source. It is kept for two things:
recording the injection time and intent, and **cross-checking** what the kernel
reported. When the two disagree it is surfaced as `hart_mismatch` rather than
overwritten — the disagreement itself is the finding. Where the marker is the only
source available, `detail.hart_source = "injection_marker"` marks it as the weaker
grade of evidence.
"""

from __future__ import annotations

import os
import stat
import time
from dataclasses import dataclass, field

from .classify import classify_ghes, classify_line
from .fault import Fault
from .ghes import GhesAssembler
from .kmsg import KmsgReader, KmsgRecord, ReplayReader
from .topology import HartTopology


@dataclass
class PipelineStats:
    records_read: int = 0
    faults_emitted: int = 0
    ghes_events: int = 0
    lost_chunks: int = 0
    by_type: dict = field(default_factory=dict)

    def note(self, fault: Fault) -> None:
        self.faults_emitted += 1
        self.by_type[fault.fault_type] = self.by_type.get(fault.fault_type, 0) + 1


class FaultPipeline:
    """Feed kmsg records in, get :class:`Fault` objects out."""

    def __init__(
        self,
        node_name: str = "kernel_monitor",
        ghes_timeout: float = 0.5,
        marker_window: float = 60.0,
        live: bool = True,
        topology: HartTopology | None = None,
    ):
        self.node_name = node_name
        # The hart<->cpu mapping can differ per boot, so it is read once at construction.
        self.topology = topology if topology is not None else HartTopology.load()
        self.assembler = GhesAssembler(timeout=ghes_timeout)
        self.marker_window = marker_window
        # In replay mode the host clock and the timestamps in the file are different
        # axes, so no wall-clock latency is computed — it would write nonsense values
        # of millions of seconds into the log.
        self.live = live
        self.stats = PipelineStats()
        self._last_marker: Fault | None = None

    def process(self, records: list[KmsgRecord]) -> list[Fault]:
        faults: list[Fault] = []
        for record in records:
            self.stats.records_read += 1

            # (1) Reassemble the GHES block; a completed event becomes a RERI_FAULT
            for event in self.assembler.feed(record):
                faults.append(self._emit_ghes(event))

            # (2) The GHES lines are not classified individually — ten lines becoming
            if GhesAssembler.match(record) is not None:
                continue

            fault = classify_line(record, node_name=self.node_name,
                                  topology=self.topology)
            if fault is not None:
                if fault.fault_type == "INJECTION_MARKER":
                    self._last_marker = fault
                self._stamp_emit(fault)
                self.stats.note(fault)
                faults.append(fault)
        return faults

    def tick(self, now_monotonic: float | None = None) -> list[Fault]:
        """Close an unfinished GHES block during a quiet stretch; call periodically."""
        if now_monotonic is None:
            now_monotonic = _uptime_now()
        return [self._emit_ghes(e) for e in self.assembler.flush_if_stale(now_monotonic)]

    def flush(self) -> list[Fault]:
        """Emit any remaining events just before shutdown."""
        return [self._emit_ghes(e) for e in self.assembler.flush()]

    # -- internal -----------------------------------------------------------
    def _stamp_emit(self, fault: Fault) -> None:
        """Detection delay = (when the monitor settled the event) - (when the kernel
        printed the line).

        The value itself is written into the record so the TC.10.007 verdict
        ("detection response <= 20 s / 10 s") can be computed **from the log alone**.
        `fault.timestamp` stays the kernel time. It is attached to line-level events
        such as core isolation (`CPU3: off`) as well as RERI events, so that the
        fault-to-isolation duration closes inside the log (TC.04.008/013).
        """
        if not self.live:
            return
        emit_wall = time.time()
        fault.detail["emit_wall_time"] = round(emit_wall, 6)
        fault.detail["detect_delay_s"] = round(emit_wall - fault.timestamp, 6)

    def _emit_ghes(self, event) -> Fault:
        fault = classify_ghes(event, node_name=self.node_name,
                              topology=self.topology)
        self._attach_hart(fault)
        self._stamp_emit(fault)
        self.stats.ghes_events += 1
        self.stats.note(fault)
        return fault

    def _attach_hart(self, fault: Fault) -> None:
        """Attach the most recent injection marker to the event.

        The kernel-reported values are the primary evidence for hart identity (handled
        in classify_ghes). The marker is used for two things: recording injection time
        and intent, and **cross-checking** the reported value. A disagreement is
        surfaced as `hart_mismatch` rather than overwritten — it is itself a finding
        about the hart numbering or the source-id assignment.
        """
        marker = self._last_marker
        if marker is None or "hart" not in marker.detail:
            return
        if fault.timestamp - marker.timestamp > self.marker_window:
            return                        # marker too old to claim a correlation

        marker_hart = str(marker.detail["hart"])
        fault.detail["inject_ts"] = marker.timestamp
        fault.detail["detect_latency_s"] = round(fault.timestamp - marker.timestamp, 6)
        for key in ("base", "code", "tag", "ec", "class", "tt"):
            if key in marker.detail:
                fault.detail[f"inject_{key}"] = marker.detail[key]

        if "hart" in fault.detail:                 # kernel already reported it; only compare
            fault.detail["hart_marker"] = marker_hart
            if str(fault.detail["hart"]) != marker_hart:
                fault.detail["hart_mismatch"] = {
                    "ghes_source": fault.detail["hart"], "marker": marker_hart,
                }
                fault.tags.append("hart_mismatch")
        else:                                       # fallback: marker guess (weaker evidence)
            fault.detail["hart"] = marker_hart
            fault.detail["hart_source"] = "injection_marker"
            fault.tags.append(f"hart:{marker_hart}")


def _uptime_now() -> float:
    """Seconds since boot — the same axis as the kmsg timestamps."""
    try:
        with open("/proc/uptime", "r") as fp:
            return float(fp.readline().split()[0])
    except (OSError, ValueError, IndexError):
        return time.monotonic()


def make_reader(source: str, from_beginning: bool = False):
    """A replayer for a regular file, the real reader for a character device.

    The decision is not made by extension: `/dev/kmsg` itself ends in `kmsg`, so any
    string rule is guaranteed to misfire eventually. The file type (stat) decides.
    """
    try:
        mode = os.stat(source).st_mode
        if stat.S_ISREG(mode):
            return ReplayReader(source)
    except OSError:
        pass
    return KmsgReader(source, from_beginning=from_beginning)
