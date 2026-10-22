# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The Fault record — the shared schema for the log file and the ROS 2 topic.

The field set follows the ETRI deck (2026-05-22, p.14) **fault classification**
exactly, extended with what RERI hardware events need.

  ETRI Fault message         ->  this schema
  ------------------------------------------------
  fault_type                 ->  fault_type
  severity                   ->  severity
  source_layer               ->  source_layer
  recoverable                ->  recoverable
  raw_log                    ->  raw_log
  tags                       ->  tags
  (extra fields in their JSON)-> timestamp / node_name / topic_name / description
                                expected_value / actual_value
  (our extension)            ->  detail{}  GHES parse results (error_type, operation, ...)
                                kmsg{}    kernel record metadata (seq, uptime, level)

`detail` and `kmsg` are separate dicts because the values a test verdict depends on
(error_type, detection time) must be **readable straight out of a field** rather than
re-scraped from the description text with a regex.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field

#: ETRI p.14 classification plus our extension (INJECTION_MARKER)
FAULT_TYPES = (
    "SYSTEM_FAULT",       # kernel-level failure (panic, OOM, lockup)
    "SYSTEM_DEGRADED",    # kernel-level degradation (CPU offline, memory failure)
    "DRIVER_FAULT",       # driver error
    "RERI_FAULT",         # RISC-V hardware error (via GHES/APEI)
    "DDS_FAULT",          # ROS 2 / DDS communication error
    "SENSOR_HW_FAULT",    # sensor hardware fault
    "SENSOR_LOGIC_FAULT",  # sensor logical fault
    "INJECTION_MARKER",   # (extension) marks injection time and target hart
)

SEVERITIES = ("INFO", "WARN", "ERROR", "CRITICAL")


def format_timestamp(unix_time: float) -> str:
    """`"1778570146.893353343"` — string with ns precision, as in the ETRI JSON."""
    sec = int(unix_time)
    nsec = int(round((unix_time - sec) * 1_000_000_000))
    if nsec >= 1_000_000_000:      # rounding pushed us past a whole second
        sec += 1
        nsec -= 1_000_000_000
    return f"{sec}.{nsec:09d}"


@dataclass
class Fault:
    fault_type: str
    severity: str
    description: str
    source_layer: str = "kernel"
    recoverable: bool = True
    raw_log: str = ""
    tags: list[str] = field(default_factory=list)
    node_name: str = "kernel_monitor"
    topic_name: str = ""
    expected_value: float = 0.0
    actual_value: float = 0.0
    timestamp: float = field(default_factory=time.time)
    detail: dict = field(default_factory=dict)
    kmsg: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "timestamp": format_timestamp(self.timestamp),
            "fault_type": self.fault_type,
            "severity": self.severity,
            "source_layer": self.source_layer,
            "recoverable": self.recoverable,
            "node_name": self.node_name,
            "topic_name": self.topic_name,
            "description": self.description,
            "expected_value": self.expected_value,
            "actual_value": self.actual_value,
            "tags": list(self.tags),
            "detail": self.detail,
            "kmsg": self.kmsg,
            "raw_log": self.raw_log,
        }

    def to_json(self) -> str:
        # ensure_ascii=False keeps non-ASCII descriptions readable in the log
        return json.dumps(self.to_dict(), ensure_ascii=False)

    @staticmethod
    def from_dict(data: dict) -> "Fault":
        ts = data.get("timestamp", "0")
        try:
            ts_float = float(ts)
        except (TypeError, ValueError):
            ts_float = 0.0
        return Fault(
            fault_type=data.get("fault_type", "UNKNOWN"),
            severity=data.get("severity", "INFO"),
            description=data.get("description", ""),
            source_layer=data.get("source_layer", "kernel"),
            recoverable=bool(data.get("recoverable", True)),
            raw_log=data.get("raw_log", ""),
            tags=list(data.get("tags", [])),
            node_name=data.get("node_name", ""),
            topic_name=data.get("topic_name", ""),
            expected_value=float(data.get("expected_value", 0.0)),
            actual_value=float(data.get("actual_value", 0.0)),
            timestamp=ts_float,
            detail=dict(data.get("detail", {})),
            kmsg=dict(data.get("kmsg", {})),
        )

    def one_line(self) -> str:
        """One line for console output, shaped like the ETRI demo screen."""
        tags = f" [{','.join(self.tags)}]" if self.tags else ""
        return f"[{self.severity}] {self.fault_type}: {self.description}{tags}"
