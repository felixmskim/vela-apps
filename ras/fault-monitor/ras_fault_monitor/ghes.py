# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""**Reassembler** for GHES/APEI hardware error reports.

One RERI injection produces **ten kernel log lines, not one** (measured on q-vela):

    {2}[Hardware Error]: Hardware error from APEI Generic Hardware Error Source: 1
    {2}[Hardware Error]: event severity: recoverable
    {2}[Hardware Error]:  Error 0, type: recoverable
    {2}[Hardware Error]:   section_type: general processor error
    {2}[Hardware Error]:   processor_type: 3, RISCV
    {2}[Hardware Error]:   processor_isa: 6, RISCV64
    {2}[Hardware Error]:   error_type: 0x02
    {2}[Hardware Error]:   TLB error
    {2}[Hardware Error]:   operation: 1, data read
    {2}[Hardware Error]:   target_address: 0x0000000000000000

The leading `{2}` is the **event serial number** the kernel GHES code
(`cper_estatus_print`) attaches. Same number = same single error. So the parser's
first job is not "find the lines" but **group by that number and restore one event**.
Handling it line by line would inflate one injection into ten log entries and throw
off every verdict that counts records (TC.10.007 "0% detection miss", "N CE records").
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .kmsg import KmsgRecord

#: `{2}[Hardware Error]:   error_type: 0x02` → ('2', '  error_type: 0x02')
HE_LINE_RE = re.compile(r"^\{(\d+)\}\[Hardware Error\]:\s?(.*)$")

#: CPER generic processor error types (bit flags). Measured on q-vela: 0x02=TLB, 0x08=micro-arch
ERROR_TYPE_NAMES = {
    0x01: "cache error",
    0x02: "TLB error",
    0x04: "bus error",
    0x08: "micro-architectural error",
}

#: `event severity` -> (severity, recoverable)
#: The correctable/uncorrectable distinction TC.04.012 (severity-based policy response)
#: requires comes from here.
SEVERITY_MAP = {
    "corrected": ("WARN", True),
    "recoverable": ("ERROR", True),
    "fatal": ("CRITICAL", False),
    "corrective action required": ("ERROR", True),
    "informational": ("INFO", True),
    "none": ("INFO", True),
}

#: `error_type` enumeration for the CPER Memory Error Section (UEFI Appendix N,
#: Table N-27). **The notation differs from the processor section**: processor uses a
#: hex bit flag (`0x08`), memory uses a decimal enum plus a name (`3, multi-bit ECC`).
#: The kernel `cper.c` prints them that way per section; it is not a bug, so parsing
#: has to branch per section too. This path only appears after the OpenSBI RAS agent
#: patches (memory-section branch + cper_mem2_sec aligned to UEFI) — before them every
#: error was reported as a processor section.
MEM_ERROR_TYPE_NAMES = {
    0: "unknown",
    1: "no error",
    2: "single-bit ECC",
    3: "multi-bit ECC",
    4: "single-symbol chipkill ECC",
    5: "multi-symbol chipkill ECC",
    6: "master abort",
    7: "target abort",
    8: "parity error",
    9: "watchdog timeout",
    10: "invalid address",
    11: "mirror Broken",
    12: "memory sparing",
    13: "scrub corrected error",
    14: "scrub uncorrected error",
    15: "physical memory map-out event",
}

#: `error_type: 3, multi-bit ECC` → ('3', 'multi-bit ECC')
_ENUM_VALUE_RE = re.compile(r"^\s*(\d+)\s*,\s*(.+?)\s*$")

_KV_RE = re.compile(r"^([A-Za-z_][\w ]*?):\s*(.*)$")
_ERROR_N_RE = re.compile(r"^Error (\d+), type:\s*(.*)$")
_APEI_SRC_RE = re.compile(r"^Hardware error from APEI Generic Hardware Error Source:\s*(\d+)$")


@dataclass
class GhesEvent:
    """One reassembled hardware error."""

    seq_id: int                                   # `{N}` kernel GHES event number
    records: list[KmsgRecord] = field(default_factory=list)
    fields: dict = field(default_factory=dict)    # parsed key/value pairs
    notes: list[str] = field(default_factory=list)  # descriptive lines like 'TLB error'

    @property
    def first_ts(self) -> float:
        return self.records[0].ts_monotonic if self.records else 0.0

    @property
    def last_ts(self) -> float:
        return self.records[-1].ts_monotonic if self.records else 0.0

    @property
    def wall_time(self) -> float:
        return self.records[0].wall_time if self.records else 0.0

    @property
    def line_count(self) -> int:
        return len(self.records)

    @property
    def raw(self) -> str:
        return "\n".join(r.raw for r in self.records)

    @property
    def event_severity(self) -> str:
        return self.fields.get("event severity", "unknown")

    @property
    def section_type(self) -> str:
        """`general processor error` / `memory error` / '' when not reported."""
        return str(self.fields.get("section_type", ""))

    @property
    def is_memory_section(self) -> bool:
        return "memory error" in self.section_type

    @property
    def error_type(self) -> int | None:
        """The error_type value, read with the notation its section actually uses.

        The processor section carries a **hex bit flag** such as `0x08`; the memory
        section carries a **decimal enum** such as `3, multi-bit ECC`. Handling both
        with a single `int(value, 0)` makes the memory case raise ValueError and
        silently become None, so the log loses the value the firmware reported
        correctly (confirmed by measurement).
        """
        value = self.fields.get("error_type")
        if value is None:
            return None
        text = str(value)
        enum_match = _ENUM_VALUE_RE.match(text)
        if enum_match:                       # '3, multi-bit ECC' shape
            return int(enum_match.group(1))
        try:
            return int(text, 0)              # '0x08' shape
        except ValueError:
            return None

    @property
    def error_type_name(self) -> str:
        code = self.error_type
        if code is None:
            return "unknown"
        if self.is_memory_section:
            # The kernel prints the name too, so that string wins. The local table
            # is second: when the kernel's table is updated, its names change first.
            enum_match = _ENUM_VALUE_RE.match(str(self.fields.get("error_type", "")))
            if enum_match:
                return enum_match.group(2)
            return MEM_ERROR_TYPE_NAMES.get(code, "unknown")
        names = [name for bit, name in ERROR_TYPE_NAMES.items() if code & bit]
        # For a bit not in the spec, fall back to a descriptive line ('TLB error')
        return ", ".join(names) if names else (self.notes[0] if self.notes else "unknown")

    @property
    def physical_address(self) -> int | None:
        """Physical address from the memory section — the input to page isolation."""
        value = self.fields.get("physical_address")
        if value is None:
            return None
        try:
            return int(str(value), 0)
        except ValueError:
            return None

    def pfn(self, page_size: int = 4096) -> int | None:
        """Physical address -> PFN, the value written to `hwpoison/corrupt-pfn`."""
        addr = self.physical_address
        return None if addr is None else addr // page_size

    @property
    def severity(self) -> str:
        return SEVERITY_MAP.get(self.event_severity, ("ERROR", True))[0]

    @property
    def recoverable(self) -> bool:
        return SEVERITY_MAP.get(self.event_severity, ("ERROR", True))[1]

    def summary(self) -> str:
        """A one-line human-readable summary."""
        parts = [f"GHES#{self.seq_id}", self.event_severity]
        if self.section_type:
            parts.append(self.section_type.replace(" error", ""))
        if self.error_type is not None:
            # Show each section in its own notation — printing a decimal enum as hex
            # would stop the line from matching kernel dmesg.
            shown = (str(self.error_type) if self.is_memory_section
                     else f"0x{self.error_type:02x}")
            parts.append(f"error_type={shown}({self.error_type_name})")
        if self.physical_address is not None:
            parts.append(f"pa=0x{self.physical_address:x} pfn=0x{self.pfn():x}")
        if "operation" in self.fields:
            parts.append(f"operation={self.fields['operation']}")
        if "target_address" in self.fields:
            parts.append(f"target={self.fields['target_address']}")
        return " ".join(parts)


class GhesAssembler:
    """Feed kmsg records in, get completed :class:`GhesEvent` objects out.

    An event is complete when:
      * the `{N}` number changes (the previous event closes)
      * `timeout` seconds pass after its last line (nothing more is coming)
      * :meth:`flush` is called explicitly (on node shutdown)

    Note: an unrelated kernel message in the middle does **not** close an event —
    printk can interleave, so only the `{N}` number is treated as a boundary.

    Why `timeout` defaults to 0.5 s: a measured block prints all ten lines **within
    5 ms** (`256.021037` -> `256.026094`). That is a 100x margin, and since this value
    is the floor of detection latency (TC.10.007 "CPU CE detection response <= 20 s /
    10 s") it is deliberately not set larger than needed.
    """

    def __init__(self, timeout: float = 0.5):
        self.timeout = timeout
        self._current: GhesEvent | None = None
        self._last_ts: float = 0.0

    @staticmethod
    def match(record: KmsgRecord) -> tuple[int, str] | None:
        m = HE_LINE_RE.match(record.text)
        if not m:
            return None
        return int(m.group(1)), m.group(2)

    def feed(self, record: KmsgRecord) -> list[GhesEvent]:
        """Feed one record; return the events completed by it (usually zero or one)."""
        done: list[GhesEvent] = []
        hit = self.match(record)

        if hit is None:
            # Not a hardware-error line. If enough time has passed, close the open event.
            if self._current and record.ts_monotonic - self._last_ts > self.timeout:
                done.append(self._current)
                self._current = None
            return done

        seq_id, payload = hit
        if self._current is not None and self._current.seq_id != seq_id:
            done.append(self._current)
            self._current = None
        if self._current is None:
            self._current = GhesEvent(seq_id=seq_id)

        self._current.records.append(record)
        self._absorb(self._current, payload)
        self._last_ts = record.ts_monotonic
        return done

    def flush(self) -> list[GhesEvent]:
        """Force the in-progress event to complete (called just before shutdown)."""
        if self._current is None:
            return []
        event, self._current = self._current, None
        return [event]

    def flush_if_stale(self, now_monotonic: float) -> list[GhesEvent]:
        """Complete the event if timeout has passed since its last line.

        Called periodically from a timer. On a quiet system where **no kernel message
        follows the last line of a GHES block**, the event would never close without
        this path, because feed() only decides when the next line arrives.
        """
        if self._current is not None and now_monotonic - self._last_ts > self.timeout:
            return self.flush()
        return []

    @property
    def pending(self) -> bool:
        return self._current is not None

    # -- internal -----------------------------------------------------------
    @staticmethod
    def _absorb(event: GhesEvent, payload: str) -> None:
        text = payload.strip()
        if not text:
            return

        m = _APEI_SRC_RE.match(text)
        if m:
            event.fields["ghes_source"] = int(m.group(1))
            return

        m = _ERROR_N_RE.match(text)
        if m:
            event.fields["error_index"] = int(m.group(1))
            event.fields["error_severity"] = m.group(2).strip()
            return

        m = _KV_RE.match(text)
        if m:
            key, value = m.group(1).strip(), m.group(2).strip()
            event.fields[key] = value
            return

        # 'TLB error', 'micro-architectural error',
        # A descriptive line rather than key:value, e.g.
        # 'It has been corrected by h/w and requires no further action'
        event.notes.append(text)
