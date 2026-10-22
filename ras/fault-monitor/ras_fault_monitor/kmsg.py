# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""`/dev/kmsg` reader — reads the kernel log **record by record**.

Why `/dev/kmsg` and not `dmesg -w`:
- `dmesg` is human-readable text; `/dev/kmsg` hands over the
  `priority,sequence,timestamp,flags;message` structure directly, so parsing rests
  on a specification rather than on guesswork.
- No subprocess (fork cost is not negligible under TCG emulation).

Three traps in `/dev/kmsg` (confirmed against the kernel source and
Documentation/ABI/testing/dev-kmsg):
1. **open() starts at the front of the buffer** (`devkmsg_open` starts from
   `prb_first_valid_seq`), so the whole log since boot pours out. For new messages
   only, `lseek(fd, 0, SEEK_END)`.
2. **If the reader falls behind and records are overwritten, read() returns `EPIPE`.**
   That is not an error but a "records were lost" signal — count it and keep reading.
3. A record may carry ` KEY=value` continuation lines after a `\n`, so the message
   body is only the first line.
"""

from __future__ import annotations

import errno
import os
import re
import time
from dataclasses import dataclass

#: escapes the kernel puts in the body for non-printable characters (`\x0a` etc.)
_ESCAPE_RE = re.compile(r"\\x([0-9a-fA-F]{2})")

#: maximum kernel record length (CONSOLE_EXT_LOG_MAX). A smaller read gets EINVAL.
_READ_SIZE = 8192

_LEVEL_NAMES = {
    0: "EMERG",
    1: "ALERT",
    2: "CRIT",
    3: "ERR",
    4: "WARNING",
    5: "NOTICE",
    6: "INFO",
    7: "DEBUG",
}


@dataclass(frozen=True)
class KmsgRecord:
    """A single `/dev/kmsg` record."""

    priority: int          # facility * 8 + level
    seq: int               # kernel-global sequence number (usable for loss detection)
    ts_usec: int           # microseconds since boot (printk timestamp)
    flags: str             # '-', 'c' (continuation), etc.
    text: str              # message body (first line only)
    raw: str               # original record string, kept as evidence
    wall_time: float       # estimated wall clock (unix epoch) = boot_time + ts
    caller: str = ""       # CONFIG_PRINTK_CALLER 'T<pid>' / 'C<cpu>' ('' if absent)

    @property
    def facility(self) -> int:
        return self.priority >> 3

    @property
    def level(self) -> int:
        return self.priority & 7

    @property
    def level_name(self) -> str:
        return _LEVEL_NAMES.get(self.level, str(self.level))

    @property
    def ts_monotonic(self) -> float:
        """Seconds since boot — the same value as `dmesg`'s `[  256.021037]`."""
        return self.ts_usec / 1_000_000.0

    @property
    def caller_cpu(self) -> int | None:
        """`caller=C7` -> 7, a **Linux CPU number**, not a hart id.

        The kernel prints `T<pid>` in task context and `C<cpu>` in interrupt or
        no-task context. GHES reports arrive over SSE, so they come as `C<cpu>`
        (measured) — which is what tells us which core the error came from. To use
        it as a hart it **must** go through :class:`~.topology.HartTopology`.
        """
        if self.caller.startswith("C"):
            try:
                return int(self.caller[1:])
            except ValueError:
                return None
        return None

    @property
    def caller_pid(self) -> int | None:
        """`caller=T1234` -> 1234, when printed from task context."""
        if self.caller.startswith("T"):
            try:
                return int(self.caller[1:])
            except ValueError:
                return None
        return None


def estimate_boot_time() -> float:
    """Estimate boot time (unix epoch) — the anchor that maps kmsg stamps to wall clock."""
    try:
        with open("/proc/uptime", "r") as fp:
            uptime = float(fp.readline().split()[0])
        return time.time() - uptime
    except (OSError, ValueError, IndexError):
        return time.time() - time.monotonic()


def unescape(text: str) -> str:
    return _ESCAPE_RE.sub(lambda m: chr(int(m.group(1), 16)), text)


def parse_record(raw: str, boot_time: float) -> KmsgRecord | None:
    """Record string -> :class:`KmsgRecord`. None if it does not match the format."""
    raw = raw.rstrip("\n")
    if not raw:
        return None
    head, sep, body = raw.partition(";")
    if not sep:
        return None
    fields = head.split(",")
    if len(fields) < 3:
        return None
    try:
        priority = int(fields[0])
        seq = int(fields[1])
        ts_usec = int(fields[2])
    except ValueError:
        return None
    flags = fields[3] if len(fields) > 3 else "-"
    # With CONFIG_PRINTK_CALLER=y one more field, `caller=C7` / `caller=T0`, appears
    # before the `;`. It is found by name rather than pinned to index 4, so another
    # added field does not break this.
    caller = ""
    for extra in fields[4:]:
        if extra.startswith("caller="):
            caller = extra[len("caller="):].strip()
            break
    text = unescape(body.split("\n", 1)[0])
    return KmsgRecord(
        priority=priority,
        seq=seq,
        ts_usec=ts_usec,
        flags=flags,
        text=text,
        raw=raw,
        wall_time=boot_time + ts_usec / 1_000_000.0,
        caller=caller,
    )


class KmsgReader:
    """Non-blocking `/dev/kmsg` reader.

    Used by calling :meth:`read_available` periodically from an rclpy timer. A
    blocking read inside a node would stall the executor, so this polls instead of
    running a separate thread.
    """

    def __init__(self, path: str = "/dev/kmsg", from_beginning: bool = False):
        self.path = path
        self.from_beginning = from_beginning
        self.boot_time = estimate_boot_time()
        self.lost_chunks = 0        # EPIPE count = number of lost stretches
        self.malformed = 0
        self._fd: int | None = None

    def open(self) -> None:
        fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        if not self.from_beginning:
            # SEEK_END moves past the last record, skipping the existing boot log
            os.lseek(fd, 0, os.SEEK_END)
        self._fd = fd

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def read_available(self, max_records: int = 500) -> list[KmsgRecord]:
        """Take every record available now, up to max_records."""
        if self._fd is None:
            self.open()
        out: list[KmsgRecord] = []
        for _ in range(max_records):
            try:
                chunk = os.read(self._fd, _READ_SIZE)
            except BlockingIOError:
                break                     # EAGAIN — nothing more right now (normal)
            except OSError as exc:
                if exc.errno == errno.EPIPE:
                    self.lost_chunks += 1  # records overwritten; continue from the next
                    continue
                raise
            if not chunk:
                break
            rec = parse_record(chunk.decode("utf-8", "replace"), self.boot_time)
            if rec is None:
                self.malformed += 1
                continue
            out.append(rec)
        return out

    # -- context manager --------------------------------------------------
    def __enter__(self) -> "KmsgReader":
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


class ReplayReader:
    """Replay a file as if it were `/dev/kmsg` — for verifying the parser on the host.

    The point is to exercise the parsing logic without booting QEMU; the guest is
    slow under TCG, so repeated logic checks finish on the host.
    """

    def __init__(self, path: str, boot_time: float | None = None):
        self.path = path
        self.boot_time = boot_time if boot_time is not None else estimate_boot_time()
        self.lost_chunks = 0
        self.malformed = 0
        self._lines: list[str] | None = None
        self._pos = 0

    def open(self) -> None:
        records: list[str] = []
        with open(self.path, "r", encoding="utf-8") as fp:
            for line in fp.read().split("\n"):
                if not line.strip():
                    continue
                if line.startswith((" ", "\t")) and records:
                    # continuation lines such as ` SUBSYSTEM=usb` belong to the record above
                    records[-1] += "\n" + line
                    continue
                records.append(line)
        self._lines = records
        self._pos = 0

    def close(self) -> None:
        self._lines = None

    def read_available(self, max_records: int = 500) -> list[KmsgRecord]:
        if self._lines is None:
            self.open()
        out: list[KmsgRecord] = []
        while self._lines and self._pos < len(self._lines) and len(out) < max_records:
            rec = parse_record(self._lines[self._pos], self.boot_time)
            self._pos += 1
            if rec is None:
                self.malformed += 1
                continue
            out.append(rec)
        return out

    @property
    def exhausted(self) -> bool:
        return self._lines is not None and self._pos >= len(self._lines)

    def __enter__(self) -> "ReplayReader":
        self.open()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
