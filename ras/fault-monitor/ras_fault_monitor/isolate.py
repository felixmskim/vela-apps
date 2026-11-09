# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The isolation path — actually detaching a page or a core for a detected fault.

There are two routes and they carry different risk.

**Page isolation** goes through `corrupt-pfn` from `hwpoison-inject`, which calls
`memory_failure()` directly. This environment has no `CONFIG_MEMORY_HOTPLUG`, so
`/sys/devices/system/memory/` does not exist at all: the usual
`soft_offline_page` / `hard_offline_page` are unavailable and `corrupt-pfn` is the only
entry point. But it **panics outright on a kernel page** (slab, page table, reserved),
and the physical address used in injection testing is arbitrary (e.g. 0x90000000), so
handing it over unchecked destroys the VM. Hence:

  1. is the PFN inside a System RAM range from `/proc/iomem`?
  2. what is the page, according to `/proc/kpageflags`? slab / page table / reserved
     are refused.

**Core isolation** writes 0 to `/sys/devices/system/cpu/cpuN/online`, where N is a
**Linux CPU number**. RERI reports hart ids, so the :mod:`~.topology` translation is
mandatory (the boot hart takes cpu0 and shifts the rest, e.g. hart6 -> cpu7). Using the
hart number directly kills the wrong core.

Design rules
------------
* **Dry-run by default.** An irreversible write requires an explicit `armed=True`.
* **Refuse when unsure.** If the page flags cannot be read or the topology is
  unresolved, nothing runs. Isolating on a guess leaves a log that says "isolated"
  while nothing was isolated.
* **Record the outcome.** Every attempt produces an :class:`IsolationResult` so that
  fault -> response -> result closes inside the log.
"""

from __future__ import annotations

import os
import struct
from dataclasses import dataclass, field

from .topology import HartTopology

HWPOISON_DIR = "/sys/kernel/debug/hwpoison"
CORRUPT_PFN = HWPOISON_DIR + "/corrupt-pfn"
UNPOISON_PFN = HWPOISON_DIR + "/unpoison-pfn"
KPAGEFLAGS = "/proc/kpageflags"
IOMEM = "/proc/iomem"
CPU_DIR = "/sys/devices/system/cpu"
PAGE_SIZE = 4096

#: `/proc/kpageflags` bits (Documentation/admin-guide/mm/pagemap.rst).
#: Only the ones used here.
KPF = {
    "LOCKED": 0, "ERROR": 1, "DIRTY": 4, "LRU": 5, "SLAB": 7,
    "BUDDY": 10, "MMAP": 11, "ANON": 12, "COMPOUND_HEAD": 15,
    "COMPOUND_TAIL": 16, "HUGE": 17, "HWPOISON": 19, "NOPAGE": 20,
    "KSM": 21, "THP": 22, "OFFLINE": 23, "ZERO_PAGE": 24, "PGTABLE": 26,
    "RESERVED": 32,
}

#: With any of these flags, `memory_failure()` touches kernel structures and panics.
#: `force` does not override them — destroying the VM is not a "response".
FATAL_FLAGS = ("SLAB", "PGTABLE", "RESERVED", "NOPAGE")

#: These work but have wide blast radius, so they require `force`.
RISKY_FLAGS = ("COMPOUND_HEAD", "COMPOUND_TAIL", "HUGE", "THP", "KSM", "ZERO_PAGE")

#: Already handled pages — no need to act again.
DONE_FLAGS = ("HWPOISON", "OFFLINE")


@dataclass
class IsolationResult:
    """The outcome of one isolation attempt, recorded in the log as-is."""

    action: str                  # 'page_isolate' | 'page_restore' | 'core_isolate' | ...
    target: str                  # 'pfn=0x90000' | 'cpu=7 (hart6)'
    outcome: str                 # 'applied' | 'dry_run' | 'refused' | 'error' | 'noop'
    reason: str = ""
    reversible: bool = True
    detail: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.outcome in ("applied", "dry_run", "noop")

    def one_line(self) -> str:
        return f"[{self.outcome}] {self.action} {self.target}" + (
            f" — {self.reason}" if self.reason else "")

    def to_dict(self) -> dict:
        return {
            "action": self.action, "target": self.target, "outcome": self.outcome,
            "reason": self.reason, "reversible": self.reversible,
            "detail": dict(self.detail),
        }


class RamMap:
    """System RAM ranges from `/proc/iomem`, used to validate a PFN."""

    def __init__(self, ranges: list[tuple[int, int]] | None = None,
                 resolved: bool = True):
        self.ranges = list(ranges or [])
        self.resolved = resolved and bool(self.ranges)

    def contains_pfn(self, pfn: int, page_size: int = PAGE_SIZE) -> bool:
        addr = pfn * page_size
        return any(lo <= addr <= hi for lo, hi in self.ranges)

    @classmethod
    def from_text(cls, text: str) -> "RamMap":
        ranges = []
        for line in text.splitlines():
            if "System RAM" not in line:
                continue
            span = line.split(":", 1)[0].strip()
            if "-" not in span:
                continue
            lo, _, hi = span.partition("-")
            try:
                ranges.append((int(lo, 16), int(hi, 16)))
            except ValueError:
                continue
        return cls(ranges) if ranges else cls(None, resolved=False)

    @classmethod
    def load(cls, path: str = IOMEM) -> "RamMap":
        try:
            with open(path, "r") as fp:
                return cls.from_text(fp.read())
        except OSError:
            return cls(None, resolved=False)


def read_page_flags(pfn: int, path: str = KPAGEFLAGS) -> int | None:
    """The flag word for one PFN from `/proc/kpageflags`. None if unreadable."""
    try:
        with open(path, "rb") as fp:
            fp.seek(pfn * 8)
            data = fp.read(8)
        if len(data) != 8:
            return None
        return struct.unpack("<Q", data)[0]
    except OSError:
        return None


def decode_flags(word: int) -> list[str]:
    return sorted(name for name, bit in KPF.items() if word & (1 << bit))


class Isolator:
    """Executor for page and core isolation.

    With `armed=False` (the default) nothing is written; only the verdict is returned.
    """

    def __init__(self, armed: bool = False, topology: HartTopology | None = None,
                 ram: RamMap | None = None, min_online_cpus: int = 2,
                 cpu_dir: str = CPU_DIR, corrupt_path: str = CORRUPT_PFN,
                 unpoison_path: str = UNPOISON_PFN, kpageflags: str = KPAGEFLAGS):
        self.armed = armed
        self.topology = topology if topology is not None else HartTopology.load()
        self.ram = ram if ram is not None else RamMap.load()
        self.min_online_cpus = min_online_cpus
        self.cpu_dir = cpu_dir
        self.corrupt_path = corrupt_path
        self.unpoison_path = unpoison_path
        self.kpageflags = kpageflags

    # -- pages --------------------------------------------------------------
    def inspect_page(self, pfn: int) -> dict:
        """Collect the evidence for the verdict; identical under dry-run."""
        info: dict = {"pfn": pfn, "physical_address": pfn * PAGE_SIZE}
        info["in_ram"] = self.ram.contains_pfn(pfn) if self.ram.resolved else None
        word = read_page_flags(pfn, self.kpageflags)
        if word is None:
            info["flags_readable"] = False
            return info
        info["flags_readable"] = True
        info["flags_word"] = word
        info["flags"] = decode_flags(word)
        return info

    def isolate_page(self, pfn: int, force: bool = False) -> IsolationResult:
        """Detach one PFN through hwpoison."""
        tgt = f"pfn=0x{pfn:x}"
        info = self.inspect_page(pfn)

        if pfn <= 0:
            return IsolationResult("page_isolate", tgt, "refused",
                                   "PFN 0 or below is not a valid target", detail=info)
        if not self.ram.resolved:
            return IsolationResult("page_isolate", tgt, "refused",
                                   "cannot read System RAM ranges, target unverifiable",
                                   detail=info)
        if not info["in_ram"]:
            return IsolationResult("page_isolate", tgt, "refused",
                                   "physical address outside System RAM", detail=info)
        if not info["flags_readable"]:
            # Refuse when unsure: it may be a kernel page, and that means a panic.
            return IsolationResult("page_isolate", tgt, "refused",
                                   "cannot read kpageflags, page kind unknown",
                                   detail=info)

        flags = info["flags"]
        done = [f for f in DONE_FLAGS if f in flags]
        if done:
            return IsolationResult("page_isolate", tgt, "noop",
                                   f"already isolated page ({','.join(done)})", detail=info)
        fatal = [f for f in FATAL_FLAGS if f in flags]
        if fatal:
            return IsolationResult(
                "page_isolate", tgt, "refused",
                f"kernel-structure page ({','.join(fatal)}) — memory_failure() would panic",
                reversible=False, detail=info)
        risky = [f for f in RISKY_FLAGS if f in flags]
        if risky and not force:
            return IsolationResult(
                "page_isolate", tgt, "refused",
                f"wide-blast-radius page ({','.join(risky)}) — force required", detail=info)

        info["risky"] = risky
        if not self.armed:
            return IsolationResult("page_isolate", tgt, "dry_run",
                                   "armed=False — nothing written", detail=info)
        try:
            with open(self.corrupt_path, "w") as fp:
                fp.write(f"0x{pfn:x}\n")
        except OSError as exc:
            return IsolationResult("page_isolate", tgt, "error", str(exc), detail=info)
        return IsolationResult("page_isolate", tgt, "applied",
                               "memory_failure() called", detail=info)

    def restore_page(self, pfn: int) -> IsolationResult:
        """`unpoison-pfn` — used to roll back after a test."""
        tgt = f"pfn=0x{pfn:x}"
        if not self.armed:
            return IsolationResult("page_restore", tgt, "dry_run", "armed=False")
        try:
            with open(self.unpoison_path, "w") as fp:
                fp.write(f"0x{pfn:x}\n")
        except OSError as exc:
            return IsolationResult("page_restore", tgt, "error", str(exc))
        return IsolationResult("page_restore", tgt, "applied")

    # -- cores --------------------------------------------------------------
    def online_cpus(self) -> list[int]:
        found = []
        try:
            names = os.listdir(self.cpu_dir)
        except OSError:
            return found
        for name in names:
            if not name.startswith("cpu") or not name[3:].isdigit():
                continue
            path = os.path.join(self.cpu_dir, name, "online")
            try:
                with open(path, "r") as fp:
                    if fp.read().strip() == "1":
                        found.append(int(name[3:]))
            except OSError:
                # A CPU with no online file cannot be offlined; treat it as always up.
                found.append(int(name[3:]))
        return sorted(found)

    def isolate_hart(self, hart: int) -> IsolationResult:
        """Take a hart id, **translate it to a Linux CPU number**, then offline it."""
        if not self.topology.resolved:
            return IsolationResult(
                "core_isolate", f"hart={hart}", "refused",
                "hart<->cpu mapping unresolved — guessing would offline the wrong core")
        cpu = self.topology.cpu_of_hart(hart)
        if cpu is None:
            return IsolationResult("core_isolate", f"hart={hart}", "refused",
                                   "hart absent from the mapping")
        return self.isolate_cpu(cpu, hart=hart)

    def isolate_cpu(self, cpu: int, hart: int | None = None) -> IsolationResult:
        tgt = f"cpu={cpu}" + (f" (hart{hart})" if hart is not None else "")
        info = {"linux_cpu": cpu, "hart": hart}
        online = self.online_cpus()
        info["online_before"] = online

        if cpu == 0:
            # cpu0 is the boot CPU. Even where the online file looks writable on
            # riscv, detaching the boot hart endangers the whole system, which is
            # the opposite of the availability goal.
            return IsolationResult("core_isolate", tgt, "refused",
                                   "boot CPU (cpu0) is never an isolation target",
                                   detail=info)
        if cpu not in online:
            return IsolationResult("core_isolate", tgt, "noop",
                                   "already offline", detail=info)
        if len(online) - 1 < self.min_online_cpus:
            return IsolationResult(
                "core_isolate", tgt, "refused",
                f"would leave fewer than {self.min_online_cpus} online CPUs",
                detail=info)

        path = os.path.join(self.cpu_dir, f"cpu{cpu}", "online")
        if not os.path.exists(path):
            return IsolationResult("core_isolate", tgt, "refused",
                                   "no online control file (CPU cannot be offlined)",
                                   detail=info)
        if not self.armed:
            return IsolationResult("core_isolate", tgt, "dry_run",
                                   "armed=False — nothing written", detail=info)
        try:
            with open(path, "w") as fp:
                fp.write("0\n")
        except OSError as exc:
            return IsolationResult("core_isolate", tgt, "error", str(exc), detail=info)
        info["online_after"] = self.online_cpus()
        return IsolationResult("core_isolate", tgt, "applied", detail=info)

    def restore_cpu(self, cpu: int) -> IsolationResult:
        tgt = f"cpu={cpu}"
        path = os.path.join(self.cpu_dir, f"cpu{cpu}", "online")
        if not self.armed:
            return IsolationResult("core_restore", tgt, "dry_run", "armed=False")
        try:
            with open(path, "w") as fp:
                fp.write("1\n")
        except OSError as exc:
            return IsolationResult("core_restore", tgt, "error", str(exc))
        return IsolationResult("core_restore", tgt, "applied")

    # -- Fault integration --------------------------------------------------
    def respond(self, fault, force: bool = False) -> list[IsolationResult]:
        """Look at a classified :class:`~.fault.Fault` and attempt the right isolation.

        The `pfn` and `hart` that `classify_ghes` already placed in `detail` are used
        directly; `raw_log` is never re-scraped with a regex here.
        """
        results: list[IsolationResult] = []
        detail = getattr(fault, "detail", {}) or {}

        if not getattr(fault, "recoverable", True):
            results.append(IsolationResult(
                "policy", "fault", "refused",
                "recoverable=False — not a fault isolation can respond to"))
            return results

        pfn = detail.get("pfn")
        if isinstance(pfn, int):
            results.append(self.isolate_page(pfn, force=force))

        hart = detail.get("hart")
        if hart is not None and detail.get("isolation_target") == "core":
            try:
                results.append(self.isolate_hart(int(hart)))
            except (TypeError, ValueError):
                results.append(IsolationResult("core_isolate", f"hart={hart}",
                                               "error", "could not parse hart value"))
        return results
