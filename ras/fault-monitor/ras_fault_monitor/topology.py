# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""hart id to Linux CPU number mapping.

**These are not the same number.** Measured on q-vela (smp 8), two boots of the
same image produced two different mappings:

    boot hart 7:  Linux CPU  0  1  2  3  4  5  6  7
                  hart       7  0  1  2  3  4  5  6

    boot hart 0:  Linux CPU  0  1  2  3  4  5  6  7
                  hart       0  1  2  3  4  5  6  7      (identity)

The boot hart takes CPU 0 and pushes the rest along. Which hart boots depends on
`-smp` and the firmware, so the mapping **must not be hardcoded** — it is read at
runtime. A table baked in from the first boot would have offlined the wrong core
on the second.

Why it matters
--------------
RERI/GHES reports **hart ids** (`processor_id`, error-bank address, source id) but
CPU isolation takes **Linux CPU numbers** (`/sys/devices/system/cpu/cpuN/online`).
Using one where the other belongs **offlines the wrong core**: for "hart 6 faulted",
offlining cpu6 kills hart 5 while the faulty hart 6 keeps running — and the log
still says the isolation succeeded. It is the kind of bug that is silently wrong.
"""

from __future__ import annotations

import re

CPUINFO_PATH = "/proc/cpuinfo"

_PROCESSOR_RE = re.compile(r"^processor\s*:\s*(\d+)", re.M)
_HART_RE = re.compile(r"^hart\s*:\s*(\d+)", re.M)


class HartTopology:
    """Bidirectional hart <-> Linux CPU mapping.

    When the mapping cannot be read it **does not silently fall back to identity** —
    that would let isolation offline the wrong core while appearing to succeed.
    `resolved` carries whether the mapping can be trusted.
    """

    def __init__(self, hart_to_cpu: dict[int, int] | None = None,
                 resolved: bool = True):
        self.hart_to_cpu: dict[int, int] = dict(hart_to_cpu or {})
        self.cpu_to_hart: dict[int, int] = {c: h for h, c in self.hart_to_cpu.items()}
        self.resolved = resolved and bool(self.hart_to_cpu)

    # -- lookup -------------------------------------------------------------
    def cpu_of_hart(self, hart: int) -> int | None:
        return self.hart_to_cpu.get(hart)

    def hart_of_cpu(self, cpu: int) -> int | None:
        return self.cpu_to_hart.get(cpu)

    def is_identity(self) -> bool:
        """Whether the numbers happen to coincide on this boot.

        Recorded in the log so a later reader is not confused by a boot where
        hart N really did equal cpu N.
        """
        return all(h == c for h, c in self.hart_to_cpu.items())

    def as_dict(self) -> dict:
        return {
            "resolved": self.resolved,
            "identity": self.is_identity() if self.resolved else None,
            "hart_to_cpu": {str(h): c for h, c in sorted(self.hart_to_cpu.items())},
        }

    # -- construction -------------------------------------------------------
    @classmethod
    def from_text(cls, text: str) -> "HartTopology":
        """Parse `/proc/cpuinfo` contents.

        On riscv each block carries `processor: N` followed by `hart: M`. Block
        boundaries are not trusted; the two are **paired by order of appearance**,
        because long lines such as `hart isa:` sit between them (measured) while
        the ordering itself holds.
        """
        cpus = [int(m) for m in _PROCESSOR_RE.findall(text)]
        harts = [int(m) for m in _HART_RE.findall(text)]
        if not cpus or len(cpus) != len(harts):
            return cls(None, resolved=False)
        return cls({h: c for c, h in zip(cpus, harts)})

    @classmethod
    def load(cls, path: str = CPUINFO_PATH) -> "HartTopology":
        try:
            with open(path, "r") as fp:
                return cls.from_text(fp.read())
        except OSError:
            return cls(None, resolved=False)
