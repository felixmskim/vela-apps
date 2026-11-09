# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Safety guards in the isolation path.

A bug here has two outcomes: a **VM panic** (memory_failure on a kernel page) or
**isolating the wrong core** (mistaking a hart number for a cpu number). Both happen
silently, so every guard gets a test.

The RAM ranges and the hart<->cpu mapping are values measured on the q-vela VM.
"""

import os
import struct

import pytest

from ras_fault_monitor.isolate import (
    Isolator, IsolationResult, KPF, RamMap, decode_flags,
)
from ras_fault_monitor.topology import HartTopology

# measured from the guest /proc/iomem
IOMEM_TEXT = """\
80080000-bfdfffff : System RAM
bfe80000-107fe3cfff : System RAM
107ffff000-107fffffff : System RAM
10000000-10000007 : serial
"""

CPUINFO = "".join(
    "processor\t: %d\nhart\t\t: %d\n\n" % (cpu, hart)
    for cpu, hart in enumerate([7, 0, 1, 2, 3, 4, 5, 6])
)

SAFE_PFN = 0x90000          # 0x90000000 — inside the first RAM range (injection test address)
OUTSIDE_PFN = 0x10000       # 0x10000000 — serial MMIO, not RAM


def _flags(*names) -> int:
    word = 0
    for n in names:
        word |= 1 << KPF[n]
    return word


def _make(tmp_path, pfn_flags: dict[int, int] | None = None, armed=False,
          ram=None, topo=None, online=(0, 1, 2, 3, 4, 5, 6, 7), min_online=2):
    """Lay down a fake sysfs/procfs and build an Isolator."""
    kpf = tmp_path / "kpageflags"
    if pfn_flags is None:
        kpf.write_bytes(b"")
    else:
        top = max(pfn_flags) + 1
        buf = bytearray(8 * top)
        for pfn, word in pfn_flags.items():
            struct.pack_into("<Q", buf, pfn * 8, word)
        kpf.write_bytes(bytes(buf))

    cpu_dir = tmp_path / "cpu"
    for n in range(8):
        d = cpu_dir / f"cpu{n}"
        d.mkdir(parents=True)
        (d / "online").write_text("1\n" if n in online else "0\n")

    return Isolator(
        armed=armed,
        topology=topo if topo is not None else HartTopology.from_text(CPUINFO),
        ram=ram if ram is not None else RamMap.from_text(IOMEM_TEXT),
        min_online_cpus=min_online,
        cpu_dir=str(cpu_dir),
        corrupt_path=str(tmp_path / "corrupt-pfn"),
        unpoison_path=str(tmp_path / "unpoison-pfn"),
        kpageflags=str(kpf),
    )


# --- RAM ranges -------------------------------------------------------------

def test_ram_map_parses_only_system_ram():
    ram = RamMap.from_text(IOMEM_TEXT)
    assert ram.resolved
    assert ram.contains_pfn(SAFE_PFN) is True
    assert ram.contains_pfn(OUTSIDE_PFN) is False       # serial MMIO


def test_unparsable_iomem_is_not_treated_as_all_ram():
    ram = RamMap.from_text("no ram here")
    assert ram.resolved is False


# --- page isolation guards --------------------------------------------------

def test_refuses_pfn_outside_system_ram(tmp_path):
    iso = _make(tmp_path, {OUTSIDE_PFN: _flags("ANON", "LRU")})
    r = iso.isolate_page(OUTSIDE_PFN)
    assert r.outcome == "refused" and "System RAM" in r.reason


def test_refuses_slab_page_because_memory_failure_panics(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("SLAB")}, armed=True)
    r = iso.isolate_page(SAFE_PFN)
    assert r.outcome == "refused" and "panic" in r.reason
    assert not os.path.exists(iso.corrupt_path)          # nothing was written


def test_fatal_flags_not_bypassable_by_force(tmp_path):
    """`force` must not override these — destroying the VM is not a response."""
    for flag in ("SLAB", "PGTABLE", "RESERVED", "NOPAGE"):
        sub = tmp_path / flag
        sub.mkdir()
        iso = _make(sub, {SAFE_PFN: _flags(flag)}, armed=True)
        r = iso.isolate_page(SAFE_PFN, force=True)
        assert r.outcome == "refused", f"{flag} was overridden by force"


def test_refuses_when_page_flags_unreadable(tmp_path):
    """Refuse when unsure: it may be a kernel page."""
    iso = _make(tmp_path, None, armed=True)             # empty kpageflags
    r = iso.isolate_page(SAFE_PFN)
    assert r.outcome == "refused" and "kpageflags" in r.reason


def test_already_poisoned_is_noop(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("HWPOISON")}, armed=True)
    r = iso.isolate_page(SAFE_PFN)
    assert r.outcome == "noop"
    assert not os.path.exists(iso.corrupt_path)


def test_risky_page_needs_force(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("THP", "ANON")}, armed=True)
    assert iso.isolate_page(SAFE_PFN).outcome == "refused"
    assert iso.isolate_page(SAFE_PFN, force=True).outcome == "applied"


def test_dry_run_writes_nothing(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("ANON", "LRU")}, armed=False)
    r = iso.isolate_page(SAFE_PFN)
    assert r.outcome == "dry_run"
    assert not os.path.exists(iso.corrupt_path)


def test_armed_writes_pfn_in_hex(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("ANON", "LRU")}, armed=True)
    r = iso.isolate_page(SAFE_PFN)
    assert r.outcome == "applied"
    assert open(iso.corrupt_path).read().strip() == hex(SAFE_PFN)


def test_free_buddy_page_is_allowed(tmp_path):
    """An unallocated page is the safest possible isolation target."""
    iso = _make(tmp_path, {SAFE_PFN: _flags("BUDDY")}, armed=True)
    assert iso.isolate_page(SAFE_PFN).outcome == "applied"


# --- core isolation guards --------------------------------------------------

def test_hart_is_translated_to_linux_cpu(tmp_path):
    """hart6 -> cpu7. Offlining cpu6 would kill hart5."""
    iso = _make(tmp_path, armed=True)
    r = iso.isolate_hart(6)
    assert r.outcome == "applied"
    assert r.detail["linux_cpu"] == 7
    assert open(tmp_path / "cpu" / "cpu7" / "online").read().strip() == "0"
    assert open(tmp_path / "cpu" / "cpu6" / "online").read().strip() == "1"


def test_boot_cpu_is_never_isolated(tmp_path):
    """hart7 is cpu0, the boot CPU — it must not be detached."""
    iso = _make(tmp_path, armed=True)
    r = iso.isolate_hart(7)
    assert r.outcome == "refused" and "cpu0" in r.reason
    assert open(tmp_path / "cpu" / "cpu0" / "online").read().strip() == "1"


def test_refuses_when_topology_unresolved(tmp_path):
    """With no mapping, the number is never guessed."""
    iso = _make(tmp_path, armed=True,
                topo=HartTopology.from_text("garbage"))
    r = iso.isolate_hart(6)
    assert r.outcome == "refused" and "unresolved" in r.reason


def test_keeps_minimum_online_cpus(tmp_path):
    iso = _make(tmp_path, armed=True, online=(0, 7), min_online=2)
    r = iso.isolate_hart(6)                              # cpu7
    assert r.outcome == "refused" and "fewer than" in r.reason


def test_already_offline_is_noop(tmp_path):
    iso = _make(tmp_path, armed=True, online=(0, 1, 2, 3, 4, 5, 6))
    r = iso.isolate_hart(6)                              # cpu7 already off
    assert r.outcome == "noop"


def test_core_dry_run_writes_nothing(tmp_path):
    iso = _make(tmp_path, armed=False)
    r = iso.isolate_hart(6)
    assert r.outcome == "dry_run"
    assert open(tmp_path / "cpu" / "cpu7" / "online").read().strip() == "1"


# --- Fault integration ------------------------------------------------------

class _Fault:
    def __init__(self, detail, recoverable=True):
        self.detail = detail
        self.recoverable = recoverable


def test_respond_uses_precomputed_pfn_and_hart(tmp_path):
    """The detail values are used as-is; raw_log is never re-parsed."""
    iso = _make(tmp_path, {SAFE_PFN: _flags("ANON", "LRU")}, armed=False)
    results = iso.respond(_Fault({"pfn": SAFE_PFN, "hart": "6",
                                  "isolation_target": "core"}))
    actions = {r.action: r for r in results}
    assert actions["page_isolate"].outcome == "dry_run"
    assert actions["core_isolate"].detail["linux_cpu"] == 7


def test_respond_skips_unrecoverable_faults(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("ANON")}, armed=True)
    results = iso.respond(_Fault({"pfn": SAFE_PFN}, recoverable=False))
    assert [r.outcome for r in results] == ["refused"]
    assert not os.path.exists(iso.corrupt_path)


def test_page_only_fault_does_not_offline_a_core(tmp_path):
    iso = _make(tmp_path, {SAFE_PFN: _flags("ANON", "LRU")}, armed=True)
    results = iso.respond(_Fault({"pfn": SAFE_PFN, "hart": "6",
                                  "isolation_target": "page"}))
    assert [r.action for r in results] == ["page_isolate"]


def test_decode_flags_roundtrip():
    assert decode_flags(_flags("ANON", "LRU")) == ["ANON", "LRU"]
