# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Kernel log -> fault classification.

Two paths:

1. :func:`classify_ghes` turns a reassembled GHES event into a `RERI_FAULT`. Severity
   is **derived** from the `event severity` the kernel reported
   (corrected/recoverable/fatal). The ETRI table marks RERI_FAULT uniformly as
   recoverable=x, but the real RERI hardware emulation distinguishes the three, so that
   value is kept — TC.04.012 ("severity-based policy response") requires exactly this
   distinction.

2. :func:`classify_line` classifies other kernel messages through a rule table. It is a
   **whitelist** by default: catching boot-time noise as well would make the log
   useless as evidence.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from .fault import Fault
from .ghes import GhesEvent
from .kmsg import KmsgRecord
from .topology import HartTopology


@dataclass(frozen=True)
class Rule:
    pattern: re.Pattern
    fault_type: str
    severity: str
    source_layer: str
    recoverable: bool
    tags: tuple[str, ...]


def _r(pattern: str, fault_type: str, severity: str, source_layer: str,
       recoverable: bool, *tags: str) -> Rule:
    return Rule(re.compile(pattern), fault_type, severity, source_layer, recoverable, tags)


#: The first rule that matches wins, so the more severe rules come first.
RULES: tuple[Rule, ...] = (
    # -- Injection marker (tools/inject_reri.sh writes it to /dev/kmsg before injecting)
    #    Kept for cross-checking the hart the kernel reports; see classify_ghes.
    _r(r"\bRASINJ\b", "INJECTION_MARKER", "INFO", "test", True, "injection", "marker"),

    # -- SYSTEM_FAULT: kernel-level failure -------------------------------------
    _r(r"Kernel panic|Oops:|BUG: unable to handle|Unable to handle kernel",
       "SYSTEM_FAULT", "CRITICAL", "kernel", False, "panic"),
    _r(r"Out of memory: Kill(ed)? process|oom-kill|Out of memory and no killable",
       "SYSTEM_FAULT", "CRITICAL", "kernel", False, "oom"),
    _r(r"watchdog: BUG: soft lockup|watchdog: BUG: hard lockup|rcu:? .*stall",
       "SYSTEM_FAULT", "CRITICAL", "kernel", False, "lockup"),

    # -- SYSTEM_DEGRADED: partial loss / degradation ----------------------------
    #    Evidence for TC.04.008 (fault isolation) and TC.04.013 (CPU core isolation):
    #    to confirm "that core is offline and the rest are healthy" mechanically from
    #    the log, core on/offline messages must be recorded with tags.
    _r(r"CPU(\d+): (off|shutdown)|Offlined CPU (\d+)|CPU (\d+) is now offline|"
       r"psci: CPU(\d+) killed|smpboot: CPU (\d+) is now offline",
       "SYSTEM_DEGRADED", "WARN", "kernel", True, "cpu_hotplug", "isolation", "cpu_offline"),
    _r(r"Booting CPU (\d+)|CPU(\d+): Booted|smpboot: Booting Node",
       "SYSTEM_DEGRADED", "INFO", "kernel", True, "cpu_hotplug", "cpu_online"),
    _r(r"Memory failure:|hardware memory corruption|soft offline|HardwareCorrupted",
       "SYSTEM_DEGRADED", "ERROR", "kernel", True, "memory_failure"),
    _r(r"EDAC .*(CE|UE)|edac_mc.*error", "SYSTEM_DEGRADED", "ERROR", "kernel", True, "edac"),
    _r(r"hung task|blocked for more than \d+ seconds",
       "SYSTEM_DEGRADED", "WARN", "kernel", True, "hung_task"),
    _r(r"page allocation failure|slab out of memory",
       "SYSTEM_DEGRADED", "WARN", "kernel", True, "alloc_fail"),

    # -- DRIVER_FAULT ------------------------------------------------------------
    _r(r"Buffer I/O error|blk_update_request: I/O error|EXT4-fs error|"
       r"virtio\w*: .*(error|fail)|nvme.*: .*(error|reset)|"
       r"\b(usb|scsi|virtio_blk|virtio_net)\b.*(error|failed|timeout)",
       "DRIVER_FAULT", "ERROR", "driver", True, "driver", "io"),
)

#: RERI error type -> tag, for search and filtering
_ERROR_TYPE_TAGS = {
    0x01: "cache",
    0x02: "tlb",
    0x04: "bus",
    0x08: "micro_arch",
}

_CPU_NUM_RE = re.compile(r"CPU\s*#?(\d+)|cpu(\d+)")


def _parse_int(value) -> int | None:
    """`'0x0000000000000004'` or `4` -> 4. None if it cannot be parsed."""
    if value is None:
        return None
    if isinstance(value, int):
        return value
    try:
        return int(str(value).strip(), 0)
    except (TypeError, ValueError):
        return None


#: Base for mapping a GHES error source id to a hardware unit.
#: QEMU virt assigns `base-source-id = 1` to the hart banks (`create_fdt_reri_harts`)
#: and OpenSBI hands them out in hart order via
#: `reri_hart_devices[i].dev.src_id = src_id++`.
#: -> **source id of hart i = 1 + i** (measured: hart0 -> 1, hart3 -> 4)
#: DRAM banks are meant to take `2 + smp.cpus`, but QEMU never creates the device so
#: they never appear (only riscv.reri.hart exists in `info mtree`).
GHES_SOURCE_BASE = 1


def classify_ghes(event: GhesEvent, node_name: str = "kernel_monitor",
                  ghes_source_base: int = GHES_SOURCE_BASE,
                  topology: HartTopology | None = None) -> Fault:
    """A reassembled GHES event -> `RERI_FAULT`.

    The hart is taken from values the kernel reported, not guessed from the injection
    marker; the marker is only used to cross-check. See the evidence ordering below.
    """
    detail = dict(event.fields)
    if event.notes:
        detail["notes"] = list(event.notes)
    detail["ghes_seq"] = event.seq_id
    detail["line_count"] = event.line_count
    detail["error_type_name"] = event.error_type_name

    tags = ["reri", "hardware_error", f"ghes:{event.seq_id}"]
    code = event.error_type
    if event.section_type:
        detail["section_type"] = event.section_type
        tags.append("section:memory" if event.is_memory_section else "section:processor")
    if code is not None:
        if event.is_memory_section:
            # The memory section is a decimal enum; tagging it as hex breaks dmesg matching.
            tags.append(f"mem_error_type:{code}")
            tags.append(f"mem_error:{event.error_type_name.replace(' ', '_')}")
        else:
            tags.append(f"error_type:0x{code:02x}")
            for bit, tag in _ERROR_TYPE_TAGS.items():
                if code & bit:
                    tags.append(tag)
    if event.event_severity != "unknown":
        tags.append(f"severity:{event.event_severity}")

    # -- Page isolation target (memory section) -------------------------------
    # The PFN goes into detail so the isolation path can hand it straight to
    # hwpoison (`corrupt-pfn`) instead of re-scraping raw_log with a regex.
    pa = event.physical_address
    if pa is not None:
        detail["physical_address_int"] = pa
        detail["pfn"] = event.pfn()
        detail["isolation_target"] = "page"
        tags.append("page_isolatable")

    # -- Which hardware unit it came from (values the kernel reported) ---------
    # First-class evidence is `processor_id`: the RAS agent core-identification patch
    # writes the hart number into the CPER directly, so nothing has to be derived.
    # `ghes_source - base` is second-class — it depends on the source-id assignment
    # rule — and is kept for cross-checking. A disagreement is surfaced rather than
    # silently overwritten, because the disagreement itself is a finding.
    proc_hart = _parse_int(event.fields.get("processor_id"))
    source = event.fields.get("ghes_source")
    src_hart = (source - ghes_source_base
                if isinstance(source, int) and source - ghes_source_base >= 0 else None)

    # Third source: `caller=C<n>` from kmsg (CONFIG_PRINTK_CALLER). The memory section
    # carries no processor_id, so this is its only independent cross-check.
    # `C<n>` is a **Linux CPU number**, so using it as a hart requires the topology.
    caller_cpu = event.records[0].caller_cpu if event.records else None
    caller_hart = topology.hart_of_cpu(caller_cpu) if (
        topology is not None and topology.resolved and caller_cpu is not None) else None

    hart = proc_hart
    hart_src = "processor_id"
    if hart is None:
        hart, hart_src = src_hart, "ghes_source"
    if hart is None:
        hart, hart_src = caller_hart, "printk_caller"

    if hart is not None:
        detail["hart"] = str(hart)
        detail["hart_source"] = hart_src
        detail["hw_unit"] = f"hart{hart}"
        detail["isolation_target"] = detail.get("isolation_target", "core")
        tags.append(f"hart:{hart}")
        tags.append(f"unit:hart{hart}")

        # The Linux CPU number the isolation path needs. Offlining the hart number
        # directly would take out the wrong core (the boot hart takes cpu0 and shifts
        # the rest), so it is translated here. If translation is impossible the field
        # is left out — absent is safer than wrong.
        if topology is not None and topology.resolved:
            cpu = topology.cpu_of_hart(hart)
            if cpu is not None:
                detail["linux_cpu"] = cpu
                tags.append(f"cpu:{cpu}")
            detail["topology_identity"] = topology.is_identity()
        else:
            detail["linux_cpu_unresolved"] = True
            tags.append("topology_unresolved")

        # Cross-check the sources; a disagreement is surfaced, not overwritten.
        corroborate = {}
        if src_hart is not None:
            corroborate["ghes_source"] = src_hart
        if caller_hart is not None:
            corroborate["printk_caller"] = caller_hart
        if caller_cpu is not None:
            detail["caller_cpu"] = caller_cpu
        if corroborate:
            detail["hart_corroboration"] = corroborate
            disagree = {k: v for k, v in corroborate.items() if v != hart}
            if disagree:
                detail["hart_mismatch"] = {hart_src: hart, **disagree}
                tags.append("hart_mismatch")

    description = event.summary()

    return Fault(
        fault_type="RERI_FAULT",
        severity=event.severity,
        description=description,
        source_layer="hardware",
        recoverable=event.recoverable,
        raw_log=event.raw,
        tags=tags,
        node_name=node_name,
        timestamp=event.wall_time,
        detail=detail,
        kmsg={
            "seq": event.records[0].seq if event.records else -1,
            "ts_monotonic": round(event.first_ts, 6),
            "ts_monotonic_last": round(event.last_ts, 6),
            "level": event.records[0].level if event.records else -1,
            "level_name": event.records[0].level_name if event.records else "",
        },
    )


def classify_line(record: KmsgRecord, node_name: str = "kernel_monitor",
                  topology: HartTopology | None = None) -> Fault | None:
    """One ordinary kernel message line -> Fault, or None if no rule matches."""
    for rule in RULES:
        m = rule.pattern.search(record.text)
        if not m:
            continue

        tags = list(rule.tags)
        detail: dict = {}

        # Promote the core number to a tag so "which core was isolated" is one grep away
        if "cpu_hotplug" in tags:
            cpu = next((g for g in m.groups() if g and g.isdigit()), None)
            if cpu is None:
                cm = _CPU_NUM_RE.search(record.text)
                cpu = next((g for g in cm.groups() if g), None) if cm else None
            if cpu is not None:
                tags.append(f"cpu:{cpu}")
                detail["cpu"] = int(cpu)
                # The kernel prints a **Linux CPU number** (`CPU7: off`) while RERI
                # faults are reported by hart. Recording the hart here is what lets
                # "hart6 faulted -> that core was isolated" join up mechanically in the
                # log. The numbers are shifted (hart6 = cpu7), so without translation
                # the correlation cannot be claimed at all.
                if topology is not None and topology.resolved:
                    hart = topology.hart_of_cpu(int(cpu))
                    if hart is not None:
                        detail["hart"] = str(hart)
                        detail["hart_source"] = "topology_from_cpu"
                        tags.append(f"hart:{hart}")
                        tags.append(f"unit:hart{hart}")

        # Copy the marker key=value pairs into detail (hart=3 base=0x4013000 code=...)
        if rule.fault_type == "INJECTION_MARKER":
            for token in record.text.split():
                if "=" in token:
                    key, _, value = token.partition("=")
                    detail[key] = value
            if "hart" in detail:
                tags.append(f"hart:{detail['hart']}")

        return Fault(
            fault_type=rule.fault_type,
            severity=rule.severity,
            description=record.text,
            source_layer=rule.source_layer,
            recoverable=rule.recoverable,
            raw_log=record.raw,
            tags=tags,
            node_name=node_name,
            timestamp=record.wall_time,
            detail=detail,
            kmsg={
                "seq": record.seq,
                "ts_monotonic": round(record.ts_monotonic, 6),
                "level": record.level,
                "level_name": record.level_name,
            },
        )
    return None
