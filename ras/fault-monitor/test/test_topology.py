# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""hart <-> Linux CPU mapping and the `caller=C<n>` correlation.

The expected values are measured on the q-vela VM: with `-smp 8` the boot hart was 7, so

    Linux CPU  0  1  2  3  4  5  6  7
    hart       7  0  1  2  3  4  5  6

and when SMD was injected on hart 6, the caller in the raw record the kernel actually
printed was `C7`.
"""

from ras_fault_monitor.classify import classify_ghes
from ras_fault_monitor.ghes import GhesAssembler
from ras_fault_monitor.kmsg import parse_record
from ras_fault_monitor.topology import HartTopology

# reproduces the measured shape — a long `hart isa:` line sits between the fields.
CPUINFO = "".join(
    "processor\t: %d\nhart\t\t: %d\nhart isa\t: rv64imafdch_zicbom\n\n" % (cpu, hart)
    for cpu, hart in enumerate([7, 0, 1, 2, 3, 4, 5, 6])
)

#: a record copied verbatim from the guest `/dev/kmsg` on a hart 6 injection.
RAW_MEM_HART6 = [
    "3,408,612003519,-,caller=C7;{1}[Hardware Error]: Hardware error from APEI Generic Hardware Error Source: 7",
    "3,409,612005148,-,caller=C7;{1}[Hardware Error]: event severity: recoverable",
    "3,410,612006513,-,caller=C7;{1}[Hardware Error]:  Error 0, type: recoverable",
    "3,411,612008163,-,caller=C7;{1}[Hardware Error]:   section_type: memory error",
    "3,413,612011077,-,caller=C7;{1}[Hardware Error]:   physical_address: 0x0000000090000000",
    "3,415,612014567,-,caller=C7;{1}[Hardware Error]:   error_type: 3, multi-bit ECC",
]


def _assemble(raw_lines):
    recs = [parse_record(r, boot_time=1_800_000_000.0) for r in raw_lines]
    assert all(recs), "raw record failed to parse"
    asm = GhesAssembler(timeout=0.5)
    events = []
    for r in recs:
        events.extend(asm.feed(r))
    events.extend(asm.flush())
    assert len(events) == 1
    return events[0]


# --- the mapping itself -----------------------------------------------------

def test_boot_hart_takes_cpu0_so_numbering_shifts():
    topo = HartTopology.from_text(CPUINFO)
    assert topo.resolved
    assert topo.cpu_of_hart(7) == 0        # the boot hart
    assert topo.cpu_of_hart(0) == 1
    assert topo.cpu_of_hart(6) == 7
    assert topo.hart_of_cpu(7) == 6


def test_numbering_is_not_identity():
    """Non-identity has to surface — assuming they match offlines the wrong core."""
    assert HartTopology.from_text(CPUINFO).is_identity() is False


def test_identity_case_is_detected():
    text = "".join("processor\t: %d\nhart\t\t: %d\n\n" % (n, n) for n in range(4))
    topo = HartTopology.from_text(text)
    assert topo.resolved and topo.is_identity() is True


def test_unparsable_cpuinfo_is_not_silently_identity():
    """Passing a failed read off as identity makes isolation silently wrong."""
    topo = HartTopology.from_text("garbage without fields")
    assert topo.resolved is False
    assert topo.cpu_of_hart(6) is None


def test_mismatched_field_counts_rejected():
    topo = HartTopology.from_text("processor\t: 0\nprocessor\t: 1\nhart\t\t: 7\n")
    assert topo.resolved is False


# --- caller field parsing ---------------------------------------------------

def test_caller_cpu_parsed_from_extra_field():
    rec = parse_record(RAW_MEM_HART6[0], boot_time=0.0)
    assert rec.caller == "C7"
    assert rec.caller_cpu == 7
    assert rec.caller_pid is None


def test_caller_task_form_is_not_a_cpu():
    rec = parse_record("6,0,0,-,caller=T1234;hello", boot_time=0.0)
    assert rec.caller_pid == 1234
    assert rec.caller_cpu is None          # T is a pid; it must not read as a CPU


def test_record_without_caller_still_parses():
    """Compatibility with a kernel without PRINTK_CALLER — a missing field must not
    break it."""
    rec = parse_record("6,0,0,-;hello", boot_time=0.0)
    assert rec is not None and rec.caller == "" and rec.caller_cpu is None


# --- correlation and isolation target ---------------------------------------

def test_caller_corroborates_hart_for_memory_section():
    """A memory section has no processor_id — caller is the only independent
    cross-check."""
    fault = classify_ghes(_assemble(RAW_MEM_HART6),
                          topology=HartTopology.from_text(CPUINFO))
    assert fault.detail["hart"] == "6"
    assert fault.detail["caller_cpu"] == 7
    assert fault.detail["hart_corroboration"] == {"ghes_source": 6, "printk_caller": 6}
    assert "hart_mismatch" not in fault.detail


def test_isolation_targets_translated_cpu_not_hart_number():
    """hart 6 -> cpu 7. Offlining cpu 6 would kill hart 5."""
    fault = classify_ghes(_assemble(RAW_MEM_HART6),
                          topology=HartTopology.from_text(CPUINFO))
    assert fault.detail["linux_cpu"] == 7
    assert "cpu:7" in fault.tags
    assert fault.detail["topology_identity"] is False


def test_no_topology_means_no_cpu_guess():
    """With no translation, no value is invented — absent is safer than wrong."""
    fault = classify_ghes(_assemble(RAW_MEM_HART6), topology=None)
    assert "linux_cpu" not in fault.detail
    assert fault.detail["linux_cpu_unresolved"] is True
    assert "topology_unresolved" in fault.tags


# --- isolation event <-> RERI fault correlation -----------------------------

def test_cpu_offline_line_records_hart_for_correlation():
    """The kernel prints `CPU7: off`; linking that to a hart6 fault needs the
    translation."""
    from ras_fault_monitor.classify import classify_line
    rec = parse_record("4,999,1320173091,-,caller=C0;CPU7: off", boot_time=1.8e9)
    fault = classify_line(rec, topology=HartTopology.from_text(CPUINFO))
    assert fault.fault_type == "SYSTEM_DEGRADED"
    assert fault.detail["cpu"] == 7
    assert fault.detail["hart"] == "6"                 # cpu7 == hart6
    assert fault.detail["hart_source"] == "topology_from_cpu"
    assert "hart:6" in fault.tags and "cpu:7" in fault.tags


def test_cpu_offline_without_topology_claims_no_hart():
    """With no translation no hart is claimed — a wrong correlation is worse than none."""
    from ras_fault_monitor.classify import classify_line
    rec = parse_record("4,999,1320173091,-,caller=C0;CPU7: off", boot_time=1.8e9)
    fault = classify_line(rec, topology=None)
    assert fault.detail["cpu"] == 7
    assert "hart" not in fault.detail
    assert not any(t.startswith("hart:") for t in fault.tags)
