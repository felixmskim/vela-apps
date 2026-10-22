# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""memory-section GHES parsing — a path that only appears after the RAS agent patch.

Every expected value here was **measured on the q-vela VM** (devmem injection on harts
5-7, confirmed in dmesg as `error_type: 3 / 2 / 14`). None of it was copied from a table.

The regression risk: a processor section carries `error_type: 0x08` (a hex bitflag) while
a memory section carries `error_type: 3, multi-bit ECC` (a decimal enum). Handling both
with one `int(x, 0)` silently turns the memory side into None and loses exactly what the
firmware reported.
"""

from ras_fault_monitor.classify import classify_ghes
from ras_fault_monitor.ghes import GhesAssembler
from ras_fault_monitor.kmsg import KmsgRecord


def _event(lines, block_id=1, seq0=100):
    """The `{N}[Hardware Error]:` body lines -> one reassembled GhesEvent."""
    asm = GhesAssembler(timeout=0.5)
    events = []
    for i, body in enumerate(lines):
        text = "{%d}[Hardware Error]: %s" % (block_id, body)
        ts_usec = int((900.0 + i * 0.001) * 1e6)
        rec = KmsgRecord(
            priority=2, seq=seq0 + i, ts_usec=ts_usec, flags="-",
            text=text, raw="2,%d,%d,-;%s" % (seq0 + i, ts_usec, text),
            wall_time=1_800_000_000.0 + i * 0.001,
        )
        events.extend(asm.feed(rec))
    events.extend(asm.flush())
    assert len(events) == 1, f"expected 1 event, got {len(events)}"
    return events[0]


def _mem_lines(source, error_type_line):
    return [
        f"Hardware error from APEI Generic Hardware Error Source: {source}",
        "event severity: recoverable",
        " Error 0, type: recoverable",
        "  section_type: memory error",
        "   error_status: Reserved (0x0000000000000000)",
        "  physical_address: 0x0000000090000000",
        "  physical_address_mask: 0xffffffffffffffff",
        f"  error_type: {error_type_line}",
    ]


PROC_LINES = [
    "Hardware error from APEI Generic Hardware Error Source: 5",
    "event severity: recoverable",
    " Error 0, type: recoverable",
    "  section_type: general processor error",
    "  processor_type: 3, RISCV",
    "  processor_isa: 6, RISCV64",
    "  error_type: 0x08",
    "  micro-architectural error",
    "  operation: 1, data read",
    "  processor_id: 0x0000000000000004",
    "  target_address: 0x0000000090000000",
]


# --- memory section: mapping per ec (3 measured) ----------------------------

def test_smd_uncorrectable_is_multi_bit_ecc():
    ev = _event(_mem_lines(6, "3, multi-bit ECC"), block_id=2)
    assert ev.is_memory_section
    assert ev.error_type == 3
    assert ev.error_type_name == "multi-bit ECC"


def test_smd_correctable_is_single_bit_ecc():
    ev = _event(_mem_lines(8, "2, single-bit ECC"), block_id=4)
    assert ev.error_type == 2
    assert ev.error_type_name == "single-bit ECC"


def test_sms_is_scrub_uncorrected():
    ev = _event(_mem_lines(7, "14, scrub uncorrected error"), block_id=3)
    assert ev.error_type == 14
    assert ev.error_type_name == "scrub uncorrected error"


# --- page isolation input ---------------------------------------------------

def test_physical_address_becomes_pfn():
    ev = _event(_mem_lines(6, "3, multi-bit ECC"), block_id=2)
    assert ev.physical_address == 0x90000000
    assert ev.pfn() == 0x90000                      # 0x90000000 >> 12

    fault = classify_ghes(ev)
    assert fault.detail["pfn"] == 0x90000
    assert fault.detail["isolation_target"] == "page"
    assert "page_isolatable" in fault.tags


def test_memory_tags_use_decimal_not_hex():
    """It has to be comparable against dmesg — a decimal enum must not be tagged hex."""
    fault = classify_ghes(_event(_mem_lines(6, "3, multi-bit ECC"), block_id=2))
    assert "mem_error_type:3" in fault.tags
    assert "section:memory" in fault.tags
    assert not any(t.startswith("error_type:0x") for t in fault.tags)


# --- processor section, no regression ---------------------------------------

def test_processor_section_still_hex_bitflag():
    ev = _event(PROC_LINES)
    assert not ev.is_memory_section
    assert ev.error_type == 0x08
    assert ev.error_type_name == "micro-architectural error"

    fault = classify_ghes(ev)
    assert "error_type:0x08" in fault.tags
    assert "section:processor" in fault.tags
    assert "page_isolatable" not in fault.tags


def test_processor_id_is_primary_hart_evidence():
    """The core-identification patch writes the hart straight into CPER, so nothing
    has to be inferred."""
    fault = classify_ghes(_event(PROC_LINES))
    assert fault.detail["hart"] == "4"
    assert fault.detail["hart_source"] == "processor_id"
    # source 5 - base 1 = 4 -> the corroboration agrees
    assert fault.detail["hart_corroboration"]["ghes_source"] == 4
    assert "hart_mismatch" not in fault.detail


def test_hart_mismatch_is_surfaced_not_hidden():
    """A disagreement between processor_id and the source-id inference must surface."""
    lines = [ln.replace("Source: 5", "Source: 9") for ln in PROC_LINES]
    fault = classify_ghes(_event(lines, seq0=300))
    assert fault.detail["hart"] == "4"                  # processor_id takes precedence
    assert fault.detail["hart_mismatch"]["processor_id"] == 4
    assert fault.detail["hart_mismatch"]["ghes_source"] == 8
    assert "hart_mismatch" in fault.tags


def test_memory_section_falls_back_to_ghes_source():
    """A memory section has no processor_id — it must fall back to second-class
    inference."""
    fault = classify_ghes(_event(_mem_lines(6, "3, multi-bit ECC"), block_id=2))
    assert fault.detail["hart"] == "5"
    assert fault.detail["hart_source"] == "ghes_source"
