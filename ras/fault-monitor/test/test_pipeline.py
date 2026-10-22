# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Parser verification — runs on the host, no QEMU needed.

    cd ros2/src/ras_fault_monitor && python3 -m pytest test/ -v
    (without pytest)  python3 test/test_pipeline.py

What is verified is not "does the regex match the string" but **is the aggregation
right**. Two injections have to become two events, not 20 (the line count) — the
TC.10.007 verdict of "N CE records / 0% detection miss rate" rests on this.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ras_fault_monitor.kmsg import ReplayReader, parse_record       # noqa: E402
from ras_fault_monitor.pipeline import FaultPipeline                # noqa: E402

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fixtures", "reri_sample.kmsg")


def run_fixture():
    reader = ReplayReader(FIXTURE, boot_time=1_800_000_000.0)
    reader.open()
    # live=False: a replay file's time axis differs from the host clock (same as
    # standalone --replay)
    pipeline = FaultPipeline(node_name="test", live=False)
    faults = pipeline.process(reader.read_available())
    faults.extend(pipeline.flush())
    reader.close()
    return faults, pipeline


# -- kmsg record parsing -----------------------------------------------------

def test_parse_record_fields():
    rec = parse_record("3,548,256024630,-;{2}[Hardware Error]:   error_type: 0x02", 1000.0)
    assert rec is not None
    assert rec.priority == 3 and rec.facility == 0 and rec.level == 3
    assert rec.level_name == "ERR"
    assert rec.seq == 548
    assert abs(rec.ts_monotonic - 256.024630) < 1e-6
    assert rec.text.endswith("error_type: 0x02")


def test_parse_record_drops_metadata_lines():
    raw = "6,552,258000000,-;usb 1-1: new device\n SUBSYSTEM=usb\n DEVICE=+usb:1-1"
    rec = parse_record(raw, 0.0)
    assert rec.text == "usb 1-1: new device"      # the body stops at the first line
    assert "SUBSYSTEM" in rec.raw                  # the original is kept whole


def test_parse_record_rejects_garbage():
    assert parse_record("this is not a kmsg record", 0.0) is None
    assert parse_record("", 0.0) is None


# -- GHES reassembly ---------------------------------------------------------

def test_ghes_blocks_become_one_event_each():
    faults, pipeline = run_fixture()
    reri = [f for f in faults if f.fault_type == "RERI_FAULT"]
    assert len(reri) == 2, f"there must be 2 RERI events (got {len(reri)})"
    assert reri[0].detail["line_count"] == 10     # did 10 lines fold into one event
    assert reri[1].detail["line_count"] == 11     # a corrected block adds one notice line


def test_reri_field_extraction():
    faults, _ = run_fixture()
    tlb = [f for f in faults if f.fault_type == "RERI_FAULT"][0]
    assert tlb.detail["error_type"] == "0x02"
    assert tlb.detail["error_type_name"] == "TLB error"
    assert tlb.detail["event severity"] == "recoverable"
    assert tlb.detail["operation"] == "1, data read"
    assert tlb.detail["processor_isa"] == "6, RISCV64"
    assert tlb.detail["ghes_source"] == 1
    assert tlb.severity == "ERROR" and tlb.recoverable is True
    assert "tlb" in tlb.tags


def test_severity_split_corrected_vs_recoverable():
    """Is the distinction TC.04.012 (severity-based policy response) needs still
    alive?"""
    faults, _ = run_fixture()
    reri = [f for f in faults if f.fault_type == "RERI_FAULT"]
    assert [f.severity for f in reri] == ["ERROR", "WARN"]
    corrected = reri[1]
    assert corrected.detail["error_type"] == "0x08"
    assert corrected.detail["error_type_name"] == "micro-architectural error"
    assert "micro_arch" in corrected.tags


# -- hart correlation (TC.04.008 / TC.04.013) --------------------------------

def test_hart_identified_from_ghes_source():
    """The first-class evidence for a hart is the kernel-reported ghes_source
    (hart i -> source 1+i)."""
    faults, _ = run_fixture()
    reri = [f for f in faults if f.fault_type == "RERI_FAULT"]
    assert reri[0].detail["hart"] == "0"          # source 1 → hart0
    assert reri[1].detail["hart"] == "3"          # source 4 → hart3
    for f in reri:
        assert f.detail["hart_source"] == "ghes_source"   # kernel-reported, not guessed
        assert f.detail["hw_unit"] == f"hart{f.detail['hart']}"
    assert "unit:hart3" in reri[1].tags


def test_marker_cross_checks_and_records_injection():
    """The marker records the injection and cross-checks the kernel-reported value."""
    faults, _ = run_fixture()
    reri = [f for f in faults if f.fault_type == "RERI_FAULT"]
    assert reri[1].detail["hart_marker"] == "3"       # checked against the marker
    assert "hart_mismatch" not in reri[1].detail      # they agree -> no mismatch flag
    assert reri[1].detail["inject_base"] == "0x4013000"
    assert reri[1].detail["detect_latency_s"] >= 0


def test_hart_mismatch_is_surfaced_not_hidden():
    """When the kernel report and the marker disagree, it must surface, not be papered
    over."""
    from ras_fault_monitor.kmsg import parse_record
    from ras_fault_monitor.pipeline import FaultPipeline as FP

    pipe = FP(node_name="test", live=False)
    lines = [
        "12,900,300000000,-;RASINJ hart=7 base=0x4017000 code=0x9001404 tag=tlb seq=1",
        "3,901,300100000,-;{9}[Hardware Error]: Hardware error from APEI Generic "
        "Hardware Error Source: 4",     # source 4 = hart3, disagrees with the marker (7)
        "3,902,300101000,-;{9}[Hardware Error]: event severity: recoverable",
        "3,903,300102000,-;{9}[Hardware Error]:   error_type: 0x02",
        "3,904,300103000,-;{9}[Hardware Error]:   operation: 1, data read",
    ]
    faults = pipe.process([parse_record(ln, 0.0) for ln in lines])
    faults.extend(pipe.flush())
    reri = [f for f in faults if f.fault_type == "RERI_FAULT"][0]
    assert reri.detail["hart"] == "3"                       # the kernel report wins
    assert reri.detail["hart_mismatch"]["marker"] == "7"
    assert "hart_mismatch" in reri.tags


def test_cpu_offline_is_captured_with_cpu_tag():
    faults, _ = run_fixture()
    offline = [f for f in faults if "cpu_offline" in f.tags]
    assert len(offline) == 1
    assert offline[0].fault_type == "SYSTEM_DEGRADED"
    assert offline[0].detail["cpu"] == 3
    assert "cpu:3" in offline[0].tags


# -- other classifications ---------------------------------------------------

def test_kernel_and_driver_rules():
    faults, _ = run_fixture()
    types = [f.fault_type for f in faults]
    assert types.count("SYSTEM_FAULT") == 1        # OOM
    assert types.count("DRIVER_FAULT") == 1        # virtio I/O error
    oom = [f for f in faults if f.fault_type == "SYSTEM_FAULT"][0]
    assert oom.severity == "CRITICAL" and oom.recoverable is False


def test_benign_lines_are_ignored():
    """Catching boot noise too makes the log useless as evidence — the whitelist stays."""
    faults, pipeline = run_fixture()
    assert pipeline.stats.records_read == 30
    assert len(faults) == 7   # RERI 2 + markers 2 + CPU off 1 + OOM 1 + driver 1


def test_json_roundtrip_schema():
    faults, _ = run_fixture()
    data = faults[0].to_dict()
    for key in ("timestamp", "fault_type", "severity", "source_layer", "recoverable",
                "node_name", "topic_name", "description", "tags", "raw_log"):
        assert key in data, f"missing ETRI schema field: {key}"
    assert "." in data["timestamp"]                # ns precision as a string


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{'FAILED' if failed else 'ALL PASSED'} ({failed} failed)")
    sys.exit(1 if failed else 0)
