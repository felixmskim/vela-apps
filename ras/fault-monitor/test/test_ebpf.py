# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""eBPF collector — program generation, output parsing, Fault mapping.

No bpftrace process is started (a test that depends on the kernel cannot run anywhere).
Instead the parsing is checked against **output lines copied verbatim from a real guest
run**.
"""

from ras_fault_monitor.ebpf import (
    EbpfCollector, available_probes, build_program, event_to_fault, parse_kv,
    probe_capability,
)

#: output copied verbatim from a guest bpftrace run.
REAL_OUTPUT = [
    "Attaching 8 probes...",
    "READY",
    "EVT signal sig=11 comm=python3 pid=1544 code=1 result=0",
    "--- AGG tick ---",
    "",
    "@softirq[3]: 4",
    "@softirq[9]: 118",
    "@softirq[1]: 214",
    "@softirq[7]: 999",
    "--- AGG tick ---",
    "",
    "@skb_drop[104]: 3",
    "@skb_drop[58]: 3",
    "@softirq[7]: 191",
]


def _events_dir(tmp_path, names):
    root = tmp_path / "events"
    for n in names:
        (root / n).mkdir(parents=True)
    return str(root)


# --- program generation -----------------------------------------------------

def test_only_present_tracepoints_are_included(tmp_path):
    d = _events_dir(tmp_path, ["ras/memory_failure_event", "signal/signal_generate"])
    ok, missing = available_probes(events_dir=d)
    assert [p.name for p in ok] == ["memory_failure", "signal"]
    assert "skb_drop" in [p.name for p in missing]

    text = build_program(ok)
    assert "tracepoint:ras:memory_failure_event" in text
    assert "kfree_skb" not in text                 # including a missing probe fails to attach


def test_interval_block_only_when_aggregated_probes_exist(tmp_path):
    d = _events_dir(tmp_path, ["signal/signal_generate"])
    ok, _ = available_probes(events_dir=d)
    assert "interval:s:" not in build_program(ok)

    d2 = _events_dir(tmp_path / "b", ["skb/kfree_skb"])
    ok2, _ = available_probes(events_dir=d2)
    text = build_program(ok2, interval_s=7)
    assert "interval:s:7" in text
    assert "print(@skb_drop);" in text and "clear(@skb_drop);" in text


def test_capability_reports_missing_bpftrace(tmp_path):
    cap = probe_capability(events_dir=_events_dir(tmp_path, ["oom/mark_victim"]))
    # whether bpftrace exists depends on the host, so only the reason string is checked.
    assert cap["probes_ok"] == ["oom"]
    if not cap["bpftrace"]:
        assert cap["available"] is False and "not installed" in cap["reason"]


# --- parsing ----------------------------------------------------------------

def test_parse_kv_coerces_numbers():
    got = parse_kv("sig=11 comm=python3 pid=1544 code=1")
    assert got == {"sig": 11, "comm": "python3", "pid": 1544, "code": 1}


def test_parse_kv_ignores_junk():
    assert parse_kv("no-equals here sig=7") == {"sig": 7}


def test_consume_real_output_stream(tmp_path):
    col = EbpfCollector(events_dir=_events_dir(tmp_path, ["signal/signal_generate"]))
    faults = []
    for line in REAL_OUTPUT:
        faults.extend(col._consume(line))
    windows = col.flush()

    assert col.ready is True
    assert len(faults) == 1 and faults[0].tags.count("app_crash") == 1

    # an AGG tick closes a window -> 2 windows (the last one closed by flush)
    assert len(windows) == 2
    assert windows[0].maps["softirq"]["7"] == 999
    assert windows[1].maps["skb_drop"] == {"104": 3, "58": 3}
    assert windows[0].total("softirq") == 4 + 118 + 214 + 999


def test_attaching_banner_is_not_an_event(tmp_path):
    col = EbpfCollector(events_dir=_events_dir(tmp_path, ["oom/mark_victim"]))
    assert col._consume("Attaching 8 probes...") == []
    assert col._consume("") == []


# --- Fault mapping ----------------------------------------------------------

def test_sigbus_is_flagged_as_hwpoison_suspect_not_asserted():
    """SIGBUS is likely a hwpoison outcome, but that is not asserted."""
    f = event_to_fault("signal", {"sig": 7, "comm": "ros2_node", "pid": 900})
    assert "hwpoison_suspect" in f.tags
    assert "signal:SIGBUS" in f.tags
    assert "comm:ros2_node" in f.tags
    assert f.severity == "ERROR"


def test_sigsegv_is_app_crash():
    f = event_to_fault("signal", {"sig": 11, "comm": "python3", "pid": 1544})
    assert "app_crash" in f.tags and "hwpoison_suspect" not in f.tags


def test_sigkill_is_lower_severity_than_crash():
    f = event_to_fault("signal", {"sig": 9, "comm": "sh", "pid": 5})
    assert f.severity == "WARN" and "app_crash" not in f.tags


def test_memory_failure_carries_pfn_and_physical_address():
    f = event_to_fault("memory_failure", {"pfn": 0x90000, "type": 0, "result": 0})
    assert f.detail["physical_address_int"] == 0x90000000
    assert "isolation_result" in f.tags
    assert "0x90000" in f.description


def test_oom_is_system_fault_and_unrecoverable():
    f = event_to_fault("oom", {"pid": 42})
    assert f.fault_type == "SYSTEM_FAULT"
    assert f.severity == "CRITICAL" and f.recoverable is False


def test_block_error_is_driver_fault():
    f = event_to_fault("block_error", {"dev": 254, "sector": 100,
                                       "nr_sector": 8, "error": -5})
    assert f.fault_type == "DRIVER_FAULT" and f.source_layer == "driver"


def test_unknown_event_name_yields_nothing():
    assert event_to_fault("something_else", {"a": 1}) is None


def test_existing_fault_types_only():
    """No new fault_type is invented — log consumers are written against the ETRI
    table."""
    from ras_fault_monitor.fault import FAULT_TYPES
    for name, fields in (("memory_failure", {"pfn": 1, "type": 0, "result": 0}),
                         ("signal", {"sig": 7, "comm": "x", "pid": 1}),
                         ("oom", {"pid": 1}),
                         ("block_error", {"dev": 1, "sector": 1,
                                          "nr_sector": 1, "error": -5})):
        f = event_to_fault(name, fields)
        assert f.fault_type in FAULT_TYPES, f.fault_type


# --- self-monitoring --------------------------------------------------------

def test_collector_death_is_reported_as_a_fault(tmp_path):
    """Collection stopping silently is the most dangerous outcome."""
    col = EbpfCollector(events_dir=_events_dir(tmp_path, ["oom/mark_victim"]))
    col._queue.put(None)                     # EOF = bpftrace exited
    faults = col.drain()
    assert len(faults) == 1
    assert "ebpf_collector_down" in faults[0].tags
    assert "self_monitor" in faults[0].tags


# --- runtime enum-name resolution -------------------------------------------

#: print fmt copied verbatim from the guest tracefs format files (excerpt).
MF_FORMAT = ('print fmt: "pfn %#lx: recovery action for %s: %s", REC->pfn, '
             '__print_symbolic(REC->type, { 0, "reserved kernel page" }, '
             '{ 13, "dirty LRU page" }, { 16, "free buddy page" }), '
             '__print_symbolic(REC->result, { 0, "Ignored" }, { 1, "Failed" }, '
             '{ 2, "Delayed" }, { 3, "Recovered" })')

SKB_FORMAT = ('print fmt: "reason: %s", __print_symbolic(REC->reason, '
              '{ 6, "SOCKET_RCVBUFF" }, { 58, "NEIGH_FAILED" }, '
              '{ 104, "QUEUE_PURGE" })')


def _decoder(tmp_path, events):
    from ras_fault_monitor.ebpf import SymbolDecoder
    root = tmp_path / "events"
    for name, text in events.items():
        d = root / name
        d.mkdir(parents=True)
        (d / "format").write_text(text)
    return SymbolDecoder(str(root))


def test_parse_print_symbolic_from_real_format():
    from ras_fault_monitor.ebpf import parse_print_symbolic
    got = parse_print_symbolic(MF_FORMAT)
    assert got["type"][13] == "dirty LRU page"
    assert got["result"][3] == "Recovered"
    assert got["type"][16] == "free buddy page"


def test_memory_failure_names_resolved_from_tracefs(tmp_path):
    """Measured type=13 result=3 -> 'dirty LRU page' / 'Recovered'."""
    dec = _decoder(tmp_path, {"ras/memory_failure_event": MF_FORMAT})
    f = event_to_fault("memory_failure",
                       {"pfn": 0x151a9e, "type": 13, "result": 3}, decoder=dec)
    assert f.detail["page_kind"] == "dirty LRU page"
    assert f.detail["mf_result"] == "Recovered"
    assert "isolation_effective" in f.tags
    assert "dirty LRU page" in f.description and "Recovered" in f.description


def test_failed_isolation_is_marked_ineffective(tmp_path):
    """Logging Failed/Ignored as "isolated" would make the log a lie."""
    dec = _decoder(tmp_path, {"ras/memory_failure_event": MF_FORMAT})
    f = event_to_fault("memory_failure", {"pfn": 1, "type": 0, "result": 1},
                       decoder=dec)
    assert f.detail["mf_result"] == "Failed"
    assert "isolation_ineffective" in f.tags
    assert "isolation_effective" not in f.tags


def test_no_decoder_leaves_numbers_and_claims_no_name():
    f = event_to_fault("memory_failure", {"pfn": 1, "type": 13, "result": 3})
    assert "page_kind" not in f.detail and "mf_result" not in f.detail
    assert not any(t.startswith("mf_result:") for t in f.tags)


def test_unknown_enum_value_gets_no_invented_name(tmp_path):
    dec = _decoder(tmp_path, {"ras/memory_failure_event": MF_FORMAT})
    f = event_to_fault("memory_failure", {"pfn": 1, "type": 999, "result": 999},
                       decoder=dec)
    assert "page_kind" not in f.detail and "mf_result" not in f.detail


def test_skb_drop_reasons_named_in_aggregate(tmp_path):
    """DDS diagnosis needs the "why", not the "how many"."""
    from ras_fault_monitor.ebpf import AggWindow
    dec = _decoder(tmp_path, {"skb/kfree_skb": SKB_FORMAT})
    w = AggWindow(0.0, {"skb_drop": {"58": 3, "104": 3, "6": 12}})
    named = w.named("skb_drop", dec)
    assert named == {"NEIGH_FAILED": 3, "QUEUE_PURGE": 3, "SOCKET_RCVBUFF": 12}


def test_unresolvable_agg_key_keeps_the_number(tmp_path):
    from ras_fault_monitor.ebpf import AggWindow
    dec = _decoder(tmp_path, {"skb/kfree_skb": SKB_FORMAT})
    w = AggWindow(0.0, {"skb_drop": {"9999": 1}})
    assert w.named("skb_drop", dec) == {"9999": 1}
