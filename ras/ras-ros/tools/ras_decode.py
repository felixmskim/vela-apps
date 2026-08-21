#!/usr/bin/env python3
"""Decode kernel RAS records into plain dicts. No ROS, no root, no side effects.

Two producers on this platform, and they carry different things:

  ras:non_standard_event   the tracepoint rasdaemon reads. Carries the CPER
                           section as raw bytes, so everything the firmware put
                           in the record survives -- including the fields dmesg
                           never prints. This is the authoritative source.

  /dev/kmsg                the kernel's own GHES report. Already decoded into
                           strings, but lossy: several fields are dropped and
                           one logical error arrives as a dozen separate lines.

Kept ROS-free on purpose. The parser is the part most likely to be wrong, and it
has to be testable on a fixture without a running ROS graph, without root and
without a RERI-capable guest -- which is every environment this repository has
today. `python3 ras_decode.py --self-test` does exactly that.

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import argparse
import json
import pathlib
import re
import struct
import sys

# --- CPER Processor Generic Error Section -----------------------------------
#
# UEFI 2.10 N.2.4.1, and `struct cper_sec_proc_generic` in include/linux/cper.h.
# The offsets below are that struct, and 192 is its sizeof -- which is exactly
# the `data len:192` the recorded sample carries, so the layout is confirmed by
# the data and not only by the header.

PROC_GENERIC_LEN = 192

OFF_VALIDATION = 0    # u64
OFF_PROC_TYPE = 8     # u8
OFF_PROC_ISA = 9      # u8
OFF_ERROR_TYPE = 10   # u8
OFF_OPERATION = 11    # u8
OFF_FLAGS = 12        # u8
OFF_LEVEL = 13        # u8
OFF_CPU_VERSION = 16  # u64
OFF_CPU_BRAND = 24    # char[128]
OFF_PROC_ID = 152     # u64
OFF_TARGET_ADDR = 160  # u64
OFF_REQUESTOR_ID = 168  # u64
OFF_RESPONDER_ID = 176  # u64
OFF_IP = 184          # u64

# Strings come from drivers/firmware/efi/cper.c so a record reads the same here
# as it does in dmesg. If the two ever disagree, this table is the one that is
# wrong.
PROC_TYPE = {0: "IA32/X64", 1: "IA64", 2: "ARM", 3: "RISCV"}
PROC_ISA = {0: "IA32", 1: "IA64", 2: "X64", 3: "ARM A32/T32", 4: "ARM A64",
            5: "RISCV32", 6: "RISCV64", 7: "RISCV128"}
PROC_OP = {0: "unknown or generic", 1: "data read", 2: "data write",
           3: "instruction execution"}

# proc_error_type is a bitmask, not an enum. Twelve RERI codes fold onto 0x08
# alone -- see ../README.md, "The RERI error code does not survive into the log".
PROC_ERROR_TYPE_BITS = [
    (0x01, "cache error"),
    (0x02, "TLB error"),
    (0x04, "bus error"),
    (0x08, "micro-architectural error"),
]
PROC_FLAG_BITS = [
    (0x01, "restartable"),
    (0x02, "precise IP"),
    (0x04, "overflow"),
    (0x08, "corrected"),
]

# The tracepoint's `severity:` is the GHES severity, NOT the CPER severity in
# the record. ghes_do_proc() passes ghes_severity(...), which remaps them:
#
#   CPER_SEV_RECOVERABLE   0  ->  GHES_SEV_RECOVERABLE   2
#   CPER_SEV_FATAL         1  ->  GHES_SEV_PANIC         3
#   CPER_SEV_CORRECTED     2  ->  GHES_SEV_CORRECTED     1
#   CPER_SEV_INFORMATIONAL 3  ->  GHES_SEV_NO            0
#
# So the two numberings are near-inverses of each other and confusing them
# turns a corrected error into a fatal one. The recorded sample settles which
# is which: a deferred RERI error maps to CPER_SEV_RECOVERABLE, and the sample
# shows `severity: 2` beside a dmesg line reading "recoverable".
GHES_SEV = {0: "none", 1: "corrected", 2: "recoverable", 3: "fatal"}

# What TC.10.007 injects is a corrected error, which by definition does not stop
# the workload. It is a warning, not a failure -- see the note in README.md on
# why that matters for the availability criterion.
GHES_SEV_TO_DIAG = {0: 0, 1: 1, 2: 2, 3: 2}   # OK / WARN / ERROR / ERROR

# CPER_SEC_PROC_GENERIC. The tracepoint prints GUIDs with %pUl, which byte-swaps
# the first three groups, so the same GUID appears in two spellings depending on
# who printed it. Both are recognised rather than one being "normalised", because
# a mismatch here should be visible and not silently repaired.
SEC_TYPE_PROC_GENERIC = {
    "9876ccad-47b4-4bdb-b65e-16f193c4f3db",   # as written in include/linux/cper.h
    "adcc7698-b447-db4b-b65e-16f193c4f3db",   # as printed by %pUl
}

# --- ras:non_standard_event -------------------------------------------------

# TASK-PID [CPU] FLAGS TIMESTAMP: event: fields
# Task names contain '-' (rasdaemon does), so the PID is anchored to the bracket
# that follows it rather than to the first '-'.
TRACE_RE = re.compile(
    r"^\s*(?P<task>\S.*?)-(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+(?P<flags>\S+)\s+"
    r"(?P<ts>\d+\.\d+):\s+non_standard_event:\s+(?P<body>.*)$"
)
BODY_RE = re.compile(
    r"severity:\s*(?P<sev>\d+);\s*"
    r"sec type:\s*(?P<sec_type>[0-9a-fA-F-]+);\s*"
    r"FRU:\s*(?P<fru_id>[0-9a-fA-F-]+)\s*(?P<fru_text>.*?);\s*"
    r"data len:\s*(?P<len>\d+);\s*"
    r"raw data:\s*(?P<raw>.*)$"
)
HEX_RE = re.compile(r"\b[0-9a-fA-F]{2}\b")


def _u64(buf, off):
    if off + 8 > len(buf):
        return None
    return struct.unpack_from("<Q", buf, off)[0]


def _u8(buf, off):
    return buf[off] if off < len(buf) else None


def _bits(value, table):
    if value is None:
        return []
    return [name for bit, name in table if value & bit]


def decode_proc_generic(buf):
    """Decode a CPER Processor Generic Error Section. Tolerates a short buffer.

    Truncation is tolerated because trace_pipe output can be clipped, and a
    partial record still carries the fields that matter -- they all live in the
    first 16 bytes. `truncated` says so rather than the caller having to guess.
    """
    out = {
        "section_len": len(buf),
        "truncated": len(buf) < PROC_GENERIC_LEN,
        "validation_bits": _u64(buf, OFF_VALIDATION),
        "proc_type": _u8(buf, OFF_PROC_TYPE),
        "proc_isa": _u8(buf, OFF_PROC_ISA),
        "proc_error_type": _u8(buf, OFF_ERROR_TYPE),
        "operation": _u8(buf, OFF_OPERATION),
        "flags": _u8(buf, OFF_FLAGS),
        "level": _u8(buf, OFF_LEVEL),
        "cpu_version": _u64(buf, OFF_CPU_VERSION),
        "proc_id": _u64(buf, OFF_PROC_ID),
        "target_address": _u64(buf, OFF_TARGET_ADDR),
        "requestor_id": _u64(buf, OFF_REQUESTOR_ID),
        "responder_id": _u64(buf, OFF_RESPONDER_ID),
        "ip": _u64(buf, OFF_IP),
    }
    out["proc_type_name"] = PROC_TYPE.get(out["proc_type"], f"unknown ({out['proc_type']})")
    out["proc_isa_name"] = PROC_ISA.get(out["proc_isa"], f"unknown ({out['proc_isa']})")
    out["operation_name"] = PROC_OP.get(out["operation"], f"unknown ({out['operation']})")
    et = _bits(out["proc_error_type"], PROC_ERROR_TYPE_BITS)
    out["error_type_names"] = et
    out["error_type_name"] = ", ".join(et) if et else "none set"
    out["flag_names"] = _bits(out["flags"], PROC_FLAG_BITS)
    if len(buf) >= OFF_CPU_BRAND + 128:
        brand = bytes(buf[OFF_CPU_BRAND:OFF_CPU_BRAND + 128]).split(b"\0", 1)[0]
        out["cpu_brand"] = brand.decode("ascii", "replace").strip()
    else:
        out["cpu_brand"] = ""
    return out


def parse_trace_line(line):
    """Parse one ras:non_standard_event line. Returns None if it is not one.

    trace_pipe carries every event enabled in its instance plus blank lines and
    '#' headers, so "not one" is the common case and has to be cheap and quiet.
    """
    m = TRACE_RE.match(line)
    if not m:
        return None
    b = BODY_RE.search(m.group("body"))
    if not b:
        # The header matched, so this really is our event, but the payload did
        # not parse -- a kernel that changed TP_printk, or a clipped line. Say so
        # loudly instead of dropping it: a silently ignored RAS event is the one
        # failure mode this whole component exists to prevent.
        return {
            "source": "trace",
            "parse_error": "event header matched but the payload did not",
            "trace_ts": float(m.group("ts")),
            "raw_line": line.rstrip("\n"),
        }

    raw_hex = HEX_RE.findall(b.group("raw"))
    buf = bytes(int(h, 16) for h in raw_hex)
    sev = int(b.group("sev"))
    sec_type = b.group("sec_type").lower()

    ev = {
        "source": "trace",
        "trace_ts": float(m.group("ts")),
        "task": m.group("task"),
        "pid": int(m.group("pid")),
        "cpu": int(m.group("cpu")),
        "ghes_severity": sev,
        "ghes_severity_name": GHES_SEV.get(sev, f"unknown ({sev})"),
        "sec_type": sec_type,
        "sec_type_known": sec_type in SEC_TYPE_PROC_GENERIC,
        "fru_id": b.group("fru_id").lower(),
        "fru_text": b.group("fru_text").strip(),
        "declared_len": int(b.group("len")),
        "decoded_len": len(buf),
        "raw": " ".join(raw_hex),
    }
    # A record whose section type we do not know must not be decoded as if we
    # did. Report it and keep the bytes.
    if ev["sec_type_known"]:
        ev.update(decode_proc_generic(buf))
    else:
        ev["section_note"] = "unknown section type -- bytes kept, not decoded"
    return ev


# --- /dev/kmsg --------------------------------------------------------------
#
# One logical error is many printk lines that share a {N} correlation id, so
# lines are accumulated per id and flushed when the id changes or the group goes
# quiet. `severity:` here is a word, not a number.

KMSG_RE = re.compile(r"^(?P<prio>\d+),(?P<seq>\d+),(?P<usec>\d+),(?P<flags>[^;]*);(?P<msg>.*)$")
HWERR_RE = re.compile(r"\{(?P<cid>\d+)\}\[Hardware Error\]:\s*(?P<text>.*)$")
# Keys are lower-case and may contain spaces ("event severity"). Anchoring to a
# lower-case first letter is what keeps the prose lines out -- "Error 0, type:
# recoverable" and "Hardware error from APEI ...: 1" both end in a colon and
# neither is a field.
KV_RE = re.compile(r"^(?P<key>[a-z][a-z_ ]*):\s*(?P<value>.+?)\s*$")


def parse_kmsg_line(line):
    """Return (correlation_id, field_dict) for a Hardware Error line, else None."""
    m = KMSG_RE.match(line)
    text, usec = (m.group("msg"), int(m.group("usec"))) if m else (line, None)
    h = HWERR_RE.search(text)
    if not h:
        return None
    fields = {"kmsg_usec": usec, "text": h.group("text").strip()}
    kv = KV_RE.match(fields["text"])
    if kv:
        fields["key"] = kv.group("key")
        fields["value"] = kv.group("value")
    return int(h.group("cid")), fields


class KmsgAssembler:
    """Group Hardware Error lines by correlation id.

    Flushing on "the id changed" alone would hold the last error forever, since
    nothing follows it until the next one -- which on a healthy system is never.
    So the caller also polls `due()` on a timer.
    """

    def __init__(self, quiet_s=2.0):
        self.quiet_s = quiet_s
        self.open = {}          # cid -> {"fields": [...], "last": monotonic}

    def add(self, cid, fields, now):
        g = self.open.setdefault(cid, {"fields": [], "first": now, "last": now})
        g["fields"].append(fields)
        g["last"] = now

    def due(self, now):
        ready = [cid for cid, g in self.open.items() if now - g["last"] >= self.quiet_s]
        return [self._build(cid, self.open.pop(cid)) for cid in ready]

    def flush_all(self, now):
        cids = list(self.open)
        return [self._build(cid, self.open.pop(cid)) for cid in cids]

    @staticmethod
    def _build(cid, g):
        kv = {f["key"]: f["value"] for f in g["fields"] if "key" in f}
        sev = kv.get("event severity", "")
        ev = {
            "source": "kmsg",
            "correlation_id": cid,
            "kmsg_severity_name": sev,
            "section_type": kv.get("section_type", ""),
            "error_type_name": kv.get("error_type", ""),
            "operation_name": kv.get("operation", ""),
            "processor_type": kv.get("processor_type", ""),
            "processor_isa": kv.get("processor_isa", ""),
            "target_address": kv.get("target_address", ""),
            "line_count": len(g["fields"]),
            "first_usec": g["fields"][0].get("kmsg_usec"),
            "text": " | ".join(f["text"] for f in g["fields"]),
        }
        # dmesg spells severity; the tracepoint numbers it. Map back so both
        # sources produce the same field and counts can be compared at all.
        for num, name in GHES_SEV.items():
            if name and name in sev:
                ev["ghes_severity"] = num
                ev["ghes_severity_name"] = name
                break
        else:
            ev["ghes_severity"] = 3 if sev else 2
            ev["ghes_severity_name"] = sev or "unknown"
        return ev


def diag_level(event):
    """diagnostic_msgs level for an event: 0 OK, 1 WARN, 2 ERROR."""
    if event.get("parse_error"):
        return 2
    return GHES_SEV_TO_DIAG.get(event.get("ghes_severity"), 2)


def summary(event):
    """One human-readable line. Goes into DiagnosticStatus.message."""
    if event.get("parse_error"):
        return f"unparsable RAS record: {event['parse_error']}"
    sev = event.get("ghes_severity_name", "?")
    et = event.get("error_type_name") or "?"
    op = event.get("operation_name") or "?"
    where = event.get("proc_isa_name") or event.get("processor_isa") or "?"
    return f"{sev} hardware error: {et} ({op}) on {where}"


# --- self-test --------------------------------------------------------------

def _self_test(sample_dir):
    """Parse the recorded fixtures and assert the values the README documents.

    These are not invented lines: they are the output recorded in ../README.md
    from a real injection on a RERI-capable guest. The point is that the parser
    is exercised against ground truth on a machine that cannot produce an event.
    """
    ok = True

    def check(label, got, want):
        nonlocal ok
        good = got == want
        ok = ok and good
        print(f"  {'PASS' if good else 'FAIL'}  {label}: {got!r}"
              + ("" if good else f"  (expected {want!r})"))

    trace_file = sample_dir / "non_standard_event.trace"
    print(f"trace fixture: {trace_file}")
    events = [e for e in (parse_trace_line(l) for l in trace_file.read_text().splitlines()) if e]
    check("events parsed", len(events), 2)

    tlb = events[0]
    check("tlb sec type recognised", tlb["sec_type_known"], True)
    check("tlb ghes severity", tlb["ghes_severity"], 2)
    check("tlb ghes severity name", tlb["ghes_severity_name"], "recoverable")
    check("tlb proc_type", tlb["proc_type_name"], "RISCV")
    check("tlb proc_isa", tlb["proc_isa_name"], "RISCV64")
    check("tlb error type", tlb["error_type_name"], "TLB error")
    check("tlb operation", tlb["operation_name"], "data read")
    check("tlb declared len", tlb["declared_len"], 192)
    check("tlb decoded len", tlb["decoded_len"], 192)
    check("tlb truncated", tlb["truncated"], False)
    check("tlb target address", tlb["target_address"], 0)
    check("tlb diag level", diag_level(tlb), 2)

    ce = events[1]
    check("ce ghes severity name", ce["ghes_severity_name"], "corrected")
    check("ce error type", ce["error_type_name"], "micro-architectural error")
    check("ce operation", ce["operation_name"], "data write")
    check("ce diag level (corrected is a warning)", diag_level(ce), 1)

    kmsg_file = sample_dir / "hardware_error.kmsg"
    print(f"kmsg fixture: {kmsg_file}")
    asm = KmsgAssembler(quiet_s=0.0)
    n = 0
    for line in kmsg_file.read_text().splitlines():
        parsed = parse_kmsg_line(line)
        if parsed:
            asm.add(parsed[0], parsed[1], 0.0)
            n += 1
    check("kmsg lines matched", n, 10)
    groups = asm.flush_all(1.0)
    check("kmsg groups", len(groups), 1)
    check("kmsg severity", groups[0]["ghes_severity_name"], "recoverable")
    check("kmsg error type", groups[0]["error_type_name"], "0x02")
    check("kmsg diag level", diag_level(groups[0]), 2)

    print("SELF-TEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true",
                    help="parse the recorded fixtures and check the decoded values")
    # samples/ sits beside this file in the guest (install_guest.sh flattens
    # tools/) and one level up in the repository. Both layouts are real.
    here = pathlib.Path(__file__).resolve().parent
    default_samples = next(
        (c for c in (here / "samples", here.parent / "samples")
         if (c / "non_standard_event.trace").is_file()), here / "samples")
    ap.add_argument("--samples", default=str(default_samples))
    ap.add_argument("files", nargs="*", help="trace files to decode to JSON lines")
    args = ap.parse_args()

    if args.self_test:
        return _self_test(pathlib.Path(args.samples))
    if not args.files:
        ap.error("give a file to decode, or --self-test")
    for path in args.files:
        for line in pathlib.Path(path).read_text().splitlines():
            ev = parse_trace_line(line)
            if ev:
                print(json.dumps(ev, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
