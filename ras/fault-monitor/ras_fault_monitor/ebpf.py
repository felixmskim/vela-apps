# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""eBPF kernel collector — attaches tracepoints/kprobes to pull in-kernel signals.

Why bpftrace
------------
The guest has **neither clang nor BCC**. A libbpf CO-RE program cannot be compiled in
the guest, and building LLVM under TCG emulation is not realistic. `bpftrace v0.20.2`
carries its own codegen and works with no extra toolchain (measured: 8 probes attached
successfully). It also satisfies the requirement to attach dynamically without touching
kernel source.

Probe selection under TCG
-------------------------
Per-event probes are expensive under emulation, so the set splits in two.

  **Rare and high-value -> emitted individually**
    `ras:memory_failure_event`  page isolation outcome (structured fields:
                                pfn/type/result)
    `signal:signal_generate`    SIGBUS is **the moment hwpoison hit the app** — the most
                                direct link there is between a hardware fault and an
                                application failure. SIGSEGV/SIGABRT/SIGKILL come along.
    `oom:mark_victim`           the moment memory pressure kills a process
    `block:block_rq_error`      storage I/O errors

  **Frequent -> aggregated in a kernel map, printed periodically**
    `skb:kfree_skb`             packet loss by reason (the evidence for DDS comms
                                trouble)
    `irq:softirq_entry`         softirq rate per vec

Probes that fire thousands of times a second — `sched:sched_switch`,
`exceptions:page_fault_user` — are left out of the defaults: under TCG the observation
itself ruins what is being observed.

Limitation
----------
bpftrace is a separate process. If it dies, collection stops silently, so this module
**watches itself** and reports that as a fault (`ebpf_collector_down`).
"""

from __future__ import annotations

import os
import re
import shutil
import signal as signal_mod
import subprocess
import threading
import time
from dataclasses import dataclass, field
from queue import Empty, Queue

from .fault import Fault

BPFTRACE = "bpftrace"
TRACEFS_EVENTS = "/sys/kernel/tracing/events"

#: signal number -> name (only the range common to riscv/Linux)
SIGNAL_NAMES = {6: "SIGABRT", 7: "SIGBUS", 9: "SIGKILL", 11: "SIGSEGV"}

#: `__print_symbolic(REC->field, { 0, "name" }, ...)` from a tracepoint `format` file.
#: This mapping is **read at runtime** — enum values change between kernel versions.
#: 6.18-rc5's `mf_action_page_type` really does differ from older ones (no
#: `MF_MSG_SLAB`, `GET_HWPOISON` added). A table baked into the code puts a silently
#: wrong name in the log the moment the kernel is bumped.
_SYMBOLIC_RE = re.compile(
    r"__print_symbolic\(REC->(\w+),\s*((?:\{\s*\d+\s*,\s*\"[^\"]*\"\s*\}\s*,?\s*)+)\)")
_SYM_PAIR_RE = re.compile(r"\{\s*(\d+)\s*,\s*\"([^\"]*)\"\s*\}")

#: parses `EVT <name> k=v k=v`
_EVT_RE = re.compile(r"^EVT\s+(\w+)\s*(.*)$")
#: `@skb_drop[104]: 3` / `@softirq[7]: 1913`
_MAP_RE = re.compile(r"^@(\w+)\[([^\]]+)\]:\s*(\d+)$")
_AGG_TICK = "--- AGG tick ---"


@dataclass
class ProbeSpec:
    """One probe. If `required_event` is absent from tracefs it drops out of the
    program."""

    name: str
    required_event: str          # 'ras/memory_failure_event'
    program: str                 # bpftrace body
    aggregated: bool = False


def _probe_registry(interval_s: int) -> list[ProbeSpec]:
    return [
        ProbeSpec(
            "memory_failure", "ras/memory_failure_event",
            'tracepoint:ras:memory_failure_event {\n'
            '    printf("EVT memory_failure pfn=%lu type=%d result=%d\\n",\n'
            '           args.pfn, args.type, args.result);\n'
            '}\n'),
        ProbeSpec(
            "signal", "signal/signal_generate",
            # SIGBUS (7) is the point. The rest are app-collapse signals.
            'tracepoint:signal:signal_generate\n'
            '    /args.sig == 7 || args.sig == 11 || args.sig == 6 || args.sig == 9/ {\n'
            '    printf("EVT signal sig=%d comm=%s pid=%d code=%d result=%d\\n",\n'
            '           args.sig, args.comm, args.pid, args.code, args.result);\n'
            '}\n'),
        ProbeSpec(
            "oom", "oom/mark_victim",
            'tracepoint:oom:mark_victim {\n'
            '    printf("EVT oom pid=%d\\n", args.pid);\n'
            '}\n'),
        ProbeSpec(
            "block_error", "block/block_rq_error",
            'tracepoint:block:block_rq_error {\n'
            '    printf("EVT block_error dev=%d sector=%lu nr=%u error=%d\\n",\n'
            '           args.dev, args.sector, args.nr_sector, args.error);\n'
            '}\n'),
        ProbeSpec(
            "skb_drop", "skb/kfree_skb",
            'tracepoint:skb:kfree_skb { @skb_drop[args.reason] = count(); }\n',
            aggregated=True),
        ProbeSpec(
            "softirq", "irq/softirq_entry",
            'tracepoint:irq:softirq_entry { @softirq[args.vec] = count(); }\n',
            aggregated=True),
    ]


def build_program(probes: list[ProbeSpec], interval_s: int = 10) -> str:
    """Build the bpftrace program body from the selected probes."""
    parts = ['BEGIN { printf("READY\\n"); }\n']
    parts.extend(p.program for p in probes)
    agg = [p for p in probes if p.aggregated]
    if agg:
        body = "".join(f"    print(@{p.name});\n" for p in agg)
        clear = "".join(f"    clear(@{p.name});\n" for p in agg)
        parts.append(f'interval:s:{interval_s} {{\n'
                     f'    printf("{_AGG_TICK}\\n");\n{body}{clear}}}\n')
    return "\n".join(parts)


def available_probes(interval_s: int = 10,
                     events_dir: str = TRACEFS_EVENTS) -> tuple[list, list]:
    """(available, missing) — decided by what actually exists in tracefs."""
    ok, missing = [], []
    for spec in _probe_registry(interval_s):
        if os.path.isdir(os.path.join(events_dir, spec.required_event)):
            ok.append(spec)
        else:
            missing.append(spec)
    return ok, missing


def probe_capability(events_dir: str = TRACEFS_EVENTS) -> dict:
    """Decide what is possible before starting the collector."""
    ok, missing = available_probes(events_dir=events_dir)
    path = shutil.which(BPFTRACE)
    return {
        "bpftrace": path or "",
        "available": bool(path) and bool(ok),
        "reason": "" if path else "bpftrace not installed",
        "probes_ok": [p.name for p in ok],
        "probes_missing": [p.name for p in missing],
    }


# -- runtime enum-name resolution --------------------------------------------

def parse_print_symbolic(format_text: str) -> dict[str, dict[int, str]]:
    """tracepoint format body -> `{field_name: {value: name}}`."""
    out: dict[str, dict[int, str]] = {}
    for field_name, body in _SYMBOLIC_RE.findall(format_text):
        out[field_name] = {int(v): name for v, name in _SYM_PAIR_RE.findall(body)}
    return out


class SymbolDecoder:
    """Lazily loads per-tracepoint enum names from tracefs.

    If they cannot be read it **does not invent a name** — the number stays and the name
    is left absent. A wrong name is worse than no name (verdicts end up being written
    against it later).
    """

    def __init__(self, events_dir: str = TRACEFS_EVENTS):
        self.events_dir = events_dir
        self._cache: dict[str, dict[str, dict[int, str]]] = {}

    def maps_for(self, event: str) -> dict[str, dict[int, str]]:
        if event not in self._cache:
            path = os.path.join(self.events_dir, event, "format")
            try:
                with open(path, "r") as fp:
                    self._cache[event] = parse_print_symbolic(fp.read())
            except OSError:
                self._cache[event] = {}
        return self._cache[event]

    def name_of(self, event: str, field_name: str, value) -> str | None:
        if not isinstance(value, int):
            return None
        return self.maps_for(event).get(field_name, {}).get(value)


#: aggregation map name -> which tracepoint field its keys are (for enum resolution)
_AGG_FIELD = {"skb_drop": "reason", "softirq": "vec"}

#: probe name -> the tracefs path of that event (for enum resolution)
PROBE_EVENT = {
    "memory_failure": "ras/memory_failure_event",
    "skb_drop": "skb/kfree_skb",
    "softirq": "irq/softirq_entry",
    "signal": "signal/signal_generate",
    "block_error": "block/block_rq_error",
    "oom": "oom/mark_victim",
}


# -- parsing -----------------------------------------------------------------

def parse_kv(text: str) -> dict:
    """`sig=11 comm=python3 pid=1544` -> dict, with numbers as int."""
    out: dict = {}
    for token in text.split():
        if "=" not in token:
            continue
        key, _, value = token.partition("=")
        try:
            out[key] = int(value, 0)
        except ValueError:
            out[key] = value
    return out


def event_to_fault(name: str, fields: dict, node_name: str = "ebpf_monitor",
                   wall_time: float | None = None,
                   decoder: SymbolDecoder | None = None) -> Fault | None:
    """bpftrace event -> :class:`Fault`.

    Classification uses only the types from the existing ETRI table. No new type is
    invented, because log consumers (aggregation and verdict scripts) are written
    against that table. The distinction is carried in tags instead — adding a tag does
    not break an existing consumer.
    """
    ts = time.time() if wall_time is None else wall_time
    base = dict(fault_type="SYSTEM_DEGRADED", severity="WARN",
                source_layer="kernel", recoverable=True)

    if name == "memory_failure":
        pfn = fields.get("pfn")
        detail = dict(fields)
        if isinstance(pfn, int):
            detail["physical_address_int"] = pfn * 4096

        # Resolve the enum names from tracefs. `type` says what kind of page it was and
        # `result` says whether the isolation actually succeeded — with only the numbers,
        # the log cannot answer "was it isolated?".
        ev = PROBE_EVENT["memory_failure"]
        page_kind = decoder.name_of(ev, "type", fields.get("type")) if decoder else None
        outcome = decoder.name_of(ev, "result", fields.get("result")) if decoder else None
        if page_kind:
            detail["page_kind"] = page_kind
        if outcome:
            detail["mf_result"] = outcome

        tags = ["ebpf", "memory_failure", "isolation_result",
                f"pfn:{pfn}" if pfn is not None else "pfn:unknown"]
        if outcome:
            tags.append(f"mf_result:{outcome}")
            # Only Recovered means the isolation really took. On Failed/Ignored the log
            # must not claim it was isolated.
            tags.append("isolation_effective" if outcome == "Recovered"
                        else "isolation_ineffective")
        desc = (f"memory_failure pfn=0x{pfn:x}" if isinstance(pfn, int)
                else "memory_failure")
        if page_kind:
            desc += f" ({page_kind})"
        if outcome:
            desc += f" -> {outcome}"
        return Fault(**{**base, "severity": "ERROR"}, description=desc, tags=tags,
                     node_name=node_name, timestamp=ts, detail=detail)

    if name == "signal":
        sig = fields.get("sig")
        signame = SIGNAL_NAMES.get(sig, f"SIG{sig}")
        comm = fields.get("comm", "?")
        tags = ["ebpf", "signal", f"signal:{signame}", "app_impact",
                f"comm:{comm}"]
        severity = "WARN"
        if sig == 7:
            # SIGBUS is likely hwpoison hitting the app. Rather than asserting that, a
            # suspicion tag is left and the correlation against the page-isolation event
            # happens a layer up.
            tags.append("hwpoison_suspect")
            severity = "ERROR"
        elif sig in (11, 6):
            tags.append("app_crash")
            severity = "ERROR"
        return Fault(**{**base, "severity": severity},
                     description=f"{signame} -> {comm}(pid {fields.get('pid')})",
                     tags=tags, node_name=node_name, timestamp=ts,
                     detail=dict(fields))

    if name == "oom":
        return Fault(fault_type="SYSTEM_FAULT", severity="CRITICAL",
                     source_layer="kernel", recoverable=False,
                     description=f"OOM victim pid={fields.get('pid')}",
                     tags=["ebpf", "oom", "app_impact"],
                     node_name=node_name, timestamp=ts, detail=dict(fields))

    if name == "block_error":
        return Fault(fault_type="DRIVER_FAULT", severity="ERROR",
                     source_layer="driver", recoverable=True,
                     description=(f"block I/O error dev={fields.get('dev')} "
                                  f"sector={fields.get('sector')} "
                                  f"err={fields.get('error')}"),
                     tags=["ebpf", "block", "io"],
                     node_name=node_name, timestamp=ts, detail=dict(fields))
    return None


@dataclass
class AggWindow:
    """One aggregation window — map name -> {key: count}."""

    wall_time: float
    maps: dict = field(default_factory=dict)

    def total(self, map_name: str) -> int:
        return sum(self.maps.get(map_name, {}).values())

    def named(self, map_name: str, decoder: "SymbolDecoder | None" = None,
              field_name: str | None = None) -> dict:
        """A copy with numeric keys replaced by enum names.

        What matters in DDS diagnosis is not "how many" but "why" —
        `SOCKET_RCVBUFF` (receive buffer overrun) and `NEIGH_FAILED` (address resolution
        failure) are entirely different problems. Keys that cannot be resolved keep
        their number.
        """
        raw = self.maps.get(map_name, {})
        event = PROBE_EVENT.get(map_name)
        if decoder is None or event is None:
            return dict(raw)
        guess = field_name or _AGG_FIELD.get(map_name)
        if guess is None:
            return dict(raw)
        out = {}
        for key, count in raw.items():
            try:
                name = decoder.name_of(event, guess, int(key))
            except (TypeError, ValueError):
                name = None
            out[name or str(key)] = count
        return out


class EbpfCollector:
    """Runs bpftrace as a child process and turns its output into Faults/aggregates."""

    def __init__(self, node_name: str = "ebpf_monitor", interval_s: int = 10,
                 events_dir: str = TRACEFS_EVENTS, bpftrace: str = BPFTRACE):
        self.node_name = node_name
        self.interval_s = interval_s
        self.events_dir = events_dir
        self.bpftrace = bpftrace
        self.decoder = SymbolDecoder(events_dir)
        self.proc: subprocess.Popen | None = None
        self.ready = False
        self._queue: Queue = Queue()
        self._thread: threading.Thread | None = None
        self._cur: dict = {}        # the aggregation window in progress
        self.aggregates: list[AggWindow] = []
        self.probes: list[ProbeSpec] = []

    # -- lifecycle ----------------------------------------------------------
    def program_text(self) -> str:
        self.probes, _ = available_probes(self.interval_s, self.events_dir)
        return build_program(self.probes, self.interval_s)

    def start(self) -> bool:
        text = self.program_text()
        if not self.probes:
            return False
        try:
            self.proc = subprocess.Popen(
                [self.bpftrace, "-e", text],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1)
        except OSError:
            self.proc = None
            return False
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        return True

    def _pump(self) -> None:
        assert self.proc is not None and self.proc.stdout is not None
        for line in self.proc.stdout:
            self._queue.put(line.rstrip("\n"))
        self._queue.put(None)      # EOF marker

    def stop(self) -> None:
        if self.proc is None:
            return
        try:
            self.proc.send_signal(signal_mod.SIGINT)   # bpftrace runs its END block
            self.proc.wait(timeout=10)
        except (subprocess.TimeoutExpired, OSError):
            try:
                self.proc.kill()
            except OSError:
                pass
        self.proc = None

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    # -- consumption --------------------------------------------------------
    def drain(self) -> list[Fault]:
        """Turn the lines produced so far into Faults (never blocks)."""
        faults: list[Fault] = []
        while True:
            try:
                line = self._queue.get_nowait()
            except Empty:
                break
            if line is None:
                # bpftrace died. To keep collection from stopping silently, that fact
                # is raised as a fault of its own.
                faults.append(self._collector_down())
                break
            faults.extend(self._consume(line))
        return faults

    def _consume(self, line: str) -> list[Fault]:
        line = line.strip()
        if not line:
            return []
        if line == "READY":
            self.ready = True
            return []
        if line == _AGG_TICK:
            self._close_window()
            return []
        m = _MAP_RE.match(line)
        if m:
            name, key, count = m.group(1), m.group(2), int(m.group(3))
            self._cur.setdefault(name, {})[key] = count
            return []
        m = _EVT_RE.match(line)
        if m:
            fault = event_to_fault(m.group(1), parse_kv(m.group(2)),
                                   node_name=self.node_name,
                                   decoder=self.decoder)
            return [fault] if fault else []
        return []

    def _close_window(self) -> None:
        if self._cur:
            self.aggregates.append(AggWindow(time.time(), self._cur))
            self._cur = {}

    def flush(self) -> list[AggWindow]:
        self._close_window()
        out = self.aggregates
        self.aggregates = []
        return out

    def _collector_down(self) -> Fault:
        code = self.proc.poll() if self.proc else None
        err = ""
        if self.proc is not None and self.proc.stderr is not None:
            try:
                err = self.proc.stderr.read()[:400]
            except OSError:
                err = ""
        return Fault(
            fault_type="SYSTEM_DEGRADED", severity="ERROR",
            source_layer="monitor", recoverable=True,
            description=f"eBPF collector (bpftrace) exited, exit={code}",
            tags=["ebpf", "ebpf_collector_down", "self_monitor"],
            node_name=self.node_name, detail={"exit_code": code, "stderr": err})
