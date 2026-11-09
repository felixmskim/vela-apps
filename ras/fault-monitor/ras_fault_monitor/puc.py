# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Platform / hardware state collector (the "PuC hardware state" slot in the deck).

What is obtainable and what is not — measured
---------------------------------------------
With `-M virt,rpmi=true` the RPMI path is alive and the kernel attaches
(`riscv-sbi-mpxy-mbox: mailbox registered with 1 channels`). But the **sensor service
group is absent from librpmi** (no temperature / fan / power), and CPPC returns dummy
values:

  * `cppc_get_reg()` → `DELIVERED = CSR_TIME * 4`, `REFERENCE = CSR_TIME`
    → the ratio is a **constant 4**. The source comment says
    `dummy delivered performance` too.
  * `cppc_get_current_freq()` → `0xdeadbeeffeedbead` (a placeholder)
  * `cppc_set_reg()` / `cppc_update_perf()` → no-op → **clock control does nothing**
  * even the kernel cannot read the counters:
    `cppc_cpufreq_cpu_fie_init: failed to read perf counters ... -14` (EFAULT)

So this collector gathers **only the live signals** and leaves the dead metrics as
`unavailable` rather than inventing a value. Logging a constant handed over by the
emulator as "hardware state" teaches a prediction model a fake feature, and later
nobody knows where it came from.

Alive (measured: the values move under load)
  * `/proc/stat` per-cpu jiffies → CPU utilization
  * `/sys/.../cpuidle/state*/{usage,time}` → hart idle residency (matches RPMI HSM
    suspend)
  * `/proc/interrupts` → per-cpu interrupt rate
  * `/proc/loadavg`, the online CPU set

If CPPC gets a real implementation on the QEMU side, `probe()` flips to `available`
by itself — the collector does not need rewriting.
"""

from __future__ import annotations

import os
import re
import time
from dataclasses import dataclass, field

CPU_DIR = "/sys/devices/system/cpu"
PROC_STAT = "/proc/stat"
PROC_INTERRUPTS = "/proc/interrupts"
PROC_LOADAVG = "/proc/loadavg"

_CPU_LINE_RE = re.compile(r"^cpu(\d+)\s+(.*)$")

#: field order of the `/proc/stat` cpu lines (Documentation/filesystems/proc.rst)
STAT_FIELDS = ("user", "nice", "system", "idle", "iowait",
               "irq", "softirq", "steal", "guest", "guest_nice")

#: fields counted as idle — the idle share of the denominator in utilization.
IDLE_FIELDS = ("idle", "iowait")


@dataclass
class Capability:
    """Availability of one metric. The reason is recorded so that "why is this
    missing" never has to be investigated twice."""

    name: str
    available: bool
    reason: str = ""
    sample: object = None

    def to_dict(self) -> dict:
        return {"available": self.available, "reason": self.reason}


def _read(path: str) -> str | None:
    try:
        with open(path, "r") as fp:
            return fp.read().strip()
    except OSError:
        return None


def _read_int(path: str) -> int | None:
    text = _read(path)
    if text is None:
        return None
    try:
        return int(text)
    except ValueError:
        return None


@dataclass
class PucSample:
    """A raw snapshot at one instant. Rates come from the difference of two."""

    wall_time: float
    cpu_jiffies: dict = field(default_factory=dict)      # cpu -> {field: value}
    idle_states: dict = field(default_factory=dict)      # cpu -> {state: {usage,time}}
    irq_counts: dict = field(default_factory=dict)       # cpu -> total irqs
    loadavg: tuple = ()
    online: list = field(default_factory=list)


class PucCollector:
    """Platform state collector.

    `probe()` decides once which metrics are available, `sample()` takes a snapshot,
    and `delta()` turns two snapshots into rates.
    """

    def __init__(self, cpu_dir: str = CPU_DIR, proc_stat: str = PROC_STAT,
                 proc_interrupts: str = PROC_INTERRUPTS,
                 proc_loadavg: str = PROC_LOADAVG):
        self.cpu_dir = cpu_dir
        self.proc_stat = proc_stat
        self.proc_interrupts = proc_interrupts
        self.proc_loadavg = proc_loadavg

    # -- capability probing --------------------------------------------------
    def probe(self) -> dict[str, Capability]:
        caps: dict[str, Capability] = {}

        stat = self._read_cpu_jiffies()
        caps["cpu_utilization"] = Capability(
            "cpu_utilization", bool(stat),
            "" if stat else f"cannot read per-cpu lines from {self.proc_stat}",
            sample=len(stat))

        idle = self._read_idle_states()
        caps["idle_residency"] = Capability(
            "idle_residency", bool(idle),
            "" if idle else "no cpuidle sysfs", sample=len(idle))

        irq = self._read_irq_counts()
        caps["irq_rate"] = Capability("irq_rate", bool(irq),
                                      "" if irq else "no per-cpu interrupt columns")

        load = self._read_loadavg()
        caps["loadavg"] = Capability("loadavg", bool(load))

        # -- CPPC: welcome if real, but dummy on this QEMU. No value is invented. --
        cppc_dir = os.path.join(self.cpu_dir, "cpu0", "acpi_cppc")
        ctrs_path = os.path.join(cppc_dir, "feedback_ctrs")
        ctrs = _read(ctrs_path)
        if not os.path.exists(cppc_dir):
            caps["cppc_counters"] = Capability(
                "cppc_counters", False, "no acpi_cppc sysfs")
        elif ctrs is None:
            # Measured: EFAULT (-14). The kernel log carries
            # `cppc_cpufreq_cpu_fie_init: failed to read perf counters` as well.
            caps["cppc_counters"] = Capability(
                "cppc_counters", False,
                "feedback_ctrs read failed (EFAULT) — QEMU cppc_get_reg is a dummy")
        else:
            caps["cppc_counters"] = Capability("cppc_counters", True, sample=ctrs)

        caps["cppc_caps"] = self._probe_cppc_caps(cppc_dir)
        caps["cpu_freq"] = self._probe_cpu_freq()
        return caps

    def _probe_cppc_caps(self, cppc_dir: str) -> Capability:
        """The static caps do read, but they are **constants** — a spec value, not a
        state signal."""
        values = {}
        for name in ("highest_perf", "nominal_perf", "lowest_perf",
                     "nominal_freq", "lowest_freq"):
            v = _read_int(os.path.join(cppc_dir, name))
            if v is not None:
                values[name] = v
        if not values:
            return Capability("cppc_caps", False, "no acpi_cppc static registers")
        return Capability("cppc_caps", True,
                          "static spec value — does not change over time",
                          sample=values)

    def _probe_cpu_freq(self) -> Capability:
        path = os.path.join(self.cpu_dir, "cpu0", "cpufreq", "scaling_cur_freq")
        v = _read_int(path)
        if v is None:
            return Capability("cpu_freq", False, "no scaling_cur_freq")
        # Measured: pinned at 80000 regardless of load, because QEMU's
        # cppc_get_current_freq returns 0xdeadbeeffeedbead and the kernel substitutes
        # the caps value.
        return Capability("cpu_freq", False,
                          f"value is pinned ({v}) — QEMU cppc_get_current_freq is a placeholder",
                          sample=v)

    # -- snapshots -----------------------------------------------------------
    def sample(self) -> PucSample:
        return PucSample(
            wall_time=time.time(),
            cpu_jiffies=self._read_cpu_jiffies(),
            idle_states=self._read_idle_states(),
            irq_counts=self._read_irq_counts(),
            loadavg=self._read_loadavg(),
            online=self._read_online(),
        )

    def _read_cpu_jiffies(self) -> dict:
        out: dict = {}
        try:
            with open(self.proc_stat, "r") as fp:
                for line in fp:
                    m = _CPU_LINE_RE.match(line)
                    if not m:
                        continue
                    nums = m.group(2).split()
                    out[int(m.group(1))] = {
                        name: int(nums[i])
                        for i, name in enumerate(STAT_FIELDS) if i < len(nums)
                    }
        except OSError:
            return {}
        return out

    def _read_idle_states(self) -> dict:
        out: dict = {}
        try:
            names = sorted(n for n in os.listdir(self.cpu_dir)
                           if n.startswith("cpu") and n[3:].isdigit())
        except OSError:
            return {}
        for name in names:
            base = os.path.join(self.cpu_dir, name, "cpuidle")
            if not os.path.isdir(base):
                continue
            states: dict = {}
            for st in sorted(os.listdir(base)):
                if not st.startswith("state"):
                    continue
                usage = _read_int(os.path.join(base, st, "usage"))
                dwell = _read_int(os.path.join(base, st, "time"))
                if usage is None or dwell is None:
                    continue
                label = _read(os.path.join(base, st, "name")) or st
                states[label] = {"usage": usage, "time_us": dwell}
            if states:
                out[int(name[3:])] = states
        return out

    def _read_irq_counts(self) -> dict:
        """Per-cpu column totals from `/proc/interrupts`; the header pairs column to
        cpu."""
        try:
            with open(self.proc_interrupts, "r") as fp:
                lines = fp.read().splitlines()
        except OSError:
            return {}
        if not lines:
            return {}
        header = lines[0].split()
        cpus = [int(h[3:]) for h in header if h.startswith("CPU") and h[3:].isdigit()]
        if not cpus:
            return {}
        totals = {c: 0 for c in cpus}
        for line in lines[1:]:
            parts = line.split()
            if len(parts) < 1 + len(cpus):
                continue
            for i, cpu in enumerate(cpus):
                token = parts[1 + i]
                if token.isdigit():
                    totals[cpu] += int(token)
        return totals

    def _read_loadavg(self) -> tuple:
        text = _read(self.proc_loadavg)
        if not text:
            return ()
        parts = text.split()
        try:
            return (float(parts[0]), float(parts[1]), float(parts[2]))
        except (IndexError, ValueError):
            return ()

    def _read_online(self) -> list:
        out = []
        try:
            names = os.listdir(self.cpu_dir)
        except OSError:
            return out
        for name in names:
            if not name.startswith("cpu") or not name[3:].isdigit():
                continue
            state = _read(os.path.join(self.cpu_dir, name, "online"))
            if state is None or state == "1":
                out.append(int(name[3:]))
        return sorted(out)

    # -- rate computation ----------------------------------------------------
    @staticmethod
    def delta(prev: PucSample, cur: PucSample) -> dict:
        """Rates between two snapshots. Nothing is emitted if the interval is zero or
        a counter went backwards."""
        span = cur.wall_time - prev.wall_time
        out: dict = {"interval_s": round(span, 6), "cpus": {}}
        if span <= 0:
            out["error"] = "interval is zero or negative — no rate can be computed"
            return out

        for cpu, cur_j in cur.cpu_jiffies.items():
            prev_j = prev.cpu_jiffies.get(cpu)
            entry: dict = {}
            if prev_j:
                diffs = {k: cur_j.get(k, 0) - prev_j.get(k, 0) for k in cur_j}
                total = sum(v for v in diffs.values() if v > 0)
                idle = sum(max(diffs.get(k, 0), 0) for k in IDLE_FIELDS)
                if total > 0:
                    entry["busy_ratio"] = round((total - idle) / total, 4)
                    entry["jiffies_delta"] = {k: v for k, v in diffs.items() if v}

            prev_irq, cur_irq = prev.irq_counts.get(cpu), cur.irq_counts.get(cpu)
            if prev_irq is not None and cur_irq is not None and cur_irq >= prev_irq:
                entry["irq_per_s"] = round((cur_irq - prev_irq) / span, 2)

            prev_idle = prev.idle_states.get(cpu, {})
            cur_idle = cur.idle_states.get(cpu, {})
            residency = {}
            for label, cur_st in cur_idle.items():
                prev_st = prev_idle.get(label)
                if not prev_st:
                    continue
                d_time = cur_st["time_us"] - prev_st["time_us"]
                d_use = cur_st["usage"] - prev_st["usage"]
                if d_time < 0 or d_use < 0:
                    continue           # counter went backwards (offline etc.) — drop
                residency[label] = {
                    "residency_ratio": round(d_time / (span * 1_000_000.0), 4),
                    "entries_per_s": round(d_use / span, 2),
                }
            if residency:
                entry["idle"] = residency

            if entry:
                out["cpus"][cpu] = entry

        out["loadavg"] = cur.loadavg
        out["online"] = cur.online
        if set(prev.online) != set(cur.online):
            # An isolated core shows up here, so it can be cross-checked against the
            # isolation event.
            out["online_changed"] = {"before": prev.online, "after": cur.online}
        return out
