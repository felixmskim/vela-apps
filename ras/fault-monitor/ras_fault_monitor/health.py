# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""General threat observation — the things that hold no matter which app is running.

Why this is the main body of the work
-------------------------------------
Hardware ECC errors are **lowest in frequency and highest in severity**. What actually
breaks availability in a robot/drone ROS 2 system is almost always below that layer:
the disk fills, fds run out, the clock jumps, a link drops, a node falls into a restart
loop. This module watches that layer, and it never needs to know what the app is.

The defaults are **values that are safe in general**. Configuration only adds
precision; observation has to hold with no configuration at all (:mod:`~.policy`
overrides the thresholds).

Design rules
------------
* **Probe, then use.** A missing metric is left empty with a reason instead of being
  invented (that the emulator has no hwmon is itself information — real hardware does).
* **A state check reports once.** Emitting a fault every cycle while a threshold stays
  crossed turns the log into garbage and buries the new events that matter. When the
  condition clears, a `recovered` entry is written and the check may report again.
* **It counts itself out.** This daemon's own log can be what fills the disk, so a disk
  warning also carries the usage of the log path.
"""

from __future__ import annotations

import os
import re
import time
from dataclasses import dataclass, field

from .fault import Fault

PROC = "/proc"
MOUNTS = "/proc/mounts"
MEMINFO = "/proc/meminfo"
PRESSURE_DIR = "/proc/pressure"
NET_CLASS = "/sys/class/net"
HWMON = "/sys/class/hwmon"
THERMAL = "/sys/class/thermal"

#: only filesystems worth watching — a full tmpfs/proc/sys means something else.
REAL_FS = frozenset({"ext2", "ext3", "ext4", "xfs", "btrfs", "f2fs", "vfat",
                     "exfat", "ubifs", "jffs2", "squashfs", "overlay"})

#: interfaces whose link state is not watched
SKIP_IFACE = ("lo", "sit", "docker", "veth", "br-", "virbr")


@dataclass
class Threshold:
    """General defaults, overridable by configuration."""

    disk_used_warn: float = 0.85          # 85% used -> WARN
    disk_used_crit: float = 0.95          # 95% → CRITICAL
    inode_used_warn: float = 0.85
    mem_available_warn: float = 0.10      # MemAvailable below 10% of the total
    mem_available_crit: float = 0.05
    fd_used_warn: float = 0.80            # 80% of the rlimit
    thread_warn: int = 512                # per process
    clock_jump_s: float = 1.0             # REALTIME diverging this far from MONOTONIC
    restart_loop_n: int = 3               # systemd restart count
    temp_warn_c: float = 80.0
    temp_crit_c: float = 95.0
    psi_some_warn: float = 20.0           # PSI avg10 (%)


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
        return int(text.split()[0])
    except (ValueError, IndexError):
        return None


# -- individual observers ----------------------------------------------------

def real_mounts(text: str) -> list[tuple[str, str, list]]:
    """`(mountpoint, fstype, options)` — real storage only."""
    out = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        _dev, mnt, fstype, opts = parts[0], parts[1], parts[2], parts[3]
        if fstype in REAL_FS:
            out.append((mnt.replace("\\040", " "), fstype, opts.split(",")))
    return out


def disk_usage(mountpoint: str) -> dict | None:
    try:
        st = os.statvfs(mountpoint)
    except OSError:
        return None
    total = st.f_blocks * st.f_frsize
    free = st.f_bavail * st.f_frsize
    if total <= 0:
        return None
    inodes_total, inodes_free = st.f_files, st.f_favail
    return {
        "total_bytes": total,
        "free_bytes": free,
        "used_ratio": round(1.0 - free / total, 4),
        "inode_used_ratio": (round(1.0 - inodes_free / inodes_total, 4)
                             if inodes_total > 0 else None),
    }


def clock_skew() -> dict:
    """Raw values for tracking divergence between REALTIME and MONOTONIC.

    ROS 2 hangs its timers and TF directly off the clock, so a clock that goes backwards
    or jumps collapses nodes with no hardware fault involved. It is a common failure and
    a hard one to diagnose.
    """
    return {"realtime": time.time(), "monotonic": time.monotonic()}


def process_limits(pid: int, proc_dir: str = PROC) -> dict | None:
    """fd usage and thread count. The rlimit differs per process, so it is read too."""
    base = os.path.join(proc_dir, str(pid))
    try:
        fds = len(os.listdir(os.path.join(base, "fd")))
    except OSError:
        return None
    soft = None
    limits = _read(os.path.join(base, "limits"))
    if limits:
        m = re.search(r"^Max open files\s+(\S+)", limits, re.M)
        if m and m.group(1).isdigit():
            soft = int(m.group(1))
    threads = None
    status = _read(os.path.join(base, "status"))
    if status:
        m = re.search(r"^Threads:\s+(\d+)", status, re.M)
        if m:
            threads = int(m.group(1))
    return {"fd_count": fds, "fd_soft_limit": soft, "threads": threads,
            "fd_used_ratio": (round(fds / soft, 4) if soft else None)}


def link_states(net_class: str = NET_CLASS) -> dict:
    out: dict = {}
    try:
        names = sorted(os.listdir(net_class))
    except OSError:
        return out
    for name in names:
        if name.startswith(SKIP_IFACE):
            continue
        base = os.path.join(net_class, name)
        out[name] = {
            "operstate": _read(os.path.join(base, "operstate")),
            "carrier": _read_int(os.path.join(base, "carrier")),
            "rx_errors": _read_int(os.path.join(base, "statistics/rx_errors")),
            "tx_errors": _read_int(os.path.join(base, "statistics/tx_errors")),
            "rx_dropped": _read_int(os.path.join(base, "statistics/rx_dropped")),
        }
    return out


def temperatures(hwmon: str = HWMON, thermal: str = THERMAL) -> dict:
    """Degrees Celsius. Absent under emulation, **present on real hardware**."""
    out: dict = {}
    try:
        for zone in sorted(os.listdir(thermal)):
            if not zone.startswith("thermal_zone"):
                continue
            milli = _read_int(os.path.join(thermal, zone, "temp"))
            if milli is not None:
                out[zone] = round(milli / 1000.0, 2)
    except OSError:
        pass
    try:
        for chip in sorted(os.listdir(hwmon)):
            base = os.path.join(hwmon, chip)
            label = _read(os.path.join(base, "name")) or chip
            for entry in sorted(os.listdir(base)):
                if re.fullmatch(r"temp\d+_input", entry):
                    milli = _read_int(os.path.join(base, entry))
                    if milli is not None:
                        out[f"{label}/{entry}"] = round(milli / 1000.0, 2)
    except OSError:
        pass
    return out


def memory_state(meminfo: str = MEMINFO) -> dict:
    text = _read(meminfo) or ""
    vals: dict = {}
    for key in ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree",
                "HardwareCorrupted"):
        m = re.search(rf"^{key}:\s+(\d+)", text, re.M)
        if m:
            vals[key] = int(m.group(1)) * 1024
    total, avail = vals.get("MemTotal"), vals.get("MemAvailable")
    if total and avail is not None:
        vals["available_ratio"] = round(avail / total, 4)
    return vals


def pressure(pressure_dir: str = PRESSURE_DIR) -> dict:
    """PSI — the best available "the system is about to saturate" signal. Depends on
    kernel config."""
    out: dict = {}
    for res in ("cpu", "memory", "io"):
        text = _read(os.path.join(pressure_dir, res))
        if not text:
            continue
        for line in text.splitlines():
            kind, _, rest = line.partition(" ")
            m = re.search(r"avg10=([\d.]+)", rest)
            if m:
                out[f"{res}_{kind}_avg10"] = float(m.group(1))
    return out


# -- the monitor -------------------------------------------------------------

@dataclass
class HealthSnapshot:
    wall_time: float = 0.0
    disks: dict = field(default_factory=dict)
    readonly: list = field(default_factory=list)
    memory: dict = field(default_factory=dict)
    psi: dict = field(default_factory=dict)
    links: dict = field(default_factory=dict)
    temps: dict = field(default_factory=dict)
    clock: dict = field(default_factory=dict)
    procs: dict = field(default_factory=dict)


class HealthMonitor:
    """Periodically sweeps for app-independent threats.

    These are state checks, so **the same condition is never reported twice**: active
    alerts are remembered and a `recovered` entry is written when one clears.
    """

    def __init__(self, thresholds: Threshold | None = None,
                 node_name: str = "health_monitor",
                 mounts: str = MOUNTS, proc_dir: str = PROC,
                 net_class: str = NET_CLASS, hwmon: str = HWMON,
                 thermal: str = THERMAL, meminfo: str = MEMINFO,
                 pressure_dir: str = PRESSURE_DIR,
                 log_path: str = ""):
        self.t = thresholds or Threshold()
        self.node_name = node_name
        self.mounts = mounts
        self.proc_dir = proc_dir
        self.net_class = net_class
        self.hwmon = hwmon
        self.thermal = thermal
        self.meminfo = meminfo
        self.pressure_dir = pressure_dir
        self.log_path = log_path
        self.active: dict[str, str] = {}      # alert key -> description (dedup)
        self._clock_ref: dict | None = None

    # -- capability probing -------------------------------------------------
    def probe(self) -> dict:
        snap = self.scan(watch_pids=[])
        return {
            "disk": bool(snap.disks),
            "memory": bool(snap.memory),
            "psi": bool(snap.psi) or "kernel CONFIG_PSI not set",
            "links": bool(snap.links),
            "thermal": bool(snap.temps) or "no hwmon/thermal (emulation)",
            "clock": True,
        }

    # -- snapshot -----------------------------------------------------------
    def scan(self, wall_time: float = 0.0,
             watch_pids: list | None = None) -> HealthSnapshot:
        snap = HealthSnapshot(wall_time=wall_time or time.time())
        text = _read(self.mounts) or ""
        for mnt, fstype, opts in real_mounts(text):
            usage = disk_usage(mnt)
            if usage:
                usage["fstype"] = fstype
                snap.disks[mnt] = usage
            if "ro" in opts:
                snap.readonly.append(mnt)
        snap.memory = memory_state(self.meminfo)
        snap.psi = pressure(self.pressure_dir)
        snap.links = link_states(self.net_class)
        snap.temps = temperatures(self.hwmon, self.thermal)
        snap.clock = clock_skew()
        for pid in (watch_pids or []):
            info = process_limits(pid, self.proc_dir)
            if info:
                snap.procs[pid] = info
        return snap

    # -- verdicts -----------------------------------------------------------
    def check(self, snap: HealthSnapshot, prev: HealthSnapshot | None = None
              ) -> list[Fault]:
        faults: list[Fault] = []
        seen: set = set()

        faults.extend(self._check_disks(snap, seen))
        faults.extend(self._check_memory(snap, seen))
        faults.extend(self._check_psi(snap, seen))
        faults.extend(self._check_links(snap, prev, seen))
        faults.extend(self._check_temps(snap, seen))
        faults.extend(self._check_clock(snap, seen))
        faults.extend(self._check_procs(snap, seen))

        # cleared alerts: the condition disappearing has to be recorded too, or the
        # outcome of a response cannot be judged
        for key in [k for k in self.active if k not in seen]:
            desc = self.active.pop(key)
            faults.append(self._fault(
                key, f"cleared: {desc}", "SYSTEM_DEGRADED", "INFO",
                ["health", "recovered", f"alert:{key.split(':', 1)[0]}"]))
        return faults

    # -- individual verdicts ------------------------------------------------
    def _check_disks(self, snap, seen) -> list:
        out = []
        for mnt, d in sorted(snap.disks.items()):
            used = d["used_ratio"]
            key = f"disk_full:{mnt}"
            if used >= self.t.disk_used_crit:
                seen.add(key)
                out += self._raise(key, f"{mnt} disk {used:.1%} used",
                                   "SYSTEM_FAULT", "CRITICAL",
                                   ["health", "disk", "disk_full"],
                                   expected=self.t.disk_used_crit, actual=used,
                                   detail=self._disk_detail(mnt, d))
            elif used >= self.t.disk_used_warn:
                seen.add(key)
                out += self._raise(key, f"{mnt} disk {used:.1%} used",
                                   "SYSTEM_DEGRADED", "WARN",
                                   ["health", "disk", "disk_pressure"],
                                   expected=self.t.disk_used_warn, actual=used,
                                   detail=self._disk_detail(mnt, d))
            ikey = f"inode_full:{mnt}"
            iused = d.get("inode_used_ratio")
            if iused is not None and iused >= self.t.inode_used_warn:
                seen.add(ikey)
                out += self._raise(ikey, f"{mnt} inodes {iused:.1%} used",
                                   "SYSTEM_DEGRADED", "WARN",
                                   ["health", "disk", "inode_pressure"],
                                   expected=self.t.inode_used_warn, actual=iused)
        for mnt in snap.readonly:
            key = f"readonly:{mnt}"
            seen.add(key)
            out += self._raise(key, f"{mnt} switched to read-only",
                               "SYSTEM_FAULT", "CRITICAL",
                               ["health", "disk", "readonly_remount"])
        return out

    def _disk_detail(self, mnt: str, d: dict) -> dict:
        """This daemon's own log may be the cause, so it is recorded alongside."""
        detail = dict(d)
        if self.log_path:
            try:
                own = sum(os.path.getsize(p) for p in self._own_logs()
                          if os.path.exists(p))
                detail["monitor_log_bytes"] = own
            except OSError:
                pass
        return detail

    def _own_logs(self) -> list:
        return [self.log_path] + [f"{self.log_path}.{i}" for i in range(1, 6)]

    def _check_memory(self, snap, seen) -> list:
        ratio = snap.memory.get("available_ratio")
        if ratio is None:
            return []
        key = "mem_low"
        if ratio <= self.t.mem_available_crit:
            seen.add(key)
            return self._raise(key, f"MemAvailable {ratio:.1%}",
                               "SYSTEM_FAULT", "CRITICAL",
                               ["health", "memory", "mem_exhaustion"],
                               expected=self.t.mem_available_crit, actual=ratio,
                               detail=dict(snap.memory))
        if ratio <= self.t.mem_available_warn:
            seen.add(key)
            return self._raise(key, f"MemAvailable {ratio:.1%}",
                               "SYSTEM_DEGRADED", "WARN",
                               ["health", "memory", "mem_pressure"],
                               expected=self.t.mem_available_warn, actual=ratio,
                               detail=dict(snap.memory))
        return []

    def _check_psi(self, snap, seen) -> list:
        out = []
        for name, value in sorted(snap.psi.items()):
            if not name.endswith("some_avg10") or value < self.t.psi_some_warn:
                continue
            key = f"psi:{name}"
            seen.add(key)
            out += self._raise(key, f"PSI {name} = {value}",
                               "SYSTEM_DEGRADED", "WARN",
                               ["health", "psi", "resource_pressure"],
                               expected=self.t.psi_some_warn, actual=value)
        return out

    def _check_links(self, snap, prev, seen) -> list:
        out = []
        for name, st in sorted(snap.links.items()):
            key = f"link_down:{name}"
            if st.get("carrier") == 0 or st.get("operstate") == "down":
                seen.add(key)
                out += self._raise(key, f"{name} link down",
                                   "SYSTEM_DEGRADED", "ERROR",
                                   ["health", "network", "link_down",
                                    f"iface:{name}"], detail=dict(st))
            if prev is None:
                continue
            before = prev.links.get(name) or {}
            for field_name in ("rx_errors", "tx_errors"):
                a, b = before.get(field_name), st.get(field_name)
                if isinstance(a, int) and isinstance(b, int) and b > a:
                    out.append(self._fault(
                        f"link_err:{name}:{field_name}",
                        f"{name} {field_name} +{b - a}",
                        "DRIVER_FAULT", "WARN",
                        ["health", "network", "link_errors", f"iface:{name}"],
                        actual=float(b - a)))
        return out

    def _check_temps(self, snap, seen) -> list:
        out = []
        for sensor, celsius in sorted(snap.temps.items()):
            key = f"temp:{sensor}"
            if celsius >= self.t.temp_crit_c:
                seen.add(key)
                out += self._raise(key, f"{sensor} {celsius}°C",
                                   "SYSTEM_FAULT", "CRITICAL",
                                   ["health", "thermal", "overheat"],
                                   expected=self.t.temp_crit_c, actual=celsius)
            elif celsius >= self.t.temp_warn_c:
                seen.add(key)
                out += self._raise(key, f"{sensor} {celsius}°C",
                                   "SYSTEM_DEGRADED", "WARN",
                                   ["health", "thermal", "thermal_pressure"],
                                   expected=self.t.temp_warn_c, actual=celsius)
        return out

    def _check_clock(self, snap, seen) -> list:
        """When REALTIME jumps against MONOTONIC, ROS 2 timers and TF collapse."""
        cur = snap.clock
        if not cur:
            return []
        ref = self._clock_ref
        self._clock_ref = cur
        if not ref:
            return []
        d_real = cur["realtime"] - ref["realtime"]
        d_mono = cur["monotonic"] - ref["monotonic"]
        skew = d_real - d_mono
        if abs(skew) < self.t.clock_jump_s:
            return []
        direction = "went backwards" if skew < 0 else "jumped"
        return [self._fault(
            "clock_jump", f"clock {direction} {skew:+.3f}s (against the monotonic clock)",
            "SYSTEM_DEGRADED", "ERROR" if skew < 0 else "WARN",
            ["health", "clock", "clock_backwards" if skew < 0 else "clock_jump"],
            actual=skew, detail={"delta_realtime": round(d_real, 6),
                                 "delta_monotonic": round(d_mono, 6)})]

    def _check_procs(self, snap, seen) -> list:
        out = []
        for pid, info in sorted(snap.procs.items()):
            ratio = info.get("fd_used_ratio")
            if ratio is not None and ratio >= self.t.fd_used_warn:
                key = f"fd_high:{pid}"
                seen.add(key)
                out += self._raise(
                    key, f"pid {pid} fd {info['fd_count']}/{info['fd_soft_limit']}",
                    "SYSTEM_DEGRADED", "WARN",
                    ["health", "fd", "fd_exhaustion", f"pid:{pid}"],
                    expected=self.t.fd_used_warn, actual=ratio, detail=dict(info))
            threads = info.get("threads")
            if isinstance(threads, int) and threads >= self.t.thread_warn:
                key = f"threads_high:{pid}"
                seen.add(key)
                out += self._raise(
                    key, f"pid {pid} has {threads} threads",
                    "SYSTEM_DEGRADED", "WARN",
                    ["health", "threads", "thread_leak", f"pid:{pid}"],
                    expected=float(self.t.thread_warn), actual=float(threads),
                    detail=dict(info))
        return out

    # -- helpers ------------------------------------------------------------
    def _raise(self, key: str, desc: str, ftype: str, severity: str,
               tags: list, expected: float = 0.0, actual: float = 0.0,
               detail: dict | None = None) -> list:
        """An active alert is not re-reported; log spam buries the new events."""
        if self.active.get(key) == desc:
            return []
        self.active[key] = desc
        return [self._fault(key, desc, ftype, severity, tags,
                            expected, actual, detail)]

    def _fault(self, key: str, desc: str, ftype: str, severity: str,
               tags: list, expected: float = 0.0, actual: float = 0.0,
               detail: dict | None = None) -> Fault:
        return Fault(
            fault_type=ftype, severity=severity, source_layer="system",
            recoverable=True, description=desc,
            tags=list(tags) + [f"key:{key}"],
            node_name=self.node_name,
            expected_value=float(expected), actual_value=float(actual),
            detail={"alert_key": key, **(detail or {})})
