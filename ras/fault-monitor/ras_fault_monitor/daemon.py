# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The always-on daemon — it binds the collectors together, responds, and records.

It **does not import ROS 2**. When the ROS 2 stack dies this process has to keep running
and record that fact (the point of the hybrid design). Topic-level detection belongs to a
separate bridge node (`ros2topic.py`), and that node dying is observed from here too.

Structure
---------
    /dev/kmsg  --+
    bpftrace   --+
    procfs     --+--> Fault stream --> response policy --> JSONL log
    /proc/net  --+          |                 |
                            +-----------------+--> the response outcome is recorded
                                                   as a Fault too

Poll periods differ per collector. Sweeping everything every second under TCG emulation
lets the observation eat what it observes, so the event-like sources (kmsg/eBPF) are read
often and the state-like ones (PuC/ROS2) rarely.

Why a single-threaded poll loop: a predictable order is what makes "fault -> response ->
outcome" close in chronological order in the log. Only the eBPF collector uses a thread
internally, to read its child process.

Response policy
---------------
The default is **observe only** — without `--arm` no isolation is executed. Making an
irreversible action the default means an unexpected isolation the first time the daemon
is started.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from dataclasses import dataclass, field

from .ebpf import EbpfCollector, probe_capability as ebpf_capability
from .fault import Fault
from .health import HealthMonitor
from .isolate import Isolator
from .pipeline import FaultPipeline, _uptime_now, make_reader
from .policy import load_policy
from .puc import PucCollector
from .ros2app import Ros2Observer
from .topology import HartTopology


@dataclass
class DaemonConfig:
    kmsg_source: str = "/dev/kmsg"
    out_path: str = ""
    armed: bool = False              # whether isolation is actually executed
    kmsg_poll_s: float = 0.5
    ebpf_poll_s: float = 1.0
    puc_interval_s: float = 15.0
    ros2_interval_s: float = 10.0
    ebpf_agg_interval_s: int = 15
    duration_s: float = 0.0          # 0 = forever
    stdout: bool = False
    force_isolation: bool = False    # isolate risky pages too
    min_online_cpus: int = 2
    max_log_bytes: int = 32 * 1024 * 1024
    log_backups: int = 3
    health_interval_s: float = 20.0
    policy_path: str = ""


@dataclass
class DaemonStats:
    faults: int = 0
    responses: int = 0
    puc_windows: int = 0
    ros2_scans: int = 0
    by_type: dict = field(default_factory=dict)
    by_tag: dict = field(default_factory=dict)

    def note(self, fault: Fault) -> None:
        self.faults += 1
        self.by_type[fault.fault_type] = self.by_type.get(fault.fault_type, 0) + 1
        for tag in fault.tags:
            # to keep the tag set from exploding, valued tags (`hart:6`) count by prefix
            key = tag.split(":", 1)[0] if ":" in tag else tag
            self.by_tag[key] = self.by_tag.get(key, 0) + 1


class JsonlSink:
    """Line-oriented JSON recording.

    **It never raises, even when the file cannot be opened.** A monitoring daemon that
    dies over the permissions on its own log file stops monitoring — which really
    happened here: `/var/log/ras` was root-owned while the bridge ran as `ubuntu`, so it
    died with `PermissionError` and `Restart=always` hid that as a restart loop
    (`systemctl is-active` still read `active`). The reason it could not be opened is kept
    in `open_error` and shipped out with the first record.
    """

    def __init__(self, path: str = "", echo: bool = False,
                 max_bytes: int = 32 * 1024 * 1024, backups: int = 3):
        self.path = path
        self.echo = echo
        self.fp = None
        self.write_errors = 0
        self.open_error = ""
        self.rotations = 0
        # Even in a healthy state the measurement records pile up at roughly 15 MB a day
        # (measured). A monitoring daemon that fills the disk defeats its own purpose, so
        # it rotates itself — leaving it to logrotate would add SIGHUP handling or
        # copytruncate coupling, and a deployment missing that config grows without bound
        # in silence.
        self.max_bytes = max_bytes
        self.backups = backups
        self._written = 0
        if path:
            try:
                os.makedirs(os.path.dirname(os.path.abspath(path)) or ".",
                            exist_ok=True)
                self.fp = open(path, "a", encoding="utf-8", buffering=1)
                self._written = self.fp.tell()
            except OSError as exc:
                self.open_error = f"{path}: {exc}"
                # recording is lost but detection continues; the reason goes to stderr
                print(f"warn: cannot open the log file ({self.open_error}) — "
                      "detection continues, only recording is skipped",
                      file=sys.stderr, flush=True)

    def _rotate(self) -> None:
        """Shift `x.jsonl` -> `x.jsonl.1` -> ... -> `.N` and drop the oldest.

        A failed rotation must not kill the daemon — it is counted and writing continues
        (the file just keeps growing).
        """
        if self.fp is None or not self.max_bytes:
            return
        try:
            self.fp.close()
            for i in range(self.backups, 0, -1):
                older = f"{self.path}.{i}"
                newer = self.path if i == 1 else f"{self.path}.{i - 1}"
                if i == self.backups and os.path.exists(older):
                    os.remove(older)
                if os.path.exists(newer):
                    os.replace(newer, older)
            self.fp = open(self.path, "a", encoding="utf-8", buffering=1)
            self._written = 0
            self.rotations += 1
        except OSError as exc:
            self.write_errors += 1
            try:
                self.fp = open(self.path, "a", encoding="utf-8", buffering=1)
                self._written = 0
            except OSError:
                self.fp = None
                self.open_error = f"{self.path}: could not resume after rotation, {exc}"

    def write(self, record: dict, one_line: str = "") -> None:
        """A failed write must not kill the daemon — it is only counted.

        Catching `OSError` alone is not enough: a closed file raises `ValueError`, and a
        non-serializable value raises `TypeError`. Either way, the daemon dying is far
        worse than losing one fault record.
        When serialization fails the value is not dropped but forced to a string.
        """
        try:
            text = json.dumps(record, ensure_ascii=False)
        except (TypeError, ValueError):
            self.write_errors += 1
            text = json.dumps({"kind": "unserializable_record",
                               "repr": repr(record)[:2000]}, ensure_ascii=False)
        if self.fp is not None:
            line = text + "\n"
            try:
                self.fp.write(line)
                # no stat per write — bytes are counted and rotation happens at the limit
                self._written += len(line.encode("utf-8"))
                if self.max_bytes and self._written >= self.max_bytes:
                    self._rotate()
            except (OSError, ValueError):
                self.write_errors += 1
        if self.echo:
            try:
                print(one_line or text, flush=True)
            except (OSError, ValueError):
                self.write_errors += 1

    def close(self) -> None:
        if self.fp is not None:
            self.fp.close()
            self.fp = None


class RasDaemon:
    """The collect -> respond -> record loop.

    One `step()` is one cycle. Tests inject collectors and call `step()` directly — the
    policy has to be verifiable without a kernel or root.
    """

    def __init__(self, config: DaemonConfig, sink: JsonlSink | None = None,
                 pipeline: FaultPipeline | None = None,
                 reader=None, ebpf: EbpfCollector | None = None,
                 puc: PucCollector | None = None,
                 ros2: Ros2Observer | None = None,
                 isolator: Isolator | None = None,
                 health: HealthMonitor | None = None,
                 policy=None):
        self.cfg = config
        # a policy is optional — the defaults are safe in general
        self.policy = policy if policy is not None else load_policy(config.policy_path)
        self.sink = sink if sink is not None else JsonlSink(
            config.out_path, config.stdout,
            max_bytes=config.max_log_bytes, backups=config.log_backups)
        self.topology = HartTopology.load()
        self.pipeline = pipeline if pipeline is not None else FaultPipeline(
            node_name="ras_daemon", live=True, topology=self.topology)
        self.reader = reader
        self.ebpf = ebpf
        self.puc = puc if puc is not None else PucCollector()
        self.ros2 = ros2 if ros2 is not None else Ros2Observer()
        # If the policy forbids core loss, the minimum-online count rises to the policy
        # value — with a real-time deadline, detaching a core is itself the failure.
        min_cores = max(config.min_online_cpus, self.policy.min_cores)
        if not self.policy.core_loss_allowed:
            min_cores = max(min_cores, len(Isolator(armed=False).online_cpus()))
        self.isolator = isolator if isolator is not None else Isolator(
            armed=config.armed, topology=self.topology,
            min_online_cpus=min_cores)
        self.health = health if health is not None else HealthMonitor(
            thresholds=self.policy.thresholds, log_path=config.out_path)
        self.stats = DaemonStats()

        self._next_puc = 0.0
        self._next_ros2 = 0.0
        self._next_health = 0.0
        self._prev_puc = None
        self._prev_ros2 = None
        self._prev_health = None
        self._stopping = False

    # -- lifecycle -----------------------------------------------------------
    def start(self, now: float | None = None) -> dict:
        """Open the collectors and report what came up (the log's first line)."""
        now = time.time() if now is None else now
        status: dict = {"armed": self.cfg.armed,
                        "topology_resolved": self.topology.resolved,
                        "topology_identity": (self.topology.is_identity()
                                              if self.topology.resolved else None)}

        if self.reader is None:
            self.reader = make_reader(self.cfg.kmsg_source, from_beginning=False)
        try:
            self.reader.open()
            status["kmsg"] = "ok"
        except (PermissionError, OSError) as exc:
            status["kmsg"] = f"failed: {exc}"

        if self.ebpf is None:
            cap = ebpf_capability()
            if cap["available"]:
                self.ebpf = EbpfCollector(node_name="ras_daemon",
                                          interval_s=self.cfg.ebpf_agg_interval_s)
                status["ebpf"] = ("ok" if self.ebpf.start()
                                  else "failed: cannot start bpftrace")
                status["ebpf_probes"] = cap["probes_ok"]
            else:
                status["ebpf"] = f"unavailable: {cap['reason'] or 'no probes'}"

        status["puc"] = {k: v.available for k, v in self.puc.probe().items()}
        status["health"] = self.health.probe()
        status["policy"] = self.policy.summary()
        self._next_puc = now
        self._next_ros2 = now
        self._next_health = now
        return status

    def stop(self) -> None:
        self._stopping = True
        if self.ebpf is not None:
            self.ebpf.stop()
        if self.reader is not None:
            try:
                self.reader.close()
            except OSError:
                pass
        self.sink.close()

    # -- one cycle -----------------------------------------------------------
    def step(self, now: float | None = None) -> list[Fault]:
        now = time.time() if now is None else now
        faults: list[Fault] = []

        # (1) kernel log (event-like — often)
        if self.reader is not None:
            try:
                faults.extend(self.pipeline.process(self.reader.read_available()))
                faults.extend(self.pipeline.tick(_uptime_now()))
            except OSError as exc:
                faults.append(self._self_fault(
                    "kmsg read failed", ["kmsg", "collector_down"], str(exc)))

        # (2) eBPF (event-like — often)
        if self.ebpf is not None:
            faults.extend(self.ebpf.drain())
            for window in self.ebpf.flush():
                self.stats.puc_windows += 0     # an aggregation window is not a fault
                self._emit_metric("ebpf_agg", {
                    "wall_time": window.wall_time,
                    "skb_drop": window.named("skb_drop", self.ebpf.decoder),
                    "softirq": window.named("softirq", self.ebpf.decoder),
                })

        # (3) platform state (state-like — rarely)
        if now >= self._next_puc:
            self._next_puc = now + self.cfg.puc_interval_s
            sample = self.puc.sample()
            if self._prev_puc is not None:
                delta = PucCollector.delta(self._prev_puc, sample)
                self.stats.puc_windows += 1
                self._emit_metric("puc", delta)
                if delta.get("online_changed"):
                    faults.append(self._self_fault(
                        f"online CPU set changed: {delta['online_changed']}",
                        ["puc", "cpu_online_changed"], "",
                        severity="WARN", detail=delta["online_changed"]))
            self._prev_puc = sample

        # (4) ROS 2 outside observation (state-like — rarely)
        if now >= self._next_ros2:
            self._next_ros2 = now + self.cfg.ros2_interval_s
            snap = self.ros2.scan(wall_time=now)
            self.stats.ros2_scans += 1
            if self._prev_ros2 is not None:
                faults.extend(self.ros2.diff(self._prev_ros2, snap))
            self._prev_ros2 = snap

        # (5) general threats (app-independent: disk, memory, clock, link, thermal, fd)
        if now >= self._next_health:
            self._next_health = now + self.cfg.health_interval_s
            watch = sorted(self._prev_ros2.processes) if self._prev_ros2 else []
            snap = self.health.scan(wall_time=now, watch_pids=watch)
            faults.extend(self.health.check(snap, self._prev_health))
            self._prev_health = snap

        # (6) respond and record
        for fault in faults:
            self._apply_policy(fault)
            self.stats.note(fault)
            record = fault.to_dict()
            results = self._respond(fault)
            if results:
                record["response"] = [r.to_dict() for r in results]
            self.sink.write(record, fault.one_line())
            for res in results:
                self.stats.responses += 1
                if self.cfg.stdout:
                    print("    " + res.one_line(), flush=True)
        return faults

    def _apply_policy(self, fault: Fault) -> None:
        """Layer application knowledge onto a fault.

        The daemon cannot learn which nodes are critical by observation — that comes only
        from the policy. An unlisted node is **not assumed** critical: treating the unknown
        as critical makes every node death CRITICAL and the severity scale stops meaning
        anything.
        """
        detail = fault.detail or {}
        name = detail.get("identity") or detail.get("node")
        if not name or "node_down" not in fault.tags:
            return
        if self.policy.is_critical(str(name)):
            fault.severity = "CRITICAL"
            fault.tags.append("critical_node")
            recovery = self.policy.recovery_for(str(name))
            if recovery:
                # It is not executed; only what could be done is recorded. Running an
                # arbitrary command is a privilege that needs its own approval.
                detail["suggested_recovery"] = recovery
                fault.tags.append("recovery_known")
            else:
                fault.tags.append("recovery_unknown")

    def _respond(self, fault: Fault):
        """Respond only to faults that carry an isolation target.

        `respond()` returns its verdict under dry-run as well, so **what it intended to do
        lands in the log** regardless of `armed` — which is what a test verdict rests on.
        """
        detail = fault.detail or {}
        if not (isinstance(detail.get("pfn"), int) or detail.get("hart") is not None):
            return []
        if "isolation_result" in fault.tags:
            return []       # responding to an isolation result again would loop forever
        try:
            return self.isolator.respond(fault, force=self.cfg.force_isolation)
        except Exception as exc:            # a failed response must not kill the daemon
            self.sink.write({"kind": "response_error", "error": str(exc),
                             "fault": fault.to_dict()})
            return []

    # -- helpers -------------------------------------------------------------
    def _emit_metric(self, kind: str, payload: dict) -> None:
        """A measurement, not a fault. It goes into the same file tagged by `kind`, so
        that chronological comparison shows "load climbed, then the fault hit" in one
        log."""
        self.sink.write({"kind": kind, "timestamp": time.time(), kind: payload})

    def _self_fault(self, description: str, tags: list, reason: str = "",
                    severity: str = "ERROR", detail: dict | None = None) -> Fault:
        return Fault(
            fault_type="SYSTEM_DEGRADED", severity=severity,
            source_layer="monitor", recoverable=True,
            description=description,
            tags=["self_monitor"] + list(tags),
            node_name="ras_daemon",
            detail={"reason": reason, **(detail or {})})

    def run(self) -> int:
        status = self.start()
        self.sink.write({"kind": "daemon_start", "timestamp": time.time(),
                         "status": status})
        if self.cfg.stdout:
            print("=== daemon start ===",
                  json.dumps(status, ensure_ascii=False), flush=True)

        started = time.time()
        stop = {"flag": False}

        def _handle(signum, _frame):
            stop["flag"] = True

        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, _handle)
            except (OSError, ValueError):
                pass

        try:
            while not stop["flag"]:
                self.step()
                if self.cfg.duration_s and time.time() - started >= self.cfg.duration_s:
                    break
                time.sleep(self.cfg.kmsg_poll_s)
        finally:
            summary = {
                "kind": "daemon_stop", "timestamp": time.time(),
                "uptime_s": round(time.time() - started, 3),
                "faults": self.stats.faults, "responses": self.stats.responses,
                "puc_windows": self.stats.puc_windows,
                "ros2_scans": self.stats.ros2_scans,
                "by_type": self.stats.by_type, "by_tag": self.stats.by_tag,
                "sink_write_errors": self.sink.write_errors,
                "log_rotations": self.sink.rotations,
            }
            self.sink.write(summary)
            if self.cfg.stdout:
                print("=== daemon stop ===",
                      json.dumps(summary, ensure_ascii=False), flush=True)
            self.stop()
        return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="ras_fault_monitor.daemon",
        description="RAS fault collection and response daemon (independent of ROS 2)")
    p.add_argument("--out", default="/var/log/ras/faults.jsonl",
                   help="JSONL log path")
    p.add_argument("--stdout", action="store_true", help="also print to the screen")
    p.add_argument("--arm", action="store_true",
                   help="actually execute isolation (the default is verdict only)")
    p.add_argument("--force-isolation", action="store_true",
                   help="isolate wide-blast-radius pages (THP/KSM etc.) too")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds (0 = forever)")
    p.add_argument("--kmsg-poll", type=float, default=0.5)
    p.add_argument("--puc-interval", type=float, default=15.0)
    p.add_argument("--ros2-interval", type=float, default=10.0)
    p.add_argument("--min-online-cpus", type=int, default=2)
    p.add_argument("--max-log-mb", type=float, default=32.0,
                   help="rotate past this size (0 = never rotate)")
    p.add_argument("--log-backups", type=int, default=3,
                   help="how many rotated files to keep")
    p.add_argument("--policy", default="",
                   help="policy file (default /etc/ras/policy.yaml; defaults if absent)")
    p.add_argument("--health-interval", type=float, default=20.0)
    p.add_argument("--kmsg-source", default="/dev/kmsg")
    return p


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    cfg = DaemonConfig(
        kmsg_source=a.kmsg_source, out_path=a.out, armed=a.arm,
        kmsg_poll_s=a.kmsg_poll, puc_interval_s=a.puc_interval,
        ros2_interval_s=a.ros2_interval, duration_s=a.duration,
        stdout=a.stdout, force_isolation=a.force_isolation,
        min_online_cpus=a.min_online_cpus,
        max_log_bytes=int(a.max_log_mb * 1024 * 1024),
        log_backups=a.log_backups,
        health_interval_s=a.health_interval,
        policy_path=a.policy)
    try:
        return RasDaemon(cfg).run()
    except PermissionError as exc:
        print(f"error: insufficient privileges ({exc}). It must run as root.",
              file=sys.stderr)
        return 13


if __name__ == "__main__":
    raise SystemExit(main())
