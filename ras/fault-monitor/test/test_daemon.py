# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Daemon orchestration — collectors are injected so the policy is verified without a
kernel or root.

The two properties that matter most:
  * it never imports ROS 2 (it has to keep running when ROS 2 dies)
  * without `--arm` it performs no irreversible action
"""

import json
import pathlib

from ras_fault_monitor.daemon import DaemonConfig, JsonlSink, RasDaemon
from ras_fault_monitor.fault import Fault
from ras_fault_monitor.isolate import IsolationResult
from ras_fault_monitor.puc import PucCollector, PucSample
from ras_fault_monitor.ros2app import Ros2Observer, Ros2Snapshot


class _StubReader:
    def __init__(self, batches=()):
        self.batches = list(batches)
        self.opened = False
        self.closed = False

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def read_available(self):
        return self.batches.pop(0) if self.batches else []


class _StubPipeline:
    def __init__(self, faults=()):
        self.faults = list(faults)

    def process(self, _records):
        out, self.faults = self.faults, []
        return out

    def tick(self, _now=None):
        return []

    def flush(self):
        return []


class _StubEbpf:
    def __init__(self, faults=(), windows=()):
        self.faults = list(faults)
        self.windows = list(windows)
        self.decoder = None
        self.stopped = False

    def drain(self):
        out, self.faults = self.faults, []
        return out

    def flush(self):
        out, self.windows = self.windows, []
        return out

    def stop(self):
        self.stopped = True


class _StubIsolator:
    def __init__(self, results=None):
        self.results = results
        self.calls = []

    def respond(self, fault, force=False):
        self.calls.append((fault, force))
        if self.results is None:
            return [IsolationResult("page_isolate", "pfn=0x1", "dry_run",
                                    "armed=False")]
        return self.results


class _StubPuc(PucCollector):
    def __init__(self, samples):
        super().__init__()
        self._samples = list(samples)

    def probe(self):
        from ras_fault_monitor.puc import Capability
        return {"cpu_utilization": Capability("cpu_utilization", True)}

    def sample(self):
        return self._samples.pop(0) if self._samples else PucSample(wall_time=0.0)


class _StubRos2(Ros2Observer):
    def __init__(self, snapshots, faults=()):
        super().__init__()
        self._snaps = list(snapshots)
        self._faults = list(faults)

    def scan(self, wall_time=0.0):
        return self._snaps.pop(0) if self._snaps else Ros2Snapshot(wall_time)

    def diff(self, prev, cur):
        out, self._faults = self._faults, []
        return out


class _StubHealth:
    """Stub for the general threat monitor.

    The real HealthMonitor reads the host's actual procfs and emits disk warnings and the
    like, which is noise in a test asserting on the fault flow, so it is isolated out.
    health itself is verified by test_health.py.
    """

    def probe(self):
        return {"stub": True}

    def scan(self, wall_time=0.0, watch_pids=None):
        return None

    def check(self, snap, prev=None):
        return []


def _daemon(tmp_path, **kw):
    cfg = DaemonConfig(out_path=str(tmp_path / "faults.jsonl"),
                       puc_interval_s=kw.pop("puc_interval_s", 1e9),
                       ros2_interval_s=kw.pop("ros2_interval_s", 1e9),
                       **{k: v for k, v in kw.items()
                          if k in DaemonConfig.__dataclass_fields__})
    stubs = {k: v for k, v in kw.items()
             if k not in DaemonConfig.__dataclass_fields__}
    stubs.setdefault("health", _StubHealth())
    return RasDaemon(cfg, **stubs), cfg


def _lines(path):
    p = pathlib.Path(path)
    return [json.loads(ln) for ln in p.read_text().splitlines() if ln.strip()]


# --- structural properties --------------------------------------------------

def test_daemon_does_not_import_ros2():
    """The daemon has to keep running when the ROS 2 stack dies."""
    src = (pathlib.Path(__file__).resolve().parents[1]
           / "ras_fault_monitor" / "daemon.py").read_text()
    assert "import rclpy" not in src and "from rclpy" not in src


def test_default_is_not_armed():
    """An irreversible action as the default causes an accident on first start."""
    assert DaemonConfig().armed is False
    assert DaemonConfig().force_isolation is False


# --- fault flow -------------------------------------------------------------

def test_kmsg_fault_is_logged(tmp_path):
    fault = Fault(fault_type="RERI_FAULT", severity="ERROR", description="test")
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([fault]),
                     isolator=_StubIsolator())
    got = d.step(now=100.0)
    d.stop()
    assert len(got) == 1
    recs = _lines(cfg.out_path)
    assert recs[0]["fault_type"] == "RERI_FAULT"
    assert d.stats.by_type == {"RERI_FAULT": 1}


def test_ebpf_faults_merged_into_stream(tmp_path):
    f = Fault(fault_type="SYSTEM_DEGRADED", severity="WARN", description="ebpf")
    d, cfg = _daemon(tmp_path, reader=_StubReader(), pipeline=_StubPipeline(),
                     ebpf=_StubEbpf([f]), isolator=_StubIsolator())
    assert len(d.step(now=1.0)) == 1
    d.stop()


def test_response_recorded_alongside_fault(tmp_path):
    """fault -> response -> outcome has to close inside the log."""
    fault = Fault(fault_type="RERI_FAULT", severity="ERROR", description="mem",
                  detail={"pfn": 0x90000, "isolation_target": "page"})
    iso = _StubIsolator()
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([fault]), isolator=iso)
    d.step(now=1.0)
    d.stop()
    rec = _lines(cfg.out_path)[0]
    assert rec["response"][0]["action"] == "page_isolate"
    assert rec["response"][0]["outcome"] == "dry_run"
    assert len(iso.calls) == 1


def test_fault_without_isolation_target_is_not_responded(tmp_path):
    fault = Fault(fault_type="DDS_FAULT", severity="WARN", description="drops")
    iso = _StubIsolator()
    d, _ = _daemon(tmp_path, reader=_StubReader([[1]]),
                   pipeline=_StubPipeline([fault]), isolator=iso)
    d.step(now=1.0)
    d.stop()
    assert iso.calls == []


def test_isolation_result_does_not_trigger_another_response(tmp_path):
    """Responding to an isolation result again would loop forever."""
    fault = Fault(fault_type="SYSTEM_DEGRADED", severity="ERROR",
                  description="memory_failure",
                  tags=["ebpf", "memory_failure", "isolation_result"],
                  detail={"pfn": 0x90000})
    iso = _StubIsolator()
    d, _ = _daemon(tmp_path, reader=_StubReader(), pipeline=_StubPipeline(),
                   ebpf=_StubEbpf([fault]), isolator=iso)
    d.step(now=1.0)
    d.stop()
    assert iso.calls == []


def test_response_exception_does_not_kill_daemon(tmp_path):
    class _Boom:
        def respond(self, fault, force=False):
            raise RuntimeError("isolation failed")

    fault = Fault(fault_type="RERI_FAULT", severity="ERROR", description="x",
                  detail={"pfn": 1})
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([fault]), isolator=_Boom())
    d.step(now=1.0)          # the exception must not escape
    d.stop()
    kinds = [r.get("kind") for r in _lines(cfg.out_path)]
    assert "response_error" in kinds


# --- period control ---------------------------------------------------------

def test_puc_sampled_on_its_own_interval(tmp_path):
    a = PucSample(wall_time=0.0, cpu_jiffies={0: {"user": 1, "idle": 9}},
                  online=[0, 1])
    b = PucSample(wall_time=1.0, cpu_jiffies={0: {"user": 5, "idle": 9}},
                  online=[0])
    d, cfg = _daemon(tmp_path, reader=_StubReader(), pipeline=_StubPipeline(),
                     isolator=_StubIsolator(), puc=_StubPuc([a, b]),
                     puc_interval_s=10.0)
    d._next_puc = 0.0
    d.step(now=0.0)          # first sample only (nothing to compare against)
    faults = d.step(now=10.0)
    d.stop()
    assert d.stats.puc_windows == 1
    # the online CPU count dropped, so it surfaces as a fault (comparable to isolation)
    assert [f for f in faults if "cpu_online_changed" in f.tags]
    recs = _lines(cfg.out_path)
    assert any(r.get("kind") == "puc" for r in recs)


def test_ros2_scanned_on_its_own_interval(tmp_path):
    node_down = Fault(fault_type="SYSTEM_DEGRADED", severity="ERROR",
                      description="node gone", tags=["ros2", "node_down"])
    d, _ = _daemon(tmp_path, reader=_StubReader(), pipeline=_StubPipeline(),
                   isolator=_StubIsolator(),
                   ros2=_StubRos2([Ros2Snapshot(1.0), Ros2Snapshot(2.0)],
                                  [node_down]),
                   ros2_interval_s=5.0)
    d._next_ros2 = 0.0
    d.step(now=0.0)
    faults = d.step(now=5.0)
    d.stop()
    assert d.stats.ros2_scans == 2
    assert [f for f in faults if "node_down" in f.tags]


# --- log sink ---------------------------------------------------------------

def test_metrics_and_faults_share_one_file_for_time_correlation(tmp_path):
    """"load climbed, then the fault hit" has to be visible in one log."""
    f = Fault(fault_type="RERI_FAULT", severity="ERROR", description="x")
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([f]), isolator=_StubIsolator(),
                     ebpf=_StubEbpf(windows=[_Window()]))
    d.step(now=1.0)
    d.stop()
    kinds = {r.get("kind") for r in _lines(cfg.out_path)}
    assert "ebpf_agg" in kinds
    assert any("fault_type" in r for r in _lines(cfg.out_path))


class _Window:
    wall_time = 1.0
    maps = {"skb_drop": {"6": 3}}

    def named(self, name, decoder=None, field_name=None):
        return self.maps.get(name, {})


def test_sink_write_error_is_counted_not_raised(tmp_path):
    sink = JsonlSink(str(tmp_path / "x.jsonl"))
    sink.fp.close()                      # force it closed to provoke a write failure
    sink.write({"a": 1})
    assert sink.write_errors == 1


def test_sink_without_path_only_echoes(capsys):
    sink = JsonlSink("", echo=True)
    sink.write({"a": 1}, "one-line")
    assert "one-line" in capsys.readouterr().out
    assert sink.fp is None


def test_unserializable_value_is_kept_not_lost(tmp_path):
    """A serialization failure never loses the fault — repr is kept if nothing else."""
    class Weird:
        pass

    sink = JsonlSink(str(tmp_path / "y.jsonl"))
    sink.write({"detail": Weird()})
    sink.close()
    recs = _lines(tmp_path / "y.jsonl")
    assert recs[0]["kind"] == "unserializable_record"
    assert "Weird" in recs[0]["repr"]
    assert sink.write_errors == 1


def test_sink_survives_unwritable_path(tmp_path):
    """A monitor dying over its own log permissions stops monitoring (a real incident)."""
    blocked = tmp_path / "nodir"
    blocked.write_text("this is a file, so no subpath can be created under it")
    sink = JsonlSink(str(blocked / "sub" / "x.jsonl"))   # raising here is a failure
    assert sink.fp is None
    assert sink.open_error
    sink.write({"a": 1})                                 # skips recording without dying
    sink.close()


def test_daemon_runs_with_unwritable_log(tmp_path):
    blocked = tmp_path / "f"
    blocked.write_text("x")
    cfg = DaemonConfig(out_path=str(blocked / "sub" / "y.jsonl"))
    d = RasDaemon(cfg, reader=_StubReader([[1]]),
                  pipeline=_StubPipeline([Fault(fault_type="RERI_FAULT",
                                               severity="ERROR",
                                               description="x")]),
                  isolator=_StubIsolator(), health=_StubHealth())
    assert len(d.step(now=1.0)) == 1        # detection continues
    d.stop()


# --- log rotation (even healthy it grows ~15MB a day — measured) ------------

def test_rotates_at_size_limit(tmp_path):
    path = tmp_path / "r.jsonl"
    sink = JsonlSink(str(path), max_bytes=400, backups=2)
    for i in range(60):
        sink.write({"kind": "filler", "i": i, "pad": "x" * 40})
    sink.close()
    assert sink.rotations >= 1
    assert path.exists()
    assert (tmp_path / "r.jsonl.1").exists()


def test_keeps_only_configured_backups(tmp_path):
    path = tmp_path / "r.jsonl"
    sink = JsonlSink(str(path), max_bytes=200, backups=2)
    for i in range(200):
        sink.write({"i": i, "pad": "y" * 40})
    sink.close()
    assert (tmp_path / "r.jsonl.1").exists()
    assert (tmp_path / "r.jsonl.2").exists()
    assert not (tmp_path / "r.jsonl.3").exists()      # the oldest is dropped


def test_rotation_preserves_newest_in_primary(tmp_path):
    """The newest record stays in the base file after rotation, keeping lookup simple."""
    path = tmp_path / "r.jsonl"
    sink = JsonlSink(str(path), max_bytes=250, backups=2)
    for i in range(80):
        sink.write({"i": i, "pad": "z" * 40})
    sink.write({"marker": "last"})
    sink.close()
    assert "last" in path.read_text()


def test_rotation_disabled_when_max_bytes_zero(tmp_path):
    path = tmp_path / "r.jsonl"
    sink = JsonlSink(str(path), max_bytes=0)
    for i in range(50):
        sink.write({"i": i, "pad": "w" * 40})
    sink.close()
    assert sink.rotations == 0
    assert not (tmp_path / "r.jsonl.1").exists()


def test_existing_file_size_counted_on_open(tmp_path):
    """Ignoring the existing size in append mode delays rotation."""
    path = tmp_path / "r.jsonl"
    path.write_text("x" * 500 + "\n")
    sink = JsonlSink(str(path), max_bytes=400, backups=1)
    sink.write({"a": 1})
    sink.close()
    assert sink.rotations == 1


# --- policy integration (application knowledge comes only from the policy) --

def test_critical_node_escalated_by_policy(tmp_path):
    from ras_fault_monitor.policy import parse_policy
    pol = parse_policy({"nodes": [{"name": "cmd_vel_mux",
                                   "restart": "systemctl restart mux"}]})
    fault = Fault(fault_type="SYSTEM_DEGRADED", severity="ERROR",
                  description="ROS 2 node disappeared: cmd_vel_mux",
                  tags=["ros2", "node_down"],
                  detail={"identity": "cmd_vel_mux"})
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([fault]),
                     isolator=_StubIsolator(), policy=pol)
    d.step(now=1.0)
    d.stop()
    rec = _lines(cfg.out_path)[0]
    assert rec["severity"] == "CRITICAL"
    assert "critical_node" in rec["tags"] and "recovery_known" in rec["tags"]
    assert rec["detail"]["suggested_recovery"] == "systemctl restart mux"


def test_unlisted_node_is_not_escalated(tmp_path):
    """Treating an unknown node as critical empties the severity scale of meaning."""
    fault = Fault(fault_type="SYSTEM_DEGRADED", severity="ERROR",
                  description="ROS 2 node disappeared: whatever",
                  tags=["ros2", "node_down"], detail={"identity": "whatever"})
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([fault]), isolator=_StubIsolator())
    d.step(now=1.0)
    d.stop()
    rec = _lines(cfg.out_path)[0]
    assert rec["severity"] == "ERROR" and "critical_node" not in rec["tags"]


def test_stateful_critical_node_has_no_recovery(tmp_path):
    """No recovery mechanism is offered for a node that gets worse when restarted."""
    from ras_fault_monitor.policy import parse_policy
    pol = parse_policy({"nodes": [{"name": "imu_node", "stateful": True,
                                   "restart": "systemctl restart imu"}]})
    fault = Fault(fault_type="SYSTEM_DEGRADED", severity="ERROR",
                  description="disappeared", tags=["ros2", "node_down"],
                  detail={"identity": "imu_node"})
    d, cfg = _daemon(tmp_path, reader=_StubReader([[1]]),
                     pipeline=_StubPipeline([fault]),
                     isolator=_StubIsolator(), policy=pol)
    d.step(now=1.0)
    d.stop()
    rec = _lines(cfg.out_path)[0]
    assert rec["severity"] == "CRITICAL"
    assert "recovery_unknown" in rec["tags"]
    assert "suggested_recovery" not in rec["detail"]


def test_recovery_command_is_suggested_not_executed(tmp_path):
    """Running an arbitrary command is a privilege that needs its own approval."""
    src = (pathlib.Path(__file__).resolve().parents[1]
           / "ras_fault_monitor" / "daemon.py").read_text()
    assert "subprocess" not in src
    assert "os.system" not in src
