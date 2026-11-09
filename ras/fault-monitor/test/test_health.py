# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""General threat observation — what holds without knowing the app.

This is the layer that breaks availability far more often than a hardware ECC error.
A fake sysfs/procfs is laid down and only the verdicts are checked — the disk cannot
actually be filled nor the clock actually turned.
"""

import os

from ras_fault_monitor.health import (
    HealthMonitor, HealthSnapshot, Threshold, memory_state, pressure,
    process_limits, real_mounts, temperatures,
)

MOUNTS = """\
/dev/vda1 / ext4 rw,relatime 0 0
proc /proc proc rw,nosuid 0 0
tmpfs /run tmpfs rw,nosuid 0 0
/dev/vda2 /data ext4 ro,relatime 0 0
sysfs /sys sysfs rw 0 0
"""

MEMINFO = """\
MemTotal:       64305728 kB
MemFree:        60000000 kB
MemAvailable:    3200000 kB
SwapTotal:             0 kB
SwapFree:              0 kB
HardwareCorrupted:     0 kB
"""


def _mon(tmp_path, **kw):
    return HealthMonitor(thresholds=kw.pop("thresholds", Threshold()),
                         mounts=str(tmp_path / "mounts"),
                         proc_dir=str(tmp_path / "proc"),
                         net_class=str(tmp_path / "net"),
                         hwmon=str(tmp_path / "hwmon"),
                         thermal=str(tmp_path / "thermal"),
                         meminfo=str(tmp_path / "meminfo"),
                         pressure_dir=str(tmp_path / "pressure"), **kw)


# --- mount selection --------------------------------------------------------

def test_only_real_filesystems_are_watched():
    """A full tmpfs/proc means something else."""
    got = real_mounts(MOUNTS)
    assert [m for m, _, _ in got] == ["/", "/data"]


def test_readonly_option_detected():
    opts = {m: o for m, _, o in real_mounts(MOUNTS)}
    assert "ro" in opts["/data"] and "rw" in opts["/"]


# --- disk -------------------------------------------------------------------

def _snap(disks=None, readonly=(), memory=None, psi=None, links=None,
          temps=None, procs=None):
    return HealthSnapshot(
        wall_time=100.0, disks=disks or {}, readonly=list(readonly),
        memory=memory or {}, psi=psi or {}, links=links or {},
        temps=temps or {}, clock={}, procs=procs or {})


def test_disk_warn_then_crit(tmp_path):
    mon = _mon(tmp_path)
    warn = mon.check(_snap(disks={"/": {"used_ratio": 0.90,
                                        "inode_used_ratio": 0.1}}))
    assert [f.severity for f in warn] == ["WARN"]
    assert "disk_pressure" in warn[0].tags

    mon2 = _mon(tmp_path)
    crit = mon2.check(_snap(disks={"/": {"used_ratio": 0.97,
                                         "inode_used_ratio": 0.1}}))
    assert crit[0].severity == "CRITICAL"
    assert crit[0].fault_type == "SYSTEM_FAULT"


def test_inode_pressure_separate_from_space(tmp_path):
    """Running out of inodes with space to spare really does happen."""
    mon = _mon(tmp_path)
    got = mon.check(_snap(disks={"/": {"used_ratio": 0.2,
                                       "inode_used_ratio": 0.93}}))
    assert len(got) == 1 and "inode_pressure" in got[0].tags


def test_readonly_remount_is_critical(tmp_path):
    got = _mon(tmp_path).check(_snap(readonly=["/"]))
    assert got[0].severity == "CRITICAL"
    assert "readonly_remount" in got[0].tags


def test_monitor_own_log_size_reported_with_disk_alert(tmp_path):
    """This daemon's own log may be the cause, so it is recorded alongside."""
    log = tmp_path / "f.jsonl"
    log.write_text("x" * 500)
    mon = _mon(tmp_path, log_path=str(log))
    got = mon.check(_snap(disks={"/": {"used_ratio": 0.9,
                                       "inode_used_ratio": 0.1}}))
    assert got[0].detail["monitor_log_bytes"] == 500


# --- duplicate suppression --------------------------------------------------

def test_same_condition_reported_once(tmp_path):
    """Reporting every cycle while the state persists buries new events."""
    mon = _mon(tmp_path)
    s = _snap(disks={"/": {"used_ratio": 0.9, "inode_used_ratio": 0.1}})
    assert len(mon.check(s)) == 1
    assert mon.check(s) == []
    assert mon.check(s) == []


def test_recovery_is_recorded(tmp_path):
    """The clearing has to be recorded too, or a response cannot be judged."""
    mon = _mon(tmp_path)
    mon.check(_snap(disks={"/": {"used_ratio": 0.9, "inode_used_ratio": 0.1}}))
    got = mon.check(_snap(disks={"/": {"used_ratio": 0.3,
                                       "inode_used_ratio": 0.1}}))
    assert len(got) == 1
    assert "recovered" in got[0].tags and got[0].severity == "INFO"
    # after clearing it must be able to report again
    again = mon.check(_snap(disks={"/": {"used_ratio": 0.9,
                                         "inode_used_ratio": 0.1}}))
    assert len(again) == 1 and "disk_pressure" in again[0].tags


def test_worsening_condition_is_reported_again(tmp_path):
    """Same key, changed description (worsening) reports again."""
    mon = _mon(tmp_path)
    mon.check(_snap(disks={"/": {"used_ratio": 0.86, "inode_used_ratio": 0.1}}))
    got = mon.check(_snap(disks={"/": {"used_ratio": 0.91,
                                       "inode_used_ratio": 0.1}}))
    assert len(got) == 1


# --- memory / PSI -----------------------------------------------------------

def test_meminfo_parsed_to_ratio(tmp_path):
    path = tmp_path / "meminfo"
    path.write_text(MEMINFO)
    got = memory_state(str(path))
    assert got["MemTotal"] == 64305728 * 1024
    assert got["MemAvailable"] == 3200000 * 1024
    assert got["available_ratio"] == round(3200000 / 64305728, 4)


def test_meminfo_missing_file_is_empty(tmp_path):
    assert memory_state(str(tmp_path / "nope")) == {}


def test_memory_ratio_thresholds(tmp_path):
    mon = _mon(tmp_path)
    got = mon.check(_snap(memory={"MemTotal": 100, "MemAvailable": 4,
                                  "available_ratio": 0.04}))
    assert got[0].severity == "CRITICAL" and "mem_exhaustion" in got[0].tags


def test_psi_missing_is_not_an_alert(tmp_path):
    """A kernel without PSI is not a fault."""
    assert _mon(tmp_path).check(_snap(psi={})) == []


def test_psi_over_threshold(tmp_path):
    mon = _mon(tmp_path, thresholds=Threshold(psi_some_warn=10.0))
    got = mon.check(_snap(psi={"memory_some_avg10": 42.0}))
    assert len(got) == 1 and "resource_pressure" in got[0].tags


def test_pressure_reader_handles_missing_dir(tmp_path):
    assert pressure(str(tmp_path / "nope")) == {}


# --- links ------------------------------------------------------------------

def test_link_down_detected(tmp_path):
    mon = _mon(tmp_path)
    got = mon.check(_snap(links={"eth0": {"operstate": "down", "carrier": 0}}))
    assert got[0].fault_type == "SYSTEM_DEGRADED"
    assert "link_down" in got[0].tags and "iface:eth0" in got[0].tags


def test_link_error_growth_needs_previous(tmp_path):
    mon = _mon(tmp_path)
    a = _snap(links={"eth0": {"operstate": "up", "carrier": 1,
                              "rx_errors": 5, "tx_errors": 0}})
    b = _snap(links={"eth0": {"operstate": "up", "carrier": 1,
                              "rx_errors": 12, "tx_errors": 0}})
    assert mon.check(a) == []
    got = mon.check(b, prev=a)
    assert len(got) == 1 and got[0].actual_value == 7.0


# --- thermals (real hardware) -----------------------------------------------

def test_thermal_absent_in_emulation(tmp_path):
    assert temperatures(str(tmp_path / "hwmon"), str(tmp_path / "thermal")) == {}


def test_temperature_thresholds(tmp_path):
    mon = _mon(tmp_path, thresholds=Threshold(temp_warn_c=70, temp_crit_c=90))
    warn = mon.check(_snap(temps={"thermal_zone0": 75.0}))
    assert warn[0].severity == "WARN" and "thermal_pressure" in warn[0].tags
    mon2 = _mon(tmp_path, thresholds=Threshold(temp_warn_c=70, temp_crit_c=90))
    crit = mon2.check(_snap(temps={"thermal_zone0": 96.0}))
    assert crit[0].severity == "CRITICAL" and "overheat" in crit[0].tags


# --- clock ------------------------------------------------------------------

def test_clock_backwards_is_error(tmp_path):
    """ROS 2 timers and TF hang directly off the clock."""
    mon = _mon(tmp_path, thresholds=Threshold(clock_jump_s=0.5))
    mon.check(HealthSnapshot(clock={"realtime": 1000.0, "monotonic": 10.0}))
    got = mon.check(HealthSnapshot(clock={"realtime": 995.0, "monotonic": 11.0}))
    assert len(got) == 1
    assert got[0].severity == "ERROR" and "clock_backwards" in got[0].tags


def test_clock_forward_jump_is_warn(tmp_path):
    mon = _mon(tmp_path, thresholds=Threshold(clock_jump_s=0.5))
    mon.check(HealthSnapshot(clock={"realtime": 1000.0, "monotonic": 10.0}))
    got = mon.check(HealthSnapshot(clock={"realtime": 1030.0, "monotonic": 11.0}))
    assert got[0].severity == "WARN" and "clock_jump" in got[0].tags


def test_normal_clock_advance_is_quiet(tmp_path):
    mon = _mon(tmp_path, thresholds=Threshold(clock_jump_s=1.0))
    mon.check(HealthSnapshot(clock={"realtime": 1000.0, "monotonic": 10.0}))
    assert mon.check(HealthSnapshot(clock={"realtime": 1015.0,
                                          "monotonic": 25.0})) == []


def test_first_clock_sample_makes_no_claim(tmp_path):
    mon = _mon(tmp_path)
    assert mon.check(HealthSnapshot(clock={"realtime": 1.0, "monotonic": 1.0})) == []


# --- fd / threads -----------------------------------------------------------

def test_fd_ratio_threshold(tmp_path):
    mon = _mon(tmp_path, thresholds=Threshold(fd_used_warn=0.75))
    got = mon.check(_snap(procs={900: {"fd_count": 800, "fd_soft_limit": 1024,
                                       "fd_used_ratio": 0.78, "threads": 5}}))
    assert len(got) == 1 and "fd_exhaustion" in got[0].tags
    assert "pid:900" in got[0].tags


def test_no_rlimit_means_no_fd_claim(tmp_path):
    """With no known limit, no ratio is invented."""
    mon = _mon(tmp_path)
    assert mon.check(_snap(procs={900: {"fd_count": 5000,
                                        "fd_soft_limit": None,
                                        "fd_used_ratio": None,
                                        "threads": 3}})) == []


def test_thread_leak_threshold(tmp_path):
    mon = _mon(tmp_path, thresholds=Threshold(thread_warn=100))
    got = mon.check(_snap(procs={900: {"fd_count": 3, "fd_soft_limit": 1024,
                                       "fd_used_ratio": 0.003, "threads": 250}}))
    assert len(got) == 1 and "thread_leak" in got[0].tags


def test_process_limits_reads_real_procfs(tmp_path):
    base = tmp_path / "proc" / "42"
    (base / "fd").mkdir(parents=True)
    for i in range(3):
        os.symlink(f"socket:[{i}]", base / "fd" / str(i))
    (base / "limits").write_text(
        "Limit                     Soft Limit  Hard Limit  Units\n"
        "Max open files            1024        4096        files\n")
    (base / "status").write_text("Name:\tx\nThreads:\t7\n")
    got = process_limits(42, str(tmp_path / "proc"))
    assert got == {"fd_count": 3, "fd_soft_limit": 1024, "threads": 7,
                   "fd_used_ratio": round(3 / 1024, 4)}


def test_process_limits_missing_pid(tmp_path):
    assert process_limits(999999, str(tmp_path / "proc")) is None


# --- classification scheme stays intact -------------------------------------

def test_only_existing_fault_types(tmp_path):
    from ras_fault_monitor.fault import FAULT_TYPES
    mon = _mon(tmp_path, thresholds=Threshold(temp_warn_c=1, fd_used_warn=0.1,
                                              thread_warn=1, psi_some_warn=1))
    got = mon.check(_snap(
        disks={"/": {"used_ratio": 0.99, "inode_used_ratio": 0.99}},
        readonly=["/data"], memory={"available_ratio": 0.01},
        psi={"cpu_some_avg10": 50.0},
        links={"eth0": {"operstate": "down", "carrier": 0}},
        temps={"z0": 99.0},
        procs={1: {"fd_count": 900, "fd_soft_limit": 1024,
                   "fd_used_ratio": 0.88, "threads": 99}}))
    assert len(got) >= 7
    for f in got:
        assert f.fault_type in FAULT_TYPES, f.fault_type
        assert f.source_layer == "system"
