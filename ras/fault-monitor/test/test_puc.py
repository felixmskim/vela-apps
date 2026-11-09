# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""Platform state collector.

Its most important property is that it **never invents a value**. QEMU's CPPC is a
dummy (the delivered/reference ratio is a constant 4, current_freq is
0xdeadbeeffeedbead), and logging those as "hardware state" teaches a prediction model
a fake feature.
"""

import os

from ras_fault_monitor.puc import PucCollector, PucSample

STAT = """\
cpu  100 0 50 900 10 0 0 0 0 0
cpu0 10 0 5 90 1 0 0 0 0 0
cpu1 20 0 10 180 2 0 0 0 0 0
intr 12345
ctxt 6789
"""

INTERRUPTS = """\
           CPU0       CPU1
  5:       1000       2000  RISC-V INTC   5 Edge      riscv-timer
  6:         10         20  SiFive PLIC   8 Level     virtio0
IPI0:        100        200  Rescheduling interrupts
"""


def _tree(tmp_path, *, online=(0, 1), idle=True, cppc=None, freq=None):
    """A fake sysfs/procfs tree."""
    cpu_dir = tmp_path / "cpu"
    for n in online if isinstance(online, (list, tuple)) else [online]:
        d = cpu_dir / f"cpu{n}"
        d.mkdir(parents=True)
        (d / "online").write_text("1\n")
        if idle:
            for i, (name, usage, dwell) in enumerate(
                    [("LPI-0", 5, 100), ("LPI-2", 500, 2_000_000)]):
                s = d / "cpuidle" / f"state{i}"
                s.mkdir(parents=True)
                (s / "name").write_text(name + "\n")
                (s / "usage").write_text(f"{usage}\n")
                (s / "time").write_text(f"{dwell}\n")

    if cppc is not None:
        c = cpu_dir / "cpu0" / "acpi_cppc"
        c.mkdir(parents=True)
        for k, v in cppc.items():
            (c / k).write_text(f"{v}\n")
    if freq is not None:
        f = cpu_dir / "cpu0" / "cpufreq"
        f.mkdir(parents=True)
        (f / "scaling_cur_freq").write_text(f"{freq}\n")

    stat = tmp_path / "stat"
    stat.write_text(STAT)
    irq = tmp_path / "interrupts"
    irq.write_text(INTERRUPTS)
    load = tmp_path / "loadavg"
    load.write_text("0.19 0.11 0.14 1/163 870\n")

    return PucCollector(cpu_dir=str(cpu_dir), proc_stat=str(stat),
                        proc_interrupts=str(irq), proc_loadavg=str(load))


# --- snapshot parsing -------------------------------------------------------

def test_reads_per_cpu_jiffies(tmp_path):
    s = _tree(tmp_path).sample()
    assert s.cpu_jiffies[1]["user"] == 20
    assert s.cpu_jiffies[1]["idle"] == 180
    assert 999 not in s.cpu_jiffies          # the 'cpu ' total line is excluded


def test_reads_idle_states_by_name(tmp_path):
    s = _tree(tmp_path).sample()
    assert s.idle_states[0]["LPI-2"]["usage"] == 500
    assert s.idle_states[0]["LPI-2"]["time_us"] == 2_000_000


def test_irq_columns_matched_to_cpus_by_header(tmp_path):
    s = _tree(tmp_path).sample()
    assert s.irq_counts[0] == 1000 + 10 + 100
    assert s.irq_counts[1] == 2000 + 20 + 200


def test_loadavg_and_online(tmp_path):
    s = _tree(tmp_path).sample()
    assert s.loadavg == (0.19, 0.11, 0.14)
    assert s.online == [0, 1]


# --- capability probing: no invented values ---------------------------------

def test_cppc_counters_unavailable_when_read_faults(tmp_path):
    """Measured: feedback_ctrs gives EFAULT. It must be reported as absent."""
    col = _tree(tmp_path, cppc={"highest_perf": 5})
    # not creating the feedback_ctrs file = the read-failure case
    caps = col.probe()
    assert caps["cppc_counters"].available is False
    assert "EFAULT" in caps["cppc_counters"].reason


def test_cppc_static_caps_marked_as_spec_not_state(tmp_path):
    """It does read, but it is a constant — it must not pass for a state signal."""
    col = _tree(tmp_path, cppc={"highest_perf": 5, "nominal_perf": 4,
                                "lowest_perf": 2, "nominal_freq": 80,
                                "lowest_freq": 40})
    cap = col.probe()["cppc_caps"]
    assert cap.available is True
    assert "does not change" in cap.reason
    assert cap.sample["highest_perf"] == 5


def test_cpu_freq_reported_unavailable_because_placeholder(tmp_path):
    """QEMU's cppc_get_current_freq is a placeholder, so the value is pinned."""
    cap = _tree(tmp_path, freq=80000).probe()["cpu_freq"]
    assert cap.available is False
    assert cap.sample == 80000


def test_live_signals_reported_available(tmp_path):
    caps = _tree(tmp_path).probe()
    for name in ("cpu_utilization", "idle_residency", "irq_rate", "loadavg"):
        assert caps[name].available is True, name


def test_missing_procfs_is_unavailable_not_zero(tmp_path):
    col = PucCollector(cpu_dir=str(tmp_path / "nope"),
                       proc_stat=str(tmp_path / "nope"),
                       proc_interrupts=str(tmp_path / "nope"),
                       proc_loadavg=str(tmp_path / "nope"))
    caps = col.probe()
    assert caps["cpu_utilization"].available is False
    assert caps["idle_residency"].available is False


# --- rate computation -------------------------------------------------------

def _sample(t, *, user0=10, idle0=90, irq0=1110, usage=500, dwell=2_000_000,
            online=(0, 1)):
    return PucSample(
        wall_time=t,
        cpu_jiffies={0: {"user": user0, "system": 5, "idle": idle0, "iowait": 1}},
        idle_states={0: {"LPI-2": {"usage": usage, "time_us": dwell}}},
        irq_counts={0: irq0},
        loadavg=(0.1, 0.2, 0.3),
        online=list(online),
    )


def test_busy_ratio_from_jiffy_deltas():
    a = _sample(0.0, user0=10, idle0=90)
    b = _sample(1.0, user0=20, idle0=90)          # 10 busy, 0 idle
    d = PucCollector.delta(a, b)
    assert d["cpus"][0]["busy_ratio"] == 1.0


def test_idle_residency_ratio():
    a = _sample(0.0, dwell=0)
    b = _sample(1.0, dwell=500_000)               # 0.5 s idle out of 1 s
    d = PucCollector.delta(a, b)
    assert d["cpus"][0]["idle"]["LPI-2"]["residency_ratio"] == 0.5


def test_irq_rate_per_second():
    d = PucCollector.delta(_sample(0.0, irq0=1000), _sample(2.0, irq0=1400))
    assert d["cpus"][0]["irq_per_s"] == 200.0


def test_zero_interval_yields_error_not_division():
    d = PucCollector.delta(_sample(5.0), _sample(5.0))
    assert "error" in d and not d["cpus"]


def test_counter_going_backwards_is_discarded():
    """Counters reset when a CPU goes offline and back online — no negative rate."""
    a = _sample(0.0, irq0=5000, dwell=9_000_000)
    b = _sample(1.0, irq0=10, dwell=100)
    d = PucCollector.delta(a, b)
    assert "irq_per_s" not in d["cpus"].get(0, {})
    assert "idle" not in d["cpus"].get(0, {})


def test_online_change_is_surfaced():
    """Core isolation must surface here so it can be matched to the isolation event."""
    d = PucCollector.delta(_sample(0.0, online=(0, 1)), _sample(1.0, online=(0,)))
    assert d["online_changed"] == {"before": [0, 1], "after": [0]}
