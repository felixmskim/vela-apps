# RAS events on ROS 2

The kernel detects a hardware error; a ROS 2 workload never hears about it. This
closes that gap: one node reads the kernel's RAS records and republishes them as
ROS 2 diagnostics, so a robot application can react to a fault in the silicon it
is running on.

```
   kernel                        ras_monitor                 consumers
 ras:non_standard_event ──┐
 /dev/kmsg               ─┼──►  decode · stamp · count ──►  /ras/events
 rasdaemon (sqlite)      ─┘                             ──►  /diagnostics
                                                        ──►  /var/log/ras/events.log
```

The TSP asks for this twice — TC.04.010 wants the error reported into user space
"(ROS2 topic 또는 daemon)", TC.04.011 is an alarm-notification test — and it did
not exist. It is also the half of TC.10.007 that can be built and verified
*before* Config C: the injection needs RERI, the reporting path does not.

---

## What it publishes

| Topic | Type | Carries |
|---|---|---|
| `/ras/events` | `diagnostic_msgs/DiagnosticArray` | one status per hardware error, nothing else |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | the same events, plus a heartbeat every 10 s |
| `/var/log/ras/events.log` | JSON lines | one object per event, flushed on write |

The decoded fields ride as `KeyValue` pairs: `ghes_severity`, `error_type_name`,
`operation_name`, `proc_isa_name`, `target_address`, `seq`, `detect_mono`,
`detect_latency_ms`, and the raw bytes.

### Why `DiagnosticArray` and not a custom message

A `vela_ras_msgs/RasEvent` would map the fields more precisely. It would also
need a `rosidl` build — hours on an emulated riscv64 guest — and would introduce
a type hash the x86-64 host container does not have, so a host-side subscriber
could not read it without building the same package. `diagnostic_msgs` is already
installed on the guest, is what `rqt_robot_monitor` and `diagnostic_aggregator`
consume, and costs nothing. The custom type is an upgrade, not a prerequisite.

### Severity, and why a corrected error is a warning

The tracepoint's `severity:` field is the **GHES** severity, not the CPER
severity in the record. `ghes_do_proc()` remaps them and the two numberings are
near-inverses, so confusing them turns a corrected error into a fatal one.

| GHES | Meaning | CPER it came from | `DiagnosticStatus.level` |
|---|---|---|---|
| 0 | none | informational (3) | `OK` |
| 1 | corrected | corrected (2) | `WARN` |
| 2 | recoverable | recoverable (0) | `ERROR` |
| 3 | fatal | fatal (1) | `ERROR` |

TC.10.007 injects a **corrected** error. By definition it does not stop the
workload, so publishing it as a failure would be wrong — and any downtime
measured from that alarm would be fiction. This is the same point the correction
request makes as D-02.

---

## Install

From the host. It copies the tools and prepares the kernel state they read.

```bash
scripts/install_guest.sh
```

```
1/4 copy tools, samples and the setup script to /home/vela/ras_ros
2/4 kernel support
      ras tracepoints: present
3/4 tracefs group, instance, clock, log directory
      trace_clock:  local global counter uptime perf [mono] mono_raw boot tai
      event enable: 1
      pipe:         /sys/kernel/tracing/instances/ras_ros/trace_pipe
      log dir:      /var/log/ras
4/4 checking the pipe is readable without root
      OK
```

Add `--service` to install the systemd unit as well. TC.10.007 collects
unattended for the better part of an hour, so the monitor has to outlive the
session that started it — the same argument [`../README.md`](../README.md) makes
for running rasdaemon as a service.

Three pieces of kernel state, none of which survives a reboot, all of which
[`scripts/guest-tracefs-setup.sh`](scripts/guest-tracefs-setup.sh) rebuilds (the
service runs the same script as `ExecStartPre`, so the two cannot drift apart):

| What | Why |
|---|---|
| group `tracing`, tracefs remounted `gid=tracing,mode=750` | tracefs files are `0640 root:root`. Running the node as root instead would give Fast DDS a root-owned participant that does not share `/dev/shm` cleanly with the Nav2 nodes it exists to watch |
| trace instance `ras_ros` | reading `trace_pipe` **consumes** events. rasdaemon already has its own instance; independent ring buffers mean neither can starve the other |
| `trace_clock` = `mono` | detection latency is `read time − event time`. tracefs defaults to `local`, which is not `CLOCK_MONOTONIC`, and subtracting across clock domains gives a plausible number that means nothing |

---

## Check it

```bash
scripts/run_check.sh
```

```
1/2 decode -- parser against the recorded fixtures
      SELF-TEST: PASS
2/2 deliver -- records to /ras/events, on ROS_DOMAIN_ID=71
      PASS  2 events published on /ras/events: got 2
      PASS    TLB severity is ERROR (recoverable): level=2
      PASS  corrected CPU record delivered (what TC.10.007 injects)
      PASS    corrected error is WARN, not ERROR: level=1
      PASS  detection latency measured against CLOCK_MONOTONIC: 2/2
      PASS    worst detection latency under 5000 ms: 9.1 ms (all: 9.1, 4.0)
      PASS  sequence numbers are complete and gapless: [1, 2]
      PASS  JSON log written (events.log): 2 line(s)
      PASS    a heartbeat accounts for every event: counts seen: 0, 2
      RESULT: PASS (18/18 checks)
G-R1: PASS
```

Two stages, because they fail for different reasons: `ras_decode.py --self-test`
is pure Python against the recorded fixtures, so a failure there is a parser
defect and nothing else; `check_ras_monitor.py` then feeds those same records
through a FIFO standing in for `trace_pipe` and asserts they arrive on
`/ras/events`.

Read the latency the way it is meant: **9.1 ms** is the monitor's own
read-to-publish time on an emulated riscv64 guest, against a TC.10.007 detection
budget of 20 s. That is three orders of magnitude of headroom, which is also why
D-03 argues the 20 s criterion cannot discriminate anything.

Everything downstream of the read is the production path — same parser, same
decode, same publisher, same log writer. What it cannot show is that the *kernel*
emits the event: this guest is Config A, so no error can be injected.

---

## Reading an event

An injected TLB error, as it arrives on `/ras/events`:

```
ros2 topic echo /ras/events --once
```

```yaml
status:
- level: 2                       # ERROR -- GHES severity 2, recoverable
  name: ras/trace
  message: "recoverable hardware error: TLB error (data read) on RISCV64"
  hardware_id: vela
  values:
  - {key: error_type_name, value: TLB error}
  - {key: ghes_severity, value: '2'}
  - {key: operation_name, value: data read}
  - {key: proc_isa_name, value: RISCV64}
  - {key: target_address, value: '0'}
  - {key: detect_latency_ms, value: '3.6'}
  - {key: seq, value: '1'}
```

The heartbeat on `/diagnostics` is what makes silence meaningful — without it, a
dead monitor and a quiet machine look identical, and "탐지 누락률 0%" cannot be
asserted from an absence:

```yaml
- level: 0
  name: ras/monitor
  message: alive, 4 event(s)
  values:
  - {key: count_trace, value: '4'}
  - {key: count_kmsg, value: '4'}
  - {key: count_rasdaemon, value: '4'}
  - {key: count_agreement, value: 'true'}
  - {key: clock_comparable, value: 'true'}
```

`count_agreement` is the point of running three sources. The TSP asks for
`EDAC ce_count = 4`, which cannot work on this kernel — `CONFIG_EDAC` is unset,
and a CPU error would not touch a memory-controller counter even if it were set.
CR-4 proposes agreement between the tracepoint, rasdaemon and dmesg instead; this
is the field a script asserts on.

---

## What this closes

| Filed as | Was | Now |
|---|---|---|
| D-10 | `/var/log/ras/events.log` has no producer, and the precondition's `truncate -s0` *creates* it — so the run looks prepared while the evidence file stays empty | the monitor is that producer, in JSON lines |
| D-11 | no kernel → ROS 2 alarm path exists, so TC.04.011 cannot be run | `/ras/events` and `/diagnostics` |
| CR-4 | `EDAC ce_count` is unusable | `count_agreement` across three independent paths |
| CR-9 | absolute timings are not attributable | `trace_clock=mono` enforced and checked; latency is refused rather than faked when the clocks are not comparable |

The correction request still stands as filed — this makes the replacements it
proposes something that exists rather than something requested.

---

## On Config C

Nothing here changes. The check runs the monitor with `--trace-path` pointing at
a FIFO because Config A cannot produce an event; on a RERI-capable guest you drop
that flag and the same node reads the real pipe:

```bash
python3 ~/ras_ros/ras_monitor.py --log-file /var/log/ras/events.log
sudo ~/ms_kim/riscv-vela/vela-apps/ras/inject/inject_cpu_ce.sh -n 4 -i 720
```

Injection is [`../README.md`](../README.md). Expect `count_trace` to reach 4 and
`/ras/events` to carry four `WARN` records — and expect all four to be
*indistinguishable from RERI code 12 or 20*, because the firmware folds twelve
codes onto CPER `0x08` before the kernel ever sees them. That is D-09, it is not
a defect in this node, and no consumer of this topic can work around it.

---

## If it does not work

| Symptom | Cause | Fix |
|---|---|---|
| `[trace] cannot open …: Permission denied` | the login predates `usermod -aG tracing` | log in again; groups are set at session start |
| `[trace] cannot open …: No such file or directory` | the instance is gone — a reboot removes it | `sudo ~/ras_ros/guest-tracefs-setup.sh` |
| node runs, no events, `count_trace` stays 0 | the event is enabled in the *top-level* buffer, not in the instance | `cat /sys/kernel/tracing/instances/ras_ros/events/ras/non_standard_event/enable` must be `1` |
| `detect_latency_note` instead of a latency | `trace_clock` is not `mono` | `echo mono \| sudo tee /sys/kernel/tracing/instances/ras_ros/trace_clock` |
| `[rasdaemon] no non-standard table` | rasdaemon was built without `--enable-non-standard` | `../rasdaemon/install-rasdaemon.sh`, which passes it |
| heartbeat says sources disagree | one path missed an event — which is the finding, not a bug | compare `count_trace`, `count_kmsg`, `count_rasdaemon` and read the JSON log |
| topic silent, no error anywhere | a stale Fast DDS segment | `rm -f /dev/shm/fastrtps_*` and restart |

---

## Environment

| Piece | Value |
|---|---|
| Guest | Ubuntu 24.04 riscv64, kernel 6.18.0-rc5, `CONFIG_RAS=y`, `CONFIG_ACPI_APEI_GHES=y` |
| ROS 2 | Jazzy, `rmw_fastrtps_cpp`, `diagnostic_msgs` from the Vela APT repository |
| Tracing | tracefs instance `ras_ros`, `trace_clock=mono`, `ras:non_standard_event` |
| rasdaemon | 0.8.5 built with `--enable-non-standard`, its own trace instance |
| Injection | not available — this guest is Config A (`wg=on`, no `reri=true`, no pflash pair) |

## Known limitations

- **No error can be injected here.** Config A has no RERI device, so every event
  in every test is a recorded one replayed through the production path. The
  kernel half of the chain is unverified until Config C exists, and that is the
  gating prerequisite for TC.10.007 — not this node.

- **The RERI error code is already gone.** By the time a record reaches this
  node the firmware has folded twelve RERI codes onto CPER `0x08`, so
  `/ras/events` cannot report which one was injected. Reading it back from the
  MMIO register proves what was injected but not what was recorded.

- **Absolute latencies are the emulator's.** `detect_latency_ms` is measured
  against `CLOCK_MONOTONIC` and is honest about what it measures, but QEMU TCG
  has no timing model: the figure describes this host under this load, and is a
  regression baseline rather than a characterisation of RISC-V hardware.

- **The kmsg source is coarser than the tracepoint.** One hardware error is a
  dozen printk lines reassembled by correlation id, and a group is published
  after 2 s of quiet — so its timestamps are later than the tracepoint's by
  design. Use it for the count cross-check, not for latency.

- **`/ras/events` is `VOLATILE`.** A subscriber that starts after an event does
  not receive it. The JSON log is the record; the topic is the alarm.

---

## Files

| File | Role |
|---|---|
| `tools/ras_decode.py` | the parser and CPER decoder. No ROS, no root — `--self-test` runs on any machine |
| `tools/ras_monitor.py` | the node: three sources, two topics, one log |
| `tools/check_ras_monitor.py` | acceptance test — records in, topic out, latency measured |
| `scripts/install_guest.sh` | copies the tools and prepares the guest; `--service` also installs the unit |
| `scripts/guest-tracefs-setup.sh` | the kernel-side state, as one idempotent script the service reuses |
| `scripts/run_check.sh` | gate G-R1: decode, then deliver |
| `samples/*.trace`, `samples/*.kmsg` | recorded records from a RERI-capable guest, restored to full length |
| `systemd/ras-monitor.service` | unattended operation across a reboot |

## See also

- [`../README.md`](../README.md) — injecting the errors this node reports, and the RERI-to-CPER folding
- [`../rasdaemon/`](../rasdaemon/) — the collector this cross-checks against
- [`../workload-nav2.md`](../workload-nav2.md) — the Nav2 workload TC.10.007 runs this beside
- [q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md) — the RAS boot chain
