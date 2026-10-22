# ras/fault-monitor

Fault detection daemon for the RISC-V RAS stack. It reads `/dev/kmsg`, turns the
kernel's own decoded CPER output into one structured JSONL record per fault, and
attributes each record to the **hart** that produced it.

This is part one of a series — see [#6](https://github.com/riscv-vela/vela-apps/issues/6).
It carries the ROS 2-independent core only; the eBPF collector, the ROS 2 bridge,
the remaining detectors and the systemd packaging follow in later pull requests.

---

## Contents

| File | Purpose |
|------|---------|
| `fault.py` | Fault record type and the fault-class table |
| `kmsg.py` | `/dev/kmsg` reader — groups multi-line records, extracts `caller=C<n>` |
| `ghes.py` | GHES/CPER block parser (processor and memory sections) |
| `classify.py` | Maps a parsed record to a fault class and tags; cross-checks hart evidence |
| `topology.py` | Reads the hart ↔ Linux CPU map from `/proc/cpuinfo` at runtime |
| `pipeline.py` | The ROS 2-free core that ties the above together |
| `standalone.py` | Entry point for running without ROS 2 |
| `test/` | pytest suite with a captured `/dev/kmsg` fixture |

`pipeline.py` imports no ROS 2. Detection keeps working when the ROS 2 stack does not.

---

## Why not extend `ras/rasdaemon/`

They answer different questions and both are needed.

| | `ras/rasdaemon/` | `ras/fault-monitor/` |
|---|---|---|
| Input | kernel tracepoints via libtraceevent | `/dev/kmsg` |
| RERI vendor records | stored raw — no decoder for the GUID | parsed into fields |
| Faulting hart | not identified | identified, with three corroborating sources |
| Scope | hardware error collection | hardware, kernel and application layers |

rasdaemon storing raw bytes is not a defect — it has no decoder for this vendor
GUID. The kernel already prints the same record in readable form, and that is
what this daemon reads.

---

## hart ids are not Linux CPU numbers

RERI and GHES report **hart ids**. Linux CPU hotplug uses **CPU numbers**. They are
not the same map: on an `-smp 8` guest, boot hart 7 is CPU 0 and hart N is CPU N+1.

`topology.py` reads the map at runtime and never falls back to identity. Offlining
CPU 6 for "hart 6 faulted" would take down hart 5 while the faulty hart kept running.

---

## Usage

```bash
python3 -m ras_fault_monitor.standalone --out /var/log/ras/faults.jsonl
```

One record, from RERI code 17 injected as a corrected error into hart 3:

```json
{"fault_type": "RERI_FAULT", "severity": "WARN", "source_layer": "hardware",
 "tags": ["reri", "hardware_error", "section:processor", "error_type:0x08",
          "hart:3", "cpu:4"],
 "detail": {"ghes_source": 4, "processor_id": "0x0000000000000003",
            "hart": "3", "hart_source": "processor_id", "linux_cpu": 4,
            "hart_corroboration": {"ghes_source": 3, "printk_caller": 3},
            "detect_delay_s": 0.661937}}
```

The three hart sources are kept side by side rather than overwriting each other; a
disagreement surfaces as `hart_mismatch`. `caller=C<n>` needs `CONFIG_PRINTK_CALLER=y`
and is the only independent check for memory sections, which carry no `processor_id`.

Reproducing the record above needs firmware that fills `processor_id` — see
[vela-fws#3](https://github.com/riscv-vela/vela-fws/issues/3).

---

## Tests

```bash
pytest test/
```

---

## Not included

`puc.py` (CPPC performance counters and frequency) is left out: those values are not
obtainable on the emulation platform.

---

## License

Apache-2.0, matching `ras/inject/`. Every file carries an SPDX identifier.
