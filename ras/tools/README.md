# RAS availability measurement tools

Two tools for the availability test (TSP TC.10.007): a one-hour window owner that
computes the numbers the procedure asks for, and the topic-gap recorder it drives.

| Tool | What it does |
|---|---|
| `availability_monitor.sh` | Owns the measurement clock. Starts the tracker, prints progress every 10 minutes, and on expiry writes `rounds.csv`, `gaps.csv` and `availability.txt`. |
| `downtime_tracker.py` | Subscribes to the monitored topics and writes one CSV row per gap longer than `--threshold-ms`. |

## Usage

```sh
sudo ./availability_monitor.sh \
    --duration 3600 --threshold 99.9 --gap-ms 1000 \
    --outdir /tmp/tc10run \
    --tracker ./downtime_tracker.py \
    --inject-times /tmp/tc10run/inject-times.txt \
    --topics /demo/scan_a /demo/scan_b /demo/scan_c /demo/scan_d
```

The monitor does not inject. Run `../inject/inject_cpu_ce.sh -n 4 -i 720` on its own
schedule and point `--inject-times` at the file it writes, so the injector's clock and
the monitor's clock cannot drift apart.

## What the numbers mean

- `T_inj` comes from the injector's own `[HH:MM:SS]` stamp, not from the monitor.
- `T_det` is the fault record's `timestamp`, which is the kernel's time for the record.
- `T_rec` is `detail.emit_wall_time`, the moment the daemon finished deciding what to do.
  The daemon is observe-only, so nothing is actually repaired. `mttr_s` is therefore the
  response-decision latency, and a report must say so rather than calling it a repair.
- `downtime_s` is the union of monitored-topic gaps over `--gap-ms`, so two topics that
  stall at the same time are counted once.

## Two things that silently produce a wrong result

**Run the tracker as the user that owns the publishers.** Cross-user DDS discovery
succeeds while data delivery does not, so the tracker sees a topic it never receives
from. The monitor passes `--tracker-user` (default `ubuntu`) and pre-creates the output
files, because the output directory is usually root-owned and the tracker would
otherwise die on `EACCES` before the first message.

**Give the workload a lifetime longer than the measurement window.** A publisher that
exits mid-window looks exactly like a gap, so the run fails with no hardware fault
involved. In our own measurement the workload's default lifetime was shorter than the
window and the first attempt recorded 149 gaps before we noticed.

A topic that is subscribed but never delivers is reported in `topics_never_seen` rather
than ignored: a tracker that quietly receives nothing would otherwise report 100 %
availability.

## Output

```
round,T_inj,T_det,det_latency_s,T_rec,mttr_s,downtime_s,topics_ok
1,06:32:18,06:32:18,0.464,06:32:19,0.759,0.000,yes
```

```
  측정 구간      : 3600s (06:12:15 시작)
  주입 시각 수   : 4
  탐지 레코드 수 : 4
  누적 다운타임  : 0.000s
  가용성         : ((3600 - 0.000) / 3600) x 100 = 100.0000%
  기준           : 99.9%  ->  충족
```

## Requirements

ROS 2 (tested on Jazzy), Python 3.12, `bash`. The tracker subscribes with `raw=True`, so
it needs no message class and works for custom `.msg` types.
