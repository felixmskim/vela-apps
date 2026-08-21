# Gazebo + SLAM, split across the host and the guest

A Gazebo simulation whose robot is driven by Nav2 and mapped by slam_toolbox
**inside** the RISC-V guest, with the simulator itself on the x86 host. The robot
actually drives; the map is built from the lidar it sees.

```
   x86-64 host (Docker)                    RISC-V guest (Q-Vela)
   gz sim -s  physics + gpu_lidar          Nav2      plans and drives
   ros_gz_bridge                           slam_toolbox   builds the map
        /clock /scan /odom /tf       ───►   subscribe
                        /cmd_vel   ◄───     publish
             Fast DDS TCPv4, guest dials 10.0.2.2:42100
```

The split is not a workaround — it is the topology the TSP test-environment
figure (§3.1.2, p.16-17) prescribes. Why every choice below is what it is:
[`DESIGN.md`](DESIGN.md). What actually happened when it was run:
[`RUNLOG.md`](RUNLOG.md).

## Layout

| Directory | Holds |
|---|---|
| `scripts/` | the runners — one per gate — plus preflight and interop checks. Run them from this directory |
| `tools/` | all Python: world generation, probes, the scorer. Containers see them as `/work/tools/`, the guest gets its copies scp'd flat into `~/gz_split/` |
| `config/` | Nav2 / slam_toolbox parameters, bridge configs, Fast DDS profiles |
| `artifacts/` | everything a run writes: timestamped logs, generated worlds, `robot.env`, saved maps. Gitignored wholesale |
| `logs/` | curated transcripts and demo media that `RUNLOG.md` cites by name. Gitignored; force-add one deliberately if it is documentation |

## Prerequisites

| Where | Requirement | Check |
|---|---|---|
| host | Docker, and your account in the `docker` group — no `sudo` needed | `docker info` |
| host | ~4 GB disk for the image | `df -h /var/lib/docker` |
| host | outbound HTTPS (Docker Hub, `packages.ros.org`, and Fuel for `tb4`) | `scripts/00-preflight.sh` P2b |
| guest | booted, reachable, ROS 2 Jazzy present | `ssh -p 2222 vela@localhost 'ls /opt/ros'` |
| guest | Nav2 installed per [`../workload-nav2.md`](../workload-nav2.md) | `dpkg -l ros-jazzy-nav2-planner` |
| guest | ~200 MB free for `slam_toolbox` | `df -h /` |

The guest launch command lives in
[q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md) §3.
Run it under `screen -e '^Bb' -S ras` so a dropped ssh session does not take the
VM with it.

## Steps

Ten, in order. Each has an observable outcome to record in
[`RUNLOG.md`](RUNLOG.md); the two check scripts exist because both failures they
catch are silent and expensive.

| Step | Do | Records | Gate |
|---|---|---|---|
| 0 | `scripts/00-preflight.sh` | environment, guest type hashes | GO / NO-GO |
| 1 | `docker build` | image built, apt manifest | — |
| 2 | `docker run` | container up | — |
| 3 | `scripts/check_interop.sh` | version skew, type hashes | interop |
| 4 | guest `apt-get install ros-jazzy-slam-toolbox` | package count, wall time | — |
| 5 | `ROBOT=tb3 scripts/run_rtf.sh` then `ROBOT=tb4 scripts/run_rtf.sh` | real-time factor per robot | **G1** |
| 6 | `ROBOT=tb3 scripts/run_container_loop.sh` ×2 | waypoints, map IoU | **G2 / G2b** |
| 7 | `scripts/run_dds_check.sh` | discovery, round-trip p95 | **G3 / G4** |
| 8 | `ROBOT=tb4 scripts/run_split.sh` | the deliverable | **G5 / G6 / G7** |
| 9 | `EXPLORE=1 scripts/run_split.sh` | the robot chooses its own goals | **G5-E** |
| 10 | `YOLO=1 scripts/run_split.sh` | all three workloads at once | **G-Y5** |

Step 9 needs one more guest package, in the same hand-typed spirit as step 4:
`sudo apt-get install -y ros-jazzy-explore-lite` — 2 packages, 318 kB, because
every C++ dependency is already there. Already installed on this bench; the line is
here for a fresh guest, and `run_split.sh` checks rather than installs.

Steps 1, 2 and 4 are typed by hand rather than scripted, because they are the
three that change state and are worth watching. Everything goes through `tee`:

```bash
mkdir -p logs
scripts/00-preflight.sh 2>&1 | tee logs/p0-preflight.log
```

`ROBOT` defaults to `tb3` everywhere except `run_split.sh`, which defaults to
`tb4` because that is what the report needs. See
[`DESIGN.md`](DESIGN.md#turtlebot3-or-turtlebot4).

## Build — host

Run from **this directory** and mount it as `/work`. There is deliberately no
`COPY`: the container sees the version-controlled files directly, so what ran is
what is committed, and a fix made mid-run is a fix to the repository. Everything a
run writes lands under `artifacts/`, which git ignores wholesale.

```bash
cd <repo>/ras/gz-split
mkdir -p logs

docker build -t vela-gzsim:jazzy . 2>&1 | tee logs/s1-build.log
```

The base image is pinned by digest inside the `Dockerfile`. If your host account
is not uid 1000, add `--build-arg HOST_UID=$(id -u) --build-arg HOST_GID=$(id -g)`
— the container runs as that uid so its output in `/work` is yours, not root's.

```bash
docker run -d --name vela-gz \
  --network host \
  --shm-size=512m \
  --init \
  -v "$PWD":/work -w /work \
  vela-gzsim:jazzy
```

No trailing command: the image's `CMD` is `sleep infinity`. All three flags are
load-bearing:

| Flag | Why |
|---|---|
| `--network host` | `10.0.2.2` reaches listeners in the **host** network namespace only. In a bridge namespace the guest can never connect. |
| `--shm-size=512m` | Docker's default 64 MB `/dev/shm` is too small for the Fast DDS shared-memory transport, which fails in ways that look like discovery problems. |
| `--init` | every run starts nodes with `docker exec -d`, and `sleep` as PID 1 never calls `wait()`, so orphans stay `<defunct>` for the life of the container. `--init` runs a real reaper. Harmless to omit — zombies hold a PID slot and nothing else, and an empty cmdline means they never match the scripts' `pgrep` checks. |

With the container up, keep the two records the image carries:

```bash
docker images vela-gzsim:jazzy                                    # size
docker exec vela-gz id                                            # expect uid=1000(vela)
docker exec vela-gz cat /opt/vela/base-image.txt                  # base digest
docker exec vela-gz cat /opt/vela/apt-manifest.txt > logs/s1-manifest.txt
```

`logs/s1-manifest.txt` is the reproducibility artefact: apt versions are not
pinned with `=` (the ROS repo keeps only the newest build, so a pin fails rather
than reproduces), so what the build resolved to is recorded instead.

For `ROBOT=tb4`, seed the Fuel cache so the first `gz sim` start is not also a
download. The cache lives in the container, so repeat this after any
`docker rm`. `tb3` needs none of it — `tb3_sandbox` has no remote dependency.

```bash
docker exec vela-gz gz fuel download \
  -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"
```

## Build — guest

```bash
ssh -p 2222 vela@localhost

# Suppress man-db: regenerating the index costs more emulated wall time than
# unpacking the packages does.
sudo tee /etc/dpkg/dpkg.cfg.d/01-nodoc >/dev/null <<'EOF'
path-exclude=/usr/share/man/*
EOF
sudo dpkg-divert --local --rename --add /usr/bin/mandb
sudo ln -sf /bin/true /usr/bin/mandb

sudo apt-get install -y ros-jazzy-slam-toolbox     # 192 packages, 165 MiB

# ROBOT=tb4 only: the guest runs robot_state_publisher to get the
# base_link->shell_link->rplidar_link chain that Gazebo's /tf does not carry.
sudo apt-get install -y ros-jazzy-robot-state-publisher

mkdir -p ~/gz_split
```

Then from the host:

```bash
scp -P 2222 config/nav2_split.yaml config/mapper_params_tcg.yaml \
            config/guest_tcp_client.xml tools/slam_nav.launch.py \
            tools/send_goal.py tools/liveliness_watchdog.py tools/rtt_probe.py \
            tools/score_map.py tools/latency_probe.py \
            vela@localhost:~/gz_split/
```

## Run

Four scripts, one per gate, in this order. Thresholds and the reasoning behind
them: [`DESIGN.md`](DESIGN.md#gates).

```bash
# G1 — is headless Gazebo fast enough, and is the lidar really rendering?
ROBOT=tb3 scripts/run_rtf.sh 2>&1 | tee logs/g1-rtf-tb3.log
ROBOT=tb4 scripts/run_rtf.sh 2>&1 | tee logs/g1-rtf-tb4.log
```

Generates the world and robot, starts `gz sim -s --headless-rendering`, spawns
the robot, reads `real_time_factor`, and counts finite ranges in the first
`/scan` — a topic that exists but publishes only `inf` means the render path is
dead, which a topic listing cannot distinguish from a working lidar. It also
prints the render-path evidence (`eglinfo`, `glxinfo`, the engine's own
initialisation lines); copy that into `RUNLOG.md`. Pass: RTF ≥ 0.30. Below 0.10
is a kill — take the bag fallback and attach the number to defect D-14.

```bash
# G2 / G2b — close the loop with no DDS in the way
ROBOT=tb3 scripts/run_container_loop.sh 2>&1 | tee logs/g2-loop-tb3.log
ROBOT=tb3 RTF_LIMIT=0.5 scripts/run_container_loop.sh 2>&1 | tee logs/g2b-loop-tb3.log
```

Gazebo, Nav2 and slam_toolbox all inside the container. Drives the waypoint tour,
saves the map with `map_saver_cli`, scores it with `score_map.py`. Pass: every
waypoint reached **and** `precision ≥ 0.80`, `recall ≥ 0.85`, `coverage ≥ 0.90`,
at **both** real-time factors.

Three map numbers, not one, because each catches a different failure and no other
number sees it: precision catches free space marked as wall, recall catches an
eroded map, coverage catches a run that died early. Override with
`MIN_PRECISION` / `MIN_RECALL` / `MIN_COVERAGE`; the floors are per-world and come
from the generated `artifacts/robot.env` — coverage 0.90 for tb3 and **0.30 for tb4**,
both measured, and tb4's precision floor is 0 (printed, not gated) because on
`depot.pgm` precision tracks which region the tour observed rather than map quality. Rationale and the fault-injection table: [DESIGN.md](DESIGN.md).

```bash
# G3 / G4 — does ROS 2 cross SLIRP?
scripts/run_dds_check.sh 2>&1 | tee logs/g3g4-dds.log
```

SLIRP forwards no multicast, so simple discovery cannot cross; the two Fast DDS
profiles turn the guest into a TCP client that dials `10.0.2.2:42100`. No
`hostfwd` rule and no QEMU restart — measured, not assumed: of the four fallback
rungs the design anticipated, only `<wan_addr>` was needed. Pass: a topic crosses
within 120 s, round-trip p95 < 500 ms.

Copy the guest's files first, and expect discovery to take tens of seconds — that
is TCG, not a fault:

```bash
scp -P 2222 config/guest_tcp_client.xml config/nav2_split.yaml \
    config/mapper_params_tcg.yaml tools/rtt_probe.py tools/slam_nav.launch.py \
    tools/send_goal.py tools/liveliness_watchdog.py tools/clock_lag_probe.py \
    vela@localhost:~/gz_split/
```

`run_split.sh` re-pushes `tools/*.py` and `config/*.yaml` on every run. It used to
push only the two probes, on the argument that those are what its conclusions rest
on — true, and too narrow: a Nav2 parameter fix once sat in the repository while
the guest ran a two-day-old copy, and the run failed with the exact symptom the fix
removes. `score_map.py` runs on the **host**, where G2's reference already is.

```bash
# G5 — the deliverable
ROBOT=tb4 scripts/run_split.sh 2>&1 | tee logs/g5-split-tb4.log
```

Gazebo and the bridge on the host, Nav2 and slam_toolbox in the guest. Waits for
lifecycle activation (up to 900 s — every TF lookup now crosses TCP), drives the
tour, saves and scores the map, reports co-tenant CPU (G6) and optimizer stalls
(G7). Pass: every waypoint SUCCEEDED, the same three map floors as G2, no `/scan`
or `/odom` gaps.

Two settings decide whether the robot moves at all, and both are about giving an
emulated guest room rather than about Nav2:

```bash
WATCHDOG= RTF_LIMIT=0.5 CLOCK_LAG_MAX=8 ROBOT=tb3 scripts/run_split.sh
```

| Setting | Effect | When to use it |
|---|---|---|
| `WATCHDOG=` | drops `liveliness_watchdog.py`, which measured 69% of an emulated core — the largest consumer in the guest, larger than any Nav2 node | any run that is not measuring G6 |
| `RTF_LIMIT=0.5` | halves sim time per wall second, so a node that cannot keep up gets twice as long | whenever the clock lag approaches `CLOCK_LAG_MAX` |

With both, the clock lag went from the 30–42 s that aborted every earlier run to
0.5–3.1 s, and bringup from 90 s to 60–70 s. G6 needs the watchdog, so a
measurement run keeps it and pays the 69%.

### The two drive modes the TSP figure names

The figure labels its two modes `Host Teleop(manual)` and
`Explore_lite(autonomous)`. Both exist; neither is the default, because `send_goal.py`'s
fixed waypoint list is what makes a run comparable to the previous one.

```bash
# G5-E — the autonomous mode: the robot picks its own goals
EXPLORE=1 WATCHDOG= RTF_LIMIT=0.5 CLOCK_LAG_MAX=8 ROBOT=tb4 scripts/run_split.sh
```

`EXPLORE=1` swaps the waypoint list for `explore_lite`, which finds frontiers in
slam_toolbox's `/map` and sends them to the same `navigate_to_pose` server Nav2
already serves. That shared server is why the two modes are mutually exclusive —
two clients would preempt each other's goals — so step 8 is an if/else, not two
calls.

**It cannot start itself, and that is why `EXPLORE_SEED` exists.** slam_toolbox's
`minimum_travel_distance` is 0.5 m, so a robot that has not moved contributes no
further scan and `/map` stays empty — measured here: 0 occupied cells, 99.1% unknown.
`explore_lite` takes its goals *from* that map, so the only frontier it can find is
two or three cells away, and it proposes the robot's own position forever: 554 goals,
every one 0.13 m off, robot stationary throughout. `EXPLORE_SEED=2` drives the first
two waypoints first, so slam_toolbox has something to map before exploration takes
over. Set `EXPLORE_SEED=` to watch the deadlock instead — it is instructive, and the
run prints a named diagnosis rather than leaving you to find it.

Pass: `EXPLORE_MIN_GOALS` (2) goals **more than 0.5 m from where the robot was when
Nav2 accepted them**, plus the same map floors. The distance qualifier is the gate: a
bare goal count called 554 stationary goals a success. Reaching `EXPLORE_TIMEOUT`
(900 s wall) is **not** a failure — the map is still scored.

The count comes from Nav2's own `Begin navigating` lines, not from `explore_lite`'s
log: its `Goal ACCEPTED` literal exists in the binary but never reached the redirected
log, so a gate reading that counted 0 while Nav2 was accepting hundreds.

Parameters and the reason for each: [`config/explore_split.yaml`](config/explore_split.yaml).
Two of them are worth knowing before reading a result:

| Setting | Default | |
|---|---|---|
| `planner_frequency` | 0.2 | **wall** Hz. Verified from the binary's typeinfo, not assumed — `RTF_LIMIT` does not stretch it |
| `progress_timeout` | 150.0 | chosen to exceed Nav2's own 120 s progress checker under either clock, because which clock it uses cannot be read out of a stripped binary |
| `min_frontier_size` | 0.30 | metres. Upstream's 0.75 is wider than the 0.44 m gap the robot can actually drive through, so it discards reachable frontiers |
| `costmap_topic` | `/map` | slam_toolbox's grid, not Nav2's costmap: it is the same grid the map gate scores, and it costs a fifth of the deserialisation |

Coverage is the number this mode exists to move — a four-waypoint tb4 tour reaches
0.387 of depot — so an `EXPLORE` run is the one worth raising `MIN_COVERAGE` for.

**Status: not yet passing, and the honest reason.** With the seed the churn is gone,
but `explore_lite` then issues no goals at all — connected to the action server,
holding 47.7% of an emulated core, six minutes, nothing. The run reports that plainly
(`explore_lite: 0 goal(s) accepted`) and the map gate fails it on coverage 0.196. Two
of the three obstacles found are fixed in the rig; this one is open. See
[`RUNLOG.md`](RUNLOG.md) — do not read a `G5-E` row as a demonstrated capability until
one exists.

```bash
# Host Teleop (manual) — the demo image, the run's ROS domain, and a TTY
docker run --rm -it --network host vela-gzsim:demo bash -lc \
  'source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42 &&
   ros2 run teleop_twist_keyboard teleop_twist_keyboard'
```

Three things in that line are load-bearing. `--network host` puts the container in
the same network namespace as `vela-gz`, which is how one container's publisher
reaches the other's subscriber at all — `run_demo.sh` relies on the same thing for
RViz. `ROS_DOMAIN_ID=42` is the domain the whole rig runs on; without it teleop
publishes into an empty graph and nothing moves, with no error. `-it` is required
because the node reads keystrokes from a TTY, so `docker exec -d` cannot drive it.
No Fast DDS profile is needed: teleop only *publishes*, and its consumer is the
host-side bridge, so it never needs a TCP acceptor of its own.

Manual driving is a **host** publisher, exactly as the figure places it: `/cmd_vel`
is bridged `ROS_TO_GZ` on this side, so keystrokes reach Gazebo's `DiffDrive`
without crossing the TCP link at all. There is no riscv64 build of
`teleop_twist_keyboard` either, which settles where it belongs. It ships in
`Dockerfile.demo` rather than the measurement image so the recorded apt manifest
that `RUNLOG.md` cites stays the environment G1–G7 were measured in.

**Do not teleop into a Nav2 goal.** There is no arbitration on `/cmd_vel`: the
bridge forwards whatever arrives and `DiffDrive` re-applies the newest command
every physics step. `controller_server` publishes at 5 Hz, so each keystroke steers
for ≤200 ms of sim time and is then overwritten, and Nav2 sees only unexplained
tracking error. Drive manually against a stack that is up but idle — or against
`run_demo.sh --standalone`, which brings up world and robot with nothing driving.

`cmd_vel_watchdog.py` makes manual driving behave the way a person expects: it
zeroes `/cmd_vel` after 2 s of sim-time silence, so releasing the keys stops the
robot rather than leaving `DiffDrive` latched on the last command forever.

### What "topic latency" can and cannot mean here

The TSP gates on topic latency — 1,000 ms in the 3rd year, 500 ms in the 4th, on
`/scan`, `/odom`, `/cmd_vel`. **Publish-to-subscribe latency is not measurable
across this split, and the plausible way of getting it is a trap worth naming.**

Every topic the TSP lists crosses the machine boundary, and the two machines' clocks
are not synchronised, so DDS `source_timestamp` differences carry the offset — which
is what [`tools/latency_probe.py`](tools/latency_probe.py) says about itself and why
it is only valid same-machine. Using `header.stamp` against the guest's `now()` looks
like it escapes that, because both sides do share one sim clock. It does not: the
guest's `now()` is a sample-and-hold of the last `/clock` sample the guest managed to
service, and `/clock` arrives over the very link being measured into a CPU-starved
node. Such a probe reports

```
now()_guest − header.stamp  =  RTF × L_transport  −  LAG_sim
```

and `LAG_sim` is what step 7 already gates on: 0.5–3.1 s in passing runs against a
wanted signal of 5–25 ms. It would measure the negative of the clock lag.

What is measurable, and what the rig now reports:

| Question | Instrument | Clocks |
|---|---|---|
| did a consumer ever go without data longer than the budget | `liveliness_watchdog.py`, inter-arrival p50/p99/max per topic | one, `CLOCK_MONOTONIC`, in the consumer |
| same for `/cmd_vel`, which has no `Header` at all | `cmd_vel_watchdog.py` on the host, port 42102 | one, in the host consumer |
| what the crossing itself costs | `rtt_probe.py` — G4, p95 **12.1 ms** | one, guest→host→guest |

```bash
# the interval measurement: needs the watchdog, so it costs 69% of a core
ROBOT=tb4 WATCHDOG=1 RTF_LIMIT=0.5 scripts/run_split.sh
```

`TOPIC_BUDGET_MS` defaults to `TSP_BUDGET_MS` (500) **divided by `RTF_LIMIT`**, and
the scaling is not a fudge: the sensors run on sim time while the instrument measures
wall time, so `/scan` at 5 Hz of sim is 400 ms of wall at `RTF_LIMIT=0.5` and an
unscaled 500 ms budget would fail on jitter with nothing wrong. State the RTF beside
any number quoted from this — a wall interval at RTF < 1 is not the TSP's figure, it
is the TSP's figure for a simulator running that slowly.

```bash
# G-Y5 — all three workloads the TSP names, at once
YOLO=1 WATCHDOG= RTF_LIMIT=0.25 ROBOT=tb3 scripts/run_split.sh
```

**`RTF_LIMIT=0.25`, not 0.5.** Three workloads in four emulated vCPUs do not fit
at half speed: measured 2026-08-19, the detector took 112% of a core, one
inference stretched from 11.4 s to 31.5 s, and `bt_navigator` fell 7.3 s behind
its own TF buffer and aborted every goal. At quarter speed the same run reached
both waypoints and produced the best map this rig has made.

`YOLO=1` adds the detector as a third co-tenant: a camera and photo billboards
spawned into the same running world, one extra entry appended to a **copy** of
the bridge config, and `yolo_node` started in the guest with the Nav2 stack — so
its CPU is present during bringup and the drive, which is the point. Pass: G5's
own floors, plus at least one detection.

Off by default. G5/G6/G7 were measured without it and the numbers above describe
that configuration; a co-tenant that changed them silently would make every
earlier `RUNLOG.md` entry unreadable.

The camera scene is spawned at `(30, 0)`, well clear of the arena, and that is
not tidiness. A `gpu_lidar` samples the render scene, so a billboard is an
obstacle to it even with no collision geometry — inside the arena it would land
in the SLAM map, the map gate would fall, and the cause would read as a mapping
defect. What that costs in honesty: this is **co-tenancy, not perception for
navigation**. The detector shares the guest's CPU, the DDS link and the simulator
with Nav2 and SLAM; it is not a robot reacting to its own camera.

| Setting | Default | |
|---|---|---|
| `YOLO_ASSETS` | `~/ms_kim/yolo-artifacts/assets` | the billboard photographs; nothing binary is committed |
| `YOLO_RATE` | `0.1` Hz | one 320×320 inference costs 11.4 s on this guest, so a faster camera only spends TCP bandwidth on frames the node drops |
| `YOLO_X` / `YOLO_Y` | `30` / `0` | where the scene is spawned |

## Demonstrating it

Everything above is headless because that is what can be measured. For showing
the simulation to people there is a viewer layer, built once:

```bash
docker build -f Dockerfile.demo -t vela-gzsim:demo .
```

The Gazebo server and the Gazebo GUI are separate processes that find each other
over gz-transport, so the viewer attaches to a run that is already going. Two
terminals:

```bash
# 1. the run, built so it can be watched (adds SceneBroadcaster)
GUI=1 ROBOT=tb3 scripts/run_container_loop.sh   # or GUI=1 ROBOT=tb4 scripts/run_split.sh

# 2. the viewer: Xvfb + gz sim -g + VNC on 127.0.0.1:5900
scripts/run_demo.sh
```

### Showing what the guest believes

The Gazebo GUI shows ground truth — the robot moving through the world. It cannot
show the map slam_toolbox built, the path Nav2 planned, or what the detector
found, because those are the **guest's** opinions and they come back over the TCP
link. For a demonstration of all three workloads, add RViz on the same display:

```bash
RVIZ=1 scripts/run_demo.sh
```

RViz needs its **own** host listening port, 42103, and the split runner has to give
the guest a locator for it — `GUI=1` does that. Until 2026-08-19 RViz was started on
`host_tcp_server.xml`, which declares the bridge's 42100, so in a split run it came
up with no TCP acceptor and the guest had nothing to dial: `/map`, `/plan` and
`map->odom` never arrived, and since the fixed frame is `map`, the pane was blank
including `/scan`. Two ports, two profiles, one locator each: 42103
[`host_rviz_server.xml`](config/host_rviz_server.xml) for RViz, 42104
[`host_overlay_server.xml`](config/host_overlay_server.xml) for
`detection_overlay.py`, which used to share RViz's and take the bind from it.

### Recording the split topology

The full recipe, and the timings are measured rather than estimated:

```bash
# terminal 1 -- the run. GUI=1 builds the world with SceneBroadcaster AND generates
# the guest profile that carries the 42103/42104 locators. PRE_DRIVE_SLEEP holds at
# step 7 so the recorder is live before the robot moves.
WATCHDOG= RTF_LIMIT=0.5 GUI=1 ROBOT=tb3 PRE_DRIVE_SLEEP=120 scripts/run_split.sh

# terminal 2 -- the moment terminal 1 prints "7/10"
RVIZ=1 ROBOT=tb3 GEOMETRY=1920x1080 FOLLOW_OFFSET_OVERRIDE=-3.0,0,2.0 \
  scripts/run_demo.sh --record --duration 200
```

| stage | measured |
|---|---|
| run start to `7/10` | 4 min 8 s (of which Nav2 bringup 1 min 53 s) |
| viewer launch to ffmpeg live | 1 min 35 s |
| the drive itself | 2 legs, 30 s and 33 s, at RTF 0.5 |
| file | 1920x1080 H.264 yuv420p, 15 fps, ~4.8 MB for 200 s |

**`ROBOT=tb3`, not tb4, for a recording.** tb4's depot world pulls its geometry from
Fuel, and that 101 MB cache lives in `vela-gz`'s writable layer only — the viewer
container is created fresh every run, so it has nothing to render the depot from.
tb3's world comes from the `nav2_minimal_tb3_sim` package, which is in the image.

**The layout is not optional.** Xvfb has no window manager, nothing in
`Dockerfile.demo` provides one, and the last window mapped sits on top: untiled,
RViz covered the Gazebo GUI completely and the recording showed a map with nothing
driving. `run_demo.sh` tiles both windows with `xdotool` after RViz is up. Two
further layout defects were measured off a real frame and fixed the same way:
RViz's Displays and Views docks plus the Image widget took ~450 px of an 864 px
pane, so a recording uses [`demo_nav.rviz`](config/demo_nav.rviz), which has no
panels and no Image display, whether or not the detector is running; and the
Gazebo GUI's Component inspector and Entity
tree took ~270 px to show a solver name and five entities, so
[`demo_gui.config`](config/demo_gui.config) drops them and keeps
`CameraTracking` — without that plugin `/gui/follow` does nothing and the robot
drives out of frame.

[`config/demo.rviz`](config/demo.rviz) is the third layout and `run_demo.sh` never
selects it: it is for driving RViz by hand over VNC, with the docks that let you
tick displays on and off. It cannot film a detector run — its Image display is
`Enabled: false` with the dock `collapsed: true`, and the only documented way to
undo either is to click in a Displays panel that a headless Xvfb display has
nobody to click. That is why the detections get a window instead.
`ros-jazzy-rviz2` is in `Dockerfile.demo` rather than the measurement image, for
the same reason `x11vnc` is — a GUI run is not a measurement run.

RViz has no display for `vision_msgs/Detection2DArray`, so a working detector
looks on screen exactly like a broken one. `../yolo/tools/detection_overlay.py`
draws the boxes and shows them.

**`OVERLAY=1` starts it, not `RVIZ=1`** — they are separate switches, and until
2026-08-19 both this file and `run_demo.sh`'s own comment said otherwise. With
`RVIZ=1` alone the overlay never starts and nothing in the recording shows the
detector; `run_demo.sh` now says so before a take rather than after one.

It runs on the **host**. The frames are already here — Gazebo produced them — and
the guest is a four-vCPU emulated machine already carrying three workloads, so it
publishes only the small `Detection2DArray` and the drawing happens on this side.
Frames are matched to detections by timestamp, not "newest wins": one inference
takes tens of seconds here, so the newest frame is not the one the boxes describe.

The annotated frame goes two places: `/detection_image`, for anything that wants
to subscribe, and **its own window on the recorded display**. The window is why
the detector is visible at all, and it is a window rather than RViz's Image
display because of what RViz does with a single dock: measured on a 1920x1080
frame, an 864 px pane gave the Image dock ~320x1000 for a 320x240 image, so a
third of the frame became empty grey and the map lost 300 px to it
(`artifacts/rviz-yolo-smoke.png`, local to the run). A window is
tiled by the same `xdotool` call as the other two, costs the map nothing, and
lives inside the overlay process — so it needs no second Fast DDS participant and
no second listening port. `OVERLAY_GEOM`, `OVERLAY_POS` and `OVERLAY_SCALE` move
and size it; the default is 640x480 at the bottom-left of the Gazebo pane.

Verified before the first take, on a bare Xvfb with a stand-in publisher: the
window maps, `xdotool` finds it by title, and the boxes and labels render
edge-to-edge (`artifacts/rviz-yolo-pip-smoke.png`).
`WINDOW_GUI_NORMAL` is set on it for a measured reason — cv2's Qt build otherwise
adds a zoom toolbar that took ~28 px and letterboxed the image inside the rest.

Note what RViz costs the guest: `/map` and `/plan` are guest publishers, so an
extra subscriber is extra work on the emulated side. `/scan` is not — it comes
from the host bridge and RViz reads it locally.

Verified 2026-08-19 on a bare Xvfb: the config loads, all six displays register
and RViz reports ready at 10 fps under llvmpipe. The detections panel opens small
— drag it larger and **File → Save Config** if an audience has to read it from
across a room.

`x11vnc` binds `127.0.0.1:5900` on the server only, so watching it means an ssh
tunnel. Two steps, both **on the machine with the screen** — your laptop, not the
server:

```bash
ssh -N -L 15900:localhost:5900 <user>@<server LAN address>   # 1. leave running
#                    then point any VNC viewer at localhost:15900   # 2.
```

`run_demo.sh` prints that line with the address filled in. Step 1 asks for a
password, then **prints nothing and never returns — that is success**: `-N` means
"no remote command", so an idle prompt is the tunnel holding open. Do step 2 in a
different window.

Only the remote half (`5900`) is fixed; the local half is arbitrary, and `15900`
is the default because `5900` is often unavailable on the client. Client-side
failures and what they mean:

| Symptom | Cause and fix |
|---|---|
| `Could not resolve hostname cslab192` | the server's hostname exists only in its own `/etc/hosts`, as `127.0.1.1`. Use the LAN address |
| `connect to host 169.254.x.x port 22: Unknown error` | a link-local BMC/USB NIC address, not routable. This host has five IPv4 addresses and only one is reachable, which is why the script reads `$SSH_CONNECTION` instead of guessing — `hostname -I` lists the BMC NICs first |
| `bind [127.0.0.1]:15900: Permission denied` | the **client** refusing the local port. Windows reserves ranges for Hyper-V / WSL2 / Docker Desktop — see `netsh interface ipv4 show excludedportrange protocol=tcp` — so pick another, e.g. `-L 25900:localhost:5900` |
| tunnel is up but the viewer shows nothing | check the server end answers: `python3 -c "import socket;print(socket.create_connection(('127.0.0.1',5900),timeout=4).recv(12))"` should print `RFB 003.008` |

| Variant | What it does |
|---|---|
| `scripts/run_demo.sh --standalone` | no run in progress — brings up world + robot itself, nothing driving |
| `scripts/run_demo.sh --shot` | one PNG into `artifacts/`, then exits. For the report |
| `scripts/run_demo.sh --record` | live VNC **and** an mp4 in `artifacts/` |
| `FOLLOW=0 scripts/run_demo.sh` | leave the GUI on its default camera |
| `VIEW_PORT=25900 scripts/run_demo.sh` | change the local port it tells you to tunnel |
| `RVIZ=1 scripts/run_demo.sh` | RViz beside the Gazebo GUI — what the **guest** believes |
| `OVERLAY=1 RVIZ=1 scripts/run_demo.sh` | and the detector's own window. Needs `YOLO=1` on the run |

If the tunnel cannot be made to work at all, `--record` is the better demo
anyway: an mp4 cannot fail in front of an audience. Do **not** drop `-localhost`
from `x11vnc` to reach it directly — it runs `-nopw`, so that publishes an
unauthenticated desktop on the lab network.

The viewer follows the robot automatically (`/gui/follow`, offset from
the generated `robot.env`) — without that the GUI opens on a default pose where a 0.28 m
TurtleBot3 is about 14 px wide and looks like it failed to render.

Two things to know before doing this in front of anyone:

- **`GUI=1` is required.** Both shipped worlds default `headless=true`, which
  removes `SceneBroadcaster`; a GUI then connects successfully and draws an empty
  world forever. `run_demo.sh` refuses rather than showing you that. Verified on
  this host: with SceneBroadcaster on, `/world/<name>/scene/info` exists; without
  it, it does not.
- **A GUI run is not a measurement run.** Measured here: the GUI took 20 cores of
  llvmpipe (2007 %CPU) against the server's 20.5 %CPU. On this 192-core host the
  server was unaffected (`real_time_factor` stayed 1.00), but that will not hold
  on a smaller machine. Numbers in [`RUNLOG.md`](RUNLOG.md) come from headless
  runs only.

### Recording it with the detector

`YOLO=1` on the run and `OVERLAY=1` on the viewer. Four values differ from the
recipe above and each one has a run behind it:

```bash
# terminal 1
YOLO=1 WATCHDOG= RTF_LIMIT=0.25 GUI=1 ROBOT=tb3 HOLD=1 PRE_DRIVE_SLEEP=180 \
  WAYPOINTS_OVERRIDE="1.83,0.08 -1.88,0.08 1.83,0.08 -1.88,0.08" \
  scripts/run_split.sh

# terminal 2 -- the moment terminal 1 prints "holding 180s before driving"
OVERLAY=1 RVIZ=1 ROBOT=tb3 GEOMETRY=1920x1080 scripts/run_demo.sh --record --duration 420
```

| Changed | Why |
|---|---|
| `RTF_LIMIT=0.25`, not 0.5 | at 0.5 with the detector, Nav2 aborted both goals on a TF lookup 7.3 s behind its own buffer. G-Y5 attempt 1 |
| `HOLD=1` | the run's EXIT trap kills the Gazebo **server** the instant the verdict prints, and `run_demo.sh` watches the **viewer's** GUI, so the recording would carry on over a frozen scene. Take 3 was timed to finish first; this run is roughly twice as long and that is not a bet worth taking |
| no `FOLLOW_OFFSET_OVERRIDE` | `-3.0,0,2.0` puts the camera outside a 4.5 m walled arena and the wall then hides the robot. Take 2 |
| `--duration 420`, not 200 | the drive is twice as long at half the rate, and the detections refresh every ~40 s — a 200 s take shows the boxes appear once and then look like a still |

That refresh interval is the camera's, not the detector's: 0.1 Hz of sim time at
RTF 0.25 is one frame every 40 s of wall clock, and one inference is 19-34 s under
co-tenancy, so the node waits on frames rather than the reverse. Raising
`YOLO_RATE` makes the detector the bottleneck instead and puts the per-frame
deserialisation cost back that shrinking the camera removed — do not do it without
re-measuring G6.

**Post-processing is `setpts=0.25*PTS`, not Take 3's `0.5`.** The factor undoes the
real-time factor, so it has to match the `RTF_LIMIT` the take was filmed at. A
420 s take becomes 105 s at real speed.

## Results

G1–G4 measured 2026-08-17, G5–G7 on 2026-08-19. Long form in
[`RUNLOG.md`](RUNLOG.md).

| Gate | Measure | Threshold | Observed | Verdict |
|---|---|---|---|---|
| G1 tb3 | `real_time_factor` | ≥ 0.30 | **1.0000**, 360/360 finite | **PASS** |
| G1 tb4 | `real_time_factor` | ≥ 0.30 | **1.0000**, 336/360 finite | **PASS** |
| G2 | waypoints + precision / recall / coverage | all and 0.80 / 0.85 / 0.90 | **2/2**, 0.910–0.932 / 0.953–0.959 / 0.959–0.983 at RTF 1.0000 | **PASS** |
| G2b | same outcome at a second RTF | consistent | **2/2**, 0.934–0.994 / 0.971–0.980 / 0.973–0.977 at RTF 0.5000 | **PASS** |
| G3 | topic crosses SLIRP host → guest | < 120 s | **38 s** | **PASS** |
| G4 | round-trip p95 | < 500 ms | **12.1 ms**, 200/200, 0 lost | **PASS** |
| G5 | split live drive + map score | all and the same three floors | **2/2 waypoints** twice; 0.937–0.955 / 0.969–0.979 / 0.979–0.983; bringup 65–70 s | **PASS** |
| G6 | slam_toolbox + Nav2 combined %CPU | < 250 of 400 | **249% and 252%**, loadavg 24.96 | **PASS** |
| G7 | longest loop-closure / Ceres stall | < 20 s | **0 stalls** | **PASS** |
| G-Y5 | Nav2 + SLAM + YOLO together | ≥ 1 detection, G5's floors still met | **2/2 waypoints, 0.992 / 0.990 / 0.973, 9 detections**, 272% of 400% | **PASS** |
| G-Y5 **filmed** | the same three, with a viewer and a recording attached | as above | **4/4 waypoints, 0.961 / 0.969 / 0.999, 24 passes**, 291% of 400%, `infer_s` 14.1 / 21.4 / 32.0 | **PASS** |
| G5 **tb4** | split live drive + map, the TSP's 4th-year robot | all waypoints, recall / coverage floors | **4/4 waypoints** at 16–19 s per leg, recall 0.968, coverage 0.387, bringup 123 s | **PASS** |
| G5-L | longest inter-arrival per gated topic | max < 1000 ms wall (TSP 500 ms ÷ RTF 0.5) | `/scan` **1557 ms** over, on the run carrying the watchdog. `/odom` 551 ms — but see below, that number was capped by the instrument | **FAIL** |

G1–G4 and G5/G6/G7 for tb3 were measured before the `MAP_OFFSET` defect below was
found; none of them used the affected reference, so none of the numbers move.

**The filmed G-Y5 row is the first run of `GUI=1` and `YOLO=1` together**, and the
19 points of extra load over the headless run are not the detector's: `yolo_node`
went 81.8% to 82.5% while the Nav2 and slam_toolbox nodes took the rest, from four
waypoints instead of two and from RViz and the overlay subscribing to guest
publishers. Take 4 in [`RUNLOG.md`](RUNLOG.md) also lists what the union of those
two switches broke, all of it silent: the RViz layout that would have recorded the
detector as nothing at all, and a camera failure that took 17 minutes to surface.

**The tb4 rows are what the TSP's 4th-year scenario asks for**, and they cost one
defect each in the tooling rather than in Nav2 or SLAM: the map offset was derived
from the spawn pose when `depot.pgm`'s origin is already spawn-relative (precision
0.421 → 0.792 once corrected), and the tb4 bridge config still bridged the raw
`/clock` and `/imu` that the tb3 config had dropped on 08-17. Full account:
[`RUNLOG.md`](RUNLOG.md).

**G5-L is a real breach and is reported as one.** `/scan` reached 1557 ms between
messages against a 1000 ms budget, on the one run carrying `liveliness_watchdog.py`
— 57% of an emulated core. The same run missed a waypoint that the identical run
without the watchdog reached, so all of it is one fact: the instrument is a quarter
of the guest. p99 was 561 ms, so this is a tail event, not the steady state.

**And `/odom`'s 551 ms is not evidence that `/odom` was fine.** An adversarial review
of the instrument found that its silent-topic detector re-stamped the arrival time it
also used for the interval series, capping the largest interval it could ever report
at about `2 × period × SLACK`. For `/odom` that ceiling was ~0.9 s — *below* the
1000 ms budget — so the gate could not have failed whatever the link did. Two related
defects sat beside it: `/odom`'s nominal period was 20 Hz where the rig publishes 10,
and the thresholds were sim-time values compared against wall-time arrivals. All three
are fixed ([`RUNLOG.md`](RUNLOG.md) has the list), and the numbers above are therefore
**`/scan`: a real breach; `/odom`: not measured.** Re-measurement needs another
`WATCHDOG=1` run.
`tb4` precision is deliberately **not** gated — see `make_world.py`, and the two-run
table in [`RUNLOG.md`](RUNLOG.md) for why it tracks exploration geometry rather than
map quality.

**G2b earned its keep.** It failed 5 runs of 5 before passing, and the cause was
neither Nav2 nor SLAM: `RTF_LIMIT` used to be applied by calling
`/world/<name>/set_physics` after startup, and that call **stops rigid-body motion**
while sim time, the sensors and the odometry publisher keep running. The robot
travelled 0.007 m of true distance while its wheels reported 3.99 m, so one
unchanging scan got inserted at a pose that kept moving — which is what drew the
arena repeatedly at rotating angles and read as a mapping defect. The throttle is
now declarative (`make_world.py --rtf` writes `<physics><real_time_factor>`), and
SLAM tracks ground truth to within 0.053 m. Two other real defects were fixed on
the way (recovery behaviours inoperable via `publish_frequency: 0.0`;
`controller_frequency` is a wall rate), and neither was the cause. Full account,
including the two wrong theories: [`RUNLOG.md`](RUNLOG.md) Step 6.

**The split topology delivers.** A RISC-V guest under TCG runs Nav2 and
slam_toolbox against Gazebo on the x86 host, over TCP through SLIRP, drives a
waypoint tour and builds a 112×103 map from the host's simulated lidar — the
topology the TSP figure prescribes. Reproduced twice on 2026-08-19: 2/2
waypoints in 30–36 s per leg, every map floor met, co-tenant CPU inside budget.

Getting there took seven fixes, and not one of them was in Nav2 or SLAM:

| Fixed | Was |
|---|---|
| ROS on the guest's `PATH` over `ssh` | `bash -l` reads `~/.profile`, not `~/.bashrc` |
| Fast DDS asynchronous publish mode | a synchronous writer cannot fragment, so `/scan` (~2.9 kB) and `/odom` (~700 B) were dropped with nothing logged, while `/clock` (~16 B) and `/tf` (~200 B) crossed |
| `rviz2` on 42103 and `detection_overlay.py` on 42104, generated locators | Both are host acceptors like any other, so the guest needs an `initialPeersList` entry per port. `GUI=1` writes `guest_tcp_client_demo.xml` from the committed profile instead of editing it, so the file G3/G4/G5 were measured on stays byte-identical and ~13 guest participants in a headless run do not re-dial two dead ports every announcement interval |
| `clock_decimator.py`, own port 42101 | Gazebo's 100 Hz `/clock` is more than the guest can service, and a second host participant without its own listening port is discoverable and unreachable |
| `WATCHDOG=` | `liveliness_watchdog.py` was 69% of an emulated core — the largest consumer in the guest, and an instrument |
| `RTF_LIMIT=0.5` | at half speed a node that cannot keep up gets twice as long. Clock lag 30–42 s → **0.5–3.1 s** |
| `default_server_timeout: 5000` | upstream's 20 ms is how long a BT node waits for a server to *acknowledge* a goal. `planner_server` did not answer in 20 ms and every goal aborted with no message from the planner at all |
| `run_split.sh` pushes `config/` too | the fix above sat in the repository for two runs while the guest used an 08-17 copy |

The last two are the ones that made the robot move, and the first of them had
been solved months earlier for the guest-local case in
[`../nav2/loopback_nav.yaml`](../nav2/loopback_nav.yaml) — it was simply never
carried across.

One more defect surfaced only because the run finally succeeded twice: the map
gate flipped PASS/FAIL between two runs whose maps are indistinguishable, because
`coverage()` projected grid cells with `floor()` and one map's rows landed on
exact cell boundaries. That is fixed and the metric still fails a map of a robot
that never moved. [`RUNLOG.md`](RUNLOG.md) Step 8 has all of it.

**This rig cannot produce an availability number.** The guest is Config A —
`wg=on`, no `reri=true`, no pflash pair — so no error can be injected. It builds
the co-tenant workload TC.10.007 needs, never a TC.10.007 result. See
[`DESIGN.md`](DESIGN.md#what-this-cannot-tell-you).

## Do not

| Don't | Because |
|---|---|
| `apt install ros-jazzy-navigation2` in the guest | 704 packages, 411 of them unusable graphics |
| `apt install ros-jazzy-nav2-bringup` or `nav2-minimal-tb3-sim` in the guest | pulls the whole Gazebo chain; extract maps and params with `apt-get download` + `dpkg -x` from the 35–44 kB deb instead |
| wildcard the `ros_gz_bridge` config | switches the unbridged depth camera on |
| `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA` | discovery stays on UDP multicast and the TCP direction can invert to host→guest, which is unroutable |
| drop `<wan_addr>10.0.2.2</wan_addr>` from the host descriptor | the TCP connection still comes up and then delivers nothing, with no error on either side. The host would announce locators from its own interfaces, and SLIRP routes only `10.0.2.2` from the guest |
| run a viewer in a second container and expect it to receive anything | Docker gives it a private IPC namespace and therefore its own `/dev/shm`, so a shared-memory locator names a segment the other side cannot open. Measured 2026-08-19 on a 5 Hz topic: a subscriber inside `vela-gz` read **5.000 Hz**, the same subscriber in the viewer container read **nothing in 25 s**, and `/dev/shm` held 14 `fastrtps_*` segments in one and none in the other. `host_rviz_server.xml` sets `useBuiltinTransports` **false** and declares UDPv4 explicitly, which is the one place in this rig where false is right; it then read **5.001 Hz** |
| probe a topic with the same profile the viewer uses | A `ros2` CLI call binds the same listening port, so a check running beside RViz takes 42103 away from it. Found while testing the fix: two probes in one shell collided with each other. Probe before the viewer starts, or read the viewer's own log — `run_demo.sh` reports `/map` by grepping RViz for `Trying to create a map of size` |
| call a topic missing because a short probe did not see it | The guest only starts dialling a host acceptor once something owns it, and the first `/map` took **33 s** to arrive after 42103 appeared. A 12 s probe reported it absent while it was on its way |
| start a second host participant with `host_tcp_server.xml` and assume it works | 42100 has one owner. The second one logs `Address already in use`, comes up with **no** TCP acceptor, and stays silent while looking healthy. Written after the clock decimator hit it, and then `cmd_vel_watchdog.py` did the same thing for two days: every split-mode `artifacts/watchdog-*.log` opens with that error, so the watchdog saw no `/cmd_vel` from the guest and its own zeros could not reach the bridge either — a no-op in both directions, in the one place a latched `DiffDrive` is the hazard. It has port 42102 now |
| rely on ROS being on `PATH` over `ssh` | `bash -l -s` reads `~/.profile`, not `~/.bashrc`, and the guest's `~/.profile` does not source ROS. Every script sources `/opt/ros/jazzy/setup.bash` itself |
| expect discovery across SLIRP in a few seconds | it took 38 s under TCG. A 30 s budget fails a working system |
| leave Fast DDS writers in the default synchronous publish mode | a synchronous writer cannot fragment, so any sample too large to send in one piece is dropped with **nothing logged on either side**. `/tf` (~200 B) crossed and `/odom` (~700 B) did not, from the same plugin at the same rate and QoS |
| bridge a topic nothing subscribes to | `/imu` at 200 Hz sim was crossing a TCP link into an emulated guest for no consumer. Every bridged topic costs the guest deserialisation on an emulated CPU |
| bridge Gazebo's `/clock` straight through to the guest | 100 Hz is its physics rate, not a requirement. Seven guest nodes each carry their own `/clock` subscription, and the guest's own notion of time falls tens of seconds behind. Use `clock_decimator.py` |
| add a second host participant without its own listening port | a port has one owner. The second participant is discoverable and unreachable, and forwards data happily into nothing. Give it a port and add it to the guest's `initialPeersList` |
| omit `--network host` or use the default `--shm-size` | see [Build — host](#build--host) |
| make `/tf` BIDIRECTIONAL | the guest's `map→odom` returns to Gazebo, which republishes it |
| deserialise a message you only need the arrival time of | `liveliness_watchdog.py` read no field of `/scan`, `/odom` or `/map` and still paid rclpy to turn 360 floats and 11 322 cells into Python objects — 93% of an emulated core, the largest consumer in the guest, starving the workload it existed to watch. Subscribe with `raw=True` |
| read a G6 CPU figure from a named list of processes | `ps -C` with three of the eight came to ~99% against a 250-of-400 budget, while the guest was actually at loadavg 23 on 4 cores. List every workload process and print `loadavg` beside the sum — `%CPU` cannot show queueing |
| score the saved map in the guest | `MAP_REF` is nav2_bringup's `tb3_sandbox`, which is in the container and deliberately not in the guest. Bring the map back to the host, where G2's reference and scorer already are |
| leave `save_map_timeout` at its 2.0 s default | two runs lost their map to a timeout 2.1 s after `Saving map from 'map' topic`. Use 30.0, and `use_sim_time:=false` so a stalled clock cannot turn the timeout into a hang |
| lower `CLOCK_HZ` to make a lagging node catch up | `KEEP_LAST(1)` drops the backlog, so one callback makes a node current however far behind it is — measured, `--block 15` cost nothing lasting. 2 Hz left `bt_navigator` no better and made `planner_server` worse |
| trust `ros2 topic hz /clock` to show a clock problem | it reports the arrival rate, which is correct and healthy while a node's own `now()` is a minute stale. The deficit is on the consuming side; use `clock_lag_probe.py` |
| put `/clock` in `ros2 bag play --topics` | `--clock` already synthesizes it; two publishers break monotonicity |
| pass a negative-looking value as `--opt value` | argparse reads any leading-dash token as an option unless it matches `^-\d+$`/`^-\d*\.\d+$`. Every offset here is negative, so use `--offset=-2,-0.5` |
| set `default_nav_to_pose_bt_xml: ""` | an empty string is not "use the default": bt_navigator loads no tree and every goal aborts in 0 s with `Empty Tree` |
| leave `default_server_timeout` at upstream's 20 ms | that is how long a BT action node waits for a server to *acknowledge* a goal — not to plan one. Under TCG `planner_server` does not answer in 20 ms, and the goal aborts ~140 ms after it arrives with **no message from the planner at all**, which reads as a planning failure. The symptom moves to `follow_path` rather than disappearing if only one leg is raised. `../nav2/loopback_nav.yaml` already carried the fix for the guest-local case |
| put a setting only one runner wants into a config both runners share | `bridge_split.yaml` is used by `run_container_loop.sh` (G2) and `run_split.sh` (G5). Renaming Gazebo's clock to `/clock_gz` inside it — correct for G5, which puts `clock_decimator.py` behind it — left G2 with **no `/clock` publisher at all** while every node it starts asks for `use_sim_time`. Introduced 08-17 21:34, one hour and forty-four minutes after the last G2 run at 19:50, so nothing caught it. The rename belongs to the runner: `run_split.sh` passes `-r /clock:=/clock_gz` |
| copy `*.py` and `*.yaml` to the guest and think that is "push everything" | `guest_tcp_client.xml` is the file that makes the guest dial the host's TCP acceptors, and it is `.xml`. Adding a host listening port means adding a peer entry there, and with the old two-glob `scp` that edit would sit in the repository while the guest kept dialling the old ports. The glob is `*.py *.yaml *.xml` |
| derive `MAP_OFFSET` from the spawn pose | it converts SLAM coordinates into **the reference map's** frame, and the two shipped references do not use the same one: `tb3_sandbox.yaml`'s origin is in world coordinates so the offset is the spawn, while `depot.yaml`'s is already spawn-relative so the offset is `0,0`. Deriving it cost tb4 a spurious map FAIL at precision 0.421 where the real score was 0.792 |
| sweep an offset over a window centred on the value you suspect | that tests its neighbourhood, not the hypothesis. The tb4 sweep covered dx ∈ [−8.5, −7.0], found a shallow monotone ridge, and concluded no offset error was possible. The peak was at dx = 0, seven metres outside the window, and sharp: 0.437 at −1, 0.608 at −0.1, **0.792 at 0**, 0.636 at +0.1. Sweep through zero and read the profile, not its endpoints |
| build a publish-to-subscribe latency probe for a topic that crosses the split | the two machines' clocks are not synchronised, and the sim clock does not rescue it — the guest's `now()` is a sample-and-hold of the last `/clock` it serviced, so the probe returns `RTF × L − LAG_sim` where `LAG_sim` is 0.5–3.1 s against a 5–25 ms signal. Measure inter-arrival at the consumer on one clock, and use `rtt_probe.py` for the crossing |
| push only the instrumentation to the guest and call the rest a one-time `scp` | a stale guest file cannot announce itself. It produces a result that describes code nobody is reading — measured: a parameter fix sat in the repository for two runs while the guest used an 08-17 copy. Push `tools/` and `config/` every run |
| set `publish_frequency: 0.0` on a costmap to save CPU | it stops `costmap_raw` publishing, and `behavior_server` reaches its costmap only through that topic. Every recovery then aborts with `Costmap is not available`, and nothing says so until a run first needs a recovery |
| throttle Gazebo by calling `/world/<name>/set_physics` at runtime | it stops rigid-body motion while sim time, the sensors and the odometry publisher keep going. The robot sits still while its wheels report metres of travel, and the only symptom is a SLAM map that looks like a mapper bug. Set `<physics><real_time_factor>` in the world instead |
| read a smeared SLAM map as a SLAM defect | measure the true pose first. `TRUTH_PROBE=1 scripts/run_container_loop.sh` answers it in one run; here the robot had not moved at all |
| gate map quality on a single number | precision, recall and coverage each catch a different failure and no other number sees it. And never dilate one map and not the other: it caps the score below its own threshold and rewards blurrier maps |
| tune `mapper_params_tcg.yaml` without a measurement | the previous tuning set killed slam_toolbox mid-drive with `Mapper FATAL ERROR - unable to get pointer in probability search!` while RTF was already 1.0000. Add one parameter at a time, each with its number |
| launch `slam_toolbox` without listing it in `lifecycle_manager_nav.node_names` | Jazzy's `async_slam_toolbox_node` is a lifecycle node with no self-activation: it sits in `unconfigured`, never publishes `map→odom`, and every costmap waits forever on a `map` frame that does not exist |
| change a `*_search_space_resolution` without its `*_smear_deviation` | Karto requires `0.5×resolution ≤ smear ≤ 10×resolution` and aborts the node with SIGABRT at sensor registration |
| use `nav2_simple_commander.BasicNavigator` | twelve action clients ≈ 70 DDS endpoints starve the executor, producing a false `SERVER planner_server IS DOWN` |
| `set -u` before sourcing `/opt/ros/jazzy/setup.bash` | `AMENT_TRACE_SETUP_FILES` is dereferenced while unset |
| write log-matching regexes without `^` | apt's own `NOTE: This is only a simulation!` banner contains `E: ` |
| `pkill -f <pattern>` without the bracket trick | the pattern matches `pkill`'s own command line |
| pin apt versions with `=` in the `Dockerfile` | the ROS repo keeps only the newest build; the pin fails instead of reproducing |

## See also

- [`DESIGN.md`](DESIGN.md) — why the rig is shaped this way, and what it cannot tell you
- [`RUNLOG.md`](RUNLOG.md) — the reproducibility record
- [`../workload-nav2.md`](../workload-nav2.md) — the Nav2 stack and TCG tuning this builds on
- [`../README.md`](../README.md) — RERI injection and collection
