# Run log

One row or block per step, filled in as it happens. This is the reproducibility
record: someone reading it should be able to tell what was run, on what, what
came out, and where it stopped.

Rules that keep it useful:

- Paste **real output**, trimmed, not a summary of it. A number nobody can trace
  is not evidence.
- Record failures with the same weight as successes. A gate that killed the run
  is the most valuable entry in the file.
- Fill `Verdict` from the script's own `RESULT:` line, not from an impression.
- Keep the raw logs next to this file under `logs/` and reference them by name.

```bash
mkdir -p logs
scripts/00-preflight.sh        2>&1 | tee logs/p0-preflight.log
scripts/check_interop.sh       2>&1 | tee logs/p1-interop.log
ROBOT=tb3 scripts/run_rtf.sh   2>&1 | tee logs/g1-rtf-tb3.log
ROBOT=tb4 scripts/run_rtf.sh   2>&1 | tee logs/g1-rtf-tb4.log
ROBOT=tb3 scripts/run_container_loop.sh 2>&1 | tee logs/g2-loop-tb3.log
ROBOT=tb3 RTF_LIMIT=0.5 scripts/run_container_loop.sh 2>&1 | tee logs/g2b-loop-tb3-rtf05.log
scripts/run_dds_check.sh       2>&1 | tee logs/g3g4-dds.log
ROBOT=tb4 scripts/run_split.sh 2>&1 | tee logs/g5-split-tb4.log
```

Entries below were recorded under the pre-2026-08-18 flat layout, so their
command lines lack the `scripts/` prefix and their output paths lack
`artifacts/`; see "Re-layout" at the end of this file.

---

## Environment

Fill from `00-preflight.sh` P0–P3.

| | Value |
|---|---|
| Date | 2026-08-14 |
| Operator | ms.kim — Step 1 run by hand; Steps 0 and 2 executed on the same host during review |
| Host OS / arch | Ubuntu 24.04.3 LTS x86_64, kernel 6.14.0-THP |
| Host CPUs visible / total | 192 / 192 (affinity 0-191) |
| Host RAM | 503 GiB total, 481 GiB available, 0 swap |
| Host GPU / render node | 2× Matrox G200eR2, **no `/dev/dri/renderD*`** — rendering is llvmpipe |
| Docker version | 28.4.0, overlay2, 310 GiB free under the docker root |
| Base image digest | `ros:jazzy-ros-base@sha256:da725acf8b0f9f30c683e33ffbdcd6482d077af96d6fdc7688c5f4f280b7d923` |
| Image apt manifest | `logs/s1-manifest.txt`, **1543** packages |
| Guest kernel | 6.18.0-rc5, riscv64 |
| Guest ROS packages installed | 242 (`ros-jazzy-*`), Nav2 `nav2-planner 1.3.11-1noble`, `robot-state-publisher 3.3.3-3noble` |
| Guest disk free | 12 GiB |
| QEMU command | see `q-vela/docs/ras.md` §3 — paste the exact line used |
| Config | A (`wg=on`, no `reri`, no pflash) — **no RAS** |

---

## Step 0 — preflight

`logs/p0-preflight.log`

```
=== P5  verdict ===
  stop conditions: 0
  warnings:        0
  RESULT: GO -- continue to Step 1 (docker build)
```

Verdict: ☑ GO ☐ GO WITH WARNINGS ☐ NO-GO

Warnings seen, and what was done about them:

| Warning | Action |
|---|---|
| P2b reported Fuel unreachable | **false negative in the check, not a network fault.** The Fuel API answers `HEAD` with 405, and P2b used `curl -sfI`. Changed to a GET (~725 B body); rerun then gave 0 warnings. Rows 1 and 2 below |

---

## Step 1 — `docker build`

Command (add `--build-arg HOST_UID=$(id -u) --build-arg HOST_GID=$(id -g)` if the
host account is not uid 1000):

```
docker build -t vela-gzsim:jazzy . 2>&1 | tee logs/s1-build.log
```

| | |
|---|---|
| Wall time | **335.2 s** summed over 9 steps — apt install 238.4 s, image export 47.2 s, base pull 46.4 s |
| Image size (`docker images vela-gzsim:jazzy`) | **3.68 GB**, id `5481b6380273` |
| Base image pinned (`/opt/vela/base-image.txt`) | `ros:jazzy-ros-base@sha256:da725acf8b0f…`, matches the image's `org.opencontainers.image.base.name` label |
| Build args used | none — the host account is already uid 1000, so the defaults applied |
| apt manifest saved | ☑ `logs/s1-manifest.txt`, **1543** packages |
| Result | ☑ ok ☐ failed |

Log: `logs/s1-build.log` (462 kB). The only warnings in it are
`update-alternatives: warning: skip creation of /usr/share/man/…` from
`--no-install-recommends` leaving man pages uninstalled — cosmetic, and expected.

The manifest is the reproducibility artefact — apt versions are not `=`-pinned,
so this records what the build actually resolved to. Versions that matter:

| Package | Version |
|---|---|
| `ros-jazzy-nav2-minimal-tb3-sim` | 1.0.1-1noble.20260616.074421 |
| `ros-jazzy-nav2-minimal-tb4-sim` | 1.0.1-1noble.20260615.175354 |
| `ros-jazzy-nav2-minimal-tb4-description` | 1.0.1-1noble.20260615.164333 |
| `ros-jazzy-navigation2` | 1.3.12-1noble.20260615.181551 |
| `ros-jazzy-slam-toolbox` | 2.8.5-1noble.20260615.161600 |
| `ros-jazzy-ros-gz-sim` / `-bridge` | 1.0.22-1noble |
| `ros-jazzy-gz-sim-vendor` | 0.0.10-1noble (`gz sim 8.11.0`) |
| `ros-jazzy-rmw-fastrtps-cpp` | 8.4.4-1noble.20260615.124621 |
| `ros-jazzy-sensor-msgs` / `-tf2-msgs` / `-nav2-msgs` | 5.3.8 / 0.36.21 / 1.3.12 |
| `mesa-libgallium`, `libgl1-mesa-dri` | 25.2.8-0ubuntu0.24.04.2 |
| `python3-numpy` | 1:1.26.4+ds-6ubuntu1 |

`mesa-utils-extra` and `iproute2` confirmed **absent** (both were dropped after
checking they contribute nothing); `mesa-utils` pulled `mesa-utils-bin`, which is
what actually carries `eglinfo` and `glxinfo`.

If the build failed, the failing `RUN` line and the error:

```
```

---

## Step 2 — `docker run`

```
docker run -d --name vela-gz --network host --shm-size=512m --init \
  -v "$PWD":/work -w /work vela-gzsim:jazzy
```

> `--init` was added after this run. The container in use was created without it,
> so orphaned `docker exec -d` children accumulate as `<defunct>`. Harmless — an
> empty cmdline means they never match the scripts' `pgrep` checks — but the next
> recreate should include it.

| | |
|---|---|
| Container id | `ec5d635b78ae`, `Up`, command `/ros_entrypoint.sh sleep infinity` |
| `printenv ROS_DISTRO` | `jazzy`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| `id` | `uid=1000(vela) gid=1000(vela)` — the non-root change works |
| `touch /work/.permcheck` | `-rw-r--r-- 1 cslab cslab` — **not root**, which was the point |
| `$HOME` | `/home/vela`, writable (`~/.gz` Fuel cache, `~/.ros`) |
| `/dev/shm` | 512M |
| network namespace | container hostname = host hostname `cslab192`, so `--network host` took effect |
| `LIBGL_ALWAYS_SOFTWARE` / `GALLIUM_DRIVER` | `1` / `llvmpipe` |
| Result | ☑ ok ☐ failed |

Log: `logs/s2-run.log`, `logs/s2-verify.log`.

ROS tooling resolves only after `setup.bash` is sourced — `docker exec` does not
run the entrypoint, which is why every `dex()` sources it:

```
gz       /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz     (gz sim 8.11.0)
xacro    /opt/ros/jazzy/bin/xacro
ros2     /opt/ros/jazzy/bin/ros2
```

Assets for both robots confirmed present in the image: `gz_waffle.sdf.xacro`,
`tb3_sandbox.sdf.xacro`, `depot.sdf`, `turtlebot4.urdf.xacro`, 12 TB4 meshes, and
both `nav2_bringup/maps/{tb3_sandbox,depot}.yaml`.

### Step 2 smoke test — `make_world.py` on the real image

Not a gate. Run before G1 because it is the cheapest way to prove the tb4 package
addition actually works, rather than discovering it during a 3-minute gz start.
Log: `logs/s2-makeworld-smoke.log`.

| | tb3 | tb4 |
|---|---|---|
| elements removed | 2 matching `['camera']` | **29** matching `['oakd']` |
| `gpu_lidar` / camera sensors | 1 / **0** | 1 / **0** |
| lidar `update_rate` | 5 (untouched), 360 samples over 6.28 rad | 10.0 → **5.0** |
| `max_step_size` | 0.003 → 0.01 | 0.003 → 0.01 |
| world / spawn | `default` / (-2.0, -0.5) | `depot` / (-8.0, 0.0) |
| `MAP_OFFSET` | `-2,-0.5` | `-8,0` |
| TF-critical links surviving | `base_footprint base_link base_scan` | `base_footprint base_link shell_link rplidar_link` |
| generated XML | `world.sdf` + `robot.sdf` parse | `world.sdf` + `robot.urdf` parse |

The lidar geometry matches what `DESIGN.md` cites for the render budget: 360
samples over 0→6.28 rad at 5 Hz on both robots after the edit.

Fuel cache pre-seeded for tb4 at the end of Step 2: `Depot` version 6, 101 MB in
`/home/vela/.gz/fuel`, 22.8 s. Done now so a tb4 G1 failure cannot be a download.

---

## Step 3 — interop check (`check_interop.sh`)

```
=== 2  type hashes -- this is what actually decides interop ===
  all type hashes identical:   (9 types)
RESULT: PASS -- the version skew is cosmetic. Proceed to Step 4.
```

Result: ☑ PASS ☐ mismatched. Log: `logs/s3-interop.log`.

Measured skew, 2026-08-14 (`logs/s3-interop.log`) — every one of these differs and
none of it matters, because the type hashes match:

| Package | container (amd64) | guest (riscv64) |
|---|---|---|
| `ros-jazzy-sensor-msgs` / `-geometry-msgs` / `-nav-msgs` | 5.3.8 | 5.3.7 |
| `ros-jazzy-tf2-msgs` | 0.36.21 | 0.36.20 |
| `ros-jazzy-rosgraph-msgs` | 2.0.4 | 2.0.3 |
| `ros-jazzy-rmw-fastrtps-cpp` | 8.4.4 | 8.4.3 |
| `ros-jazzy-rclcpp` | 28.1.21 | 28.1.18 |
| `ros-jazzy-nav2-msgs` | 1.3.12 | 1.3.11 |
| `ros-jazzy-fastrtps` | 2.14.6 | 2.14.6 (different build stamp) |

Type hashes: ☑ identical ☐ mismatched — all 9, including
`nav2_msgs/action/NavigateToPose` and `nav_msgs/msg/OccupancyGrid`.

---

## Demonstration (not a gate)

Run when the simulation has to be shown to people. `Dockerfile.demo` →
`vela-gzsim:demo`, then `GUI=1 <run script>` in one terminal and `./run_demo.sh`
in another. Proven on this host 2026-08-14:

| | Observed |
|---|---|
| `gz sim -g` under Xvfb + llvmpipe, no GPU | works — 14183 distinct colours in the grab, scene and robot both drawn |
| time from `--standalone --shot` to PNG | 2 min 6 s (≈ 60 s of it OGRE-Next shader compilation) |
| GUI cost | 2007 %CPU (20 cores); server 20.5 %CPU; `real_time_factor` **1.00** |
| VNC | `x11vnc` on 127.0.0.1:5900, reached over `ssh -L` |
| evidence | `logs/demo-tb3-20260814-165135.png` |

`real_time_factor = 1.00` here is **indicative, not G1**: SceneBroadcaster was on
and a GUI was attached, and `run_rtf.sh` has its own procedure. It does suggest G1
will pass comfortably for tb3 on this host.

### The recording, 2026-08-17

Made for a workshop slide. The robot drives the tour in Gazebo; the clip is the
visual evidence that the stack works end to end.

| | |
|---|---|
| clip for the deck | `logs/gazebo-nav2-slam-tb3-drive.mp4` — 38 s, 1920×1080, 15 fps, H.264/yuv420p, 2.5 MB |
| full recording kept | `logs/gazebo-nav2-slam-tb3-full-210s.mp4` — 210 s, 3.9 MB |
| what it shows | tb3 in `tb3_sandbox`, chase camera following it, Entity Tree listing `turtlebot3_world` and `tb3` |
| the drive in it | `RESULT: PASS (2/2 waypoints reached)`, both `status=4 SUCCEEDED`, final errors 0.234 m and 0.236 m |
| produced by | `PRE_DRIVE_SLEEP=75 GUI=1 ROBOT=tb3 ./run_container_loop.sh` + `GEOMETRY=1920x1080 ./run_demo.sh --record --duration 130`, trimmed with `ffmpeg -ss 52 -t 38` |
| motion verified | mean grey-level change between sampled frames 23.8 / 33.1 / 38.2 — not a still image |

**This is a demonstration, not a gate.** SceneBroadcaster was on and a GUI was
attached. The same run's `IoU 0.543` is also not a G2 result for the same reason,
and would not pass if it were: see the coverage note under Step 6.

---

## Step 4 — guest install (`slam_toolbox`)

```
sudo apt-get install -y ros-jazzy-slam-toolbox
```

| | |
|---|---|
| Packages installed | (expected ~192) |
| Download / disk | (expected ~165 MiB) |
| Wall time under TCG | |
| Guest disk free after | |
| Result | ☐ ok ☐ failed |

---

## Step 5 — G1, real-time factor (`run_rtf.sh`)

```
paste the tail: real_time_factor, /scan finite count, RESULT line
```

Run for both robots. TB3 answers "does headless Gazebo work on this host";
TB4 answers "does the TSP's robot fit in the budget".

| Robot | `real_time_factor` | `/scan` finite returns | Verdict |
|---|---|---|---|
| tb3 | **1.0000** | **360** of 360 | **PASS** (≥ 0.30) |
| tb4 | **1.0000** | **336** of 360 | **PASS** (≥ 0.30) |

Measured 2026-08-17. `logs/g1-rtf-tb3.log`, `logs/g1-rtf-tb4.log`.

| | tb3 | tb4 |
|---|---|---|
| world | `default` (tb3_sandbox) | `depot`, from the pre-seeded Fuel cache |
| stats topic up after | 5 s | 5 s |
| spawn | `-file` | `-topic robot_description` (waited for the topic, then ok) |
| `real_time_factor` | 1.0000131 | 1.0000020 |
| finite `/scan` returns | 360 / 360 | 336 / 360 |

**Real time, both robots, headless, software-rendered, no GPU.** Against a 0.30
pass threshold and a 0.10 kill threshold, the margin is 3.3×. The tb4 result is
the one the TSP figure's robot needs, and it lands identically to tb3 — the camera
deletion and the 10 → 5 Hz lidar change did their job.

tb4's 24 non-finite returns are correct, not a fault: `depot` is a large open
space and those rays reach the 20 m `rplidar` limit without hitting anything, so
they come back `inf`. What the check exists to catch is *all* 360 being `inf`,
which is what a dead render path looks like.

Render path evidence in both logs is the `Sensors.cc` / `SdfEntityCreator` block;
the only warning is `Gazebo does not support Ogre material scripts`, which is the
shipped world's own material syntax and affects appearance, not the lidar.

Thresholds: ≥ 0.30 pass, 0.10–0.30 marginal, < 0.10 kill. `/scan` needs ≥ 1
finite return — a topic that publishes only `inf` means the render path is dead.

Render engine used: ☐ ogre2 + headless EGL ☐ ogre2 + Xvfb ☐ ogre1 (`--render-engine-server ogre`)

**Render path evidence.** Nothing in the `Dockerfile` proves software rendering is
in use; this is where it is established. Paste step 1's three blocks (env vars +
`/dev/dri`, `eglinfo`, `glxinfo`) and step 3's "render path, from the engine's own
log". If a block came back empty, write *empty* rather than leaving it blank.

```
```

Fuel pre-seeded for tb4? ☑ yes ☐ no — download time: **22.8 s**, `Depot` version 6,
101 MB under `/home/vela/.gz/fuel` (done at the end of Step 2). The cache lives in
the container, so a `docker rm` means re-seeding.

If the fallback ladder was needed, which rung and why:

```
```

---

## Step 6 — G2 / G2b, closed loop in the container

Run twice at different real-time factors. `use_sim_time` does not immunise Nav2
against a low RTF, so a result that only holds at one RTF is not a result.

| Run | Robot | `RTF_LIMIT` | Observed RTF | Waypoints | Precision | Recall | Coverage | Verdict |
|---|---|---|---|---|---|---|---|---|
| G2 | tb3 | 0 (unthrottled) | 1.000 | **2 / 2** | 0.910 | 0.959 | 0.959 | **PASS** |
| G2 | tb3 | 0 (unthrottled) | 1.000 | **2 / 2** | 0.932 | 0.953 | 0.983 | **PASS** |
| G2b | tb3 | 0.5 | 0.500 | **2 / 2** | 0.934 | 0.971 | 0.973 | **PASS** |
| G2b | tb3 | 0.5, `CONTROLLER_HZ=5.0` | 0.500 | **2 / 2** | 0.994 | 0.980 | 0.977 | **PASS** |
| G2b | tb3 | 0.5 (defaults, confirming) | 0.500 | **2 / 2** | 0.932 | 0.944 | 0.967 | **PASS** |

G2b passed only after the throttling defect below was fixed. Before it, the same
command failed 5 runs out of 5 at 1/2 waypoints with precision 0.37–0.44.

Ground truth over the three passing G2b runs, from `pose_truth_probe.py`: SLAM
position error mean 0.016–0.017 m and max 0.042–0.053 m, yaw error max 1.23–2.64°,
with true travel 7.907 m against 7.87 m of odometry. The estimate tracks the
simulator, at half real time.

Gates: `precision ≥ 0.80`, `recall ≥ 0.85`, `coverage ≥ 0.90`. Coverage is
per-world and comes from `robot.env`; it is **off** for tb4 (see below).

G2 raw: `logs/g2-loop-tb3-new-metric.log`, scored in `logs/g2-score-tb3-163002.log`.
G2b raw, in the order run: `logs/g2b-loop-tb3-new-metric.log` (first failure),
`-fix1` (after the `publish_frequency` fix), `-probe` (rate census attempt),
`-hzfix` (after scaling `controller_frequency`), `-census`.

Bringup time to `Managed nodes are active`: **5 s** at RTF 1.0, **10 s** at RTF 0.5.

### G2b failed 5 runs of 5, and the cause was the throttle itself

The robot was not moving. `RTF_LIMIT` was applied by calling
`/world/<name>/set_physics` after startup, and that call stops rigid-body motion
while leaving sim time, the sensors and the odometry publisher running normally.
Measured with `pose_truth_probe.py` against the simulator's own pose:

| At `RTF_LIMIT=0.5` | Distance travelled |
|---|---|
| **truth** (gz `PosePublisher`) | **0.007 m**, and `truth_yaw` span **0.000 rad** — it never even turned |
| odom (DiffDrive wheel integration) | **3.99 m** |
| SLAM estimate | 3.96 m |

So `/scan` returned the same view forever while odometry insisted the robot was
driving. slam_toolbox split the difference, which is exactly the smear seen in the
saved maps: one unchanging scan inserted at a pose that keeps moving draws the
arena over and over at rotating angles. Everything downstream — the 0.13–0.30 m
pose corrections, the poisoned global costmap, the unplannable return leg, the
14 `Failed to create plan` warnings — followed from that.

Fixed by setting the RTF declaratively: `make_world.py --rtf` writes
`<physics><real_time_factor>` into the world, and the runner no longer calls
`set_physics` at all. The same command then gives **2/2 waypoints at RTF 0.5000**,
with SLAM within **0.053 m** and **1.66°** of ground truth over the whole drive,
and truth travel 7.907 m against odom 7.87 m.

Passing `max_step_size` alongside `real_time_factor`, so no protobuf field could
arrive as its zero default, did **not** help — it is the runtime call itself, not
one missing field.

**Two hypotheses were refuted along the way. Both are recorded because both looked
convincing.**

1. `publish_frequency: 0.0` really did make every recovery behaviour inoperable
   (row 24), and fixing it was necessary for the recoveries to run at all — but the
   run still ended 1/2.
2. `controller_frequency` really is a **wall** rate: at RTF 0.5, `/cmd_vel` 5.000 Hz
   wall against `/scan` 2.502 Hz wall, two commands per scan (row 25). Scaling it to
   2.5 Hz changed nothing, and after the real fix RTF 0.5 passes either way —
   unscaled 0.994/0.980, scaled 0.934/0.971. The scaling is therefore **not** in the
   default path; `CONTROLLER_HZ` remains for experiments.

What the failure looked like before the fix, kept for recognising it again:

| Signature | RTF 1.0 | RTF 0.5 |
|---|---|---|
| waypoints | 2 / 2 | 1 / 2 |
| saved map size | 112 × 103 cells | 178–184 × 142–183 |
| SLAM pose discontinuities faster than 3× the robot's top speed | **2** | **11–14** |
| total distance jumped | **0.06 m** | **0.70–1.05 m** |
| largest single jump | 0.032 m | 0.127–0.297 m |
| first jump | — | sim t ≈ 7.0–7.2 s, i.e. **during the first leg** |
| driven corridor (world y) | −0.62 … −0.11, below the cylinder row | −0.53 … **+0.15**, into it |

The saved maps make it visible: at RTF 1.0 the arena comes out as a clean hexagon
with the 3 × 3 cylinder grid; at RTF 0.5 it is drawn several times over at
rotating angles, spreading to x = 7.98 m where the arena ends at 4.65 m. The
smeared map then poisons the global costmap through `static_layer`, NavFn cannot
find a route back (`Failed to create plan with tolerance of: 0.500000` ×14), and
the run ends in recovery.

A wall-clock rate census at RTF 0.5 also found nothing decoupled from sim time:
`/clock` 49.997, `/scan` 2.501, `/odom` 12.501 (= 25 Hz sim, the 30 Hz request
quantised to the 0.01 s physics step), `/tf` 62.750, `/cmd_vel` 2.501 — every one a
clean half of its RTF 1.0 value. Worth recording because it is what ruled the
timing explanations out and forced the ground-truth measurement.

### The tool that settled it

`pose_truth_probe.py` (new) logs the simulator's true pose beside the SLAM estimate
and the wheel odometry. It answers one question, which is the question that had to
be answered before any of the timing theories could be tested:

| If the probe shows | The cause is | Look at |
|---|---|---|
| odom drifting from truth, SLAM staying near it | real odometry error the mapper is correcting | wheel slip, contact with the cylinders |
| odom tracking truth, SLAM leaving it | the scan matcher placing scans wrongly | `mapper_params_tcg.yaml`, scan timestamps |

Run it with `TRUTH_PROBE=1 ROBOT=tb3 RTF_LIMIT=0.5 ./run_container_loop.sh`. Two
things had to be built for it, and both are worth knowing before reusing them:

- ground truth is normally on `/world/<name>/dynamic_pose/info`, published by
  **SceneBroadcaster** — which every measurement world here removes via
  `headless:=true`. Using `--gui` to get it back would change the RTF under
  investigation, so `make_world.py --truth-pose` attaches a per-model
  `PosePublisher` instead, which needs no SceneBroadcaster;
- that plugin's topic follows the **entity** name assigned at spawn
  (`create -name tb3`), not the SDF's `<model name="turtlebot3_waffle">`. Both
  names get advertised, so the first attempt bridged
  `/model/turtlebot3_waffle/pose`, which is never written to: `/gz_truth` existed
  in ROS and stayed silent at 0 Hz. With `/model/tb3/pose` it runs at 10.007 Hz
  wall = 20 Hz sim.

Neither of those two facts is specific to this bug, and both will bite again the
next time ground truth is needed — which is why they are here and not only in the
code.

### What this changes for G5

The scare it caused was not real: the stack does hold at half real time, so a guest
that runs the closed loop at RTF 0.5 is not disqualified. What survives is narrower
and still worth carrying:

- **the throttle is now part of the world, not a runtime command.** G5 must not
  reintroduce a `set_physics` call to slow Gazebo down;
- **`controller_frequency` is a wall rate.** Harmless at RTF 0.5, but G5 runs Nav2
  under TCG where the achieved RTF is whatever the guest manages, and nothing scales
  it there. If a G5 map comes back smeared, measure the true pose first —
  `pose_truth_probe.py` exists now and answers it in one run;
- **a smeared SLAM map is not evidence of a SLAM defect.** Here it was a stationary
  robot. Five runs and two plausible, well-supported wrong theories went by before
  anyone measured whether the robot had moved at all.

### The occupied-IoU gate was invalid, and my first diagnosis of it was also wrong

Measured 2026-08-17, before recording any G2 number. Both findings changed the
code rather than the threshold.

**1. The metric could not be passed, and rewarded worse maps.** `score_map.py`
dilated the *reference* only, then took occupied IoU. A 1-cell reference wall
becomes a 5-cell plus shape that a 1-cell SLAM wall cannot fill, so:

| Map scored against tb3_sandbox, over one run's observed region | Old IoU |
|---|---|
| a **flawless** map (reference geometry, same observed region) | **0.533** |
| the real run 15:34:43 | 0.543 |
| a run that died in bringup, 13.6% of the arena mapped | 0.676 |
| gate | ≥ 0.70 |

The gate was unreachable by construction, and the ordering is inverted: the
aborted run scored highest because thin coverage and blurry walls both increase
overlap with an inflated reference. Fixed by matching **symmetrically** with a
1-cell tolerance, under which a flawless map scores exactly 1.000, and by
splitting the verdict into three numbers.

**2. One number cannot gate this.** Faults injected into the best real map
(`loop-tb3-20260817-153044`), against `P ≥ 0.85 ∧ R ≥ 0.85 ∧ coverage ≥ 0.90`:

| Injected fault | P | R | Coverage | Caught by |
|---|---|---|---|---|
| none (the run as measured) | 0.989 | 0.981 | 0.970 | — PASS |
| TF drift 0.05 m | 0.916 | 0.959 | 0.970 | — PASS (inside tolerance) |
| TF drift 0.10 m | 0.533 | 0.614 | 0.962 | P and R |
| TF drift 0.40 m | 0.225 | 0.186 | 0.876 | all three |
| 50% of wall cells erased | 0.990 | **0.832** | 0.970 | recall only |
| 5% of free space marked occupied | **0.632** | 0.981 | 0.970 | precision only |
| flawless map of 25% of the arena | 1.000 | 1.000 | **0.150** | coverage only |

Every fault class is caught by exactly one number, so any single combined score
lets the other two hide — F1 alone passes the 50%-erased map at 0.904.

**3. Coverage was never the problem — my earlier claim that the tour missed 44%
of the walls was wrong.** That 44% was the dilation artefact. Measured: the
straight out-and-back already observes **0.957–0.973** of tb3_sandbox's 7903 free
cells, and a 360-ray cast at the sensor's own 1° resolution along that path
reaches **100%** of reference wall cells. The lidar's range is 20 m (`gz_waffle.sdf.xacro`
`<max>20.0</max>`), not 3.5 m, so nothing in a 5.7 m arena is out of reach and
the nine cylinders occlude nothing that the far side of the drive does not
recover. **No waypoint change is needed for G2.** The straight path is still a
weak *demonstration* — it never triggers avoidance — but that is a video
question, not a gate question.

Thresholds are set against a measured ceiling of 1.000 and the gap between two
observed clusters (complete runs 0.957–0.973 coverage, aborted runs 0.135–0.141),
not against what a run happened to produce.

tb4's coverage gate is 0 (off): no tb4 closed-loop run exists to set a floor
against. The same ray cast over `depot.pgm` predicts 0.592 of its 179481 free
cells, but ray casting marks every endpoint and sees through nothing, so it is an
unvalidated upper bound. Set it after the first tb4 G2 run. `precision` and
`recall` still gate tb4.

Parameters that had to change from `nav2_split.yaml` / `mapper_params_tcg.yaml`,
with the observed symptom that forced each one:

| Parameter | From → To | Symptom |
|---|---|---|
| | | |

---

## Step 7 — G3 / G4, DDS across SLIRP (`run_dds_check.sh`)

Guest: `vela@localhost:2222`, riscv64, kernel 6.18.0-rc5, ROS Jazzy with
`rmw_fastrtps_cpp` already the active RMW. Config A, so no RERI.

| Measure | Threshold | Observed | Verdict |
|---|---|---|---|
| guest → `10.0.2.2:42100` TCP | connects | `tcp ok` | **PASS** |
| publisher → subscriber across SLIRP | < 120 s | **38 s** | **PASS** (G3) |
| `ss -tn \| grep :42100` | ≥ 1 ESTAB | 1 ESTAB pair via `qemu-system-ris` ↔ `ros2` on 127.0.0.1:42100 | **PASS** |
| round-trip p95 | < 500 ms | **12.1 ms** (p50 11.4, p99 18.4), 200/200 samples, 0 lost | **PASS** (G4) |
| intra-guest pub/sub still works | yes | yes, with the profile loaded | **PASS** |

Re-measured after Step 8 changed `host_tcp_server.xml` (async writers, larger buffers)
and dropped `/imu` from the bridge, because the numbers first recorded here were taken
with a profile that is no longer the shipped one. G3 was 38 s both times. G4 **improved**,
p95 21.2 → 12.1 ms: asynchronous writers move the send off the caller's thread, which
costs nothing on a small `Header` round trip and is what makes the large topics work at
all.

Ladder rungs needed: **rung 2 only, `wan_addr`.** `interfaceWhiteList` was not needed,
no `hostfwd_add` was needed, and no Discovery Server was needed — so the design's
central claim holds: **no QEMU restart and no `hostfwd` rule to get ROS 2 across
SLIRP.**

`<wan_addr>10.0.2.2</wan_addr>` on the host descriptor is not optional. Without it
the TCP connection came up and stayed silent: the server announces metatraffic
locators from its own interfaces (`10.198.137.110`, `172.17.0.1`, `127.0.0.1`), and
SLIRP routes exactly one address from the guest to this host. The guest connected,
discovered the participant, had no reachable locator to match, and delivered nothing
in a 90 s budget — 2 ESTABLISHED sockets on 42100 and no data. With `wan_addr` the
first sample crossed in 37–38 s.

Expected side effect, visible in the `ss` output and harmless here: host-side
participants also try to reach the announced `10.0.2.2`, which means nothing on the
host, and sit in `SYN-SENT`. The guest is the only peer that has to reach this side.

**38 s is not a defect, it is TCG.** Fast DDS retries its initial peer on an
interval, and a guest that emulates every instruction takes tens of seconds to get
through participant discovery. A 30 s budget would have failed a working system: an
earlier 35 s probe timed out about 3 s after the socket came up.

### Two things this step found that Step 8 depends on

**Port 42100 has exactly one owner.** Whichever host participant starts first binds
the TCP acceptor; every later one logs `TCPTransport Error binding at port: (42100)
... Address already in use` and comes up with **no acceptor at all**, still running
and still silent. G4 first failed as `echo node never discovered` purely because the
G3 publisher was still holding the port. `run_dds_check.sh` now stops it and asserts
the echo node bound.

**`cmd_vel_watchdog.py` is inert in the split topology, and silently so.**
`run_split.sh` starts `parameter_bridge` first and the watchdog second, both with
`host_tcp_server.xml`, so the watchdog gets no acceptor. With
`useBuiltinTransports: false` it also has no SHM or UDP, which leaves it unable to
see the guest's `/cmd_vel` *or* reach the local bridge. It does not misbehave — it
starts with `self.stopped = True` and `_check()` returns immediately in that state,
so it never publishes — but the protection it exists for is **absent**: a guest-side
stall would leave `gz-sim-diff-drive-system` latching the last velocity forever,
which is exactly the "downtime measurement becomes fiction" case its own docstring
warns about. Reasoned from the port evidence and the source, **not yet measured** —
confirm in Step 8 and fix before any downtime number is quoted. The likely fix is to
take the watchdog off DDS entirely and have it watch and drive the gz-transport side,
which is host-local and immune to this topology.

---

## Step 8 — G5, the split live run (`run_split.sh`)

Robot: ☑ tb3 (fully validated by G2/G2b first) ☐ tb4 (the TSP's robot, next)
World: `default` (tb3_sandbox)  Spawn: `(-2.00, -0.50, 0.01)`  `MAP_OFFSET`: `-2,-0.5`

**The split topology is live. G5 does not pass yet.**

What now demonstrably works, and had never run before:

| | Observed |
|---|---|
| Guest sees the bridged topics | `/clock /cmd_vel /odom /scan /tf` |
| `/clock` in the guest | 99.1 Hz |
| `/scan` in the guest | **4.990 Hz** (the lidar's 5 Hz sim rate) |
| `/odom` in the guest | **25.2 Hz** |
| `/tf` in the guest | 25.0 Hz |
| Nav2 + slam_toolbox bringup, in the guest | **235 s** to `Managed nodes are active` (56 s locally) |
| `map→base_footprint` from guest slam_toolbox | appeared after **34 s** |
| Map built in the guest from host Gazebo's lidar | **111 × 102 @ 0.05 m** |

So a RISC-V guest under TCG ran Nav2 and slam_toolbox against a Gazebo instance on
the x86 host, over one TCP connection through SLIRP, and produced a map. That is
the topology the TSP figure prescribes, and this is the first time it has run.

| Measure | Threshold | Observed | Verdict |
|---|---|---|---|
| bringup to active | — | **235 s** | — |
| waypoints SUCCEEDED | all | **0 / 2**, both aborted in 7 s | **FAIL** |
| map precision / recall / coverage | 0.80 / 0.85 / 0.90 | not reached — `map_saver` found no `/map` to subscribe to | **FAIL** |
| `/scan` gaps (`liveliness-*.csv`) | 0 | | |
| `/odom` gaps | 0 | | |
| G6 slam_toolbox + Nav2 %CPU | < 250 of 400 | | |
| G7 longest optimizer stall | < 20 s | | |

Track: `~/track-split-20260817-210317.csv` (646 samples, robot pose constant at the
origin). No map file: `map_saver` exited `Failed to spin map subscription`.

### What was fixed to get this far

**Synchronous Fast DDS writers silently dropped every larger sample.** With the
profile as first written, the guest saw the bridge's publishers — `Publisher count: 1`,
node `ros_gz_bridge`, matching type hash, `RELIABLE` — and received nothing on the
topics that matter:

| Topic | Payload | Before | After `ASYNCHRONOUS` |
|---|---|---|---|
| `/clock` | ~16 B | 102.6 Hz | 101.6 Hz |
| `/tf` | ~200 B | 25.0 Hz | 25.0 Hz |
| `/odom` | ~700 B | **nothing** | **25.2 Hz** |
| `/scan` | ~2.9 kB | **nothing** | **4.990 Hz** |
| `/imu` | ~330 B | nothing | nothing — dropped from the bridge instead |

QoS was identical on all four (`RELIABLE`, `VOLATILE`), so QoS was not the variable.
The decisive pair is `/tf` and `/odom`: **same DiffDrive plugin, same rate, same QoS**,
one crosses and one does not. Size is the only difference. A synchronous writer hands
the sample to the transport on the caller's thread and cannot fragment it, so anything
that will not go out in one piece is discarded with nothing logged on either side.
`<publishMode><kind>ASYNCHRONOUS</kind></publishMode>` as a default `data_writer`
profile fixes it for every ROS publisher without naming them.

`/imu` was the one topic still undelivered afterwards, and it was removed from
`bridge_split.yaml` rather than fixed: nothing in this stack subscribes to it, and
`gz_waffle.sdf.xacro` runs it at 200 Hz of sim time, which is a 200 Hz publisher on a
TCP link to an emulated guest for no consumer. Same reasoning that dropped
`/joint_states`.

### The clock: fixed for every node except bt_navigator

`/clock` no longer crosses the link raw. `bridge_split.yaml` puts Gazebo's 100 Hz
clock on `/clock_gz`, and `clock_decimator.py` republishes the real `/clock` at
`CLOCK_HZ`. Decimation is by **sim time**, not by message count, so a throttled
simulator does not silently change the output rate.

Two things had to be built around it, and both are properties of the design rather
than of this bug:

- **one listening port per host participant that must reach the guest.** The
  decimator first shared `host_tcp_server.xml`, could not bind 42100 because
  `parameter_bridge` already owned it, and ran with no TCP acceptor: it forwarded
  correctly (`in 1903, out 351`) while the guest listed `/clock` and received
  nothing. It now has `host_clock_server.xml` and port **42101**, with a matching
  entry in the guest's `initialPeersList`. `cmd_vel_watchdog.py` needs the same
  treatment on 42102 (Step 7);
- **`useBuiltinTransports` on the host flipped to `true`.** With it false, the only
  host participant that can talk to anything is the 42100 owner, and only to the
  guest — so a decimator that subscribes to the bridge's `/clock_gz` was impossible
  to build at all. The original reasoning for `false` was never measured; flipped
  and re-measured, G3 and G4 both still pass (G4 p95 12.1 → 20.8 ms, still 24×
  inside budget).

The effect, measured across four runs:

| Clock on the wire | `planner_server` lag | `bt_navigator` lag | Bringup |
|---|---|---|---|
| 100 Hz (raw) | 65 s | 65 s | 235 s |
| 18 Hz | 3–9 s | 109 s | 100 s |
| 5 Hz | **no errors at all** | 42 s | 90 s |
| 5 Hz + `bt_loop_duration: 200` | no errors | 34–42 s, **2 errors** instead of dozens | 90 s |

Bringup fell from 235 s to 90 s, `map→base_footprint` from 34 s to 7 s, and
`planner_server` — which produced eleven extrapolation errors per goal — now produces
none.

### The clock rate was the wrong lever, and here is the measurement that killed it

The plan above was to lower `CLOCK_HZ` further and raise `bt_loop_duration` again.
Both halves are now refuted, and the second one was refuted by re-reading the
evidence that was supposed to support it.

**`bt_loop_duration` cannot explain the initial lag.** The first failing lookup
happens when the goal *arrives*, before the tree has ticked once. At that instant
`bt_navigator` was already 44 s behind. Raising 10 ms → 200 ms cut the *number* of
logged errors because fewer ticks means fewer repeated lookups; the gap itself did
not move (34–42 s before, 34–42 s after).

**The clock is a latest-value, not a queue.** `clock_lag_probe.py --block 15` in the
guest: the clock froze at 296.00 for the block, then went 296 → 316 in the next
4.9 s (`sim_rate 4.08x`) while `received` rose by only 10, the ordinary publish
rate. It became current *without consuming a backlog* — the ~30 samples published
during the block were dropped. `KEEP_LAST(1)` behaves as documented even over TCP,
so one callback is enough to become current however far behind a node is, and a
blocking operation leaves no lasting debt. The theory that each sample only advances
a node's clock by one decimation period, so a slower clock catches up faster, is
simply not what happens.

**And the run confirmed it.** `CLOCK_HZ=2`, single variable against the 5 Hz run:

| | 5 Hz | 2 Hz |
|---|---|---|
| `bt_navigator` clock rate | 0.21× wall | **0.17×** |
| `bt_navigator` lag at the goal | 34–42 s | **49–58 s** |
| `planner_server` errors | **none** | 2 |
| Waypoints | 0/2 | 0/2 |

Halving the rate should have improved the catch-up rate 2.5× under the queue theory.
It did not improve at all, and `planner_server` regressed from clean to failing.
`CLOCK_HZ=5` is the better setting and the clock rate is not the constraint.

### What the probe settled: the clock path is healthy, the guest is not

The decisive comparison, same guest, same instant, from the `CLOCK_HZ=2` run:

| | sim time |
|---|---|
| actual simulation (TF buffer front + 10 s cache) | ~443.4 |
| `clock_lag_probe` — one idle participant | **443.53**, 100% delivery, `sim_rate 1.00x` |
| `bt_navigator` | **385.33** |

A node in that guest *can* hold current sim time to within a fraction of a second
across the TCP link. So the link, the decimator and the rate are all exonerated, and
the deficit is inside the loaded Nav2 processes. `run_split.sh` step 7 now measures
this before sending any goal, because the symptom Nav2 prints names TF, not the clock.

`ps` in the guest with the stack up says what the resource actually is:

```
cores 4, loadavg 23.12         <- 5.8x oversubscribed
93.1%  python3            <- liveliness_watchdog.py, the largest single consumer
34.7%  controller_server        33.6% bt_navigator      33.5% planner_server
31.4%  async_slam_toolbox       25.8% behavior_server   26.8% python3 (the probe)
```

**Our own instrument was the biggest consumer.** `liveliness_watchdog.py` subscribed
to `/scan` (360 floats), `/odom` (25 Hz) and `/map` (11 322 cells) in Python and
deserialised every message — to record *arrival timestamps*, having never read a
field. It now subscribes with `raw=True`, which hands the callback the bytes and
skips all of it. An instrument that changes what it measures is worse than none.

Two other defects this exposed, both of which would have produced wrong numbers
rather than failures:

- **G6 was measuring three processes of eight.** `ps -C
  async_slam_toolbox_node,controller_server,planner_server` came to ~99% against a
  budget of 250 of 400, which reads as comfortable; `bt_navigator`,
  `behavior_server` and both Python nodes were never counted. Step 10 now lists
  every workload process with its arguments, sums that set, and prints `loadavg`
  beside it — `%CPU` cannot show queueing, and eight processes at 30% on four cores
  is a very different machine from two at 120%;
- **the map was being scored in the wrong place.** `MAP_REF` is nav2_bringup's
  `tb3_sandbox`, which is installed in the container and deliberately *not* in the
  guest, so `score_map.py` there died with `FileNotFoundError` on a map that had
  saved perfectly. The map now comes back to the host and is scored beside the same
  reference G2 uses. Also `save_map_timeout` had to go from its 2.0 s default to
  30.0 — two earlier runs lost their map to a timeout 2.1 s after `Saving map from
  'map' topic`, and `use_sim_time` is now false there so a stalled clock cannot turn
  that timeout into a hang.

Scoring the map from the run where the robot never moved is a good check on the
metric: **precision 0.962, recall 0.990, coverage 0.133**. Accuracy is excellent on
the little it could see from the spawn point, and only coverage catches it —
precision and recall alone would have passed a run in which nothing happened.

### G6, measured properly for the first time: 279% of 400%

The load-reduced run completed all ten steps. The fixed step 10 reports what the old
three-process version could not:

```
cores 4, loadavg 20.73
workload total 279% of 400% available, 8 processes
```

**279% against a budget of 250.** The old measurement would have said ~99%. Of that
279%, **109% is this project's own instrumentation** — `liveliness_watchdog.py` at 69%
(down from 93% with `raw=True`, so most of rclpy's per-message cost here is callback
dispatch and not deserialisation) and `clock_lag_probe.py` at 40% for a single 5 Hz
subscription. Nav2's five nodes and slam_toolbox come to ~170% between them, which is
inside budget. So G6 is reportable either way, but only if the report says which
processes are counted: the workload is ~170%, the bench with its instruments attached
is 279%.

The load reduction did not fix the clock, and the numbers say why it could not: cutting
`/odom` and `/tf` from 25 Hz to 10 Hz and making the watchdog `raw` moved loadavg from
23.12 to 20.73. Bringup did improve, 90 s → **85 s**, the best so far.

What the same run showed about the lag, and it is no longer one node:

| | requested | buffer starts at | behind |
|---|---|---|---|
| `bt_navigator` | 346.69 | 372.00 | **~30 s** |
| `planner_server` | 350.99 | 361.00 | ~12 s |
| `global_costmap` (a third symptom, `TF Exception that should never happen`) | 361.00 | 373.89 | ~13 s |
| `clock_lag_probe` | — | — | **0**, `sim_rate 1.00x` |

Every heavy node lags and the trivial one does not. `bt_navigator`'s clock advanced
1.70 s in 6.43 s of wall time (0.26x) while the TF buffer advanced at full sim rate, so
the gap widens on its own. This is a property of the guest's throughput, not of any one
node's parameters.

Also fixed here, found while reading the RTF path for the next experiment:
`run_split.sh` still called `gz service ... set_physics` at runtime -- the exact call
that failed G2b five runs out of five, because `gz.msgs.Physics` has no optional fields
and a request naming only `real_time_factor` also sends `max_step_size 0`, stopping
rigid-body motion. `run_container_loop.sh` was fixed at the time and this file was
missed, so `RTF_LIMIT` here was a trap and not a knob. It now goes through
`make_world.py --rtf`, declaratively, like the container script.

### Untried, and the argument for it

Slow the simulation. The guest cannot keep up with real time, so stop asking it to: at
real-time factor R every lag measured in *sim* seconds scales by R, and the 10 s tf2
buffer becomes 10/R seconds of wall time to reach it in. At R = 0.5 the 30 s gap above
would be ~15 s of sim time against a buffer worth 20 s of wall clock. Nothing else
tried so far has that property -- lowering the clock rate does not shrink the gap
(measured), and cutting message load moved loadavg by 10%.

It also satisfies this file's own rule from G2b: a functional result has to be
reproduced at two different real-time factors before it is believed. G2b already passed
at R = 0.5, so the path is validated; `run_split.sh` needed the fix above before it
could be used.

### The remaining blocker: bt_navigator's own clock, on emulated hardware

Everything else in the guest keeps up. `bt_navigator` does not: at the instant a goal
arrives it asks for the robot pose at a timestamp 34–42 s older than the oldest entry
in its own TF buffer, and aborts on `Initial robot pose is not available`. The robot
never receives a velocity command, which is why every run reports the pose constant
at the origin and `0/2` waypoints.

Why this node specifically: it ticks its behaviour tree on an `rclcpp::WallRate`, and
each tick does TF lookups and blackboard work on the same single-threaded executor
that has to service `/clock`. `bt_loop_duration: 200` (upstream 10 ms, i.e. 100 ticks
per second) cut the failure count by an order of magnitude but did not close the gap.
Three options, none yet tried, in the order worth trying:

1. lower `CLOCK_HZ` further (2 Hz) and raise `bt_loop_duration` again. Cheap, and the
   trend above is monotonic in both — but a 2 Hz clock coarsens every timeout Nav2 has;
2. cut total guest load so the executor gets more of the four emulated cores:
   `/tf` and `/odom` are 25 Hz each and neither needs to be, and slam_toolbox's Ceres
   solve competes for the same cores;
3. give `bt_navigator` a multi-threaded executor, which is not a parameter — it needs
   a launch-level change or a small wrapper node.

What this is *not*: the TF stamps arriving beside the clock are current, so the link is
not backlogged; `planner_server` on the identical clock has no errors, so the clock on
the wire is adequate; and the guest nodes' `exit code -9` entries are this script's own
SIGKILL during teardown.

### Superseded: the original clock diagnosis

Both goals aborted in 7 s, and the guest stack log says exactly why:

```
[bt_navigator] [ERROR] [transformPoseInTargetFrame]: Extrapolation Error looking up
  target frame: Lookup would require extrapolation into the past. Requested time
  489.820000 but the earliest data is at time 555.400000, when looking up transform
  from frame [base_footprint] to frame [map]
[bt_navigator] [ERROR]: Initial robot pose is not available.
[bt_navigator] [WARN]: [navigate_to_pose] [ActionServer] Aborting handle.
```

That is a single node comparing its **own** clock against TF stamps arriving over the
**same** link: `now` = 489.82 while the oldest entry in its TF buffer is already
555.40. The guest's sim time is **65.6 s behind** the simulator, and both goals then
fail on `Initial robot pose is not available` rather than on anything to do with
planning or the map.

The mechanism to check first: every topic shares **one** TCP connection, and every
guest node with `use_sim_time` runs its own `/clock` subscription. Seven guest nodes
against a 100 Hz clock is ~700 clock callbacks per second inside TCG, on top of
`/scan`, `/odom` and `/tf`. `rclcpp::ClockQoS` is KEEP_LAST 1 and BEST_EFFORT, so the
reader keeps only the newest sample it has actually been handed — which does not help
when the delay is in delivery rather than in the queue depth.

That fix is now built and measured above. Original note kept for the record: Its
100 Hz comes from `max_step_size: 0.01`, so it is tied to physics resolution today;
a small host-side node that subscribes to gz `/clock` and republishes at 20–25 Hz on
ROS `/clock`, with `/clock` removed from `bridge_split.yaml`, decouples the two. Nav2
and slam_toolbox need a monotonic clock, not a 100 Hz one. Verify by re-reading the
same `bt_navigator` numbers: the gap between `Requested time` and `earliest data`
should go to zero.

Two things NOT to conclude from this run: the guest nodes' `exit code -9` entries are
this script's own SIGKILL escalation during teardown, not crashes; and `map_saver`
finding no `/map` is downstream of the clock lag, since `send_goal.py` had already read
a `111x102` map from the same topic minutes earlier.


---

## Re-layout, 2026-08-18

The directory had accumulated 229 machine-written files in its root beside 33
sources. Restructured, no behaviour change intended:

| Now | Was |
|---|---|
| `scripts/*.sh` | root — the seven runners and checks |
| `tools/*.py` | root — world generation, probes, scorer, launch file |
| `config/*.{yaml,xml}` | root — Nav2/SLAM params, bridge configs, DDS profiles |
| `artifacts/` (gitignored wholesale) | root — every run log, CSV, generated `robot.env`/`world.sdf`/`robot.sdf`/`robot.urdf`/`nav2_split.gen.yaml`, saved maps (`maps/` and the root `split-*.{pgm,yaml}`) |
| `logs/` | unchanged — curated transcripts this file cites by name |

What had to change with it, because paths are baked into code:

- `make_world.py` writes `BRIDGE_FILE="/work/config/…"` and defaults
  `--out-dir /work/artifacts`; stale root copies of its outputs were deleted so
  nothing can read a pre-move `robot.env` whose paths no longer exist.
- Every `/work/<name>` reference in the runners gained its `tools/`, `config/`
  or `artifacts/` prefix (262 references, inventoried mechanically before the
  move); each script now computes `ROOT` from `$0` and `mkdir -p`s
  `artifacts/maps`.
- Four pre-existing CWD dependences fixed while at it: `./type_hashes.py` in
  `00-preflight.sh`/`check_interop.sh` (with the silent
  `docker cp … >/dev/null 2>&1` now failing loudly), the viewer container's
  `-v "$PWD":/work` in `run_demo.sh`, the host-side bare-name
  `awk … nav2_split.yaml` in `run_container_loop.sh`, and `hashes.guest.txt`
  written to `.` by preflight but read as `./hashes.guest.txt` by interop —
  both now anchor on `artifacts/hashes.guest.txt`.
- `run_demo.sh --shot/--record` media moved from `logs/` (curated) to
  `artifacts/`; its VNC warning pointed at `logs/demo-vnc-*.log` while writing
  to the root — now both say `artifacts/`.
- `.gitignore`/`.dockerignore` rewritten around `artifacts/`; the previous
  rules never matched the root `split-*.pgm` maps, which were one `git add -A`
  away from being committed.

Verified after the move: `bash -n` ×7, `py_compile` ×12, `yaml.safe_load` ×4,
`xmllint` ×3 all clean; in the live container `make_world.py --robot tb3
--out-dir /work/artifacts` produced a `robot.env` with the new paths, and
`tools/score_map.py` re-scored `artifacts/maps/split-20260817-225847.yaml` to
the same numbers as the 08-17 run (precision 0.951, coverage 0.138).

---

## Step 8 again, 2026-08-19: G5 passes, and none of it was the clock

Three runs, each isolating one thing. The robot now drives the whole tour and the
map it builds meets every floor.

| | attempt 1 | attempt 2 | attempt 3 |
|---|---|---|---|
| changed | `WATCHDOG=` + `RTF_LIMIT=0.5` | `default_server_timeout` in the repo | the same, actually on the guest |
| bringup | 3m23s | 60 s | 70 s |
| clock lag | **2.9 s** | **0.5 s** | 3.1 s |
| waypoints | 0/2, `status=6` in 2 s | 0/2, wp1 rejected, wp2 `status=6` | **2/2, 32 s and 36 s** |
| map | coverage 0.135 | coverage 0.135 | **precision 0.937, recall 0.969, coverage 0.983** |
| G6 | 250% of 400% | — | **249% of 400%**, loadavg 24.96 |
| G7 | — | — | **0 stalls** |
| verdict | FAIL | FAIL | **PASS** |

### The clock was fixed by two levers, and then stopped being the problem

`liveliness_watchdog.py` off returns the 69% of a core Step 8 measured it taking,
and `RTF_LIMIT=0.5` halves how much sim time passes per wall second, so a node
that cannot keep up has twice as long to. Together the lag went from the 30-42 s
that had aborted every previous run to **2.9 s on the first try**, and
`Initial robot pose is not available` has not appeared since. The argument for
slowing the simulation was written up under "Untried" the day before; it works.

Two notes on what this does *not* mean. The lag is not zero and does not need to
be — `CLOCK_LAG_MAX` was raised to 8 s for these runs and the measured 0.5-3.1 s
sits well inside it. And `WATCHDOG=` is a demonstration setting: G6 still needs
the watchdog, so a measurement run keeps it and pays the 69%.

### What was actually blocking the drive: a 20 ms timeout

With the clock healthy, attempt 1 failed in a completely new way — the goal was
accepted, TF resolved in 0 s, the map arrived, and then:

```
[bt_navigator] Begin navigating from current location (0.00, -0.00) to (3.83, 0.58)
[bt_navigator_navigate_to_pose_rclcpp_node] Timed out while waiting for action
    server to acknowledge goal request for compute_path_to_pose
[bt_navigator] [navigate_to_pose] [ActionServer] Aborting handle.
[bt_navigator] Goal failed
```

with **no message from `planner_server` at all** — it was never asked. That is
`default_server_timeout`, upstream 20 ms, which is how long a BT action node
waits for a server to *acknowledge* a goal. Not to plan one, not to drive it:
to say "received". On an emulated CPU with seven Nav2 processes and slam_toolbox
resident, 20 ms is not enough for a round trip that never leaves the guest.

The value was already known here. `../nav2/loopback_nav.yaml` has carried
`default_server_timeout: 120` since the Milestone A work, with
`wait_for_service_timeout: 30000` and `action_server_result_timeout: 900.0`
beside it. None of the three had been carried into `nav2_split.yaml`, which adds
a TCP link and a heavier guest on top of the case those numbers were measured
for. They are now there at 5000 / 30000 / 900.0.

The symptom moves rather than disappearing if the timeout is only partly raised:
attempt 2 showed the same message for `follow_path` instead of
`compute_path_to_pose`, and wp1 failed differently again —
`Failed to send goal response … (timeout): client will not receive response`,
which `send_goal.py` reports as `goal rejected` 60 s later. Same cause, three
faces.

### The run that proved it also proved the deployment was broken

Attempt 2 changed nothing, because the fix never reached the guest.
`run_split.sh` step 0 pushed exactly two files — `clock_lag_probe.py` and
`liveliness_watchdog.py` — on the argument that those are what the run's
conclusions rest on. True, and too narrow: `nav2_split.yaml` on the guest was
dated 08-17 and contained no `default_server_timeout`, so attempt 2 ran the old
parameters and failed with the old symptom while the repository held the fix.

A stale guest file cannot announce itself. It can only produce a result that
describes code nobody is reading — which is the same failure class as the
`set_physics` trap and the pre-move `robot.env`, and it cost a seven-minute run
to see. Step 0 now pushes `tools/*.py` and `config/*.yaml` on every run; both
directories are a few hundred kB and one wasted run costs more.

### Where the CPU went

```
cores 4, loadavg 24.96 15.77 9.54
 52.5%  clock_lag_probe.py
 39.9%  bt_navigator
 38.5%  async_slam_toolbox_node
 38.2%  controller_server
 35.3%  planner_server
 28.3%  behavior_server
 16.0%  lifecycle_manager
workload total 249% of 400% available, 7 processes
```

Inside the 250% budget, but only because the watchdog is gone — and the largest
single consumer is again an instrument, `clock_lag_probe.py` at 52.5% for one
5 Hz subscription. Step 8's finding holds: rclpy's per-message cost on this guest
is dominated by callback dispatch, and any probe that subscribes to a periodic
topic is expensive whatever it does with the message.

That number is the constraint on the next step. YOLOv8n as a third co-tenant has
to fit in the ~150% that is left, and it is not free: one 320x320 inference is
11.4 s of CPU. G-Y5 has not been run.

### The reproduction passed the drive and failed the map, and the map was fine

Attempt 4 repeated attempt 3 with nothing changed: bringup 65 s, clock lag
-0.7 s, **2/2 waypoints** in 30 s and 34 s. Then `score_map.py` returned
coverage **0.598** against a 0.90 floor and the run was recorded FAIL.

The two maps are the same map. Side by side:

| | attempt 3 | attempt 4 |
|---|---|---|
| grid | 112x103 @ 0.05 | 112x103 @ 0.05 |
| origin | (-0.949, -2.072) | (-0.952, -2.075) |
| occupied / free / unknown | 657 / 7741 / 3138 | 640 / 7706 / 3190 |
| known-cell extent | x[-0.92, 4.63] y[-2.05, 3.05] | x[-0.93, 4.62] y[-2.05, 3.05] |
| coverage | 0.983 | **0.598** |

Rendered as ASCII they are indistinguishable — same hexagonal arena, same four
pillars, same doorways. Three millimetres of origin separate them.

`coverage()` projects every known SLAM cell onto the reference grid and counts
the distinct reference cells hit. Measured directly:

```
attempt 3: slam_known=8398  in_bounds=8398  distinct_ref_cells=8398   (1:1)
attempt 4: slam_known=8346  in_bounds=8346  distinct_ref_cells=5199   (1.6:1)
```

3147 of attempt 4's cells landed on a reference cell another had already
claimed. The projection is `floor((origin + (i+0.5)*res + offset - ref_origin)
/ ref_res)`, and for attempt 4 the y term evaluates to **exactly 149.0 + iy**.
On an exact cell boundary the result is one ULP from either neighbour, and which
one `floor` picks depends on how the value accumulated — so rows collapsed in
pairs, leaving half the reference rows never marked. Attempt 3's origin put the
same expression at 149.06 + iy, comfortably inside a cell, and projected 1:1.

So the gate was flipping PASS/FAIL on a defect in the scorer, and the failing
side was reporting a mapping result it had not measured. That is the second
metric in this file to fail that way; the first was the occupied-IoU gate,
retired 08-17 for having a ceiling below its own threshold.

Fixed two ways, because the two paths have different needs:

- `coverage()` now takes an **integer translation** when the two resolutions are
  equal, which is every run here. `sx = round((slam_ox + dx - ref_ox)/res)` and
  `cx = ix + sx` is 1:1 by construction — no two cells can collide and none can
  be skipped. The float path remains for unequal resolutions.
- `sample()`, which takes arbitrary world coordinates and cannot use an integer
  shift, floors with a `CELL_EPS = 1e-6` cell nudge so an exact boundary resolves
  upward every time. At 0.05 m that is 50 nm.
- `coverage()` also now **warns** when projected cells collapse. A projection
  that loses cells is a scorer defect and it reads as low coverage; it should say
  so rather than quietly returning the number.

Re-scored with the fix, no other change:

| map | before | after |
|---|---|---|
| attempt 3 (drove) | 0.983 PASS | **0.983 PASS** |
| attempt 4 (drove) | 0.598 FAIL | **0.979 PASS** |
| attempt 2 (never moved) | 0.135 FAIL | **0.135 FAIL** |

The third row is the one that matters: the metric still fails a map of a robot
that sat still, so the fix removed an artefact and not the gate.

**G5 is reproducible.** Two consecutive runs, 2/2 waypoints each, precision
0.937 / 0.955, recall 0.969 / 0.979, coverage 0.983 / 0.979, G6 249% and 252% of
400%, G7 zero stalls.

---

## G-Y5, 2026-08-19: the third co-tenant, and what it costs

`YOLO=1` spawns a camera and photo billboards into the running world, appends one
entry to a copy of the bridge config, and starts `yolo_node` in the guest beside
Nav2 and slam_toolbox. This is the workload TC.10.007 actually names.

### Attempt 1 — the detector worked, the navigator did not

| | without YOLO | with YOLO |
|---|---|---|
| bringup | 65–70 s | **110 s** |
| clock lag | −0.7 to 3.1 s | 2.8 s |
| waypoints | 2/2 | **0/2**, aborted at 13 s and 15 s |
| G6 | 249% of 400%, 7 processes | **304% of 400%, 8 processes** |
| detections | — | **12**, `person:0.83 … car:0.48` |

The detector passed its own gate and the numbers say why the navigator did not:

```
[yolo_detector] [detect] seq=11 stamp=220.000 n=6 infer_s=31.5 rx=15 ...
112 234708  24  395 python3 /home/vela/yolo/yolo_node.py
```

`yolo_node` at **112% of a core** is the largest single consumer in the guest,
and one inference went from the 11.4 s measured alone to **25.9–31.5 s**. Nav2
then failed with the symptom this rig spent a day removing:

```
[transformPoseInTargetFrame] Extrapolation Error looking up target frame:
    Requested time 202.480000 but the earliest data is at time 209.800000
[bt_navigator] Initial robot pose is not available.
```

7.3 s behind the oldest entry in its own TF buffer — clock starvation, back
because a third workload took the headroom that made it go away.

**`rx=15` is the part worth acting on.** Fifteen frames arrived per inference and
the node kept the newest, by design. But the guest deserialised all fifteen:
640x480 rgb8 is 900 kB, so that is ~13.5 MB of TCP and rclpy object construction
per detection, almost all of it discarded. The model letterboxes to 320x320
whatever it is given, so the large frame bought no accuracy at all.

Two levers for attempt 2, both aimed at giving the clock room rather than at the
detector:

- camera **320x240**, a quarter of the bytes for the same detections;
- **`RTF_LIMIT=0.25`** — the lever that fixed the same symptom at 0.5, applied
  again. Lag measured in sim seconds scales with the real-time factor.

The `[detect]` line also confirms `make_billboards.py` works as intended: six
detections per frame, stable across passes, and the SLAM map is unaffected by the
scene because it is spawned 30 m outside the arena.

### Attempt 2 — all three, passing

Two changes, both aimed at the clock rather than at the detector: camera
**320x240** and **`RTF_LIMIT=0.25`**.

| | attempt 1 | attempt 2 |
|---|---|---|
| camera | 640x480 | **320x240** |
| RTF | 0.50 | **0.25** |
| bringup | 110 s | **80 s** |
| clock lag | 2.8 s | **-0.6 s** |
| waypoints | 0/2 | **2/2**, 59 s and 66 s |
| precision / recall / coverage | — (0.142) | **0.992 / 0.990 / 0.973** |
| G6 | 304% of 400% | **272% of 400%** |
| detections | 12 | **9** |
| verdict | FAIL | **PASS (G5 and G-Y5)** |

Nav2, slam_toolbox and YOLOv8n in one four-vCPU emulated guest, against one
simulator, with the robot driving the whole tour — the workload TC.10.007 names,
running together for the first time.

Two things in that table are worth reading carefully.

**The map is the best this rig has produced** — 0.992 / 0.990 / 0.973, better
than either run without the detector. Not because the detector helps: at RTF 0.25
the robot covers the same ground in twice the sim-normalised wall time, so
slam_toolbox gets roughly twice as many scans matched per metre travelled. It is
the real-time factor doing it, and it is the same lever that fixed the clock.

**The load went DOWN when the frames got smaller** — 304% to 272% — while the
detector kept working. 32 points of an emulated core were being spent turning
900 kB images into Python objects for frames the node then discarded. Shrinking
the camera to the size the model letterboxes to anyway cost nothing and returned
all of it.

Fewer detections (9 vs 12) is the expected consequence of a longer wall-clock run
at a lower sim rate, not a regression: G-Y5 asserts that the detector kept
producing under co-tenancy, and how fast it infers is the emulator's property.

### The viewer, verified rather than documented

`Dockerfile.demo` gained `ros-jazzy-rviz2` and `run_demo.sh` gained `RVIZ=1`,
which starts RViz on the Xvfb display beside the Gazebo GUI and starts
`detection_overlay.py` with it.

Why both windows: the Gazebo GUI shows ground truth, and it cannot show the map
slam_toolbox built, the path Nav2 planned, or what the detector found — those are
the guest's opinions, arriving back over the TCP link. A demonstration of this rig
that shows only Gazebo shows one of the three workloads.

The image cost +10 MB, not the few hundred expected: `ros-jazzy-slam-toolbox`
already dragged Qt 5 into the base image, which `DESIGN.md` files as a packaging
defect and which happens to pay for itself here.

Smoke-tested on a bare Xvfb with no ROS graph running, because a `.rviz` file
with a wrong display class or a mismatched QoS fails **silently** — the panel
simply stays empty, which during a demonstration is indistinguishable from a
workload that is not working:

```
[INFO] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
RViz is ready.        10 fps
```

All six displays registered — Grid, SLAM map, Lidar, Nav2 plan, TF, YOLO
detections. The two orange warnings are "no messages yet", which is correct with
nothing publishing. Screenshot: `artifacts/rviz-smoke.png`.

`/map` is the one to watch: slam_toolbox publishes it TRANSIENT_LOCAL, and a Map
display left on the default VOLATILE durability stays empty forever with no error
anywhere. `config/demo.rviz` sets it explicitly.

---

## TSP gap closure, 2026-08-19

A clause-by-clause reading of the TSP against this rig found two things the
§3.1.2 figure names and this rig did not have (`explore_lite`, host teleop), the
TSP's own robot never driven in the split (tb4), and no measured figure for the
topic-latency budget the test plan gates on. Closing those in order surfaced a
regression that had nothing to do with any of them.

### The shared bridge config had silently broken G2

`bridge_split.yaml` is used by **both** runners: `run_container_loop.sh` (G2/G2b,
container-local) and `run_split.sh` (G5, split). When the clock decimator was
added for G5 on 08-17, the rename was done **inside that shared config** — the
ROS side of Gazebo's clock became `/clock_gz` so `clock_decimator.py` could sit
between it and the guest.

That is correct for G5 and wrong for G2, and nothing caught it:

| | |
|---|---|
| last `run_container_loop.sh` run | `artifacts/stack-20260817-195011.log`, **08-17 19:50** |
| `bridge_split.yaml` clock rename | mtime **08-17 21:34** |
| `clock_decimator.py` created | mtime 08-17 21:35 |
| `run_container_loop.sh` decimator | **none** — `grep -c decimator` = 0 |
| what its nodes ask for | `use_sim_time:=true` (`slam_nav.launch.py:110` default, not overridden) |

So from 21:34 onward, a G2 run would have started Gazebo, Nav2 and slam_toolbox
with **no `/clock` publisher anywhere** while every node waited on sim time. The
G2/G2b numbers in Step 6 were measured 1 h 44 m before the change and are sound;
they were simply no longer reproducible from the tree. The config's own comment
asserted the opposite — "run_container_loop.sh does not need the decimator and
keeps /clock straight through" — which was true when written and false 90 seconds
later.

**The fix puts the indirection in the runner that needs it, not in the file both
share.** Both bridge configs now publish a plain `/clock`; `run_split.sh` starts
the bridge with `-r /clock:=/clock_gz` and the decimator behind it. G2 gets its
clock back with no decimator in the path, so what Step 6 measured is what Step 6
would measure again.

Verified in the container before the tb4 run, both paths, no simulator needed —
`parameter_bridge` advertises at startup whether or not Gazebo is publishing:

```
A. bridge_split.yaml, no remap   (run_container_loop.sh)  ->  /clock
B. bridge_tb4.yaml, with remap   (run_split.sh)           ->  /clock_gz
   /clock_gz /cmd_vel /odom /scan /tf        5 bridges created
```

The general lesson is the one this log keeps relearning in different clothes: a
file two runners share cannot carry a setting only one of them wants. `robot.env`
gets this right by deriving everything per robot; the bridge configs did not.

### `bridge_tb4.yaml` carried two defects already fixed for tb3

tb4 is the robot the TSP's 4th-year scenario names, and its bridge config had not
been touched since 08-14 — before the split run existed. It therefore still had:

| Defect | Consequence on a tb4 split run | Fixed for tb3 in |
|---|---|---|
| `clock` bridged straight to the guest | the exact failure Step 8 spent a day on: seven guest nodes servicing a 100 Hz clock, `bt_navigator` falling tens of seconds behind the TF stamps arriving beside it, every goal aborting on `Initial robot pose is not available`. The decimator would have been a silent no-op — it subscribes to `/clock_gz`, which nothing would have published, and the existing check only greps that it *started* | `bridge_split.yaml`, 08-17 |
| `/imu` bridged | no subscriber exists — no `robot_localization`, and slam_toolbox works from `/scan` and odom. `imu.urdf.xacro` defaults `update_rate:=200`, so this put a 200 Hz publisher on the TCP link into an emulated guest for no consumer | `bridge_split.yaml`, 08-17 |

Both removed. The two robots' bridge sets are now the same five topics —
`/clock`, `scan`, `odom`, `tf`, `cmd_vel` — which is also what the README diagram
now shows; it had still listed `/imu` for tb3, a topic that config dropped on
08-17.

Confirmed no consumer before removing, rather than after: `grep -i imu` over
`nav2_split.yaml`, `mapper_params_tcg.yaml` and `slam_nav.launch.py` returns
nothing.

### G5 with tb4, 2026-08-19: the TSP's robot drives, and the map gate had the wrong offset

First split live run with `ROBOT=tb4` — the robot the TSP's 4th-year scenario
names. `WATCHDOG= RTF_LIMIT=0.5 CLOCK_LAG_MAX=8`, the settings tb3 was proven
with. `logs/g5-split-tb4.log`.

| | tb4, this run | tb3, 08-19 for contrast |
|---|---|---|
| bringup to all-active | **123 s**, wall clock between log lines | 65–70 s, the runner's own "active after Ns" counter — a different instrument, so read the two as "same order", not as a ratio |
| waypoints | **4/4 SUCCEEDED** | 2/2 |
| per leg | **16 / 19 / 16 / 16 s** | 30–36 s |
| final error per goal | 0.337–0.389 m (tol 0.25, cross-check 0.5) | similar |
| `real_time_factor` held | **0.50000030** | 0.5 |
| G6 workload | **270% of 400%**, 8 processes, loadavg 14.64 | 249–252% |
| G7 stalls | **0** | 0 |
| map, as run | precision 0.421, recall 0.283, coverage 0.241 → FAIL | 0.937–0.955 / 0.969–0.979 / 0.979–0.983 |
| map, offset corrected | **0.792 / 0.968 / 0.387 → PASS** | unchanged |

Depot is a 30 m warehouse, the tour is four waypoints rather than two, and the
legs came in at half tb3's time. Nothing about tb4 is harder for this rig than
tb3 was.

**`MAP_OFFSET` was wrong for tb4, and it was wrong because it was derived.**
`make_world.py` wrote the offset as the spawn pose, on the reasoning that the SLAM
frame starts at the spawn so world = slam + spawn. That is right for tb3 and wrong
for tb4, because the offset's job is to reach *the reference map's* frame and the
two shipped references do not use the same one:

| Reference | Origin | Extent in its own frame | Frame | Offset |
|---|---|---|---|---|
| `tb3_sandbox.yaml` | (−10.00, −10.00) | 19.2 m square about that origin | world | the spawn, (−2, −0.5) |
| `depot.yaml` | (−7.14, −7.83) | 30.2 × 15.4 m | **the SLAM frame, i.e. spawn-relative** | **(0, 0)** |

`tb3_sandbox` is a 19.2 m square centred on the world origin, which is what a
world-frame map of that arena looks like; its offset is therefore the spawn pose.

The tell for depot was in the scorer's own first three lines all along: the SLAM
map's origin is (−7.13, −7.68) and the reference's is (−7.14, −7.83). Two grids do
not land within 0.15 m of each other in their own frames unless they share one — and
the SLAM frame starts at the spawn, so depot.pgm's does too. Note this coincidence
test is specific to depot; for tb3 the two origins are 8-9 m apart, exactly as a
world-frame reference against a spawn-relative SLAM map should be.

| `--offset` | compared cells | precision | recall | coverage |
|---|---|---|---|---|
| −8, 0 (derived) | 44 192 | 0.421 | 0.283 | 0.241 |
| **0, 0 (correct)** | **70 422** | **0.792** | **0.968** | **0.387** |

`map_offset` is now a per-robot value in `make_world.py` rather than a
spawn-derived one, with the frame difference written down beside it.

**My first diagnosis of this was wrong, and the way it was wrong is worth keeping.**
It concluded the reference map did not describe this world at all — that 48% of
SLAM's occupied cells fell outside it and the rest disagreed — and that the three
tb4 floors therefore had to be zeroed. Both halves of the evidence were real
measurements of an artefact: at a 7 m offset error most of the SLAM map *does* fall
outside the reference, and what remains *does* disagree. The reasoning failure was
in the sweep used to rule out an offset error. It covered dx ∈ [−8.5, −7.0] — a
window centred on the value being tested — found a shallow monotone ridge with no
peak, and concluded no offset could explain it. The peak was at dx = 0, seven
metres outside the window:

```
dx      -4     -1    -0.4   -0.2   -0.1    0     +0.1   +0.2   +1     +4
prec   0.379  0.437  0.519  0.547  0.608  0.792  0.636  0.531  0.430  0.404
```

Sharp, and monotone on both sides. **A sweep that starts from the value under
suspicion tests its neighbourhood, not the hypothesis.** Sweep through zero, and
plot the profile rather than reading its endpoints.

Two other things the wrong diagnosis leaned on, and what they were actually saying:

- *"Widening the tolerance to 0.30 m moves recall only 0.283 → 0.320, so it is not
  smearing."* True and correctly reasoned — a 7 m offset is not fixed by a 0.30 m
  tolerance either. The inference "therefore the geometry is absent" skipped the
  offset case entirely.
- *"dy = 0 is a sharp optimum while dx is flat."* Also true, and it was the clue.
  Sharp in one axis and flat in the other means the walls run along the flat axis
  and the sweep is sliding along them — which is exactly what happens when you are
  7 m off along a 30 m wall. It reads as evidence for alignment only if you have
  already assumed dx is near its optimum.

**The tb4 floors, from this run — superseded the same afternoon by a second run;
see "The tb4 map floors, after two runs" below.** They were set to
`precision 0.75`, `coverage 0.30`, recall left at `score_map.py`'s own 0.85, on the
argument that precision belonged below the 0.80 default because `depot.pgm` draws
walls thicker and sparser than the lidar sees them: SLAM marked 1148 occupied cells
where the reference has 991 over the same region, and with recall at 0.968 the whole
shortfall is SLAM finding *more* wall than the reference records. That reading of the
shortfall was right. The floor drawn from it was not, because one run cannot found
one — run 2 scored 0.679 and failed it. Coverage 0.387 is what a four-waypoint tour of 448.7 m² of free space
reaches; it is a tour-length figure, not a map-quality one, and an `EXPLORE=1` run
should leave it far behind. All three come from one run and should be tightened
when a second agrees.

**G6, tb4, with the accounting stated.** 270% of 400% across 8 processes, of which
`clock_lag_probe.py` alone is 51.6% — an instrument, not workload. Workload-only
is **218%**. Same split as 08-19's tb3 run and the same conclusion: the figure is
reportable either way and meaningless unless the report says which processes were
counted.

### Both drive modes the TSP figure names, 2026-08-19

The §3.1.2 figure labels two modes, `Host Teleop(manual)` and
`Explore_lite(autonomous)`. Neither existed; the robot was only ever driven by
`send_goal.py`'s fixed waypoint list. Both are cheap, which is the uncomfortable
part — the gap was not a hard problem, it was an unchecked assumption that the
figure's left column was fully implemented.

**`explore_lite` is two packages.** `ros-jazzy-explore-lite 1.0.0-0noble` is in the
vela riscv64 repository, and `apt-get install -s` reports `2 newly installed,
318 kB` because every C++ dependency — `nav2-costmap-2d`, `nav2-msgs`, `map-msgs`,
`visualization-msgs` — is already there. Against the 602 packages `DESIGN.md` cites
for putting Gazebo in the guest, and the 192 `slam_toolbox` cost.

Everything about the node was read off the installed artefact rather than upstream
documentation, because the two disagree:

| | Value | How it was established |
|---|---|---|
| executable | `explore` | `/opt/ros/jazzy/lib/explore_lite/explore` |
| node name | `explore_node` | the key upstream's own `params_costmap.yaml` uses |
| action | `navigate_to_pose` | string literal in the binary — the same server `send_goal.py` drives, hence the two modes are exclusive |
| parameters | **twelve** | `declare_parameter` name literals in the binary, cross-checked against both shipped yaml files |
| `planner_frequency` clock | **wall** | `strings … \| grep -c N6rclcpp9WallTimerIZN7explore7ExploreC` → 2: `rclcpp::WallTimer` instantiated on a lambda from `Explore`'s own constructor |
| `progress_timeout` clock | unsettled | `_ZNK6rclcpp4Node3nowEv` is in the dynamic undefined table, so sim time is used somewhere, but the binary is stripped and no `.cpp` ships. The value is chosen to be right either way |
| terminal log lines | verified | `No frontiers found, stopping.` / `All frontiers traversed/tried out, stopping.` / `Goal ACCEPTED` / `Adding current goal to black list` / `Robot out of costmap bounds` all present as literals |

The clock question mattered enough to check twice. A first attempt cited
"`create_wall_timer` referenced and no `create_timer`", which cannot be reproduced —
the binary is stripped, `nm -u` returns nothing, and both factories are templates
that leave no undefined symbol either way. The typeinfo name in `.rodata` is the
evidence that survives.

Parameters that had to move off upstream's values, with the reason in
`config/explore_split.yaml`: `robot_base_frame` (`base_link` exists on neither
robot's chain here), `costmap_topic` `/map` (slam_toolbox's own grid — the one the
map gate scores, and a fifth of the deserialisation of Nav2's costmap),
`transform_tolerance` 5.0 (every tolerance in this rig is 5.0 because TF crosses
TCP), `min_frontier_size` 0.30 m (upstream's 0.75 m is wider than the 0.44 m gap the
robot can physically drive through, so it discards reachable frontiers), and
`visualize: false` (no `rviz2` for riscv64, no subscriber).

It is started by `run_split.sh` step 8 and **not** by `slam_nav.launch.py`, because
that file is shared with `run_container_loop.sh` and because `Costmap2DClient`'s
constructor blocks until the first costmap and the first TF — in the launch file
that block would span the whole bringup, and a co-tenant during bringup is measured
harm here: `yolo_node` took bringup from 65–70 s to 110 s and every goal aborted.
Three near-misses are recorded in `DESIGN.md` rather than discovered again: it must
not go in `lifecycle_manager_nav.node_names` (it is a plain node, so the manager
would wait forever), `explore_lite_msgs` must not go in `type_hashes.py` or
`check_interop.sh` (guest-only, so the interop diff would report a false mismatch),
and its own `explore.launch.py` cannot be used (it hardcodes its parameter file).

**Teleop is a host package, and the figure already said so.** `Host Teleop(manual)`
sits in the x86-64 column, and it has to: there is no riscv64 build of
`teleop_twist_keyboard`, and `/cmd_vel` is bridged `ROS_TO_GZ` on the host, so a
host publisher reaches `DiffDrive` without crossing the TCP link at all. It went
into `Dockerfile.demo`, not `Dockerfile`: the measurement image's resolved package
set is `apt-manifest.txt`, which `RUNLOG.md` cites as the environment G1–G7 were
measured in, and rebuilding it would also force a `docker rm` of the long-lived
`vela-gz` and discard the 101 MB Depot Fuel cache in its writable layer.

What manual driving cannot be: simultaneous with Nav2. There is no arbitration on
`/cmd_vel` — the bridge forwards whatever arrives and `DiffDrive` re-applies the
newest command every physics step with no timeout of its own. `controller_server`
publishes at 5 Hz, so a keystroke steers for ≤200 ms of sim time and is then
overwritten, and Nav2 sees only unexplained tracking error.

### `cmd_vel_watchdog.py` had been a no-op in every split run

Found while looking for a host-side `/cmd_vel` observer for the latency work, and
it is this rig's own documented trap sprung on itself. Every split-mode watchdog log
opens with

```
[RTCP_MSG_OUT Error] TCPTransport Error binding at port: (42100) with msg:
    bind: Address already in use -> Function create_acceptor_socket
```

because `run_split.sh` started it on `$HOST_ENV`, i.e. `host_tcp_server.xml`, whose
port the bridge already owns. A Fast DDS participant that loses that bind comes up
with **no TCP acceptor** — discoverable and unreachable. So the watchdog saw no
`/cmd_vel` from the guest at all, and the only reason it did not fight Nav2 is that
its own zero-velocity commands could not reach the bridge either. A no-op in both
directions, in the one place where a latched `DiffDrive` is the hazard.

A census across all 40 watchdog logs: the 16 with the bind error are all split-mode
and all have zero deadman events; of the 24 without it — container-loop runs, where
the participant is local and unconfigured — 18 fired `published zero velocity`. The
correlation is perfect.

`host_clock_server.xml` had reserved the fix in a comment since 08-17: *"42102 free.
`cmd_vel_watchdog.py` needs one for the same reason; it is a silent no-op without
one."* The note was right and the runner kept using 42100. It now has
`config/host_cmdvel_server.xml`, port 42102, plus the matching
`initialPeersList` entry in `guest_tcp_client.xml`.

Which surfaced a second defect of the same family: `run_split.sh` pushed
`tools/*.py config/*.yaml` to the guest every run, under a comment arguing that a
stale guest file cannot announce itself — and `guest_tcp_client.xml`, the file that
makes the guest dial the host's acceptors, is `.xml`. It was the one file the
push-everything rule did not push, so the new peer entry would never have arrived.
The glob is now `*.py *.yaml *.xml`.

Both fixed live in the same run: `artifacts/watchdog-20260819-142941.log` opens with
the `watching /cmd_vel` line and no bind error, and `grep -c 42102
~/gz_split/guest_tcp_client.xml` on the guest returns 2.

### G5-L, 2026-08-19: what the topic budget actually measures

Second tb4 run, `WATCHDOG=1 RTF_LIMIT=0.5`, to produce the topic figure the TSP
gates on. `logs/g5-split-tb4-latency.log`. The instrument is
`liveliness_watchdog.py`, extended to report the whole inter-arrival distribution
rather than only the gaps that breached a threshold.

```
inter-arrival at the consumer, wall ms (budget 1000 ms on max)
  topic           n      p50      p99      max
  /scan         854    399.9    561.4   1556.8  OVER
  /odom        1699    199.9    324.3    551.0  ok
  /cmd_vel      237    208.5   1614.3   1993.4  OVER      <- see below
  /map           51   4978.8   6395.2   7668.2  OVER      <- see below
```

The log says `OVER` on all three, because when it ran, one budget was applied to
every watched topic. The per-topic scope described next was added afterwards, on the
strength of exactly these numbers; re-scored under it, `/cmd_vel` and `/map` read
`not gated` and `/scan` remains the only breach. The verdict for this run stands
either way — `/scan` fails it — but the block above is the log, not the current
tool's output.

**The instrument is calibrated, and the p50s are how you can tell.** `/scan` is
5 Hz of sim, so 200 ms sim, so 400 ms of wall at RTF 0.5 — measured 399.9. `/odom`
is 10 Hz of sim — measured 199.9. Two independent topics landing on their
theoretical wall period to 0.1 ms means the clock, the scaling and the arithmetic
are all right, before any conclusion is drawn from the tails.

The host side of the same measurement, from `cmd_vel_watchdog.py` now that it owns
port 42102 and can actually see the guest: `n=237, p50 209.4, p99 1684.4,
max 4011.6` ms — `controller_frequency` is 5 Hz, so the p50 is right too.

**Only two of the four topics may fail a run, and getting that wrong was the first
version of this gate.** One budget applied to everything flagged `/map` and
`/cmd_vel` as breaches, and neither is one:

| Topic | Cadence set by | Gated |
|---|---|---|
| `/scan`, `/odom` | the simulator, at a fixed rate. A long interval means the link or the consumer stalled | **yes** |
| `/cmd_vel` | `controller_server`, and only while a goal is active. The gaps between legs are Nav2 deciding | no |
| `/map` | `map_update_interval`, 5.0 s. A 1 s budget on it describes slam_toolbox's configuration | no |

That is the difference between measuring delivery and measuring somebody's
publishing schedule. `/cmd_vel` is the case the correction request already handles by
asking for non-degradation against a baseline instead of an absolute bound.

**With the scope right, `/scan` is a real breach: max 1556.8 ms against a 1000 ms
budget.** Not an artefact — p99 is 561 ms, so it is a tail event, and this run was
the one carrying the watchdog's 57% of a core. The same run missed a waypoint (3/4,
against 4/4 for the identical run without the watchdog) and `wp1` aborted after 12 s
with the robot still at the origin. All three are the same fact: an instrument that
costs a quarter of the guest changes what it measures. The rig already knew this —
`WATCHDOG=` exists because of it — and this is the first time the cost has been
visible in the delivery figures rather than only in CPU.

G6 for this run: 296% of 400% across 9 processes, of which the two instruments
(`liveliness_watchdog` 57.3%, `clock_lag_probe` 47.6%) are 105%. Workload-only
**191%**. RTF held 0.4999997, G7 zero stalls.

**Reporting rule that comes out of this.** The budget is the TSP figure divided by
`RTF_LIMIT` — 500 ms becomes 1000 ms at RTF 0.5 — because the sensors run on sim
time while the instrument measures wall time. A wall interval measured at RTF < 1 is
not the TSP's number; it is the TSP's number for a simulator running that slowly, and
it is quoted with the RTF beside it.

### The tb4 map floors, after two runs

|  | precision | recall | coverage | legacy IoU |
|---|---|---|---|---|
| run 1, `WATCHDOG=` | 0.792 | 0.968 | 0.387 | 0.505 |
| run 2, `WATCHDOG=1` | 0.679 | 0.932 | 0.368 | 0.516 |

A floor set from run 1 alone would have been `precision 0.75`. It was, for about an
hour, and run 2 failed it — which is the useful part of this pair.

**precision is not a usable gate against `depot.pgm`, and the IoU column is why.**
The two maps are indistinguishable by that measure — 0.505 and 0.516, a spread of
0.011 — while precision moved 0.11, ten times as far, on the same pair of maps. One
of the two numbers is tracking something other than map quality, and it is not hard
to say which: `depot.pgm` is sparser than the world the lidar sees, so
`precision = matched / slam_occupied` falls whenever a tour observes a region where
the reference holds few cells. Run 2 marked *more* wall (1222 cells vs 1148) against
*fewer* reference cells in its co-known region (844 vs 991). That makes precision a
function of exploration geometry here, not of map quality.

An earlier version of this argument also quoted "0.533, the score of a flawless map"
as depot's ceiling, and that number was wrong: `score_map.py` printed the same
hardcoded 0.533 for every reference, having measured it against `tb3_sandbox`. It
said "a flawless map scores 0.533 **here**" while meaning "there". The scorer no
longer claims a ceiling it has not measured for the reference in hand, and the
argument above does not need one — the 0.011-against-0.11 spread carries it.

So tb4 gates `coverage ≥ 0.30` and recall at `score_map.py`'s own 0.85 — both stable
across the pair — and precision is printed every run and gated nowhere, the same
treatment the legacy IoU already gets. Both runs PASS on those terms.

### G5-E, 2026-08-19: explore_lite runs, and cannot make the robot move at RTF 0.5

Two attempts, `ROBOT=tb4 WATCHDOG= RTF_LIMIT=0.5`, `min_frontier_size` 0.30 then
0.50. `logs/g5e-explore-tb4.log`. The node itself is fine:

```
[explore_node]: Waiting for costmap to become available, topic: /map
[explore_node]: Waiting to connect to move_base nav2 server
[explore_node]: Connected to move_base nav2 server
```

So the executable name, the `/**:` parameter file, `costmap_topic: /map`,
`robot_base_frame: base_footprint` and the `navigate_to_pose` client all work on the
first try. What does not work is the drive.

| | attempt 1, `min_frontier_size` 0.30 | attempt 2, 0.50 |
|---|---|---|
| goals Nav2 accepted | 407 | 554 |
| of those, further than 0.5 m away | **0** | **0** |
| mean / max goal distance | 0.13 m / 0.13 m | 0.13 m / 0.13 m |
| robot pose, every goal | (0.00, −0.00) | (0.00, 0.00) |
| map score | — | `nothing to match` |

Nav2 was cooperating throughout — `Begin navigating ... to (-0.10, 0.08)` followed by
`Reached the goal!`, hundreds of times. The robot was already at every goal it was
given, so it never moved, so the map never grew, so the frontiers stayed at its feet.

**The first diagnosis was `min_frontier_size`, and it was wrong.** Raising it 0.30 →
0.50 changed nothing, and the tell was there before the second run: mean *equals* max
at 0.13 m across 407 goals. A frontier-size threshold changes *which* frontiers are
eligible; it cannot make every goal land at the same distance. A constant is a
different kind of fact.

**The cause is a bootstrap deadlock, not CPU — and the second diagnosis was wrong
too.** Keep reading before believing the CPU evidence below; it is real and it is not
the answer. The saved maps settle it:

| Run | map | occupied | free | unknown |
|---|---|---|---|---|
| waypoint tour, tb4 | 602×303 | **1148** | 69 274 | 61.4% |
| EXPLORE, tb4 | 511×302 | **0** | 1313 | **99.1%** |

slam_toolbox built *nothing* in the EXPLORE runs — not one occupied cell. On a map
that is 99% unknown the nearest frontier is two or three cells from the robot, which
is 0.10–0.15 m, which is the invariant 0.13 m every one of those 554 goals had.

And the reason slam_toolbox built nothing is that the robot never moved:
`mapper_params_tcg.yaml` sets `minimum_travel_distance: 0.5`, so from a standstill no
further scan is ever added to the graph. That closes the loop:

> no motion → no scan added → no map → every frontier is at the robot's feet → no goal
> worth driving to → no motion

`send_goal.py` never hits this because its waypoints come from `robot.env`, not from
the map: it drives regardless of what slam_toolbox knows, and the map bootstraps from
the motion. `explore_lite` derives its goals *from* the map, so with an empty map the
only thing it can propose is where the robot already is. **Frontier exploration cannot
start itself here.** It needs a seed motion — which is what the figure's *other* drive
mode is for.

**The CPU evidence, and why it was the wrong conclusion.** slam_toolbox does say this
at RTF 0.5:

```
[slam_toolbox]: Message Filter dropping message: frame 'rplidar_link' at time 258.800
                for reason 'discarding message because the queue is full'
[planner_server]: Extrapolation Error looking up target frame: Lookup would require
                extrapolation into the past. Requested time 247.200000 but the
                earliest data is at time 252.510000 ... [base_footprint] to [map]
```

`explore_lite` measured 41.6% of an emulated core. Adding it tips slam_toolbox past
the point where it can service the lidar, so scans are discarded, the map barely
advances, and planner TF lookups fall *behind* the oldest data in the buffer. A
deadlock made of three parts, each of which is the others' cause: no movement → no
new scans → no map growth → no frontier worth driving to → no movement.

Note what this is not. `map → base_footprint` **does** resolve — the extrapolation
error names that exact frame pair, so the transform exists and only its timestamps
are wrong. An earlier reading of a `tf2_echo map base_footprint` that returned
nothing inside 30 s suggested the transform was missing; on a guest this loaded, a
silent `tf2_echo` means the tool timed out, not that the frame is absent. The log
settled it and the tool could not have.

That reading was tested and refuted rather than believed. At `RTF_LIMIT=0.25` — the
setting that bought YOLO the room it needed as a third co-tenant — the scan drops fell
from many to **one**, so the CPU pressure was real and was relieved. The churn did not
move at all: 107 goals, 0 substantive, mean and max still 0.13 m, two distinct robot
poses. Relieving the symptom left the deadlock untouched, which is how a symptom can be
distinguished from a cause: fix it and see whether anything else changes.

The 0.13 m invariant was the clue in all three runs and it was visible before any of
the CPU evidence: a threshold on frontier *size* cannot make every goal land at the
same *distance*, and neither can CPU. A constant wants an explanation that is also
constant, and "the map is empty" is one.

**Two defects in the new mode's own instrumentation, both found only by running it.**

`grep -c 'Goal ACCEPTED'` on explore's log counted **0** while Nav2 was accepting
hundreds. The literal is in the binary but never reached the redirected log, so the
gate read a number that had nothing to do with the run. It now counts Nav2's own
`Begin navigating` lines, which cannot disagree with what happened.

And a bare goal count would have called 554 goals a success. The gate now counts only
goals **further than 0.5 m** from the pose Nav2 reported when it accepted them — Nav2's
own cross-check limit for "somewhere else" — and prints a named diagnosis when the two
counts diverge:

```
554 goal(s) accepted, 0 of them > 0.5 m away, 0 blacklisted, complete=no
DIAGNOSIS: frontier churn. Many goals, none worth driving to, so the robot
stood still.
```

The arithmetic was checked against the recorded log before being trusted:
`total 407  substantive(>0.5m) 0  mean 0.13m  max 0.13m`.

### What an adversarial review of the day's changes found

35 agents over the diff, in five dimensions, each finding then handed to a separate
agent told to refute it. 22 survived. The instrument defects are the ones worth
recording, because every one of them would have produced a number that looked fine.

| Defect | Why it mattered |
|---|---|
| `liveliness_watchdog._check()` re-stamps `self.last` during a silence, so the new interval series inherited that cap: the longest interval it could ever report was ~`2·period·SLACK + 0.5 s`. For `/odom` that ceiling is **0.9 s, below the 1000 ms budget** | the `/odom` gate could not fail, however badly the link stalled. The `ok` reported for `/odom` was structural, not measured |
| `/odom`'s nominal period was 0.05 s (20 Hz) while `run_split.sh` forces `odom_publish_frequency=10` | the outage threshold `0.05 × SLACK` = 200 ms sat on `/odom`'s own 199.9 ms median, so roughly half of all healthy arrivals were logged as outages |
| thresholds are stated in **sim** seconds and compared against **wall** intervals | at RTF 0.5 every threshold was 2× too tight. The node now takes `--rtf` and scales them; `/odom`'s becomes 0.8 s wall |
| `cmd_vel_watchdog` subscribes to the topic it publishes zeros on | it was sampling its own deadman messages, so the reported max was its own `timeout` rather than anything the link did. Nonzero commands only now |
| `pct()` used `round()` where nearest-rank needs `ceil()` | p99 over 854 samples returned rank 845 instead of 846 — one sample optimistic every time |
| `LAT_RC` was raised only by grepping the watchdog's report | a watchdog that died, or a `dump()` slower than the fixed `sleep 8`, left both greps false and the gate **passed** a budget nothing had measured. The report's existence is now asserted first |
| `score_map.py` printed `a flawless map scores 0.533 here` for **every** reference, having measured it against `tb3_sandbox` | the argument for not gating tb4 precision quoted it as depot's ceiling. It no longer claims a ceiling it has not measured for the reference in hand, and the argument stands on the 0.011-vs-0.11 spread instead |
| `EXPLORE_TIMEOUT` under 15 gave `seq 1 0` | the poll loop never ran, nothing waited, and the verdict was an exploration failure indistinguishable from a rig that cannot explore. Ceiling division with a floor of one poll |

Plus a documentation sweep: the retracted "its reference map is wrong" conclusion
survived in both runners' comments; `tb3_sandbox`'s origin was quoted as
(−1.86, −2.44) when it is (−10, −10) — which inverts the coincidence test used to
prove depot's origin is spawn-relative, so the table now gives both extents and says
the test is depot-specific; a bringup comparison mixed two instruments; the
inter-arrival block was quoted as log output when it was re-scored under a scope added
afterwards; `DESIGN.md` still documented host `useBuiltinTransports` as `false` when
all four profiles have said `true` since 08-17; and the one documented teleop command
named a container nothing creates and omitted `ROS_DOMAIN_ID`, so it could not have
worked.

The pattern across all of them: **every defect was in an instrument or a claim, not in
the thing being measured.** That is the third time in this log.

### G5-E, seeded: the deadlock breaks and a second blocker appears

`EXPLORE_SEED=2` drives the first two waypoints before exploration starts.
`logs/g5e-explore-tb4-seeded.log`, `RTF_LIMIT=0.5`, `EXPLORE_TIMEOUT=420`.

The seed did its job: 2/2 waypoints, 17 s and 18 s, robot at (3.68, −0.13). The
churn is gone — where the unseeded runs produced 95–107 goals in the first 120 s,
this run produced **2 in 360 s, and both were the seed's own**. So a real map does
stop `explore_lite` proposing the robot's own feet.

It also does not make it explore. `explore_lite` connected to the action server at
15:38:33 and issued **no goal at all** in the following six minutes, while holding
47.7% of an emulated core — so it is working at something, not blocked on a topic. No
`No frontiers found, stopping.`, no `Robot out of costmap bounds`, no
`Goal was REJECTED`. Whatever it is doing, it is not reaching `sendGoal`.

| | unseeded | seeded |
|---|---|---|
| goals in first 120 s | 95–107 | 2 (both the seed's) |
| mean / max goal distance | 0.13 m / 0.13 m | 2.00 m / 2.21 m (the seed's) |
| explore_lite's own goals | 407–554, all 0.13 m | **0** |
| map | 0 occupied, 99.1% unknown | recall 0.977, coverage 0.196 |
| verdict | FAIL, churn | FAIL, coverage below floor |

**The map gate caught it without being asked to.** coverage 0.196 against a 0.30
floor — the seed alone reaches about half of what the four-waypoint tour reaches, and
exploration added nothing on top. That is exactly the argument for gating this mode on
coverage: it is the number exploration exists to move, so a mode that does not explore
cannot pass it however many goals it logs.

**And the goal gate needed the same lesson applied to itself.** As first written it
reported `2 goal(s) accepted, 2 of them > 0.5 m away` and set `NAV_RC=0` — a pass, on
the seed's two goals, while `explore_lite` did nothing. The seed drives through the
same action server and logs the same `Begin navigating` lines, so the count has to be
differenced: `run_split.sh` now records the goal and substantive counts immediately
before launching the node and subtracts them. Re-evaluated against the recorded log:
`total=2 substantive=2, minus baseline (2, 2) -> 0 goals, 0 substantive -> NAV_RC=1`.

That is the third instrument defect in this mode in one afternoon, and all three had
the same shape: a number that was easy to read and did not mean what it said. First
`Goal ACCEPTED`, counted from a log the literal never reached. Then a bare goal count,
which a stationary robot can run up indefinitely. Then a count that included the very
motion introduced to make the mode possible.

**Where this leaves G5-E.** Wired, guarded, torn down cleanly, and honestly gated —
`EXPLORE=1` starts the node, the node configures itself from
`config/explore_split.yaml`, connects, and the run reports what it did with a named
diagnosis when it does nothing. It has **not** produced a passing exploration on this
rig. Two of the three obstacles found are understood and fixed in the rig (bootstrap
deadlock, gate arithmetic); the third — why a connected `explore_lite` with a real map
issues no goals — is open, and the next thing to look at is whether its frontier search
is completing at all on a 602×303 grid inside TCG, since 47.7% of a core with no output
is consistent with a search that never finishes a pass.

**The `/cmd_vel` instrument fix validated itself in this run.** With its own
zero-velocity publications excluded, the host-side statistic reads `n=129, p50 206.9,
p99 1364.9, max 1385.1 ms` — against `max 4011.6 ms` before the fix, which was the
watchdog's own 2 s timeout showing up twice over as if it were link behaviour.

## The demonstration recording, 2026-08-19: what it took to film the split

The goal was narrow — a video showing the rig working reasonably — and `RVIZ=1` had
never been run against a live split. Six read-only audits and eight adversarial
verifications found the demo path, and then three takes were needed. Two of the
things that went wrong were mine and two of the things that went right were
measurements that replaced a guess.

### The viewer could not receive anything, and the reason was a container boundary

`run_demo.sh` started `rviz2` on `host_tcp_server.xml`, which declares the bridge's
own listening port 42100. In a split run the bridge owns it, so RViz came up with no
TCP acceptor and the guest had nothing to dial: `/map`, `/plan` and `map->odom` could
never arrive, and because `demo.rviz` sets `Fixed Frame: map`, that made the pane
blank including `/scan`. The same class of defect as the clock decimator (42101) and
`cmd_vel_watchdog` (42102), in the third place it could occur.

Fixing the port was not enough, and the reason was worth measuring rather than
reasoning about. The viewer is a **second container**, so Docker gives it a private
IPC namespace and its own `/dev/shm`. Measured directly, publisher in `vela-gz`, same
host network, same `ROS_DOMAIN_ID`, 5 Hz `std_msgs/String`:

| subscriber | profile | result |
|---|---|---|
| inside `vela-gz` | the bridge's | **5.000 Hz** |
| in the viewer container | `host_rviz_server.xml`, `useBuiltinTransports true` | **nothing in 25 s** |
| in the viewer container | same file, `false` + explicit UDPv4 | **5.001 Hz** |

`/dev/shm` held 14 `fastrtps_*` segments in `vela-gz` and none in the viewer, which is
the whole mechanism: a shared-memory locator names a segment the other side cannot
open. `--ipc=container:vela-gz` is not available as a fix — dockerd refuses it because
`vela-gz` is `IpcMode=private`, and `run_demo.sh`'s `|| exit 1` would then produce no
viewer at all. So the viewer profiles are the one place in this rig where
`useBuiltinTransports false` is right, and they declare UDPv4 by hand so the
host-local half still arrives.

Ports are now 42100 bridge, 42101 clock decimator, 42102 cmd_vel watchdog, 42103
`rviz2`, 42104 `detection_overlay`. The last two are demo-only, so `GUI=1` generates
`guest_tcp_client_demo.xml` from the committed profile instead of adding locators to
the file G3, G4 and every G5 were measured on.

**Two probes of my own collided while I was testing this.** A `ros2` CLI call loads
the same profile and binds the same port, so a reachability check running beside RViz
takes 42103 away from it — and a 12 s `ros2 topic hz /map` probe reported the map
missing when the first grid was 33 s away, because the guest only starts dialling an
acceptor once something owns it. `run_demo.sh` now reads RViz's own log for `Trying to
create a map of size WxH`, which costs nothing and cannot compete for the port.

### Take 1 — the run passed, the frame wasted 60% of itself

`2/2 waypoints` (30 s, 34 s), precision 0.952 / recall 0.973 / coverage 0.983, RTF
0.50000295. The recording was real and playable, and the layout was wrong in three
independent ways, all visible in one grabbed frame:

| | cost | fixed by |
|---|---|---|
| RViz's Displays and Views docks plus the Image widget | ~450 px of an 864 px pane, leaving ~410 px of map | `demo_nav.rviz`: no `Panels:`, no Image display |
| the Gazebo GUI's Component inspector and Entity tree | ~270 px, showing a solver name and five entities | `demo_gui.config`: 10 plugins kept, 8 dropped |
| nothing tiled the two windows | RViz mapped last at 1500x900 and covered the simulation completely | `xdotool` after RViz is up, 55/45 split |

`Enabled: false` does **not** remove an Image display's dock widget — verified: the
config parsed as `Enabled=False` and a grey "No Image" tile sat in the frame anyway.
The display has to be absent from the file, which is why there are two RViz configs
rather than one.

### Take 2 — my camera fix put the camera outside the wall

`FOLLOW_OFFSET_OVERRIDE=-3.0,0,2.0`, on the reasoning that a wider shot shows more
context. In a walled 4.5 m arena, 3 m behind the robot is outside the arena, and the
wall then occludes the robot: the Gazebo pane showed the arena from outside with no
robot in it. The per-robot `FOLLOW_OFFSET` of `-1.0,0,0.45` was already tuned for this
world. Reverted.

That take also lost a waypoint — `[wp1] finished in 20s status=6`, robot at
(0.28, -0.01), 3.6 m short — which is the first-goal-after-bringup abort, seen on 5 of
the last 12 split runs. On a two-leg tour one abort costs half the drive and the map
never grows enough to score: coverage 0.137 against a 0.90 floor. Hence
`WAYPOINTS_OVERRIDE`, off by default and used by no measured number: repeating the two
proven points doubles the footage and survives one abort.

### Take 3 — the deliverable

`WAYPOINTS_OVERRIDE="1.83,0.08 -1.88,0.08 1.83,0.08 -1.88,0.08"`, default camera,
`demo_nav.rviz`, `demo_gui.config`, 1920x1080.

| | |
|---|---|
| waypoints | **4/4**, 33 / 32 / 34 / 33 s |
| map | **precision 1.000, recall 0.988, coverage 0.998** |
| RTF during the run | 0.50000407 |
| G6 co-tenant load | 276% of 400%, loadavg 16.4 |
| raw take | `artifacts/demo-tb3-20260819-172308.mp4`, 242 s, 10.8 MB |
| delivered clip | `logs/gz-split-tb3-nav2-slam-riscv.mp4`, 102 s, 8.1 MB, 1920x1080 H.264 yuv420p 30 fps |

The clip is the raw take from 38 s with `setpts=0.5*PTS`, which undoes RTF 0.5 and puts
the robot back at real speed. 2123 of 3067 frames are unique — the capture is 15 fps
and llvmpipe renders at 9-10, so a third of the frames are duplicates by construction,
not a dropped-frame defect.

The best map this rig has produced without the detector, and the first evidence that
the two panes show what they were built to show: the Gazebo pane is the host's ground
truth, the RViz pane is what the emulated RISC-V guest believes, and by the end of the
clip all nine cylinders are resolved in a map that slam_toolbox built under TCG from
scans that crossed a TCP link.

### What the audit found and this did not fix

Named rather than left implicit: the `run_split.sh` teardown still kills the Gazebo
server the instant the verdict prints, so a recording that outlives the run ends on a
frozen scene (take 3 was timed to end first); `--record` has no blankness check, so a
black recording exits 0; and RViz's `Map` display logs a GLSL link error under llvmpipe
(`indexed_8bit_image`, `active samplers with a different type refer to the same texture
image unit`) yet renders correctly, so the error is noise here but would be the first
suspect if a map pane ever comes up empty with data arriving.

---

## Take 4, 2026-08-19: the detector on screen, and what had to be fixed first

Take 3 was the deliverable and it had no detector in it. This one does: Nav2,
slam_toolbox and YOLOv8n in one four-vCPU emulated guest, all three visible in one
1920x1080 frame. Four audits and one live-edit mistake sit between the two takes.

### `GUI=1` and `YOLO=1` had never been run together, and the union did not work

Each axis had passed alone. G-Y5 attempt 2 ran the three workloads headless;
Take 3 recorded with a viewer and no detector. Nothing had run both, and the
intersection contained a defect that produces a plausible recording with the
subject missing.

`OVERLAY=1` selected `demo.rviz`, whose Image display is committed as
`Enabled: false` with the dock `collapsed: true`. The only documented way to undo
either is to tick it in the Displays panel — which a headless Xvfb display has
nobody to click. So the overlay would have run, published `/detection_image`, and
been invisible in every frame.

Enabling it was not the fix either. Measured on a real frame with a stand-in
publisher: as the only dock in an 864 px pane, Qt stretched the Image dock to
~320x1000 for a 320x240 image, so a third of the RViz pane became empty grey and
the map lost 300 px to it (`artifacts/rviz-yolo-smoke.png`). That is Take 1's
rejected framing with the detector added.

The detections went into **their own window** instead, inside the overlay process
(`detection_overlay.py --show`). One process, one Fast DDS participant, one
listening port — an external viewer would have needed a sixth port and a sixth
profile. `run_demo.sh` tiles it with the same `xdotool` call as the other two
windows, at 640x480 in the bottom-left of the Gazebo pane, and checks the window
exists by title before the take rather than after it. `WINDOW_GUI_NORMAL` is set
because cv2's Qt build otherwise adds a zoom toolbar that took ~28 px and, by
changing the aspect ratio, letterboxed the image inside what was left
(`artifacts/rviz-yolo-pip-smoke.png`).

`config/demo.rviz` keeps its docks and its disabled Image display and is now
documented as the hand-driven layout that `run_demo.sh` never selects. Recordings
use `demo_nav.rviz` whether or not the detector is running.

### The 230 kB Image across the container boundary was unverified

The only container-to-container measurement on record was a 5 Hz
`std_msgs/String`, and `detection_overlay.py`'s one previous run had been in a
world with no camera, so it had never drawn a box. 230 kB fragments where 20 bytes
does not. Measured directly, publisher in `vela-gz` on `host_tcp_server.xml`,
subscriber in a viewer container on `host_overlay_server.xml` with
`useBuiltinTransports false`:

```
PUB sent 64
SUB received 64 images, 14.1 MB
```

Zero loss. The existing 1 MB socket buffers and `maxMessageSize` 65000 are enough;
no tuning was needed. Confirmed again in the live rig, where the overlay logged
`#1 6 box(es) person:0.82, ...` with no `[no matching frame]` — timestamp matching
holds against a real 19-21 s inference latency.

### Two gates added, because both failures were silent and late

`YOLO=1` had no early camera check. `ros_gz_sim create` returning 0 means the
spawn request was accepted, not that the sensor advertises, so a camera that never
came up was discovered by the G-Y5 check 17 minutes later. There are now two: gz
transport must show `/camera` within 60 s of the spawn, and after lifecycle
activation the detector's own `/diagnostics` heartbeat must report `rx > 0`.

The second gate is only possible because `yolo_node.py` now publishes a 1 Hz
`DiagnosticStatus`. Reading `rx` off a heartbeat costs one second; `ros2 topic hz
/camera/image_raw` would need 90 s at 0.1 Hz of sim time and RTF 0.25.

That parse took three attempts and the first two shipped. `ros2 topic echo --field
status` prints a Python **repr**, not YAML — one line of `KeyValue(key='rx',
value='14')`. The first version squeezed the whitespace out and matched
`key:rxvalue:`; the second assumed the key and value were on separate YAML lines.
Both reported `rx=?` while the detector's log said `rx=3`. The third was written
against a captured sample.

### The failure that was mine: editing a running script

The first attempt died at step 6 with `syntax error near unexpected token '('` on a
line that `bash -n` accepted. Bash reads a script incrementally by byte offset, and
I had edited `run_split.sh` while it was executing — the offsets moved under it.
Nothing was wrong with the file. Finish the edits, then start the run.

### Take 4

`YOLO=1 WATCHDOG= RTF_LIMIT=0.25 GUI=1 ROBOT=tb3 HOLD=1 PRE_DRIVE_SLEEP=180` with
Take 3's four-waypoint override, and `OVERLAY=1 RVIZ=1 GEOMETRY=1920x1080
scripts/run_demo.sh --record --duration 420`.

| | attempt 2 (headless) | Take 3 (no detector) | **Take 4** |
|---|---|---|---|
| RTF | 0.25 | 0.50 | **0.25** |
| bringup | 80 s | — | **75 s** |
| clock lag | −0.6 s | — | **1.0 s** |
| waypoints | 2/2 | 4/4, 33/32/34/33 s | **4/4, 76/63/60/60 s** |
| precision / recall / coverage | 0.992 / 0.990 / 0.973 | 1.000 / 0.988 / 0.998 | **0.961 / 0.969 / 0.999** |
| G6 | 272% of 400% | 276% | **291%, 8 processes** |
| detections | 9 | — | **24 passes, infer_s 14.1 / 21.4 / 32.0 (min/p50/max)** |
| raw take | — | 242 s, 10.8 MB | `artifacts/demo-tb3-20260819-224707.mp4`, 426 s, 13 MB |
| delivered clip | — | 102 s, 8.1 MB | `logs/gz-split-tb3-yolo-nav2-slam-riscv.mp4`, 91.7 s, 7.4 MB |

The clip is the raw take from 60 s with `setpts=0.25*PTS`, which undoes
`RTF_LIMIT=0.25`. **Take 3's factor was 0.5 and copying it here would report the
robot at twice its real speed** — the factor is the real-time factor, so it has to
match the take. 2033 of 2752 frames are unique; llvmpipe renders at 9-10 fps into a
15 fps capture, so duplicates are expected by construction. Blankness checked on
eight sampled frames: 6384-8204 distinct colours each.

Two numbers in that table are worth reading.

**291% against attempt 2's 272%, and the 1 Hz heartbeat is not why.** `yolo_node`
went 81.8% to 82.5% — 0.7 points of an emulated core for a status every second,
which is the measured answer to whether that cadence is affordable. The other 18
points are in the Nav2 and slam_toolbox nodes: this take drove four waypoints
instead of two, and RViz and the overlay subscribe to `/map`, `/plan` and
`/detection_result`, which are guest publishers.

**The map is slightly below attempt 2's on precision and recall and above it on
coverage** — 0.961 / 0.969 / 0.999 against 0.992 / 0.990 / 0.973. Four legs instead
of two is more of the arena seen and more chances to smear a wall, and all six
figures are well clear of their floors.

`HOLD=1` was not needed in the end: the recording closed at 22:56:04 and the
verdict landed at 22:57:13, so the last 70 s of map scoring went unrecorded, which
costs the video nothing. It stays in as the insurance it was added to be — the run
is 17 minutes and timing a take to finish inside it is not a bet worth repeating.

### The RTF default had to be hoisted, and the reason is an ordering one

`YOLO=1` lowering `RTF_LIMIT` to 0.25 itself was written beside the other YOLO
knobs, which is the wrong place: `TOPIC_BUDGET_MS` divides the TSP figure by
`RTF_LIMIT` a few dozen lines earlier, so a measurement run got 500 ms where it
should have got 2000. Silent, `WATCHDOG=1`-only, and it would have read as a topic
breach rather than as a script defect. The block now sits immediately after
`RTF_LIMIT` is defaulted.

Verified by executing lines 43-66 and 145-148 of the script directly:

| | RTF_LIMIT | TOPIC_BUDGET_MS |
|---|---|---|
| `YOLO=1`, none given | **0.25** (defaulted) | **2000** |
| `YOLO=1 RTF_LIMIT=0.5` | 0.5 (kept, warned) | 1000 |
| `YOLO=0`, none given | 0 | 500 |

The first version of the check was also unreachable: it tested `${RTF_LIMIT+x}`,
which is always non-empty by then because `RTF_LIMIT=${RTF_LIMIT:-0}` has already
run. Whether the *caller* set it is now captured before that line.

### What this still does not fix

`--record` has no blankness check; the check above was run by hand afterwards, and
the one in the `--shot` path could be reused. The RViz view sits slightly
right-of-centre on a map this size (`Focal Point X: 1` against an arena whose
mapped extent runs to about 4 m) — visible in `artifacts/take4-end.png`, not worth
re-filming, worth a look before the next take. And `liveliness_watchdog.py` still
does not watch `/detection_result`: deliberately, because its inter-arrival budget
is meaningless against a 40 s publish interval and the 1 Hz heartbeat answers the
same question three orders of magnitude more finely, for 0.7% of a core against the
watchdog's 69%.
