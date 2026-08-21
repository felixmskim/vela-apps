# Why this rig is built the way it is

[`README.md`](README.md) is the build-and-run procedure. This file holds the
reasoning behind it, so the procedure stays short and the reasoning stays
checkable. Every number here cites the file it was read from.

## Why the simulator is on the host

The TSP test-environment figure (§3.1.2, p.16-17) puts Gazebo, RViz2 and the
TurtleBot stack in the **x86-64 column** and SLAM, Nav2 and `explore_lite` on the
RISC-V side, joined by ROS 2. The split here is that figure. The only deviation
is that the RISC-V side is an emulated guest rather than a HiFive Premier P550.

It is also far cheaper: installing Gazebo *in* the guest costs 602 packages,
while this route costs zero extra guest packages beyond `slam_toolbox`.

## Why Gazebo can be headless here

Two beliefs worth correcting, because they change what is worth attempting.

**"No GPU, so no Gazebo."** The host has only a Matrox BMC VGA and no
`/dev/dri/renderD*` node, and the guest has no DRM driver at all. Neither
matters: Mesa's llvmpipe is entirely userspace and needs no render node.
Exercising the exact code path `gz-rendering`'s ogre2 backend uses gives

```
Found Num EGL Devices: 1
EGL_MESA_device_software
GL_RENDERER = llvmpipe (LLVM 20.1.2)
PBUFFER CONTEXT OK
```

`gz sim -s -r --headless-rendering` is therefore a supported configuration, and
`Ogre2RenderEngine` falls back to headless on its own when `XOpenDisplay` fails.

That output came from a standalone probe, not from this image. Nothing in the
`Dockerfile` proves it — `LIBGL_ALWAYS_SOFTWARE` is a GLX-path variable whose EGL
reach varies by Mesa version, and `GALLIUM_DRIVER=llvmpipe` only rules out
softpipe. The proof is produced at run time: `run_rtf.sh` step 1 prints
`eglinfo`, `glxinfo` and the two env vars, and step 3 greps the engine's own
initialisation lines out of the `gz sim -v4` log. Both blocks go in
[`RUNLOG.md`](RUNLOG.md).

## What rendering actually costs

**"Rendering will dominate."** For these robots it does not, on the numbers the
assets and the source give.

| | Source | Figure |
|---|---|---|
| gpu_lidar, first-pass raster | `Ogre2GpuRays::ConfigureCamera()` | 128×128 per cube face, 4 faces |
| TB3 `hls_lfcd_lds` | `gz_waffle.sdf.xacro:136-146` | 360 samples, 360°, 5 Hz → ≈ 0.33 Mpix/s |
| TB3 `intel_realsense_r200_depth` | `gz_waffle.sdf.xacro:378-387` | depth 320×240, 5 Hz → ≈ 0.38 Mpix/s |
| TB4 `rplidar` | `rplidar.urdf.xacro:37-40` | 360 samples, 360°, **10 Hz** → ≈ 0.66 Mpix/s |
| TB4 `oakd` `rgbd_camera` | `oakd.urdf.xacro:124-138` | colour **and** depth 320×240, 10 Hz → ≈ 1.54 Mpix/s |

The cube-face size is derived, not measured: for a 360-sample 360° scan
`ConfigureCamera()` computes `(π/2 / hFOV) × samples = 90`, rounds up to a power
of two, then clamps to `min1stPassSamples = 128u`; the faces rendered are the
ones the sample directions hit, four for a planar scan. Treat the lidar figure as
a **lower bound** — each face renders a depth target beside the colour one.

`make_world.py` deletes both cameras and drops the TB4 lidar to 5 Hz, which puts
the two robots on the same ≈ 0.33 Mpix/s. The cameras go not because they are the
larger cost but because **nothing bridges them**: not one rendered pixel is
consumed. What remains is bounded by QEMU TCG and the ODE step rate, not by
pixels.

The one thing genuinely unknown until measured is the real-time factor. That is
what gate G1 is for, and a measured G1 overrides everything in this section.

## Showing it to people

Measurement is headless; a demonstration is not. `README.md` has the commands —
this is why they are shaped that way.

**The server and the GUI are separate processes.** `gz sim -s` and `gz sim -g`
find each other over gz-transport, so a viewer can be attached to a run that is
already going, and detached again, without restarting or reconfiguring it. That is
what makes a demo possible without a second code path: `run_demo.sh` is only the
viewer half.

**The GUI runs on this host with no GPU.** Verified end to end: Xvfb, then
`gz sim -g` under `QT_QPA_PLATFORM=xcb`, rendering through llvmpipe, exported by
`x11vnc`. Qt 5 and `libqxcb.so` were already in the image — `ros-jazzy-slam-toolbox`
drags them in, which is the over-packaging filed as a defect below being
accidentally useful. `libgz-gui8.so.8.4.0` has zero unresolved libraries.
`Dockerfile.demo` adds five packages — `x11vnc`, `ffmpeg`, `xdotool`, `rviz2` and
`teleop-twist-keyboard` — and it is a **separate image** so the measured image's
digest and apt manifest stay the record they are. That separation is why teleop went
here rather than into the measurement image: adding a package there would rewrite
`/opt/vela/apt-manifest.txt`, which `RUNLOG.md` cites as the environment G1–G7 were
measured in, and would force a `docker rm` of the long-lived `vela-gz` and discard
the 101 MB Depot Fuel cache in its writable layer.

**`headless=true` removes `SceneBroadcaster`, and that is the one failure that
looks like a broken GUI.** Both shipped worlds — `tb3_sandbox.sdf.xacro:3,14` and
`depot.sdf:3,22-27` — default `headless` to true and wrap
`gz::sim::systems::SceneBroadcaster` in `<xacro:unless value="$(arg headless)">`.
Without that system there is no scene or pose stream, so the GUI connects
successfully, reports no error, and draws an empty world for as long as you care to
wait. Nothing measured here needs it: `/scan` comes from the sensors system and
`/tf` from DiffDrive's own `tf_topic`. So it stays off, `make_world.py --gui` turns
it on, `GUI=1` passes that through from the run scripts, and `run_demo.sh` refuses
to start when `robot.env` says `SCENE_BROADCASTER=0`.

**What the GUI costs.** Measured on this host with tb3: the GUI took 20 cores of
llvmpipe (2007 %CPU) while the server needed 20.5 %CPU, and `real_time_factor`
stayed at 1.00. That is a property of a 192-core host, not of the design — on a
smaller machine the GUI competes with the physics, and `SceneBroadcaster` is extra
server work either way. `run_rtf.sh` therefore has no `GUI` switch at all: the
gate that produces the number everything else is judged against must not be able
to inflate it.

**The camera has to be pointed at the robot.** The GUI opens on a default pose
where a 0.28 m TurtleBot3 is roughly 14 px wide and off-centre, which reads
exactly like "the robot did not render" — the meshes
(`package://nav2_minimal_tb3_sim/models/turtlebot3_model/meshes/*.dae`) were
loading correctly the whole time. `run_demo.sh` calls `/gui/follow` with the model
name and `/gui/follow/offset` with the per-robot offset from `robot.env`
(tb3 `-1.0,0,0.45`; tb4 `-1.6,0,0.8`, a taller robot in a much larger world).
Follow also keeps the robot framed once it starts driving, which is the point.

## Image reproducibility

| Decision | Why |
|---|---|
| base pinned by index digest | `ros:jazzy-ros-base` is a moving tag; a rebuild months later has to be able to re-check a number in `RUNLOG.md`. `--build-arg BASE_IMAGE=ros:jazzy-ros-base` floats it for one build |
| apt versions **not** pinned with `=` | `packages.ros.org` keeps only the newest build of each package, so a version pin stops resolving within days — it makes the build fail, not reproduce |
| `/opt/vela/apt-manifest.txt` | what the build actually resolved to, recorded inside the image. This is the reproducibility artefact; copy it out per `README.md` step 1 |
| runs as uid 1000 (`vela`), not root | `/work` is a bind-mounted git working tree. Root-owned run artefacts there need `sudo` to clean up. `ubuntu:noble` already ships a uid-1000 user, so the `RUN` deletes whoever holds the uid and recreates it as `vela` with a real `$HOME` — `gz` needs one for `~/.gz` (Fuel cache) and `slam_toolbox` for `~/.ros` |
| `docker exec -u 0` for mid-run installs | consequence of the above, worth knowing before it surprises you |

For a bit-exact rebuild the base digest and the manifest are what you need; if
even the apt side must be frozen, `snapshots.ros.org` is the mechanism, and it is
not used here.

## TurtleBot3 or TurtleBot4

The TSP figure names **TurtleBot4**; the scripts support both and default to TB3
for the early gates. The reasons belong in one place.

First, what the TSP actually requires. The figure puts "TurtleBot4 ROS2 Stack" in
the **x86-64 column**, not the RISC-V one. That matters:

| Package | amd64 (host) | riscv64 (guest) |
|---|---|---|
| `nav2-minimal-tb4-sim`, `-description` | 1.0.1 | 1.0.1 |
| `turtlebot4-description` | 2.1.1 | **absent** |
| `turtlebot4-navigation` | 2.1.1 | **absent** |
| `turtlebot4-simulator` | 2.0.2 | **absent** |
| `irobot-create-description` | 3.0.4 | **absent** |

Since the TB4 stack belongs on the host, its riscv64 absence does not block the
scenario — the split topology is what makes it satisfiable. Note that
`nav2-minimal-tb4-sim` is Nav2's own minimal asset set, **not** the TurtleBot4
stack, so using it is a partial rather than a literal match.

Second, what switching costs. Seven things change, and one of them lands on the
gate that is already binding.

| # | Change | Consequence | Handled by |
|---|---|---|---|
| 1 | World becomes `depot.sdf` (`warehouse.sdf` also ships) | no more `tb3_sandbox`. `nav2_bringup` does ship `depot.{pgm,yaml}` and `warehouse.{pgm,yaml}`, so `score_map.py` still has a reference | `make_world.py` picks the world and writes `MAP_REF` and `MAP_OFFSET` into `robot.env` |
| 2 | The world fetches from **Fuel over the network** | depot 1 model; warehouse **26**, including human meshes. First run needs internet from the container and caches to `~/.gz/fuel`. `tb3_sandbox` has no remote dependency, so this is a new reproducibility hazard | pre-seed once per container, see `README.md`. `00-preflight.sh` P2b checks reachability |
| 3 | Render load would rise ≈ 6.7× | 0.66 Mpix/s of lidar plus 1.54 of `rgbd_camera` against TB3's tuned 0.33. G1 is the binding gate, so this pushes on exactly the wrong place | `make_world.py` deletes the camera and sets the lidar to 5 Hz, back to ≈ 0.33 |
| 4 | `tb4_bridge.yaml` bridges `/rgbd_camera/camera_info` | **that alone makes the camera render.** `RgbdCameraSensor::HasConnections()` is colour ∥ depth ∥ points ∥ **info**, and gz-sim deactivates a sensor only when it has no connections (`Sensors.cc:480`, `:1282`; `RgbdCameraSensor.cc:487` "don't render if there are no subscribers"). There is no metadata-only option | `bridge_tb4.yaml` omits it |
| 5 | Spawning changes | `spawn_tb4.launch.py` uses `-topic robot_description`; upstream's `-file` variant is commented out as `# TODO SDF file is unhappy`. The xacro→SDF→`create -file` path does not apply | `SPAWN_MODE=topic` in `robot.env`; the run scripts start `robot_state_publisher` and spawn from the topic |
| 6 | TF chain lengthens | `base_link → shell_link → rplidar_link` plus `base_link → base_footprint`, lidar rotated `rpy 0 0 π/2` at `(-0.04, 0, 0.098715)`. TB3's two hand-written static TFs carry Waffle geometry and are wrong here | `slam_nav.launch.py robot:=tb4` runs `robot_state_publisher` from the expanded URDF instead. Only the `.urdf` file is copied to the guest — `robot_state_publisher` never loads the meshes it references, so the description package is not needed there |
| 7 | Spawn pose differs | upstream spawns at **(-8.00, 0.00, 0.01)**; at the origin the robot lands inside the Depot building | `robot.env` carries the pose, the waypoints and the matching `MAP_OFFSET` |

Two things that look like problems and are not. `robot_base_frame: base_footprint`
works for both robots: TB4's DiffDrive publishes `odom→base_link` and the URDF
hangs `base_footprint` off `base_link` at zero offset, so there is no two-parent
conflict and no frame rename in `nav2_split.yaml`. `mapper_params_tcg.yaml` needs
no edit either — slam_toolbox takes the scan frame from the `LaserScan` header,
not from a parameter.

**Recommended sequencing:** bring the pipeline up on TB3 through G1 and G2, then
switch to TB4 for G5. TB4 introduces three independent failure sources — Fuel,
the `robot_description` spawn path, and `robot_state_publisher` in the guest. If a
TB4-only G1 fails there is no way to tell which caused it; on TB3 the question is
only "does headless Gazebo work on this host at all".

`ROBOT` selects the robot everywhere. Everything that differs — world file and
name, spawn pose and mode, bridge config, reference map, offset, waypoints,
frames — is derived once by `make_world.py` into `artifacts/robot.env`, which the run
scripts source. Nothing else in the tree branches on the robot.

## Why the interop check exists

The two sides install ROS 2 from different repositories, so every message package
sits at a different patch version. Measured 2026-08-13:

| Package | container (amd64) | guest (riscv64) |
|---|---|---|
| `ros-jazzy-sensor-msgs` | 5.3.8 | 5.3.6 |
| `ros-jazzy-tf2-msgs` | 0.36.21 | 0.36.14 |
| `ros-jazzy-rmw-fastrtps-cpp` | 8.4.4 | 8.4.3 |
| `ros-jazzy-nav2-msgs` | 1.3.12 | 1.3.10 |

Since Iron, what decides whether two endpoints match is the **type hash**
(`RIHS01_…`), derived from the message definition rather than the package
version. Skew is therefore usually cosmetic — but if a patch release did touch a
`.msg`, discovery silently finds nothing and it looks exactly like a transport
fault. `check_interop.sh` compares the hashes for all five bridged types so that
possibility is eliminated before `run_dds_check.sh` gets blamed.

The guest's hashes as measured on 2026-08-13, for reference:

```
rosgraph_msgs/msg/Clock       RIHS01_692f7a66e93a3c83e71765d033b60349ba68023a8c689a79e48078bcb5c58564
sensor_msgs/msg/LaserScan     RIHS01_64c191398013af96509d518dac71d5164f9382553fce5c1f8cca5be7924bd828
nav_msgs/msg/Odometry         RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0
sensor_msgs/msg/Imu           RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b
tf2_msgs/msg/TFMessage        RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087
geometry_msgs/msg/Twist       RIHS01_9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a
```

Also note the Nav2 skew: the container runs 1.3.12 and the guest 1.3.11. G2
therefore validates *the rig* — world, robot, bridge, SLAM parameters — not the
guest's exact Nav2 build. G5 is what validates the guest.

## How ROS 2 crosses SLIRP

QEMU's SLIRP user-mode networking does not forward multicast, so Fast DDS simple
discovery cannot cross. It does permit the guest to open outbound TCP to
`10.0.2.2`. The two profiles exploit exactly that asymmetry:

| | `host_tcp_server.xml` | `guest_tcp_client.xml` |
|---|---|---|
| `<listening_ports>` | `42100` — its presence elects the server | **empty** — dials out only |
| `useBuiltinTransports` | **`true`** — see below | **`true`** — the guest's nine local participants have no TCP listener and would stop discovering each other |

`useBuiltinTransports` is `true` on **both** sides, in all four profiles, and this
table said `false` for the host until 2026-08-19. The original reasoning — that
leaving builtin transports on makes the host reply to the guest's unroutable
`10.0.2.15` locator — was superseded on 08-17 by `<wan_addr>`, which fixes the
locator problem directly, and by two host participants that *must* have builtin
transports to reach the bridge: `clock_decimator.py` subscribes to `/clock_gz` from
it, and `cmd_vel_watchdog.py` publishes `/cmd_vel` to it. Both are host-local
traffic, which TCP-only profiles cannot carry. Each profile states this in its own
comment; the table had not been updated with them.

**Except in the viewer, where `false` is right — and the reason is a container
boundary, not a network one.** `run_demo.sh` runs RViz and `detection_overlay.py`
in a *second* container. Docker gives it a private IPC namespace and therefore its
own `/dev/shm`, so a shared-memory locator announced by either side names a segment
the other cannot open. Measured 2026-08-19 on a 5 Hz topic, same host, same ROS
domain, both containers on the host network: a subscriber inside `vela-gz` read
5.000 Hz; the identical subscriber in the viewer container received **nothing in
25 s**; `/dev/shm` held 14 `fastrtps_*` segments in `vela-gz` and none in the
viewer. `--ipc=container:vela-gz` is not the fix — dockerd rejects it because
`vela-gz` is `IpcMode=private`, and `run_demo.sh` would exit before starting a
viewer at all. So `host_rviz_server.xml` and `host_overlay_server.xml` set
`useBuiltinTransports` `false` **and declare UDPv4 explicitly**: no SHM in the
transport list, while UDP still carries the host-local half (`/scan`, the bridge's
`/tf`) across the container boundary and TCP carries the guest's. Re-measured with
that profile: 5.001 Hz. The 08-17 decision above is untouched — it applies to
participants that live in the same container as the bridge and had no explicit UDP
transport to fall back on.

Ports now: 42100 `parameter_bridge`, 42101 `clock_decimator`, 42102
`cmd_vel_watchdog`, 42103 `rviz2`, 42104 `detection_overlay`. One acceptor each,
one `initialPeersList` locator each. The last two are demo-only, so `GUI=1`
generates `guest_tcp_client_demo.xml` from the committed profile rather than adding
locators to the file the measurements were taken on.
| `initialPeersList` | — | `tcpv4 10.0.2.2:42100` |

One full-duplex connection multiplexes discovery and both directions of user data
over logical ports. **No `hostfwd` rule, so no QEMU restart.** `keep_alive` is
raised to 30 s / 120 s because the 5 s / 15 s defaults drop the channel during a
long guest stall.

Never set `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`: discovery stays on UDP
multicast and the TCP direction can invert to host→guest, which is unroutable.

## Gates

Each script isolates one variable, so a failure names its own cause.

| Gate | Script | Measure | Pass | Kill |
|---|---|---|---|---|
| G1 | `scripts/run_rtf.sh` | `real_time_factor`, and finite `/scan` returns | ≥ 0.30 and ≥ 1 finite | < 0.10 → bag fallback, escalate D-14 |
| G2 | `scripts/run_container_loop.sh` | waypoints reached, map precision / recall / coverage | all, and 0.80 / 0.85 / 0.90 | — |
| G2b | same, `RTF_LIMIT=0.5` | same outcome at a second RTF | consistent | — |
| G3 | `scripts/run_dds_check.sh` | talker → listener across SLIRP | < 120 s | — |
| G4 | `tools/rtt_probe.py` | round-trip p95 | < 500 ms | — |
| G5 | `scripts/run_split.sh` | live split drive + map | all waypoints, same three map floors | — |
| G6 | `scripts/run_split.sh` | slam_toolbox + Nav2 combined %CPU | < 250 of 400 | — |
| G7 | `scripts/run_split.sh` | longest loop-closure / Ceres stall | < 20 s | — |
| G5-E | `EXPLORE=1 scripts/run_split.sh` | frontier goals accepted, then the same map floors | ≥ `EXPLORE_MIN_GOALS` (2), and the three floors | — |
| G-Y5 | `YOLO=1 scripts/run_split.sh` | detections while Nav2 drives | ≥ 1, and G5's own floors still met | — |
| G5-L | `WATCHDOG=1 scripts/run_split.sh` | longest inter-arrival per watched topic | max < `TOPIC_BUDGET_MS` (TSP figure ÷ RTF) | — |

G5-L is not a latency gate and is deliberately not named one. It bounds the longest
interruption a consumer saw, which is the measurable reading of the TSP's topic
requirement; see [Topic latency](#topic-latency-what-the-split-can-and-cannot-measure)
for why the publish-to-subscribe figure is unavailable here. It rides on
`liveliness_watchdog.py`, so it is only produced by a run that keeps the watchdog and
pays its 69% of a core.

G-Y5 asserts co-tenancy, not detection rate. TC.10.007 names Nav2, SLAM and
YOLOv8n running together; what can fail here is that three workloads in a
four-vCPU emulated guest starve each other, and that shows up as a detector that
stops producing or a drive that stops meeting its floors. How *fast* it infers is
the emulator's property and is gated nowhere — `../yolo/README.md` argues that,
and CR-12 asks the TSP to relax the inference-rate expectation for the same
reason.

G2b is not redundant. `use_sim_time: true` does **not** make a low RTF harmless:
`bondcpp` uses `create_wall_timer` with `RCL_STEADY_TIME`, and
`controller_server` and `behavior_tree_engine` use `rclcpp::WallRate`. At
real-time factor *R* the effective control period inflates by 1/*R* while the
"control loop missed its rate" warning stays silent. A result that holds at one
RTF only is not a result.

If the render engine will not start at G1, the ladder is: drop
`--headless-rendering` and wrap in `xvfb-run`, then `--render-engine-server ogre`
(OGRE 1.9, far lighter than ogre2 — both arches' `gz-rendering-vendor` ship it,
and the CLI value overrides the world's pinned `<render_engine>ogre2</render_engine>`
per `Sensors.cc:829-835`), then halve the lidar samples.

## Why explore_lite attaches to the runner and not to the launch file

The TSP figure names two drive modes, `Host Teleop(manual)` and
`Explore_lite(autonomous)`. `tools/send_goal.py` is the scripted counterpart to the
first, `explore_lite` is the second, and `EXPLORE=1` selects it.

`run_split.sh` step 8 starts it, **not** `tools/slam_nav.launch.py`. Three reasons,
all properties of this rig:

1. That launch file is shared with `run_container_loop.sh`. A node added there
   changes the configuration G2's published numbers describe, silently.
2. `Costmap2DClient`'s constructor blocks until the first costmap arrives *and*
   until `map → base_footprint` resolves. In the launch file that block spans the
   whole 60–123 s bringup, and a co-tenant during bringup is measured harm here
   rather than a worry: `yolo_node` took bringup from 65–70 s to 110 s and every
   goal aborted.
3. Teardown is by pattern, so one more guest process is one entry in `GUEST_PROCS`
   — exactly how `send_goal.py` is handled.

**It cannot bootstrap itself, and the deadlock belongs to the pair rather than to
either node.** `mapper_params_tcg.yaml` sets `minimum_travel_distance: 0.5`, so a
stationary robot adds no scan after the first and `/map` stays empty — measured
2026-08-19 on a saved EXPLORE map: **0 occupied cells, 1313 free, 99.1% unknown**,
against 1148 / 69 274 / 61.4% for a waypoint run in the same world. `explore_lite`
derives goals from that map, so on a 99%-unknown grid the nearest frontier is two or
three cells away:

> no motion → no scan added → no map → every frontier at the robot's feet → no goal
> worth driving to → no motion

`send_goal.py` never meets this because its waypoints come from `robot.env` rather than
from the map. `EXPLORE_SEED` (default 2) therefore drives the first waypoints before
exploration starts. Two rather than one because the first goal after bringup is
unreliable on this rig — a tb4 run reached 3 of 4 and the one it missed was `wp1`,
aborted at 12 s with the robot still at the origin.

Worth recording how this was misdiagnosed twice, because both wrong answers had real
evidence behind them. First `min_frontier_size`: raising it 0.30 → 0.50 changed
nothing. Then CPU, on the strength of slam_toolbox dropping scans with `queue is full`
and `planner_server` extrapolating past the TF buffer — both genuine, and
`RTF_LIMIT=0.25` cut the drops from many to one while the churn stayed identical. The
clue that survived all three runs was that mean *equalled* max goal distance at 0.13 m:
neither a size threshold nor CPU can make every goal land at the same distance. **A
constant needs an explanation that is also constant.**

Three things that look like the right move and are not:

- **Do not** add `explore_node` to `lifecycle_manager_nav.node_names`. It is a plain
  `rclcpp::Node`, not a lifecycle node, and the manager would wait forever for a
  transition it cannot request.
- **Do not** add `explore_lite_msgs` to `tools/type_hashes.py` or to
  `check_interop.sh`'s package list. Both work by diffing the container against the
  guest, and this package exists only in the guest, so it would report a mismatch
  that is not one.
- **Do not** use the package's own `explore.launch.py`. It hardcodes its own
  `config/params.yaml` and exposes no argument to override it, so it cannot load
  this rig's parameters at all.

`explore_lite` appears in the G6 census pattern as a **tripwire, not accounting**.
That census runs at step 10, after the drive, and step 8 kills the explorer before
step 9 — the same reason `send_goal.py` never appears in a G6 listing. If it ever
does appear, it survived teardown and the next run is already poisoned.

## Topic latency: what the split can and cannot measure

The TSP gates on topic latency, and this rig cannot produce the quantity as written.
Worth writing down carefully, because the wrong instrument is easy to build and its
output looks plausible.

Every topic the TSP names crosses the machine boundary: `/scan` and `/odom` are
published by the host bridge and consumed in the guest, `/cmd_vel` goes the other
way. The two machines' clocks are not synchronised, so a DDS
`received − source` difference carries the offset. `latency_probe.py` says exactly
this about itself and is valid same-machine only.

The tempting escape is `header.stamp` against the consumer's `now()`, on the
argument that both sides take sim time from one host-authored `/clock`. They share
it **in value and not in phase**. The guest's `now()` is a sample-and-hold of the
last `/clock` sample the guest actually serviced, and `/clock` arrives over the link
being measured, into a node competing for four emulated cores. So the probe reports

```
now()_guest − header.stamp  =  RTF × L_transport  −  LAG_sim
```

where `LAG_sim` is the guest clock's staleness — the quantity `clock_lag_probe.py`
exists to watch and step 7 gates on. Measured: 0.5–3.1 s in passing runs, 6.5 s in
failing ones, against a wanted signal of 5–25 ms. The instrument would report the
negative of the clock lag, 20× to 600× the signal. A probe built this way would also
refuse itself: the negative-age guard fires on almost every run, and that guard is
diagnosing this exactly.

What survives is one-clock measurement, and it answers the requirement's actual
words — `중단 없이 유지`, maintained without interruption:

| Question | Instrument | Where the clock lives |
|---|---|---|
| longest interruption a consumer saw on `/scan`, `/odom`, `/map` | `liveliness_watchdog.py` inter-arrival p50/p99/max | consumer, `CLOCK_MONOTONIC` |
| same for `/cmd_vel`, which carries no `Header` | `cmd_vel_watchdog.py`, host side, port 42102 | host consumer |
| cost of the crossing itself | `rtt_probe.py` (G4, p95 12.1 ms) | one, round trip |

Two consequences for reporting. The budget is compared against **wall** intervals
while the sensors run on **sim** time, so it is scaled by `RTF_LIMIT` — `/scan` at
5 Hz of sim is 400 ms of wall at RTF 0.5, and an unscaled 500 ms budget fails on
jitter. And a wall interval measured at RTF < 1 is not the TSP's number; it is the
TSP's number for a simulator running that slowly, so it is reported with the RTF
beside it, which is what the emulation clause in the correction request asks for.

## Fallback: bag replay

If G1 or G3 cannot be made to pass, record on the host and replay in the guest:

```bash
# host container, while driving the tour with Nav2 there
ros2 bag record -s mcap -o /work/artifacts/bags/run1 /clock /scan /odom /tf /tf_static
scp -P 2222 -r artifacts/bags/run1 vela@localhost:~/bags/

# guest: slam_toolbox alone
ros2 bag play ~/bags/run1 --clock 100 -r 0.25 \
  --topics /scan /odom /tf /tf_static
```

This still builds a SLAM map from genuine Gazebo lidar data. It is **open-loop**:
Nav2's `/cmd_vel` goes nowhere, so it cannot support any claim that the robot
drove. Keep `/clock` out of `--topics` — `--clock 100` synthesizes it, and two
publishers make time non-monotonic.

## What this cannot tell you

**There is no RERI in this configuration.** The guest runs with
`-machine virt,wg=on` and no `reri=true` and no EDK2 pflash pair, so no error can
be injected. This produces the *co-tenant workload* that TC.10.007 needs, never a
TC.10.007 result. The merged configuration — Ubuntu root disk **and**
`reri=true` **and** the pflash pair, with HEST/EINJ still reaching the kernel —
does not exist yet and remains the gating prerequisite. Any MTTR or downtime
figure derived from this rig would be meaningless.

**The map comparison is self-consistency, not verification.** Gazebo renders the
world geometry and `score_map.py` compares the result against that world's shipped
occupancy map. A high score shows the pipeline is coherent end to end. It does not
independently confirm the map is correct.

### Why the map gate is three numbers and not occupied IoU

`score_map.py` originally dilated the reference map by one cell and gated on
occupied IoU ≥ 0.70. Two measurements retired that metric on 2026-08-17.

*It had a ceiling below its own threshold.* A 1-cell reference wall dilated in
four directions becomes a 5-cell plus shape, and a correct 1-cell SLAM wall can
only ever fill the centre. Scoring a **flawless** map — the reference geometry
itself, restricted to the region one real run observed — gives IoU 0.533. No
mapper could pass 0.70. Worse, the metric was inverted: the real run scored 0.543
and a run that died in bringup with 13.6% of the arena mapped scored 0.676,
because both thin coverage and blurry walls increase overlap with an inflated
reference. Dilating **both** maps by the same tolerance fixes the direction and
puts a flawless map at exactly 1.000.

*One number cannot separate the failure modes.* Injecting known faults into the
best real map showed each is visible in exactly one place: localisation drift in
precision *and* recall together, an eroded map in recall alone (50% of wall cells
erased still leaves precision at 0.990), free space wrongly marked occupied in
precision alone (5% flipped leaves recall at 0.981), and a run that died early in
coverage alone (a flawless map of a quarter of the arena scores precision 1.000,
recall 1.000). Any single combined score lets the other two hide — F1 passes the
50%-erased map at 0.904. Hence `precision ≥ 0.80 ∧ recall ≥ 0.85 ∧ coverage ≥ 0.90`,
with F1 printed and not gated. The full table is in `RUNLOG.md` Step 6.

Coverage is measured on the **reference** grid, not the SLAM grid: a SLAM grid
only spans what it already saw, so measuring on it would always answer ~100%. It
is also the one threshold that is per-world, because tb3_sandbox has 19.8 m² of
free space and depot has 448.7 m²; `make_world.py` writes it into `robot.env`.

The legacy IoU is still printed for continuity with `RUNLOG.md` entries that
quote it, labelled as not a gate.

### Grid projection is integer, not floating point

Coverage projects each known SLAM cell onto the reference grid. Doing that with
`floor((origin + (i+0.5)*res + offset - ref_origin) / ref_res)` is correct until
the expression lands on an exact cell boundary, and then it is one ULP from
either neighbour — so which cell a row claims depends on how the value
accumulated. Measured 2026-08-19: two maps of the same arena three millimetres
apart in origin projected as 8398 distinct cells and 5199, and coverage read
0.983 and 0.598 for maps identical to the eye. The gate flipped on the scorer,
not on the mapping.

Both grids have the same resolution in every run here, so the projection is a
pure integer translation and is computed as one — `cx = ix + round((slam_ox + dx
− ref_ox)/res)` — which is 1:1 by construction. The float path survives for
unequal resolutions with a `CELL_EPS` nudge, and `coverage()` warns when cells
collapse, because a projection that loses cells reads as low coverage and should
say so instead. `RUNLOG.md` Step 8 has the measurements.

**Emulated timings are a property of the host.** QEMU TCG has no timing model;
guest throughput is measured at 10.8× native on scalar FP and worse on
vectorisable code, and it varies with host load. Wall-clock latency thresholds
evaluated here describe this emulation host, not the RISC-V RAS design.

**Loop closure is indistinguishable from a stall.** A slam_toolbox Ceres solve
that runs long looks exactly like recovery delay. G7 bounds it; if it fails, set
`do_loop_closing: false` and say so, or drop SLAM from the co-load.

**`/cmd_vel` has no Header — but its latency is still measurable.**
`rmw_message_info_t.source_timestamp` and `received_timestamp` are populated for
every sample regardless of the message body. Verified on the guest with an
unmodified `geometry_msgs/Twist`: 30/30 samples carried a nonzero
`source_timestamp`, giving 0.227 / 0.228 / 17.605 ms for min / median / max.
`latency_probe.py` does this. Note the scope — the figure is only an absolute
latency when publisher and subscriber share a clock, so use `rtt_probe.py` across
the host/guest boundary.

**No `rviz2` for riscv64.** Only the rviz libraries are packaged, so every
guest-side check here is headless by necessity. Visualise on the host instead.

**slam_toolbox is over-packaged.** `ros-jazzy-slam-toolbox` declares its build
dependencies as runtime `Depends`, so a 19 MB package expands to 192 packages and
pulls `mesa-vulkan-drivers` and Qt 5 into a guest with no `/dev/dri`. `objdump -p`
confirms `async_slam_toolbox_node` links none of it. Reported as a packaging
defect; the extra 165 MB is harmless against 12 GB free, so it is not worked
around here.

## See also

- [`README.md`](README.md) — build and run
- [`RUNLOG.md`](RUNLOG.md) — what actually happened
- [`../workload-nav2.md`](../workload-nav2.md) — the Nav2 stack and TCG tuning this builds on
- [`../README.md`](../README.md) — RERI injection and collection
- [q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md) — guest launch, RERI registers
