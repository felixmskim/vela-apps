# Nav2 on the RISC-V guest

Bringing up the ROS 2 navigation stack inside the Q-Vela emulated guest, and
proving it plans real paths — headless, with no simulator and no GPU.

This exists because the RAS availability test (TSP **TC.10.007**) measures whether
a ROS 2 workload keeps delivering while RERI errors are injected. There has to be
a workload first. Nav2 is that workload.

```
falinux riscv64 apt   →   curated install   →   static map   →   ComputePathToPose
   (Nav2 1.3.11)          (293 packages)      (tb3_sandbox)     (planner proven)
                                                                        ↓
                                                              NavigateToPose
                                                          (loopback kinematic sim)
```

Everything below was measured on the guest described in
[Environment](#environment), not estimated.

---

## Nav2 for riscv64 is a package install, not a source build

The natural assumption — "ROS 2 does not publish riscv64, so Nav2 must be built
from source" — is half right and leads to the wrong conclusion. The upstream ROS
repository indeed has no riscv64, but the Vela repository the guest already uses
does, and it carries the whole stack prebuilt.

| Repository | Architectures | Nav2? |
|---|---|---|
| `packages.ros.org/ros2/ubuntu` | `i386 amd64 arm64 armhf` — `binary-riscv64` returns **HTTP 404** | no riscv64 at all |
| `vela.falinux.com/ubuntu noble main` | **`all riscv64`**, 659 riscv64 packages | **`ros-jazzy-navigation2` 1.3.11-1noble**, complete |

Verify both in one command, from the guest:

```bash
apt-cache policy ros-jazzy-navigation2
```

```
ros-jazzy-navigation2:
  Candidate: 1.3.11-1noble
  Version table:
     1.3.11-1noble 500
        500 http://vela.falinux.com/ubuntu noble/main riscv64 Packages
```

A full Nav2 source build under TCG emulation would cost on the order of the
six-plus hours the ROS 2 core build took. None of it is necessary.

### What the repository ships, and what it does not

| Present for riscv64 | Notes |
|---|---|
| `ros-jazzy-navigation2` and all 32 `nav2-*` packages | 1.3.11-1noble |
| `ros-jazzy-ompl` 1.7.0 | needed by `nav2-smac-planner` |
| `ros-jazzy-behaviortree-cpp` 4.9.0 | needed by `nav2-behavior-tree` |
| `ros-jazzy-slam-toolbox` 2.8.4, `ros-jazzy-cartographer-ros` | |
| `ros-jazzy-ros-gz-sim`, `ros-jazzy-gz-sim-vendor` | Gazebo is installable, but see [Known limitations](#known-limitations) |
| `ros-jazzy-tf2-tools`, `ros-jazzy-ros2bag`, `rosbag2-storage-mcap` | |

| **Absent** | Consequence |
|---|---|
| `ros-jazzy-nav2-loopback-sim` | The kinematic simulator Nav2's own loopback launch files depend on. Source-built in [Install](#install) — it is `ament_python`, so nothing compiles. |
| `ros-jazzy-rviz2` | Only the rviz *libraries* are packaged, not the executable. All verification here is headless by necessity. |
| `ros-jazzy-turtlebot4-*` | Does **not** block the TSP 4th-year AMR scenario, though it reads that way. The figure puts the TurtleBot4 stack in the x86-64 column, so the split rig satisfies it from the host: `gz-split` runs tb4 with `nav2-minimal-tb4-sim` there and needs only `robot-state-publisher` here. Demonstrated 2026-08-19 — tb4 drove 4/4 waypoints in the split topology. See [`gz-split/DESIGN.md`](gz-split/DESIGN.md) "TurtleBot3 or TurtleBot4"; `nav2-minimal-tb4-*` is a partial rather than a literal match for the stack. |

### Why `rosdep` cannot answer this question

`rosdep` maps ROS package keys to OS package names using rosdistro YAML whose
rules are keyed by **OS and codename only** — `ubuntu: noble: [...]`. There is no
architecture axis anywhere in that data model. So `rosdep resolve nav2_planner`
prints `ros-jazzy-nav2-planner` whether or not a riscv64 build exists. It is
telling you a *name*, never an *availability*.

`apt-cache policy` and `apt-get install -s` consult the real
`binary-riscv64/Packages` index. Those are the authoritative tools here.

`rosdep` earns its keep in exactly one place in this document: resolving the
dependencies of the source overlay, which is the only thing actually built from
source. Even there it needs `--skip-keys`, for the reason given in
[Install](#install).

---

## Before you start

These commands install nothing. Run them first; if the dependency closure does
not resolve, stop and report it rather than spending an hour of emulated dpkg
time finding out.

```bash
apt-cache policy | grep -A2 vela.falinux

for p in ros-jazzy-navigation2 ros-jazzy-nav2-planner ros-jazzy-nav2-map-server \
         ros-jazzy-nav2-loopback-sim ros-jazzy-rviz2; do
  printf '%-36s %s\n' "$p" "$(apt-cache policy $p 2>/dev/null | awk '/Candidate:/{print $2}')"
done

apt-get install -s ros-jazzy-navigation2 2>&1 | grep -E 'upgraded,.*newly installed'
```

```
ros-jazzy-navigation2                1.3.11-1noble
ros-jazzy-nav2-planner               1.3.11-1noble
ros-jazzy-nav2-map-server            1.3.11-1noble
ros-jazzy-nav2-loopback-sim
ros-jazzy-rviz2
28 upgraded, 676 newly installed, 0 to remove and 131 not upgraded.
```

The two blank lines are the expected result, not a failure — they are the gaps
listed above.

> ⚠️ **`apt-get install -s` takes 5–15 minutes here.** Resolving a ~700-package
> closure is CPU-bound and this guest is emulated. It is not hung. Also note it
> prints no size figures at all; to see those, run the real `apt-get install`,
> read the `Need to get` / `After this operation` lines, and answer `n`.

<details>
<summary>Why an ISA mismatch is not among the things that can go wrong</summary>

The falinux binaries are built for plain `rv64gc`:

```bash
readelf -A /opt/ros/jazzy/lib/librclcpp.so | grep Tag_RISCV_arch
```

```
Tag_RISCV_arch: "rv64i2p1_m2p0_a2p1_f2p2_d2p2_c2p0_zicsr2p0_zifencei2p0_zmmul1p0"
```

The guest CPU is `rv64imafdcv...zba,zbb,zbc,zbkb,zbs,zicond`, a strict superset.
An `Illegal instruction` from a Nav2 binary is therefore not a possible failure
mode on this platform, and should not be investigated as one.
</details>

---

## Install

Two package sets are traps. Both look like the obvious choice and both cost
hundreds of packages that can never be used on a `-nographic` guest. Under TCG,
dpkg unpacking dominates wall-clock time, so package count *is* the cost.

| Do not install | Why | Instead |
|---|---|---|
| `ros-jazzy-nav2-bringup` | `Depends:` on `nav2-minimal-tb3-sim`, `nav2-minimal-tb4-sim`, `ros-gz-bridge`, `ros-gz-sim`, `slam-toolbox` → pulls the **entire Gazebo chain**, 345 packages | The 35 KB deb holds the maps, params and launch files. Extract it. |
| `ros-jazzy-navigation2` | `Depends:` on `nav2-rviz-plugins` → `rviz-ogre-vendor`, `rviz-default-plugins`, mesa, `va-driver-all`, `vdpau-driver-all`. **704 packages** for functionality that needs 293. | Name the nav2 packages explicitly. |

Measured on this guest:

| Target | Packages | Covers |
|---|---|---|
| planner-only set | **112** (17 upgraded + 95 new) | static-map path planning |
| explicit list, no rviz | **293** (26 upgraded + 267 new) | full navigation |
| `ros-jazzy-navigation2` | 704 (28 upgraded + 676 new) | the same, plus 411 unusable graphics packages |

### Maps and parameters, without installing the package

```bash
mkdir -p ~/nav2_data && cd ~/nav2_data
apt-get download ros-jazzy-nav2-bringup
dpkg -x ros-jazzy-nav2-bringup_*.deb ~/nav2_data
ls ~/nav2_data/opt/ros/jazzy/share/nav2_bringup/maps/
```

```
depot.pgm  depot.yaml  tb3_sandbox.pgm  tb3_sandbox.yaml  warehouse.pgm  warehouse.yaml
```

### Packages

Suppress the man-db trigger first — rebuilding the manual index after every
package costs more emulated time than the unpacking itself.

```bash
echo 'path-exclude=/usr/share/man/*' | sudo tee /etc/dpkg/dpkg.cfg.d/99-no-man

sudo apt-get install -y --no-install-recommends \
  ros-jazzy-nav2-map-server ros-jazzy-nav2-planner ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-simple-commander ros-jazzy-nav2-msgs
```

Run long installs inside `screen`, so a dropped host session cannot kill them.

For full navigation, add the rest and the build tooling:

```bash
sudo apt-get install -y --no-install-recommends \
  ros-jazzy-nav2-amcl ros-jazzy-nav2-behavior-tree ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-controller ros-jazzy-nav2-core \
  ros-jazzy-nav2-dwb-controller ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-nav2-rotation-shim-controller ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-smoother ros-jazzy-nav2-util ros-jazzy-nav2-velocity-smoother \
  ros-jazzy-nav2-voxel-grid ros-jazzy-nav2-theta-star-planner \
  python3-colcon-common-extensions python3-rosdep python3-transforms3d git
```

### The source overlay — two packages, no compiler

`nav2_loopback_sim` is in neither `binary-riscv64` nor `binary-all`. It also
imports `tf_transformations`, which has no riscv64 deb either, so this is **two**
packages rather than one. Both are `ament_python`, so the build is cheap even
under emulation.

```bash
mkdir -p ~/nav2_ws/src && cd ~/nav2_ws/src

# Sparse checkout: one subdirectory instead of a ~700 MB clone
git clone --depth 1 --filter=blob:none --sparse -b jazzy \
    https://github.com/ros-navigation/navigation2.git nav2_sparse
cd nav2_sparse && git sparse-checkout set nav2_loopback_sim && cd ..

git clone --depth 1 https://github.com/DLu/tf_transformations.git

cd ~/nav2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tf_transformations nav2_loopback_sim --symlink-install
```

If you use `rosdep` on this workspace, `--skip-keys` is mandatory:

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "tf_transformations"
```

Without it, rosdep resolves `tf_transformations` to `ros-jazzy-tf-transformations`
— an architecture-agnostic rule for a package name that exists in no riscv64
index — and apt fails. This is exactly the rosdep limitation described earlier,
showing up in the one place rosdep is actually being used.

---

## Static-map path planning

The smallest configuration that proves the claim: three lifecycle nodes, one
fake transform, no robot, no simulator, no AMCL.

```
  static_transform_publisher ──(map → base_link)──┐
                                                  ▼
  map_server ──(/map, transient_local)──►  planner_server
       ▲                                    └─ global_costmap
       │                                        ├─ static_layer
       └──────── lifecycle_manager ─────────────┴─ inflation_layer
                 (configure → activate)              │
                                                     ▼
                                       /compute_path_to_pose  (action)
```

Doing this before the full stack is deliberate. If the falinux riscv64 binaries
were broken, three nodes surface it in minutes with one clean error, instead of
twelve nodes interleaving their logs after an hour of installing.

### Two changes that are correctness, not tuning

Both come from running Nav2 without a robot, and skipping either one causes a
hang that looks like something else entirely.

| Change | Upstream | Why it is required |
|---|---|---|
| `global_costmap.plugins: ["static_layer", "inflation_layer"]` | `["static_layer", "obstacle_layer", "inflation_layer"]` | `obstacle_layer.observation_sources` is `scan` on `/scan`. With no robot, `/scan` never arrives, the layer never becomes current, and the planner aborts every goal with a costmap timeout. |
| a static `map → base_link` transform | published by the robot | `Costmap2DROS::activate()` blocks waiting for this transform. With no robot, nothing publishes it and bringup never completes. |

### Run it

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ros2 launch ~/nav2_tcg/planner_only.launch.py
```

Then, in a second shell with the same environment:

```bash
python3 ~/nav2_tcg/check_plan.py
```

### What a successful run looks like

Bringup, abridged — the four lines that matter:

```
[map_server-2] [map_io]: Read map .../tb3_sandbox.pgm: 384 X 384 map @ 0.05 m/cell
[planner_server-3] [global_costmap]: Using plugin "static_layer"
[planner_server-3] [global_costmap]: StaticLayer: Resizing costmap to 384 X 384 at 0.050000 m/pix
[lifecycle_manager-4] [lifecycle_manager_nav]: Managed nodes are active
```

The acceptance test:

```
[12:15:43]   active after 23s
[12:16:16] waiting for /compute_path_to_pose action server (up to 180s)...
[12:16:16]   action server up
[12:16:16] waiting for /map (transient_local, up to 180s)...
[12:16:16]   map 384x384 res=0.050 origin=(-10.0,-10.0)
[12:16:16]   start (-1.88, 0.08) -> occupancy 0
[12:16:16]   goal  (1.83, 0.08) -> occupancy 0
[12:16:17] plan 1/3  positive: START -> GOAL across the obstacle bands
[12:16:17]   poses=169 length=4.26m (straight=3.71m) wall=0.6s
[12:16:17]   first=(-1.90,0.10) last=(1.83,0.08) lethal_waypoints=0
[12:16:17] plan 2/3  negative: off-map goal (100.0, 100.0) must be refused
[12:16:17]   correctly refused (action status 6 (not SUCCEEDED))
[12:16:17] plan 3/3  negative: off-map goal (-50.0, 0.0) must be refused
[12:16:17]   correctly refused (action status 6 (not SUCCEEDED))

[12:16:17] RESULT: PASS
```

| Phase | Wall time |
|---|---|
| launch → all nodes `active` | 23 s |
| one `ComputePathToPose` round trip | 0.4 – 0.6 s |
| whole acceptance test | ~35 s |

### The path it produced

`check_plan.py --dump path.csv` writes the returned path, so a run can be drawn
afterwards rather than taken on trust. Start and goal sit on the same line with
three obstacle blocks between them; the planner routes over the top:

```
tb3_sandbox.pgm  crop x=[-2.60,2.60] y=[-0.90,1.50] m   1 char = 0.05 x 0.10 m
  '#' lethal   '.' free   ' ' unknown   '*' planned path   S start   G goal
  +--------------------------------------------------------------------------------------------------------+
  |          ##.................................................................................##         |
  |        ###...................................................................................###       |
  |       ###....................###..............................................................##       |
  |      ###...................######...............#######................######..................###     |
  |     ##....................##    ##..............##    ##...............#    ##..................###    |
  |    ##......................######...............#######................######....................###   |
  |  ###...............................................###..................####.......................##  |
  | ###.................................................................................................## |
  |###...................................................................................................##|
  |#.........................**************...............................................................#|
  |..................*********............********************************************.....................|
  |...............***................................................................***...................|
  |.............**.....................................................................***...............##|
  |............**...............####..................###................................***............###|
  |.............*S............#######...............#######...............######...........G...........### |
  |...........................##   ##...............##   ##...............##   ##.....................##   |
  |............................######................######...............#######.....................##   |
  |........................................................................#####.......................##  |
  |.....................................................................................................## |
  |......................................................................................................##|
  |#......................................................................................................#|
  |##......................................................................................................|
  |###...................................................................................................##|
  |  ##.................................................................................................###|
  |   ###.......................###...................##...............................................### |
  +--------------------------------------------------------------------------------------------------------+
```

### Why the assertions are what they are

The decisive check is **`lethal_waypoints=0`**, not the path length.

If the static layer silently fails to load, every cell reads as free and navfn
returns a clean straight line — 3.71 m, plenty of poses, endpoints exactly right.
A length bound and a pose count both pass. Only comparing every waypoint against
the occupancy grid catches it. The map is fetched with `transient_local` QoS for
the same reason: `map_server` publishes `/map` as
`KeepLast(1).transient_local().reliable()`, and a volatile subscriber never
matches, which is indistinguishable from "the map failed to load".

There is deliberately **no** "a goal inside a wall must be refused" assertion.
Walls on `tb3_sandbox` are 1–2 px thick — at most 0.10 m from a traversable cell —
so with the default `tolerance: 0.5` the planner correctly satisfies such a goal
from a free cell beside it. Asserting failure there reports a defect that is not
one. The negative cases used instead are goals outside the costmap, which spans
x, y ∈ [−10.0, 9.2].

<details>
<summary>Map facts the assertions depend on</summary>

`tb3_sandbox`: 384 × 384 at 0.05 m/cell, origin `[-10, -10]`.
Cell census: **7,903 free, 870 lethal, 138,683 unknown**.

Because unknown dominates, `allow_unknown: true` (the upstream default) and
`track_unknown_space: true` both matter. Setting `track_unknown_space: false`
turns unknown into free and degenerates the test into a straight line.

Start `(-1.88, 0.08)` and goal `(1.83, 0.08)` are both free and lie in the same
4-connected free component of 7,895 cells, with lethal bands on the straight line
between them. That is what makes a 4.26 m answer to a 3.71 m straight-line
question the correct one.
</details>

---

## End-to-end navigation

Static-map planning proves the planner. Closing the loop — behaviour tree →
global plan → controller → `/cmd_vel` → odometry → goal reached — needs something
that turns velocity commands into motion. `nav2_loopback_sim` does that
kinematically, with no physics engine and no rendering.

```
  seed_pose ──(/initialpose)──►  loopback_simulator  ◄──(/cmd_vel)── controller_server
                                   │                                        ▲
                                   ├──(map→odom, odom→base_link TF)         │
                                   ├──(/odom)───────────────────────────────┤
                                   └──(/scan, raycast of the static map)    │
                                                                            │
  map_server → planner_server → controller_server → behavior_server → bt_navigator
                              lifecycle_manager (5 nodes)
```

Five lifecycle nodes, not the ten in `navigation_launch.py`. `collision_monitor`
(`source_timeout: 1.0`) silently zeroes `/cmd_vel` when TCG cannot deliver a scan
in time; `velocity_smoother` and `docking_server` add 20–50 Hz timers. None are
needed to reach a goal.

### The bringup deadlock, and how to break it

This is the one failure that looks like a hang and is not:

```
[loopback_simulator] Transform lookup failed: "base_link" ... does not exist
[global_costmap] Failed to activate global_costmap because transform from
                 base_link to map did not become available before timeout
[lifecycle_manager_nav] Failed to bring up all requested nodes. Aborting bringup.
```

There is a cycle:

| waits for | which is produced by | which requires |
|---|---|---|
| `global_costmap` activation | `map → base_link` | `loopback_simulator` publishing `map → odom` |
| `loopback_simulator` publishing | its first `/initialpose` | a client that publishes it |
| that client | the stack being active | `global_costmap` activation |

`initialPoseCallback` is what creates the integration timer
(`self.timer = self.create_timer(...)`), and `cmdVelCallback` returns immediately
while `initial_pose is None`. Until `/initialpose` arrives the simulator publishes
only an identity `odom → base_link`, so `map → base_link` is never resolvable.

**Break it by seeding `/initialpose` from launch time**, which is also what a real
system does — rviz's *2D Pose Estimate* is available before the stack is up, not
after. `seed_pose.py` publishes at 0.5 Hz and is stopped once bringup completes.

> ⚠️ **Stop the seeder before navigating.** Every `/initialpose` after the first
> re-derives `map → odom` so that `map → base_link` equals the seeded pose. Sent
> during navigation, it teleports the robot back to the start and the goal is
> never reached.

### Run it

```bash
bash ~/nav2_tcg/run_milestone_a.sh --dump ~/track_a.csv
```

### What a successful run looks like

```
[13:20:35] launching 5 lifecycle nodes + loopback simulator
[13:20:35] seeding /initialpose from launch time (breaks the activation deadlock)
[13:21:46]   active after 56s
[13:21:46] stopping the /initialpose seeder before navigation
[13:22:31]   action server up
[13:22:31]   map->base_link present (0 /initialpose sent from here)
[13:22:31]   robot starts at (-1.88, 0.08)
[13:22:31] sending NavigateToPose goal (1.83, 0.08)
[13:22:31]   goal accepted
[13:22:31]   t+    0s  robot=(-1.88,0.08)  remaining=3.71m
[13:22:46]   t+   15s  robot=( 1.00,0.50)  remaining=0.93m
[13:22:51]   action finished in 20s, status=4 (SUCCEEDED)
[13:22:53]   final pose (1.70, 0.26), goal error 0.223 m (tolerance 0.25)

[13:22:53] RESULT: PASS
```

| Phase | Wall time |
|---|---|
| launch → 5 nodes `active` | 56 s |
| `NavigateToPose` traverse, 3.71 m straight-line | 20 s |

The `t+15s` sample is the useful one: the robot is at y = 0.50 when start and
goal are both at y = 0.08. It is not driving the straight line — it is following
the detour the planner produced.

### Where it actually drove

`--dump` records the ground-truth `map → base_link` transform throughout the run.
Overlaying it on the plan from the previous section:

```
  '#' lethal   '.' free   '*' planned path   'o' actual driven trajectory   '@' both   S start   G goal
  +--------------------------------------------------------------------------------------------------------+
  |      ##...................##    ##..............###  ###...............######...................##     |
  |    ###....................###  ###..............###  ###...............#   ###...................##    |
  |   ###.......................###..................######................######.....................##   |
  |  ###...............................................................................................### |
  | ##..................................................................................................###|
  |##....................................................................................................##|
  |#...................***************@@@@@ooooooooooooo@@@@@@@@@@@@@oooooo...............................#|
  |................****......oooooo..................................*****@@@@@@@@@****....................|
  |..............***......oooo.......................................................o@**.................#|
  |............***.....oooo.............................................................@*...............##|
  |............**..ooooo.......#####.................#####.................####...........**...........### |
  |.............*So...........##   ##...............##   ##...............#######..........G..........###  |
  |...........................### ###...............###  ##...............###  ##.....................#    |
  |............................#####.................#####.................#####......................###  |
  |....................................................................................................##. |
  |.....................................................................................................###|
  |......................................................................................................##|
  |#......................................................................................................#|
  |###....................................................................................................#|
  +--------------------------------------------------------------------------------------------------------+
```

| | |
|---|---|
| samples | 545 over 20.7 s |
| driven length | **3.74 m** |
| planned length | 4.26 m |
| straight line | 3.71 m |
| mean speed | 0.18 m/s (`desired_linear_vel` 0.30) |

Two things in that table are expected behaviour, not defects. The driven path is
*shorter* than the plan because Regulated Pure Pursuit tracks a lookahead point
and cuts the corners the global planner squared off. The mean speed is below the
commanded maximum because `use_regulated_linear_velocity_scaling` slows the robot
on high-curvature segments — which is most of this route.

**The final check is the transform, not the action status.** A behaviour tree can
report `SUCCEEDED` on a goal checker satisfied for the wrong reason, so the test
reads `map → base_link` after the action returns and requires it to be within
`xy_goal_tolerance` of the requested goal.

---

## TCG tuning reference

Nav2's defaults are written for real hardware. This guest runs under QEMU TCG
with no KVM, roughly 10–50× slower than native, so several defaults are
unreachable. Values below are what this configuration uses.

| Parameter | Where | Upstream | Here | Why |
|---|---|---|---|---|
| `bond_timeout` | `lifecycle_manager` | 4.0 | **0.0** | Disables bond monitoring. See the note below — 60.0 was tried and still failed. |
| `bond_heartbeat_period` | every nav2 node | 0.1 | **0.0** | Server side of the same decision. |
| `costmap_update_timeout` | `planner_server` | 1.0 | **30.0** | The planner aborts the goal if the costmap is not fresh within this. |
| `expected_planner_frequency` | `planner_server` | 20.0 | **1.0** | Only drives a warning, but at 20.0 it floods the log on every plan. |
| `transform_tolerance` | costmaps | 0.3 | **5.0** | TF lookups fail otherwise. |
| `initial_transform_timeout` | costmaps | 60.0 | **300.0** | Costmap activation blocks on this. |
| `update_frequency` | global_costmap | 1.0 | **0.5** | |
| `publish_frequency` | global_costmap | 1.0 | **0.0** | 0.0 disables costmap publishing outright. Nothing consumes it — there is no `rviz2` — and with `always_send_full_costmap` it is a 147 KB serialize-and-publish that is thrown away. |
| `controller_frequency` | `controller_server` | 20.0 | **5.0** | 20 Hz is unreachable; missing it aborts with "control loop missed rate". |
| `movement_time_allowance` | `progress_checker` | 10.0 | **120.0** | |
| `update_duration` | `loopback_simulator` | 0.02 | **0.1** | The simulator integrates with a **fixed dt equal to this parameter**, not elapsed time. If it cannot sustain 50 Hz the robot moves proportionally slower than commanded and the progress checker aborts the goal. |

> **On disabling bond.** Bond exists so the lifecycle manager can detect a crashed
> server and tear the stack down. Under TCG that detector produces false
> positives: a healthy `planner_server` whose executor is momentarily saturated
> misses heartbeats and gets killed. A false "server is down" destroys the run,
> whereas a real crash is still caught — the action call fails and the test
> reports it. The trade is deliberate and only appropriate for a bring-up test.

---

## If it does not work

| Symptom | Cause | Fix |
|---|---|---|
| `CRITICAL FAILURE: SERVER planner_server IS DOWN after not receiving a heartbeat` | Executor saturation, not a crash. Commonly triggered by a client that creates many DDS endpoints at once — `nav2_simple_commander.BasicNavigator` builds 12 action clients, roughly 70 endpoints, and the discovery burst starves the servers. | `bond_timeout: 0.0`, and use a minimal client with one action client rather than `BasicNavigator`. |
| `ros2 topic echo /map --once` hangs forever | QoS mismatch. `/map` is `transient_local`; a volatile subscriber never matches. | `--qos-durability transient_local --qos-reliability reliable` |
| Costmap never becomes current; every goal aborts with a timeout | `obstacle_layer` is starved of `/scan` because there is no robot. | Remove `obstacle_layer` from `global_costmap.plugins`. |
| `Timed out waiting for transform from base_link to map` | Nothing publishes the robot transform. | The `static_transform_publisher` shown above. |
| Nodes start but never discover each other | Stale FastDDS shared-memory segments left by a hard-killed node. | `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` before each run. |
| A shell script dies immediately at `source /opt/ros/jazzy/setup.bash` | The ROS setup files dereference unset variables. | Do not use `set -u` in scripts that source them. |
| `waitUntilNav2Active()` never returns | Its default is `localizer='amcl'`, and there is no AMCL here. | Pass `localizer='map_server'`, or avoid `BasicNavigator`. |
| Goal aborts with "failed to make progress" | `loopback_simulator` under-running its fixed-dt integration. | `update_duration: 0.1` plus `movement_time_allowance: 120.0`. |
| `/cmd_vel` is always zero | `collision_monitor` `source_timeout: 1.0` exceeded; it silently gates all motion. | Raise it to 30.0, or do not run that node. |

---

## Environment

| | |
|---|---|
| Guest | Ubuntu 24.04 noble riscv64, 16 GB RAM, 4 vCPU, no swap, 16 GiB disk |
| Emulator | Q-Vela QEMU **10.0.2**, `-accel tcg,thread=multi` — **no KVM** |
| Networking | SLIRP user mode, `hostfwd tcp::2222-:22`. **Multicast is not forwarded**, so host↔guest DDS discovery does not work; everything here runs inside the guest. |
| ROS 2 | Jazzy, binary install from `deb [trusted=yes] http://vela.falinux.com/ubuntu noble main` |
| RMW | `rmw_fastrtps_cpp` only — no cyclonedds in `/opt/ros/jazzy/lib` |

---

## Known limitations

- **No visualisation.** `ros-jazzy-rviz2` is not packaged for riscv64; only the
  rviz libraries are. Verification is headless, which is why the acceptance test
  asserts against the occupancy grid rather than against a picture.

- **The loopback simulator is kinematic, not physical.** It integrates `/cmd_vel`
  perfectly into `/odom`. There is no wheel slip, no inertia, no actuator lag, and
  localisation is an identity `map → odom`. For an availability test this is
  defensible — executors, timers, DDS traffic and publication rates are all real,
  and those are what a stalled RAS handler would disturb — but aggregate CPU load
  is materially lower than a physically simulated robot, and AMCL, collision
  handling and recovery behaviours contribute nothing. Any claim of the form "the
  system stayed available under load" must state the measured load level.

- **Gazebo is installable but not usable in practice — *in the guest*.** `ros-gz-sim`
  and `gz-sim-vendor` exist for riscv64, but under 4-vCPU TCG with software rendering
  they cannot approach real time, which makes wall-clock latency criteria
  meaningless. That limitation is what [`gz-split/`](gz-split/) exists to route
  around, and it is also what the TSP figure prescribes: the simulator runs on the
  x86 host at `real_time_factor` 1.0, the guest runs only Nav2 and slam_toolbox, and
  the robot drives for real. So this bullet bounds where Gazebo may run, not whether
  the Gazebo + SLAM scenario is reachable.

- **Absolute timings here are not silicon performance.** QEMU TCG has no timing
  model. Every number in this document is a property of this host, this QEMU
  build and its concurrent load — valid as a regression baseline, not as a
  characterisation of RISC-V hardware.

- **This guest has no RAS.** The configuration used here boots with
  `-machine virt,wg=on` and no `reri=true`, no EDK2 pflash pair. The RAS-capable
  configuration in [q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md)
  boots a buildroot initramfs, where ROS 2 cannot run at all. **The two are
  currently mutually exclusive**, and TC.10.007 needs a merged configuration —
  Ubuntu root disk plus `reri=true` plus the pflash pair — that has not yet been
  built or validated. That is the gating prerequisite for the availability test,
  not Nav2.

---

## Files

Everything in this document is in [`nav2/`](nav2/). Copy it to the guest with
`scp -P 2222 -r nav2 vela@localhost:~/nav2_tcg` and run the numbered scripts in
order; each prints its own PASS/FAIL verdict.

| File | Role |
|---|---|
| `00-preflight.sh` | Dependency-closure check. Installs nothing. |
| `01-install-phase1.sh` | Curated planner set + extraction of maps and params |
| `02-install-phase3.sh` | Full navigation set + the two source-built packages |
| `planner_only.{yaml,launch.py}`, `check_plan.py` | Milestone B |
| `loopback_nav.{yaml,launch.py}`, `check_navigate.py`, `seed_pose.py` | Milestone A |
| `run_milestone_b.sh`, `run_milestone_a.sh` | One-terminal orchestration: clean → launch → wait → verify → tear down → verdict |

`check_plan.py --dump FILE` writes the planned path and `check_navigate.py --dump
FILE` writes the driven trajectory, as CSV. Both diagrams above were produced
from those files.

---

## See also

- [`gz-split/`](gz-split/) — the next step: a real Gazebo simulation and SLAM, with the simulator on the x86 host and Nav2 plus slam_toolbox still in the guest
- [`ras/README.md`](README.md) — RERI injection, rasdaemon, the error-code table
- [q-vela `docs/ras.md`](https://github.com/riscv-vela/q-vela/blob/main/docs/ras.md) — the RAS boot chain and its four mandatory options
- [Nav2 documentation](https://docs.nav2.org/) — upstream configuration reference
