# YOLOv8n on the RISC-V guest

The AI-inference co-tenant for TC.10.007: a camera topic in, YOLOv8n through
onnxruntime, `vision_msgs/Detection2DArray` out on `/detection_result` -- the
topic the TSP's monitoring precondition names. Sensor input is a replayed
rosbag, as the test-plan correction request (CR-12) specifies; a live Gazebo
camera exists as the demonstration variant.

Like `../gz-split`, everything here is Config A -- no `reri=true`, no EDK2
pflash -- so this produces the workload for TC.10.007, never a TC.10.007
result.

## The runtime is built, not installed

There is no way to install onnxruntime for this guest, and each dead end is
worth knowing before someone retries it:

- PyPI ships no riscv64 wheel and no sdist at all, so `pip install
  onnxruntime` fails before it can even try to compile.
- Debian trixie/sid and Ubuntu questing do build `python3-onnxruntime` for
  riscv64, but against Python 3.13; the guest's noble has 3.12. Their C++
  `libonnxruntime` packages would work, but only for a C++ node.
- SpacemiT's `python3-spacemit-ort` 2.0.3 is the one noble-based (cp312)
  prebuilt binding in existence, and it dies with SIGILL at `import` under
  QEMU -- the library executes SpacemiT K1 IME instructions that no standard
  emulator implements. Measured 2026-08-18 under qemu-user; the q-vela TCG
  guest traps identically. Their 1.2.4 depends on Python 3.13. Both dead.

So `scripts/build_ort_wheel.sh` builds the upstream wheel natively inside a
riscv64 Ubuntu 24.04 container (qemu-user via binfmt, no sudo needed on this
host). Same distro as the guest means the wheel's glibc, libstdc++ and CPython
ABI match by construction, and the finished wheel is smoke-tested in that same
container before it ever touches the guest. Version v1.23.2 is the tag
Debian's riscv64 buildd compiles routinely; RVV stays off because TCG emulates
vector kernels at ~70x scalar cost (gz-split HANDOFF measurement). Measured
2026-08-18 on this host: about 1.5 h wall at `PARALLEL=48` (Debian's real
riscv64 hardware takes five to six).

If the Python path ever regresses, the fallbacks in order: extract Debian
trixie's `libonnxruntime1.21` deb (noble-compatible: libc6 >= 2.38,
libstdc++6 >= 14) and write the node in C++; or `pip install ncnn`, which
does publish riscv64 cp312 wheels, and re-export the model with
`yolo export format=ncnn`.

## The model

`scripts/export_model.sh` exports `yolov8n.onnx` on the host -- opset 12
(runs on every runtime candidate down to 1.15), imgsz 320 static (D-13 sized
the TCG budget at 320x320), no embedded NMS (the reference decoders all read
the raw `[1,84,2100]` head, and conf/iou stay runtime-tunable). It then runs
the exported graph through `tools/detect.py` on the host and asserts the
expected objects, so any later riscv64 failure is an environment problem, not
a model problem.

`tools/yolov8_onnx.py` carries what ultralytics would have provided at
runtime -- letterbox, decode, NMS, COCO names -- because ultralytics needs
PyTorch and there is no riscv64 PyTorch (D-13). Plain numpy; cv2 is used for
image I/O only, never `cv2.dnn` (the guest's OpenCV 4.6 cannot load YOLOv8
graphs).

## The camera is billboards, and why

The gz-split worlds carry no camera on purpose -- bridging the TB4's RGBD
costs five times the lidar -- and a COCO detector cannot name geometric
primitives anyway. `tools/make_camera_world.py` builds a separate world: a
static 640x480 camera at 1 Hz staring at photo billboards (the ultralytics
bus/zidane test photos), which a COCO model detects reliably and which render
for the cost of two textured quads. `scripts/make_bag.sh` records ~40 frames
of it inside the vela-gz container (sqlite3 storage, spelled out, because
Jazzy's default is mcap and the guest installs the sqlite3 plugin) and copies
the bag to the guest. A frame pulled back out of the bag scores person 0.83 /
bus 0.47 on the host -- the pipeline is validated before the guest is
involved.

## Bring-up

One-time, on the host. The guest must be up (`q-vela/run.sh`) and key-auth
reachable -- the `vela` account has an empty password, which non-interactive
ssh cannot type, so once: `ssh -p 2222 vela@localhost` (empty password), then
append your `~/.ssh/id_ed25519.pub` to `~/.ssh/authorized_keys` there.

```bash
scripts/00-preflight.sh        # what is missing, if anything
scripts/build_ort_wheel.sh     # starts the hours-long wheel build, detached
scripts/export_model.sh        # model + host golden run
scripts/make_bag.sh            # camera world -> 40 s bag -> guest
CHECK=1 scripts/build_ort_wheel.sh   # poll the build
scripts/build_ort_wheel.sh     # wheel exists now -> container smoke test (G-Y1)
scripts/install_guest.sh       # apt deps + wheel + model + tools into the guest
```

## Run it

```bash
scripts/run_node_check.sh      # G-Y6: the daemon-facing contract, on x86, in 40 s
scripts/run_detect.sh          # G-Y2: guest, no ROS -- runtime + model only
scripts/run_yolo.sh            # G-Y3: bag replay -> node -> /detection_result
LIVE=1 scripts/run_yolo.sh     # G-Y4: live Gazebo camera over the TCP profiles
KEEP=1 scripts/run_yolo.sh     # leave it running for an audience
```

G-Y6 is first because it is the only one that does not need the guest: it runs the
node against a stub of the runtime inside the x86 container, so a broken heartbeat
or a swallowed exception is found in 40 s rather than after a 300 s bring-up. The
others need the wheel, the model and the bag.

All five run the detector on its own. The one that matters for TC.10.007 runs it
**beside Nav2 and slam_toolbox**, in the same guest against the same simulator,
and lives in the split rig because that is where the other two already are:

```bash
cd ../gz-split
YOLO=1 WATCHDOG= RTF_LIMIT=0.25 ROBOT=tb3 scripts/run_split.sh   # G-Y5
```

`RTF_LIMIT=0.25` is not a preference. At 0.5 the detector took the headroom the
clock needed and Nav2 aborted both goals on a TF lookup 7.3 s behind its own
buffer; halving the simulation rate gave it back. Neither is the camera's
320x240: at 640x480 the guest spent 32 points of an emulated core turning frames
into Python objects that the node then discarded, and the model letterboxes to
320x320 whatever it is given. Both numbers are measured, in
[`../gz-split/RUNLOG.md`](../gz-split/RUNLOG.md) under G-Y5.

`tools/make_billboards.py` is what makes that possible: it emits the camera and
its billboards as one spawnable model, so they can be dropped into a world that
is already driving a robot. The scene goes several metres outside the arena --
a `gpu_lidar` samples the render scene, so a billboard is an obstacle to it even
with no collision geometry, and inside the arena it would appear in the SLAM map
and fail the map gate as if the mapper were broken. See
[`../gz-split/README.md`](../gz-split/README.md).

A successful G-Y3 prints the node's first detection line and then confirms
the topic itself. Measured 2026-08-19 on the q-vela guest:

```
[detect] seq=1 stamp=23.000 n=7 infer_s=11.4 rx=12 labels=person:0.83,person:0.57,person:0.55,bus:0.47,...
/detection_result: message received
RESULT: PASS (G-Y3 bag)
```

`rx=12` against `seq=1` is the frame-drop design doing its job: twelve frames
arrived while the first pass ran, and the node processed the newest one. One
320x320 pass measured 11.3-11.5 s -- the good end of D-13's 10-60 s budget. That
line predates the `drop=` and `mono=` fields; the current format is below.

## Filming it

The rig that runs the three workloads together is also the one that films them, so
the recipe lives with the recorder:
[`../gz-split/README.md`](../gz-split/README.md), "Recording it with the detector".
Three facts belong here rather than there.

**The detections get their own window, not an RViz panel.** RViz gives its only dock
the full height of the pane, which spent a third of an 864 px frame on a 320x240
image, so `detection_overlay.py --show` opens a window that the recorder tiles like
any other. The overlay runs on the host and draws nothing in the guest.

**One screen refresh every 40 s, and the camera is why.** 0.1 Hz of sim time at
RTF 0.25 is a frame every 40 s of wall clock, against 14-32 s to infer on it: the
node waits for frames, not the reverse (`rx` tracks `seq` exactly). A recording
shorter than ~240 s shows the boxes appear once and then look like a still.

**Post-processing is `setpts=0.25*PTS`.** The factor undoes the real-time factor, so
it has to match the `RTF_LIMIT` the take was filmed at. Take 3, filmed without the
detector at RTF 0.5, used 0.5; reusing that here would show the robot at twice its
real speed.

Measured 2026-08-19, the first take with all three workloads in frame: 4/4
waypoints, map 0.961 / 0.969 / 0.999, 24 inference passes at 14.1 / 21.4 / 32.0 s
(min / p50 / max), guest load 291% of 400% across 8 processes. The detector's share
was 82.5% of one emulated core, and the 1 Hz `/diagnostics` heartbeat added 0.7 of
those points.

## What the RAS error-logging daemon reads

Two things, and they answer different questions. Both are the detector's side of
TC.10.007's "AI inference stayed healthy across the injection", and both are
gated by `scripts/run_node_check.sh` (G-Y6) so a consumer can be written against
them before the guest is involved.

**The `[detect]` line, one per completed pass.** Append to it; do not reorder it.

```
[detect] seq=12 stamp=105.200 n=2 infer_s=38.4 rx=61 drop=48 mono=8123.412 labels=person:0.87,bus:0.72
```

`stamp` is the frame's, so it is **sim** time: it stops when the simulator does
and runs backwards when a bag loops. `mono` is `CLOCK_MONOTONIC` at the end of
the pass, which is the only field comparable with anything outside this
simulator, and it is the clock
[`../ras-ros/`](../ras-ros/) stamps RAS events with under the same name. "What
was the detector doing when the error arrived" is then a subtraction rather than
an argument. A failed pass logs `[detect-error]` in the same shape.

**`diagnostic_msgs/DiagnosticArray` on `/diagnostics`, once a second**, named
`yolo/detector`. A log line is evidence for a person reading a file afterwards; a
topic is evidence a program can act on while the test is running, and TC.10.007
asks about a moment. `diagnostic_msgs` is what ras_monitor publishes for the same
reason, so the error and the workload's reaction to it arrive on one topic in one
type, with no `rosidl` build on an emulated guest.

| Field | Reading it |
|---|---|
| `level` | `OK` inferring, `WARN` starting up or the camera has gone quiet, `ERROR` a pass raised |
| `state` | `waiting`, `inferring`, `idle`, `failed` -- what a consumer switches on |
| `consecutive_failures` | reset by any pass that completes |
| `seq` / `rx` / `dropped` | passes completed, frames received, frames replaced before being inferred on |
| `mono` | `CLOCK_MONOTONIC` when the status was built |
| `detect_mono` | the same clock at the end of the last completed pass |
| `frame_age_s` | since the newest frame arrived. This is what turns `WARN` on, at `stale_s`, default 120 s |
| `last_infer_s` / `last_n` / `last_labels` | the last pass |
| `last_stamp_sim` | the frame's sim-time stamp, for joining to the simulator's log |

Three deliberate choices in that status:

**One second, not the ten ras_monitor uses.** There the events are the signal and
the heartbeat only proves the reporter exists. Here the heartbeat **is** the
signal: a pass completes every 40 s of wall clock under co-tenancy, so a status
every 10 s cannot resolve the TSP's 20 s detection criterion, let alone its 3 s
MTTR budget. It is also the only thing in this node that runs at a fixed rate.

**`TRANSIENT_LOCAL`.** Every monitor attaches mid-test, because the rig takes 80 s
to come up. A late subscriber on a volatile topic waits a full period seeing
nothing, and nothing is exactly what a dead detector looks like. Offered
durability may exceed requested, so a volatile subscriber still matches.

**A failed inference reports and the node stays up.** A node that exited on an
exception would leave the same silence as a node the fault killed, and an
availability test cannot tell those apart. G-Y6 asserts this with a stub that
raises on demand.

What is still the daemon's own, and deliberately not here: the join itself, the
downtime arithmetic, and the verdict table. Also `ras-monitor.service` sets no
`ROS_DOMAIN_ID` and no Fast DDS profile, so it comes up on domain 0 over shared
memory -- a different ROS graph from a rig running on domain 42 over TCP. The
transport half of that is
[`../gz-split/config/host_ras_server.xml`](../gz-split/config/host_ras_server.xml)
(port 42105); the two environment variables are the unit's to set.

The node publishes an empty `Detection2DArray` when a frame has no objects:
for an availability test the message flow is the liveness signal, and a fault
must show as `/detection_result` going quiet rather than as an ambiguous
empty scene. Frames that arrive mid-inference are dropped by design -- the
newest frame wins, a backlog of stale camera frames is pure cost. Inference
takes 2 of the 4 vCPUs so Nav2 and slam_toolbox keep breathing beside it.

## If it does not work

| Symptom | Cause and fix |
|---|---|
| `port 2222: Connection refused` | the VM is down -- `q-vela/run.sh`; the SLIRP listener also accepts before sshd is ready, so right after boot, wait |
| ssh asks for a password | key not installed in the guest -- see Bring-up |
| `Illegal instruction` importing onnxruntime | a vendor build slipped in; only the wheel from `build_ort_wheel.sh` is known clean |
| `pip: error: externally-managed-environment` | install with `--break-system-packages`, as `install_guest.sh` does |
| bag replay: unknown storage format | the bag is mcap (recorded without `--storage sqlite3`) or the guest lacks `ros-jazzy-rosbag2-storage-sqlite3` |
| node runs, `/detection_result` silent | domain mismatch -- everything here uses `ROS_DOMAIN_ID=42`; check the profile env in LIVE mode |
| `ros2: command not found` over ssh | `bash -l` reads `~/.profile`, which does not source ROS -- every script sources `/opt/ros/jazzy/setup.bash` explicitly; do the same |
| a pass takes a minute | expected -- TCG runs 320x320 in tens of seconds (D-13); it is not hung until well past that |

## Environment

| Piece | Value |
|---|---|
| Guest | Ubuntu 24.04 riscv64, Python 3.12, ROS 2 Jazzy (Vela APT), 4 TCG vCPU, RVV 1.0 vlen=512 |
| Runtime | onnxruntime v1.23.2, self-built cp312 linux_riscv64 wheel, CPU EP, scalar MLAS |
| Model | yolov8n.onnx, opset 12, 320x320 static, raw head + numpy NMS |
| Camera, standalone | Gazebo Sim 8 (vela-gz container), 640x480 rgb8, 1 Hz for the bag / 0.2 Hz live, photo billboards |
| Camera, co-tenant (G-Y5) | 320x240 rgb8 at 0.1 Hz of **sim** time, billboards spawned at (30, 0) |
| Bag | sqlite3, ~44 frames / ~39 MB, replayed `--loop` in the guest |
| Host build | docker + binfmt qemu-riscv64, riscv64 Ubuntu 24.04 container, GCC 14 |

## Known limitations

**The cadence is the emulator's, not the design's.** One 320x320 pass costs
tens of seconds under TCG, so detection frequency criteria are meaningless
here; CR-12 relaxes the inference-rate expectation for exactly this reason.

**And under co-tenancy the camera is slower still than the detector.** Measured
2026-08-19 in the G-Y5 configuration, `/detection_result` publishes every 39.98 s
(p50, max 48.04) -- 40 to 48 times the TSP's 1,000 ms topic-latency budget, and
longer than its 20 s detection window. So that budget cannot be applied to this
topic and `ros2 topic hz` on it cannot answer whether the detector is alive: at
0.025 Hz, two samples take 80 s. That is what the 1 Hz `/diagnostics` heartbeat is
for, and it is filed as D-36 and D-39 with CR-16 proposing the replacement
criteria.

**A corrected error is unlikely to be visible in `infer_s` at all.** The normal
spread within one run is 14-32 s, so the kernel's cost of handling one CE is far
under the noise. The claim this workload can support is that the heartbeat did not
stop and at least one pass completed inside the injection window -- not that
inference time was statistically unchanged. 720 s between injections is about 18
passes, which has no power for that test.

**This is the co-tenant workload, not the availability result.** The guest is
Config A. Until the Config C merge (Ubuntu root disk + `reri=true` + pflash)
exists, nothing here feeds TC.10.007 numbers.

**Billboards are photographs, not simulated humans.** The camera exercises
the full sensor-to-detection path; it does not claim scene fidelity. If the
robot's own camera view is ever wanted, the RGBD bridge cost documented in
gz-split applies.

**The wheel is ABI-specific.** cp312 / noble / glibc 2.39. A guest image
rebased on another Ubuntu means rebuilding it (`ORT_TAG` is the only knob).

## Files

| File | Role |
|---|---|
| `tools/yolov8_onnx.py` | letterbox, decode, numpy NMS, COCO names -- the runtime-side model glue |
| `tools/detect.py` | standalone smoke test with `--expect`; the same file gates host, container and guest |
| `tools/yolo_node.py` | the detector node: latest-frame-wins, empty-array heartbeat, `[detect]` log line |
| `tools/image_publisher.py` | images-to-topic stand-in when the bag or Gazebo is unavailable |
| `tools/make_camera_world.py` | camera + billboard world and bridge yaml generator, for the detector on its own |
| `tools/make_billboards.py` | the same scene as one **spawnable model**, for dropping into a world that is already driving a robot (G-Y5) |
| `tools/detection_overlay.py` | host-side: draws the guest's detections onto the host's frames, publishes `/detection_image`, and with `--show` opens the window a recording films |
| `tools/check_yolo_node.py` | G-Y6 -- heartbeat, failure reporting and `detect_mono`, against a stubbed runtime |
| `scripts/00-preflight.sh` | what is missing before anything runs |
| `scripts/build_ort_wheel.sh` | detached wheel build + container smoke test (G-Y1) |
| `scripts/build-in-container.sh` | the build itself; runs inside the riscv64 container |
| `scripts/export_model.sh` | onnx export + host golden run |
| `scripts/make_bag.sh` | Gazebo camera -> rosbag -> guest |
| `scripts/install_guest.sh` | apt deps, wheel, model, tools into the guest |
| `scripts/run_detect.sh` | G-Y2 -- guest inference without ROS |
| `scripts/run_yolo.sh` | G-Y3 bag replay / G-Y4 `LIVE=1` / `KEEP=1` demo |
| `scripts/run_node_check.sh` | G-Y6 -- the daemon-facing contract, in the x86 container |

Build artifacts (wheel, model, bag, goldens) live outside the repository in
`~/ms_kim/yolo-artifacts/`; nothing binary is committed.

## See also

- [`../gz-split/README.md`](../gz-split/README.md) -- the Nav2/SLAM split this
  detector runs beside, and the TCP profiles LIVE mode borrows
- [`../workload-nav2.md`](../workload-nav2.md) -- the guest-only Nav2 workload
- [`../README.md`](../README.md) -- RAS error injection and rasdaemon, which
  this workload exists to run under
