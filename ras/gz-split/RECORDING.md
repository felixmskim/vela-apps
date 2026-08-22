# Recording the circuit demonstration

How to reproduce the Take 11 recording end to end: a TurtleBot3 driving a slalom
between the pillars of `turtlebot3_world` while YOLOv8n labels the person models
it passes, with Gazebo on the x86-64 host and Nav2 plus slam_toolbox in the
RISC-V guest.

**What this procedure produced on 2026-08-22, stated plainly.** The rig came up,
drove, mapped and detected; the gates did not pass.

```
RESULT: FAIL (G5, tb3)  nav_rc=1 map_rc=1 yolo_rc=0 lat_rc=0
  waypoints  4/8        wp1-wp4 reached in 18-22 s each; wp5 onward MISSED
  precision  0.645      floor 0.80   LOW
  recall     0.937      floor 0.85   ok
  coverage   0.880      floor 0.90   LOW
  G6 CPU     287% of 400%
  RTF        0.24999511
  G-Y5       23 detections, drop=0, infer_s 17.7-36.3 s, best person:0.92
```

From wp5 the robot wedged against the centre pillar and oscillated there for the
rest of the run. Follow this procedure and expect the same outcome unless
something changes; the deliverable is a trimmed clip of the part that drove, not
a passing gate run. Section 5 says which part.

Everything below was executed on 2026-08-22 and the expected outputs are that
run's actual stdout, not estimates.

---

## 1. Preconditions

Run all of these before launching anything. Each has an expected output; a
different one is the answer, not a puzzle.

### 1.1 Host container and viewer image

```bash
docker ps --filter name=vela-gz --format '{{.Names}} {{.Status}}'
docker image inspect vela-gzsim:demo --format '{{.RepoTags}}'
```

Expected:

```
vela-gz Up 3 days
[vela-gzsim:demo]
```

If the container is missing, see [`README.md`](README.md) "Build — host". If the
image is missing:
`docker build -f Dockerfile.demo -t vela-gzsim:demo .`

### 1.2 Host Fuel cache — the viewer needs it

`../yolo/tools/make_people.py` writes `artifacts/people/*.sdf` with **absolute**
mesh URIs under `file:///home/vela/.gz/fuel/`. `vela-gz` holds that cache in its
own writable layer; the viewer image has no `~/.gz` at all. `run_demo.sh`
bind-mounts a host copy read-only so the same URI resolves on both sides.

```bash
find ~/ms_kim/gz-fuel-cache -type f | wc -l
ls -la ~/ms_kim/gz-fuel-cache/fuel.gazebosim.org/openrobotics/models/"standing person"/3/meshes/
```

Expected:

```
108
total 4420
-rw-r--r-- 1 cslab cslab 4514010 Aug 20 02:47 standing.dae
```

If it is missing or empty, seed it from the container that has it:

```bash
mkdir -p ~/ms_kim/gz-fuel-cache
docker cp 'vela-gz:/home/vela/.gz/fuel/.' ~/ms_kim/gz-fuel-cache/
```

`FUEL_CACHE=` overrides the location in both scripts. Both refuse to start
without it rather than filming an arena with no people in it.

### 1.3 Guest reachable and correctly sized

```bash
ssh -p 2222 -o BatchMode=yes vela@localhost 'uname -m; nproc; free -g | head -2'
```

Expected:

```
riscv64
4
               total        used        free      shared  buff/cache   available
Mem:              15           0          15           0           0          15
```

`Connection refused` means the guest is not running — go to section 2.

**Port 2222 only.** Port 2221 may also be listening; that is a different
person's Config C VM. Do not touch it.

### 1.4 Guest stack

```bash
G="ssh -p 2222 -o BatchMode=yes vela@localhost"
$G 'source /opt/ros/jazzy/setup.bash && ros2 pkg list | grep -cE "^nav2_"'
$G 'dpkg -l ros-jazzy-slam-toolbox ros-jazzy-vision-msgs | grep -c ^ii'
$G 'python3 -c "import onnxruntime;print(onnxruntime.__version__)"'
$G 'ls -la ~/yolo/yolov8n.onnx'
$G 'for d in ~/gz_split ~/yolo; do echo "$d $(ls -1 $d | wc -l)"; done'
```

Expected:

```
23
2
1.23.2
-rw-rw-r-- 1 vela vela 12725063 Aug 18 14:53 /home/vela/yolo/yolov8n.onnx
/home/vela/gz_split 28
/home/vela/yolo 11
```

Anything missing: `../yolo/scripts/install_guest.sh` reinstalls the detector
side; the `gz_split` side is copied by `run_split.sh` itself at step 0.

### 1.5 The guest is running the same code as this checkout

The one failure mode with no symptom: an old `send_goal.py` in the guest has no
`--settle`, so `SETTLE=30` is silently ignored and the take fails the way Take 10
did.

```bash
md5sum tools/send_goal.py tools/slam_nav.launch.py tools/clock_lag_probe.py \
       tools/score_map.py config/nav2_split.yaml config/guest_tcp_client.xml \
       ../yolo/tools/yolo_node.py
ssh -p 2222 -o BatchMode=yes vela@localhost \
  'cd ~/gz_split && md5sum send_goal.py slam_nav.launch.py clock_lag_probe.py \
   score_map.py nav2_split.yaml guest_tcp_client.xml; cd ~/yolo && md5sum yolo_node.py'
```

Expected: **the seven checksums match pairwise.** They did on 2026-08-22:

```
8d3abbfdb0f01dbb370ae0cabff6c456  send_goal.py
0a4f0b55e8afd19fabd22f36c46b0fe3  slam_nav.launch.py
fc725ebfc601171e549c25bfd3191617  clock_lag_probe.py
3b3137d1bb3c9c208f40256ca9d679df  score_map.py
2af848fb40ac1bc980c175ace02f98ad  nav2_split.yaml
e808eec5b2254812af76ec25f927432f  guest_tcp_client.xml
cba20673cbe8c86d49453d274dfee6a7  yolo_node.py
```

### 1.6 DDS crosses SLIRP

```bash
scripts/run_dds_check.sh 2>&1 | tail -12
```

Expected (about 3 minutes):

```
[..] G3 PASS -- first sample crossed after 42s of the 120s budget
     samples 200/200, lost 0
       p50      11.4 ms
       p95      17.2 ms   <- G4
     RESULT: PASS (p95 17.2 ms vs budget 500 ms, lost 0)
[..] RESULT: PASS (G3 + G4 + intra-guest regression)
```

### 1.7 Clean start line

```bash
ls -d /tmp/run_split.release /tmp/run_demo.vela-gz-view.lock 2>/dev/null
docker ps -a --filter name=vela-gz-view --format '{{.Names}}'
docker exec vela-gz bash -lc "pgrep -af '[g]z sim'"
ssh -p 2222 -o BatchMode=yes vela@localhost \
  'pgrep -af "[y]olo_node|[s]lam_toolbox|[b]t_navigator|[i]mage_publisher"'
```

Expected: **all four produce no output.** Anything printed is left over from a
previous run — see section 7 and tear it down first. A stray
`image_publisher.py` in particular would put a JPEG slideshow on `/camera/image_raw`
instead of the Gazebo camera.

---

## 2. Starting the guest VM — q-vela Config A

Config A is the demonstration guest: port 2222, account `vela`, the persistent
`vela/ubuntu-noble-riscv64.img`. It has no `reri=true` and no EDK2 pflash pair,
so it produces the co-tenant workload for TC.10.007 and never a TC.10.007
result. Every default in these scripts (`GUEST=vela@localhost`, `PORT=2222`,
`GUEST_DIR=/home/vela/gz_split`) already matches it, so there is nothing to
override.

This is `q-vela/run.sh` in full:

```bash
cd ~/ms_kim/riscv-vela/q-vela
./qemu/build/qemu-system-riscv64 \
    -machine virt,wg=on -nographic -m 16384 -smp 4 \
    -bios ./vela-fws/opensbi/build/platform/generic/firmware/fw_jump.bin \
    -kernel ./vela-linux/linux/arch/riscv/boot/Image \
    -append "root=/dev/vda1 rw console=ttyS0 earlycon=sbi" \
    -cpu rv64,v=true,vlen=512,vext_spec=v1.0,zba=true,zbb=true,zbs=true,zbc=true,zbkb=true,zcb=true,zicond=true \
    -accel tcg,thread=multi \
    -semihosting-config enable=on,target=native,userspace=on \
    -netdev user,id=eth0,hostfwd=tcp::2222-:22 -device virtio-net-pci,netdev=eth0 \
    -device virtio-rng-pci \
    -drive file=./vela/ubuntu-noble-riscv64.img,format=raw,if=virtio \
    -device gemmini,id=gemmini0 \
    -device gemmini,id=gemmini1 \
    -device gemmini,id=gemmini2 \
    -device gemmini,id=gemmini3 \
    -D qemu.log
```

No `sudo`. `-nographic` puts the serial console on this terminal and keeps it,
so use a terminal you can leave alone; there is no QMP socket, so this terminal
is the only way to reach the console.

Wait for SSH rather than watching the console:

```bash
until ssh -p 2222 -o BatchMode=yes -o ConnectTimeout=5 vela@localhost true 2>/dev/null; do sleep 5; done
ssh -p 2222 -o BatchMode=yes vela@localhost uptime
```

Expected: about 60-90 s to first successful login, then

```
 07:49:31 up 1 min,  2 users,  load average: 1.77, 0.60, 0.21
```

The rootfs is persistent, so `~/gz_split`, `~/yolo` and `~/yolo/yolov8n.onnx`
survive a reboot. Verify with 1.4 rather than assuming it.

---

## 3. Running it — two terminals

### Terminal 1 — the rig

```bash
cd ~/ms_kim/riscv-vela/vela-apps/ras/gz-split
YOLO=1 YOLO_CAM=robot YOLO_RATE=0.1 SETTLE=30 WATCHDOG= GUI=1 ROBOT=tb3 \
HOLD=1 PRE_DRIVE_SLEEP=180 \
WAYPOINTS_OVERRIDE="-0.55,-0.55 0.55,-0.55 1.70,0.00 0.55,0.55 -0.55,0.55 -1.70,0.00 0.00,-1.70 -2.00,-0.50" \
  scripts/run_split.sh 2>&1 | tee logs/p4-take11-$(date +%Y%m%d).log
```

Every value has a reason:

| Set | Why |
|---|---|
| `YOLO_CAM=robot` | **Explicit on purpose.** `scene` builds photo billboards from `bus.jpg`/`zidane.jpg` outside the arena and films those instead. It is not the default any more, but the 2026-08-19 video was shot that way and the mistake is invisible until you watch the recording |
| `YOLO_RATE=0.1` | what Takes 8-11 used. `robot` mode defaults to 0.25; raising it makes the detector the bottleneck and changes G6 — do not without re-measuring |
| `SETTLE=30` | probe goals at the spawn point until `bt_navigator` serves one. Take 10 used 12, exhausted them and lost wp1-wp5. Take 11 converged on probe 11 |
| `WATCHDOG=` (empty) | the liveliness instrument takes 57-69% of a core and changes what it measures. Off for a recording; say so in any report |
| `HOLD=1` | the run's EXIT trap kills the Gazebo **server** the moment the verdict prints, while `run_demo.sh` watches the **viewer's** GUI — without this the recording continues over a frozen scene with no warning |
| `PRE_DRIVE_SLEEP=180` | holds at step 7 so the recorder is live and past its own 90 s startup before the robot moves |
| `GUI=1` | builds the world with SceneBroadcaster and generates the guest profile carrying the 42103/42104/42105 locators. Without it the GUI draws an empty world forever |
| `ROBOT=tb3` | tb4's depot world needs a 101 MB Fuel cache the viewer container does not have |
| no `RTF_LIMIT` | the script lowers it to 0.25 itself and prints that it did, which is a free confirmation |
| no `FOLLOW_OFFSET_OVERRIDE` | `robot.env`'s per-robot `-1.0,0,0.45` is correct for this arena. See section 6 |
| 8 waypoints | `+/-0.55` are the exact midpoints of adjacent pillar gaps (gap 0.80 m, robot radius 0.22, so 0.18 m each side); `1.70,0` and `-1.70,0` and `0,-1.70` leave the pillar field; `-2.00,-0.50` is the spawn point, closing the loop |

Expected output, in order, with the lines that matter:

```
[17:14:54] 0/10 preflight
guest files ok
[17:14:56]       GUI=1 -- generated guest_tcp_client_demo.xml (adds 42103 rviz2, 42104 overlay,
[17:14:56]       42105 ras_monitor -- see config/host_ras_server.xml)
[17:15:01] 1/10  host: world + robot for tb3
[17:15:01]       GUI=1 -- demonstration run, NOT a measurement run
robot  : RGB camera on base_link -> gz topic /camera, 320x240 @ 0.1 Hz sim
[17:15:02]       YOLO=1 (robot camera) -- merging the camera bridge entry
[17:15:02]       RTF_LIMIT was unset or 0, so it is 0.25 -- three workloads do not fit faster
[17:15:05]       host Fuel cache OK for the viewer (/home/cslab/ms_kim/gz-fuel-cache)
[17:15:06]       world=default bridge=/work/artifacts/bridge_with_camera.yaml base=base_footprint offset=-2,-0.5
[17:15:06]       WAYPOINTS_OVERRIDE in effect: -0.55,-0.55 0.55,-0.55 1.70,0.00 0.55,0.55 -0.55,0.55 -1.70,0.00 0.00,-1.70 -2.00,-0.50
[17:15:06] 2/10  host: gz sim, headless
[17:15:17] 3/10  host: spawn robot (file mode)
[17:15:19]       spawning person_1 (standing) at (1.85, -0.75) facing 4.71
[17:15:20]       spawning person_2 (walking) at (-0.60, 1.50) facing 0.99
[17:15:22]       spawning person_3 (standing) at (-1.90, 0.60) facing 1.55
[17:15:23]       spawning person_4 (standing) at (0.75, -1.85) facing 4.15
[17:15:25]       waiting for the camera to advertise /camera on gz transport
[17:15:32]       /camera is live
[17:15:42] 4/10  host: bridge + clock decimator + cmd_vel watchdog, on the TCP profile
[17:15:56] 5/10  guest: can it see the bridged clock and scan?
/clock /cmd_vel /odom /scan /tf
[17:17:07]       /clock: average rate: 4.636
[17:17:07] 6/10  guest: Nav2 + slam_toolbox
[17:17:28]       + yolo_node (G-Y5 co-tenant)
[17:17:39]       waiting for lifecycle activation (up to 900s; bringup was 56s locally,
[17:17:39]       and every TF lookup now crosses a TCP link)
[17:20:26]       active after 95s
[17:20:26]       checking the detector is receiving frames (its own /diagnostics)
[17:21:18]       NOTE: the detector has no frames yet (rx=?). At 0.1 Hz sim and
[17:21:18]       RTF_LIMIT=0.25 that is up to 40 s apart, so this is only a
[17:21:19]       failure if the G-Y5 check at the end also finds no [detect] line.
[17:21:19] 7/10  guest: is the guest's clock keeping up with the host's?
[17:21:22]       host sim 91s   guest clock 91.32s   lag -0.3s
[17:21:22]       inside CLOCK_LAG_MAX=5s -- TF lookups at now() can resolve
[17:21:22]       holding 180s before driving so a recorder can attach     <-- TRIGGER
```

That `NOTE: the detector has no frames yet` is not a fault and does not stop the
run. One camera frame every 0.1 Hz of sim time at RTF 0.25 is one every 40 s of
wall clock, so the check can easily land between two of them.

Five of those lines are checks in disguise:

- `RGB camera on base_link ... @ 0.1 Hz sim` — `YOLO_CAM` really is `robot`.
- `host Fuel cache OK for the viewer` — the people will render in the viewer.
- `spawning person_1 ... person_4` — four models went in, before the guest stack
  started, so slam_toolbox's first scan already contains their legs.
- `active after 95s` — Nav2 lifecycle came up. 900 s is the ceiling.
- `lag -0.3s ... inside CLOCK_LAG_MAX=5s` — necessary, **not sufficient**. Take 10
  passed this line at `lag 0.3s` and still failed to drive. The trustworthy
  signal is the settle probe, later.

`[..] 0/10` to the trigger took 6 min 28 s on 2026-08-22.

### Terminal 2 — the recorder

The moment terminal 1 prints

```
      holding 180s before driving so a recorder can attach
```

run, in a second terminal:

```bash
cd ~/ms_kim/riscv-vela/vela-apps/ras/gz-split
OVERLAY=1 RVIZ=1 ROBOT=tb3 GEOMETRY=1920x1080 \
  scripts/run_demo.sh --record --duration 1500 2>&1 | tee logs/p4-take11-record.log
```

Do not wait. The viewer needs about 90 s to reach ffmpeg (container, Xvfb, GUI
shader compilation under llvmpipe, RViz, overlay, window tiling) and the hold is
180 s. Starting late means the drive begins before the recording does.

`OVERLAY=1` is separate from `RVIZ=1`. With `RVIZ=1` alone the detector's window
never opens and nothing in the recording shows the detector, even though it is
inferring.

`--duration 1500`, not 420: 30 settle probes can take up to ~12 minutes before
the tour starts. The tail is trimmed in post — see section 5.

Expected output:

```
[17:21:26] checking a server is publishing on /world/default/stats
[17:21:31] starting the viewer container (vela-gzsim:demo, host netns so gz-transport reaches the server)
[17:21:31]       Fuel cache /home/cslab/ms_kim/gz-fuel-cache -> /home/vela/.gz/fuel (ro)
[17:21:34] Xvfb :99 (1920x1080)
[17:21:38] gz sim -g on that display
[17:22:01]       GUI up (60s of shader compilation is normal here)
[17:22:01] rviz2 on the same display (/work/config/demo_nav.rviz)
[17:22:08]       /scan reaches the viewer (1.253 Hz)
[17:22:14]       detection_overlay.py -> /detection_image + a window
[17:22:15]       the detection window is on the display
[17:22:35]       RViz up
[17:22:58]       /map arrived from the guest (111 x 102)
[17:23:00]       'Gazebo Sim' -> 1056x1080+0+0
[17:23:00]       'RViz' -> 864x1080+1056+0
[17:23:01]       'YOLO detections' -> 640x480+16+584
[17:23:07] following tb3, camera offset (-1.0, 0, 0.45) m
[17:23:17] recording to /home/.../artifacts/demo-tb3-20260822-172125.mp4
[17:23:17] x11vnc on 127.0.0.1:5900
[17:23:21] recording for 1500s, then stopping on its own
```

Trigger to ffmpeg was 1 min 56 s (17:21:25 to 17:23:21), inside the 180 s hold
with 64 s to spare. `/map arrived ... (111 x 102)` is the map as it stood then;
it had grown to 112 x 132 by the time `map_saver` wrote it.

Three of those lines are the recording's own preconditions:

- **`'YOLO detections' -> 640x480+16+584`** — the PiP window exists and was
  placed. If it is missing the detector is invisible for the whole take.
- **`/map arrived from the guest`** — RViz has something to draw. `WARNING: no
  /map in 60s` means the pane stays empty grey; Takes 9 and 10 both hit it.
- **`camera offset (-1.0, 0, 0.45) m`** — the correct per-robot value.

### Terminal 1, continued — the verdict

```
[wpsettle-11] finished in 2s status=4 (SUCCEEDED)
settle: bt_navigator healthy after 11 probe(s)      <-- the take is alive
[wp1] goal world (-0.55, -0.55) -> map (1.45, -0.05)
[wp1] finished in 20s status=4 (SUCCEEDED)
[wp1] final (1.22, -0.02) error 0.229 m -> ok
... wp2 20s ok, wp3 22s ok, wp4 18s ok ...
[wp5] finished in 90s status=6 (not SUCCEEDED) -> MISSED
... wp6, wp7, wp8 MISSED ...
RESULT: FAIL (4/8 waypoints reached)
[..] 9/10  guest: save and score the SLAM map
  precision      0.645   >= 0.80  LOW
  recall         0.937   >= 0.85  ok
  coverage       0.880   >= 0.90  LOW
[..] 10/10 co-tenant load (G6) and optimizer stalls (G7)
workload total 287% of 400% available, 8 processes
[..] real_time_factor during the run = 0.24999511259554877
[..] G-Y5: 23 detection(s) during the run
[..] RESULT: FAIL (G5, tb3)  nav_rc=1 map_rc=1 yolo_rc=0 lat_rc=0
[..] HOLD=1 -- the stack stays up so a recording can outlive the verdict.
```

Two lines to read carefully.

`settle: bt_navigator healthy after N probe(s)` is the only trustworthy
predictor that the robot will move. `settle: NOT healthy after 30 probes` means
the take is already lost — stop and do not spend the recording.

Check the coordinate sign on the first goal, one line, once:
`goal world (a, b) -> map (c, d)` must satisfy `c = a + 2.0` and `d = b + 0.5`,
because `send_goal.py` subtracts the spawn offset `-2,-0.5`. Above,
`(-0.55, -0.55) -> (1.45, -0.05)` is correct. A sign error moves everything by a
whole quadrant and still looks plausible.

`real_time_factor during the run` sets the post-processing factor. If it is not
close to 0.25, section 5's `setpts` value is wrong for this take.

---

## 4. What to watch on screen

Over VNC, or on the finished file. Four things, in the order they can go wrong.

| When | Watch for | If it is wrong |
|---|---|---|
| First ~80 s of the recording | The PiP is a flat grey placeholder. **This is normal** — `detection_overlay.py` creates the window before any detection exists so `xdotool` has something to place | Nothing. Trim it in post |
| First detection, ~80-120 s in | The PiP switches to the **robot's camera view**: floor, pillars, a person. Boxes and labels appear | A photograph of a bus or a footballer means `YOLO_CAM=scene`. Stop the take; that is the 2026-08-19 mistake |
| Throughout the drive | **The PiP scene matches what the Gazebo pane shows.** Same people, same pillars, same direction of travel | This is the one that broke the last video. If the robot faces a wall and the PiP shows something else, stop — nothing else in the take can compensate |
| Last 10 s | The final frames are live, not a still | A frozen frame means the Gazebo server died before ffmpeg stopped. `HOLD=1` prevents it; check it was actually set |

Measured on 2026-08-22: the PiP refreshed at 17:27:01, 17:27:41 and 17:28:20 —
about every 40 s, which is the camera's 0.1 Hz of sim time at RTF 0.25, not the
detector's speed. One inference took 17.7-36.3 s. Three refreshes inside an
81 s drive.

---

## 5. Post-processing

### 5.1 Find t=0

`-ss` offsets are relative to the start of the mp4, and the log is in wall clock,
so anchor them:

```bash
SRC=artifacts/demo-tb3-<STAMP>.mp4
ffprobe -v error -show_entries format=duration -of csv=p=0 "$SRC"
grep -E 'recording to|closing the recording' logs/p4-take11-record.log
```

Expected:

```
1507.733333
[17:23:17] recording to /home/.../artifacts/demo-tb3-20260822-172125.mp4
[17:48:25] closing the recording
```

`t=0 = 17:48:25 - 1507.7 s = 17:23:17`, which matches the `recording to` line —
that agreement is the check. If the two disagree, trust the subtraction.

### 5.2 Choose the window

Keep the part where the robot drives. The criteria, in priority order:

1. **Starts after the grey placeholder.** The first PiP refresh must be inside
   the window, so start a little before it.
2. **Contains at least three PiP refreshes** — one box that appears and never
   changes looks like a still image. Refreshes are the `[detect]` lines in the
   guest's `~/yolo-node-<STAMP>.log`; convert with the offset from 5.4.
3. **Ends before the robot stops making progress.** Watch the `remaining=` field
   in the `[wpN] t+ Ns` lines: while it falls the robot is driving, and when it
   starts *rising* the robot is being pushed off course or is wedged.
4. **Cut nothing in the middle.** One continuous window, speed change only.
   A montage cannot be checked against the log.

For Take 11 that gave 17:26:40 to 17:29:00, i.e. `-ss 202.7 -t 140`:

| Wall clock | Offset | Event |
|---|---|---|
| 17:26:40 | t+202.7 | window opens, late settle probes |
| 17:27:01 | t+223.7 | PiP refresh, `person:0.88 x2` |
| 17:27:10 | t+232.7 | wp1 goal sent — driving starts |
| 17:27:41 | t+263.7 | PiP refresh, `person:0.92, person:0.89` |
| 17:27:50 | t+272.7 | wp2 reached |
| 17:28:20 | t+302.7 | PiP refresh, `person:0.84` |
| 17:28:30 | t+312.7 | wp4 reached — last success |
| 17:29:00 | t+342.7 | window closes, before the wedge is obvious |

### 5.3 Encode

```bash
ffmpeg -ss 202.7 -t 140 -i "$SRC" \
  -filter:v "setpts=0.25*PTS" -an \
  -c:v libx264 -crf 20 -pix_fmt yuv420p -movflags +faststart \
  logs/take11-circuit-slalom-4of8-yolo-nav2-slam-riscv.mp4

ffprobe -v error -show_entries format=duration,size \
  -show_entries stream=width,height,r_frame_rate,nb_frames \
  -of default=noprint_wrappers=1 logs/take11-circuit-slalom-4of8-yolo-nav2-slam-riscv.mp4
```

Expected:

```
width=1920
height=1080
r_frame_rate=15/1
nb_frames=527
duration=35.133333
size=4373099
```

**`0.25` is not a style choice.** The factor has to undo the `RTF_LIMIT` the take
was filmed at, so it comes from the `real_time_factor during the run` line
(`0.24999511`). At any other value the video misstates how fast the robot moved.
140 s of wall clock becomes 35 s at simulated real speed.

`-ss` before `-i` is frame-accurate when transcoding — ffmpeg seeks to the
keyframe and discards up to the requested time. It matters here: x264 at
`veryfast` uses a 250-frame keyint, which is 16.7 s at 15 fps.

### 5.4 Check the result

`--record` has no black-frame or empty-frame detection, so a person has to look.

```bash
OUT=logs/take11-circuit-slalom-4of8-yolo-nav2-slam-riscv.mp4
for t in 0.2 12 24 34.6; do
  ffmpeg -v error -y -ss $t -i "$OUT" -frames:v 1 /tmp/check-$t.png
  python3 -c "
import subprocess
raw = subprocess.run(['ffmpeg','-loglevel','error','-i','/tmp/check-$t.png',
                      '-f','rawvideo','-pix_fmt','rgb24','-'], capture_output=True).stdout
px = {raw[i:i+3] for i in range(0, len(raw), 3)}
print('$t s:', len(px), 'colours', '-- rendered' if len(px) > 100 else '-- BLANK')"
done
```

Expected: four lines around 26,000-27,000 colours. Under about 100 means the
scene did not draw. Then open the four PNGs and confirm, on each, that the PiP
scene is the same scene as the Gazebo pane.

To convert a `[detect]` line's `mono=` to wall clock, measure the offset while
the guest is still up:

```bash
date +%s.%N; ssh -p 2222 -o BatchMode=yes vela@localhost 'cut -d" " -f1 /proc/uptime'; date +%s.%N
```

`host_epoch = guest_mono + (midpoint of the two host readings - guest_mono)`.
On 2026-08-22 the offset was `1787384904.3` and the ssh round trip 1.8 s.

---

## 6. Troubleshooting

| Symptom | Check | Cause | Fix |
|---|---|---|---|
| Gazebo pane shows the arena from outside, no robot in it | `grep 'camera offset' logs/*-record.log` — expect `(-1.0, 0, 0.45)` | `FOLLOW_OFFSET_OVERRIDE=-3.0,0,2.0`. Three metres behind the robot is outside a 4.5 m walled arena and the wall occludes it. Take 2 | Do not set `FOLLOW_OFFSET_OVERRIDE`. `robot.env`'s per-robot value is already tuned for this world |
| Recording continues over a frozen scene after the verdict | `grep 'HOLD=1' logs/p4-*.log` | `run_split.sh`'s EXIT trap kills the Gazebo **server** when the verdict prints; `run_demo.sh` watches the **viewer's** GUI, which is still alive | `HOLD=1` on terminal 1. Default is 0 and there is no warning |
| mp4 looks fine in `ls` but no player opens it | `ffprobe <file>` errors; `pgrep -f 'run_demo.sh --record'` still running | `+faststart` moves the index to the front only when ffmpeg **finishes**. Mid-write there is no moov atom | Wait for `closing the recording` in the record log. Never `SIGKILL` ffmpeg — Ctrl-C on `run_demo.sh` is the clean path |
| Take looks perfect, PiP shows a bus or a footballer | `grep 'RGB camera' logs/p4-*.log` | `YOLO_CAM=scene` stands `bus.jpg`/`zidane.jpg` up as billboards outside the arena and films those. The 2026-08-19 video was this. The pipeline was working; the scene was wrong | Pass `YOLO_CAM=robot` explicitly, even though it is the default |
| PiP shows photographs sliding past, unrelated to the robot | `ssh -p 2222 vela@localhost 'pgrep -af "[i]mage_publisher"'` | `image_publisher.py` is a JPEG-to-topic stand-in that no gate runs, but `install_guest.sh` installs it. Running, it overwrites `/camera/image_raw` | Kill it. Check for it in 1.7 before every take |
| Arena renders, people do not, nothing logs an error | `docker run --rm -v ~/ms_kim/gz-fuel-cache:/home/vela/.gz/fuel:ro vela-gzsim:demo bash -lc 'ls "/home/vela/.gz/fuel/fuel.gazebosim.org/openrobotics/models/standing person/3/meshes/standing.dae"'` | `artifacts/people/*.sdf` carry absolute mesh URIs. `vela-gz` holds that Fuel cache in its writable layer; the viewer image has no `~/.gz` | Seed `~/ms_kim/gz-fuel-cache` per 1.2. Both scripts now refuse to start without it |
| Robot never moves, every `[wpN] final (0.00, -0.00)`, legs end in 4-26 s | `grep -E 'wpsettle|settle:' logs/p4-*.log` | `bt_navigator` received its first goal while behind its own TF buffer. The step-7 clock gate does not catch this — it measures global clock lag, a different quantity | Raise `SETTLE`. Take 10's 12 was not enough; 30 converged on probe 11. Abandon a take that prints `settle: NOT healthy` |
| `[wpsettle-N] goal rejected`, each taking exactly ~60 s, no `finished in Ns status=` lines | `ssh ... 'ros2 lifecycle get /bt_navigator'`; `ros2 action list \| grep navigate_to_pose` | The action server is not accepting goals at all — a different fault from the one above. Take 9 | Raising `SETTLE` will not help. Clear SHM (`sem.fastrtps_*` included) and reboot the guest, which needs the console terminal from section 2 |
| RViz pane stays empty grey for the whole take | `grep 'no /map in 60s' logs/*-record.log` | The guest has no 42103 locator, so `/map` never reaches the viewer; or `/map` is subscribed VOLATILE against a TRANSIENT_LOCAL publisher, which fails silently | `GUI=1` on terminal 1 generates `guest_tcp_client_demo.xml` with 42103-42105. `demo_nav.rviz` already subscribes TRANSIENT_LOCAL |
| Detector inferring, nothing about it on screen | `grep "'YOLO detections'" logs/*-record.log` | `RVIZ=1` alone does not start the overlay; RViz has no display for `vision_msgs/Detection2DArray`, so a working detector looks exactly like a dead one | `OVERLAY=1` as well as `RVIZ=1` |
| Robot drives the first legs, then wedges and `remaining=` rises | `grep -E '\[wp[0-9]\] final' logs/p4-*.log` — watch the error column grow | This take's failure. Errors ran 0.229, 0.232, 0.226, **0.491** and the next leg needs a 0.80 m gap with 0.18 m clearance each side | Not solved. Open question — see the note at the top of this file |
| `precision` below 0.80 on a map that looks right | `grep -A3 'slam occupied' logs/p4-*.log` | The person models are real obstacles the reference map does not contain, so SLAM marking them is scored as false positives. Take 11: `slam occupied 726` vs `ref occupied 494` | Report it, do not lower the floor. `MIN_PRECISION_OVERRIDE` exists but needs a stated reason |

---

## 7. Teardown

`HOLD=1` leaves the whole stack running on purpose. Release it when the
recording is safely written:

```bash
grep 'closing the recording' logs/p4-take11-record.log   # confirm ffmpeg finished first
touch /tmp/run_split.release
```

Expected: terminal 1 prints its teardown and exits. `run_split.sh`'s trap stops
the guest nodes, the bridge, the decimator, the watchdog and the Gazebo server.

`run_demo.sh` tears its own half down when `--duration` expires:

```
[..] closing the recording
[..] wrote /home/.../artifacts/demo-tb3-<STAMP>.mp4 (36M)
[..] tearing down the viewer
```

Then confirm nothing is left:

```bash
docker ps -a --filter name=vela-gz-view --format '{{.Names}} {{.Status}}'
docker exec vela-gz bash -lc "pgrep -af '[g]z sim'"
ls -d /tmp/run_demo.vela-gz-view.lock /tmp/run_split.release 2>/dev/null
ssh -p 2222 -o BatchMode=yes vela@localhost \
  'pgrep -af "[y]olo_node|[s]lam_toolbox|[b]t_navigator"'
docker ps --filter name=vela-gz --format '{{.Names}} {{.Status}}'
```

Expected: the first four produce no output; the last still shows `vela-gz Up`.
That container is deliberately long-lived — it carries the Fuel cache and the
measured image.

A `--standalone` viewer run also restores the headless world on exit
(`world : args ['headless:=true'] -> SceneBroadcaster off`), which a following
measurement run needs. A `GUI=1` run of `run_split.sh` rebuilds the world itself,
so this only matters if you used `--standalone`.

Leave the guest VM up if another take is coming: bringup is 6-7 minutes of the
cycle, and the VM contributes nothing to it once it is running.

---

## 8. Outputs

| Path | What | Tracked? |
|---|---|---|
| `artifacts/demo-<robot>-<STAMP>.mp4` | the raw 1920x1080 15 fps capture, whole duration | no — `artifacts/` is rebuilt every run |
| `artifacts/demo-{server,gui,rviz,xvfb,vnc,ffmpeg}-<STAMP>.log` | per-process logs from the viewer half | no |
| `artifacts/people/person_{standing,walking}.sdf` | generated model files | no |
| `logs/p4-take<N>-<date>.log` | terminal 1 in full — the record of what the run did | no — `logs/` is gitignored, `RUNLOG.md` is the curated record |
| `logs/p4-take<N>-record.log` | terminal 2 in full | no |
| `logs/<name>.mp4` | the curated deliverable | no |
| guest `~/yolo-node-<STAMP>.log` | every `[detect]` line: `infer_s`, `rx`, `drop`, labels | guest-only — copy it off |
| guest `~/split-stack-<STAMP>.log`, `~/clocklag-<STAMP>.log` | guest bringup and clock lag | guest-only — copy it off |
| guest `~/maps/split-<STAMP>.{pgm,yaml}` | the saved SLAM map | guest-only |
| guest `~/track-split-<STAMP>.csv` | the driven trajectory, one row per sample | guest-only |

`STAMP` is `%Y%m%d-%H%M%S` and is generated per script, so terminal 1 and
terminal 2 carry different stamps in the same take. Terminal 2's stamp is the one
in the mp4 name.

Naming the deliverable:

```
<take>-<scenario>-<result>-<workloads>-<arch>.mp4
take11  -circuit-slalom  -4of8  -yolo-nav2-slam  -riscv.mp4
```

**Carry the result in the name.** `4of8` is there so this file cannot be mistaken
for a passing run; a take that meets every floor would be `8of8`. The
scenario-only name `gz-split-tb3-circuit-yolo-nav2-slam-riscv.mp4` is reserved
for a take that passes, and nothing has claimed it yet.

Everything under `logs/` and `artifacts/` is gitignored. Copy anything that has
to survive off the host — the guest rootfs is one 17 GB image file and `/tmp` is
cleared on reboot.

## See also

- [`README.md`](README.md) — the gates, the topology, the measured results
- [`RUNLOG.md`](RUNLOG.md) — run-by-run record. Stops at Take 4; Takes 5-11 are not in it yet
- [`../yolo/README.md`](../yolo/README.md) — the detector, and what the RAS daemon reads from it
- [`DESIGN.md`](DESIGN.md) — why the simulator is on the host
