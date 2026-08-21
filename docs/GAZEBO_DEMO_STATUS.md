# Gazebo 시연 리그 (SLAM + Nav2 + YOLO) — STATUS

작성 2026-08-22. **읽기 전용 조사. 이 조사 중 코드는 한 줄도 수정하지 않았습니다.**
저장소: `~/ms_kim/riscv-vela/vela-apps` (브랜치 `feat/ras-tools`)
대상 디렉터리: `ras/gz-split/`, `ras/yolo/`, `ras/nav2/`
ROS 2 에러 로깅 시스템은 별도 문서 → [`RAS_ROS_LOGGING_STATUS.md`](RAS_ROS_LOGGING_STATUS.md)

---

## 0. 먼저 알아야 할 사실 — 지금 재검증이 불가능합니다

이 리그가 대상으로 하는 RISC-V 게스트(`vela@localhost:2222`)가 **현재 떠 있지 않습니다.**

```
$ timeout 25 ssh -p 2222 -o BatchMode=yes vela@localhost 'uptime; uname -m'
ssh: connect to host localhost port 2222: Connection refused

$ pgrep -af "hostfwd.*2222"
(없음)

$ ps -eo pid,user,lstart,etime,args | grep [q]emu-system-riscv64
847792 root Fri Aug 21 22:17:10 2026 05:26:23 ./qemu/build/qemu-system-riscv64 -s -accel tcg
 -m 65536 -smp 8 -serial file:/home/hayong/q-vela-ubuntu/vm-console.log -display none
 -machine virt,wg=on -bios ./opensbi/.../fw_dynamic.bin
 -netdev user,id=net0,host=10.0.2.10,hostfwd=tcp::2221-:22
 -blockdev ...RISCV_VIRT_CODE.fd -blockdev ...RISCV_VIRT_VARS.fd
 -M virt,pflash0=pflash0,pflash1=pflash1,rpmi=true,reri=true,aia=aplic-imsic
 -kernel ./linux/arch/riscv/boot/Image
 -drive file=./ubuntu-24.04.4-preinstalled-server-riscv64.img,format=raw,if=virtio
```

읽어낼 수 있는 것 두 가지:

1. 제 리그가 쓰던 **Config A 게스트(4 vCPU / 16 GB / hostfwd 2222)는 사라졌습니다.** 따라서 이 문서의
   게스트측 수치는 전부 **2026-08-17 ~ 08-20에 측정된 기록**이고, 오늘 재현한 것이 아닙니다.
2. 지금 도는 riscv64 VM은 다른 계정(`/home/hayong/`) 소유이고 **`reri=true` + EDK2 pflash 쌍 + Ubuntu
   preinstalled 이미지 + hostfwd 2221**입니다. 즉 다른 스레드에 넘겼던 **Config C에 해당하는 VM이
   누군가에 의해 이미 떠 있습니다.** 제 스크립트 기본값(포트 2222)과 포트가 다릅니다. 저는 이 VM을
   건드리지 않았습니다.

오늘 이 호스트에서 실제로 재실행해 성공을 확인한 것은 **호스트 x86에서 도는 것뿐**입니다
(§9의 "오늘 실행" 표).

---

## 1) 파일 전체 목록

### `git status`

```
$ git status
On branch feat/ras-tools
Your branch is up to date with 'fork/feat/ras-tools'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   ras/README.md

Untracked files:
  (use "git add <file>..." to include in what will be committed)
	ras/gz-split/
	ras/nav2/
	ras/ras-ros/
	ras/workload-nav2.md
	ras/yolo/

no changes added to commit (use "git add" and/or "git commit -a")
```

**커밋된 것은 아직 하나도 없습니다.** 커밋·푸시·PR은 사용자 몫이라는 규칙에 따라 손대지 않았습니다.

### `git log --oneline -20`

```
$ git log --oneline -20
ac7fe69 ras: add reri-inject and the TC.10.007 injection scripts
920a597 ras: install rasdaemon as a service with a RISC-V CPER decoder
bd82b7d docs: add the RAS Getting Started
ab1cfb2 Update README to remove related repositories section
74a187b Update README to specify enclave isolation method
caa8bb7 Enhance README with project details and instructions
2b52705 Update README.md
1df5f4a Clean up README.md by removing unnecessary content
4e6ebf4 Enhance README with project details and instructions
5308ad2 Fix table formatting in README.md
696c27a Update examples table in README.md
c095815 add components for TEE
869888c add reri register generator
d6cff44 Initial commit
```

위 3개(`ac7fe69`, `920a597`, `bd82b7d`)는 제가 초안을 쓰고 사용자가 커밋한 것입니다
(`author: Minseong Kim <ms.kim@chungbuk.ac.kr>`, 2026-08-09). 그 아래는 ETRI 강하영님 것입니다.
**이 문서가 다루는 gz-split / yolo / nav2 는 전부 커밋 이전의 untracked 상태입니다.**

### 커밋되면 추적될 파일 (`git add -n`, 실제 add 안 함)

`gz-split` — 43개
```
ras/gz-split/.dockerignore              ras/gz-split/config/host_clock_server.xml
ras/gz-split/.gitignore                 ras/gz-split/config/host_cmdvel_server.xml
ras/gz-split/DESIGN.md                  ras/gz-split/config/host_overlay_server.xml
ras/gz-split/Dockerfile                 ras/gz-split/config/host_ras_server.xml
ras/gz-split/Dockerfile.demo            ras/gz-split/config/host_rviz_server.xml
ras/gz-split/README.md                  ras/gz-split/config/host_tcp_server.xml
ras/gz-split/RUNLOG.md                  ras/gz-split/config/mapper_params_tcg.yaml
ras/gz-split/config/bridge_split.yaml   ras/gz-split/config/nav2_split.yaml
ras/gz-split/config/bridge_tb4.yaml     ras/gz-split/scripts/00-preflight.sh
ras/gz-split/config/camera_bridge_fragment.yaml
ras/gz-split/config/demo.rviz           ras/gz-split/scripts/check_interop.sh
ras/gz-split/config/demo_gui.config     ras/gz-split/scripts/run_container_loop.sh
ras/gz-split/config/demo_nav.rviz       ras/gz-split/scripts/run_dds_check.sh
ras/gz-split/config/explore_split.yaml  ras/gz-split/scripts/run_demo.sh
ras/gz-split/config/guest_tcp_client.xml       ras/gz-split/scripts/run_rtf.sh
ras/gz-split/config/guest_tcp_client_demo.xml  ras/gz-split/scripts/run_split.sh
ras/gz-split/tools/clock_decimator.py   ras/gz-split/tools/rtt_probe.py
ras/gz-split/tools/clock_lag_probe.py   ras/gz-split/tools/score_map.py
ras/gz-split/tools/cmd_vel_watchdog.py  ras/gz-split/tools/send_goal.py
ras/gz-split/tools/latency_probe.py     ras/gz-split/tools/slam_nav.launch.py
ras/gz-split/tools/liveliness_watchdog.py      ras/gz-split/tools/type_hashes.py
ras/gz-split/tools/make_world.py
ras/gz-split/tools/pose_truth_probe.py
```

`yolo` — 21개 (**그중 2개가 `.pyc`, 아래 §7 참조**)
```
ras/yolo/README.md                      ras/yolo/tools/__pycache__/make_people.cpython-312.pyc
ras/yolo/scripts/00-preflight.sh        ras/yolo/tools/__pycache__/yolov8_onnx.cpython-312.pyc
ras/yolo/scripts/build-in-container.sh  ras/yolo/tools/check_yolo_node.py
ras/yolo/scripts/build_ort_wheel.sh     ras/yolo/tools/detect.py
ras/yolo/scripts/export_model.sh        ras/yolo/tools/detection_overlay.py
ras/yolo/scripts/install_guest.sh       ras/yolo/tools/image_publisher.py
ras/yolo/scripts/make_bag.sh            ras/yolo/tools/make_billboards.py
ras/yolo/scripts/run_detect.sh          ras/yolo/tools/make_camera_world.py
ras/yolo/scripts/run_node_check.sh      ras/yolo/tools/make_people.py
ras/yolo/scripts/run_yolo.sh            ras/yolo/tools/yolo_node.py
                                        ras/yolo/tools/yolov8_onnx.py
```

`nav2` — 12개
```
ras/nav2/00-preflight.sh        ras/nav2/check_navigate.py   ras/nav2/planner_only.launch.py
ras/nav2/01-install-phase1.sh   ras/nav2/check_plan.py       ras/nav2/planner_only.yaml
ras/nav2/02-install-phase3.sh   ras/nav2/loopback_nav.launch.py  ras/nav2/run_milestone_a.sh
ras/nav2/seed_pose.py           ras/nav2/loopback_nav.yaml       ras/nav2/run_milestone_b.sh
```

### 추적되지 않는 실행 산출물

`gz-split/.gitignore`가 `artifacts/`(498개)와 `logs/`(71개 — mp4 3개, png 3개 포함)를 제외합니다.
`RUNLOG.md`(약 2,100줄)가 그 기록의 정본이고 이것만 추적됩니다.

---

## 2) 실행 진입점 — 정확한 한 줄

작업 디렉터리는 `~/ms_kim/riscv-vela/vela-apps/ras/gz-split`입니다.

**측정용(헤드리스, 세 워크로드 전부 = 게이트 G-Y5):**
```bash
YOLO=1 WATCHDOG= RTF_LIMIT=0.25 ROBOT=tb3 scripts/run_split.sh
```

**시연용(화면·녹화, 2개 터미널):**
```bash
# 터미널 1 — 실행
YOLO=1 WATCHDOG= RTF_LIMIT=0.25 GUI=1 ROBOT=tb3 HOLD=1 PRE_DRIVE_SLEEP=180 \
  WAYPOINTS_OVERRIDE="1.83,0.08 -1.88,0.08 1.83,0.08 -1.88,0.08" scripts/run_split.sh
# 터미널 2 — 터미널 1이 "holding 180s before driving"을 찍는 순간
OVERLAY=1 RVIZ=1 ROBOT=tb3 GEOMETRY=1920x1080 scripts/run_demo.sh --record --duration 420
```

선행 게이트(각각 변수 하나씩 분리): `scripts/run_rtf.sh`(G1) → `scripts/run_container_loop.sh`(G2) →
`scripts/run_dds_check.sh`(G3/G4). 뷰어 이미지는 한 번만: `docker build -f Dockerfile.demo -t vela-gzsim:demo .`

`nav2/`(게스트 단독 Nav2 검증)와 `yolo/`(검출기 단독)는 별개 진입점입니다:
```bash
scripts/run_milestone_b.sh     # ras/nav2 — 정적 맵 경로계획
scripts/run_milestone_a.sh     # ras/nav2 — NavigateToPose 종단
scripts/run_node_check.sh      # ras/yolo — 검출기 노드 단독 게이트 (G-Y6)
```

---

## 3) 구독하는 ROS 2 토픽

QoS 열은 **제가 코드에 명시한 값**입니다. "기본"은 rclpy 기본(RELIABLE / VOLATILE / KEEP_LAST)이고,
"상류"는 Nav2·slam_toolbox·ros_gz_bridge가 정하는 값으로 **제가 설정하지 않았습니다.**

### 호스트 x86-64 (컨테이너 `vela-gz` / 뷰어 `vela-gz-view`)

| 노드 | 토픽 | 타입 | QoS (reliability / durability / depth) | 위치 |
|---|---|---|---|---|
| `parameter_bridge` | `/cmd_vel` | `geometry_msgs/Twist` | 상류(ros_gz_bridge 기본) | `config/bridge_split.yaml` |
| `clock_decimator.py` | `/clock_gz` | `rosgraph_msgs/Clock` | BEST_EFFORT / VOLATILE / 1 | `tools/clock_decimator.py:68-72` |
| `cmd_vel_watchdog.py` | `/cmd_vel` | `geometry_msgs/Twist` | 기본 / 기본 / 10 | `tools/cmd_vel_watchdog.py:51` |
| `rviz2` (`demo.rviz`) | `/map` | `nav_msgs/OccupancyGrid` | Reliable / **Transient Local** / 1 | `config/demo.rviz` |
| `rviz2` | `/scan` | `sensor_msgs/LaserScan` | Best Effort / Volatile / 5 | `config/demo.rviz` |
| `rviz2` | `/plan` | `nav_msgs/Path` | Reliable / Volatile / 1 | `config/demo.rviz` |
| `rviz2` | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | 상류(TF 디스플레이 기본) | `config/demo.rviz` |
| `rviz2` | `/detection_image` | `sensor_msgs/Image` | Reliable / Volatile / 2 | `config/demo.rviz` |
| `detection_overlay.py` | `/camera/image_raw` | `sensor_msgs/Image` | RELIABLE / VOLATILE / 10 | `yolo/tools/detection_overlay.py:93-96` |
| `detection_overlay.py` | `/detection_result` | `vision_msgs/Detection2DArray` | RELIABLE / VOLATILE / 10 | `yolo/tools/detection_overlay.py:93-97` |

`/map`의 **Transient Local은 필수**입니다. slam_toolbox의 퍼블리셔가 TRANSIENT_LOCAL + RELIABLE이라
Volatile 구독자는 매칭되지 않고, 화면은 **에러 없이 그냥 비어 있습니다.**

### RISC-V 게스트 (QEMU q-vela, ssh 2222)

| 노드 | 토픽 | 타입 | QoS | 위치 |
|---|---|---|---|---|
| `async_slam_toolbox_node` | `/scan`, `/tf`, `/tf_static` | `LaserScan`, `TFMessage` | 상류(slam_toolbox) | `config/mapper_params_tcg.yaml` |
| `planner_server` / `controller_server` / `behavior_server` / `bt_navigator` | `/scan`, `/odom`, `/tf`, `/map` 등 | Nav2 표준 | 상류(Nav2 1.3.11) | `config/nav2_split.yaml` |
| `send_goal.py` | `map` | `nav_msgs/OccupancyGrid` | RELIABLE / **TRANSIENT_LOCAL** / 1 | `tools/send_goal.py:68-73` |
| `liveliness_watchdog.py` | `/scan` | `LaserScan` | `qos_profile_sensor_data`, `raw=True` | `tools/liveliness_watchdog.py:158-180` |
| `liveliness_watchdog.py` | `/odom` | `Odometry` | `qos_profile_sensor_data`, `raw=True` | 같음 |
| `liveliness_watchdog.py` | `/cmd_vel` | `Twist` | 기본 / 기본 / 10, `raw=True` | 같음 |
| `liveliness_watchdog.py` | `/map` | `OccupancyGrid` | RELIABLE / TRANSIENT_LOCAL / 1, `raw=True` | `:121-125` |
| `clock_lag_probe.py` | `/clock` | `Clock` | BEST_EFFORT / VOLATILE / 1 | `tools/clock_lag_probe.py:80-84` |
| `yolo_node.py` | `/camera/image_raw` | `sensor_msgs/Image` | BEST_EFFORT / VOLATILE / **1** | `yolo/tools/yolo_node.py:141-145` |
| `explore_lite` (G5-E만) | `/map`, `/tf` | 상류 | 상류 | `config/explore_split.yaml` |

`raw=True`는 최적화가 아니라 **정정**입니다. 이 노드는 메시지 본문을 읽지 않는데
역직렬화가 에뮬레이트 코어의 93%를 먹어 Nav2를 굶겼습니다(08-17 실측).

액션 클라이언트: `send_goal.py` → `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`).

---

## 4) 발행하는 ROS 2 토픽

### 호스트 x86-64

| 노드 | 토픽 | 타입 | QoS | 위치 |
|---|---|---|---|---|
| `parameter_bridge` | `/clock`(런너가 `/clock_gz`로 리네임) | `rosgraph_msgs/Clock` | 상류 | `bridge_split.yaml` |
| `parameter_bridge` | `/scan` | `sensor_msgs/LaserScan` | 상류 | 같음 |
| `parameter_bridge` | `/odom` | `nav_msgs/Odometry` | 상류 | 같음 |
| `parameter_bridge` | `/tf` (GZ→ROS 단방향) | `tf2_msgs/TFMessage` | 상류 | 같음 |
| `parameter_bridge` | `/camera/image_raw` (`YOLO=1`) | `sensor_msgs/Image` | 상류 | `camera_bridge_fragment.yaml` |
| `clock_decimator.py` | `/clock` | `rosgraph_msgs/Clock` | BEST_EFFORT / VOLATILE / 1 | `clock_decimator.py:71` |
| `cmd_vel_watchdog.py` | `/cmd_vel` (0 Twist 데드맨) | `geometry_msgs/Twist` | 기본 / 기본 / 1 | `cmd_vel_watchdog.py:50` |
| `robot_state_publisher` (tb4만) | `/robot_description`, `/tf_static` | 상류 | 상류 | `run_split.sh:510` |
| `detection_overlay.py` | `/detection_image` | `sensor_msgs/Image` | RELIABLE / VOLATILE / 10 | `detection_overlay.py:98` |

`/tf`를 BIDIRECTIONAL로 두면 게스트의 `map→odom`이 Gazebo로 돌아가고 서로를 먹입니다 —
`bridge_split.yaml`에 그 이유가 적혀 있습니다.

### RISC-V 게스트

| 노드 | 토픽 | 타입 | QoS |
|---|---|---|---|
| `async_slam_toolbox_node` | `/map` | `nav_msgs/OccupancyGrid` | 상류: RELIABLE / TRANSIENT_LOCAL |
| `async_slam_toolbox_node` | `/tf` (`map→odom`) | `tf2_msgs/TFMessage` | 상류 |
| `planner_server` | `/plan` | `nav_msgs/Path` | 상류 |
| `controller_server` | `/cmd_vel` | `geometry_msgs/Twist` | 상류 |
| `yolo_node.py` | `/detection_result` | `vision_msgs/Detection2DArray` | 기본 / 기본 / **10** |
| `yolo_node.py` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RELIABLE / **TRANSIENT_LOCAL** / 1 |
| `liveliness_watchdog.py`, `clock_lag_probe.py`, `send_goal.py` | **발행 없음** (CSV/stdout만) | — | — |

`/diagnostics`가 TRANSIENT_LOCAL인 이유: 리그가 뜨는 데 80초가 걸리고 감시자는 항상 그 뒤에 붙는데,
VOLATILE이면 최초 하트비트까지 "아무것도 없음"이 보이고 그건 **죽은 검출기와 구분되지 않습니다.**

---

## 5) 어떤 머신을 가정했는가

**둘 다입니다. 의도적으로 쪼갰습니다.** 실기 보드(HiFive P550 EVB)는 **가정하지 않았습니다.**

| 실행 위치 | 무엇이 도는가 | 근거 |
|---|---|---|
| **호스트 x86-64** (docker `vela-gz`) | Gazebo Sim 8 서버, `parameter_bridge`, `clock_decimator.py`, `cmd_vel_watchdog.py`, tb4용 `robot_state_publisher` | `run_split.sh:223` `dex()  { docker exec "$CONTAINER" bash -lc ...; }` |
| **호스트 x86-64** (뷰어 `vela-gz-view`) | `gz sim -g`, `rviz2`, `detection_overlay.py`, `x11vnc`, `ffmpeg` | `run_demo.sh` `vexd()`; `Dockerfile.demo`의 `x11vnc ffmpeg xdotool ros-jazzy-rviz2` |
| **RISC-V 게스트** (QEMU TCG) | `slam_toolbox`, Nav2 4노드 + `lifecycle_manager`, `send_goal.py`, `liveliness_watchdog.py`, `clock_lag_probe.py`, `yolo_node.py`, `explore_lite` | `run_split.sh:228` `gsh()  { ssh -p "$PORT" ... "$GUEST" ...; }`, `GUEST=${GUEST:-vela@localhost}`, `PORT=${PORT:-2222}` |

인용 — `scripts/run_split.sh:2-6`:
```
# Gate G5 -- the deliverable. Gazebo on the host, Nav2 and slam_toolbox in the
# RISC-V guest, robot drives a waypoint tour, SLAM map saved and scored.
#
# This is the topology the TSP test-environment figure (p.16-17) prescribes:
# Gazebo and the visualisation stack on x86-64, SLAM and Nav2 on the RISC-V side.
```

인용 — `gz-split/DESIGN.md:12` (실기 보드가 아니라는 유일한 언급):
```
is that the RISC-V side is an emulated guest rather than a HiFive Premier P550.
```

**`ras/nav2/`는 게스트 단독**입니다(호스트 Gazebo 없음, loopback 기구학). `ras/yolo/`의
`yolo_node.py`는 게스트, `detection_overlay.py`는 호스트입니다 — 이유는 §9 참조.

---

## 6) 설정한 환경변수 전부

| 변수 | 값 | 어디서 | 왜 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | `42` (리그 전체) | `run_split.sh:231,236,238,258`, `run_demo.sh:295-300`, `run_dds_check.sh:55,63`, `yolo/scripts/run_yolo.sh:36,42` | 리그 전체가 한 도메인. 없으면 teleop·RViz가 안 붙음 |
| `ROS_DOMAIN_ID` | `42` | `nav2/run_milestone_{a,b}.sh:26,21` | 같은 도메인 |
| `ROS_DOMAIN_ID` | `97` | `yolo/tools/check_yolo_node.py:129` (`CHECK_DOMAIN`으로 덮어쓰기 가능) | 단독 게이트를 워크로드 그래프와 분리 |
| `RMW_IMPLEMENTATION` | **설정하지 않음** | — | 게스트에 `rmw_fastrtps_cpp` 하나뿐이라 선택지가 없음. `RUNLOG.md:143`에 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`로 실측 확인 기록 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 호스트: `/work/config/host_tcp_server.xml`(42100), `host_clock_server.xml`(42101), `host_cmdvel_server.xml`(42102), `host_rviz_server.xml`(42103), `host_overlay_server.xml`(42104), `host_ras_server.xml`(42105) | `run_split.sh:230-238`, `run_demo.sh:296,299` | SLIRP는 멀티캐스트를 전달하지 않음 → TCPv4. **호스트 참가자마다 리스닝 포트 하나**, 공유 불가 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 게스트: `$GUEST_DIR/guest_tcp_client.xml` 또는 `GUI=1`일 때 `guest_tcp_client_demo.xml` | `run_split.sh:255-258` | 측정에 쓰인 파일을 바이트 동일하게 유지하려고 데모용 사본을 생성 |
| `ROS_LOCALHOST_ONLY` | **unset** (명시적으로 해제) | `run_split.sh:231` 등 모든 ENV 문자열 | 설정돼 있으면 TCP 로케이터를 무시함 |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | **unset** (리그), `LOCALHOST` (nav2 단독) | 같음 / `run_milestone_{a,b}.sh:27,22` | 리그는 게스트↔호스트를 넘고, nav2 단독은 넘지 않음 |
| `RCUTILS_COLORIZED_OUTPUT` | `0` | `run_milestone_{a,b}.sh` | 로그 grep 가능하게 |
| `DISPLAY` | 헤드리스: 빈 값 `DISPLAY=` / 뷰어: `:99` (`DISPLAY_NUM`) | `run_split.sh:499`, `run_demo.sh:273` | `DISPLAY=`를 비우지 않으면 `gz sim -s`가 X를 잡으려 함 |
| `QT_QPA_PLATFORM` | `xcb` | `run_demo.sh:273,322,332` | Xvfb 위 GUI |
| `LD_LIBRARY_PATH`, `PYTHONPATH` | **설정하지 않음** | — | `/opt/ros/jazzy/setup.bash` 소싱만 |

`ssh ... bash -l -s`는 `~/.profile`만 읽고 게스트 `~/.profile`은 ROS를 소싱하지 않으므로
**모든 원격 ENV 문자열이 `source /opt/ros/jazzy/setup.bash`를 직접 포함합니다**(`run_split.sh:255-258`).

---

## 7) 하드코딩·샘플·더미·mock / fallback

### 요청받은 grep을 그대로 실행한 결과 (`ras/` 기준, `logs/`·`artifacts/`·`__pycache__` 제외)

전체 출력은 길어서, **실제로 더미 데이터·mock·fallback 로직인 것만** 아래 표로 분류했습니다.
나머지 히트는 전부 `sample`이 "표본/샘플링"이라는 뜻으로 쓰인 산문(주석·문서)이거나
`Ogre2GpuRays`의 라이다 `samples` 파라미터입니다.

**(a) 진짜 샘플 이미지 — YOLO 정확도 검증용 COCO 표준 사진**

| 파일:라인 | 내용 |
|---|---|
| `yolo/scripts/run_yolo.sh:61-64` | `docker cp .../assets/bus.jpg`, `zidane.jpg` → `--images` |
| `yolo/scripts/make_bag.sh:39-42` | 같음 (rosbag 생성용) |
| `yolo/scripts/install_guest.sh:33` | `bus.jpg`, `zidane.jpg`를 게스트로 복사 |
| `yolo/scripts/build_ort_wheel.sh:44,52` | 휠 스모크 테스트 `--source /art/assets/bus.jpg --expect person=3,bus=1` |
| `yolo/tools/detect.py:9,60` | `cv2.imread(str(path))` — 파일 기반 단발 추론 CLI |
| `yolo/tools/image_publisher.py:35` | `cv2.imread` — 파일을 `/camera/image_raw`로 재생 |
| `yolo/tools/make_camera_world.py:33`, `make_billboards.py:47` | `cv2.imread` — 사진을 Gazebo 빌보드 텍스처로 굽기 |

**중요:** `bus.jpg`/`zidane.jpg`는 **모델이 맞는지 검증하는 기준 사진**이고, G-Y5 시연 경로에는
쓰이지 않습니다. 시연에서 검출기가 보는 것은 Gazebo가 렌더한 로봇 카메라 프레임입니다
(`camera_bridge_fragment.yaml` → `/camera/image_raw`). 다만 **빌보드 모드(`YOLO_CAM=scene`)를
쓰면 이 사진들이 3D 월드 안의 간판으로 서 있고, 검출기는 그 간판을 봅니다** —
합성 장면이지 실사가 아니라는 점을 보고서에 반드시 써야 합니다.

**(b) placeholder / fallback 로직**

| 파일:라인 | 내용 | 성격 |
|---|---|---|
| `yolo/tools/detection_overlay.py:103,121,128` | `self.placeholder()` — 창을 첫 프레임 전에 회색 이미지로 생성 | 시각적 편의. 검출 결과를 위조하지 않음 |
| `gz-split/tools/pose_truth_probe.py:95-96` | truth 토픽이 없으면 `/odom`으로 대체하고 `'fallback'`을 1회 경고 | 프로브 전용 |
| `gz-split/Dockerfile:40` | `xvfb fallback rung` — ogre2 헤드리스 실패 시 xvfb-run | 빌드 경로 |
| `gz-split/scripts/run_rtf.sh:145`, `DESIGN.md:284` | RTF < 0.10이면 `bag fallback`, 결함 D-14 에스컬레이션 | **미구현 계획**. 실측 RTF 1.0000이라 발동 안 함 |
| `gz-split/README.md:191` | 같은 fallback 서술 | 문서 |

**(b-2) 테스트 더블 — 요청받은 grep 키워드에 걸리지 않는 것이 하나 있습니다**

`mock`·`dummy` 히트는 0건입니다. 그러나 `stub`이라는 이름으로 **onnxruntime 대체물이 존재합니다.**

| 파일:라인 | 내용 |
|---|---|
| `yolo/tools/check_yolo_node.py:58-72` | 문자열 상수 `STUB` — `Yolov8Onnx`와 같은 호출 시그니처를 가진 클래스를 임시 파일로 써서 `sys.path` 앞에 꽂음. `self.runtime = "stub"`, `time.sleep(1.0)` 후 고정 검출 2개(`person` 0.83, `bus` 0.47)를 반환. `STUB_FAIL_AFTER` 환경변수로 N번째 이후 `RuntimeError` 발생 |

이것은 **의도된 테스트 더블이고 시연 경로에는 절대 들어가지 않습니다.** 목적은
`yolo_node.py`의 *ROS 행위*(하트비트, `detect_mono`, `dropped` 증가, 추론 예외 시 ERROR 보고 후
노드 생존, 정수 픽셀이 `rosidl` C 변환기에서 SIGABRT를 내지 않는지)를 x86에서 초 단위로 검증하는
것입니다. 게스트에서 한 번 추론이 11–31초라 같은 계약을 게스트에서 주장하면 어서션당 수 분이 듭니다.

**다만 판정에 쓸 때 반드시 구분해야 합니다:** `run_node_check.sh`(G-Y6)가 PASS라는 것은
**모델이 맞다는 뜻이 아니고** 노드의 배관이 맞다는 뜻입니다. 모델 정확도는 별개 게이트
(`bus.jpg`/`zidane.jpg`에 대한 `--expect person=3,bus=1`)가 담당합니다.

**(c) 하드코딩 절대경로 — 전부 환경변수로 덮어쓸 수 있음**

| 파일:라인 | 값 | 덮어쓰기 |
|---|---|---|
| `gz-split/scripts/run_split.sh:29` | `/home/vela/gz_split` | `GUEST_DIR=` |
| `gz-split/scripts/run_split.sh:173` | `/home/vela/yolo` | `YOLO_DIR=` |
| `gz-split/scripts/run_dds_check.sh:32` | `/home/vela/gz_split` | `GUEST_DIR=` |
| `yolo/scripts/{run_yolo,make_bag,install_guest,run_detect}.sh` | `/home/vela/yolo` | `GUEST_DIR=` |
| `yolo/scripts/run_yolo.sh:40` | `/home/vela/gz_split/guest_tcp_client.xml` | **없음 — 고정** |
| `gz-split/scripts/run_split.sh:428-429` | `/home/vela/.gz/fuel/.../standing person/3`, `walking person/3` | **없음 — Fuel 캐시 경로 고정** |

컨테이너 내부 경로는 바인드 마운트라 고정이 정상입니다(`/work` = `gz-split/`):
`run_rtf.sh:28`, `run_demo.sh:328,436`, `run_container_loop.sh:187`, `make_world.py:360,525`.
`/tmp/yolo`(`run_yolo.sh`, `make_bag.sh`), `/tmp/yolo-node-check`(`run_node_check.sh:25`),
`/tmp/run_split.release`(`run_split.sh` `HOLD_FILE`)도 고정입니다.

**(d) 보고해야 할 위생 결함 1건 — 제 잘못**

`ras/yolo/`에 `.gitignore`가 없어서 `git add ras/yolo`가
`tools/__pycache__/make_people.cpython-312.pyc`와 `yolov8_onnx.cpython-312.pyc`를 **함께 스테이징합니다.**
`gz-split/.gitignore`에는 `__pycache__/`가 있는데 `yolo/`·`nav2/`·`ras-ros/`에는 없습니다.
읽기 전용 조사이므로 **고치지 않았습니다.** 커밋 전에 `.gitignore` 추가나 `git reset` 필요.

---

## 8) 인터페이스를 문서로 합의한 적이 있는가

**부분적으로 있습니다.** 구분해서 답합니다.

| 상대 | 합의 문서 | 성격 |
|---|---|---|
| Gazebo ↔ 게스트 (SLAM/Nav2) | `gz-split/config/bridge_split.yaml` | **저 혼자 정한 뒤 문서화.** 토픽 5개·타입·방향이 전부 여기 있고, 빠진 토픽(`/joint_states`, `/imu`)마다 왜 뺐는지 적혀 있음 |
| 호스트 ↔ 게스트 전송 | `gz-split/config/host_*.xml`, `guest_tcp_client*.xml` + `DESIGN.md §How ROS 2 crosses SLIRP` | 저 혼자 정함. 포트 42100–42105 배정이 6개 파일에 중복 기재돼 서로를 참조 |
| YOLO ↔ 녹화/오버레이 | `yolo/README.md`, `camera_bridge_fragment.yaml` | 저 혼자 정함 |
| **YOLO ↔ RAS 에러로깅 데몬** | `yolo/README.md` §"What the RAS error-logging daemon reads" | **사용자와 실제로 합의한 유일한 계약** (2026-08-19). 조인 키 = `CLOCK_MONOTONIC`, 양쪽 필드명 `detect_mono`; 생존 판정은 `/detection_result`가 아니라 `/diagnostics` 1 Hz 하트비트 (`/detection_result`의 실측 발행 간격 p50 = **39.98초** > TSP의 20초 탐지창) |
| 다른 스레드 / 다른 담당자 | **없음** | Config C, TC.10.007 계측 툴링은 인터페이스 합의 없이 넘겼습니다 |

**합의되지 않아 지금 어긋나 있는 것:** §0에서 확인했듯 도는 Config C VM은 **포트 2221**이고
제 스크립트 기본값은 **2222**입니다. 이건 문서로 합의한 적이 없습니다.

---

## 9) 미완성 / 미검증 항목

### 오늘(2026-08-22) 실제로 실행해 확인한 것

| 명령 | 결과 |
|---|---|
| `bash -n` × 27개 스크립트 | 27/27 OK |
| `python3 -c "ast.parse(...)"` × 29개 파이썬 | 29/29 OK |
| `docker ps -a` | `vela-gz` **Up 3 days** (측정 컨테이너 생존), `ort-build` Exited(0), `node-exporter` Exited |
| `ssh -p 2222 vela@localhost` | **Connection refused — 게스트 없음** |

**게스트가 필요한 것은 오늘 아무것도 재검증하지 못했습니다.**

### 기록으로만 존재하는 실측치 (2026-08-17 ~ 08-20)

`gz-split/README.md §Results`가 정본이고 원본 stdout은 `gz-split/logs/`에 있습니다.
검증 가능한 예 — `logs/gy5-attempt2.log` 마지막 부분:
```
  precision      0.992   >= 0.80  ok
  recall         0.990   >= 0.85  ok
  coverage       0.973   >= 0.90  ok
RESULT: PASS (precision 0.992, recall 0.990, coverage 0.973)
...
workload total 272% of 400% available, 8 processes
[12:36:59] real_time_factor during the run = 0.24999846875937884
[12:37:01] G-Y5: 9 detection(s) during the run
[12:37:01] RESULT: PASS (G5, tb3) -- robot drove to every waypoint in Gazebo and the
[12:37:01]         SLAM map built from that drive meets the precision/recall/coverage floors
[12:37:01]         PASS (G-Y5) -- and the detector kept inferring beside them
```

### 미완성 / 미검증 — 솔직하게

1. **`G5-L` (토픽 도착 간격 게이트)은 FAIL이고 고치지 않았습니다.** `/scan`이 1000 ms 예산에
   1557 ms를 기록했습니다. 단, 그 breach는 계측기 자신이 코어의 57%를 먹은 실행에서 나왔고
   같은 실행이 웨이포인트도 하나 놓쳤습니다 — 즉 **계측기가 관측을 바꾸고 있습니다.**
   `README.md §Results`에 FAIL로 보고돼 있습니다. `WATCHDOG=`로 끄면 나머지 게이트가 통과합니다.
2. **RERI 오류 주입은 이 리그로 불가능합니다.** 게스트가 Config A(`reri=true` 없음)입니다.
   따라서 이것은 TC.10.007의 **워크로드**이고 TC.10.007의 **결과가 아닙니다.**
3. **tb4 coverage 0.387.** TSP의 4차년 로봇으로 4/4 웨이포인트는 도달했지만 `depot.pgm`
   커버리지가 낮습니다 — 웨이포인트 투어가 창고의 일부만 지나기 때문이고, 탐색 모드로
   재실행하지 않았습니다.
4. **RTF ≠ 1의 절대 시간값은 시험 대상의 속성이 아닙니다.** `RTF_LIMIT=0.25`에서 나온
   초 단위 수치는 이 호스트·이 부하의 성질입니다(정정요청 D-16과 같은 논지).
5. **`bag fallback` 경로는 문서에만 있고 구현되지 않았습니다** (`run_rtf.sh:145`가 KILL로
   안내만 함). RTF가 1.0000이라 필요하지 않았습니다.
6. **`ras/nav2/`의 Milestone A/B는 게스트에서 실행돼야 하고, 지금 게스트가 없습니다.**
   `workload-nav2.md`에 기록된 통과 이력이 유일한 증거입니다.
7. **`yolo/` 위생 결함**: `.gitignore` 부재로 `.pyc` 2개가 커밋에 섞입니다(§7-d).
8. **다른 스레드로 넘긴 것**: Config C 구축, TC.10.007 계측 툴링
   (`availability_monitor.sh`, `downtime_tracker.py`, `reri-inject -T` CLOCK_MONOTONIC 패치).
   §0에서 확인된 대로 Config C VM은 이미 누군가 띄웠지만 **제 스크립트와 포트가 다릅니다.**
9. **커밋·푸시·PR은 하지 않았습니다** — 사용자 몫입니다.
