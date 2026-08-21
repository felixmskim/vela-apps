# gz-split 컴포넌트 상태 보고

조사일 **2026-08-22**. 읽기 전용 조사이며 코드는 한 줄도 바꾸지 않았다. 이 파일만 새로 만들었다.
모르는 항목은 "모름"이라고 적었고, 추측으로 채운 곳은 없다.

| | |
|---|---|
| 컴포넌트 | **gz-split** — 분리형 Gazebo + SLAM 리그 |
| 하는 일 | 시뮬레이터(Gazebo)는 x86-64 호스트 Docker, Nav2와 slam_toolbox는 RISC-V 게스트. 로봇이 실제로 주행하고 그 라이다로 지도를 만든다 |
| 저장소 | `riscv-vela/vela-apps` (독립 git 저장소), 브랜치 `feat/ras-tools`, 리모트 `fork` = `felixmskim/vela-apps` |
| 컴포넌트 루트 | `ras/gz-split/` |
| 설계 근거 | `ras/gz-split/DESIGN.md`, 실행 기록 `ras/gz-split/RUNLOG.md` |

---

## 1) 파일 목록

### 1.1 `git status` (실제 출력)

```
On branch feat/ras-tools
Your branch is up to date with 'fork/feat/ras-tools'.

Changes not staged for commit:
  (use "git add <file>..." to update what will be committed)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   ras/README.md

Untracked files:
  (use "git add <file>..." to include in what will be committed)
	docs/
	ras/gz-split/
	ras/nav2/
	ras/ras-ros/
	ras/workload-nav2.md
	ras/yolo/

no changes added to commit (use "git add" and/or "git commit -a")
```

### 1.2 `git log --oneline -20` (실제 출력)

```
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

### 1.3 내가 만든 파일 (43개 — `artifacts/`·`logs/`·`__pycache__` 제외, repo 기준 상대경로)

```
vela-apps/ras/gz-split/config/bridge_split.yaml
vela-apps/ras/gz-split/config/bridge_tb4.yaml
vela-apps/ras/gz-split/config/camera_bridge_fragment.yaml
vela-apps/ras/gz-split/config/demo_gui.config
vela-apps/ras/gz-split/config/demo_nav.rviz
vela-apps/ras/gz-split/config/demo.rviz
vela-apps/ras/gz-split/config/explore_split.yaml
vela-apps/ras/gz-split/config/guest_tcp_client_demo.xml
vela-apps/ras/gz-split/config/guest_tcp_client.xml
vela-apps/ras/gz-split/config/host_clock_server.xml
vela-apps/ras/gz-split/config/host_cmdvel_server.xml
vela-apps/ras/gz-split/config/host_overlay_server.xml
vela-apps/ras/gz-split/config/host_ras_server.xml
vela-apps/ras/gz-split/config/host_rviz_server.xml
vela-apps/ras/gz-split/config/host_tcp_server.xml
vela-apps/ras/gz-split/config/mapper_params_tcg.yaml
vela-apps/ras/gz-split/config/nav2_split.yaml
vela-apps/ras/gz-split/DESIGN.md
vela-apps/ras/gz-split/Dockerfile
vela-apps/ras/gz-split/Dockerfile.demo
vela-apps/ras/gz-split/.dockerignore
vela-apps/ras/gz-split/.gitignore
vela-apps/ras/gz-split/README.md
vela-apps/ras/gz-split/RUNLOG.md
vela-apps/ras/gz-split/scripts/00-preflight.sh
vela-apps/ras/gz-split/scripts/check_interop.sh
vela-apps/ras/gz-split/scripts/run_container_loop.sh
vela-apps/ras/gz-split/scripts/run_dds_check.sh
vela-apps/ras/gz-split/scripts/run_demo.sh
vela-apps/ras/gz-split/scripts/run_rtf.sh
vela-apps/ras/gz-split/scripts/run_split.sh
vela-apps/ras/gz-split/tools/clock_decimator.py
vela-apps/ras/gz-split/tools/clock_lag_probe.py
vela-apps/ras/gz-split/tools/cmd_vel_watchdog.py
vela-apps/ras/gz-split/tools/latency_probe.py
vela-apps/ras/gz-split/tools/liveliness_watchdog.py
vela-apps/ras/gz-split/tools/make_world.py
vela-apps/ras/gz-split/tools/pose_truth_probe.py
vela-apps/ras/gz-split/tools/rtt_probe.py
vela-apps/ras/gz-split/tools/score_map.py
vela-apps/ras/gz-split/tools/send_goal.py
vela-apps/ras/gz-split/tools/slam_nav.launch.py
vela-apps/ras/gz-split/tools/type_hashes.py
```

수정한 추적 파일은 1개다.

| 파일 | 변경 |
|---|---|
| `ras/README.md` | `## See also`에 4행 추가 (`git diff` 위 §1.1) — 그중 `workload-nav2.md`·`gz-split/` 2행이 이 작업선의 것이고, `yolo/`·`ras-ros/` 2행은 그 뒤에 추가되었다 |

### 1.4 소유 경계 (겹침 있음 — 사실만)

| 경로 | 판단 | 근거 |
|---|---|---|
| `ras/gz-split/**` | **이 스레드 산출물** | 계획 파일(`~/.claude/plans/`)이 이 디렉터리 전체를 산출물로 규정하고, `RUNLOG.md`가 실행 기록을 담고 있다 |
| `ras/nav2/`, `ras/workload-nav2.md` | 같은 작업선의 **선행 단계**(Milestone A/B). gz-split이 이것을 전제로 하고 `README.md` Prerequisites가 링크한다 | `workload-nav2.md:613,658`이 `gz-split/`을 "다음 단계"로 지목. 파일 mtime 2026-08-12 |
| — 그 두 개의 **작성 세션** | **모름** | 커밋되지 않은 파일이라 git 이력이 없고, 트랜스크립트의 경로 언급 횟수는 "읽음"과 "씀"을 구분하지 못한다 |
| `ras/yolo/`, `ras/ras-ros/` | **다른 세션 산출물**. 다만 내 스크립트가 이것을 소비한다 (§8) | `run_split.sh:417,425,450`이 `../yolo/tools/{detection_overlay,make_people,make_billboards}.py`를 `docker cp`로 가져온다 |
| `docs/YOLO_STATUS.md` | 같은 시각(03:48) 다른 세션이 작성. **그 문서는 `ras/gz-split/`을 자기 컴포넌트 범위에 포함시킨다** | 해당 파일 6행: "`ras/gz-split/`(Nav2/SLAM/Gazebo 분할 리그 및 녹화 경로). 두 개를 합쳐 'YOLO 파트'로 보고한다" |

---

## 2) 실행 진입점

```bash
cd ~/ms_kim/riscv-vela/vela-apps/ras/gz-split && WATCHDOG= RTF_LIMIT=0.5 ROBOT=tb3 scripts/run_split.sh
```

전제: (a) 호스트 컨테이너 `vela-gz`가 떠 있어야 하고, (b) 게스트 VM이 부팅되어 `ssh -p 2222 vela@localhost`가 되어야 한다. **현재 (b)는 충족되지 않는다** — §9 참조.

시연 녹화는 터미널 2개다.

```bash
# 터미널 1 — 런
WATCHDOG= RTF_LIMIT=0.5 GUI=1 ROBOT=tb3 PRE_DRIVE_SLEEP=120 scripts/run_split.sh
# 터미널 2 — 터미널 1이 "7/10"을 찍는 순간
RVIZ=1 ROBOT=tb3 scripts/run_demo.sh --record --duration 200
```

게이트별 다른 진입점: `scripts/run_rtf.sh`(G1) / `run_container_loop.sh`(G2) / `run_dds_check.sh`(G3·G4) / `run_split.sh`(G5~G7) / `run_demo.sh`(시연·녹화). 전체 절차는 `ras/gz-split/README.md` "Steps".

---

## 3) 구독하는 ROS 2 토픽

### 3.1 컴포넌트 경계 — 호스트→게스트로 넘기는 6토픽 (`config/bridge_split.yaml`)

`ros_gz_bridge parameter_bridge`가 Gazebo와 ROS 2를 잇는 지점이고, 게스트의 Nav2/slam_toolbox가 이것을 구독한다.

| gz 토픽 | ROS 토픽 | 메시지 타입 | 방향 | QoS |
|---|---|---|---|---|
| `/clock` | `/clock` → 런타임에 `/clock_gz`로 개명 | `rosgraph_msgs/msg/Clock` | GZ→ROS | 브리지 기본값 |
| `scan` | `/scan` | `sensor_msgs/msg/LaserScan` | GZ→ROS | 브리지 기본값 |
| `odom` | `/odom` | `nav_msgs/msg/Odometry` | GZ→ROS | 브리지 기본값 |
| `tf` | `/tf` | `tf2_msgs/msg/TFMessage` | GZ→ROS **only** | 브리지 기본값 |
| `cmd_vel` | `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS→GZ | 브리지 기본값 |
| `/camera` | `/camera/image_raw` | `sensor_msgs/msg/Image` | GZ→ROS | 브리지 기본값 (`YOLO=1`일 때만, `config/camera_bridge_fragment.yaml`) |

**QoS를 "브리지 기본값"이라고 쓴 이유**: `run_split.sh`의 `parameter_bridge` 호출(`scripts/run_split.sh:571-576`)은 `config_file`·`expand_gz_topic_names`·`use_sim_time`만 넘기고 QoS 오버라이드를 하지 않는다. 즉 `ros_gz_bridge`가 gz 측 설정을 따라 만드는 값이 그대로 쓰인다. **이번 조사에서 `ros2 topic info -v`로 실측하지 않았다** — 컨테이너가 유휴 상태(`sleep infinity` 1개)이고 게스트가 내려가 있어 라이브 그래프가 없다. 코드가 이 값에 대해 남긴 실측 근거는 `tools/pose_truth_probe.py:71-73` 주석 하나뿐이다: "브리지는 gz 측 설정으로 재발행하므로 BEST_EFFORT 발행자에 RELIABLE 구독을 걸면 아무것도 매칭되지 않는다".

### 3.2 내가 만든 노드가 직접 구독하는 것

| 노드 (파일:라인) | 실행 위치 | 토픽 | 타입 | reliability / durability / depth |
|---|---|---|---|---|
| `clock_decimator` (`tools/clock_decimator.py:72`) | 호스트 컨테이너 | `/clock_gz` (`--in`) | `rosgraph_msgs/Clock` | BEST_EFFORT / VOLATILE / KEEP_LAST 1 |
| `cmd_vel_watchdog` (`tools/cmd_vel_watchdog.py:51`) | 호스트 컨테이너 | `/cmd_vel` | `geometry_msgs/Twist` | rclpy 기본(RELIABLE / VOLATILE) / depth 10 |
| `liveliness_watchdog` (`tools/liveliness_watchdog.py:175`) | **게스트** | `/scan` | `sensor_msgs/LaserScan` | `qos_profile_sensor_data` (BEST_EFFORT / VOLATILE / 5), `raw=True` |
| 〃 | 게스트 | `/odom` | `nav_msgs/Odometry` | `qos_profile_sensor_data`, `raw=True` |
| 〃 | 게스트 | `/cmd_vel` | `geometry_msgs/Twist` | 기본 / depth 10, `raw=True` |
| 〃 | 게스트 | `/map` | `nav_msgs/OccupancyGrid` | RELIABLE / **TRANSIENT_LOCAL** / 1, `raw=True` |
| `send_goal` (`tools/send_goal.py:68-73`) | 게스트 | `map` | `nav_msgs/OccupancyGrid` | RELIABLE / **TRANSIENT_LOCAL** / KEEP_LAST 1 |
| 〃 (`:61`) | 게스트 | 액션 `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | rclpy 액션 기본 |
| 〃 | 게스트 | `/tf`, `/tf_static` (tf2 `TransformListener`) | `tf2_msgs/TFMessage` | tf2 기본 |
| `clock_lag_probe` (`tools/clock_lag_probe.py:108`) | 게스트 | `/clock` | `rosgraph_msgs/Clock` | BEST_EFFORT / VOLATILE / KEEP_LAST 1 |
| `pose_truth_probe` (`tools/pose_truth_probe.py:77-78`) | 호스트 컨테이너 | `/gz_truth`, `/odom` | `tf2_msgs/TFMessage`, `nav_msgs/Odometry` | BEST_EFFORT / VOLATILE / KEEP_LAST 10 |
| `latency_probe` (`tools/latency_probe.py:76-79`) | 지정한 쪽 | `--topic` (임의) | `--type`으로 지정 | `--sensor-qos`면 `qos_profile_sensor_data`, 아니면 기본/depth 10 |
| `rtt_probe` (`tools/rtt_probe.py:50,67`) | 양쪽 각 1개 | `rtt_ping` / `rtt_pong` | `std_msgs/Header` | RELIABLE / KEEP_LAST 50 |

---

## 4) 발행하는 ROS 2 토픽

| 노드 (파일:라인) | 실행 위치 | 토픽 | 타입 | QoS |
|---|---|---|---|---|
| `clock_decimator` (`tools/clock_decimator.py:71`) | 호스트 컨테이너 | `/clock` (`--out`, 기본 20 Hz로 감쇄) | `rosgraph_msgs/Clock` | BEST_EFFORT / VOLATILE / KEEP_LAST 1 |
| `cmd_vel_watchdog` (`tools/cmd_vel_watchdog.py:50`) | 호스트 컨테이너 | `/cmd_vel` (2.0 sim s 무입력 시 0 Twist) | `geometry_msgs/Twist` | 기본 / depth 1 |
| `rtt_probe` (`tools/rtt_probe.py:49,66`) | 양쪽 | `rtt_pong` / `rtt_ping` | `std_msgs/Header` | RELIABLE / KEEP_LAST 50 |
| `send_goal` | 게스트 | 액션 goal (토픽 발행 없음) | `NavigateToPose` | — |
| `static_transform_publisher` ×2 (`tools/slam_nav.launch.py:70-77`) | 게스트 | `/tf_static` (`base_footprint→base_link`, `base_link→base_scan`) | `tf2_msgs/TFMessage` | tf2 기본 (TRANSIENT_LOCAL) |

내가 만든 코드가 **발행하지 않는** 것도 명시해 둔다: `/map`은 slam_toolbox가, `map→odom` TF도 slam_toolbox가, `/odom`과 `odom→base_footprint`는 Gazebo의 DiffDrive 플러그인이 발행한다. `tools/slam_nav.launch.py`는 이들을 **띄우는 런치 파일**이고 스스로 발행하는 토픽은 위의 `/tf_static`뿐이다.

---

## 5) 실행 머신: **둘 다** — 분리형이 이 컴포넌트의 정체다

x86 호스트냐 RISC-V냐가 아니라, **프로세스별로 갈린다.** 그것이 이 컴포넌트가 존재하는 이유다.

| 프로세스 | 머신 | 근거 |
|---|---|---|
| `gz sim -s` (물리 + gpu_lidar) | **x86-64 호스트**, Docker `vela-gz` | `scripts/run_split.sh:499` `dexd "DISPLAY= gz sim -s -r -v2 --headless-rendering ..."` — `dexd()`는 `docker exec -d`(`:224`) |
| `ros_gz_bridge parameter_bridge` | 호스트 컨테이너 | `run_split.sh:571` `dexd "$HOST_ENV ros2 run ros_gz_bridge parameter_bridge ..."` |
| `clock_decimator.py` | 호스트 컨테이너 | `run_split.sh:583` `dexd "$CLOCK_ENV python3 /work/tools/clock_decimator.py ..."` |
| `cmd_vel_watchdog.py` | 호스트 컨테이너 | 같은 `dexd` 경로, 프로파일 `host_cmdvel_server.xml` (포트 42102) |
| `rviz2`, `detection_overlay.py` | 호스트 (별 컨테이너 `vela-gz-view`) | `run_demo.sh:322,332` `vexd "..."` = 뷰어 컨테이너 exec. `slam_nav.launch.py:21` "rviz2 — not packaged for riscv64" |
| **slam_toolbox** | **RISC-V 게스트** | `run_split.sh:230` 블록의 `gsh()` = `ssh -p "$PORT" ... "$GUEST"`(`:228`), `GUEST=${GUEST:-vela@localhost}`, `PORT=${PORT:-2222}` (`:26-27`). 런치는 `tools/slam_nav.launch.py:120` |
| **Nav2 5노드** (`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager`) | **RISC-V 게스트** | `slam_nav.launch.py:64-69` `LIFECYCLE` 목록 + `:124-131` |
| `send_goal.py`, `liveliness_watchdog.py`, `clock_lag_probe.py` | **RISC-V 게스트** | `run_split.sh`의 `gsh "$GUEST_ENV python3 $GUEST_DIR/..."` 호출들. `GUEST_DIR=${GUEST_DIR:-/home/vela/gz_split}` (`:29`) |

문서에 명시된 근거를 그대로 인용한다.

> `README.md:1-5` — "A Gazebo simulation whose robot is driven by Nav2 and mapped by slam_toolbox **inside** the RISC-V guest, with the simulator itself on the x86 host."

> `run_split.sh:5-6` — "This is the topology the TSP test-environment figure (p.16-17) prescribes: Gazebo and the visualisation stack on x86-64, SLAM and Nav2 on the RISC-V side."

게스트의 실체는 **q-vela QEMU VM**이며 **Config A**다 (RERI 없음).

> `run_split.sh:12-15` — "What this does NOT do: there is no RERI here. The guest is Config A — no `reri=true`, no EDK2 pflash pair — so this produces the co-tenant workload for TC.10.007, never a TC.10.007 result."

**HiFive P550 EVB는 이 컴포넌트의 대상이 아니다.** 실기 보드를 가정한 코드는 없다. `grep -rni "p550\|hifive"`는 1건이고 그것도 반대 방향의 명시다 — `DESIGN.md:12` "the RISC-V side is an emulated guest rather than a HiFive Premier P550".

---

## 6) 설정한 환경변수 전부

### 6.1 ROS 2 / DDS (통신 동작을 바꾸는 것)

| 변수 | 값 | 어디서 | 왜 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | **42** (`DOMAIN=${DOMAIN:-42}`) | 호스트 참가자 4종 + 게스트 (`run_split.sh:230-258`) | 호스트/게스트 양쪽 동일 도메인 |
| `RMW_IMPLEMENTATION` | **`rmw_fastrtps_cpp`** | `Dockerfile:89` (`ENV`) | 호스트 컨테이너. **게스트에는 설정하지 않는다** — Jazzy 기본값이 같아서 그대로 둔다 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 참가자마다 다른 XML | `run_split.sh:230,235,237,257`, `run_demo.sh:296,299`, `run_dds_check.sh:54,62` | **호스트 참가자 1개 = 리스닝 포트 1개**. 아래 표 |
| `ROS_LOCALHOST_ONLY` | **unset** | 위 모든 블록 | 설정되어 있으면 TCP 링크가 죽는다 |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | **unset** | 위 모든 블록 | 동일 |
| `ROS_SHARE` | (선택) `/opt/ros/jazzy/share` | `tools/type_hashes.py:32` | 타입 해시 비교용 |

포트 배정 (각 호스트 참가자마다 XML 1개, 게스트 `initialPeersList`에 locator 1개):

| 포트 | 참가자 | 프로파일 |
|---|---|---|
| 42100 | `parameter_bridge` | `config/host_tcp_server.xml` |
| 42101 | `clock_decimator.py` | `config/host_clock_server.xml` |
| 42102 | `cmd_vel_watchdog.py` | `config/host_cmdvel_server.xml` |
| 42103 | `rviz2` | `config/host_rviz_server.xml` |
| 42104 | `detection_overlay.py` | `config/host_overlay_server.xml` |
| 42105 | 사용자의 RAS 데몬용 배관 (미사용) | `config/host_ras_server.xml` |

게스트는 `config/guest_tcp_client.xml`(42100–42102)을 쓰고, `GUI=1`이면 `run_split.sh`가 42103–42105까지 담은 `guest_tcp_client_demo.xml`을 **생성**한다. 측정에 쓰인 파일을 바꾸지 않기 위해 생성 방식을 택했다 (`run_split.sh:246-256` 주석).

`RMW_FASTRTPS_PUBLICATION_MODE`와 `RMW_FASTRTPS_USE_QOS_FROM_XML`은 **어디에서도 설정하지 않는다.** 세 XML의 주석에만 등장하며(`host_rviz_server.xml:90-91`, `host_overlay_server.xml:94-95`, `host_ras_server.xml:103-104`), 내용은 "rmw_fastrtps는 `RMW_FASTRTPS_PUBLICATION_MODE`를 보고 XML `data_writer` 프로파일은 `RMW_FASTRTPS_USE_QOS_FROM_XML` 없이는 무시하는데 여기서는 아무것도 그것을 설정하지 않는다"는 사실 기록이다. 즉 **XML의 `data_writer` 비동기 설정은 현재 효력이 없다**(대칭성 유지 목적으로만 남아 있음).

### 6.2 렌더링 / 화면

| 변수 | 값 | 어디서 |
|---|---|---|
| `LIBGL_ALWAYS_SOFTWARE` | `1` | `Dockerfile:80` |
| `GALLIUM_DRIVER` | `llvmpipe` | `Dockerfile:81` |
| `GZ_SIM_RESOURCE_PATH` | `/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share` | `Dockerfile:88` |
| `DISPLAY` | 서버는 빈 값(`DISPLAY= gz sim -s`), 뷰어는 `:$DISPLAY_NUM`(기본 99) | `run_split.sh:499`, `run_demo.sh:264-274` |
| `QT_QPA_PLATFORM` | `xcb` | `run_demo.sh:273,322,332` |

### 6.3 호출자 노브

`run_split.sh` 30개 + `run_demo.sh` 20개(내부 변수 포함 총 77개 이름 참조). 통신·판정에 직접 영향을 주는 것만: `ROBOT`(tb3/tb4) · `RTF_LIMIT` · `WATCHDOG` · `GUI` · `YOLO`/`YOLO_CAM` · `EXPLORE` · `ODOM_HZ`(기본 10) · `CLOCK_HZ`(기본 20) · `CLOCK_LAG_MAX` · `SETTLE` · `PRE_DRIVE_SLEEP` · `WAYPOINTS_OVERRIDE` · `MIN_{COVERAGE,PRECISION,RECALL}_OVERRIDE` · `HOLD` · `RVIZ` · `OVERLAY`. 각각의 의미는 `README.md` 본문 표에 있다.

---

## 7) 하드코딩 절대경로 / 샘플 이미지 / 더미 데이터 / mock·fallback

### 7.1 요청한 grep의 실제 결과

```
$ grep -rn "bus.jpg\|zidane\|sample\|dummy\|mock\|fallback\|placeholder\|cv2.imread\|imread(" . \
    --exclude-dir=artifacts --exclude-dir=logs --exclude-dir=__pycache__ | wc -l
98
```

키워드별 건수:

| 키워드 | 건수 | 내용 |
|---|---|---|
| `bus.jpg` | **0** | — |
| `zidane` | **0** | — |
| `dummy` | **0** | — |
| `mock` | **0** | — |
| `cv2.imread` / `imread(` | **0** | 이 컴포넌트는 이미지를 읽지 않는다 (OpenCV 의존 없음) |
| `sample` | 89 | 전부 영어 단어. **DDS sample**(메시지 1건)과 **lidar samples**(360 빔) — 샘플 데이터가 아니다 |
| `fallback` | 8 | 아래 7.3 |
| `placeholder` | 1 | `run_demo.sh:342` 주석 — 오버레이 창이 시작 시 표시하는 초기 화면 설명 |

즉 **이 컴포넌트 안에는 더미 데이터도 mock도 샘플 이미지도 없다.** 로봇이 보는 것은 Gazebo가 실제로 렌더한 `gpu_lidar` 스캔이고, 지도는 그 스캔으로 slam_toolbox가 만든 것이다.

### 7.2 다만 — 샘플 이미지에 **닿는 경로가 하나 있다**

`YOLO=1 YOLO_CAM=scene`으로 돌리면 내 스크립트가 다른 컴포넌트의 샘플 이미지를 읽어 Gazebo 안 빌보드 텍스처로 쓴다.

| 파일:라인 | 내용 |
|---|---|
| `run_split.sh:172` | `YOLO_ASSETS=${YOLO_ASSETS:-$HOME/ms_kim/yolo-artifacts/assets}` |
| `run_split.sh:442-446` | `ls "$YOLO_ASSETS"/*.jpg` → 없으면 FAIL, 있으면 컨테이너로 `docker cp` |
| 그 디렉터리의 실제 내용 | `bus.jpg` (137 KB), `zidane.jpg` (50 KB) — **ultralytics 배포 샘플 이미지** |

**기본값에서는 이 경로를 타지 않는다**: `run_split.sh:171`이 `YOLO_CAM=${YOLO_CAM:-robot}`이므로 기본은 로봇 탑재 카메라이고, 이때 장면은 `../yolo/tools/make_people.py`가 Fuel의 person 모델을 배치해 만든다. 그러나 2026-08-19의 시연 영상 1편(`logs/gz-split-tb3-yolo-nav2-slam-riscv.mp4`)은 `scene` 모드로 찍힌 것이고, 즉 **그 영상에서 YOLO가 검출한 것은 아레나 밖 빌보드에 붙은 위 두 장의 샘플 사진이다.** 사용자가 2026-08-20에 지적한 지점이며, 그래서 로봇 카메라 + 사람 모델로 재작업이 진행 중이다(§9).

### 7.3 fallback / mock 성격의 로직 전수

| 파일:라인 | 무엇 | 판정 |
|---|---|---|
| `tools/pose_truth_probe.py:90-101` | `/gz_truth`에서 `--model` 이름을 못 찾을 때, 엔트리가 **정확히 1개**면 그것을 진값으로 쓴다 | 실제 fallback. 다만 조용하지 않다 — `get_logger().warn`으로 어떤 프레임을 대신 썼는지 1회 출력. G2/G5 판정에는 쓰이지 않는 진단 도구다 |
| `scripts/run_rtf.sh:145` | `RESULT: KILL (... RTF < k) - abandon live Gazebo for this robot, use bag fallback` | 코드가 아니라 **판정 문구**. bag 재생 대체안을 사람에게 권하고 종료한다. 자동으로 bag로 넘어가지 않는다 |
| `Dockerfile:40` | 주석 — "xvfb fallback rung: ogre2 without `--headless-rendering`, under `xvfb-run`" | 빌드 시 렌더 경로 대안 설명. 런타임 분기 아님 |
| `README.md:191,219` 외 | "bag fallback" 서술 | 문서 |
| `scripts/run_demo.sh:342` | "It exists from startup holding a placeholder, so it is checkable now." | 오버레이 창 준비 판정 근거 주석 |

**측정값을 만들어내는 fallback은 없다.** 지도 점수·웨이포인트 성공·CPU는 모두 실제 실행 산출물에서만 나온다.

### 7.4 하드코딩된 절대경로 전수

| 경로 | 위치 | 성격 |
|---|---|---|
| `/work`, `/work/config/*`, `/work/tools/*`, `/work/artifacts/*` | 스크립트 전반 | **컨테이너 내부 경로.** `-v "$ROOT":/work` 바인드 마운트의 반대쪽이므로 고정이 정상 (`run_demo.sh:260`) |
| `/opt/ros/jazzy/setup.bash` | `dex()`/`dexd()`/`GUEST_ENV` | ROS 표준 위치 |
| `/home/vela/gz_split` | `run_split.sh:29` (`GUEST_DIR` 기본값) | 게스트 홈. 변수로 덮을 수 있다 |
| `/home/vela/yolo` | `run_split.sh:173` (`YOLO_DIR` 기본값) | 동일 |
| `$HOME/ms_kim/yolo-artifacts/assets` | `run_split.sh:172` | **저장소 밖**을 가리키는 유일한 기본값. `YOLO_CAM=scene`에서만 읽는다 |
| `/home/vela/.gz/fuel/fuel.gazebosim.org/openrobotics/models/{standing,walking} person/3` | `run_split.sh:428-429` | 컨테이너 안 Fuel 캐시 존재 검사. 없으면 FAIL로 멈춘다(조용히 넘어가지 않음) |
| `~/maps/split-$STAMP.{yaml,pgm}` | `run_split.sh:924` | 게스트가 지도를 저장하는 위치 |
| `/tmp/robot.env.$$`, `/tmp/robot.urdf.$$`, `/tmp/split-$STAMP.*` | `run_split.sh:375,493,924` 등 | PID·타임스탬프 접미사가 붙는 임시 파일. 즉시 `rm -f` |
| `/tmp/run_split.release` | `run_split.sh:218` (`HOLD_FILE`) | `HOLD=1`일 때 해제 신호 파일 |
| `/tmp/run_demo.$VIEWER.lock` | `run_demo.sh:180` | 뷰어 중복 실행 방지 락 |
| `/tmp/type_hashes.py`, `/tmp/gt.log` | `check_interop.sh:44,52`, `run_dds_check.sh:143` | 점검용 임시 |
| `10.0.2.2:42100`~`42105` | `config/guest_tcp_client*.xml`, `host_*_server.xml` | SLIRP 게이트웨이 주소. 게스트에서 호스트를 가리키는 QEMU user-net 고정 주소 |
| `10.0.2.2` (`wan_addr`) | 모든 `host_*_server.xml` | 동일 |

---

## 8) 다른 컴포넌트와의 인터페이스 — **문서로 합의한 것이 있다**

| 상대 | 합의 문서 | 내용 |
|---|---|---|
| **Gazebo ↔ 게스트 Nav2/SLAM** (내부 경계) | `ras/gz-split/config/bridge_split.yaml` + `README.md` Architecture 절 + `DESIGN.md` | 정확히 6토픽, 방향, TF 소유권(Gazebo=`odom→base_footprint`, slam_toolbox=`map→odom`). 각 항목에 "이것을 BIDIRECTIONAL로 바꾸면 무슨 일이 나는가"까지 주석 |
| **YOLO 검출기** | `ras/yolo/README.md` + 내 `config/camera_bridge_fragment.yaml` | 카메라 gz 토픽은 `/camera`, ROS 토픽 `/camera/image_raw`. `YOLO_CAM`이 robot/scene 어느 쪽이든 이 이름을 유지하기로 맞췄다(해당 파일 주석) |
| **YOLO ↔ 사용자의 RAS 에러로깅 데몬** | `ras/yolo/README.md:174-246` "What the RAS error-logging daemon reads" | 조인 키 `detect_mono`(CLOCK_MONOTONIC), `/diagnostics` 1 Hz `DiagnosticArray`(`status.name="yolo/detector"`, TRANSIENT_LOCAL). 내 쪽 기여는 **전송 배관**뿐: `config/host_ras_server.xml`(포트 42105)과 `GUI=1`의 게스트 locator 생성. 유닛의 `ROS_DOMAIN_ID=42`·프로파일 환경변수 2줄은 사용자 몫 (`yolo/README.md:238`) |
| **녹화** | `ras/gz-split/README.md` "Recording the split topology" / "Recording it with the detector" | 터미널 2개 절차, 측정된 소요 시간, `setpts` 후처리 배율 |
| **RAS(RERI) 주입** | **없음** | 이 컴포넌트는 Config A(RERI 없음)를 전제한다. TC.10.007의 co-tenant 워크로드를 만들 뿐 TC.10.007 결과를 내지 않는다 (`run_split.sh:12-15`) |

---

## 9) 미완성 / 미검증 (솔직하게)

### 9.1 지금 당장 실행 검증이 불가능하다

| 사실 | 확인 방법 (실제 출력) |
|---|---|
| 호스트 컨테이너는 살아 있으나 **유휴** | `docker top vela-gz` → `sleep infinity` 1개뿐 |
| **게스트 VM이 내려가 있다** | `ss -ltn \| grep -E ":2222"` → 없음. 떠 있는 `qemu-system-riscv64`는 다른 사용자(`/home/hayong/q-vela-ubuntu/`, `hostfwd tcp::2221`, `reri=true`)의 것이다 |
| 따라서 이번 조사는 **정적 검사까지만** | `bash -n` ×7 OK, python compile ×12 OK, `xmllint` ×8 OK, `yaml.safe_load` ×6 OK — 전부 통과 |
| **QoS 실측을 하지 않았다** | 라이브 그래프가 없어 `ros2 topic info -v`를 돌릴 수 없다. §3.1의 "브리지 기본값"은 코드가 QoS를 오버라이드하지 않는다는 사실이고, 와이어에서 확인한 값이 아니다 |

VM 부팅은 대화형 콘솔이 필요해 사용자 몫이다.

### 9.2 게이트 중 미결/실패

| 게이트 | 상태 | 내용 |
|---|---|---|
| **G6** (co-tenant CPU) | **임계·기준 미확정** | README는 `< 250 of 400`에 `249% / 252% → PASS`로 적혀 있다. 그러나 `logs/`에 기록된 워크로드 총합 16건 중 250 미만은 **2건뿐**이다 (아래 census). 임계값을 어디에 둘지, peak인지 mean인지, census 창이 주행 구간을 덮는지는 사용자 결정 사항이며 **내가 정하지 않았다** |
| **G5-L** (`/odom` 최대 도착 간격) | **미측정** | 계측기 자체 결함(`liveliness_watchdog.py`의 재스탬프가 보고 가능한 최대 간격을 ~0.9 s로 상한) 수정 후 `WATCHDOG=1` 재측정을 하지 않았다. `/scan`은 1557 ms로 **FAIL**이 이미 기록되어 있다 |
| **G5-E** (`explore_lite` 자율탐사) | **통과 런 없음** | 배선·게이트는 완성. 실측: 554 goal 중 0.5 m 초과 0건(사실상 제자리), 시드 후 2건은 얻었으나 map coverage 미달로 FAIL. `logs/g5e-*.log` |
| **G-Y5** Results 행 | 조건 표기 불완전 | 통과 조건인 `RTF_LIMIT=0.25`와 카메라 320x240은 README 본문(`:401`, `:404`, `:642`)에 있으나 **Results 표의 행 자체에는 없다**. 표만 읽으면 무조건 PASS로 보인다 |
| **1시간 soak** | 미실시 | TC.10.007은 한 시간 가까이 수집한다. 이 스크립트의 임무는 2구간 투어에서 끝난다 (`HOLD=1`이 완화책) |

```
$ grep -rh "workload total" logs/*.log | grep -o "[0-9]*% of 400%" | sort -n | uniq -c
```
| 관측값 (% of 400) | 건수 | `< 250` |
|---|---|---|
| 243 | 1 | ✅ |
| 249 | 1 | ✅ |
| 250 | 1 | ❌ |
| 252, 261, 263, 266, 270, 272, 276 | 각 1 | ❌ |
| 279 | 2 | ❌ |
| 291 | 2 | ❌ |
| 296, 303, 304 | 각 1 | ❌ |
| **합계 16건** | | **통과 2건** |

### 9.3 문서·코드 불일치 (내가 만든 것 안에서)

| 위치 | 내용 |
|---|---|
| `README.md:487` | 녹화 레시피가 `FOLLOW_OFFSET_OVERRIDE=-3.0,0,2.0`을 권하는데, 같은 파일 `:644`와 `RUNLOG.md:1931-1934`는 그 값이 **카메라를 4.5 m 아레나 벽 밖으로 보내 로봇을 가린다**고 기록한다. Take 2가 그것으로 망했다. 레시피 쪽이 고쳐지지 않았다 |
| `host_*_server.xml`의 `data_writer` 비동기 프로파일 | **효력 없음.** `RMW_FASTRTPS_PUBLICATION_MODE`도 `RMW_FASTRTPS_USE_QOS_FROM_XML`도 어디에서도 설정하지 않는다(§6.1). XML 주석은 이 사실을 정직하게 적어 두었지만, 파일 자체는 "대칭성 유지" 목적으로 남아 있다 |
| `RUNLOG.md` / `README.md` | **2026-08-20 작업이 기록되지 않았다.** `logs/p4-take5~10-*.log`, `p1-cotenancy`, `p2-circuit-validation`이 존재하고 `run_split.sh`·`make_world.py`·`send_goal.py`가 그날 수정되었으나, RUNLOG는 08-19 23:50에서 멈춰 있다. 이것은 방치가 아니라 **의도된 보류**다 — 문서화(Phase 5)는 사용자 go-ahead 대기 중이다 |

### 9.4 재작업 진행 중인 것 (다른 세션이 이어받은 상태)

사용자가 2026-08-20에 지적: 기존 영상은 YOLO가 아레나 밖 빌보드 사진을 검출하고 로봇은 링만 돌았다. → 로봇 카메라 + 실장면 사람 검출 + 기둥 사이 슬라럼으로 재작업 지시.

| 항목 | 상태 |
|---|---|
| `make_world.py --robot-camera`, `send_goal.py --settle`, `run_split.sh`의 `YOLO_CAM=robot`/`YOLO_PEOPLE`/`SETTLE` | **코드 완성** (커밋 전) |
| Take 7·8·9 | 전부 실패 (planner 클럭 동결 / bt_navigator 16.6 s 지연 / 액션서버 무응답) |
| Take 10 | **부분 성공** — settle 프로브 소진 후 8개 중 3개 웨이포인트, 실주행 ~4.8 m. 부분 맵 precision 0.790 (< 0.80 floor) |
| **Take 11** | **미촬영.** `SETTLE=30`, 녹화 1500 s로 재촬영만 남았다 |
| 슬라럼 웨이포인트 | `WAYPOINTS_OVERRIDE`로 명령줄에서만 전달된다 — 저장소에 커밋된 정의가 없다 |

### 9.5 그 밖에 알고 있는 미완 항목

| 항목 | 내용 |
|---|---|
| 첫 골 abort | 분리형 런에서 bringup 직후 첫 goal이 실패하는 현상이 반복 관측된다 (`logs/`의 로그 64개 중 20개에 `status=6`/abort 문자열이 있다). Nav2 코드 결함으로 추적된 것은 아니고, 게스트 클럭 회복 지연이 원인으로 기록되어 있다. `SETTLE`이 완화책이고 근본 해결은 아니다 |
| bond liveliness | `nav2_split.yaml:258` `bond_timeout: 0.0` — 전 노드에서 비활성. TCG 게스트에서 오탐이 나서 껐고, 대체 감시가 `liveliness_watchdog.py`인데 그것은 코어의 69%를 먹는다 |
| 게스트 Nav2 로그 | 실행 후 호스트로 회수하지 않는다. 실패 분석이 매번 재현에 의존한다 |
| teardown | `run_split.sh`의 EXIT 트랩이 판정 출력 직후 Gazebo 서버를 죽인다. 녹화 중이면 **얼어붙은 장면이 계속 녹화되고 아무 경고도 없다**. `HOLD=1`이 완화책이고 기본값은 0이다 |
| `--record` | 검은 화면/빈 프레임 검사가 없다. 녹화가 끝난 뒤 사람이 봐야 안다 |
| 커밋·푸시·PR | **보류 중** (사용자 지시, 2026-08-19: "제가 추가적인 요청이 있을 때까지는 커밋, 푸시, PR을 하지 마세요"). `ras/gz-split/` 43개 파일이 전부 untracked다 |
| TSP 정정요청 문서 개정 | 미착수. 해당 문서는 **대외비**이며 git 대상이 아니다 |

---

## 부록: 정적 검사 실제 출력 (2026-08-22)

```
bash -n         scripts/*.sh          7/7 OK
python compile  tools/*.py           12/12 OK   (.pyc 미생성 방식)
xmllint --noout config/*.xml          8/8 OK
yaml.safe_load  config/*.yaml         6/6 OK
```
