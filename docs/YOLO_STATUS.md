# YOLO 컴포넌트 상태 보고 (읽기 전용 조사)

- 조사 일시: 2026-08-22 03:40~03:50 KST
- 조사 방식: 읽기 전용. 코드 수정 없음. 이 파일 1개만 신규 생성 (`docs/` 디렉터리도 이때 처음 생성됨).
- 저장소: `~/ms_kim/riscv-vela/vela-apps` (branch `feat/ras-tools`)
- 컴포넌트 정의: **TC.10.007 가용성 시험의 AI 추론 co-tenant** = `ras/yolo/` (검출기 본체)
  + `ras/gz-split/` (Nav2/SLAM/Gazebo 분할 리그 및 녹화 경로). 두 개를 합쳐 "YOLO 파트"로 보고한다.

---

## 1) 내가 만들거나 수정한 파일 전체 목록

### 1-1. `git status` (실행 출력 그대로)

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

```
$ git status --porcelain
 M ras/README.md
?? ras/gz-split/
?? ras/nav2/
?? ras/ras-ros/
?? ras/workload-nav2.md
?? ras/yolo/
```

### 1-2. `git log --oneline -20` (실행 출력 그대로)

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

`git log`가 이 컴포넌트에 대해 알려주는 것은 **없다.** `ras/yolo/`와 `ras/gz-split/`은 전부
untracked이고, 커밋은 금지되어 있다 (사용자 지시: "커밋 및 푸시는 하지 말아주세요").
커밋 계획은 `~/ms_kim/yolo-artifacts/commit-draft.txt`에 3-커밋 초안으로만 준비돼 있다.

remote 확인:

```
$ git remote -v
fork	https://github.com/felixmskim/vela-apps.git (fetch)
fork	https://github.com/felixmskim/vela-apps.git (push)
origin	https://github.com/riscv-vela/vela-apps.git (fetch)
origin	https://github.com/riscv-vela/vela-apps.git (push)
```

### 1-3. 파일 목록 — `ras/yolo/` (소스 19개, 전부 신규 생성)

| 경로 | bytes | mtime |
|---|---|---|
| `ras/yolo/README.md` | 18990 | 2026-08-19 23:09 |
| `ras/yolo/scripts/00-preflight.sh` | 1871 | 2026-08-18 21:42 |
| `ras/yolo/scripts/build-in-container.sh` | 2170 | 2026-08-18 22:07 |
| `ras/yolo/scripts/build_ort_wheel.sh` | 2299 | 2026-08-18 21:43 |
| `ras/yolo/scripts/export_model.sh` | 1566 | 2026-08-18 21:42 |
| `ras/yolo/scripts/install_guest.sh` | 1955 | 2026-08-18 21:43 |
| `ras/yolo/scripts/make_bag.sh` | 3544 | 2026-08-18 21:46 |
| `ras/yolo/scripts/run_detect.sh` | 835 | 2026-08-18 21:44 |
| `ras/yolo/scripts/run_node_check.sh` | 2252 | 2026-08-19 22:35 |
| `ras/yolo/scripts/run_yolo.sh` | 5468 | 2026-08-18 21:45 |
| `ras/yolo/tools/check_yolo_node.py` | 9233 | 2026-08-19 22:29 |
| `ras/yolo/tools/detect.py` | 3456 | 2026-08-18 21:37 |
| `ras/yolo/tools/detection_overlay.py` | 11164 | 2026-08-19 23:06 |
| `ras/yolo/tools/image_publisher.py` | 2739 | 2026-08-18 21:39 |
| `ras/yolo/tools/make_billboards.py` | 6574 | 2026-08-19 11:48 |
| `ras/yolo/tools/make_camera_world.py` | 5389 | 2026-08-18 21:42 |
| `ras/yolo/tools/make_people.py` | 5074 | 2026-08-20 03:07 |
| `ras/yolo/tools/yolo_node.py` | 15638 | 2026-08-19 22:28 |
| `ras/yolo/tools/yolov8_onnx.py` | 6175 | 2026-08-18 21:37 |

`ras/yolo/`에는 `.gitignore`가 **없다.** 빌드 산출물(wheel, `.onnx`, bag, 골든 결과,
`assets/*.jpg`)은 저장소 밖 `~/ms_kim/yolo-artifacts/`에 두어 회피하고 있으나,
`ras/gz-split/.gitignore`와 달리 방어선이 없고 **이미 새어 들어온 것이 있다**:

```
$ git status --porcelain --untracked-files=all ras/yolo | grep pycache
?? ras/yolo/tools/__pycache__/make_people.cpython-312.pyc
?? ras/yolo/tools/__pycache__/yolov8_onnx.cpython-312.pyc
```

즉 git-visible 항목은 21개이고 그 중 2개가 `.pyc`다. → 9)에 미완성 항목으로 기록.

### 1-4. 파일 목록 — `ras/gz-split/` (소스 43개, 전부 신규 생성)

`config/` — `bridge_split.yaml`, `bridge_tb4.yaml`, `camera_bridge_fragment.yaml`,
`demo_gui.config`, `demo.rviz`, `demo_nav.rviz`, `explore_split.yaml`,
`guest_tcp_client.xml`, `guest_tcp_client_demo.xml`, `host_clock_server.xml`,
`host_cmdvel_server.xml`, `host_overlay_server.xml`, `host_ras_server.xml`,
`host_rviz_server.xml`, `host_tcp_server.xml`, `mapper_params_tcg.yaml`, `nav2_split.yaml`

`scripts/` — `00-preflight.sh`, `check_interop.sh`, `run_container_loop.sh`,
`run_dds_check.sh`, `run_demo.sh`, `run_rtf.sh`, `run_split.sh` (64081 bytes)

`tools/` — `clock_decimator.py`, `clock_lag_probe.py`, `cmd_vel_watchdog.py`,
`latency_probe.py`, `liveliness_watchdog.py`, `make_world.py`, `pose_truth_probe.py`,
`rtt_probe.py`, `score_map.py`, `send_goal.py`, `slam_nav.launch.py`, `type_hashes.py`

루트 — `README.md` (55267), `RUNLOG.md` (114749), `DESIGN.md` (33943),
`Dockerfile`, `Dockerfile.demo`, `.dockerignore`, `.gitignore`

### 1-5. 수정한 tracked 파일 — `ras/README.md` (4줄 추가)

```
$ git diff ras/README.md
@@ -553,6 +553,10 @@ rewritten around what the record actually holds.
 ## See also
 
+- [`workload-nav2.md`](workload-nav2.md) — bringing up the ROS 2 / Nav2 workload the availability test runs against
+- [`gz-split/`](gz-split/) — adding Gazebo and SLAM to that workload, with the simulator on the x86 host
+- [`yolo/`](yolo/) — the AI inference co-tenant, on a self-built riscv64 ONNX Runtime
+- [`ras-ros/`](ras-ros/) — publishing the errors injected here onto ROS 2, and into the event log the TSP names
```

### 1-6. 저장소 밖에 만든 문서 (git 대상 아님)

| 경로 | 내용 |
|---|---|
| `~/ms_kim/nav2관련log/[RVSS-TSP]시험계획서-검토-YOLO파트-20260819.md` | TSP YOLO 파트 검토서 (D-35~D-42, CR-16~CR-19, D-13/14/15 철회·수정) |
| `~/ms_kim/nav2관련log/[q-vela]yolo-ort작업-20260819.md` | 한글 인계 메모 |
| `~/ms_kim/nav2관련log/[handoff]road-driving-demo-prompt-20260819.md` | 도로 주행 촬영용 새 세션 프롬프트 |
| `~/ms_kim/yolo-artifacts/commit-draft.txt` | 3-커밋 초안 (미실행) |
| `~/ms_kim/yolo-artifacts/` | wheel, `yolov8n.onnx`, bag, `assets/bus.jpg`·`zidane.jpg`, 골든 결과 |

### 1-7. 내 것이 아닌 / 판단 불가

- `ras/inject/`, `ras/rasdaemon/` — 커밋됨(`ac7fe69`, `920a597`). 이 컴포넌트 범위 아님.
- `ras/nav2/`, `ras/workload-nav2.md` — 게스트 단독 Nav2 워크로드. YOLO 파트가 참조는 하지만 **내가 만들었는지 git으로는 확인 불가 → 모름.**
- `ras/ras-ros/` — RAS 에러 로깅 데몬. YOLO 파트가 인터페이스를 맞추는 **상대편**이며, `ras/yolo/README.md`는 이것을 별도 소유자의 컴포넌트로 취급한다. **작성 주체는 git으로 확인 불가 → 모름.** 이 조사에서 읽기만 했고 수정하지 않았다.
- 커밋 3개의 author는 모두 `Minseong Kim <ms.kim@chungbuk.ac.kr>`이고 `Co-Authored-By` trailer가 없어, git만으로는 작성 주체를 구분할 수 없다.

---

## 2) 실행 진입점 — 정확한 명령어 한 줄

**Nav2 + SLAM + Gazebo + YOLO를 동시에 켜는 통합 진입점** (`ras/gz-split/`에서):

```
YOLO=1 WATCHDOG= ROBOT=tb3 scripts/run_split.sh
```

`RTF_LIMIT`은 `YOLO=1`일 때 스크립트가 스스로 0.25로 내린다(`scripts/run_split.sh:56-62`).
녹화까지 붙이려면 `GUI=1 OVERLAY=1 RVIZ=1 HOLD=1`을 추가하고 `scripts/run_demo.sh --record`를 쓴다.

**검출기 단독 게이트** (`ras/yolo/`에서):

| 게이트 | 명령어 | 실행 위치 |
|---|---|---|
| G-Y6 | `scripts/run_node_check.sh` | x86 호스트 (`vela-gz` 컨테이너), 약 40 s |
| G-Y2 | `scripts/run_detect.sh` | 게스트, ROS 없음 |
| G-Y3 | `scripts/run_yolo.sh` | 게스트, bag 재생 |
| G-Y4 | `LIVE=1 scripts/run_yolo.sh` | 게스트 + 호스트 Gazebo 카메라 |

**노드 자체**가 게스트에서 실제로 뜨는 명령 (`run_split.sh:636-638` 원문):

```
source /opt/ros/jazzy/setup.bash; \
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/vela/gz_split/guest_tcp_client.xml; \
export ROS_DOMAIN_ID=42; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE; \
nohup python3 /home/vela/yolo/yolo_node.py --ros-args \
  -p model_path:=/home/vela/yolo/yolov8n.onnx > $HOME/yolo-node-<STAMP>.log 2>&1 &
```

---

## 3) 구독(subscribe)하는 ROS 2 토픽

### `tools/yolo_node.py` (게스트)

| 토픽 | 타입 | QoS | 근거 |
|---|---|---|---|
| `/camera/image_raw` (파라미터 `input_topic`) | `sensor_msgs/Image` | reliability **BEST_EFFORT**, durability **VOLATILE**(기본), history KEEP_LAST, **depth 1** | `yolo_node.py:141-145` |

depth 1 + best-effort는 의도적이다. 추론 1회가 TCG에서 수십 초이므로 큐잉된 프레임은
전부 낭비다. 덮어써진 프레임은 `dropped` 카운터로 센다 (`yolo_node.py:175-176`).

### `tools/detection_overlay.py` (호스트 전용)

| 토픽 | 타입 | QoS | 근거 |
|---|---|---|---|
| `/camera/image_raw` (`--image-topic`) | `sensor_msgs/Image` | **RELIABLE / VOLATILE / KEEP_LAST / depth 10** | `detection_overlay.py:93-96` |
| `/detection_result` (`--det-topic`) | `vision_msgs/Detection2DArray` | 동일 | `detection_overlay.py:93-97` |

### `tools/image_publisher.py`

구독 없음 (발행 전용).

---

## 4) 발행(publish)하는 ROS 2 토픽

### `tools/yolo_node.py`

| 토픽 | 타입 | QoS | 근거 |
|---|---|---|---|
| `/detection_result` (`output_topic`) | `vision_msgs/Detection2DArray` | 기본 QoS, **depth 10** (RELIABLE / VOLATILE / KEEP_LAST) | `yolo_node.py:146-147` |
| `/diagnostics` (`diagnostics_topic`) | `diagnostic_msgs/DiagnosticArray` | **RELIABLE / TRANSIENT_LOCAL / KEEP_LAST / depth 1** | `yolo_node.py:155-159` |

- `/detection_result`는 검출이 0건이어도 빈 배열을 발행한다. 가용성 시험에서 "메시지 흐름
  자체가 liveness 신호"이기 때문 (`yolo_node.py:17-20`).
- `/diagnostics`는 `yolo/detector`라는 단일 `DiagnosticStatus`를 **1 Hz**로 낸다
  (`heartbeat` 파라미터 기본값 `1.0`, `yolo_node.py:104`). `TRANSIENT_LOCAL`인 이유는
  리그가 뜨는 데 80 s가 걸려 모든 모니터가 중간에 붙기 때문 (`yolo_node.py:148-154`).

`/diagnostics` KeyValue 키 전체: `state`, `consecutive_failures`, `seq`, `rx`, `dropped`,
`mono`, `frame_age_s`, `model`, `imgsz`, `threads` + (패스 1회 이상이면) `last_seq`,
`last_infer_s`, `last_n`, `last_labels`, `detect_mono`, `last_stamp_sim`.

### `tools/detection_overlay.py`

| 토픽 | 타입 | QoS |
|---|---|---|
| `/detection_image` (`--out-topic`) | `sensor_msgs/Image` | RELIABLE / VOLATILE / KEEP_LAST / depth 10 |

`--show`를 주면 `$DISPLAY`에 OpenCV 창도 띄운다 (녹화가 찍는 대상).

### `tools/image_publisher.py`

| 토픽 | 타입 | QoS |
|---|---|---|
| `/camera/image_raw` (`--topic`) | `sensor_msgs/Image` | 기본, **depth 1** (`image_publisher.py:41`) |

---

## 5) 실행되도록 가정한 머신

**둘 다 있고, 파일별로 다르다. 명시적으로 갈라 놓았다.**

### RISC-V 게스트 (q-vela QEMU VM)에서 도는 것

- `tools/yolo_node.py`, `tools/yolov8_onnx.py`, `tools/detect.py` — 게스트.
  근거: `scripts/run_detect.sh:10` `GUEST_DIR=${GUEST_DIR:-/home/vela/yolo}`,
  `scripts/run_yolo.sh:85-87`이 `gsh`(= `ssh -p 2222 vela@localhost`)로 노드를 띄운다.
  `run_split.sh:636`도 동일.
- 근거 인용 (`tools/yolov8_onnx.py:3-4`):
  > "The guest has no riscv64 PyTorch, so the ultralytics package cannot run there (defect D-13)."
- 런타임은 자체 빌드 wheel: onnxruntime, `providers=["CPUExecutionProvider"]`,
  `intra_op_num_threads=2` (4 vCPU 중 2개, `yolov8_onnx.py:94-98`, `yolo_node.py:93-94`).
- **HiFive P550 EVB는 전혀 가정하지 않았다.** 리포지토리 어디에도 P550 경로가 없다.

### x86-64 호스트에서 도는 것

- `tools/detection_overlay.py` — 호스트 전용. 근거 (`detection_overlay.py:23-26`):
  > "**Runs on the host, never in the guest.** The guest is a four-vCPU emulated machine
  > already carrying Nav2, slam_toolbox and the detector; JPEG-drawing there would take
  > CPU from the workload the demonstration exists to show."
- `tools/make_billboards.py`, `tools/make_camera_world.py`, `tools/make_people.py` —
  `vela-gz` 컨테이너(x86)에서 Gazebo world/SDF를 생성.
- `tools/check_yolo_node.py` + `scripts/run_node_check.sh` — x86 컨테이너.
  근거 (`run_node_check.sh:6-9`):
  > "Runs on the HOST, inside the vela-gz container, in about 40 s. ... None of that needs
  > riscv64, and asserting it on the guest would cost minutes per assertion."
- `ras/gz-split/`의 Gazebo, ros_gz bridge, rviz2, `clock_decimator.py`,
  `cmd_vel_watchdog.py`, `detection_overlay.py` — 전부 호스트 컨테이너.

### 현재 실측된 환경 상태 (2026-08-22 03:41)

```
$ docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
NAMES           IMAGE                       STATUS
ort-build       ubuntu-riscv64:24.04        Exited (0) 3 days ago
vela-gz         vela-gzsim:jazzy            Up 3 days
node-exporter   prom/node-exporter:latest   Exited (143) 7 weeks ago

$ timeout 12 ssh -p 2222 -o BatchMode=yes vela@localhost 'echo GUEST_OK'
ssh: connect to host localhost port 2222: Connection refused
```

**내 게스트 VM은 지금 꺼져 있다.** 호스트에 돌고 있는 QEMU는 다른 사용자(`/home/hayong/`)의
것이고 포트 2221, `-smp 8 -m 65536`, 그리고 `reri=true` + EDK2 pflash가 붙어 있다.
즉 **다른 사람의 VM은 Config C, 내 VM은 Config A**다. 이 조사에서는 그 VM을 건드리지 않았다.

---

## 6) 환경변수 전부

### 이 컴포넌트가 설정하는 것

| 변수 | 값 | 어디서 |
|---|---|---|
| `ROS_DOMAIN_ID` | **42** (`DOMAIN` 기본값) | `run_split.sh:28`, `run_yolo.sh:24` |
| `ROS_DOMAIN_ID` | **97** (G-Y6 전용, 리그와 격리) | `run_node_check.sh:24` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 게스트 `/home/vela/gz_split/guest_tcp_client.xml` (GUI=1이면 `guest_tcp_client_demo.xml`) | `run_split.sh:254-258` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | 호스트 `/work/config/host_tcp_server.xml` 등 참가자별 1개 | `run_split.sh:230,235,237` |
| `ROS_LOCALHOST_ONLY` | **unset** (리그), **1** (G-Y6) | `run_split.sh:231`, `run_node_check.sh:44` |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | **unset** | `run_split.sh:231` |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` — 스크립트가 아니라 **이미지에 박혀 있다** (`gz-split/Dockerfile:89`). 게스트는 Vela APT에 `rmw_fastrtps_cpp`만 있어 기본값이 곧 이것이다. |
| `DISPLAY` | 녹화용 Xvfb 번호 (`run_demo.sh`), 게스트에서는 `DISPLAY=` 로 비움 |
| `QT_QPA_PLATFORM` | `xcb` (오버레이 창) | `run_demo.sh` |

### 스크립트 knob (기본값)

`CONTAINER=vela-gz`, `GUEST=vela@localhost`, `PORT=2222`, `GUEST_DIR=/home/vela/gz_split`,
`YOLO_DIR=/home/vela/yolo`, `ROBOT=tb4`(YOLO 리그는 tb3 권장), `YOLO_CAM=robot`,
`YOLO_RATE=0.25`(robot) / `0.1`(scene), `YOLO_W=320`, `YOLO_H=240`, `YOLO_X=30`, `YOLO_Y=0`,
`YOLO_ASSETS=$HOME/ms_kim/yolo-artifacts/assets`, `WAIT=420`,
`RTF_LIMIT` = `YOLO=1`이면 자동 0.25.

### 노드 ROS 파라미터

`model_path`(기본 `~/yolo/yolov8n.onnx`), `input_topic`, `output_topic`,
`diagnostics_topic`, `imgsz=320`, `conf=0.25`, `iou=0.45`, `threads=2`,
`heartbeat=1.0`, `stale_s=120.0`, `hardware_id`(빈 값이면 `socket.gethostname()`).

`use_sim_time`은 검출기에 **일부러 주지 않는다** (`run_split.sh:632-634`).

---

## 7) 하드코딩 절대경로 / 샘플 이미지 / 더미 / mock / fallback

지정된 grep을 `ras/yolo/`와 `ras/gz-split/`(artifacts·logs 제외) 대상으로 실행했다.
결과 중 "sample"은 대부분 DDS/lidar 용어(`samples`, `h_samples`)라 무관하므로,
실제로 해당하는 항목만 분류한다.

### 7-1. 샘플 이미지 — `bus.jpg` / `zidane.jpg`

| 파일:라인 | 내용 |
|---|---|
| `ras/yolo/scripts/install_guest.sh:33` | `"$ART"/assets/bus.jpg "$ART"/assets/zidane.jpg` 를 게스트로 복사 |
| `ras/yolo/scripts/run_yolo.sh:61,62,64` | 컨테이너로 복사 후 카메라 world 텍스처로 사용 |
| `ras/yolo/scripts/make_bag.sh:39,40,42` | 동일, bag 녹화용 |
| `ras/yolo/scripts/build_ort_wheel.sh:44,52` | wheel 스모크 테스트: `--source /art/assets/bus.jpg --expect person=3,bus=1` |
| `ras/yolo/scripts/run_detect.sh:5` | 주석 |
| `ras/yolo/tools/make_billboards.py:27` | docstring 예시 |
| `ras/yolo/README.md:67` | 문서 |

파일 자체는 저장소 밖 `~/ms_kim/yolo-artifacts/assets/`에 있고 커밋되지 않는다.
`YOLO_CAM=scene` 모드는 이 두 사진을 빌보드로 세워 카메라가 보게 한다. **즉 co-tenant 시나리오의
피검출 대상은 시뮬레이션된 사람/차량이 아니라 사진 텍스처다.** `ras/yolo/README.md`
"Known limitations"에 명시:
> "Billboards are photographs, not simulated humans. The camera exercises the full
> sensor-to-detection path; it does not claim scene fidelity."

`YOLO_CAM=robot`(현재 기본값) + `tools/make_people.py`는 Fuel 캐시의 사람 모델을 실제
3D 모델로 세우므로 이 한계를 부분적으로 벗어난다.

### 7-2. `cv2.imread` 호출

| 파일:라인 | 성격 |
|---|---|
| `ras/yolo/tools/detect.py:60` | 정상. ROS 없는 단독 스모크 테스트의 입력 경로 |
| `ras/yolo/tools/make_billboards.py:47` | 정상. 빌보드 텍스처 생성 |
| `ras/yolo/tools/make_camera_world.py:33` | 정상. 카메라 world 텍스처 생성 |
| `ras/yolo/tools/image_publisher.py:35` | **폴백 경로** — 아래 7-3 |

### 7-3. fallback / stand-in / placeholder 로직 (전부 열거)

1. **`ras/yolo/tools/image_publisher.py` 전체가 폴백이다.** docstring 1-7행:
   > "This is the development-time stand-in for the recorded bag: same topic, same message
   > type, no rosbag2 involved. ... so a demo can run from a handful of JPEGs when Gazebo is not."

   실제 게이트(G-Y3/G-Y4/G-Y5)는 이 파일을 **실행하지 않는다.** grep 결과 호출부가 없다:

   ```
   $ grep -rn "image_publisher" ras/ --exclude-dir=artifacts --exclude-dir=logs
   ras/yolo/scripts/install_guest.sh:32:    "$ROOT"/tools/yolo_node.py "$ROOT"/tools/image_publisher.py \
   ras/yolo/tools/image_publisher.py:28:        super().__init__("image_publisher")
   ras/yolo/README.md:313:| `tools/image_publisher.py` | images-to-topic stand-in when the bag or Gazebo is unavailable |
   ```

   다만 `install_guest.sh:32`가 이것을 **게스트에 설치는 한다.** 즉 어느 게이트도 쓰지 않는
   폴백이 게스트에 상주한다. 시연에서 실수로 이걸로 찍으면 "Gazebo 카메라"가 아니라
   JPEG 슬라이드쇼가 된다 → 9)에 위험으로 기록.

2. **`ras/yolo/tools/detection_overlay.py:103,121,128` — `placeholder()`.**
   Xvfb에 WM이 없어 `xdotool`이 창을 배치하려면 창이 먼저 존재해야 한다. 그래서 첫 검출
   (80-120 s 뒤)이 오기 전까지 회색 프레임을 띄운다. **녹화 초반 최대 2분간 화면의
   PiP는 실제 검출이 아니라 이 placeholder다.**

3. **`ras/gz-split/tools/pose_truth_probe.py:93-96`** — TF 이름이 안 맞으면 메시지에
   엔트리가 1개일 때 그것을 쓴다. spawn이 모델명을 바꿔 3분 런을 잃지 않으려는 방어.
   경고를 1회 로그로 남긴다.

4. **`ras/gz-split/scripts/run_rtf.sh:145`** — RTF가 임계 미달이면
   "abandon live Gazebo for this robot, use bag fallback, escalate D-14" 라는 판정을 출력.
   조치 안내이지 자동 폴백은 아니다.

5. **`ras/yolo/README.md:39`, `scripts/export_model.sh:7`** — 런타임 확보 폴백 사다리
   (Debian trixie 추출 / spacemit 1.15 / ncnn)의 **문서 기록**. 코드에 분기는 없다.
   현재 채택된 경로는 자체 빌드 wheel 하나뿐이다.

**mock / dummy 는 한 곳뿐이고, 테스트 전용이다.** `tools/check_yolo_node.py`가 onnxruntime을
같은 호출 시그니처의 스텁으로 바꿔 G-Y6를 40 s에 돈다. 스텁은 `/tmp/yolo-node-check`에
임시로 쓰이고, 프로덕션 경로에는 존재하지 않는다. 스텁은 **일부러 정수 픽셀을 반환**해
`float()` 캐스팅을 매번 검증한다.

### 7-4. 하드코딩 절대경로

| 파일:라인 | 경로 | 성격 |
|---|---|---|
| `tools/yolo_node.py:87` | `Path.home()/"yolo"/"yolov8n.onnx"` | 파라미터 기본값. 오버라이드 가능 |
| `tools/make_people.py:84` | `Path.home()/'.gz/fuel/fuel.gazebosim.org'` | 인자 기본값. 오버라이드 가능 |
| `scripts/run_node_check.sh:25` | `/tmp/yolo-node-check` | 고정. 컨테이너 내부 |
| `scripts/run_yolo.sh:40` | `/home/vela/gz_split/guest_tcp_client.xml` | **고정. 환경변수 없음** |
| `scripts/run_yolo.sh:41`, `run_split.sh:230` | `/work/config/host_*.xml` | 컨테이너 마운트 경로. 고정 |
| `run_detect.sh:10`, `make_bag.sh:19`, `install_guest.sh:12`, `run_yolo.sh:23` | `/home/vela/yolo` | `GUEST_DIR` 기본값 |
| `run_split.sh:24` | `/home/vela/gz_split` | `GUEST_DIR` 기본값 |
| `make_bag.sh`, `run_yolo.sh` 다수 | `/tmp/yolo/...` | 컨테이너 내부 작업 경로. 고정 |
| `*/scripts/*.sh` 다수 | `/opt/ros/jazzy/setup.bash` | ROS 배포판 경로. 고정 |
| `run_split.sh:172` | `$HOME/ms_kim/yolo-artifacts/assets` | `YOLO_ASSETS` 기본값. **내 홈 디렉터리 구조에 의존** |

가장 이식성이 나쁜 것은 `run_yolo.sh:40`의 게스트 프로파일 절대경로와
`YOLO_ASSETS`의 `$HOME/ms_kim/...` 가정이다.

---

## 8) 다른 컴포넌트와의 인터페이스 — 문서로 합의한 적이 있는가

**부분적으로 있다. 다만 "합의"라고 부를 수 있는 것은 한쪽뿐이다.**

### 문서화되어 있는 것

| 상대 | 문서 | 내용 |
|---|---|---|
| RAS 에러 로깅 데몬 (`ras-ros`) | `ras/yolo/README.md` §"What the RAS error-logging daemon reads" | `[detect]` 로그 1줄 포맷 + `/diagnostics`의 `yolo/detector` 상태 필드 전체 표 + 세 가지 설계 선택(1 Hz / TRANSIENT_LOCAL / 예외 시 노드 생존)의 근거 |
| 같음 | `ras/yolo/tools/check_yolo_node.py` + `scripts/run_node_check.sh` | 그 계약을 **실행 가능한 게이트 G-Y6**로 고정. 16개 단정 |
| 같음 | `ras/gz-split/config/host_ras_server.xml` (port 42105) | 호스트 측 구독자용 Fast DDS 전송 배관. 토픽/타입을 명시하지 않아 메시지가 바뀌어도 유효 |
| Nav2 / SLAM / Gazebo | `ras/gz-split/README.md`, `DESIGN.md`, `RUNLOG.md` | 브리지 토픽 목록, 참가자별 포트(42100~42105), 게이트 G1~G7, RTF/CPU 예산 |
| Gazebo 카메라 → 검출기 | `ras/gz-split/config/camera_bridge_fragment.yaml` | `/camera`(gz) → `/camera/image_raw`(ROS), `gz.msgs.Image` → `sensor_msgs/msg/Image`, `GZ_TO_ROS`. robot/scene 두 모드가 같은 토픽을 쓰도록 고정 |
| 녹화 | `ras/gz-split/README.md` §"Recording it with the detector" | `RTF_LIMIT=0.25`, `HOLD=1`, `OVERLAY=1`, 후처리 `setpts=0.25*PTS` |

### "합의"라고 말할 수 없는 부분 (정직하게)

- **양방향 합의 문서는 없다.** 계약은 전부 **내가 쓴 문서**이고, 상대 컴포넌트가 그것을
  참조하거나 승인한 기록은 없다. `ras/ras-ros/`를 grep한 결과 `yolo`, `yolo/detector`,
  `detection_result`에 대한 언급이 **0건**이다:
  ```
  $ grep -rn "yolo\|detect_mono\|detection_result" ras-ros/
  ras-ros/tools/ras_monitor.py:327:        event["detect_mono"] = round(seen_mono, 6)
  ras-ros/README.md:32:`operation_name`, `proc_isa_name`, `target_address`, `seq`, `detect_mono`,
  ```
- 그리고 그 두 줄이 **이름 충돌**을 드러낸다. `detect_mono`의 의미가 서로 다르다:
  - `ras_monitor.py`: 모니터가 **RAS 이벤트를 읽은** CLOCK_MONOTONIC 시각 (`emit(event, seen_mono)`)
  - `yolo_node.py`: **마지막 추론 패스가 끝난** CLOCK_MONOTONIC 시각
  
  시계는 같으므로 뺄셈은 유효하다(그것이 원래 의도였다). 그러나 키 이름이 같고 의미가
  달라, `name` 필드(`ras/<source>` vs `yolo/detector`)로 구분하지 않는 소비자는 두 값을
  혼동할 수 있다. **이것은 아직 어느 문서에도 경고로 적혀 있지 않다** → 9)에 기록.
- Nav2/SLAM/Gazebo는 같은 사람(나)이 만든 리그이므로 상대가 없다. 즉 컴포넌트 간
  **조직적 합의**는 없고, **단일 작성자의 문서화**만 있다.

---

## 9) 현재 알고 있는 미완성 / 미검증 항목

### 9-1. 오늘 실제로 재검증한 것 (1개)

```
$ cd ras/yolo && scripts/run_node_check.sh
[03:42:38] 0/1 copying the node and its check into vela-gz:/tmp/yolo-node-check
1/3 the node comes up and says so before any frame arrives
      PASS  a status arrived before the first frame: 5
      PASS  it is WARN, not OK: level=1 msg=no camera frame yet (0s)
2/3 frames in, detections and status out
      PASS  /detection_result published: 3 messages
      PASS  /diagnostics reached level OK: 6 of 24
      PASS  one status, named yolo/detector: {'yolo/detector'}
      PASS  every documented key is present: all
      PASS  the labels ride on the status: person:0.83,bus:0.47
      PASS  dropped frames are counted, not inferred from rx: 2
      PASS  detect_mono is the caller's own CLOCK_MONOTONIC: 9.0s ago
      PASS  integer pixels did not abort the process: 2 boxes survived the cast
3/3 a failing inference is reported, and the node survives it
      PASS  the failure is published as ERROR: inference: synthetic inference failure
      PASS  state says failed: failed
      PASS  consecutive_failures counted: 23
      PASS  the node is still running
      PASS  and still publishing after the failure
      RESULT: PASS
[03:43:12] G-Y6: PASS
---- exit=0 ----
```

**G-Y6만 재실행 가능했다.** 이유: 내 게스트 VM이 꺼져 있고(5절 참조), VM 부팅과 대화형
조작은 사용자 몫이다. 따라서 아래 게이트는 **오늘 확인한 것이 아니라 2026-08-19~20 기록**이다.

### 9-2. 문서에 기록된 값 — 오늘 재확인 안 됨

| 게이트 | 기록된 결과 | 출처 |
|---|---|---|
| G-Y1 (컨테이너 런타임+모델) | PASS | `yolo/README.md` |
| G-Y2 (게스트 단독 추론) | PASS | `yolo/README.md` |
| G-Y3 (게스트 bag → 노드) | PASS | `yolo/README.md` |
| G-Y4 (라이브 카메라, TCP 프로파일) | PASS | `yolo/README.md` |
| G-Y5 (Nav2+SLAM+YOLO 공존) | 2/2 waypoints, 0.992/0.990/0.973, 9 detections, 272%/400% | `gz-split/README.md` Results |
| G-Y5 filmed | 4/4 waypoints, 0.961/0.969/0.999, 24 passes, 291%/400%, `infer_s` 14.1/21.4/32.0 | 같음 |
| G5-L (토픽 inter-arrival) | **FAIL** — `/scan` 1557 ms (예산 1000 ms) | 같음 |

이 값들은 전부 내 서술이 아니라 커밋 대기 중인 문서의 표에서 그대로 옮긴 것이며,
**오늘 재현하지 않았다.**

### 9-3. 미완성 / 미검증 / 위험 (솔직한 목록)

1. **가장 큰 것: 이 컴포넌트는 TC.10.007 결과를 아직 하나도 만들지 못한다.**
   내 게스트는 Config A — `reri=true`도 EDK2 pflash도 없어 **에러 주입이 물리적으로
   불가능하다.** co-tenant 워크로드는 완성됐지만 "에러 주입 중에도 추론이 살아 있었다"는
   측정은 0건이다. Config C 병합이 선행 조건이고, 그건 내가 못 한다.

2. **`detect_mono` 이름 충돌** (8절). 문서에 경고가 없다.

3. **`yolo_node.py` docstring이 코드와 어긋난다.** 46-48행:
   > "The status carries a HEARTBEAT: it is published every 10 s ..."
   
   그런데 실제 파라미터 기본값은 `heartbeat = 1.0`(104행)이고, 그 바로 위 주석은 1 Hz를
   택한 이유를 길게 설명한다. 즉 모듈 docstring만 옛 값이 남았다. README와 G-Y6는 1 Hz가
   맞다. **읽기 전용 지시 때문에 고치지 않았다.**

4. **`ras/yolo/`에 `.gitignore`가 없고, 이미 `.pyc` 2개가 새어 들어와 있다**
   (`tools/__pycache__/make_people.cpython-312.pyc`,
   `tools/__pycache__/yolov8_onnx.cpython-312.pyc`). `gz-split`과 달리 방어선이 없어
   wheel(수십 MB)이나 `.onnx`도 같은 경로로 들어갈 수 있다.

5. **`image_publisher.py`는 어느 게이트도 실행하지 않는 폴백인데 게스트에는 설치된다**(7-3).
   시연에서 잘못 쓰면 Gazebo 카메라가 아닌 JPEG 슬라이드쇼를 찍는다. 삭제할지 게이트에
   편입할지 결정 안 됨.

6. **녹화 초반의 PiP는 placeholder 회색 프레임**이다(7-3). 최대 2분. 영상 편집에서
   잘라내야 하는데 그 절차가 문서에 없다.

7. **G5-L은 FAIL 상태로 남아 있다.** `/scan` inter-arrival 1557 ms > 1000 ms 예산.
   YOLO 파트가 원인인지(3번째 co-tenant의 CPU 압박) watchdog 계측 자체의 한계인지
   분리되지 않았다. `RUNLOG.md`에 기록만 되어 있다.

8. **`/detection_result`의 발행 간격이 39.98 s (p50, max 48.04)** — TSP의 1,000 ms 토픽
   지연 예산의 40~48배이고 20 s 검출 창보다도 길다. 그래서 이 토픽에는 그 기준을 적용할 수
   없고 `ros2 topic hz`로 생존 확인도 못 한다. D-36/D-39로 제기하고 CR-16으로 기준 대체를
   제안했지만 **TSP 반영 여부는 미확정.**

9. **정정된 에러(CE)가 `infer_s`에 보일 가능성은 사실상 없다.** 한 런 내 정상 산포가
   14-32 s여서 CE 1건의 커널 처리 비용은 노이즈에 묻힌다. 720 s 간격 주입은 약 18 패스로
   통계적 검정력이 없다. 이 워크로드가 지지할 수 있는 주장은 "heartbeat가 멈추지 않았고
   주입 창 안에 최소 1회 패스가 완료됐다"까지다.

10. **도로 주행 시나리오와 그 영상은 착수조차 안 됐다.** 현재 촬영된 영상은 2점 왕복
    (`WAYPOINTS="1.83,0.08 -1.88,0.08"`)이다. 링 도로(중앙 섬 2.50×2.50 m, 통로 폭 약
    1.45 m)를 쓰는 경로는 `[handoff]road-driving-demo-prompt-20260819.md`에 계획만 있다.
    **경로 좌표는 검증하지 않았다** — 내 map→world 부호 추론이 주행 로그와 어긋나서
    절대 좌표를 문서에 넣지 않고 도출 방법만 남겼다.

11. **`ras-monitor.service`는 `ROS_DOMAIN_ID`도 Fast DDS 프로파일도 설정하지 않는다.**
    기본값이면 domain 0 + 공유 메모리로 떠서, domain 42 + TCP로 도는 리그와 **다른 ROS
    그래프**가 된다. 전송 배관(`host_ras_server.xml`, 42105)은 만들어 뒀지만 unit 파일의
    두 줄은 사용자 몫이고 아직 안 들어갔다. **즉 데몬과 검출기가 실제로 같은 그래프에서
    만나는 것을 한 번도 확인한 적이 없다.**

12. **커밋/푸시 전무.** 19 + 43 = 62개 소스 파일이 untracked이며, 사용자 지시로 커밋하지 않았다.
    `~/ms_kim/yolo-artifacts/commit-draft.txt`에 3-커밋 초안이 있다. 백업은
    `~/ms_kim/riscv-vela/backup_20260822.tar.gz` (1.45 GB, 2026-08-22 03:40) 하나뿐이다.

13. **`RMW_IMPLEMENTATION`을 게스트에서 명시적으로 설정한 곳이 없다.** 호스트 이미지에는
    `Dockerfile:89`로 박혀 있지만, 게스트는 "Vela APT에 `rmw_fastrtps_cpp`만 있다"는
    사실에 의존한다. 게스트에 다른 RMW가 설치되면 조용히 깨진다. 검증된 적 없음.

14. **`ras/nav2/`, `ras/ras-ros/`의 작성 주체를 git으로 확인할 수 없다 → 모름.**
    (1-7절)

---

## 부록: 이 파일의 위치와 이웃 감사

`vela-apps`에는 `docs/`가 없었다. 이 조사에서 처음 만들었다. 같은 시각(03:48)에 다른
채팅창이 **gz-split 컴포넌트에 대해 같은 감사**를 수행해 `docs/GZ_SPLIT_STATUS.md`와
`ras/gz-split/docs/GZ_SPLIT_STATUS.md` 두 곳에 결과를 썼다. 두 감사 결과가 한자리에
모이도록 이 파일은 `docs/YOLO_STATUS.md` 한 곳에만 둔다. 이웃 채팅창의 파일은
읽기만 했고 건드리지 않았다.

주의: 그 채팅창의 `git status` 인용문에는 `docs/`가 untracked로 이미 찍혀 있는데,
그것은 내가 이 조사에서 만든 디렉터리다. 두 감사가 같은 저장소에서 동시에 돌아
서로의 출력에 나타난 결과다.
