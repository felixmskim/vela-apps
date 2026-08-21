# ROS 2 에러 로깅 시스템 (`ras-ros`) — STATUS

작성 2026-08-22. **읽기 전용 조사. 이 조사 중 코드는 한 줄도 수정하지 않았습니다.**
저장소: `~/ms_kim/riscv-vela/vela-apps` (브랜치 `feat/ras-tools`)
대상 디렉터리: `ras/ras-ros/`
Gazebo 시연 리그는 별도 문서 → [`GAZEBO_DEMO_STATUS.md`](GAZEBO_DEMO_STATUS.md)

---

## 0. 한 문장 요약

커널의 RAS 이벤트(tracepoint / `/dev/kmsg` / rasdaemon sqlite)를 읽어
`diagnostic_msgs/DiagnosticArray`로 ROS 2에 발행하고 `/var/log/ras/events.log`에 JSON으로 남기는
단일 노드입니다. **커널이 이벤트를 실제로 내는 절반은 아직 검증되지 않았습니다** — 이 게스트는
Config A(`reri=true` 없음)라 오류를 주입할 수 없고, 지금은 그 게스트조차 떠 있지 않습니다(§9).

---

## 1) 파일 전체 목록

### `git status` / `git log --oneline -20`

`GAZEBO_DEMO_STATUS.md` §1과 동일한 출력입니다(같은 저장소·같은 브랜치). 요점만:

```
On branch feat/ras-tools
Your branch is up to date with 'fork/feat/ras-tools'.
Changes not staged for commit:
	modified:   ras/README.md
Untracked files:
	ras/gz-split/
	ras/nav2/
	ras/ras-ros/          <-- 이 문서의 대상
	ras/workload-nav2.md
	ras/yolo/
```

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

`ras-ros/`는 **아직 커밋되지 않았습니다.** 상위 3개 커밋(제가 초안, 사용자가 커밋)은
`ras/inject/`·`ras/rasdaemon/`·`ras/README.md`이고 이 컴포넌트가 그 위에 얹힙니다.

### 파일 10개 (`git add -n ras-ros`, 실제 add 안 함)

| 파일 | 줄수 | 역할 |
|---|---|---|
| `ras/ras-ros/README.md` | 292 | 절차서 |
| `ras/ras-ros/tools/ras_decode.py` | 427 | 파서 + CPER 디코더. **ROS 없음, root 없음** — 어느 머신에서든 `--self-test` 가능 |
| `ras/ras-ros/tools/ras_monitor.py` | 468 | 노드 본체. 소스 3개, 토픽 2개, 로그 1개 |
| `ras/ras-ros/tools/check_ras_monitor.py` | 278 | 인수 시험 (게이트 G-R1의 2단계) |
| `ras/ras-ros/scripts/install_guest.sh` | 76 | 호스트에서 실행, ssh로 게스트에 설치 |
| `ras/ras-ros/scripts/guest-tracefs-setup.sh` | 58 | 커널측 상태의 **단일 진실원** (systemd가 같은 파일을 `ExecStartPre`로 재사용) |
| `ras/ras-ros/scripts/run_check.sh` | 50 | 게이트 G-R1 |
| `ras/ras-ros/systemd/ras-monitor.service` | 44 | 무인 운용 |
| `ras/ras-ros/samples/non_standard_event.trace` | 12 | **기록된** tracepoint 레코드 2건 |
| `ras/ras-ros/samples/hardware_error.kmsg` | 13 | **기록된** kmsg 레코드 10줄 = 1 이벤트 |

`ras-ros/`에는 `.gitignore`가 없습니다. 현재 `__pycache__`도 없어서 문제는 안 나지만,
`python3 ras_monitor.py`를 이 디렉터리에서 한 번 돌리면 `.pyc`가 생겨 커밋에 섞입니다.
읽기 전용 조사이므로 고치지 않았습니다.

---

## 2) 실행 진입점 — 정확한 한 줄

작업 디렉터리는 `~/ms_kim/riscv-vela/vela-apps/ras/ras-ros`입니다.

**설치(호스트에서 실행, ssh로 게스트를 구동):**
```bash
scripts/install_guest.sh            # 도구 + 커널 상태만
scripts/install_guest.sh --service  # systemd 유닛까지
```

**검증(게이트 G-R1):**
```bash
scripts/run_check.sh
```

**노드 직접 실행(게스트 안에서):**
```bash
python3 /home/vela/ras_ros/ras_monitor.py --log-file /var/log/ras/events.log
```

**파서만 — 어느 머신에서든 (ROS 불필요):**
```bash
python3 tools/ras_decode.py --self-test
```

---

## 3) 구독하는 ROS 2 토픽

**없습니다. 이 노드는 ROS 토픽을 하나도 구독하지 않습니다.**

`grep -n "create_subscription" tools/ras_monitor.py` → 히트 0건. 입력은 전부 ROS 밖입니다:

| 입력 | 경로 | 스레드 | 기본값 위치 |
|---|---|---|---|
| tracepoint | `/sys/kernel/tracing/instances/ras_ros/trace_pipe` | `TraceSource` | `ras_monitor.py:426`, `DEFAULT_INSTANCE` = `:57` |
| 커널 printk | `/dev/kmsg` (`SEEK_END`) | `KmsgSource` | `:429` |
| rasdaemon DB | `/var/lib/rasdaemon/ras-mc_event.db` (30초 주기 폴링) | `RasdaemonSource` | `:430`, `DEFAULT_DB` = `:58` |

각 소스는 `--no-trace` / `--no-kmsg` / `--no-rasdaemon`으로 끌 수 있습니다.

구독하는 쪽은 **인수 시험 노드**뿐입니다:

| 노드 | 토픽 | 타입 | QoS (reliability / durability / depth) | 위치 |
|---|---|---|---|---|
| `check_ras_monitor.py` | `/ras/events` | `diagnostic_msgs/DiagnosticArray` | RELIABLE / VOLATILE / 100 | `check_ras_monitor.py:79-84` |
| `check_ras_monitor.py` | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RELIABLE / VOLATILE / 100 | `:79-85` |

---

## 4) 발행하는 ROS 2 토픽

| 토픽 | 타입 | QoS (reliability / durability / depth) | 무엇을 나르는가 | 위치 |
|---|---|---|---|---|
| `/ras/events` | `diagnostic_msgs/DiagnosticArray` | **RELIABLE / VOLATILE / 100** | 하드웨어 오류 1건당 status 1개. 그 외엔 아무것도 안 나감 | `ras_monitor.py:253-259` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | **RELIABLE / VOLATILE / 100** | 같은 이벤트 + 10초 주기 하트비트 | `:253-260` |

토픽 이름과 depth는 CLI로 바꿀 수 있습니다: `--events-topic`(기본 `/ras/events`),
`--diagnostics-topic`(기본 `/diagnostics`), `--depth`(기본 100), `--heartbeat`(기본 10.0초).

ROS가 아닌 세 번째 싱크: `/var/log/ras/events.log` — JSON Lines, 쓸 때마다 flush.
TSP §4.3.1이 사전조건으로 `truncate -s0 /var/log/ras/events.log`를 요구하는데
**시스템의 어떤 것도 그 파일을 쓴 적이 없었습니다**(정정요청 D-10). 이 노드가 그 생산자입니다.

### `KeyValue`로 실리는 필드

`ghes_severity`, `error_type_name`, `operation_name`, `proc_isa_name`, `target_address`,
`seq`, `detect_mono`, `detect_latency_ms`, 그리고 원시 바이트.

### 심각도 매핑 — 여기가 틀리면 판정 전체가 뒤집힙니다

tracepoint의 `severity:`는 **GHES** 심각도이고 레코드 안의 **CPER** 심각도가 아닙니다.
`ghes_do_proc()`이 둘을 재매핑하며 두 번호 체계는 거의 서로의 역순입니다.

| GHES | 뜻 | 원래 CPER | `DiagnosticStatus.level` |
|---|---|---|---|
| 0 | none | informational (3) | `OK` (0) |
| 1 | corrected | corrected (2) | **`WARN` (1)** |
| 2 | recoverable | recoverable (0) | `ERROR` (2) |
| 3 | fatal | fatal (1) | `ERROR` (2) |

**TC.10.007은 corrected 오류를 주입합니다.** 정의상 워크로드가 멈추지 않으므로 이를 실패로
발행하면 틀리고, 그 알람에서 잰 다운타임은 허구입니다(정정요청 D-02와 같은 논지).

### 왜 커스텀 메시지가 아닌가

`vela_ras_msgs/RasEvent`가 필드를 더 정확히 담겠지만 `rosidl` 빌드가 필요하고
(에뮬레이트 riscv64에서 수 시간), x86-64 호스트 컨테이너에 없는 타입 해시를 만들어
**호스트측 구독자가 같은 패키지를 빌드하지 않으면 읽을 수 없게 됩니다.**
`diagnostic_msgs`는 이미 게스트에 있고 `rqt_robot_monitor`·`diagnostic_aggregator`가 소비합니다.
커스텀 타입은 업그레이드이고 전제조건이 아닙니다.

---

## 5) 어떤 머신을 가정했는가

**RISC-V 게스트(QEMU q-vela)입니다.** HiFive P550 EVB는 가정하지 않았습니다 — 이 디렉터리에
`p550`/`hifive`/`evb` 문자열이 0건입니다.

근거 — `scripts/install_guest.sh:4-6`:
```
# Run it from the HOST -- it drives the guest over ssh, like the other
# install_guest.sh scripts here. Idempotent: run it again after editing a tool
# and only the copies happen.
```

근거 — `scripts/install_guest.sh:22-24`, `scripts/run_check.sh:24-26`:
```bash
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/ras_ros}
```

근거 — `README.md §Environment`:
```
| Guest | Ubuntu 24.04 riscv64, kernel 6.18.0-rc5, CONFIG_RAS=y, CONFIG_ACPI_APEI_GHES=y |
| Injection | not available -- this guest is Config A (wg=on, no reri=true, no pflash pair) |
```

**단, 컴포넌트가 두 조각으로 갈립니다:**

| 조각 | 도는 곳 | 근거 |
|---|---|---|
| `ras_decode.py` | **어디서든.** ROS도 root도 필요 없음 | `ras_monitor.py:32-35` 도크스트링; **오늘 x86 호스트에서 실제로 실행해 확인**(§9) |
| `ras_monitor.py` + `check_ras_monitor.py` | 게스트 (tracefs·kmsg를 읽으므로) | 위 |
| `install_guest.sh` / `run_check.sh` | **호스트** — ssh로 게스트를 몰음 | 위 |
| `guest-tracefs-setup.sh` | 게스트, root | `ras-monitor.service:26` `ExecStartPre=+...` |

root로 돌리지 **않는** 것도 의도입니다: root 소유 Fast DDS 참가자는 Nav2 노드들과
`/dev/shm`을 깔끔하게 공유하지 못합니다(`ras_monitor.py:32-35`, `ras-monitor.service:16-20`).

---

## 6) 설정한 환경변수 전부

| 변수 | 값 | 어디서 | 비고 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | **71** | `scripts/run_check.sh:35` (`DOMAIN=${DOMAIN:-71}` `:27`) | 전용 도메인. 기본 0이면 이미 도는 그래프에 섞이고, gz-split은 42를 씀 — 워크로드와 도메인을 공유하는 검사는 워크로드에 흔들리고 워크로드를 흔듦 |
| `RMW_IMPLEMENTATION` | **설정하지 않음** | — | 게스트에 `rmw_fastrtps_cpp` 하나뿐 |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | **설정하지 않음** | — | 게스트 안에서만 통신하면 SHM으로 충분. 호스트에서 보려면 `../gz-split/config/host_ras_server.xml`(포트 42105)가 필요 |
| `SupplementaryGroups` | `tracing` | `systemd/ras-monitor.service:20` | tracefs가 `0640 root:tracing`이 된 뒤 일반 사용자로 읽게 하는 유일한 수단 |
| `User` | `vela` | `:16` | root 회피 |
| `STUB_FAIL_AFTER` | 시험에서만 | `check_ras_monitor.py` | 합성 추론 실패 주입 (해당 노드 아님 — YOLO 쪽 참조) |

**미완성 1건 — 아래 §9-3에 다시 씁니다.** `ras-monitor.service`는 `ROS_DOMAIN_ID`도
Fast DDS 프로파일도 설정하지 않습니다. 그대로 부팅하면 **도메인 0 + SHM**으로 떠서
도메인 42의 워크로드와 **다른 ROS 그래프**가 됩니다. 전송 배관은 만들어 뒀지만
(`../gz-split/config/host_ras_server.xml`, `GUI=1`이 게스트 locator 생성)
유닛의 `Environment=` 2줄은 아직 없습니다.

---

## 7) 하드코딩·샘플·더미·mock / fallback

### (a) 기록된 픽스처 — **가장 중요한 정직성 항목**

| 파일 | 무엇인가 |
|---|---|
| `samples/non_standard_event.trace` | **합성 트래픽이 아닙니다.** RERI 가능 게스트에서 실제로 나온 `ras:non_standard_event` 2줄을, `trace_pipe`가 실제로 뱉는 **192바이트 전체 길이로 복원**한 것 |
| `samples/hardware_error.kmsg` | 같은 TLB 이벤트를 커널 자신의 GHES 보고로 본 것 — `{1}` 상관 id를 공유하는 printk 10줄 |

파일 머리말이 스스로 밝힙니다(`samples/non_standard_event.trace:1-10`):
```
# Recorded ras:non_standard_event output. Not synthetic traffic: both lines are
# the events documented in ../../README.md, restored to the full 192-byte record
#   line 1  RERI code 9,  TLB/page-walk, deferred   -> CPER 0x02, GHES 2 recoverable
#   line 2  RERI code 17, execution unit, corrected -> CPER 0x08, GHES 1 corrected
# Line 2 is what TC.10.007 injects four times an hour.
```

자기정합성 근거: 레코드의 `validation_bits = 0x20f`
(= proc_type | proc_isa | error_type | operation | target_addr)이고, 이것은 dmesg가 출력한
**바로 그 다섯 필드**입니다. 즉 픽스처와 커널 출력이 서로를 검증합니다.

**그러나 이것은 재생(replay)입니다.** G-R1이 PASS라는 말은 "레코드가 들어가면 토픽으로
나온다"는 뜻이고 **"커널이 그 레코드를 낸다"는 뜻이 아닙니다.** 후자는 Config C가 있어야
검증됩니다. 이 구분을 보고서에서 흐리면 안 됩니다.

### (b) 시험 하네스가 대체하는 것

| 파일:라인 | 대체물 | 성격 |
|---|---|---|
| `check_ras_monitor.py:118-121` | `os.mkfifo(tmp/"trace_pipe")` — `trace_pipe` 자리에 FIFO | 커널 링버퍼 대역. 하류(파서·디코더·퍼블리셔·로그 writer)는 **전부 프로덕션 경로 그대로** |
| `check_ras_monitor.py:65,121` | 가짜 `trace_clock` 파일에 `local global counter uptime perf [mono] mono_raw boot tai` | 실제 인스턴스가 하는 말을 그대로 씀 → `_check_trace_clock()`이 프로덕션과 같은 분기를 탐 |
| `check_ras_monitor.py:134` | `--no-kmsg --no-rasdaemon` | 한 번에 한 소스만 시험 |

`mock`·`dummy`·`fallback`·`placeholder`·`imread` 히트는 이 디렉터리에 **0건**입니다.
`sample`은 전부 `samples/` 경로이거나 산문입니다.

### (c) 하드코딩 절대경로

| 파일:라인 | 값 | 덮어쓰기 |
|---|---|---|
| `systemd/ras-monitor.service:26` | `/home/vela/ras_ros/guest-tracefs-setup.sh vela` | **없음 — 유닛에 고정** |
| `systemd/ras-monitor.service:31` | `/home/vela/ras_ros/ras_monitor.py`, `/var/log/ras/events.log`, `/opt/ros/jazzy/setup.bash` | **없음 — 유닛에 고정** |
| `scripts/run_check.sh:26`, `install_guest.sh:24` | `/home/vela/ras_ros` | `GUEST_DIR=` |
| `ras_monitor.py:57` | `/sys/kernel/tracing/instances/ras_ros` | `--trace-path` |
| `ras_monitor.py:58` | `/var/lib/rasdaemon/ras-mc_event.db` | `--rasdaemon-db` |
| `ras_monitor.py:429` | `/dev/kmsg` | `--kmsg-path` |
| `install_guest.sh:27-28` | `/sys/kernel/tracing`, 인스턴스명 `ras_ros` | `TRACING=`, `INSTANCE=` |

경로 탐색 1건: `check_ras_monitor.py:48-58`, `ras_decode.py:404-410`의 `find_samples()`가
`HERE/"samples"`와 `HERE.parent/"samples"`를 순서대로 시도합니다. 설치 스크립트가
`tools/`를 평탄화해서 게스트에서는 전자, 저장소에서는 후자가 맞기 때문입니다.

---

## 8) 인터페이스를 문서로 합의한 적이 있는가

**한 건 있습니다.** 사용자와 2026-08-19에 확정했습니다.

| 상대 | 문서 | 내용 |
|---|---|---|
| **YOLO 검출기** (`../yolo/tools/yolo_node.py`) | `../yolo/README.md:174` §"What the RAS error-logging daemon reads" (필드는 `:209`) | **조인 키는 `CLOCK_MONOTONIC`이고 이름은 양쪽 모두 `detect_mono`** (`time.clock_gettime(time.CLOCK_MONOTONIC)`). sim time을 쓰면 bag 루프에서 역행하므로 판정에 쓰지 않음. 생존 판정은 `/diagnostics` 1 Hz 하트비트 — `/detection_result`는 co-tenancy 실측 발행 간격 p50 = **39.98초**로 TSP의 20초 탐지창보다 길어 쓸 수 없음(`../yolo/README.md:279`) |
| 호스트측 구독자 | `../gz-split/config/host_ras_server.xml` | 전송 배관만. **토픽도 타입도 명시하지 않음** — 데몬의 메시지가 바뀌어도 이 파일은 계속 맞도록. 포트 42105, `GUI=1`일 때 게스트 locator 자동 생성 |
| rasdaemon | `../rasdaemon/` | 독립 trace 인스턴스를 읽음 → 서로 이벤트를 훔치지 않음. 카운트 교차검증만 공유 |

**다른 스레드 / 다른 담당자와 합의한 것은 없습니다.** Config C도 TC.10.007 계측 툴링도
인터페이스 합의 없이 넘겼습니다.

---

## 9) 미완성 / 미검증 항목

### 오늘(2026-08-22) 실제로 실행해 확인한 것

```
$ cd ras/ras-ros/tools && python3 ras_decode.py --self-test
trace fixture: /home/cslab/ms_kim/riscv-vela/vela-apps/ras/ras-ros/samples/non_standard_event.trace
  PASS  events parsed: 2
  PASS  tlb sec type recognised: True
  PASS  tlb ghes severity: 2
  PASS  tlb ghes severity name: 'recoverable'
  PASS  tlb proc_type: 'RISCV'
  PASS  tlb proc_isa: 'RISCV64'
  PASS  tlb error type: 'TLB error'
  PASS  tlb operation: 'data read'
  PASS  tlb declared len: 192
  PASS  tlb decoded len: 192
  PASS  tlb truncated: False
  PASS  tlb target address: 0
  PASS  tlb diag level: 2
  PASS  ce ghes severity name: 'corrected'
  PASS  ce error type: 'micro-architectural error'
  PASS  ce operation: 'data write'
  PASS  ce diag level (corrected is a warning): 1
kmsg fixture: /home/cslab/ms_kim/riscv-vela/vela-apps/ras/ras-ros/samples/hardware_error.kmsg
  PASS  kmsg lines matched: 10
  PASS  kmsg groups: 1
  PASS  kmsg severity: 'recoverable'
  PASS  kmsg error type: '0x02'
  PASS  kmsg diag level: 2
SELF-TEST: PASS
exit=0
```

구문 검사도 오늘 돌렸습니다: `bash -n` 3/3 OK, `ast.parse` 3/3 OK.

### 오늘 확인 **불가능**했던 것

```
$ timeout 25 ssh -p 2222 -o BatchMode=yes vela@localhost 'uptime; uname -m'
ssh: connect to host localhost port 2222: Connection refused
```

`vela@localhost:2222` 게스트가 떠 있지 않습니다. 따라서 `install_guest.sh`와 `run_check.sh`는
**오늘 실행하지 못했습니다.** 지금 도는 riscv64 VM은 다른 계정 소유이고 포트 2221이며
`reri=true` + pflash 쌍을 가진 Config C 계열입니다(상세는 `GAZEBO_DEMO_STATUS.md` §0).

### 미완성 / 미검증 — 솔직하게

1. **커널 절반이 미검증입니다.** Config A에는 RERI 장치가 없으므로 모든 시험의 모든 이벤트가
   재생된 기록입니다. **이것이 TC.10.007의 게이팅 전제조건이고, 이 노드의 결함이 아닙니다.**
   Config C에서 바뀌는 것은 `--trace-path`를 빼는 것 하나뿐입니다(`run_check.sh:14-16`).

2. **G-R1 결과가 저장소에 로그로 남아 있지 않습니다.** `18/18 checks`, 최악 탐지 지연 `9.1 ms`
   같은 수치는 `README.md §Check it`에 **붙여넣은 형태로만** 존재하고, `gz-split/logs/`처럼
   원본 stdout 파일이 없습니다. `grep -rln "G-R1"` → `README.md`, `run_check.sh` 2건뿐.
   **재실행해 로그 파일로 남겨야 합니다.**

3. **`ras-monitor.service`에 `Environment=ROS_DOMAIN_ID=42`와 게스트 Fast DDS 프로파일이
   없습니다.** 그대로 부팅하면 도메인 0 + SHM으로 떠서 워크로드(도메인 42)와 다른 ROS
   그래프가 됩니다. 배관은 준비했고(`../gz-split/config/host_ras_server.xml`, 42105)
   유닛의 2줄이 비어 있습니다. **읽기 전용 조사이므로 추가하지 않았습니다.**

4. **RERI 오류 코드는 이미 소실된 상태로 도착합니다.** 펌웨어가 12개 RERI 코드를 CPER `0x08`로
   접기 때문에(`opensbi/lib/utils/ras/reri_drv.c:373-405`) `/ras/events`는 **무엇을 주입했는지
   보고할 수 없습니다.** MMIO 레지스터를 되읽으면 주입한 것은 알 수 있지만 기록된 것은 모릅니다.
   (정정요청 D-09)

5. **절대 지연값은 에뮬레이터의 것입니다.** `detect_latency_ms`는 `CLOCK_MONOTONIC` 기준으로
   정직하게 측정되지만 QEMU TCG에는 타이밍 모델이 없습니다. 이 수치는 **회귀 기준선**이고
   RISC-V 하드웨어의 특성 서술이 아닙니다.

6. **kmsg 소스는 tracepoint보다 거칩니다.** 한 하드웨어 오류가 printk 10여 줄이고 상관 id로
   재조립한 뒤 2초 정적 후 발행하므로, 타임스탬프가 설계상 tracepoint보다 늦습니다.
   **카운트 교차검증에만 쓰고 지연 측정에는 쓰지 마십시오.**

7. **`/ras/events`는 VOLATILE입니다.** 이벤트 이후에 시작한 구독자는 그 이벤트를 받지 못합니다.
   **기록은 JSON 로그이고 토픽은 알람입니다.** (`/diagnostics`도 VOLATILE — 참고로 YOLO 쪽
   `yolo_node.py`의 `/diagnostics` 퍼블리셔는 TRANSIENT_LOCAL이고, 제공 durability가
   요청보다 강해도 되므로 두 발행자가 한 토픽에 공존해도 매칭은 성립합니다.)

8. **`.gitignore` 부재.** 아직 `.pyc`가 없지만 이 디렉터리에서 노드를 한 번 실행하면 생깁니다.

9. **커밋·푸시·PR은 하지 않았습니다** — 사용자 몫입니다.
