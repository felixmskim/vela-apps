# Take 11 인계 노트 — 2026-08-20 Take 5~10 기록

작성 2026-08-22. **RUNLOG.md는 2026-08-19 23:50(Take 4)에서 멈춰 있고, 08-20 작업은
그 어디에도 기록되지 않았다.** 이 파일은 그 공백을 메우기 위한 임시 인계 노트이며,
RUNLOG.md에 정식 편입되기 전까지만 유효하다. (Phase 5 = RUNLOG take5~11 기록은
사용자 go-ahead 대기 상태.)

한국어로 쓴다. 이 저장소의 README/RUNLOG는 영문 하우스 스타일이지만 이것은 인계
메모이므로 예외로 둔다.

## 이 노트의 출처와 신뢰도

**08-20 작업은 이 대화창에서 수행된 것이 아니다.** 메모리 파일
`project-circuit-video-take11.md`의 `originSessionId`가 `2b7270d0-...`이고, 현재
세션은 `7bedc1b2-...`다. 현재 세션의 컨텍스트는 Take 4(08-19 23:50)까지만 남아
있고 그 이후는 압축으로 사라졌다. 따라서 이 노트는 **기억이 아니라 디스크 증거를
재구성한 것**이다.

| 근거 | 위치 | 신뢰도 |
|---|---|---|
| 런 로그 8개 | `logs/p1-cotenancy-20260820.log`, `logs/p2-circuit-validation-20260820.log`, `logs/p4-take{5,6,7,8,9,10}-20260820.log` | **1차 증거** |
| 녹화 로그 6개 | `logs/p4-take{5-shot,5,6,7,9,10}-record.log` | **1차 증거** |
| Take 10 RAS 증거 | `/tmp/claude-1000/-home-cslab-ms-kim/2b7270d0-e323-479f-940a-d0daafa2fbee/scratchpad/take10/{echo,events}.log`, `track-split-20260820-114808.csv` | **1차 증거** (스크래치, 휘발성 — 보존하려면 옮겨야 함) |
| 코드 주석 | `tools/send_goal.py` `settle()` docstring, `scripts/run_split.sh:475-481, 865-892` | **1차 증거** (진단이 코드에 남아 있다) |
| 메모리 파일 | `~/.claude/.../memory/project-circuit-video-take11.md` | **2차** — 로그로 교차검증된 것만 인용하고, 안 되는 것은 "로그 미기록"으로 표시 |

**로그 미기록 목록** (기억/메모리에만 있고 `logs/p4-*.log`에서 확인되지 않음):
- T7 "planner 클럭 동결 sim 135" 의 **sim 135** 라는 수치
- T7/T8/T9의 검출 패스 수 (세 런 모두 `map_saver` 실패로 step 10에 도달하지 못해
  `G-Y5: N detection(s)` 줄이 없다)
- "take7~10 합계 140+ 검출, person:0.88×2" — 게스트 `~/yolo-node-*.log`에만 있었고
  VM이 꺼져 있어 현재 확인 불가
- 사용자가 각 회차를 실제로 어떤 문자열로 호출했는지 — `run_split.sh`는 자기 호출
  명령을 에코하지 않는다. 아래 표의 명령은 **로그가 에코한 값에서 역산한 재구성**이다

## 1. Take별 표

`p1`/`p2`는 Take 번호가 붙지 않았지만 같은 밤의 선행 검증이므로 함께 싣는다.
모든 런 공통: `ROBOT=tb3`, `GUI=1`, `YOLO=1`, `WATCHDOG=`(비움), 측정 RTF 0.2500.

| 회차 | 실행 명령 (로그에서 역산) | 결과 | 실패 원인 | 다음 회차에 바꾼 것 |
|---|---|---|---|---|
| **p1** 00:06–00:18 | `YOLO=1 WATCHDOG= GUI=1 ROBOT=tb3 scripts/run_split.sh` (웨이포인트 오버라이드 없음 → `robot.env`의 기본 2점) | **PASS** 2/2 wp · p/r/c **0.995 / 0.988 / 0.983** · 검출 11 · CPU 279%/400% | — | 도로 주행으로 가기 위해 8점 링 경로 도입 |
| **p2** 00:24–00:37 | 위 + `WAYPOINTS_OVERRIDE="-1.52,-1.52 0,-1.90 1.52,-1.52 1.80,0 1.52,1.52 0,1.85 -1.52,1.52 -2.00,0"` | **PASS** 8/8 wp · **1.000 / 1.000 / 0.999** · 검출 13 · CPU 276% | — | 경로 자체는 검증 완료 → 녹화 착수 |
| **Take 5** 01:42–02:01 | p2 + `RTF_LIMIT=0.25`(명시) `PRE_DRIVE_SLEEP=300` `HOLD=1`; 녹화 `RVIZ=1 OVERLAY=1 --duration 420` (01:52 리허설 `--shot` 먼저) | **FAIL** **0/8 wp** · coverage **0.138** LOW · 검출 19 · CPU 303% | 전 wp `final (0.00,-0.00)`, 4~11 s만에 `status=6`. 로봇이 아예 움직이지 않음 → bt_navigator/planner가 자기 TF 버퍼보다 뒤처진 상태에서 첫 골 수신. 부수적으로 `/cmd_vel inter-arrival: NO MESSAGES SEEN` | `PRE_DRIVE_SLEEP` 300→180 |
| **Take 6** 02:04–02:26 | Take 5 + `PRE_DRIVE_SLEEP=180`; 녹화 `--duration 420` | **PASS** 8/8 wp · **1.000 / 1.000 / 1.000** · 검출 18 · CPU 291% · 실주행 **12.287 m** · mp4 12 MB 남아 있음 | — | **사용자 지적**: 아레나 밖 빌보드 사진을 검출하고 링만 돌아 과제에 안 어울린다 → 로봇 탑재 카메라 + 실장면 + 기둥 사이 슬라럼으로 전면 재작업 |
| **Take 7** 03:13–03:34 | `YOLO=1 YOLO_CAM=robot YOLO_RATE=0.25 WATCHDOG= GUI=1 ROBOT=tb3 PRE_DRIVE_SLEEP=180 WAYPOINTS_OVERRIDE="<슬라럼 8점>"`; 녹화 `--duration 450` | **FAIL** **0/8 wp** · `FAIL: map_saver` · 검출 수 **로그 미기록** | 전 wp `final (0.00,-0.00)`, 20~26 s `status=6`. 지도가 자라지 않아 `map_saver`가 `Failed to spin map subscription`. Take 5와 동일 계열 | 카메라 레이트 0.25→**0.1** Hz (게스트 CPU 확보) |
| **Take 8** 03:36–03:51 | Take 7 + `YOLO_RATE=0.1`; **녹화 로그 없음** | **FAIL** **0/8 wp** · `FAIL: map_saver` · 검출 수 **로그 미기록** | 동일. wp가 **4~15 s**만에 abort — 실주행이라면 불가능한 속도. `send_goal.py` `settle()` docstring에 이 런의 측정치가 남아 있다: **wp1에서 16.6 s 뒤처짐 → wp8에서 2.7 s**, 즉 abort 자체가 클럭을 갚는다 | 이 관찰을 코드로: `send_goal.py --settle N` 신설 (제자리 골 프로브로 펌프+게이트), `run_split.sh`에 `SETTLE` knob 추가 |
| **Take 9** 03:55–04:31 | Take 8 + `SETTLE=12`; 녹화 `--duration 480` | **FAIL** **0/8 wp** · `FAIL: map_saver` · 검출 수 **로그 미기록** | **프로브 12개 전부 `goal rejected`** — 각 정확히 60 s(골 전송 타임아웃). Take 7/8과 **다른 고장**: 수락 후 abort가 아니라 액션 서버가 수락 자체를 안 함. 이어진 wp 8개도 각 60 s 소진 후 MISSED. 녹화 로그에 `WARNING: no /map in 60s` | VM 재부팅 (대화형 → 사용자 수행) |
| **Take 10** 11:48–12:08 | Take 9와 동일 (`SETTLE=12`, `PRE_DRIVE_SLEEP=180`, `HOLD=1`); 녹화 `--duration 480` | **부분 성공 / 판정 FAIL** **3/8 wp** (**wp6·7·8**) · p/r/c **0.790 LOW / 0.954 / 0.624 LOW** · 검출 19 · 실주행 **4.723 m** · **RAS 완전 성공** | 프로브 12개가 이번엔 응답(1~93 s, 전부 `status=6`)했지만 소진, 이어 wp1~5 MISSED, 그 뒤 wp6(15 s)·wp7(39 s)·wp8(42 s) **SUCCEEDED**. 즉 클럭 부채는 갚혔지만 **프로브 12개로는 부족**했다. `map_rc=1`은 부분 주행의 당연한 결과 | **Take 11: `SETTLE` 12→30, `--duration` 480→1500** |

### Take 10에서 확인된 두 가지 중요한 사실

**클럭 게이트 통과가 주행 성공을 보장하지 않는다.** Take 10의 step 7은
`host sim 96s guest clock 95.65s lag 0.3s` / `inside CLOCK_LAG_MAX=5s` 로 **정상**이었다.
그런데도 프로브 12개와 wp 5개가 실패했다. `run_split.sh`의 클럭 게이트는
`clock_lag_probe.py`가 보는 전역 지연이고, 실패하는 것은 **bt_navigator 자기 노드
시계와 자기 TF 버퍼 사이의 부채**다. 둘은 다른 양이다. 주행 가능성의 유일하게
믿을 수 있는 신호는 **`settle` 프로브가 SUCCEEDED 하는 것**이다.

**RAS 파이프라인은 Take 10에서 완전히 동작했다.** 이벤트 2건이
`/ras/events`로 나와 `ros2 topic echo`로 캡처됐다:

| | seq 1 | seq 2 |
|---|---|---|
| `detect_mono` | 1159.617174 | 1396.819988 |
| `trace_ts` | 1159.533795 | 1396.814335 |
| `detect_latency_ms` | **83.379** | **5.653** |
| 시점 | 로봇 **주행 중** (replay-1) | HOLD 중 (replay-2) |

둘 다 `ras/trace`, level `\x01`(WARN), `ghes_severity_name: corrected`,
`error_type_name: micro-architectural error`, `operation_name: data write`,
`proc_isa_name: RISCV64`, `hardware_id: vela-guest`, `task: reri-inject`,
`sec_type_known: true`, `declared_len == decoded_len == 192`.

**이 이벤트는 replay다. 실제 주입이 아니다.** 이 게스트는 Config A(`reri=true` 없음,
EDK2 pflash 없음)이므로 주입이 물리적으로 불가능하다. 게스트의
`~/ras_demo/replay.py`가 기록된 CPER를 프로덕션 경로(tracefs → ras_monitor →
`/ras/events`)로 흘려보낸 것이다. **영상에 "REPLAYED" 자막이 없으면 실제 주입을
암시하게 되므로 촬영 중단 사유다** (6절 참조).

## 2. Take 10에서 실제로 사용한 환경변수 전문

`run_split.sh`는 자기 호출 명령을 에코하지 않는다. 아래는 **로그가 출력한 값에서
역산**한 것이며, 각 항목마다 근거 줄을 붙였다.

```
YOLO=1 \
YOLO_CAM=robot \
YOLO_RATE=0.1 \
SETTLE=12 \
WATCHDOG= \
GUI=1 \
ROBOT=tb3 \
HOLD=1 \
PRE_DRIVE_SLEEP=180 \
WAYPOINTS_OVERRIDE="-0.55,-0.55 0.55,-0.55 1.70,0.00 0.55,0.55 -0.55,0.55 -1.70,0.00 0.00,-1.70 -2.00,-0.50" \
scripts/run_split.sh > logs/p4-take10-20260820.log 2>&1
```

그리고 `holding 180s`가 뜬 뒤 별도 셸에서:

```
RVIZ=1 OVERLAY=1 scripts/run_demo.sh --record --duration 480
```

| 변수 | 값 | 근거 (p4-take10-20260820.log) |
|---|---|---|
| `ROBOT` | `tb3` | `1/10 host: world + robot for tb3` |
| `GUI` | `1` | `GUI=1 -- generated guest_tcp_client_demo.xml (adds 42103 rviz2, 42104 overlay, 42105 ras_monitor)` |
| `YOLO` | `1` | `YOLO=1 (robot camera) -- merging the camera bridge entry` |
| `YOLO_CAM` | `robot` | 같은 줄. 그리고 `robot : RGB camera on base_link -> gz topic /camera, 320x240 @ 0.1 Hz sim` |
| `YOLO_RATE` | `0.1` | 위 줄의 `@ 0.1 Hz sim` (Take 7은 같은 자리에 `0.25 Hz`) |
| `RTF_LIMIT` | **미지정** → 자동 0.25 | `RTF_LIMIT was unset or 0, so it is 0.25 -- three workloads do not fit faster`. 실측 `real_time_factor during the run = 0.2499991687527639` |
| `WATCHDOG` | 비움 | `liveliness not recorded (WATCHDOG= )` |
| `PRE_DRIVE_SLEEP` | `180` | `holding 180s before driving so a recorder can attach` |
| `SETTLE` | `12` | `[wpsettle-1]` … `[wpsettle-12]` 후 `settle: NOT healthy after 12 probes` |
| `HOLD` | `1` | `HOLD=1 -- the stack stays up so a recording can outlive the verdict.` |
| `WAYPOINTS_OVERRIDE` | 슬라럼 8점 | `WAYPOINTS_OVERRIDE in effect: -0.55,-0.55 0.55,-0.55 1.70,0.00 0.55,0.55 -0.55,0.55 -1.70,0.00 0.00,-1.70 -2.00,-0.50` |
| `--duration` | `480` | `p4-take10-record.log`: `recording for 480s, then stopping on its own` |
| `FOLLOW_OFFSET_OVERRIDE` | **미지정** | `p4-take10-record.log`: `following tb3, camera offset (-1.0, 0, 0.45) m` — `robot.env`의 tb3 기본값. Take 5~10 전 회차 동일 |
| `MAP_OFFSET` (스폰) | `-2,-0.5` | `world=default bridge=... base=base_footprint offset=-2,-0.5` |
| `MIN_COVERAGE` 외 | 미지정 | `map gates: --min-coverage 0.9 (unset floors come from score_map.py)` |

참고 — 회차별로 달랐던 것만:

| | RTF_LIMIT | YOLO_CAM / RATE | PRE_DRIVE_SLEEP | SETTLE | HOLD | --duration |
|---|---|---|---|---|---|---|
| p1 | 자동 0.25 | scene / 0.1 | 없음 | 없음 | 없음 | 녹화 없음 |
| p2 | 자동 0.25 | scene / 0.1 | 없음 | 없음 | 없음 | 녹화 없음 |
| T5 | **명시 0.25** | scene / 0.1 | **300** | 없음 | 1 | 420 (+ `--shot` 리허설) |
| T6 | **명시 0.25** | scene / 0.1 | 180 | 없음 | 1 | 420 |
| T7 | 자동 0.25 | **robot / 0.25** | 180 | 없음 | 없음 | 450 |
| T8 | 자동 0.25 | robot / **0.1** | 180 | 없음 | 없음 | **녹화 로그 없음** |
| T9 | 자동 0.25 | robot / 0.1 | 180 | **12** | 없음 | 480 |
| T10 | 자동 0.25 | robot / 0.1 | 180 | **12** | **1** | 480 |

`RTF_LIMIT` 명시/자동 구분 근거: `run_split.sh:56-62`의 자동 하향 블록은 값이
주어지지 않았을 때만 `RTF_LIMIT was unset or 0` 줄을 찍는다. T5/T6 로그에는 그
줄이 없다.

## 3. 슬라럼 웨이포인트 좌표와 도출 방법

```
WAYPOINTS_OVERRIDE="-0.55,-0.55 0.55,-0.55 1.70,0.00 0.55,0.55 -0.55,0.55 -1.70,0.00 0.00,-1.70 -2.00,-0.50"
```

**전부 world 좌표다.** 8점의 의미:

| # | 좌표 | 왜 그 값인가 |
|---|---|---|
| 1–2 | `-0.55,-0.55` `0.55,-0.55` | `turtlebot3_world`의 기둥 9개는 반지름 0.15 m, 중심이 x,y ∈ {−1.1, 0, +1.1}. **±0.55는 인접한 두 기둥 중심의 정확한 중점** — 즉 틈의 정중앙. 틈 폭 = 1.1 − 2×0.15 = **0.80 m**, `robot_radius 0.22`이므로 편측 여유 **0.18 m** |
| 3 | `1.70,0.00` | 기둥 밭 바깥(외곽 기둥 표면 1.1+0.15 = 1.25) 이면서 자유 영역 경계(약 2.7) 안쪽 |
| 4–5 | `0.55,0.55` `-0.55,0.55` | 다시 틈 정중앙, 반대쪽 대각 |
| 6 | `-1.70,0.00` | 3번의 반대편 |
| 7 | `0.00,-1.70` | 중앙 열을 관통해 남쪽 링으로 |
| 8 | `-2.00,-0.50` | **스폰 지점** = `offset=-2,-0.5`. 루프를 닫아 출발점 복귀 |

즉 1·2·4·5는 기둥 사이를 지그재그로 통과하고 3·6·7은 링으로 빠져나온다. 링만
도는 p2/T5/T6 경로(`-1.52,-1.52 0,-1.90 1.52,-1.52 1.80,0 1.52,1.52 0,1.85
-1.52,1.52 -2.00,0`)와 대비되는 지점이 이것이고, 사용자가 요구한 "기둥 사이
슬라럼"이 바로 1·2·4·5다.

실주행 거리 근거 (`track-*.csv`를 좌표 차분 합으로 직접 계산):
- Take 6 링 8점 완주 = **12.287 m**
- Take 10 부분 주행(wp6·7·8만) = **4.723 m**, x 0.00~1.81 / y −1.15~0.28

### map ↔ world 부호 문제 — 이것이 함정이다

`send_goal.py`는 **world 좌표를 받아** `--world-offset=$SPAWN_X,$SPAWN_Y`
(= `-2,-0.5`)를 **빼서** map 좌표를 만든다. 따라서:

```
map = world − (−2.0, −0.5) = world + (2.0, +0.5)
world = map + (−2.0, −0.5)
```

**로그 3곳에서 독립적으로 확인된다:**

1. settle 프로브: `[wpsettle-1] goal world (-2.00, -0.50) -> map (0.00, -0.00)`
2. Take 6 wp4: 목표 world `1.80,0` → 도착 `final (3.80, 0.25) error 0.250`
   (= map `3.80, 0.50`)
3. `score_map.py`: `offset: SLAM + (-2.00, -0.50) -> world`

**왜 함정인가:** `WAYPOINTS_OVERRIDE`는 world인데 RViz·`map_saver`·`score_map`이
쓰는 것은 map이다. 부호를 뒤집으면 (2.0, 0.5)만큼, 즉 **아레나 한 사분면만큼**
어긋난다. 그 정도 오차는 좌표가 여전히 아레나 안에 있어서 "그럴듯해 보인다".
이전 세션에서 내 map→world 부호 추론이 주행 로그와 어긋났고, 그래서
`[handoff]road-driving-demo-prompt-20260819.md`에는 절대 좌표를 넣지 않고 도출
방법만 남겼다. 위 3개 로그 줄이 그 부호를 확정했으므로 이제 좌표를 쓸 수 있다.

**검증 습관:** 새 웨이포인트를 쓰면 첫 골의 `goal world (a, b) -> map (c, d)`
줄에서 `c = a + 2.0`, `d = b + 0.5`인지 눈으로 확인할 것. 한 줄이면 끝난다.

## 4. Take 11에서 바꾸기로 한 것과 근거

| 바꾸는 것 | Take 10 | Take 11 | 근거 |
|---|---|---|---|
| `SETTLE` | 12 | **30** | T10은 프로브 12개를 **다 쓰고도** 부족했다(`settle: NOT healthy after 12 probes`, 이어 wp1~5 MISSED, wp6부터 성공). `send_goal.py` `settle()` docstring: 서브된 abort 1건이 갚는 클럭은 **~1.5–2 sim-s**. 부채 규모는 **~25–30 sim-s** (이 수치는 **로그 미기록**, 메모리 출처). 12 × 1.75 ≈ 21 sim-s로는 모자라고 30 × 1.75 ≈ 52 sim-s면 덮는다 |
| `--duration` | 480 | **1500** | 프로브 30개가 T10의 최악 케이스(93 s)까지 가면 투어 시작 전에만 최대 ~12분이 지나간다. 480 s는 settle 중간에 끝난다. 1500 s로 덮고 **후반 트리밍 전제** |
| 나머지 | | **전부 유지** | 슬라럼 8점, `YOLO_CAM=robot`, `YOLO_RATE=0.1`, `RTF_LIMIT` 미지정(자동 0.25), `PRE_DRIVE_SLEEP=180`, `GUI=1`, `HOLD=1`, `WATCHDOG=`, `ROBOT=tb3`, `FOLLOW_OFFSET_OVERRIDE` 미지정 |

Take 11 명령 (메모리의 재개 절차 그대로, `SETTLE=30`):

```
cd ~/ms_kim/riscv-vela/vela-apps/ras/gz-split && \
YOLO=1 YOLO_RATE=0.1 SETTLE=30 WATCHDOG= GUI=1 ROBOT=tb3 HOLD=1 PRE_DRIVE_SLEEP=180 \
WAYPOINTS_OVERRIDE="-0.55,-0.55 0.55,-0.55 1.70,0.00 0.55,0.55 -0.55,0.55 -1.70,0.00 0.00,-1.70 -2.00,-0.50" \
scripts/run_split.sh > logs/p4-take11-<date>.log 2>&1
```

`holding 180s`가 로그에 뜨면 즉시:

```
RVIZ=1 OVERLAY=1 scripts/run_demo.sh --record --duration 1500
```

동시에 게스트에서 `source ~/ras_demo/env.sh` 후 ras_monitor(`--trace-path` FIFO)와
`ros2 topic echo --no-daemon /ras/events`. **게스트 CLI는 항상 `--no-daemon`.**

RAS replay 타이밍: gz `dynamic_pose`로 로봇 실이동을 확인한 뒤 **+50 s**에
`python3 ~/ras_demo/replay.py` (replay-1, 주행 중), 판정 후 HOLD 중에 replay-2.
T10에서 이 순서가 그대로 통했다.

후처리: 원본에서 주행 구간만 트림 → `setpts=0.25*PTS` (계수는 **그 테이크의
`RTF_LIMIT`과 일치해야 한다**) → 납품본
`logs/gz-split-tb3-circuit-yolo-nav2-slam-riscv.mp4` 갱신.

**지도 게이트 주의:** T10 부분 맵은 precision **0.790** (< 0.80)이었다. 전체 랩이면
0.97+ 가 예상되지만(p2·T6가 1.000), 전체 랩에서도 미달하면 원인은 **사람 모델
다리 셀**이다 — 기준 지도에 없는 장애물이므로 SLAM이 맞게 찍어도 false positive로
계산된다. 그 경우 `MIN_PRECISION_OVERRIDE`로 근거를 명시하고 완화를 검토한다.
근거 없이 먼저 낮추지 말 것.

## 5. Take 7/8/9 실패가 재발하면

세 실패는 **증상이 비슷하고 원인이 다르다.** 구분하는 단서가 로그에 있다.

### 먼저 볼 한 줄: `rejected` 인가 `status=6` 인가

```
[wpsettle-3] goal rejected                          <- 서버가 수락조차 안 함  (T9)
[wpsettle-3] finished in 2s status=6 (not SUCCEEDED) <- 수락 후 abort        (T8, T10)
```

이 구분이 대응을 완전히 갈라놓는다.

### T7/T8 계열 — 수락 후 abort (클럭 부채)

**알아채는 법**
- 모든 wp가 `final (0.00, -0.00)` + 0이 아닌 `error` → 로봇이 한 번도 안 움직였다
- wp가 **4~26 s**만에 끝난다. 실주행이라면 leg당 15~42 s가 정상(T6: 23–28 s, T10 성공분: 15/39/42 s)
- `track-*.csv`의 x,y가 전 구간 ~0
- 뒤이어 `FAIL: map_saver` + `[ERROR] map_saver: Failed to spin map subscription`
  — 지도가 안 자라서 `/map`이 안 나온 결과이고, 원인이 아니라 증상이다

**무엇을 하는가**
1. `SETTLE`을 늘린다. 이것이 정확히 이 고장을 위해 만든 장치다
   (`send_goal.py settle()`). Take 11이 12→30인 이유
2. **step 7의 클럭 게이트가 통과했다고 안심하지 말 것.** T10은 `lag 0.3s`로
   통과하고도 실패했다. 전역 클럭 지연과 bt_navigator 내부 부채는 다른 양이다
3. 증거는 게스트의 `~/split-stack-$STAMP.log`(초기 pose 관련 abort 메시지)와
   `~/clocklag-$STAMP.log`에 있다. **VM이 꺼지면 사라지므로 판정 전에 긁어올 것**
4. settle 프로브가 **SUCCEEDED** 하고도 투어가 실패하면 그것은 새로운 고장이다.
   이 절의 대응을 반복하지 말고 원인을 다시 찾아야 한다

### T9 계열 — 액션 서버 무응답 (수락 안 함)

**알아채는 법**
- `goal rejected`가 반복되고, 각 프로브가 **정확히 ~60 s**씩 걸린다(전송 타임아웃).
  `finished in Ns status=` 줄이 아예 없다
- 녹화 로그에 `WARNING: no /map in 60s -- a split run needs GUI=1 so the guest gets a 42103 locator`

**무엇을 하는가**
1. 게스트에서 `ros2 lifecycle get /bt_navigator` (`active`가 아니면 여기서 끝)
2. `ros2 action list | grep navigate_to_pose` — 액션 서버가 존재하는지
3. 없거나 `inactive`면 SETTLE을 늘려도 무의미하다. T9의 실제 해결책은
   **VM 재부팅**이었다(→ T10). 재부팅은 sudo/QMP가 필요해 **사용자만 할 수 있다**
   (`env-qvela-reset-boundary`). 진단만 하고 요청할 것
4. 재부팅 전에 SHM을 지운다 — `sem.fastrtps_*`까지 포함해서

### 공통: 재발 여부 자체를 오해하지 않기

Take 6은 **성공했지만 요행이었다.** 같은 명령의 Take 5가 0/8이었고 Take 7~9도
0/8이다. 즉 이 부채는 재부팅과 무관한 **런 자체의 속성**이다. 한 번 성공했다고
설정이 안정화된 것으로 읽지 말 것. (이 판단은 메모리 출처이나, T5·T6가 동일 명령에
서로 다른 결과를 낸 것은 로그로 확인된다.)

## 6. 촬영 중 "중단하고 다시 찍는다" 신호

모두 로그에서 실제로 관측된 문자열이다. 하나라도 뜨면 녹화를 계속할 이유가 없다.

**주행이 안 된다 — 정지 화면을 찍고 있다**
- `settle: NOT healthy after N probes` → T9(0/8), T10(3/8) 모두 이 줄 뒤에 실패했다.
  **이 줄이 뜨면 그 테이크는 버린다**
- `[wp1] final (0.00, -0.00)` → 로봇이 안 움직였다
- wp가 15 s보다 빨리 `status=6`으로 끝난다 → 실주행이 아니다
- `[wpsettle-N] goal rejected` → 액션 서버가 죽었다. 녹화를 태우지 말 것

**화면에 필요한 것이 없다** (`*-record.log`에서)
- `WARNING: no /map in 60s` → RViz 패널이 테이크 내내 빈 회색이 된다 (T9·T10에서 실제로 발생)
- `'YOLO detections' -> 640x480+16+584` 줄이 **없다** → PiP 창이 배치되지 않았고 검출기가 안 보인다
- `/scan reaches the viewer` 줄이 없다 → RViz가 아무것도 못 그린다
- 주행 시작 시점까지 게스트 로그에 `[detect]` 줄이 하나도 없다 → PiP가 테이크
  내내 회색 placeholder다 (`detection_overlay.py`의 `placeholder()`)

**배관이 끊겼다**
- `/cmd_vel inter-arrival: NO MESSAGES SEEN.` → T5에서 발생. 워치독/브리지가 눈이 멀었다
- 게스트 yolo 로그의 `[detect-error]`, 또는 `/diagnostics`의 `state=failed`
- `real_time_factor during the run`이 0.25에서 크게 벗어남 → 후처리
  `setpts=0.25*PTS` 계수가 안 맞아 영상 속도가 거짓이 된다

**정직성 — 이건 기술 실패가 아니라 납품 불가 사유다**
- RAS 패널에 이벤트가 뜨는데 화면에 **"REPLAYED" 자막이 없다.** 이 게스트는
  Config A이고 실제 주입은 불가능하다. 자막 없는 영상은 하지 않은 주입을
  했다고 주장하는 것이 된다. **무조건 중단**

## 남은 일

1. **Take 11 촬영** — 리그는 완전 유휴 상태로 정리되어 있고, 위 4절 명령 그대로
   재개하면 된다. VM 부팅과 대화형 조작은 사용자 몫
2. **Phase 5** (사용자 go-ahead 대기): RUNLOG.md에 take 5~11 정식 기록,
   `README.md` 2건 갱신, TSP 시사점 정리 — CR-16 + 신규 발견 4건. **이 파일은
   그 작업이 끝나면 삭제 대상이다**
3. **스크래치 증거 보존**: Take 10의 RAS 증거 3개 파일이
   `/tmp/claude-1000/.../2b7270d0-.../scratchpad/take10/`에 있다. `/tmp`이므로
   재부팅에 사라진다. RUNLOG에 인용하려면 먼저 옮겨야 한다

관련: `RUNLOG.md`(Take 4까지), `README.md` "Recording it with the detector",
`../yolo/README.md` "What the RAS error-logging daemon reads",
`../ras-ros/README.md`, `docs/YOLO_STATUS.md`(2026-08-22 읽기 전용 감사).
