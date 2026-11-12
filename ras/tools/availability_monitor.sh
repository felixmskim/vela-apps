#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
#
# availability_monitor.sh - run the TC.10.007 availability window and compute the
# numbers the test procedure asks for.
#
#   sudo ./availability_monitor.sh --duration 3600 --threshold 99.9 \
#        --topics /demo/scan_a /demo/scan_b --outdir /tmp/tc10run
#
# It owns the clock. It does NOT inject - run inject_cpu_ce.sh separately (the test
# procedure wants the injections on their own 12-minute schedule) and point
# --inject-times at the file it tees to.
#
# What it measures, and why each definition is what it is:
#
#   T_inj   when the error was injected. Taken from the injector's own [HH:MM:SS]
#           stamp, not from this script, so the two clocks cannot drift apart.
#   T_det   when the fault record appeared. The daemon writes `timestamp` = the
#           kernel's time for the record, which is the earliest defensible moment.
#   T_rec   `detail.emit_wall_time` = when the daemon finished deciding what to do.
#           The daemon is observe-only, so nothing is actually repaired; this is the
#           response-decision latency, and the report must say so rather than calling
#           it a repair.
#   downtime  sum of monitored-topic gaps over --gap-ms, from downtime_tracker.py.
#
set -u

DURATION=3600
THRESHOLD=99.9
GAP_MS=1000
OUTDIR=.
TOPICS=()
FAULTLOG=/var/log/ras/faults.jsonl
INJECT_TIMES=""
TRACKER=""
TRACKER_USER=ubuntu

while [ $# -gt 0 ]; do
    case "$1" in
        --duration)      DURATION=$2; shift 2 ;;
        --threshold)     THRESHOLD=$2; shift 2 ;;
        --gap-ms)        GAP_MS=$2; shift 2 ;;
        --outdir)        OUTDIR=$2; shift 2 ;;
        --fault-log)     FAULTLOG=$2; shift 2 ;;
        --inject-times)  INJECT_TIMES=$2; shift 2 ;;
        --tracker)       TRACKER=$2; shift 2 ;;
        --tracker-user)  TRACKER_USER=$2; shift 2 ;;
        --topics)        shift; while [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; do TOPICS+=("$1"); shift; done ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "$OUTDIR"
GAPS=$OUTDIR/gaps.csv
ROUNDS=$OUTDIR/rounds.csv
SUMMARY=$OUTDIR/availability.txt
START=$(date +%s.%N)

echo "availability_monitor: start $(date -Is)  duration=${DURATION}s threshold=${THRESHOLD}%"
echo "  fault log : $FAULTLOG"
echo "  topics    : ${TOPICS[*]:-(none - topic continuity will be N/A)}"

# 기존 로그 줄 수를 기억해 이번 창에서 생긴 레코드만 센다
BASELINE=$(wc -l < "$FAULTLOG" 2>/dev/null || echo 0)
echo "  log baseline: $BASELINE lines"

TRACKER_PID=""
if [ ${#TOPICS[@]} -gt 0 ] && [ -n "$TRACKER" ]; then
    # tracker 는 퍼블리셔와 같은 계정으로 돌아야 DDS 데이터가 온다. outdir 은 보통
    # root 소유라 그대로 두면 열기부터 EACCES 로 죽는다 - 쓸 파일을 미리 만들어 넘긴다.
    : > "$GAPS"; : > "$OUTDIR/gaps-summary.json"; : > "$OUTDIR/tracker.log"
    chown "$TRACKER_USER" "$GAPS" "$OUTDIR/gaps-summary.json" "$OUTDIR/tracker.log" 2>/dev/null
    sudo -u "$TRACKER_USER" bash -lc "source /home/ubuntu/ros2_jazzy/install/setup.bash && \
        python3 '$TRACKER' --topics ${TOPICS[*]} --threshold-ms $GAP_MS \
        --duration $DURATION --out '$GAPS' --summary-out '$OUTDIR/gaps-summary.json'" \
        > "$OUTDIR/tracker.log" 2>&1 &
    TRACKER_PID=$!
    echo "  tracker pid: $TRACKER_PID ($TRACKER_USER)"
else
    echo "  tracker   : 미실행 (토픽 미지정)"
fi

trap 'echo "중단됨 - 집계만 수행"; [ -n "$TRACKER_PID" ] && kill "$TRACKER_PID" 2>/dev/null' INT TERM

# 10분마다 진행 상황 한 줄
ELAPSED=0
while [ "$ELAPSED" -lt "$DURATION" ]; do
    sleep 60
    ELAPSED=$((ELAPSED + 60))
    if [ $((ELAPSED % 600)) -eq 0 ]; then
        NOW=$(wc -l < "$FAULTLOG" 2>/dev/null || echo 0)
        NG=$(( $(wc -l < "$GAPS" 2>/dev/null || echo 1) - 1 ))
        printf '[%s] %4ds / %ds  로그 +%s줄  gap %s건\n' \
            "$(date +%T)" "$ELAPSED" "$DURATION" "$((NOW - BASELINE))" "$NG"
    fi
done

[ -n "$TRACKER_PID" ] && { kill "$TRACKER_PID" 2>/dev/null; wait "$TRACKER_PID" 2>/dev/null; }

echo "집계 중 ..."
python3 - "$FAULTLOG" "$BASELINE" "$GAPS" "$ROUNDS" "$SUMMARY" \
         "$START" "$DURATION" "$THRESHOLD" "${INJECT_TIMES:-}" <<'PYEOF'
import csv, json, os, sys, time

faultlog, baseline, gaps_csv, rounds_csv, summary_out, start, duration, threshold, inject_times = sys.argv[1:10]
baseline = int(baseline); start = float(start)
duration = float(duration); threshold = float(threshold)

# --- 이번 창에서 나온 RERI 레코드 ---
recs = []
with open(faultlog, errors="replace") as fp:
    for n, line in enumerate(fp, 1):
        if n <= baseline or '"RERI_FAULT"' not in line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        t = d.get("detail", {})
        recs.append({
            "t_det": float(d["timestamp"]),
            "t_rec": float(t.get("emit_wall_time", d["timestamp"])),
            "code": t.get("error_type_name", "?"),
            "hart": t.get("hart", "?"),
        })
recs.sort(key=lambda r: r["t_det"])

# --- 주입 시각 ---
injs = []
if inject_times:
    import re
    try:
        for line in open(inject_times, errors="replace"):
            m = re.search(r"\[(\d\d):(\d\d):(\d\d)\]", line)
            if m:
                injs.append(m.groups())
    except FileNotFoundError:
        pass

def match_inj(t_det, injs):
    """같은 날짜의 [HH:MM:SS] 중 탐지 직전의 것."""
    lt = time.localtime(t_det)
    best = None
    for hh, mm, ss in injs:
        cand = time.mktime((lt.tm_year, lt.tm_mon, lt.tm_mday,
                            int(hh), int(mm), int(ss), 0, 0, lt.tm_isdst))
        if cand <= t_det and (best is None or cand > best):
            best = cand
    return best

# --- 다운타임 ---
gap_rows = []
try:
    with open(gaps_csv) as fp:
        for row in csv.DictReader(fp):
            gap_rows.append(row)
except FileNotFoundError:
    gap_rows = None
def union_seconds(rows):
    """겹치는 구간을 합집합으로 센다.

    토픽별 공백을 그대로 더하면 두 토픽이 동시에 끊긴 한 번의 장애가 두 번 세어져
    다운타임이 측정 창보다 길어진다. 절차서의 정의는 "토픽 중 하나라도 끊긴 구간"이다.
    """
    iv = sorted((float(r["gap_start"]), float(r["gap_end"])) for r in rows)
    total, cur_s, cur_e = 0.0, None, None
    for s_, e_ in iv:
        if cur_e is None or s_ > cur_e:
            if cur_e is not None:
                total += cur_e - cur_s
            cur_s, cur_e = s_, e_
        else:
            cur_e = max(cur_e, e_)
    if cur_e is not None:
        total += cur_e - cur_s
    return total

if gap_rows is not None and not gap_rows:
    # 헤더조차 없으면 tracker 가 뜨지 못한 것이다 - 0초로 보고하면 거짓이 된다
    import os
    if os.path.getsize(gaps_csv) == 0:
        gap_rows = None

# --- 회차별 표 ---
with open(rounds_csv, "w", newline="") as fp:
    w = csv.writer(fp)
    w.writerow(["round", "T_inj", "T_det", "det_latency_s",
                "T_rec", "mttr_s", "downtime_s", "topics_ok"])
    for i, r in enumerate(recs, 1):
        ti = match_inj(r["t_det"], injs)
        lat = (r["t_det"] - ti) if ti else ""
        dt = union_seconds([g for g in (gap_rows or [])
                            if r["t_det"] <= float(g["gap_start"]) <= r["t_det"] + 60])
        w.writerow([
            i,
            time.strftime("%H:%M:%S", time.localtime(ti)) if ti else "",
            time.strftime("%H:%M:%S", time.localtime(r["t_det"])),
            f"{lat:.3f}" if lat != "" else "",
            time.strftime("%H:%M:%S", time.localtime(r["t_rec"])),
            f"{r['t_rec'] - r['t_det']:.3f}",
            f"{dt:.3f}",
            "N/A" if gap_rows is None else ("yes" if dt == 0 else "no"),
        ])

downtime = union_seconds(gap_rows) if gap_rows else 0.0
avail = (duration - downtime) / duration * 100.0

# 퍼블리셔가 중간에 사라진 창은 고장을 잰 것이 아니다. 그 사실을 숨기고 숫자만
# 내보내면, 워크로드가 스스로 끝났을 뿐인데 가용성 미달로 읽힌다.
lost = []
try:
    import json as _json
    _sum = _json.load(open(os.path.join(os.path.dirname(gaps_csv), "gaps-summary.json")))
    lost = _sum.get("topics_lost_publisher", [])
except Exception:
    pass

lines = [
    "TC.10.007 가용성 집계",
    f"  측정 구간      : {duration:.0f}s ({time.strftime('%H:%M:%S', time.localtime(start))} 시작)",
    f"  주입 시각 수   : {len(injs)}",
    f"  탐지 레코드 수 : {len(recs)}",
    f"  누적 다운타임  : {downtime:.3f}s" + ("" if gap_rows is not None else "  (토픽 미감시 - N/A)"),
    f"  가용성         : (({duration:.0f} - {downtime:.3f}) / {duration:.0f}) x 100 = {avail:.4f}%",
    f"  기준           : {threshold}%  ->  {'충족' if avail >= threshold else '미달'}",
    "",
    "  주의: T_rec 은 데몬이 대응을 결정한 시각(emit_wall_time)이다. 데몬은 observe-only이므로",
    "        실제 복구는 일어나지 않는다. MTTR 은 '대응 결정까지의 시간'으로 읽어야 한다.",
]
if lost:
    lines[0] = "TC.10.007 가용성 집계  ** 측정 무효 **"
    lines += [
        "",
        "  !! 측정 창 도중 퍼블리셔가 사라진 토픽: " + ", ".join(lost),
        "     워크로드가 스스로 끝났는지 확인하라. 그 공백은 고장이 아니므로 위 가용성",
        "     수치를 판정 근거로 쓸 수 없다. 워크로드 수명을 측정 구간보다 길게 잡고 다시 측정한다.",
    ]
text = "\n".join(lines)
print(text)
open(summary_out, "w").write(text + "\n")
PYEOF

echo
echo "결과: $ROUNDS / $SUMMARY / $GAPS"
