#!/bin/bash
# Milestone B — one terminal, start to verdict.
#
# Replaces the two-terminal dance. Cleans stale DDS state, launches the three
# nodes, waits for them to reach active, runs the acceptance test, tears down,
# and prints a verdict. Every phase prints a timestamp, so a slow step is always
# distinguishable from a hung one.
#
#   bash ~/nav2_tcg/run_milestone_b.sh
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

# NOTE: no `set -u`. /opt/ros/jazzy/setup.bash dereferences unset variables
# (AMENT_TRACE_SETUP_FILES among others), so -u kills the script on the source
# line before anything runs.
set -o pipefail
TCG=~/nav2_tcg
STAMP=$(date +%Y%m%d-%H%M%S)
LAUNCH_LOG=~/milestoneB-launch-$STAMP.log
CHECK_LOG=~/milestoneB-check-$STAMP.log

say() { echo "[$(date +%H:%M:%S)] $*"; }

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
export RCUTILS_COLORIZED_OUTPUT=0

cleanup() {
  say "tearing down..."
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "-$LAUNCH_PID" 2>/dev/null
  sleep 5
  pkill -f 'map_server|planner_server|lifecycle_manager|static_transform_publisher' 2>/dev/null
  sleep 2
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
trap cleanup EXIT

# A hard-killed node leaves its shared-memory segment behind and the next run can
# block on it. This presents as "nodes start but never discover each other".
say "clearing stale FastDDS shared memory ($(ls /dev/shm 2>/dev/null | grep -c fastrtps) segments)"
pkill -f 'map_server|planner_server|lifecycle_manager|static_transform_publisher' 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

say "launching (log: $LAUNCH_LOG)"
setsid ros2 launch "$TCG/planner_only.launch.py" > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

say "waiting for 'Managed nodes are active' (up to 300s)"
for i in $(seq 300); do
  if grep -q 'Managed nodes are active' "$LAUNCH_LOG" 2>/dev/null; then
    say "  active after ${i}s"; break
  fi
  if grep -qE 'CRITICAL FAILURE|process has died' "$LAUNCH_LOG" 2>/dev/null; then
    say "  LAUNCH FAILED — tail of $LAUNCH_LOG:"; tail -25 "$LAUNCH_LOG"; exit 1
  fi
  [ "$i" -eq 300 ] && { say "  TIMEOUT. tail:"; tail -25 "$LAUNCH_LOG"; exit 1; }
  sleep 1
done

say "running acceptance test (log: $CHECK_LOG)"
# extra args pass through, e.g. --dump ~/path.csv
python3 "$TCG/check_plan.py" "$@" 2>&1 | tee "$CHECK_LOG"
rc=${PIPESTATUS[0]}

echo
if grep -qE 'CRITICAL FAILURE|process has died' "$LAUNCH_LOG"; then
  say "NOTE: the launch log reports a node failure — inspect $LAUNCH_LOG"
  grep -nE 'CRITICAL FAILURE|process has died|ERROR' "$LAUNCH_LOG" | tail -10
fi
say "Milestone B verdict: $([ "$rc" -eq 0 ] && echo PASS || echo FAIL)   (exit $rc)"
say "logs: $LAUNCH_LOG   $CHECK_LOG"
exit "$rc"
