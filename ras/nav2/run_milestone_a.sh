#!/bin/bash
# Milestone A — one terminal, start to verdict.
#
#   bash ~/nav2_tcg/run_milestone_a.sh
#
# NOTE: no `set -u`. /opt/ros/jazzy/setup.bash dereferences unset variables.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

set -o pipefail
TCG=~/nav2_tcg
STAMP=$(date +%Y%m%d-%H%M%S)
LAUNCH_LOG=~/milestoneA-launch-$STAMP.log
CHECK_LOG=~/milestoneA-check-$STAMP.log

say() { echo "[$(date +%H:%M:%S)] $*"; }

source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash      # nav2_loopback_sim + tf_transformations
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
export RCUTILS_COLORIZED_OUTPUT=0

NODES='map_server|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager|loopback_simulator|static_transform_publisher'

cleanup() {
  say "tearing down..."
  pkill -f '[s]eed_pose.py' 2>/dev/null
  pkill -f "$NODES" 2>/dev/null
  sleep 3
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
}
trap cleanup EXIT

say "clearing stale FastDDS shared memory ($(ls /dev/shm 2>/dev/null | grep -c fastrtps) segments)"
pkill -f "$NODES" 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

say "launching 5 lifecycle nodes + loopback simulator (log: $LAUNCH_LOG)"
setsid ros2 launch "$TCG/loopback_nav.launch.py" > "$LAUNCH_LOG" 2>&1 &

# Break the bringup deadlock: global_costmap activation blocks on map->base_link,
# which loopback_simulator only publishes after it receives /initialpose. Seed it
# from launch time, not after activation.
say "seeding /initialpose from launch time (breaks the activation deadlock)"
setsid python3 "$TCG/seed_pose.py" -1.88 0.08 0.5 > ~/seed_pose.log 2>&1 &

say "waiting for 'Managed nodes are active' (up to 600s)"
for i in $(seq 600); do
  if grep -q 'Managed nodes are active' "$LAUNCH_LOG" 2>/dev/null; then
    say "  active after ${i}s"; break
  fi
  if grep -qE 'CRITICAL FAILURE|process has died' "$LAUNCH_LOG" 2>/dev/null; then
    say "  LAUNCH FAILED — tail of $LAUNCH_LOG:"; tail -30 "$LAUNCH_LOG"; exit 1
  fi
  [ "$i" -eq 600 ] && { say "  TIMEOUT. tail:"; tail -30 "$LAUNCH_LOG"; exit 1; }
  sleep 1
done

# Stop seeding NOW. Each further /initialpose re-derives map->odom so that
# map->base_link equals the seeded pose — during navigation that teleports the
# robot back to the start and the goal is never reached.
say "stopping the /initialpose seeder before navigation"
pkill -f '[s]eed_pose.py' 2>/dev/null
sleep 3

say "running acceptance test (log: $CHECK_LOG)"
python3 "$TCG/check_navigate.py" "$@" 2>&1 | tee "$CHECK_LOG"
rc=${PIPESTATUS[0]}

echo
if grep -qE 'CRITICAL FAILURE|process has died' "$LAUNCH_LOG"; then
  say "NOTE: the launch log reports a node failure — inspect $LAUNCH_LOG"
  grep -nE 'CRITICAL FAILURE|process has died|\[ERROR\]' "$LAUNCH_LOG" | tail -12
fi
say "Milestone A verdict: $([ "$rc" -eq 0 ] && echo PASS || echo FAIL)   (exit $rc)"
say "logs: $LAUNCH_LOG   $CHECK_LOG"
exit "$rc"
