#!/bin/bash
# Nav2 on riscv64 — Phase 1. Curated planner-only install + map/param extraction.
#
# Installs 112 packages (17 upgraded + 95 new), which is everything Milestone B
# needs and nothing more. Deliberately does NOT install:
#
#   ros-jazzy-nav2-bringup   Depends: nav2-minimal-tb3-sim, nav2-minimal-tb4-sim,
#                            ros-gz-bridge, ros-gz-sim, slam-toolbox  -> +345 Gazebo
#                            packages, all useless under -nographic. Its maps and
#                            launch files are extracted from the 31 KB deb instead.
#   ros-jazzy-navigation2    Pulls nav2-rviz-plugins -> rviz/OGRE/mesa/va-driver.
#                            704 packages vs 293 for the same functionality headless.
#                            Phase 3 uses an explicit package list instead.
#
# Run inside the RISC-V guest, ideally under screen.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

set -u
LOG=~/nav2-install-$(date +%Y%m%d-%H%M%S).log
exec > >(tee "$LOG") 2>&1
echo "=== Phase 1 install   $(date '+%F %T')   log: $LOG"

df -h / | awk 'NR==2{print "disk before: "$4" free"}'

# --- 1. Suppress man-db/mandb triggers -------------------------------------
# dpkg's man-db trigger rebuilds the manual index after every package. Under TCG
# that costs more wall time than the unpacking itself.
echo '--- suppressing man-db triggers (TCG cost, not correctness)'
echo 'path-exclude=/usr/share/man/*' | sudo tee /etc/dpkg/dpkg.cfg.d/99-no-man >/dev/null
sudo dpkg-divert --local --rename --add /usr/bin/mandb >/dev/null 2>&1 || true
sudo ln -sf /bin/true /usr/bin/mandb

# --- 2. Extract nav2_bringup data WITHOUT installing it --------------------
echo '--- extracting maps/params/launch from the nav2-bringup deb (no install)'
mkdir -p ~/nav2_data && cd ~/nav2_data
apt-get download ros-jazzy-nav2-bringup
dpkg -x ros-jazzy-nav2-bringup_*.deb ~/nav2_data
SHARE=~/nav2_data/opt/ros/jazzy/share/nav2_bringup
ls -l "$SHARE/maps/" || { echo "FAIL: maps not extracted"; exit 1; }

# --- 3. Curated planner set -------------------------------------------------
echo "--- installing the curated planner set   $(date '+%T')"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-navfn-planner \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-nav2-msgs
echo "--- install finished   $(date '+%T')"

sudo apt-get clean

# --- 4. Verify --------------------------------------------------------------
echo
echo '=== Phase 1 exit criteria ==='
source /opt/ros/jazzy/setup.bash
ok=0
chk() { if eval "$2" >/dev/null 2>&1; then printf '  PASS  %s\n' "$1"; else printf '  FAIL  %s\n' "$1"; ok=1; fi; }
chk "nav2_planner provides planner_server"      "ros2 pkg executables nav2_planner    | grep -q planner_server"
chk "nav2_map_server provides map_server"       "ros2 pkg executables nav2_map_server | grep -q ' map_server'"
chk "nav2_lifecycle_manager present"            "ros2 pkg executables nav2_lifecycle_manager | grep -q lifecycle_manager"
chk "nav2_simple_commander importable"          "python3 -c 'from nav2_simple_commander.robot_navigator import BasicNavigator'"
chk "tb3_sandbox map extracted"                 "test -f $SHARE/maps/tb3_sandbox.yaml"
chk "nav2_params.yaml extracted"                "test -f $SHARE/params/nav2_params.yaml"
df -h / | awk 'NR==2{print "  disk after : "$4" free"}'
echo
[ "$ok" -eq 0 ] && echo "VERDICT: PASS -> proceed to Phase 2 (Milestone B)" \
                || echo "VERDICT: FAIL -> see failures above"
exit "$ok"
