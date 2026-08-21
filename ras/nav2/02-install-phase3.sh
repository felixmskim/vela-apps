#!/bin/bash
# Nav2 on riscv64 — Phase 3. Everything Milestone A (NavigateToPose) needs.
#
# Installs an EXPLICIT package list, not the ros-jazzy-navigation2 metapackage.
# Measured on this guest:
#     ros-jazzy-navigation2   ->  28 upgraded + 676 new = 704 packages
#     this list               ->  26 upgraded + 267 new = 293 packages
# The 411-package difference is nav2-rviz-plugins pulling rviz-ogre-vendor,
# rviz-default-plugins, mesa, va-driver-all, vdpau-driver-all and friends. There
# is no ros-jazzy-rviz2 executable for riscv64 and the guest runs -nographic, so
# none of it can ever be used. Under TCG, dpkg unpacking dominates wall time, so
# package count is the cost that matters.
#
# Then source-builds the two packages the riscv64 repo does not ship. Both are
# ament_python, so no compiler runs.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

# No `set -u`: /opt/ros/jazzy/setup.bash dereferences unset variables.
LOG=~/nav2-phase3-$(date +%Y%m%d-%H%M%S).log
exec > >(tee "$LOG") 2>&1
say() { echo "[$(date +%H:%M:%S)] $*"; }
say "Phase 3 start   log: $LOG"
df -h / | awk 'NR==2{print "disk before: "$4" free"}'

NAV2="ros-jazzy-nav2-amcl ros-jazzy-nav2-behavior-tree ros-jazzy-nav2-behaviors
 ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-controller ros-jazzy-nav2-core
 ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-dwb-controller ros-jazzy-nav2-lifecycle-manager
 ros-jazzy-nav2-map-server ros-jazzy-nav2-msgs ros-jazzy-nav2-navfn-planner
 ros-jazzy-nav2-planner ros-jazzy-nav2-regulated-pure-pursuit-controller
 ros-jazzy-nav2-rotation-shim-controller ros-jazzy-nav2-simple-commander
 ros-jazzy-nav2-smac-planner ros-jazzy-nav2-smoother ros-jazzy-nav2-util
 ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-voxel-grid
 ros-jazzy-nav2-theta-star-planner"
TOOLS="python3-colcon-common-extensions python3-rosdep python3-transforms3d git"

say "installing Nav2 (293 packages) + build tooling"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $NAV2 $TOOLS \
  || { say "APT FAILED"; exit 1; }
sudo apt-get clean
say "apt finished"

# ---- source overlay -------------------------------------------------------
# ros-jazzy-nav2-loopback-sim is in NEITHER binary-riscv64 NOR binary-all.
# It also imports tf_transformations, which has no riscv64 deb either, so this
# is two packages, not one. Both are pure Python.
say "fetching nav2_loopback_sim (sparse checkout, avoids a ~700 MB full clone)"
mkdir -p ~/nav2_ws/src && cd ~/nav2_ws/src
if [ ! -d nav2_sparse ]; then
  git clone --depth 1 --filter=blob:none --sparse -b jazzy \
      https://github.com/ros-navigation/navigation2.git nav2_sparse || { say "CLONE FAILED"; exit 1; }
  (cd nav2_sparse && git sparse-checkout set nav2_loopback_sim)
fi
[ -d tf_transformations ] || git clone --depth 1 https://github.com/DLu/tf_transformations.git

say "compatibility check — the only 1.3.x API loopback_simulator touches"
grep -m1 '<version>' ~/nav2_ws/src/nav2_sparse/nav2_loopback_sim/package.xml
dpkg -s ros-jazzy-nav2-simple-commander | grep '^Version'
source /opt/ros/jazzy/setup.bash
python3 -c 'from nav2_simple_commander.line_iterator import LineIterator; print("  LineIterator import OK")' \
  || { say "LineIterator import FAILED — pin nav2_sparse to tag 1.3.11 and retry"; exit 1; }

say "colcon build (2 ament_python packages, no compiler)"
cd ~/nav2_ws
colcon build --packages-select tf_transformations nav2_loopback_sim --symlink-install \
  || { say "COLCON BUILD FAILED"; exit 1; }

# ---- verify ---------------------------------------------------------------
echo
say "=== Phase 3 exit criteria ==="
source ~/nav2_ws/install/setup.bash
ok=0
chk() { if eval "$2" >/dev/null 2>&1; then printf '  PASS  %s\n' "$1"; else printf '  FAIL  %s\n' "$1"; ok=1; fi; }
chk "bt_navigator present"            "ros2 pkg executables nav2_bt_navigator | grep -q bt_navigator"
chk "controller_server present"       "ros2 pkg executables nav2_controller  | grep -q controller_server"
chk "behavior_server present"         "ros2 pkg executables nav2_behaviors   | grep -q behavior_server"
chk "loopback_simulator present"      "ros2 pkg executables nav2_loopback_sim | grep -q loopback_simulator"
chk "tf_transformations importable"   "python3 -c 'import tf_transformations'"
chk "no rviz2 pulled in (expected)"   "! test -x /opt/ros/jazzy/lib/rviz2/rviz2"
df -h / | awk 'NR==2{print "  disk after : "$4" free"}'
echo
[ "$ok" -eq 0 ] && say "VERDICT: PASS -> Milestone A can run" || say "VERDICT: FAIL"
exit "$ok"
