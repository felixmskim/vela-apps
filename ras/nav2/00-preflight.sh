#!/bin/bash
# Nav2 on riscv64 — Phase 0 preflight. Installs NOTHING; only queries apt.
#
# Answers one question: does the Nav2 dependency closure fully resolve on riscv64
# before we spend disk on it? Run inside the RISC-V guest.
#
# Note on rosdep: it is deliberately NOT used here. rosdep's rules are keyed by
# OS + codename only ("ubuntu: noble:") with no architecture axis, so it happily
# prints ros-jazzy-* names that have no riscv64 build. apt-cache policy and
# apt-get install -s are the authoritative tools. rosdep earns its keep only in
# Phase 3, where we actually source-build an overlay.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab

LOG=~/nav2-preflight-$(date +%Y%m%d-%H%M%S).log
exec > >(tee "$LOG") 2>&1

echo "=========================================================="
echo " Nav2 riscv64 preflight   $(date '+%F %T')"
echo " host: $(uname -srm)   log: $LOG"
echo "=========================================================="

fail=0
note() { printf '\n--- %s\n' "$1"; }

note "P0  environment"
echo "arch      : $(dpkg --print-architecture)"
echo "disk free : $(df -h / | awk 'NR==2{print $4" of "$2" ("$5" used)"}')"
echo "RAM       : $(free -h | awk '/^Mem:/{print $2" total, "$7" available"}')"
echo "swap      : $(free -h | awk '/^Swap:/{print $2}')"
echo "ros distro: ${ROS_DISTRO:-<not sourced>}"
echo "ros pkgs  : $(dpkg -l 'ros-jazzy-*' 2>/dev/null | grep -c '^ii')"

# T8 (worklog): the guest was debootstrapped with 'main' only. Many ROS deps
# live in universe. If universe is missing, P4 fails in a way that looks like a
# broken ROS repo but is not.
if grep -Rqs 'universe' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
  echo "universe  : enabled"
else
  echo "universe  : *** NOT ENABLED — see worklog T8, P4 will fail ***"; fail=1
fi

note "P1  falinux riscv64 repository is present in the apt index"
if apt-cache policy 2>/dev/null | grep -q 'vela.falinux'; then
  apt-cache policy 2>/dev/null | grep -B1 -A1 'vela.falinux' | sed 's/^/    /'
  echo "  -> PASS"
else
  echo "  -> FAIL: vela.falinux repo not in apt index. Add:"
  echo "     deb [trusted=yes] http://vela.falinux.com/ubuntu noble main"
  fail=1
fi

note "P2  every package we intend to install has a riscv64 candidate"
NEED="ros-jazzy-navigation2
ros-jazzy-nav2-map-server
ros-jazzy-nav2-planner
ros-jazzy-nav2-costmap-2d
ros-jazzy-nav2-navfn-planner
ros-jazzy-nav2-lifecycle-manager
ros-jazzy-nav2-simple-commander
ros-jazzy-nav2-msgs
ros-jazzy-nav2-bt-navigator
ros-jazzy-nav2-controller
ros-jazzy-nav2-behaviors
ros-jazzy-nav2-smac-planner
ros-jazzy-ompl
ros-jazzy-behaviortree-cpp
ros-jazzy-nav2-bringup
python3-colcon-common-extensions
python3-rosdep
python3-transforms3d"
for p in $NEED; do
  v=$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate:/{print $2}')
  if [ -z "$v" ] || [ "$v" = "(none)" ]; then
    printf '    %-42s %s   <-- MISSING\n' "$p" "${v:-<no such package>}"; fail=1
  else
    printf '    %-42s %s\n' "$p" "$v"
  fi
done

note "P3  confirm the known gaps really are gaps (these SHOULD be missing)"
for p in ros-jazzy-nav2-loopback-sim ros-jazzy-rviz2 ros-jazzy-turtlebot4-navigation; do
  v=$(apt-cache policy "$p" 2>/dev/null | awk '/Candidate:/{print $2}')
  if [ -z "$v" ]; then
    printf '    %-42s absent  (expected — handled in Phase 3)\n' "$p"
  else
    printf '    %-42s %s   <-- unexpectedly PRESENT, plan can be simplified\n' "$p" "$v"
  fi
done

note "P4  full dependency closure  ***  GO / NO-GO GATE  ***"
echo "    This takes 5-15 minutes under TCG emulation. It is NOT hung."
echo "    Started at $(date '+%T'). Resolving ~700 packages is CPU-bound..."
if out=$(apt-get install -s ros-jazzy-navigation2 2>&1); then
  echo "$out" | grep -cE '^Inst ' | xargs printf '    packages to install : %s\n'
  # The summary line precedes the Inst/Conf block, so tail would never show it.
  echo "$out" | grep -E 'upgraded,.*newly installed' | sed 's/^/    /'
  # Anchor at line start. An unanchored 'E: ' matches the 'E: ' inside apt's own
  # "NOTE: This is only a simulation!" banner and reports a false failure.
  if echo "$out" | grep -qE '^E: |Unable to correct|but it is not going to be installed'; then
    echo "    -> FAIL: unresolved dependencies (see above)"; fail=1
  else
    echo "    -> PASS: closure resolves on riscv64"
  fi
  # Confirm the metapackage really drags no Gazebo, which is why we can use it.
  gz=$(echo "$out" | grep -cE '^Inst (ros-jazzy-)?(gz-|ros-gz-)')
  echo "    gazebo packages in closure : $gz   (expected 0)"
  [ "$gz" -ne 0 ] && echo "    -> WARNING: unexpected gz pull-in; re-check before installing"
else
  echo "$out" | tail -20 | sed 's/^/    /'
  echo "    -> FAIL: apt-get install -s returned non-zero"; fail=1
fi
echo "    Finished at $(date '+%T')."

note "P5  what the curated Phase 1 set will cost (still installs nothing)"
CURATED="ros-jazzy-nav2-map-server ros-jazzy-nav2-planner ros-jazzy-nav2-costmap-2d
         ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-lifecycle-manager
         ros-jazzy-nav2-simple-commander ros-jazzy-nav2-msgs"
apt-get install -s --no-install-recommends $CURATED 2>&1 | tail -2 | sed 's/^/    /'

echo
echo "=========================================================="
if [ "$fail" -eq 0 ]; then
  echo " VERDICT: PASS  -> proceed to Phase 1 (curated install)"
else
  echo " VERDICT: FAIL  -> stop. Fix the items marked above first."
fi
echo " log saved: $LOG"
echo "=========================================================="
exit "$fail"
