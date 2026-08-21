#!/bin/bash
# Gate G-R1 -- does a kernel RAS record reach a ROS 2 topic?
#
# Runs tools/check_ras_monitor.py in the guest and reports its verdict. Two
# stages, because they fail for different reasons and only one of them needs a
# ROS graph:
#
#   decode   ras_decode.py --self-test, against the recorded fixtures. Pure
#            Python; a failure here is a parser defect and nothing else.
#   deliver  check_ras_monitor.py. Feeds those same records through a FIFO
#            standing in for trace_pipe and asserts they arrive on /ras/events
#            with the right severity, a measured latency and a JSON log line.
#
# What this cannot show: that the KERNEL emits the event. The guest is Config A
# -- no reri=true, no pflash pair -- so no error can be injected. On Config C the
# only change is dropping --trace-path, and ../README.md has the injection.
#
#   scripts/run_check.sh
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
set -eo pipefail

GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/ras_ros}
DOMAIN=${DOMAIN:-71}

say() { echo "[$(date +%H:%M:%S)] $*"; }
gsh() { ssh -p "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

# A domain of its own. The default 0 would put this test into whatever graph is
# already running, and gz-split uses 42; a check that shares a domain with the
# workload can be perturbed by it and can perturb it back.
ENV="export ROS_DOMAIN_ID=$DOMAIN; source /opt/ros/jazzy/setup.bash;"

say "1/2 decode -- parser against the recorded fixtures"
gsh "cd $GUEST_DIR && python3 ras_decode.py --self-test --samples $GUEST_DIR/samples" \
  | sed 's/^/      /'

say "2/2 deliver -- records to /ras/events, on ROS_DOMAIN_ID=$DOMAIN"
# Stale segments from an earlier run make a fresh participant fail to match, and
# the symptom is a silent topic rather than an error.
gsh "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; true"
gsh "set -o pipefail; cd $GUEST_DIR && $ENV python3 check_ras_monitor.py 2>&1" \
  | sed 's/^/      /'
RC=$?

say "G-R1: $([ $RC -eq 0 ] && echo PASS || echo FAIL)"
exit $RC
