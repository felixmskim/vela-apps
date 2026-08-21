#!/bin/bash
# Gate G-Y6 -- the interface yolo_node.py owes the RAS error-logging daemon.
#
# Runs on the HOST, inside the vela-gz container, in about 40 s. It replaces
# onnxruntime with a stub of the same call signature, so what is under test is the
# node's ROS behaviour and not the model: whether a heartbeat exists, whether a
# failed inference is reported instead of swallowed, and whether `detect_mono` is
# the clock ras_monitor.py stamps its events with. None of that needs riscv64, and
# asserting it on the guest would cost minutes per assertion.
#
# G-Y2/G-Y3 are the gates that need the real runtime. This one is deliberately
# cheap so it can run after every edit to the node.
#
#   scripts/run_node_check.sh
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
set -o pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
CONTAINER=${CONTAINER:-vela-gz}
# Its own domain, and localhost-only: this must not join a running rig, and a
# running rig must not be able to satisfy its assertions for it.
DOMAIN=${DOMAIN:-97}
DIR=/tmp/yolo-node-check

say() { echo "[$(date +%H:%M:%S)] $*"; }

docker inspect "$CONTAINER" >/dev/null 2>&1 || {
  say "FAIL: no container '$CONTAINER'. This runs in the x86 ROS container, not"
  say "      the guest -- see the header."; exit 1; }

say "0/1 copying the node and its check into $CONTAINER:$DIR"
docker exec "$CONTAINER" bash -lc "rm -rf $DIR && mkdir -p $DIR" || exit 1
for f in yolo_node.py check_yolo_node.py; do
  docker cp "$ROOT/tools/$f" "$CONTAINER:$DIR/" >/dev/null || {
    say "FAIL: could not copy tools/$f"; exit 1; }
done

# ROS_LOCALHOST_ONLY and no profiles file: the committed Fast DDS profiles declare
# listening ports that a running rig owns, and binding one out from under the
# bridge would break the rig this check is supposed to be independent of.
docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; \
  export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1; \
  unset FASTRTPS_DEFAULT_PROFILES_FILE; \
  cd $DIR && timeout 180 python3 check_yolo_node.py" 2>&1 | grep -v ROS_LOCALHOST_ONLY
RC=$?

if [ "$RC" -eq 0 ]; then say "G-Y6: PASS"; else say "G-Y6: FAIL (rc=$RC)"; fi
exit $RC
