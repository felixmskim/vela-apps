#!/bin/bash
# Gate G-Y2: the detector finds the right objects IN THE GUEST, no ROS
# involved. Isolates runtime + model on the real target from everything
# DDS/rosbag adds later. Expect minutes: one 320x320 pass costs tens of
# seconds under TCG (D-13), and bus.jpg + zidane.jpg is two passes.
set -eo pipefail

GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/yolo}

say() { echo "[$(date +%H:%M:%S)] $*"; }
gsh() { ssh -p "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

say "running detect.py on the guest (two images, expect person>=3 bus>=1 total)"
gsh "cd $GUEST_DIR && python3 detect.py --model yolov8n.onnx --source . \
     --json guest-riscv64.json --expect person=3,bus=1"
RC=$?
[ $RC -eq 0 ] && say "G-Y2 PASS" || say "G-Y2 FAIL (rc=$RC)"
exit $RC
