#!/bin/bash
# Put everything the detector needs into the guest: apt deps, the onnxruntime
# wheel, the model, and tools/. Idempotent -- run it again after changing any
# tool and only the copies happen.
set -eo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
ART=${ART:-$HOME/ms_kim/yolo-artifacts}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/yolo}

say() { echo "[$(date +%H:%M:%S)] $*"; }
gsh() { ssh -p "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

WHEEL=$(ls "$ART"/ort-build/onnxruntime-*.whl 2>/dev/null | head -1)
[ -n "$WHEEL" ] || { say "FAIL: no wheel under $ART/ort-build -- build_ort_wheel.sh first"; exit 1; }

say "1/4 apt dependencies (numpy, opencv for I/O, cv_bridge, vision_msgs, rosbag2)"
# python3-opencv is 4.6 and that is fine: tools/ use cv2 for image I/O only,
# never cv2.dnn (which cannot load YOLOv8 graphs at 4.6).
gsh "sudo apt-get install -y -qq python3-numpy python3-opencv python3-pip \
     ros-jazzy-cv-bridge ros-jazzy-vision-msgs \
     ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-sqlite3 >/dev/null && echo apt-ok"

say "2/4 copy wheel + model + tools"
gsh "mkdir -p $GUEST_DIR"
scp -q -P "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no \
    "$WHEEL" "$ART/models/yolov8n.onnx" \
    "$ROOT"/tools/yolov8_onnx.py "$ROOT"/tools/detect.py \
    "$ROOT"/tools/yolo_node.py "$ROOT"/tools/image_publisher.py \
    "$ART"/assets/bus.jpg "$ART"/assets/zidane.jpg \
    "$GUEST:$GUEST_DIR/"

say "3/4 install the wheel (system python, PEP 668 override -- this guest is a"
say "    disposable test image, a venv would only complicate the launch lines)"
gsh "pip install -q --break-system-packages $GUEST_DIR/$(basename "$WHEEL") && echo wheel-ok"

say "4/4 import check"
gsh "python3 -c 'import onnxruntime as o; print(\"onnxruntime\", o.__version__, o.get_available_providers())'"
say "guest ready: $GUEST_DIR"
