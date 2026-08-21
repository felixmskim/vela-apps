#!/bin/bash
# Launch the onnxruntime riscv64 wheel build in a detached container and, when
# it finishes, prove the wheel in the SAME container image the guest mirrors:
# a smoke test here is ABI-identical to the guest without booting it.
#
# The build is hours long under qemu-user (Debian's real riscv64 buildd takes
# 5-6.5 h); start it, go do something else, and re-run this script with
# CHECK=1 to poll.
set -eo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
ART=${ART:-$HOME/ms_kim/yolo-artifacts}
IMG=${IMG:-ubuntu-riscv64:24.04}   # docker pull --platform linux/riscv64 ubuntu:24.04, retagged

say() { echo "[$(date +%H:%M:%S)] $*"; }

mkdir -p "$ART/ort-build"

if [ "$CHECK" = 1 ]; then
  say "container: $(docker inspect -f '{{.State.Status}}' ort-build 2>/dev/null || echo absent)"
  docker logs ort-build 2>&1 | tail -5
  ls -la "$ART"/ort-build/*.whl 2>/dev/null || say "no wheel yet"
  exit 0
fi

if ls "$ART"/ort-build/onnxruntime-*.whl >/dev/null 2>&1; then
  say "wheel already present: $(ls "$ART"/ort-build/onnxruntime-*.whl)"
else
  docker image inspect "$IMG" >/dev/null 2>&1 || {
    say "pulling riscv64 base image (binfmt qemu-riscv64 must be registered)"
    docker pull --platform linux/riscv64 ubuntu:24.04
    docker tag ubuntu:24.04 "$IMG"
    docker pull --platform linux/amd64 ubuntu:24.04 >/dev/null   # restore the default tag
  }
  docker rm -f ort-build >/dev/null 2>&1 || true
  cp "$HERE/build-in-container.sh" "$ART/ort-build/"
  docker run -d --name ort-build --platform linux/riscv64 \
      -v "$ART/ort-build":/out "$IMG" bash /out/build-in-container.sh
  say "build started -- poll with CHECK=1 $0; expect several hours"
  exit 0
fi

say "smoke test: import + one bus.jpg inference in the riscv64 container"
docker run --rm --platform linux/riscv64 \
    -v "$ART":/art -v "$ROOT/tools":/tools "$IMG" bash -c "
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq && apt-get install -yqq python3 python3-pip python3-numpy \
      python3-opencv >/dev/null 2>&1
  pip install -q --break-system-packages /art/ort-build/onnxruntime-*.whl
  python3 /tools/detect.py --model /art/models/yolov8n.onnx \
      --source /art/assets/bus.jpg --expect person=3,bus=1"
say "G-Y1 PASS: the wheel runs YOLOv8n correctly on riscv64"
