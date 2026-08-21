#!/bin/bash
# Export yolov8n.onnx on the host and prove the exported graph + our
# pre/post-processing find the right objects BEFORE anything touches riscv64.
# A failure after this point is then an environment problem, never the model.
#
# opset 12: onnxruntime is backward compatible to opset 7, so 12 runs on every
# runtime candidate including the oldest fallback (spacemit 1.15, trixie 1.21).
# imgsz 320, static: D-13 sized the TCG budget at 320x320 (10-60 s per frame);
# 640 would quadruple it for nothing the availability test needs.
set -eo pipefail

VENV=${VENV:-$HOME/ms_kim/venv-yolo}
ART=${ART:-$HOME/ms_kim/yolo-artifacts}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)

say() { echo "[$(date +%H:%M:%S)] $*"; }

if [ ! -x "$VENV/bin/yolo" ]; then
  say "creating venv $VENV (ultralytics + onnx + onnxruntime, several minutes)"
  python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q --upgrade pip
  "$VENV/bin/pip" install -q ultralytics onnx onnxruntime
fi

mkdir -p "$ART/models" "$ART/assets" "$ART/golden"
cp -n "$VENV"/lib/python3*/site-packages/ultralytics/assets/*.jpg "$ART/assets/" 2>/dev/null

say "exporting yolov8n.onnx (opset 12, imgsz 320, static)"
cd "$ART/models"
"$VENV/bin/yolo" export model=yolov8n.pt format=onnx opset=12 imgsz=320 | tail -2

say "golden run on the host (x86 onnxruntime)"
"$VENV/bin/python" "$ROOT/tools/detect.py" \
    --model "$ART/models/yolov8n.onnx" --source "$ART/assets" \
    --json "$ART/golden/host-x86.json" --expect person=3,bus=1

say "model and golden reference ready under $ART"
