#!/bin/bash
# What must be true before any yolo/ script can do its job. Read-only.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
ART=${ART:-$HOME/ms_kim/yolo-artifacts}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}

say() { echo "[$(date +%H:%M:%S)] $*"; }
ok=0; bad=0
pass() { say "OK   $*"; ok=$((ok+1)); }
fail() { say "MISS $*"; bad=$((bad+1)); }

# Key-based ssh, not the empty password: every script here runs ssh
# non-interactively, and BatchMode cannot type even an empty password.
# One-time fix if this fails: log in once and append ~/.ssh/id_ed25519.pub
# to the guest's ~/.ssh/authorized_keys (see README).
if ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
       -p "$PORT" "$GUEST" true 2>/dev/null; then
  pass "guest ssh ($GUEST:$PORT, key auth)"
else
  fail "guest ssh -- is the VM up (q-vela/run.sh)? is the key installed?"
fi

[ -f "$ART/models/yolov8n.onnx" ] \
  && pass "model $ART/models/yolov8n.onnx" \
  || fail "model missing -- run scripts/export_model.sh"

if ls "$ART"/ort-build/onnxruntime-*.whl >/dev/null 2>&1; then
  pass "runtime wheel $(ls "$ART"/ort-build/onnxruntime-*.whl | head -1)"
else
  RUNNING=$(docker inspect -f '{{.State.Status}}' ort-build 2>/dev/null)
  if [ "$RUNNING" = running ]; then
    fail "wheel not built yet -- ort-build container still $RUNNING"
  else
    fail "no wheel and no build container -- run scripts/build_ort_wheel.sh"
  fi
fi

docker inspect vela-gz >/dev/null 2>&1 \
  && pass "vela-gz container (for make_bag.sh and LIVE=1)" \
  || fail "vela-gz container not found -- bag recording and live camera need it"

ls "$ART"/assets/*.jpg >/dev/null 2>&1 \
  && pass "billboard/test images in $ART/assets" \
  || fail "no images in $ART/assets -- export_model.sh copies them from ultralytics"

say "$ok ok, $bad missing"
exit $((bad > 0))
