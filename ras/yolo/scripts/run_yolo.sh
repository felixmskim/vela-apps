#!/bin/bash
# Gate G-Y3 (default): bag replay -> detector -> /detection_result, entirely
# inside the guest. This is the TC.10.007 co-tenant shape CR-12 describes:
# sensor input is a replayed bag, the detector is YOLOv8n on onnxruntime.
#
# Gate G-Y4 (LIVE=1): the demonstration variant. The camera world runs in the
# vela-gz container and frames cross the SLIRP boundary on the gz-split TCP
# profiles. It costs host rendering plus a ~1 MB/s image stream through the
# one TCP path, so it demonstrates the pipeline; the bag variant measures it.
#
#   scripts/run_yolo.sh            # bag replay, asserts a detection arrives
#   LIVE=1 scripts/run_yolo.sh     # live Gazebo camera instead of the bag
#   KEEP=1 ...                     # leave everything running for a demo
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
GZSPLIT=$(cd "$ROOT/../gz-split" && pwd)
ART=${ART:-$HOME/ms_kim/yolo-artifacts}
CONTAINER=${CONTAINER:-vela-gz}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/yolo}
DOMAIN=${DOMAIN:-42}
# First /detection_result must arrive within this budget. One frame is 10-60 s
# under TCG (D-13), plus model load; 420 s tolerates the slow end twice over.
WAIT=${WAIT:-420}
STAMP=$(date +%Y%m%d-%H%M%S)

say() { echo "[$(date +%H:%M:%S)] $*"; }
dex()  { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
dexd() { docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
gsh()  { ssh -p "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

GUEST_PROCS="[y]olo_node.py|[i]mage_publisher.py|[r]os2 bag play"
GUEST_ENV="source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$DOMAIN; \
unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"
# The TCP profile is only for LIVE=1: bag replay is guest-local and the
# default transport is both simpler and cheaper there.
GUEST_ENV_LIVE="$GUEST_ENV export FASTRTPS_DEFAULT_PROFILES_FILE=/home/vela/gz_split/guest_tcp_client.xml;"
HOST_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_tcp_server.xml; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"

cleanup() {
  [ "$KEEP" = 1 ] && { say "KEEP=1 -- leaving the pipeline running"; return; }
  say "tearing down"
  gsh "pkill -f '$GUEST_PROCS'" >/dev/null 2>&1
  docker exec "$CONTAINER" bash -lc \
    "pkill -f '[c]amera_world.sdf|[c]amera_bridge.yaml'" >/dev/null 2>&1
}
trap cleanup EXIT

say "0/4 preflight"
gsh "test -f $GUEST_DIR/yolov8n.onnx && test -f $GUEST_DIR/yolo_node.py" || {
  say "FAIL: guest files missing -- run install_guest.sh"; exit 1; }

if [ "$LIVE" = 1 ]; then
  say "1/4 live camera in $CONTAINER (TCP profile, domain $DOMAIN)"
  docker exec "$CONTAINER" mkdir -p /tmp/yolo
  docker cp "$ROOT/tools/make_camera_world.py" "$CONTAINER":/tmp/yolo/ >/dev/null
  docker cp "$ART/assets/bus.jpg" "$CONTAINER":/tmp/yolo/ >/dev/null
  docker cp "$ART/assets/zidane.jpg" "$CONTAINER":/tmp/yolo/ >/dev/null
  dex "python3 /tmp/yolo/make_camera_world.py \
       --images /tmp/yolo/bus.jpg /tmp/yolo/zidane.jpg \
       --out-dir /tmp/yolo/cam --rate ${RATE:-0.2}"
  dexd "DISPLAY= gz sim -s -r -v2 --headless-rendering /tmp/yolo/cam/camera_world.sdf \
        > /tmp/yolo/gzsim.log 2>&1"
  for i in $(seq 1 24); do
    sleep 5; dex "gz topic -l 2>/dev/null | grep -q '^/camera$'" && break
  done
  dex "gz topic -l 2>/dev/null | grep -q '^/camera$'" || {
    say "FAIL: /camera never appeared"; dex "tail -20 /tmp/yolo/gzsim.log"; exit 1; }
  dexd "$HOST_ENV ros2 run ros_gz_bridge parameter_bridge --ros-args \
        -p config_file:=/tmp/yolo/cam/camera_bridge.yaml \
        > /tmp/yolo/bridge.log 2>&1"
  NODE_ENV="$GUEST_ENV_LIVE"
else
  say "1/4 bag replay in the guest (loop)"
  gsh "test -d $GUEST_DIR/camera_bag" || {
    say "FAIL: no bag on the guest -- run make_bag.sh"; exit 1; }
  NODE_ENV="$GUEST_ENV"
fi

say "2/4 detector node"
gsh "$NODE_ENV nohup python3 $GUEST_DIR/yolo_node.py --ros-args \
     -p model_path:=$GUEST_DIR/yolov8n.onnx \
     > \$HOME/yolo-node-$STAMP.log 2>&1 &" >/dev/null 2>&1
sleep 5
gsh "grep -q 'loaded' \$HOME/yolo-node-$STAMP.log" || sleep 20

if [ "$LIVE" != 1 ]; then
  say "3/4 bag play --loop"
  gsh "$GUEST_ENV nohup ros2 bag play $GUEST_DIR/camera_bag --loop \
       > \$HOME/bagplay-$STAMP.log 2>&1 &" >/dev/null 2>&1
else
  say "3/4 (live camera is already publishing)"
fi

say "4/4 waiting up to ${WAIT}s for the first /detection_result"
DEADLINE=$((SECONDS + WAIT))
PASS=no
while [ $SECONDS -lt $DEADLINE ]; do
  sleep 15
  if gsh "grep -m1 '\[detect\]' \$HOME/yolo-node-$STAMP.log" 2>/dev/null; then
    PASS=yes; break
  fi
  if gsh "grep -qE 'Traceback|Error' \$HOME/yolo-node-$STAMP.log" 2>/dev/null; then
    say "FAIL: node errored"; gsh "tail -30 \$HOME/yolo-node-$STAMP.log"; exit 1
  fi
done
[ "$PASS" = yes ] || {
  say "FAIL: no detection within ${WAIT}s"; gsh "tail -30 \$HOME/yolo-node-$STAMP.log"; exit 1; }

# The topic itself, not just the log line: this is what the TSP monitors.
gsh "$GUEST_ENV timeout 180 ros2 topic echo /detection_result --once >/dev/null 2>&1 \
     && echo '/detection_result: message received'" || \
  say "WARNING: log shows detections but ros2 topic echo timed out (domain mismatch?)"

MODE=$([ "$LIVE" = 1 ] && echo "G-Y4 live" || echo "G-Y3 bag")
say "RESULT: PASS ($MODE) -- node log \$HOME/yolo-node-$STAMP.log on the guest"
