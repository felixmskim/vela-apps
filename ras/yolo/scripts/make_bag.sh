#!/bin/bash
# Record the camera bag CR-12 promises ("sensor input = rosbag replay"): a
# Gazebo camera films photo billboards, the bridge turns the frames into
# sensor_msgs/Image, rosbag2 records them -- all inside the vela-gz container,
# where Gazebo and rosbag2 already exist. The finished bag is copied to the
# guest, which then needs no Gazebo and no host link to feed its detector.
#
# Recording happens on the container's default local transport; the TCP
# profiles exist for crossing the host-guest SLIRP boundary and nothing here
# crosses it.
set -eo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
ART=${ART:-$HOME/ms_kim/yolo-artifacts}
CONTAINER=${CONTAINER:-vela-gz}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/yolo}
DURATION=${DURATION:-40}      # seconds; 1 Hz camera -> ~40 frames, ~35 MB
RATE=${RATE:-1.0}

say() { echo "[$(date +%H:%M:%S)] $*"; }
dex()  { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
dexd() { docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
gsh()  { ssh -p "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

# Only OUR gz sim and OUR bridge, by their unmistakable argument strings --
# a plain 'gz sim' pattern would murder a concurrent gz-split run.
cleanup() {
  docker exec "$CONTAINER" bash -lc \
    "pkill -f '[c]amera_world.sdf|[c]amera_bridge.yaml' " >/dev/null 2>&1 || true
}
trap cleanup EXIT

say "1/5 world + bridge config into the container"
docker exec "$CONTAINER" mkdir -p /tmp/yolo
docker cp "$ROOT/tools/make_camera_world.py" "$CONTAINER":/tmp/yolo/ >/dev/null
docker cp "$ART/assets/bus.jpg" "$CONTAINER":/tmp/yolo/ >/dev/null
docker cp "$ART/assets/zidane.jpg" "$CONTAINER":/tmp/yolo/ >/dev/null
dex "python3 /tmp/yolo/make_camera_world.py \
     --images /tmp/yolo/bus.jpg /tmp/yolo/zidane.jpg \
     --out-dir /tmp/yolo/cam --rate $RATE"

say "2/5 gz sim, headless"
dexd "DISPLAY= gz sim -s -r -v2 --headless-rendering /tmp/yolo/cam/camera_world.sdf \
      > /tmp/yolo/gzsim.log 2>&1"
for i in $(seq 1 24); do
  sleep 5
  dex "gz topic -l 2>/dev/null | grep -q '^/camera$'" && break
done
dex "gz topic -l 2>/dev/null | grep -q '^/camera$'" || {
  say "FAIL: /camera never appeared"; dex "tail -20 /tmp/yolo/gzsim.log"; exit 1; }

say "3/5 bridge /camera -> /camera/image_raw"
dexd "ros2 run ros_gz_bridge parameter_bridge --ros-args \
      -p config_file:=/tmp/yolo/cam/camera_bridge.yaml \
      > /tmp/yolo/bridge.log 2>&1"
sleep 8
dex "timeout 20 ros2 topic hz /camera/image_raw --window 5 2>/dev/null | grep -m1 average" || {
  say "FAIL: no images on /camera/image_raw"; dex "tail -20 /tmp/yolo/bridge.log"; exit 1; }

say "4/5 record ${DURATION}s"
# sqlite3 spelled out: Jazzy's default is mcap, and the guest gets the
# sqlite3 storage plugin from the Vela APT -- record in what will replay.
dex "rm -rf /tmp/yolo/camera_bag; timeout --signal=INT $((DURATION + 5)) \
     ros2 bag record --storage sqlite3 -o /tmp/yolo/camera_bag /camera/image_raw" || true
dex "ros2 bag info /tmp/yolo/camera_bag" | sed -n '1,12p'
cleanup

say "5/5 bag -> host artifacts -> guest"
rm -rf "$ART/camera_bag"
docker cp "$CONTAINER":/tmp/yolo/camera_bag "$ART/" >/dev/null
gsh "mkdir -p $GUEST_DIR && rm -rf $GUEST_DIR/camera_bag"
scp -q -r -P "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no \
    "$ART/camera_bag" "$GUEST:$GUEST_DIR/"
gsh "du -sh $GUEST_DIR/camera_bag"
say "bag ready on the guest: $GUEST_DIR/camera_bag"
