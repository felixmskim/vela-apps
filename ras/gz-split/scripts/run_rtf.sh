#!/bin/bash
# Gate G1 -- can headless Gazebo hit a usable real-time factor on this host?
#
# First gate, because everything downstream is wasted if it fails.
# Pass: real_time_factor >= 0.30.  Kill: < 0.10 -- abandon live Gazebo, fall back
# to bag replay, and attach the measured number to defect D-14.
#
# Run it for both robots. TB3 answers "does headless Gazebo work on this host at
# all"; TB4 answers "does the TSP's robot fit in the budget". A TB4-only failure
# cannot distinguish render cost from Fuel, the topic spawn path, or the TF chain.
#
#   ROBOT=tb3 scripts/run_rtf.sh
#   ROBOT=tb4 scripts/run_rtf.sh
#
# No `set -u`: sourcing /opt/ros/jazzy/setup.bash dereferences
# AMENT_TRACE_SETUP_FILES while unset and would abort the script on line one.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
# Everything a run writes goes under artifacts/ (bind-mounted as /work/artifacts).
mkdir -p "$ROOT/artifacts/maps"
CONTAINER=${CONTAINER:-vela-gz}
ROBOT=${ROBOT:-tb3}
PASS=${PASS:-0.30}
KILL=${KILL:-0.10}
STAMP=$(date +%Y%m%d-%H%M%S)
LOG=/work/artifacts/gz-rtf-$ROBOT-$STAMP.log

say() { echo "[$(date +%H:%M:%S)] $*"; }
dex() { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }

cleanup() {
  say "tearing down"
  # Bracket trick: a plain pattern would match this pkill's own command line.
  for p in '[g]z sim' '[r]obot_state_publisher' '[r]uby'; do
    docker exec "$CONTAINER" bash -lc "pkill -f '$p'" >/dev/null 2>&1
  done
}
trap cleanup EXIT

docker inspect "$CONTAINER" >/dev/null 2>&1 || {
  say "FAIL: container '$CONTAINER' not running -- see README Build (host)"
  exit 1
}

say "step 1/6  software GL present?"
# The Dockerfile asserts nothing about the render path; this is where it gets
# established. Record all three blocks in RUNLOG.md -- the env vars alone do not
# prove llvmpipe is in use, and LIBGL_ALWAYS_SOFTWARE acts on the GLX path with
# an EGL reach that varies by Mesa version.
dex 'printenv LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER; ls /dev/dri 2>/dev/null || echo "/dev/dri: absent (expected)"'
dex 'EGL_PLATFORM=surfaceless eglinfo -B 2>&1 | grep -iE "device|renderer|OpenGL core" | head -8'
dex 'xvfb-run -s "-screen 0 1280x1024x24" glxinfo -B 2>&1 | grep -iE "renderer|core profile version" | head -4'
say "      expect llvmpipe and a core profile >= 3.3 from at least one of them"

say "step 2/6  generate world and robot for $ROBOT"
# No --gui here, ever, and no GUI= switch to add one: this gate produces the
# real-time factor everything else is judged against, and SceneBroadcaster plus a
# GUI would inflate the very number being measured. Use run_demo.sh to watch a
# run of run_container_loop.sh or run_split.sh instead.
dex "python3 /work/tools/make_world.py --robot $ROBOT --out-dir /work/artifacts" || {
  say "FAIL: make_world.py"; exit 1; }
# robot.env fixes the world name, spawn mode and file paths in one place.
docker cp "$CONTAINER":/work/artifacts/robot.env /tmp/robot.env.$$ >/dev/null 2>&1
. /tmp/robot.env.$$ && rm -f /tmp/robot.env.$$
say "      world=$WORLD_NAME file=$WORLD_FILE robot=$ROBOT_FILE spawn_mode=$SPAWN_MODE"

say "step 3/6  start gz sim server, headless"
# -v4 here only. This gate is the diagnostic one, and the engine's own
# initialisation lines are the second half of the render-path evidence; the
# closed-loop scripts stay at -v2 so their logs stay readable.
docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/jazzy/setup.bash; DISPLAY= gz sim -s -r -v4 --headless-rendering $WORLD_FILE > $LOG 2>&1"

# Two reasons to be patient here: OGRE-Next compiles its HLMS shaders on first
# render, tens of seconds under llvmpipe; and depot fetches its geometry from
# Fuel on first run, which is a network download.
for i in $(seq 1 36); do
  sleep 5
  if dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'"; then
    say "      stats topic up after $((i * 5))s"; break
  fi
done

if dex "grep -qE 'Failed to create render engine|Unable to create the rendering window' $LOG"; then
  say "FAIL: render engine did not initialise. Ladder:"
  say "  1) drop --headless-rendering, wrap in xvfb-run -s '-screen 0 1280x1024x24'"
  say "  2) --render-engine-server ogre   (OGRE 1.9; the CLI overrides the world's"
  say "     pinned <render_engine>ogre2</render_engine> -- Sensors.cc:829-835)"
  say "  3) reduce lidar <samples> 360 -> 180"
  dex "tail -30 $LOG"
  exit 1
fi
dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" || {
  say "FAIL: no stats on /world/$WORLD_NAME/stats"; dex "tail -40 $LOG"; exit 1; }

# Positive evidence, not just the absence of a failure string: which engine came
# up, on which EGL device, with which GL renderer. If this block is empty, say so
# in RUNLOG.md rather than assuming llvmpipe.
say "      render path, from the engine's own log:"
dex "grep -iE 'render engine|ogre|EGL|GL_RENDERER|llvmpipe|headless' $LOG \
     | sort -u | head -10" || say "      (nothing matched -- record that)"

say "step 4/6  spawn the robot ($SPAWN_MODE mode)"
if [ "$SPAWN_MODE" = topic ]; then
  # TB4 is a URDF. Upstream's own spawn uses -topic robot_description and leaves
  # the -file path commented out, so publish the description and spawn from it.
  docker exec -d "$CONTAINER" bash -lc \
    "source /opt/ros/jazzy/setup.bash; ros2 run robot_state_publisher robot_state_publisher \
     --ros-args -p use_sim_time:=true -p robot_description:=\"\$(cat $ROBOT_FILE)\" \
     > /work/artifacts/rsp-$STAMP.log 2>&1"
  sleep 8
  dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name $ROBOT_KIND \
       -topic robot_description -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z" || {
    say "FAIL: spawn from topic. Is robot_description published?"
    dex "ros2 topic list | grep -c robot_description"; exit 1; }
else
  dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name $ROBOT_KIND \
       -file $ROBOT_FILE -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z" || {
    say "FAIL: spawn from file"; exit 1; }
fi
sleep 15

say "step 5/6  read real_time_factor"
RTF=$(dex "gz topic -e -t /world/$WORLD_NAME/stats -n 10 2>/dev/null" \
      | awk '/real_time_factor/ {print $2}' | tail -1)
say "      real_time_factor = ${RTF:-<none>}"

say "step 6/6  is /scan actually producing returns?"
# A /scan topic existing is not enough: an all-inf scan means the render path
# produced nothing, which is indistinguishable from a working lidar in a topic
# listing. Read the gz topic directly -- no ros_gz_bridge runs at this gate.
FINITE=$(dex "gz topic -e -t /scan -n 1 2>/dev/null" \
         | grep -E '^ranges:' | grep -cvE 'inf')
say "      finite returns in the first message = ${FINITE:-0}"

if [ -z "$RTF" ]; then
  say "RESULT: FAIL (no stats published)"; exit 1
fi

awk -v r="$RTF" -v p="$PASS" -v k="$KILL" -v f="${FINITE:-0}" -v rb="$ROBOT" 'BEGIN {
  if (f + 0 < 1) { printf "RESULT: FAIL (%s: /scan produced no finite ranges - render path dead)\n", rb; exit 1 }
  if (r + 0 >= p) { printf "RESULT: PASS (%s: RTF %.3f >= %.2f)\n", rb, r, p; exit 0 }
  if (r + 0 <  k) { printf "RESULT: KILL (%s: RTF %.3f < %.2f) - abandon live Gazebo for this robot, use bag fallback, escalate D-14\n", rb, r, k; exit 2 }
  printf "RESULT: MARGINAL (%s: RTF %.3f between %.2f and %.2f) - proceed but expect long runs\n", rb, r, k, p; exit 3
}'
