#!/bin/bash
# Gate G2 -- close the loop entirely inside the host container: Gazebo + Nav2 +
# slam_toolbox, robot drives a waypoint tour, map is saved and scored.
#
# This runs before anything crosses the host/guest boundary on purpose. It fixes
# the world, the robot, the bridge and the SLAM parameters while there is no DDS
# transport variable in play. If it fails here it would have failed in the split
# too, and the split would have been blamed.
#
# G2b: run twice at different real-time factors and require the same outcome.
#
# RTF_LIMIT is applied by writing <physics><real_time_factor> into the world, NOT by
# calling /world/<name>/set_physics after startup. That service call is how this
# script used to do it, and it silently froze the robot: measured 2026-08-17 with
# pose_truth_probe.py, TRUE travel 0.007 m and zero rotation while DiffDrive
# integrated 3.99 m of odometry. Everything downstream then looked like a Nav2 or
# SLAM defect -- the arena drawn several times over at rotating angles, the return
# leg unplannable, 1/2 waypoints, 5 runs out of 5. Declaratively: 2/2 at RTF 0.5000
# with SLAM within 0.053 m of ground truth. See RUNLOG.md Step 6.
#
#   ROBOT=tb3 scripts/run_container_loop.sh
#   ROBOT=tb3 RTF_LIMIT=0.5 scripts/run_container_loop.sh
#
# No `set -u`: see run_rtf.sh.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
# Everything a run writes goes under artifacts/ (bind-mounted as /work/artifacts).
mkdir -p "$ROOT/artifacts/maps"
CONTAINER=${CONTAINER:-vela-gz}
ROBOT=${ROBOT:-tb3}
# 0 means unthrottled (as fast as the host manages).
RTF_LIMIT=${RTF_LIMIT:-0}
# score_map.py's three map gates. Each number has exactly one home, and it is not
# here: precision and recall default inside score_map.py, the coverage floor is
# per-world and comes from robot.env, which make_world.py owns. This script only
# forwards what the caller overrode -- a copy of the default here silently went
# stale once already, when score_map.py's precision floor moved 0.85 -> 0.80.
# MIN_COVERAGE is captured before robot.env is sourced, or sourcing would win.
MIN_COVERAGE_OVERRIDE=$MIN_COVERAGE
# Same for the other two, since 2026-08-19: robot.env now sets them as well
# (tb4 carries min_precision 0 because precision tracks exploration geometry on
# depot.pgm rather than map quality -- see make_world.py), so without this a
# caller-supplied floor would be silently overwritten by the sourcing below and the
# run would gate on something nobody asked for.
MIN_PRECISION_OVERRIDE=$MIN_PRECISION
MIN_RECALL_OVERRIDE=$MIN_RECALL
# GUI=1 builds the world with SceneBroadcaster so scripts/run_demo.sh can attach a
# viewer to this run. It makes the run watchable and its numbers unusable -- the
# broadcast is extra server work and the GUI competes for the same cores. Use it
# to show the robot driving, never to fill in RUNLOG.md.
GUI=${GUI:-0}
GUI_ARG=""
[ "$GUI" = 1 ] && GUI_ARG="--gui"
# TRUTH_PROBE=1 adds a PosePublisher to the robot and logs the simulator's true
# pose beside the SLAM estimate and the wheel odometry, to tell a scan-matching
# failure apart from odometry drift. It adds a publisher the measured
# configuration does not have, so its runs are diagnosis, never RUNLOG.md numbers.
TRUTH_PROBE=${TRUTH_PROBE:-0}
TRUTH_ARG=""
[ "$TRUTH_PROBE" = 1 ] && TRUTH_ARG="--truth-pose"
STAMP=$(date +%Y%m%d-%H%M%S)

say() { echo "[$(date +%H:%M:%S)] $*"; }
dex() { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
dexd() { docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }

PROCS="[g]z sim|[p]arameter_bridge|[a]sync_slam_toolbox|[p]lanner_server|\
[c]ontroller_server|[b]ehavior_server|[b]t_navigator|[l]ifecycle_manager|\
[c]md_vel_watchdog|[s]end_goal|[r]obot_state_publisher|[s]tatic_transform|\
[p]ose_truth_probe"

cleanup() {
  say "tearing down"
  docker exec "$CONTAINER" bash -lc "pkill -f '$PROCS'" >/dev/null 2>&1
  # SIGTERM is not enough. An interrupted bringup leaves bt_navigator and
  # lifecycle_manager wedged mid-transition and they ignore it -- observed
  # 2026-08-17, two orphans still running afterwards. Survivors poison the next
  # run: a second lifecycle_manager transitioning the same nodes, and duplicate
  # node names.
  sleep 3
  if docker exec "$CONTAINER" bash -lc "pgrep -f '$PROCS' >/dev/null" 2>/dev/null; then
    say "      some nodes ignored SIGTERM, sending SIGKILL"
    docker exec "$CONTAINER" bash -lc "pkill -9 -f '$PROCS'" >/dev/null 2>&1
    sleep 1
  fi
  docker exec "$CONTAINER" bash -lc "pgrep -af '$PROCS'" 2>/dev/null \
    && say "      WARNING: still running (above) -- kill by hand before the next run"
  docker exec "$CONTAINER" bash -lc 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*' >/dev/null 2>&1
}
trap cleanup EXIT

docker inspect "$CONTAINER" >/dev/null 2>&1 || { say "FAIL: no container"; exit 1; }

say "clearing stale Fast DDS shared memory"
docker exec "$CONTAINER" bash -lc 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*' >/dev/null 2>&1

say "1/7  world + robot for $ROBOT"
[ "$GUI" = 1 ] && say "      GUI=1 -- watchable run, NOT a measurement run"
RTF_ARG=""
[ "$RTF_LIMIT" != "0" ] && RTF_ARG="--rtf $RTF_LIMIT"
dex "python3 /work/tools/make_world.py --robot $ROBOT --out-dir /work/artifacts \
     $GUI_ARG $TRUTH_ARG $RTF_ARG" || exit 1
docker cp "$CONTAINER":/work/artifacts/robot.env /tmp/robot.env.$$ >/dev/null 2>&1
. /tmp/robot.env.$$ && rm -f /tmp/robot.env.$$
[ -n "$MIN_COVERAGE_OVERRIDE" ] && MIN_COVERAGE=$MIN_COVERAGE_OVERRIDE
[ -n "$MIN_PRECISION_OVERRIDE" ] && MIN_PRECISION=$MIN_PRECISION_OVERRIDE
[ -n "$MIN_RECALL_OVERRIDE" ] && MIN_RECALL=$MIN_RECALL_OVERRIDE
MAP_GATES="--min-coverage ${MIN_COVERAGE:-0}"
[ -n "$MIN_PRECISION" ] && MAP_GATES="$MAP_GATES --min-precision $MIN_PRECISION"
[ -n "$MIN_RECALL" ] && MAP_GATES="$MAP_GATES --min-recall $MIN_RECALL"
say "      world=$WORLD_NAME bridge=$BRIDGE_FILE map_ref=$MAP_REF offset=$MAP_OFFSET"
say "      map gates: $MAP_GATES (unset floors come from score_map.py)"

# Waypoints are per-world: tb3_sandbox is a ~5 m sandbox, depot is much larger.
# WAYPOINTS comes from robot.env (world coordinates, per robot).

say "2/7  gz sim (rtf limit=$RTF_LIMIT)"
dexd "DISPLAY= gz sim -s -r -v2 --headless-rendering $WORLD_FILE \
      > /work/artifacts/gz-loop-$ROBOT-$STAMP.log 2>&1"
for i in $(seq 1 36); do
  sleep 5
  dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" && break
done
dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" || {
  say "FAIL: gz sim never published stats"; dex "tail -30 /work/artifacts/gz-loop-$ROBOT-$STAMP.log"; exit 1; }
# The RTF is now set declaratively, in <physics><real_time_factor> by make_world.py
# above, and NOT by calling /world/<name>/set_physics at runtime. That service call
# is what this used to do, and measured 2026-08-17 with pose_truth_probe.py it
# stops rigid-body motion outright: the robot travelled 0.007 m of TRUE distance
# and did not rotate at all (truth_yaw span 0.000) while DiffDrive integrated
# 3.99 m of odometry and slam_toolbox smeared the arena around a pose that was
# moving only in its own estimate. Passing max_step_size alongside
# real_time_factor, so the omitted protobuf field could not arrive as 0, did not
# help -- so it is the call itself, not one missing field.

say "3/7  spawn the robot ($SPAWN_MODE mode)"
if [ "$SPAWN_MODE" = topic ]; then
  dexd "ros2 run robot_state_publisher robot_state_publisher --ros-args \
        -p use_sim_time:=true -p robot_description:=\"\$(cat $ROBOT_FILE)\" \
        > /work/artifacts/rsp-$STAMP.log 2>&1"
  sleep 8
  dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name $ROBOT_KIND \
       -topic robot_description -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z" || exit 1
else
  dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name $ROBOT_KIND \
       -file $ROBOT_FILE -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z" || exit 1
fi
sleep 10

say "4/7  bridge + cmd_vel watchdog"
dexd "ros2 run ros_gz_bridge parameter_bridge --ros-args \
      -p config_file:=$BRIDGE_FILE \
      -p expand_gz_topic_names:=$EXPAND_GZ_TOPICS -p use_sim_time:=true \
      > /work/artifacts/bridge-$STAMP.log 2>&1"
dexd "python3 /work/tools/cmd_vel_watchdog.py --ros-args -p use_sim_time:=true \
      > /work/artifacts/watchdog-$STAMP.log 2>&1"
# A second bridge instance, only for the ground-truth pose. It is remapped off /tf
# on purpose: gz.msgs.Pose_V maps to tf2_msgs/msg/TFMessage, and letting the
# simulator's true pose land on /tf would hand Nav2 a second, perfect source of
# map->base and hide the very error being measured.
if [ "$TRUTH_PROBE" = 1 ] && [ -n "$TRUTH_TOPIC" ]; then
  dexd "ros2 run ros_gz_bridge parameter_bridge \
        '$TRUTH_TOPIC@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
        --ros-args -r $TRUTH_TOPIC:=/gz_truth -p use_sim_time:=true \
        > /work/artifacts/truthbridge-$STAMP.log 2>&1"
fi
sleep 8
dex "ros2 topic hz /clock --window 5 2>/dev/null | head -3" || true

# CONTROLLER_HZ overrides controller_frequency. Off by default, and deliberately so.
#
# The asymmetry it exists to explore is real: controller_server ticks on
# rclcpp::WallRate, so controller_frequency is a WALL rate whatever use_sim_time
# says, while the lidar runs at 5 Hz of SIM time (gz_waffle.sdf.xacro
# <update_rate>5</update_rate>). Measured 2026-08-17 at RTF 0.5: /cmd_vel 5.000 Hz
# wall against /scan 2.502 Hz wall -- two commands per scan, the second computed
# from an observation that had not changed. At RTF 1.0 the two coincide 1:1, by
# coincidence rather than by design.
#
# It is also harmless. It was the leading suspect for the G2b failure and was
# refuted twice: scaling it to 2.5 Hz left the failure identical, and once the real
# cause was fixed, RTF 0.5 passes either way -- unscaled scored precision 0.994 /
# recall 0.980, scaled 0.934 / 0.971. So this stays an override for experiments and
# not something the runner does behind your back.
NAV_PARAMS=/work/config/nav2_split.yaml
if [ -n "$CONTROLLER_HZ" ]; then
  BASE_HZ=$(awk '/^ *controller_frequency:/ {print $2; exit}' "$ROOT/config/nav2_split.yaml")
  NAV_PARAMS=/work/artifacts/nav2_split.gen.yaml
  dex "sed 's/^\( *controller_frequency:\).*/\1 $CONTROLLER_HZ/' \
       /work/config/nav2_split.yaml > /work/artifacts/nav2_split.gen.yaml" || exit 1
  dex "grep -q '^ *controller_frequency: $CONTROLLER_HZ\$' /work/artifacts/nav2_split.gen.yaml" \
    || { say "FAIL: controller_frequency rewrite did not take"; exit 1; }
  say "      controller_frequency $BASE_HZ -> $CONTROLLER_HZ wall Hz (by request)"
fi

say "5/7  Nav2 + slam_toolbox"
dexd "ros2 launch /work/tools/slam_nav.launch.py robot:=$ROBOT_KIND \
      urdf_file:=$ROBOT_FILE \
      nav_params_file:=$NAV_PARAMS \
      slam_params_file:=/work/config/mapper_params_tcg.yaml \
      > /work/artifacts/stack-$STAMP.log 2>&1"

say "      waiting for lifecycle activation (up to 600s)"
ACTIVE=no
HINTED=no
for i in $(seq 1 120); do
  sleep 5
  # Unanchored on purpose: these literals are long enough to match nothing
  # else. The ^-anchor rule (README, Do not) exists for short patterns like
  # 'E: ', which apt's own banner contains.
  if dex "grep -qE 'Managed nodes are active' /work/artifacts/stack-$STAMP.log"; then
    ACTIVE=yes; say "      active after $((i * 5))s"; break
  fi
  if dex "grep -qE 'CRITICAL FAILURE|process has died' /work/artifacts/stack-$STAMP.log"; then
    say "FAIL: stack died early"; dex "tail -40 /work/artifacts/stack-$STAMP.log"; exit 1
  fi
  # Name the stall rather than sitting silent for ten minutes. This one does not
  # resolve on its own once it starts, but it can take a while to appear in the
  # guest, so hint instead of failing.
  if [ "$HINTED" = no ] && [ "$i" -ge 12 ] &&
     dex "grep -qF 'Invalid frame ID \"map\"' /work/artifacts/stack-$STAMP.log"; then
    HINTED=yes
    say "      NOTE: costmaps are waiting on a map frame that does not exist."
    say "      That means slam_toolbox is not publishing map->odom. Check:"
    say "        docker exec $CONTAINER bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 lifecycle get /slam_toolbox'"
    say "      'unconfigured' means it is not in lifecycle_manager_nav's node_names."
    say "      A SIGABRT right after 'Registering sensor' is a mapper parameter"
    say "      out of range -- see the smear-deviation note in mapper_params_tcg.yaml."
  fi
done
[ "$ACTIVE" = yes ] || { say "FAIL: never activated"; dex "tail -40 /work/artifacts/stack-$STAMP.log"; exit 1; }

# PRE_DRIVE_SLEEP exists for recordings. run_demo.sh needs about 50 s to bring up
# Xvfb, compile the GUI's shaders and start ffmpeg, while this script reaches the
# tour in about 45 s -- so without a delay the recorder misses the first half of
# the drive. Zero by default; a measurement run must not wait around.
if [ "${PRE_DRIVE_SLEEP:-0}" != 0 ]; then
  say "      holding ${PRE_DRIVE_SLEEP}s before driving so a recorder can attach"
  sleep "$PRE_DRIVE_SLEEP"
fi

if [ "$TRUTH_PROBE" = 1 ] && [ -n "$TRUTH_TOPIC" ]; then
  say "      starting pose_truth_probe (truth vs slam vs odom)"
  dexd "python3 /work/tools/pose_truth_probe.py --truth-topic /gz_truth \
        --model $(basename "$(dirname "$TRUTH_TOPIC")") \
        --base-frame $BASE_FRAME --world-offset=$SPAWN_X,$SPAWN_Y \
        --duration ${TRUTH_DURATION:-240} --csv /work/artifacts/truth-$ROBOT-$STAMP.csv \
        --ros-args -p use_sim_time:=true > /work/artifacts/truth-$ROBOT-$STAMP.log 2>&1"
fi

say "6/7  waypoint tour: $WAYPOINTS"
# pipefail inside the container shell too, or the exit status would be tail's
# and every run would look like a pass.
# --world-offset=... and --offset=... must use the `=` form. Both values start
# with a minus sign for every robot here (tb3 -2.0,-0.5; tb4 -8.0,0.0), and
# argparse treats a leading-dash token as an option unless it matches its
# negative-number regex (^-\d+$ or ^-\d*\.\d+$) -- which "-2.0,-0.5" does not. The
# space form dies with "expected one argument" *before* the robot is ever sent a
# goal, and the loop then reports FAIL with an empty drive.
dex "set -o pipefail; python3 /work/tools/send_goal.py --waypoints '$WAYPOINTS' \
     --base-frame $BASE_FRAME \
     --world-offset=$SPAWN_X,$SPAWN_Y --track-csv /work/artifacts/track-$ROBOT-$STAMP.csv 2>&1 | tail -40"
NAV_RC=$?

say "7/7  save and score the map"
dex "mkdir -p /work/artifacts/maps && ros2 run nav2_map_server map_saver_cli \
     -f /work/artifacts/maps/loop-$ROBOT-$STAMP --ros-args -p use_sim_time:=true" || {
  say "FAIL: map_saver"; exit 1; }
dex "python3 /work/tools/score_map.py --slam /work/artifacts/maps/loop-$ROBOT-$STAMP.yaml \
     --ref $MAP_REF --offset=$MAP_OFFSET $MAP_GATES"
MAP_RC=$?

RTF=$(dex "gz topic -e -t /world/$WORLD_NAME/stats -n 5 2>/dev/null" \
      | awk '/real_time_factor/ {print $2}' | tail -1)
say "observed real_time_factor = ${RTF:-<none>}"

if [ "$NAV_RC" -eq 0 ] && [ "$MAP_RC" -eq 0 ]; then
  say "RESULT: PASS (G2, $ROBOT) -- now rerun with a different RTF_LIMIT for G2b"
  exit 0
fi
say "RESULT: FAIL (G2, $ROBOT)  nav_rc=$NAV_RC map_rc=$MAP_RC"
exit 1
