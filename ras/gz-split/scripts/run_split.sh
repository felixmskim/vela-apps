#!/bin/bash
# Gate G5 -- the deliverable. Gazebo on the host, Nav2 and slam_toolbox in the
# RISC-V guest, robot drives a waypoint tour, SLAM map saved and scored.
#
# This is the topology the TSP test-environment figure (p.16-17) prescribes:
# Gazebo and the visualisation stack on x86-64, SLAM and Nav2 on the RISC-V side.
# Run run_rtf.sh (G1), run_container_loop.sh (G2) and run_dds_check.sh (G3/G4)
# first; each isolates one variable this script depends on.
#
#   ROBOT=tb4 scripts/run_split.sh     # the TSP's robot -- what the report needs
#   ROBOT=tb3 scripts/run_split.sh     # cheaper, for bringing the path up
#
# What this does NOT do: there is no RERI here. The guest is Config A -- no
# reri=true, no EDK2 pflash pair -- so this produces the co-tenant workload for
# TC.10.007, never a TC.10.007 result.
#
# No `set -u`: see run_rtf.sh.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
# Everything a run writes goes under artifacts/ (bind-mounted as /work/artifacts).
mkdir -p "$ROOT/artifacts/maps"
CONTAINER=${CONTAINER:-vela-gz}
ROBOT=${ROBOT:-tb4}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
DOMAIN=${DOMAIN:-42}
GUEST_DIR=${GUEST_DIR:-/home/vela/gz_split}
# Map gate floors live in score_map.py and robot.env, never here. See
# run_container_loop.sh for why.
MIN_COVERAGE_OVERRIDE=$MIN_COVERAGE
# Same for the other two, since 2026-08-19: robot.env now sets them as well
# (tb4 carries min_precision 0 because precision tracks exploration geometry on
# depot.pgm rather than map quality -- see make_world.py), so without this a
# caller-supplied floor would be silently overwritten by the sourcing below and the
# run would gate on something nobody asked for.
MIN_PRECISION_OVERRIDE=$MIN_PRECISION
MIN_RECALL_OVERRIDE=$MIN_RECALL
# Whether the CALLER set it, recorded before the default hides the difference.
# The YOLO block below needs to know, and by the time it runs RTF_LIMIT is always
# set -- to 0 if nobody asked -- so ${RTF_LIMIT+x} there is always non-empty.
RTF_LIMIT_GIVEN=${RTF_LIMIT+yes}
RTF_LIMIT=${RTF_LIMIT:-0}
# A third workload does not fit in four emulated vCPUs at half speed, and the
# failure does not look like a CPU problem: at RTF_LIMIT=0.5 with the detector,
# bringup went 70s to 110s, the guest clock fell 2.8 s behind and Nav2 aborted both
# goals with "Initial robot pose is not available" -- which names TF. So YOLO=1
# lowers the ceiling itself rather than leaving it in the README, where the value
# was correct and unenforced while the default (0, unlimited) reproduced the
# failure for anyone who ran the obvious command. An explicit RTF_LIMIT still wins.
# Placed HERE, not beside the other YOLO knobs below: TOPIC_BUDGET_MS divides the
# TSP figure by RTF_LIMIT a few dozen lines down, so lowering it after that point
# left a measurement run with a budget four times too tight (WATCHDOG=1 only, but
# silently). ${YOLO:-0} because YOLO=${YOLO:-0} has not run yet.
if [ "${YOLO:-0}" = 1 ]; then
  if [ "${RTF_LIMIT_GIVEN:-}" != yes ] || [ "$RTF_LIMIT" = 0 ]; then
    RTF_LIMIT=0.25
    YOLO_RTF_DEFAULTED=1
  elif awk -v r="$RTF_LIMIT" 'BEGIN{exit !(r+0 > 0.25)}'; then
    YOLO_RTF_HIGH=1
  fi
fi
# /odom and the odom->base_footprint /tf both come from DiffDrive at one rate, and
# 25 Hz was measured for both. Nothing here needs it: slam_toolbox matches scans at
# 5 Hz and transform_tolerance is 5 s. Every one of those messages is deserialised
# separately by every subscribing node in the guest, and that load is what starves
# bt_navigator's clock -- see RUNLOG.md Step 8. Lowered in the split run only;
# run_container_loop.sh is deliberately left alone so G2's passing numbers still
# describe the inputs they were measured on. Set ODOM_HZ= to disable.
ODOM_HZ=${ODOM_HZ-10}
# GUI=1 builds the world with SceneBroadcaster so scripts/run_demo.sh can attach a
# viewer and an audience can watch the robot drive. It costs server work and host
# cores, so a GUI=1 run demonstrates the deliverable rather than measuring it.
GUI=${GUI:-0}
GUI_ARG=""
[ "$GUI" = 1 ] && GUI_ARG="--gui"
# liveliness_watchdog.py measured 69% of an emulated core in Step 8 -- the single
# largest consumer in the guest, larger than any Nav2 node, and it exists only to
# record arrival timestamps. G6 needs it; a demonstration run does not, and the
# 69% it returns goes to the nodes that are starving. WATCHDOG= disables it.
# clock_lag_probe.py stays on either way: step 7 gates on its output.
WATCHDOG=${WATCHDOG-1}

# EXPLORE=1 replaces the scripted waypoint tour with explore_lite: frontier-based
# autonomous exploration, the second drive mode the TSP figure names ("Host
# Teleop(manual) / Explore_lite(autonomous)"). The two are mutually exclusive and
# the code at step 8 is an if/else rather than two calls: both drive the same
# navigate_to_pose action server -- the action name is a literal in the explore
# binary -- and two clients racing it would preempt each other's goals.
#
# Needs ros-jazzy-explore-lite in the guest. It is 2 packages (explore-lite plus
# explore-lite-msgs, 318 kB) because every C++ dependency is already installed;
# contrast the 602 packages DESIGN.md cites for putting Gazebo in the guest.
# Checked at step 0 rather than installed: this script installs nothing.
EXPLORE=${EXPLORE:-0}
# WALL seconds. Exploration stops at this point even if frontiers remain, so a
# guest that is merely slow still yields a scorable map instead of hanging. Sized
# off measured leg cost -- 32-36 s wall for tb3, 16-19 s for tb4 -- so 900 s buys
# roughly twenty to fifty legs. Consumed in 15 s polls, each paying an ssh round
# trip to a slow guest, so real elapsed runs somewhat over.
EXPLORE_TIMEOUT=${EXPLORE_TIMEOUT:-900}
# Frontier goals that reached the action server. Zero means explore_lite never
# found a frontier or never reached bt_navigator, and the map that follows would
# describe the spawn pose rather than an exploration. This is the EXPLORE=1
# analogue of "every waypoint SUCCEEDED".
EXPLORE_MIN_GOALS=${EXPLORE_MIN_GOALS:-2}
# Waypoints driven BEFORE explore_lite starts, to break a bootstrap deadlock that is
# a property of the pair rather than of either node. slam_toolbox's
# minimum_travel_distance is 0.5 m, so from a standstill it adds no scan after the
# first and /map stays empty -- measured 2026-08-19: 0 occupied cells, 99.1% unknown.
# explore_lite derives its goals from that map, so the only frontier it can find is
# two or three cells from the robot: 554 goals, every one 0.13 m away, robot never
# moved. Driving a little first gives slam_toolbox something to map, and the frontiers
# then have somewhere to be.
#
# Two waypoints rather than one, because the first goal after bringup is unreliable
# here -- measured, a tb4 run reached 3 of 4 and the one it missed was wp1, aborted at
# 12 s with the robot still at the origin. The second always drove.
# Set EXPLORE_SEED= to skip it and watch the deadlock instead.
EXPLORE_SEED=${EXPLORE_SEED-2}

# WALL milliseconds, applied to the LONGEST inter-arrival interval per watched
# topic. This is the TSP's topic figure -- 1,000 ms for the 3rd year, 500 ms for
# the 4th -- read the only way this rig can answer it: as the longest interruption
# a consumer actually saw. Publish-to-subscribe latency is not measurable across
# this split; liveliness_watchdog.py's docstring carries the derivation, and the
# short version is that the guest's clock lags the host's by 0.5-3.1 s against a
# wanted signal of 5-25 ms, so any stamp-difference method reports the lag instead.
#
# Requires WATCHDOG=1, because liveliness_watchdog.py is what measures it -- and
# that instrument costs 69% of an emulated core, which is why WATCHDOG= exists.
# A measurement run pays it; a drive-only run does not, and then this is skipped.
# The default is the 4th-year figure SCALED BY THE SIMULATION RATE, and the scaling
# is not a fudge: the sensors run on sim time and this instrument measures wall
# time. /scan is 5 Hz of sim, i.e. 200 ms sim between scans, which at RTF_LIMIT=0.5
# is 400 ms of wall -- so an unscaled 500 ms wall budget leaves 100 ms of margin and
# fails on jitter while nothing is wrong. Same class of mistake as reading
# controller_frequency as a sim rate.
#
# Consequence for reporting, and it must be stated wherever the number is quoted: a
# wall interval measured at RTF < 1 is NOT the TSP's number. It is the TSP's number
# for a simulator running this slowly. Report it with the RTF beside it, per the
# emulation clause the correction request asks for (CR-9).
TSP_BUDGET_MS=${TSP_BUDGET_MS:-500}
TOPIC_BUDGET_MS=${TOPIC_BUDGET_MS:-$(awk -v b="$TSP_BUDGET_MS" -v r="$RTF_LIMIT" \
  'BEGIN{printf "%.0f", (r+0 > 0 ? b/r : b)}')}
# YOLO=1 adds the detector as a third co-tenant: a camera bridged to the guest
# and yolo_node inferring on it beside Nav2 and slam_toolbox. This is gate
# G-Y5 -- the workload TC.10.007 actually names, all of it at once in one guest.
#
# Default off. G5/G6/G7 were measured without it and the numbers in RUNLOG.md
# describe that configuration; a co-tenant that changes them silently would make
# every earlier entry unreadable.
#
# YOLO_CAM picks where the camera is and what it sees:
#
#   robot   (default) the camera rides base_link (make_world.py --robot-camera)
#           and static person models stand in the arena (make_people.py), so
#           the detector labels what the robot is looking at. The people are
#           real obstacles: the lidar maps their legs and Nav2 curves around
#           them. That costs the map score a few unmatched cells against the
#           shipped reference -- honest, measured, gated the same as always.
#
#   scene   the original rig: a static camera and photo billboards spawned far
#           outside the arena (make_billboards.py). Pure co-tenancy -- nothing
#           the robot sees is what the detector labels, which is exactly why a
#           demo built on it was rejected on review (2026-08-20). Kept because
#           every G-Y5 number before that date was measured in this mode.
YOLO=${YOLO:-0}
YOLO_CAM=${YOLO_CAM:-robot}
YOLO_ASSETS=${YOLO_ASSETS:-$HOME/ms_kim/yolo-artifacts/assets}
# Host-side copy of the Gazebo Fuel cache, checked under GUI=1 only. The people
# models are container-local state: the block below downloads them into
# $CONTAINER, and `docker inspect vela-gz` shows /work as its only mount, so that
# cache never reaches the host or the viewer image. run_demo.sh mounts this
# directory into the viewer read-only, and must agree with it on the path.
FUEL_CACHE=${FUEL_CACHE:-$HOME/ms_kim/gz-fuel-cache}
YOLO_DIR=${YOLO_DIR:-/home/vela/yolo}
YOLO_X=${YOLO_X:-30}
YOLO_Y=${YOLO_Y:-0}
# Where the people stand (robot mode), as x,y,yaw,kind in WORLD coordinates.
# Chosen against the slalom tour (see the WAYPOINTS_OVERRIDE note) so someone
# is dead ahead on most legs: every viewpoint was rendered and inferred before
# adoption -- person:0.85-0.91 from all six probe poses, 2026-08-20. Each is
# >=0.6 m off every leg's straight line, >=0.63 m from every goal (a goal in
# lethal cost aborts; these sit in inflation only) and >=0.24 m clear of
# cylinders. Yaw faces each toward the leg it is watched from; both meshes
# look along -Y of their frame, so yaw = facing + pi/2 (make_people.py).
YOLO_PEOPLE=${YOLO_PEOPLE:-"1.85,-0.75,4.71,standing -0.60,1.50,0.99,walking -1.90,0.60,1.55,standing 0.75,-1.85,4.15,standing"}
# SIM-time Hz, and the default differs by mode because what a frame is worth
# differs. scene: the view never changes, so anything above 0.1 Hz is identical
# frames spent on TCP and guest deserialisation (one 320x320 inference costs
# 11.4 s on this guest; rx == seq at these defaults, one frame in and one pass
# out every ~40 s of wall clock). robot: the view is what the tour is showing,
# and at 0.1 Hz the robot moves ~2.4 m between frames -- past a person entirely.
# 0.25 Hz is one frame per ~0.95 m of travel; yolo_node keeps only the newest,
# so the extra frames cost 230 kB of TCP each and no inference at all.
if [ "${YOLO_CAM}" = robot ]; then
  YOLO_RATE=${YOLO_RATE:-0.25}
else
  YOLO_RATE=${YOLO_RATE:-0.1}
fi
# 320x240, not the detector's 640x480 default. Every frame the guest receives is
# deserialised by rclpy whether it is inferred on or not, and the node keeps only
# the newest -- so at rx=15 per pass, 640x480 rgb8 was 13.5 MB of TCP and Python
# object construction per detection, nearly all of it thrown away. The model
# letterboxes to 320x320 regardless, so the larger frame buys no accuracy: four
# times the cost for none of the benefit.
YOLO_W=${YOLO_W:-320}
YOLO_H=${YOLO_H:-240}
# HOLD=1 keeps the whole stack up after the verdict prints instead of tearing it
# down immediately. It exists because of what teardown does to a recording:
# run_demo.sh watches the VIEWER container's `gz sim -g`, not the server in
# vela-gz, so when this script's EXIT trap kills the server the moment the
# verdict lands, the recording keeps running on a frozen scene and nobody
# watching the viewer is told. Take 3 avoided that by being timed to finish
# first; with the detector the run is roughly twice as long (17.7 min measured at
# RTF_LIMIT=0.25) and that timing is no longer something to bet a take on.
#
# It also gives an availability run somewhere to live: TC.10.007 collects for the
# better part of an hour, and this script's job ends after a two-leg tour.
HOLD=${HOLD:-0}
HOLD_FILE=${HOLD_FILE:-/tmp/run_split.release}
VERDICT_PRINTED=0
STAMP=$(date +%Y%m%d-%H%M%S)

say() { echo "[$(date +%H:%M:%S)] $*"; }
dex()  { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
dexd() { docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
# The remote command arrives on stdin rather than inside a quoted -c argument.
# Several commands below contain single quotes -- the pkill bracket trick, the
# waypoint list -- which would close an outer '...' early and mangle the command.
gsh()  { ssh -p "$PORT" -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

HOST_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_tcp_server.xml; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"
# Same domain, one profile and one listening port per host participant that has to
# reach the guest: parameter_bridge owns 42100, clock_decimator 42101, and
# cmd_vel_watchdog 42102. See host_clock_server.xml for why they cannot share one.
CLOCK_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_clock_server.xml; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"
CMDVEL_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_cmdvel_server.xml; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"
# Sourcing ROS here is not optional: `ssh ... bash -l -s` reads ~/.profile and not
# ~/.bashrc, and the guest's ~/.profile does not source it, so every remote ros2 call
# would die with "failed to run command 'ros2': No such file or directory". Found
# 2026-08-17 in run_dds_check.sh, which has the same line.
#
# GUI=1 swaps in a generated profile rather than editing the committed one. The
# viewer participants run_demo.sh starts -- rviz2 on 42103, detection_overlay.py on
# 42104 -- are host acceptors like any other, so the guest can only reach them if it
# has an initialPeersList entry for each. But guest_tcp_client.xml is the file G3, G4
# and every G5 were measured on, and it is read by ~13 guest participants; adding two
# locators nobody listens on during a headless run would make each of them re-dial two
# dead ports once per announcement interval, on a 4-vCPU TCG guest whose CPU budget is
# itself a gate. Generating a demo copy from the committed one keeps a single source of
# truth and leaves the measured file byte-identical. Same reasoning as the /clock
# rename living in this runner instead of in bridge_split.yaml.
GUEST_PROFILE=guest_tcp_client.xml
[ "$GUI" = 1 ] && GUEST_PROFILE=guest_tcp_client_demo.xml
GUEST_ENV="source /opt/ros/jazzy/setup.bash; \
export FASTRTPS_DEFAULT_PROFILES_FILE=$GUEST_DIR/$GUEST_PROFILE; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"

HOST_PROCS="[g]z sim|[p]arameter_bridge|[c]md_vel_watchdog|[r]uby|\
[r]obot_state_publisher|[c]lock_decimator"
GUEST_PROCS="[a]sync_slam_toolbox|[p]lanner_server|[c]ontroller_server|\
[b]ehavior_server|[b]t_navigator|[l]ifecycle_manager|[s]end_goal|\
[l]iveliness_watchdog|[s]tatic_transform|[r]obot_state_publisher|[c]lock_lag_probe|\
[y]olo_node|[e]xplore_lite"

hold_if_asked() {
  [ "$HOLD" = 1 ] && [ "$VERDICT_PRINTED" = 1 ] || return 0
  # Only after the verdict: holding on a Ctrl-C during bringup, or on a preflight
  # exit, would leave a half-built stack up and the operator waiting on it.
  HOLD_DONE=0
  # An explicit INT trap, because this runs from inside the EXIT trap. Without it
  # Ctrl-C during the wait terminates the shell mid-trap and the teardown below
  # never runs -- which is the state HOLD exists to avoid arriving at by accident.
  trap 'HOLD_DONE=1' INT
  rm -f "$HOLD_FILE"
  say "HOLD=1 -- the stack stays up so a recording can outlive the verdict."
  say "         Ctrl-C here, or 'touch $HOLD_FILE' from anywhere, tears it down."
  while [ "$HOLD_DONE" = 0 ] && [ ! -f "$HOLD_FILE" ]; do sleep 2; done
  trap - INT
  rm -f "$HOLD_FILE"
  say "      released"
}

cleanup() {
  hold_if_asked
  say "tearing down"
  docker exec "$CONTAINER" bash -lc "pkill -f '$HOST_PROCS'" >/dev/null 2>&1
  gsh "pkill -f '$GUEST_PROCS'" >/dev/null 2>&1
  # SIGTERM alone leaves bt_navigator and lifecycle_manager wedged when bringup is
  # interrupted mid-transition; survivors poison the next run. Worse in the guest,
  # where TCG makes every shutdown slower, so give it longer before escalating.
  sleep 5
  if gsh "pgrep -f '$GUEST_PROCS' >/dev/null" 2>/dev/null; then
    say "      guest nodes ignored SIGTERM, sending SIGKILL"
    gsh "pkill -9 -f '$GUEST_PROCS'" >/dev/null 2>&1
  fi
  if docker exec "$CONTAINER" bash -lc "pgrep -f '$HOST_PROCS' >/dev/null" 2>/dev/null; then
    docker exec "$CONTAINER" bash -lc "pkill -9 -f '$HOST_PROCS'" >/dev/null 2>&1
  fi
  gsh "pgrep -af '$GUEST_PROCS'" 2>/dev/null \
    && say "      WARNING: guest nodes still running (above) -- kill before the next run"
}
trap cleanup EXIT

say "0/10 preflight"
docker inspect "$CONTAINER" >/dev/null 2>&1 || { say "FAIL: no container"; exit 1; }
gsh "test -f $GUEST_DIR/slam_nav.launch.py && echo guest files ok" || {
  say "FAIL: guest files missing -- scp them to $GUEST_DIR (see README)"; exit 1; }
# Push tools/ and config/ on every run. This used to copy only the two probes,
# on the argument that the run's conclusions rest on them -- which was true and
# too narrow. Measured 2026-08-19: a Nav2 parameter fix sat in the repository
# while the guest ran a two-day-old copy, and the run failed with the exact
# symptom the fix removes. A stale guest file cannot announce itself; it can only
# produce a result that describes code nobody is reading. Both directories are a
# few hundred kB, so the copy is cheaper than one wasted run.
# robot.urdf is NOT here: it is generated per robot in step 1.
#
# config/*.xml is in the list and that is not cosmetic: guest_tcp_client.xml is the
# single most load-bearing file in the split -- it is what makes the guest dial the
# host's acceptors -- and until 2026-08-19 this glob was `*.py *.yaml`, so it was
# the one file the "push everything every run" rule did not push. Adding a host
# listening port means adding an initialPeersList entry there, and with the old glob
# that edit would have sat in the repository while the guest kept a copy that never
# dialled the new port. The host_*.xml profiles ride along unused, exactly as
# bridge_*.yaml already does.
if [ "$GUI" = 1 ]; then
  # Two extra locators, appended by rewriting the closing tag. Written into config/
  # so it rides the same glob, and regenerated every run so it cannot drift from the
  # file it is derived from.
  awk '/<\/initialPeersList>/ {
         for (p = 42103; p <= 42105; p++) {
           print "            <locator>"
           print "              <tcpv4>"
           print "                <address>10.0.2.2</address>"
           print "                <physical_port>" p "</physical_port>"
           print "              </tcpv4>"
           print "            </locator>"
         }
       } { print }' \
      "$ROOT/config/guest_tcp_client.xml" > "$ROOT/config/guest_tcp_client_demo.xml" || {
    say "FAIL: could not generate the demo guest profile"; exit 1; }
  xmllint --noout "$ROOT/config/guest_tcp_client_demo.xml" 2>/dev/null || {
    say "FAIL: the generated demo guest profile is not well-formed XML"; exit 1; }
  say "      GUI=1 -- generated $GUEST_PROFILE (adds 42103 rviz2, 42104 overlay,"
  say "      42105 ras_monitor -- see config/host_ras_server.xml)"
fi

scp -q -P "$PORT" -o StrictHostKeyChecking=no \
    "$ROOT"/tools/*.py "$ROOT"/config/*.yaml "$ROOT"/config/*.xml \
    "$GUEST:$GUEST_DIR/" || {
  say "FAIL: could not copy tools and config to the guest"; exit 1; }
docker exec "$CONTAINER" bash -lc 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*' >/dev/null 2>&1
gsh "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*" >/dev/null 2>&1

say "1/10  host: world + robot for $ROBOT"
[ "$GUI" = 1 ] && say "      GUI=1 -- demonstration run, NOT a measurement run"
ODOM_ARG=""
[ -n "$ODOM_HZ" ] && ODOM_ARG="--odom-hz $ODOM_HZ"
# RTF is written INTO the world, never set at runtime. This script used to call
#   gz service -s /world/$WORLD_NAME/set_physics --req 'real_time_factor: N'
# after gz sim came up, which is the call that failed G2b five runs out of five:
# gz.msgs.Physics has no optional fields, so a request naming only
# real_time_factor also sends max_step_size 0 and rigid-body motion stops dead.
# Measured 2026-08-17: TRUE travel 0.007 m and yaw span 0.000 while odometry
# happily reported 3.99 m. run_container_loop.sh was fixed at the time and this
# file was missed, so RTF_LIMIT here was a trap rather than a knob.
RTF_ARG=""
[ "$RTF_LIMIT" != "0" ] && RTF_ARG="--rtf $RTF_LIMIT"
CAM_ARG=""
[ "$YOLO" = 1 ] && [ "$YOLO_CAM" = robot ] && \
  CAM_ARG="--robot-camera --camera-rate $YOLO_RATE --camera-width $YOLO_W --camera-height $YOLO_H"
dex "python3 /work/tools/make_world.py --robot $ROBOT --out-dir /work/artifacts \
     $GUI_ARG $ODOM_ARG $RTF_ARG $CAM_ARG" || exit 1
docker cp "$CONTAINER":/work/artifacts/robot.env /tmp/robot.env.$$ >/dev/null 2>&1
. /tmp/robot.env.$$ && rm -f /tmp/robot.env.$$
[ -n "$MIN_COVERAGE_OVERRIDE" ] && MIN_COVERAGE=$MIN_COVERAGE_OVERRIDE
[ -n "$MIN_PRECISION_OVERRIDE" ] && MIN_PRECISION=$MIN_PRECISION_OVERRIDE
[ -n "$MIN_RECALL_OVERRIDE" ] && MIN_RECALL=$MIN_RECALL_OVERRIDE
MAP_GATES="--min-coverage ${MIN_COVERAGE:-0}"
[ -n "$MIN_PRECISION" ] && MAP_GATES="$MAP_GATES --min-precision $MIN_PRECISION"
[ -n "$MIN_RECALL" ] && MAP_GATES="$MAP_GATES --min-recall $MIN_RECALL"
if [ "$EXPLORE" = 1 ]; then
  say "      EXPLORE=1 -- explore_lite drives; the waypoint tour is SKIPPED"
  gsh "test -x /opt/ros/jazzy/lib/explore_lite/explore" >/dev/null 2>&1 || {
    say "FAIL: explore_lite is not installed in the guest. In the guest:"
    say "        sudo apt-get install -y ros-jazzy-explore-lite"
    say "      2 packages, 318 kB; every C++ dependency is already there."
    say "      This script installs nothing."
    exit 1; }
  gsh "test -f $GUEST_DIR/explore_split.yaml" >/dev/null 2>&1 || {
    say "FAIL: explore_split.yaml is not in $GUEST_DIR -- the scp at step 0 should"
    say "      have put it there. config/*.yaml is copied on every run."
    exit 1; }
  # Coverage is the number exploration exists to raise, and MIN_COVERAGE is already
  # an env override, so an EXPLORE run judged against a different floor passes
  # MIN_COVERAGE= on the command line instead of needing a knob of its own. The
  # floor in force is printed because a coverage PASS is the headline of this mode.
  say "      coverage floor in force: ${MIN_COVERAGE:-0} (MIN_COVERAGE= overrides)"
fi
if [ "$YOLO" = 1 ]; then
  say "      YOLO=1 ($YOLO_CAM camera) -- merging the camera bridge entry"
  [ "${YOLO_RTF_DEFAULTED:-0}" = 1 ] && \
    say "      RTF_LIMIT was unset or 0, so it is 0.25 -- three workloads do not fit faster"
  if [ "${YOLO_RTF_HIGH:-0}" = 1 ]; then
    say "      WARNING: RTF_LIMIT=$RTF_LIMIT with YOLO=1. At 0.5 the guest clock fell"
    say "      2.8 s behind and Nav2 aborted every goal (RUNLOG, G-Y5 attempt 1)."
    say "      0.25 is the measured value; this run is yours."
  fi
  gsh "test -f $YOLO_DIR/yolo_node.py && test -f $YOLO_DIR/yolov8n.onnx" >/dev/null 2>&1 || {
    say "FAIL: the detector is not installed in the guest -- ../yolo/scripts/install_guest.sh"
    exit 1; }
  dex "mkdir -p /work/artifacts/yolo_scene" >/dev/null 2>&1
  # Not used by this script -- it is the viewer half, started by hand when there
  # is an audience. Copied here so the command in README.md's "Demonstrating it"
  # has something to run without a second manual copy.
  docker cp "$ROOT/../yolo/tools/detection_overlay.py" \
            "$CONTAINER":/work/artifacts/yolo_scene/ >/dev/null 2>&1 || true
  if [ "$YOLO_CAM" = robot ]; then
    # The camera is already in robot.sdf (make_world.py --robot-camera, above).
    # What is built here is what it will look at: the person models. The meshes
    # come from the Fuel cache; the one-time download is ~2 MB per model and is
    # done here rather than left to a FAIL message because the cache is
    # container-local state a fresh container will not have.
    docker cp "$ROOT/../yolo/tools/make_people.py" \
              "$CONTAINER":/work/artifacts/yolo_scene/ >/dev/null || {
      say "FAIL: ../yolo/tools/make_people.py is missing"; exit 1; }
    dex "test -d '/home/vela/.gz/fuel/fuel.gazebosim.org/openrobotics/models/standing person/3' &&
         test -d '/home/vela/.gz/fuel/fuel.gazebosim.org/openrobotics/models/walking person/3'" \
      >/dev/null 2>&1 || {
      say "      downloading person models from Gazebo Fuel (one-time, ~4 MB)"
      dex "gz fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person' &&
           gz fuel download -u 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Walking person'" \
        >/dev/null || { say "FAIL: Fuel download failed -- no network in the container?"; exit 1; }; }
    # The check above is `dex`, so it only ever asks $CONTAINER -- the SERVER. The
    # GUI is a different container from a different image, and make_people.py
    # writes ABSOLUTE mesh URIs (file:///home/vela/.gz/fuel/...), so a viewer
    # without the same cache renders the robot and silently drops every person.
    # That is what the 2026-08-22 GUI run did: nothing failed, the people were
    # just not there. Under GUI=1 the host copy run_demo.sh mounts is checked too,
    # here at step 6 rather than twenty minutes later when the viewer starts.
    if [ "$GUI" = 1 ]; then
      FC_STANDING="$FUEL_CACHE/fuel.gazebosim.org/openrobotics/models/standing person/3/meshes/standing.dae"
      FC_WALKING="$FUEL_CACHE/fuel.gazebosim.org/openrobotics/models/walking person/3/meshes/walking.dae"
      if [ ! -f "$FC_STANDING" ] || [ ! -f "$FC_WALKING" ]; then
        say "FAIL: GUI=1 but the host Fuel cache has no person meshes"
        say "      checked: $FC_STANDING"
        say "               $FC_WALKING"
        say "      $CONTAINER has them, the viewer image does not, and nothing"
        say "      copies them across. Seed the host copy once:"
        say "        mkdir -p \"$FUEL_CACHE\""
        say "        docker cp '$CONTAINER:/home/vela/.gz/fuel/.' \"$FUEL_CACHE\"/"
        say "      run_demo.sh mounts it read-only at /home/vela/.gz/fuel."
        exit 1
      fi
      say "      host Fuel cache OK for the viewer ($FUEL_CACHE)"
    fi
    dex "python3 /work/artifacts/yolo_scene/make_people.py \
         --out-dir /work/artifacts/people" \
      || { say "FAIL: could not build the person models"; exit 1; }
    CAMERA_FRAGMENT=/work/config/camera_bridge_fragment.yaml
  else
    # The billboard photographs live outside the repository (nothing binary is
    # committed), so they are copied in rather than mounted.
    ls "$YOLO_ASSETS"/*.jpg >/dev/null 2>&1 || {
      say "FAIL: no images in $YOLO_ASSETS -- see ../yolo/README.md"; exit 1; }
    for img in "$YOLO_ASSETS"/*.jpg; do
      docker cp "$img" "$CONTAINER":/work/artifacts/yolo_scene/ >/dev/null || exit 1
    done
    # The container mounts gz-split as /work and nothing else, so ../yolo is not
    # visible from inside it. Copy the generator in rather than reaching for a
    # path that resolves to the container's own root.
    docker cp "$ROOT/../yolo/tools/make_billboards.py" \
              "$CONTAINER":/work/artifacts/yolo_scene/ >/dev/null || {
      say "FAIL: ../yolo/tools/make_billboards.py is missing"; exit 1; }
    dex "python3 /work/artifacts/yolo_scene/make_billboards.py \
         --images /work/artifacts/yolo_scene/*.jpg \
         --out-dir /work/artifacts/yolo_scene --rate $YOLO_RATE \
         --width $YOLO_W --height $YOLO_H" \
      || { say "FAIL: could not build the camera scene"; exit 1; }
    CAMERA_FRAGMENT=/work/artifacts/yolo_scene/camera_bridge_fragment.yaml
  fi
  # One bridge participant, not a second one: a second host participant needs its
  # own listening port and an entry in the guest's initialPeersList, and without
  # them it is discoverable, unreachable and silent. So the camera entry is
  # appended to a COPY of the measured config -- config/ itself is never edited,
  # or G2's and G5's numbers would stop describing the file they were measured on.
  dex "cat $BRIDGE_FILE $CAMERA_FRAGMENT \
       > /work/artifacts/bridge_with_camera.yaml" || exit 1
  BRIDGE_FILE=/work/artifacts/bridge_with_camera.yaml
fi

say "      world=$WORLD_NAME bridge=$BRIDGE_FILE base=$BASE_FRAME offset=$MAP_OFFSET"
say "      map gates: $MAP_GATES (unset floors come from score_map.py)"

# WAYPOINTS comes from robot.env (world coordinates, per robot), so it has to be
# overridden AFTER that file is sourced rather than through the environment.
# WAYPOINTS_OVERRIDE exists for recordings: tb3 ships two waypoints, and the first
# goal after bringup sometimes aborts on a TF lookup at now() while the guest clock
# is still catching up -- measured on 5 of the last 12 split runs. On a two-leg tour
# that single abort costs half the drive and the map never grows enough to score, so
# a take is lost to something that is not Nav2 misbehaving. Repeating the two proven
# points doubles the footage and survives one abort. It changes what the run
# measures, so it is off by default and no recorded G5 number used it.
if [ -n "${WAYPOINTS_OVERRIDE:-}" ]; then
  WAYPOINTS="$WAYPOINTS_OVERRIDE"
  say "      WAYPOINTS_OVERRIDE in effect: $WAYPOINTS"
fi

# tb4 needs the expanded URDF in the guest as well: robot_state_publisher there
# supplies the base_link->shell_link->rplidar_link chain that Gazebo's /tf omits.
# Only the file is needed, not the description package -- robot_state_publisher
# never loads the meshes the URDF references.
if [ "$SPAWN_MODE" = topic ]; then
  say "      copying $(basename "$ROBOT_FILE") to the guest for robot_state_publisher"
  docker cp "$CONTAINER":"$ROBOT_FILE" /tmp/robot.urdf.$$ >/dev/null 2>&1
  scp -q -P "$PORT" -o StrictHostKeyChecking=no /tmp/robot.urdf.$$ \
      "$GUEST:$GUEST_DIR/robot.urdf" && rm -f /tmp/robot.urdf.$$
fi

say "2/10  host: gz sim, headless"
dexd "DISPLAY= gz sim -s -r -v2 --headless-rendering $WORLD_FILE \
      > /work/artifacts/gz-split-$ROBOT-$STAMP.log 2>&1"
for i in $(seq 1 36); do
  sleep 5
  dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" && break
done
dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" || {
  say "FAIL: gz sim never published stats"; dex "tail -30 /work/artifacts/gz-split-$ROBOT-$STAMP.log"; exit 1; }

say "3/10  host: spawn robot ($SPAWN_MODE mode)"
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
if [ "$YOLO" = 1 ]; then
  if [ "$YOLO_CAM" = robot ]; then
    # The camera spawned with the robot; these are the people it will look at.
    # Spawned BEFORE the guest stack starts, so slam_toolbox's very first scan
    # already contains their legs -- people appearing mid-map would look like a
    # mapping fault. Real obstacles, deliberately: the lidar sees the meshes,
    # the local costmap inflates them, Nav2 gives them the same berth as a
    # cylinder. Each tuple is x,y,yaw,kind -- see YOLO_PEOPLE at the top.
    N=0
    for person in $YOLO_PEOPLE; do
      IFS=, read -r PX PY PYAW PKIND <<< "$person"
      N=$((N + 1))
      say "      spawning person_$N ($PKIND) at ($PX, $PY) facing $PYAW"
      dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name person_$N \
           -file /work/artifacts/people/person_$PKIND.sdf \
           -x $PX -y $PY -z 0 -Y $PYAW" \
        || { say "FAIL: could not spawn person_$N"; exit 1; }
    done
  else
    say "      spawning the camera scene at ($YOLO_X, $YOLO_Y), clear of the arena"
    dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name yolo_scene \
         -file /work/artifacts/yolo_scene/yolo_scene.sdf -x $YOLO_X -y $YOLO_Y -z 0" \
      || { say "FAIL: could not spawn the camera scene"; exit 1; }
  fi
  # Gate it here, not at the end. `create` returning 0 means the spawn request was
  # accepted, not that the sensor is publishing -- a camera that never advertises
  # leaves the detector with nothing, which this script does not notice until the
  # G-Y5 check 17 minutes later. At 0.1 Hz the first message is up to 10 s away,
  # so wait rather than sample once. In robot mode the sensor rides robot.sdf and
  # this wait normally clears on the first probe.
  say "      waiting for the camera to advertise /camera on gz transport"
  CAM=no
  for _ in $(seq 1 12); do
    sleep 5
    dex "gz topic -l 2>/dev/null | grep -qx /camera" && { CAM=yes; break; }
  done
  [ "$CAM" = yes ] || {
    say "FAIL: /camera never appeared on gz transport."
    say "      robot mode: check the yolo_camera sensor in /work/artifacts/robot.sdf;"
    say "      scene mode: check /work/artifacts/yolo_scene/yolo_scene.sdf. Then:"
    dex "gz topic -l 2>/dev/null | head -20"
    exit 1; }
  say "      /camera is live"
fi
sleep 10

say "4/10  host: bridge + clock decimator + cmd_vel watchdog, on the TCP profile"
# -r /clock:=/clock_gz is what makes room for the decimator, and it is done here
# rather than in the bridge config because that config is shared with
# run_container_loop.sh, which needs a plain /clock. Both robots' configs publish
# /clock; only this runner renames it.
dexd "$HOST_ENV ros2 run ros_gz_bridge parameter_bridge --ros-args \
      -p config_file:=$BRIDGE_FILE \
      -p expand_gz_topic_names:=$EXPAND_GZ_TOPICS -p use_sim_time:=true \
      -r /clock:=/clock_gz \
      > /work/artifacts/bridge-$STAMP.log 2>&1"
# The bridge now publishes the raw clock on /clock_gz; this publishes the real /clock
# at CLOCK_HZ. Without it the guest's nodes cannot service a 100 Hz clock and their
# own time falls tens of seconds behind the TF stamps arriving beside it -- see
# RUNLOG.md Step 8. use_sim_time must be false: this node is what defines sim time.
# Its own profile and its own listening port, 42101. Sharing host_tcp_server.xml
# leaves it without a TCP acceptor -- the bridge already owns 42100 -- and the guest
# then lists /clock and receives nothing while the decimator forwards happily.
dexd "$CLOCK_ENV python3 /work/tools/clock_decimator.py --in /clock_gz --out /clock \
      --hz ${CLOCK_HZ:-20} --ros-args -p use_sim_time:=false \
      > /work/artifacts/clockdec-$STAMP.log 2>&1"
# Its own profile and its own listening port, 42102, for the same reason the
# decimator has 42101: a TCP acceptor has exactly one owner, and this node has to
# RECEIVE /cmd_vel from the guest. On $HOST_ENV it lost the bind to the bridge and
# ran the whole tb4 drive with no acceptor -- seeing nothing, and unable to deliver
# its own zeros either. See config/host_cmdvel_server.xml.
dexd "$CMDVEL_ENV python3 /work/tools/cmd_vel_watchdog.py --ros-args -p use_sim_time:=true \
      > /work/artifacts/watchdog-$STAMP.log 2>&1"
sleep 12
# Assert the decimator is actually forwarding before waiting 900 s on a bringup that
# cannot succeed without a clock.
dex "grep -q 'clock_decimator' /work/artifacts/clockdec-$STAMP.log" || {
  say "FAIL: clock_decimator did not start"; dex "tail -10 /work/artifacts/clockdec-$STAMP.log"
  exit 1; }

say "5/10  guest: can it see the bridged clock and scan?"
gsh "$GUEST_ENV timeout 60 ros2 topic list | grep -E '^/(clock|scan|odom|cmd_vel|tf)$' | sort | tr '\n' ' '"
echo
CLK=$(gsh "$GUEST_ENV timeout 40 ros2 topic hz /clock --window 5 2>/dev/null | grep -m1 average")
say "      /clock: ${CLK:-NOT ARRIVING}"
if [ -z "$CLK" ]; then
  say "FAIL: the guest is not receiving /clock. Run run_dds_check.sh (G3) first."
  exit 1
fi

say "6/10  guest: Nav2 + slam_toolbox${WATCHDOG:+ + liveliness watchdog}"
gsh "$GUEST_ENV nohup ros2 launch $GUEST_DIR/slam_nav.launch.py \
     robot:=$ROBOT_KIND urdf_file:=$GUEST_DIR/robot.urdf \
     nav_params_file:=$GUEST_DIR/nav2_split.yaml \
     slam_params_file:=$GUEST_DIR/mapper_params_tcg.yaml \
     > \$HOME/split-stack-$STAMP.log 2>&1 &" >/dev/null 2>&1
if [ -n "$WATCHDOG" ]; then
gsh "$GUEST_ENV nohup python3 $GUEST_DIR/liveliness_watchdog.py \
     --out \$HOME/liveliness-$STAMP.csv --budget-ms $TOPIC_BUDGET_MS \
     --rtf ${RTF_LIMIT:-1} \
     > \$HOME/live-$STAMP.log 2>&1 &" >/dev/null 2>&1
fi
# Started with the stack, not after it, because the lag this measures is built up
# during bringup: bt_navigator blocks for ~9 s at a time parsing behaviour-tree
# XML under TCG, and whatever clock backlog that leaves is still there when the
# first goal arrives. use_sim_time TRUE deliberately -- it has to be subject to
# the same clock as the Nav2 nodes to say anything about them.
gsh "$GUEST_ENV nohup python3 $GUEST_DIR/clock_lag_probe.py --every 10 \
     --ros-args -p use_sim_time:=true > \$HOME/clocklag-$STAMP.log 2>&1 &" >/dev/null 2>&1
if [ "$YOLO" = 1 ]; then
  # Started with the stack rather than after the tour, so the CPU it takes is
  # present during bringup and during the drive -- which is the whole point of a
  # co-tenancy measurement. use_sim_time is deliberately NOT set: the detector
  # has no relationship to sim time, and giving it one would make it wait for a
  # clock it does not need.
  say "      + yolo_node (G-Y5 co-tenant)"
  gsh "$GUEST_ENV nohup python3 $YOLO_DIR/yolo_node.py --ros-args \
       -p model_path:=$YOLO_DIR/yolov8n.onnx \
       > \$HOME/yolo-node-$STAMP.log 2>&1 &" >/dev/null 2>&1
fi

say "      waiting for lifecycle activation (up to 900s; bringup was 56s locally,"
say "      and every TF lookup now crosses a TCP link)"
ACTIVE=no
HINTED=no
for i in $(seq 1 180); do
  sleep 5
  if gsh "grep -qE 'Managed nodes are active' \$HOME/split-stack-$STAMP.log"; then
    ACTIVE=yes; say "      active after $((i * 5))s"; break
  fi
  if gsh "grep -qE 'CRITICAL FAILURE|process has died' \$HOME/split-stack-$STAMP.log"; then
    say "FAIL: stack died"; gsh "tail -40 \$HOME/split-stack-$STAMP.log"; exit 1
  fi
  # Same stall as G2, and here it is also the shape a dead /scan across the DDS
  # link takes: no scans means no map->odom means no map frame.
  if [ "$HINTED" = no ] && [ "$i" -ge 24 ] &&
     gsh "grep -qF 'Invalid frame ID \"map\"' \$HOME/split-stack-$STAMP.log"; then
    HINTED=yes
    say "      NOTE: costmaps are waiting on a map frame that does not exist --"
    say "      slam_toolbox is not publishing map->odom. In the guest:"
    say "        ros2 lifecycle get /slam_toolbox     ('unconfigured' = not managed)"
    say "        ros2 topic hz /scan                  (nothing = the bridge link is down, see G3)"
  fi
done
[ "$ACTIVE" = yes ] || {
  say "FAIL: never activated"; gsh "tail -60 \$HOME/split-stack-$STAMP.log"; exit 1; }

if [ "$YOLO" = 1 ]; then
  # Do the camera frames reach the GUEST? The gz-side gate above proves the sensor
  # publishes; this proves the bridge entry, the TCP link and the guest's own
  # subscription all work, which are three more places it can silently fail.
  #
  # Read it off the detector's own heartbeat rather than probing the topic: at
  # 0.1 Hz of sim time and RTF_LIMIT=0.25 a frame arrives every 40 s of wall
  # clock, so `ros2 topic hz /camera/image_raw` needs a minute and a half to say
  # anything, while the node publishes rx on /diagnostics every second. One echo,
  # one second, and no participant that could contend for a port.
  say "      checking the detector is receiving frames (its own /diagnostics)"
  YSTAT=$(gsh "$GUEST_ENV timeout 90 ros2 topic echo --once --field status                /diagnostics 2>/dev/null" 2>/dev/null | tr -d ' ' | tr '\n' ' ')
  # `ros2 topic echo --field status` prints a Python REPR, not YAML: one long line of
  # KeyValue(key='rx', value='14'). Two earlier attempts at this parse assumed YAML
  # and reported rx=? while the detector's own log showed rx=3 -- so the shape is
  # taken from a captured sample rather than from what the flag's name suggests
  # (measured 2026-08-19, take 4).
  YRX=$(printf '%s' "$YSTAT" | sed -n "s/.*key='rx', value='\([0-9]*\)'.*/\1/p" | head -1)
  case "$YSTAT" in
    *state*) : ;;
    *) say "      WARNING: no /diagnostics from the detector in 90s -- it may not be up."
       gsh "tail -12 \$HOME/yolo-node-$STAMP.log" ;;
  esac
  if [ -n "$YRX" ] && [ "$YRX" -gt 0 ] 2>/dev/null; then
    say "      the detector has received $YRX frame(s)"
  else
    say "      NOTE: the detector has no frames yet (rx=${YRX:-?}). At 0.1 Hz sim and"
    say "      RTF_LIMIT=$RTF_LIMIT that is up to 40 s apart, so this is only a"
    say "      failure if the G-Y5 check at the end also finds no [detect] line."
  fi
fi

say "7/10  guest: is the guest's clock keeping up with the host's?"
# The failure this exists to catch, measured 2026-08-17: bt_navigator's own clock
# sat 34-42 s behind the TF stamps arriving beside it and aborted every goal with
#   [bt_navigator]: Initial robot pose is not available.
# which names TF and not the clock, so it reads as a SLAM or a link fault. Both
# runs before this check went the whole way to 0/2 waypoints and a failed map save
# before showing anything. `ros2 topic hz /clock` cannot see it either -- samples
# do arrive at CLOCK_HZ; the deficit is on the consuming side.
#
# Host sim time comes from gz stats, guest sim time from the probe's own
# get_clock(). They are read a fraction of a second apart, which is noise against
# a lag measured in tens of seconds.
#
# What this check does NOT prove: the probe is one more participant with one
# subscription and nothing else to do, so it is the best case in this guest. A
# pass here means the link and the decimator are delivering time correctly and
# that a node CAN keep up; it does not mean a loaded Nav2 node did. When the two
# disagree -- probe current, bt_navigator tens of seconds behind -- that
# disagreement is itself the finding, and it points at guest CPU rather than at
# the clock path. See clock_lag_probe.py.
LAG=""
for i in $(seq 1 "${CLOCK_SETTLE_TRIES:-24}"); do
  # sim_time is a nested Time message, so `sec:` appears again under real_time;
  # take the first one after sim_time and stop. $1=="sec:" and not /sec:/, which
  # would also match nsec:.
  HOST_SIM=$(dex "gz topic -e -t /world/$WORLD_NAME/stats -n 1 2>/dev/null" \
             | awk '/sim_time/{s=1} s&&$1=="sec:"{print $2; exit}')
  GUEST_SIM=$(gsh "sed -n 's/.*node_clock= *\([0-9.]*\).*/\1/p' \
              \$HOME/clocklag-$STAMP.log | tail -1")
  if [ -n "$HOST_SIM" ] && [ -n "$GUEST_SIM" ]; then
    LAG=$(awk -v h="$HOST_SIM" -v g="$GUEST_SIM" 'BEGIN{printf "%.1f", h-g}')
    say "      host sim ${HOST_SIM}s   guest clock ${GUEST_SIM}s   lag ${LAG}s"
    awk -v l="$LAG" -v m="${CLOCK_LAG_MAX:-5}" 'BEGIN{exit !(l<=m)}' && break
  fi
  sleep 10
done
if [ -z "$LAG" ]; then
  # Not fatal: an unmeasurable lag is a defect in this check, and killing a run
  # that is otherwise healthy over it would cost more than it saves. The tour
  # below still reports the truth, just less directly.
  say "      WARNING: could not measure the lag (no probe output, or no gz stats)."
  say "      Continuing. If the tour aborts with 'Initial robot pose is not"
  say "      available', this is why, and \$HOME/clocklag-$STAMP.log says how badly."
elif ! awk -v l="$LAG" -v m="${CLOCK_LAG_MAX:-5}" 'BEGIN{exit !(l<=m)}'; then
  say "FAIL: the guest's clock is ${LAG}s behind the host's and did not catch up in"
  say "      $(( ${CLOCK_SETTLE_TRIES:-24} * 10 ))s. Every TF lookup at now() will fail."
  say "      CLOCK_HZ is ${CLOCK_HZ:-20}; lower it so each sample carries more sim"
  say "      time, and read the diagnosis at the top of clock_lag_probe.py."
  gsh "tail -6 \$HOME/clocklag-$STAMP.log"
  exit 1
else
  say "      inside CLOCK_LAG_MAX=${CLOCK_LAG_MAX:-5}s -- TF lookups at now() can resolve"
fi

if [ "$EXPLORE" = 1 ]; then
say "8/10  guest: explore_lite -- the robot picks its own goals (up to ${EXPLORE_TIMEOUT}s)"
# Started here and not in slam_nav.launch.py, for three reasons that are properties
# of this rig rather than preferences:
#   * that launch file is shared with run_container_loop.sh. A node added there
#     would silently change the configuration G2's published numbers describe.
#   * Costmap2DClient's constructor BLOCKS until the first costmap arrives and until
#     map->base_footprint resolves. In the launch file that block spans the whole
#     60-123 s bringup, and a co-tenant during bringup is measured harm here, not a
#     theory: yolo_node took bringup from 65-70 s to 110 s and every goal aborted.
#   * teardown is by pattern, so one more guest process is one entry in GUEST_PROCS
#     -- exactly how send_goal.py is handled.
# Do NOT add it to lifecycle_manager_nav.node_names either: explore_lite is a plain
# rclcpp::Node, not a lifecycle node, and the manager would wait forever for a
# transition it cannot request.
#
# nohup and & because explore_lite never exits on its own -- it stops issuing goals
# and idles. The gate is a log poll, the same shape as the activation wait above,
# and it costs the guest nothing: a second rclpy subscription just to read
# /explore/status would cost 40-52% of an emulated core, which is what
# clock_lag_probe.py measures for exactly one subscription.
if [ -n "$EXPLORE_SEED" ] && [ "$EXPLORE_SEED" -gt 0 ]; then
  SEED_WP=$(echo "$WAYPOINTS" | awk -v n="$EXPLORE_SEED" '{for (i = 1; i <= n && i <= NF; i++) printf "%s%s", $i, (i < n && i < NF ? " " : "")}')
  say "      seeding $EXPLORE_SEED waypoint(s) first: $SEED_WP"
  say "      (explore_lite cannot bootstrap from a standstill -- see DESIGN.md)"
  gsh "set -o pipefail; $GUEST_ENV python3 $GUEST_DIR/send_goal.py \
       --waypoints '$SEED_WP' --base-frame $BASE_FRAME \
       --world-offset=$SPAWN_X,$SPAWN_Y \
       --track-csv \$HOME/track-seed-$STAMP.csv 2>&1 | tail -12"
  # Deliberately not gated. A seed goal that aborts still moves the robot part of the
  # way, and partway is enough to give slam_toolbox a map; only a seed that moved the
  # robot nowhere matters, and the frontier-distance check below already catches that.
  say "      seed finished (rc=$?, not gated); starting explore_lite"
fi
# Baseline, because the seed above drives through the SAME action server and logs the
# same lines. Without subtracting it the gate passed on the seed's own two goals while
# explore_lite issued none -- measured 2026-08-19: "2 goal(s) accepted, 2 of them
# > 0.5 m away" was entirely the seed, and G5-E would have been called a pass.
GOALS_PRE=$(gsh "grep -c 'Begin navigating' \$HOME/split-stack-$STAMP.log" 2>/dev/null | tr -d '[:space:]')
GOALS_PRE=${GOALS_PRE:-0}
SUBST_PRE=$(gsh "grep -o 'Begin navigating from current location (.*) to (.*)' \
     \$HOME/split-stack-$STAMP.log | tr -d '(),' \
     | awk '{dx=\$9-\$6; dy=\$10-\$7; if (sqrt(dx*dx+dy*dy) > 0.5) n++} END{print n+0}'" \
     2>/dev/null | tr -d '[:space:]')
SUBST_PRE=${SUBST_PRE:-0}
[ "$GOALS_PRE" -gt 0 ] && say "      baseline before explore_lite: $GOALS_PRE goal(s), $SUBST_PRE substantive (the seed)"
gsh "$GUEST_ENV nohup ros2 run explore_lite explore --ros-args \
     --params-file $GUEST_DIR/explore_split.yaml \
     > \$HOME/explore-$STAMP.log 2>&1 &" >/dev/null 2>&1
EXPLORE_DONE=no
# Ceiling division, and at least one poll. With floor division an
# EXPLORE_TIMEOUT under 15 gave `seq 1 0` -- the body never ran, so nothing waited,
# nothing checked the terminal messages, and the run reported a bare exploration
# failure that looked identical to a rig that cannot explore.
for i in $(seq 1 $(( EXPLORE_TIMEOUT < 15 ? 1 : (EXPLORE_TIMEOUT + 14) / 15 ))); do
  sleep 15
  # Both terminal messages mean the frontier list came back empty. Literals taken
  # from the binary: "No frontiers found, stopping." and "All frontiers
  # traversed/tried out, stopping."
  if gsh "grep -qE 'frontiers found, stopping|frontiers traversed' \$HOME/explore-$STAMP.log"; then
    EXPLORE_DONE=yes; say "      exploration complete after $((i * 15))s"; break
  fi
  # The one unrecoverable configuration error, and it names itself: the robot is not
  # inside the grid explore_lite is reading, i.e. costmap_topic or robot_base_frame
  # is wrong. Not a slow guest, so fail rather than wait out the timeout.
  if gsh "grep -q 'Robot out of costmap bounds' \$HOME/explore-$STAMP.log"; then
    say "FAIL: explore_lite cannot find the robot in its costmap. costmap_topic or"
    say "      robot_base_frame in explore_split.yaml is wrong. In the guest:"
    say "        ros2 topic hz /map            (nothing = slam_toolbox is not mapping)"
    say "        ros2 run tf2_ros tf2_echo map $BASE_FRAME"
    gsh "tail -20 \$HOME/explore-$STAMP.log"; exit 1
  fi
  if [ $((i % 8)) -eq 0 ]; then
    G=$(gsh "grep -c 'Begin navigating' \$HOME/split-stack-$STAMP.log" 2>/dev/null | tr -d '[:space:]')
    say "      t+$((i * 15))s  goals since explore_lite started: $(( ${G:-0} - GOALS_PRE ))"
  fi
done
# Counted from NAV2's log, not from explore_lite's. Measured 2026-08-19: explore's
# own "Goal ACCEPTED, uuid: %s" literal exists in the binary but did not appear in
# the redirected log during a run where Nav2 demonstrably accepted 243 goals, so a
# gate reading that line counted 0 while the mode was working. Nav2's own
# "Begin navigating from current location (x, y) to (gx, gy)" is the signal that
# cannot disagree with what happened, and it carries both poses.
GOALS=$(gsh "grep -c 'Begin navigating' \$HOME/split-stack-$STAMP.log" 2>/dev/null | tr -d '[:space:]')
GOALS=$(( ${GOALS:-0} - GOALS_PRE ))
# And of those, the ones worth driving to. This is the gate, because a goal at the
# robot's own feet is indistinguishable from progress in a bare count: at
# min_frontier_size 0.30 this rig produced 243 goals averaging 0.13 m and the robot
# never moved. > 0.5 m is Nav2's own cross-check limit for "arrived somewhere else".
SUBSTANTIVE=$(gsh "grep -o 'Begin navigating from current location (.*) to (.*)' \
     \$HOME/split-stack-$STAMP.log | tr -d '(),' \
     | awk '{dx=\$9-\$6; dy=\$10-\$7; if (sqrt(dx*dx+dy*dy) > 0.5) n++} END{print n+0}'" \
     2>/dev/null | tr -d '[:space:]')
SUBSTANTIVE=$(( ${SUBSTANTIVE:-0} - SUBST_PRE ))
BLACKLISTED=$(gsh "grep -c 'black list' \$HOME/explore-$STAMP.log" 2>/dev/null | tr -d '[:space:]')
say "      explore_lite: $GOALS goal(s) accepted, $SUBSTANTIVE of them > 0.5 m away, ${BLACKLISTED:-0} blacklisted, complete=$EXPLORE_DONE"
if [ "$GOALS" -gt 20 ] && [ "$SUBSTANTIVE" -lt 2 ]; then
  say "      DIAGNOSIS: frontier churn. Many goals, none worth driving to, so the robot"
  say "      stood still. Raise min_frontier_size in config/explore_split.yaml, or lower"
  say "      potential_scale so distance stops dominating frontier size."
fi
# Stop it BEFORE the map is saved. A goal in flight keeps the robot moving while
# map_saver spins the /map subscription for up to 30 s, and the map would then be
# stitched across poses. pkill rather than publishing /explore/resume false, which
# would need a second rclpy process on a guest that cannot afford one.
gsh "pkill -f '[e]xplore_lite'" >/dev/null 2>&1
sleep 5
# Reaching EXPLORE_TIMEOUT is NOT a failure: a time-boxed run still produced a map,
# and step 9's map gate is what judges it. Zero goals IS a failure -- the robot
# never drove, and coverage would say so too (0.135-0.142 for runs that never moved).
if [ "$SUBSTANTIVE" -ge "$EXPLORE_MIN_GOALS" ]; then NAV_RC=0; else NAV_RC=1; fi
else
# PRE_DRIVE_SLEEP exists for recordings, and it is the same knob
# run_container_loop.sh carries. A split run reaches this line about 4.5 minutes
# after it starts -- most of that is Nav2 bringup across the TCP link -- while
# run_demo.sh needs roughly 50 s to bring up Xvfb, compile the GUI's shaders and
# start ffmpeg, plus ~25 s more when RVIZ=1 adds a reachability probe and RViz
# itself. Attaching the viewer at step 6 lands inside that window, so a hold is
# only needed when the operator is later than that; it also lets the guest's TF
# buffer fill, which is what the first goal aborts on when it aborts. Zero by
# default: a measurement run must not wait around, and every G5 number was taken
# without it.
if [ "${PRE_DRIVE_SLEEP:-0}" != 0 ]; then
  say "      holding ${PRE_DRIVE_SLEEP}s before driving so a recorder can attach"
  sleep "$PRE_DRIVE_SLEEP"
fi

say "8/10  guest: waypoint tour -- the robot drives in Gazebo: $WAYPOINTS"
# pipefail on the remote side too, or the exit status would be tail's and every
# run would look like a pass.
# `=` form, not a space: every offset here starts with a minus sign and argparse
# would read it as an option name. See the note in run_container_loop.sh.
# SETTLE=N sends up to N zero-distance goals before the tour and starts the
# real waypoints only after bt_navigator has served one -- the fixed-length
# pre-drive sleep cannot guarantee that, because the clock debt it waits out
# depends on what attached to the graph and when (takes 5, 7, 8: 0/8 each,
# bt_navigator or planner_server behind their own TF buffer at drive start).
# tail -60, was -40: the probes add lines and the tour's own 40 must survive.
gsh "set -o pipefail; $GUEST_ENV python3 $GUEST_DIR/send_goal.py \
     --waypoints '$WAYPOINTS' --base-frame $BASE_FRAME \
     --world-offset=$SPAWN_X,$SPAWN_Y \
     ${SETTLE:+--settle $SETTLE} \
     --track-csv \$HOME/track-split-$STAMP.csv 2>&1 | tail -60"
NAV_RC=$?
fi

say "9/10  guest: save and score the SLAM map"
# save_map_timeout defaults to 2.0 s and that is not enough here: measured
# 2026-08-17, two runs died with `Failed to spin map subscription` 2.1 s after
# `Saving map from 'map' topic`, leaving ~/maps empty and no map to score, in runs
# whose real failure was upstream. 30.0 verified as the accepted type and unit --
# the same call with 30.0 failed at 30.07 s instead of 2.1 s.
#
# use_sim_time FALSE, unlike every other node here, and deliberately: the only
# thing map_saver uses a clock for is that timeout, it stamps nothing, and on sim
# time a stalled clock makes the timeout unreachable -- the script would hang
# forever instead of failing in 30 s. A wall timeout cannot do that.
gsh "$GUEST_ENV mkdir -p \$HOME/maps && ros2 run nav2_map_server map_saver_cli \
     -f \$HOME/maps/split-$STAMP --ros-args -p use_sim_time:=false \
     -p save_map_timeout:=30.0" || {
  say "FAIL: map_saver"; exit 1; }
# Scored on the HOST, not in the guest, and the map is copied back for it. MAP_REF
# is nav2_bringup's tb3_sandbox, which is installed in the container and NOT in the
# guest -- the guest deliberately has only slam_toolbox, so scoring there died with
#   FileNotFoundError: /opt/ros/jazzy/share/nav2_bringup/maps/tb3_sandbox.yaml
# after a map that had saved perfectly well. Copying the reference into the guest
# instead would put a second copy of the ground truth on the test bench, and it
# would run numpy dilation on an emulated core for no reason. The saved map is the
# artefact; where it is scored does not affect the score, and this way G2 and G5 go
# through one reference and one scorer.
scp -q -P "$PORT" -o StrictHostKeyChecking=no \
    "$GUEST:~/maps/split-$STAMP.yaml" "$GUEST:~/maps/split-$STAMP.pgm" /tmp/ && \
  docker cp /tmp/split-$STAMP.yaml "$CONTAINER":/work/artifacts/maps/ >/dev/null && \
  docker cp /tmp/split-$STAMP.pgm "$CONTAINER":/work/artifacts/maps/ >/dev/null || {
    say "FAIL: could not bring the saved map back to the host for scoring"; exit 1; }
rm -f /tmp/split-$STAMP.yaml /tmp/split-$STAMP.pgm
# Both files, and no path rewriting: map_saver writes `image:` as a bare filename
# and score_map.py resolves it as yaml_path.parent / image, so the pair is portable
# as long as they travel together.
dex "python3 /work/tools/score_map.py --slam /work/artifacts/maps/split-$STAMP.yaml --ref $MAP_REF \
     --offset=$MAP_OFFSET $MAP_GATES"
MAP_RC=$?

say "10/10  co-tenant load (G6) and optimizer stalls (G7)"
# Named -C processes are NOT enough for G6, whose budget is 250 of 400. The three
# this used to list came to ~99%, which reads as comfortable. The truth on
# 2026-08-17 was loadavg 23.12 on 4 cores -- 5.8x oversubscribed -- because
# bt_navigator, behavior_server and two Python nodes were never counted, and one of
# those Python nodes was averaging 93% of a core on its own. Any G6 number from a
# partial process list understates the load, so list everything and let the log
# show which process it was, rather than inferring it afterwards from elapsed time.
#
# ps %CPU is the average over each process's lifetime, not an instantaneous
# sample, which is what G6 wants: a run-long figure. loadavg is printed beside it
# because %CPU cannot show queueing -- 8 processes averaging 30% each on 4 cores
# is a very different machine from 2 averaging 120%.
# explore_lite is in the pattern as a TRIPWIRE, not as accounting. This census runs
# at step 10, after the drive, so it never sees a drive-window process: send_goal.py
# is absent from every listing for the same reason, and step 8 pkills explore_lite
# before step 9. If explore_lite ever DOES appear here, it survived teardown and the
# next run is already poisoned -- that is what the entry is for. The drive-window
# load is a different measurement and this script does not make it.
# Summed over the MATCHED processes only, and captured once into a variable so the
# listing and the total cannot come from two different ps runs. Summing every
# process instead would be meaningless: %CPU is each process's own lifetime
# average, so short-lived ones (ssh, ps itself) contribute large numbers -- that
# version read 309% on an idle guest whose loadavg was 1.10. These matched
# processes all live for the whole run, so each one's lifetime average IS its
# run average, and the sum is the workload figure G6 asks for.
gsh "echo \"cores \$(nproc), loadavg \$(cut -d' ' -f1-3 /proc/loadavg)\"
G6=\$(ps -eo pcpu,rss,nlwp,etimes,args --sort=-pcpu | grep -E \
 'slam_toolbox|planner_server|controller_server|behavior_server|bt_navigator|\
lifecycle_manager|robot_state_pub|liveliness_watchdog|clock_lag_probe|send_goal|\
yolo_node|explore_lite' \
 | grep -v grep)
echo \"\$G6\" | cut -c1-120
echo \"\$G6\" | awk -v c=\$(nproc) '{t+=\$1} END \
 {printf \"workload total %.0f%% of %d%% available, %d processes\\n\", t, 100*c, NR}'" \
 2>/dev/null
gsh "grep -cE 'Loop closure|optimiz' \$HOME/split-stack-$STAMP.log" 2>/dev/null

RTF=$(dex "gz topic -e -t /world/$WORLD_NAME/stats -n 5 2>/dev/null" \
      | awk '/real_time_factor/ {print $2}' | tail -1)
say "real_time_factor during the run = ${RTF:-<none>}"
LAT_RC=0
if [ -n "$WATCHDOG" ]; then
  say "liveliness gaps recorded in ~/liveliness-$STAMP.csv on the guest"
  # SIGTERM it HERE rather than leaving it to teardown: the distribution is printed
  # by its exit path, and teardown runs after the verdict line. SIGTERM and not
  # SIGKILL for the same reason -- the handler is what triggers the report.
  gsh "pkill -TERM -f '[l]iveliness_watchdog'" >/dev/null 2>&1
  sleep 8
  # Say which arithmetic actually produced the budget. RTF_LIMIT=0 means the world
  # was written unthrottled, and the awk guard leaves the budget unscaled -- printing
  # "scaled by RTF 0" then asserts a division that did not happen. The measured
  # factor is $RTF, read from gz stats above, and it is the one to quote.
  if [ "$RTF_LIMIT" = "0" ]; then
    say "topic inter-arrival at the consumer (budget ${TOPIC_BUDGET_MS} ms wall on max"
    say "      = TSP ${TSP_BUDGET_MS} ms, unscaled: no RTF_LIMIT was set. Measured"
    say "      real_time_factor was ${RTF:-<none>}; report it with the number):"
  else
    say "topic inter-arrival at the consumer (budget ${TOPIC_BUDGET_MS} ms wall on max"
    say "      = TSP ${TSP_BUDGET_MS} ms / RTF_LIMIT ${RTF_LIMIT}, measured RTF"
    say "      ${RTF:-<none>}; report both):"
  fi
  gsh "sed -n '/inter-arrival at the consumer/,\$p' \$HOME/live-$STAMP.log" 2>/dev/null
  # OVER on any watched topic is a budget breach. Read from the report rather than
  # from the node's exit status: it was backgrounded with nohup and its status is
  # not available here.
  # The report has to EXIST before its absence can be read as a pass. Without this
  # a watchdog that died at startup, or a dump() that outran the sleep above on a
  # starved guest, left both greps below false and LAT_RC at 0 -- and the verdict
  # then asserted a budget nothing had measured.
  if ! gsh "grep -q 'inter-arrival at the consumer' \$HOME/live-$STAMP.log" >/dev/null 2>&1; then
    LAT_RC=1
    say "      FAIL: liveliness_watchdog.py produced no interval report, so the topic"
    say "      budget was never evaluated. Guest log: ~/live-$STAMP.log"
    gsh "tail -15 \$HOME/live-$STAMP.log" 2>/dev/null
  fi
  if gsh "grep -q 'OVER' \$HOME/live-$STAMP.log" >/dev/null 2>&1; then
    LAT_RC=1
    say "      FAIL: at least one topic exceeded ${TOPIC_BUDGET_MS} ms between messages"
  fi
  # A watchdog that saw nothing is not a pass. It is how a blind instrument looks.
  # Only a GATED topic delivering nothing is a failure. /cmd_vel legitimately has
  # no messages if no goal ever ran, and the report labels the difference.
  if gsh "grep -q 'gated topic, this is a failure' \$HOME/live-$STAMP.log" >/dev/null 2>&1; then
    LAT_RC=1
    say "      FAIL: a budget-gated topic delivered nothing at all"
  fi
else
  say "liveliness not recorded (WATCHDOG= ); the CPU figure above is the workload,"
  say "      and no topic inter-arrival figure exists for this run"
fi
# The host side of the same measurement: /cmd_vel has no Header, so its interval is
# all there is, and cmd_vel_watchdog.py on port 42102 is the host consumer.
dex "pkill -TERM -f '[c]md_vel_watchdog'" >/dev/null 2>&1
sleep 4
dex "sed -n '/cmd_vel inter-arrival/,\$p' /work/artifacts/watchdog-$STAMP.log" 2>/dev/null || true

YOLO_RC=0
if [ "$YOLO" = 1 ]; then
  # G-Y5 is a co-tenancy gate, so what it asserts is that the detector kept
  # inferring while Nav2 drove -- not how fast. One 320x320 pass is 11.4 s here
  # and the camera runs at YOLO_RATE, so a tour of a few minutes yields a handful
  # of detections. Zero means the detector was starved or never saw a frame, and
  # that is the finding.
  DET=$(gsh "grep -c '\[detect\]' \$HOME/yolo-node-$STAMP.log 2>/dev/null" | tr -d '[:space:]')
  DET=${DET:-0}
  say "G-Y5: $DET detection(s) during the run -- ~/yolo-node-$STAMP.log on the guest"
  if [ "$DET" -eq 0 ]; then
    say "      FAIL (G-Y5): the detector produced nothing. Last lines:"
    gsh "tail -15 \$HOME/yolo-node-$STAMP.log" 2>/dev/null
    YOLO_RC=1
  fi
fi

# G5-E is a distinct label because the two drive modes produce non-comparable
# evidence: one reaches a fixed waypoint list, the other chooses its own goals and
# is judged on coverage.
GATE=G5
[ "$EXPLORE" = 1 ] && GATE=G5-E
# Everything from here is a verdict, so HOLD may now take effect.
VERDICT_PRINTED=1
if [ "$NAV_RC" -eq 0 ] && [ "$MAP_RC" -eq 0 ] && [ "$YOLO_RC" -eq 0 ] && [ "$LAT_RC" -eq 0 ]; then
  if [ "$EXPLORE" = 1 ]; then
    say "RESULT: PASS ($GATE, $ROBOT) -- explore_lite chose and drove $SUBSTANTIVE frontier"
    say "        goal(s) of its own worth moving for, of $GOALS total (complete=$EXPLORE_DONE),"
    say "        and the SLAM map built"
    say "        from that drive meets the precision/recall/coverage floors"
    [ "$YOLO" = 1 ] && say "        PASS (G-Y5) -- and the detector kept inferring beside them"
    exit 0
  fi
  say "RESULT: PASS ($GATE, $ROBOT) -- robot drove to every waypoint in Gazebo and the"
  say "        SLAM map built from that drive meets the precision/recall/coverage floors"
  [ "$YOLO" = 1 ] && \
    say "        PASS (G-Y5) -- and the detector kept inferring beside them"
  exit 0
fi
say "RESULT: FAIL ($GATE, $ROBOT)  nav_rc=$NAV_RC map_rc=$MAP_RC yolo_rc=$YOLO_RC lat_rc=$LAT_RC"
[ "$EXPLORE" = 1 ] && [ "$NAV_RC" -ne 0 ] && \
  say "        nav_rc: $SUBSTANTIVE of $GOALS goal(s) were further than 0.5 m away, below EXPLORE_MIN_GOALS=$EXPLORE_MIN_GOALS"
exit 1
