#!/bin/bash
# Make the simulation visible -- for a demonstration, not for a measurement.
#
# The Gazebo server and the Gazebo GUI are separate processes that find each other
# over gz-transport, so the viewer can be attached to a run that is already going
# without restarting or reconfiguring it. This script is only the viewer half: it
# provides a display, a VNC exporter and, optionally, a recording.
#
#   # terminal 1 -- the run, built so it can be watched
#   GUI=1 ROBOT=tb3 scripts/run_container_loop.sh
#   # terminal 2 -- watch it
#   scripts/run_demo.sh
#
#   scripts/run_demo.sh --standalone   # no run in progress: world + robot, nothing driving
#   scripts/run_demo.sh --shot         # one PNG for the report, then exit
#   scripts/run_demo.sh --record       # live VNC *and* an mp4
#
# Then from your own machine:
#   ssh -N -L 5900:localhost:5900 <user>@<this host>
#   and point any VNC viewer at localhost:5900
#
# Why the tunnel: x11vnc binds 127.0.0.1 only. The container runs with
# --network host, so an unbound VNC port would be open on the lab network with no
# password.
#
# What this costs, measured on this host with tb3: the GUI took 20 cores of
# llvmpipe (2007 %CPU) while the server needed 20.5 %CPU. On a 192-core host the
# server was unaffected -- real_time_factor stayed at 1.00 -- but that will not
# hold on a smaller machine, and SceneBroadcaster is extra work for the server in
# any case. Numbers in RUNLOG.md come from headless runs only.
#
# No `set -u`: see run_rtf.sh.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
mkdir -p "$ROOT/artifacts"
CONTAINER=${CONTAINER:-vela-gz}
VIEWER=${VIEWER:-vela-gz-view}
IMAGE=${IMAGE:-vela-gzsim:demo}
# The viewer needs the Fuel cache the SERVER has. ../yolo/tools/make_people.py
# writes artifacts/people/*.sdf with ABSOLUTE mesh URIs --
# file:///home/vela/.gz/fuel/fuel.gazebosim.org/openrobotics/models/standing person/3/meshes/standing.dae
# -- and vela-gz carries that cache in its own writable layer, not on a bind
# mount: `docker inspect vela-gz` lists /work and nothing else. This image has no
# ~/.gz at all, so the GUI resolved the robot (GZ_SIM_RESOURCE_PATH carries
# nav2_minimal_tb3_sim/models) and drew no people, with nothing on screen or in
# the log to say why. Mounting a host copy read-only makes the same URI resolve on
# this side, so neither the SDF nor make_people.py has to change.
#
# Seed it once from the server container, which already has it:
#   mkdir -p ~/ms_kim/gz-fuel-cache
#   docker cp 'vela-gz:/home/vela/.gz/fuel/.' ~/ms_kim/gz-fuel-cache/
# It is deliberately outside the repository and outside artifacts/: artifacts/ is
# rebuilt every run, and 157 MB of collada does not belong in git.
FUEL_CACHE=${FUEL_CACHE:-$HOME/ms_kim/gz-fuel-cache}
ROBOT=${ROBOT:-tb3}
DISPLAY_NUM=${DISPLAY_NUM:-99}
# Default set once RVIZ is known, below: two tiled panes need the width.
GEOMETRY=${GEOMETRY:-}
VNC_PORT=${VNC_PORT:-5900}
# The port the operator's own machine binds for the ssh tunnel. Only the client
# side, so this server never sees it -- but it is printed, and the printed line
# has to work first time. Defaults high rather than to 5900: on Windows,
# Hyper-V / WSL2 / Docker Desktop reserve dynamic port ranges, and 5900 landed
# inside one, failing with "bind [127.0.0.1]:5900: Permission denied" (measured
# 2026-08-17). 15900 bound without complaint on the same machine.
VIEW_PORT=${VIEW_PORT:-15900}
SHOT_DELAY=${SHOT_DELAY:-60}
# FOLLOW=0 leaves the GUI on its default camera. FOLLOW_OFFSET_OVERRIDE takes
# "x,y,z" in metres and beats the per-robot value robot.env carries.
FOLLOW=${FOLLOW:-1}
# RVIZ=1 adds RViz beside the Gazebo GUI on the same display. The two show
# different things and a demonstration of this rig needs both: Gazebo shows
# ground truth -- the robot moving through the world -- while RViz shows what the
# GUEST believes, which is the map slam_toolbox built, the path Nav2 planned and
# what the detector found. All three come back over the TCP link.
#
# It also starts detection_overlay.py, because RViz has no display for
# vision_msgs/Detection2DArray: without it a working detector and a dead one look
# identical on screen.
RVIZ=${RVIZ:-0}
# OVERLAY=1 starts detection_overlay.py, which draws the guest's
# Detection2DArray onto the host's camera frames. Off by default: it is only
# useful with a detector running in the guest.
#
# It gets its OWN WINDOW on the same display (`--show`), not RViz's Image display.
# RViz gives its only dock the full height of the pane, so an 864 px pane spent
# ~320x1000 of itself on a 320x240 image and the map lost 300 px to empty grey
# (measured 2026-08-19, artifacts/rviz-yolo-smoke.png). A window is tiled by the
# same xdotool call as the other two, costs the map nothing, and lives inside the
# overlay process -- so it needs no second Fast DDS participant and no second
# listening port. Which means the RViz layout is demo_nav.rviz either way.
OVERLAY=${OVERLAY:-0}
# 2x the 320x240 camera. The annotation is drawn at native size and magnified
# with INTER_NEAREST, so the window is tiled at exactly scale x native and Qt
# resamples nothing -- 2 px box edges and 0.5-scale labels stay crisp, which they
# do not survive an arbitrary window resize.
OVERLAY_SCALE=${OVERLAY_SCALE:-2}
OVERLAY_GEOM=${OVERLAY_GEOM:-640x480}
# The window title, which is what xdotool matches on. It must share no substring
# with 'Gazebo Sim' or 'RViz': tile() searches by substring, and a title that
# matched two windows would move whichever xdotool returned first.
OVERLAY_TITLE=${OVERLAY_TITLE:-YOLO detections}
# Bottom-left of the Gazebo half. The arena is centred in that pane and the robot
# is followed, so the corner is the part of it carrying no information.
OVERLAY_POS=${OVERLAY_POS:-16,-16}
RVIZ_CONFIG=${RVIZ_CONFIG:-/work/config/demo_nav.rviz}
# The stock Gazebo GUI docks Component inspector and Entity tree on the right and
# they cost ~270 px of the pane to show a solver name and five entities. See the
# file. GUI_CONFIG= (empty) restores the stock layout.
GUI_CONFIG=${GUI_CONFIG-/work/config/demo_gui.config}
# One pane fits 1280x800. Two do not: measured off artifacts/rviz-smoke.png,
# RViz's left dock column is about 450 px, so a 640-wide half leaves ~190 px of
# map -- unreadable in a recording. 1920 wide gives each pane ~950.
if [ -z "$GEOMETRY" ]; then
  if [ "$RVIZ" = 1 ]; then GEOMETRY=1920x1080; else GEOMETRY=1280x800; fi
fi
DOMAIN=${DOMAIN:-42}
STAMP=$(date +%Y%m%d-%H%M%S)

MODE=live
STANDALONE=no
RECORD=no
# --duration N: record for N seconds, then stop and exit without waiting for a
# Ctrl-C. This is the mode that produces a file for a slide deck, so it has to
# finish on its own and it has to finalise the container properly.
DURATION=0
while [ $# -gt 0 ]; do
  case "$1" in
    --standalone) STANDALONE=yes ;;
    --shot)       MODE=shot ;;
    --record)     RECORD=yes ;;
    --duration)   RECORD=yes; DURATION=$2; shift ;;
    --duration=*) RECORD=yes; DURATION=${1#*=} ;;
    -h|--help)    sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
  shift
done

say()  { echo "[$(date +%H:%M:%S)] $*"; }
dex()  { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
dexd() { docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
vex()  { docker exec "$VIEWER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
vexd() { docker exec -d "$VIEWER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }

RECORD_CLOSED=no
MP4=""
finish_recording() {
  [ "$RECORD" = yes ] || return 0
  # $MP4 is set only once ffmpeg is actually started; any earlier exit
  # (--shot, missing image, SCENE_BROADCASTER=0) has nothing to close.
  [ -n "$MP4" ] || return 0
  [ "$RECORD_CLOSED" = no ] || return 0
  RECORD_CLOSED=yes
  say "closing the recording"
  docker exec "$VIEWER" bash -lc "pkill -INT -f '[f]fmpeg'" >/dev/null 2>&1
  # -movflags +faststart rewrites the whole file on shutdown; give a long
  # recording time to finish that pass or the kill below truncates it.
  for _ in $(seq 1 45); do
    sleep 1
    docker exec "$VIEWER" bash -lc "pgrep -f '[f]fmpeg' >/dev/null" 2>/dev/null || break
  done
  if [ -f "$MP4" ]; then
    say "wrote $MP4 ($(du -h "$MP4" | cut -f1))"
  else
    say "WARNING: $MP4 missing -- see $ROOT/artifacts/demo-ffmpeg-$STAMP.log"
  fi
}

cleanup() {
  # Before anything else: a Ctrl-C during a recording must still leave a playable
  # file, and `docker rm -f` below would cut ffmpeg off mid-trailer.
  finish_recording
  say "tearing down the viewer"
  for p in '[g]z sim -g' '[r]viz2' '[d]etection_overlay' '[x]11vnc' '[f]fmpeg' '[X]vfb'; do
    docker exec "$VIEWER" bash -lc "pkill -f '$p'" >/dev/null 2>&1
  done
  docker rm -f "$VIEWER" >/dev/null 2>&1
  if [ "$STANDALONE" = yes ]; then
    docker exec "$CONTAINER" bash -lc "pkill -f '[g]z sim -s'" >/dev/null 2>&1
    say "restoring the headless world so the next run is measurable"
    dex "python3 /work/tools/make_world.py --robot $ROBOT --out-dir /work/artifacts" 2>&1 | grep -E '^world  : args' || true
  fi
  # Released here rather than in a second `trap ... EXIT`: a second trap on the
  # same signal replaces the first, which would have silently disabled all of the
  # teardown above.
  rmdir "$LOCK" 2>/dev/null
}

# Two viewers cannot coexist: they share the container name, so the first one's
# cleanup does `docker rm -f` on the second one's container and the second dies
# with "No such container: vela-gz-view" seconds after starting. mkdir is atomic,
# so it is the lock.
LOCK=${LOCK:-/tmp/run_demo.$VIEWER.lock}
mkdir "$LOCK" 2>/dev/null || {
  say "FAIL: another run_demo.sh holds $LOCK."
  say "      Wait for it, or if nothing is running: rmdir $LOCK"
  exit 1; }
# Armed only now that the lock is ours: an exit on the contention path above
# must not tear down a viewer another instance owns.
trap cleanup EXIT

docker inspect "$CONTAINER" >/dev/null 2>&1 || { say "FAIL: no container '$CONTAINER'"; exit 1; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  say "FAIL: image '$IMAGE' missing. Build the viewer layer first:"
  say "      docker build -f \"$ROOT/Dockerfile.demo\" -t vela-gzsim:demo \"$ROOT\""
  exit 1; }

if [ "$STANDALONE" = yes ]; then
  say "standalone: building the world with SceneBroadcaster and spawning $ROBOT"
  dex "python3 /work/tools/make_world.py --robot $ROBOT --out-dir /work/artifacts --gui" || exit 1
fi

docker cp "$CONTAINER":/work/artifacts/robot.env /tmp/robot.env.$$ >/dev/null 2>&1 || {
  say "FAIL: no /work/artifacts/robot.env -- run make_world.py, or use --standalone"; exit 1; }
. /tmp/robot.env.$$ && rm -f /tmp/robot.env.$$

# The one failure that looks like a broken GUI and is not. Both shipped worlds
# default headless=true, which removes SceneBroadcaster, and without it the GUI
# connects fine and then draws an empty scene for as long as you care to wait.
if [ "${SCENE_BROADCASTER:-0}" != "1" ]; then
  say "FAIL: the world was built without SceneBroadcaster, so there is"
  say "      nothing for a GUI to draw. Rebuild the world for viewing:"
  say "        GUI=1 ROBOT=$ROBOT scripts/run_container_loop.sh     (or run_split.sh)"
  say "        scripts/run_demo.sh --standalone                     (viewer only)"
  exit 1
fi

# Two things worth saying before a take, both of which produce a plausible-looking
# recording that is missing the thing it was filmed for.
if [ "$RVIZ" = 1 ] && [ "$OVERLAY" != 1 ]; then
  say "NOTE: OVERLAY is off, so detection_overlay.py will not run and nothing in"
  say "      this recording will show the detector -- even if it is inferring in"
  say "      the guest. Pass OVERLAY=1 for a run with YOLO=1."
fi
if [ "${ROBOT_KIND:-}" = tb4 ] || [ "$ROBOT" = tb4 ]; then
  say "NOTE: tb4. Every recording on record is tb3, and the depot world is about"
  say "      three times the span the saved RViz view is framed for -- expect to"
  say "      set RVIZ_CONFIG or the Orbit distance by hand over VNC."
fi

if [ "$STANDALONE" = yes ]; then
  say "starting the server, headless rendering as always"
  dexd "DISPLAY= gz sim -s -r -v2 --headless-rendering $WORLD_FILE \
        > /work/artifacts/demo-server-$STAMP.log 2>&1"
  for i in $(seq 1 24); do
    sleep 5
    dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" && break
  done
  if [ "$SPAWN_MODE" = topic ]; then
    dexd "ros2 run robot_state_publisher robot_state_publisher --ros-args \
          -p use_sim_time:=true -p robot_description:=\"\$(cat $ROBOT_FILE)\" \
          > /work/artifacts/demo-rsp-$STAMP.log 2>&1"
    sleep 8
    dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name $ROBOT_KIND \
         -topic robot_description -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z" || exit 1
  else
    dex "ros2 run ros_gz_sim create -world $WORLD_NAME -name $ROBOT_KIND \
         -file $ROBOT_FILE -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z" || exit 1
  fi
  sleep 8
fi

say "checking a server is publishing on /world/$WORLD_NAME/stats"
dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/stats'" || {
  say "FAIL: no server. Start a run first, or pass --standalone"; exit 1; }
dex "gz topic -l 2>/dev/null | grep -q '/world/$WORLD_NAME/scene/info'" || {
  say "FAIL: the running server has no scene/info -- it was started from a"
  say "      headless world. Restart that run with GUI=1"; exit 1; }

# Refuse to start rather than film an empty arena. A viewer with no Fuel cache
# renders the robot and silently omits every people model, and nothing in the GUI
# log says so -- the first evidence is the finished recording. Re-shooting a
# 1500 s take costs far more than failing here. The MESH is checked and not just
# the directory, because a `docker cp` of the wrong source leaves the directory
# behind with nothing under it.
PERSON_MESH="$FUEL_CACHE/fuel.gazebosim.org/openrobotics/models/standing person/3/meshes/standing.dae"
if [ ! -d "$FUEL_CACHE" ]; then
  say "FAIL: no Fuel cache directory at $FUEL_CACHE"
  say "      The viewer needs it: artifacts/people/*.sdf carry absolute mesh URIs"
  say "      under file:///home/vela/.gz/fuel/, and $IMAGE ships no ~/.gz."
  say "      Seed it from $CONTAINER, which has the cache in its writable layer:"
  say "        mkdir -p \"$FUEL_CACHE\""
  say "        docker cp '$CONTAINER:/home/vela/.gz/fuel/.' \"$FUEL_CACHE\"/"
  say "      Or point FUEL_CACHE= at an existing copy."
  exit 1
fi
if [ ! -f "$PERSON_MESH" ]; then
  say "FAIL: the Fuel cache at $FUEL_CACHE has no standing person mesh"
  say "      expected: $PERSON_MESH"
  say "      Re-seed it:"
  say "        docker cp '$CONTAINER:/home/vela/.gz/fuel/.' \"$FUEL_CACHE\"/"
  exit 1
fi

say "starting the viewer container ($IMAGE, host netns so gz-transport reaches the server)"
say "      Fuel cache $FUEL_CACHE -> /home/vela/.gz/fuel (ro)"
docker rm -f "$VIEWER" >/dev/null 2>&1
docker run -d --name "$VIEWER" --network host --shm-size=512m \
  -v "$ROOT":/work -v "$FUEL_CACHE":/home/vela/.gz/fuel:ro -w /work "$IMAGE" >/dev/null || exit 1
sleep 2

say "Xvfb :$DISPLAY_NUM ($GEOMETRY)"
vexd "Xvfb :$DISPLAY_NUM -screen 0 ${GEOMETRY}x24 > /work/artifacts/demo-xvfb-$STAMP.log 2>&1"
sleep 4
vex "test -S /tmp/.X11-unix/X$DISPLAY_NUM" || { say "FAIL: Xvfb did not start"; exit 1; }

say "gz sim -g on that display"
# QT_QPA_PLATFORM=xcb is explicit: with no DISPLAY inherited, Qt would otherwise
# try to guess a platform and abort.
GUI_CONFIG_ARG=""
[ -n "$GUI_CONFIG" ] && GUI_CONFIG_ARG="--gui-config $GUI_CONFIG"
vexd "DISPLAY=:$DISPLAY_NUM QT_QPA_PLATFORM=xcb gz sim -g -v2 $GUI_CONFIG_ARG \
      > /work/artifacts/demo-gui-$STAMP.log 2>&1"

# The GUI compiles OGRE-Next's shaders on first render, which is tens of seconds
# under llvmpipe. Wait for the process, then give it time to actually draw.
for i in $(seq 1 24); do
  sleep 5
  vex "pgrep -f '[g]z sim -g' >/dev/null" || {
    say "FAIL: the GUI exited. Log:"; vex "tail -25 /work/artifacts/demo-gui-$STAMP.log"; exit 1; }
  [ $i -ge 4 ] && break
done
say "      GUI up (${SHOT_DELAY}s of shader compilation is normal here)"

if [ "$RVIZ" = 1 ]; then
  say "rviz2 on the same display ($RVIZ_CONFIG)"
  # One acceptor per viewer participant, and neither of them is the bridge's.
  # rviz2 used host_tcp_server.xml from the day RVIZ=1 was added, which declares
  # the bridge's own 42100: in a split run the bridge owns it, so RViz came up
  # with no TCP acceptor, the guest had nothing to dial, and /map, /plan and
  # map->odom could never arrive. demo.rviz's Fixed Frame is map, so that made the
  # whole pane blank, /scan included. 42103 is rviz2's, 42104 is the overlay's,
  # and run_split.sh under GUI=1 gives the guest a locator for each.
  RVIZ_ENV="export ROS_DOMAIN_ID=$DOMAIN; \
        export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_rviz_server.xml; \
        unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"
  OVER_ENV="export ROS_DOMAIN_ID=$DOMAIN; \
        export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_overlay_server.xml; \
        unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"

  # Probe BEFORE rviz2 starts, never after: a `ros2` CLI call on the same profile
  # binds the same 42103, so a check run alongside RViz takes the port away from
  # it. This costs ~24 s and it is the only thing that separates "RViz is up" from
  # "RViz can see anything", which are not the same fact -- the viewer runs in a
  # second container with its own /dev/shm, and with shared memory in the
  # transport list it received nothing at all (measured 2026-08-19).
  hzprobe() {  # hzprobe <topic> <seconds>
    vex "$RVIZ_ENV timeout $2 ros2 topic hz $1 2>/dev/null | grep -m1 'average rate'" \
        2>/dev/null | head -1
  }
  R=$(hzprobe /scan 12)
  [ -n "$R" ] && say "      /scan reaches the viewer (${R#*: } Hz)" \
              || say "      WARNING: no /scan in the viewer -- the lidar and TF panels will be empty"
  # /map is NOT probed the same way. It is the guest's, and the guest only starts
  # dialling 42103 once something owns that acceptor, which took 33 s to complete
  # on the first measured run -- a 12 s probe called it missing while it was on
  # its way. RViz itself is the authoritative witness and costs nothing: it logs
  # "Trying to create a map of size WxH" the first time a grid arrives. Checked
  # after it has had time to come up, below.

  vexd "$RVIZ_ENV DISPLAY=:$DISPLAY_NUM QT_QPA_PLATFORM=xcb \
        rviz2 -d $RVIZ_CONFIG > /work/artifacts/demo-rviz-$STAMP.log 2>&1"

  # The detector publishes only the small Detection2DArray; the drawing happens
  # here, on a 192-core host, rather than in a four-vCPU emulated guest already
  # carrying three workloads.
  OVERLAY_PY=/work/artifacts/yolo_scene/detection_overlay.py
  if [ "$OVERLAY" = 1 ] && [ -f "$ROOT/../yolo/tools/detection_overlay.py" ]; then
    mkdir -p "$ROOT/artifacts/yolo_scene"
    cp "$ROOT/../yolo/tools/detection_overlay.py" "$ROOT/artifacts/yolo_scene/"
    vexd "$OVER_ENV DISPLAY=:$DISPLAY_NUM QT_QPA_PLATFORM=xcb \
          python3 $OVERLAY_PY --show --window '$OVERLAY_TITLE' \
          --scale $OVERLAY_SCALE > /work/artifacts/demo-overlay-$STAMP.log 2>&1"
    sleep 5
    vex "grep -q 'Address already in use' /work/artifacts/demo-overlay-$STAMP.log" \
      && say "      WARNING: the overlay lost the 42104 bind -- no detections will reach it" \
      || say "      detection_overlay.py -> /detection_image + a window"
    # The window is the only thing an audience sees the detector through, and it
    # fails silently: cv2's Qt window needs a DISPLAY and an xcb plugin, and
    # without either the node keeps publishing while the frame shows nothing.
    # It exists from startup holding a placeholder, so it is checkable now.
    vex "DISPLAY=:$DISPLAY_NUM xdotool search --onlyvisible --name '$OVERLAY_TITLE'" \
        >/dev/null 2>&1 \
      && say "      the detection window is on the display" \
      || { say "      WARNING: no '$OVERLAY_TITLE' window -- the recording will not show"
           say "               the detector. Log:"
           vex "tail -12 /work/artifacts/demo-overlay-$STAMP.log"; }
  fi

  sleep 20
  vex "pgrep -f '[r]viz2' >/dev/null" \
    && say "      RViz up" \
    || { say "      WARNING: rviz2 exited. Log:"
         vex "tail -20 /work/artifacts/demo-rviz-$STAMP.log"; }
  # Did the guest's map actually arrive? Poll RViz's own log rather than the DDS
  # graph, so nothing competes for 42103.
  MAPLINE=""
  for _ in $(seq 1 12); do
    MAPLINE=$(vex "grep -m1 -o 'Trying to create a map of size [0-9]* x [0-9]*' \
                   /work/artifacts/demo-rviz-$STAMP.log" 2>/dev/null | head -1)
    [ -n "$MAPLINE" ] && break
    sleep 5
  done
  [ -n "$MAPLINE" ] \
    && say "      /map arrived from the guest (${MAPLINE#Trying to create a map of size })" \
    || say "      WARNING: no /map in 60s -- a split run needs GUI=1 so the guest gets a 42103 locator"
fi

# Lay the windows out. Xvfb has no window manager -- nothing in the demo image
# provides one -- so every window keeps its own default size at its own default
# position, and the last one mapped sits on top of the others. Untiled, a single
# Gazebo GUI opens at about 1000x845 and a 1920x1080 capture is two-thirds black
# border; untiled with RVIZ=1, RViz maps last at 1500x900 and covers the
# simulation completely, so the recording shows the map and nothing driving.
# This has to run after the RViz readiness check above, because a window that
# does not exist yet cannot be placed.
if vex "command -v xdotool >/dev/null"; then
  W=${GEOMETRY%x*}; H=${GEOMETRY#*x}
  tile() {  # tile <title-substring> <x> <y> <w> <h>
    # `timeout` is not optional: `search --sync` waits forever for a title that
    # never appears, which would hang this script before ffmpeg ever starts.
    vex "DISPLAY=:$DISPLAY_NUM timeout 20 xdotool search --sync --onlyvisible \
         --name '$1' windowmove $2 $3 windowsize $4 $5" >/dev/null 2>&1 \
      && say "      '$1' -> ${4}x${5}+${2}+${3}" \
      || say "      WARNING: no window matched '$1' -- the frame will be wrong"
  }
  if [ "$RVIZ" = 1 ]; then
    # Gazebo takes the larger half: it is the half with the robot in it. Even
    # widths only -- yuv420p refuses odd dimensions.
    GW=$(( (W * 55 / 100) / 2 * 2 ))
    tile 'Gazebo Sim' 0 0 "$GW" "$H"
    tile 'RViz'  "$GW" 0 $((W - GW)) "$H"
  else
    tile 'Gazebo Sim' 0 0 "$W" "$H"
  fi
  # The detection window last, and raised: Xvfb has no window manager, so
  # stacking is map order and a window placed over the Gazebo pane before Gazebo
  # was placed would end up under it. Negative positions count back from the far
  # edge, so OVERLAY_POS=16,-16 is 16 px in from the bottom-left corner.
  if [ "$OVERLAY" = 1 ]; then
    OW=${OVERLAY_GEOM%x*}; OH=${OVERLAY_GEOM#*x}
    OX=${OVERLAY_POS%,*}; OY=${OVERLAY_POS#*,}
    [ "${OX#-}" != "$OX" ] && OX=$(( W - OW - ${OX#-} ))
    [ "${OY#-}" != "$OY" ] && OY=$(( H - OH - ${OY#-} ))
    tile "$OVERLAY_TITLE" "$OX" "$OY" "$OW" "$OH"
    vex "DISPLAY=:$DISPLAY_NUM xdotool search --onlyvisible --name '$OVERLAY_TITLE' \
         windowraise" >/dev/null 2>&1
  fi
  # Last, so nothing leaves a tooltip mid-frame: a cursor resting over the entity
  # tree raises one that then appears in every recorded frame.
  vex "DISPLAY=:$DISPLAY_NUM xdotool mousemove $((W - 2)) $((H - 2))" >/dev/null 2>&1
  sleep 5
fi

# Point the camera at the robot. Without this the GUI opens on its default pose,
# where a 0.28 m TurtleBot3 is about 14 px wide somewhere off-centre -- which
# reads as "the robot did not render" and cost an hour of chasing mesh loading
# that was working the whole time. /gui/follow also keeps it framed once it
# starts driving, which is the thing an audience is here to see.
if [ "$FOLLOW" != 0 ]; then
  OFF=${FOLLOW_OFFSET_OVERRIDE:-$FOLLOW_OFFSET}
  OX=${OFF%%,*}; OZ=${OFF##*,}; OY=${OFF#*,}; OY=${OY%%,*}
  say "following $ROBOT_KIND, camera offset ($OX, $OY, $OZ) m"
  vex "gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
       --reptype gz.msgs.Boolean --timeout 5000 --req 'data: \"$ROBOT_KIND\"'" \
       >/dev/null 2>&1
  vex "gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
       --reptype gz.msgs.Boolean --timeout 5000 \
       --req 'x: $OX, y: $OY, z: $OZ'" >/dev/null 2>&1
  sleep 8
fi

if [ "$MODE" = shot ]; then
  sleep "$SHOT_DELAY"
  OUT=/work/artifacts/demo-${ROBOT_KIND:-$ROBOT}-$STAMP.png
  vex "mkdir -p /work/artifacts && ffmpeg -loglevel error -y -f x11grab \
       -video_size $GEOMETRY -i :$DISPLAY_NUM -frames:v 1 $OUT" || {
    say "FAIL: screen grab"; exit 1; }
  say "wrote $ROOT/artifacts/demo-${ROBOT_KIND:-$ROBOT}-$STAMP.png"
  # A blank window is one or two colours; a rendered scene is thousands. Say which
  # it was rather than leaving a black PNG to be discovered later.
  vex "python3 -c \"
import subprocess
raw = subprocess.run(['ffmpeg','-loglevel','error','-i','$OUT','-f','rawvideo',
                      '-pix_fmt','rgb24','-'], capture_output=True).stdout
px = {raw[i:i+3] for i in range(0, len(raw), 3)}
print('      distinct colours:', len(px),
      '-- rendered' if len(px) > 100 else '-- BLANK, the scene did not draw')\""
  exit 0
fi

REC_NOTE=""
MP4="$ROOT/artifacts/demo-${ROBOT_KIND:-$ROBOT}-$STAMP.mp4"
if [ "$RECORD" = yes ]; then
  REC_NOTE="recording: $MP4"
  say "recording to $MP4"
  # Encoder settings chosen for a slide deck, not for speed:
  #   libx264 + yuv420p  what PowerPoint and Keynote will actually play
  #   -crf 23 veryfast   ultrafast triples the file for no visible gain, and
  #                      there are cores to spare on this host
  #   -framerate 15      the sim runs at 5 Hz lidar; 10 looked stuttery on pans
  #   +faststart         moves the index to the front, so it starts instantly
  #                      when embedded rather than buffering the whole file
  #   -thread_queue_size  the x11grab default is 8 packets, and each packet is a
  #                       whole 1080p frame; a briefly slow encoder drops frames
  #                       with only a warning, which -loglevel error hid
  #   -threads 8          x264 would otherwise open ~128 frame threads on this
  #                       192-core host for a 15 fps stream
  # $MP4 is a host path and the container sees $ROOT as /work: same file, one
  # name. It used to be spelled from $ROBOT here and from $ROBOT_KIND in the
  # message, so a successful tb4 recording was announced as missing.
  vexd "mkdir -p /work/artifacts && ffmpeg -loglevel warning -y -f x11grab -framerate 15 \
        -thread_queue_size 64 \
        -video_size $GEOMETRY -i :$DISPLAY_NUM -c:v libx264 -threads 8 -preset veryfast \
        -crf 23 -pix_fmt yuv420p -movflags +faststart \
        /work/artifacts/$(basename "$MP4") > /work/artifacts/demo-ffmpeg-$STAMP.log 2>&1"
fi

# Stop ffmpeg with SIGINT and give it time to write the trailer. SIGKILL leaves an
# mp4 with no moov atom, which most players refuse to open -- and the file looks
# fine in `ls` until someone tries to play it during a talk.

say "x11vnc on 127.0.0.1:$VNC_PORT"
vexd "x11vnc -display :$DISPLAY_NUM -localhost -nopw -forever -shared \
      -rfbport $VNC_PORT > /work/artifacts/demo-vnc-$STAMP.log 2>&1"
sleep 3
ss -tlnp 2>/dev/null | grep -qE "[:.]$VNC_PORT([^0-9]|$)" || {
  say "WARN: nothing listening on $VNC_PORT -- check $ROOT/artifacts/demo-vnc-$STAMP.log"; }

# The address to tell the operator to ssh to, in order of trustworthiness.
#
# 1. $SSH_CONNECTION field 3 is the server address this very session's client
#    connected to, so it is reachable from their machine by definition.
# 2. Otherwise the source address of the default route.
#
# Not `hostname -I | awk '{print $1}'`: it lists every interface in kernel order,
# and on this host the first two are BMC/USB NICs on 169.254.0.0/16. That printed
# a link-local address the operator's PC could not route to. Not `hostname`
# either -- cslab192 is only in this host's own /etc/hosts, as 127.0.1.1.
HOST_IP=$(set -- $SSH_CONNECTION; echo "$3")
[ -n "$HOST_IP" ] || HOST_IP=$(ip route get 1.1.1.1 2>/dev/null |
                               sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -1)
case "$HOST_IP" in 169.254.*|127.*|'') HOST_IP="<this host's LAN address>";; esac
cat <<EOF

  ---------------------------------------------------------------
  Run these two ON THE MACHINE WITH THE SCREEN -- your laptop or desktop,
  not this server. Run here, the tunnel only points $VIEW_PORT at itself.

    1. ssh -N -L $VIEW_PORT:localhost:$VNC_PORT $(id -un)@$HOST_IP
    2. point any VNC viewer at   localhost:$VIEW_PORT

  Step 1 asks for your password, then prints nothing and does not return.
  That is the tunnel working: -N means "no remote command", so an idle
  prompt is success. Leave that window open and do step 2 in another.

  If step 1 says "bind [127.0.0.1]:$VIEW_PORT: Permission denied", your own
  machine is refusing that local port -- on Windows, Hyper-V, WSL2 and
  Docker Desktop reserve ranges of them:
     netsh interface ipv4 show excludedportrange protocol=tcp
  The local half is arbitrary, so try 25900 and view localhost:25900.
  Only the remote half ($VNC_PORT) is fixed.

  Robot: $ROBOT_KIND   world: $WORLD_NAME   spawn: $SPAWN_X,$SPAWN_Y
  $REC_NOTE

  This is a demonstration, not a measurement. Leave the numbers in
  RUNLOG.md to the headless runs.

  Ctrl-C here when the demo is over.
  ---------------------------------------------------------------

EOF

if [ "$DURATION" -gt 0 ] 2>/dev/null; then
  say "recording for ${DURATION}s, then stopping on its own"
  # Deadline on the wall clock, not a count of sleeps: each iteration also pays for
  # a `docker exec`, so counting to 130 took about 180 s and the tail of the file
  # was dead scene after the run it was filming had already torn down.
  END=$(( $(date +%s) + DURATION ))
  while [ "$(date +%s)" -lt "$END" ]; do
    sleep 5
    vex "pgrep -f '[g]z sim -g' >/dev/null" || {
      say "the GUI exited early, $(( END - $(date +%s) ))s short"; break; }
  done
  finish_recording
  exit 0
fi

# Hold the viewer open. Report if the GUI dies so a dead demo is not mistaken for
# a frozen VNC connection.
while vex "pgrep -f '[g]z sim -g' >/dev/null"; do sleep 10; done
say "the GUI exited"
finish_recording
