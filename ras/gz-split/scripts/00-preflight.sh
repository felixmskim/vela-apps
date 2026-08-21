#!/bin/bash
# Preflight for the split Gazebo/SLAM topology. INSTALLS NOTHING, BUILDS NOTHING.
#
# Run this first. It answers one question -- is it worth starting -- and it
# records the environment so the run can be reproduced later. Everything it
# prints is meant to be pasted into RUNLOG.md.
#
# Same contract as ras/nav2/00-preflight.sh: read-only, and a GO/NO-GO verdict
# at the end.
#
# No `set -u`: this script sources nothing, but the convention holds across the
# directory because /opt/ros/jazzy/setup.bash dereferences unset variables.
set -o pipefail

GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
# Read the pin out of the Dockerfile rather than repeating it: checking that
# `ros:jazzy-ros-base` resolves proves nothing about the digest the build asks
# for. Falls back to the tag if the ARG is ever removed.
BASE_IMAGE=${BASE_IMAGE:-$(sed -n 's/^ARG BASE_IMAGE=//p' "$ROOT/Dockerfile" 2>/dev/null | head -1)}
BASE_IMAGE=${BASE_IMAGE:-ros:jazzy-ros-base}
OUT_DIR=${OUT_DIR:-$ROOT/artifacts}
mkdir -p "$OUT_DIR"

WARN=0
STOP=0

hdr() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*"; WARN=$((WARN + 1)); }
bad()  { printf '  STOP  %s\n' "$*"; STOP=$((STOP + 1)); }
info() { printf '        %s\n' "$*"; }

gsh() { ssh -p "$PORT" -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
            -o BatchMode=yes "$GUEST" "bash -l -s" <<< "$*"; }

printf 'gz-split preflight   %s\n' "$(date -Is)"

# ---------------------------------------------------------------- P0 host facts
hdr "P0  host environment"
info "$(. /etc/os-release; echo "$PRETTY_NAME") $(uname -m)  kernel $(uname -r)"
info "cpus visible to this shell: $(nproc)   total: $(getconf _NPROCESSORS_CONF)"
info "affinity: $(taskset -cp $$ 2>/dev/null | sed 's/.*list: //')"
info "ram: $(free -g | awk '/^Mem:/ {print $2" GiB total, "$7" GiB available"}')"
info "swap: $(free -g | awk '/^Swap:/ {print $2" GiB"}')"
DISK_FREE=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
info "root free: ${DISK_FREE} GiB"
[ "${DISK_FREE:-0}" -ge 10 ] && ok "disk headroom for the image (need ~4 GiB)" \
                             || bad "less than 10 GiB free on /"

# GPU is expected to be absent. Say so explicitly so the log shows it was checked
# rather than overlooked -- llvmpipe does not need a render node.
if ls /dev/dri/renderD* >/dev/null 2>&1; then
  info "render node present: $(ls /dev/dri/renderD* | tr '\n' ' ')"
else
  ok "no /dev/dri/renderD* -- expected; rendering will be Mesa llvmpipe"
fi
info "vga: $(lspci 2>/dev/null | grep -i 'vga\|display' | head -2 | sed 's/^/        /' | tr '\n' ' ')"

# --------------------------------------------------------------- P1 docker
hdr "P1  docker without sudo"
if ! command -v docker >/dev/null 2>&1; then
  bad "docker not installed"
elif ! docker info >/dev/null 2>&1; then
  bad "docker present but 'docker info' failed -- are you in the docker group? (groups: $(groups))"
else
  ok "docker $(docker version --format '{{.Server.Version}}' 2>/dev/null) reachable as $(id -un)"
  info "storage driver: $(docker info --format '{{.Driver}}' 2>/dev/null)"
  info "docker root free: $(df -BG --output=avail "$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)" 2>/dev/null | tail -1 | tr -d ' ')"
fi

hdr "P2  base image reachable (metadata only, no pull)"
if docker manifest inspect "$BASE_IMAGE" >/dev/null 2>&1; then
  ok "$BASE_IMAGE resolvable from the registry"
else
  warn "cannot inspect $BASE_IMAGE -- registry unreachable or needs login. 'docker pull' will fail."
fi

# tb4's depot world pulls its geometry from Fuel at load time. tb3_sandbox has no
# remote dependency, so this is a warning rather than a stop.
hdr "P2b  Fuel reachable (ROBOT=tb4 only)"
# GET, not -I: the Fuel API answers HEAD with 405 Method Not Allowed, so a HEAD
# probe reports the site unreachable when it is fine. The metadata body is ~725 B.
if curl -sf --max-time 10 -o /dev/null \
     "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot" 2>/dev/null; then
  ok "fuel.gazebosim.org reachable -- depot can fetch its geometry"
  info "pre-seed the cache once so the first gz sim start is not a download:"
  info '  docker exec vela-gz gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"'
else
  warn "Fuel unreachable. ROBOT=tb3 is unaffected; ROBOT=tb4 needs depot from Fuel"
fi

# --------------------------------------------------------------- P3 guest
hdr "P3  guest reachable and ready"
if ! gsh 'echo alive' >/dev/null 2>&1; then
  bad "cannot ssh to $GUEST on port $PORT -- is the VM booted? (run it under screen)"
else
  ok "ssh to $GUEST works"
  info "guest kernel: $(gsh 'uname -r' 2>/dev/null)"
  info "guest arch:   $(gsh 'uname -m' 2>/dev/null)"
  G_DISK=$(gsh "df -BG --output=avail / | tail -1 | tr -dc '0-9'" 2>/dev/null)
  info "guest root free: ${G_DISK:-?} GiB"
  [ "${G_DISK:-0}" -ge 1 ] && ok "guest has room for slam_toolbox (~200 MiB)" \
                           || bad "guest is nearly full"

  if gsh 'test -d /opt/ros/jazzy' >/dev/null 2>&1; then
    ok "ROS 2 Jazzy present in the guest"
    info "ros-jazzy packages installed: $(gsh "dpkg -l | grep -c '^ii.*ros-jazzy'" 2>/dev/null)"
  else
    bad "no /opt/ros/jazzy in the guest"
  fi

  if gsh 'apt-cache policy ros-jazzy-slam-toolbox 2>/dev/null | grep -q Candidate' >/dev/null 2>&1; then
    ok "slam_toolbox candidate: $(gsh "apt-cache policy ros-jazzy-slam-toolbox | awk '/Candidate/{print \$2}'" 2>/dev/null)"
  else
    bad "ros-jazzy-slam-toolbox has no candidate -- is the vela.falinux.com source configured?"
  fi

  # Nav2 must already be there; this builds on ras/nav2, it does not replace it.
  NAV2=$(gsh "dpkg -l ros-jazzy-nav2-planner 2>/dev/null | awk '/^ii/{print \$3}'" 2>/dev/null)
  [ -n "$NAV2" ] && ok "Nav2 already installed (nav2-planner $NAV2)" \
                 || warn "Nav2 not installed -- do ras/nav2 first (workload-nav2.md)"

  # tb4 only: the guest runs robot_state_publisher to get the
  # base_link->shell_link->rplidar_link chain that Gazebo's /tf does not carry.
  # tb3 uses two hand-written static transforms instead and needs none of this.
  RSP=$(gsh "dpkg -l ros-jazzy-robot-state-publisher 2>/dev/null | awk '/^ii/{print \$3}'" 2>/dev/null)
  if [ -n "$RSP" ]; then
    ok "robot_state_publisher installed ($RSP) -- tb4 ready"
  else
    RSPC=$(gsh "apt-cache policy ros-jazzy-robot-state-publisher | awk '/Candidate/{print \$2}'" 2>/dev/null)
    warn "robot_state_publisher not installed (candidate ${RSPC:-none}). Needed for ROBOT=tb4 only"
  fi

  # The guest clock runs UTC while the host runs KST. Harmless for this stack
  # because everything is either sim time or a single-clock delta, but it must
  # be on the record before anyone correlates timestamps across the boundary.
  info "host time:  $(date -Is)"
  info "guest time: $(gsh 'date -Is' 2>/dev/null)"
fi

# --------------------------------------------------- P4 interop reference
hdr "P4  record the guest type hashes (interop reference)"
if [ -f "$ROOT/tools/type_hashes.py" ] && gsh 'test -d /opt/ros/jazzy' >/dev/null 2>&1; then
  scp -q -P "$PORT" -o StrictHostKeyChecking=no "$ROOT/tools/type_hashes.py" "$GUEST:/tmp/type_hashes.py" 2>/dev/null
  # Collect into a temp file first. A plain `> hashes.guest.txt` truncates before
  # the remote command runs, so a rerun with the guest down would destroy the
  # reference hashes and leave an empty file that check_interop.sh reads as
  # "no types match".
  if gsh 'python3 /tmp/type_hashes.py' > "$OUT_DIR/hashes.guest.new.txt" 2>/dev/null &&
     [ -s "$OUT_DIR/hashes.guest.new.txt" ]; then
    mv "$OUT_DIR/hashes.guest.new.txt" "$OUT_DIR/hashes.guest.txt"
    ok "wrote $OUT_DIR/hashes.guest.txt"
    sed 's/^/        /' "$OUT_DIR/hashes.guest.txt"
    info "after the docker build, run scripts/check_interop.sh to diff this against the container"
  else
    rm -f "$OUT_DIR/hashes.guest.new.txt"
    warn "could not collect guest type hashes -- any previous hashes.guest.txt is kept"
  fi
else
  warn "tools/type_hashes.py missing, or guest has no ROS -- skipping"
fi

# --------------------------------------------------------------- P5 verdict
hdr "P5  verdict"
printf '  stop conditions: %d\n  warnings:        %d\n' "$STOP" "$WARN"
if [ "$STOP" -gt 0 ]; then
  echo '  RESULT: NO-GO -- fix the STOP items above first'
  exit 1
fi
if [ "$WARN" -gt 0 ]; then
  echo '  RESULT: GO WITH WARNINGS -- read them, then continue to Step 1 (docker build)'
  exit 0
fi
echo '  RESULT: GO -- continue to Step 1 (docker build)'
