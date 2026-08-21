#!/bin/bash
# Run straight after the docker build, before any gate. Answers one question:
# can these two ROS 2 installations talk to each other at all?
#
# They come from different repositories, so every message package sits at a
# different patch version. What decides interop is not the version but the type
# hash, which is derived from the message definition. If a hash differs,
# discovery finds nothing and it looks exactly like a transport fault -- so rule
# it out here rather than inside run_dds_check.sh.
#
# No `set -u`: see 00-preflight.sh.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
CONTAINER=${CONTAINER:-vela-gz}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
# One home for the interop reference, shared with 00-preflight.sh OUT_DIR.
REF=${REF:-${OUT_DIR:-$ROOT/artifacts}/hashes.guest.txt}

hdr() { printf '\n=== %s ===\n' "$*"; }
gsh() { ssh -p "$PORT" -o StrictHostKeyChecking=no -o BatchMode=yes "$GUEST" "bash -l -s" <<< "$*"; }

# -f State.Running: bare `docker inspect` also passes for a *stopped* container
# (docker cp would then still succeed and the failure would surface later with
# the wrong message, from docker exec).
[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ] || {
  echo "STOP: container '$CONTAINER' is not running -- do Step 1 and Step 2 first"; exit 1; }

hdr "1  middleware and message package versions"
printf '%-30s %-24s %-24s\n' PACKAGE CONTAINER GUEST
for p in ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs \
         ros-jazzy-tf2-msgs ros-jazzy-rosgraph-msgs ros-jazzy-rmw-fastrtps-cpp \
         ros-jazzy-fastrtps ros-jazzy-rclcpp ros-jazzy-nav2-msgs; do
  c=$(docker exec "$CONTAINER" dpkg-query -W -f='${Version}' "$p" 2>/dev/null)
  g=$(gsh "dpkg-query -W -f='\${Version}' $p" 2>/dev/null)
  printf '%-30s %-24s %-24s%s\n' "$p" "${c:-none}" "${g:-none}" \
    "$([ "$c" != "$g" ] && echo '  differ')"
done
echo "  Version skew is expected and is not by itself a problem."

hdr "2  type hashes -- this is what actually decides interop"
docker cp "$ROOT/tools/type_hashes.py" "$CONTAINER":/tmp/type_hashes.py >/dev/null 2>&1 || {
  echo "STOP: could not copy tools/type_hashes.py into the container"; exit 1; }
docker exec "$CONTAINER" python3 /tmp/type_hashes.py > "$ROOT/artifacts/hashes.container.txt" 2>/dev/null || {
  echo "STOP: could not read type hashes in the container"; exit 1; }

if [ ! -f "$REF" ]; then
  mkdir -p "$(dirname "$REF")"
  echo "  no $REF -- collecting from the guest now"
  scp -q -P "$PORT" -o StrictHostKeyChecking=no "$ROOT/tools/type_hashes.py" "$GUEST:/tmp/type_hashes.py" 2>/dev/null
  # Collect into a temp file first, exactly like 00-preflight.sh P4: `> "$REF"`
  # truncates before ssh runs, so a down guest would leave an empty reference
  # that every later run silently diffs against as a spurious MISMATCH.
  if gsh 'python3 /tmp/type_hashes.py' > "$REF.new" 2>/dev/null && [ -s "$REF.new" ]; then
    mv "$REF.new" "$REF"
  else
    rm -f "$REF.new"
    echo "STOP: could not read type hashes in the guest"; exit 1
  fi
fi

if diff -u "$REF" "$ROOT/artifacts/hashes.container.txt" > "$ROOT/artifacts/hashes.diff" 2>&1; then
  echo "  all type hashes identical:"
  sed 's/^/    /' "$ROOT/artifacts/hashes.container.txt"
  echo
  echo "RESULT: PASS -- the version skew is cosmetic. Proceed to Step 4"
  echo "        (guest: apt-get install ros-jazzy-slam-toolbox). G1 is Step 5."
  exit 0
fi

echo "  MISMATCH:"
sed 's/^/    /' "$ROOT/artifacts/hashes.diff"
cat <<'EOF'

RESULT: FAIL -- endpoints for the mismatched types will never match, and
discovery will look broken for reasons that have nothing to do with transport.

Which types matter, and what to do:

  /clock /scan /odom /imu /tf /cmd_vel   bridged -- a mismatch here blocks the
                                         split entirely
  NavigateToPose, OccupancyGrid          guest-internal only -- a mismatch is
                                         harmless for the split

Options, cheapest first:
  1. Pin the container's message packages to the guest's versions in the
     Dockerfile (apt-get install ros-jazzy-sensor-msgs=<guest version> ...).
     packages.ros.org keeps old versions only briefly, so this may not resolve.
  2. Build the container from the same falinux repository the guest uses --
     it publishes 'all' and 'riscv64' only, so this needs the source packages.
  3. Fall back to bag replay (README, "Fallback"), which crosses the boundary as
     a file rather than as live DDS and is immune to hash mismatch.
EOF
exit 1
