#!/bin/bash
# Gates G3 and G4 -- does ROS 2 discovery and data cross QEMU SLIRP, and how fast?
#
# SLIRP does not forward multicast, so Fast DDS simple discovery over UDP cannot
# work host<->guest. The asymmetric TCP arrangement does: the host listens on
# 42100, the guest dials 10.0.2.2:42100 outbound, and one full-duplex connection
# multiplexes discovery plus both directions of user data over logical ports.
#
# Consequence worth stating: no hostfwd rule is needed, so no QEMU restart.
#
# G3: a topic crosses host -> guest within 120 s.
# G4: round-trip p95 < 500 ms, measured on the guest's CLOCK_MONOTONIC only.
#
# G3 uses `ros2 topic pub` / `ros2 topic echo` rather than demo_nodes_cpp. The
# measured container is built from ros:jazzy-ros-base plus a named package list, and
# demo_nodes_cpp is not on it -- `ros2 run demo_nodes_cpp talker` exits immediately
# with "Package 'demo_nodes_cpp' not found", nothing ever listens on 42100, and the
# failure reads as a transport problem. ros2topic and std_msgs are in ros-base on
# both sides, so this needs no rebuild of the image whose digest is the record.
#
# No `set -u`: see run_rtf.sh.
set -o pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
# Everything a run writes goes under artifacts/ (bind-mounted as /work/artifacts).
mkdir -p "$ROOT/artifacts/maps"
CONTAINER=${CONTAINER:-vela-gz}
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
DOMAIN=${DOMAIN:-42}
GUEST_DIR=${GUEST_DIR:-/home/vela/gz_split}
RTT_N=${RTT_N:-200}
BUDGET_MS=${BUDGET_MS:-500}
PROBE_TOPIC=${PROBE_TOPIC:-/vela_probe}
PROBE_DATA=vela-crossed-slirp

say() { echo "[$(date +%H:%M:%S)] $*"; }
dex()  { docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
dexd() { docker exec -d "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash; $*"; }
# The remote command arrives on stdin rather than inside a quoted -c argument.
# Several commands below contain single quotes -- the pkill bracket trick, the
# waypoint list -- which would close an outer '...' early and mangle the command.
gsh()  { ssh -p "$PORT" -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }

cleanup() {
  say "tearing down"
  docker exec "$CONTAINER" bash -lc "pkill -f '[t]opic pub'" >/dev/null 2>&1
  docker exec "$CONTAINER" bash -lc "pkill -f '[r]tt_probe'" >/dev/null 2>&1
  gsh "pkill -f '[r]tt_probe'; pkill -f '[t]opic pub'" >/dev/null 2>&1
}
trap cleanup EXIT

HOST_ENV="export FASTRTPS_DEFAULT_PROFILES_FILE=/work/config/host_tcp_server.xml; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"
# `ssh ... bash -l -s` is a non-interactive login shell: it reads ~/.profile but not
# ~/.bashrc, and the guest's ~/.profile does not source ROS. Without this every
# remote ros2 call dies with "failed to run command 'ros2': No such file or
# directory" -- which looks nothing like a missing PATH. Source it here rather than
# editing the guest's dotfiles, so the script stands on its own.
GUEST_ENV="source /opt/ros/jazzy/setup.bash; \
export FASTRTPS_DEFAULT_PROFILES_FILE=$GUEST_DIR/guest_tcp_client.xml; \
export ROS_DOMAIN_ID=$DOMAIN; unset ROS_LOCALHOST_ONLY; unset ROS_AUTOMATIC_DISCOVERY_RANGE;"

say "1/5  profiles in place"
dex "test -f /work/config/host_tcp_server.xml && echo host-xml ok" || {
  say "FAIL: host_tcp_server.xml missing in the container"; exit 1; }
gsh "test -f $GUEST_DIR/guest_tcp_client.xml && echo guest-xml ok" || {
  say "FAIL: copy guest_tcp_client.xml to $GUEST_DIR first (see README)"; exit 1; }

say "2/5  clear stale shared memory on both sides"
docker exec "$CONTAINER" bash -lc 'rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*' >/dev/null 2>&1
gsh "rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*" >/dev/null 2>&1

say "3/5  G3: publisher on host, subscriber on guest"
dexd "$HOST_ENV ros2 topic pub -r 2 $PROBE_TOPIC std_msgs/msg/String \
      '{data: $PROBE_DATA}' > /work/artifacts/talker.log 2>&1"
sleep 8
# Fail here rather than blaming the transport: if the publisher never started, the
# host is not listening and every downstream symptom is a consequence.
dex "pgrep -f '[t]opic pub' >/dev/null" || {
  say "FAIL: the host publisher did not start"; dex "tail -5 /work/artifacts/talker.log"; exit 1; }

# Before blaming DDS, confirm the guest can open a plain TCP connection to the
# listening port. If this fails the problem is SLIRP or the profile, not ROS.
say "      guest -> 10.0.2.2:42100 reachable?"
gsh "timeout 5 bash -c 'exec 3<>/dev/tcp/10.0.2.2/42100' && echo '      tcp ok' \
     || echo '      tcp UNREACHABLE - host is not listening on 42100'"

T0=$(date +%s)
OUT=$(gsh "$GUEST_ENV timeout 120 ros2 topic echo --once $PROBE_TOPIC \
           std_msgs/msg/String" 2>&1 | tail -5)
T1=$(date +%s)
echo "$OUT"

if echo "$OUT" | grep -qF "$PROBE_DATA"; then
  say "      G3 PASS -- first sample crossed after $((T1 - T0))s of the 120s budget"
  G3=0
else
  say "      G3 FAIL. Ladder, in order:"
  say "        1) add <interfaceWhiteList><address>127.0.0.1</address> to the host descriptor"
  say "        2) add <wan_addr>10.0.2.2</wan_addr> to the host descriptor"
  say "        3) [user] in the QEMU terminal: Ctrl-A c then 'hostfwd_add tcp::42100-:42100'"
  say "           (no restart needed, but that terminal must still be attached)"
  say "        4) Fast DDS Discovery Server with the guest as SUPER_CLIENT"
  say "      Do NOT set FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA: discovery stays"
  say "      UDP multicast and the TCP direction can invert to host->guest."
  G3=1
fi

say "      established connections on 42100:"
ss -tnp 2>/dev/null | grep ':42100' || say "        (none visible)"

if [ "$G3" -ne 0 ]; then
  say "RESULT: FAIL (G3) -- skipping G4"
  exit 1
fi

say "4/5  G4: round-trip latency, $RTT_N samples"
# Stop the G3 publisher first. Port 42100 has exactly one owner: whichever host
# participant starts first binds the TCP acceptor, and every later one logs
# `TCPTransport Error binding at port: (42100) ... Address already in use` and comes
# up with no acceptor at all. Leaving the publisher running made the echo node
# unreachable and G4 failed as `echo node never discovered`, which reads like a
# discovery problem rather than a port conflict.
dex "pkill -f '[t]opic pub'" >/dev/null 2>&1
sleep 3
dexd "$HOST_ENV python3 /work/tools/rtt_probe.py --role echo > /work/artifacts/rtt-echo.log 2>&1"
sleep 5
dex "grep -q 'Address already in use' /work/artifacts/rtt-echo.log" && {
  say "FAIL: the echo node could not bind 42100 -- something else still holds it"
  dex "ss -tlnp 2>/dev/null | grep ':42100'"; exit 1; }
gsh "$GUEST_ENV python3 $GUEST_DIR/rtt_probe.py --role ping -n $RTT_N --budget-ms $BUDGET_MS"
G4=$?

say "5/5  regression: does the in-guest path still work with the profile loaded?"
# The guest profile keeps useBuiltinTransports true precisely so the nine local
# Nav2/SLAM participants keep discovering each other over SHM/UDP. Prove it.
# The exit status has to be the subscriber's, not pkill's -- pkill succeeds whenever
# it finds the publisher, which would make this step pass unconditionally.
gsh "$GUEST_ENV rm -f /dev/shm/fastrtps_*; \
     (ros2 topic pub -r 2 /vela_local std_msgs/msg/String '{data: $PROBE_DATA}' \
      > /tmp/gt.log 2>&1 &) ; sleep 8; \
     timeout 30 ros2 topic echo --once /vela_local std_msgs/msg/String; RC=\$?; \
     pkill -f '[t]opic pub' >/dev/null 2>&1; exit \$RC"
G5=$?

if [ "$G4" -eq 0 ] && [ "$G5" -eq 0 ]; then
  say "RESULT: PASS (G3 + G4 + intra-guest regression)"
  exit 0
fi
say "RESULT: FAIL  g4=$G4 intra_guest=$G5"
[ "$G5" -ne 0 ] && say "  intra-guest broke: add a UDPv4 descriptor and a 127.0.0.1 initial peer to the guest userTransports"
exit 1
