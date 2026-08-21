#!/bin/bash
# Put ras_monitor into the guest and prepare the kernel state it reads.
#
# Run it from the HOST -- it drives the guest over ssh, like the other
# install_guest.sh scripts here. Idempotent: run it again after editing a tool
# and only the copies happen.
#
#   scripts/install_guest.sh              # tools + kernel state
#   scripts/install_guest.sh --service    # ... and run it as a systemd service
#
# The kernel-side work lives in guest-tracefs-setup.sh, which runs on the guest
# as root. It is a separate file on purpose: the systemd unit runs the same
# script as ExecStartPre, so the setup this installs and the setup the service
# performs at boot cannot drift apart.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
set -eo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
GUEST=${GUEST:-vela@localhost}
PORT=${PORT:-2222}
GUEST_DIR=${GUEST_DIR:-/home/vela/ras_ros}
GUEST_USER=${GUEST_USER:-vela}
INSTANCE=${INSTANCE:-ras_ros}
TRACING=${TRACING:-/sys/kernel/tracing}

SERVICE=no
[ "$1" = "--service" ] && SERVICE=yes

say() { echo "[$(date +%H:%M:%S)] $*"; }
gsh() { ssh -p "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$GUEST" "bash -l -s" <<< "$*"; }
gcp() { scp -q -P "$PORT" -o BatchMode=yes -o StrictHostKeyChecking=no "$@"; }

say "1/4 copy tools, samples and the setup script to $GUEST_DIR"
gsh "mkdir -p $GUEST_DIR/samples"
gcp "$ROOT"/tools/*.py "$GUEST:$GUEST_DIR/"
gcp "$ROOT"/samples/* "$GUEST:$GUEST_DIR/samples/"
gcp "$HERE/guest-tracefs-setup.sh" "$GUEST:$GUEST_DIR/"
gsh "chmod +x $GUEST_DIR/guest-tracefs-setup.sh $GUEST_DIR/*.py"

say "2/4 kernel support"
# Without CONFIG_RAS there is no tracepoint at all. The monitor would still run
# on kmsg alone, so this reports rather than exits.
gsh "test -d $TRACING/events/ras \
     && echo '      ras tracepoints: present' \
     || echo '      ras tracepoints: MISSING -- kernel lacks CONFIG_RAS; the trace source will be unavailable'"

say "3/4 tracefs group, instance, clock, log directory"
gsh "sudo $GUEST_DIR/guest-tracefs-setup.sh $GUEST_USER" | sed 's/^/      /'

# `sg` applies the group to one command, so this verifies the permissions now
# rather than after the user's next login.
say "4/4 checking the pipe is readable without root"
gsh "sg tracing -c 'test -r $TRACING/instances/$INSTANCE/trace_pipe && \
     head -c 1 $TRACING/instances/$INSTANCE/trace_clock >/dev/null' \
     && echo '      OK' \
     || { echo '      FAIL -- see README.md, If it does not work'; exit 1; }"

if [ "$SERVICE" = yes ]; then
  say "installing the systemd service"
  gcp "$ROOT/systemd/ras-monitor.service" "$GUEST:/tmp/ras-monitor.service"
  gsh "set -e
       sudo install -m 0644 /tmp/ras-monitor.service /etc/systemd/system/ras-monitor.service
       rm -f /tmp/ras-monitor.service
       sudo systemctl daemon-reload
       sudo systemctl enable --now ras-monitor.service
       sleep 3
       systemctl is-active ras-monitor.service" | sed 's/^/      /'
  say "  journalctl -u ras-monitor -f     # on the guest"
fi

say "done. Next:"
say "  ssh -p $PORT $GUEST     # a NEW login, so the tracing group is in effect"
say "  python3 $GUEST_DIR/check_ras_monitor.py"
