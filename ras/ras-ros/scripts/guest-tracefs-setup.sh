#!/bin/bash
# Runs ON THE GUEST, as root. Idempotent. The single source of truth for the
# kernel-side state ras_monitor needs, so install_guest.sh and the systemd unit
# cannot drift apart.
#
#   sudo ./guest-tracefs-setup.sh [user]      # user defaults to vela
#
# Everything here is runtime state and none of it survives a reboot, which is why
# the service runs it again as ExecStartPre.
#
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
set -eo pipefail

USER_NAME=${1:-${GUEST_USER:-vela}}
TRACING=${TRACING:-/sys/kernel/tracing}
INSTANCE=${INSTANCE:-ras_ros}
LOG_DIR=${LOG_DIR:-/var/log/ras}
DIR=$TRACING/instances/$INSTANCE

[ "$(id -u)" = 0 ] || { echo "must run as root" >&2; exit 1; }

mountpoint -q "$TRACING" || mount -t tracefs nodev "$TRACING"

# tracefs files are 0640 root:root. The node must read them as an ordinary user:
# a root-owned Fast DDS participant does not share /dev/shm cleanly with the
# Nav2 nodes it exists to watch. tracefs takes gid= and mode= at mount time.
# mode=750 lets the group in and keeps the world out -- trace data carries kernel
# addresses and task names.
groupadd -f tracing
GID=$(getent group tracing | cut -d: -f3)
mount -o remount,gid="$GID",mode=750 -t tracefs nodev "$TRACING"
id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx tracing || usermod -aG tracing "$USER_NAME"

# A private ring buffer. Reading trace_pipe CONSUMES events, and rasdaemon is
# already reading its own instance -- neither may starve the other.
mkdir -p "$DIR"

# Clock before enable: records written under the previous clock would carry
# timestamps from a domain the monitor cannot subtract from CLOCK_MONOTONIC.
echo mono > "$DIR/trace_clock"
echo 1 > "$DIR/events/ras/non_standard_event/enable"

# A remount does not reliably reach an instance created after it, and an
# unreadable trace_pipe is the failure that leaves the monitor looking healthy
# while seeing nothing.
chgrp -R tracing "$DIR" 2>/dev/null || true
chmod -R g+rX "$DIR" 2>/dev/null || true

# setgid so events.log stays group-writable whoever creates it.
mkdir -p "$LOG_DIR"
chgrp tracing "$LOG_DIR"
chmod 2775 "$LOG_DIR"

echo "trace_clock:  $(cat "$DIR/trace_clock")"
echo "event enable: $(cat "$DIR/events/ras/non_standard_event/enable")"
echo "pipe:         $DIR/trace_pipe"
echo "log dir:      $LOG_DIR"
