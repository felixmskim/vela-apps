#!/bin/bash
# ras-fault-monitor deployment. Run as root in the guest.
set -eu
SRC="$(cd "$(dirname "$0")" && pwd)"
BRIDGE_USER="${BRIDGE_USER:-ubuntu}"

# The unit files are templates. Nothing here may hardcode one machine's home directory,
# so the paths are resolved now and substituted in. Override any of them from the
# environment when the layout differs.
#   RAS_HOME    where this package lives (the parent of this directory)
#   BRIDGE_HOME working directory for the bridge, which runs as BRIDGE_USER
#   ROS_SETUP   the ROS 2 setup.bash the bridge has to source
RAS_HOME="${RAS_HOME:-$(cd "$SRC/.." && pwd)}"
BRIDGE_HOME="${BRIDGE_HOME:-$(getent passwd "$BRIDGE_USER" | cut -d: -f6)}"
BRIDGE_HOME="${BRIDGE_HOME:-/tmp}"
if [ -z "${ROS_SETUP:-}" ]; then
    for c in "$BRIDGE_HOME"/ros2_*/install/setup.bash /opt/ros/*/setup.bash; do
        [ -f "$c" ] && ROS_SETUP="$c" && break
    done
fi
echo "RAS_HOME=$RAS_HOME"
echo "BRIDGE_USER=$BRIDGE_USER  BRIDGE_HOME=$BRIDGE_HOME"
echo "ROS_SETUP=${ROS_SETUP:-(none found)}"

render() {   # render <template> <destination>
    sed -e "s#@RAS_HOME@#$RAS_HOME#g" \
        -e "s#@BRIDGE_HOME@#$BRIDGE_HOME#g" \
        -e "s#@BRIDGE_USER@#$BRIDGE_USER#g" \
        -e "s#@ROS_SETUP@#${ROS_SETUP:-/nonexistent}#g" \
        "$1" > "$2"
    chmod 644 "$2"
}

# The core daemon runs as root, the bridge as an unprivileged user. They write into the
# same directory, so it is created setgid (2775) and new files inherit the group.
# Skip this and the bridge dies with PermissionError while Restart=always hides it as a
# restart loop (measured: is-active still read active).
install -d -m 2775 /var/log/ras
if getent passwd "$BRIDGE_USER" >/dev/null; then
    chgrp "$BRIDGE_USER" /var/log/ras
    echo "log directory group: $BRIDGE_USER (setgid)"
fi

# The example policy is installed for reference only. It is not placed as `policy.yaml`
# because general monitoring has to work with no configuration, and an example threshold
# must never take effect as if it were this machine's own value.
install -d -m 755 /etc/ras
install -m 644 "$SRC/policy.example.yaml" /etc/ras/policy.example.yaml

install -m 644 "$SRC/ras-modules.conf" /etc/modules-load.d/ras.conf
render "$SRC/ras-fault-monitor.service.in" /etc/systemd/system/ras-fault-monitor.service
render "$SRC/ras-topic-bridge.service.in" /etc/systemd/system/ras-topic-bridge.service

modprobe hwpoison-inject 2>/dev/null || echo "warn: hwpoison-inject failed to load"

systemctl daemon-reload
systemctl enable --now ras-fault-monitor.service

if [ -n "${ROS_SETUP:-}" ] && [ -f "$ROS_SETUP" ]; then
    systemctl enable --now ras-topic-bridge.service
else
    echo "no ROS 2 setup.bash — skipping the topic bridge"
fi

# **Startup check**: Restart=always hides a failure as a restart loop. Wait a moment and
# check that the restart counter has not moved — without this, "active" is all you see.
echo "checking startup stability (30s) ..."
sleep 30
rc=0
for unit in ras-fault-monitor ras-topic-bridge; do
    systemctl is-enabled --quiet "$unit" 2>/dev/null || continue
    n=$(systemctl show -p NRestarts --value "$unit")
    state=$(systemctl is-active "$unit" || true)
    printf "  %-22s state=%-10s restarts=%s\n" "$unit" "$state" "$n"
    if [ "$state" != "active" ] || [ "${n:-0}" -gt 1 ]; then
        echo "    ^ unstable — check journalctl -u $unit"
        rc=1
    fi
done
exit $rc
