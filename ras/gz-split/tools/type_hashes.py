#!/usr/bin/env python3
"""Print the ROS 2 type hash for every message type that crosses the host/guest link.

Why this matters here. The two sides install from different repositories, so
every message package sits at a different patch version -- measured 2026-08-13:

    sensor_msgs      amd64 5.3.8   vs  riscv64 5.3.6
    tf2_msgs         amd64 0.36.21 vs  riscv64 0.36.14
    rmw_fastrtps_cpp amd64 8.4.4   vs  riscv64 8.4.3

Since Iron, ROS 2 attaches a type hash (RIHS01_...) derived from the message
*definition*, not from the package version, and endpoints whose hashes disagree
do not match. A patch release that never touched a .msg file is therefore
harmless, and one that did is fatal -- silently. Discovery simply finds nothing,
which is indistinguishable from a transport problem, so check this before
spending any time debugging Fast DDS.

The hashes are published per type in
/opt/ros/jazzy/share/<pkg>/{msg,action}/<Name>.json, so no node has to run.

Run on both sides and diff:
    python3 type_hashes.py > /tmp/hashes.host.txt        # in the container
    python3 type_hashes.py > /tmp/hashes.guest.txt       # in the guest
    diff /tmp/hashes.host.txt /tmp/hashes.guest.txt
"""

import json
import os
import pathlib
import sys

PREFIX = os.environ.get('ROS_SHARE', '/opt/ros/jazzy/share')

# The six bridged types, plus the three the guest-side stack needs internally.
TYPES = [
    ('rosgraph_msgs', 'Clock'),
    ('sensor_msgs', 'LaserScan'),
    ('nav_msgs', 'Odometry'),
    ('sensor_msgs', 'Imu'),
    ('tf2_msgs', 'TFMessage'),
    ('geometry_msgs', 'Twist'),
    ('nav_msgs', 'OccupancyGrid'),
    ('nav2_msgs', 'NavigateToPose'),
    ('geometry_msgs', 'PoseWithCovarianceStamped'),
]


def hash_of(pkg, name):
    for sub in ('msg', 'action'):
        p = pathlib.Path(PREFIX) / pkg / sub / f'{name}.json'
        if not p.exists():
            continue
        d = json.loads(p.read_text())
        for e in d.get('type_hashes', []):
            if e['type_name'].endswith('/' + name):
                return f'{pkg}/{sub}/{name}', e['hash_string']
    return f'{pkg}/{name}', None


def main():
    missing = 0
    for pkg, name in TYPES:
        label, h = hash_of(pkg, name)
        if h is None:
            print(f'{label:<42s} MISSING')
            missing += 1
        else:
            print(f'{label:<42s} {h}')
    if missing:
        print(f'\n{missing} type(s) not installed on this side', file=sys.stderr)
    return 1 if missing else 0


if __name__ == '__main__':
    sys.exit(main())
