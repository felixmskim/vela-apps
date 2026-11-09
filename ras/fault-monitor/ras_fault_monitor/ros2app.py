# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""ROS 2 application detector — **outside observation that does not depend on ROS 2**.

Why watch from outside
----------------------
A ROS 2 node **cannot report its own death**. A detector running inside rclpy stops the
moment DDS stalls or the node dies — its eyes close exactly at the event that matters
most. So this module never imports `rclpy`; it only reads procfs. Running in the
always-on core under systemd, it can still record the fact when the whole ROS 2 stack
goes down.

What it looks at (signatures confirmed by measurement)
-----------------------------------------------------
* `rclpy` / `ros2` / a workspace path in `/proc/<pid>/cmdline` -> the ROS 2 process list
* the UDP sockets that process owns (`/proc/<pid>/fd` -> `socket:[inode]`)
  -> DDS participants. **Port numbers are never used to decide this**: an RTPS port is
  `7400 + 250*domain + ...`, so it moves with the domain and participant id, and a
  hardcoded range goes blind the instant the domain changes. Finding them by owner
  carries no such assumption.
* per-socket `rx_queue` and `drops` from `/proc/net/udp`
  -> **a direct indicator that a subscriber cannot keep up**. A growing rx_queue is
  imminent loss, and it is the other end of the same event the eBPF collector sees as
  the `SOCKET_RCVBUFF` drop reason.

A caution on parsing `/proc/net/udp`
------------------------------------
The header names 15 columns while a data row has 13 fields, because `tx_queue:rx_queue`
and `tr:tm->when` are each one colon-joined token. Pairing naively by header name reads
`drops` from the wrong column.

Classification
--------------
No new `fault_type` is created (the 7 ETRI types stay; the distinction lives in tags).
A dead node is not a communication error, so it is recorded as `SYSTEM_DEGRADED` +
`node_down` rather than `DDS_FAULT`.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field

from .fault import Fault

PROC = "/proc"
NET_UDP = "/proc/net/udp"
NET_UDP6 = "/proc/net/udp6"

#: any of these in the cmdline makes a process a ROS 2 **candidate**.
ROS_MARKERS = ("rclpy", "rclcpp", "ros2 ", "/ros2_", "ros2_jazzy",
               "ros2_humble", "/opt/ros/", "ament_", "_ros2_")

#: Shells are excluded. A wrapper shell that ran
#: `source .../ros2_jazzy/install/setup.bash` carries the ROS path in its cmdline and
#: hits the markers, but it is not a node (a measured false positive). Left in, the
#: "whole stack gone" verdict can never hold, because the surviving wrapper keeps the
#: process list non-empty.
#: Shells **and launchers** are excluded — both carry a ROS path in cmdline without
#: being nodes. Measured false positive:
#: `runuser -u ubuntu -- bash -lc "source .../setup.bash; python3 ..."` was counted as a
#: node and polluted both the participant count and the identity list. A launcher maps
#: no ROS libraries, so it never trips the first-class evidence in the first place —
#: only the second-class (cmdline) path needs blocking.
SHELL_COMMS = frozenset({
    "bash", "sh", "dash", "zsh", "ksh", "fish", "busybox",
    "runuser", "sudo", "su", "env", "timeout", "nohup", "setsid",
    "stdbuf", "nice", "ionice", "systemd-run",
})

#: **First-class evidence** — mapped ROS/DDS shared libraries.
#: cmdline markers alone miss launch forms that never mention ROS, such as
#: `python3 /opt/myapp/node.py` (measured: such a node dropped out of observation
#: entirely). Real apps mostly start that way, so cmdline is demoted to second-class
#: evidence.
#: Measured distinction: a real node maps rcl/rmw/fastcdr/fastrtps/rosidl; a wrapper
#: shell or `timeout` maps none of them.
ROS_LIB_RE = re.compile(
    r"lib(rcl|rmw|fastcdr|fastrtps|fastdds|ddsc|rosidl)[a-z_0-9]*\.so")

#: node-name hint from `--ros-args -r __node:=imu_node` / `Node("imu_node")`
_NODE_ARG_RE = re.compile(r"__node:=([\w/]+)")
_NODE_CTOR_RE = re.compile(r"Node\(\s*[\"']([\w/]+)[\"']")

#: socket fd symlink
_SOCK_RE = re.compile(r"^socket:\[(\d+)\]$")


@dataclass
class DdsSocket:
    """One UDP socket used by a DDS participant."""

    inode: int
    port: int
    tx_queue: int = 0
    rx_queue: int = 0
    drops: int = 0
    pid: int | None = None

    def key(self) -> str:
        return f"{self.pid}:{self.port}"


@dataclass
class RosProcess:
    """One ROS 2 process."""

    pid: int
    comm: str
    cmdline: str
    node_hint: str = ""
    detect_source: str = "cmdline"   # 'maps' (first-class) | 'cmdline' (second-class)

    def identity(self) -> str:
        """Identity used to recognize a restart. The pid changes, so it leans on the
        name."""
        return self.node_hint or self.comm


@dataclass
class Ros2Snapshot:
    wall_time: float = 0.0
    processes: dict = field(default_factory=dict)   # pid -> RosProcess
    sockets: dict = field(default_factory=dict)     # key -> DdsSocket

    def identities(self) -> set:
        return {p.identity() for p in self.processes.values()}

    def participant_pids(self) -> set:
        """pids that actually own a DDS socket — ROS 2 participants on the strongest
        evidence."""
        return {s.pid for s in self.sockets.values() if s.pid is not None}

    def participants(self) -> dict:
        pids = self.participant_pids()
        return {pid: p for pid, p in self.processes.items() if pid in pids}


def parse_net_udp(text: str) -> dict[int, DdsSocket]:
    """`/proc/net/udp` → `{inode: DdsSocket}`.

    Fields are read by **fixed index** rather than located by header name (see the
    module note above).
    """
    out: dict[int, DdsSocket] = {}
    for line in text.splitlines()[1:]:
        f = line.split()
        if len(f) < 13:
            continue
        try:
            port = int(f[1].rsplit(":", 1)[1], 16)
            txq, _, rxq = f[4].partition(":")
            inode = int(f[9])
            drops = int(f[12])
        except (IndexError, ValueError):
            continue
        out[inode] = DdsSocket(inode=inode, port=port,
                               tx_queue=int(txq, 16), rx_queue=int(rxq, 16),
                               drops=drops)
    return out


def _read_text(path: str) -> str:
    try:
        with open(path, "rb") as fp:
            return fp.read().decode("utf-8", "replace")
    except OSError:
        return ""


def is_ros_process(cmdline: str) -> bool:
    """Second-class evidence — cmdline markers. Misses nodes that never mention ROS."""
    return any(marker in cmdline for marker in ROS_MARKERS)


def ros_libs(pid: int, proc_dir: str = PROC) -> list:
    """First-class evidence — mapped ROS/DDS libraries, independent of the cmdline."""
    text = _read_text(os.path.join(proc_dir, str(pid), "maps"))
    return sorted(set(ROS_LIB_RE.findall(text))) if text else []


def node_hint(cmdline: str) -> str:
    m = _NODE_ARG_RE.search(cmdline)
    if m:
        return m.group(1)
    m = _NODE_CTOR_RE.search(cmdline)
    return m.group(1) if m else ""


class Ros2Observer:
    """Observes ROS 2 through procfs alone, with no rclpy."""

    def __init__(self, proc_dir: str = PROC, net_udp: str = NET_UDP,
                 net_udp6: str = NET_UDP6, node_name: str = "ros2_observer",
                 rx_backlog_bytes: int = 65536):
        self.proc_dir = proc_dir
        self.net_udp = net_udp
        self.net_udp6 = net_udp6
        self.node_name = node_name
        self.rx_backlog_bytes = rx_backlog_bytes

    # -- snapshot ------------------------------------------------------------
    def scan(self, wall_time: float = 0.0) -> Ros2Snapshot:
        snap = Ros2Snapshot(wall_time=wall_time)
        by_inode: dict[int, DdsSocket] = {}
        for path in (self.net_udp, self.net_udp6):
            by_inode.update(parse_net_udp(_read_text(path)))

        try:
            entries = os.listdir(self.proc_dir)
        except OSError:
            return snap

        for name in entries:
            if not name.isdigit():
                continue
            pid = int(name)
            base = os.path.join(self.proc_dir, name)
            cmdline = _read_text(os.path.join(base, "cmdline")).replace("\0", " ")
            if not cmdline:
                continue
            comm = _read_text(os.path.join(base, "comm")).strip() or f"pid{pid}"
            if comm in SHELL_COMMS:
                continue          # a shell wrapper naming a ROS path — not a node

            # First-class evidence comes first: with the libraries mapped it is a ROS
            # process even without a cmdline marker, and that is how most real apps run.
            libs = ros_libs(pid, self.proc_dir)
            if libs:
                source = "maps"
            elif is_ros_process(cmdline):
                source = "cmdline"
            else:
                continue

            snap.processes[pid] = RosProcess(pid=pid, comm=comm,
                                             cmdline=cmdline.strip(),
                                             node_hint=node_hint(cmdline),
                                             detect_source=source)
            # the UDP sockets this process owns = a DDS participant
            for inode in self._socket_inodes(base):
                sock = by_inode.get(inode)
                if sock is None:
                    continue
                sock.pid = pid
                snap.sockets[sock.key()] = sock
        return snap

    def _socket_inodes(self, proc_base: str) -> list[int]:
        fd_dir = os.path.join(proc_base, "fd")
        out = []
        try:
            names = os.listdir(fd_dir)
        except OSError:
            return out           # no permission / already gone — skip quietly
        for fd in names:
            try:
                link = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            m = _SOCK_RE.match(link)
            if m:
                out.append(int(m.group(1)))
        return out

    # -- change verdicts -----------------------------------------------------
    def diff(self, prev: Ros2Snapshot, cur: Ros2Snapshot) -> list[Fault]:
        """Turn what went wrong between two snapshots into Faults."""
        faults: list[Fault] = []
        ts = cur.wall_time

        prev_parts, cur_parts = prev.participants(), cur.participants()

        gone = prev.identities() - cur.identities()
        for ident in sorted(gone):
            old = next((p for p in prev.processes.values()
                        if p.identity() == ident), None)
            # A candidate found only by cmdline that never owned a DDS socket was never
            # a participant, so its disappearance is not a node death. Sourcing
            # `setup.bash` briefly spawns helpers such as `register-python-argcomplete`
            # whose cmdline carries the workspace path (measured false positive); a real
            # node either maps the ROS libraries or owns a socket.
            if (old is not None and old.detect_source == "cmdline"
                    and old.pid not in prev_parts):
                continue
            faults.append(Fault(
                fault_type="SYSTEM_DEGRADED", severity="ERROR",
                source_layer="application", recoverable=True,
                description=f"ROS 2 node disappeared: {ident}",
                tags=["ros2", "node_down", "app_impact", f"node:{ident}"],
                node_name=self.node_name, timestamp=ts,
                detail={"identity": ident,
                        "pid": old.pid if old else None,
                        "detect_source": old.detect_source if old else "",
                        "cmdline": old.cmdline if old else ""}))

        # The whole ROS 2 stack gone — **an event that can never be reported from
        # inside ROS 2**. The criterion is "DDS participants": is there no process left
        # that owned a socket? Counting by cmdline match alone means one surviving
        # wrapper prevents the verdict, so it would go quiet exactly when it is needed.
        if prev_parts and not cur_parts:
            faults.append(Fault(
                fault_type="SYSTEM_DEGRADED", severity="CRITICAL",
                source_layer="application", recoverable=True,
                description=f"all ROS 2 DDS participants gone ({len(prev_parts)} just before)",
                tags=["ros2", "stack_down", "app_impact", "external_observer"],
                node_name=self.node_name, timestamp=ts,
                detail={"previous_pids": sorted(prev_parts),
                        "still_alive": sorted(cur.processes)}))

        # processes alive but the sockets gone = discovery collapse (distinct from all-gone)
        if cur.processes and prev.sockets and not cur.sockets:
            faults.append(Fault(
                fault_type="DDS_FAULT", severity="ERROR",
                source_layer="application", recoverable=True,
                description="all DDS sockets gone while processes survive — suspected discovery collapse",
                tags=["ros2", "dds", "discovery_down"],
                node_name=self.node_name, timestamp=ts,
                detail={"live_pids": sorted(cur.processes)}))

        faults.extend(self._socket_faults(prev, cur, ts))
        return faults

    def _socket_faults(self, prev: Ros2Snapshot, cur: Ros2Snapshot,
                       ts: float) -> list[Fault]:
        out: list[Fault] = []
        for key, sock in sorted(cur.sockets.items()):
            before = prev.sockets.get(key)

            # loss increased — no comparison if the counter went backwards (socket recreated).
            if before is not None and sock.drops > before.drops:
                delta = sock.drops - before.drops
                out.append(Fault(
                    fault_type="DDS_FAULT", severity="WARN",
                    source_layer="application", recoverable=True,
                    description=(f"DDS socket drops +{delta} "
                                 f"(pid {sock.pid} port {sock.port})"),
                    tags=["ros2", "dds", "udp_drops", f"port:{sock.port}"],
                    node_name=self.node_name, timestamp=ts,
                    actual_value=float(delta),
                    detail={"port": sock.port, "pid": sock.pid,
                            "drops_delta": delta, "drops_total": sock.drops}))

            # receive backlog — the subscriber is not keeping up; SOCKET_RCVBUFF loss next.
            if sock.rx_queue > self.rx_backlog_bytes:
                out.append(Fault(
                    fault_type="DDS_FAULT", severity="WARN",
                    source_layer="application", recoverable=True,
                    description=(f"DDS receive backlog {sock.rx_queue}B "
                                 f"(pid {sock.pid} port {sock.port})"),
                    tags=["ros2", "dds", "rx_backlog", f"port:{sock.port}"],
                    node_name=self.node_name, timestamp=ts,
                    expected_value=float(self.rx_backlog_bytes),
                    actual_value=float(sock.rx_queue),
                    detail={"port": sock.port, "pid": sock.pid,
                            "rx_queue": sock.rx_queue}))
        return out
