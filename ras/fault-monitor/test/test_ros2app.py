# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The ROS 2 outside-observation detector.

Its essential property: **it never imports rclpy.** A detector running inside ROS 2
stops the moment a node dies or DDS stalls, missing exactly the event that matters most.

The `/proc/net/udp` body is copied verbatim from the guest — the header names 15 columns
while a data row has 13 fields (`tx_queue:rx_queue` is one token), so pairing naively
reads `drops` from the wrong column.
"""

import os

from ras_fault_monitor.ros2app import (
    Ros2Observer, Ros2Snapshot, DdsSocket, RosProcess, is_ros_process,
    node_hint, parse_net_udp,
)

#: verbatim from the guest /proc/net/udp (7400 = 0x1CE8, the discovery socket)
NET_UDP = """\
   sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref pointer drops
 1519: 00000000:1CE8 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 10477 2 ff60000092069f80 0
 1520: 00000000:1CF2 00000000:0000 07 00000000:00012345 00:00000000 00000000     0        0 10478 2 ff60000092069f81 7
"""


def test_parse_net_udp_uses_fixed_indices():
    got = parse_net_udp(NET_UDP)
    assert set(got) == {10477, 10478}
    a, b = got[10477], got[10478]
    assert a.port == 0x1CE8 == 7400 and a.drops == 0 and a.rx_queue == 0
    assert b.port == 0x1CF2 == 7410
    assert b.drops == 7                      # pairing by header goes wrong right here
    assert b.rx_queue == 0x12345


def test_parse_net_udp_skips_malformed_rows():
    assert parse_net_udp("header\nnot enough fields\n") == {}


# --- ROS 2 process identification -------------------------------------------

def test_identifies_rclpy_process():
    assert is_ros_process("python3 -c import rclpy; ...") is True
    assert is_ros_process("/opt/ros/jazzy/lib/demo_nodes_cpp/talker") is True
    assert is_ros_process("/usr/bin/sshd -D") is False


def test_node_hint_from_ros_args():
    assert node_hint("talker --ros-args -r __node:=imu_node") == "imu_node"


def test_node_hint_from_constructor():
    assert node_hint('python3 -c n = Node("lidar_node")') == "lidar_node"


def test_no_hint_is_empty_not_guessed():
    assert node_hint("python3 -c import rclpy") == ""


# --- snapshots (fake procfs) ------------------------------------------------

def _procfs(tmp_path, procs, net_udp=NET_UDP):
    """procs: {pid: (cmdline, comm, [socket_inode, ...])}"""
    root = tmp_path / "proc"
    root.mkdir()
    (root / "net").mkdir()
    (root / "net" / "udp").write_text(net_udp)
    (root / "net" / "udp6").write_text("header only\n")
    for pid, (cmdline, comm, inodes) in procs.items():
        d = root / str(pid)
        (d / "fd").mkdir(parents=True)
        (d / "cmdline").write_bytes(cmdline.replace(" ", "\0").encode())
        (d / "comm").write_text(comm + "\n")
        for i, inode in enumerate(inodes):
            # imitate socket:[N] with os.symlink (the target need not exist)
            os.symlink(f"socket:[{inode}]", d / "fd" / str(i))
    return Ros2Observer(proc_dir=str(root),
                        net_udp=str(root / "net" / "udp"),
                        net_udp6=str(root / "net" / "udp6"))


def test_scan_finds_ros_process_and_its_sockets(tmp_path):
    obs = _procfs(tmp_path, {
        1818: ("python3 -c import rclpy", "python3", [10477, 10478]),
        1: ("/sbin/init", "systemd", []),
    })
    snap = obs.scan(wall_time=100.0)
    assert set(snap.processes) == {1818}
    assert {s.port for s in snap.sockets.values()} == {7400, 7410}
    assert all(s.pid == 1818 for s in snap.sockets.values())


def test_sockets_identified_by_owner_not_port_range(tmp_path):
    """RTPS ports move with the domain — a hardcoded range goes blind when it changes."""
    odd = NET_UDP.replace("1CE8", "2710").replace("1CF2", "2711")   # 10000/10001
    obs = _procfs(tmp_path,
                  {1818: ("python3 -c import rclpy", "python3", [10477, 10478])},
                  net_udp=odd)
    snap = obs.scan()
    assert {s.port for s in snap.sockets.values()} == {10000, 10001}


def test_unreadable_fd_dir_does_not_crash(tmp_path):
    obs = _procfs(tmp_path, {1818: ("python3 -c import rclpy", "python3", [])})
    snap = obs.scan()
    assert set(snap.processes) == {1818} and snap.sockets == {}


# --- change verdicts --------------------------------------------------------

def _snap(procs=(), socks=()):
    s = Ros2Snapshot(wall_time=1.0)
    for pid, ident in procs:
        s.processes[pid] = RosProcess(pid=pid, comm="python3",
                                      cmdline="x rclpy", node_hint=ident)
    for pid, port, drops, rxq in socks:
        sock = DdsSocket(inode=port, port=port, drops=drops, rx_queue=rxq, pid=pid)
        s.sockets[sock.key()] = sock
    return s


def test_node_disappearance_is_degraded_not_dds_fault():
    """A dead node is not a communication error.

    The nodes own sockets here on purpose: a cmdline-only candidate that never owned
    one is not treated as a node death (see
    `test_cmdline_only_nonparticipant_is_not_a_node_death`).
    """
    obs = Ros2Observer()
    faults = obs.diff(_snap([(10, "imu_node"), (11, "gps_node")],
                            [(10, 7400, 0, 0), (11, 7410, 0, 0)]),
                      _snap([(11, "gps_node")], [(11, 7410, 0, 0)]))
    down = [f for f in faults if "node_down" in f.tags]
    assert len(down) == 1
    assert down[0].fault_type == "SYSTEM_DEGRADED"
    assert "node:imu_node" in down[0].tags
    assert down[0].source_layer == "application"


def test_whole_stack_death_is_critical_and_flagged_external():
    """An event that can never be reported from inside ROS 2.

    The criterion is **DDS participants** (socket owners) — counting only processes
    without sockets means one surviving wrapper prevents the verdict.
    """
    obs = Ros2Observer()
    before = _snap([(10, "imu_node"), (11, "gps_node")],
                   [(10, 7400, 0, 0), (11, 7410, 0, 0)])
    faults = obs.diff(before, _snap([]))
    stack = [f for f in faults if "stack_down" in f.tags]
    assert len(stack) == 1
    assert stack[0].severity == "CRITICAL"
    assert "external_observer" in stack[0].tags
    assert stack[0].detail["previous_pids"] == [10, 11]


def test_restart_with_new_pid_is_not_reported_as_death():
    """The pid changes — identity is by name, so a restart is not a death."""
    obs = Ros2Observer()
    faults = obs.diff(_snap([(10, "imu_node")]), _snap([(99, "imu_node")]))
    assert not [f for f in faults if "node_down" in f.tags]


def test_drop_increase_becomes_dds_fault():
    obs = Ros2Observer()
    faults = obs.diff(_snap([(10, "n")], [(10, 7400, 5, 0)]),
                      _snap([(10, "n")], [(10, 7400, 12, 0)]))
    drops = [f for f in faults if "udp_drops" in f.tags]
    assert len(drops) == 1
    assert drops[0].fault_type == "DDS_FAULT"
    assert drops[0].detail["drops_delta"] == 7
    assert drops[0].actual_value == 7.0


def test_drop_counter_reset_is_not_reported():
    """A recreated socket resets the counter — a negative increase must not be reported."""
    obs = Ros2Observer()
    faults = obs.diff(_snap([(10, "n")], [(10, 7400, 500, 0)]),
                      _snap([(10, "n")], [(10, 7400, 3, 0)]))
    assert not [f for f in faults if "udp_drops" in f.tags]


def test_rx_backlog_over_threshold():
    obs = Ros2Observer(rx_backlog_bytes=1000)
    faults = obs.diff(_snap([(10, "n")], [(10, 7400, 0, 0)]),
                      _snap([(10, "n")], [(10, 7400, 0, 5000)]))
    back = [f for f in faults if "rx_backlog" in f.tags]
    assert len(back) == 1
    assert back[0].actual_value == 5000.0 and back[0].expected_value == 1000.0


def test_rx_backlog_under_threshold_is_quiet():
    obs = Ros2Observer(rx_backlog_bytes=10000)
    faults = obs.diff(_snap([(10, "n")], [(10, 7400, 0, 0)]),
                      _snap([(10, "n")], [(10, 7400, 0, 500)]))
    assert not [f for f in faults if "rx_backlog" in f.tags]


def test_discovery_collapse_when_processes_live_but_sockets_gone():
    obs = Ros2Observer()
    faults = obs.diff(_snap([(10, "n")], [(10, 7400, 0, 0)]), _snap([(10, "n")], []))
    disc = [f for f in faults if "discovery_down" in f.tags]
    assert len(disc) == 1 and disc[0].fault_type == "DDS_FAULT"


def test_no_new_fault_types_invented():
    from ras_fault_monitor.fault import FAULT_TYPES
    obs = Ros2Observer(rx_backlog_bytes=1)
    faults = obs.diff(_snap([(10, "a"), (11, "b")], [(10, 7400, 0, 0)]),
                      _snap([], []))
    faults += obs.diff(_snap([(10, "a")], [(10, 7400, 0, 0)]),
                       _snap([(10, "a")], [(10, 7400, 9, 99)]))
    assert faults
    for f in faults:
        assert f.fault_type in FAULT_TYPES, f.fault_type


def test_module_does_not_import_rclpy():
    """The point of the hybrid design — the core has to run without ROS 2."""
    import pathlib
    src = pathlib.Path(__file__).resolve().parents[1] / "ras_fault_monitor" / "ros2app.py"
    text = src.read_text()
    assert "import rclpy" not in text
    assert "from rclpy" not in text


# --- shell-wrapper false positive (it really happened in measurement) -------

def test_shell_wrapper_is_not_a_ros_process(tmp_path):
    """A shell that ran `source .../ros2_jazzy/install/setup.bash` is not a node."""
    obs = _procfs(tmp_path, {
        1911: ("bash -c source /home/ubuntu/ros2_jazzy/install/setup.bash",
               "bash", []),
        1967: ("python3 -c import rclpy; Node(\"imu_node\")", "python3", [10477]),
    })
    snap = obs.scan()
    assert set(snap.processes) == {1967}         # the shell drops out
    assert snap.identities() == {"imu_node"}


def test_lingering_nonparticipant_does_not_suppress_stack_down():
    """The all-gone verdict must hold even with a false positive left over.

    In measurement one surviving shell wrapper kept `cur.processes` non-empty and the
    all-gone detection failed silently — at the very moment it mattered most.
    """
    obs = Ros2Observer()
    before = _snap([(10, "imu_node"), (99, "leftover")], [(10, 7400, 0, 0)])
    after = _snap([(99, "leftover")], [])       # participants gone, only leftovers
    stack = [f for f in obs.diff(before, after) if "stack_down" in f.tags]
    assert len(stack) == 1
    assert stack[0].detail["still_alive"] == [99]


def test_discovery_down_distinct_from_stack_down():
    """Processes alive with only the sockets gone is not the all-gone case."""
    obs = Ros2Observer()
    before = _snap([(10, "imu_node")], [(10, 7400, 0, 0)])
    after = _snap([(10, "imu_node")], [])
    tags = {t for f in obs.diff(before, after) for t in f.tags}
    assert "discovery_down" in tags
    assert "stack_down" in tags          # participants hit 0 too, so both hold


# --- maps as first-class evidence (measured: cmdline alone misses real apps) -

def _procfs_maps(tmp_path, procs, net_udp=NET_UDP):
    """procs: {pid: (cmdline, comm, [inode...], maps_text)}"""
    root = tmp_path / "proc"
    root.mkdir()
    (root / "net").mkdir()
    (root / "net" / "udp").write_text(net_udp)
    (root / "net" / "udp6").write_text("header only\n")
    for pid, (cmdline, comm, inodes, maps) in procs.items():
        d = root / str(pid)
        (d / "fd").mkdir(parents=True)
        (d / "cmdline").write_bytes(cmdline.replace(" ", "\0").encode())
        (d / "comm").write_text(comm + "\n")
        (d / "maps").write_text(maps)
        for i, inode in enumerate(inodes):
            os.symlink(f"socket:[{inode}]", d / "fd" / str(i))
    return Ros2Observer(proc_dir=str(root),
                       net_udp=str(root / "net" / "udp"),
                       net_udp6=str(root / "net" / "udp6"))


#: measured maps excerpt — the libraries a real node maps
REAL_MAPS = """\
7f8000000000-7f8000100000 r-xp 00000000 fe:01 100 /usr/lib/librcl.so.9
7f8000100000-7f8000200000 r-xp 00000000 fe:01 101 /usr/lib/librmw.so.7
7f8000200000-7f8000300000 r-xp 00000000 fe:01 102 /usr/lib/libfastrtps.so.2.14
7f8000300000-7f8000400000 r-xp 00000000 fe:01 103 /usr/lib/libfastcdr.so.2
7f8000400000-7f8000500000 r-xp 00000000 fe:01 104 /usr/lib/librosidl_runtime_c.so
"""

PLAIN_MAPS = """\
7f9000000000-7f9000100000 r-xp 00000000 fe:01 200 /usr/lib/libc.so.6
7f9000100000-7f9000200000 r-xp 00000000 fe:01 201 /usr/lib/libpthread.so.0
"""


def test_node_without_cmdline_marker_found_via_maps(tmp_path):
    """`python3 /opt/myapp/node.py` — a common real-app form that dropped out entirely."""
    from ras_fault_monitor.ros2app import ros_libs
    obs = _procfs_maps(tmp_path, {
        5960: ("python3 /tmp/nodeapp.py", "python3", [10477, 10478], REAL_MAPS),
    })
    snap = obs.scan()
    assert set(snap.processes) == {5960}
    assert snap.processes[5960].detect_source == "maps"
    assert len(snap.sockets) == 2 and snap.participants()
    assert ros_libs(5960, str(tmp_path / "proc"))


def test_wrapper_without_libs_or_sockets_excluded(tmp_path):
    """timeout/wrappers have neither libraries nor sockets — a clean measured split."""
    obs = _procfs_maps(tmp_path, {
        5958: ("timeout 120 python3 /tmp/nodeapp.py", "timeout", [], PLAIN_MAPS),
        5960: ("python3 /tmp/nodeapp.py", "python3", [10477], REAL_MAPS),
    })
    snap = obs.scan()
    assert set(snap.processes) == {5960}


def test_cmdline_marker_remains_secondary_evidence(tmp_path):
    """Unreadable maps (permissions etc.) still catch on cmdline — the evidence grade
    is recorded."""
    obs = _procfs_maps(tmp_path, {
        7000: ("python3 -c import rclpy", "python3", [10477], PLAIN_MAPS),
    })
    snap = obs.scan()
    assert snap.processes[7000].detect_source == "cmdline"


def test_detect_source_recorded_on_node_death():
    """Which evidence caught it has to survive into the fault."""
    obs = Ros2Observer()
    before = Ros2Snapshot(wall_time=1.0)
    before.processes[10] = RosProcess(pid=10, comm="python3",
                                      cmdline="python3 app.py",
                                      node_hint="imu", detect_source="maps")
    faults = obs.diff(before, Ros2Snapshot(wall_time=2.0))
    down = [f for f in faults if "node_down" in f.tags]
    assert down and down[0].detail["detect_source"] == "maps"


def test_launcher_processes_excluded(tmp_path):
    """Measured false positive: runuser counted as a node and polluted the identity
    list."""
    obs = _procfs_maps(tmp_path, {
        6000: ("runuser -u ubuntu -- bash -lc source /home/ubuntu/ros2_jazzy/"
               "install/setup.bash", "runuser", [], PLAIN_MAPS),
        6001: ("sudo -E python3 -c import rclpy", "sudo", [], PLAIN_MAPS),
        6002: ("timeout 120 python3 node.py", "timeout", [], PLAIN_MAPS),
        6003: ("python3 node.py", "python3", [10477], REAL_MAPS),
    })
    snap = obs.scan()
    assert set(snap.processes) == {6003}


def test_cmdline_only_nonparticipant_is_not_a_node_death():
    """Measured false positive: sourcing `setup.bash` spawns short-lived helpers.

    `register-python-argcomplete` carries the workspace path in its cmdline, so it
    matched the markers, and its exit was reported as a ROS 2 node death. A process
    that never owned a DDS socket and was only ever a cmdline match was never a
    participant, so its disappearance is not a node death.
    """
    obs = Ros2Observer()
    before = _snap([(10, "imu_node"), (77, "register-python")],
                   [(10, 7400, 0, 0)])          # only pid 10 owns a socket
    after = _snap([(10, "imu_node")], [(10, 7400, 0, 0)])
    downs = [f for f in obs.diff(before, after) if "node_down" in f.tags]
    assert downs == []


def test_participant_death_is_still_reported_without_maps():
    """The guard must not silence a real node: owning a socket is enough."""
    obs = Ros2Observer()
    before = _snap([(10, "imu_node"), (11, "gps_node")],
                   [(10, 7400, 0, 0), (11, 7410, 0, 0)])
    after = _snap([(10, "imu_node")], [(10, 7400, 0, 0)])
    downs = [f for f in obs.diff(before, after) if "node_down" in f.tags]
    assert len(downs) == 1 and "gps_node" in downs[0].description
