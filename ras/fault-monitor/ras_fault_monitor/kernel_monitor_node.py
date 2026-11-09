# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""The `kernel_monitor` node — reads `/dev/kmsg` live and publishes fault events.

It corresponds to the `kernel_monitor` block of the p.13 pipeline in the ETRI deck.
One difference: the ETRI demo **imitated RERI events as strings** in `/dev/kmsg` and only
verified collection (deck p.15, scenario 3), whereas this node receives real hardware
errors that came up q-vela QEMU's actual path: **RERI device -> OpenSBI -> SBI SSE ->
GHES/APEI -> printk**.

Publishes: `/fault_log` (std_msgs/String, one JSON record)
"""

from __future__ import annotations

import errno

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

from .pipeline import FaultPipeline, _uptime_now, make_reader

_DURABILITY = {
    "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL,
    "volatile": QoSDurabilityPolicy.VOLATILE,
}


def fault_qos(durability: str = "transient_local") -> QoSProfile:
    """QoS for fault events.

    * RELIABLE — nothing may be lost (the TC.10.007 criterion: "CE detection miss rate =
      0%").
    * TRANSIENT_LOCAL (the default) — **a late subscriber still receives the preceding
      events**, so the outcome does not depend on node startup order (a real incident:
      starting the logger 12 seconds after the monitor received not one already-published
      event). The cost is that a subscriber restart replays past events, so the logger
      deduplicates on `kmsg.seq`.
    * Publisher and subscriber must agree for the match to happen -> both nodes read the
      same parameters.
    """
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=_DURABILITY.get(durability, QoSDurabilityPolicy.TRANSIENT_LOCAL),
        depth=100,
    )


class KernelMonitor(Node):

    def __init__(self):
        super().__init__("kernel_monitor")

        self.declare_parameter("kmsg_path", "/dev/kmsg")
        self.declare_parameter("fault_topic", "/fault_log")
        self.declare_parameter("poll_period", 0.1)
        self.declare_parameter("from_beginning", False)
        self.declare_parameter("ghes_timeout", 0.5)
        self.declare_parameter("marker_window", 60.0)
        self.declare_parameter("echo_console", True)
        self.declare_parameter("durability", "transient_local")

        path = self.get_parameter("kmsg_path").value
        topic = self.get_parameter("fault_topic").value
        period = float(self.get_parameter("poll_period").value)
        self.echo_console = bool(self.get_parameter("echo_console").value)

        self.pipeline = FaultPipeline(
            node_name=self.get_name(),
            ghes_timeout=float(self.get_parameter("ghes_timeout").value),
            marker_window=float(self.get_parameter("marker_window").value),
        )

        self.reader = make_reader(
            path, from_beginning=bool(self.get_parameter("from_beginning").value)
        )
        try:
            self.reader.open()
        except PermissionError as exc:
            self.get_logger().fatal(
                f"failed to open {path} ({exc.strerror}). Reading /dev/kmsg needs "
                "CAP_SYSLOG.\n"
                "  fix 1: sudo sysctl -w kernel.dmesg_restrict=0   (test VM only, reverts "
                "on reboot)\n"
                "  fix 2: run the node as root"
            )
            raise
        except OSError as exc:
            if exc.errno == errno.EACCES:
                self.get_logger().fatal(
                    f"no permission for {path} — check kernel.dmesg_restrict")
            raise

        durability = str(self.get_parameter("durability").value)
        self.publisher = self.create_publisher(String, topic, fault_qos(durability))
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Kernel Monitor started — source={path} topic={topic} "
            f"poll={period}s durability={durability} "
            f"from_beginning={self.get_parameter('from_beginning').value}"
        )

    # -- periodic processing ------------------------------------------------
    def _on_timer(self) -> None:
        records = self.reader.read_available()
        faults = self.pipeline.process(records)
        faults.extend(self.pipeline.tick(_uptime_now()))
        for fault in faults:
            self._publish(fault)

    def _publish(self, fault) -> None:
        msg = String()
        msg.data = fault.to_json()
        self.publisher.publish(msg)
        if self.echo_console:
            self._echo(fault.severity, fault.one_line())

    @staticmethod
    def _echo_impl(logger, severity: str, text: str) -> None:
        """Warning: **each severity needs its own line**.

        rclpy's `RcutilsLogger` remembers the severity per call site (file, function, line)
        and raises `ValueError: Logger severity cannot be changed between calls.` when the
        same line is called again with a different severity. Dispatching from one line as
        `getattr(logger, name)(text)` therefore dies on the second event (measured: the
        node crashed on a RERI_FAULT ERROR right after a marker INFO).
        """
        if severity == "CRITICAL":
            logger.fatal(text)
        elif severity == "ERROR":
            logger.error(text)
        elif severity == "WARN":
            logger.warn(text)
        else:
            logger.info(text)

    def _echo(self, severity: str, text: str) -> None:
        self._echo_impl(self.get_logger(), severity, text)

    def shutdown(self) -> None:
        # On Ctrl-C the rclpy signal handler has already closed the context, so publishing
        # fails. To avoid losing the remaining events silently, they go to stdout when
        # publishing is impossible.
        pending = self.pipeline.flush()
        for fault in pending:
            if rclpy.ok():
                self._publish(fault)
            else:
                print("[shutdown] unpublished event: " + fault.to_json(), flush=True)

        stats = self.pipeline.stats
        self._final(
            f"Kernel Monitor stopping — records={stats.records_read} "
            f"faults={stats.faults_emitted} ghes={stats.ghes_events} "
            f"lost_chunks={getattr(self.reader, 'lost_chunks', 0)} by_type={stats.by_type}"
        )
        self.reader.close()

    def _final(self, text: str) -> None:
        """The exit summary — sent to rosout only while the context is alive.

        After SIGINT it prints `Failed to publish log message to rosout: publisher's
        context is invalid` (harmless functionally, but it leaves noise in the evidence
        log).
        """
        if rclpy.ok():
            self.get_logger().info(text)
        else:
            print(text, flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KernelMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
