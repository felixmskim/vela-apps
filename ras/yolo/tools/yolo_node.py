#!/usr/bin/env python3
"""YOLOv8n inference node -- the AI-inference co-tenant for TC.10.007.

Subscribes to a camera Image topic, runs YOLOv8n through onnxruntime, and
publishes vision_msgs/Detection2DArray on /detection_result -- the topic the
TSP names in its monitoring precondition (`ros2 topic hz ... /detection_result`).

Two design points exist only because of TCG:

Inference does not run in the subscription callback. One 320x320 pass takes
tens of seconds on an emulated core (D-13 relaxed the cadence for exactly this
reason), and a callback that long would starve the executor. The callback only
stores the newest frame under a lock; a worker thread takes whatever is newest
when it finishes the previous pass. Frames that arrive while a pass is running
are dropped by design -- detection freshness matters, backlog does not.

An empty Detection2DArray is still published after every pass. For an
availability test the message flow itself is the liveness signal: a fault that
kills this node must be visible as /detection_result going quiet, not as an
ambiguous "no objects in view".

Every pass also logs one parseable line:

    [detect] seq=12 stamp=105.2 n=2 infer_s=38.4 rx=61 drop=48 mono=8123.412 \
             labels=person:0.87,bus:0.72

`stamp` is the frame's, so it is SIM time; `mono` is CLOCK_MONOTONIC at the end
of the pass, which is the guest's wall clock and the only field that can be
compared with anything outside this simulator. ras_monitor.py stamps its RAS
events with the same clock under the same name, so "the detector was doing this
when the error arrived" is a join on two numbers rather than a guess. Append to
this line; do not reorder it.

The same state also goes out as `diagnostic_msgs/DiagnosticArray` on
`/diagnostics`, under the name `yolo/detector`:

    ros2 topic echo /diagnostics --once

A log line is evidence for a person reading a file afterwards. A topic is
evidence a program can act on while the test is running, and TC.10.007 asks
whether AI inference stayed healthy across an error injection -- which is a
question about a moment, not about a file. `diagnostic_msgs` is what
ras_monitor.py publishes for the same reason and for the same consumer, so the
error and the workload's reaction to it arrive on one topic in one type.

The status carries a HEARTBEAT: it is published every 10 s whether or not a pass
finished, because an availability test has to distinguish a detector that found
nothing from a detector that is gone, and only a periodic message can do that.
"""

import pathlib
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                      DurabilityPolicy)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)
from cv_bridge import CvBridge


def mono():
    """CLOCK_MONOTONIC, in seconds.

    Not `Node.get_clock()`: that is ROS time, which is sim time here whenever
    use_sim_time is on and jumps backwards when a bag loops. And not
    `time.time()`, which NTP can step. ras_monitor.py uses this same call for its
    `detect_mono`, and the whole point of the field is that the two are joinable.
    """
    return time.clock_gettime(time.CLOCK_MONOTONIC)

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yolov8_onnx import Yolov8Onnx


class YoloNode(Node):

    def __init__(self):
        super().__init__("yolo_detector")
        self.declare_parameter("model_path", str(pathlib.Path.home() / "yolo" / "yolov8n.onnx"))
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/detection_result")
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        # 2 of 4 vCPUs -- leave room for Nav2/slam_toolbox, see yolov8_onnx.py.
        self.declare_parameter("threads", 2)
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        # 1 Hz, and deliberately not ras_monitor.py's 10 s. That period is right
        # for an event reporter, where the events are the signal and the heartbeat
        # only proves the reporter is there. Here the heartbeat IS the signal: the
        # TSP's 3rd-year criterion is detection within 20 s and its 4th-year MTTR
        # budget is 3 s, and a status published every 10 s cannot resolve either.
        # It is also the only thing that runs at a fixed rate in this node -- a
        # pass finishes every 40 s of wall clock at RTF 0.25 -- so it is what
        # separates "found nothing" from "gone".
        self.declare_parameter("heartbeat", 1.0)
        # How long without a camera frame before the status turns WARN. Default
        # 120 s and it is not arbitrary: the split rig runs the camera at 0.1 Hz
        # of SIM time, which at RTF_LIMIT=0.25 is a frame every 40 s of wall
        # clock, so this is three missed frames. A rig with a faster camera
        # should lower it -- a stale threshold that never trips reports a dead
        # camera as healthy.
        self.declare_parameter("stale_s", 120.0)
        self.declare_parameter("hardware_id", "")

        model_path = self.get_parameter("model_path").value
        self.model = Yolov8Onnx(
            model_path,
            imgsz=self.get_parameter("imgsz").value,
            conf=self.get_parameter("conf").value,
            iou=self.get_parameter("iou").value,
            threads=self.get_parameter("threads").value)
        self.get_logger().info(
            f"loaded {model_path} on {self.model.runtime}, "
            f"imgsz={self.model.imgsz} threads={self.get_parameter('threads').value}")

        self.bridge = CvBridge()
        self.latest = None            # newest unprocessed (msg,) under the lock
        self.cond = threading.Condition()
        self.seq = 0
        self.rx = 0
        self.dropped = 0              # frames replaced before they were inferred on
        self.last = None              # the last completed pass, for the status
        self.error = None             # the last exception, if any
        self.failures = 0             # consecutive, reset by a pass that completes
        self.inferring = False        # a pass is in flight right now
        self.started_mono = mono()
        self.frame_mono = None        # when the newest frame arrived
        self.hardware_id = self.get_parameter("hardware_id").value or socket.gethostname()

        # Depth 1 + best effort: a queue of stale camera frames is pure cost
        # here -- see the module docstring.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(
            Image, self.get_parameter("input_topic").value, self.on_image, qos)
        self.pub = self.create_publisher(
            Detection2DArray, self.get_parameter("output_topic").value, 10)
        # TRANSIENT_LOCAL, unlike /detection_result. A monitor that attaches
        # mid-test -- which is every monitor, since the rig takes 80 s to come up
        # and an operator starts watching after that -- would otherwise wait up to
        # a heartbeat period seeing nothing, and "nothing" is exactly what a dead
        # detector looks like. Offered durability may exceed requested, so a
        # VOLATILE subscriber (the /diagnostics default, and what ras_monitor's
        # own consumers use) still matches this publisher.
        self.diag = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(
            self.get_parameter("heartbeat").value, self.publish_status)

        self.worker = threading.Thread(target=self.loop, daemon=True)
        self.worker.start()
        # One status before the first frame, so a consumer that started first sees
        # the detector exists and is waiting rather than nothing at all.
        self.publish_status()

    def on_image(self, msg):
        self.rx += 1
        with self.cond:
            # Counting the overwrite is the only way the drop rate is knowable
            # from outside: rx - seq also counts the frame currently being
            # inferred on and the one waiting, so it drifts by up to two.
            if self.latest is not None:
                self.dropped += 1
            self.latest = msg
            self.frame_mono = mono()
            self.cond.notify()

    def loop(self):
        while rclpy.ok():
            with self.cond:
                while self.latest is None:
                    if not self.cond.wait(timeout=1.0):
                        if not rclpy.ok():
                            return
                msg, self.latest = self.latest, None

            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.error = f"cv_bridge: {e}"
                self.failures += 1
                self.get_logger().warning(
                    f"[detect-error] seq={self.seq} fail={self.failures} cv_bridge: {e}")
                self.publish_status()
                continue

            self.inferring = True
            try:
                dets, dt = self.model(frame)
            except Exception as e:
                # An inference that raises must not take the node with it: the
                # status is the evidence that it failed, and a node that exited
                # instead would leave the same silence as a node that was killed.
                self.error = f"inference: {e}"
                self.failures += 1
                self.inferring = False
                self.get_logger().error(
                    f"[detect-error] seq={self.seq} fail={self.failures} {e}")
                self.publish_status()
                continue
            self.error = None
            self.failures = 0
            self.inferring = False
            self.seq += 1

            out = Detection2DArray()
            out.header = msg.header
            for d in dets:
                x1, y1, x2, y2 = d["xyxy"]
                det = Detection2D()
                det.header = msg.header
                # float(), explicitly, on every one. These are `double` fields
                # and rosidl's C converter does not coerce -- handed an int it
                # fails a PyFloat_Check ASSERTION, which aborts the process. So a
                # detector that returned integer pixels would not raise an
                # exception this node could report; it would vanish mid-test with
                # a SIGABRT, which in an availability test is indistinguishable
                # from the fault being measured. yolov8_onnx.py casts already;
                # this makes that a local guarantee rather than a remote one.
                det.bbox.center.position.x = float(x1 + x2) / 2
                det.bbox.center.position.y = float(y1 + y2) / 2
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(d["cls_name"])
                hyp.hypothesis.score = float(d["conf"])
                det.results.append(hyp)
                out.detections.append(det)
            self.pub.publish(out)

            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            labels = ",".join(f"{d['cls_name']}:{d['conf']:.2f}" for d in dets)
            self.last = {"seq": self.seq, "stamp": stamp, "n": len(dets),
                         "infer_s": dt, "labels": labels, "mono": mono()}
            self.get_logger().info(
                f"[detect] seq={self.seq} stamp={stamp:.3f} n={len(dets)} "
                f"infer_s={dt:.1f} rx={self.rx} drop={self.dropped} "
                f"mono={self.last['mono']:.3f} labels={labels}")
            self.publish_status()

    def publish_status(self):
        """One DiagnosticStatus for this detector, on /diagnostics.

        Called after every pass AND on a timer. Both, deliberately: the pass gives
        a consumer the result as soon as it exists, and the timer is what makes an
        absence of results readable -- at RTF 0.25 a pass is 40 s apart, so
        without the timer a healthy detector and a dead one look the same for most
        of a minute.
        """
        now = mono()
        last, err = self.last, self.error
        age = None if self.frame_mono is None else now - self.frame_mono
        stale_s = self.get_parameter("stale_s").value

        if err:
            level, message = DiagnosticStatus.ERROR, err
        elif age is None:
            level = DiagnosticStatus.WARN
            message = f"no camera frame yet ({now - self.started_mono:.0f}s)"
        elif age > stale_s:
            level = DiagnosticStatus.WARN
            message = f"no camera frame for {age:.0f}s (>{stale_s:.0f}s)"
        elif last is None:
            level = DiagnosticStatus.WARN
            message = f"{self.rx} frame(s) in, first pass not finished yet"
        else:
            level = DiagnosticStatus.OK
            message = (f"inferring, {last['n']} object(s) in "
                       f"{last['infer_s']:.1f}s: {last['labels'] or 'none'}")

        st = DiagnosticStatus()
        st.level = level
        st.name = "yolo/detector"
        st.message = message
        st.hardware_id = self.hardware_id
        # `state` is what a consumer switches on; the counters are what it graphs.
        state = ("failed" if err else
                 "inferring" if self.inferring else
                 "waiting" if age is None or age > stale_s else
                 "idle")
        kv = [("state", state),
              ("consecutive_failures", str(self.failures)),
              ("seq", str(self.seq)),
              ("rx", str(self.rx)),
              ("dropped", str(self.dropped)),
              ("mono", f"{now:.6f}"),
              ("frame_age_s", "" if age is None else f"{age:.3f}"),
              ("model", str(pathlib.Path(self.get_parameter('model_path').value).name)),
              ("imgsz", str(self.model.imgsz)),
              ("threads", str(self.get_parameter("threads").value))]
        if last:
            kv += [("last_seq", str(last["seq"])),
                   ("last_infer_s", f"{last['infer_s']:.3f}"),
                   ("last_n", str(last["n"])),
                   ("last_labels", last["labels"]),
                   # The field ras_monitor.py calls detect_mono. Same clock, same
                   # name: a consumer joins the two without a conversion.
                   ("detect_mono", f"{last['mono']:.6f}"),
                   ("last_stamp_sim", f"{last['stamp']:.3f}")]
        st.values = [KeyValue(key=k, value=v) for k, v in kv]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag.publish(arr)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
