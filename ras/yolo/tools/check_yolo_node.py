#!/usr/bin/env python3
"""Acceptance test for the interface yolo_node.py owes the RAS error-logging
daemon -- frames in, /detection_result and /diagnostics out.

Gate G-Y6. It runs on x86, in seconds, because it replaces onnxruntime with a
stub of the same call signature: the thing under test is the node's ROS
behaviour, not the model. On the guest one pass costs 11-31 s, so asserting this
contract there would mean minutes per assertion and a dependency on the wheel,
the model and the bag all being in place -- which is what G-Y2/G-Y3 are for.

What it asserts, and why each one matters to a consumer:

  /detection_result carries the detections.
  /diagnostics carries exactly one status named `yolo/detector`.
  A status exists BEFORE the first camera frame. A monitor that starts first must
      see the detector exists and is waiting, not nothing at all.
  `detect_mono` is the caller's own CLOCK_MONOTONIC. This is the field that joins
      a detection to a RAS event; ras_monitor.py publishes the same clock under
      the same name, and if either used ROS time the join would be silently wrong
      rather than absent.
  `dropped` grows. The node keeps only the newest frame, so a consumer computing
      a rate from `rx` alone would overstate the work done by an order of
      magnitude at these speeds.
  An inference that RAISES reports ERROR and the node stays up. This is the one
      that decides whether TC.10.007 can tell a failed detector from a killed
      one: a node that exited would leave the same silence as the fault.
  Integer pixels do not abort the process. rosidl's C converter fails a
      PyFloat_Check assertion rather than raising, so this crashes with SIGABRT
      instead of an exception -- an availability test would record it as the
      workload dying for no stated reason.

    python3 check_yolo_node.py

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import os
import pathlib
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

HERE = pathlib.Path(__file__).resolve().parent

# Integer pixels on purpose: see the docstring. A faithful stub would return
# floats and would not test the cast that keeps the node alive.
STUB = '''
import time
class Yolov8Onnx:
    def __init__(self, path, imgsz=320, conf=0.25, iou=0.45, threads=2):
        self.imgsz, self.runtime, self.n = imgsz, "stub", 0
        self.fail_after = int(os.environ.get("STUB_FAIL_AFTER", "0"))
    def __call__(self, bgr):
        time.sleep(1.0)
        self.n += 1
        if self.fail_after and self.n > self.fail_after:
            raise RuntimeError("synthetic inference failure")
        return ([{"xyxy": [10, 20, 60, 120], "conf": 0.83, "cls_id": 0, "cls_name": "person"},
                 {"xyxy": [80, 30, 200, 140], "conf": 0.47, "cls_id": 5, "cls_name": "bus"}], 1.0)
import os
'''


class Driver(Node):
    """The camera the node does not have, and the monitor that will watch it."""

    def __init__(self):
        super().__init__('yolo_node_check')
        cam = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Image, '/camera/image_raw', cam)
        self.diags, self.dets = [], []
        self.create_subscription(DiagnosticArray, '/diagnostics',
                                 lambda m: self.diags.extend(m.status), 10)
        self.create_subscription(Detection2DArray, '/detection_result',
                                 lambda m: self.dets.append(m), 10)

    def frame(self):
        img = np.full((240, 320, 3), 40, np.uint8)
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'camera'
        m.height, m.width, m.encoding, m.step = 240, 320, 'rgb8', 960
        m.data = img.tobytes()
        self.pub.publish(m)


class Report:
    def __init__(self):
        self.failed = 0

    def __call__(self, ok, what, extra=''):
        print(f"      {'PASS' if ok else 'FAIL'}  {what}" + (f": {extra}" if extra else ''))
        if not ok:
            self.failed += 1


def spin(node, seconds, pub_every=0.5):
    """Drive the camera and pump callbacks for `seconds`."""
    end = time.time() + seconds
    nxt = 0.0
    while time.time() < end:
        if pub_every and time.time() >= nxt:
            node.frame()
            nxt = time.time() + pub_every
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    tmp = tempfile.mkdtemp(prefix='yolo-node-check-')
    (pathlib.Path(tmp) / 'yolov8_onnx.py').write_text(STUB)
    node_py = pathlib.Path(tmp) / 'yolo_node.py'
    node_py.write_text((HERE / 'yolo_node.py').read_text())

    env = dict(os.environ, STUB_FAIL_AFTER='3')
    # Isolated domain: this must not join a running rig, and a rig must not
    # satisfy its assertions for it.
    env['ROS_DOMAIN_ID'] = env.get('CHECK_DOMAIN', '97')
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)

    rclpy.init()
    d = Driver()
    r = Report()

    # The subscriber first, then the node: the pre-frame status is VOLATILE, so a
    # subscriber that starts second cannot see it and the assertion below would
    # pass or fail on timing rather than on behaviour.
    print("1/3 the node comes up and says so before any frame arrives")
    log = pathlib.Path(tmp) / 'node.log'
    with open(log, 'w') as fh:
        proc = subprocess.Popen([sys.executable, str(node_py)], env=env,
                                stdout=fh, stderr=subprocess.STDOUT, cwd=tmp)
    try:
        spin(d, 6.0, pub_every=0)          # no frames yet
        pre = [s for s in d.diags if s.name == 'yolo/detector']
        r(bool(pre), "a status arrived before the first frame", f"{len(pre)}")
        if pre:
            r(int.from_bytes(pre[0].level, 'big') == 1,
              "it is WARN, not OK", f"level={int.from_bytes(pre[0].level, 'big')} "
                                    f"msg={pre[0].message}")

        print("2/3 frames in, detections and status out")
        d.diags.clear()
        spin(d, 12.0)
        r(len(d.dets) >= 2, "/detection_result published", f"{len(d.dets)} messages")
        ok_st = [s for s in d.diags if int.from_bytes(s.level, 'big') == 0]
        r(bool(ok_st), "/diagnostics reached level OK", f"{len(ok_st)} of {len(d.diags)}")
        names = {s.name for s in d.diags}
        r(names == {'yolo/detector'}, "one status, named yolo/detector", str(names))
        kv = {v.key: v.value for v in (ok_st or d.diags)[-1].values}
        missing = [k for k in ('state', 'consecutive_failures', 'seq', 'rx',
                               'dropped', 'mono', 'frame_age_s', 'model', 'imgsz',
                               'threads', 'detect_mono', 'last_infer_s', 'last_n',
                               'last_labels', 'last_stamp_sim')
                   if k not in kv]
        r(not missing, "every documented key is present", ','.join(missing) or 'all')
        r(kv.get('last_labels', '').startswith('person:0.83'),
          "the labels ride on the status", kv.get('last_labels'))
        r(int(kv.get('dropped', 0)) > 0,
          "dropped frames are counted, not inferred from rx", kv.get('dropped'))
        ours = time.clock_gettime(time.CLOCK_MONOTONIC)
        drift = ours - float(kv['detect_mono'])
        r(0 <= drift < 30,
          "detect_mono is the caller's own CLOCK_MONOTONIC", f"{drift:.1f}s ago")
        r(len(d.dets) >= 2 and len(d.dets[0].detections) == 2,
          "integer pixels did not abort the process", "2 boxes survived the cast")

        print("3/3 a failing inference is reported, and the node survives it")
        d.diags.clear()
        spin(d, 14.0)                       # STUB_FAIL_AFTER=3 has tripped by now
        err = [s for s in d.diags if int.from_bytes(s.level, 'big') == 2]
        r(bool(err), "the failure is published as ERROR",
          err[0].message if err else 'no ERROR status')
        if err:
            ekv = {v.key: v.value for v in err[-1].values}
            r(ekv.get('state') == 'failed', "state says failed", ekv.get('state'))
            r(int(ekv.get('consecutive_failures', 0)) >= 1,
              "consecutive_failures counted", ekv.get('consecutive_failures'))
        r(proc.poll() is None, "the node is still running")
        r(len([s for s in d.diags if s.name == 'yolo/detector']) >= 2,
          "and still publishing after the failure")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        d.destroy_node()
        rclpy.try_shutdown()

    print(f"      RESULT: {'PASS' if not r.failed else f'FAIL ({r.failed})'}")
    if r.failed:
        print(f"      node log: {log}")
    return 1 if r.failed else 0


if __name__ == '__main__':
    sys.exit(main())
