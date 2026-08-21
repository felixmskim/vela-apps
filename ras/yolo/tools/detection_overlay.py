#!/usr/bin/env python3
"""Draw the guest's detections onto the host's camera frames, for a viewer.

RViz has no display for `vision_msgs/Detection2DArray`, so a run where YOLO is
working looks, on screen, exactly like a run where it is not. This subscribes to
both the image and the detections and republishes an annotated image that any
Image display can show.

    /camera/image_raw ─┐                          ┌─► /detection_image
                       ├─► detection_overlay ─────┤
    /detection_result ─┘                          └─► a window, with --show

`--show` opens an OpenCV window on `$DISPLAY` carrying the same annotated frame.
That is how a recording shows the detector, and it is a window rather than RViz's
Image display because RViz gives its only dock the whole height of the pane: an
864 px pane spent ~320x1000 of itself on a 320x240 image and the map lost 300 px
to empty grey (measured 2026-08-19, `gz-split/artifacts/rviz-yolo-smoke.png`).
A window is placed by the same `xdotool` call that tiles the other two, costs the
map nothing, and -- because it lives in this process -- needs no second Fast DDS
participant and therefore no second listening port.

**Runs on the host, never in the guest.** The guest is a four-vCPU emulated
machine already carrying Nav2, slam_toolbox and the detector; JPEG-drawing there
would take CPU from the workload the demonstration exists to show. The host has
the frames already -- Gazebo produced them -- so the guest pays nothing for this.

Frames are matched to detections by timestamp rather than "most recent wins".
One inference takes 11.4 s here, so the newest frame when a detection arrives is
not the frame it describes, and drawing on it would put boxes in the wrong place
and quietly misrepresent the detector.

    ros2 run ... python3 detection_overlay.py

SPDX-License-Identifier: Apache-2.0
Copyright (C) 2026 Chungbuk National University, Computer Systems Lab
"""

import argparse
import sys
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

try:
    import cv2
except ImportError:                      # labels need a text renderer; boxes do not
    cv2 = None

PALETTE = [(255, 96, 96), (96, 200, 255), (144, 238, 144), (255, 210, 96),
           (220, 140, 255), (255, 160, 90)]


def to_array(msg):
    """sensor_msgs/Image -> HxWx3 uint8 RGB. Only the encodings Gazebo emits."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == 'bgr8' else img.copy()
    if msg.encoding == 'mono8':
        return np.repeat(buf.reshape(msg.height, msg.width, 1), 3, axis=2)
    raise ValueError(f'unsupported encoding {msg.encoding}')


def draw_box(img, x1, y1, x2, y2, colour, thickness=2):
    h, w = img.shape[:2]
    x1, x2 = max(0, min(x1, w - 1)), max(0, min(x2, w - 1))
    y1, y2 = max(0, min(y1, h - 1)), max(0, min(y2, h - 1))
    t = thickness
    img[y1:y1 + t, x1:x2] = colour
    img[max(y1, y2 - t):y2, x1:x2] = colour
    img[y1:y2, x1:x1 + t] = colour
    img[y1:y2, max(x1, x2 - t):x2] = colour


class Overlay(Node):

    def __init__(self, args):
        super().__init__('detection_overlay')
        self.args = args
        self.frames = OrderedDict()      # stamp_ns -> ndarray
        self.published = 0
        self.unmatched = 0

        # The image crosses no link here (Gazebo and this node are both on the
        # host) but the detections come from the guest over TCP, so both are
        # RELIABLE with room to buffer.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, args.image_topic, self.on_image, qos)
        self.create_subscription(Detection2DArray, args.det_topic, self.on_det, qos)
        self.pub = self.create_publisher(Image, args.out_topic, qos)
        self.get_logger().info(
            f'{args.image_topic} + {args.det_topic} -> {args.out_topic}'
            + ('' if cv2 else '  (no cv2: boxes without labels)'))

        # The window is created NOW, holding a placeholder, rather than when the
        # first detection arrives. Two reasons: the runner tiles windows by name
        # and cannot place one that does not exist yet, and the first detection is
        # 80-120 s after bringup -- an audience that sees the panel waiting knows
        # the detector is starting, where a panel that is simply absent looks like
        # a rig with no detector in it.
        self.window = None
        if args.show:
            if cv2 is None:
                self.get_logger().error('--show needs cv2; continuing without a window')
            else:
                self.window = args.window
                # WINDOW_GUI_NORMAL, not the default GUI_EXPANDED: cv2's Qt build
                # otherwise puts a zoom toolbar and a status bar on the window.
                # Measured -- they took ~28 px off a 640x480 pane and, by changing
                # its aspect ratio, letterboxed the image inside what was left.
                # WINDOW_NORMAL keeps it resizable so xdotool can place it.
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
                self.shown = self.placeholder()
                # 10 Hz, and it is not for the image -- one arrives every 40 s of
                # wall clock at RTF 0.25. It is cv2.waitKey: without it being
                # pumped the Qt window never repaints, and an X window that never
                # repaints records as a grey rectangle.
                self.create_timer(0.1, self.pump)

    def placeholder(self):
        img = np.full((240, 320, 3), 32, np.uint8)
        cv2.putText(img, 'waiting for', (10, 110), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, self.args.det_topic, (10, 138), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (140, 200, 255), 1, cv2.LINE_AA)
        return img

    def pump(self):
        """Repaint the window. See the timer comment in __init__."""
        img = self.shown
        if self.args.scale != 1:
            img = cv2.resize(img, None, fx=self.args.scale, fy=self.args.scale,
                             interpolation=cv2.INTER_NEAREST)
        cv2.imshow(self.window, img[:, :, ::-1])   # our frames are RGB, cv2 wants BGR
        cv2.waitKey(1)

    @staticmethod
    def key(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def on_image(self, msg):
        try:
            self.frames[self.key(msg.header.stamp)] = (to_array(msg), msg.header)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        while len(self.frames) > self.args.history:
            self.frames.popitem(last=False)

    def on_det(self, msg):
        k = self.key(msg.header.stamp)
        entry = self.frames.get(k)
        stale = entry is None
        if stale:
            if not self.frames:
                return
            self.unmatched += 1
            entry = next(reversed(self.frames.values()))
        img, header = entry[0].copy(), entry[1]

        labels = []
        for i, det in enumerate(msg.detections):
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y
            x1, y1 = int(round(cx - w / 2)), int(round(cy - h / 2))
            x2, y2 = int(round(cx + w / 2)), int(round(cy + h / 2))
            colour = PALETTE[i % len(PALETTE)]
            draw_box(img, x1, y1, x2, y2, colour)
            name, score = '', 0.0
            if det.results:
                name = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score
            labels.append(f'{name}:{score:.2f}')
            if cv2 is not None and name:
                cv2.putText(img, f'{name} {score:.2f}', (x1, max(12, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

        if cv2 is not None:
            note = f'{len(msg.detections)} detection(s)' + ('  [stale frame]' if stale else '')
            cv2.putText(img, note, (6, img.shape[0] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        out = Image()
        out.header = header
        out.height, out.width = img.shape[0], img.shape[1]
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = img.shape[1] * 3
        out.data = img.tobytes()
        self.pub.publish(out)
        if self.window is not None:
            self.shown = img
        self.published += 1
        self.get_logger().info(
            f'#{self.published} {len(msg.detections)} box(es)'
            + (f' {", ".join(labels)}' if labels else '')
            + ('  [no matching frame, drew on the newest]' if stale else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--det-topic', default='/detection_result')
    ap.add_argument('--out-topic', default='/detection_image')
    ap.add_argument('--history', type=int, default=30,
                    help='frames kept for timestamp matching. One inference is '
                         '11.4 s on the guest, so this has to outlast it')
    ap.add_argument('--show', action='store_true',
                    help='also draw into a window on $DISPLAY, for a recording')
    ap.add_argument('--window', default='YOLO detections',
                    help='the window title, which is what xdotool matches on')
    ap.add_argument('--scale', type=float, default=2.0,
                    help='window magnification. The camera is 320x240 so that it '
                         'costs the guest little; a recording wants it readable')
    args, _ = ap.parse_known_args([a for a in sys.argv[1:] if a != '--ros-args'])

    rclpy.init(args=None)
    node = Overlay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException as well as KeyboardInterrupt: run_demo.sh stops
        # this node with SIGINT during teardown, and rclpy raises it out of spin()
        # rather than returning. Unhandled it prints a traceback and exits non-zero,
        # so a normal shutdown left a log that reads like a crash -- which is the
        # first thing anyone looks at when a recording has no boxes in it.
        pass
    finally:
        if node.window is not None and cv2 is not None:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
