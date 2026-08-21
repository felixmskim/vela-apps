#!/usr/bin/env python3
"""Publish a directory of images as a camera topic, in a loop.

This is the development-time stand-in for the recorded bag: same topic, same
message type, no rosbag2 involved. It exists so the detector can be brought up
and debugged before the Gazebo camera world and its bag exist, and so a demo
can run from a handful of JPEGs when Gazebo is not.

The default 0.2 Hz is deliberate: the guest's detector needs tens of seconds
per frame (D-13), so anything faster only burns guest CPU deserialising
frames that will be dropped.
"""

import argparse
import pathlib
import sys

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePublisher(Node):

    def __init__(self, image_dir, topic, hz, loop):
        super().__init__("image_publisher")
        self.paths = sorted(p for p in pathlib.Path(image_dir).iterdir()
                            if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"))
        if not self.paths:
            raise SystemExit(f"no images under {image_dir}")
        self.images = []
        for p in self.paths:
            img = cv2.imread(str(p))
            if img is not None:
                self.images.append((p.name, img))
        self.loop = loop
        self.i = 0
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, 1)
        self.timer = self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f"publishing {len(self.images)} images on {topic} at {hz} Hz "
            f"(loop={loop})")

    def tick(self):
        if self.i >= len(self.images):
            if not self.loop:
                self.get_logger().info("done, shutting down")
                raise SystemExit(0)
            self.i = 0
        name, img = self.images[self.i]
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = name
        self.pub.publish(msg)
        self.i += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--topic", default="/camera/image_raw")
    ap.add_argument("--hz", type=float, default=0.2)
    ap.add_argument("--once", action="store_true",
                    help="publish each image once, then exit")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=sys.argv[:1] + ros_args)
    node = ImagePublisher(args.dir, args.topic, args.hz, loop=not args.once)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
