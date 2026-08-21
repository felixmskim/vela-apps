#!/usr/bin/env python3
"""Generate a minimal Gazebo world whose camera sees detectable objects.

The gz-split worlds deliberately carry no camera -- bridging the TB4's RGBD
sensor costs five times the lidar (see gz-split/DESIGN.md) -- and their
geometric primitives are nothing a COCO-trained detector can name anyway. So
the camera gets its own world: a static camera staring at photo billboards.
A photograph on a plane is exactly what YOLO can detect (persons, a bus), it
is deterministic run to run, and it renders for the cost of three textured
quads instead of a simulated scene worth simulating.

Writes camera_world.sdf and camera_bridge.yaml into --out-dir and copies the
billboard images next to them, so the out-dir is self-contained and the SDF's
absolute texture paths stay valid wherever the directory lands (it is
generated inside the vela-gz container, where /work paths exist).

Run inside the container by scripts/make_bag.sh; plain python3, cv2 optional
(only for image aspect ratios).
"""

import argparse
import pathlib
import shutil

PANEL_H = 1.6          # metres; billboard height
CAM_Z = 1.0            # camera and panel-centre height
DIST = 3.0             # camera-to-panel distance


def image_aspect(path):
    try:
        import cv2
        img = cv2.imread(str(path))
        if img is not None:
            h, w = img.shape[:2]
            return w / h
    except ImportError:
        pass
    return 4 / 3


def billboard(i, img_path, y, aspect):
    w = PANEL_H * aspect
    return f"""
    <model name="billboard_{i}">
      <static>true</static>
      <pose>{DIST} {y:.2f} {CAM_Z} 0 0 3.14159</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>0.02 {w:.2f} {PANEL_H}</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal>
              <albedo_map>{img_path}</albedo_map>
              <roughness>1.0</roughness><metalness>0.0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="+", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--rate", type=float, default=1.0,
                    help="camera update rate, Hz -- keep low, the consumer "
                         "needs tens of seconds per frame")
    args = ap.parse_args()

    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    panels = []
    n = len(args.images)
    # Spread the panels across the 60-degree view at DIST metres.
    span = 2 * DIST * 0.577                       # tan(30 deg)
    for i, src in enumerate(args.images):
        dst = out / pathlib.Path(src).name
        if pathlib.Path(src).resolve() != dst.resolve():
            shutil.copy(src, dst)
        y = 0.0 if n == 1 else -span / 2 + span * (i + 0.5) / n
        panels.append(billboard(i, dst.resolve(), y, image_aspect(dst)))

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="yolo_cam">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.7 0.8 0.9 1</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>0.3 0.2 -0.9</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="camera_rig">
      <static>true</static>
      <pose>0 0 {CAM_Z} 0 0 0</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <topic>camera</topic>
          <update_rate>{args.rate}</update_rate>
          <always_on>1</always_on>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image><width>{args.width}</width><height>{args.height}</height></image>
            <clip><near>0.1</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
    {"".join(panels)}
  </world>
</sdf>
"""
    (out / "camera_world.sdf").write_text(sdf)

    (out / "camera_bridge.yaml").write_text(
        "- gz_topic_name: /camera\n"
        "  ros_topic_name: /camera/image_raw\n"
        "  gz_type_name: gz.msgs.Image\n"
        "  ros_type_name: sensor_msgs/msg/Image\n"
        "  direction: GZ_TO_ROS\n")

    print(f"world: {out / 'camera_world.sdf'}")
    print(f"bridge: {out / 'camera_bridge.yaml'}")
    print(f"panels: {n} at {DIST} m, camera {args.width}x{args.height} @ {args.rate} Hz")


if __name__ == "__main__":
    main()
