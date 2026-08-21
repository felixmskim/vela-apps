#!/usr/bin/env python3
"""Expand the shipped robot/world assets into what gz sim can load, then remove
what costs render time and buys nothing.

Supports both robots. The TSP figure names TurtleBot4; TurtleBot3 is kept because
it is far cheaper and isolates "does headless Gazebo work on this host at all"
from "does the TB4 asset set fit in the budget".

    --robot tb3   nav2_minimal_tb3_sim   world tb3_sandbox   SDF robot
    --robot tb4   nav2_minimal_tb4_sim   world depot         URDF robot

Edits applied. Every figure below is read off the shipped asset, cited so it can
be re-checked; none of it is a profile, and G1 is what actually decides.

  tb3  delete link camera_link. It carries intel_realsense_r200_depth, a depth
       sensor at 320x240 and 5 Hz -- 0.38 Mpix/s through the full PBS path
       (gz_waffle.sdf.xacro:378-387). The reason to remove it is not that it is
       the larger cost but that it is pure waste:
       turtlebot3_waffle_bridge.yaml never bridges it, so nothing consumes a
       single pixel of it.

  tb4  delete the oakd links. rgbd_camera renders colour AND depth at 320x240,
       10 Hz -- about 1.54 Mpix/s (oakd.urdf.xacro:124-138) -- and nothing here
       consumes that either. Dropping it from the model is only half the job:
       bridging /rgbd_camera/camera_info is by itself enough to make it render,
       because RgbdCameraSensor::HasConnections() is colour OR depth OR points OR
       *info*. bridge_tb4.yaml omits it.

  tb4  rplidar update_rate 10.0 -> 5.0 (rplidar.urdf.xacro:37-40 sets
       update_rate="10.0" h_samples="360"). Both robots then rasterise the same
       128x128 x 4 cube faces at the same rate, which is what makes their G1
       numbers comparable at all.

  both max_step_size 0.003 -> 0.01. The shipped 333 Hz ODE step is the second
       largest cost after rendering; 100 Hz is stable for a diff-drive base.

Parsing uses xml.etree, not sed. The elements involved are nested and repeated,
and a regex over them truncates the file at the first mismatched tag.

Writes robot.env for the run scripts to source, so world names, spawn modes and
file paths are decided in exactly one place.

    python3 make_world.py --robot tb3 --out-dir /work/artifacts
    python3 make_world.py --robot tb4 --out-dir /work/artifacts
"""

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SHARE = Path('/opt/ros/jazzy/share')

ROBOTS = {
    'tb3': {
        # world name inside the SDF, needed for -world and /world/<name>/stats
        'world_name': 'default',
        'world_src': SHARE / 'nav2_minimal_tb3_sim/worlds/tb3_sandbox.sdf.xacro',
        # Explicit, not defaulted: tb3_sandbox.sdf.xacro:3 already defaults
        # headless to true, and line 14 hides SceneBroadcaster behind
        # <xacro:unless value="$(arg headless)">. Leaving it implicit made a
        # measurement-only choice look like an oversight. --gui flips it.
        'world_args': ['headless:=true'],
        # SDF robot: spawn straight from the file
        'robot_src': SHARE / 'nav2_minimal_tb3_sim/urdf/gz_waffle.sdf.xacro',
        'robot_args': ['namespace:='],
        'robot_out': 'robot.sdf',
        'spawn_mode': 'file',
        # Substring, not an exact name: only the realsense link matches.
        'drop_links': ['camera'],
        'lidar_rate': None,
        'bridge': 'bridge_split.yaml',
        'expand_gz_topics': 'true',
        'map_ref': SHARE / 'nav2_bringup/maps/tb3_sandbox.yaml',
        # Measured, not guessed: three complete tours on 2026-08-17 observed
        # 0.957, 0.970 and 0.973 of tb3_sandbox's 7903 free cells, and the runs
        # that died during bringup sat at 0.135-0.141. Any floor between those
        # two clusters separates them; 0.90 leaves the complete runs ~0.06 of
        # margin.
        'min_coverage': 0.90,
        # Unset: score_map.py's own 0.80 / 0.85 apply, and three tours cleared
        # them by 0.09-0.15.
        'min_precision': None,
        'min_recall': None,
        'spawn': (-2.00, -0.50, 0.01),
        # tb3_sandbox.yaml's origin is in world coordinates, so this is the spawn.
        'map_offset': '-2,-0.5',
        # World coordinates, so they stay comparable to Milestone B, which drove
        # (-1.88, 0.08) -> (1.83, 0.08) through this same arena. send_goal.py
        # converts to the SLAM map frame using --world-offset.
        'waypoints': '1.83,0.08 -1.88,0.08',
        'base_frame': 'base_footprint',
        'scan_frame': 'base_scan',
        # Chase-camera offset for run_demo.sh, in metres behind / beside / above
        # the followed model. A Waffle is 0.28 m across, so the GUI's default
        # camera leaves it about 14 px wide and effectively invisible; this was
        # mistaken for the robot failing to render. Verified to frame it.
        'follow_offset': '-1.0,0,0.45',
    },
    'tb4': {
        'world_name': 'depot',
        'world_src': SHARE / 'nav2_minimal_tb4_sim/worlds/depot.sdf',
        # depot.sdf is a xacro despite the extension. headless drops
        # SceneBroadcaster (depot.sdf:22-27), which no measurement needs -- but a
        # GUI needs it to have any scene to draw at all. --gui flips it.
        'world_args': ['headless:=true'],
        # URDF robot. Upstream's own spawn_tb4.launch.py uses -topic
        # robot_description and leaves the -file path commented out as
        # "# TODO SDF file is unhappy, not sure why", so do not fight it.
        'robot_src': SHARE / 'nav2_minimal_tb4_description/urdf/standard/turtlebot4.urdf.xacro',
        # No args, matching upstream simulation.launch.py:134, which calls
        # Command(['xacro', ' ', robot_sdf]) with none. create3.urdf.xacro:10-13
        # already defaults gazebo="ignition" and namespace="", and the empty
        # namespace is what leaves the topics unprefixed for bridge_tb4.yaml.
        'robot_args': [],
        'robot_out': 'robot.urdf',
        'spawn_mode': 'topic',
        # One substring covers the whole OAK-D subtree: the expansion produces
        # oakd_link, oakd_camera_bracket, oakd_rgb_camera_frame and six more.
        'drop_links': ['oakd'],
        'lidar_rate': '5.0',
        'bridge': 'bridge_tb4.yaml',
        'expand_gz_topics': 'false',
        'map_ref': SHARE / 'nav2_bringup/maps/depot.yaml',
        # 0 = coverage gate off, deliberately. No tb4 closed-loop run exists yet,
        # so there is nothing to set a floor against. A 360-ray visibility cast
        # over depot.pgm along the waypoints below predicts 0.592 of its 179481
        # free cells, but that is an unvalidated upper bound -- ray casting sees
        # through nothing and marks every ray endpoint, which slam_toolbox does
        # not.
        #
        # Measured on two tb4 split runs, 2026-08-19, map_offset corrected to
        # (0, 0) -- read the MAP_OFFSET note below before touching any of this.
        #
        #                       precision  recall  coverage  legacy IoU
        #   run 1, WATCHDOG=      0.792    0.968    0.387       0.505
        #   run 2, WATCHDOG=1     0.679    0.932    0.368       0.516
        #
        # recall and coverage are stable and are gated. precision is NOT, and the
        # reason is a property of the reference rather than of the map: the two runs
        # are indistinguishable by IoU -- 0.505 and 0.516, a spread of 0.011 -- while
        # precision moved 0.11, ten times as far on the same pair of maps. (An earlier
        # version of this note called 0.533 depot's flawless-map ceiling. It is not:
        # score_map.py printed that one hardcoded tb3_sandbox figure for every
        # reference. It no longer claims a ceiling it has not measured, and the
        # 0.011-against-0.11 spread is the argument.)
        # depot.pgm is sparser than the world the lidar sees,
        # so precision = matched / slam_occupied falls whenever the tour observes a
        # region where the reference holds few cells: run 2 marked MORE wall (1222
        # vs 1148) against FEWER reference cells in its co-known region (844 vs
        # 991). That makes precision a function of exploration geometry here, not of
        # map quality, so gating on it would fail runs for driving somewhere else.
        # Printed every run, gated nowhere -- the same treatment the legacy IoU gets.
        #
        # A floor set from run 1 alone would have been 0.75, and run 2 would have
        # failed it. One run is not enough to found a gate on, and that is the
        # lesson worth keeping from this pair.
        'min_coverage': 0.30,
        'min_precision': 0,
        'min_recall': None,
        'spawn': (-8.00, 0.00, 0.01),
        # depot.yaml's origin is already spawn-relative. See the MAP_OFFSET note.
        'map_offset': '0,0',
        # World coordinates. depot.pgm is 604x307 at 0.05 with origin
        # (-7.14, -7.83), so the mapped area is x in [-7.14, 23.06] and y in
        # [-7.83, 7.52]; these four sit comfortably inside it.
        'waypoints': '-6.0,0.0 -4.0,0.0 -4.0,2.0 -6.0,2.0',
        'base_frame': 'base_footprint',
        'scan_frame': 'rplidar_link',
        # Depot is a much larger world and TB4 is a taller robot; pulled back
        # accordingly so the surroundings are visible in the same frame.
        'follow_offset': '-1.6,0,0.8',
    },
}


def xacro(src: Path, dst: Path, args):
    if not src.exists():
        print(f'FAIL: {src} not found -- is the package installed?', file=sys.stderr)
        return False
    subprocess.run(['xacro', str(src), '-o', str(dst)] + list(args), check=True)
    return True


def _hit(name, patterns):
    return bool(name) and any(p in name for p in patterns)


def strip_links(path: Path, patterns):
    """Remove links whose name contains any pattern, plus what references them.

    Matching is by substring, not by exact name, because the TB4 names do not
    exist until xacro expands the macros. Verified against a real expansion:
    <xacro:oakd model="pro" parent_link="oakd_camera_bracket"> does not pass
    `name`, and the macro defaults it to "oakd" (oakd.urdf.xacro:4), so the links
    are oakd_link, oakd_rgb_camera_frame, oakd_left_camera_optical_frame and so
    on -- not the oakd_pro_* an exact list would predict. A literal list silently
    removes nothing and the camera keeps rendering.

    Three kinds of element go: the <link> itself, any <joint> whose parent or
    child is one of them, and any URDF <gazebo reference="..."> block naming one
    -- that last is where the sensor definitions live.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    pats = list(patterns)
    removed = 0
    # Materialise the iterator: removing children while root.iter() is live
    # skips siblings.
    for parent in list(root.iter()):
        for link in list(parent.findall('link')):
            if _hit(link.get('name'), pats):
                parent.remove(link)
                removed += 1
        for joint in list(parent.findall('joint')):
            refs = set()
            for tag in ('parent', 'child'):
                el = joint.find(tag)
                if el is None:
                    continue
                # URDF puts the name in an attribute, SDF in the text.
                refs.add(el.get('link') or (el.text or '').strip())
            if any(_hit(r, pats) for r in refs) or _hit(joint.get('name'), pats):
                parent.remove(joint)
                removed += 1
        for gz in list(parent.findall('gazebo')):
            if _hit(gz.get('reference'), pats):
                parent.remove(gz)
                removed += 1
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return removed


def set_all(path: Path, tag: str, value: str):
    tree = ET.parse(path)
    root = tree.getroot()
    n = 0
    for el in root.iter(tag):
        el.text = value
        n += 1
    if n:
        tree.write(path, encoding='utf-8', xml_declaration=True)
    return n


def set_lidar_rate(path: Path, value: str):
    """Only the gpu_lidar's update_rate, not every sensor's."""
    tree = ET.parse(path)
    root = tree.getroot()
    n = 0
    for sensor in root.iter('sensor'):
        if sensor.get('type') != 'gpu_lidar':
            continue
        el = sensor.find('update_rate')
        if el is not None:
            el.text = value
            n += 1
    if n:
        tree.write(path, encoding='utf-8', xml_declaration=True)
    return n


def add_robot_camera(path: Path, rate: str, width: str, height: str):
    """Attach an RGB camera to base_link, looking where the robot is going.

    This exists because the billboard rig (yolo/tools/make_billboards.py) put
    the camera outside the arena, and a demo built on it showed the detector
    labelling photographs unrelated to the scene -- rejected on review,
    2026-08-20. With the camera on the robot, what the detector labels is what
    the robot is looking at; make_people.py provides the people it can name.

    NOT the shipped realsense that drop_links removes: that is a depth sensor
    (nothing here consumes depth) and it renders whether or not anything
    subscribes. This one is a plain camera at an update_rate the guest can
    actually infer at, and <topic>camera</topic> keeps the gz topic /camera
    regardless of the entity name at spawn -- the same topic the billboard
    scene used, so the bridge fragment and yolo_node need no change at all.

    Pose, verified by rendering before adoption (2026-08-20): 0.25 m above
    base_link, pitched 0.25 rad UP, because the lens is 0.26 m off the ground
    and a person's head is at 1.7 m -- level, it decapitates everyone within
    3 m. person:0.909 at 2 m and 0.879 at 3.5 m through this exact pose.

    The sensor renders only while something subscribes (Gazebo skips cameras
    with no connections), so its cost is zero until the bridge attaches.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    link = None
    for el in root.iter('link'):
        if el.get('name') == 'base_link':
            link = el
            break
    if link is None:
        return False
    sensor = ET.SubElement(link, 'sensor',
                           {'name': 'yolo_camera', 'type': 'camera'})
    ET.SubElement(sensor, 'pose').text = '0.12 0 0.25 0 -0.25 0'
    ET.SubElement(sensor, 'topic').text = 'camera'
    ET.SubElement(sensor, 'update_rate').text = rate
    ET.SubElement(sensor, 'always_on').text = '1'
    cam = ET.SubElement(sensor, 'camera')
    ET.SubElement(cam, 'horizontal_fov').text = '1.047'
    img = ET.SubElement(cam, 'image')
    ET.SubElement(img, 'width').text = width
    ET.SubElement(img, 'height').text = height
    clip = ET.SubElement(cam, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '50'
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return True


def add_pose_publisher(path: Path, hz: str, entity: str):
    """Attach PosePublisher to the robot model so its TRUE pose is observable.

    Diagnosis only, off by default. Gazebo's ground-truth poses normally arrive on
    /world/<name>/dynamic_pose/info, which SceneBroadcaster publishes -- and every
    measurement world here is built headless, which removes SceneBroadcaster. Using
    --gui to get it back would change the very thing under investigation, since the
    broadcast and the GUI both cost RTF. This plugin is per-model and needs no
    SceneBroadcaster.

    Returns the topic it will publish on, or None if the model element is missing.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    model = root.find('model')
    if model is None:
        return None
    plugin = ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-pose-publisher-system',
        'name': 'gz::sim::systems::PosePublisher',
    })
    for tag, value in (('publish_link_pose', 'false'),
                       ('publish_collision_pose', 'false'),
                       ('publish_visual_pose', 'false'),
                       ('publish_sensor_pose', 'false'),
                       ('publish_nested_model_pose', 'false'),
                       ('publish_model_pose', 'true'),
                       # Pose_V rather than a bare Pose: ros_gz_bridge maps
                       # gz.msgs.Pose_V to tf2_msgs/msg/TFMessage, which keeps the
                       # model name in child_frame_id so the probe can find it.
                       ('use_pose_vector_msg', 'true'),
                       ('update_frequency', hz)):
        ET.SubElement(plugin, tag).text = value
    tree.write(path, encoding='utf-8', xml_declaration=True)
    # The topic follows the ENTITY name assigned at spawn time, not the SDF
    # <model name>. The runners spawn with `create -name <robot kind>`, so this SDF's
    # "turtlebot3_waffle" becomes "tb3" and the plugin publishes on /model/tb3/pose.
    # Advertising both names is what made the first attempt bridge a topic that was
    # never written to: /gz_truth existed in ROS and stayed silent.
    return f'/model/{entity}/pose'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', choices=sorted(ROBOTS), default='tb3')
    ap.add_argument('--out-dir', default='/work/artifacts')
    ap.add_argument('--step-size', default='0.01')
    ap.add_argument('--rtf', default='',
                    help='write <real_time_factor> into the world instead of '
                         'throttling at runtime. Calling the set_physics service '
                         'with a partial gz.msgs.Physics stops rigid-body motion '
                         'altogether -- see run_container_loop.sh')
    ap.add_argument('--odom-hz', default='',
                    help='DiffDrive <odom_publish_frequency>. Drives BOTH /odom and '
                         'the odom->base_footprint /tf, so this is the single '
                         'biggest lever on how much traffic the guest has to '
                         'deserialise. Left alone by default so G2 keeps scoring '
                         'the inputs it was validated on')
    ap.add_argument('--keep-camera', action='store_true',
                    help='leave the unbridged camera in place')
    ap.add_argument('--keep-lidar-rate', action='store_true')
    ap.add_argument('--gui', action='store_true',
                    help='build the world with SceneBroadcaster so gz sim -g has '
                         'a scene to draw. A demo run, not a measurement run')
    ap.add_argument('--robot-camera', action='store_true',
                    help='attach an RGB camera to base_link on gz topic /camera, '
                         'for the YOLO co-tenant. See add_robot_camera')
    ap.add_argument('--camera-rate', default='0.25',
                    help='camera update rate in SIM Hz. The default is one frame '
                         'per 4 sim-seconds: the robot moves ~1 m between frames '
                         'at tour speed, and the guest infers a frame in 11-40 s '
                         'of wall clock anyway -- yolo_node keeps only the newest')
    ap.add_argument('--camera-width', default='320')
    ap.add_argument('--camera-height', default='240')
    ap.add_argument('--truth-pose', metavar='HZ', nargs='?', const='20',
                    help='attach PosePublisher to the robot at HZ (default 20) so '
                         'pose_truth_probe.py can compare the SLAM estimate against '
                         'the simulator. Diagnosis only; it adds a publisher the '
                         'measured configuration does not have')
    a = ap.parse_args()

    cfg = ROBOTS[a.robot]
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    world = out / 'world.sdf'
    robot = out / cfg['robot_out']

    # Both shipped worlds gate <gz::sim::systems::SceneBroadcaster> behind
    # <xacro:unless value="$(arg headless)"> and default headless to true. Without
    # that system there is no pose/scene stream, so `gz sim -g` connects and draws
    # an empty world forever -- the failure looks like a broken GUI, not a world
    # built without a scene. Nothing measured here needs it (/scan comes from the
    # sensors system and /tf from DiffDrive's own tf_topic), so it stays off
    # except for a demo.
    world_args = list(cfg['world_args'])
    if a.gui:
        world_args = ['headless:=false' if w.startswith('headless:=') else w
                      for w in world_args]
        if not any(w.startswith('headless:=') for w in world_args):
            world_args.append('headless:=false')

    if not xacro(cfg['world_src'], world, world_args):
        return 1
    if not xacro(cfg['robot_src'], robot, cfg['robot_args']):
        return 1

    n = set_all(world, 'max_step_size', a.step_size)
    print(f'world  : max_step_size -> {a.step_size} ({n} element(s))'
          if n else 'world  : WARNING no <max_step_size> found')

    if a.rtf:
        n = set_all(world, 'real_time_factor', a.rtf)
        if not n:
            print('FAIL: no <real_time_factor> to set', file=sys.stderr)
            return 1
        print(f'world  : real_time_factor -> {a.rtf} ({n} element(s))')

    scene = 'SceneBroadcaster' in world.read_text()
    print(f'world  : args {world_args} -> SceneBroadcaster '
          f'{"ON (demo: gz sim -g can render; costs RTF)" if scene else "off (headless, measurable)"}')

    if a.keep_camera:
        print('robot  : camera kept by request')
    else:
        n = strip_links(robot, cfg['drop_links'])
        print(f'robot  : removed {n} element(s) matching {cfg["drop_links"]}')
        if n == 0:
            print('FAIL: nothing matched -- the camera would still render',
                  file=sys.stderr)
            return 1

    if cfg['lidar_rate'] and not a.keep_lidar_rate:
        n = set_lidar_rate(robot, cfg['lidar_rate'])
        print(f'robot  : gpu_lidar update_rate -> {cfg["lidar_rate"]} ({n} sensor(s))')

    if a.robot_camera:
        if cfg['spawn_mode'] != 'file':
            print(f'FAIL: --robot-camera inserts an SDF <sensor>, and {a.robot} '
                  f'spawns from {cfg["robot_out"]} in {cfg["spawn_mode"]} mode',
                  file=sys.stderr)
            return 1
        if not add_robot_camera(robot, a.camera_rate,
                                a.camera_width, a.camera_height):
            print('FAIL: no base_link to attach the camera to', file=sys.stderr)
            return 1
        print(f'robot  : RGB camera on base_link -> gz topic /camera, '
              f'{a.camera_width}x{a.camera_height} @ {a.camera_rate} Hz sim')

    if a.odom_hz:
        # set_all is safe for this tag specifically: <odom_publish_frequency> exists
        # only inside the DiffDrive plugin. <update_rate> would NOT be safe -- it
        # also appears on the lidar, the IMU and JointStatePublisher.
        n = set_all(robot, 'odom_publish_frequency', a.odom_hz)
        if not n:
            print('FAIL: no <odom_publish_frequency> to set', file=sys.stderr)
            return 1
        # The requested rate is not always the delivered rate: DiffDrive publishes on
        # simulation steps, so the effective frequency is quantised to 1/max_step_size
        # -- with 0.01 s steps, 30 Hz became a measured 25 Hz.
        print(f'robot  : odom_publish_frequency -> {a.odom_hz} '
              f'({n} element(s)); /odom and the odom->{cfg["base_frame"]} /tf both '
              f'follow it, quantised to multiples of 1/{a.step_size}s steps')

    truth_topic = None
    if a.truth_pose:
        if cfg['spawn_mode'] != 'file':
            print(f'FAIL: --truth-pose edits an SDF <model>, and {a.robot} spawns '
                  f'from {cfg["robot_out"]} in {cfg["spawn_mode"]} mode',
                  file=sys.stderr)
            return 1
        truth_topic = add_pose_publisher(robot, a.truth_pose, a.robot)
        if truth_topic is None:
            print('FAIL: no <model> element to attach PosePublisher to',
                  file=sys.stderr)
            return 1
        print(f'robot  : PosePublisher at {a.truth_pose} Hz -> {truth_topic} '
              f'(DIAGNOSIS build, not the measured configuration)')

    # Assert the postconditions instead of trusting the edits.
    text = robot.read_text()
    lidar = text.count('gpu_lidar')
    cam = text.count('rgbd_camera') + text.count('intel_realsense_r200_depth')
    print(f'check  : gpu_lidar={lidar} camera_sensors={cam}')
    if lidar != 1:
        print('FAIL: expected exactly one gpu_lidar', file=sys.stderr)
        return 1
    if not a.keep_camera and cam != 0:
        print('FAIL: a camera sensor survived', file=sys.stderr)
        return 1
    if a.robot_camera and text.count('yolo_camera') != 1:
        print('FAIL: expected exactly one yolo_camera', file=sys.stderr)
        return 1

    x, y, z = cfg['spawn']
    env = out / 'robot.env'
    # Every value is quoted. WAYPOINTS contains spaces, and unquoted
    # `WAYPOINTS=1.83,0.08 -1.88,0.08` does not assign a two-element string: bash
    # reads it as running the command `-1.88,0.08` with WAYPOINTS set in its
    # environment. The command fails, the variable ends up empty, and the only
    # symptom is send_goal.py getting --waypoints '' -- a robot that never moves,
    # with no error anywhere. Quoting the rest costs nothing and stops the next
    # value that grows a space from doing the same.
    env.write_text(
        '# generated by make_world.py -- source this, do not edit\n'
        f'ROBOT_KIND="{a.robot}"\n'
        f'WORLD_NAME="{cfg["world_name"]}"\n'
        f'WORLD_FILE="{world}"\n'
        f'ROBOT_FILE="{robot}"\n'
        f'SPAWN_MODE="{cfg["spawn_mode"]}"\n'
        f'BRIDGE_FILE="/work/config/{cfg["bridge"]}"\n'
        f'EXPAND_GZ_TOPICS="{cfg["expand_gz_topics"]}"\n'
        f'MAP_REF="{cfg["map_ref"]}"\n'
        f'SPAWN_X="{x}"\nSPAWN_Y="{y}"\nSPAWN_Z="{z}"\n'
        # score_map.py --offset converts SLAM-frame coords into the reference
        # map's frame. It is NOT derivable from the spawn pose, and deriving it
        # was a real defect: the two shipped references do not express their
        # origin in the same frame.
        #
        #   tb3_sandbox.yaml  origin is in WORLD coordinates, so the offset is the
        #                     spawn pose itself, (-2, -0.5).
        #   depot.yaml        origin (-7.14, -7.83) is already SPAWN-RELATIVE, and
        #                     it lands within 0.15 m of the SLAM map's own origin
        #                     (-7.13, -7.68). Two grids that nearly coincide in
        #                     their own frames can only do so if they share a
        #                     frame, so the offset is (0, 0).
        #
        # Measured 2026-08-19 on the first tb4 run: at the spawn-derived (-8, 0)
        # precision was 0.421 / recall 0.283 over 44192 comparable cells; at (0, 0)
        # it is 0.792 / 0.968 over 70422. Sweeping dx with dy=0 gives 0.437 at -1,
        # 0.608 at -0.1, 0.792 at 0, 0.636 at +0.1, 0.430 at +1 -- a sharp peak at
        # zero. An earlier revision of this file claimed the opposite, that dx had
        # no peak and the reference was therefore invalid; that conclusion came
        # from sweeping dx only over [-8.5, -7.0], which is 7 m away from the
        # optimum and saw nothing but the tail. Sweep through zero.
        f'MAP_OFFSET="{cfg["map_offset"]}"\n'
        # score_map.py's coverage gate, as a fraction of the reference map's free
        # space. It has to be per-world: tb3_sandbox is 19.8 m2 of free space and
        # a two-waypoint tour observes 96% of it, while depot is 448.7 m2 and the
        # same length of tour cannot.
        f'MIN_COVERAGE="{cfg["min_coverage"]}"\n'
        # Empty means "use score_map.py's default floor"; the run scripts append
        # --min-precision / --min-recall only when these are non-empty. 0 is not
        # the same as empty -- it is an explicit "measured, and not gated".
        f'MIN_PRECISION="{"" if cfg["min_precision"] is None else cfg["min_precision"]}"\n'
        f'MIN_RECALL="{"" if cfg["min_recall"] is None else cfg["min_recall"]}"\n'
        f'WAYPOINTS="{cfg["waypoints"]}"\n'
        f'BASE_FRAME="{cfg["base_frame"]}"\n'
        f'SCAN_FRAME="{cfg["scan_frame"]}"\n'
        f'FOLLOW_OFFSET="{cfg["follow_offset"]}"\n'
        # run_demo.sh reads this: attaching a GUI to a world built without
        # SceneBroadcaster shows an empty scene, so it refuses instead.
        f'SCENE_BROADCASTER="{"1" if scene else "0"}"\n'
        # Empty unless --truth-pose was passed. Non-empty means this build carries
        # an extra publisher and is for diagnosis, not for RUNLOG.md numbers.
        f'TRUTH_TOPIC="{truth_topic or ""}"\n'
    )

    # Assert the file survives being sourced, rather than trusting the quoting.
    # This is the postcondition that would have caught the bug above on the day it
    # was written instead of during the first drive.
    probe = subprocess.run(
        ['bash', '-c', f'set -e; . "{env}"; printf "%s" "$WAYPOINTS"'],
        capture_output=True, text=True)
    if probe.returncode != 0 or probe.stdout != cfg['waypoints']:
        print(f'FAIL: {env} does not survive `source`: rc={probe.returncode} '
              f'WAYPOINTS={probe.stdout!r} err={probe.stderr.strip()!r}',
              file=sys.stderr)
        return 1

    print(f'wrote  : {world}\n         {robot}\n         {env}')

    if a.robot == 'tb4':
        print('note   : depot pulls its geometry from Fuel on first run. Pre-seed with')
        print('         gz fuel download -u '
              '"https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"')
        # ROS 2 node executables live in lib/<pkg>/ and are launched through
        # `ros2 run`, so they are never on PATH: shutil.which() here always
        # failed and printed this note on every tb4 run. Check the real path.
        rsp = SHARE.parent / 'lib' / 'robot_state_publisher' / 'robot_state_publisher'
        if not rsp.exists():
            print(f'note   : {rsp} missing; the topic spawn mode needs it')
    return 0


if __name__ == '__main__':
    sys.exit(main())
