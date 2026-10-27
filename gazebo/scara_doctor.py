#!/usr/bin/env python3
"""
SCARA doctor - checks a running Gazebo (or MuJoCo-ROS) cell and tells you
exactly what is missing when the conveyor, the pucks or the arm do not move.

    ros2 launch scara_description gazebo.launch.py mode:=command      # terminal 1
    python3 gazebo/scara_doctor.py                                    # terminal 2 (takes ~15 s)

It listens for a few seconds and checks, in the order things depend on each other:
  1. simulation clock          /clock advancing (Gazebo running, ros_gz bridge up)
  2. robot joints               /joint_states
  3. arm controllers            /arm_controller + /gripper_controller action servers
  4. cameras                    /fixed_camera/image and depth_image arriving
  5. vision                     /scara/detections, with the belt position "s" in it
  6. conveyor command           /conveyor/speed published by the sorter (Gazebo: also the gz topic)
  7. pucks                      /scara/world_poses - pucks exist and move along the belt
  8. sorter                     /scara/status and /scara/sorter_state
Every failed check prints the most likely reason and the fix.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import json
import math
import os
import shutil
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scara_scene as S   # noqa: E402

GREEN, RED, YEL, END = '\033[32m', '\033[31m', '\033[33m', '\033[0m'


class Doctor(Node):
    def __init__(self):
        super().__init__('scara_doctor')
        self.n = {k: 0 for k in ('clock', 'js', 'img', 'depth', 'dets', 'belt', 'poses', 'status', 'state')}
        self.clock = []
        self.js = {}
        self.dets = None
        self.belt = []
        self.pucks = {}
        self.first_pucks = {}
        self.status = []
        self.state = {}
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Image, '/fixed_camera/image', lambda m: self.inc('img'), qos_profile_sensor_data)
        self.create_subscription(Image, '/fixed_camera/depth_image', lambda m: self.inc('depth'), qos_profile_sensor_data)
        self.create_subscription(String, '/scara/detections', self.on_dets, 10)
        self.create_subscription(Float64, '/conveyor/speed', self.on_belt, 10)
        self.create_subscription(TFMessage, '/scara/world_poses', self.on_poses, 10)
        self.create_subscription(String, '/scara/status', lambda m: (self.inc('status'), self.status.append(m.data)), 50)
        self.create_subscription(String, '/scara/sorter_state', self.on_state, 10)
        from control_msgs.action import FollowJointTrajectory
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def inc(self, k):
        self.n[k] += 1

    def on_clock(self, m):
        self.inc('clock')
        self.clock.append(m.clock.sec + m.clock.nanosec * 1e-9)

    def on_js(self, m):
        self.inc('js')
        self.js = dict(zip(m.name, m.position))

    def on_dets(self, m):
        self.inc('dets')
        try:
            self.dets = json.loads(m.data)
        except ValueError:
            pass

    def on_belt(self, m):
        self.inc('belt')
        self.belt.append(m.data)

    def on_poses(self, m):
        self.inc('poses')
        for t in m.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_'):
                p = t.transform.translation
                self.pucks[name] = (p.x, p.y, p.z)
                self.first_pucks.setdefault(name, (p.x, p.y, p.z))

    def on_state(self, m):
        self.inc('state')
        try:
            self.state = json.loads(m.data)
        except ValueError:
            pass


def gz_check():
    """Is the TrackController's topic there and does it get the speed?"""
    if not shutil.which('gz'):
        return None, 'gz command not found (MuJoCo-ROS backend or Gazebo not installed) - skipped'
    try:
        topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=10).stdout
    except Exception as e:
        return False, f'gz topic -l failed: {e}'
    t = '/model/conveyor_belt/link/belt_link/track_cmd_vel'
    if t not in topics:
        return False, (f'{t} does not exist in Gazebo: the conveyor_belt model / TrackController plugin is not loaded '
                       '(world file not the one from this package? run tools/build_scene.py and rebuild)')
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', t, '-n', '1'], capture_output=True, text=True,
                             timeout=6).stdout
    except subprocess.TimeoutExpired:
        return False, f'nothing arrives on {t} in Gazebo: the ros_gz bridge does not forward /conveyor/speed'
    out = out.strip().replace(chr(10), ' ')
    return True, (f'Gazebo belt topic receives: {out}' if out else
                  f'{t} exists (the speed is only sent when it changes)')


def main():
    rclpy.init()
    d = Doctor()
    print('SCARA doctor: listening for 12 s ...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 12.0:
        rclpy.spin_once(d, timeout_sec=0.1)
    arm_ok = d.arm.wait_for_server(timeout_sec=2.0)
    grip_ok = d.grip.wait_for_server(timeout_sec=1.0)
    dt = time.time() - t0
    results = []

    def check(name, ok, good, bad):
        results.append(ok)
        mark = f'{GREEN}PASS{END}' if ok else (f'{YEL}----{END}' if ok is None else f'{RED}FAIL{END}')
        print(f'[{mark}] {name:<22} {good if ok else bad}')
        if ok is None:
            results[-1] = True

    sim_dt = (d.clock[-1] - d.clock[0]) if len(d.clock) > 1 else 0.0
    check('simulation clock', sim_dt > 1.0 or (d.n['clock'] == 0 and d.n['js'] > 0),
          f'/clock runs (real-time factor {sim_dt / dt:.2f})' if d.n['clock'] else 'no /clock (MuJoCo-ROS backend uses wall time)',
          'no /clock: Gazebo is paused or not running, or the ros_gz bridge is not started '
          '(launch gazebo.launch.py; in the Gazebo window press PLAY if it is paused)')
    check('robot joints', d.n['js'] > 5, f'/joint_states at {d.n["js"] / dt:.0f} Hz',
          'no /joint_states: joint_state_broadcaster not active - check `ros2 control list_controllers`')
    check('arm controllers', arm_ok and grip_ok, 'arm_controller + gripper_controller ready',
          'controller action servers missing: gz_ros2_control did not load. Check GZ_SIM_SYSTEM_PLUGIN_PATH contains '
          '~/gz_ros2_control_ws/install/gz_ros2_control/lib and `ros2 control list_controllers` shows them active')
    cell = d.n['dets'] > 0 or d.n['img'] > 0 or d.n['belt'] > 0
    if cell:
        cam_ok = d.n['img'] > 3 and d.n['depth'] > 3
        if not cam_ok and d.n['clock'] == 0 and d.n['dets'] > 3:
            cam_ok = None                         # MuJoCo-ROS backend: detections come from the physics
        check('fixed camera', cam_ok,
              f'image {d.n["img"] / dt:.1f} Hz, depth {d.n["depth"] / dt:.1f} Hz',
              'no camera topics (MuJoCo-ROS backend finds the pucks itself)' if cam_ok is None else
              'no camera images: the Sensors system / ogre2 rendering is not working (GPU / drivers?) or the bridge '
              'is not running - `ros2 topic hz /fixed_camera/image`')
        has_s = bool(d.dets) and all('s' in x for x in d.dets)
        check('vision', d.n['dets'] > 3 and (not d.dets or has_s),
              f'/scara/detections at {d.n["dets"] / dt:.1f} Hz, {len(d.dets or [])} pucks seen now',
              'no detections (vision node not running or no images)' if d.n['dets'] <= 3 else
              'detections lack the belt position "s" - old scara_vision_node.py: update the package and rebuild')
        speeds = [abs(v) for v in d.belt]
        # the sorter stops the belt on purpose while it picks: that is not a fault
        picking = bool(d.state.get('running')) and abs(float(d.state.get('belt', 1.0) or 0.0)) < 1e-6
        belt_ok = bool(speeds) and max(speeds) > 0.01
        if not belt_ok and picking:
            belt_ok = None
            print(f'[{YEL}----{END}] conveyor command       belt stopped because the robot is picking right now '
                  '(normal) - run the doctor again in a few seconds to see it run')
            results.append(True)
        if belt_ok is not None:
            check('conveyor command', belt_ok,
                  f'/conveyor/speed = {d.belt[-1]:+.3f} m/s',
                  'the sorter does not command the belt (sorter not running, stopped with "stop", or waiting '
                  'for /clock) - send: ros2 topic pub --once /scara/pick_color std_msgs/msg/String "{data: start}"')
        ok, msg = gz_check()
        check('Gazebo belt topic', ok, msg, msg)
        moved = []
        for n, p in d.pucks.items():
            q = d.first_pucks.get(n)
            if (q and abs(p[2] - S.BELT_TOP_Z) < 0.03 and abs(q[2] - S.BELT_TOP_Z) < 0.03
                    and not S.is_hidden(p[0], p[1]) and not S.is_hidden(q[0], q[1])):
                dd = math.hypot(p[0] - q[0], p[1] - q[1])
                if dd < 1.3:
                    moved.append(dd)
        check('pucks on the belt', len(d.pucks) > 0, f'{len(d.pucks)} pucks in the world',
              'no pucks: the spawner could not create them (look for "Could not spawn" in the launch terminal)')
        if moved and picking and max(moved) <= 0.05:
            print(f'[{YEL}----{END}] belt moves pucks       not checked - the belt is stopped for a pick')
        elif moved:
            check('belt moves pucks', max(moved) > 0.05, f'pucks travelled up to {max(moved) * 100:.0f} cm',
                  'pucks lie on the belt but do not move: TrackController gets no speed (see "Gazebo belt topic") '
                  'or the robot is picking right now - run the doctor again')
        check('sorter', d.n['state'] > 0, f'sorter: {d.state} ; last status: {(d.status or ["-"])[-1]}',
              'no /scara/sorter_state: scara_color_sorter.py is not running (mode:=manual has no sorter)')
    else:
        print(f'[{YEL}----{END}] cell checks            skipped: no conveyor cell (mode:=manual?)')
    print()
    print('All good.' if all(r is not False for r in results) else
          'Fix the first FAIL above - the later checks usually depend on it.')
    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
