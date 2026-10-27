#!/usr/bin/env python3
"""
MuJoCo as a stand-in for Gazebo, for every ROS 2 tool of this project.

It offers exactly the ROS interface the Gazebo launch file gives, so the same
ROS nodes run unchanged on MuJoCo physics:

  out  /joint_states                              50 Hz, all 6 joints
  in   /arm_controller/joint_trajectory           like joint_trajectory_controller
  in   /gripper_controller/joint_trajectory
  act  /arm_controller/follow_joint_trajectory    (control_msgs/FollowJointTrajectory)
  act  /gripper_controller/follow_joint_trajectory
  in   /conveyor/speed                            std_msgs/Float64, belt surface speed (m/s)
  out  /scara/world_poses                         tf2_msgs/TFMessage, puck poses (like Gazebo's dynamic_pose)
  out  /scara/detections                          JSON pucks on the belt (like scara_vision_node.py)

    python3 mujoco/scara_mujoco_ros.py --world command          # then e.g.
    ros2 run scara_description scara_color_sorter.py --ros-args -p mode:=command
    python3 gazebo/scara_ask.py

    python3 mujoco/scara_mujoco_ros.py --world manual --viewer  # + scara_joystick.py / scara_teleop.py /
                                                                 #   gesture_control_ros2.py / scara_twin_server.py --sim gazebo

Worlds: sort (4 bins) | command (1 bin) | manual (table + pucks).  In sort /
command worlds the endless puck feeder runs (pucks come out of a hidden pool).
Detections: --detections vision renders the fixed depth camera and runs
scara_vision (needs OpenCV), --detections truth (default) reads the puck
positions straight from the physics.
Use it when Gazebo is not available, or to check the ROS side quickly.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import json
import math
import os
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_scene as S                       # noqa: E402
from scara_feeder import PuckFeeder, classify  # noqa: E402

XML = {'sort': 'scara_scene_mujoco.xml', 'command': 'scara_command_mujoco.xml', 'manual': 'scara_manual_mujoco.xml'}
ACT = {'column_joint': 'act_column', 'shoulder_joint': 'act_shoulder', 'forearm_joint': 'act_forearm',
       'wrist_joint': 'act_wrist', 'left_finger_joint': 'act_left_finger', 'right_finger_joint': 'act_right_finger'}


def _dur(d):
    return d.sec + d.nanosec * 1e-9


class Segment:
    """A trajectory being followed: linear between points, like a JTC with position interfaces."""

    def __init__(self, names, start, points, t0):
        self.names, self.t0 = list(names), t0
        self.times = [0.0] + [max(1e-3, t) for t, _ in points]
        self.pos = [list(start)] + [list(p) for _, p in points]

    def sample(self, t):
        dt = t - self.t0
        if dt >= self.times[-1]:
            return self.pos[-1], True
        for i in range(1, len(self.times)):
            if dt < self.times[i]:
                a = (dt - self.times[i - 1]) / max(1e-6, self.times[i] - self.times[i - 1])
                return [p0 + (p1 - p0) * a for p0, p1 in zip(self.pos[i - 1], self.pos[i])], False
        return self.pos[-1], True


class MujocoRos(Node):
    def __init__(self, args):
        super().__init__('scara_mujoco_sim')
        self.world = args.world
        self.model = mujoco.MjModel.from_xml_path(S.pkg_file('mujoco', 'models', XML[args.world]))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.act = {mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.model.nu)}
        self.lock = threading.Lock()
        self.cmd = {n: 0.0 for n in ACT}
        self.seg = {'arm': None, 'grip': None}
        self.belt = 0.0
        self.t = time.time()
        self.q = {n: 0.0 for n in ACT}
        self.pucks = {}
        self.detections_mode = args.detections
        names = [n for n in S.puck_names() if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n) >= 0]
        self.feeder = (PuckFeeder(self._get_pos, self._set_pos, names=names, seed=args.seed, max_spawn=args.pucks,
                                  log=lambda m: self.get_logger().info(m), mode=args.world)
                       if args.world in ('sort', 'command') else None)
        if self.detections_mode == 'vision':
            import cv2
            import scara_vision as V
            self.cv2, self.V = cv2, V
            self.renderer = mujoco.Renderer(self.model, height=S.FIXED_CAM_HEIGHT, width=S.FIXED_CAM_WIDTH)
            self.cam = V.fixed_camera_model()
        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_pub = self.create_publisher(TFMessage, '/scara/world_poses', 10)
        self.det_pub = self.create_publisher(String, '/scara/detections', 10)
        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory',
                                 lambda m: self.on_traj('arm', m), 10, callback_group=cb)
        self.create_subscription(JointTrajectory, '/gripper_controller/joint_trajectory',
                                 lambda m: self.on_traj('grip', m), 10, callback_group=cb)
        self.create_subscription(Float64, '/conveyor/speed', self.on_belt, 10, callback_group=cb)
        for key, ctrl in (('arm', 'arm_controller'), ('grip', 'gripper_controller')):
            ActionServer(self, FollowJointTrajectory, f'/{ctrl}/follow_joint_trajectory',
                         execute_callback=lambda gh, k=key: self.execute(k, gh),
                         goal_callback=lambda g: GoalResponse.ACCEPT,
                         cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info(f'MuJoCo {args.world} world ready - Gazebo-style ROS interface is up')

    # ------------------------------------------------------------ pucks
    def _get_pos(self, name):
        return tuple(float(v) for v in self.data.joint(f'{name}_free').qpos[:3])

    def _set_pos(self, name, xyz, yaw=0.0):
        j = self.data.joint(f'{name}_free')
        j.qpos[:] = [xyz[0], xyz[1], xyz[2], math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        j.qvel[:] = 0

    # ------------------------------------------------------------ controllers
    def _start(self, key, names, points):
        """points: [(t_from_start, positions in `names` order)] -> controller segment."""
        group = S.ARM_JOINTS if key == 'arm' else S.GRIPPER_JOINTS
        with self.lock:
            start = [self.cmd[n] for n in group]
            full = []
            for t, pos in points:
                d = dict(zip(names, pos))
                if key == 'grip' and 'left_finger_joint' in d and 'right_finger_joint' not in d:
                    d['right_finger_joint'] = d['left_finger_joint']
                full.append((t, [d.get(n, start[i]) for i, n in enumerate(group)]))
            self.seg[key] = Segment(group, start, full, time.time())
            return self.seg[key]

    def on_traj(self, key, msg):
        if msg.points:
            self._start(key, msg.joint_names, [(_dur(p.time_from_start), list(p.positions)) for p in msg.points])

    def execute(self, key, gh):
        tr = gh.request.trajectory
        seg = self._start(key, tr.joint_names, [(_dur(p.time_from_start), list(p.positions)) for p in tr.points])
        fb = FollowJointTrajectory.Feedback()
        while True:
            if gh.is_cancel_requested:
                gh.canceled()
                return FollowJointTrajectory.Result()
            with self.lock:
                if self.seg[key] is not seg:            # replaced by a newer command
                    break
            _, done = seg.sample(time.time())
            if done:
                break
            gh.publish_feedback(fb)
            time.sleep(0.02)
        time.sleep(0.1)
        gh.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = 0
        return res

    def on_belt(self, msg):
        self.belt = float(msg.data)

    # ------------------------------------------------------------ physics loop
    def step(self, viewer=None):
        now = time.time()
        with self.lock:
            for key in ('arm', 'grip'):
                seg = self.seg[key]
                if seg:
                    pos, _ = seg.sample(now)
                    self.cmd.update(dict(zip(seg.names, pos)))
            for n, a in ACT.items():
                self.data.ctrl[self.act[a]] = self.cmd[n]
            if 'act_belt' in self.act:
                self.data.ctrl[self.act['act_belt']] = self.belt
        n = 0
        while self.data.time < self.sim_t0 + (now - self.wall0) and n < 50:
            mujoco.mj_step(self.model, self.data)
            n += 1
        if n == 50:
            self.wall0, self.sim_t0 = now, self.data.time
        if self.world != 'manual':                  # endless belt: slide the plate back
            q = self.data.joint('belt_joint').qpos
            if q[0] > S.BELT_WRAP:
                q[0] -= S.BELT_WRAP
        if self.feeder and self.data.time >= self.next_feed:
            self.next_feed = self.data.time + 0.25
            self.feeder.update(self.data.time)
        self.q = {n_: float(self.data.joint(n_).qpos[0]) for n_ in ACT}
        if viewer is not None:
            viewer.sync()

    def publish(self):
        now = time.time()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(ACT)
        js.position = [self.q[n] for n in ACT]
        self.js_pub.publish(js)
        if now - self.last_pose >= 0.1:
            self.last_pose = now
            tf = TFMessage()
            pucks = {}
            for n in (self.feeder.names if self.feeder else [p for p, _ in S.MANUAL_PUCKS]):
                try:
                    x, y, z = self._get_pos(n)
                except Exception:
                    continue
                pucks[n] = (x, y, z)
                t = TransformStamped()
                t.header.frame_id = 'scara_world'
                t.child_frame_id = n
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
                t.transform.rotation.w = 1.0
                tf.transforms.append(t)
            self.pucks = pucks
            self.pose_pub.publish(tf)
            if self.world != 'manual':
                self.det_pub.publish(String(data=json.dumps(self.detections())))

    def detections(self):
        if self.detections_mode == 'vision':
            r = self.renderer
            r.update_scene(self.data, camera='fixed_camera')
            rgb = r.render()
            r.enable_depth_rendering()
            r.update_scene(self.data, camera='fixed_camera')
            depth = r.render()
            r.disable_depth_rendering()
            dets = self.V.detect_pucks(self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR), depth, self.cam)
            return [{k: d[k] for k in ('color', 'x', 'y', 'z', 's', 'yaw', 'u', 'v', 'depth', 'area')} for d in dets]
        out = []
        for n, (x, y, z) in self.pucks.items():
            if classify(x, y, z, self.world) != 'belt':
                continue
            s = S.belt_s(x, y)
            if not (S.CONVEYOR_Y_RANGE[0] < s < S.CONVEYOR_Y_RANGE[1]):
                continue
            out.append({'color': S.puck_color(n), 'x': x, 'y': y, 'z': S.BELT_TOP_Z + S.PUCK_HEIGHT, 's': s,
                        'yaw': None, 'u': 0.0, 'v': 0.0, 'depth': 0.0, 'area': 1000})
        out.sort(key=lambda d: -d['s'])
        return out

    def run(self, use_viewer):
        self.wall0, self.sim_t0 = time.time(), self.data.time
        self.next_feed = 0.0
        self.last_pose = 0.0
        last_pub = 0.0
        viewer_ctx = None
        if use_viewer:
            import mujoco.viewer
            viewer_ctx = mujoco.viewer.launch_passive(self.model, self.data)
        viewer = viewer_ctx.__enter__() if viewer_ctx else None
        try:
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                t = time.time()
                self.step(viewer)
                if t - last_pub >= 0.02:
                    last_pub = t
                    self.publish()
                time.sleep(max(0.0, 0.004 - (time.time() - t)))
        finally:
            if viewer_ctx:
                viewer_ctx.__exit__(None, None, None)


def main():
    ap = argparse.ArgumentParser(description='MuJoCo with the Gazebo ROS interface')
    ap.add_argument('--world', default='manual', choices=list(XML))
    ap.add_argument('--viewer', action='store_true', help='open the MuJoCo viewer')
    ap.add_argument('--no-viewer', dest='viewer', action='store_false')
    ap.add_argument('--detections', default='truth', choices=['truth', 'vision'])
    ap.add_argument('--pucks', type=int, default=0, help='sort / command: pucks to drop (0 = endless)')
    ap.add_argument('--seed', type=int, default=None)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = MujocoRos(args)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run(args.viewer)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
