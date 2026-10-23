#!/usr/bin/env python3
"""
Keyboard control of the Gazebo SCARA (manual table world).

    ros2 launch scara_description gazebo.launch.py mode:=manual     # terminal 1
    python3 gazebo/scara_teleop.py                                  # terminal 2
    python3 gazebo/scara_teleop.py --safety normal                  # faster

Every key changes a joint target; the targets go through the safety filter
(common/scara_safety.py) and are streamed to the controllers 20 times a second,
so the arm always moves smoothly at the safety mode's speed.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import os
import select
import sys
import termios
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scara_scene as S     # noqa: E402
import scara_safety as SF   # noqa: E402

HELP_MSG = """
==================================================
        SCARA Robot Teleoperation (Gazebo)
==================================================
The robot faces you, so "left" means YOUR left.

  [a] / [d] : column - swing the arm to your left / right
  [w] / [s] : Z axis up / down
  [j] / [l] : forearm to your left / right
  [i] / [k] : wrist turn
  [o]       : close gripper (grips the puck: it fits between the open fingers)
  [u]       : open gripper
  [h]       : home (all joints 0, gripper open)

  [SPACE]   : EMERGENCY STOP on / off
  [m]       : next safety mode (safe -> normal -> off)
  [q] / Ctrl+C : quit

Safe / normal mode: when you lower the open gripper close to a puck it centres
itself over it, and a finger never presses down on a puck.
==================================================
"""

STEP = {'a': (0, -0.05), 'd': (0, 0.05), 'w': (1, 0.005), 's': (1, -0.005),
        'j': (2, -0.05), 'l': (2, 0.05), 'i': (3, 0.1), 'k': (3, -0.1)}


class ScaraTeleopNode(Node):
    def __init__(self, safety='safe', assist=True):
        super().__init__('scara_teleop')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(TFMessage, '/scara/world_poses', self.on_poses, 10)
        self.js = {}
        self.pucks = {}
        self.target = None                 # [col, z, fore, wrist] - starts from the robot's pose
        self.grip = S.GRIPPER_OPEN
        self.filt = SF.SafetyFilter(safety, assist=assist)
        self.last = time.time()
        self.last_grip = None
        self.last_note = ''
        self.get_logger().info('SCARA keyboard teleop started - waiting for /joint_states')

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def on_poses(self, msg):
        for t in msg.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_'):
                p = t.transform.translation
                self.pucks[name] = (p.x, p.y, p.z)

    def ready(self):
        if self.target is None and all(n in self.js for n in S.ARM_JOINTS):
            self.target = [self.js[n] for n in S.ARM_JOINTS]
            self.grip = self.js.get('left_finger_joint', S.GRIPPER_OPEN)
            self.get_logger().info('Robot found - use the keys.')
        return self.target is not None

    def key(self, k):
        if k in STEP:
            i, d = STEP[k]
            lo, hi = S.LIMITS[S.ARM_JOINTS[i]]
            self.target[i] = min(max(self.target[i] + d, lo), hi)
        elif k == 'o':
            self.grip = S.GRIP_CLOSE
        elif k == 'u':
            self.grip = S.GRIPPER_OPEN
        elif k == 'h':
            self.target = [0.0, 0.0, 0.0, 0.0]
            self.grip = S.GRIPPER_OPEN
        elif k == ' ':
            self.filt.estop = not self.filt.estop
            self.get_logger().warn('EMERGENCY STOP' if self.filt.estop else 'stop released')
        elif k == 'm':
            self.filt.set_mode(SF.next_mode(self.filt.mode))
            self.get_logger().info(f'safety mode: {self.filt.mode}')

    def tick(self):
        now = time.time()
        dt, self.last = now - self.last, now
        qm = [self.js[n] for n in S.ARM_JOINTS]
        q, g = self.filt.filter(self.target, self.grip, dt, q_meas=qm, grip_meas=self.js.get('left_finger_joint'),
                                pucks=list(self.pucks.values()))
        self.target = list(q) if self.filt.estop else self.filt.clamp_back(self.target)
        if self.filt.note and self.filt.note != self.last_note:
            self.get_logger().info(self.filt.note)
        self.last_note = self.filt.note
        self.publish(self.arm_pub, S.ARM_JOINTS, q)
        if self.last_grip is None or abs(g - self.last_grip) > 1e-4:
            self.publish(self.gripper_pub, S.GRIPPER_JOINTS, [g, g])     # both fingers
            self.last_grip = g

    @staticmethod
    def publish(pub, names, positions):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points = [point]
        pub.publish(msg)


def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    ap = argparse.ArgumentParser(description='SCARA keyboard control (Gazebo)')
    SF.add_args(ap)
    args, ros_args = ap.parse_known_args()
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=ros_args)
    node = ScaraTeleopNode(args.safety, not args.no_assist)
    print(HELP_MSG)
    try:
        while rclpy.ok():
            key = get_key(settings).lower()
            for _ in range(20):                       # drain queued /joint_states etc.
                rclpy.spin_once(node, timeout_sec=0.0)
            if key in ('q', '\x03'):
                break
            if not node.ready():
                continue
            if key:
                node.key(key)
            node.tick()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
