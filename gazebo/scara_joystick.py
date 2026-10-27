#!/usr/bin/env python3
"""
Drive the Gazebo SCARA with an Xbox (or any) gamepad.

    ros2 launch scara_description gazebo.launch.py mode:=manual     # terminal 1
    python3 gazebo/scara_joystick.py                                 # terminal 2

The pad is read straight from /dev/input/js0 - you do NOT need joy_node.
If you prefer joy_node, run `ros2 run joy joy_node` and add --joy-topic:
the app then uses /joy instead.  Options:
    --device /dev/input/js1     another pad
    --layout gamecontroller     pads whose right stick is on axes 2/3
    --safety safe|normal|off    safety mode (default safe: slow, pick assist, guards - common/scara_safety.py)
    --no-assist                 no automatic centring over a puck

Controls (the robot faces you):
  left stick  X : swing the arm left / right        Y : Z up / down
  right stick X : forearm left / right              Y : turn the wrist
  A : close gripper    B : open gripper    Y : home    hold LB : slow   hold RB : fast
  BACK : emergency stop on / off            START : next safety mode
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scara_scene as S                                  # noqa: E402
from scara_joy_core import JoyMapper, JsDevice          # noqa: E402
import scara_safety as SF                               # noqa: E402


class ScaraJoystick(Node):
    def __init__(self, args):
        super().__init__('scara_joystick')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(TFMessage, '/scara/world_poses', self.on_poses, 10)
        self.js = {}
        self.pucks = {}
        self.filt = SF.SafetyFilter(args.safety, assist=not args.no_assist)
        self.last_note = ''
        self.joy = None
        self.dev = None
        if args.joy_topic:
            self.create_subscription(Joy, '/joy', self.on_joy, 10)
            self.get_logger().info('Reading the pad from /joy (ros2 run joy joy_node)')
        else:
            try:
                self.dev = JsDevice(args.device)
                self.get_logger().info(f'Pad: {self.dev.name} on {self.dev.path}')
            except (FileNotFoundError, OSError) as e:
                self.get_logger().warn(f'{e} - falling back to /joy (run: ros2 run joy joy_node)')
                self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.mapper = None
        self.layout = args.layout
        self.last = time.time()
        self.last_grip = None
        self.warned = False
        self.t0 = time.time()
        self.create_timer(0.05, self.tick)      # 20 Hz
        self.get_logger().info(__doc__.split('Controls')[1].strip())

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def on_poses(self, msg):
        for t in msg.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_'):
                p = t.transform.translation
                self.pucks[name] = (p.x, p.y, p.z)

    def on_joy(self, msg):
        self.joy = (list(msg.axes), list(msg.buttons))

    def tick(self):
        now = time.time()
        dt, self.last = now - self.last, now
        if self.mapper is None:
            if not all(n in self.js for n in S.ARM_JOINTS):
                if now - self.t0 > 5.0 and not self.warned:
                    self.warned = True
                    self.get_logger().warn('No /joint_states yet - is Gazebo running with the controllers?')
                return
            # start from where the robot is, so it does not jump
            self.mapper = JoyMapper(self.layout, start=[self.js[n] for n in S.ARM_JOINTS])
            self.get_logger().info('Robot found - move the sticks.')
        if self.dev is not None:
            axes, buttons = self.dev.state()
        elif self.joy is not None:
            axes, buttons = self.joy
        else:
            return
        q, grip = self.mapper.update(axes, buttons, dt)
        if self.mapper.message == 'estop':
            self.filt.estop = not self.filt.estop
            self.get_logger().warn('EMERGENCY STOP' if self.filt.estop else 'stop released')
        elif self.mapper.message == 'mode':
            self.filt.set_mode(SF.next_mode(self.filt.mode))
            self.get_logger().info(f'safety mode: {self.filt.mode}')
        elif self.mapper.message:
            self.get_logger().info(self.mapper.message.upper())
        qm = [self.js[n] for n in S.ARM_JOINTS]
        q, grip = self.filt.filter(q, grip, dt, q_meas=qm, grip_meas=self.js.get('left_finger_joint'),
                                   pucks=list(self.pucks.values()))
        self.mapper.q = list(q) if self.filt.estop else self.filt.clamp_back(self.mapper.q)
        if self.filt.note and self.filt.note != self.last_note:
            self.get_logger().info(self.filt.note)
        self.last_note = self.filt.note
        self.publish(self.arm_pub, S.ARM_JOINTS, q)
        if self.last_grip is None or abs(grip - self.last_grip) > 1e-4:
            self.publish(self.grip_pub, S.GRIPPER_JOINTS, [grip, grip])
            self.last_grip = grip

    @staticmethod
    def publish(pub, names, positions):
        msg = JointTrajectory()
        msg.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points = [pt]
        pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description='SCARA gamepad control (Gazebo)')
    ap.add_argument('--device', default=None, help='joystick device, default the first /dev/input/js*')
    ap.add_argument('--joy-topic', action='store_true', help='read sensor_msgs/Joy from /joy instead')
    ap.add_argument('--layout', default='xbox', choices=['xbox', 'gamecontroller'])
    SF.add_args(ap)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = ScaraJoystick(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
