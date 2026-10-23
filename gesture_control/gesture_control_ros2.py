#!/usr/bin/env python3
"""
Two-hand gesture control of the SCARA in Gazebo (ROS 2).

    LEFT hand : left/right = base column,  up/down = Z height
    RIGHT hand: left/right = forearm,      tilt = wrist,  pinch thumb+index = close gripper
    FIST      : freeze that hand's joints      Hand out of view: hold
    q / Esc in the camera window: quit

Start Gazebo first (no sorter, so they don't fight over the arm):
    ros2 launch scara_description gazebo.launch.py mode:=manual
    python3 gesture_control/gesture_control_ros2.py
    python3 gesture_control/gesture_control_ros2.py --ros-args -p camera:=1 -p safety:=normal

Parameters: camera (default 0), rate (publish Hz, default 20),
            safety safe|normal|off (default safe), assist true|false (default true)
In the camera window: q / Esc quit, SPACE emergency stop on/off, m next safety mode.
Safety mode (common/scara_safety.py): slow joints + gentle gripper, pick assist
(the open gripper centres itself over a puck - puck poses from /scara/world_poses),
a guard against pressing a finger down on a puck, table floor, emergency stop.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import os
import sys
import time

import cv2
import mediapipe as mp
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gesture_core as G   # noqa: E402
S = G.S
import scara_safety as SF   # noqa: E402


class GestureNode(Node):
    def __init__(self):
        super().__init__('scara_gesture_control')
        self.declare_parameter('camera', 0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('safety', 'safe')
        self.declare_parameter('assist', True)
        self.filt = SF.SafetyFilter(str(self.get_parameter('safety').value),
                                    assist=bool(self.get_parameter('assist').value))
        self.pucks = {}
        self.create_subscription(TFMessage, '/scara/world_poses', self.on_poses, 10)
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.js = {}
        self.create_subscription(JointState, '/joint_states', self.on_js, 20)
        self.cap = cv2.VideoCapture(int(self.get_parameter('camera').value))
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open the webcam (parameter camera:=<index>)')
        self.hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, model_complexity=1,
                                              min_detection_confidence=0.6, min_tracking_confidence=0.6)
        self.draw = mp.solutions.drawing_utils
        self.mapper = None
        self.last_grip = None
        self.last_t = time.time()
        self.period = 1.0 / float(self.get_parameter('rate').value)
        self.create_timer(1.0 / 30.0, self.loop)
        self.get_logger().info('Gesture control: show BOTH hands. Left = column + Z, right = forearm + wrist, pinch = grip.')

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def on_poses(self, msg):
        for t in msg.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_'):
                p = t.transform.translation
                self.pucks[name] = (p.x, p.y, p.z)

    def start_mapper(self):
        """Start from where the robot really is, so it never jumps."""
        j = self.js
        self.mapper = G.GestureMapper({'column': j.get('column_joint', 0.0), 'z': j.get('shoulder_joint', 0.0),
                                       'forearm': j.get('forearm_joint', 0.0), 'wrist': j.get('wrist_joint', 0.0),
                                       'gripper': j.get('left_finger_joint', S.GRIPPER_OPEN)})
        self.get_logger().info('Robot state received - gesture control active')

    def publish(self, names, positions, pub, t):
        msg = JointTrajectory()
        msg.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=0, nanosec=int(t * 1e9))
        msg.points = [pt]
        pub.publish(msg)

    def loop(self):
        ok, frame = self.cap.read() if self.cap.isOpened() else (False, None)
        if not ok:
            return
        frame = cv2.flip(frame, 1)                  # mirror view = MediaPipe's handedness convention
        now = time.time()
        dt = min(0.1, now - self.last_t)
        self.last_t = now
        res = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        hands = []
        if res.multi_hand_landmarks:
            for lms, handed in zip(res.multi_hand_landmarks, res.multi_handedness or [None] * 8):
                arr = [[p.x, p.y, p.z] for p in lms.landmark]
                label = handed.classification[0].label if handed else None
                hands.append((G.np.array(arr), label))
                self.draw.draw_landmarks(frame, lms, mp.solutions.hands.HAND_CONNECTIONS)
        if self.mapper is None:
            if self.js:
                self.start_mapper()
            else:
                cv2.putText(frame, 'Waiting for /joint_states (is Gazebo running?)', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow('SCARA Hand Control', frame)
                cv2.waitKey(1)
                return
        _, grip = self.mapper.update(hands, now, dt)
        qm = [self.js.get(n, 0.0) for n in S.ARM_JOINTS]
        q, grip = self.filt.filter(self.mapper.arm_vector(), grip, dt, q_meas=qm,
                                   grip_meas=self.js.get('left_finger_joint'), pucks=list(self.pucks.values()))
        self.mapper.note = f'[{self.filt.mode}] ' + (self.filt.note or 'ready')
        # stream the arm target (short horizon keeps the motion smooth)
        self.publish(S.ARM_JOINTS, q, self.arm_pub, max(0.1, 2 * self.period))
        if self.last_grip is None or abs(grip - self.last_grip) > 1e-4:
            self.publish(S.GRIPPER_JOINTS, [grip, grip], self.grip_pub, max(0.1, 2 * self.period))   # BOTH fingers
            self.last_grip = grip
        G.draw_hud(frame, self.mapper)
        cv2.imshow('SCARA Hand Control', frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):
            raise KeyboardInterrupt
        if k == ord(' '):
            self.filt.estop = not self.filt.estop
            self.get_logger().warn('EMERGENCY STOP' if self.filt.estop else 'stop released')
        if k == ord('m'):
            self.filt.set_mode(SF.next_mode(self.filt.mode))
            self.get_logger().info(f'safety mode: {self.filt.mode}')


def main():
    rclpy.init()
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
