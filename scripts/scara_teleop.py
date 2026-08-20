#!/usr/bin/env python3

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

HELP_MSG = """
==================================================
        SCARA Robot Teleoperation Interface
==================================================
Control the 5-DOF SCARA robot joints using keys:

  Base Column Rotation (column_joint):
    [a] : Turn Left  (+0.05 rad)
    [d] : Turn Right (-0.05 rad)

  Shoulder Vertical Axis (shoulder_joint):
    [w] : Lift Up   (+0.005 m)
    [s] : Move Down (-0.005 m)

  Forearm Rotation (forearm_joint):
    [j] : Rotate Left  (+0.05 rad)
    [l] : Rotate Right (-0.05 rad)

  Wrist Rotation (wrist_joint):
    [i] : Pitch/Roll Up   (+0.1 rad)
    [k] : Pitch/Roll Down (-0.1 rad)

  Gripper (left_finger_joint):
    [u] : Open Gripper  (-0.05 m)
    [o] : Close Gripper ( 0.00 m)

  General:
    [h] : Reset to Home Position
    [q] / Ctrl+C : Quit Teleoperation
==================================================
"""

# Joint limits
LIMITS = {
    'column_joint': (-2.0, 2.0),
    'shoulder_joint': (-0.15, 0.02),
    'forearm_joint': (-2.0, 2.0),
    'wrist_joint': (-4.712, 4.712),
    'left_finger_joint': (-0.06, 0.0)
}

class ScaraTeleopNode(Node):
    def __init__(self):
        super().__init__('scara_teleop')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.arm_joints = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']
        self.gripper_joints = ['left_finger_joint']

        # Initial positions
        self.arm_pos = [0.0, 0.0, 0.0, 0.0]
        self.gripper_pos = [0.0]

        self.get_logger().info("SCARA Teleop Node started.")

    def clamp(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def publish_arm(self):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in self.arm_pos]
        point.time_from_start = Duration(sec=0, nanosec=200000000)
        msg.points = [point]
        self.arm_pub.publish(msg)

    def publish_gripper(self):
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in self.gripper_pos]
        point.time_from_start = Duration(sec=0, nanosec=200000000)
        msg.points = [point]
        self.gripper_pub.publish(msg)

    def update_arm_joint(self, idx, delta):
        j_name = self.arm_joints[idx]
        low, high = LIMITS[j_name]
        self.arm_pos[idx] = self.clamp(self.arm_pos[idx] + delta, low, high)
        self.get_logger().info(f"Updated {j_name}: {self.arm_pos[idx]:.3f}")
        self.publish_arm()

    def update_gripper(self, target_pos):
        low, high = LIMITS['left_finger_joint']
        self.gripper_pos[0] = self.clamp(target_pos, low, high)
        self.get_logger().info(f"Updated gripper: {self.gripper_pos[0]:.3f}")
        self.publish_gripper()

    def home(self):
        self.arm_pos = [0.0, 0.0, 0.0, 0.0]
        self.gripper_pos = [0.0]
        self.get_logger().info("Reset to HOME position")
        self.publish_arm()
        self.publish_gripper()


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = ScaraTeleopNode()
    print(HELP_MSG)

    try:
        while rclpy.ok():
            key = getKey(settings)
            if key == 'a':
                node.update_arm_joint(0, 0.05)
            elif key == 'd':
                node.update_arm_joint(0, -0.05)
            elif key == 'w':
                node.update_arm_joint(1, 0.005)
            elif key == 's':
                node.update_arm_joint(1, -0.005)
            elif key == 'j':
                node.update_arm_joint(2, 0.05)
            elif key == 'l':
                node.update_arm_joint(2, -0.05)
            elif key == 'i':
                node.update_arm_joint(3, 0.1)
            elif key == 'k':
                node.update_arm_joint(3, -0.1)
            elif key == 'u':
                node.update_gripper(-0.05)
            elif key == 'o':
                node.update_gripper(0.0)
            elif key == 'h':
                node.home()
            elif key == 'q' or key == '\x03':
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
