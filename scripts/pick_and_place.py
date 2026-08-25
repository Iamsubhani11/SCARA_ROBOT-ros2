#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ScaraPickAndPlace(Node):
    def __init__(self):
        super().__init__('scara_pick_and_place')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.arm_joints = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']
        self.gripper_joints = ['left_finger_joint']

        self.get_logger().info("SCARA Yellow Puck Pick-and-Place Controller Node started.")

    def send_arm_cmd(self, positions, duration_sec=1.5):
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        secs = int(duration_sec)
        nsecs = int((duration_sec - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)
        msg.points = [point]
        self.arm_pub.publish(msg)

    def send_gripper_cmd(self, position, duration_sec=1.0):
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joints
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        secs = int(duration_sec)
        nsecs = int((duration_sec - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)
        msg.points = [point]
        self.gripper_pub.publish(msg)

    def run_sequence(self):
        self.get_logger().info("=== Starting Automated Pick and Place Routine for Yellow Puck ===")

        # Step 1: Initial Home Position & Open Gripper
        self.get_logger().info("[Step 1] Initializing Home Position & Opening Gripper...")
        self.send_arm_cmd([0.0, 0.0, 0.0, 0.0], duration_sec=2.0)
        self.send_gripper_cmd(-0.05, duration_sec=1.5)
        time.sleep(2.5)

        # Step 2: Pre-Pick Position over Yellow Puck on Conveyor Track
        self.get_logger().info("[Step 2] Traversing SCARA Arm over Yellow Puck on Conveyor...")
        self.send_arm_cmd([-0.32, 0.0, 0.75, 0.0], duration_sec=2.0)
        time.sleep(2.5)

        # Step 3: Descend Z-Axis to Grasp Yellow Puck
        self.get_logger().info("[Step 3] Descending Z-Axis to Grasp Yellow Puck...")
        self.send_arm_cmd([-0.32, -0.14, 0.75, 0.0], duration_sec=1.5)
        time.sleep(2.0)

        # Step 4: Grasp Yellow Puck with Gripper
        self.get_logger().info("[Step 4] Closing Gripper Fingers around Yellow Puck...")
        self.send_gripper_cmd(0.0, duration_sec=1.5)
        time.sleep(2.0)

        # Step 5: Lift Yellow Puck off Conveyor Track
        self.get_logger().info("[Step 5] Lifting Yellow Puck off Conveyor Track...")
        self.send_arm_cmd([-0.32, 0.0, 0.75, 0.0], duration_sec=1.5)
        time.sleep(2.0)

        # Step 6: Swing Arm to Wooden Table Drop-Off Zone
        self.get_logger().info("[Step 6] Traversing Arm across Wooden Table Platform...")
        self.send_arm_cmd([0.785, 0.0, 0.85, 0.0], duration_sec=2.5)
        time.sleep(3.0)

        # Step 7: Descend Z-Axis to Table Surface
        self.get_logger().info("[Step 7] Descending Z-Axis onto Wooden Table Surface...")
        self.send_arm_cmd([0.785, -0.10, 0.85, 0.0], duration_sec=1.5)
        time.sleep(2.0)

        # Step 8: Release Yellow Puck
        self.get_logger().info("[Step 8] Releasing Gripper...")
        self.send_gripper_cmd(-0.05, duration_sec=1.5)
        time.sleep(2.0)

        # Step 9: Retract Z-Axis
        self.get_logger().info("[Step 9] Retracting Z-Axis...")
        self.send_arm_cmd([0.785, 0.0, 0.85, 0.0], duration_sec=1.5)
        time.sleep(2.0)

        # Step 10: Return Home
        self.get_logger().info("[Step 10] Returning SCARA Arm to Home Position...")
        self.send_arm_cmd([0.0, 0.0, 0.0, 0.0], duration_sec=2.0)
        time.sleep(2.5)

        self.get_logger().info("=== Automated Yellow Puck Pick-and-Place Routine Completed ===")

def main():
    rclpy.init()
    node = ScaraPickAndPlace()
    time.sleep(1.0)
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
