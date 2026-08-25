#!/usr/bin/env python3

import os
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import mediapipe as mp

class SCARAGestureControllerROS2(Node):
    def __init__(self):
        super().__init__('scara_gesture_controller_ros2')

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.get_logger().info("=============================================================")
        self.get_logger().info("=== SCARA Robot ROS 2 Gesture Controller (Gazebo) Started ===")
        self.get_logger().info("=============================================================")

        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Camera Capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warn("Webcam camera index 0 not detected.")

        self.timer = self.create_timer(0.05, self.process_frame)

    def process_frame(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                wrist = hand_landmarks.landmark[0]
                index_tip = hand_landmarks.landmark[8]
                thumb_tip = hand_landmarks.landmark[4]

                # Map Hand X to Base Column
                hand_x = np.clip(wrist.x, 0.1, 0.9)
                column_target = float(np.interp(hand_x, [0.1, 0.9], [-1.8, 1.8]))

                # Map Hand Y to Z-Height
                hand_y = np.clip(wrist.y, 0.2, 0.8)
                shoulder_target = float(np.interp(hand_y, [0.2, 0.8], [0.02, -0.14]))

                # Map Index Tip to Forearm
                index_y = np.clip(index_tip.y, 0.2, 0.8)
                forearm_target = float(np.interp(index_y, [0.2, 0.8], [-1.2, 1.5]))

                # Pinch to Gripper
                pinch_dist = np.hypot((thumb_tip.x - index_tip.x) * w, (thumb_tip.y - index_tip.y) * h)
                gripper_target = 0.0 if pinch_dist < 40 else -0.05

                # Publish to ROS 2 Trajectory Controllers
                self.publish_targets(column_target, shoulder_target, forearm_target, 0.0, gripper_target)

                # Draw HUD
                gesture_str = "CLOSED (GRASP)" if pinch_dist < 40 else "OPEN (RELEASE)"
                cv2.putText(frame, f"Gesture: {gesture_str}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, f"Column: {column_target:.2f} rad", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, f"Z-Height: {shoulder_target:.3f} m", (20, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("SCARA ROS 2 Gesture Controller", frame)
        cv2.waitKey(1)

    def publish_targets(self, col, sh, fore, wr, grip):
        # Arm trajectory
        arm_traj = JointTrajectory()
        arm_traj.joint_names = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']
        p_arm = JointTrajectoryPoint()
        p_arm.positions = [col, sh, fore, wr]
        p_arm.time_from_start.sec = 0
        p_arm.time_from_start.nanosec = 100000000
        arm_traj.points.append(p_arm)
        self.arm_pub.publish(arm_traj)

        # Gripper trajectory
        grip_traj = JointTrajectory()
        grip_traj.joint_names = ['left_finger_joint']
        p_grip = JointTrajectoryPoint()
        p_grip.positions = [grip]
        p_grip.time_from_start.sec = 0
        p_grip.time_from_start.nanosec = 100000000
        grip_traj.points.append(p_grip)
        self.gripper_pub.publish(grip_traj)

def main(args=None):
    rclpy.init(args=args)
    node = SCARAGestureControllerROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
