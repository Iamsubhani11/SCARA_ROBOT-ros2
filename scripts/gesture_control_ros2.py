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

        self.get_logger().info("==================================================================")
        self.get_logger().info("=== SCARA AI 5-Joint Gesture ROS 2 Controller (Gazebo) Started ===")
        self.get_logger().info("==================================================================")

        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Camera Capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warn("Webcam camera index 0 not detected.")

        # Target states
        self.raw_column = 0.0
        self.raw_shoulder = 0.0
        self.raw_forearm = 0.0
        self.raw_wrist = 0.0
        self.raw_gripper = -0.05

        self.smooth_column = 0.0
        self.smooth_shoulder = 0.0
        self.smooth_forearm = 0.0
        self.smooth_wrist = 0.0
        self.smooth_gripper = -0.05

        self.ALPHA = 0.18

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

                lm = hand_landmarks.landmark
                wrist_lm = lm[0]
                index_tip = lm[8]
                index_mcp = lm[5]
                thumb_tip = lm[4]
                pinky_mcp = lm[17]

                # 1. Joint 1: Base Column (Hand X Position: 0.15 to 0.85 -> -1.8 to 1.8 rad)
                hand_x = np.clip(wrist_lm.x, 0.15, 0.85)
                self.raw_column = float(np.interp(hand_x, [0.15, 0.85], [-1.8, 1.8]))

                # 2. Joint 2: Z-Axis Height (Hand Y Position: 0.20 to 0.80 -> 0.02 to -0.14 m)
                hand_y = np.clip(wrist_lm.y, 0.20, 0.80)
                self.raw_shoulder = float(np.interp(hand_y, [0.20, 0.80], [0.02, -0.14]))

                # 3. Joint 3: Forearm Elbow (Distance from Wrist to Index Tip: 0.15 to 0.45 -> -1.2 to 1.5 rad)
                index_dist = np.hypot((index_tip.x - wrist_lm.x) * w, (index_tip.y - wrist_lm.y) * h) / w
                index_dist = np.clip(index_dist, 0.15, 0.45)
                self.raw_forearm = float(np.interp(index_dist, [0.15, 0.45], [-1.2, 1.5]))

                # 4. Joint 4: Wrist Rotation (Hand Tilt / Roll Angle between Index MCP and Pinky MCP)
                dx = (pinky_mcp.x - index_mcp.x) * w
                dy = (pinky_mcp.y - index_mcp.y) * h
                hand_roll = np.arctan2(dy, dx)
                self.raw_wrist = float(np.clip(hand_roll * 2.0, -3.14, 3.14))

                # 5. Joint 5: Gripper Open/Close (Pinch Distance between Thumb Tip & Index Tip)
                pinch_dist = np.hypot((thumb_tip.x - index_tip.x) * w, (thumb_tip.y - index_tip.y) * h)
                if pinch_dist < 40:
                    self.raw_gripper = 0.0  # CLOSED (GRASP)
                    gripper_status = "CLOSED (GRASPING PUCK)"
                    status_color = (0, 0, 255)
                else:
                    self.raw_gripper = -0.05  # OPEN (RELEASE)
                    gripper_status = "OPEN (RELEASED)"
                    status_color = (0, 255, 0)

                # EMA Low-Pass Filter
                self.smooth_column = self.ALPHA * self.raw_column + (1 - self.ALPHA) * self.smooth_column
                self.smooth_shoulder = self.ALPHA * self.raw_shoulder + (1 - self.ALPHA) * self.smooth_shoulder
                self.smooth_forearm = self.ALPHA * self.raw_forearm + (1 - self.ALPHA) * self.smooth_forearm
                self.smooth_wrist = self.ALPHA * self.raw_wrist + (1 - self.ALPHA) * self.smooth_wrist
                self.smooth_gripper = self.ALPHA * self.raw_gripper + (1 - self.ALPHA) * self.smooth_gripper

                # Publish Trajectory to ROS 2 Controller Topics
                self.publish_targets(self.smooth_column, self.smooth_shoulder, self.smooth_forearm, self.smooth_wrist, self.smooth_gripper)

                # Draw Visual HUD Overlay with All 5 Joint Statuses
                cv2.rectangle(frame, (10, 10), (w - 10, 200), (30, 30, 30), -1)
                cv2.putText(frame, "SCARA ROS 2 5-Joint Gesture Controller", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                cv2.putText(frame, f"1. Base Column  : {self.smooth_column:+.2f} rad (Move Left/Right)", (20, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(frame, f"2. Z-Axis Height: {self.smooth_shoulder:+.3f} m   (Move Up/Down)", (20, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(frame, f"3. Forearm Angle: {self.smooth_forearm:+.2f} rad (Index Stretch)", (20, 115),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(frame, f"4. Wrist Roll   : {self.smooth_wrist:+.2f} rad (Hand Tilt)", (20, 140),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(frame, f"5. Gripper State: {gripper_status}", (20, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

        cv2.imshow("SCARA ROS 2 AI Gesture Controller", frame)
        cv2.waitKey(1)

    def publish_targets(self, col, sh, fore, wr, grip):
        arm_traj = JointTrajectory()
        arm_traj.joint_names = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']
        p_arm = JointTrajectoryPoint()
        p_arm.positions = [col, sh, fore, wr]
        p_arm.time_from_start.sec = 0
        p_arm.time_from_start.nanosec = 100000000
        arm_traj.points.append(p_arm)
        self.arm_pub.publish(arm_traj)

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
