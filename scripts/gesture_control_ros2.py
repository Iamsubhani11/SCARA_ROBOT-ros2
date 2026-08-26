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

def count_extended_fingers(lm):
    tips = [4, 8, 12, 16, 20]
    pips = [2, 6, 10, 14, 18]

    extended = []
    if abs(lm[4].x - lm[0].x) > 0.08 and (lm[4].x > lm[2].x or lm[4].y < lm[3].y):
        extended.append(1)
    else:
        extended.append(0)

    for i in range(1, 5):
        if lm[tips[i]].y < lm[pips[i]].y:
            extended.append(1)
        else:
            extended.append(0)

    return extended

class SCARAGestureControllerROS2(Node):
    def __init__(self):
        super().__init__('scara_gesture_controller_ros2')

        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.get_logger().info("==================================================================")
        self.get_logger().info("=== SCARA Finger Select + Hand Tilt ROS 2 Controller Started ===")
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

        # Current Joint Positions
        self.curr_column = 0.0
        self.curr_shoulder = 0.0
        self.curr_forearm = 0.0
        self.curr_wrist = 0.0
        self.curr_gripper = -0.05

        # Speed constants
        self.SPEED_COLUMN = 0.025
        self.SPEED_SHOULDER = 0.002
        self.SPEED_FOREARM = 0.025
        self.SPEED_WRIST = 0.035

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

        selected_joint = "None (No Hand Detected)"
        direction_str = "HOLD"
        status_color = (200, 200, 200)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                lm = hand_landmarks.landmark
                extended = count_extended_fingers(lm)
                total_fingers = sum(extended)

                index_mcp = lm[5]
                pinky_mcp = lm[17]

                # Hand Tilt / Roll Angle
                dx = (pinky_mcp.x - index_mcp.x) * w
                dy = (pinky_mcp.y - index_mcp.y) * h
                roll_angle = np.arctan2(dy, dx)

                # Determine Direction from Hand Tilt
                if roll_angle > 0.25:
                    direction = 1
                    direction_str = "▶️ RIGHT / UP / CLOSE"
                    status_color = (0, 255, 0)
                elif roll_angle < -0.25:
                    direction = -1
                    direction_str = "◀️ LEFT / DOWN / OPEN"
                    status_color = (0, 165, 255)
                else:
                    direction = 0
                    direction_str = "⏹️ HOLD POSITION"
                    status_color = (255, 255, 0)

                # Step 1: Select Joint by Finger Count & Step 2: Apply Motion Direction
                if total_fingers == 1:
                    selected_joint = "1. Base Column (column_joint)"
                    self.curr_column += direction * self.SPEED_COLUMN
                    self.curr_column = float(np.clip(self.curr_column, -2.0, 2.0))

                elif total_fingers == 2:
                    selected_joint = "2. Z-Axis Elevation (shoulder_joint)"
                    self.curr_shoulder += direction * self.SPEED_SHOULDER
                    self.curr_shoulder = float(np.clip(self.curr_shoulder, -0.15, 0.02))

                elif total_fingers == 3:
                    selected_joint = "3. Forearm Elbow (forearm_joint)"
                    self.curr_forearm += direction * self.SPEED_FOREARM
                    self.curr_forearm = float(np.clip(self.curr_forearm, -2.0, 2.0))

                elif total_fingers == 4:
                    selected_joint = "4. Wrist Rotation (wrist_joint)"
                    self.curr_wrist += direction * self.SPEED_WRIST
                    self.curr_wrist = float(np.clip(self.curr_wrist, -4.71, 4.71))

                elif total_fingers == 5:
                    selected_joint = "5. Gripper (left_finger_joint)"
                    if direction == 1:
                        self.curr_gripper = 0.0
                    elif direction == -1:
                        self.curr_gripper = -0.05

                # Publish Trajectory to ROS 2 Controller Topics
                self.publish_targets(self.curr_column, self.curr_shoulder, self.curr_forearm, self.curr_wrist, self.curr_gripper)

                # Visual HUD Panel
                cv2.rectangle(frame, (10, 10), (w - 10, 190), (30, 30, 30), -1)
                cv2.putText(frame, "SCARA ROS 2 Finger Select + Tilt Controller", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                cv2.putText(frame, f"Fingers: {total_fingers} -> Active Joint: {selected_joint}", (20, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Hand Tilt: {direction_str}", (20, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

                cv2.putText(frame, f"Col: {self.curr_column:+.2f}r | Z: {self.curr_shoulder:+.3f}m | Forearm: {self.curr_forearm:+.2f}r | Wrist: {self.curr_wrist:+.2f}r | Grip: {'CLOSED' if self.curr_gripper > -0.02 else 'OPEN'}",
                            (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

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
