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

def count_extended_fingers(landmarks):
    tips = [4, 8, 12, 16, 20]
    pips = [2, 6, 10, 14, 18]

    extended = []
    if landmarks[tips[0]].x < landmarks[pips[0]].x:
        extended.append(1)
    else:
        extended.append(0)

    for i in range(1, 5):
        if landmarks[tips[i]].y < landmarks[pips[i]].y:
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
        self.get_logger().info("=== SCARA AI Hand Gesture ROS 2 Controller (Gazebo) Started  ===")
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

        self.ALPHA = 0.15

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

        gesture_name = "Neutral"
        color_bg = (100, 100, 100)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                lm = hand_landmarks.landmark
                extended = count_extended_fingers(lm)
                total_extended = sum(extended)

                wrist = lm[0]
                index_tip = lm[8]
                thumb_tip = lm[4]

                pinch_dist = np.hypot((thumb_tip.x - index_tip.x) * w, (thumb_tip.y - index_tip.y) * h)

                if pinch_dist < 40 or total_extended == 0:
                    gesture_name = "✊ FIST / PINCH: GRASP PUCK"
                    self.raw_gripper = 0.0
                    color_bg = (0, 0, 255)

                elif total_extended >= 4:
                    gesture_name = "🖐️ OPEN PALM: RELEASE PUCK"
                    self.raw_gripper = -0.05
                    color_bg = (0, 255, 0)

                    hand_x = np.clip(wrist.x, 0.15, 0.85)
                    self.raw_column = float(np.interp(hand_x, [0.15, 0.85], [-1.8, 1.8]))

                    hand_y = np.clip(wrist.y, 0.2, 0.8)
                    self.raw_shoulder = float(np.interp(hand_y, [0.2, 0.8], [0.02, -0.14]))

                elif total_extended == 2 and extended[1] == 1 and extended[2] == 1:
                    gesture_name = "✌️ PEACE: ALIGN OVER CONVEYOR"
                    self.raw_column = -0.32
                    self.raw_forearm = 0.75
                    self.raw_shoulder = 0.0
                    color_bg = (255, 0, 255)

                elif total_extended == 3 and extended[1] == 1 and extended[2] == 1 and extended[3] == 1:
                    gesture_name = "👌 OK: ALIGN OVER TABLE"
                    self.raw_column = 0.785
                    self.raw_forearm = 0.85
                    self.raw_shoulder = 0.0
                    color_bg = (255, 255, 0)

                elif extended[1] == 1:
                    gesture_name = "☝️ POINTING: DIRECTIONAL TRACKING"
                    hand_x = np.clip(index_tip.x, 0.15, 0.85)
                    self.raw_column = float(np.interp(hand_x, [0.15, 0.85], [-1.8, 1.8]))

                    hand_y = np.clip(index_tip.y, 0.2, 0.8)
                    self.raw_shoulder = float(np.interp(hand_y, [0.2, 0.8], [0.02, -0.14]))
                    color_bg = (0, 165, 255)

                # EMA Filter
                self.smooth_column = self.ALPHA * self.raw_column + (1 - self.ALPHA) * self.smooth_column
                self.smooth_shoulder = self.ALPHA * self.raw_shoulder + (1 - self.ALPHA) * self.smooth_shoulder
                self.smooth_forearm = self.ALPHA * self.raw_forearm + (1 - self.ALPHA) * self.smooth_forearm
                self.smooth_wrist = self.ALPHA * self.raw_wrist + (1 - self.ALPHA) * self.smooth_wrist
                self.smooth_gripper = self.ALPHA * self.raw_gripper + (1 - self.ALPHA) * self.smooth_gripper

                # Publish Trajectory
                self.publish_targets(self.smooth_column, self.smooth_shoulder, self.smooth_forearm, self.smooth_wrist, self.smooth_gripper)

                # Visual HUD
                cv2.rectangle(frame, (10, 10), (w - 10, 60), color_bg, -1)
                cv2.putText(frame, gesture_name, (20, 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, f"Column:   {self.smooth_column:.2f} rad", (20, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, f"Z-Height: {self.smooth_shoulder:.3f} m", (20, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

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
