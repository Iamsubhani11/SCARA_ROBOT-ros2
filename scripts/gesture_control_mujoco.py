#!/usr/bin/env python3

import os
import sys
import time
import cv2
import numpy as np
import mujoco
import mujoco.viewer
import mediapipe as mp

def get_scene_xml_path():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xml_path = os.path.join(pkg_dir, 'urdf', 'scara_scene_mujoco.xml')
    if not os.path.exists(xml_path):
        xml_path = '/home/hemanthros/scara_description_ws/urdf/scara_scene_mujoco.xml'
    return xml_path

def main():
    print("==================================================================")
    print("===  SCARA Robot Full 5-Joint Hand Gesture Teleop (MuJoCo)  ===")
    print("==================================================================")
    print(" Joint Mapping:")
    print("   ↔️  Hand Move Left / Right : Joint 1 - Base Column Rotation")
    print("   ↕️  Hand Move Up / Down    : Joint 2 - Z-Axis Height (Elevation)")
    print("   👈 Index Finger Bend/Extend: Joint 3 - Forearm Elbow Angle")
    print("   🔄 Hand Tilt / Wrist Roll  : Joint 4 - Wrist Rotation")
    print("   🤏 Thumb-Index Pinch / Fist: Joint 5 - Gripper Open / Close")
    print("==================================================================\n")

    xml_path = get_scene_xml_path()
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Initialize MediaPipe Hand Tracking
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    )
    mp_draw = mp.solutions.drawing_utils

    # Open Webcam
    cap = cv2.VideoCapture(0)
    camera_available = cap.isOpened()

    if not camera_available:
        print("[WARNING] Webcam camera index 0 not available.")

    # Target joint positions (Raw & Smoothed)
    raw_column = 0.0
    raw_shoulder = 0.0
    raw_forearm = 0.0
    raw_wrist = 0.0
    raw_gripper = -0.05

    smooth_column = 0.0
    smooth_shoulder = 0.0
    smooth_forearm = 0.0
    smooth_wrist = 0.0
    smooth_gripper = -0.05

    ALPHA = 0.18  # Low-pass EMA filter coefficient (0.18 = fast & smooth)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if camera_available:
                ret, frame = cap.read()
                if ret:
                    frame = cv2.flip(frame, 1)
                    h, w, _ = frame.shape
                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = hands.process(rgb_frame)

                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                            lm = hand_landmarks.landmark
                            wrist_lm = lm[0]
                            index_tip = lm[8]
                            index_mcp = lm[5]
                            thumb_tip = lm[4]
                            pinky_mcp = lm[17]

                            # 1. Joint 1: Base Column (Hand X Position: 0.15 to 0.85 -> -1.8 to 1.8 rad)
                            hand_x = np.clip(wrist_lm.x, 0.15, 0.85)
                            raw_column = float(np.interp(hand_x, [0.15, 0.85], [-1.8, 1.8]))

                            # 2. Joint 2: Z-Axis Height (Hand Y Position: 0.20 to 0.80 -> 0.02 to -0.14 m)
                            hand_y = np.clip(wrist_lm.y, 0.20, 0.80)
                            raw_shoulder = float(np.interp(hand_y, [0.20, 0.80], [0.02, -0.14]))

                            # 3. Joint 3: Forearm Elbow (Distance from Wrist to Index Tip: 0.15 to 0.45 -> -1.2 to 1.5 rad)
                            index_dist = np.hypot((index_tip.x - wrist_lm.x) * w, (index_tip.y - wrist_lm.y) * h) / w
                            index_dist = np.clip(index_dist, 0.15, 0.45)
                            raw_forearm = float(np.interp(index_dist, [0.15, 0.45], [-1.2, 1.5]))

                            # 4. Joint 4: Wrist Rotation (Hand Tilt / Roll Angle between Index MCP and Pinky MCP)
                            dx = (pinky_mcp.x - index_mcp.x) * w
                            dy = (pinky_mcp.y - index_mcp.y) * h
                            hand_roll = np.arctan2(dy, dx)
                            raw_wrist = float(np.clip(hand_roll * 2.0, -3.14, 3.14))

                            # 5. Joint 5: Gripper Open/Close (Pinch Distance between Thumb Tip & Index Tip)
                            pinch_dist = np.hypot((thumb_tip.x - index_tip.x) * w, (thumb_tip.y - index_tip.y) * h)
                            if pinch_dist < 40:
                                raw_gripper = 0.0  # CLOSED (GRASP)
                                gripper_status = "CLOSED (GRASPING PUCK)"
                                status_color = (0, 0, 255)
                            else:
                                raw_gripper = -0.05  # OPEN (RELEASE)
                                gripper_status = "OPEN (RELEASED)"
                                status_color = (0, 255, 0)

                            # Apply EMA Low-Pass Filter
                            smooth_column = ALPHA * raw_column + (1 - ALPHA) * smooth_column
                            smooth_shoulder = ALPHA * raw_shoulder + (1 - ALPHA) * smooth_shoulder
                            smooth_forearm = ALPHA * raw_forearm + (1 - ALPHA) * smooth_forearm
                            smooth_wrist = ALPHA * raw_wrist + (1 - ALPHA) * smooth_wrist
                            smooth_gripper = ALPHA * raw_gripper + (1 - ALPHA) * smooth_gripper

                            # Draw Visual HUD Overlay with All 5 Joint Statuses
                            cv2.rectangle(frame, (10, 10), (w - 10, 200), (30, 30, 30), -1)
                            cv2.putText(frame, "SCARA AI 5-Joint Gesture Controller", (20, 35),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                            cv2.putText(frame, f"1. Base Column  : {smooth_column:+.2f} rad (Move Left/Right)", (20, 65),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                            cv2.putText(frame, f"2. Z-Axis Height: {smooth_shoulder:+.3f} m   (Move Up/Down)", (20, 90),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                            cv2.putText(frame, f"3. Forearm Angle: {smooth_forearm:+.2f} rad (Index Stretch)", (20, 115),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                            cv2.putText(frame, f"4. Wrist Roll   : {smooth_wrist:+.2f} rad (Hand Tilt)", (20, 140),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                            cv2.putText(frame, f"5. Gripper State: {gripper_status}", (20, 175),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

                    cv2.imshow("SCARA AI Hand Gesture Teleoperation", frame)
                    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                        break

            # Update SCARA actuators in MuJoCo
            data.ctrl[0] = smooth_column
            data.ctrl[1] = smooth_shoulder
            data.ctrl[2] = smooth_forearm
            data.ctrl[3] = smooth_wrist
            data.ctrl[4] = smooth_gripper
            data.ctrl[5] = smooth_gripper

            # Step physics simulation forward
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    if camera_available:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
