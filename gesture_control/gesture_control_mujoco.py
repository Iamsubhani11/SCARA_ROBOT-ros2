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
        xml_path = '/home/robotics/Scara_robot/urdf/scara_scene_mujoco.xml'
    return xml_path

def count_extended_fingers(lm):
    tips = [4, 8, 12, 16, 20]
    pips = [2, 6, 10, 14, 18]

    extended = []
    # Thumb check
    if abs(lm[4].x - lm[0].x) > 0.08 and (lm[4].x > lm[2].x or lm[4].y < lm[3].y):
        extended.append(1)
    else:
        extended.append(0)

    # 4 Fingers (Index, Middle, Ring, Pinky: tip above PIP in Y-axis)
    for i in range(1, 5):
        if lm[tips[i]].y < lm[pips[i]].y:
            extended.append(1)
        else:
            extended.append(0)

    return extended

def main():
    print("==================================================================")
    print("=== SCARA 2-Step Finger Select + Hand Tilt Controller (MuJoCo) ===")
    print("==================================================================")
    print(" STEP 1: Select Joint by Number of Extended Fingers:")
    print("   ☝️ 1 Finger  : Joint 1 - Base Column Rotation")
    print("   ✌️ 2 Fingers : Joint 2 - Z-Axis Elevation Height")
    print("   🤟 3 Fingers : Joint 3 - Forearm Elbow Angle")
    print("   🖖 4 Fingers : Joint 4 - Wrist Rotation Angle")
    print("   🖐️ 5 Fingers : Joint 5 - Gripper Open / Close")
    print("\n STEP 2: Choose Motion Direction by Tilting Hand:")
    print("   ↩️ Tilt Hand LEFT  : Move Selected Joint LEFT / DOWN / OPEN")
    print("   ⏹️ Keep Hand LEVEL : HOLD Selected Joint Position")
    print("   ↪️ Tilt Hand RIGHT : Move Selected Joint RIGHT / UP / CLOSE")
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

    # Target joint positions
    curr_column = 0.0
    curr_shoulder = 0.0
    curr_forearm = 0.0
    curr_wrist = 0.0
    curr_gripper = -0.05

    # Incremental speeds per frame
    SPEED_COLUMN = 0.025
    SPEED_SHOULDER = 0.002
    SPEED_FOREARM = 0.025
    SPEED_WRIST = 0.035

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            selected_joint = "None (No Hand Detected)"
            direction_str = "HOLD"
            status_color = (200, 200, 200)

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
                            extended = count_extended_fingers(lm)
                            total_fingers = sum(extended)

                            index_mcp = lm[5]
                            pinky_mcp = lm[17]

                            # Calculate Hand Tilt / Roll Angle (radians)
                            dx = (pinky_mcp.x - index_mcp.x) * w
                            dy = (pinky_mcp.y - index_mcp.y) * h
                            roll_angle = np.arctan2(dy, dx)

                            # Determine Direction from Hand Tilt Angle
                            if roll_angle > 0.25:
                                direction = 1   # RIGHT / UP / CLOSE
                                direction_str = "▶️ RIGHT / UP / CLOSE"
                                status_color = (0, 255, 0)
                            elif roll_angle < -0.25:
                                direction = -1  # LEFT / DOWN / OPEN
                                direction_str = "◀️ LEFT / DOWN / OPEN"
                                status_color = (0, 165, 255)
                            else:
                                direction = 0   # HOLD
                                direction_str = "⏹️ HOLD POSITION"
                                status_color = (255, 255, 0)

                            # Step 1: Select Joint by Finger Count & Step 2: Apply Motion Direction
                            if total_fingers == 1:
                                selected_joint = "1. Base Column (column_joint)"
                                curr_column += direction * SPEED_COLUMN
                                curr_column = float(np.clip(curr_column, -2.0, 2.0))

                            elif total_fingers == 2:
                                selected_joint = "2. Z-Axis Elevation (shoulder_joint)"
                                curr_shoulder += direction * SPEED_SHOULDER
                                curr_shoulder = float(np.clip(curr_shoulder, -0.15, 0.02))

                            elif total_fingers == 3:
                                selected_joint = "3. Forearm Elbow (forearm_joint)"
                                curr_forearm += direction * SPEED_FOREARM
                                curr_forearm = float(np.clip(curr_forearm, -2.0, 2.0))

                            elif total_fingers == 4:
                                selected_joint = "4. Wrist Rotation (wrist_joint)"
                                curr_wrist += direction * SPEED_WRIST
                                curr_wrist = float(np.clip(curr_wrist, -4.71, 4.71))

                            elif total_fingers == 5:
                                selected_joint = "5. Gripper (left_finger_joint)"
                                if direction == 1:
                                    curr_gripper = 0.0    # CLOSE / GRASP PUCK
                                elif direction == -1:
                                    curr_gripper = -0.05  # OPEN / RELEASE PUCK

                            # Draw Visual HUD Panel
                            cv2.rectangle(frame, (10, 10), (w - 10, 190), (30, 30, 30), -1)
                            cv2.putText(frame, "SCARA Finger Select + Hand Tilt Controller", (20, 35),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                            cv2.putText(frame, f"Fingers: {total_fingers} -> Active Joint: {selected_joint}", (20, 65),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                            cv2.putText(frame, f"Hand Tilt: {direction_str}", (20, 95),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

                            # Joint Progress Readings
                            cv2.putText(frame, f"Col: {curr_column:+.2f}r | Z: {curr_shoulder:+.3f}m | Forearm: {curr_forearm:+.2f}r | Wrist: {curr_wrist:+.2f}r | Grip: {'CLOSED' if curr_gripper > -0.02 else 'OPEN'}",
                                        (20, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

                    cv2.imshow("SCARA AI Hand Gesture Teleoperation", frame)
                    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                        break

            # Update SCARA actuators in MuJoCo
            data.ctrl[0] = curr_column
            data.ctrl[1] = curr_shoulder
            data.ctrl[2] = curr_forearm
            data.ctrl[3] = curr_wrist
            data.ctrl[4] = curr_gripper
            data.ctrl[5] = curr_gripper

            # Step physics simulation forward
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    if camera_available:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
