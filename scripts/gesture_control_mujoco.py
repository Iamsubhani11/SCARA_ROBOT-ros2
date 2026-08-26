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

def count_extended_fingers(landmarks):
    # Landmark indices for fingertips and PIP joints
    tips = [4, 8, 12, 16, 20]
    pips = [2, 6, 10, 14, 18]

    extended = []
    # Thumb (check horizontal / vertical offset)
    if landmarks[tips[0]].x < landmarks[pips[0]].x:
        extended.append(1)
    else:
        extended.append(0)

    # 4 Fingers (Index, Middle, Ring, Pinky: tip above PIP joint in Y-axis)
    for i in range(1, 5):
        if landmarks[tips[i]].y < landmarks[pips[i]].y:
            extended.append(1)
        else:
            extended.append(0)

    return extended

def main():
    print("==================================================================")
    print("===  SCARA AI Hand Gesture Teleop Controller (MuJoCo)  ===")
    print("==================================================================")
    print(" 🖐️  OPEN PALM (5 Fingers)    : OPEN Gripper (Release Puck)")
    print(" ✊ CLOSED FIST / PINCH       : CLOSE Gripper (Grasp Puck)")
    print(" ✌️  PEACE SIGN (2 Fingers)    : Auto-Align Over Conveyor Puck")
    print(" 👌 OK / 3 FINGERS             : Auto-Align Over Wooden Table")
    print(" ☝️  POINTING INDEX FINGER     : Proportional Directional Mode")
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
    raw_column = 0.0
    raw_shoulder = 0.0
    raw_forearm = 0.0
    raw_wrist = 0.0
    raw_gripper = -0.05

    # Smoothed joint positions (Exponential Moving Average)
    smooth_column = 0.0
    smooth_shoulder = 0.0
    smooth_forearm = 0.0
    smooth_wrist = 0.0
    smooth_gripper = -0.05

    ALPHA = 0.15  # Low-pass filter smoothing coefficient (0.15 = buttery smooth)
    gesture_name = "Neutral"

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
                            extended = count_extended_fingers(lm)
                            total_extended = sum(extended)

                            wrist = lm[0]
                            index_tip = lm[8]
                            thumb_tip = lm[4]

                            # Distance between Thumb Tip (4) and Index Tip (8)
                            pinch_dist = np.hypot((thumb_tip.x - index_tip.x) * w, (thumb_tip.y - index_tip.y) * h)

                            # Gesture State Machine Logic
                            if pinch_dist < 40 or total_extended == 0:
                                gesture_name = "✊ FIST / PINCH: GRASP PUCK"
                                raw_gripper = 0.0
                                color_bg = (0, 0, 255)  # Red HUD

                            elif total_extended >= 4:
                                gesture_name = "🖐️ OPEN PALM: RELEASE PUCK"
                                raw_gripper = -0.05
                                color_bg = (0, 255, 0)  # Green HUD

                                # Map hand position smoothly
                                hand_x = np.clip(wrist.x, 0.15, 0.85)
                                raw_column = float(np.interp(hand_x, [0.15, 0.85], [-1.8, 1.8]))

                                hand_y = np.clip(wrist.y, 0.2, 0.8)
                                raw_shoulder = float(np.interp(hand_y, [0.2, 0.8], [0.02, -0.14]))

                            elif total_extended == 2 and extended[1] == 1 and extended[2] == 1:
                                gesture_name = "✌️ PEACE: ALIGN OVER CONVEYOR"
                                raw_column = -0.32
                                raw_forearm = 0.75
                                raw_shoulder = 0.0
                                color_bg = (255, 0, 255)  # Magenta HUD

                            elif total_extended == 3 and extended[1] == 1 and extended[2] == 1 and extended[3] == 1:
                                gesture_name = "👌 OK: ALIGN OVER TABLE"
                                raw_column = 0.785
                                raw_forearm = 0.85
                                raw_shoulder = 0.0
                                color_bg = (255, 255, 0)  # Cyan HUD

                            elif extended[1] == 1:  # Pointing Index Finger
                                gesture_name = "☝️ POINTING: DIRECTIONAL TRACKING"
                                hand_x = np.clip(index_tip.x, 0.15, 0.85)
                                raw_column = float(np.interp(hand_x, [0.15, 0.85], [-1.8, 1.8]))

                                hand_y = np.clip(index_tip.y, 0.2, 0.8)
                                raw_shoulder = float(np.interp(hand_y, [0.2, 0.8], [0.02, -0.14]))
                                color_bg = (0, 165, 255)  # Orange HUD

                            # Apply EMA Low-Pass Filter for Ultra-Smooth Motion
                            smooth_column = ALPHA * raw_column + (1 - ALPHA) * smooth_column
                            smooth_shoulder = ALPHA * raw_shoulder + (1 - ALPHA) * smooth_shoulder
                            smooth_forearm = ALPHA * raw_forearm + (1 - ALPHA) * smooth_forearm
                            smooth_wrist = ALPHA * raw_wrist + (1 - ALPHA) * smooth_wrist
                            smooth_gripper = ALPHA * raw_gripper + (1 - ALPHA) * smooth_gripper

                            # Draw Visual HUD Overlay
                            cv2.rectangle(frame, (10, 10), (w - 10, 60), color_bg, -1)
                            cv2.putText(frame, gesture_name, (20, 45),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                            # Status Bars
                            cv2.putText(frame, f"Base Column: {smooth_column:.2f} rad", (20, 90),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            cv2.putText(frame, f"Z-Height:    {smooth_shoulder:.3f} m", (20, 120),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            cv2.putText(frame, f"Gripper:     {'CLOSED' if smooth_gripper > -0.02 else 'OPEN'}", (20, 150),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if smooth_gripper <= -0.02 else (0, 0, 255), 2)

                    cv2.imshow("SCARA AI Hand Gesture Teleoperation", frame)
                    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                        break

            # Update SCARA actuators in MuJoCo with smoothed targets
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
