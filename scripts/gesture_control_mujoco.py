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
    print("=============================================================")
    print("=== SCARA Robot AI Hand Gesture Controller (MuJoCo) ===")
    print("=============================================================")
    print(" Gesture Controls:")
    print("   🖐️  Move Hand Left / Right  : Base Column Rotation")
    print("   🖐️  Move Hand Up / Down     : Z-Axis Height (Elevation)")
    print("   ✌️  Finger Pinch / Fist      : Close Gripper (Grasp Puck)")
    print("   🖐️  Open Palm (5 Fingers)    : Open Gripper (Release Puck)")
    print("=============================================================\n")

    xml_path = get_scene_xml_path()
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Initialize MediaPipe Hand Tracking
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    mp_draw = mp.solutions.drawing_utils

    # Open Camera Video Capture
    cap = cv2.VideoCapture(0)
    camera_available = cap.isOpened()

    if not camera_available:
        print("[WARNING] Webcam camera index 0 not available.")
        print("[INFO] Starting Gesture Simulation & Control Interface...")

    # Default Target Values
    column_target = 0.0
    shoulder_target = 0.0
    forearm_target = 0.0
    wrist_target = 0.0
    gripper_target = -0.05
    gesture_name = "Neutral"

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            if camera_available:
                ret, frame = cap.read()
                if ret:
                    # Flip frame horizontally for natural mirror view
                    frame = cv2.flip(frame, 1)
                    h, w, _ = frame.shape
                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    results = hands.process(rgb_frame)

                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                            # Get Wrist (0), Index Tip (8), Thumb Tip (4), Middle Tip (12)
                            wrist = hand_landmarks.landmark[0]
                            index_tip = hand_landmarks.landmark[8]
                            thumb_tip = hand_landmarks.landmark[4]

                            # 1. Base Column Rotation (Hand X position: 0.1 to 0.9 -> -2.0 to 2.0 rad)
                            hand_x = np.clip(wrist.x, 0.1, 0.9)
                            column_target = float(np.interp(hand_x, [0.1, 0.9], [-1.8, 1.8]))

                            # 2. Z-Axis Elevation (Hand Y position: 0.2 to 0.8 -> 0.02 to -0.14 m)
                            hand_y = np.clip(wrist.y, 0.2, 0.8)
                            shoulder_target = float(np.interp(hand_y, [0.2, 0.8], [0.02, -0.14]))

                            # 3. Forearm Angle (Index Tip Y position)
                            index_y = np.clip(index_tip.y, 0.2, 0.8)
                            forearm_target = float(np.interp(index_y, [0.2, 0.8], [-1.2, 1.5]))

                            # 4. Gripper Control (Pinch distance between Thumb & Index Tip)
                            pinch_dist = np.hypot((thumb_tip.x - index_tip.x) * w, (thumb_tip.y - index_tip.y) * h)

                            if pinch_dist < 40:
                                gripper_target = 0.0
                                gesture_name = "FIST / PINCH (GRASP)"
                            else:
                                gripper_target = -0.05
                                gesture_name = "OPEN PALM (RELEASE)"

                            # Draw HUD Info on OpenCV Window
                            cv2.putText(frame, f"Gesture: {gesture_name}", (20, 40),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            cv2.putText(frame, f"Base Column: {column_target:.2f} rad", (20, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            cv2.putText(frame, f"Z-Height: {shoulder_target:.3f} m", (20, 110),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                            cv2.putText(frame, f"Pinch Dist: {pinch_dist:.1f} px", (20, 140),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                    cv2.imshow("SCARA AI Hand Gesture Teleoperation", frame)
                    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                        break

            # Update SCARA actuators in MuJoCo
            data.ctrl[0] = column_target
            data.ctrl[1] = shoulder_target
            data.ctrl[2] = forearm_target
            data.ctrl[3] = wrist_target
            data.ctrl[4] = gripper_target
            data.ctrl[5] = gripper_target

            # Step physics simulation forward
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

    if camera_available:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
