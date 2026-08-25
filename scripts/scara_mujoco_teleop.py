#!/usr/bin/env python3

import os
import sys
import time
import select
import termios
import tty
import mujoco
import mujoco.viewer

HELP_MSG = """
============================================================
 SCARA Robot Manual Teleoperation & Picking (MuJoCo)
============================================================
 Joint Controls:
   A / D : Rotate Base Column Left / Right
   W / S : Move Z-Axis UP / DOWN
   J / L : Rotate Forearm Left / Right
   I / K : Rotate Wrist CW / CCW

 Gripper Controls:
   G     : CLOSE Gripper (Grasp Puck)
   O     : OPEN Gripper (Release Puck)

 Preset Action Keys:
   1     : Pre-Pick Pose (Over Conveyor Puck)
   2     : Lower Z-Axis onto Puck
   3     : Drop-Off Pose (Over Wooden Table)
   R     : RESET to Home Position

   Q / ESC : Exit Teleoperation
============================================================
"""

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def get_scene_xml_path():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xml_path = os.path.join(pkg_dir, 'urdf', 'scara_scene_mujoco.xml')
    if not os.path.exists(xml_path):
        xml_path = '/home/hemanthros/scara_description_ws/urdf/scara_scene_mujoco.xml'
    return xml_path

def main():
    print(HELP_MSG)

    xml_path = get_scene_xml_path()
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Initial targets
    column_target = 0.0
    shoulder_target = 0.0
    forearm_target = 0.0
    wrist_target = 0.0
    gripper_target = -0.05

    STEP_ANGLE = 0.05
    STEP_Z = 0.008

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            key = get_key()

            if key.lower() == 'a':
                column_target = min(2.0, column_target + STEP_ANGLE)
                print(f"[Joint] Base Column: {column_target:.3f} rad")
            elif key.lower() == 'd':
                column_target = max(-2.0, column_target - STEP_ANGLE)
                print(f"[Joint] Base Column: {column_target:.3f} rad")
            elif key.lower() == 'w':
                shoulder_target = min(0.02, shoulder_target + STEP_Z)
                print(f"[Joint] Z-Axis Height: {shoulder_target:.3f} m")
            elif key.lower() == 's':
                shoulder_target = max(-0.15, shoulder_target - STEP_Z)
                print(f"[Joint] Z-Axis Height: {shoulder_target:.3f} m")
            elif key.lower() == 'j':
                forearm_target = min(2.0, forearm_target + STEP_ANGLE)
                print(f"[Joint] Forearm Angle: {forearm_target:.3f} rad")
            elif key.lower() == 'l':
                forearm_target = max(-2.0, forearm_target - STEP_ANGLE)
                print(f"[Joint] Forearm Angle: {forearm_target:.3f} rad")
            elif key.lower() == 'i':
                wrist_target = min(4.71, wrist_target + STEP_ANGLE)
                print(f"[Joint] Wrist Angle: {wrist_target:.3f} rad")
            elif key.lower() == 'k':
                wrist_target = max(-4.71, wrist_target - STEP_ANGLE)
                print(f"[Joint] Wrist Angle: {wrist_target:.3f} rad")
            elif key.lower() == 'g':
                gripper_target = 0.0
                print("[Gripper] CLOSED (Grasping)")
            elif key.lower() == 'o':
                gripper_target = -0.05
                print("[Gripper] OPENED (Released)")
            elif key == '1':
                # Pre-Pick Pose over conveyor puck
                column_target = -0.32
                shoulder_target = 0.0
                forearm_target = 0.75
                wrist_target = 0.0
                gripper_target = -0.05
                print("[Preset 1] Pre-Pick Pose over Conveyor Track")
            elif key == '2':
                # Lower Z-Axis onto puck
                shoulder_target = -0.14
                print("[Preset 2] Lowering Z-Axis onto Yellow Puck")
            elif key == '3':
                # Drop-Off Pose over wooden table
                column_target = 0.785
                shoulder_target = 0.0
                forearm_target = 0.85
                wrist_target = 0.0
                print("[Preset 3] Drop-off Pose over Wooden Table Platform")
            elif key.lower() == 'r':
                column_target = 0.0
                shoulder_target = 0.0
                forearm_target = 0.0
                wrist_target = 0.0
                gripper_target = -0.05
                print("[Reset] Returned to Home Position")
            elif key.lower() == 'q' or key == '\x1b':
                print("Exiting teleoperation...")
                break

            # Set controls
            data.ctrl[0] = column_target
            data.ctrl[1] = shoulder_target
            data.ctrl[2] = forearm_target
            data.ctrl[3] = wrist_target
            data.ctrl[4] = gripper_target
            data.ctrl[5] = gripper_target

            # Step physics
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == '__main__':
    main()
