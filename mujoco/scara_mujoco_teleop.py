#!/usr/bin/env python3

import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import os
import sys
import time
import select
import termios
import tty
import math
import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scara_scene as S          # noqa: E402
import scara_kinematics as K     # noqa: E402
import scara_safety as SF        # noqa: E402

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
   1     : Above the middle blue puck (fingers open around it)
   2     : Lower around the puck (then G to grab, W to lift)
   3     : Carry to a free spot on the table
   R     : RESET to Home Position

 Safety (common/scara_safety.py, start with --safety safe|normal|off):
   SPACE : EMERGENCY STOP on / off
   M     : next safety mode (safe -> normal -> off)
   In safe / normal mode the gripper centres itself over a puck when you lower
   it close to one, and a finger never presses down on a puck.

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
    xml_path = S.pkg_file('mujoco', 'models', 'scara_manual_mujoco.xml')   # table + pucks, no conveyor
    if not os.path.exists(xml_path):
        raise FileNotFoundError(xml_path)
    return xml_path

def main():
    import argparse
    ap = argparse.ArgumentParser(description='SCARA keyboard control (MuJoCo)')
    SF.add_args(ap)
    args = ap.parse_args()
    filt = SF.SafetyFilter(args.safety, assist=not args.no_assist)
    print(HELP_MSG)

    xml_path = get_scene_xml_path()
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Initial targets
    column_target = 0.0
    shoulder_target = 0.0
    forearm_target = 0.0
    wrist_target = 0.0
    gripper_target = S.GRIPPER_OPEN
    belt_speed = 0.0

    STEP_ANGLE = 0.05
    STEP_Z = 0.008

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall0, sim0 = time.time(), data.time
        last_t = time.time()
        last_note = ''
        while viewer.is_running():
            key = get_key()
            if key == ' ':
                filt.estop = not filt.estop
                print('\r\n*** EMERGENCY STOP ***' if filt.estop else '\r\nstop released')
            elif key.lower() == 'm':
                filt.set_mode(SF.next_mode(filt.mode))
                print(f'\r\nsafety mode: {filt.mode}')

            if key.lower() == 'a':          # robot faces you: 'a' = towards your left
                column_target = max(-2.0, column_target - STEP_ANGLE)
                print(f"[Joint] Base Column: {column_target:.3f} rad")
            elif key.lower() == 'd':
                column_target = min(2.0, column_target + STEP_ANGLE)
                print(f"[Joint] Base Column: {column_target:.3f} rad")
            elif key.lower() == 'w':
                shoulder_target = min(0.02, shoulder_target + STEP_Z)
                print(f"[Joint] Z-Axis Height: {shoulder_target:.3f} m")
            elif key.lower() == 's':
                shoulder_target = max(-0.15, shoulder_target - STEP_Z)
                print(f"[Joint] Z-Axis Height: {shoulder_target:.3f} m")
            elif key.lower() == 'j':
                forearm_target = max(-2.0, forearm_target - STEP_ANGLE)
                print(f"[Joint] Forearm Angle: {forearm_target:.3f} rad")
            elif key.lower() == 'l':
                forearm_target = min(2.0, forearm_target + STEP_ANGLE)
                print(f"[Joint] Forearm Angle: {forearm_target:.3f} rad")
            elif key.lower() == 'i':
                wrist_target = min(4.71, wrist_target + STEP_ANGLE)
                print(f"[Joint] Wrist Angle: {wrist_target:.3f} rad")
            elif key.lower() == 'k':
                wrist_target = max(-4.71, wrist_target - STEP_ANGLE)
                print(f"[Joint] Wrist Angle: {wrist_target:.3f} rad")
            elif key.lower() == 'g':
                gripper_target = S.GRIP_CLOSE
                print("[Gripper] CLOSED (Grasping)")
            elif key.lower() == 'o':
                gripper_target = S.GRIPPER_OPEN
                print("[Gripper] OPENED (Released)")
            elif key == '1':
                # Above the middle blue puck, fingers wide open around it (real IK)
                px, py, pyaw = dict(S.MANUAL_PUCKS)['puck_blue_2']
                column_target, shoulder_target, forearm_target, wrist_target = K.arm_pose(
                    px, py, S.Z_PRE_PICK_TABLE, yaw=pyaw, symmetry=math.pi)
                gripper_target = S.GRIP_PREGRASP
                print("[Preset 1] Above the middle blue puck, gripper open")
            elif key == '2':
                # Lower the Z axis until the fingers are around the puck body
                shoulder_target = S.Z_PICK_TABLE
                print("[Preset 2] Lowered around the puck - press G to grab")
            elif key == '3':
                # Carry it to a free spot on the table (left of the robot, seen from you)
                fx, fy = S.cell_xy(0.0, -0.30)
                column_target, shoulder_target, forearm_target, wrist_target = K.arm_pose(fx, fy, S.Z_TRAVEL)
                print("[Preset 3] Above a free spot on the table - S to lower, O to let go")
            elif key.lower() == 'r':
                column_target = 0.0
                shoulder_target = 0.0
                forearm_target = 0.0
                wrist_target = 0.0
                gripper_target = S.GRIPPER_OPEN
                print("[Reset] Returned to Home Position")
            elif key.lower() == 'q' or key == '\x1b':
                print("Exiting teleoperation...")
                break

            # Set controls (through the safety filter)
            t = time.time()
            dt, last_t = t - last_t, t
            qm = [float(data.joint(n).qpos[0]) for n in S.ARM_JOINTS]
            q, g = filt.filter([column_target, shoulder_target, forearm_target, wrist_target], gripper_target, dt,
                               q_meas=qm, grip_meas=float(data.joint('left_finger_joint').qpos[0]),
                               pucks=SF.mujoco_pucks(model, data))
            if filt.estop:
                column_target, shoulder_target, forearm_target, wrist_target = q
            else:
                column_target, shoulder_target, forearm_target, wrist_target = filt.clamp_back(
                    [column_target, shoulder_target, forearm_target, wrist_target])
            if filt.note and filt.note != last_note:
                print(f'\r\n[{filt.mode}] {filt.note}')
            last_note = filt.note
            data.ctrl[0], data.ctrl[1], data.ctrl[2], data.ctrl[3] = q
            data.ctrl[4] = g
            data.ctrl[5] = g
            if model.nu > 6:
                data.ctrl[6] = belt_speed

            # Step physics in real time (the key read above waits up to 50 ms)
            now = time.time()
            n = 0
            while data.time - sim0 < now - wall0 and n < 50:
                mujoco.mj_step(model, data)
                n += 1
            if n == 50:
                wall0, sim0 = now, data.time
            viewer.sync()

if __name__ == '__main__':
    main()
