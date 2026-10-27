#!/usr/bin/env python3
"""
Two-hand gesture control of the SCARA in MuJoCo (same gestures as Gazebo).

    LEFT hand : left/right = base column,  up/down = Z height
    RIGHT hand: left/right = forearm,      tilt = wrist,  pinch thumb+index = close gripper
    FIST      : freeze that hand's joints      Hand out of view: hold
    In the camera window:  q / Esc = quit    SPACE = emergency stop on/off    m = safety mode
    Pick a puck: open hand over it (it fits between the open fingers), lower Z, pinch, lift.

    python3 gesture_control/gesture_control_mujoco.py [--camera 0] [--safety safe|normal|off]

Safety mode (default "safe", see common/scara_safety.py): slow joints and a gentle
gripper, pick assist (the open gripper centres itself over a puck when you lower it
close to one), a guard that never lets a finger press down on a puck, a table floor
and an emergency stop.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import os
import sys
import time

import cv2
import mediapipe as mp
import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gesture_core as G   # noqa: E402
S = G.S
import scara_safety as SF   # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', type=int, default=0)
    SF.add_args(ap)
    args = ap.parse_args()
    model = mujoco.MjModel.from_xml_path(S.pkg_file('mujoco', 'models', 'scara_manual_mujoco.xml'))   # table + pucks, like the video
    data = mujoco.MjData(model)
    act = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(model.nu)}
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f'[ERROR] cannot open webcam {args.camera}')
        return
    hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, model_complexity=1,
                                     min_detection_confidence=0.6, min_tracking_confidence=0.6)
    draw = mp.solutions.drawing_utils
    mapper = G.GestureMapper()
    filt = SF.SafetyFilter(args.safety, assist=not args.no_assist)
    print(__doc__)
    last = time.time()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)
            now = time.time()
            dt = min(0.1, now - last)
            last = now
            res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            found = []
            if res.multi_hand_landmarks:
                for lms, handed in zip(res.multi_hand_landmarks, res.multi_handedness or [None] * 8):
                    found.append((G.np.array([[p.x, p.y, p.z] for p in lms.landmark]),
                                  handed.classification[0].label if handed else None))
                    draw.draw_landmarks(frame, lms, mp.solutions.hands.HAND_CONNECTIONS)
            _, grip = mapper.update(found, now, dt)
            qm = [float(data.joint(n).qpos[0]) for n in S.ARM_JOINTS]
            (col, z, fore, wrist), grip = filt.filter(mapper.arm_vector(), grip, dt, q_meas=qm,
                                                      grip_meas=float(data.joint('left_finger_joint').qpos[0]),
                                                      pucks=SF.mujoco_pucks(model, data))
            mapper.note = f'[{filt.mode}] ' + (filt.note or 'ready')
            data.ctrl[act['act_column']] = col
            data.ctrl[act['act_shoulder']] = z
            data.ctrl[act['act_forearm']] = fore
            data.ctrl[act['act_wrist']] = wrist
            data.ctrl[act['act_left_finger']] = grip     # both fingers, same sign
            data.ctrl[act['act_right_finger']] = grip
            # run physics in real time for the duration of this camera frame
            t_end = data.time + min(dt, 0.1)
            while data.time < t_end:
                mujoco.mj_step(model, data)
            viewer.sync()
            G.draw_hud(frame, mapper)
            cv2.imshow('SCARA Hand Control', frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break
            if k == ord(' '):
                filt.estop = not filt.estop
                print('EMERGENCY STOP' if filt.estop else 'stop released')
            if k == ord('m'):
                filt.set_mode(SF.next_mode(filt.mode))
                print('safety mode:', filt.mode)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
