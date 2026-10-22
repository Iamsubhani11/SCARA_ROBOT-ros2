#!/usr/bin/env python3
"""
MuJoCo digital twin of the real SCARA - no ROS needed, just the USB cable.

    python3 scripts/mujoco_twin.py --port /dev/ttyACM0            # the simulated arm FOLLOWS the real robot
    python3 scripts/mujoco_twin.py --port /dev/ttyACM0 --drive    # the viewer sliders DRIVE the real robot
    python3 scripts/mujoco_twin.py --fake [--drive]               # try it without hardware

    --homing switches|zero|none   (default switches: home on the 4 limit switches like the original
                                   HowToMechatronics build; zero: no switches, the arm is at the ready pose -> H)

Follow mode is the MechaMind "potentiometer twin" idea: move the real robot
(TEACH_WITH_POTS knobs, or its own GUI) and the model moves with it.
Drive mode: open the viewer's "Control" panel and move act_column,
act_shoulder, act_forearm, act_wrist, act_left_finger - the real robot
copies the simulated one.  The colour sorter can drive the real robot too:
    python3 scripts/launch_mujoco.py --auto --twin /dev/ttyACM0
"""
import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_scene as S                              # noqa: E402
from scara_twin_protocol import open_link            # noqa: E402

XML = S.pkg_file('urdf', 'scara_manual_mujoco.xml')   # table + pucks
ACT = {'column_joint': 'act_column', 'shoulder_joint': 'act_shoulder', 'forearm_joint': 'act_forearm',
       'wrist_joint': 'act_wrist', 'left_finger_joint': 'act_left_finger', 'right_finger_joint': 'act_right_finger'}


def main():
    ap = argparse.ArgumentParser(description='MuJoCo digital twin of the real SCARA')
    ap.add_argument('--port', default=None, help='serial port of the Arduino, e.g. /dev/ttyACM0 or COM5')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--fake', action='store_true', help='software robot, no hardware')
    ap.add_argument('--drive', action='store_true', help='the simulation drives the real robot')
    ap.add_argument('--homing', default='switches', choices=['switches', 'zero', 'none'])
    args = ap.parse_args()
    if not args.port and not args.fake:
        ap.error('give --port /dev/ttyACM0 (or --fake to try without hardware)')

    link = open_link(args.port, args.baud, fake=args.fake)
    if args.homing == 'switches':
        link.home()
    elif args.homing == 'zero':
        link.zero_here()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    act = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(model.nu)}
    print(__doc__)
    print('Mode:', 'DRIVE (sim -> real)' if args.drive else 'FOLLOW (real -> sim)')
    next_io = 0.0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            t0 = time.time()
            if data.time >= next_io:
                next_io = data.time + 0.05            # 20 Hz, same as the firmware's status rate
                if args.drive:
                    q = {j: float(data.ctrl[act[a]]) for j, a in ACT.items()}
                    link.send_joints(q)
                else:
                    real = link.joints()
                    if real is not None:
                        with viewer.lock():
                            for j, a in ACT.items():
                                data.ctrl[act[a]] = real[j]
                real = link.joints()
                if real is not None:
                    err = max(abs(real[j] - float(data.joint(j).qpos[0])) for j in S.ARM_JOINTS)
                    st = link.status()
                    print(f'\rreal: col {real["column_joint"]:+.2f} z {real["shoulder_joint"]:+.3f} '
                          f'fore {real["forearm_joint"]:+.2f} wrist {real["wrist_joint"]:+.2f}  '
                          f'sim-real {err:.3f}  {"moving" if st[2] else "idle  "} {"homed" if st[3] else "NOT HOMED"}',
                          end='', flush=True)
            with viewer.lock():
                mujoco.mj_step(model, data)
            viewer.sync()
            dt = model.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    link.close()


if __name__ == '__main__':
    main()
