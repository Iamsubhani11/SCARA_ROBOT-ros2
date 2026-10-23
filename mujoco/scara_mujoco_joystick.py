#!/usr/bin/env python3
"""
Drive the MuJoCo SCARA (manual-control table) with an Xbox (or any) gamepad.

    python3 mujoco/scara_mujoco_joystick.py

No ROS needed: the pad is read straight from /dev/input/js0.  Options:
    --device /dev/input/js1     another pad
    --joy-topic                 read sensor_msgs/Joy from /joy (ros2 run joy joy_node) instead
    --layout gamecontroller     pads whose right stick is on axes 2/3
    --twin /dev/ttyACM0         the real robot copies the simulated one ("--twin fake" to try)
    --safety safe|normal|off    safety mode (default safe: slow, pick assist, guards - common/scara_safety.py)
    --no-assist                 no automatic centring over a puck

Controls (the robot faces you):
  left stick  X : swing the arm left / right        Y : Z up / down
  right stick X : forearm left / right              Y : turn the wrist
  A : close gripper    B : open gripper    Y : home    hold LB : slow   hold RB : fast
  BACK : emergency stop on / off            START : next safety mode
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_scene as S                                  # noqa: E402
from scara_joy_core import JoyMapper, JsDevice          # noqa: E402
import scara_safety as SF                               # noqa: E402

XML = S.pkg_file('mujoco', 'models', 'scara_manual_mujoco.xml')   # table + pucks, no conveyor


class RosJoy:
    """Optional /joy subscriber (only used with --joy-topic)."""

    def __init__(self):
        import rclpy
        from sensor_msgs.msg import Joy
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('scara_mujoco_joystick')
        self.data = None
        self.node.create_subscription(Joy, '/joy', self._cb, 10)

    def _cb(self, msg):
        self.data = (list(msg.axes), list(msg.buttons))

    def state(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)
        return self.data


def main():
    ap = argparse.ArgumentParser(description='SCARA gamepad control (MuJoCo)')
    ap.add_argument('--device', default=None)
    ap.add_argument('--joy-topic', action='store_true')
    ap.add_argument('--layout', default='xbox', choices=['xbox', 'gamecontroller'])
    ap.add_argument('--twin', default=None, help='serial port of the real robot, or "fake"')
    SF.add_args(ap)
    args = ap.parse_args()

    source = None
    if args.joy_topic:
        source = RosJoy()
        print('Reading the pad from /joy')
    else:
        try:
            dev = JsDevice(args.device)
            print(f'Pad: {dev.name} on {dev.path}')
            source = dev
        except (FileNotFoundError, OSError) as e:
            print(f'[ERROR] {e}')
            return 1
    twin = None
    if args.twin:
        from scara_twin_protocol import open_link
        twin = open_link(None if args.twin == 'fake' else args.twin, fake=args.twin == 'fake')
        twin.zero_here()

    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    act = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(model.nu)}
    mapper = JoyMapper(args.layout)
    filt = SF.SafetyFilter(args.safety, assist=not args.no_assist)
    print(__doc__.split('Controls')[1])
    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall0, sim0 = time.time(), data.time
        last = time.time()
        last_twin = 0.0
        last_print = 0.0
        while viewer.is_running():
            now = time.time()
            dt, last = now - last, now
            st = source.state()
            if st is not None:
                q, grip = mapper.update(st[0], st[1], dt)
                if mapper.message == 'estop':
                    filt.estop = not filt.estop
                    print('\nEMERGENCY STOP' if filt.estop else '\nstop released')
                elif mapper.message == 'mode':
                    filt.set_mode(SF.next_mode(filt.mode))
                    print('\nsafety mode:', filt.mode)
                with viewer.lock():
                    qm = [float(data.joint(n).qpos[0]) for n in S.ARM_JOINTS]
                    q, grip = filt.filter(q, grip, dt, q_meas=qm, grip_meas=float(data.joint('left_finger_joint').qpos[0]),
                                          pucks=SF.mujoco_pucks(model, data))
                mapper.q = filt.clamp_back(mapper.q)
                if filt.estop:
                    mapper.q = list(q)          # no wind-up while stopped
                with viewer.lock():
                    for name, v in zip(('act_column', 'act_shoulder', 'act_forearm', 'act_wrist'), q):
                        data.ctrl[act[name]] = v
                    data.ctrl[act['act_left_finger']] = grip
                    data.ctrl[act['act_right_finger']] = grip
                if twin and now - last_twin > 0.05:
                    last_twin = now
                    twin.send_joints({**dict(zip(S.ARM_JOINTS, q)), 'left_finger_joint': grip,
                                      'right_finger_joint': grip})
            # run the physics in real time (several 2 ms steps per frame)
            with viewer.lock():
                n = 0
                while data.time - sim0 < now - wall0 and n < 50:
                    mujoco.mj_step(model, data)
                    n += 1
                if n == 50:                      # computer too slow: do not try to catch up
                    wall0, sim0 = now, data.time
            viewer.sync()
            if now - last_print > 0.25 and st is not None:
                last_print = now
                print(f'\r[{filt.mode}] column {q[0]:+.2f}  Z {q[1]:+.3f}  forearm {q[2]:+.2f}  wrist {q[3]:+.2f}  '
                      f'gripper {"CLOSED" if grip < -0.005 else "open  "}  {filt.note:<48.48}', end='', flush=True)
            time.sleep(max(0.0, 0.016 - (time.time() - now)))
    print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
