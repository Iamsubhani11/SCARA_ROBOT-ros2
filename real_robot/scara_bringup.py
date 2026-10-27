#!/usr/bin/env python3
"""
First start of the real robot, step by step - run this ONCE after you have built and wired it
(and again whenever you change the wiring). It checks one thing at a time and tells you in plain
words what to fix:

  1. the Arduino answers on the USB port (firmware flashed, right port, right speed)
  2. homing: on the limit switches (K) - or "zero here" (H) for a robot without switches
  3. every joint, one at a time: it moves a little and you say if it went the right way.
     A wrong way is saved (~/.scara_twin_hardware.yaml) - the app, the website, the webcam cell
     and the digital twin all use it from then on
  4. the gripper opens and closes
  5. the conveyor relay switches the belt on and off
  6. the digital twin: a test move is sent the way the app sends it, and the robot must report
     arriving at the same joint values as the simulation

    python3 real_robot/scara_bringup.py --port /dev/ttyACM0            # robot with limit switches
    python3 real_robot/scara_bringup.py --port /dev/ttyACM0 --no-switches   # arm straight out, Z at 150 mm first
    python3 real_robot/scara_bringup.py --fake --yes                   # try it with the software robot

The robot has no encoders: it only counts the steps it sent, so this program cannot see a motor
that skips or does not turn at all - that is why it asks you to watch each joint.
A report is written to ~/scara_bringup_report.txt.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import math
import os
import time

import scara_twin_protocol as P

# (index in the firmware status, name, unit, nudge, the part that moves). Every joint axis in the
# URDF is +z: a positive change turns anticlockwise seen from above (the Z axis: goes up)
JOINTS = [
    (0, 'column_joint', 'deg', 10.0, 'the whole arm'),
    (1, 'forearm_joint', 'deg', 10.0, 'the forearm (second arm)'),
    (2, 'wrist_joint', 'deg', 15.0, 'the gripper'),
    (3, 'shoulder_joint', 'mm', 10.0, 'the arm'),
]


def expected(name, part, dq):
    """what the person should see when the simulation's joint changes by dq"""
    if name == 'shoulder_joint':
        return f'{part} goes {"UP" if dq > 0 else "DOWN"}'
    return f'{part} turns {"ANTICLOCKWISE" if dq > 0 else "CLOCKWISE"} seen from above'
FIX = {
    'port': ['Is the USB cable in? Which port: ls /dev/ttyACM* /dev/ttyUSB* (CH340 clones are /dev/ttyUSB0)',
             'Permission: sudo usermod -aG dialout $USER, then log out and in again',
             'Is the firmware flashed? Arduino IDE -> real_robot/firmware/scara_twin/scara_twin.ino -> Upload',
             'Close the Arduino Serial Monitor - only one program can use the port'],
    'home': ['Limit switches: normally closed, between the pin and GND (J1 D11, J2 D10, J3 D9, Z A3)',
             'Press each switch by hand and send ? in the Serial Monitor - the status must change',
             'No switches on your robot? Flash with #define USE_LIMIT_SWITCHES 0 and run with --no-switches'],
    'move': ['Driver not in its slot the right way round, or the motor cable in the wrong header',
             'Turn the A4988 current screw up a little (the motor hums but does not turn = too little current)',
             '12 V supply on the shield screw terminal? (USB alone powers only the Arduino)',
             'A jumper from EN to GND on the shield keeps the drivers on; D8 must not be cut'],
    'grip': ['Servo signal on A0 ("Abort"), power from a 5-6 V buck converter, grounds together',
             'Put a cube between the fingers, send G 120, G 100, G 90 ... and set SERVO_CLOSED_DEG in the firmware'],
    'belt': ['Relay IN pin: A1 ("Hold") on a robot WITH limit switches, D10 (Y end-stop header, like the Mechamind build) WITHOUT',
             'Belt on when it should be off? Your relay board switches on with LOW: set RELAY_ACTIVE_LOW true',
             'Relay board VCC to 5 V, GND to GND; the belt motor gets its own supply through the relay contacts'],
}


class Report:
    def __init__(self, auto):
        self.auto, self.lines, self.bad = auto, [], []

    def say(self, text=''):
        print(text, flush=True)
        self.lines.append(text)

    def ok(self, what):
        self.say(f'  OK    {what}')

    def fail(self, what, fix_key):
        self.say(f'  FIX   {what}')
        for f in FIX.get(fix_key, []):
            self.say(f'        - {f}')
        self.bad.append(what)

    def ask(self, question):
        if self.auto:
            self.say(f'  ?     {question} -> yes (--yes)')
            return True
        while True:
            a = input(f'  ?     {question} [y/n] ').strip().lower()
            if a in ('y', 'yes', 'n', 'no'):
                self.lines.append(f'  ?     {question} -> {a}')
                return a.startswith('y')


def wait_status(link, secs, cond=lambda st: True):
    t0 = time.time()
    while time.time() - t0 < secs:
        st = link.status()
        if st and cond(st):
            return st
        time.sleep(0.05)
    return None


def move_hw(link, hw, secs=15.0, tol=(0.3, 0.3, 0.5, 0.3)):
    """send a firmware target (deg, deg, deg, mm, grip %) and wait until the robot reports it"""
    link.write('M ' + ' '.join(f'{v:.2f}' for v in hw))
    time.sleep(0.3)
    st = wait_status(link, secs, lambda s: not s[2] and all(abs(s[1][i] - hw[i]) <= tol[i] for i in range(4)))
    return st


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--fake', action='store_true', help='the software robot (no hardware)')
    ap.add_argument('--no-switches', action='store_true', help='robot without limit switches: zero here (H)')
    ap.add_argument('--yes', action='store_true', help='answer yes to every question (tests)')
    ap.add_argument('--report', default=os.path.join(os.path.expanduser('~'), 'scara_bringup_report.txt'))
    a = ap.parse_args()
    R = Report(a.yes)
    msgs = []
    R.say('SCARA bring-up - one step at a time. Keep a hand near the power switch.\n')

    # ---- 1. the Arduino answers
    R.say('1. Connection')
    try:
        link = P.open_link(a.port, fake=a.fake, on_message=lambda k, t: msgs.append((k, t)))
    except Exception as e:                       # no such port, no permission
        R.fail(f'cannot open {a.port}: {e}', 'port')
        return finish(R, a)
    link.write('?')
    st = wait_status(link, 6.0)
    if not st:
        R.fail(f'the robot does not answer on {"the software robot" if a.fake else a.port} (no status in 6 s)', 'port')
        return finish(R, a)
    R.ok('the firmware answers' + (f': {msgs[0][1]}' if msgs else ''))

    # ---- 2. homing
    R.say('\n2. Homing')
    if a.no_switches:
        R.ask('Is the arm straight out (both arms in one line, gripper square) with Z at 150 mm?') or R.say('  (put it there first)')
        link.zero_here()
    else:
        R.say('  homing on the limit switches - every axis moves to its switch, then parks (up to 2 minutes)')
        link.home()
    st = wait_status(link, 150.0, lambda s: s[3] and not s[2])
    if not st:
        R.fail('homing did not finish', 'home' if not a.no_switches else 'port')
        return finish(R, a)
    R.ok(f'homed - firmware pose {[round(v, 1) for v in st[1][:4]]}')
    home = list(st[1][:4]) + [0.0]

    # ---- 3. every joint
    R.say('\n3. Joints - watch the robot')
    for i, name, unit, step, what in JOINTS:
        lo, hi = P.HW_LIMITS[i]
        d = step if home[i] + step <= hi - 1 else -step       # away from the soft limit
        tgt = list(home)
        tgt[i] += d
        R.say(f'  {name}: {"+" if d > 0 else "-"}{abs(d):g} {unit}')
        if not move_hw(link, tgt):
            R.fail(f'{name}: the firmware did not report arriving', 'move')
            continue
        moved = R.ask(f'Did it move? ({abs(d):g} {unit})')
        if not moved:
            R.fail(f'{name} did not move', 'move')
        else:
            dq = link.map.from_hw(tgt)[name] - link.map.from_hw(home)[name]     # the same move in the simulation
            right = R.ask(f'Was it this way: {expected(name, what, dq)}?')
            if right:
                R.ok(f'{name} moves the right way')
            else:
                key, val = link.map.flip(name)
                where = P.save_map_change(key, val)
                R.ok(f'{name} turned the other way - saved {key}: {val} in {where} (every program uses it now)')
        move_hw(link, home)

    # ---- 4. gripper
    R.say('\n4. Gripper')
    move_hw(link, home[:4] + [100.0])
    if R.ask('Did the gripper close?'):
        R.ok('gripper closes')
    else:
        R.fail('gripper does not close', 'grip')
    move_hw(link, home[:4] + [0.0])
    if R.ask('Did it open again?'):
        R.ok('gripper opens')
    else:
        R.fail('gripper does not open', 'grip')

    # ---- 5. conveyor relay
    R.say('\n5. Conveyor')
    link.conveyor(True)
    ok_on = wait_status(link, 3.0, lambda s: link.conveyor_on) is not None
    on_seen = R.ask('Is the belt running now?')
    link.conveyor(False)
    time.sleep(0.5)
    off_seen = R.ask('Has it stopped?')
    if ok_on and on_seen and off_seen:
        R.ok('the relay switches the belt')
    else:
        R.fail('the belt does not follow the relay', 'belt')

    # ---- 6. the digital twin: the app's units in, the same joints out
    R.say('\n6. Digital twin check (the same conversion the app and the website use)')
    q = {'column_joint': math.radians(20), 'forearm_joint': math.radians(-30), 'wrist_joint': math.radians(25),
         'shoulder_joint': -0.03, 'left_finger_joint': 0.0}
    link.send_joints(q, min_change=0.0)
    hw = link.map.to_hw(q)
    st = wait_status(link, 15.0, lambda s: not s[2] and all(abs(s[1][k] - hw[k]) < 0.3 for k in range(4)))
    got = link.joints() if st else None
    err = max(abs(got[k] - q[k]) for k in ('column_joint', 'forearm_joint', 'wrist_joint')) if got else None
    zerr = abs(got['shoulder_joint'] - q['shoulder_joint']) if got else None
    if got and err < math.radians(0.5) and zerr < 0.0005:
        R.ok(f'robot reports the twin target: worst {math.degrees(err):.2f} deg, Z {zerr * 1000:.2f} mm')
        if R.ask('Does the real arm look like the 3-D arm at column 20, elbow -30, wrist 25 deg, 30 mm down?'):
            R.ok('the real arm matches the simulation')
        else:
            R.fail('real arm and simulation do not match - check the offsets (*_offset_deg, z_zero_mm) in '
                   'config/twin_hardware.yaml, or redo the direction check', 'move')
    else:
        R.fail('the robot did not report the twin target', 'move')
    move_hw(link, home)
    link.close()
    return finish(R, a)


def finish(R, a):
    R.say('\n' + ('ALL BRING-UP CHECKS OK - next: python3 server/scara_twin_server.py --port ' + (a.port if not a.fake else '<port>')
                  + '  (then Robot tab -> Home switches -> Twin)' if not R.bad else
                  f'{len(R.bad)} THING(S) TO FIX (see FIX above) - then run this again'))
    try:
        with open(a.report, 'w') as fh:
            fh.write('\n'.join(R.lines) + '\n')
        print(f'(report: {a.report})')
    except OSError:
        pass
    return 1 if R.bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
