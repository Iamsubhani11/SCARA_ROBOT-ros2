#!/usr/bin/env python3
"""
Compiles real_robot/firmware/scara_twin/scara_twin.ino on the PC (g++ with small Arduino /
AccelStepper / Servo mocks in tools/firmware_host_test) and checks the serial
protocol, in both builds:

  default (HowToMechatronics build, 4 limit switches): K homes on the (simulated)
    switches and parks like the original sketch, G drives the servo directly
  USE_LIMIT_SWITCHES=0 (no switches): moves refused before "zero here" (H), H sets the
    ready pose, a move is reached, soft limits clamp, F frees the motors (moves refused),
    L locks them again, C 1 / C 0 switch the conveyor relay, the status line reports it

    python3 tools/test_firmware_host.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T = os.path.join(ROOT, 'tools', 'firmware_host_test')
fails = []


def build(name, *defs):
    exe = os.path.join(tempfile.gettempdir(), name)
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', *defs, '-I', T,
                    '-include', 'Arduino.h', os.path.join(T, 'host_main.cpp'), '-o', exe], check=True)
    return exe


def run(exe, lines):
    script = '\n'.join(lines) + '\n'
    return subprocess.run([exe], input=script, capture_output=True, text=True, timeout=60).stdout.splitlines()


def check(name, ok):
    print(('PASS ' if ok else 'FAIL ') + name)
    if not ok:
        fails.append(name)


def P(out):
    return [[float(x) for x in l.split()[1:]] for l in out if l.startswith('P ')]


def near(v, want, tol=0.05):
    return all(abs(a - b) <= tol for a, b in zip(v, want))


exe = build('scara_twin_fw_host', '-DUSE_LIMIT_SWITCHES=0')
out = run(exe, ['0 M 10 20 30 100 50', '100 H', '100 M 30 -45 90 120 100', '4000 ?',
                '10 M 300 -200 0 -5 0', '6000 ?', '10 C 1', '100 ?', '10 F', '10 M 0 0 0 150 0',
                '100 ?', '10 L', '10 M 0 0 0 150 0', '5000 ?', '10 C 0', '100 ?'])
p = P(out)
check('moves refused before zero (H)', any(l.startswith('W not homed') for l in out))
check('H sets the ready pose', 'I ZERO SET HERE' in out)
check('reached target 30 -45 90 120 mm, gripper 100 %', any(near(v[:5], [30, -45, 90, 120, 100]) for v in p))
check('soft limits clamp 300 / -200 / -5 mm', any(near(v[:4], [266, -150, 0, 0]) for v in p))
check('C 1 switches the conveyor relay on', 'I CONVEYOR ON' in out and any(v[8] == 1 for v in p))
check('F frees the motors, moves refused', 'I MOTORS FREE' in out and any(l.startswith('W motors are free') for l in out)
      and any(v[7] == 0 for v in p))
check('L locks them, moves work again', 'I MOTORS LOCKED' in out and near(p[-2][:4], [0, 0, 0, 150]))
check('C 0 switches the relay off', 'I CONVEYOR OFF' in out and p[-1][8] == 0)
check('K refused without switches', not any(l.startswith('I HOMED') for l in out))

exe2 = build('scara_twin_fw_host_sw')
out2 = run(exe2, ['0 K', '10 ?', '100 M 10 20 30 120 0', '4000 ?', '10 G 95'])
p2 = P(out2)
check('default build: K homes on the switches', 'I HOMED' in out2)
check('after homing: arm straight, Z 100 mm (original moveTo 0 / 10000)', any(near(v[:4], [0, 0, 0, 100]) for v in p2))
check('then a move is reached', any(near(v[:4], [10, 20, 30, 120]) for v in p2))
check('G 95 drives the servo directly', 'I SERVO 95' in out2)
exe3 = build('scara_twin_fw_host_pots', '-DUSE_LIMIT_SWITCHES=0', '-DTEACH_WITH_POTS=1')
out3 = run(exe3, ['500 ?'])
check('TEACH_WITH_POTS=1 builds and follows the knobs', any(abs(v[0] - 88.5) < 1.0 for v in P(out3)))
print('ALL FIRMWARE TESTS PASSED' if not fails else f'{len(fails)} FAILED')
sys.exit(1 if fails else 0)
