#!/usr/bin/env python3
"""Unit tests for gesture_control/gesture_core.py with synthetic MediaPipe hands."""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'gesture_control'))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import gesture_core as G   # noqa: E402
import scara_scene as S    # noqa: E402

OPEN = {0: (0, 0), 1: (-0.3, -0.25), 2: (-0.55, -0.45), 3: (-0.7, -0.65), 4: (-0.85, -0.85),
        5: (-0.35, -1.0), 6: (-0.37, -1.35), 7: (-0.38, -1.65), 8: (-0.38, -1.9),
        9: (-0.1, -1.05), 10: (-0.1, -1.45), 11: (-0.1, -1.75), 12: (-0.1, -2.0),
        13: (0.12, -1.0), 14: (0.13, -1.38), 15: (0.14, -1.65), 16: (0.15, -1.85),
        17: (0.32, -0.9), 18: (0.36, -1.2), 19: (0.39, -1.42), 20: (0.41, -1.6)}


def hand(cx, cy, scale=0.08, roll=0.0, kind='open', mirror=False):
    pts = dict(OPEN)
    if kind == 'fist':
        for mcp, tip in G.FINGERS:
            mx, my = OPEN[mcp]
            pts[tip] = (mx * 0.8, my * 0.55)
            pts[tip - 1] = (mx * 0.9, my * 0.7)
            pts[tip - 2] = (mx, my * 1.1)
    if kind == 'pinch':
        pts[4] = (-0.36, -1.82)
    arr = np.zeros((21, 3))
    c, s = math.cos(roll), math.sin(roll)
    for i, (x, y) in pts.items():
        if mirror:
            x = -x
        xr, yr = c * x - s * y, s * x + c * y
        arr[i] = (xr, yr, 0)
    arr[:, :2] *= scale
    arr[:, :2] += np.array([cx, cy]) - arr[[0, 5, 9, 17], :2].mean(axis=0)
    return arr


def run(mapper, hands, seconds, rate=30):
    t = getattr(mapper, '_t', 0.0)
    out = None
    for _ in range(int(seconds * rate)):
        t += 1.0 / rate
        out = mapper.update(hands, t, 1.0 / rate)
    mapper._t = t
    return out


def check(name, cond):
    print(('PASS ' if cond else 'FAIL ') + name)
    if not cond:
        global FAILED
        FAILED = True


FAILED = False

# 1. classification helpers
check('open hand is not a fist', not G.is_fist(hand(0.3, 0.5)))
check('fist detected', G.is_fist(hand(0.3, 0.5, kind='fist')))
check('fist detected when tilted 40 deg', G.is_fist(hand(0.3, 0.5, roll=0.7, kind='fist')))
check('tilted open hand is still open (old bug: count changed with tilt)', not G.is_fist(hand(0.3, 0.5, roll=0.7)))
check('pinch ratio small when pinching', G.pinch_ratio(hand(0.7, 0.5, kind='pinch')) < G.PINCH_CLOSE)
check('pinch ratio large when open', G.pinch_ratio(hand(0.7, 0.5)) > G.PINCH_OPEN)

# 2. left hand drives column + Z, right hand forearm, smoothly
m = G.GestureMapper()
cmd, grip = run(m, [(hand(0.10, 0.20), 'Left'), (hand(0.60, 0.5, mirror=True), 'Right')], 4.0)
check(f'left hand far left/top -> column near -1.8, arm swings to your left ({cmd["column"]:+.2f})', cmd['column'] < -1.6)
check(f'left hand top -> Z near top ({cmd["z"]:+.3f})', cmd['z'] > 0.0)
check(f'right hand at x=0.60 -> forearm negative ({cmd["forearm"]:+.2f})', cmd['forearm'] < -0.8)
check('gripper open with open hand', grip == S.GRIPPER_OPEN)

# 3. rate limit: a sudden hand jump never moves a joint faster than MAX_SPEED
m = G.GestureMapper()
prev = m.arm_vector()
worst = 0.0
for i in range(30):
    cmd, _ = m.update([(hand(0.44, 0.80), 'Left')], i / 30.0, 1 / 30.0)
    cur = m.arm_vector()
    worst = max(worst, abs(cur[0] - prev[0]) * 30)
    prev = cur
check(f'column speed limited ({worst:.2f} rad/s <= {G.MAX_SPEED["column"]})', worst <= G.MAX_SPEED['column'] + 1e-9)

# 4. pinch closes, release opens (with hysteresis)
m = G.GestureMapper()
_, g1 = run(m, [(hand(0.7, 0.5, kind='pinch', mirror=True), 'Right')], 0.5)
_, g2 = run(m, [(hand(0.7, 0.5, mirror=True), 'Right')], 0.5)
check(f'pinch -> gripper closed ({g1})', g1 == S.GRIP_CLOSE)
check(f'release -> gripper open ({g2})', g2 == S.GRIPPER_OPEN)
check('closed value moves fingers inwards (negative)', S.GRIP_CLOSE < 0 and S.GRIPPER_OPEN == 0.0)
# 4b. a short accidental pinch (shorter than PINCH_HOLD) must not close the gripper
m = G.GestureMapper()
run(m, [(hand(0.7, 0.5, mirror=True), 'Right')], 0.3)
_, g3 = run(m, [(hand(0.7, 0.5, kind='pinch', mirror=True), 'Right')], G.PINCH_HOLD * 0.5)
_, g4 = run(m, [(hand(0.7, 0.5, mirror=True), 'Right')], 0.3)
check(f'short pinch ({G.PINCH_HOLD * 0.5:.2f} s) ignored ({g3}, {g4})', g3 == S.GRIPPER_OPEN and g4 == S.GRIPPER_OPEN)
check(f'gesture speeds slowed (column {G.MAX_SPEED["column"]} rad/s, z {G.MAX_SPEED["z"]} m/s)',
      G.MAX_SPEED['column'] <= 0.7 and G.MAX_SPEED['z'] <= 0.05)

# 5. fist freezes, hand loss holds
m = G.GestureMapper()
run(m, [(hand(0.25, 0.5), 'Left')], 3.0)
held = m.arm_vector()
run(m, [(hand(0.10, 0.2, kind='fist'), 'Left')], 2.0)
check('fist: joints frozen while the hand moves', np.allclose(m.arm_vector(), held, atol=1e-6))
check('fist state reported', m.state['left'] == 'hold (fist)')
run(m, [], 2.0)
check('hand lost: joints hold', np.allclose(m.arm_vector(), held, atol=1e-6) and m.state['left'] == 'lost')

# 6. wrist follows roll relative to where the hand started
m = G.GestureMapper()
run(m, [(hand(0.7, 0.5, mirror=True), 'Right')], 1.0)
w0 = m.cmd['wrist']
run(m, [(hand(0.7, 0.5, roll=0.4, mirror=True), 'Right')], 3.0)
check(f'tilting the right hand turns the wrist ({w0:+.2f} -> {m.cmd["wrist"]:+.2f})', abs(m.cmd['wrist'] - w0) > 0.5)

# 7. two hands both labelled "Left" -> assigned by position
a = G.GestureMapper.assign_hands([(hand(0.8, 0.5), 'Left'), (hand(0.2, 0.5), 'Left')])
check('duplicate labels resolved by screen position', a['left'][0, 0] < a['right'][0, 0])

# 8. all targets inside joint limits
for k, rng in (('column', 'column_joint'), ('z', 'shoulder_joint'), ('forearm', 'forearm_joint'), ('wrist', 'wrist_joint')):
    lo, hi = S.LIMITS[rng]
    vals = []
    m = G.GestureMapper()
    for x in np.linspace(0, 1, 11):
        for y in np.linspace(0, 1, 11):
            c, _ = m.update([(hand(x, y), 'Left'), (hand(x, y, roll=1.2, mirror=True), 'Right')], 0, 10.0)
            vals.append(c[k])
    check(f'{k} always within limits [{min(vals):+.3f}, {max(vals):+.3f}]', lo <= min(vals) and max(vals) <= hi)

print('ALL GESTURE TESTS PASSED' if not FAILED else 'SOME GESTURE TESTS FAILED')
sys.exit(1 if FAILED else 0)
