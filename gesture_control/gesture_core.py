#!/usr/bin/env python3
"""
Two-hand gesture -> SCARA joint targets (shared by the Gazebo and MuJoCo apps).

Works like the "SCARA Hand Control" window in the Mechamind Labs video: both
hands are tracked at the same time and the robot follows them continuously.

  LEFT hand  (blue)    move left/right   -> column_joint   (base rotation)
                       move up/down      -> shoulder_joint (Z height)
  RIGHT hand (orange)  move left/right   -> forearm_joint
                       tilt (roll)       -> wrist_joint
                       pinch thumb+index -> gripper close  (release = open)
  FIST on either hand  -> freeze that hand's joints (move your hand freely, open it to continue)
  Hand out of view     -> its joints hold still

Why this replaces the old "finger count + tilt" scheme: tilting the hand
changed MediaPipe's finger count, so the selected joint kept jumping while
you were trying to move it; and the gripper sign was reversed.

Pure NumPy - landmarks come in as (21, 3) arrays in image-normalised units
(x right, y down) from a MIRRORED (selfie) camera frame.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import math

import numpy as np

try:
    import scara_scene as S
except ImportError:   # gesture_control/ is next to common/
    import os
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'common'))
    import scara_scene as S

WRIST, THUMB_TIP, INDEX_MCP, INDEX_TIP, MIDDLE_MCP, PINKY_MCP = 0, 4, 5, 8, 9, 17
FINGERS = [(5, 8), (9, 12), (13, 16), (17, 20)]      # (MCP, TIP) of index..pinky

# Working ranges (inside the joint limits)
COLUMN_RANGE = (-1.8, 1.8)
FOREARM_RANGE = (-1.8, 1.8)
WRIST_RANGE = (-3.0, 3.0)
Z_RANGE = S.LIMITS['shoulder_joint']

# Where in the (mirrored) image each hand works, as fractions of the frame
LEFT_X = (0.08, 0.46)       # left half of the picture
RIGHT_X = (0.54, 0.92)      # right half
HAND_Y = (0.15, 0.75)       # top .. bottom (full Z reachable without the hand leaving the picture)

MAX_SPEED = {'column': 0.7, 'z': 0.05, 'forearm': 0.7, 'wrist': 1.2}   # per second (the safety filter may slow it more)
PINCH_CLOSE, PINCH_OPEN = 0.30, 0.50     # thumb-index distance / palm size (hysteresis)
SMOOTH = 0.25                            # landmark low-pass (0..1, higher = snappier)
PINCH_HOLD = 0.25                        # s a pinch / release must last before the gripper reacts
LOST_AFTER = 0.3                         # s without a hand -> hold


def _lerp_clip(v, src, dst):
    a = (v - src[0]) / (src[1] - src[0])
    a = min(1.0, max(0.0, a))
    return dst[0] + a * (dst[1] - dst[0])


def palm_size(lm):
    return float(np.linalg.norm(lm[MIDDLE_MCP, :2] - lm[WRIST, :2])) + 1e-6


def palm_center(lm):
    return lm[[WRIST, INDEX_MCP, MIDDLE_MCP, PINKY_MCP], :2].mean(axis=0)


def is_fist(lm):
    """All four fingers curled: tips closer to the wrist than their knuckles (rotation-proof)."""
    w = lm[WRIST, :2]
    curled = 0
    for mcp, tip in FINGERS:
        if np.linalg.norm(lm[tip, :2] - w) < 1.05 * np.linalg.norm(lm[mcp, :2] - w):
            curled += 1
    return curled >= 4


def pinch_ratio(lm):
    return float(np.linalg.norm(lm[THUMB_TIP, :2] - lm[INDEX_TIP, :2])) / palm_size(lm)


def roll_angle(lm):
    """Hand roll in the image plane, 0 when the knuckle line is horizontal."""
    d = lm[PINKY_MCP, :2] - lm[INDEX_MCP, :2]
    a = math.atan2(d[1], abs(d[0]) + 1e-6)
    return a


class GestureMapper:
    def __init__(self, start=None):
        """start: dict with column, z, forearm, wrist, gripper (e.g. from /joint_states)."""
        s = start or {}
        self.target = {'column': s.get('column', 0.0), 'z': s.get('z', 0.0), 'forearm': s.get('forearm', 0.0),
                       'wrist': s.get('wrist', 0.0)}
        self.cmd = dict(self.target)
        self.gripper = s.get('gripper', S.GRIPPER_OPEN)
        self.smoothed = {}
        self.last_seen = {'left': -1e9, 'right': -1e9}
        self.state = {'left': 'not seen', 'right': 'not seen'}
        self.pinched = False
        self.pinch_since = None
        self.wrist_ref = None
        self.note = ''          # safety filter message shown in the HUD

    @staticmethod
    def assign_hands(hands):
        """
        hands: list of (landmarks (21,3), label 'Left'/'Right' or None).
        MediaPipe labels assume a mirrored image, which is what we feed it, so
        the label is the user's real hand.  If both get the same label, fall
        back to screen position.
        """
        out = {}
        labels = [(lab or '').lower() for _, lab in hands]
        if len(hands) == 2 and labels[0] == labels[1]:
            a, b = hands
            if palm_center(a[0])[0] < palm_center(b[0])[0]:
                return {'left': a[0], 'right': b[0]}
            return {'left': b[0], 'right': a[0]}
        for (lm, _), lab in zip(hands, labels):
            side = lab if lab in ('left', 'right') else ('left' if palm_center(lm)[0] < 0.5 else 'right')
            out[side] = lm
        return out

    def _smooth(self, side, lm):
        prev = self.smoothed.get(side)
        lm = np.asarray(lm, dtype=float)
        if prev is None or self.state.get(side) in ('not seen', 'lost'):
            self.smoothed[side] = lm
        else:
            self.smoothed[side] = prev + SMOOTH * (lm - prev)
        return self.smoothed[side]

    def update(self, hands, now, dt):
        """
        hands: list of (landmarks, handedness label).  Returns the commanded
        joints dict (column, z, forearm, wrist) and gripper value.
        """
        seen = self.assign_hands(hands)
        for side in ('left', 'right'):
            if side in seen:
                raw = np.asarray(seen[side], dtype=float)
                lm = self._smooth(side, raw)
                self.last_seen[side] = now
                if is_fist(raw):          # raw landmarks: freeze on the very first fist frame
                    self.state[side] = 'hold (fist)'
                    if side == 'right':
                        self.wrist_ref = None
                    continue
                self.state[side] = 'active'
                c = palm_center(lm)
                if side == 'left':
                    self.target['column'] = _lerp_clip(c[0], LEFT_X, COLUMN_RANGE)   # robot faces you: your left = its right
                    self.target['z'] = _lerp_clip(c[1], HAND_Y, Z_RANGE[::-1])
                else:
                    self.target['forearm'] = _lerp_clip(c[0], RIGHT_X, FOREARM_RANGE)
                    # wrist follows the change in hand roll since the hand appeared
                    r = roll_angle(lm)
                    if self.wrist_ref is None:
                        self.wrist_ref = (r, self.target['wrist'])
                    w = self.wrist_ref[1] + 2.0 * (r - self.wrist_ref[0])
                    self.target['wrist'] = float(np.clip(w, *WRIST_RANGE))
                    # pinch must be held PINCH_HOLD s (no accidental grabs / drops)
                    p = pinch_ratio(lm)
                    want = (p < PINCH_CLOSE) if not self.pinched else not (p > PINCH_OPEN)
                    if want != self.pinched:
                        if self.pinch_since is None:
                            self.pinch_since = now
                        elif now - self.pinch_since >= PINCH_HOLD:
                            self.pinched = want
                            self.pinch_since = None
                    else:
                        self.pinch_since = None
                    self.gripper = S.GRIP_CLOSE if self.pinched else S.GRIPPER_OPEN
            elif now - self.last_seen[side] > LOST_AFTER:
                if self.state[side] != 'not seen':
                    self.state[side] = 'lost'
                if side == 'right':
                    self.wrist_ref = None
                # hold: freeze the target where the robot currently is
                keys = ('column', 'z') if side == 'left' else ('forearm', 'wrist')
                for k in keys:
                    self.target[k] = self.cmd[k]
        # rate limit towards the targets -> smooth, no jumps
        for k, v in self.target.items():
            step = MAX_SPEED[k] * dt
            self.cmd[k] += float(np.clip(v - self.cmd[k], -step, step))
        return dict(self.cmd), self.gripper

    def arm_vector(self):
        return [self.cmd['column'], self.cmd['z'], self.cmd['forearm'], self.cmd['wrist']]


# ---------------------------------------------------------------------- HUD
BLUE = (235, 140, 40)      # BGR
ORANGE = (40, 160, 245)


def draw_hud(frame, mapper):
    """Video-style overlay: title, hand status and five joint bars."""
    import cv2
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 38), (40, 40, 40), -1)
    cv2.putText(frame, 'SCARA Hand Control', (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (235, 235, 235), 2)
    cv2.putText(frame, f"Left hand: {mapper.state['left']}", (w - 250, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, BLUE, 1)
    cv2.putText(frame, f"Right hand: {mapper.state['right']}", (w - 250, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, ORANGE, 1)
    if mapper.note:
        col = (60, 60, 230) if ('STOP' in mapper.note or 'guard' in mapper.note) else (80, 200, 80)
        cv2.putText(frame, mapper.note, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    bars = [('COLUMN', mapper.cmd['column'], COLUMN_RANGE, BLUE, '{:+.2f}'),
            ('Z', mapper.cmd['z'], Z_RANGE, BLUE, '{:+.3f}'),
            ('FOREARM', mapper.cmd['forearm'], FOREARM_RANGE, ORANGE, '{:+.2f}'),
            ('WRIST', mapper.cmd['wrist'], WRIST_RANGE, ORANGE, '{:+.2f}'),
            ('GRIP', 1.0 if mapper.gripper < -0.01 else 0.0, (0.0, 1.0), ORANGE, None)]
    panel_h = 110
    y0 = h - panel_h
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, y0), (w, h), (30, 30, 30), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    bw = w // len(bars)
    for i, (name, v, rng, col, fmt) in enumerate(bars):
        x = i * bw + bw // 2 - 18
        frac = (v - rng[0]) / (rng[1] - rng[0])
        top, bottom = y0 + 22, h - 28
        cv2.rectangle(frame, (x, top), (x + 36, bottom), (70, 70, 70), -1)
        fill = int((bottom - top) * min(1.0, max(0.0, frac)))
        cv2.rectangle(frame, (x, bottom - fill), (x + 36, bottom), col, -1)
        cv2.putText(frame, name, (x - 6, y0 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1)
        label = ('CLOSED' if v > 0.5 else 'OPEN') if fmt is None else fmt.format(v)
        cv2.putText(frame, label, (x - 6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
    return frame
