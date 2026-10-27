#!/usr/bin/env python3
"""
Endless puck supply for the MuJoCo cell (the Gazebo equivalent lives in
scara_puck_spawner.py).

  * All pucks wait in a pool far away from the cell (out of sight - nothing
    is parked on or near the table).
  * Whenever the start of the belt is clear, a random puck from the pool is
    dropped there, so the cell keeps running until you stop it.
  * Pucks are taken back to the pool when they are finished with:
      - sorted into a bin (the bin is "emptied", so bins never overflow),
      - ridden to the end of the belt (colours you did not ask for), or
      - dropped on the table / floor.
    This is what an operator or an out-feed chute does in a real cell.  To
    model a real cell WITHOUT emptying, use clear_bins=False together with
    Sorter(bin_capacity=N): the robot then reports "bin full" and lets that
    colour pass until you say "empty bins".
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import math
import random

try:
    import scara_scene as S
except ImportError:
    from . import scara_scene as S


def _bins(mode):
    if mode == 'command':
        return [('drop', *S.command_bin_xy(), S.command_bin_yaw())]
    return [(c, *S.bin_xy(c), S.bin_yaw(c)) for c in S.COLOR_ORDER]


def classify(x, y, z, mode='sort'):
    """Where a puck is: 'pool', 'belt', 'end', 'bin_<colour>' (command mode: 'bin_drop'),
    'table', 'lost' or 'air' (held)."""
    if S.is_hidden(x, y):
        return 'pool'
    for c, bx, by, a in _bins(mode):
        lx = (x - bx) * math.cos(a) + (y - by) * math.sin(a)
        ly = -(x - bx) * math.sin(a) + (y - by) * math.cos(a)
        half = S.BIN_SIZE / 2 - S.BIN_WALL
        if abs(lx) < half and abs(ly) < half and z < S.TABLE_TOP_Z + 0.03:
            return f'bin_{c}'
    if z < S.TABLE_TOP_Z - 0.05:
        return 'lost'
    on_belt = abs(S.belt_lat(x, y)) < S.BELT_WIDTH / 2 + 0.01 and S.BELT_TOP_Z - 0.01 < z < S.BELT_TOP_Z + 0.02
    if on_belt:
        return 'end' if S.belt_s(x, y) > S.END_STOP_S - S.PUCK_RADIUS - 0.04 else 'belt'
    if z < S.TABLE_TOP_Z + 0.01:
        return 'table'
    return 'air'


class PuckFeeder:
    def __init__(self, get_pos, set_pos, names=None, seed=None, interval=2.5, clear_bins=True,
                 clear_after=2.0, lat_jitter=0.01, max_spawn=0, order=None, log=print, mode='sort'):
        """
        get_pos(name) -> (x, y, z);  set_pos(name, (x, y, z), yaw=0) teleports a puck (zero velocity).
        max_spawn: 0 = endless.  order: optional list of colours to drop in turn.
        """
        self.names = list(names or S.puck_names())
        self.mode = mode
        self.get_pos, self.set_pos = get_pos, set_pos
        self.rng = random.Random(seed)
        self.interval, self.clear_bins, self.clear_after = interval, clear_bins, clear_after
        self.lat_jitter, self.max_spawn = lat_jitter, max_spawn
        self.order = list(order) if order else None
        self.log = log
        self.spawned = 0
        self.last_spawn = -1e9
        self.since = {}
        self.removed = {}
        self.history = []          # (time, puck name, where it was taken from)
        self.enabled = True

    def hide_all(self):
        for i, n in enumerate(self.names):
            self.set_pos(n, S.hidden_xyz(i))

    def update(self, now):
        poses = {n: self.get_pos(n) for n in self.names}
        # 1. take finished pucks back to the pool
        for i, n in enumerate(self.names):
            where = classify(*poses[n], mode=self.mode)
            done = where in ('end', 'table', 'lost') or (where.startswith('bin_') and self.clear_bins)
            if not done:
                self.since.pop(n, None)
                continue
            t0 = self.since.setdefault(n, now)
            if now - t0 >= (self.clear_after if where.startswith('bin_') else self.clear_after + 1.0):
                self.set_pos(n, S.hidden_xyz(i))
                poses[n] = S.hidden_xyz(i)
                self.since.pop(n, None)
                self.removed[where] = self.removed.get(where, 0) + 1
                self.history.append((now, n, where))
        # 2. drop a new puck when the start of the belt is clear
        if not self.enabled or (self.max_spawn and self.spawned >= self.max_spawn):
            return None
        if now - self.last_spawn < self.interval:
            return None
        for (x, y, z) in poses.values():
            if classify(x, y, z, mode=self.mode) == 'belt' and abs(S.belt_s(x, y) - S.SPAWN_S) < S.MIN_PUCK_SPACING:
                return None
        pool = [n for n in self.names if S.is_hidden(*poses[n][:2])]
        if self.order:
            want = self.order[self.spawned % len(self.order)]
            pool = [n for n in pool if S.puck_color(n) == want]
        if not pool:
            return None
        n = self.rng.choice(pool)
        x, y = S.belt_point(S.SPAWN_S, self.rng.uniform(-self.lat_jitter, self.lat_jitter))
        # random handle direction, like pucks dropped by hand - the camera finds it
        self.set_pos(n, (x, y, S.BELT_TOP_Z + 0.002), self.rng.uniform(-math.pi, math.pi))
        self.spawned += 1
        self.last_spawn = now
        return n


def place_row_for_manual(model, data):     # kept for old scripts; the manual world already has its pucks
    """Manual / keyboard / joystick / gesture modes: put one puck of each colour on
    the belt (the rest stay in the hidden pool) so there is something to pick."""
    import mujoco
    for i, c in enumerate(S.COLOR_ORDER):
        name = f'puck_{c}_1'
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            continue
        x, y = S.belt_point(S.SPAWN_S + S.MIN_PUCK_SPACING * i)
        data.joint(f'{name}_free').qpos[:] = [x, y, S.BELT_TOP_Z + 0.002, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
