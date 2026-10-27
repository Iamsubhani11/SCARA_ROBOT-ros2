#!/usr/bin/env python3
"""
Colour-sorting logic shared by the Gazebo node (scara_color_sorter.py), the
MuJoCo demo (mujoco_sorting_demo.py) and the automated test.

The robot is reached through a small "IO" object so the same decisions run
everywhere:

    io.now() -> float                      seconds
    io.sleep(dt)                           wait (simulation keeps running)
    io.move_arm([col, z, fore, wrist], T)  blocking joint-space move
    io.move_gripper(q, T)                  blocking, both fingers to q
    io.set_belt(speed)                     conveyor surface speed (m/s)
    io.joints() -> dict                    current joint positions
    io.detections() -> list                latest pucks seen on the belt
    io.log(text)

Optional, for belt tracking (the belt keeps running while the robot picks):
    io.follow(points)                      points = [(t, [col, z, fore, wrist], gripper), ...] with t in s
                                           from now: arm AND gripper go through them together, in time
    io.det_time() -> float                 when the latest detections were taken (camera time stamp)
An IO without follow() simply stops the belt for every pick, as before.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import math
import os

try:
    import scara_scene as S
    import scara_kinematics as K
except ImportError:
    from . import scara_scene as S
    from . import scara_kinematics as K

MAX_SPEED = {'column_joint': 0.9, 'shoulder_joint': 0.08, 'forearm_joint': 0.9, 'wrist_joint': 1.2}
PASS_CLEARANCE = 0.08          # a skipped puck must move this far past the pick window
# belt tracking: the arm meets the puck while it rides on the belt, follows it down, grabs and lifts
TRACK_MEET_S = -0.12           # belt position where the arm meets the puck (reach on the belt: -0.16 .. 0.16)
TRACK_LATEST_S = -0.06         # a puck already past this when we could meet it: stop the belt as before
TRACK_DOWN, TRACK_CLOSE, TRACK_UP = 1.0, 1.0, 0.8     # s: go down, close (and wait), lift - all on the move
# the arm meets the puck just above its handle, so Z has only ~7 cm to go down in TRACK_DOWN
# (0.07 m/s - inside the 0.08 m/s Z speed limit; from travel height it would need 0.17 m/s)


class BeltSpeed:
    """The real belt speed, measured by the camera: every puck on the belt moves with it, so each puck
    seen again a little further on gives one measurement (same colour, same side of the belt, moved
    forward). The median of the recent ones is the speed - one bad detection cannot spoil it.
    A real belt (and Gazebo's) does not run at exactly the speed it is told: 10 % off is 2 cm over
    one pick, more than the 10 mm of room the fingers have."""

    def __init__(self, keep=40):
        self.prev, self.prev_t, self.v, self.keep = [], None, [], keep

    def reset(self):
        self.prev, self.prev_t = [], None             # the belt stopped: pucks speed up again from rest

    def update(self, dets, t):
        if self.prev_t is not None and t - self.prev_t >= 0.12:
            dt = t - self.prev_t
            for d in dets:
                for p in self.prev:
                    ds = d['s'] - p['s']
                    if (d['color'] == p['color'] and 0.0 < ds < 0.25 * dt + 0.01
                            and abs(S.belt_lat(d['x'], d['y']) - S.belt_lat(p['x'], p['y'])) < 0.015):
                        self.v.append(ds / dt)
                        break
            self.v = self.v[-self.keep:]
        if self.prev_t is None or t - self.prev_t >= 0.12:
            self.prev, self.prev_t = [dict(d) for d in dets], t

    def speed(self, n_min=4):
        if len(self.v) < n_min:
            return None
        v = sorted(self.v)[len(self.v) // 2]
        return min(max(v, 0.5 * S.BELT_SPEED), 2.0 * S.BELT_SPEED)


def move_time(q_from, q_to, t_min=0.6):
    t = t_min
    for i, name in enumerate(S.ARM_JOINTS):
        t = max(t, abs(q_to[i] - q_from[i]) / MAX_SPEED[name])
    return t


class Sorter:
    def __init__(self, io, colors='all', bin_capacity=0, mode='sort'):
        """
        mode='sort'    : every puck goes into the bin of its own colour (4 bins).
        mode='command' : ONE bin.  Nothing is picked until you ask.  request('red')
                         makes the robot take EVERY red puck off the belt and drop
                         it in the single bin, until you give the next colour
                         (request('blue')), 'any' (every colour) or 'none'.
                         request('3 red') picks three red pucks, then waits.
                         request('sequence red blue green') picks ONE red, then
                         one blue, then one green, in that order (the next colour
                         only after the previous one is in the bin), then waits;
                         '2 red 1 blue' inside a sequence repeats a colour.
                         Everything not asked for rides past.
        bin_capacity: 0 = bins are emptied automatically (the simulators take
        sorted pucks away, like an operator / out-feed would).  N > 0 = a real
        cell without automatic emptying: after N pucks the bin counts as FULL,
        pucks of that colour ride past and the robot asks for the bin to be
        emptied ("empty bins" / "empty red" command -> empty_bins()).
        """
        self.io = io
        self.mode = mode
        self.active = set()                 # command mode: colours being picked now ('any' = every colour)
        self.count_left = None              # command mode: pucks still to pick (None = until the next command)
        self.queue = []                     # sequence mode: colours still to come, in order
        if mode == 'command':
            self.wanted = set()
            self.io.log('Command mode: waiting for a colour (say / type e.g. "red") - it is picked until the next colour')
        else:
            self.set_colors(colors)
        self.sorted = {c: 0 for c in S.COLOR_ORDER}
        self.failures = 0
        self.bin_capacity = int(bin_capacity or 0)
        self.in_bin = {c: 0 for c in S.COLOR_ORDER}
        self._full_warned = set()
        self.track_belt = False             # True: follow the running belt instead of stopping it (needs io.follow)
        self.tracked = 0                    # picks done without stopping the belt
        self.belt = BeltSpeed()             # belt tracking: the belt speed as the camera measures it

    def bin_full(self, color):
        if self.mode == 'command':
            return self.bin_capacity > 0 and sum(self.in_bin.values()) >= self.bin_capacity
        return self.bin_capacity > 0 and self.in_bin[color] >= self.bin_capacity

    def bin_pose(self, color):
        """(x, y, yaw) of the bin a puck of this colour goes into."""
        if self.mode == 'command':
            return (*S.command_bin_xy(), S.command_bin_yaw())
        return (*S.bin_xy(color), S.bin_yaw(color))

    # ------------------------------------------------------------ commands
    @property
    def requests(self):
        """Colours being picked right now (command mode), for status displays."""
        return sorted(self.active)

    def request(self, colors):
        """
        Command mode: pick these colours from now on, until the next command.
        'red' / 'red blue' / 'any' (= all colours) / 'none' (= stop picking);
        a number limits the count: '3 red', 'red 2'.
        Returns the new list of active colours.
        """
        if isinstance(colors, str):
            colors = colors.replace(',', ' ').split()
        words = [c.strip().lower() for c in colors]
        if words[:1] == ['track'] or words[:2] == ['follow', 'belt']:     # "track on" / "track off"
            self.set_tracking(words[-1] not in ('off', 'stop', 'no', '0'))
            return self.requests
        if words and (words[0] in ('sequence', 'seq', 'order', 'then') or 'then' in words):
            return self._sequence([w for w in words if w not in ('sequence', 'seq', 'order', 'then', 'and')])
        self.queue = []
        new, count, unknown = set(), None, []
        for c in colors:
            c = c.strip().lower()
            if c.isdigit():
                count = int(c)
            elif c in ('all', 'any', 'everything', 'anything'):
                new.add('any')
            elif c in ('none', 'nothing', 'clear'):
                new = {'none'}
            elif c in S.COLORS:
                new.add(c)
            elif c:
                unknown.append(c)
        if unknown:
            self.io.log(f'Unknown colour {unknown} - use {", ".join(S.COLOR_ORDER)}, any or none')
        if not new:
            return self.requests
        if 'none' in new:
            self.active, self.count_left, self.queue = set(), None, []
            self.io.log('Picking stopped - the belt keeps running. Give a colour to start again.')
            return []
        self.active = {'any'} if 'any' in new else new
        self.count_left = count if count and count > 0 else None
        what = 'every colour' if 'any' in self.active else ' + '.join(sorted(self.active))
        self.io.log(f'Now picking {what}' + (f' ({self.count_left} pucks)' if self.count_left
                                             else ' until you give the next colour'))
        return self.requests

    def set_tracking(self, on):
        """Belt tracking on / off. Only works when the simulator can move arm and gripper together
        along a timed path (io.follow); the real camera cell keeps stopping the belt."""
        if on and not hasattr(self.io, 'follow'):
            self.io.log('Belt tracking needs a simulator (MuJoCo or Gazebo) - here the belt stops for every pick')
            on = False
        if on and self.mode != 'command':
            # tested: in the sorting cell every puck is wanted and they come every 2.5 s - chasing one
            # lets the next ones ride off the belt. Stopping the belt is the better plan there.
            self.io.log('Belt tracking is for the command cell (a few wanted pucks); sorting keeps stopping the belt')
            on = False
        self.track_belt = bool(on)
        self.io.log('Belt tracking ON: the robot follows the moving belt instead of stopping it' if on
                    else 'Belt tracking off: the belt stops for every pick')

    def _sequence(self, words):
        seq, rep = [], 1
        for w in words:
            if w.isdigit():
                rep = max(1, int(w))
            elif w in S.COLORS:
                seq += [w] * rep
                rep = 1
            elif w:
                self.io.log(f'Unknown colour {w!r} in the sequence - ignored')
        if not seq:
            self.io.log('Sequence needs colours, e.g. "sequence red blue green"')
            return self.requests
        self.queue = seq[1:]
        self.active, self.count_left = {seq[0]}, 1
        self.io.log('Sequence: ' + ' -> '.join(seq) + f'  (now: {seq[0]})')
        return self.requests

    @property
    def sequence(self):
        """Remaining sequence including the colour being picked now (empty when not in a sequence)."""
        if not self.queue and not (self.count_left == 1 and len(self.active) == 1 and 'any' not in self.active):
            return []
        return sorted(self.active) + list(self.queue)

    def wants(self, color):
        if self.bin_full(color):
            return False
        if self.mode == 'command':
            return 'any' in self.active or color in self.active
        return color in self.wanted

    def _fulfil(self, color):
        if self.mode == 'command' and self.count_left:
            self.count_left -= 1
            if self.count_left == 0:
                if self.queue:
                    nxt = self.queue.pop(0)
                    self.active, self.count_left = {nxt}, 1
                    self.io.log(f'Sequence: next {nxt}' + (f' (then {" -> ".join(self.queue)})' if self.queue else ' (last one)'))
                    return
                self.active, self.count_left = set(), None
                self.io.log('Done - waiting for the next colour')

    def empty_bins(self, colors=None):
        for c in (colors or S.COLOR_ORDER):
            self.in_bin[c] = 0
            self._full_warned.discard(c)
        self.io.log(f'Bins emptied: {list(colors or S.COLOR_ORDER)}')

    # ------------------------------------------------------------------ setup
    def set_colors(self, colors):
        if isinstance(colors, str):
            colors = [c.strip().lower() for c in colors.replace(',', ' ').split()]
        if 'none' in colors:
            self.wanted = set()            # let everything pass until a colour is requested
        elif not colors or 'all' in colors:
            self.wanted = set(S.COLOR_ORDER)
        else:
            unknown = [c for c in colors if c not in S.COLORS]
            if unknown:
                raise ValueError(f'unknown colour(s) {unknown}; use {S.COLOR_ORDER} or all')
            self.wanted = set(colors)
        self.io.log(f'Sorting colours: {sorted(self.wanted)}')

    # ------------------------------------------------------------- motion
    def _current_arm(self):
        j = self.io.joints()
        return [j.get(n, 0.0) for n in S.ARM_JOINTS]

    def move_to(self, x, y, z_joint, t_min=0.6, yaw=None, symmetry=math.pi / 2):
        cur = self._current_arm()
        target = K.arm_pose(x, y, z_joint, prefer=(cur[0], cur[2]), yaw=yaw, symmetry=symmetry)
        # lift first if we are low, so we never drag across the belt or bins
        if cur[1] < S.Z_TRAVEL - 0.005 and abs(target[0] - cur[0]) + abs(target[2] - cur[2]) > 0.06:
            up = [cur[0], S.Z_TRAVEL, cur[2], cur[3]]
            self.io.move_arm(up, move_time(cur, up))
            cur = up
        if abs(z_joint - S.Z_TRAVEL) > 0.005 and abs(target[0] - cur[0]) + abs(target[2] - cur[2]) > 0.06:
            above = [target[0], S.Z_TRAVEL, target[2], target[3]]
            self.io.move_arm(above, move_time(cur, above))
            cur = above
        self.io.move_arm(target, move_time(cur, target, t_min))
        return target

    def go_ready(self):
        self.move_to(S.READY_XY[0], S.READY_XY[1], S.Z_TRAVEL)

    def gripper_open(self):
        self.io.move_gripper(S.GRIPPER_OPEN, 0.5)

    def gripper_close(self):
        self.io.move_gripper(S.GRIP_CLOSE, 0.7)
        self.io.sleep(0.3)

    def holding(self):
        j = self.io.joints()
        return min(j.get('left_finger_joint', 0.0), j.get('right_finger_joint', 0.0)) > S.GRASP_OK_ABOVE

    # --------------------------------------------------------------- picking
    def gripper_yaw(self):
        """World direction of the gripper (wrist x axis) right now."""
        q = self._current_arm()
        return S.BASE_YAW + q[0] + q[2] + q[3]

    def pick_at(self, x, y, yaw, surface_z=None, label='puck'):
        """
        Grab a puck standing at (x, y): the fingers, wide open (91 mm), come
        down around the 70 mm puck body, close on it and lift.  `yaw` turns
        the wrist to a given direction; None = any direction (round puck).
        Returns True when holding.
        """
        surface_z = S.BELT_TOP_Z if surface_z is None else surface_z
        z_pick = S.z_pick_on(surface_z)
        z_pre = S.z_for_pad_bottom(surface_z + S.HANDLE_TOP_Z + 0.015)
        # yaw=None: the puck is round, any gripper direction works (wrist kept at 0)
        self.io.move_gripper(S.GRIP_PREGRASP, 0.4)
        self.move_to(x, y, z_pre, yaw=yaw, symmetry=math.pi)
        self.move_to(x, y, z_pick, t_min=0.8, yaw=yaw, symmetry=math.pi)
        self.gripper_close()
        self.move_to(x, y, S.Z_TRAVEL, t_min=0.8, yaw=yaw, symmetry=math.pi)
        if not self.holding():
            self.io.log(f'Missed the {label} - will look again')
            self.gripper_open()
            self.failures += 1
            return False
        return True

    def place_at(self, x, y, surface_z, yaw=None):
        """Set the held puck down on surface_z at (x, y) and let go.  yaw squares the
        gripper to a bin's walls (any multiple of 90 deg); None keeps the wrist as it is."""
        if yaw is None:
            yaw = self.gripper_yaw()
        z_place = S.z_pick_on(surface_z + S.DROP_HEIGHT)   # puck let go just above the surface
        self.move_to(x, y, S.Z_TRAVEL, yaw=yaw, symmetry=math.pi / 2)
        self.move_to(x, y, z_place, t_min=1.0, yaw=yaw, symmetry=math.pi / 2)
        self.io.move_gripper(S.GRIP_RELEASE, 0.4)
        self.io.sleep(0.3)
        self.move_to(x, y, S.Z_TRAVEL, t_min=0.8, yaw=self.gripper_yaw(), symmetry=math.pi / 2)
        self.gripper_open()

    def track_plan(self, det, t_det, speed=None):
        """The timed path for grabbing a puck that rides on the running belt, or None if it cannot
        be done (puck too far on, a joint would hit its limit, too close to a singularity).

        The puck moves in a straight line at the belt speed, so we know where it will be:
            p(t) = p_seen + v_belt * (t - t_seen)
        1. pick a meeting point on the belt and the time the arm needs to get above it
        2. from there follow the puck with the Jacobian (scara_kinematics.track_line) while going
           down, closing the gripper and lifting - all at the speed of the belt
        speed: the belt speed MEASURED by the camera (m/s); a real belt (and Gazebo's) does not run at
        exactly the speed it is told - 10 % off is 2 cm over a pick, more than the fingers' room"""
        io = self.io
        sp = speed or S.BELT_SPEED
        v = (S.BELT_DIR[0] * sp, S.BELT_DIR[1] * sp)
        now = io.now()
        s_now = det['s'] + sp * (now - t_det)
        cur = self._current_arm()
        s_meet, q_above, t_go = max(TRACK_MEET_S, s_now + 0.3 * sp), None, 0.0
        for _ in range(4):                         # meeting point <-> time to get there, a few rounds
            dt = (s_meet - s_now) / sp
            px, py = det['x'] + v[0] * (now + dt - t_det), det['y'] + v[1] * (now + dt - t_det)
            try:
                q_above = K.arm_pose(px, py, S.Z_TRAVEL, prefer=(cur[0], cur[2]))
            except K.IKError:
                return None
            t_go = move_time(cur, q_above)
            s_meet = max(s_meet, s_now + sp * t_go)
        if s_meet > TRACK_LATEST_S:
            return None
        t_meet = (s_meet - s_now) / sp            # from now; >= t_go: the arm is early or just in time
        p_meet = (det['x'] + v[0] * (now + t_meet - t_det), det['y'] + v[1] * (now + t_meet - t_det))
        T = TRACK_DOWN + TRACK_CLOSE + TRACK_UP
        try:
            path = K.track_line(q_above[0], q_above[2], p_meet, v, T, dt=0.1)
        except K.IKError:
            return None
        z_pick = S.z_pick_on(S.BELT_TOP_Z)
        z_pre = S.z_for_pad_bottom(S.BELT_TOP_Z + S.HANDLE_TOP_Z + 0.015)    # just above the handles
        # 1. over the meeting point at travel height (the arm comes from beside the belt), 2. down to
        # just above the handles while the puck arrives, then follow it: down, close, lift
        t_pre = max(0.4 * t_meet, t_meet - 1.0)
        pts = [(t_pre, [q_above[0], S.Z_TRAVEL, q_above[2], 0.0], S.GRIP_PREGRASP),
               (t_meet, [q_above[0], z_pre, q_above[2], 0.0], S.GRIP_PREGRASP)]
        for t, a, b in path[1:]:
            if t <= TRACK_DOWN:
                z, g = z_pre + (z_pick - z_pre) * t / TRACK_DOWN, S.GRIP_PREGRASP
            elif t <= TRACK_DOWN + TRACK_CLOSE:
                k = min(1.0, (t - TRACK_DOWN) / 0.7)
                z, g = z_pick, S.GRIP_PREGRASP + (S.GRIP_CLOSE - S.GRIP_PREGRASP) * k
            else:
                z, g = z_pick + (S.Z_TRAVEL - z_pick) * (t - TRACK_DOWN - TRACK_CLOSE) / TRACK_UP, S.GRIP_CLOSE
            pts.append((t_meet + t, [a, z, b, 0.0], g))
        return pts

    def track_pick(self, det, t_det, speed=None):
        """Grab the puck without stopping the belt. Returns True holding, False missed, None not possible."""
        pts = self.track_plan(det, t_det, speed)
        if pts is None:
            return None
        self.io.log(f'Following the belt to pick the {det["color"]} puck (belt keeps running)')
        self.io.follow(pts)
        if not self.holding():
            self.io.log(f'Missed the {det["color"]} puck on the moving belt - will look again')
            self.gripper_open()
            self.failures += 1
            return False
        self.tracked += 1
        return True

    def pick_and_place(self, det, held=False):
        color = det['color']
        # the puck is round and fits between the open fingers: any approach direction works
        yaw = None
        if not held:
            self.io.log(f'Picking {color} puck at x={det["x"]:+.3f} y={det["y"]:+.3f}')
            if not self.pick_at(det['x'], det['y'], yaw, S.BELT_TOP_Z, f'{color} puck'):
                return False
        # back off the belt towards the robot first, so the carried puck never
        # sweeps over the pucks waiting upstream
        bx0, by0 = det['x'], det['y']
        if held:                                   # tracked pick: the puck has ridden on since it was seen
            q = self._current_arm()
            bx0, by0 = K.fk_xy(q[0], q[2], q[3])
        rx, ry = S.belt_point(S.belt_s(bx0, by0), -S.BELT_WIDTH / 2)
        try:
            self.move_to(rx, ry, S.Z_TRAVEL, yaw=self.gripper_yaw(), symmetry=math.pi / 2)
        except K.IKError:
            pass
        bx, by, byaw = self.bin_pose(color)
        self.place_at(bx, by, S.TABLE_TOP_Z + S.BIN_FLOOR, yaw=byaw)
        self.sorted[color] += 1
        self.in_bin[color] += 1
        self._fulfil(color)
        where = 'the bin' if self.mode == 'command' else f'the {color} bin'
        self.io.log(f'Placed {color} puck in {where}  (totals {self.sorted})')
        if self.bin_full(color):
            self.io.log(f'The {color} bin is FULL ({self.bin_capacity} pucks) - please empty it')
        return True

    # ------------------------------------------------------------ main loop
    def _in_reach(self, d):
        # same zone that triggers a stop; pucks already let through are ignored
        return (S.PICK_WINDOW_S[0] - 0.12 <= d['s'] < S.PICK_WINDOW_S[1] + PASS_CLEARANCE - 0.01
                and K.reachable(d['x'], d['y'], margin=0.03))

    def step(self, timeout=30.0):
        """
        Run the belt until a puck the robot wants reaches the pick window, stop,
        pick it and put it in its bin.  Pucks nobody asked for just ride past
        (the belt does not stop for them).  Returns 'picked', 'missed' or
        'idle' (nothing wanted arrived before timeout - the belt keeps running).
        """
        io = self.io
        t0 = io.now()
        io.set_belt(S.BELT_SPEED)
        if self.track_belt and hasattr(io, 'follow'):
            r = self._track_step(timeout)
            if r is not None:
                return r                          # None: this puck came too late to follow - stop the belt for it
        lo, hi = S.PICK_WINDOW_S[0], S.PICK_WINDOW_S[1] + PASS_CLEARANCE - 0.01
        warned = set()
        while io.now() - t0 < timeout:
            in_window = [d for d in io.detections() if lo <= d['s'] < hi]
            if any(self.wants(d['color']) for d in in_window):
                break
            for d in in_window:
                if d['color'] in warned:
                    continue
                warned.add(d['color'])
                if self.bin_full(d['color']):
                    if d['color'] not in self._full_warned:
                        io.log(f'The {d["color"]} bin is full - letting it pass until the bin is emptied')
                        self._full_warned.add(d['color'])
            io.sleep(0.05)
        else:
            return 'idle'
        io.set_belt(0.0)
        io.sleep(0.5)                     # let the puck settle and the camera refresh
        cands = [d for d in io.detections() if self._in_reach(d) and self.wants(d['color'])]
        if not cands:
            return 'idle'
        target = max(cands, key=lambda d: d['s'])
        ok = self.pick_and_place(target)
        self.go_ready()
        return 'picked' if ok else 'missed'

    def go_wait(self):
        """Belt tracking: wait at travel height just beside the belt (robot side) near the meeting
        point, so the arm is there quickly when a puck arrives (from the ready pose it takes ~2.5 s).
        Not above the belt: from there the arm hides the coming pucks from the camera."""
        x, y = S.belt_point(TRACK_MEET_S + 0.06, -(S.BELT_WIDTH / 2 + 0.02))
        cur = self._current_arm()
        q = K.arm_pose(x, y, S.Z_TRAVEL, prefer=(cur[0], cur[2]))
        if max(abs(a - b) for a, b in zip(q, cur)) > 0.02:
            self.move_to(x, y, S.Z_TRAVEL)

    def _track_step(self, timeout):
        """Belt tracking: wait (belt running) for a wanted puck upstream, then pick it on the move.
        Returns 'picked' / 'missed' / 'idle', or None when the puck is already too far on to follow."""
        io = self.io
        self.go_wait()
        t0 = io.now()
        self.belt.reset()                          # the belt was paused (arm at the bin) - measure from now on
        while io.now() - t0 < timeout:
            dets = io.detections()
            t_det = io.det_time() if hasattr(io, 'det_time') else io.now()
            if io.now() - t0 > 0.6:                # the belt is up to speed again
                self.belt.update(dets, t_det)
            # a wanted puck already too far on to follow (the belt froze it there while the arm was at
            # the bin) is still in the normal pick window: stop the belt and take it first
            late = [d for d in dets if self.wants(d['color']) and self._in_reach(d) and d['s'] >= TRACK_LATEST_S]
            if late:
                return None
            cands = [d for d in dets if self.wants(d['color']) and d['s'] < TRACK_LATEST_S]
            # follow the belt only for a wanted puck that comes alone (tested: when wanted pucks crowd
            # together, chasing one lets another ride off the belt) - otherwise stop the belt as before
            if len([d for d in dets if self.wants(d['color']) and d['s'] > -0.55]) > 1:
                return None
            if cands:
                target = max(cands, key=lambda d: d['s'])     # the one that leaves the reach first
                speed = self.belt.speed()
                if speed is None and target['s'] < TRACK_MEET_S - 0.06:
                    io.sleep(0.05)                 # far enough upstream: let the camera measure the belt first
                    continue
                if os.environ.get('DEBUG_TRACK'):
                    io.log(f'track: s={target["s"]:+.3f} measured belt speed {speed} ({len(self.belt.v)} measurements)')
                got = self.track_pick(target, t_det, speed)
                if got is None:
                    if target['s'] > TRACK_MEET_S - 0.02:
                        return None                          # too far on: the normal (stopping) pick takes it
                    io.sleep(0.05)
                    continue
                if got:
                    # the pick itself was on the move; the trip to the bin and back is long (~15 s), so
                    # the belt pauses while the arm is away - otherwise the next pucks would ride off
                    io.set_belt(0.0)
                    self.pick_and_place(target, held=True)
                self.go_wait()
                return 'picked' if got else 'missed'
            io.sleep(0.05)
        return 'idle'
