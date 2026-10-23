#!/usr/bin/env python3
"""
Safety mode + pick assist for every MANUAL way of moving the SCARA
(gestures, joystick, keyboard, Twin Control GUI, phone app) - MuJoCo and Gazebo.

Every new joint target from the operator goes through SafetyFilter.filter()
before it is sent to the robot:

  1. speed limit      each joint (and the gripper) moves at most at the speed of
                      the chosen mode, so a jumpy hand or a full stick never
                      makes the arm jump.  The gripper closes gently (about 1 s).
  2. pick assist      gripper open, lowered close to a puck and within 4.5 cm of
                      its centre -> the arm centres itself over the puck, so the
                      open fingers go down AROUND it (no pixel-perfect aiming).
  3. puck guard       a finger would come down ON a puck (not centred) -> Z
                      stops just above the puck instead of pressing it into the
                      table; the HUD says "centre the gripper".
  4. blocked guard    the arm is pushing against something (measured Z higher
                      than commanded) -> the command stops there.
  5. table floor      the finger pads never go below the table top.
  6. emergency stop   estop = True freezes the arm where it is.

Modes:  safe (default) - slow and all guards     normal - faster, all guards
        off - raw commands (only the joint limits)

No ROS / MuJoCo inside: callers pass the measured joints and the puck
positions (MuJoCo: from the model, Gazebo: /scara/world_poses).
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import math

try:
    import scara_scene as S
    import scara_kinematics as K
except ImportError:          # imported as a package
    from . import scara_scene as S
    from . import scara_kinematics as K

SPEEDS = {   # column rad/s, z m/s, forearm rad/s, wrist rad/s, gripper m/s
    'safe': {'column': 0.6, 'z': 0.04, 'forearm': 0.6, 'wrist': 1.0, 'gripper': 0.025},
    'normal': {'column': 1.2, 'z': 0.08, 'forearm': 1.2, 'wrist': 2.0, 'gripper': 0.06},
    'off': None,
}
MODES = tuple(SPEEDS)
KEYS = ('column', 'z', 'forearm', 'wrist')

CAPTURE = 0.045          # pick assist: centre on a puck closer than this (m)
APPROACH = 0.045         # ... once the pads are this close above the puck top (m)
GUARD_CLEAR = 0.002      # puck guard: pads stop this far above a puck they would land on
BLOCKED = 0.006          # blocked guard: commanded Z this far below measured = pushing
PAD_HALF_LEN = 0.046     # finger pads: half length along the gripper (wrist x)
PAD_THICK = 0.027        # pad thickness (across the gripper)
OPEN_HALF_GAP = S.GRIPPER_GAP_OPEN / 2.0


def gripper_yaw(q):
    return S.BASE_YAW + q[0] + q[2] + q[3]


def _rect_dist(px, py, x0, x1, y0, y1):
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def resting_pucks(pucks):
    """Pucks lying on the table / belt / in a bin - not the one in the air."""
    out = []
    for p in pucks or []:
        x, y, z = p[-3:]
        if z < S.BELT_TOP_Z + 0.012 or z < S.TABLE_TOP_Z + 0.03:
            out.append((x, y, z))
    return out


def finger_hits(q, grip, puck_xy):
    """True when a finger pad (not the gap between them) is over this puck."""
    tx, ty = K.fk_xy(q[0], q[2], q[3])
    a = gripper_yaw(q)
    dx, dy = puck_xy[0] - tx, puck_xy[1] - ty
    lx = dx * math.cos(a) + dy * math.sin(a)
    ly = -dx * math.sin(a) + dy * math.cos(a)
    inner = OPEN_HALF_GAP + min(0.0, grip)            # grip < 0 closes both fingers
    for sgn in (1.0, -1.0):
        y0, y1 = sorted((sgn * inner, sgn * (inner + PAD_THICK)))
        if _rect_dist(lx, ly, -PAD_HALF_LEN, PAD_HALF_LEN, y0, y1) < S.PUCK_RADIUS:
            return True
    return False


class SafetyFilter:
    def __init__(self, mode='safe', assist=True):
        self.set_mode(mode)
        self.assist = assist
        self.estop = False
        self.speed_scale = 1.0       # 0.1 .. 1.0: operator's speed slider (GUI / phone app)
        self.out = None              # last arm command sent [col, z, fore, wrist]
        self.grip = None             # last gripper command sent
        self.note = ''               # what the filter did this tick (for HUD / log)
        self._t = 0.0
        self._zhist = []
        self._below_since = None
        self._frozen = False
        self._assist_off = None
        self._assist_xy = (0.0, 0.0)
        self.snap = None             # [column, forearm, wrist] the assist aimed for (None = untouched)

    def set_mode(self, mode):
        if mode not in SPEEDS:
            raise ValueError(f'safety mode must be one of {MODES}')
        self.mode = mode

    def reset(self, q, grip=None):
        """Start from the robot's real pose (call when control starts / after e-stop)."""
        self.out = list(q)
        self.grip = S.GRIPPER_OPEN if grip is None else grip

    # ----------------------------------------------------------------- main
    def filter(self, q_cmd, grip_cmd, dt, q_meas=None, grip_meas=None, pucks=None):
        """
        q_cmd: operator's arm target [column, z, forearm, wrist]; grip_cmd: gripper target.
        q_meas / grip_meas: measured joints (or None); pucks: [(x, y, z_bottom), ...] world.
        Returns (arm command, gripper command).  self.note tells what happened.
        """
        q_cmd = [float(v) for v in q_cmd]
        if self.out is None:
            self.reset(q_meas if q_meas is not None else q_cmd, grip_meas if grip_meas is not None else grip_cmd)
        self.note = ''
        self.snap = None
        if self.estop:
            if not self._frozen:
                self._frozen = True
                if q_meas is not None:         # hold where the arm IS, not where it was heading
                    self.out = [float(v) for v in q_meas]
            self.note = 'EMERGENCY STOP'
            return list(self.out), self.grip
        self._frozen = False
        # joint limits always
        for i, n in enumerate(S.ARM_JOINTS):
            lo, hi = S.LIMITS[n]
            q_cmd[i] = min(max(q_cmd[i], lo), hi)
        g_lo = S.LIMITS['left_finger_joint'][0]
        grip_cmd = min(max(float(grip_cmd), g_lo), S.GRIPPER_OPEN)
        if self.mode == 'off':
            self.out, self.grip = q_cmd, grip_cmd
            return list(q_cmd), grip_cmd

        target = list(q_cmd)
        grip_now = self.grip if grip_meas is None else grip_meas
        rest = resting_pucks(pucks)
        pad_bottom = K.pad_bottom_z(target[1])
        # 2. pick assist: centre the open gripper over a puck it is being lowered onto
        if self.assist and rest and grip_cmd > -0.004 and grip_now > -0.006:
            tx, ty = K.fk_xy(target[0], target[2], target[3])
            best = None
            for (x, y, z) in rest:
                d = math.hypot(x - tx, y - ty)
                top = z + S.PUCK_HEIGHT
                if d < CAPTURE and pad_bottom < top + APPROACH and (best is None or d < best[0]):
                    best = (d, x, y)
            if best and best[0] > 0.0015:
                try:
                    t1, t2, t3 = K.ik_xy_yaw(best[1], best[2], gripper_yaw(target),
                                             prefer=(target[0], target[2]), symmetry=2 * math.pi)
                    if abs(t1 - target[0]) + abs(t2 - target[2]) < 0.6:
                        self._assist_off = [t1 - q_cmd[0], t2 - q_cmd[2], t3 - q_cmd[3]]
                        self._assist_xy = (best[1], best[2])
                        target[0], target[2], target[3] = t1, t2, t3
                        self.note = 'assist: centring over the puck'
                except K.IKError:
                    pass
            elif best:
                self._assist_off = None            # already centred by the operator
        # keep the centring while the gripper closes and carries the puck (no jump back
        # to the operator's slightly-off position with the puck in the fingers)
        # (dropped once the gripper is open again and lifted clear of the pucks)
        if self.note != 'assist: centring over the puck' and self._assist_off:
            lifted = K.pad_bottom_z(q_cmd[1]) > S.TABLE_TOP_Z + S.PUCK_HEIGHT + 0.02
            ux, uy = K.fk_xy(q_cmd[0], q_cmd[2], q_cmd[3])
            away = math.hypot(ux - self._assist_xy[0], uy - self._assist_xy[1]) > CAPTURE + 0.01
            if grip_cmd > -0.004 and grip_now > -0.002 and (lifted or away):
                self._assist_off = None
            else:
                target[0] += self._assist_off[0]
                target[2] += self._assist_off[1]
                target[3] += self._assist_off[2]
                if grip_cmd < -0.004:              # gripping: controllers may adopt the centred pose
                    self.snap = [target[0], target[2], target[3]]
        # 3. puck guard: never put a finger down on top of a puck
        z_floor = S.z_for_pad_bottom(S.TABLE_TOP_Z + 0.004)
        for (x, y, z) in rest:
            top = z + S.PUCK_HEIGHT
            if K.pad_bottom_z(target[1]) < top + GUARD_CLEAR and finger_hits(target, grip_now, (x, y)):
                # only if the pads are not already beside the puck (lower than its top)
                cur_pad = K.pad_bottom_z((q_meas or self.out)[1])
                if cur_pad > top - 0.004:
                    z_floor = max(z_floor, S.z_for_pad_bottom(top + GUARD_CLEAR))
                    self.note = 'guard: a finger is over the puck - centre the gripper'
        if target[1] < z_floor:
            target[1] = z_floor
        # 4. blocked guard: Z has stopped moving although the command is well below it
        #    -> something is underneath; stop pushing (a lagging but moving Z is fine)
        if q_meas is not None:
            self._t += min(max(dt, 0.0), 0.1)
            self._zhist.append((self._t, q_meas[1]))
            while self._zhist and self._t - self._zhist[0][0] > 0.4:
                self._zhist.pop(0)
            still = (len(self._zhist) > 3 and self._t - self._zhist[0][0] > 0.3
                     and max(z for _, z in self._zhist) - min(z for _, z in self._zhist) < 0.0008)
            # compare with what was really SENT (self.out), not the new target: a fresh
            # "go 10 cm down" target is far below the arm before it has even started to move,
            # and a slow simulation (Gazebo at a low real-time factor) lags behind the command
            below = q_meas[1] - self.out[1] > BLOCKED
            if not below:
                self._below_since = None
            elif self._below_since is None:
                self._below_since = self._t
            # ... and only after the command has been well below the arm for a moment, so the
            # arm gets time to start following (dt is simulation time for a simulator)
            if (still and below and self._t - self._below_since > 0.3 and target[1] <= self.out[1]):
                target[1] = max(target[1], q_meas[1] - 0.003)
                self.note = self.note or 'guard: Z is blocked - stopped pushing'
        # 1. speed limits (towards the target, from the last command)
        sp = SPEEDS[self.mode]
        dt = min(max(dt, 0.0), 0.1)
        scale = min(max(getattr(self, 'speed_scale', 1.0), 0.1), 1.0)   # operator's speed slider
        out = []
        for i, k in enumerate(KEYS):
            step = sp[k] * dt * scale
            out.append(self.out[i] + min(max(target[i] - self.out[i], -step), step))
        gstep = sp['gripper'] * dt * max(scale, 0.5)
        grip = self.grip + min(max(grip_cmd - self.grip, -gstep), gstep)
        self.out, self.grip = out, grip
        return list(out), grip

    def clamp_back(self, mapper_q):
        """
        For controllers that keep their own target (joystick, keyboard, GUI): take over what
        the filter did, so the operator's target and the arm agree - the pick assist's
        centring (column / forearm / wrist) and a guard's Z stop (no wind-up past it).
        """
        if self.out is None:
            return mapper_q
        q = list(mapper_q)
        if q[1] < self.out[1] - 0.004 and 'guard' in self.note:
            q[1] = self.out[1]
        if self.snap is not None:
            q[0], q[2], q[3] = self.snap
            self._assist_off = None             # the operator's target is centred now
        return q


def add_args(ap):
    """--safety safe|normal|off and --no-assist for the manual-control apps."""
    ap.add_argument('--safety', default='safe', choices=MODES,
                    help='safe (default): slow + guards | normal: faster + guards | off: raw')
    ap.add_argument('--no-assist', action='store_true', help='switch off the pick assist (auto-centring)')


def next_mode(mode):
    return {'safe': 'normal', 'normal': 'off', 'off': 'safe'}[mode]


def mujoco_pucks(model, data):
    """[(name, x, y, z_bottom)] of every puck body in a MuJoCo model (the filter uses the last 3)."""
    import mujoco
    out = []
    for i in range(model.nbody):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or ''
        if n.startswith('puck_'):
            p = data.xpos[i]
            if not S.is_hidden(p[0], p[1]):
                out.append((n, float(p[0]), float(p[1]), float(p[2])))
    return out


if __name__ == '__main__':
    # self-test (no simulator): assist centres, guard stops a finger landing on a puck
    px, py, _ = dict(S.MANUAL_PUCKS)['puck_green_2']
    puck = [(px, py, S.TABLE_TOP_Z)]
    q0 = K.arm_pose(px, py, S.Z_PRE_PICK_TABLE)
    f = SafetyFilter('safe')
    f.reset(q0, S.GRIPPER_OPEN)
    # 1 cm off to the side across the fingers, lowering to the bottom
    a = gripper_yaw(q0)
    off = K.arm_pose(px - 0.012 * math.sin(a), py + 0.012 * math.cos(a), S.LIMITS['shoulder_joint'][0])
    for _ in range(200):
        q, g = f.filter(off, S.GRIPPER_OPEN, 0.05, pucks=puck)
    tx, ty = K.fk_xy(q[0], q[2], q[3])
    err = math.hypot(tx - px, ty - py)
    print(f'assist: 12 mm off -> {err * 1000:.1f} mm off, Z {q[1]:+.3f} ({f.note})')
    assert err < 0.002 and q[1] < -0.14
    # closing and lifting keep the gripper centred on the puck (no jump back to the operator's offset)
    for _ in range(40):
        q, g = f.filter(off, S.GRIP_CLOSE, 0.05, pucks=puck)
    up = list(off); up[1] = 0.0
    for _ in range(60):
        q, g = f.filter(up, S.GRIP_CLOSE, 0.05, pucks=[])
    tx, ty = K.fk_xy(q[0], q[2], q[3])
    print(f'closed + lifted: still {1000 * math.hypot(tx - px, ty - py):.1f} mm from the puck centre')
    assert math.hypot(tx - px, ty - py) < 0.002
    # 4 cm off: a finger is over the puck -> Z must stop above the puck
    f.reset(q0, S.GRIPPER_OPEN)
    off = K.arm_pose(px - 0.06 * math.sin(a), py + 0.06 * math.cos(a), S.LIMITS["shoulder_joint"][0])
    for _ in range(200):
        q, g = f.filter(off, S.GRIPPER_OPEN, 0.05, pucks=puck)
    pb = K.pad_bottom_z(q[1])
    print(f'guard: 60 mm off -> pads stop {1000 * (pb - S.TABLE_TOP_Z - S.PUCK_HEIGHT):.1f} mm above the puck ({f.note})')
    assert pb > S.TABLE_TOP_Z + S.PUCK_HEIGHT
    # speed limit
    f.reset([0, 0, 0, 0], S.GRIPPER_OPEN)
    q, g = f.filter([1.5, -0.1, 1.5, 2.0], S.GRIP_CLOSE, 0.1)
    print('one 0.1 s tick of a big jump ->', [round(v, 3) for v in q], 'gripper', round(g, 4))
    assert abs(q[0] - 0.06) < 1e-9 and abs(g + 0.0025) < 1e-9
    f.estop = True
    assert f.filter([1, 0, 0, 0], 0, 0.1)[0] == q
    print('SAFETY SELF-TEST OK')
