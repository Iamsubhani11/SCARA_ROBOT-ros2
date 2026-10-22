#!/usr/bin/env python3
"""
Link between the simulated SCARA (Gazebo / RViz / MuJoCo) and the real
HowToMechatronics SCARA running firmware/scara_twin/scara_twin.ino.

    link = open_link('/dev/ttyACM0')        # or open_link(fake=True) without hardware
    link.home()                              # home on the limit switches (K) - or link.zero_here() at the ready pose
    link.send_joints({'column_joint': 0.3, 'shoulder_joint': -0.05, ...})
    q = link.joints()                        # measured URDF joint values (None until the first status)

The firmware works in the units of the HowToMechatronics GUI (deg, deg, deg,
mm, gripper %); HardwareMap converts to / from the URDF joints of this
package.  Every offset / sign is a parameter (config/twin_hardware.yaml) so
it can be matched to your build.
"""
import math
import threading
import time

try:
    import scara_scene as S
except ImportError:
    from . import scara_scene as S

HW_LIMITS = [(-89.0, 266.0), (-150.0, 150.0), (-162.0, 162.0), (0.0, 170.0)]   # same as the firmware


class HardwareMap:
    """URDF joints <-> firmware units.  Defaults: robot zero = URDF zero."""

    def __init__(self, j1_sign=1.0, j1_offset_deg=0.0, j2_sign=1.0, j2_offset_deg=0.0,
                 j3_sign=1.0, j3_offset_deg=0.0, z_zero_mm=150.0, z_sign=1.0, grip_closed=S.GRIP_CLOSE):
        self.sign = [j1_sign, j2_sign, j3_sign]
        self.off = [j1_offset_deg, j2_offset_deg, j3_offset_deg]
        self.z_zero_mm, self.z_sign = z_zero_mm, z_sign
        self.grip_closed = grip_closed            # finger value (m, negative) that means 100 %

    def to_hw(self, q):
        """dict of URDF joints -> [j1_deg, j2_deg, j3_deg, z_mm, grip_pct]"""
        names = ('column_joint', 'forearm_joint', 'wrist_joint')
        hw = [self.off[i] + self.sign[i] * math.degrees(q.get(n, 0.0)) for i, n in enumerate(names)]
        hw.append(self.z_zero_mm + self.z_sign * 1000.0 * q.get('shoulder_joint', 0.0))
        g = q.get('left_finger_joint', 0.0)
        hw.append(max(0.0, min(100.0, 100.0 * g / self.grip_closed)))
        return hw

    def from_hw(self, hw):
        """[j1_deg, j2_deg, j3_deg, z_mm, grip_pct] -> dict of URDF joints"""
        q = {}
        for i, n in enumerate(('column_joint', 'forearm_joint', 'wrist_joint')):
            q[n] = math.radians((hw[i] - self.off[i]) * self.sign[i])
        q['shoulder_joint'] = (hw[3] - self.z_zero_mm) / (1000.0 * self.z_sign)
        g = self.grip_closed * hw[4] / 100.0
        q['left_finger_joint'] = g
        q['right_finger_joint'] = g
        return q


class _LinkBase:
    def __init__(self, hw_map=None, on_message=None):
        self.map = hw_map or HardwareMap()
        self.on_message = on_message or (lambda kind, text: print(f'[robot {kind}] {text}'))
        self.state = None            # (t, hw list, moving, homed)
        self.motors_on = True
        self.conveyor_on = False
        self.lock = threading.Lock()
        self._last_sent = None

    # ---- parsing (shared by the real and the fake link)
    def _handle(self, line):
        line = line.strip()
        if not line:
            return
        if line[0] == 'P':
            try:
                v = [float(x) for x in line.split()[1:]]
                with self.lock:
                    self.state = (time.time(), v[:5], bool(v[5]), bool(v[6]))
                    if len(v) >= 9:                       # newer firmware: motors + conveyor too
                        self.motors_on, self.conveyor_on = bool(v[7]), bool(v[8])
            except (ValueError, IndexError):
                pass
        elif line[0] in 'IWE':
            self.on_message({'I': 'info', 'W': 'warning', 'E': 'error'}[line[0]], line[2:])

    # ---- high level API (same buttons as the Mechamind "SCARA control panel")
    def zero_here(self):
        """The arm is at the ready pose (straight out, Z 150 mm): set that as zero."""
        self.write('H')
        self._last_sent = None

    def home(self):
        """Home on the 4 limit switches like the original HowToMechatronics sketch (default build)."""
        self.write('K')
        self._last_sent = None

    def free_motors(self):
        self.write('F')

    def lock_motors(self):
        self.write('L')
        self._last_sent = None

    def conveyor(self, on):
        self.write(f'C {1 if on else 0}')

    def stop(self):
        self.write('S')

    def enable(self, on=True):
        self.lock_motors() if on else self.free_motors()

    def set_speed(self, steps_per_s=4000, accel=2000):
        self.write(f'V {steps_per_s:.0f} {accel:.0f}')

    def send_joints(self, q, min_change=0.05):
        """Send a URDF joint target; skipped if nothing changed by more than min_change (deg / mm / %)."""
        hw = self.map.to_hw(q)
        if self._last_sent and max(abs(a - b) for a, b in zip(hw, self._last_sent)) < min_change:
            return False
        self._last_sent = hw
        self.write('M ' + ' '.join(f'{v:.2f}' for v in hw))
        return True

    def joints(self):
        with self.lock:
            if self.state is None:
                return None
            return self.map.from_hw(self.state[1])

    def status(self):
        with self.lock:
            return self.state

    def out_of_range(self, q):
        """Names of joints whose target the robot cannot reach (soft limits)."""
        hw = self.map.to_hw(q)
        names = ['column_joint', 'forearm_joint', 'wrist_joint', 'shoulder_joint']
        return [n for n, v, (lo, hi) in zip(names, hw, HW_LIMITS) if not lo - 1e-6 <= v <= hi + 1e-6]


class SerialLink(_LinkBase):
    def __init__(self, port, baud=115200, **kw):
        super().__init__(**kw)
        import serial                     # pip install pyserial / sudo apt install python3-serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)                   # the UNO resets when the port opens
        self._stop = False
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        buf = b''
        while not self._stop:
            try:
                buf += self.ser.read(256)
            except Exception as e:        # cable pulled
                self.on_message('error', f'serial: {e}')
                return
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._handle(line.decode('ascii', 'ignore'))

    def write(self, text):
        self.ser.write((text + '\n').encode('ascii'))

    def close(self):
        self._stop = True
        self.ser.close()


class FakeLink(_LinkBase):
    """Behaves like the firmware (speed-limited joints, soft limits, homing) - no hardware needed."""
    SPEED = [90.0, 110.0, 400.0, 40.0]     # deg/s, deg/s, deg/s, mm/s

    def __init__(self, **kw):
        super().__init__(**kw)
        self.pos = [-30.0, 40.0, 10.0, 60.0]   # unknown pose before homing
        self.target = list(self.pos)
        self.grip = 0.0
        self.homed = False
        self.motors = True
        self.belt = False
        self._stop = False
        threading.Thread(target=self._sim, daemon=True).start()
        self._handle('I SCARA twin firmware ready (FAKE) - send K to home (or H at the ready pose)')

    def write(self, text):
        p = text.split()
        if not p:
            return
        c = p[0].upper()
        if c == 'M':
            if not self.homed:
                return self._handle('W not zeroed - put the arm at the ready pose and send H')
            if not self.motors:
                return self._handle('W motors are free - send L to lock them')
            v = [float(x) for x in p[1:6]]
            self.target = [min(max(v[i], lo), hi) for i, (lo, hi) in enumerate(HW_LIMITS)]
            self.grip = min(max(v[4], 0.0), 100.0)
        elif c in ('H', 'Z'):
            self.pos = [0.0, 0.0, 0.0, 150.0]          # "the arm is at the ready pose now"
            self.target = list(self.pos)
            self.homed = True
            self._handle('I ZERO SET HERE')
        elif c == 'K':                                 # like the original homing(): parks at Z 100 mm
            self._handle('I HOMING')
            self.pos = [0.0, 0.0, 0.0, 100.0]
            self.target = list(self.pos)
            self.homed = True
            self._handle('I HOMED')
        elif c == 'G' and len(p) > 1:
            self._handle(f'I SERVO {int(float(p[1]))}')
        elif c == 'F':
            self.motors = False
            self._handle('I MOTORS FREE')
        elif c == 'L':
            self.motors = True
            self.target = list(self.pos)
            self._handle('I MOTORS LOCKED')
        elif c == 'C':
            self.belt = len(p) > 1 and p[1] != '0'
            self._handle('I CONVEYOR ON' if self.belt else 'I CONVEYOR OFF')
        elif c == 'S':
            self.target = list(self.pos)

    def _sim(self):
        dt = 0.01
        n = 0
        while not self._stop:
            time.sleep(dt)
            for i in range(4):
                d = self.target[i] - self.pos[i]
                step = self.SPEED[i] * dt
                self.pos[i] += max(-step, min(step, d))
            n += 1
            if n % 5 == 0:
                moving = any(abs(t - p) > 1e-3 for t, p in zip(self.target, self.pos))
                self._handle('P ' + ' '.join(f'{v:.2f}' for v in self.pos)
                             + f' {self.grip:.1f} {int(moving)} {int(self.homed)} {int(self.motors)} {int(self.belt)}')

    def close(self):
        self._stop = True


def open_link(port=None, baud=115200, fake=False, **kw):
    return FakeLink(**kw) if fake or not port else SerialLink(port, baud, **kw)


if __name__ == '__main__':
    # quick self-test with the fake robot
    m = HardwareMap()
    q = {'column_joint': 0.5, 'forearm_joint': -1.0, 'wrist_joint': 0.3, 'shoulder_joint': -0.1,
         'left_finger_joint': S.GRIP_CLOSE}
    back = m.from_hw(m.to_hw(q))
    assert all(abs(back[k] - v) < 1e-9 for k, v in q.items()), back
    link = open_link(fake=True)
    link.zero_here()
    link.send_joints(q)
    time.sleep(1.5)
    print('measured', {k: round(v, 3) for k, v in link.joints().items()})
    time.sleep(2.5)
    got = link.joints()
    err = max(abs(got[k] - q[k]) for k in q)
    print('after 4 s, worst joint error', round(err, 4))
    link.conveyor(True)
    link.free_motors()
    time.sleep(0.3)
    assert link.conveyor_on and not link.motors_on, (link.conveyor_on, link.motors_on)
    assert err < 1e-3
    print('twin protocol self-test OK')
