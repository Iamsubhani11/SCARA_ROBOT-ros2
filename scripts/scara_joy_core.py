#!/usr/bin/env python3
"""
Xbox / generic gamepad support shared by the Gazebo and MuJoCo joystick apps.

Two ways to get the pad:
  * JsDevice  - reads /dev/input/js0 directly (Linux joystick API, no ROS and no
                extra packages needed).  This is what the apps use by default.
  * from ROS  - sensor_msgs/Joy on /joy from `ros2 run joy joy_node`.
Both are converted to the ROS convention: stick LEFT and UP are positive.

JoyMapper turns stick deflection into joint SPEEDS (rad/s, m/s) and
integrates them with the real time step, so the arm moves at the same speed
whatever the loop rate is.

Default (Xbox 360 / One, xpad driver and ros2 joy_node) layout:
  axes    0 left X   1 left Y   2 LT   3 right X   4 right Y   5 RT   6 d-pad X   7 d-pad Y
  buttons 0 A  1 B  2 X  3 Y  4 LB  5 RB  6 Back  7 Start  8 Xbox  9 L-stick  10 R-stick

Controls (the robot faces you, so "left" means your left):
  left stick  X : swing the arm left / right (column)      Y : Z up / down
  right stick X : forearm left / right                      Y : turn the wrist
  A : close gripper   B : open gripper   Y : home   LB / RB : slow / fast
  BACK : emergency stop on / off           START : next safety mode (safe -> normal -> off)
"""
import array
import fcntl
import glob
import os
import struct
import threading
import time

try:
    import scara_scene as S
except ImportError:
    from . import scara_scene as S

JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80
JSIOCGAXES, JSIOCGBUTTONS = 0x80016a11, 0x80016a12
JSIOCGNAME = lambda n: 0x80006a13 + (0x10000 * n)      # noqa: E731

LAYOUTS = {
    # name: (left_x, left_y, right_x, right_y)
    'xbox': (0, 1, 3, 4),            # xpad driver / ros2 joy_node
    'gamecontroller': (0, 1, 2, 3),  # ros2 game_controller_node / SDL game-controller order
}
BTN = {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7}


def find_devices():
    return sorted(glob.glob('/dev/input/js*'))


class JsDevice:
    """Background reader for /dev/input/jsN. axes in [-1, 1] with ROS signs (left / up = +)."""

    def __init__(self, path=None):
        devs = [path] if path else find_devices()
        if not devs:
            raise FileNotFoundError('no joystick found (/dev/input/js*) - plug the pad in, or check '
                                    '"ls /dev/input/js*" and that you are in the "input" group')
        self.path = devs[0]
        self.fd = os.open(self.path, os.O_RDONLY)
        self.name = 'gamepad'
        n_axes, n_buttons = 8, 11
        try:
            buf = array.array('B', [0] * 64)
            fcntl.ioctl(self.fd, JSIOCGNAME(len(buf)), buf)
            self.name = buf.tobytes().split(b'\0', 1)[0].decode(errors='ignore') or self.name
            a = array.array('B', [0])
            fcntl.ioctl(self.fd, JSIOCGAXES, a)
            n_axes = a[0]
            b = array.array('B', [0])
            fcntl.ioctl(self.fd, JSIOCGBUTTONS, b)
            n_buttons = b[0]
        except OSError:
            pass                                   # a plain file / FIFO in the tests
        self.axes = [0.0] * max(n_axes, 8)
        self.buttons = [0] * max(n_buttons, 11)
        self.events = 0
        self.last_event = 0.0
        self.alive = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.alive:
            try:
                data = os.read(self.fd, 8)
            except OSError:
                self.alive = False
                return
            if len(data) < 8:
                if not data:
                    time.sleep(0.01)
                continue
            _, value, typ, number = struct.unpack('IhBB', data)
            typ &= ~JS_EVENT_INIT
            if typ == JS_EVENT_AXIS and number < len(self.axes):
                self.axes[number] = -value / 32767.0           # kernel: left / up negative -> ROS: positive
            elif typ == JS_EVENT_BUTTON and number < len(self.buttons):
                self.buttons[number] = 1 if value else 0
            self.events += 1
            self.last_event = time.time()

    def state(self):
        return list(self.axes), list(self.buttons)

    def close(self):
        self.alive = False
        try:
            os.close(self.fd)
        except OSError:
            pass


class JoyMapper:
    """Joint targets from a gamepad.  update(axes, buttons, dt) -> (arm [col, z, fore, wrist], gripper)."""

    SPEED = {'column': 0.9, 'z': 0.06, 'forearm': 0.9, 'wrist': 1.5}     # rad/s, m/s at full deflection

    def __init__(self, layout='xbox', deadzone=0.15, start=None):
        self.layout = layout
        self.deadzone = deadzone
        self.q = list(start) if start else [0.0, 0.0, 0.0, 0.0]
        self.gripper = S.GRIPPER_OPEN
        self.prev_buttons = []
        self.message = None

    def _dz(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        return (abs(v) - self.deadzone) / (1.0 - self.deadzone) * (1 if v > 0 else -1)

    def detect_layout(self, axes):
        """'xbox' (xpad / joy_node order) unless asked otherwise, or the pad only has 4 axes."""
        if self.layout in LAYOUTS:
            return self.layout
        return 'gamecontroller' if len(axes) == 4 else 'xbox'

    def update(self, axes, buttons, dt):
        lx_i, ly_i, rx_i, ry_i = LAYOUTS[self.detect_layout(axes)]
        get = lambda i: self._dz(axes[i]) if i < len(axes) else 0.0      # noqa: E731
        lx, ly, rx, ry = get(lx_i), get(ly_i), get(rx_i), get(ry_i)
        btn = lambda name: buttons[BTN[name]] if BTN[name] < len(buttons) else 0      # noqa: E731
        scale = 0.35 if btn('LB') else (1.8 if btn('RB') else 1.0)
        dt = min(max(dt, 0.0), 0.1)
        sp = self.SPEED
        # robot faces you: stick to YOUR left (+) swings the arm to your left = negative column angle
        self.q[0] -= lx * sp['column'] * scale * dt
        self.q[1] += ly * sp['z'] * scale * dt                      # stick up = arm up
        self.q[2] -= rx * sp['forearm'] * scale * dt
        self.q[3] += ry * sp['wrist'] * scale * dt
        for i, name in enumerate(S.ARM_JOINTS):
            lo, hi = S.LIMITS[name]
            self.q[i] = min(max(self.q[i], lo), hi)
        pressed = lambda name: btn(name) and not (BTN[name] < len(self.prev_buttons) and self.prev_buttons[BTN[name]])  # noqa: E731
        self.message = None
        if btn('A'):
            self.gripper = S.GRIP_CLOSE
        if btn('B'):
            self.gripper = S.GRIPPER_OPEN
        if pressed('Y'):
            self.q = [0.0, 0.0, 0.0, 0.0]
            self.gripper = S.GRIPPER_OPEN
            self.message = 'home'
        if pressed('BACK'):
            self.message = 'estop'
        if pressed('START'):
            self.message = 'mode'
        self.prev_buttons = list(buttons)
        return list(self.q), self.gripper


if __name__ == '__main__':
    # quick check: python3 scripts/scara_joy_core.py  -> prints the pad's live values
    dev = JsDevice()
    print(f'{dev.path}: {dev.name} ({len(dev.axes)} axes, {len(dev.buttons)} buttons) - move the sticks, Ctrl+C to stop')
    try:
        while True:
            a, b = dev.state()
            print('\raxes ' + ' '.join(f'{v:+.2f}' for v in a) + '  buttons ' + ''.join(str(v) for v in b), end='')
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
