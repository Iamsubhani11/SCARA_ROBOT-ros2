#!/usr/bin/env python3
"""
SCARA Twin Control - one app to drive the simulation, the real robot, or both
(the digital twin), from a browser GUI and from the SCARA Digital Twin phone app.

    python3 server/scara_twin_server.py                       # MuJoCo table + GUI (no robot yet)
    python3 server/scara_twin_server.py --port /dev/ttyACM0   # ... and connect the real robot
    python3 server/scara_twin_server.py --fake                # ... with a software robot (no hardware)
    python3 server/scara_twin_server.py --sim gazebo          # control the running Gazebo cell instead
    python3 server/scara_twin_server.py --sim gazebo --world sort   # ... the server starts Gazebo itself
    python3 server/scara_twin_server.py --sim none --port /dev/ttyACM0   # real robot only

The control page opens in your browser at http://localhost:8080 (any phone / laptop on
the same Wi-Fi can open http://<this-pc-ip>:8080 too).  It is the same page as the
SCARA Digital Twin phone app: 3-D view, arm camera, joysticks, Auto / Teach / Robot tabs,
voice and hand control, and "Try offline".

Phone app: tap the status at the top, type this PC's IP address and port 9090,
Connect.  (This server speaks a small rosbridge-compatible WebSocket protocol on
port 9090, so rosbridge_server is not needed - do not run both at the same time.)

Control target (top of the GUI):
  Simulation  - commands move the simulated robot only
  Real robot  - commands move the real robot; the simulation shows where it is
  Both        - digital twin: the simulation moves and the real robot copies it

Every manual command (sliders, keys, buttons, phone app, teach & replay) goes
through the safety filter (common/scara_safety.py): speed limits, pick assist,
puck / table guards and the emergency stop (STOP button, Space key).
Teach & replay: save poses, play them back in order (stored in ~/.scara_teach_points.json).
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import base64
import collections
import glob
import hashlib
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_scene as S                                   # noqa: E402
import scara_safety as SF                                 # noqa: E402
import scara_kinematics as K                              # noqa: E402
from scara_twin_protocol import HW_LIMITS, open_link      # noqa: E402
from scara_twin_protocol import save_map_change  # noqa: E402

TEACH_FILE = os.path.join(os.path.expanduser('~'), '.scara_teach_points.json')



def _find_ui():
    """The control page is the phone app's web page (app/ui): the same 3-D view, joysticks and
    offline mode in the browser.  Source tree: server/../app/ui; colcon install: share/.../app/ui."""
    cands = [os.path.join(HERE, '..', 'app', 'ui'),
             os.path.join(HERE, '..', '..', 'share', 'scara_description', 'app', 'ui')]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('scara_description'), 'app', 'ui'))
    except Exception:
        pass
    for c in cands:
        if os.path.isfile(os.path.join(c, 'index.html')):
            return os.path.abspath(c)
    return os.path.abspath(cands[0])


UI_DIR = _find_ui()
MIME = {'.html': 'text/html; charset=utf-8', '.js': 'application/javascript; charset=utf-8',
        '.css': 'text/css; charset=utf-8', '.ttf': 'font/ttf', '.png': 'image/png', '.svg': 'image/svg+xml',
        '.json': 'application/json', '.ico': 'image/x-icon', '.webmanifest': 'application/manifest+json',
        '.wasm': 'application/wasm', '.task': 'application/octet-stream'}   # MediaPipe (app/ui/lib/mediapipe)
JOINTS = S.ARM_JOINTS + ['left_finger_joint']
LIMITS = {**{n: S.LIMITS[n] for n in S.ARM_JOINTS}, 'left_finger_joint': (S.LIMITS['left_finger_joint'][0], 0.0)}


def list_ports():
    try:
        from serial.tools import list_ports as lp
        ports = [p.device for p in lp.comports()]
    except Exception:
        ports = []
    for pat in ('/dev/ttyACM*', '/dev/ttyUSB*'):
        ports += [p for p in glob.glob(pat) if p not in ports]
    return sorted(ports)


# ---------------------------------------------------------------------------
# Simulation backends
# ---------------------------------------------------------------------------
class NoSim:
    kind = 'none'
    label = 'none'
    autonomous = False
    has_belt = False
    commands = False

    def joints(self):
        return None

    def set_targets(self, q):
        pass

    def belt(self, on, speed=None):
        pass

    def request(self, text):
        return False

    def info(self):
        return {}

    def pucks(self):
        return None


class MujocoTable(NoSim):
    """The manual-control table (no conveyor) in a MuJoCo viewer; runs in the main thread."""
    kind = 'mujoco'
    label = 'MuJoCo - table with pucks'

    def __init__(self):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(S.pkg_file('mujoco', 'models', 'scara_manual_mujoco.xml'))
        self.data = mujoco.MjData(self.model)
        self.act = {mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(self.model.nu)}
        self.targets = {n: 0.0 for n in JOINTS}
        self.q = {n: 0.0 for n in JOINTS}
        self.lock = threading.Lock()
        self.running = True
        self._pucks = []

    def joints(self):
        with self.lock:
            return dict(self.q)

    def pucks(self):
        with self.lock:
            return list(self._pucks)

    def set_targets(self, q):
        with self.lock:
            self.targets.update({k: v for k, v in q.items() if k in self.targets})

    def run(self, stop_event, should_exit=None):
        import mujoco.viewer
        names = {'column_joint': 'act_column', 'shoulder_joint': 'act_shoulder', 'forearm_joint': 'act_forearm',
                 'wrist_joint': 'act_wrist'}
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            wall0, sim0 = time.time(), self.data.time
            while viewer.is_running() and not stop_event.is_set() and not (should_exit and should_exit()):
                now = time.time()
                with self.lock:
                    t = dict(self.targets)
                with viewer.lock():
                    for j, a in names.items():
                        self.data.ctrl[self.act[a]] = t[j]
                    self.data.ctrl[self.act['act_left_finger']] = t['left_finger_joint']
                    self.data.ctrl[self.act['act_right_finger']] = t['left_finger_joint']
                    n = 0
                    while self.data.time - sim0 < now - wall0 and n < 50:
                        self.mj.mj_step(self.model, self.data)
                        n += 1
                    if n == 50:
                        wall0, sim0 = now, self.data.time
                    q = {j: float(self.data.joint(j).qpos[0]) for j in JOINTS}
                    pucks = SF.mujoco_pucks(self.model, self.data)
                with self.lock:
                    self.q = q
                    self._pucks = pucks
                viewer.sync()
                time.sleep(max(0.0, 0.016 - (time.time() - now)))
        self.running = False
        if not (should_exit and should_exit()):
            stop_event.set()                     # the window was closed: end the program


class MujocoWorlds(NoSim):
    """MuJoCo with all three worlds, switchable from the GUI / phone app (Auto tab):
    'manual' (the table with pucks), 'sort' (conveyor, 4 bins), 'command' (conveyor, one bin).
    The viewer window closes and opens again with the new world."""
    kind = 'mujoco'
    WORLDS = ('manual', 'sort', 'command')

    def __init__(self, world='manual'):
        self.world = world if world in self.WORLDS else 'manual'
        self.inner = NoSim()
        self.switch_to = None

    # everything the twin core asks goes to the world that runs right now
    label = property(lambda self: 'MuJoCo - ' + {'manual': 'table with pucks', 'sort': 'sorting cell (4 bins)',
                                                 'command': 'command cell (one bin)'}[self.world])
    autonomous = property(lambda self: getattr(self.inner, 'autonomous', False))
    commands = property(lambda self: getattr(self.inner, 'commands', False))
    has_belt = property(lambda self: getattr(self.inner, 'has_belt', False))
    belt_running = property(lambda self: getattr(self.inner, 'belt_running', None))
    switching = property(lambda self: self.switch_to)

    def joints(self):
        return self.inner.joints()

    def set_targets(self, q):
        self.inner.set_targets(q)

    def belt(self, on, speed=None):
        self.inner.belt(on, speed)

    def request(self, text):
        return self.inner.request(text)

    def pucks(self):
        return self.inner.pucks()

    def info(self):
        return {**self.inner.info(), 'world': self.switch_to or self.world, 'worlds': list(self.WORLDS),
                'switching': self.switch_to}

    def set_world(self, w):
        if w not in self.WORLDS:
            return f'world must be one of {self.WORLDS}'
        if w != self.world:
            self.switch_to = w
        return None

    def run(self, stop_event):
        import collections
        import launch_mujoco as LM
        while not stop_event.is_set():
            switching = lambda: self.switch_to is not None or stop_event.is_set()     # noqa: E731
            if self.world == 'manual':
                self.inner = MujocoTable()
                self.inner.run(stop_event, should_exit=switching)
            else:
                backend = LM.CellBackend(self.world, collections.deque())
                self.inner = backend
                model = self.inner_model = LM.mujoco.MjModel.from_xml_path(
                    LM.XML if self.world == 'sort' else LM.XML_COMMAND)
                data = LM.mujoco.MjData(model)
                LM.mujoco.mj_forward(model, data)
                LM.run_cell(model, data, self.world, backend=backend, should_exit=switching)
                if self.switch_to is None and not stop_event.is_set():
                    stop_event.set()             # the window was closed
            if self.switch_to:
                self.world, self.switch_to = self.switch_to, None
                self.inner = NoSim()


class GazeboSim(NoSim):
    """The Gazebo cell, through ROS 2 (any mode: sort / command / manual).

    Attached to a Gazebo you started yourself, or started by this server (--world).  Either way
    the GUI / phone app can switch the world: the server stops the running Gazebo launch and
    starts gazebo.launch.py again with the new mode (about 20-40 s)."""
    kind = 'gazebo'
    WORLDS = ('manual', 'sort', 'command')
    PUCK_TIMEOUT = 2.0                           # s without a pose update: the puck is gone

    def __init__(self, world=None, headless=False):
        import rclpy
        from builtin_interfaces.msg import Duration
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        self.rclpy, self.Duration, self.JT, self.JTP, self.String = rclpy, Duration, JointTrajectory, JointTrajectoryPoint, String
        rclpy.init()
        self.node = rclpy.create_node('scara_twin_control')
        self.arm_pub = self.node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.node.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.cmd_pub = self.node.create_publisher(String, '/scara/pick_color', 10)
        self.q = {}
        self.q_time = 0.0
        self.status = collections.deque(maxlen=20)
        self.node.create_subscription(JointState, '/joint_states', self._js, 20)
        self.node.create_subscription(String, '/scara/status', lambda m: self.status.append(m.data), 20)
        self.node.create_subscription(String, '/scara/sorter_state', self._sorter_state, 10)
        self.node.create_subscription(TFMessage, '/scara/world_poses', self._poses, 10)
        from rosgraph_msgs.msg import Clock
        self.sim_time = None                     # Gazebo's clock: the safety filter runs in sim time
        self.node.create_subscription(Clock, '/clock', self._clock, 10)
        self._pucks = {}
        self.sorter_state = {}
        self.sorter_time = 0.0
        self._last_arm = None
        self._last_grip = None
        self.last_status = 0.0
        self.headless = headless
        self.proc = None                         # the Gazebo launch this server started (if any)
        self.launched = None                     # its mode
        self.switching = None                    # world being started right now
        self._log = lambda text, level='info': print(f'[{level}] {text}', flush=True)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
        if world:
            self.set_world(world, force=True)

    label = property(lambda self: 'Gazebo - ' + {'manual': 'table with pucks', 'sort': 'sorting cell (4 bins)',
                                                'command': 'command cell (one bin)'}[self.world])

    @property
    def alive(self):
        return time.time() - self.q_time < 3.0

    @property
    def autonomous(self):
        # the colour sorter (sort / command mode) is driving the arm
        return not self.switching and self.cmd_pub.get_subscription_count() > 0

    @property
    def commands(self):
        return self.autonomous and self.world == 'command'

    @property
    def has_belt(self):
        return self.autonomous

    @property
    def world(self):
        if self.switching:
            return self.switching
        mode = self.sorter_state.get('mode') if time.time() - self.sorter_time < 5.0 else None
        if mode in ('sort', 'command'):
            return mode
        if self.cmd_pub.get_subscription_count() > 0:
            return self.launched if self.launched in ('sort', 'command') else 'sort'
        return 'manual' if (self.alive or not self.launched) else self.launched

    def _clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.q[n] = p
        self.q_time = time.time()

    def _poses(self, msg):
        now = time.time()
        for t in msg.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_'):
                p = t.transform.translation
                self._pucks[name] = (p.x, p.y, p.z, now)

    def _sorter_state(self, msg):
        try:
            self.sorter_state = json.loads(msg.data)
            self.sorter_time = time.time()
        except ValueError:
            pass

    def pucks(self):
        # only pucks seen recently and not parked in the hidden pool: nothing from an earlier world stays behind
        now = time.time()
        return [(n, x, y, z) for n, (x, y, z, t) in list(self._pucks.items())
                if now - t < self.PUCK_TIMEOUT and not S.is_hidden(x, y)]

    def joints(self):
        if self.switching or not self.alive:
            return None
        return {n: self.q[n] for n in JOINTS if n in self.q} if all(n in self.q for n in S.ARM_JOINTS) else None

    def _traj(self, pub, names, pos):
        m = self.JT()
        m.joint_names = list(names)
        pt = self.JTP()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start = self.Duration(sec=0, nanosec=150_000_000)
        m.points = [pt]
        pub.publish(m)

    def set_targets(self, q):
        if self.autonomous or self.switching:
            return                               # the sorter owns the arm / Gazebo is restarting
        cur = self.joints() or {}
        arm = [q.get(n, cur.get(n, 0.0)) for n in S.ARM_JOINTS]
        # publish only while something changes, so a joystick / keyboard node can
        # drive Gazebo too while this window is open and idle
        if self._last_arm is None or max(abs(a - b) for a, b in zip(arm, self._last_arm)) > 1e-5:
            self._traj(self.arm_pub, S.ARM_JOINTS, arm)
            self._last_arm = arm
        if 'left_finger_joint' in q:
            g = q['left_finger_joint']
            if self._last_grip is None or abs(g - self._last_grip) > 1e-5:
                self._traj(self.grip_pub, S.GRIPPER_JOINTS, [g, g])
                self._last_grip = g

    def belt(self, on, speed=None):
        self.cmd_pub.publish(self.String(data='start' if on else 'stop'))

    def request(self, text):
        self.cmd_pub.publish(self.String(data=text))
        return True

    def info(self):
        st = self.sorter_state if time.time() - self.sorter_time < 5.0 else {}
        return {'status': list(self.status)[-6:], 'active': st.get('active', []), 'sorted': st.get('sorted'),
                'sequence': st.get('sequence', []), 'count_left': st.get('count_left'),
                'tracking': st.get('tracking', False), 'tracked': st.get('tracked', 0),
                'cell_mode': self.world if self.world != 'manual' else None, 'world': self.world,
                'worlds': list(self.WORLDS), 'switching': self.switching}

    # ---- switching worlds: restart gazebo.launch.py with another mode
    def set_world(self, w, force=False):
        if w not in self.WORLDS:
            return f'world must be one of {self.WORLDS}'
        if self.switching:
            return f'Gazebo is still starting the {self.switching} world - wait a moment'
        if w == self.world and not force:
            return None
        self.switching = w
        threading.Thread(target=self._relaunch, args=(w,), daemon=True).start()
        return None

    def _pids(self, pattern):
        import subprocess
        try:
            out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()
        except OSError:
            return []
        return [int(p) for p in out if int(p) != os.getpid()]

    def _stop_gazebo(self):
        import signal
        import subprocess
        procs = []
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except OSError:
                pass
            procs.append(self.proc.pid)
        else:                                    # a Gazebo someone started in another terminal
            for pid in self._pids('gazebo.launch.py'):
                try:
                    os.kill(pid, signal.SIGINT)
                    procs.append(pid)
                except OSError:
                    pass
        t0 = time.time()
        while time.time() - t0 < 25 and any(os.path.exists(f'/proc/{p}') for p in procs):
            if self.proc and self.proc.poll() is not None:
                break
            time.sleep(0.3)
        for pat in ('gazebo.launch.py', 'gz sim', 'gz-sim', 'parameter_bridge', 'scara_color_sorter',
                    'scara_puck_spawner', 'scara_vision_node', 'robot_state_publisher', 'spawner'):
            for pid in self._pids(pat):
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
        time.sleep(1.5)
        for pid in self._pids('gz sim') + self._pids('gazebo.launch.py'):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        if self.proc:
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass

    def _relaunch(self, w):
        import subprocess
        try:
            running = bool(self._pids('gazebo.launch.py')) or (self.proc and self.proc.poll() is None)
            if running:
                self._log(f'Gazebo: stopping the {self.launched or "running"} world ...')
                self._stop_gazebo()
            self._pucks.clear()
            self.sorter_state, self.sorter_time = {}, 0.0
            self.q, self.q_time = {}, 0.0
            self._last_arm = self._last_grip = None
            self.status.clear()
            log = os.path.join(os.path.expanduser('~'), f'.scara_gazebo_{w}.log')
            cmd = ['ros2', 'launch', 'scara_description', 'gazebo.launch.py', f'mode:={w}']
            if self.headless:
                cmd.append('headless:=true')
            self._log(f'Gazebo: starting the {w} world (takes 20-40 s; log in {log})')
            with open(log, 'w') as fh:
                self.proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True)
            self.launched = w
            t0 = time.time()
            while time.time() - t0 < 120:
                if self.proc.poll() is not None:
                    self._log(f'Gazebo stopped while starting - see {log}', 'error')
                    break
                ready = self.alive and all(n in self.q for n in S.ARM_JOINTS)
                if ready and (w == 'manual' or self.cmd_pub.get_subscription_count() > 0):
                    self._log(f'Gazebo: the {w} world is running')
                    break
                time.sleep(0.5)
            else:
                self._log(f'Gazebo did not come up within 2 minutes - see {log}', 'error')
        except Exception as e:
            self._log(f'Gazebo restart failed: {e}', 'error')
        finally:
            self.switching = None

    def shutdown(self):
        if self.proc and self.proc.poll() is None:
            self._stop_gazebo()


# ---------------------------------------------------------------------------
# The twin core: robot link, control target, commands, log
# ---------------------------------------------------------------------------
class TwinCore:
    def __init__(self, sim):
        self.sim = sim
        self.link = None
        self.port = None
        self.lock = threading.RLock()
        self.log = collections.deque(maxlen=200)
        self.log_id = 0
        self.targets = {n: 0.0 for n in JOINTS}           # what the operator asked for
        self.cmd = dict(self.targets)                      # what was sent (after the safety filter)
        self.control = 'sim' if sim.kind != 'none' else 'robot'
        self.belt_on = False
        self.belt_speed = S.BELT_SPEED
        self.app_clients = 0
        self.app_last = 0.0
        self._belt_sent = 0.0
        self.safety = SF.SafetyFilter('safe')
        self._need_reset = True                            # first loop: start from the measured pose
        self._last_user = 0.0
        self._err_hist = []
        self.teach = self._load_teach()
        self.playing = None                                # replay thread state
        self.stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- log
    def say(self, text, level='info'):
        with self.lock:
            self.log_id += 1
            self.log.append({'id': self.log_id, 't': time.strftime('%H:%M:%S'), 'level': level, 'text': text})
        print(f'[{level}] {text}', flush=True)

    # ---- robot connection
    def connect(self, port=None, fake=False):
        if not port and not fake:
            return 'Pick the robot\'s serial port first (or use the software robot)'
        self.disconnect()
        try:
            self.link = open_link(port, fake=fake, on_message=lambda k, t: self.say(f'robot: {t}', k))
            self.port = 'fake robot' if fake or not port else port
            self.say(f'Connected to the robot on {self.port}. Press "Home on limit switches" (K) - or, without switches, put the arm at the ready pose and press "Zero here".')
            if self.sim.kind != 'none' and self.control == 'sim':
                self.control = 'both'
        except Exception as e:
            self.link = None
            msg, low = str(e), str(e).lower()
            hint = ('add yourself to the dialout group: sudo usermod -aG dialout $USER, then log out and in'
                    if 'permission' in low else
                    'another program has the port open - close the Arduino IDE serial monitor' if 'busy' in low or 'resource' in low else
                    'install pyserial: python3 -m pip install pyserial' if 'serial' in low and 'module' in low else
                    'check the cable and pick the port again (Refresh)')
            self.say(f'Could not open {port}: {msg} - {hint}', 'error')
            return f'Could not open {port}: {hint}'
        return None

    def disconnect(self):
        if self.sim.kind == 'hardware' and self.link is getattr(self.sim, 'link', None):
            self.sim.pause()
        if self.link:
            try:
                self.link.close()
            except Exception:
                pass
            self.say(f'Disconnected from {self.port}')
        self.link = None
        self.port = None
        if self.control in ('robot', 'both') and self.sim.kind != 'none':
            self.control = 'sim'
        self._resync()

    # ---- commands from the GUI or the phone app
    def cell_busy(self):
        """An autonomous cell owns the arm: the simulated one, or the real camera cell."""
        return self.sim.autonomous and (self.control in ('sim', 'both') or self.sim.kind == 'hardware')

    def manual_blocked(self):
        if self.sim.kind == 'hardware' and self.sim.autonomous:
            return 'The camera cell is running - pause it (Conveyor stop / Space) to move the arm by hand'
        if self.control in ('sim', 'both') and self.sim.autonomous:
            return ('The cell is sorting by itself - to move the arm by hand use the manual world '
                    '(launch_mujoco.py --gui, or Gazebo mode:=manual)')
        return None

    def set_target(self, q, source='gui'):
        err = self.manual_blocked()
        if err:
            return err
        lim = self.limits()
        q = {k: min(max(float(v), lim[k][0]), lim[k][1]) for k, v in q.items() if k in lim}
        with self.lock:
            self.targets.update(q)
            self._last_user = time.time()
        return None

    def limits(self):
        """Joint ranges for the arm being commanded: the URDF's, and with the real robot in the loop
        also the robot's own soft limits - so the simulation never goes where the robot cannot follow."""
        link = self.link
        if not link or self.control == 'sim' or not hasattr(link, 'urdf_limits'):
            return LIMITS
        hw = link.urdf_limits()
        return {k: ((max(lo, hw[k][0]), min(hi, hw[k][1])) if k in hw else (lo, hi)) for k, (lo, hi) in LIMITS.items()}

    @staticmethod
    def _hw(t):
        return {**t, 'right_finger_joint': t.get('left_finger_joint', 0.0)}

    def _measured(self):
        """Joints of whatever is being commanded (for the safety filter)."""
        if self.control in ('sim', 'both'):
            return self.sim.joints()
        return self.link.joints() if self.link else None

    def _resync(self):
        """Targets and safety filter start again from where the commanded arm really is."""
        meas = self._measured()
        if meas:
            with self.lock:
                self.targets.update({k: v for k, v in meas.items() if k in self.targets})
                self.safety.reset([meas.get(n, 0.0) for n in S.ARM_JOINTS], meas.get('left_finger_joint'))

    def command(self, c, args):
        """Returns an error string or None."""
        link = self.link
        need_robot = {'zero_here', 'free', 'lock', 'home_switches'}
        if c in need_robot and not link:
            return 'Connect the robot first'
        if self.sim.kind == 'hardware' and c in ('connect', 'disconnect'):
            return 'Camera-cell mode: the robot port is set when the server starts (--port)'
        if self.sim.kind == 'hardware' and c == 'control' and args.get('mode') != 'robot':
            return 'Camera-cell mode drives the real robot only'
        if c == 'connect':
            return self.connect(args.get('port') or None, bool(args.get('fake')))
        elif c == 'disconnect':
            self.disconnect()
        elif c == 'control':
            mode = args.get('mode')
            if mode not in ('sim', 'robot', 'both'):
                return 'unknown control target'
            if mode in ('robot', 'both') and not link:
                return 'Connect the robot first'
            if mode in ('sim', 'both') and self.sim.kind == 'none':
                return 'No simulation running (start with --sim mujoco or --sim gazebo)'
            self.control = mode
            self.say({'sim': 'Controlling the simulation', 'robot': 'Controlling the real robot',
                      'both': 'Digital twin: the real robot copies the simulation'}[mode])
            self._resync()
        elif c == 'jog':
            return self.set_target({args['joint']: args['value']})
        elif c == 'jog_by':
            # one joint {'joint', 'delta'} or several at once {'deltas': {joint: delta}} (phone joysticks)
            deltas = args.get('deltas') or {args['joint']: args['delta']}
            with self.lock:
                q = {j: self.targets.get(j, 0.0) + float(d) for j, d in deltas.items() if j in self.targets}
            return self.set_target(q)
        elif c == 'save_recording':
            # a motion recorded in the phone app (Motion graph -> Save CSV) is kept on the PC
            import re
            name = re.sub(r'[^A-Za-z0-9_.-]', '_', str(args.get('name') or 'scara_motion.csv'))[:80]
            if not name.endswith('.csv'):
                name += '.csv'
            csv = str(args.get('csv') or '')
            if not csv or len(csv) > 5_000_000:
                return 'Nothing to save (or the recording is too big)'
            folder = os.path.join(os.path.expanduser('~'), 'scara_recordings')
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, name)
            with open(path, 'w') as fh:
                fh.write(csv)
            self.say(f'Recording saved: {path}')
        elif c == 'teach_set':
            # a whole program at once (Teach tab: Load / Import): a list of poses, checked and clamped
            pts = args.get('points')
            if not isinstance(pts, list) or len(pts) > 500:
                return 'A program is a list of up to 500 poses'
            lim, clean = LIMITS, []
            for p in pts:
                if not isinstance(p, dict):
                    return 'Every pose must be a set of joint values'
                try:
                    q = {k: round(min(max(float(p.get(k, 0.0)), lim[k][0]), lim[k][1]), 4) for k in JOINTS}
                except (TypeError, ValueError):
                    return 'A pose has a value that is not a number'
                clean.append(q)
            self.playing = None
            self.teach = clean
            self._save_teach()
            self.say(f'Program loaded: {len(clean)} poses')
        elif c == 'save_program':
            # Teach tab -> Export in the phone app: the program file is kept on the PC
            import re
            name = re.sub(r'[^A-Za-z0-9_.-]', '_', str(args.get('name') or 'scara_program'))[:60]
            if not name.endswith('.json'):
                name += '.json'
            folder = os.path.join(os.path.expanduser('~'), 'scara_programs')
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, name)
            with open(path, 'w') as fh:
                json.dump({'scara_program': 1, 'name': name[:-5], 'poses': self.teach}, fh, indent=1)
            self.say(f'Program saved: {path}')
        elif c == 'fake_freeze':
            # tests only: the software robot stops answering for a few seconds, like a hung Arduino
            if not link or not hasattr(link, 'frozen_until'):
                return 'only the software robot can do that'
            link.frozen_until = time.time() + float(args.get('seconds', 4.0))
        elif c == 'dir_nudge':
            # direction check: move ONE joint a little (10 deg / 10 mm); the real arm and the 3-D arm must
            # turn the same way. The robot must be homed and in Real robot or Twin.
            j = args.get('joint')
            if j not in ('column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint'):
                return 'unknown joint'
            st = link.status() if link else None
            if not link or not st or not st[3]:
                return 'Connect and home the robot first'
            if self.control == 'sim':
                return 'Choose Real robot or Twin at the top first'
            step = 0.01 if j == 'shoulder_joint' else math.radians(10)
            lo, hi = self.limits()[j]
            with self.lock:
                cur = self.targets.get(j, 0.0)
            # nudge towards the middle of the range, so it never runs into a limit
            new = cur + (step if cur < (lo + hi) / 2 else -step)
            self.say(f'Direction check: {j.split("_")[0]} moves {"+" if new > cur else "-"}'
                     f'{"10 mm" if j == "shoulder_joint" else "10 deg"} - did the real arm move like the 3-D arm?')
            return self.set_target({j: new})
        elif c == 'dir_flip':
            j = args.get('joint')
            if not link or j not in ('column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint'):
                return 'Connect the robot first'
            key, val = link.map.flip(j)
            path = save_map_change(key, val)
            meas = link.joints()
            # the robot did not move - only how its numbers are read; let the targets (and the sim) follow it
            if meas:
                with self.lock:
                    self.targets.update({k: v for k, v in meas.items() if k in self.targets})
                    self.safety.reset([meas.get(n, 0.0) for n in S.ARM_JOINTS], meas.get('left_finger_joint'))
            self.say(f'{j.split("_")[0]} direction reversed ({key} = {val:+.0f}, saved in {path})')
        elif c == 'world':
            if not hasattr(self.sim, 'set_world'):
                return ('This simulation has only one world' if self.sim.kind != 'hardware'
                        else 'The real camera cell has no worlds - start it with --mode sort | command')
            w = str(args.get('world', ''))
            if w == getattr(self.sim, 'world', None) and not getattr(self.sim, 'switching', None):
                return None
            err = self.sim.set_world(w)
            if err:
                return err
            self.playing = None
            self.belt_on = False
            self._need_reset = True
            self.say(f'Switching to the {w} world' + (' (the MuJoCo window reopens)' if self.sim.kind == 'mujoco'
                                                        else ' - Gazebo restarts, about 30 s'))
        elif c == 'move':
            # absolute joint target for several joints at once (the app's XY / tilt drive)
            return self.set_target({k: float(v) for k, v in (args.get('q') or {}).items()})
        elif c == 'all_zero':
            return self.set_target({n: 0.0 for n in S.ARM_JOINTS})
        elif c == 'gripper':
            v = S.GRIP_CLOSE if args.get('close') else S.GRIPPER_OPEN
            if 'value' in args:
                v = float(args['value'])
            return self.set_target({'left_finger_joint': v})
        elif c == 'zero_here':
            link.zero_here()
            with self.lock:
                self.targets.update({n: 0.0 for n in S.ARM_JOINTS})
            self._resync()
        elif c == 'free':
            link.free_motors()
        elif c == 'lock':
            link.lock_motors()
            self._resync()
        elif c == 'home_switches':
            link.home()
            self._resync()
        elif c == 'conveyor':
            self.belt_on = bool(args.get('run'))
            if 'speed' in args:
                self.belt_speed = float(args['speed'])
            if self.sim.kind == 'hardware':                 # real camera cell: start / pause it
                ok = self.sim.start() if self.belt_on else (self.sim.pause() or True)
                self.belt_on = self.sim.running
                if not ok:
                    return 'The camera cell is not ready yet - see the list in the log'
                self.say('Camera cell ' + ('running' if self.belt_on else 'paused'))
                return None
            else:
                if self.control in ('sim', 'both') and self.sim.has_belt:
                    self.sim.belt(self.belt_on, self.belt_speed)
                if link and self.control in ('robot', 'both'):
                    link.conveyor(self.belt_on)
            self.say('Conveyor ' + ('running' if self.belt_on else 'stopped'))
        elif c == 'pick':
            color = str(args.get('color', 'any')).lower()
            if not self.sim.commands:
                return 'Colour commands need the conveyor cell in command mode (gazebo mode:=command, or launch_mujoco --command --gui)'
            self.sim.request(color)
            if color in ('none', 'stop picking'):
                self.say('Asked the robot to stop picking')
            else:
                self.say(f'Picking {color} until the next colour')
        elif c == 'belt_tracking':
            # command cell: pick from the moving belt (Jacobian) instead of stopping it
            on = bool(args.get('on'))
            if not self.sim.commands:
                return 'Belt tracking works in the command cell (MuJoCo or Gazebo, world "command")'
            self.sim.request('track on' if on else 'track off')
            self.say('Belt tracking ' + ('on: the robot follows the moving belt' if on else 'off: the belt stops for every pick'))
        elif c in ('stop_all', 'estop'):
            self.safety.estop = True
            self.playing = None
            if link:
                link.stop()
                link.conveyor(False)
            if self.sim.has_belt:
                self.sim.belt(False)
            self.belt_on = False
            self.say('EMERGENCY STOP - the arm holds where it is. Press "Release" to continue.', 'warning')
        elif c == 'release':
            self.safety.estop = False
            self._resync()
            self.say('Stop released')
        elif c == 'safety':
            mode = args.get('mode', self.safety.mode)
            if mode not in SF.MODES:
                return f'safety mode must be one of {SF.MODES}'
            self.safety.set_mode(mode)
            if 'assist' in args:
                self.safety.assist = bool(args['assist'])
            self.say(f'Safety mode: {mode}' + ('' if self.safety.assist else ' (pick assist off)'),
                     'warning' if mode == 'off' else 'info')
        elif c == 'teach_add':
            with self.lock:
                p = {k: round(v, 4) for k, v in self.cmd.items()}
            self.teach.append(p)
            self._save_teach()
            self.say(f'Saved pose {len(self.teach)}')
        elif c == 'teach_delete':
            i = int(args.get('index', -1))
            if 0 <= i < len(self.teach):
                self.teach.pop(i)
                self._save_teach()
        elif c == 'teach_clear':
            self.teach = []
            self._save_teach()
            self.say('Saved poses cleared')
        elif c == 'teach_goto':
            i = int(args.get('index', -1))
            if not 0 <= i < len(self.teach):
                return 'no such pose'
            return self.set_target(self.teach[i])
        elif c == 'teach_play':
            err = self.manual_blocked()
            if err:
                return err
            if not self.teach:
                return 'Save some poses first'
            loop = bool(args.get('loop'))
            token = object()
            self.playing = token
            threading.Thread(target=self._play, args=(token, loop), daemon=True).start()
            self.say(f'Playing {len(self.teach)} poses' + (' in a loop' if loop else ''))
        elif c == 'teach_stop':
            self.playing = None
            self.say('Replay stopped')
        elif c == 'cell_teach':
            if self.sim.kind != 'hardware':
                return 'Teaching cell poses needs the real camera cell (--sim hardware)'
            return self.sim.teach(str(args.get('what', '')))
        elif c == 'cell_start':
            if self.sim.kind == 'hardware':
                self.sim.start()
            else:
                return self.command('conveyor', {'run': True})
        elif c == 'speed':
            self.safety.speed_scale = min(max(float(args.get('scale', 1.0)), 0.1), 1.0)
            self.say(f'Speed {self.safety.speed_scale * 100:.0f} %')
        elif c == 'home':
            err = self.manual_blocked()
            if err:
                return err
            cur = self._measured() or self.targets
            q = K.arm_pose(*S.READY_XY, S.Z_TRAVEL, prefer=(cur.get('column_joint', 0.0), cur.get('forearm_joint', 0.0)))
            return self.set_target({**dict(zip(S.ARM_JOINTS, q)), 'left_finger_joint': S.GRIPPER_OPEN})
        elif c == 'pick_place':
            # tap-to-pick: carry the puck at (x, y) on the table to (to_x, to_y)
            err = self.manual_blocked()
            if err:
                return err
            try:
                src = (float(args['x']), float(args['y']))
                dst = (float(args['to_x']), float(args['to_y']))
                for p in (src, dst):
                    K.arm_pose(p[0], p[1], S.Z_TRAVEL)
            except (KeyError, ValueError) as e:
                return f'pick & place: {e}'
            others = [p for p in (self.sim.pucks() or []) if math.hypot(p[-3] - src[0], p[-2] - src[1]) > 0.04]
            near = [p for p in others if math.hypot(p[-3] - dst[0], p[-2] - dst[1]) < 0.12]
            if near:
                return 'pick & place: that spot is too close to another puck - the fingers need 12 cm of room'
            token = object()
            self.playing = token
            threading.Thread(target=self._pick_place, args=(token, src, dst), daemon=True).start()
            self.say(f'Pick & place: ({src[0]:.2f}, {src[1]:.2f}) -> ({dst[0]:.2f}, {dst[1]:.2f})')
        else:
            return f'unknown command {c}'
        return None

    # ---- teach & replay
    def _load_teach(self):
        try:
            with open(TEACH_FILE) as fh:
                pts = json.load(fh)
            return [p for p in pts if isinstance(p, dict)]
        except (OSError, ValueError):
            return []

    def _save_teach(self):
        try:
            with open(TEACH_FILE, 'w') as fh:
                json.dump(self.teach, fh, indent=1)
        except OSError as e:
            self.say(f'could not save poses: {e}', 'warning')

    def _play(self, token, loop):
        while self.playing is token:
            for i, p in enumerate(list(self.teach)):
                if self.playing is not token:
                    return
                if self.set_target(p):
                    self.playing = None
                    return
                t0 = time.time()
                while self.playing is token and time.time() - t0 < 30:      # wait until reached
                    with self.lock:
                        err = max(abs(self.cmd.get(k, 0) - v) for k, v in p.items() if k in self.cmd)
                    m = self._measured() or {}
                    merr = max((abs(m.get(k, v) - v) for k, v in p.items() if k in S.ARM_JOINTS), default=0)
                    if err < 1e-3 and merr < 0.02:
                        break
                    time.sleep(0.05)
                time.sleep(0.4)
            if not loop:
                break
        if self.playing is token:
            self.playing = None
            self.say('Replay finished')

    def _wait_reached(self, token, p, timeout=40.0, tol=0.02):
        """Wait until the arm is where the operator target now is. The safety filter may have
        nudged the target (pick assist centring a puck) - that still counts as reached; a
        guard that stopped Z short of the wanted height does not (False after 3 s)."""
        t0 = time.time()
        stuck_since = None
        while self.playing is token and time.time() - t0 < timeout:
            with self.lock:
                tgt, cmd = dict(self.targets), dict(self.cmd)
            converged = max(abs(cmd.get(k, 0) - tgt.get(k, 0)) for k in p if k in cmd) < 2e-3
            m = self._measured() or {}
            merr = max((abs(m.get(k, cmd.get(k, 0)) - cmd.get(k, 0)) for k in p if k in S.ARM_JOINTS), default=0)
            z_short = 'shoulder_joint' in p and tgt.get('shoulder_joint', 0) - p['shoulder_joint'] > 0.004
            if converged and merr < tol:
                if not z_short:
                    return True
                stuck_since = stuck_since or time.time()
                if time.time() - stuck_since > 3.0:
                    return False
            time.sleep(0.05)
        return False

    def _pick_place(self, token, src, dst):
        """Pick the puck at src (table) and put it down at dst, through the safety filter."""
        def free_yaw(x, y):
            """Gripper direction whose fingers stay clear of every OTHER puck (the guard would
            otherwise - rightly - stop Z above a neighbour); the least wrist turn wins."""
            others = [(p[-3], p[-2]) for p in (self.sim.pucks() or [])
                      if math.hypot(p[-3] - x, p[-2] - y) > 0.03 and math.hypot(p[-3] - src[0], p[-2] - src[1]) > 0.03]
            best = None
            for k in range(12):
                yaw = k * math.pi / 12
                try:
                    q = K.arm_pose(x, y, S.Z_TRAVEL, yaw=yaw, symmetry=math.pi)
                except K.IKError:
                    continue
                if any(SF.finger_hits(q, S.GRIPPER_OPEN, o) for o in others):
                    continue
                if best is None or abs(q[3]) < best[0]:
                    best = (abs(q[3]), yaw)
            return None if best is None else best[1]

        yaw_at = {src: free_yaw(*src), dst: free_yaw(*dst)}

        def go(x, y, z, grip=None, wait=True):
            cur = self.cmd
            q = K.arm_pose(x, y, z, prefer=(cur.get('column_joint', 0.0), cur.get('forearm_joint', 0.0)),
                           yaw=yaw_at.get((x, y)), symmetry=math.pi)
            tgt = dict(zip(S.ARM_JOINTS, q))
            if grip is not None:
                tgt['left_finger_joint'] = grip
            if self.set_target(tgt):
                return False
            return self._wait_reached(token, tgt) if wait else True

        def grip(v, t):
            self.set_target({'left_finger_joint': v})
            t0 = time.time()
            while self.playing is token and time.time() - t0 < t:
                time.sleep(0.05)
            return self.playing is token

        z_pre = S.Z_PRE_PICK_TABLE
        z_pick = S.Z_PICK_TABLE
        z_put = S.z_pick_on(S.TABLE_TOP_Z) + 0.004
        steps = [lambda: grip(S.GRIP_PREGRASP, 0.8),
                 lambda: go(*src, S.Z_TRAVEL), lambda: go(*src, z_pre), lambda: go(*src, z_pick),
                 lambda: grip(S.GRIP_CLOSE, 1.5),
                 lambda: go(*src, S.Z_TRAVEL), lambda: go(*dst, S.Z_TRAVEL), lambda: go(*dst, z_put),
                 lambda: grip(S.GRIP_RELEASE, 1.2), lambda: go(*dst, S.Z_TRAVEL)]
        names = ['open', 'above the puck', 'lower', 'around the puck', 'close', 'lift', 'carry',
                 'set down', 'open', 'lift']
        for i, st in enumerate(steps):
            if self.playing is not token:
                return
            if not st():
                if self.playing is token:
                    self.playing = None
                    why = (' - ' + self.safety.note) if self.safety.note else ''
                    self.say(f'Pick & place stopped at "{names[i]}": the arm did not get there{why}. '
                             'It holds where it is.', 'warning')
                return
        if self.playing is token:
            self.playing = None
            self.say('Pick & place done')

    # ---- 20 Hz loop: safety filter -> sim / robot, and keep the twin together
    def _loop(self):
        last = time.time()
        last_sim = None
        while not self.stop.is_set():
            time.sleep(0.05)
            now = time.time()
            dt, last = now - last, now
            # a simulator that runs slower than real time (Gazebo on a weak PC) gets its
            # commands in simulation time: the speed limits and guards then mean the same thing
            st = getattr(self.sim, 'sim_time', None)
            if st is not None and self.control in ('sim', 'both'):
                if last_sim is not None and st >= last_sim:
                    dt = min(st - last_sim, 0.1)
                last_sim = st
            link = self.link
            st = link.status() if link else None
            zeroed = bool(link and st and st[3])
            # robot watchdog: a pulled cable or a hung Arduino stops the twin instead of carrying on
            answering = bool(link and (link.alive() if hasattr(link, 'alive') else True))
            if link and answering != getattr(self, '_was_answering', True):
                if answering:
                    self.say('The robot is answering again - continuing from where it is')
                    self._need_reset = True
                else:
                    self.say('The robot is NOT answering (USB cable? power? Arduino reset?) - nothing is sent to it. '
                             'Check it, then Disconnect / Connect robot in the Robot tab.', 'error')
            self._was_answering = answering if link else True
            zeroed = zeroed and answering
            if zeroed and not getattr(self, '_was_zeroed', False):
                self._need_reset = True             # just homed / zeroed: start again from where the arm is
            self._was_zeroed = zeroed
            try:
                manual = not self.cell_busy()
                if manual and getattr(self, '_was_busy', False):
                    self._need_reset = True                   # cell just paused: start from where the arm is
                self._was_busy = not manual
                meas = self._measured() if manual else None
                if manual and meas is None and self.control in ('sim', 'both'):
                    manual = False                            # simulation not ready yet (e.g. Gazebo starting)
                if manual:
                    if self._need_reset:
                        base = meas or self.targets
                        with self.lock:
                            self.targets.update({k: v for k, v in base.items() if k in self.targets})
                        self.safety.reset([base.get(n, 0.0) for n in S.ARM_JOINTS], base.get('left_finger_joint'))
                        self._need_reset = False
                    # someone else (joystick, keyboard) moves the arm while the GUI is idle: follow it,
                    # so the next GUI command starts from where the arm really is.  An arm that is
                    # still on its way to our command (error shrinking) is NOT "moved by someone else".
                    if meas and self.sim.kind == 'gazebo' and self.control in ('sim', 'both'):
                        err = max(abs(meas.get(n, 0) - self.cmd.get(n, 0)) for n in S.ARM_JOINTS)
                        reached = all(abs(self.cmd.get(n, 0) - self.targets.get(n, 0)) < 1e-4 for n in S.ARM_JOINTS)
                        self._err_hist.append((now, err))
                        while self._err_hist and now - self._err_hist[0][0] > 0.6:
                            self._err_hist.pop(0)
                        growing = len(self._err_hist) > 3 and err > self._err_hist[0][1] + 0.005
                        if (reached and err > 0.03 and growing and not self.safety.estop and not self.playing
                                and now - self._last_user > 1.0):
                            self._err_hist = []
                            self._resync()
                            with self.lock:
                                self.cmd.update({k: v for k, v in meas.items() if k in self.cmd})
                    with self.lock:
                        t = dict(self.targets)
                    q, g = self.safety.filter(
                        [t[n] for n in S.ARM_JOINTS], t['left_finger_joint'], dt,
                        q_meas=[meas[n] for n in S.ARM_JOINTS] if meas else None,
                        grip_meas=meas.get('left_finger_joint') if meas else None,
                        pucks=self.sim.pucks() if self.control in ('sim', 'both') else None)
                    with self.lock:
                        self.cmd = {**dict(zip(S.ARM_JOINTS, q)), 'left_finger_joint': g}
                        if self.safety.estop:
                            self.targets.update(self.cmd)
                        else:   # adopt the pick assist's centring / a guard's Z stop as the new target
                            adj = self.safety.clamp_back([self.targets[n] for n in S.ARM_JOINTS])
                            self.targets.update(dict(zip(S.ARM_JOINTS, adj)))
                    if self.control in ('sim', 'both'):
                        self.sim.set_targets(self.cmd)
                    elif zeroed and getattr(link, 'motors_on', True):
                        link.send_joints(self._hw(self.cmd))
                if not zeroed:
                    continue                                  # wait for "Zero here"
                simq = self.sim.joints()
                if self.control == 'both':
                    # the real robot copies what the simulated arm really does
                    if not self.safety.estop:
                        link.send_joints(self._hw(simq if simq else self.cmd))
                    belt = getattr(self.sim, 'belt_running', None)
                    if belt is not None and belt != link.conveyor_on and now - self._belt_sent > 0.5:
                        self._belt_sent = now
                        link.conveyor(belt)                   # real conveyor follows the simulated one
                elif self.control == 'robot':
                    real = link.joints()
                    if real and self.sim.kind not in ('none', 'hardware') and not self.sim.autonomous:
                        self.sim.set_targets(real)            # the simulation shows the real robot
            except Exception as e:                            # e.g. cable pulled
                self.say(f'robot link: {e}', 'error')
                self.disconnect()

    # ---- state for the GUI
    def state(self, since=0):
        link = self.link
        real, st = (link.joints(), link.status()) if link else (None, None)
        out = {
            'sim': {'kind': self.sim.kind, 'label': self.sim.label, 'joints': self.sim.joints(),
                    'autonomous': self.sim.autonomous, 'commands': self.sim.commands, 'has_belt': self.sim.has_belt,
                    'pucks': [{'c': S.puck_color(p[0]) if isinstance(p[0], str) else None, 'x': p[-3], 'y': p[-2],
                               'z': p[-1], **({'cube': True} if str(p[0]).startswith('cube_') else {})}
                              for p in (self.sim.pucks() or [])] or None,
                    **self.sim.info()},
            'robot': {'connected': bool(link), 'port': self.port,
                      'joints': real, 'age': (time.time() - st[0]) if st else None,
                      'answering': bool(link.alive()) if link and hasattr(link, 'alive') else None,
                      'moving': bool(st[2]) if st else False, 'zeroed': bool(st[3]) if st else False,
                      'motors_on': getattr(link, 'motors_on', None) if link else None,
                      'conveyor_on': getattr(link, 'conveyor_on', None) if link else None,
                      'out_of_range': link.out_of_range(self._hw(self.targets)) if link else [],
                      'limits': self.limits() if link and self.control != 'sim' else None},
            'targets': self.targets, 'cmd': self.cmd, 'control': self.control,
            'safety': {'mode': self.safety.mode, 'assist': self.safety.assist, 'estop': self.safety.estop,
                       'note': self.safety.note, 'speed': self.safety.speed_scale},
            'teach': {'points': self.teach, 'playing': self.playing is not None},
            'cell_poses': sorted(self.sim.cell['poses']) if self.sim.kind == 'hardware' else None,
            'belt': {'on': self.sim.running if self.sim.kind == 'hardware' else self.belt_on, 'speed': self.belt_speed},
            'app': {'clients': self.app_clients, 'last': (time.time() - self.app_last) if self.app_last else None,
                    'address': getattr(self, 'app_address', '')},
            'log': [e for e in list(self.log) if e['id'] > since],
        }
        return out


def scene_info():
    """Geometry for the GUI's top-down view."""
    belt0 = S.belt_point(S.CONVEYOR_Y_RANGE[0])
    belt1 = S.belt_point(S.CONVEYOR_Y_RANGE[1])
    return {
        'column': S.COLUMN_AXIS_XY, 'base_yaw': S.BASE_YAW, 'L1': S.L1, 'L2': S.L2, 'grip_offset': S.GRIP_OFFSET,
        'gap_open': S.GRIPPER_GAP_OPEN, 'table': [S.TABLE_SIZE_X, S.TABLE_SIZE_Y],
        'belt': {'a': belt0, 'b': belt1, 'width': S.BELT_WIDTH},
        'bins': [{'color': c, 'xy': S.bin_xy(c), 'yaw': S.bin_yaw(c)} for c in S.COLOR_ORDER],
        'command_bin': {'xy': S.command_bin_xy(), 'yaw': S.command_bin_yaw()},
        'bin_size': S.BIN_SIZE, 'puck_radius': S.PUCK_RADIUS, 'table_top': S.TABLE_TOP_Z,
        'manual_pucks': [{'name': n, 'xy': [x, y]} for n, (x, y, _) in S.MANUAL_PUCKS],
        'limits': LIMITS, 'grip_close': S.GRIP_CLOSE, 'hw_limits': HW_LIMITS,
        'table_center': list(S.TABLE_CENTER), 'belt_top': S.BELT_TOP_Z, 'puck_height': S.PUCK_HEIGHT,
        'bin_height': S.BIN_HEIGHT, 'base_xyz': list(S.BASE_XYZ), 'ready_xy': list(S.READY_XY),
        'z_travel': S.Z_TRAVEL,
        'arm_cam': {'parent': S.ARM_CAM_PARENT, 'xyz': list(S.ARM_CAM_XYZ), 'pitch': S.ARM_CAM_RPY[1]},
        # for the app's own offline simulator (manual + autonomous cell on the phone)
        'kin': {'L1': S.L1, 'L2': S.L2, 'grip_offset': S.GRIP_OFFSET, 'column': list(S.COLUMN_AXIS_XY),
                'base_yaw': S.BASE_YAW, 'pad_bottom_z0': S.PAD_BOTTOM_AT_Z0, 'pad_half_len': SF.PAD_HALF_LEN,
                'grip_touch': S.GRIPPER_TOUCH},
        'z': {'travel': S.Z_TRAVEL, 'pick': S.Z_PICK, 'pick_table': S.Z_PICK_TABLE, 'pre_pick': S.Z_PRE_PICK,
              'pre_pick_table': S.Z_PRE_PICK_TABLE, 'place': S.Z_PLACE},
        'cell': {'p0': list(S.belt_point(0.0)), 'dir': list(S.BELT_DIR), 'speed': S.BELT_SPEED,
                 'spawn_s': S.SPAWN_S, 'end_s': S.END_STOP_S, 'pick_window': list(S.PICK_WINDOW_S),
                 'bin_floor': S.BIN_FLOOR, 'handle_top': S.HANDLE_TOP_Z},
    }


# ---------------------------------------------------------------------------
# HTTP: the GUI + JSON API
# ---------------------------------------------------------------------------
def make_handler(core):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/api/config':
                self._send(200, {'app_port': getattr(core, 'app_port', 9090), 'server': 'scara_twin', 'version': 3})
            elif path == '/api/state':
                since = 0
                if 'since=' in self.path:
                    try:
                        since = int(self.path.split('since=')[1].split('&')[0])
                    except ValueError:
                        pass
                self._send(200, core.state(since))
            elif path == '/api/scene':
                self._send(200, scene_info())
            elif path == '/api/ports':
                self._send(200, {'ports': list_ports()})
            else:
                self._static(path)

        def _static(self, path):
            rel = 'index.html' if path in ('', '/') else path.lstrip('/')
            full = os.path.realpath(os.path.join(UI_DIR, rel))
            if not full.startswith(os.path.realpath(UI_DIR) + os.sep) or not os.path.isfile(full):
                return self._send(404, {'error': 'not found'})
            with open(full, 'rb') as fh:
                self._send(200, fh.read(), MIME.get(os.path.splitext(full)[1].lower(), 'application/octet-stream'))

        def do_POST(self):
            if self.path != '/api/cmd':
                return self._send(404, {'error': 'not found'})
            n = int(self.headers.get('Content-Length', 0))
            try:
                body = json.loads(self.rfile.read(n) or b'{}')
                err = core.command(body.get('cmd', ''), body)
            except Exception as e:
                err = str(e)
            if err:
                core.say(err, 'warning')
            self._send(200, {'ok': not err, 'error': err})
    return Handler


# ---------------------------------------------------------------------------
# Port 9090: a small rosbridge-compatible WebSocket server for the phone app
# ---------------------------------------------------------------------------
WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


def ws_recv(sock, max_len=8 * 1024 * 1024):
    """One text message from a client (handles masking, fragments, ping). None on close / bad frame."""
    parts, total = [], 0
    while True:
        hdr = _recv_exact(sock, 2)
        if hdr is None:
            return None
        b1, b2 = hdr
        op, fin, masked, ln = b1 & 0x0F, b1 & 0x80, b2 & 0x80, b2 & 0x7F
        if ln >= 126:
            ext = _recv_exact(sock, 2 if ln == 126 else 8)
            if ext is None:
                return None
            ln = struct.unpack('>H' if len(ext) == 2 else '>Q', ext)[0]
        total += ln
        if total > max_len:                     # nothing the app sends is this big: drop the client
            return None
        mask = _recv_exact(sock, 4) if masked else None
        payload = _recv_exact(sock, ln) if ln else b''
        if payload is None or (masked and mask is None):
            return None
        if masked and ln:                       # unmask the whole frame at once (fast for big messages)
            m = (mask * (ln // 4 + 1))[:ln]
            payload = (int.from_bytes(payload, 'big') ^ int.from_bytes(m, 'big')).to_bytes(ln, 'big')
        if op == 8:
            return None
        if op == 9:
            ws_send(sock, bytes(payload), opcode=10)
            continue
        if op in (0, 1, 2):
            parts.append(bytes(payload))
            if fin:
                return b''.join(parts).decode('utf-8', 'ignore')


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 20))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def ws_send(sock, data, opcode=1):
    if isinstance(data, str):
        data = data.encode()
    hdr = bytes([0x80 | opcode])
    n = len(data)
    if n < 126:
        hdr += bytes([n])
    elif n < 65536:
        hdr += bytes([126]) + struct.pack('>H', n)
    else:
        hdr += bytes([127]) + struct.pack('>Q', n)
    sock.sendall(hdr + data)


class RosbridgeLite:
    """Understands what the phone app sends: publish on /arm_controller/joint_trajectory,
    /gripper_controller/joint_trajectory, /conveyor/cmd_vel, /conveyor/enabled and
    /scara/pick_color; advertise; subscribe to /joint_states (sent back at 10 Hz)."""

    def __init__(self, core, port=9090):
        self.core, self.port = core, port
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('0.0.0.0', port))
        self.srv.listen(5)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            sock, addr = self.srv.accept()
            threading.Thread(target=self._client, args=(sock, addr), daemon=True).start()

    def _client(self, sock, addr):
        try:
            req = b''
            while b'\r\n\r\n' not in req:
                chunk = sock.recv(4096)
                if not chunk:
                    return
                req += chunk
            headers = {}
            for line in req.decode('latin1').split('\r\n')[1:]:
                if ':' in line:
                    k, v = line.split(':', 1)
                    headers[k.strip().lower()] = v.strip()
            key = headers.get('sec-websocket-key')
            if not key:
                sock.sendall(b'HTTP/1.1 400 Bad Request\r\n\r\nrosbridge websocket only')
                return
            accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
            sock.sendall(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                          f'Sec-WebSocket-Accept: {accept}\r\n\r\n').encode())
        except OSError:
            return
        self.core.app_clients += 1
        self.core.say(f'Phone app connected from {addr[0]}')
        send_lock = threading.Lock()
        subs = set()
        alive = {'on': True}

        last_log = {'id': 0}

        def send(obj):
            with send_lock:
                ws_send(sock, json.dumps(obj))

        def feeder():
            n = 0
            while alive['on']:
                time.sleep(0.1)
                n += 1
                try:
                    if '/joint_states' in subs:
                        q = self.core.sim.joints() or (self.core.link.joints() if self.core.link else None) \
                            or self.core.targets
                        send({'op': 'publish', 'topic': '/joint_states',
                              'msg': {'name': JOINTS, 'position': [q.get(n_, 0.0) for n_ in JOINTS]}})
                    if '__scara_state__' in subs:          # the v2 app: full twin state at 10 Hz
                        st = self.core.state(last_log['id'])
                        if st['log']:
                            last_log['id'] = st['log'][-1]['id']
                        send({'op': 'scara_state', 'state': st})
                except OSError:
                    return
                except Exception as e:                   # never let one bad state kill the stream
                    self.core.say(f'app stream: {e}', 'warning')
        threading.Thread(target=feeder, daemon=True).start()
        try:
            while True:
                text = ws_recv(sock)
                if text is None:
                    break
                try:
                    self.handle(json.loads(text), subs, send)
                except (ValueError, KeyError, TypeError, IndexError):
                    pass
        except OSError:
            pass
        finally:
            alive['on'] = False
            self.core.app_clients -= 1
            self.core.say('Phone app disconnected')
            try:
                sock.close()
            except OSError:
                pass

    def handle(self, m, subs, send=None):
        op = m.get('op')
        self.core.app_last = time.time()
        if op == 'scara':                            # SCARA Digital Twin app (v2+): direct twin commands
            rid, cmd, args = m.get('id'), m.get('cmd', ''), m.get('args') or {}
            reply = {'op': 'scara_reply', 'id': rid, 'cmd': cmd, 'ok': True, 'error': None}
            if cmd == 'hello':
                subs.add('__scara_state__')
                reply.update(server='scara_twin', version=2, scene=scene_info(), ports=list_ports(),
                             name_plate=os.path.exists(S.pkg_file('meshes', 'shoulder_name.stl')))
                self.core.say(f'App connected ({args.get("device", "phone")})')
            elif cmd == 'ports':
                reply['ports'] = list_ports()
            else:
                try:
                    err = self.core.command(cmd, args)
                except Exception as e:                   # bad arguments from the app
                    err = f'{cmd}: {e}'
                if err:
                    self.core.say(err, 'warning')
                    reply.update(ok=False, error=err)
            if send:
                send(reply)
            return
        if op == 'subscribe':
            subs.add(m.get('topic'))
            return
        if op != 'publish':
            return                                   # advertise / unadvertise / ping: nothing to do
        topic, msg = m.get('topic'), m.get('msg', {})
        if topic == '/arm_controller/joint_trajectory':
            names = msg['joint_names']
            pos = msg['points'][-1]['positions']
            self.core.set_target(dict(zip(names, pos)), source='app')
        elif topic == '/gripper_controller/joint_trajectory':
            pos = msg['points'][-1]['positions']
            self.core.set_target({'left_finger_joint': pos[0]}, source='app')
        elif topic == '/conveyor/enabled':
            on = bool(msg.get('data'))
            if on != self.core.belt_on:
                self.core.command('conveyor', {'run': on})
        elif topic == '/conveyor/cmd_vel':
            v = float(msg.get('linear', {}).get('x', 0.0))
            if v > 0:
                self.core.belt_speed = v
        elif topic == '/scara/pick_color':
            err = self.core.command('pick', {'color': msg.get('data', 'any')})
            if err:
                self.core.say(err, 'warning')


# ---------------------------------------------------------------------------
def serve(core, http_port=8080, app_port=9090, open_browser=False):
    core.app_port = app_port
    try:
        httpd = ThreadingHTTPServer(('0.0.0.0', http_port), make_handler(core))
    except OSError as e:
        raise SystemExit(f'GUI port {http_port} is busy ({e}) - is SCARA Twin Control already running? '
                         f'Close it or use --http <other port>')
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    try:
        RosbridgeLite(core, app_port)
        app_msg = f'phone app: ws://<this-pc-ip>:{app_port}'
    except OSError as e:
        app_msg = f'phone-app port {app_port} busy ({e}) - is rosbridge_server running? Then use that one.'
    ip = local_ip()
    core.app_address = f'{ip}  port {app_port}' if app_msg.startswith('phone app') else ''
    core.say(f'GUI: http://localhost:{http_port}  (other devices: http://{ip}:{http_port})  |  {app_msg.replace("<this-pc-ip>", ip)}')
    if open_browser:
        threading.Timer(1.0, lambda: webbrowser.open(f'http://localhost:{http_port}')).start()
    return httpd


def make_hw_cell(args):
    from scara_cam2d import Camera2D
    from scara_hw_cell import HwCell
    if not args.port and not args.fake:
        raise SystemExit('--sim hardware needs the robot: --port /dev/ttyACM0 (or --fake to try it)')
    cam = Camera2D.load()
    if args.source is not None:
        cam.cfg['source'] = int(args.source) if args.source.isdigit() else args.source
    try:
        cam.open()
    except RuntimeError as e:
        # still start: the arm, the twin view and teaching work; the cell refuses to run until the camera is there
        print(f'WARNING: {e}. The cell will not start until the camera works (restart with --source ...).')
        cam.open_error = str(e)
    link = open_link(args.port, fake=args.fake, on_message=lambda k, t: print(f'[robot {k}] {t}'))
    return HwCell(link, cam, mode=args.mode)


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '127.0.0.1'


def main():
    ap = argparse.ArgumentParser(description='SCARA Twin Control (GUI + phone app server)')
    ap.add_argument('--sim', default='mujoco', choices=['mujoco', 'gazebo', 'hardware', 'none'],
                    help='hardware = the REAL conveyor cell with the 2-D webcam (needs --port)')
    ap.add_argument('--mode', default='sort', choices=['sort', 'command'], help='--sim hardware: cell mode')
    ap.add_argument('--world', default=None, choices=['manual', 'sort', 'command'],
                    help='world to start with (switch any time from the GUI / app). With --sim gazebo the '
                         'server starts Gazebo itself in this world')
    ap.add_argument('--headless', action='store_true', help='--sim gazebo --world ...: Gazebo without its window')
    ap.add_argument('--source', default=None, help='--sim hardware: camera (index, file or URL)')
    ap.add_argument('--port', default=None, help='serial port of the robot, e.g. /dev/ttyACM0')
    ap.add_argument('--fake', action='store_true', help='software robot (no hardware)')
    ap.add_argument('--http', type=int, default=8080, help='GUI port')
    ap.add_argument('--app-port', type=int, default=9090, help='phone app (rosbridge) port')
    ap.add_argument('--open', action='store_true', help='open the GUI in the browser (otherwise just go to the address it prints)')
    ap.add_argument('--no-browser', action='store_true', help=argparse.SUPPRESS)   # old flag: the browser no longer opens by itself
    args = ap.parse_args()
    if args.sim == 'hardware':
        sim = make_hw_cell(args)
    else:
        if args.sim == 'gazebo':
            sim = GazeboSim(world=None, headless=args.headless)
        else:
            sim = {'mujoco': MujocoWorlds, 'none': NoSim}[args.sim]()
        if isinstance(sim, MujocoWorlds) and args.world:
            sim.world = args.world
    core = TwinCore(sim)
    if isinstance(sim, GazeboSim):
        sim._log = lambda t, level='info': core.say(t, level)
        if args.world:
            sim.set_world(args.world, force=True)
    serve(core, args.http, args.app_port, args.open and not args.no_browser)
    if args.sim == 'hardware':
        core.link, core.port, core.control = sim.link, args.port or 'fake robot', 'robot'
        sim._log = lambda t, level='info': core.say(t, level)
        sim.link.on_message = lambda k, t: core.say(f'robot: {t}', k)
        core.say('Real camera cell: zero the robot (Robot tab), teach the poses, then start the conveyor')
        for p in sim.problems():
            core.say('To do: ' + p, 'warning')
    elif args.port or args.fake:
        core.connect(args.port, args.fake)
    stop = threading.Event()
    import signal

    def _term(*_):
        stop.set()
    signal.signal(signal.SIGTERM, _term)          # e.g. "kill" or the launch system: exit cleanly
    try:
        if isinstance(sim, (MujocoTable, MujocoWorlds)):
            sim.run(stop)                 # MuJoCo viewer in the main thread; closing it ends the app
        else:
            while not stop.is_set():
                if isinstance(sim, GazeboSim) and not sim.rclpy.ok():  # noqa: SIM102
                    break                 # ROS was shut down
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        core.stop.set()
        core.disconnect()
        if isinstance(sim, GazeboSim):
            sim.shutdown()
            if sim.rclpy.ok():
                sim.rclpy.shutdown()


if __name__ == '__main__':
    main()
