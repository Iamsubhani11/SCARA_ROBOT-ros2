#!/usr/bin/env python3
"""
Autonomous conveyor cell on the REAL robot, with one ordinary 2-D webcam
(the setup of the Part 25 demo: camera on a stand at the pick station, no depth
camera, no camera on the gripper).

How it works - the same "stop and pick" the simulation uses:
  1. the conveyor runs (relay on the Arduino, pin 10)
  2. the webcam watches the pick zone; every cube gets a colour and its distance
     from the pick point along the belt (scara_cam2d.py)
  3. a cube the robot wants enters the zone -> conveyor stops, the camera measures
     again on the still belt
  4. the robot goes to the TAUGHT pick pose, shifted along the belt by the measured
     distance, comes down, closes, lifts - the camera checks the cube is gone
     (no gripper sensor needed), and drops it at the taught drop pose for its colour
  5. conveyor on again
Sorting mode: every colour to its own drop pose.  Command mode: only the colours you
ask for ("red", "any", "3 blue", "sequence red blue green"), everything else rides past.

One-time setup (README section "Real robot: autonomous cell"):
  a) python3 scripts/scara_camera_setup.py        train colours + belt ruler
  b) python3 scripts/scara_hw_cell.py --port /dev/ttyACM0 --teach
     jog the arm with the keys and save: the pick pose (gripper open, fingers around a
     cube that sits at the pick point), a point further upstream on the belt, the
     drop pose of each colour, and the parking pose (out of the camera's view)
  c) run it:
     python3 scripts/scara_hw_cell.py --port /dev/ttyACM0                 # sorting
     python3 scripts/scara_hw_cell.py --port /dev/ttyACM0 --mode command  # type colours
  or from the phone app / browser GUI:
     python3 scripts/scara_twin_server.py --sim hardware --port /dev/ttyACM0 [--mode command]

Keys in the camera window: SPACE start / pause, 1-4 red/green/blue/yellow (command mode),
A any, N none, Q quit.  In --teach mode see the help text on the image.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_scene as S                      # noqa: E402
import scara_kinematics as K                 # noqa: E402
from scara_sorter_core import Sorter         # noqa: E402

CELL_FILE = os.path.join(os.path.expanduser('~'), '.scara_hw_cell.json')
TEACH_ITEMS = ['pick', 'upstream', 'drop_red', 'drop_green', 'drop_blue', 'drop_yellow', 'drop_any', 'ready']
TEACH_HELP = {
    'pick': 'gripper OPEN, fingers around a cube standing at the pick point, as low as it must go to grab',
    'upstream': 'gripper over the belt centre, about 60-100 mm upstream of the pick point (same height)',
    'drop_red': 'where red cubes are let go', 'drop_green': 'where green cubes are let go',
    'drop_blue': 'where blue cubes are let go', 'drop_yellow': 'where yellow cubes are let go',
    'drop_any': 'command mode: the one bin every requested cube goes to',
    'ready': 'parking pose, out of the camera view',
}


def default_cell():
    return {'poses': {}, 'approach_mm': 45.0, 'lift_mm': 60.0, 'grip_open': S.GRIPPER_OPEN,
            'grip_close': S.GRIP_CLOSE, 'settle_s': 0.6, 'retries': 1, 'pick_timeout_s': 40.0}


def load_cell(path=None):
    cfg = default_cell()
    for p in ([path] if path else [CELL_FILE, os.path.join(HERE, '..', 'config', 'hw_cell.json')]):
        if p and os.path.exists(p):
            with open(p) as fh:
                cfg.update(json.load(fh))
            break
    return cfg


def save_cell(cfg, path=None):
    path = path or CELL_FILE
    with open(path, 'w') as fh:
        json.dump(cfg, fh, indent=1)
    return path


def tool_xy(q):
    """Gripper centre (world x, y) for URDF joints q (dict)."""
    return K.fk_xy(q['column_joint'], q['forearm_joint'], q['wrist_joint'])[:2]


def gripper_yaw(q):
    return S.BASE_YAW + q['column_joint'] + q['forearm_joint'] + q['wrist_joint']


class HardwareIO:
    """What the sorter needs, on top of the serial link and the 2-D camera (wall clock)."""

    def __init__(self, link, cam, cell, log=print):
        self.link, self.cam, self.cell = link, cam, cell
        self._log = log
        self.grip = cell['grip_open']
        self.arm = None
        self.belt_on = None
        self.frame = None
        self.dets = []
        self.stop_flag = False
        self.dets_t = 0.0
        self.cam_lock = threading.Lock()
        self.abort = threading.Event()

    def log(self, text):
        self._log(text)

    def now(self):
        return time.time()

    def sleep(self, dt):
        if self.abort.wait(dt):
            raise CellAborted()

    def joints(self):
        j = self.link.joints() or {}
        j = dict(j)
        j['left_finger_joint'] = j['right_finger_joint'] = self.grip
        return j

    def _send(self):
        q = dict(zip(S.ARM_JOINTS, self.arm))
        q['left_finger_joint'] = q['right_finger_joint'] = self.grip
        self.link.send_joints(q, min_change=0.0)

    def move_arm(self, q, T=1.0, tol=(0.01, 0.002, 0.01, 0.02)):
        """Send a joint target and wait until the robot reports it is there."""
        self.arm = list(q)
        self._send()
        t0 = self.now()
        self.sleep(0.12)
        while self.now() - t0 < 4.0 * T + 6.0:
            st = self.link.status()
            j = self.link.joints()
            if st and j and not st[2] and all(abs(j[n] - v) <= tl for n, v, tl in zip(S.ARM_JOINTS, q, tol)):
                return True
            self.sleep(0.03)
        self.log('robot did not reach the target in time - check the arm and the zero')
        return False

    def move_gripper(self, g, T=0.5):
        self.grip = g
        if self.arm is None:
            j = self.link.joints()
            self.arm = [j[n] for n in S.ARM_JOINTS] if j else [0.0] * 4
        self._send()
        self.sleep(T + 0.25)                       # servo, no feedback

    def set_belt(self, v):
        on = bool(v)
        if on != self.belt_on:
            self.link.conveyor(on)
            self.belt_on = on

    def look(self):
        with self.cam_lock:                       # the twin server may peek while the cell is paused
            frame = self.cam.read()
            dets = [d for d in self.cam.detect(frame) if d['s_mm'] is not None]
        self.frame, self.dets, self.dets_t = frame, dets, time.time()
        return dets


class CellAborted(Exception):
    pass


class HwSorter(Sorter):
    """Sorter (colour logic, command / sequence modes) with the taught real-robot geometry."""

    def __init__(self, io, cell, mode='sort'):
        super().__init__(io, 'all', mode=mode)
        self.cell = cell
        P = cell['poses']
        self.pick_q = P['pick']
        self.pick_xy = tool_xy(self.pick_q)
        up = tool_xy(P['upstream'])
        d = math.hypot(up[0] - self.pick_xy[0], up[1] - self.pick_xy[1])
        if d < 0.02:
            raise ValueError('the "upstream" pose is too close to the pick pose - teach it 60-100 mm upstream')
        self.u = ((up[0] - self.pick_xy[0]) / d, (up[1] - self.pick_xy[1]) / d)   # unit vector, upstream
        self.yaw = gripper_yaw(self.pick_q)
        self.z_pick = self.pick_q['shoulder_joint']
        lo, hi = S.LIMITS['shoulder_joint']
        self.z_travel = min(hi, self.z_pick + cell['lift_mm'] / 1000.0)
        self.z_above = min(hi, self.z_pick + cell['approach_mm'] / 1000.0)

    def drop_pose(self, color):
        P = self.cell['poses']
        key = 'drop_any' if self.mode == 'command' else f'drop_{color}'
        return P.get(key) or P.get('drop_any') or P.get(f'drop_{color}')

    def missing(self):
        P = self.cell['poses']
        need = ['pick', 'upstream', 'ready'] + (['drop_any'] if self.mode == 'command'
                                                 else [f'drop_{c}' for c in S.COLOR_ORDER])
        return [n for n in need if n not in P]

    def arm_at(self, s_m, z):
        x = self.pick_xy[0] + self.u[0] * s_m
        y = self.pick_xy[1] + self.u[1] * s_m
        cur = self.io.joints()
        c0, f0 = cur.get('column_joint', 0.0), cur.get('forearm_joint', 0.0)
        # a cube can be gripped every 90 deg, and both elbow sides reach the belt: keep the
        # solutions the REAL robot can do (its limits are tighter than the URDF's on some
        # joints) and take the one closest to where the arm is now
        best = None
        for k in range(4):
            for pref in ((c0, f0), (c0, -f0), (0.0, 0.5), (0.0, -0.5)):
                try:
                    t1, t2, t3 = K.ik_xy_yaw(x, y, self.yaw + k * math.pi / 2, prefer=pref, symmetry=2 * math.pi)
                except K.IKError:
                    continue
                q = [t1, z, t2, t3]
                if self.io.link.out_of_range(dict(zip(S.ARM_JOINTS, q))):
                    continue
                cost = abs(t1 - c0) + abs(t2 - f0) + 0.3 * abs(t3)
                if best is None or cost < best[0]:
                    best = (cost, q)
        if best is None:
            raise K.IKError(f'no pose of the real robot reaches x={x:.3f} y={y:.3f}')
        return best[1]

    def joints_list(self, q):
        return [q[n] for n in S.ARM_JOINTS]

    def lift(self):
        j = self.io.joints()
        if j.get('shoulder_joint', 0) < self.z_travel - 0.003:
            self.io.move_arm([j['column_joint'], self.z_travel, j['forearm_joint'], j['wrist_joint']], 0.6)

    def go_ready(self):
        self.lift()
        r = self.cell['poses']['ready']
        self.io.move_arm([r['column_joint'], self.z_travel, r['forearm_joint'], r['wrist_joint']], 1.0)

    def pick_cube(self, d):
        io, s = self.io, d['s_mm'] / 1000.0
        io.move_gripper(self.cell['grip_open'], 0.3)
        self.lift()
        io.move_arm(self.arm_at(s, self.z_travel), 1.0)
        io.move_arm(self.arm_at(s, self.z_above), 0.5)
        io.move_arm(self.arm_at(s, self.z_pick), 0.6)
        io.move_gripper(self.cell['grip_close'], 0.6)
        io.move_arm(self.arm_at(s, self.z_travel), 0.6)
        # no gripper sensor: the camera checks the cube really left the belt
        io.sleep(0.25)
        still = [x for x in io.look() if x['color'] == d['color'] and abs(x['s_mm'] - d['s_mm']) < 25]
        return not still

    def place(self, color):
        io = self.io
        q = self.drop_pose(color)
        io.move_arm([q['column_joint'], self.z_travel, q['forearm_joint'], q['wrist_joint']], 1.0)
        io.move_arm(self.joints_list(q), 0.6)
        io.move_gripper(self.cell['grip_open'], 0.4)
        self.lift()

    def step(self, timeout=None):
        io = self.io
        timeout = timeout or self.cell['pick_timeout_s']
        lo, hi = io.cam.cfg['window_mm']
        io.set_belt(True)
        t0 = io.now()
        while io.now() - t0 < timeout:
            hit = [d for d in io.look() if lo <= d['s_mm'] <= hi and self.wants(d['color'])]
            if hit:
                break
            io.sleep(0.03)
        else:
            return 'idle'
        io.set_belt(False)
        io.sleep(self.cell['settle_s'])
        # measure on the still belt: median of 3 frames
        seen = []
        for _ in range(3):
            seen += [d for d in io.look() if lo - 20 <= d['s_mm'] <= hi + 20 and self.wants(d['color'])]
            io.sleep(0.05)
        if not seen:
            return 'idle'
        first = min(seen, key=lambda d: abs(d['s_mm']))
        same = sorted(x['s_mm'] for x in seen if x['color'] == first['color'] and abs(x['s_mm'] - first['s_mm']) < 15)
        target = dict(first, s_mm=same[len(same) // 2])
        color = target['color']
        io.log(f'Picking {color} cube {target["s_mm"]:+.0f} mm from the pick point')
        for attempt in range(1 + int(self.cell['retries'])):
            try:
                ok = self.pick_cube(target)
            except K.IKError as e:
                io.log(f'cannot reach that cube ({e}) - letting it pass')
                self.go_ready()
                io.set_belt(True)
                io.sleep(1.5)
                return 'missed'
            if ok:
                break
            io.log('Missed it - the cube is still on the belt' + (', trying again' if attempt == 0 else ''))
            io.move_gripper(self.cell['grip_open'], 0.3)
            again = [x for x in io.look() if x['color'] == color and abs(x['s_mm'] - target['s_mm']) < 25]
            if not again:
                break
            target = dict(target, s_mm=again[0]['s_mm'])
        else:
            self.failures += 1
            self.go_ready()
            return 'missed'
        self.place(color)
        self.sorted[color] += 1
        self._fulfil(color)
        io.log(f'Placed {color}  (totals {self.sorted})')
        self.go_ready()
        return 'picked'


class HwCell:
    """The real cell as a backend of SCARA Twin Control (scara_twin_server.py --sim hardware)
    and as a stand-alone program (main below)."""
    kind = 'hardware'
    label = 'Real conveyor cell'

    def __init__(self, link, cam, mode='sort', cell_path=None, log=print):
        self.link, self.cam, self.mode = link, cam, mode
        self.cell_path = cell_path
        self.cell = load_cell(cell_path)
        self._log = log
        self.status = []
        self.io = HardwareIO(link, cam, self.cell, log=self.log)
        self.sorter = None
        self.running = False
        self.pending = []
        self.thread = None
        self.lock = threading.Lock()

    def log(self, text, level='info'):
        self.status = (self.status + [text])[-8:]
        try:
            self._log(text, level)
        except TypeError:
            self._log(text)

    # ---- twin-server backend interface
    @property
    def autonomous(self):
        return self.running

    @property
    def commands(self):
        return self.mode == 'command'

    has_belt = True

    @property
    def belt_running(self):
        return None                               # the cell drives its own relay

    def joints(self):
        j = self.link.joints()
        if j is None:
            return None
        j = dict(j)
        j['left_finger_joint'] = self.io.grip
        return {n: j[n] for n in S.ARM_JOINTS + ['left_finger_joint']}

    # ---- what the twin shows: the belt where the robot was taught, the cubes the camera sees
    def layout(self):
        """Where things are on the real table, in robot/world metres, from the taught poses.
        Empty dict entries are simply not taught yet (the app then draws its default cell)."""
        P = self.cell['poses']
        out = {'drops': {}}
        if 'pick' in P:
            px, py = tool_xy(P['pick'])
            out['pick'] = [px, py]
            if 'upstream' in P:
                ux, uy = tool_xy(P['upstream'])
                d = math.hypot(ux - px, uy - py)
                if d > 0.02:
                    out['dir'] = [(ux - px) / d, (uy - py) / d]
        for k, q in P.items():
            if k.startswith('drop_'):
                out['drops'][k[5:]] = list(tool_xy(q))
        if 'ready' in P:
            out['ready'] = list(tool_xy(P['ready']))
        out['window_mm'] = list(self.cam.cfg.get('window_mm', [-12.0, 18.0]))
        return out

    def _peek(self):
        """The cell is paused: still look now and then, so the twin shows the cubes on the belt."""
        if (self.running or time.time() - self.io.dets_t < 0.3 or self.cam.ready()
                or getattr(self.cam, 'open_error', None)):
            return
        try:
            self.io.look()
        except Exception:
            self.io.dets, self.io.dets_t = [], time.time()

    def pucks(self):
        lay = self.layout()
        if 'pick' not in lay or 'dir' not in lay:
            return []
        self._peek()
        if time.time() - self.io.dets_t > 2.0:
            return []
        (px, py), (ux, uy) = lay['pick'], lay['dir']
        out = []
        for i, d in enumerate(self.io.dets):
            sm = d['s_mm'] / 1000.0
            out.append((f'cube_{d["color"]}_{i}', px + ux * sm, py + uy * sm, S.BELT_TOP_Z))
        return out

    def set_targets(self, q):
        pass                                      # manual moves go through the twin core's link

    def belt(self, on, speed=None):
        self.start() if on else self.pause()

    def request(self, text):
        from scara_voice import parse_command
        cmd = parse_command(text) or (text.strip().lower() if text.strip().lower() in ('any', 'none') else None)
        if cmd == 'stop':
            self.pause()
        elif cmd == 'start':
            self.start()
        elif cmd:
            if self.mode != 'command':
                self.log(f'"{text}" ignored: sorting mode picks every colour (start with --mode command)')
                return False
            self.pending.append('any' if cmd == 'all' else cmd)
            self.start()
        return True

    def info(self):
        so = self.sorter
        return {'status': self.status[-6:], 'cell_mode': self.mode, 'running': self.running,
                'active': so.requests if so else [], 'sequence': so.sequence if so else [],
                'count_left': so.count_left if so else None,
                'sorted': so.sorted if so else {c: 0 for c in S.COLOR_ORDER},
                'layout': self.layout()}

    # ---- teaching
    def teach(self, what):
        if what not in TEACH_ITEMS:
            return f'teach one of {", ".join(TEACH_ITEMS)}'
        j = self.link.joints()
        st = self.link.status()
        if not j or not st:
            return 'no position from the robot yet - is it connected?'
        if not st[3]:
            return 'home the robot first (Home switches / K, or Zero here at the ready pose) - poses taught before that mean nothing'
        self.cell['poses'][what] = {n: round(j[n], 5) for n in S.ARM_JOINTS}
        path = save_cell(self.cell, self.cell_path)
        x, y = tool_xy(j)
        self.log(f'Saved "{what}" at gripper x={x:+.3f} y={y:+.3f} m, Z {j["shoulder_joint"] * 1000:+.0f} mm ({path})')
        return None

    def problems(self):
        out = list(self.cam.ready())
        if getattr(self.cam, 'open_error', None):
            out.insert(0, 'camera: ' + self.cam.open_error)
        try:
            so = HwSorter(_Quiet(), self.cell, self.mode)
            miss = so.missing()
            if miss:
                out.append('pose(s) not taught yet: ' + ', '.join(miss))
        except KeyError as e:
            out.append(f'pose not taught yet: {e.args[0]}')
        except ValueError as e:
            out.append(str(e))
        st = self.link.status()
        if not st or not st[3]:
            out.append('robot not homed - press Home switches (K), or Zero here (H) with the arm at the ready pose')
        return out

    # ---- running
    def start(self):
        with self.lock:
            if self.running:
                return True
            probs = self.problems()
            if probs:
                for p in probs:
                    self.log('Cannot start: ' + p, 'warning')
                return False
            self.io.abort.clear()
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True

    def pause(self):
        if self.running:
            self.log('Pausing - the robot finishes this move and parks')
        self.io.abort.set()
        try:
            self.link.stop()
            self.link.conveyor(False)
            self.io.belt_on = False
        except Exception:
            pass
        self.running = False

    def _run(self):
        try:
            if self.sorter is None:
                self.sorter = HwSorter(self.io, self.cell, self.mode)
            so = self.sorter
            so.go_ready()
            idle = 0
            while self.running:
                while self.pending:
                    so.request(self.pending.pop(0))
                if self.mode == 'command' and not so.requests:
                    self.io.set_belt(True)              # nothing asked for: cubes ride past
                    self.io.look()
                    self.io.sleep(0.1)
                    continue
                r = so.step()
                idle = idle + 1 if r == 'idle' else 0
                if idle == 3:
                    self.log('No cube for a while - the belt keeps running')
        except CellAborted:
            pass
        except Exception as e:
            self.log(f'cell stopped: {e}')
        finally:
            try:
                self.link.conveyor(False)
            except Exception:
                pass
            self.io.belt_on = False
            self.running = False


class _Quiet:
    def log(self, *_):
        pass

    def joints(self):
        return {}


# ---------------------------------------------------------------------------
JOG_KEYS = {ord('a'): ('column_joint', +0.03), ord('d'): ('column_joint', -0.03),
            ord('w'): ('shoulder_joint', +0.003), ord('s'): ('shoulder_joint', -0.003),
            ord('j'): ('forearm_joint', +0.03), ord('l'): ('forearm_joint', -0.03),
            ord('i'): ('wrist_joint', +0.05), ord('k'): ('wrist_joint', -0.05)}


def main():
    import cv2
    from scara_cam2d import Camera2D
    from scara_twin_protocol import open_link
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=None, help='serial port of the robot (/dev/ttyACM0, COM5)')
    ap.add_argument('--fake', action='store_true', help='software robot, for trying the program')
    ap.add_argument('--mode', default='sort', choices=['sort', 'command'])
    ap.add_argument('--source', default=None, help='camera (overrides the setup file)')
    ap.add_argument('--teach', action='store_true', help='jog the arm and save the poses')
    ap.add_argument('--home', action='store_true', help='home on the limit switches (K) at start')
    ap.add_argument('--zero-here', action='store_true', help='robot without switches: "zero here" (arm is at the ready pose)')
    a = ap.parse_args()
    if not a.port and not a.fake:
        ap.error('give --port /dev/ttyACM0 (or --fake to try without the robot)')
    cam = Camera2D.load()
    if a.source is not None:
        cam.cfg['source'] = int(a.source) if a.source.isdigit() else a.source
    cam.open()
    link = open_link(a.port, fake=a.fake)
    time.sleep(0.5)
    if a.home or a.fake:
        link.home()                       # the original build: 4 limit switches
    elif a.zero_here:
        link.zero_here()
    cell = HwCell(link, cam, a.mode)
    win = 'SCARA cell camera'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    teach_i = 0
    target = None
    grip = cell.cell['grip_open']
    print('Ready. ' + ('TEACH: jog with A/D W/S J/L I/K, O/C gripper, ENTER saves the pose shown on the image.'
                       if a.teach else 'SPACE start/pause, 1-4 colours, A any, N none, Q quit.'))
    try:
        while True:
            if not cell.running:
                try:
                    cell.io.look()
                except Exception as e:
                    print(e)
            frame = cell.io.frame.copy() if cell.io.frame is not None else None
            if frame is None:
                time.sleep(0.05)
                continue
            so = cell.sorter
            lines = []
            if a.teach:
                item = TEACH_ITEMS[teach_i]
                lines = [f'TEACH {teach_i + 1}/{len(TEACH_ITEMS)}: "{item}" - {TEACH_HELP[item]}',
                         'jog A/D column  W/S height  J/L elbow  I/K wrist  O/C gripper   ENTER save   TAB skip   Q quit']
            else:
                act = (so.requests if so else []) or (['every colour'] if a.mode == 'sort' else ['nothing'])
                lines = [f'{"RUNNING" if cell.running else "PAUSED (space)"}   mode {a.mode}   picking: {", ".join(act)}',
                         f'placed {so.sorted if so else {}}', (cell.status[-1] if cell.status else '')]
            cam.draw(frame, cell.io.dets, wanted=so.wants if so else None)
            for i, t in enumerate(lines):
                cv2.putText(frame, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(frame, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(win, frame)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            if a.teach:
                j = link.joints()
                if target is None and j:
                    target = {n: j[n] for n in S.ARM_JOINTS}
                if k in JOG_KEYS and target is not None:
                    n, dv = JOG_KEYS[k]
                    lo, hi = S.LIMITS[n]
                    target[n] = min(max(target[n] + dv, lo), hi)
                elif k == ord('o'):
                    grip = cell.cell['grip_open']
                elif k == ord('c'):
                    grip = cell.cell['grip_close']
                elif k in (13, 10):
                    err = cell.teach(TEACH_ITEMS[teach_i])
                    print(err or f'saved {TEACH_ITEMS[teach_i]}')
                    teach_i = (teach_i + 1) % len(TEACH_ITEMS)
                elif k == 9:
                    teach_i = (teach_i + 1) % len(TEACH_ITEMS)
                if target is not None:
                    link.send_joints({**target, 'left_finger_joint': grip, 'right_finger_joint': grip})
                    cell.io.grip = grip
                continue
            if k == ord(' '):
                cell.pause() if cell.running else cell.start()
            elif k in (ord('1'), ord('2'), ord('3'), ord('4')):
                cell.request(S.COLOR_ORDER[k - ord('1')] if hasattr(S, 'COLOR_ORDER') else 'red')
            elif k == ord('a'):
                cell.request('any')
            elif k == ord('n'):
                cell.request('none')
    except KeyboardInterrupt:
        pass
    finally:
        cell.pause()
        cam.close()
        link.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
