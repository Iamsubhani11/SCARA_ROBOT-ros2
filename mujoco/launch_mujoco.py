#!/usr/bin/env python3
"""
SCARA cell in MuJoCo - three modes, like the Mechamind Labs videos:

    python3 mujoco/launch_mujoco.py                      # MANUAL : table + 3 blue / 3 green pucks (no conveyor),
                                                          #          move the arm with the viewer's sliders
    python3 mujoco/launch_mujoco.py --gui                # MANUAL with the Twin Control GUI (safety mode,
                                                          #          pick assist, teach & replay, phone app)
    python3 mujoco/launch_mujoco.py --auto [--show]      # SORTING: endless conveyor, every puck goes into the
                                                          #          bin of its own colour (4 bins)
    python3 mujoco/launch_mujoco.py --command [--show]   # COMMAND: endless conveyor, ONE bin.  Type a colour
                                                          #          ("red", "blue and green", "any", "3 red") - the
                                                          #          robot picks EVERY puck of that colour until
                                                          #          you give the next one ("none" = stop picking);
                                                          #          the rest ride off the end of the belt
    python3 mujoco/launch_mujoco.py --command --voice    # same, but say "Hey SCARA ... the red one"
                                                          #   (--llm: ChatGPT understands free sentences)

In SORTING and COMMAND mode the fixed depth camera image is rendered by
MuJoCo, pucks are found with common/scara_vision.py and the arm moves with
the same code as Gazebo (common/scara_sorter_core.py).  The conveyor keeps
running; pucks are dropped on one at a time (common/scara_feeder.py) and
the bin(s) are emptied automatically, so the cell runs until you close it.

    --show                        camera windows (fixed depth camera detector + arm camera)
    --gui                         SCARA Twin Control in the browser (watch the cell, ask for colours,
                                  connect the real robot) + phone-app server on port 9090
    --twin /dev/ttyACM0           digital twin: the real SCARA (real_robot/firmware/scara_twin) copies every
                                  move of the simulated one ("--twin fake" to try without hardware)
    --keep-in-bins --capacity 3   real-cell mode: bins are NOT emptied; after 3 pucks a bin is
                                  "full" until you type / say "empty bins"
Commands (typed or spoken): a colour (stays active until the next), "any", "none", "3 red",
"stop" / "start" (belt), "empty bins".
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import math
import os
import sys
import time

import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scara_scene as S
import scara_safety as SF                 # noqa: E402
from scara_sorter_core import Sorter    # noqa: E402

XML = S.pkg_file('mujoco', 'models', 'scara_scene_mujoco.xml')           # sorting cell (--auto)
XML_COMMAND = S.pkg_file('mujoco', 'models', 'scara_command_mujoco.xml')  # one bin (--command / --voice)
XML_MANUAL = S.pkg_file('mujoco', 'models', 'scara_manual_mujoco.xml')   # table + pucks (sliders)


def open_twin(spec, homing='switches'):
    """--twin /dev/ttyACM0 | COM5 | fake  ->  link to the real robot"""
    from scara_twin_protocol import open_link
    link = open_link(None if spec == 'fake' else spec, fake=spec == 'fake')
    if homing == 'switches':
        link.home()
    elif homing == 'zero':
        link.zero_here()
    return link


def wrap_belt(data):
    """Endless belt: slide the (uniform, invisible) belt plate back once it moved BELT_WRAP."""
    q = data.joint('belt_joint').qpos
    if q[0] > S.BELT_WRAP:
        q[0] -= S.BELT_WRAP


_twin_warned = set()


def send_to_twin(link, model, data):
    q = {n: float(data.joint(n).qpos[0]) for n in S.ARM_JOINTS + S.GRIPPER_JOINTS}
    for n in link.out_of_range(q):
        if n not in _twin_warned:
            _twin_warned.add(n)
            print(f'[twin] {n} = {q[n]:+.2f} is outside the real robot\'s range - it stops at its limit '
                  '(see config/twin_hardware.yaml / README section 5)')
    link.send_joints(q)


class MujocoIO:
    """Sorter IO: blocking calls keep the physics + viewer running in real time."""

    def __init__(self, model, data, viewer, show=False, feed=True, clear_bins=True, seed=None, mode='sort'):
        import cv2
        import scara_vision as V
        from scara_feeder import PuckFeeder
        self.cv2, self.V = cv2, V
        self.m, self.d, self.viewer = model, data, viewer
        self.act = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(model.nu)}
        self.renderer = mujoco.Renderer(model, height=S.FIXED_CAM_HEIGHT, width=S.FIXED_CAM_WIDTH)
        self.arm_renderer = mujoco.Renderer(model, height=S.ARM_CAM_HEIGHT, width=S.ARM_CAM_WIDTH) if show else None
        self.cam = V.fixed_camera_model()
        self.show = show
        self.last_det_t = -1.0
        self.dets = []
        self.wall0 = time.time() - data.time
        self.ctrl = {'act_column': 0.0, 'act_shoulder': 0.0, 'act_forearm': 0.0, 'act_wrist': 0.0,
                     'act_left_finger': S.GRIPPER_OPEN, 'act_right_finger': S.GRIPPER_OPEN, 'act_belt': 0.0}
        names = [n for n in S.puck_names() if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) >= 0]
        # endless supply: pucks come out of an invisible pool one at a time, and
        # go back to it from the bins / the end of the belt (bins never fill up)
        self.feeder = PuckFeeder(self._get_pos, self._set_pos, names=names, seed=seed,
                                 clear_bins=clear_bins, log=self.log, mode=mode) if feed else None
        self.mode = mode
        self.next_feed = 0.0
        self.snapshot = None                 # joint values for other threads (GUI), refreshed by advance()
        self.puck_snapshot = []              # [(name, x, y, z_bottom)] of the pucks in view, same moment
        self.next_snap = 0.0
        self.twin = None
        self.next_twin = 0.0
        self._last_ui = 0.0                  # camera windows are refreshed ~12x a second, also while the arm moves

    def _get_pos(self, name):
        return tuple(float(v) for v in self.d.joint(f'{name}_free').qpos[:3])

    def _set_pos(self, name, xyz, yaw=0.0):
        j = self.d.joint(f'{name}_free')
        j.qpos[:] = [xyz[0], xyz[1], xyz[2], math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        j.qvel[:] = 0

    def empty_bins(self, colors=None):
        """Operator emptied the bins (used when automatic emptying is off)."""
        import scara_feeder
        for i, n in enumerate(self.feeder.names if self.feeder else []):
            where = scara_feeder.classify(*self._get_pos(n), mode=self.mode)
            if where.startswith('bin_') and (not colors or where[4:] in colors):
                with self.viewer.lock():
                    self._set_pos(n, S.hidden_xyz(i))

    # ---- simulation stepping
    def _apply(self):
        for k, v in self.ctrl.items():
            if k in self.act:
                self.d.ctrl[self.act[k]] = v

    def advance(self, dt):
        # count physics steps (comparing float times would overshoot by a whole chunk)
        n = max(1, int(round(dt / self.m.opt.timestep)))
        while n > 0:
            if not self.viewer.is_running():
                raise SystemExit
            k = min(10, n)
            with self.viewer.lock():
                self._apply()
                for _ in range(k):
                    mujoco.mj_step(self.m, self.d)
                wrap_belt(self.d)
                if self.d.time >= self.next_snap:
                    self.next_snap = self.d.time + 0.05
                    self.snapshot = {n: float(self.d.joint(n).qpos[0]) for n in S.ARM_JOINTS + ['left_finger_joint']}
                    self.puck_snapshot = SF.mujoco_pucks(self.m, self.d)   # for the app / website 3-D view
                if self.feeder and self.d.time >= self.next_feed:
                    self.next_feed = self.d.time + 0.25
                    self.feeder.update(self.d.time)
                if self.twin and self.d.time >= self.next_twin:
                    self.next_twin = self.d.time + 0.05
                    send_to_twin(self.twin, self.m, self.d)
            n -= k
            self.viewer.sync()
            if self.show and time.time() - self._last_ui > 0.08:
                self._refresh_windows()
            lag = self.d.time - (time.time() - self.wall0)
            if lag > 0:
                time.sleep(lag)

    # ---- Sorter IO interface
    def now(self):
        return self.d.time

    def sleep(self, dt):
        self.advance(dt)

    def _interp(self, keys, target, T):
        start = [self.ctrl[k] for k in keys]
        n = max(1, int(T / 0.02))
        for i in range(1, n + 1):
            a = i / n
            for k, s0, s1 in zip(keys, start, target):
                self.ctrl[k] = s0 + (s1 - s0) * a
            self.advance(T / n)

    def move_arm(self, q, T):
        self._interp(['act_column', 'act_shoulder', 'act_forearm', 'act_wrist'], q, T)
        self.advance(0.15)

    def move_gripper(self, q, T):
        self._interp(['act_left_finger', 'act_right_finger'], [q, q], T)

    def follow(self, points):
        """Belt tracking: arm and gripper together through timed points [(t, [col, z, fore, wrist], g)]."""
        keys = ['act_column', 'act_shoulder', 'act_forearm', 'act_wrist', 'act_left_finger', 'act_right_finger']
        t_prev = 0.0
        for t, q, g in points:
            self._interp(keys, list(q) + [g, g], max(t - t_prev, 0.02))
            t_prev = t

    def det_time(self):
        return self.last_det_t

    def set_belt(self, v):
        self.ctrl['act_belt'] = v

    def joints(self):
        return {n: float(self.d.joint(n).qpos[0]) for n in S.ARM_JOINTS + S.GRIPPER_JOINTS}

    def _render_fixed(self, depth=True):
        r = self.renderer
        r.update_scene(self.d, camera='fixed_camera')
        rgb = r.render()
        dep = None
        if depth:
            r.enable_depth_rendering()
            r.update_scene(self.d, camera='fixed_camera')
            dep = r.render()
            r.disable_depth_rendering()
        return self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR), dep

    def detections(self):
        if self.d.time - self.last_det_t >= 0.1:          # 10 Hz camera like Gazebo
            bgr, depth = self._render_fixed()
            self.dets = self.V.detect_pucks(bgr, depth, self.cam)
            self.last_det_t = self.d.time
            if self.show:
                self._refresh_windows(bgr)
        return self.dets

    def _refresh_windows(self, bgr=None):
        """Draw both camera windows and let OpenCV handle its events - also during long
        arm moves, otherwise the desktop marks the windows "Not Responding"."""
        self._last_ui = time.time()
        try:
            with self.viewer.lock():
                if bgr is None:
                    bgr, _ = self._render_fixed(depth=False)
                self.arm_renderer.update_scene(self.d, camera='arm_camera')
                arm = self.arm_renderer.render()
            self.cv2.imshow('Fixed Camera Box Detector', self.V.draw_detections(bgr, self.dets))
            self.cv2.imshow('SCARA Arm Camera', self.cv2.resize(self.cv2.cvtColor(arm, self.cv2.COLOR_RGB2BGR), (640, 480)))
            self.cv2.waitKey(1)
        except Exception as e:          # no display: carry on without windows
            print(f'[camera windows] {e} - continuing without them')
            self.show = False

    def log(self, text):
        print(f'[{self.d.time:7.2f}s] {text}', flush=True)


def manual(model, data, twin=None):
    print(__doc__)
    print('Manual mode: open the "Control" panel on the right of the viewer and move the sliders.')
    print('  act_left_finger / act_right_finger: 0.0 = open, -0.025 = closed on a puck')
    print('  (act_belt does nothing here - the manual world has no conveyor)')
    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall0, sim0 = time.time(), data.time
        last_twin = 0.0
        while viewer.is_running():
            now = time.time()
            with viewer.lock():
                n = 0
                while data.time - sim0 < now - wall0 and n < 50:     # real time, 60 frames a second
                    mujoco.mj_step(model, data)
                    n += 1
                if n == 50:
                    wall0, sim0 = now, data.time
            if twin and now - last_twin > 0.05:
                last_twin = now
                send_to_twin(twin, model, data)
            viewer.sync()
            time.sleep(max(0.0, 0.016 - (time.time() - now)))


def start_console_for_empty(commands):
    """Type 'empty' (or 'empty red') in the terminal after emptying the bins."""
    import threading
    from scara_voice import parse_command

    def run():
        for line in sys.stdin:
            cmd = parse_command(line)
            if cmd:
                commands.append(cmd)
    threading.Thread(target=run, daemon=True).start()


class CellBackend:
    """Lets the SCARA Twin Control GUI / phone app watch and command this MuJoCo cell (--gui)."""
    kind = 'mujoco'
    autonomous = True
    has_belt = True

    def __init__(self, mode, commands):
        self.mode, self.cmds = mode, commands
        self.label = 'MuJoCo - ' + ('command cell (one bin)' if mode == 'command' else 'sorting cell (4 bins)')
        self.io = None
        self.sorter = None

    @property
    def commands(self):
        return self.mode == 'command'

    @property
    def belt_running(self):
        return bool(self.io and abs(self.io.ctrl.get('act_belt', 0.0)) > 1e-4)

    def joints(self):
        return dict(self.io.snapshot) if self.io and self.io.snapshot else None

    def set_targets(self, q):
        pass                                     # the cell moves by itself

    def belt(self, on, speed=None):
        self.cmds.append('start' if on else 'stop')

    def request(self, text):
        from scara_voice import parse_command
        if str(text).lower().startswith('track'):        # belt tracking on / off
            self.cmds.append(str(text).lower())
            return True
        cmd = parse_command(text) or ('all' if text in ('any', 'next') else None)
        if cmd:
            self.cmds.append(cmd)
        return bool(cmd)

    def pucks_seen(self):
        return self.io.dets if self.io else []

    def pucks(self):
        # the pucks on the belt, in the hand and in the bins - the app / website draws them
        return list(getattr(self.io, 'puck_snapshot', [])) if self.io else []

    def info(self):
        if not self.sorter:
            return {}
        return {'cell_mode': self.mode, 'active': self.sorter.requests, 'count_left': self.sorter.count_left,
                'sequence': self.sorter.sequence,
                'sorted': self.sorter.sorted, 'tracking': self.sorter.track_belt, 'tracked': self.sorter.tracked}


def run_cell(model, data, mode, show=False, voice=False, llm=False, stt='auto', keep=False, capacity=0,
             seed=None, twin=None, gui=False, backend=None, should_exit=None, open_browser=False, track=False):
    """mode 'sort' (4 bins, sorts everything) or 'command' (1 bin, picks what you ask for).
    backend / should_exit: used by scara_twin_server.py when the app switches MuJoCo worlds."""
    import collections
    import threading
    commands = backend.cmds if backend is not None else collections.deque()
    if track:
        commands.append('track on')              # --track-belt: follow the belt instead of stopping it
    paused = False                               # 'stop' / 'start' commands stop / start the belt
    if mode == 'command' and backend is None:
        from scara_voice import VoiceAssistant, start_console
        assistant = VoiceAssistant(commands.append, require_wake=voice, use_llm=llm)
        if voice:
            def mic():
                try:
                    assistant.listen_forever(engine=stt)
                except Exception as e:      # no microphone / packages: fall back to typing
                    print(f'[voice] {e} - type commands instead')
                    assistant.require_wake = False
                    start_console(assistant)
            threading.Thread(target=mic, daemon=True).start()
        else:
            start_console(assistant)
        print('\nType a colour (red / green / blue / yellow / any / none / "3 red") and press Enter - it stays'
              ' active until the next colour.' + ('  Or say "Hey SCARA", then the colour.' if voice else '') + '\n')
    elif keep and backend is None:
        start_console_for_empty(commands)
    if gui and backend is None:
        from scara_twin_server import TwinCore, serve
        backend = CellBackend(mode, commands)
        core = TwinCore(backend)
        serve(core, open_browser=open_browser)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        if show:
            from scara_vision import can_show
            if not can_show():
                print('No screen for the camera windows (--show) - running without them')
                show = False
        io = MujocoIO(model, data, viewer, show=show, clear_bins=not keep, seed=seed, mode=mode)
        io.twin = twin
        io.advance(0.5)
        sorter = Sorter(io, 'all', bin_capacity=capacity if keep else 0, mode=mode)
        if backend:
            backend.io, backend.sorter = io, sorter
        sorter.gripper_open()
        sorter.go_ready()
        try:
            while viewer.is_running() and not (should_exit and should_exit()):
                while commands:
                    cmd = commands.popleft()
                    if cmd == 'stop':
                        paused = True
                        io.log('Conveyor stopped (say / type "start")')
                    elif cmd == 'start':
                        paused = False
                    elif cmd.startswith('empty'):
                        sorter.empty_bins(cmd.split()[1:] or None)
                        io.empty_bins(cmd.split()[1:] or None)
                    elif cmd.startswith('track'):         # belt tracking on / off (app, website, --track-belt)
                        sorter.request(cmd)
                    elif mode == 'command':
                        sorter.request('any' if cmd == 'all' else cmd)
                        paused = False
                if paused:
                    io.set_belt(0.0)
                    io.detections()
                    io.advance(0.1)            # (advance also keeps the camera windows alive)
                    continue
                # the belt keeps running; the robot only moves for pucks it wants
                sorter.step(timeout=1.0)
        except SystemExit:
            pass


def main():
    ap = argparse.ArgumentParser(description='SCARA MuJoCo cell')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--auto', action='store_true', help='SORTING mode: every colour into its own bin')
    g.add_argument('--command', action='store_true', help='COMMAND mode: one bin, pick the colours you ask for')
    ap.add_argument('--voice', action='store_true', help='command mode: listen to the microphone ("Hey SCARA")')
    ap.add_argument('--ask', action='store_true', help=argparse.SUPPRESS)       # old name of --command
    ap.add_argument('--show', action='store_true', help='show the camera windows')
    ap.add_argument('--gui', action='store_true', help='also serve the GUI / phone app (http://localhost:8080, ws 9090)')
    ap.add_argument('--open', action='store_true', help='with --gui: open the GUI in the browser')
    ap.add_argument('--llm', action='store_true', help='use ChatGPT to understand sentences (OPENAI_API_KEY)')
    ap.add_argument('--stt', default='auto', help='speech-to-text: auto | whisper | openai | google')
    ap.add_argument('--keep-in-bins', action='store_true', help='do not empty the bins automatically')
    ap.add_argument('--capacity', type=int, default=4, help='pucks per bin before it counts as full (with --keep-in-bins)')
    ap.add_argument('--seed', type=int, default=None, help='random seed for the puck order')
    ap.add_argument('--twin', default=None, help='digital twin: serial port of the real robot, or "fake"')
    ap.add_argument('--twin-homing', default='switches', choices=['switches', 'zero', 'none'])
    ap.add_argument('--track-belt', action='store_true',
                    help='belt tracking: pick pucks from the moving belt (Jacobian) instead of stopping it')
    args, _ = ap.parse_known_args()
    mode = 'command' if (args.command or args.voice or args.ask) else ('sort' if args.auto else 'manual')
    if mode == 'manual' and args.gui:
        # manual table + Twin Control GUI (sliders, keys, safety mode, teach & replay, phone app)
        import scara_twin_server
        sys.argv = ['scara_twin_server.py', '--sim', 'mujoco'] + (['--open'] if args.open else []) + (['--fake'] if args.twin == 'fake' else
                                                                  ['--port', args.twin] if args.twin else [])
        return scara_twin_server.main()
    model = mujoco.MjModel.from_xml_path({'sort': XML, 'command': XML_COMMAND, 'manual': XML_MANUAL}[mode])
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    twin = open_twin(args.twin, args.twin_homing) if args.twin else None
    if mode == 'manual':
        manual(model, data, twin)
    else:
        run_cell(model, data, mode, args.show, args.voice, args.llm, args.stt,
                 keep=args.keep_in_bins, capacity=args.capacity, seed=args.seed, twin=twin, gui=args.gui,
                 open_browser=args.open, track=args.track_belt)


if __name__ == '__main__':
    main()
