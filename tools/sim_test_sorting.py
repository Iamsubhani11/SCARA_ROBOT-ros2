#!/usr/bin/env python3
"""
End-to-end test of the colour sorter in MuJoCo through the headless C harness
(used during development, where the Python `mujoco` package was unavailable).

    python3 tools/sim_test_sorting.py <harness-binary> [colors] [video.mp4]

Pipeline exercised: rendered fixed-camera RGB + depth -> scara_vision ->
scara_sorter_core -> scara_kinematics -> joint trajectories -> physics.
"""
import json
import os
import subprocess
import tempfile
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import scara_scene as S              # noqa: E402
import scara_vision as V             # noqa: E402
from scara_sorter_core import Sorter  # noqa: E402
from scara_feeder import PuckFeeder, classify  # noqa: E402
from scara_scene import puck_color  # noqa: E402

DT = 0.002
# the MuJoCo "harness" the tests talk to: tools/mujoco_harness.py (needs: pip install mujoco), or a path
# to the compiled C version given on the command line / in MJ_HARNESS
DEFAULT_HARNESS = os.environ.get('MJ_HARNESS', os.path.join(ROOT, 'tools', 'mujoco_harness.py'))


class HarnessIO:
    def __init__(self, harness, model, video=None, fps=15, size=(960, 540)):
        env = dict(os.environ, EGL_PLATFORM='surfaceless')
        exe = [sys.executable, harness] if harness.endswith('.py') else [harness]   # Python (mujoco package) or C harness
        self.p = subprocess.Popen(exe + [model], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True, env=env, bufsize=1)
        assert self.p.stdout.readline().startswith('READY')
        self.t = 0.0
        self.ctrl = [0.0, 0.0, 0.0, 0.0, S.GRIPPER_OPEN, S.GRIPPER_OPEN, 0.0]   # last = belt speed
        self.send_ctrl()
        self.cam = V.fixed_camera_model()
        self.last_det_t = -1.0
        self.last_dets = []
        self.fps = fps
        self.next_frame = 0.0
        self.video = video
        self.size = size
        self.logs = []
        self.feeder = None
        self.next_feed = 0.0
        if video:
            self.raw = video + '.rgb'
            self.cmd(f'vopen {self.raw} {size[0]} {size[1]}')

    def cmd(self, line):
        self.p.stdin.write(line + '\n')
        return self.p.stdout.readline().strip()

    def send_ctrl(self):
        self.cmd('ctrl ' + ' '.join(f'{c:.6f}' for c in self.ctrl))

    def advance(self, dt):
        n = max(1, int(round(dt / DT)))
        while n > 0:
            if self.video:
                k = max(1, int(round((self.next_frame - self.t) / DT)))
                k = min(k, n)
            else:
                k = n
            r = self.cmd(f'step {k}')
            self.t = float(r.split()[1])
            n -= k
            if self.feeder and self.t >= self.next_feed:
                self.next_feed = self.t + 0.25
                _, pk = self.state()
                self._pk = pk
                self.feeder.update(self.t)
            if self.video and self.t >= self.next_frame - 1e-9:
                self.cmd('vframe overview')
                self.next_frame += 1.0 / self.fps

    # ---- IO interface used by Sorter
    def now(self):
        return self.t

    def sleep(self, dt):
        self.advance(dt)

    def _interp(self, idx, target, T):
        start = [self.ctrl[i] for i in idx]
        steps = max(1, int(T / 0.02))
        for s in range(1, steps + 1):
            a = s / steps
            for i, j in enumerate(idx):
                self.ctrl[j] = start[i] + (target[i] - start[i]) * a
            self.send_ctrl()
            self.advance(T / steps)

    def move_arm(self, q, T):
        self._interp([0, 1, 2, 3], q, T)
        self.advance(0.15)
        if os.environ.get('DEBUG_MOVES'):
            j = self.joints(); _, pk = self.state()
            tcp = self.cmd('state').split('tcp')[1].split()
            self.log(f'  move T={T:.2f} target={[round(v, 3) for v in q]} actual={[round(j[n], 3) for n in S.ARM_JOINTS]} tcp={tcp}')

    def move_gripper(self, q, T):
        self._interp([4, 5], [q, q], T)

    def set_belt(self, v):
        self.ctrl[6] = v
        self.send_ctrl()

    def get_pos(self, name):
        return self._pk[name]

    def set_pos(self, name, xyz, yaw=0.0):
        self.cmd(f'setpuck {name} {xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} {yaw:.4f}')
        self._pk[name] = tuple(xyz)

    def state(self):
        line = self.cmd('state')
        parts = line.split('|')
        head = parts[0].split()
        q = {kv.split('=')[0]: float(kv.split('=')[1]) for kv in head[2:]}
        pucks = {}
        toks = parts[1].split()
        for i in range(0, len(toks), 4):
            pucks[toks[i]] = tuple(float(v) for v in toks[i + 1:i + 4])
        return q, pucks

    def joints(self):
        q, _ = self.state()
        return q

    def detections(self):
        if self.t - self.last_det_t >= 0.1:        # 10 Hz camera, like Gazebo
            base = os.path.join(tempfile.gettempdir(), f'simcam_{os.getpid()}')
            self.cmd(f'render fixed_camera {S.FIXED_CAM_WIDTH} {S.FIXED_CAM_HEIGHT} {base}')
            rgb = np.fromfile(base + '.rgb', np.uint8).reshape(S.FIXED_CAM_HEIGHT, S.FIXED_CAM_WIDTH, 3)
            dep = np.fromfile(base + '.depth', np.float32).reshape(S.FIXED_CAM_HEIGHT, S.FIXED_CAM_WIDTH)
            self.bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            self.last_dets = V.detect_pucks(self.bgr, dep, self.cam)
            self.last_det_t = self.t
        return self.last_dets

    def log(self, text):
        if os.environ.get('DEBUG_PUCKS'):
            _, pk = self.state()
            text += '  | ' + ' '.join(f"{k[5:]}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})" for k, v in pk.items())
        line = f'[{self.t:7.2f}s] {text}'
        print(line, flush=True)
        self.logs.append(line)

    def close(self):
        if self.video:
            self.cmd('vclose')
        self.cmd('quit')
        self.p.wait(timeout=10)


def main():
    """
    MODE=sort (default): every colour into its own bin (argv[2] = colours to sort, default all).
    MODE=command       : one bin.  REQUESTS is a timed list of commands, like typed / spoken
                         ones, e.g. "red@0,blue@60,any@120,none@170" (seconds of sim time).
                         A colour stays active until the next command.  Checks: every puck in
                         the bin was of a colour active when it was picked, and no puck of an
                         active colour rode off the end of the belt.
    """
    harness = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != '-' else DEFAULT_HARNESS
    colors = sys.argv[2] if len(sys.argv) > 2 else 'all'
    video = sys.argv[3] if len(sys.argv) > 3 else None
    mode = os.environ.get('MODE', 'sort')
    n_spawn = int(os.environ.get('N_PUCKS', '8'))
    seed = int(os.environ.get('SEED', '0'))
    schedule = []
    for item in [r for r in os.environ.get('REQUESTS', 'red@0,blue@60').split(',') if r]:
        cmd, _, t = item.partition('@')
        schedule.append((float(t or 0), cmd.replace('_', ' ')))
    schedule.sort()
    xml = 'scara_command_mujoco.xml' if mode == 'command' else 'scara_scene_mujoco.xml'
    io = HarnessIO(harness, os.path.join(ROOT, 'mujoco', 'models', xml), video=video)
    _, io._pk = io.state()
    io.feeder = PuckFeeder(io.get_pos, io.set_pos, seed=seed, max_spawn=n_spawn, log=io.log, mode=mode)
    io.sleep(0.5)
    sorter = Sorter(io, colors, mode=mode)
    timeline = [(0.0, frozenset())]                # (time, active colours) - command mode
    picks = []                                     # (time, colour, active colours when picked)

    def note_active():
        cur = frozenset(sorter.active)
        if cur != timeline[-1][1]:
            timeline.append((io.now(), cur))
    orig_pp = sorter.pick_and_place

    def pick_and_place(det):
        picks.append((io.now(), det['color'], frozenset(sorter.active)))
        ok = orig_pp(det)
        note_active()
        return ok
    sorter.pick_and_place = pick_and_place
    sorter.go_ready()
    results = []
    idle = 0
    while len(results) < 6 * n_spawn:
        while mode == 'command' and schedule and schedule[0][0] <= io.now():
            sorter.request(schedule.pop(0)[1])
            note_active()
        r = sorter.step(timeout=2.0)
        results.append(r)
        idle = idle + 1 if r == 'idle' else 0
        if idle >= 8 and io.feeder.spawned >= n_spawn and not schedule:
            break
    io.sleep(4.0)                                    # let the feeder clear the last bin / drops
    _, pk = io.state()
    io._pk = pk
    left = {n: classify(*p, mode=mode) for n, p in pk.items() if classify(*p, mode=mode) != 'pool'}
    handled = [(n, w) for _, n, w in io.feeder.history] + list(left.items())
    bad = []
    if mode == 'command':
        def active_at(t):
            a = frozenset()
            for tt, act in timeline:
                if tt <= t:
                    a = act
            return a

        def active_since(c, t):
            """how long colour c has been active continuously at time t"""
            since = None
            for tt, act in timeline:
                if tt > t:
                    break
                on = 'any' in act or c in act
                if on and since is None:
                    since = tt
                elif not on:
                    since = None
            return 0.0 if since is None else t - since
        for t, c, act in picks:
            if not ('any' in act or c in act):
                bad.append(('picked', c, f'not active ({sorted(act)}) at {t:.1f}s'))
        for t, n, w in io.feeder.history:
            c = puck_color(n)
            if w in ('end', 'lost') and active_since(c, t) > 30.0:
                bad.append((n, w, f'{c} was active since {active_since(c, t):.0f}s - should have been picked'))
            if w not in ('bin_drop', 'end', 'lost'):
                bad.append((n, w, 'bin_drop or end'))
        in_bin = [puck_color(n) for n, w in handled if w == 'bin_drop']
        io.log(f'timeline {[(round(t, 1), sorted(a)) for t, a in timeline]}')
        io.log(f'picked {[(round(t, 1), c) for t, c, _ in picks]}, bin received {in_bin}')
    else:
        wanted = sorter.wanted
        for n, w in handled:
            c = puck_color(n)
            good = f'bin_{c}' if c in wanted else 'end'
            if c not in wanted and w == 'lost':
                w = 'end'                        # rode off the open end of the belt and fell - expected
            if w != good:
                bad.append((n, w, good))
    io.log(f'step results: {results}')
    io.log(f'spawned {io.feeder.spawned}, handled {len(handled)}: {handled}')
    io.log(f'wrong: {bad}')
    io.close()
    ok = not bad and len(handled) == io.feeder.spawned == n_spawn
    if mode == 'command':
        ok = ok and len(picks) >= 2
    print(json.dumps({'mode': mode, 'colors': colors, 'seed': seed, 'spawned': io.feeder.spawned,
                      'handled': len(handled), 'sorted': sorter.sorted, 'failures': sorter.failures,
                      'wrong': bad, 'PASS': ok}))
    if video:
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                        '-s', f'{io.size[0]}x{io.size[1]}', '-r', str(io.fps), '-i', io.raw,
                        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', video], check=True)
        os.remove(io.raw)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
