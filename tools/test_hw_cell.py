#!/usr/bin/env python3
"""
Dry run of the REAL-robot camera cell (real_robot/scara_hw_cell.py) against the MuJoCo cell,
used before going to the hardware:

  * the "robot" is a SimLink that behaves like the firmware (speed-limited joints,
    zero, conveyor relay) but moves the MuJoCo arm
  * the "webcam" is a plain 2-D image of the MuJoCo fixed camera (colour only, no depth)
  * colours, belt ruler and robot poses are set up exactly the way a user does it by
    hand (click on cubes, teach poses) - just scripted

    python3 tools/test_hw_cell.py command      # red, then any colour (one bin)
    python3 tools/test_hw_cell.py sort         # every colour to its own drop pose

Needs MuJoCo (pip install mujoco): it runs tools/mujoco_harness.py (or the C harness given in MJ_HARNESS).
"""
import math
import os
import sys
import tempfile

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control', 'tools')]
import scara_scene as S                                      # noqa: E402
import scara_kinematics as K                                 # noqa: E402
from scara_cam2d import Camera2D                             # noqa: E402
from scara_feeder import PuckFeeder                          # noqa: E402
from scara_hw_cell import HwCell, HardwareIO, CellAborted    # noqa: E402
from sim_test_sorting import HarnessIO                       # noqa: E402

HARNESS = os.environ.get('MJ_HARNESS', os.path.join(ROOT, 'tools', 'mujoco_harness.py'))
SPEED = {'column_joint': math.radians(90), 'shoulder_joint': 0.04, 'forearm_joint': math.radians(110),
         'wrist_joint': math.radians(300)}


class SimLink:
    """Same API as scara_twin_protocol's links, moving the MuJoCo robot."""

    def __init__(self, io):
        self.io = io
        self.pos = {n: 0.0 for n in S.ARM_JOINTS}
        self.target = dict(self.pos)
        self.grip = S.GRIPPER_OPEN
        self.zeroed = False
        self.conveyor_on = False
        self.motors_on = True

    def zero_here(self):
        self.zeroed = True

    def send_joints(self, q, min_change=0.0):
        for n in S.ARM_JOINTS:
            if n in q:
                lo, hi = S.LIMITS[n]
                self.target[n] = min(max(q[n], lo), hi)
        if 'left_finger_joint' in q:
            self.grip = q['left_finger_joint']
        return True

    def joints(self):
        return {**self.pos, 'left_finger_joint': self.grip, 'right_finger_joint': self.grip}

    def status(self):
        moving = any(abs(self.target[n] - self.pos[n]) > 1e-6 for n in S.ARM_JOINTS)
        return (self.io.t, None, moving, self.zeroed)

    def conveyor(self, on):
        self.conveyor_on = bool(on)
        self.io.set_belt(S.BELT_SPEED if on else 0.0)

    def stop(self):
        self.target = dict(self.pos)

    def close(self):
        pass

    def out_of_range(self, q):
        from scara_twin_protocol import HardwareMap, HW_LIMITS
        hw = HardwareMap().to_hw(q)
        names = ['column_joint', 'forearm_joint', 'wrist_joint', 'shoulder_joint']
        return [n for n, v, (lo, hi) in zip(names, hw, HW_LIMITS) if not lo - 1e-6 <= v <= hi + 1e-6]

    def tick(self, dt):
        for n in S.ARM_JOINTS:
            d = self.target[n] - self.pos[n]
            st = SPEED[n] * dt
            self.pos[n] += max(-st, min(st, d))
        self.io.ctrl[:4] = [self.pos[n] for n in S.ARM_JOINTS]
        self.io.ctrl[4] = self.io.ctrl[5] = self.grip
        self.io.send_ctrl()


class SimHardwareIO(HardwareIO):
    """Virtual clock: sleeping advances the simulation."""

    def now(self):
        return self.link.io.t

    def sleep(self, dt):
        if self.abort.is_set():
            raise CellAborted()
        end = self.link.io.t + dt
        while self.link.io.t < end - 1e-9:
            step = min(0.02, end - self.link.io.t)
            self.link.tick(step)
            self.link.io.advance(step)


def main(mode='command', n_pucks=14, sim_limit=300.0):
    xml = os.path.join(ROOT, 'mujoco', 'models', 'scara_command_mujoco.xml' if mode == 'command' else 'scara_scene_mujoco.xml')
    io = HarnessIO(HARNESS, xml)
    _, io._pk = io.state()
    io.log = lambda t: print(f'[{io.t:7.2f}s] {t}', flush=True)
    link = SimLink(io)
    link.zero_here()
    # move the arm out of the camera view (the real user parks it there before set-up)
    rq = K.arm_pose(*S.READY_XY, S.Z_TRAVEL)
    link.send_joints(dict(zip(S.ARM_JOINTS, rq)))
    for _ in range(200):
        link.tick(0.02)
        io.advance(0.02)

    # ---- the "webcam": 2-D colour image of the fixed camera, rendered at most every 0.1 s
    W, H = S.FIXED_CAM_WIDTH, S.FIXED_CAM_HEIGHT
    tmp = tempfile.mktemp(prefix='hwcam_')
    cache = {'t': -1.0, 'img': None}

    def grab():
        if cache['img'] is None or io.t - cache['t'] >= 0.1:
            io.cmd(f'render fixed_camera {W} {H} {tmp}')
            rgb = np.fromfile(tmp + '.rgb', np.uint8).reshape(H, W, 3)
            cache['img'], cache['t'] = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), io.t
        return cache['img'].copy()

    cam = Camera2D(path=os.path.join(tempfile.gettempdir(), 'test_hw_camera.json'))
    cam.frame_source = grab
    names = sorted(io._pk)

    def put(name, s):
        x, y = S.belt_point(s)
        io.set_pos(name, (x, y, S.BELT_TOP_Z))

    def hide_all():
        for i, n in enumerate(names):
            io.set_pos(n, S.hidden_xyz(i))

    def blob(name, s):
        """Where a puck at belt position s shows up in the image (what the user clicks)."""
        hide_all()
        io.advance(0.05)
        cache['img'] = None
        empty = grab().astype(int)
        put(name, s)
        io.advance(0.05)
        cache['img'] = None
        img = grab()
        diff = (np.abs(img.astype(int) - empty).sum(2) > 60).astype(np.uint8)
        cnts, _ = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnts, key=cv2.contourArea)
        m = cv2.moments(c)
        return img, (m['m10'] / m['m00'], m['m01'] / m['m00']), cv2.boundingRect(c)

    # ---- camera set-up, like scara_camera_setup.py: colours, belt area, ruler
    s_pick = sum(S.PICK_WINDOW_S) / 2
    boxes = []
    for color in S.COLOR_ORDER:
        n = next(x for x in names if S.puck_color(x) == color)
        for s in (s_pick - 0.10, s_pick, s_pick + 0.05):
            img, c, box = blob(n, s)
            ok, msg = cam.train_color(img, color, int(c[0]), int(c[1]))
            boxes.append(box)
        print('trained', msg)
    n0 = names[0]
    _, p0, _ = blob(n0, s_pick)
    _, p1, _ = blob(n0, s_pick - 0.06)
    _, p2, _ = blob(n0, s_pick + 0.06)
    cam.set_ruler(p0, p1, 60.0, p2)
    for s in (s_pick - 0.30, s_pick + 0.15):
        boxes.append(blob(n0, s)[2])
    x0 = min(b[0] for b in boxes) - 25
    y0 = min(b[1] for b in boxes) - 25
    x1 = max(b[0] + b[2] for b in boxes) + 25
    y1 = max(b[1] + b[3] for b in boxes) + 25
    cam.cfg['roi'] = [max(0, x0), max(0, y0), min(W, x1), min(H, y1)]
    # ruler check: a puck 30 mm upstream must read about +30 mm
    img, c, _ = blob(n0, s_pick - 0.03)
    d = [x for x in cam.detect(img)]
    print('ruler check (expect +30 mm):', [f'{x["color"]} {x["s_mm"]:+.1f}' for x in d])
    assert d and abs(d[0]['s_mm'] - 30) < 6, d
    hide_all()

    # ---- teach the robot poses, like with the jog keys on the real arm
    cellfile = os.path.join(tempfile.gettempdir(), 'test_hw_cell.json')
    if os.path.exists(cellfile):
        os.remove(cellfile)
    cell = HwCell(link, cam, mode=mode, cell_path=cellfile, log=io.log)
    cell.io = SimHardwareIO(link, cam, cell.cell, log=cell.log)

    def teach(what, x, y, z, yaw=None):
        q = K.arm_pose(x, y, z, yaw=yaw)
        link.pos = dict(zip(S.ARM_JOINTS, q))
        link.target = dict(link.pos)
        assert cell.teach(what) is None

    px, py = S.belt_point(s_pick)
    belt_yaw = math.atan2(S.BELT_DIR[1], S.BELT_DIR[0]) + math.pi / 2   # fingers close across the belt
    teach('pick', px, py, S.Z_PICK, yaw=belt_yaw)
    teach('upstream', *S.belt_point(s_pick - 0.08), S.Z_PICK, yaw=belt_yaw)
    if mode == 'command':
        teach('drop_any', *S.command_bin_xy(), S.Z_PLACE, yaw=S.command_bin_yaw())
    else:
        for c in S.COLOR_ORDER:
            teach(f'drop_{c}', *S.bin_xy(c), S.Z_PLACE, yaw=S.bin_yaw(c))
    teach('ready', *S.READY_XY, S.Z_TRAVEL)
    cell.cell['grip_close'] = S.GRIP_CLOSE
    print('problems before start:', cell.problems())
    assert not cell.problems()

    # ---- run: the feeder drops pucks on the belt; the cell does everything else
    io.feeder = PuckFeeder(io.get_pos, io.set_pos, seed=3, max_spawn=n_pucks, log=io.log, mode=mode)
    from scara_hw_cell import HwSorter
    cell.sorter = HwSorter(cell.io, cell.cell, mode)
    so = cell.sorter
    if mode == 'command':
        so.request('red')
    cell.io.abort.clear()
    so.go_ready()
    switched = False
    if os.environ.get('DEBUG_CAM'):
        orig = cell.io.look
        last = {'t': -9}

        def look():
            d = orig()
            if io.t - last['t'] > 1.0:
                last['t'] = io.t
                print(f'  [{io.t:6.1f}] cam:', [f'{x["color"]} {x["s_mm"]:+.0f}' for x in d],
                      'pucks on belt:', sum(1 for n, p in io._pk.items() if abs(p[2] - S.BELT_TOP_Z) < 0.02))
            return d
        cell.io.look = look
    lay = cell.layout()
    assert math.hypot(lay['pick'][0] - px, lay['pick'][1] - py) < 0.003, lay
    twin_err = []

    def twin_check():
        # the app's twin: every cube the camera sees is drawn where it really is on the belt
        cell.io.look()
        _, pk = io.state()
        for name, x, y, z in cell.pucks():
            c = name.split('_')[1]
            best = min((math.hypot(p[0] - x, p[1] - y) for n, p in pk.items()
                        if c in n and abs(p[2] - S.BELT_TOP_Z) < 0.02), default=None)
            if best is not None:
                twin_err.append(best)
    idle = 0
    while io.t < sim_limit:
        twin_check()
        if mode == 'command' and not switched and io.t > 50.0:
            so.request('any')
            switched = True
        r = so.step(timeout=20.0)
        idle = idle + 1 if r == 'idle' else 0
        if idle >= 2 and io.feeder.spawned >= n_pucks:
            break
    print(f'twin: {len(twin_err)} cubes drawn, worst position error {1000 * max(twin_err or [0]):.1f} mm')
    assert twin_err and max(twin_err) < 0.025, twin_err
    print('RESULT', mode, so.sorted, 'failures', so.failures, f'sim {io.t:.0f} s')
    io.cmd('quit')
    total = sum(so.sorted.values())
    assert total >= (3 if mode == 'command' else 5), so.sorted
    print('HW CELL DRY RUN OK')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'command')
