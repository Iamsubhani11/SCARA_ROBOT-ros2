#!/usr/bin/env python3
"""The real camera cell as a digital twin, end to end (no robot, no webcam needed):

  * a still picture stands in for the webcam: a red cube 60 mm upstream of the pick point
    and a green one 30 mm downstream, on a belt ruler of 0.6 mm per pixel
  * the cell poses are taught like on the real robot (pick, upstream, drops, parking)
  * scara_twin_server.py --sim hardware --fake runs with that camera and those poses
  * checks: the server reports where the belt is (from the taught poses) and puts each
    cube where it really is; the app (served by the same server) draws the hardware cell
    and the two cubes, and the FK/IK tab moves the (software) robot.

    python3 tools/test_hw_twin.py [--shots DIR]
Needs: OpenCV, numpy, playwright + chromium."""
import argparse
import asyncio
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import scara_scene as S          # noqa: E402
import scara_kinematics as K     # noqa: E402

FAIL = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what)
    if not ok:
        FAIL.append(what)


def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.loads(r.read())


def setup(home):
    # the "webcam" picture
    img = np.full((480, 640, 3), 60, np.uint8)
    cv2.rectangle(img, (0, 200), (640, 280), (25, 25, 25), -1)                  # black belt
    cv2.rectangle(img, (400, 220), (440, 260), (30, 30, 210), -1)              # red cube at x=420 -> +60 mm
    cv2.rectangle(img, (250, 220), (290, 260), (40, 170, 40), -1)              # green cube at x=270 -> -30 mm
    png = os.path.join(home, 'belt.png')
    cv2.imwrite(png, img)
    cam = {'source': png, 'roi': None, 'min_area': 300, 'max_area': 60000, 'window_mm': [-12.0, 18.0],
           'colors': {'red': {'h': [-10, 10], 's': [120, 255], 'v': [80, 255], 'samples': 3},
                      'green': {'h': [45, 75], 's': [120, 255], 'v': [80, 255], 'samples': 3}},
           'ruler': {'p0': [320, 240], 'p1': [420, 240], 'd_mm': 60.0, 'p2': None}}
    with open(os.path.join(home, '.scara_camera.json'), 'w') as fh:
        json.dump(cam, fh)
    # the taught poses (as on the real robot: jogged there, then saved)
    s_pick = 0.0
    yaw = math.atan2(S.BELT_DIR[1], S.BELT_DIR[0]) + math.pi / 2
    pose = lambda x, y, z, yw=None: dict(zip(S.ARM_JOINTS, K.arm_pose(x, y, z, yaw=yw)))  # noqa: E731
    pick = S.belt_point(s_pick)
    poses = {'pick': pose(*pick, S.Z_PICK, yaw), 'upstream': pose(*S.belt_point(s_pick - 0.08), S.Z_PICK, yaw),
             'ready': pose(*S.READY_XY, S.Z_TRAVEL)}
    for c in S.COLOR_ORDER:
        poses[f'drop_{c}'] = pose(*S.bin_xy(c), S.Z_PLACE, S.bin_yaw(c))
    with open(os.path.join(home, '.scara_hw_cell.json'), 'w') as fh:
        json.dump({'poses': poses}, fh)
    up = S.belt_point(s_pick - 0.08)
    d = math.hypot(up[0] - pick[0], up[1] - pick[1])
    return pick, ((up[0] - pick[0]) / d, (up[1] - pick[1]) / d)


async def browser(port, shots):
    from playwright.async_api import async_playwright
    async with async_playwright() as p:
        b = await p.chromium.launch(args=['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'])
        pg = await (await b.new_context(viewport={'width': 1280, 'height': 860})).new_page()
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        await pg.goto(f'http://127.0.0.1:{port}/')
        await pg.wait_for_timeout(3000)
        if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
            await pg.click('#closeHelp')
        st = await pg.evaluate("({link: window.scaraDebug.app.link, n: window.scaraDebug.app.pucks.length, "
                               "cubes: window.scaraDebug.app.pucks.filter(p => p.cube).length, "
                               "label: document.getElementById('simLabel').textContent})")
        check(st['link'] == 'twin' and st['cubes'] == 2, f'app connected by itself and draws the 2 cubes {st}')
        await pg.click('#vTop')
        await pg.wait_for_timeout(800)
        await pg.screenshot(path=os.path.join(shots, 'hw_twin_top.png'))
        await pg.click('#vHome')
        await pg.wait_for_timeout(800)
        await pg.screenshot(path=os.path.join(shots, 'hw_twin.png'))
        # FK / IK tab against the twin server (software robot): home it first, like the real one
        await pg.click('#tabs [data-tab=robot]')
        await pg.click('#rHome')
        await pg.wait_for_timeout(4000)
        await pg.click('#tabs [data-tab=kin]')
        await pg.wait_for_timeout(600)
        x, y = S.READY_XY
        await pg.click('#kFrame [data-f=world]')
        await pg.evaluate(f"window.scaraDebug.setKinInputs({x}, {y}, 0.12, 0)")
        await pg.wait_for_timeout(300)
        go = await pg.query_selector('#kSol button.primary')
        check(go is not None and await go.is_enabled(), 'IK: a solution with an enabled Go button')
        if go:
            await go.click()
            await pg.wait_for_timeout(6000)
            f = await pg.evaluate("window.scaraDebug.fkFull(window.scaraDebug.app.q)")
            err = math.hypot(f['xw'] - x, f['yw'] - y)
            check(err < 0.004 and abs(f['h'] - 0.12) < 0.004, f'IK Go: the robot reached the point ({1000 * err:.1f} mm, h {1000 * f["h"]:.1f} mm)')
        await pg.screenshot(path=os.path.join(shots, 'hw_twin_kin.png'))
        check(not errs, f'no page errors {errs}')
        await b.close()


def main(a):
    home = tempfile.mkdtemp(prefix='scara_hw_twin_')
    pick, u = setup(home)
    # bug 33: a new HOME hides the packages installed with "pip install --user" (mujoco, cv2 ...),
    # so keep pointing Python at the real user folder
    import site
    env = dict(os.environ, HOME=home, PYTHONUSERBASE=site.getuserbase())
    http, app = 8291, 9291
    proc = subprocess.Popen([sys.executable, os.path.join(ROOT, 'server', 'scara_twin_server.py'), '--sim', 'hardware',
                             '--fake', '--no-browser', '--http', str(http), '--app-port', str(app)],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        st = None
        for _ in range(60):
            time.sleep(0.5)
            try:
                st = get(f'http://127.0.0.1:{http}/api/state')
                if st['sim'].get('pucks'):
                    break
            except Exception:
                pass
        check(st is not None, 'server up')
        sim = st['sim'] if st else {}
        lay = sim.get('layout') or {}
        check(lay.get('pick') and math.hypot(lay['pick'][0] - pick[0], lay['pick'][1] - pick[1]) < 0.003,
              f'belt pick point from the taught pose {lay.get("pick")}')
        check(lay.get('dir') and abs(lay['dir'][0] * u[0] + lay['dir'][1] * u[1] - 1) < 1e-3, 'belt direction from pick -> upstream')
        cubes = {p['c']: p for p in (sim.get('pucks') or []) if p.get('cube')}
        for c, s_mm in (('red', 60.0), ('green', -30.0)):
            want = (pick[0] + u[0] * s_mm / 1000, pick[1] + u[1] * s_mm / 1000)
            p = cubes.get(c)
            err = math.hypot(p['x'] - want[0], p['y'] - want[1]) if p else None
            check(p is not None and err < 0.003, f'{c} cube drawn {s_mm:+.0f} mm along the belt (error {err if err is None else round(1000 * err, 1)} mm)')
        asyncio.run(browser(http, a.shots))
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(home, ignore_errors=True)
    print('ALL HW TWIN CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--shots', default=tempfile.gettempdir())
    sys.exit(main(ap.parse_args()))
