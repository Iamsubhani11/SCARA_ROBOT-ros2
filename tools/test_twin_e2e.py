#!/usr/bin/env python3
"""Digital twin end to end, without hardware: the twin server with MuJoCo and the software robot
(the same serial protocol and firmware behaviour as the real Arduino), driven over its HTTP API
and from the app in a browser.

  1. server up with --sim mujoco --fake (MuJoCo table), robot connected
  2. home the robot, Twin mode: a move goes to the simulation AND the robot, both arrive
  3. gripper and conveyor in Twin mode reach the robot too
  4. STOP freezes both, release works
  5. world switch table -> command -> table: the robot stays connected, nothing old left
  6. the app (served by the same server) connects by itself, shows the robot as a ghost and
     moves both with IK Go

    python3 tools/test_twin_e2e.py            (needs MuJoCo, playwright + chromium)"""
import asyncio
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAIL = []
HTTP, APP = 8491, 9491
ARM = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what, flush=True)
    if not ok:
        FAIL.append(what)


def api(path, body=None):
    url = f'http://127.0.0.1:{HTTP}{path}'
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'} if data else {})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def cmd(c, **args):
    return api('/api/cmd', dict(args, cmd=c))


def state():
    return api('/api/state')


def wait(pred, timeout, step=0.3):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            st = state()
            if pred(st):
                return st
        except Exception:
            pass
        time.sleep(step)
    return state()


def close(a, b, tol=(0.02, 0.003, 0.02, 0.03)):
    return a and b and all(abs(a[j] - b[j]) <= t for j, t in zip(ARM, tol))


async def browser():
    from playwright.async_api import async_playwright
    async with async_playwright() as p:
        b = await p.chromium.launch(args=['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'])
        pg = await (await b.new_context(viewport={'width': 1366, 'height': 820})).new_page()
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        await pg.goto(f'http://127.0.0.1:{HTTP}/')
        await pg.wait_for_timeout(3500)
        if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
            await pg.click('#closeHelp')
        st = await pg.evaluate("({link: window.scaraDebug.app.link, ghost: document.getElementById('g0').textContent,"
                               " twin: document.querySelector('#target [data-target=both]').classList.contains('on')})")
        await pg.wait_for_timeout(1500)
        gv = await pg.evaluate("window.scaraDebug.ghostVisible()")
        check(not gv, 'Twin, robot where the sim is: no see-through second arm (it used to flicker)')
        check(st['link'] == 'twin' and st['twin'] and st['ghost'].startswith('robot'),
              f'app: connected by itself, Twin selected, robot shown next to the sim {st}')
        await pg.click('#tabs [data-tab=kin]')
        await pg.click('#kFrame [data-f=robot]')
        await pg.evaluate("window.scaraDebug.setKinInputs(0.24, -0.15, 0.15, -0.4)")
        await pg.wait_for_timeout(300)
        await pg.click('#kSol button.primary')
        await pg.wait_for_timeout(9000)
        s = state()
        sim, real = s['sim']['joints'], s['robot']['joints']
        f = await pg.evaluate("window.scaraDebug.fkFull(window.scaraDebug.app.q)")
        check(abs(f['xr'] - 0.24) < 0.004 and abs(f['yr'] + 0.15) < 0.004 and close(sim, real),
              f'app IK Go in Twin mode: sim at {1000 * f["xr"]:.1f}, {1000 * f["yr"]:.1f} mm and the robot copied it')
        await pg.screenshot(path=os.path.join(os.environ.get('SHOTS', '/tmp'), 'twin_e2e.png'))
        check(not errs, f'app: no page errors {errs}')
        await b.close()


async def browser_pucks():
    from playwright.async_api import async_playwright
    async with async_playwright() as p:
        b = await p.chromium.launch(args=['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist'])
        pg = await (await b.new_context(viewport={'width': 412, 'height': 915})).new_page()
        await pg.goto(f'http://127.0.0.1:{HTTP}/')
        await pg.wait_for_timeout(4000)
        n = await pg.evaluate("window.scaraDebug.app.pucks.length")
        vis = await pg.evaluate("(() => { let k = 0; window.scaraDebug.app.pucks.forEach(() => k++); return k; })()")
        await pg.screenshot(path=os.path.join(os.environ.get('SHOTS', '/tmp'), 'twin_e2e_sorting.png'))
        check(n >= 1 and vis >= 1, f'app on the phone, sorting cell: {n} puck(s) drawn on the belt')
        await b.close()


def main():
    import tempfile
    home = tempfile.mkdtemp(prefix='scara_twin_e2e_')       # the direction check saves here, not in your home
    # bug 33: a new HOME hides the packages installed with "pip install --user" (mujoco, cv2 ...),
    # so keep pointing Python at the real user folder
    import site
    env = dict(os.environ, HOME=home, PYTHONUSERBASE=site.getuserbase())
    log = open(os.path.join(os.environ.get('SHOTS', '/tmp'), 'twin_e2e_server.log'), 'w')
    proc = subprocess.Popen([sys.executable, os.path.join(ROOT, 'server', 'scara_twin_server.py'), '--sim', 'mujoco',
                             '--fake', '--http', str(HTTP), '--app-port', str(APP)], env=env, stdout=log,
                            stderr=subprocess.STDOUT)
    try:
        st = wait(lambda s: s['sim']['joints'] and s['robot']['connected'], 60)
        check(st['sim']['joints'] is not None and st['robot']['connected'], 'server: MuJoCo running, software robot connected')
        # ---- home, then Twin
        cmd('home_switches')
        st = wait(lambda s: s['robot']['zeroed'] and not s['robot']['moving'], 40)
        check(st['robot']['zeroed'], 'robot homed on its (simulated) limit switches')
        r = cmd('connect')
        check(not r['ok'] and 'port' in (r['error'] or ''), f'Connect without a port is refused, never a silent software robot {r}')
        r = cmd('control', mode='both')
        check(r['ok'], f'Twin mode on {r}')
        target = {'column_joint': 0.6, 'shoulder_joint': -0.05, 'forearm_joint': -0.9, 'wrist_joint': 0.7}
        r = cmd('move', q=target)
        check(r['ok'], f'move accepted {r}')
        st = wait(lambda s: close(s['sim']['joints'], target) and close(s['robot']['joints'], target), 30)
        check(close(st['sim']['joints'], target), f'simulation reached the target {st["sim"]["joints"]}')
        check(close(st['robot']['joints'], target), f'robot copied the simulation {st["robot"]["joints"]}')
        # ---- a target past the REAL robot's own limit (column -89 deg): sim and robot both stop there
        cmd('move', q={'column_joint': -1.8})
        st = wait(lambda s: abs(s['sim']['joints']['column_joint'] - math.radians(-89)) < 0.02 and
                  abs(s['robot']['joints']['column_joint'] - math.radians(-89)) < 0.02, 30)
        lim = st['robot'].get('limits') or {}
        check(abs(st['sim']['joints']['column_joint'] - math.radians(-89)) < 0.02 and
              abs(st['robot']['joints']['column_joint'] - math.radians(-89)) < 0.02 and
              abs(lim.get('column_joint', [0])[0] - math.radians(-89)) < 1e-3,
              f"past the robot's limit: sim and robot both stop at -89 deg "
              f"(sim {math.degrees(st['sim']['joints']['column_joint']):.1f}, robot {math.degrees(st['robot']['joints']['column_joint']):.1f})")
        cmd('move', q=target)
        wait(lambda s: close(s['sim']['joints'], target) and close(s['robot']['joints'], target), 30)
        # ---- direction check: nudge moves sim and robot the same way; Opposite flips, saves, nothing moves
        before = state()['robot']['joints']['forearm_joint']
        r = cmd('dir_nudge', joint='forearm_joint')
        st = wait(lambda s: abs(s['robot']['joints']['forearm_joint'] - before) > math.radians(9), 15)
        moved = st['robot']['joints']['forearm_joint'] - before
        check(r['ok'] and abs(abs(moved) - math.radians(10)) < math.radians(1.5) and
              abs(st['sim']['joints']['forearm_joint'] - st['robot']['joints']['forearm_joint']) < 0.02,
              f'direction check: Nudge moved the elbow {math.degrees(moved):+.1f} deg on robot and sim')
        raw = state()['robot']['joints']['forearm_joint']
        r = cmd('dir_flip', joint='forearm_joint')
        time.sleep(1.5)
        st = state()
        saved = open(os.path.join(home, '.scara_twin_hardware.yaml')).read() if os.path.exists(os.path.join(home, '.scara_twin_hardware.yaml')) else ''
        check(r['ok'] and 'j2_sign: -1.0' in saved and abs(st['robot']['joints']['forearm_joint'] + raw) < 0.02
              and abs(st['sim']['joints']['forearm_joint'] - st['robot']['joints']['forearm_joint']) < 0.03,
              f'direction check: Opposite saved j2_sign -1, the reading flipped ({math.degrees(raw):.1f} -> '
              f'{math.degrees(st["robot"]["joints"]["forearm_joint"]):.1f} deg) and the sim followed the robot')
        cmd('dir_flip', joint='forearm_joint')
        time.sleep(1.0)
        cmd('move', q=target)
        wait(lambda s: close(s['sim']['joints'], target) and close(s['robot']['joints'], target), 30)
        # ---- gripper + belt
        cmd('gripper', close=True)
        st = wait(lambda s: s['sim']['joints']['left_finger_joint'] < -0.015, 10)
        check(st['sim']['joints']['left_finger_joint'] < -0.015, 'gripper closed in the simulation')
        cmd('gripper', close=False)
        cmd('conveyor', run=True)
        time.sleep(1.0)
        st = state()
        check(st['robot'].get('conveyor_on') is True, f'conveyor relay on at the robot ({st["robot"].get("conveyor_on")})')
        cmd('conveyor', run=False)
        # ---- 3.3 programs: a whole list of poses at once (Load / Import), clamped to the limits, saved on the PC
        prog = [{'column_joint': 0.2, 'shoulder_joint': -0.02, 'forearm_joint': 0.4, 'wrist_joint': 0.0, 'left_finger_joint': 0.0},
                {'column_joint': 9.0, 'shoulder_joint': -0.03, 'forearm_joint': -0.3, 'wrist_joint': 0.1, 'left_finger_joint': -0.02}]
        r = cmd('teach_set', points=prog)
        tp = state()['teach']['points']
        check(r.get('ok') and len(tp) == 2 and tp[1]['column_joint'] < 3.0, f'program loaded on the PC, out-of-range value clamped ({tp[1]["column_joint"] if len(tp) > 1 else None})')
        bad = cmd('teach_set', points='nothing')
        check(not bad.get('ok'), f'a broken program is refused ({bad.get("error")})')
        cmd('save_program', name='e2e test')
        saved = os.path.join(home, 'scara_programs', 'e2e_test.json')
        check(os.path.exists(saved) and len(json.load(open(saved))['poses']) == 2, f'program saved on the PC ({saved})')
        cmd('teach_clear')
        # ---- STOP
        cmd('move', q={'column_joint': -0.6})
        time.sleep(0.4)
        cmd('estop')
        a = state()
        time.sleep(1.0)                      # the steppers ramp down (an instant stop would skip steps)
        b = state()
        cmd('move', q={'column_joint': 1.2})  # ignored while STOP is latched
        time.sleep(1.5)
        c = state()
        ramp = abs(a['robot']['joints']['column_joint'] - b['robot']['joints']['column_joint'])
        after = abs(b['robot']['joints']['column_joint'] - c['robot']['joints']['column_joint'])
        check(b['safety']['estop'] and ramp < 0.05 and after < 0.002,
              f'STOP: robot ramps down ({math.degrees(ramp):.1f} deg) then stays still, moves refused ({math.degrees(after):.2f} deg)')
        cmd('release')
        check(not state()['safety']['estop'], 'STOP released')
        # ---- watchdog: the robot stops answering (hung Arduino / pulled cable) -> nothing sent, clear alarm,
        #      and when it answers again the twin carries on without a jump
        cmd('fake_freeze', seconds=4)
        st = wait(lambda s: s['robot'].get('answering') is False, 6)
        silent = st['robot'].get('answering') is False
        st = wait(lambda s: 'NOT answering' in ' '.join(e['text'] for e in s.get('log', [])), 3)
        logtxt = ' '.join(e['text'] for e in st.get('log', []))
        check(silent and 'NOT answering' in logtxt, 'watchdog: robot silent -> reported as NOT answering')
        st = wait(lambda s: s['robot'].get('answering') is True, 10)
        check(st['robot'].get('answering') is True, 'watchdog: robot answering again -> twin continues')
        # ---- worlds
        for w in ('sort', 'command', 'manual'):
            r = cmd('world', world=w)
            st = wait(lambda s: s['sim'].get('world') == w and not s['sim'].get('switching') and s['sim']['joints'], 60)
            check(st['sim'].get('world') == w and st['robot']['connected'], f'world {w}: running, robot still connected')
            if w != 'manual':
                # bug 29: the conveyor cells sent no pucks - the arm picked things nobody could see
                cmd('conveyor', run=True)
                st = wait(lambda s: len([p for p in (s['sim'].get('pucks') or []) if p.get('c')]) >= 1, 40)
                pk = [p for p in (st['sim'].get('pucks') or []) if p.get('c')]
                check(len(pk) >= 1, f'world {w}: the pucks on the belt reach the app ({len(pk)}: '
                                    f'{sorted(set(p["c"] for p in pk))})')
            if w == 'command':
                # 3.3: belt tracking reaches the MuJoCo command cell and its state comes back to the app
                r = cmd('belt_tracking', on=True)
                st = wait(lambda s: s['sim'].get('tracking') is True, 10)
                check(r.get('ok') and st['sim'].get('tracking') is True, f'belt tracking on in the MuJoCo command cell ({r})')
                cmd('belt_tracking', on=False)
                st = wait(lambda s: s['sim'].get('tracking') is False, 10)
                check(st['sim'].get('tracking') is False, 'belt tracking off again')
        # the table's pucks are reported a moment after the world has started: wait for them, then
        # there must be exactly 6 (more = something left over from the command cell, bug 4)
        st = wait(lambda s: len(s['sim'].get('pucks') or []) >= 6, 10)
        n = len(st['sim'].get('pucks') or [])
        check(n == 6, f'table world: the 6 table pucks, nothing from the command cell ({n})')
        cmd('control', mode='both')
        asyncio.run(browser())
        # the app draws the conveyor cell's pucks too (sorting world, belt running)
        cmd('world', world='sort')
        wait(lambda s: s['sim'].get('world') == 'sort' and not s['sim'].get('switching') and s['sim']['joints'], 60)
        cmd('conveyor', run=True)
        wait(lambda s: len(s['sim'].get('pucks') or []) >= 1, 40)
        asyncio.run(browser_pucks())
    finally:
        proc.terminate()
        try:
            proc.wait(8)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()
    print('ALL TWIN CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
