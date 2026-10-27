#!/usr/bin/env python3
"""The 3.3 features in the app / website, offline (the phone's own simulator):

  * Draw a path (FK / IK tab): a star is drawn, Run path - the gripper really goes along it
  * a path drawn with the mouse on the table becomes a path; an unreachable one cannot be run
  * Programs (Teach tab): save as program, load it back, export a .json file and import it again
  * Belt tracking (Auto tab, command cell): pucks are picked while the belt keeps running
  * Demo for the review: starts on the table, goes on to the sorting cell, and can be stopped

    python3 tools/test_new_features.py [--shots /tmp]
Needs: pip install playwright && playwright install chromium."""
import argparse
import asyncio
import json
import math
import os
import sys
import tempfile

from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URL = 'file://' + os.path.join(ROOT, 'app', 'ui', 'index.html')
FAIL = []
D = 'window.scaraDebug'


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what, flush=True)
    if not ok:
        FAIL.append(what)


async def until(pg, js, secs, step=250):
    v = None
    for _ in range(int(secs * 1000 / step)):
        v = await pg.evaluate(js)
        if v:
            return v
        await pg.wait_for_timeout(step)
    return v


async def main(a):
    async with async_playwright() as p:
        b = await p.chromium.launch(args=['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist',
                                          '--allow-file-access-from-files'])
        ctx = await b.new_context(viewport={'width': 1280, 'height': 860}, accept_downloads=True)
        pg = await ctx.new_page()
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        await pg.goto(URL)
        await pg.wait_for_timeout(1500)
        if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
            await pg.click('#closeHelp')
        await pg.evaluate("document.getElementById('demo').click()")      # Try offline (in the connect sheet on a PC)
        await pg.wait_for_timeout(500)
        await pg.evaluate("document.getElementById('speed').value = 100; document.getElementById('speed').dispatchEvent(new Event('input'))")

        # ---- draw a path: a star, run it, the gripper goes along it
        await pg.evaluate("document.querySelector('#tabs [data-tab=kin]').click()")
        await pg.select_option('#pShape', 'star')
        n = await pg.evaluate(f'{D}.path.pts.length')
        ok = await pg.evaluate(f'{D}.path.ok.every(Boolean)')
        check(n > 50 and ok and not await pg.is_disabled('#pRun'), f'star path made: {n} points, all reachable')
        await pg.click('#pRun')
        dev, seen = 0.0, 0
        for _ in range(240):                     # while it runs: the pad stays on the path
            await pg.wait_for_timeout(250)
            r = await pg.evaluate(f"""(() => {{ const f = {D}.fkFull({D}.app.q), P = {D}.path.pts;
                let d = 1e9; for (const [x, y] of P) d = Math.min(d, Math.hypot(x - f.xw, y - f.yw));
                return [d, !!{D}.path.run, !!({D}.path.run && {D}.path.run.started)]; }})()""")
            if r[2]:                             # measured once the gripper has reached the start of the path
                dev, seen = max(dev, r[0]), seen + 1
            elif not r[1]:
                break
        await pg.screenshot(path=os.path.join(a.shots, 'v33_path.png'))
        check(seen > 8 and dev < 0.006, f'Run path: the gripper follows the star (largest distance {dev * 1000:.1f} mm, {seen} samples)')
        check(not await pg.evaluate(f'!!{D}.path.run'), 'the path finishes by itself ("Path done")')
        # drawn with the mouse
        await pg.click('#pDraw')
        box = await pg.locator('#view').bounding_box()
        cx, cy = box['x'] + box['width'] * 0.5, box['y'] + box['height'] * 0.62
        await pg.mouse.move(cx, cy)
        await pg.mouse.down()
        for i in range(25):
            await pg.mouse.move(cx + 3 * i, cy + 20 * math.sin(i / 4), steps=2)
        await pg.mouse.up()
        n2 = await pg.evaluate(f'{D}.path.pts.length')
        check(n2 > 5, f'a path drawn with the mouse on the table: {n2} points')
        far = await pg.evaluate(f"""(() => {{ {D}.pathSet([[2, 2], [2.1, 2]]); return [document.getElementById('pRun').disabled,
                                      document.getElementById('pNote').textContent]; }})()""")
        check(far[0] and 'out of reach' in far[1], f'a path out of reach cannot be run ({far[1][:50]}...)')
        await pg.click('#pClear')

        # ---- programs
        await pg.evaluate("document.querySelector('#tabs [data-tab=teach]').click()")
        await pg.click('#tClear')
        for q in ({'column_joint': 0.3}, {'column_joint': -0.4, 'forearm_joint': 0.5}):
            await pg.evaluate(f"{D}.cmd('move', {{ q: {json.dumps(q)} }})")
            await pg.wait_for_timeout(1800)
            await pg.click('#tSave')
        await pg.fill('#prName', 'test_prog')
        await pg.click('#prSave')
        opts = await pg.evaluate("[...document.querySelectorAll('#prList option')].map((o) => o.value)")
        check('test_prog' in opts, f'program saved on this device {opts}')
        saved = await pg.evaluate(f'{D}.app.teachLocal.map((p) => p.column_joint)')
        async with pg.expect_download() as dl:
            await pg.click('#prExport')
        f = await (await dl.value).path()
        data = json.load(open(f))
        check(data.get('scara_program') == 1 and len(data['poses']) == 2, f'Export: a .json file with {len(data.get("poses", []))} poses')
        await pg.click('#tClear')
        await pg.set_input_files('#prFile', f)
        got = await until(pg, f'{D}.app.teachLocal.length === 2 && {D}.app.teachLocal.map((p) => p.column_joint)', 5)
        check(got and all(abs(x - y) < 1e-3 for x, y in zip(got, saved)), f'Import: the same poses come back {got}')
        await pg.click('#tClear')
        await pg.select_option('#prList', 'test_prog')
        await pg.click('#prLoad')
        n3 = await until(pg, f'{D}.app.teachLocal.length', 5)
        check(n3 == 2, f'Load: the saved program is back in the list ({n3} poses)')
        bad = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
        bad.write('{"poses": "nothing"}'); bad.close()
        await pg.set_input_files('#prFile', bad.name)
        t = await until(pg, "(t => /Import/.test(t) && t)(document.getElementById('toast').textContent)", 5)
        check(bool(t) and n3 == 2, f'a broken file is refused with a message ({t})')

        # ---- belt tracking in the offline command cell
        await pg.evaluate("document.querySelector('#tabs [data-tab=cell]').click()")
        await pg.click('#world [data-w=command]')
        await pg.wait_for_timeout(800)
        await pg.click('#beltTrack')
        await pg.fill('#cmdText', 'red')                 # one colour: tracking is for a wanted puck that comes alone
        await pg.press('#cmdText', 'Enter')
        await pg.click('#beltRun')
        await pg.wait_for_timeout(500)
        # the same every time: an empty belt, the feeder paused, ONE red puck coming
        await pg.evaluate(f"""(() => {{ const S = {D}.app.sim, [x, y] = S.beltPoint(-0.45);
            S.pucks = [{{ c: 'red', x, y, z: S.sc.belt_top, id: 999 }}]; S.nextSpawn = S.t + 1e9; }})()""")
        stopped = False
        for _ in range(160):                         # the belt must keep running while it is picked
            await pg.wait_for_timeout(250)
            st = await pg.evaluate(f"(() => {{ const S = {D}.app.sim, p = S.pucks[0]; return [S.tracked, S.belt, S.auto && S.auto.phase, p ? +S.beltS(p.x, p.y).toFixed(3) : null, S.note]; }})()")
            if os.environ.get('DEBUG_TRACK'):
                print('   ', st, flush=True)
            if st[2] in ('track', 'trackGo') and not st[1]:
                stopped = True
            if st[0] >= 1:
                break
        tr = st[0]
        stopped_for_pick = await pg.evaluate(f"{D}.app.sim.auto && {D}.app.sim.auto.phase === 'settle'")
        await pg.screenshot(path=os.path.join(a.shots, 'v33_tracking.png'))
        cell = await pg.evaluate(f"(() => {{ const S = {D}.app.sim, a = S.auto; return [S.world, S.track, S.running, a && a.phase, [...S.active], S.pucks.length]; }})()")
        check(tr >= 1 and not stopped, f'Follow belt: the red puck was picked from the moving belt ({tr}, belt never stopped: {not stopped}; cell {cell})')
        lbl = await pg.evaluate("document.getElementById('beltTrack').textContent")
        check('Following' in lbl and not stopped_for_pick, f'the button shows it ({lbl})')
        await pg.click('#beltTrack')
        check(not await pg.evaluate(f'{D}.app.sim.track'), 'Follow belt off again')

        # ---- demo for the review
        await pg.click('#demoRun')
        s1 = await until(pg, f"{D}.demo.on && {D}.app.sim.world === 'manual' && !!{D}.path.run && {D}.demo.step", 20)
        check(bool(s1) and '1/3' in s1, f'Demo starts on the table and draws ({s1})')
        s2 = await until(pg, f"{D}.app.sim.world === 'sort' && {D}.demo.step", 150)
        check(bool(s2) and '2/3' in s2, f'Demo goes on to the sorting cell ({s2})')
        await pg.screenshot(path=os.path.join(a.shots, 'v33_demo.png'))
        await pg.click('#demoRun')
        check(not await until(pg, f'{D}.demo.on', 2), 'Demo stops when the button is tapped again')
        check(not errs, f'no page errors {errs[:2]}')
        await b.close()
    print('ALL NEW-FEATURE CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--shots', default=tempfile.gettempdir())
    sys.exit(asyncio.run(main(ap.parse_args())))
