#!/usr/bin/env python3
"""Clicks through the phone app in a headless browser (offline mode): XY drive, tilt, gripper slider,
teach & replay, command box, sorting cell. Needs: pip install playwright && playwright install chromium."""
import asyncio, json
from playwright.async_api import async_playwright
import os
FAIL = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what)
    if not ok:
        FAIL.append(what)
URL='file://' + os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'app', 'ui', 'index.html')
async def main():
    async with async_playwright() as p:
        b = await p.chromium.launch(args=['--use-gl=swiftshader','--enable-webgl','--ignore-gpu-blocklist','--allow-file-access-from-files'])
        pg = await b.new_page(viewport={'width':412,'height':915}, device_scale_factor=2, has_touch=False)
        errs=[]; pg.on('pageerror', lambda e: errs.append(str(e)))
        await pg.goto(URL); await pg.wait_for_timeout(2200)
        await pg.screenshot(path='/tmp/scara_app_a_help.png')
        helpShown = await pg.evaluate("!document.getElementById('helpSheet').hidden")
        await pg.click('#closeHelp'); await pg.click('#demo'); await pg.wait_for_timeout(400)
        # XY drive: push left stick up (away from viewer) for 2 s
        await pg.click('#drive [data-d=xy]')
        before = await pg.evaluate("(() => { const S = window.scaraDebug.app.sim; return S.fk(S.q); })()")
        box = await pg.locator('#stickL').bounding_box()
        cx, cy = box['x']+box['width']/2, box['y']+box['height']/2
        await pg.mouse.move(cx, cy); await pg.mouse.down(); await pg.mouse.move(cx, cy - box['height']*0.4, steps=5)
        await pg.wait_for_timeout(2500); await pg.mouse.up(); await pg.wait_for_timeout(1500)
        after = await pg.evaluate("(() => { const S = window.scaraDebug.app.sim; return S.fk(S.q); })()")
        d = abs(after['x']-before['x']) + abs(after['y']-before['y'])
        check(d > 0.02 and abs(after['yaw']-before['yaw']) < 0.05, f'XY drive moved the gripper {d:.3f} m without turning it')
        # tilt: enable and fake an orientation event
        await pg.click('#tilt'); 
        await pg.evaluate("window.dispatchEvent(Object.assign(new Event('deviceorientation'), {beta: 30, gamma: 0}))")
        await pg.evaluate("window.dispatchEvent(Object.assign(new Event('deviceorientation'), {beta: 30, gamma: 20}))")
        b2 = await pg.evaluate("(() => { const S = window.scaraDebug.app.sim; return S.fk(S.q); })()")
        await pg.wait_for_timeout(2000)
        a2 = await pg.evaluate("(() => { const S = window.scaraDebug.app.sim; return S.fk(S.q); })()")
        check(abs(a2['x']-b2['x']) + abs(a2['y']-b2['y']) > 0.005, 'tilting the phone moves the gripper')
        await pg.click('#tilt')
        # gripper slider
        await pg.evaluate("(() => { const g = document.getElementById('gripFine'); g.value = 50; g.dispatchEvent(new Event('input')); })()")
        await pg.wait_for_timeout(1500)
        g = await pg.evaluate("window.scaraDebug.app.sim.q.left_finger_joint")
        check(abs(g + 0.0125) < 0.002, f'gripper slider 50 % -> finger {g:.4f}')
        await pg.screenshot(path='/tmp/scara_app_a_manual.png')
        # teach offline
        await pg.click('#tabs [data-tab=teach]'); await pg.click('#tSave')
        await pg.evaluate("window.scaraDebug.cmd('jog_by', {deltas: {column_joint: 0.5}})"); await pg.wait_for_timeout(1500)
        await pg.click('#tSave'); await pg.click('#tPlay'); await pg.wait_for_timeout(4000)
        check(await pg.evaluate("document.getElementById('tList').children.length") == 2, 'teach: two poses saved and played')
        await pg.click('#tClear') if False else None
        # command box
        await pg.click('#tabs [data-tab=cell]'); await pg.click('#world [data-w=command]')
        await pg.fill('#cmdText', 'red then blue then green'); await pg.press('#cmdText', 'Enter')
        await pg.wait_for_timeout(500)
        seq = await pg.evaluate("window.scaraDebug.app.sim.state().sequence")
        check(seq == ['red', 'blue', 'green'] and await pg.evaluate("window.scaraDebug.app.sim.running"), f'command box: sequence {seq}, belt running')
        await pg.fill('#cmdText', 'any'); await pg.press('#cmdText', 'Enter')
        tot = 0
        for _ in range(60):
            await pg.wait_for_timeout(1500)
            tot = await pg.evaluate("Object.values(window.scaraDebug.app.sim.sorted).reduce((a, b) => a + b, 0)")
            if tot >= 1:
                break
        check(tot >= 1, f'offline command cell picks pucks off the belt ({tot} sorted)')
        n = await pg.evaluate("window.scaraDebug.app.pucks.length")
        check(n >= 1, f'offline cell: pucks drawn ({n})')
        await pg.screenshot(path='/tmp/scara_app_a_auto.png')
        await pg.set_viewport_size({'width':915,'height':412}); await pg.wait_for_timeout(800)
        await pg.screenshot(path='/tmp/scara_app_a_land.png')
        check(helpShown, 'the guide shows on the first start')
        check(not errs, f'no page errors {errs}')
        await b.close()
        return errs
asyncio.run(main())
print('ALL APP CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
raise SystemExit(1 if FAIL else 0)
