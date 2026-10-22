#!/usr/bin/env python3
"""Clicks through the phone app in a headless browser (offline mode): XY drive, tilt, gripper slider,
teach & replay, command box, sorting cell. Needs: pip install playwright && playwright install chromium."""
import asyncio, json
from playwright.async_api import async_playwright
import os
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
        print('XY drive moved gripper by', round(after['x']-before['x'],3), round(after['y']-before['y'],3), 'yaw change', round(after['yaw']-before['yaw'],3))
        # tilt: enable and fake an orientation event
        await pg.click('#tilt'); 
        await pg.evaluate("window.dispatchEvent(Object.assign(new Event('deviceorientation'), {beta: 30, gamma: 0}))")
        await pg.evaluate("window.dispatchEvent(Object.assign(new Event('deviceorientation'), {beta: 30, gamma: 20}))")
        b2 = await pg.evaluate("(() => { const S = window.scaraDebug.app.sim; return S.fk(S.q); })()")
        await pg.wait_for_timeout(2000)
        a2 = await pg.evaluate("(() => { const S = window.scaraDebug.app.sim; return S.fk(S.q); })()")
        print('tilt moved gripper by', round(a2['x']-b2['x'],3), round(a2['y']-b2['y'],3))
        await pg.click('#tilt')
        # gripper slider
        await pg.evaluate("(() => { const g = document.getElementById('gripFine'); g.value = 50; g.dispatchEvent(new Event('input')); })()")
        await pg.wait_for_timeout(1500)
        print('grip after 50%', await pg.evaluate("window.scaraDebug.app.sim.q.left_finger_joint.toFixed(4)"))
        await pg.screenshot(path='/tmp/scara_app_a_manual.png')
        # teach offline
        await pg.click('#tabs [data-tab=teach]'); await pg.click('#tSave')
        await pg.evaluate("window.scaraDebug.cmd('jog_by', {deltas: {column_joint: 0.5}})"); await pg.wait_for_timeout(1500)
        await pg.click('#tSave'); await pg.click('#tPlay'); await pg.wait_for_timeout(4000)
        print('teach list', await pg.evaluate("document.getElementById('tList').children.length"), await pg.evaluate("document.getElementById('tNote').textContent"))
        await pg.click('#tClear') if False else None
        # command box
        await pg.click('#tabs [data-tab=cell]'); await pg.click('#world [data-w=command]')
        await pg.fill('#cmdText', 'red then blue then green'); await pg.press('#cmdText', 'Enter')
        await pg.wait_for_timeout(500)
        print('sequence', await pg.evaluate("JSON.stringify(window.scaraDebug.app.sim.state().sequence)"), 'running', await pg.evaluate("window.scaraDebug.app.sim.running"))
        await pg.click('#addPuck'); await pg.wait_for_timeout(30000)
        print('sorted', await pg.evaluate("JSON.stringify(window.scaraDebug.app.sim.sorted)"))
        await pg.screenshot(path='/tmp/scara_app_a_auto.png')
        await pg.set_viewport_size({'width':915,'height':412}); await pg.wait_for_timeout(800)
        await pg.screenshot(path='/tmp/scara_app_a_land.png')
        print('help shown first time', helpShown, 'errors', errs)
        await b.close()
asyncio.run(main())
