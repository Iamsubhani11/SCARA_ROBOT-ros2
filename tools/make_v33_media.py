#!/usr/bin/env python3
"""The README pictures of the 3.3 features, made from the real app (offline mode) in Chromium:

    docs/media/gesture_mediapipe.png   gesture camera with the MediaPipe skeleton of two hands (phone)
    docs/media/app_path.png            FK / IK tab, a star path being followed (PC)
    docs/media/app_tracking.png        command cell, a puck picked from the moving belt (PC)

    python3 tools/make_v33_media.py
Needs: pip install playwright && playwright install chromium, opencv-python (for the hand video)."""
import asyncio
import os
import subprocess
import sys
import tempfile

from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URL = 'file://' + os.path.join(ROOT, 'app', 'ui', 'index.html')
OUT = os.path.join(ROOT, 'docs', 'media')
ARGS = ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--allow-file-access-from-files']
D = 'window.scaraDebug'


async def start(b, w, h):
    ctx = await b.new_context(viewport={'width': w, 'height': h}, device_scale_factor=2 if w < 600 else 1)
    await ctx.grant_permissions(['camera'])
    pg = await ctx.new_page()
    await pg.goto(URL)
    await pg.wait_for_timeout(1500)
    if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
        await pg.click('#closeHelp')
    await pg.evaluate("document.getElementById('demo').click()")
    await pg.wait_for_timeout(500)
    return ctx, pg


async def until(pg, js, secs):
    for _ in range(int(secs * 4)):
        if await pg.evaluate(js):
            return True
        await pg.wait_for_timeout(250)
    return False


async def main():
    os.makedirs(OUT, exist_ok=True)
    vid = os.path.join(tempfile.mkdtemp(), 'hands.y4m')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'make_hand_video.py'), '--real', vid], check=True, capture_output=True)
    async with async_playwright() as p:
        b = await p.chromium.launch(args=ARGS + ['--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream',
                                                 '--use-file-for-fake-video-capture=' + vid])
        # 1. gesture camera, two hands
        ctx, pg = await start(b, 412, 915)
        await pg.click('#hand')
        ok = await until(pg, "window.ScaraHand.engine === 'mediapipe' && window.ScaraHand.out.hands === 2", 40)
        await pg.wait_for_timeout(400)
        await pg.screenshot(path=os.path.join(OUT, 'gesture_mediapipe.png'), clip={'x': 0, 'y': 0, 'width': 412, 'height': 520})
        print('gesture_mediapipe.png', 'two hands' if ok else 'WARNING: two hands not seen')
        await ctx.close()
        # 2. a star path, half way
        ctx, pg = await start(b, 1280, 800)
        await pg.evaluate("document.querySelector('#tabs [data-tab=kin]').click()")
        await pg.select_option('#pShape', 'star')
        await pg.evaluate("document.getElementById('kTrail').checked = true; document.getElementById('kTrail').dispatchEvent(new Event('change'))")
        await pg.click('#pRun')
        await until(pg, f'{D}.path.run && {D}.path.run.i > {D}.path.pts.length * 0.6', 90)
        await pg.screenshot(path=os.path.join(OUT, 'app_path.png'))
        print('app_path.png')
        await ctx.close()
        # 3. belt tracking: one red puck, picked on the move
        ctx, pg = await start(b, 1280, 800)
        await pg.evaluate("document.querySelector('#tabs [data-tab=cell]').click()")
        await pg.click('#world [data-w=command]')
        await pg.wait_for_timeout(800)
        await pg.click('#beltTrack')
        await pg.fill('#cmdText', 'red')
        await pg.press('#cmdText', 'Enter')
        await pg.click('#beltRun')
        await pg.wait_for_timeout(500)
        await pg.evaluate(f"""(() => {{ const S = {D}.app.sim, [x, y] = S.beltPoint(-0.45), [x2, y2] = S.beltPoint(-0.7);
            S.pucks = [{{ c: 'red', x, y, z: S.sc.belt_top, id: 998 }}, {{ c: 'blue', x: x2, y: y2, z: S.sc.belt_top, id: 999 }}];
            S.nextSpawn = S.t + 1e9; }})()""")
        await until(pg, f"{D}.app.sim.auto && {D}.app.sim.auto.phase === 'track' && {D}.app.sim.t - {D}.app.sim.auto.t0 > 1.1", 60)
        await pg.screenshot(path=os.path.join(OUT, 'app_tracking.png'))
        print('app_tracking.png')
        await ctx.close()
        await b.close()


if __name__ == '__main__':
    asyncio.run(main())
