#!/usr/bin/env python3
"""App v3 checks in a headless browser (offline mode, no PC needed):
world switching back and forth (no pucks of the old world left, the cell really changes),
voice commands (the parser and what they do), the exact-position sliders, the keyboard,
the log, and hand control with a fake camera video (--hand-video file.y4m).

    python3 tools/test_app_v3.py [--hand-video hand.y4m] [--shots /tmp]
Needs: pip install playwright && playwright install chromium."""
import argparse
import asyncio
import functools
import http.server
import os
import sys
import threading

from playwright.async_api import async_playwright

UI = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'app', 'ui')
FAIL = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what)
    if not ok:
        FAIL.append(what)


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass


def serve():
    h = functools.partial(Quiet, directory=UI)
    srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), h)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv.server_address[1]


async def main(a):
    port = serve()
    args = ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist',
            '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream']
    if a.hand_video:
        args.append('--use-file-for-fake-video-capture=' + a.hand_video)
    async with async_playwright() as p:
        b = await p.chromium.launch(args=args)
        ctx = await b.new_context(viewport={'width': 412, 'height': 915}, device_scale_factor=2)
        await ctx.grant_permissions(['camera'])
        pg = await ctx.new_page()
        errs = []
        pg.on('pageerror', lambda e: errs.append(str(e)))
        await pg.goto(f'http://127.0.0.1:{port}/index.html')
        await pg.wait_for_timeout(2500)
        if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
            await pg.click('#closeHelp')
        await pg.click('#demo')
        await pg.wait_for_timeout(400)
        S = 'window.scaraDebug.app.sim'

        # ---- worlds: switch around, nothing of the old world may stay
        await pg.click('#tabs [data-tab=cell]')
        seq = ['sort', 'command', 'manual', 'sort', 'manual', 'command', 'sort', 'command', 'manual']
        for w in seq:
            await pg.click(f'#world [data-w={w}]')
            await pg.wait_for_timeout(250)
            if w != 'manual':
                await pg.click('#beltRun')
                await pg.wait_for_timeout(2500)
            st = await pg.evaluate(f"(() => {{ const s = {S}; return {{world: s.world, n: s.pucks.length, "
                                   "onTable: s.pucks.filter(p => Math.abs(p.z - s.sc.table_top) < 1e-3).length, "
                                   "drawn: window.scaraDebug.app.pucks.length, "
                                   "on: [...document.querySelectorAll('#world button.on')].map(b => b.dataset.w)}; })()")
            ok = st['world'] == w and st['on'] == [w] and (st['n'] == 6 if w == 'manual' else st['onTable'] == 0)
            check(ok, f'world {w}: {st}')
        await pg.screenshot(path=os.path.join(a.shots, 'v3_worlds.png'))

        # ---- voice commands (the recogniser is the phone's; here the text goes in directly)
        await pg.evaluate("window.scaraDebug.runSpoken('command mode')")
        await pg.wait_for_timeout(300)
        check(await pg.evaluate(f'{S}.world') == 'command', 'voice: "command mode" switches the world')
        await pg.evaluate("window.scaraDebug.runSpoken('red then blew then grin')")
        await pg.wait_for_timeout(300)
        seqv = await pg.evaluate(f'{S}.state().sequence')
        check(seqv == ['red', 'blue', 'green'], f'voice: "red then blew then grin" -> {seqv}')
        await pg.evaluate("window.scaraDebug.runSpoken('stop picking')")
        check(await pg.evaluate(f'{S}.active.size') == 0, 'voice: "stop picking"')
        await pg.evaluate("window.scaraDebug.runSpoken('table')")
        await pg.wait_for_timeout(300)
        check(await pg.evaluate(f'{S}.world') == 'manual', 'voice: "table"')
        await pg.evaluate("window.scaraDebug.runSpoken('close the gripper')")
        await pg.wait_for_timeout(1500)
        g = await pg.evaluate(f'{S}.q.left_finger_joint')
        check(g < -0.01, f'voice: "close the gripper" -> finger {g:.4f}')
        await pg.evaluate("window.scaraDebug.runSpoken('open')")
        await pg.wait_for_timeout(2500)
        check(await pg.evaluate(f'{S}.q.left_finger_joint') > -0.002, 'voice: "open"')
        await pg.evaluate("window.scaraDebug.runSpoken('emergency stop')")
        check(await pg.evaluate("document.getElementById('estop').classList.contains('latched')"), 'voice: "emergency stop" latches STOP')
        await pg.evaluate("window.scaraDebug.runSpoken('resume')")
        check(not await pg.evaluate("document.getElementById('estop').classList.contains('latched')"), 'voice: "resume" releases it')
        # the mic in a browser without speech: a clear message, no error
        await pg.click('#mic')
        await pg.wait_for_timeout(300)
        t = await pg.evaluate("document.getElementById('toast').textContent")
        check('Voice' in t or 'voice' in t or 'Listening' in t or 'microphone' in t, f'mic button message: {t[:70]}')

        # ---- exact-position sliders
        await pg.click('#tabs [data-tab=teach]')
        await pg.evaluate("(() => { const e = document.querySelector('#sliders [data-j=column_joint]'); e.value = 0.8; "
                          "e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); })()")
        for _ in range(30):
            await pg.wait_for_timeout(250)
            q = await pg.evaluate(f'{S}.q.column_joint')
            if abs(q - 0.8) < 0.02:
                break
        check(abs(q - 0.8) < 0.02, f'slider column -> 0.8 rad (arm at {q:.3f})')
        lbl = await pg.evaluate("document.querySelector('#sliders [data-j=column_joint]').nextElementSibling.textContent")
        check(lbl.startswith('45.'), f'slider label {lbl}')

        # ---- keyboard (joint drive: J / L turn the elbow)
        await pg.click('#tabs [data-tab=manual]')
        await pg.click('#drive [data-d=joints]')
        await pg.click('#tabs [data-tab=teach]')
        before = await pg.evaluate(f'{S}.q.forearm_joint')
        await pg.keyboard.down('l')
        await pg.wait_for_timeout(1500)
        await pg.keyboard.up('l')
        await pg.wait_for_timeout(600)
        after = await pg.evaluate(f'{S}.q.forearm_joint')
        check(after > before + 0.05, f'key L turns the elbow ({before:.3f} -> {after:.3f})')

        # ---- readouts never show "-0"
        await pg.evaluate("window.scaraDebug.cmd('move', {q: {column_joint: -0.00001, shoulder_joint: -0.0000001, forearm_joint: 0, wrist_joint: 0}})")
        await pg.wait_for_timeout(4000)
        txt = await pg.evaluate("[...document.querySelectorAll('.readout b')].map(b => b.textContent)")
        check(not any(x.startswith('-0') and not x.startswith('-0.') or x in ('-0.0°', '-0 mm') for x in txt), f'readouts {txt}')

        # ---- log list + ticker fades
        n = await pg.evaluate("document.getElementById('log').children.length")
        check(n > 3, f'activity log has {n} lines')
        await pg.wait_for_timeout(10000)
        tk = await pg.evaluate("document.getElementById('ticker').children.length")
        check(tk == 0, f'old messages left the 3-D view ({tk} left)')

        # ---- hand control with the fake camera
        await pg.click('#tabs [data-tab=manual]')
        await pg.click('#drive [data-d=joints]')
        await pg.click('#hand')
        await pg.wait_for_timeout(500)
        check(await pg.evaluate("!document.getElementById('handCam').hidden"), 'hand: camera picture shown')
        if a.hand_video:
            c0 = await pg.evaluate(f'{S}.q.column_joint')
            await pg.wait_for_timeout(5000)            # calibration (2 s) then the hand still in the middle
            st = await pg.evaluate("document.getElementById('handText').textContent")
            check('on' in st, f'hand: calibrated ({st})')
            c1 = await pg.evaluate(f'{S}.q.column_joint')
            check(abs(c1 - c0) < 0.05, f'hand in the middle: arm still ({c0:.3f} -> {c1:.3f})')
            await pg.wait_for_timeout(4500)             # the video moves the hand sideways
            c2 = await pg.evaluate(f'{S}.q.column_joint')
            check(abs(c2 - c1) > 0.05, f'hand sideways: column moved ({c1:.3f} -> {c2:.3f})')
            g = 0.0                                     # the video's "fist" (smaller blob) at t = 10..14 s
            for _ in range(40):
                await pg.wait_for_timeout(200)
                g = min(g, await pg.evaluate(f'{S}.target.left_finger_joint'))
                if g < -0.01:
                    break
            check(g < -0.01, f'hand fist closes the gripper (target {g:.4f})')
            await pg.screenshot(path=os.path.join(a.shots, 'v3_hand.png'))
        await pg.click('#vHand')
        check(await pg.evaluate("document.getElementById('handCam').hidden && !document.getElementById('vHand').classList.contains('on')"), 'hand: off again (Gesture button on the 3-D view)')

        # ---- FK / IK tab (offline)
        await pg.click('#tabs [data-tab=kin]')
        await pg.wait_for_timeout(500)
        fk = await pg.evaluate(f"(() => {{ const f = window.scaraDebug.fkFull({S}.q), g = {S}.fk({S}.q); return [f.xw, f.yw, g.x, g.y, document.getElementById('fkX').textContent]; }})()")
        check(abs(fk[0] - fk[2]) < 1e-6 and abs(fk[1] - fk[3]) < 1e-6 and fk[4].endswith('mm'), f'FK matches the simulator {fk}')
        await pg.click('#kFrame [data-f=robot]')
        await pg.evaluate("window.scaraDebug.setKinInputs(0.25, 0.12, 0.14, 0.5)")
        n = await pg.evaluate("document.querySelectorAll('#kSol .sol').length")
        check(n == 2, f'IK: two elbow solutions shown ({n})')
        await pg.click('#kSol button.primary')
        await pg.wait_for_timeout(7000)
        f = await pg.evaluate(f"window.scaraDebug.fkFull({S}.q)")
        check(abs(f['xr'] - 0.25) < 0.003 and abs(f['yr'] - 0.12) < 0.003 and abs(f['h'] - 0.14) < 0.003 and abs(f['yawR'] - 0.5) < 0.02,
              f"IK Go: gripper at {f['xr']:.4f} {f['yr']:.4f} h {f['h']:.4f} yaw {f['yawR']:.3f}")
        await pg.click('#kFrame [data-f=world]')
        xw = await pg.evaluate("[+document.getElementById('kX').value, +document.getElementById('kY').value]")
        check(abs(xw[0] / 1000 - f['xw']) < 0.002 and abs(xw[1] / 1000 - f['yw']) < 0.002, f'frame switch keeps the point {xw}')
        await pg.click('#kFrame [data-f=robot]')
        await pg.evaluate("window.scaraDebug.setKinInputs(0.6, 0, 0.09, 0)")
        t = await pg.evaluate("document.getElementById('kSol').textContent")
        check('Out of reach' in t, f'IK: far point -> {t[:50]}')
        await pg.click('#kTap')
        box = await pg.evaluate("(() => { const r = document.getElementById('view').getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height * 0.6]; })()")
        await pg.mouse.click(box[0], box[1])
        await pg.wait_for_timeout(300)
        t2 = await pg.evaluate("document.getElementById('kX').value")
        check(t2 not in ('600.0', '600') and not await pg.evaluate("document.getElementById('kTap').classList.contains('on')"), f'tap the table fills X ({t2})')
        await pg.screenshot(path=os.path.join(a.shots, 'v3_kin.png'))
        # ---- motion graph, recording -> CSV, gripper trail
        await pg.click('#kTrail')
        await pg.click('#mRec')
        await pg.evaluate("window.scaraDebug.setKinInputs(0.30, -0.05, 0.12, 0)")
        await pg.click('#kSol button.primary')
        await pg.wait_for_timeout(3500)
        await pg.click('#mRec')
        n = await pg.evaluate("window.scaraDebug.MG.last.length")
        tr = await pg.evaluate("window.scaraDebug.trailCount()")
        px = await pg.evaluate("""(() => { const c = document.getElementById('mgraph'), d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
          let k = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 0) k++; return k; })()""")
        async with pg.expect_download() as dl:
            await pg.click('#mSave')
        path = await (await dl.value).path()
        rows = open(path).read().strip().split('\n')
        times = [float(r.split(',')[0]) for r in rows[1:]]
        check(n > 12 and times == sorted(times) and times[-1] > 2.5 and len(rows) == n + 1 and rows[0].startswith('time_s,column_rad') and tr >= 3 and px > 200,
              f'motion: recorded {n} samples -> CSV {len(rows)} lines, trail {tr} points, graph drawn ({px} px)')
        await pg.click('#kTrail')

        # ---- landscape + desktop layout, nothing overlapping
        for vw, vh, name in ((915, 412, 'land'), (1440, 900, 'desk')):
            await pg.set_viewport_size({'width': vw, 'height': vh})
            await pg.wait_for_timeout(700)
            ov = await pg.evaluate("""(() => {
              const r = (id) => document.getElementById(id).getBoundingClientRect();
              const hit = (a, b) => a.left < b.right && b.left < a.right && a.top < b.bottom && b.top < a.bottom;
              const s = r('screen'), p = document.querySelector('.panel').getBoundingClientRect();
              return {screenPanel: hit(s, p), screenReadout: hit(s, r('readout')), w: s.width, h: s.height};
            })()""")
            check(not ov['screenPanel'] and not ov['screenReadout'] and ov['w'] > 300, f'{name} layout {ov}')
            await pg.screenshot(path=os.path.join(a.shots, f'v3_{name}.png'))
        # ---- nothing wider than the phone (320 - 430 px, normal and 115 % system text), every tab
        for vw in (320, 360, 393, 430):
            await pg.set_viewport_size({'width': vw, 'height': 800})
            await pg.wait_for_timeout(300)
            for zoom in (1.0, 1.15):
                await pg.evaluate(f"document.documentElement.style.fontSize = ''; document.body.style.zoom = ''")
                worst = []
                for tab in ('manual', 'cell', 'teach', 'robot', 'kin'):
                    await pg.evaluate(f"document.querySelector('#tabs [data-tab={tab}]').click()")
                    await pg.wait_for_timeout(120)
                    w = await pg.evaluate(f"""(() => {{ const z = {zoom};
                      if (z !== 1) document.querySelectorAll('body *').forEach((e) => {{ if (!e.dataset.fs) e.dataset.fs = parseFloat(getComputedStyle(e).fontSize); e.style.fontSize = (e.dataset.fs * z) + 'px'; }});
                      const W = document.documentElement.clientWidth;
                      const out = [...document.querySelectorAll('body *')].filter((e) => e.offsetParent && e.getBoundingClientRect().right > W + 0.5).map((e) => e.id || e.className || e.tagName);
                      document.querySelectorAll('body *').forEach((e) => {{ e.style.fontSize = ''; }});
                      return out.slice(0, 4); }})()""")
                    if w:
                        worst.append((tab, w))
                check(not worst, f'{vw} px wide, text {int(zoom * 100)} %: nothing cut off {worst}')
                if zoom == 1.0:
                    h1 = await pg.evaluate("(() => { const h = document.querySelector('.brand h1'); return [h.scrollWidth, h.clientWidth]; })()")
                    check(h1[0] <= h1[1] + 1, f'{vw} px: the title is not cut ({h1})')
            fit = await pg.evaluate("""(() => { const v = {r0: '-114.6°', r1: '-150 mm', r2: '-114.6°', r3: '-270.0°', r4: '100 %'}, out = {};
              for (const [id, t] of Object.entries(v)) { const e = document.getElementById(id), old = e.textContent; e.textContent = t;
                out[id] = e.scrollWidth <= e.clientWidth + 0.5; e.textContent = old; } return out; })()""")
            check(all(fit.values()), f'{vw} px: the longest joint readouts fit their boxes {fit}')
        # ---- Manual tab: sliders instead of joysticks
        await pg.set_viewport_size({'width': 360, 'height': 780})
        await pg.evaluate("document.querySelector('#tabs [data-tab=manual]').click()")
        await pg.click('#mmode [data-m=sliders]')
        await pg.evaluate("(() => { const e = document.querySelector('#mSliders [data-j=wrist_joint]'); e.value = -1.0; e.dispatchEvent(new Event('input')); e.dispatchEvent(new Event('change')); })()")
        await pg.wait_for_timeout(3500)
        q = await pg.evaluate(f'{S}.q.wrist_joint')
        vis = await pg.evaluate("[getComputedStyle(document.getElementById('mSliders')).display, getComputedStyle(document.querySelector('.sticks')).display]")
        check(abs(q + 1.0) < 0.02 and vis == ['flex', 'none'], f'Manual sliders mode: wrist -> {q:.3f}, shown {vis}')
        await pg.click('#mmode [data-m=sticks]')
        # ---- dark / light
        t0 = await pg.evaluate("document.documentElement.dataset.theme")
        await pg.click('#theme')
        t1 = await pg.evaluate("[document.documentElement.dataset.theme, getComputedStyle(document.body).backgroundColor]")
        await pg.click('#theme')
        t2 = await pg.evaluate("document.documentElement.dataset.theme")
        check(t0 == 'light' and t1[0] == 'dark' and t1[1] != 'rgb(223, 227, 231)' and t2 == 'light', f'theme switch {t0} -> {t1} -> {t2}')
        # ---- arm camera: pinch smaller, never bigger than full size, double-click = full again
        w0 = await pg.evaluate("document.getElementById('insetFrame').getBoundingClientRect().width")
        cdp = await ctx.new_cdp_session(pg)
        r = await pg.evaluate("(() => { const b = document.getElementById('insetFrame').getBoundingClientRect(); return [b.left + b.width / 2, b.top + b.height / 2]; })()")
        async def pinch(d0, d1):
            cx, cy = await pg.evaluate("(() => { const b = document.getElementById('insetFrame').getBoundingClientRect(); return [b.left + b.width / 2, b.top + b.height / 2]; })()")
            pts = lambda d: [{'x': cx - d / 2, 'y': cy, 'id': 1}, {'x': cx + d / 2, 'y': cy, 'id': 2}]
            await cdp.send('Input.dispatchTouchEvent', {'type': 'touchStart', 'touchPoints': pts(d0)})
            for k in range(1, 6):
                await cdp.send('Input.dispatchTouchEvent', {'type': 'touchMove', 'touchPoints': pts(d0 + (d1 - d0) * k / 5)})
            await cdp.send('Input.dispatchTouchEvent', {'type': 'touchEnd', 'touchPoints': []})
            await pg.wait_for_timeout(200)
            return await pg.evaluate("document.getElementById('insetFrame').getBoundingClientRect().width")
        w1 = await pinch(80, 40)
        w2 = await pinch(30, 160)
        check(w1 < w0 * 0.7 and abs(w2 - w0) < 1.5, f'arm camera pinch: {w0:.0f} -> {w1:.0f} px, spread again -> {w2:.0f} px (never past {w0:.0f})')
        # ---- the arm-camera picture is drawn exactly inside its frame (phone and PC layouts)
        for vw, vh in ((412, 915), (1440, 900)):
            await pg.set_viewport_size({'width': vw, 'height': vh})
            await pg.wait_for_timeout(500)
            al = await pg.evaluate("""(() => { const f = document.getElementById('insetFrame'), cs = getComputedStyle(f), o = window.scaraDebug.insetOff();
              return [parseFloat(cs.top), parseFloat(cs.right), o.top, o.right]; })()""")
            check(abs(al[0] - al[2]) < 0.5 and abs(al[1] - al[3]) < 0.5, f'{vw} px: arm-camera picture aligned with its frame {al}')
        check(not errs, f'no page errors {errs}')
        await b.close()
    print('ALL APP V3 CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--hand-video', default=None)
    ap.add_argument('--shots', default='/tmp')
    asyncio.run(main(ap.parse_args()))
    sys.exit(1 if FAIL else 0)
