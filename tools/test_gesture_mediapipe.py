#!/usr/bin/env python3
"""MediaPipe hand tracking in the app and the website, with a camera video made from real hand photos.

    python3 tools/test_gesture_mediapipe.py [--shots /tmp]

Chrome plays tools/make_hand_video.py --real as the webcam. The test checks that
  * MediaPipe (not the colour tracker) is running, from file:// like the Android app and over http
    like the website - with no internet (everything comes from app/ui/lib/mediapipe)
  * it finds the hand (21 points) and draws the skeleton on the camera picture
  * moving the hand turns the column, a fist closes the gripper, an open hand opens it again
  * a second hand works as the right joystick (the wrist turns)
  * the switch in the camera window changes to the colour tracker
  * a refused camera and a page opened over plain http from another computer say why on screen
Needs: pip install playwright && playwright install chromium, numpy, opencv-python."""
import argparse
import asyncio
import functools
import http.server
import os
import socket
import subprocess
import sys
import tempfile
import threading

from playwright.async_api import async_playwright

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UI = os.path.join(ROOT, 'app', 'ui')
FAIL = []
ARGS = ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--allow-file-access-from-files']
S = 'window.scaraDebug.app.sim'


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what, flush=True)
    if not ok:
        FAIL.append(what)


class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass


def serve(host):
    srv = http.server.ThreadingHTTPServer((host, 0), functools.partial(Quiet, directory=UI))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv.server_address[1]


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))          # no packet is sent; it only picks the network card
        ip = s.getsockname()[0]
        s.close()
        return None if ip.startswith('127.') else ip
    except OSError:
        return None


async def until(pg, js, secs, step=250):
    """wait until the JavaScript expression is true; returns its last value"""
    v = None
    for _ in range(int(secs * 1000 / step)):
        v = await pg.evaluate(js)
        if v:
            return v
        await pg.wait_for_timeout(step)
    return v


async def open_app(ctx, url):
    pg = await ctx.new_page()
    errs = []
    pg.on('pageerror', lambda e: errs.append(str(e)))
    await pg.goto(url)
    await pg.wait_for_timeout(1500)
    if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
        await pg.click('#closeHelp')
    await pg.click('#demo')
    await pg.wait_for_timeout(300)
    return pg, errs


async def main(a):
    vid = os.path.join(tempfile.mkdtemp(), 'hand_real.y4m')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'make_hand_video.py'), '--real', vid], check=True,
                   capture_output=True)
    cam = ['--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream', '--use-file-for-fake-video-capture=' + vid]
    async with async_playwright() as p:
        b = await p.chromium.launch(args=ARGS + cam)
        ctx = await b.new_context(viewport={'width': 412, 'height': 915}, device_scale_factor=2)
        await ctx.grant_permissions(['camera'])
        # ---- like the Android app: the page from a file, no internet at all
        await ctx.route('http*://**/*', lambda r: r.abort())
        pg, errs = await open_app(ctx, 'file://' + os.path.join(UI, 'index.html'))
        await pg.evaluate("document.querySelector('#tabs [data-tab=manual]').click()")
        await pg.click('#drive [data-d=joints]')
        await pg.click('#hand')
        eng = await until(pg, "window.ScaraHand.engine === 'mediapipe' && document.getElementById('handEngine').textContent", 25)
        check(eng == 'MediaPipe hand tracking', f'MediaPipe starts offline from the app files ({eng!r})')
        n = await until(pg, "window.ScaraHand.out.hands >= 1 && window.ScaraHand.overlay().hands[0].lm.length", 15)
        check(n == 21, f'MediaPipe finds the hand: {n} points')
        await pg.wait_for_timeout(300)
        lit = await pg.evaluate("""(() => { const c = document.getElementById('handOverlay'), d = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
            let k = 0; for (let i = 3; i < d.length; i += 4) if (d[i] > 200) k++; return k; })()""")
        check(lit > 500, f'the hand skeleton is drawn on the camera picture ({lit} pixels)')
        # the video: open hand in the middle, moves sideways, fist, open, two hands (it repeats)
        c0 = await pg.evaluate(f'{S}.target.column_joint')
        moved = await until(pg, f'Math.abs({S}.target.column_joint - ({c0})) > 0.05', 30)
        check(bool(moved), 'moving the hand turns the column')
        fist = await until(pg, f'{S}.target.left_finger_joint < -0.01', 30)
        check(bool(fist), 'a fist closes the gripper')
        await pg.screenshot(path=os.path.join(a.shots, 'mediapipe_fist.png'))
        opened = await until(pg, f'{S}.target.left_finger_joint > -0.003', 30)
        check(bool(opened), 'an open hand opens it again')
        w0 = await pg.evaluate(f'{S}.target.wrist_joint')
        two = await until(pg, f"window.ScaraHand.out.hands === 2 && window.ScaraHand.out.seen2 && Math.abs({S}.target.wrist_joint - ({w0})) > 0.05", 30)
        await pg.screenshot(path=os.path.join(a.shots, 'mediapipe_two_hands.png'))
        check(bool(two), 'two hands: the second hand is the right joystick (the wrist turns)')
        # what is DRAWN must match: the left hand's blue palm dot on the left half of the shown picture,
        # and the left stick's circle right under that hand (3.3 first drew the circles swapped)
        cv = await until(pg, """(() => { const c = document.getElementById('handOverlay'), g = c.getContext('2d'), v = window.ScaraHand.overlay();
            if (v.hands.length < 2 || v.circles.length < 2) return null;
            const vid = document.getElementById('handVideo'), W = c.width, H = c.height, k = Math.min(W / vid.videoWidth, H / vid.videoHeight);
            const pw = vid.videoWidth * k, ph = vid.videoHeight * k, ox = (W - pw) / 2, oy = (H - ph) / 2;
            const at = (u, w) => [Math.round(ox + u * pw), Math.round(oy + w * ph)];      // mirrored picture -> canvas
            const count = ([x, y], test) => { const d = g.getImageData(x - 3, y - 3, 7, 7).data; let n = 0;
              for (let i = 0; i < d.length; i += 4) if (test(d[i], d[i + 1], d[i + 2], d[i + 3])) n++; return n; };
            const h = v.hands[0], palm = at(h.c.x, h.c.y), L = v.circles.find((q) => q.tag === 'L'), ctr = at(L.home.x, L.home.y);
            const blue = count(palm, (r, gg, b, a) => a > 200 && b > 200 && r < 120);
            let ring = 0;                       // white / yellow ring pixels at 8 places around the circle (bones may cover some)
            for (let i = 0; i < 8; i++) ring += count([Math.round(ctr[0] + v.deadR * pw * Math.cos(i * Math.PI / 4)),
              Math.round(ctr[1] + v.deadR * pw * Math.sin(i * Math.PI / 4))], (r, gg, b, a) => a > 150 && r > 180 && gg > 150);
            return [palm[0] < W / 2, blue, ring, Math.abs(h.c.x - L.home.x) < 0.12]; })()""", 20, step=100)
        check(cv and cv[0] and cv[1] > 3 and cv[2] > 0 and cv[3],
              f'two hands drawn right: left hand on the left, its circle drawn under it [left, blue px, ring px, near] {cv}')
        await pg.click('#handEng')
        sk = await until(pg, "window.ScaraHand.engine === 'skin' && document.getElementById('handEngine').textContent", 15)
        check(bool(sk) and 'colour' in sk, f'the switch in the camera window starts the colour tracker ({sk!r})')
        await pg.click('#handEng')                    # back to MediaPipe for the next start
        await until(pg, "window.ScaraHand.engine === 'mediapipe'", 15)
        await pg.click('#camClose')
        check(not errs, f'no page errors {errs[:2]}')
        await ctx.close()

        # ---- the website (served over http by the PC)
        port = serve('127.0.0.1')
        ctx = await b.new_context(viewport={'width': 1280, 'height': 800})
        await ctx.grant_permissions(['camera'])
        pg, _ = await open_app(ctx, f'http://127.0.0.1:{port}/index.html')
        await pg.evaluate("document.querySelector('#tabs [data-tab=manual]').click()")
        await pg.evaluate("document.getElementById('hand').click()")
        eng = await until(pg, "window.ScaraHand.engine === 'mediapipe' && window.ScaraHand.out.hands >= 1", 30)
        check(bool(eng), 'website: MediaPipe runs and finds the hand')
        await ctx.close()
        await b.close()

        # ---- camera refused: the reason is shown in the camera window
        b = await p.chromium.launch(args=ARGS + ['--use-fake-device-for-media-stream'])
        ctx = await b.new_context(viewport={'width': 412, 'height': 915})
        # what Chrome / Android do when you tap "Block" (headless Chrome has no permission prompt)
        await ctx.add_init_script("navigator.mediaDevices.getUserMedia = () => Promise.reject(new DOMException('Permission denied', 'NotAllowedError'))")
        pg, _ = await open_app(ctx, f'http://127.0.0.1:{port}/index.html')
        await pg.evaluate("document.getElementById('hand').click()")
        t = await until(pg, "(t => /permission|refused/i.test(t) && t)(document.getElementById('handText').textContent)", 15)
        vis = await pg.evaluate("!document.getElementById('handCam').hidden")
        check(bool(t) and vis, f'camera refused: the window says why ({t!r})')
        await ctx.close()

        # ---- the website opened from another computer over plain http: browsers block the camera
        ip = lan_ip()
        if ip:
            port2 = serve('0.0.0.0')
            ctx = await b.new_context(viewport={'width': 412, 'height': 915})
            pg, _ = await open_app(ctx, f'http://{ip}:{port2}/index.html')
            await pg.evaluate("document.getElementById('hand').click()")
            t = await until(pg, "(t => /localhost/.test(t) && t)(document.getElementById('handText').textContent)", 10)
            check(bool(t), f'plain http from another computer: the window says to use localhost / https / the app ({t!r})')
            await ctx.close()
        else:
            print('(no network card: plain-http check left out)')
        await b.close()
    print('ALL MEDIAPIPE CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--shots', default=tempfile.gettempdir())
    sys.exit(asyncio.run(main(ap.parse_args())))
