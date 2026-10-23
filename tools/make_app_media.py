#!/usr/bin/env python3
"""Pictures and GIFs of the app / GUI for the README (docs/media), made in a headless browser
with the offline simulator - nothing else has to run.

    python3 tools/make_app_media.py [--out docs/media]
Needs: playwright + chromium, Pillow."""
import argparse
import asyncio
import io
import os
import sys

from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_app_v3 import serve      # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARGS = ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist']


# The page's clock is ours while recording: every GIF frame is exactly `every_ms` of robot time,
# however slow the (software-rendered) screenshots are.
CLOCK = """(() => {
  const real = performance.now.bind(performance); let off = 0, frozen = null;
  window.__clock = { freeze() { frozen = real() + off; }, step(ms) { frozen += ms; },
                     thaw() { off = frozen - real(); frozen = null; } };
  performance.now = () => (frozen === null ? real() + off : frozen);
})();"""
TWO_FRAMES = "new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)))"


async def frames(pg, clip, seconds, every_ms, scale, before=None):
    out = []
    n = int(seconds * 1000 / every_ms)
    await pg.evaluate('window.__clock.freeze()')
    for i in range(n):
        if before and i == before[0]:
            await before[1]()
        await pg.evaluate(f'window.__clock.step({every_ms})')
        await pg.evaluate(TWO_FRAMES)
        png = await pg.screenshot(clip=clip)
        im = Image.open(io.BytesIO(png)).convert('RGB')
        out.append(im.resize((int(im.width * scale), int(im.height * scale)), Image.LANCZOS))
    await pg.evaluate('window.__clock.thaw()')
    return out


def save_gif(ims, path, ms):
    pal = [im.quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for im in ims]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=ms, loop=0, optimize=True)
    print('wrote', path, f'{os.path.getsize(path) / 1e6:.1f} MB')


async def start(p, w, h, dpr=1):
    b = await p.chromium.launch(args=ARGS)
    ctx = await b.new_context(viewport={'width': w, 'height': h}, device_scale_factor=dpr)
    await ctx.add_init_script(CLOCK)
    pg = await ctx.new_page()
    await pg.goto(f'http://127.0.0.1:{PORT}/index.html')
    await pg.wait_for_timeout(2500)
    if await pg.evaluate("!document.getElementById('helpSheet').hidden"):
        await pg.click('#closeHelp')
    await pg.click('#demo')
    await pg.wait_for_timeout(400)
    return b, pg


async def main(out):
    from playwright.async_api import async_playwright
    async with async_playwright() as p:
        # 1. desktop GUI: the sorting cell running by itself
        b, pg = await start(p, 1280, 800)
        await pg.click('#tabs [data-tab=cell]')
        await pg.click('#world [data-w=sort]')
        await pg.wait_for_timeout(300)
        await pg.click('#beltRun')
        await pg.wait_for_timeout(4000)
        await pg.screenshot(path=os.path.join(out, 'gui_sorting.png'))
        r = await pg.evaluate("(() => { const r = document.getElementById('screen').getBoundingClientRect(); return [r.x, r.y, r.width, r.height]; })()")
        clip = {'x': r[0], 'y': r[1], 'width': r[2], 'height': r[3]}
        save_gif(await frames(pg, clip, 12, 150, 0.5), os.path.join(out, 'app_sorting.gif'), 150)
        # 2. FK / IK: type a point, Go, the arm moves there
        await pg.click('#beltStop')
        await pg.click('#world [data-w=manual]')
        await pg.click('#tabs [data-tab=kin]')
        await pg.click('#kFrame [data-f=robot]')
        await pg.wait_for_timeout(500)
        await pg.evaluate("window.scaraDebug.setKinInputs(0.27, -0.12, 0.13, -0.6)")
        await pg.wait_for_timeout(200)
        go = pg.locator('#kSol button.primary')
        ims = await frames(pg, clip, 6, 120, 0.62, before=(4, go.click))
        save_gif(ims, os.path.join(out, 'app_ik.gif'), 120)
        await pg.screenshot(path=os.path.join(out, 'gui_fk_ik.png'))
        await pg.click('#theme')                       # the PC page in light mode too
        await pg.wait_for_timeout(3700)                # (after the "Light mode" message has gone)
        await pg.screenshot(path=os.path.join(out, 'gui_light.png'))
        await pg.click('#theme')
        await b.close()
        # 3. phone, portrait: command mode + voice / hand buttons
        b, pg = await start(p, 412, 915, 2)
        await pg.click('#tabs [data-tab=cell]')
        await pg.click('#world [data-w=command]')
        await pg.wait_for_timeout(300)
        await pg.evaluate("window.scaraDebug.runSpoken('red then blue then green')")
        await pg.wait_for_timeout(9000)
        await pg.screenshot(path=os.path.join(out, 'phone_app.png'))
        await pg.click('#tabs [data-tab=kin]')
        await pg.wait_for_timeout(600)
        await pg.screenshot(path=os.path.join(out, 'phone_fk_ik.png'))
        await pg.click('#tabs [data-tab=manual]')
        await pg.click('#theme')                       # and the phone in dark mode
        await pg.wait_for_timeout(3700)
        await pg.screenshot(path=os.path.join(out, 'phone_dark.png'))
        await pg.click('#theme')
        await b.close()
        # 4. phone, landscape: manual table
        await landscape(p, out)


async def landscape(p, out):
    b, pg = await start(p, 915, 412, 2)
    await pg.click('#tabs [data-tab=manual]')
    await pg.wait_for_timeout(1500)
    await pg.screenshot(path=os.path.join(out, 'phone_app_landscape.png'))
    await b.close()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(ROOT, 'docs', 'media'))
    ap.add_argument('--only-landscape', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    PORT = serve()
    if a.only_landscape:
        from playwright.async_api import async_playwright

        async def one():
            async with async_playwright() as p:
                await landscape(p, a.out)
        asyncio.run(one())
    else:
        asyncio.run(main(a.out))
