#!/usr/bin/env python3
"""Gesture camera layout (Docked / Window / Small / closed) and external gamepads, on phone, tablet and PC sizes.
Fails if the camera covers or shrinks the robot view, leaves the window, cuts its own buttons, or if a
USB / Bluetooth gamepad (e.g. Zebronics) does not drive the arm and the gripper.   python3 tools/test_camera_gamepad.py"""
import asyncio, functools, http.server, os, sys, tempfile, threading
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
from playwright.async_api import async_playwright
SIZES=[('phone',360,760),('phone_land',760,360),('tablet',820,1180),('pc',1440,860)]
async def main():
  vid = os.path.join(tempfile.mkdtemp(), 'hand.y4m')
  import subprocess; subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'make_hand_video.py'), vid], check=True, capture_output=True)
  class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a): pass
  h = functools.partial(Quiet, directory=os.path.join(ROOT, 'app', 'ui'))
  srv = http.server.ThreadingHTTPServer(('127.0.0.1', 0), h); threading.Thread(target=srv.serve_forever, daemon=True).start()
  url = f'http://127.0.0.1:{srv.server_port}/index.html'
  shots = os.environ.get('SHOTS')
  async with async_playwright() as p:
    b=await p.chromium.launch(args=['--use-fake-ui-for-media-stream','--use-fake-device-for-media-stream','--use-file-for-fake-video-capture='+vid])
    bad=[]
    R="""(id)=>{const r=document.getElementById(id).getBoundingClientRect();return [r.left,r.top,r.right,r.bottom]}"""
    inter=lambda a,c: min(a[2],c[2])-max(a[0],c[0])>1 and min(a[3],c[3])-max(a[1],c[1])>1
    for name,w,h in SIZES:
      ctx=await b.new_context(viewport={'width':w,'height':h}, permissions=['camera'], is_mobile=w<800, has_touch=w<800)
      pg=await ctx.new_page(); errs=[]
      pg.on('pageerror', lambda e: errs.append(str(e)))
      await pg.goto(url); await pg.wait_for_timeout(800)
      await pg.evaluate("()=>{document.querySelectorAll('.sheet').forEach(x=>x.hidden=true); document.getElementById('demo').click(); document.querySelectorAll('.sheet').forEach(x=>x.hidden=true);}")
      await pg.wait_for_timeout(500)
      scr0=await pg.evaluate(f"()=>({R})('screen')")
      await pg.evaluate("()=>scaraDebug.handOn()"); await pg.wait_for_timeout(3500)
      for m in ['dock','float','mini']:
        await pg.evaluate(f"()=>scaraDebug.setCamMode('{m}')"); await pg.wait_for_timeout(500)
        r=await pg.evaluate(f"()=>{{const R={R};return {{cam:R('handCam'),scr:R('screen'),view:R('view'),docw:document.documentElement.scrollWidth,bar:[document.getElementById('camBar').scrollWidth,document.getElementById('camBar').clientWidth]}}}}")
        sw0,sh0=scr0[2]-scr0[0],scr0[3]-scr0[1]; sw,sh=r['scr'][2]-r['scr'][0],r['scr'][3]-r['scr'][1]
        if m=='dock' and inter(r['cam'],r['scr']): bad.append(f'{name} dock: camera overlaps the robot view')
        if m=='dock' and (sw<sw0-2 or sh<min(sh0,260)-2): bad.append(f'{name} dock: robot view got smaller {sw0}x{sh0} -> {sw}x{sh}')
        if m!='dock' and (r['cam'][0]<-1 or r['cam'][2]>w+1 or r['cam'][1]<-1 or r['cam'][3]>h+1): bad.append(f'{name} {m}: camera leaves the window {r["cam"]}')
        if m=='mini' and inter(r['cam'],[0,0,w,h]) and not inter(r['cam'],r['scr']): bad.append(f'{name} mini: not on the robot view')
        if r['docw']>w+1: bad.append(f'{name} {m}: page scrolls sideways')
        if r['bar'][0]>r['bar'][1]+1: bad.append(f'{name} {m}: camera title bar cut {r["bar"]}')
        if abs((r['view'][2]-r['view'][0])-sw)>8: bad.append(f'{name} {m}: 3-D canvas does not fill the screen')
        if shots: await pg.screenshot(path=os.path.join(shots, f'camera_{name}_{m}.png'), full_page=True)
        print(name,m,'cam',[round(x) for x in r['cam']],'screen',round(sw0),'x',round(sh0),'->',round(sw),'x',round(sh))
      await pg.evaluate("()=>scaraDebug.setCamMode('dock')"); await pg.wait_for_timeout(300)
      s=await pg.evaluate(f"()=>({R})('camSplit')")
      h0=await pg.evaluate(f"()=>({R})('handCam')[3]-({R})('handCam')[1]")
      await pg.mouse.move((s[0]+s[2])/2,(s[1]+s[3])/2); await pg.mouse.down(); await pg.mouse.move((s[0]+s[2])/2,(s[1]+s[3])/2+50,steps=5); await pg.mouse.up()
      await pg.wait_for_timeout(200)
      h1=await pg.evaluate(f"()=>({R})('handCam')[3]-({R})('handCam')[1]")
      if abs(h1-h0-50)>6: bad.append(f'{name}: size bar drag {h0}->{h1}')
      await pg.click('#camClose'); await pg.wait_for_timeout(400)
      off=await pg.evaluate(f"()=>[document.getElementById('handCam').hidden, scaraDebug.app.hand, ({R})('screen')]")
      if not off[0] or off[1] or abs(off[2][2]-off[2][0]-(scr0[2]-scr0[0]))>2 or abs(off[2][3]-off[2][1]-(scr0[3]-scr0[1]))>2: bad.append(f'{name}: close did not restore the page {off} {scr0}')
      # bug 32: a device that only looks like a gamepad (triggers resting at -1, nothing pressed) must not move the arm
      await pg.evaluate("""()=>{window.__ph={id:'phantom-input (Vendor: 0000 Product: 0000)',connected:true,mapping:'',index:1,axes:[0,0,-1,0,0,-1],buttons:Array.from({length:11},()=>({pressed:false,value:0}))};
        navigator.getGamepads=()=>[null,window.__ph]; window.dispatchEvent(Object.assign(new Event('gamepadconnected'),{gamepad:window.__ph}));}""")
      await pg.wait_for_timeout(300)
      qa=await pg.evaluate("()=>JSON.stringify(['column_joint','shoulder_joint','forearm_joint','wrist_joint'].map(k=>scaraDebug.app.q[k]))")
      await pg.wait_for_timeout(1500)
      qb=await pg.evaluate("()=>JSON.stringify(['column_joint','shoulder_joint','forearm_joint','wrist_joint'].map(k=>scaraDebug.app.q[k]))")
      if qa!=qb: bad.append(f'{name}: a phantom gamepad (triggers at -1) moved the arm {qa} -> {qb}')
      if await pg.evaluate("()=>!document.getElementById('extPad').hidden"): bad.append(f'{name}: a phantom gamepad was announced')
      # an Xbox-style pad without the standard layout: triggers on axes 2 / 5 rest at -1, right stick on 3 / 4
      await pg.evaluate("""()=>{scaraDebug.setDrive('joints'); window.__xb={id:'Xbox Wireless Controller',connected:true,mapping:'',index:2,axes:[0,0,-1,0,0,-1],buttons:Array.from({length:11},()=>({pressed:false,value:0}))};
        navigator.getGamepads=()=>[null,null,window.__xb];}""")
      await pg.wait_for_timeout(200)
      s0=await pg.evaluate("()=>[scaraDebug.app.q.column_joint, scaraDebug.app.q.forearm_joint, scaraDebug.app.q.wrist_joint]")
      await pg.evaluate("()=>{window.__xb.axes[4]=1}"); await pg.wait_for_timeout(900)
      await pg.evaluate("()=>{window.__xb.axes[4]=0}"); await pg.wait_for_timeout(200)
      s1=await pg.evaluate("()=>[scaraDebug.app.q.column_joint, scaraDebug.app.q.forearm_joint, scaraDebug.app.q.wrist_joint]")
      if not s1[2] < s0[2]-0.05: bad.append(f'{name}: Xbox-style right stick down did not turn the wrist {s0}->{s1}')
      if abs(s1[0]-s0[0]) > 1e-6 or abs(s1[1]-s0[1]) > 1e-6: bad.append(f'{name}: Xbox-style right stick moved the column / elbow too {s0}->{s1}')
      await pg.evaluate("""()=>{window.__gp={id:'Zebronics USB Gamepad (Vendor: 0079 Product: 0006)',connected:true,mapping:'',index:0,axes:[0,0,0,0,0,0,0,0,0,1.2857],buttons:Array.from({length:12},()=>({pressed:false,value:0}))};
        navigator.getGamepads=()=>[window.__gp]; window.dispatchEvent(Object.assign(new Event('gamepadconnected'),{gamepad:window.__gp}));}""")
      await pg.wait_for_timeout(200)
      q0=await pg.evaluate("()=>scaraDebug.app.q.column_joint")
      await pg.evaluate("()=>{window.__gp.axes[0]=1}"); await pg.wait_for_timeout(900)
      await pg.evaluate("()=>{window.__gp.axes[0]=0; window.__gp.buttons[0].pressed=true}"); await pg.wait_for_timeout(600)
      st=await pg.evaluate("()=>[scaraDebug.app.q.column_joint, scaraDebug.app.q.left_finger_joint]")
      if abs(st[0]-q0)<0.02: bad.append(f'{name}: gamepad stick did not move the arm')
      if not st[1] < -0.002: bad.append(f'{name}: gamepad A did not close the gripper')
      pv=await pg.evaluate("()=>!document.getElementById('extPad').hidden")
      if not pv: bad.append(f'{name}: gamepad keys not shown in the Manual tab')
      # a keyboard (PC, or plugged into the phone): WASD also while a slider has the focus, arrows too
      await pg.evaluate("()=>{scaraDebug.setDrive('joints'); document.getElementById('speed').focus()}")
      q0=await pg.evaluate("()=>scaraDebug.app.q.column_joint")
      await pg.keyboard.down('a'); await pg.wait_for_timeout(700); await pg.keyboard.up('a'); await pg.wait_for_timeout(300)
      q1=await pg.evaluate("()=>scaraDebug.app.q.column_joint")
      await pg.evaluate("()=>document.activeElement.blur()")
      await pg.keyboard.down('ArrowUp'); await pg.wait_for_timeout(700); await pg.keyboard.up('ArrowUp'); await pg.wait_for_timeout(300)
      z1=await pg.evaluate("()=>scaraDebug.app.q.shoulder_joint")
      kv=await pg.evaluate("()=>!document.getElementById('extKeys').hidden")
      if q1 > q0-0.02: bad.append(f'{name}: key A (slider focused) did not turn the column {q0}->{q1}')
      if z1 < 0.002: bad.append(f'{name}: arrow up did not lift the arm ({z1})')
      if not kv: bad.append(f'{name}: keyboard keys not shown in the Manual tab')
      errs=list(dict.fromkeys(errs))
      if errs: bad.append(f'{name}: page errors {errs}')
      await ctx.close()
    await b.close()
    for x in bad: print('FAIL', x)
    print('ALL CAMERA / GAMEPAD CHECKS PASSED' if not bad else f'{len(bad)} FAILED')
    return 1 if bad else 0
sys.exit(asyncio.run(main()))
