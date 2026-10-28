#!/usr/bin/env python3
"""Gazebo end to end through the twin server, with the software robot - the Gazebo bugs we fixed
must stay fixed (docs/FIXED_BUGS.md):

  * the server starts Gazebo itself, headless (no screen: the vision node must not crash)
  * command cell + Twin: pucks get picked and the robot follows the Gazebo arm
  * world switches really happen (they used to stop working), nothing of the old world stays
  * stopping the server stops Gazebo

Needs ROS 2 + Gazebo Harmonic and this package built (colcon build) and sourced:
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 tools/test_gazebo.py            # about 8 minutes"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import urllib.request

FAIL = []
HTTP, APP = 8691, 9691


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what, flush=True)
    if not ok:
        FAIL.append(what)


def api(path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f'http://127.0.0.1:{HTTP}{path}', data=data,
                                 headers={'Content-Type': 'application/json'} if data else {})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def cmd(c, **a):
    return api('/api/cmd', dict(a, cmd=c))


def wait(pred, timeout, step=2.0):
    t0, st = time.time(), None
    while time.time() - t0 < timeout:
        try:
            st = api('/api/state')
            if pred(st):
                return st
        except Exception:
            pass
        time.sleep(step)
    return st or {}


def main():
    if not shutil.which('ros2'):
        print('SKIP: ros2 not found - source ROS 2 and this workspace first')
        return 0
    exe = shutil.which('scara_twin_server.py') or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                                                'server', 'scara_twin_server.py')
    log = open('/tmp/scara_test_gazebo_server.log', 'w')
    proc = subprocess.Popen([sys.executable, exe, '--sim', 'gazebo', '--world', 'command', '--headless', '--fake',
                             '--http', str(HTTP), '--app-port', str(APP)], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    try:
        st = wait(lambda s: s['sim'].get('world') == 'command' and not s['sim'].get('switching') and s['sim'].get('joints'), 180)
        check(st.get('sim', {}).get('joints') is not None, 'server started Gazebo (command world, headless)')
        cmd('home_switches')
        wait(lambda s: s['robot']['zeroed'] and not s['robot']['moving'], 60)
        check(cmd('control', mode='both')['ok'], 'Twin mode with the software robot')
        cmd('pick', color='any')
        st = wait(lambda s: sum((s['sim'].get('sorted') or {}).values()) >= 1, 420, step=5)
        n = sum((st['sim'].get('sorted') or {}).values())
        check(n >= 1, f'Gazebo picked {n} puck(s) - vision works without a screen')
        sj, rj = st['sim']['joints'], st['robot']['joints']
        diff = max(abs(sj[k] - rj[k]) for k in ('column_joint', 'forearm_joint', 'wrist_joint'))
        check(diff < 0.25, f'the robot follows the Gazebo arm (largest difference {diff:.3f} rad, mid-move allowed)')
        # 3.3 belt tracking is switched off in Gazebo (tested: its controller lag changes from pick to
        # pick, 1 of 6 held); asking for it must say so and leave the sorter stopping the belt as before
        r = cmd('belt_tracking', on=True)
        st = api('/api/state')
        check(not r.get('ok') and 'Gazebo' in (r.get('error') or '') and not st['sim'].get('tracking'),
              f'Gazebo: belt tracking refused with a reason, the belt still stops for picks ({(r.get("error") or "")[:60]}...)')
        for w in ('manual', 'sort'):
            cmd('world', world=w)
            st = wait(lambda s: s['sim'].get('world') == w and not s['sim'].get('switching') and s['sim'].get('joints'), 240)
            pk = st['sim'].get('pucks') or []
            check(st['sim'].get('world') == w, f'world switch to {w} happened')
            if w == 'manual':
                time.sleep(4)
                pk = api('/api/state')['sim'].get('pucks') or []
                check(len(pk) == 6, f'table world: exactly its 6 pucks, nothing left from the command cell ({len(pk)})')
            else:
                on_table = [p for p in pk if abs(p['z'] - 0.035) < 0.01]
                check(not on_table, f'sorting world: no table pucks left over ({len(on_table)})')
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(40)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()
    time.sleep(5)
    left = subprocess.run(['pgrep', '-f', 'gz sim|gazebo.launch.py'], capture_output=True, text=True).stdout.split()
    check(not left, f'stopping the server stopped Gazebo ({len(left)} processes left)')
    print('ALL GAZEBO CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
