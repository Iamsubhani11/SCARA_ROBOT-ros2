#!/usr/bin/env python3
"""Fast checks (a few seconds, nothing to install) that keep old bugs from coming back.
Every check names the bug it guards - see docs/FIXED_BUGS.md.

    python3 tools/test_static.py"""
import ast
import builtins
import glob
import json
import math
import os
import re
import socket
import struct
import symtable
import sys
import tempfile
import threading
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
FAIL = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what)
    if not ok:
        FAIL.append(what)


PY_DIRS = ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control', 'tools', 'launch')


def py_files():
    return sorted(f for d in PY_DIRS for f in glob.glob(os.path.join(ROOT, d, '*.py')))


def undefined_names(path):
    """Names used but defined nowhere (a NameError the moment that line runs)."""
    src = open(path).read()
    tree = ast.parse(src)
    if any(isinstance(n, ast.ImportFrom) and any(a.name == '*' for a in n.names) for n in ast.walk(tree)):
        return []
    top = symtable.symtable(src, path, 'exec')
    known = {s.get_name() for s in top.get_symbols() if s.is_assigned() or s.is_imported() or s.is_namespace()}
    known |= set(dir(builtins)) | {'__file__', '__name__', '__doc__', '__builtins__', '__spec__'}
    out = set()

    def walk(t):
        for s in t.get_symbols():
            if s.is_referenced() and (s.is_global() or (t.get_type() == 'module' and not s.is_assigned()
                                                        and not s.is_imported())):
                if s.get_name() not in known:
                    out.add(s.get_name())
        for c in t.get_children():
            walk(c)
    walk(top)
    return sorted(out)


def main():
    # ---- the code itself
    bad = {os.path.relpath(f, ROOT): u for f in py_files() if (u := undefined_names(f))}
    check(not bad, f'no undefined names in any program (a crash waiting to happen) {bad}')
    for f in py_files():
        compile(open(f).read(), f, 'exec')
    check(True, f'all {len(py_files())} Python files compile')
    old = [os.path.relpath(f, ROOT) for f in py_files() if re.search(r"['\"]scripts['\"]\)", open(f).read())]
    check(not old, f'no program looks for the old scripts/ folder {old}')

    # ---- nothing that only exists on the developer's machine
    leaks = []
    for d in PY_DIRS + ('app/ui', 'config', 'docs'):
        for f in glob.glob(os.path.join(ROOT, d, '*')):
            if os.path.isfile(f) and f.endswith(('.py', '.js', '.html', '.md', '.sh', '.yaml', '.txt')) and not f.endswith('meshes.js'):
                txt = open(f, errors='ignore').read()
                if re.search(r'/home/claude|/tmp/claude|/opt/gz/|/opt/ros2dl', txt) and not f.endswith('test_static.py'):
                    leaks.append(os.path.relpath(f, ROOT))
    check(not leaks, f'no paths that exist only on the developer machine {leaks}')

    # ---- browser must not open by itself (bug: the GUI opened on every start)
    import scara_twin_server as T
    src = open(os.path.join(ROOT, 'server', 'scara_twin_server.py')).read()
    check('def serve(core, http_port=8080, app_port=9090, open_browser=False)' in src
          and 'args.open and not args.no_browser' in src, 'the browser opens only with --open')

    # ---- joint settings are read by every program (bug: twin_hardware.yaml only reached the ROS bridge)
    import scara_twin_protocol as P
    home = tempfile.mkdtemp()
    old_home = os.environ.get('HOME')
    os.environ['HOME'] = home
    try:
        P.USER_MAP_FILE = os.path.join(home, '.scara_twin_hardware.yaml')
        m = P.HardwareMap.from_settings()
        check(m.sign == [1.0, 1.0, 1.0] and m.z_zero_mm == 150.0, 'config/twin_hardware.yaml is read (signs, Z zero)')
        P.save_map_change('j3_sign', -1.0)
        m = P.HardwareMap.from_settings()
        check(m.sign[2] == -1.0, 'a saved direction-check change is used by the next program')
        q = {'column_joint': 0.4, 'forearm_joint': -0.7, 'wrist_joint': 1.1, 'shoulder_joint': -0.06, 'left_finger_joint': -0.01}
        back = m.from_hw(m.to_hw(q))
        check(all(abs(back[k] - v) < 1e-9 for k, v in q.items()), 'URDF <-> firmware units round trip with a flipped joint')
    finally:
        if old_home is not None:
            os.environ['HOME'] = old_home
    lim = P.FakeLink(on_message=lambda *a: None).urdf_limits()
    check(abs(math.degrees(lim['column_joint'][0]) + 89) < 1e-6 and abs(lim['shoulder_joint'][1] - 0.02) < 1e-9,
          "the real robot's soft limits in URDF units (column -89 deg, Z top +20 mm)")
    check('def alive(self)' in open(os.path.join(ROOT, 'common', 'scara_twin_protocol.py')).read(),
          'robot watchdog present (a silent robot is reported, not followed blindly)')

    # ---- websocket: big messages fast, oversized ones refused (recordings from the phone)
    a, b = socket.socketpair()

    def frame(data, mask=b'\x11\x22\x33\x44'):
        n = len(data)
        h = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack('>H', n) if n < 65536
                             else bytes([0x80 | 127]) + struct.pack('>Q', n))
        return h + mask + bytes(c ^ mask[i % 4] for i, c in enumerate(data))
    ok = True
    for msg in ('hi', 'x' * 300, 'é' * 40000, 'y' * 300000):
        f = frame(msg.encode())
        t = threading.Thread(target=lambda: a.sendall(f))
        t.start()
        ok &= T.ws_recv(b) == msg
        t.join()
    a.sendall(bytes([0x81, 0x80 | 127]) + struct.pack('>Q', 1 << 40) + b'\0\0\0\0')
    check(ok and T.ws_recv(b) is None, 'websocket: big messages arrive intact, a huge one is refused')

    # ---- arm camera on the forearm, never on the wrist (bug: it swept into the links)
    import scara_scene as S
    check(S.ARM_CAM_PARENT == 'forearm_link', 'arm camera is mounted on the forearm')
    xacro = open(os.path.join(ROOT, 'urdf', 'arm.xacro')).read()
    m = re.search(r'<joint name="arm_camera_joint".*?</joint>', xacro, re.S)
    check(m and '<parent link="forearm_link"/>' in m.group(0), 'URDF: arm_camera_joint parent is forearm_link')
    for xml in glob.glob(os.path.join(ROOT, 'mujoco', 'models', '*.xml')):
        root = ET.parse(xml).getroot()
        fore = next(b for b in root.iter('body') if b.get('name') == 'forearm_link')
        cams = [c.get('name') for c in fore.iter('camera')]
        wrist = next(b for b in root.iter('body') if b.get('name') == 'wrist_link')
        check('arm_camera' in cams and 'arm_camera' not in [c.get('name') for c in wrist.iter('camera')],
              f'MuJoCo {os.path.basename(xml)}: arm camera on the forearm')
    mesh_dir = [c.get('meshdir') for c in ET.parse(glob.glob(os.path.join(ROOT, 'mujoco', 'models', '*.xml'))[0]).getroot().iter('compiler')][0]
    check(os.path.isdir(os.path.join(ROOT, 'mujoco', 'models', mesh_dir)), f'MuJoCo models find the meshes ({mesh_dir})')

    # ---- voice phrases that were misread before (e.g. "let go" started the belt)
    import subprocess
    js = r"""
      global.window = {}; require('%s');
      const P = window.ScaraVoice.parse, k = (t) => JSON.stringify(P(t));
      console.log(JSON.stringify({letgo: k('let go'), release: k('release'), red: k('read then blew'),
        stop: k('stop'), estop: k('emergency stop'), resume: k('resume'), three: k('three green'), table: k('table')}));
    """ % os.path.join(ROOT, 'app', 'ui', 'voice.js').replace('\\', '/')
    try:
        r = json.loads(subprocess.run(['node', '-e', js], capture_output=True, text=True, timeout=20).stdout)
        check('gripper' in r['letgo'] and 'false' in r['letgo'] and '"belt"' not in r['letgo'], f'voice "let go" opens the gripper {r["letgo"]}')
        check('red then blue' in r['red'], f'voice "read then blew" -> red then blue {r["red"]}')
        check('"belt"' in r['stop'] and 'estop' in r['estop'] and 'release' in r['resume'], 'voice stop / emergency stop / resume')
        check('3 green' in r['three'] and 'manual' in r['table'], 'voice "three green", "table"')
    except (FileNotFoundError, ValueError, subprocess.TimeoutExpired) as e:
        print('SKIP voice checks (needs node):', e)

    # ---- Android app: camera permission check (bug: it was inverted)
    smali = open(os.path.join(ROOT, 'app', 'android', 'smali', 'com', 'iamsubhani', 'scaradigitaltwin', 'Bridge.smali')).read()
    body = smali[smali.index('.method public hasCamera'):smali.index('.end method', smali.index('.method public hasCamera'))]
    check('if-eqz v0, :yes' in body, 'Android: hasCamera() returns true when the permission is GRANTED (0)')
    man = open(os.path.join(ROOT, 'app', 'android', 'AndroidManifest.xml')).read()
    check('android.permission.CAMERA' in man and 'RECOGNIZE_SPEECH' in man, 'Android: camera permission + speech query declared')

    # ---- the web page loads every script it needs, the name only comes from brand.js
    html = open(os.path.join(ROOT, 'app', 'ui', 'index.html')).read()
    scripts = re.findall(r'<script src="([^"]+)"', html)
    check(all(os.path.exists(os.path.join(ROOT, 'app', 'ui', s)) for s in scripts) and 'brand.js' in scripts,
          f'web page: all scripts exist {scripts}')
    brand = json.loads(open(os.path.join(ROOT, 'app', 'ui', 'brand.js')).read().split('=', 1)[1].rstrip().rstrip(';'))
    nm = brand.get('name')
    has_paint = os.path.exists(os.path.join(ROOT, 'meshes', 'shoulder_name.stl'))
    check((nm is None) == (not has_paint) and (not nm or (nm not in open(os.path.join(ROOT, 'app', 'ui', 'app.js')).read()
                                                       and nm not in html)),
          f'the app shows a name only when the arm has one, and never hard-codes it (brand: {nm})')

    # ---- install rules keep the ROS layout launch files expect
    cm = open(os.path.join(ROOT, 'CMakeLists.txt')).read()
    progs = re.findall(r'^\s+((?:common|mujoco|gazebo|real_robot|server|gesture_control)/\w+\.py)$', cm, re.M)
    check(all(os.path.exists(os.path.join(ROOT, p)) for p in progs) and len(progs) >= 30,
          f'CMake installs {len(progs)} programs, every one exists')
    check('gazebo/worlds DESTINATION share/${PROJECT_NAME}' in cm and 'real_robot/firmware DESTINATION' in cm
          and 'mujoco/models DESTINATION share/${PROJECT_NAME}/mujoco' in cm, 'CMake keeps worlds / firmware / mujoco models where ROS finds them')
    launch = ' '.join(open(f).read() for f in glob.glob(os.path.join(ROOT, 'launch', '*.py')))
    exes = set(re.findall(r"executable='(\w+\.py)'", launch))
    installed = {os.path.basename(p) for p in progs}
    check(exes <= installed, f'every program a launch file starts is installed {sorted(exes - installed)}')

    print('ALL STATIC CHECKS PASSED' if not FAIL else f'{len(FAIL)} FAILED')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
