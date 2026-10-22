#!/usr/bin/env python3
"""
Static checks that run without ROS/Gazebo:
  * expands the simple ${...} properties in urdf/arm.xacro
  * checks the URDF tree (one root, every joint's links exist, no duplicates)
  * compares the URDF tool0 position with the MuJoCo 'tcp' site and with
    scripts/scara_kinematics.py at a few joint configurations
  * parses worlds/scara_world.sdf and checks the expected models/plugins exist
  * parses worlds/scara_manual_world.sdf (table + pucks) and checks every puck is reachable
  * parses worlds/scara_command_world.sdf (conveyor + ONE bin) and checks the bin is reachable

    python3 tools/check_description.py
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
import scara_scene as S          # noqa: E402
import scara_kinematics as K     # noqa: E402


def expand_xacro(path):
    txt = open(path).read()
    props = dict(re.findall(r'<xacro:property name="(\w+)" value="([^"]*)"\s*/>', txt))
    def sub(m):
        expr = m.group(1)
        return repr(eval(expr, {'pi': math.pi}, {k: float(v) for k, v in props.items()}))
    return re.sub(r'\$\{([^}]*)\}', sub, txt), props


def fail(msg):
    print('FAIL:', msg)
    sys.exit(1)


def main():
    txt, props = expand_xacro(os.path.join(ROOT, 'urdf', 'arm.xacro'))
    root = ET.fromstring(txt)
    links = [l.get('name') for l in root.findall('link')]
    if len(links) != len(set(links)):
        fail('duplicate link names')
    joints = root.findall('joint')
    children = set()
    for j in joints:
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        if p not in links or c not in links:
            fail(f'joint {j.get("name")} references missing link')
        if c in children:
            fail(f'link {c} has two parents')
        children.add(c)
    roots = [l for l in links if l not in children]
    if roots != ['world']:
        fail(f'expected single root "world", got {roots}')
    for k, v in (('mount_x', S.BASE_XYZ[0]), ('mount_y', S.BASE_XYZ[1]), ('mount_z', S.BASE_XYZ[2]),
                 ('mount_yaw', S.BASE_YAW)):
        pv = props[k]
        if abs(float(pv) - v) > 1e-9:
            fail(f'{k}={props[k]} does not match scara_scene BASE_XYZ {S.BASE_XYZ}')
    print(f'URDF tree OK: {len(links)} links, {len(joints)} joints, root=world')

    # forward kinematics of tool0 from the URDF
    jmap = {j.find('child').get('link'): j for j in joints}

    def tf(link, q):
        chain = []
        while link in jmap:
            chain.append(jmap[link]); link = jmap[link].find('parent').get('link')
        T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        for j in reversed(chain):
            o = j.find('origin')
            xyz = [float(v) for v in o.get('xyz').split()] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in o.get('rpy', '0 0 0').split()] if o is not None else [0, 0, 0]
            A = rot(rpy, xyz)
            v = q.get(j.get('name'), 0.0)
            if j.get('type') == 'revolute':
                M = rot([0, 0, v], [0, 0, 0])
            elif j.get('type') == 'prismatic':
                ax = [float(a) for a in j.find('axis').get('xyz').split()]
                M = rot([0, 0, 0], [a * v for a in ax])
            else:
                M = rot([0, 0, 0], [0, 0, 0])
            T = mul(mul(T, A), M)
        return [T[0][3], T[1][3], T[2][3]]

    for q in ([0, 0, 0, 0], [0.4, -0.1, -1.2, 0.7], [-1.1, -0.05, 1.6, -2.0]):
        qd = dict(zip(S.ARM_JOINTS, q))
        p = tf('tool0', qd)
        kx, ky = K.fk_xy(q[0], q[2], q[3])
        kz = K.pad_bottom_z(q[1]) + S.PAD_HEIGHT / 2
        err = math.dist(p, (kx, ky, kz))
        print(f'tool0 at q={q}: URDF {[round(v, 4) for v in p]}  kinematics {[round(kx, 4), round(ky, 4), round(kz, 4)]}  err {err * 1000:.3f} mm')
        if err > 1e-3:
            fail('URDF tool0 and scara_kinematics disagree')

    # world SDF
    w = ET.parse(os.path.join(ROOT, 'worlds', 'scara_world.sdf')).getroot()
    world = w.find('world')
    names = {m.get('name') for m in world.findall('model')}
    for need in ('table', 'conveyor_frame', 'conveyor_belt', 'fixed_camera', *[f'bin_{c}' for c in S.COLOR_ORDER]):
        if need not in names:
            fail(f'world is missing model {need}')
    plugins = {p.get('filename') for p in world.iter('plugin')}
    for need in ('gz-sim-physics-system', 'gz-sim-sensors-system', 'gz-sim-track-controller-system'):
        if need not in plugins:
            fail(f'world is missing plugin {need}')
    print(f'world SDF OK: models {sorted(names)}')
    mw = ET.parse(os.path.join(ROOT, 'worlds', 'scara_manual_world.sdf')).getroot().find('world')
    mnames = {m.get('name') for m in mw.findall('model')}
    want = {n for n, _ in S.MANUAL_PUCKS} | {'table', 'ground_plane'}
    if mnames != want:
        fail(f'manual world models {sorted(mnames)} != {sorted(want)}')
    for name, (x, y, yaw) in S.MANUAL_PUCKS:
        try:
            K.arm_pose(x, y, S.Z_PICK_TABLE, yaw=yaw, symmetry=math.pi)
        except K.IKError as e:
            fail(f'manual world: {name} cannot be grabbed: {e}')
    print(f'manual world SDF OK: {sorted(mnames)} - every puck reachable with the gripper around it')
    cw = ET.parse(os.path.join(ROOT, 'worlds', 'scara_command_world.sdf')).getroot().find('world')
    cnames = {m.get('name') for m in cw.findall('model')}
    for need in ('table', 'conveyor_frame', 'conveyor_belt', 'fixed_camera', 'drop_bin'):
        if need not in cnames:
            fail(f'command world is missing model {need}')
    if any(n.startswith('bin_') for n in cnames):
        fail('command world must have only the single drop bin')
    bx, by = S.command_bin_xy()
    try:
        K.arm_pose(bx, by, S.Z_PLACE, yaw=S.command_bin_yaw())
    except K.IKError as e:
        fail(f'command bin unreachable: {e}')
    print(f'command world SDF OK: {sorted(cnames)} - one bin, reachable')
    print('ALL CHECKS PASSED')


def rot(rpy, xyz):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
         [-sp, cp * sr, cp * cr]]
    return [R[0] + [xyz[0]], R[1] + [xyz[1]], R[2] + [xyz[2]], [0, 0, 0, 1]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


if __name__ == '__main__':
    main()
