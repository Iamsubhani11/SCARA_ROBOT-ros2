#!/usr/bin/env python3
"""Headless MuJoCo for the simulation tests (tools/sim_test_*.py, tools/test_hw_cell.py), with the
official `mujoco` Python package. One command per line on stdin, one answer line on stdout:

    ctrl c0 c1 ...                 actuator targets
    step N                         N physics steps (moves pucks lying on the belt)
    belt speed                     belt surface speed (m/s, +x)
    beltcfg y ztop halfwidth xmin xmax
    setpuck name x y z [yaw]       teleport a free body (zero velocity)
    state                          STATE t joint=q ... | body x y z ... | tcp x y z
    render cam w h path            path.rgb (uint8 RGB, top row first) + path.depth (float32, metres)
    vopen path w h / vframe cam / vclose     raw RGB video
    contacts / pquat name / quit

    python3 tools/mujoco_harness.py mujoco/models/scara_scene_mujoco.xml
"""
import math
import sys

import mujoco
import numpy as np


def main():
    if len(sys.argv) < 2:
        print('usage: mujoco_harness.py model.xml', file=sys.stderr)
        return 1
    m = mujoco.MjModel.from_xml_path(sys.argv[1])
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    FREE = mujoco.mjtJoint.mjJNT_FREE
    free = [j for j in range(m.njnt) if m.jnt_type[j] == FREE]
    hinge = [j for j in range(m.njnt) if m.jnt_type[j] != FREE]
    belt_j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'belt_joint')
    tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
    belt = {'speed': 0.0, 'y': 0.0, 'z': 0.0, 'hw': 0.0, 'xmin': 0.0, 'xmax': 0.0}
    renderers = {}
    video = {'f': None, 'w': 0, 'h': 0}

    def renderer(w, h):
        if (w, h) not in renderers:
            m.vis.global_.offwidth = max(m.vis.global_.offwidth, w)
            m.vis.global_.offheight = max(m.vis.global_.offheight, h)
            renderers[(w, h)] = mujoco.Renderer(m, height=h, width=w)
        return renderers[(w, h)]

    def camera(spec):
        if spec.startswith('free:'):
            az, el, dist, lx, ly, lz = (float(v) for v in spec[5:].split(':'))
            c = mujoco.MjvCamera()
            c.type = mujoco.mjtCamera.mjCAMERA_FREE
            c.azimuth, c.elevation, c.distance = az, el, dist
            c.lookat[:] = (lx, ly, lz)
            return c
        return spec if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, spec) >= 0 else 0

    def draw(spec, w, h, depth):
        r = renderer(w, h)
        r.update_scene(d, camera=camera(spec))
        rgb = r.render().copy()                           # (h, w, 3) uint8, top row first
        dep = None
        if depth:
            r.enable_depth_rendering()
            r.update_scene(d, camera=camera(spec))
            dep = r.render().astype(np.float32)          # metres along the optical axis
            r.disable_depth_rendering()
        return rgb, dep

    def apply_belt():
        if belt['speed'] == 0.0:
            return
        for j in free:
            qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
            x, y, z = d.qpos[qa], d.qpos[qa + 1], d.qpos[qa + 2]
            if (abs(y - belt['y']) < belt['hw'] and belt['z'] - 0.01 < z < belt['z'] + 0.01
                    and belt['xmin'] < x < belt['xmax']):
                d.qvel[va] = belt['speed']
                d.qvel[va + 1] *= 0.5

    print(f'READY nu={m.nu} nq={m.nq}', flush=True)
    for line in sys.stdin:
        p = line.split()
        if not p:
            continue
        c, a = p[0], p[1:]
        out = 'OK'
        if c == 'ctrl':
            for i, v in enumerate(a[:m.nu]):
                d.ctrl[i] = float(v)
        elif c == 'step':
            for _ in range(int(a[0]) if a else 1):
                apply_belt()
                mujoco.mj_step(m, d)
                if belt_j >= 0 and d.qpos[m.jnt_qposadr[belt_j]] > 0.02:
                    d.qpos[m.jnt_qposadr[belt_j]] -= 0.02
            out = f'OK {d.time:.4f}'
        elif c == 'belt':
            belt['speed'] = float(a[0])
        elif c == 'beltcfg':
            belt.update(zip(('y', 'z', 'hw', 'xmin', 'xmax'), map(float, a[:5])))
        elif c == 'setpuck':
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, a[0])
            if b >= 0:
                x, y, z = map(float, a[1:4])
                yaw = float(a[4]) if len(a) > 4 else 0.0
                j = m.body_jntadr[b]
                qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
                d.qpos[qa:qa + 7] = (x, y, z, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
                d.qvel[va:va + 6] = 0.0
                mujoco.mj_forward(m, d)
        elif c == 'state':
            s = [f'STATE {d.time:.4f}']
            s += [f'{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)}={d.qpos[m.jnt_qposadr[j]]:.5f}' for j in hinge]
            s.append('|')
            for j in free:
                qa = m.jnt_qposadr[j]
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j])
                s.append(f'{name} {d.qpos[qa]:.5f} {d.qpos[qa + 1]:.5f} {d.qpos[qa + 2]:.5f}')
            if tcp >= 0:
                x, y, z = d.site_xpos[tcp]
                s.append(f'| tcp {x:.5f} {y:.5f} {z:.5f}')
            out = ' '.join(s)
        elif c == 'render':
            spec, w, h, path = a[0], int(a[1]), int(a[2]), a[3]
            rgb, dep = draw(spec, w, h, True)
            rgb.astype(np.uint8).tofile(path + '.rgb')
            dep.astype(np.float32).tofile(path + '.depth')
        elif c == 'vopen':
            video.update(f=open(a[0], 'wb'), w=int(a[1]), h=int(a[2]))
        elif c == 'vframe':
            if video['f']:
                rgb, _ = draw(a[0], video['w'], video['h'], False)
                video['f'].write(rgb.astype(np.uint8).tobytes())
        elif c == 'vclose':
            if video['f']:
                video['f'].close()
            video['f'] = None
        elif c == 'contacts':
            for i in range(d.ncon):
                ct = d.contact[i]
                g1, g2 = int(ct.geom1), int(ct.geom2)
                n1 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g1) or '?'
                n2 = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g2) or '?'
                print(f'C {n1} {n2} dist={ct.dist:.5f} pos={ct.pos[0]:.3f},{ct.pos[1]:.3f},{ct.pos[2]:.3f}')
            out = 'END'
        elif c == 'pquat':
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, a[0]) if a else -1
            if b >= 0:
                qa = m.jnt_qposadr[m.body_jntadr[b]]
                out = 'Q ' + ' '.join(f'{v:.6f}' for v in d.qpos[qa + 3:qa + 7])
            else:
                out = 'Q 1 0 0 0'
        elif c == 'quit':
            break
        else:
            out = f'ERR {c}'
        print(out, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
