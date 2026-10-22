#!/usr/bin/env python3
"""
Physics test of the manual-control world (no conveyor, pucks on the table, like
Parts 6-7 of the Mechamind series): the arm grabs each puck by its body (it fits
between the open fingers), the
way you would with the keyboard / joystick / gestures, lifts it and puts it
down at a new spot.  Uses the headless MuJoCo harness.

    python3 tools/sim_test_manual.py <harness-binary> [video.mp4]
"""
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import scara_scene as S                       # noqa: E402
from scara_sorter_core import Sorter          # noqa: E402
from sim_test_sorting import HarnessIO        # noqa: E402

# where each puck is moved to (cell frame: x towards the viewer, y sideways)
def _arc(deg, r=S.MANUAL_ARC_R):
    return (-0.155 + r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg)))


# put pucks down further round the same arc, clear of the others
TARGETS = {'puck_blue_2': _arc(-90), 'puck_green_2': _arc(90),
           'puck_blue_1': _arc(-115), 'puck_green_3': _arc(115)}


def main():
    harness = sys.argv[1]
    video = sys.argv[2] if len(sys.argv) > 2 else None
    io = HarnessIO(harness, os.path.join(ROOT, 'urdf', 'scara_manual_mujoco.xml'), video=video)
    io.sleep(0.5)
    sorter = Sorter(io, 'all')
    start = dict(S.MANUAL_PUCKS)
    results = {}
    for name, (cx, cy) in TARGETS.items():
        x, y, yaw = start[name]
        ok = sorter.pick_at(x, y, yaw, S.TABLE_TOP_Z, name)
        _, pk = io.state()
        lifted = pk[name][2] > S.TABLE_TOP_Z + 0.05
        tx, ty = S.cell_xy(cx, cy)
        if ok:
            sorter.place_at(tx, ty, S.TABLE_TOP_Z)
        sorter.move_to(*S.READY_XY, S.Z_TRAVEL)
        _, pk = io.state()
        px, py, pz = pk[name]
        err = math.hypot(px - tx, py - ty)
        results[name] = {'grabbed': ok, 'lifted': lifted, 'placed_err_mm': round(err * 1000, 1),
                         'on_table': abs(pz - S.TABLE_TOP_Z) < 0.01}
        io.log(f'{name}: {results[name]}')
    _, pk = io.state()
    untouched = [n for n in start if n not in TARGETS]
    moved = [n for n in untouched if math.hypot(pk[n][0] - start[n][0], pk[n][1] - start[n][1]) > 0.01]
    io.close()
    ok = all(r['grabbed'] and r['lifted'] and r['placed_err_mm'] < 25 and r['on_table'] for r in results.values()) and not moved
    print(json.dumps({'results': results, 'other_pucks_disturbed': moved, 'PASS': ok}))
    if video:
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                        '-s', f'{io.size[0]}x{io.size[1]}', '-r', str(io.fps), '-i', io.raw,
                        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', video], check=True)
        os.remove(io.raw)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
