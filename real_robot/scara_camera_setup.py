#!/usr/bin/env python3
"""
Camera setup for the real conveyor cell: train the cube colours and the belt ruler.

    python3 real_robot/scara_camera_setup.py                  # camera 0
    python3 real_robot/scara_camera_setup.py --source 1       # another USB camera
    python3 real_robot/scara_camera_setup.py --source http://192.168.1.23:8080/video   # phone as IP camera
    python3 real_robot/scara_camera_setup.py --source photo.jpg                        # try it on a still

Put the camera on its stand so it sees the pick station of the belt from the side
(or from above), like in the demo video, and leave it there - if it moves, run
this again.  Then, in the window:

  R  drag a box around the belt (only that area is searched)
  1 2 3 4   then click on a red / green / blue / yellow cube to teach that colour.
            Click on 2-3 cubes (and in different spots of the belt) - every click
            widens the colour a little.  0 forgets the colour you are teaching.
  P  put a cube exactly under the gripper's pick point (the spot where the taught
     pick pose closes), click on the middle of that cube
  U  move the cube upstream (towards where cubes come from) by the distance typed
     in the terminal (default 60 mm), click on it again
  D  optional: move it the same distance DOWNSTREAM from the pick point and click
     - this corrects for perspective when the camera looks from the side
  [ ]  make the pick zone narrower / wider      F  flip the image (camera upside down)
  S  save (to ~/.scara_camera.json)      Q / Esc  quit

Live view: every cube found gets a box and its distance from the pick point
("green +35 mm").  When that number reads about 0 with the cube at the pick
point, you are done.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import os
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scara_cam2d import Camera2D, COLORS    # noqa: E402

HELP = ['R belt area   1-4 + click: teach red/green/blue/yellow   0 forget colour',
        'P pick point   U upstream   D downstream (optional)   [ ] zone   F flip   S save   Q quit']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--source', default=None, help='camera index, video/image file or URL')
    ap.add_argument('--config', default=None, help='file to read/write (default ~/.scara_camera.json)')
    ap.add_argument('--distance', type=float, default=60.0, help='mm between the pick point and the U / D clicks')
    a = ap.parse_args()

    cam = Camera2D.load(a.config)
    if a.source is not None:
        cam.cfg['source'] = int(a.source) if a.source.isdigit() else a.source
    cam.open()
    win = 'SCARA camera setup'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    st = {'mode': None, 'color': None, 'drag': None, 'msg': 'Press R to mark the belt area', 'p0': None,
          'p1': None, 'p2': None, 'frame': None}
    if cam.cfg['ruler']:
        r = cam.cfg['ruler']
        st['p0'], st['p1'], st['p2'] = r['p0'], r['p1'], r.get('p2')

    def biggest_blob_near(x, y):
        """Snap a click to the centre of the cube under it, if one is found."""
        dets = cam.detect(st['frame'])
        best = min(dets, key=lambda d: (d['cx'] - x) ** 2 + (d['cy'] - y) ** 2, default=None)
        if best and (best['cx'] - x) ** 2 + (best['cy'] - y) ** 2 < 40 ** 2:
            return best['cx'], best['cy']
        return float(x), float(y)

    def on_mouse(ev, x, y, flags, _):
        if st['frame'] is None:
            return
        if st['mode'] == 'roi':
            if ev == cv2.EVENT_LBUTTONDOWN:
                st['drag'] = (x, y)
            elif ev == cv2.EVENT_LBUTTONUP and st['drag']:
                x0, y0 = st['drag']
                if abs(x - x0) > 20 and abs(y - y0) > 20:
                    cam.cfg['roi'] = [min(x, x0), min(y, y0), max(x, x0), max(y, y0)]
                    st['msg'] = 'Belt area set. Now press 1 and click on a red cube (2 green, 3 blue, 4 yellow)'
                st['drag'], st['mode'] = None, None
            return
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        if st['mode'] == 'color':
            ok, msg = cam.train_color(st['frame'], st['color'], x, y)
            st['msg'] = msg + ('  - click another cube to widen, or press the next number' if ok else '')
        elif st['mode'] in ('p0', 'p1', 'p2'):
            st[st['mode']] = list(biggest_blob_near(x, y))
            if st['mode'] == 'p0':
                st['msg'] = f'Pick point set. Move the cube {a.distance:.0f} mm UPSTREAM, press U and click it'
            elif st['mode'] == 'p1':
                st['msg'] = 'Ruler set - the labels now show mm. Optional: D for perspective. S to save'
            else:
                st['msg'] = 'Perspective point set. S to save'
            if st['p0'] and st['p1']:
                cam.set_ruler(st['p0'], st['p1'], a.distance, st['p2'])
            st['mode'] = None

    cv2.setMouseCallback(win, on_mouse)
    while True:
        frame = cam.read()
        st['frame'] = frame
        view = frame.copy()
        dets = cam.detect(frame) if cam.cfg['colors'] else []
        cam.draw(view, dets)
        for k in ('p0', 'p1', 'p2'):
            if st[k]:
                cv2.circle(view, tuple(int(v) for v in st[k]), 5, (255, 255, 255), -1)
                cv2.putText(view, {'p0': 'pick', 'p1': '+%.0f' % a.distance, 'p2': '-%.0f' % a.distance}[k],
                            (int(st[k][0]) + 7, int(st[k][1]) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        y = 18
        for line in HELP + [st['msg']]:
            cv2.putText(view, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(view, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            y += 18
        cv2.imshow(win, view)
        k = cv2.waitKey(30) & 0xFF
        if k in (ord('q'), 27):
            break
        if k == ord('r'):
            st['mode'] = 'roi'
            st['msg'] = 'Drag a box around the belt'
        elif k in (ord('1'), ord('2'), ord('3'), ord('4')):
            st['mode'], st['color'] = 'color', COLORS[k - ord('1')]
            st['msg'] = f'Click on the middle of a {st["color"]} cube'
        elif k == ord('0') and st['color']:
            cam.cfg['colors'].pop(st['color'], None)
            st['msg'] = f'Forgot {st["color"]} - click on a {st["color"]} cube again'
            st['mode'] = 'color'
        elif k in (ord('p'), ord('u'), ord('d')):
            st['mode'] = {ord('p'): 'p0', ord('u'): 'p1', ord('d'): 'p2'}[k]
            st['msg'] = {'p0': 'Click the cube at the pick point',
                         'p1': f'Click the cube {a.distance:.0f} mm upstream',
                         'p2': f'Click the cube {a.distance:.0f} mm downstream'}[st['mode']]
        elif k in (ord('['), ord(']')):
            lo, hi = cam.cfg['window_mm']
            f = 0.8 if k == ord('[') else 1.25
            cam.cfg['window_mm'] = [lo * f, hi * f]
            st['msg'] = 'pick zone %.0f .. %+.0f mm' % tuple(cam.cfg['window_mm'])
        elif k == ord('f'):
            cam.cfg['flip'] = not cam.cfg.get('flip')
        elif k == ord('s'):
            miss = cam.ready()
            path = cam.save(a.config)
            st['msg'] = f'Saved to {path}' + (' - still missing: ' + '; '.join(miss) if miss else ' - ready')
            print(st['msg'])
    cam.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
