#!/usr/bin/env python3
"""
2-D colour camera for the REAL conveyor cell (no depth camera, no camera on the arm).

One ordinary USB webcam on a stand next to the conveyor looks at the pick station,
like in the Part 25 demo.  It does not need to know 3-D positions: the cubes ride on
the belt, so the only thing that changes is how far along the belt a cube is.  The
setup tool (scara_camera_setup.py) stores:

  * the belt area in the image (only that area is searched)
  * a colour model per cube colour, trained by clicking on the cubes
  * a ruler along the belt: the pixel where a cube sits when it is exactly at the
    pick point, and where it sits D mm upstream (optionally also D mm downstream,
    which corrects for perspective)

From that, every detection gets  s_mm = distance from the pick point along the belt
(+ = still coming, - = already past).  The robot side (scara_hw_cell.py) turns s_mm
into a gripper position with the pick pose you taught on the real robot.

    cam = Camera2D.load()          # ~/.scara_camera.json (or config/camera_2d.json)
    cam.open()                     # camera index / video file / image from the config
    frame = cam.read()
    dets = cam.detect(frame)       # [{'color', 'cx', 'cy', 'box', 'area', 's_mm'}, ...]
    cam.draw(frame, dets)          # boxes, labels and the two pick-zone lines
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
USER_FILE = os.path.join(os.path.expanduser('~'), '.scara_camera.json')
COLORS = ('red', 'green', 'blue', 'yellow')
DRAW_BGR = {'red': (40, 40, 220), 'green': (60, 170, 40), 'blue': (220, 110, 30), 'yellow': (0, 200, 230)}


def default_config():
    return {
        'source': 0,                 # camera index, video file, image file, or http/rtsp URL (phone IP camera)
        'width': 640, 'height': 480,
        'roi': None,                 # [x0, y0, x1, y1] belt area; None = whole image
        'colors': {},                # name -> {'lo': [h, s, v], 'hi': [h, s, v], 'samples': n}
        'min_area': 350,             # px, smaller blobs are noise
        'max_area': 60000,
        'ruler': None,               # {'p0': [x, y], 'p1': [x, y], 'd_mm': 60, 'p2': [x, y] or None}
        'window_mm': [-12.0, 18.0],  # stop the belt when a wanted cube is inside this zone
        'flip': False,               # rotate the image 180 deg (camera mounted upside down)
    }


class Camera2D:
    def __init__(self, cfg=None, path=None):
        self.cfg = default_config()
        self.cfg.update(cfg or {})
        self.path = path or USER_FILE
        self.cap = None
        self.frame_source = None     # optional callable returning BGR frames (simulation / tests)
        self._latest = None
        self._lock = threading.Lock()
        self._stop = False

    # ------------------------------------------------------------------ config
    @classmethod
    def load(cls, path=None):
        cands = [path] if path else [USER_FILE, os.path.join(HERE, '..', 'config', 'camera_2d.json')]
        for p in cands:
            if p and os.path.exists(p):
                with open(p) as fh:
                    return cls(json.load(fh), path=path or USER_FILE)
        return cls(path=path)

    def save(self, path=None):
        path = path or self.path
        with open(path, 'w') as fh:
            json.dump(self.cfg, fh, indent=1)
        return path

    def ready(self):
        """What is still missing before the cell can run (empty list = ready)."""
        miss = []
        if not self.cfg['colors']:
            miss.append('no colours trained (press 1-4 in the setup tool and click on cubes)')
        if not self.cfg['ruler']:
            miss.append('no belt ruler (press P and U in the setup tool)')
        return miss

    # ------------------------------------------------------------------ capture
    def open(self):
        if self.frame_source is not None:
            return
        src = self.cfg['source']
        if isinstance(src, str) and src.isdigit():
            src = int(src)
        if isinstance(src, str) and os.path.splitext(src)[1].lower() in ('.png', '.jpg', '.jpeg', '.bmp'):
            img = cv2.imread(src)
            if img is None:
                raise RuntimeError(f'cannot read image {src}')
            self.frame_source = lambda: img.copy()
            return
        self.cap = cv2.VideoCapture(src)
        if isinstance(src, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg['width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg['height'])
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open camera {src!r} - check the cable / index (try 0, 1, 2) or the URL')
        self._is_file = isinstance(src, str) and not src.startswith(('http', 'rtsp'))
        if not self._is_file:
            # live camera: keep only the newest frame, so a slow loop never sees old images
            threading.Thread(target=self._grabber, daemon=True).start()

    def _grabber(self):
        while not self._stop:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self._latest = f

    def read(self):
        if self.frame_source is not None:
            f = self.frame_source()
        elif getattr(self, '_is_file', False):
            ok, f = self.cap.read()
            if not ok:                                   # loop a test video
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, f = self.cap.read()
            f = f if ok else None
        else:
            t0 = time.time()
            while True:
                with self._lock:
                    f = None if self._latest is None else self._latest.copy()
                if f is not None or time.time() - t0 > 3.0:
                    break
                time.sleep(0.02)
        if f is None:
            raise RuntimeError('no image from the camera')
        if self.cfg.get('flip'):
            f = cv2.rotate(f, cv2.ROTATE_180)
        return f

    def close(self):
        self._stop = True
        if self.cap is not None:
            self.cap.release()

    # ------------------------------------------------------------------ training
    def train_color(self, frame, name, x, y, r=7):
        """Learn a colour from the pixels around (x, y). Click several times to widen it."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        patch = hsv[max(0, y - r):min(h, y + r + 1), max(0, x - r):min(w, x + r + 1)].reshape(-1, 3).astype(float)
        # circular mean of the hue (0..180 in OpenCV), so red near 0/180 works
        ang = patch[:, 0] / 180.0 * 2 * math.pi
        hm = (math.atan2(np.sin(ang).mean(), np.cos(ang).mean()) % (2 * math.pi)) / (2 * math.pi) * 180.0
        dh = (patch[:, 0] - hm + 90) % 180 - 90
        hs = float(np.sqrt((dh ** 2).mean()))
        sm, ss = patch[:, 1].mean(), patch[:, 1].std()
        vm, vs = patch[:, 2].mean(), patch[:, 2].std()
        if sm < 50:
            return False, f'that spot is grey (saturation {sm:.0f}) - click on the middle of a coloured cube'
        new = {'h': [hm - max(7.0, 3 * hs), hm + max(7.0, 3 * hs)],
               's': [max(45.0, sm - 3 * ss - 25), 255.0],
               'v': [max(35.0, vm - 3 * vs - 45), 255.0]}
        old = self.cfg['colors'].get(name)
        if old:                                         # widen the existing model
            oh = old['h']
            centre_old = (oh[0] + oh[1]) / 2
            d = (hm - centre_old + 90) % 180 - 90       # keep both on the same side of the wrap
            lo = min(oh[0], centre_old + d - (new['h'][1] - new['h'][0]) / 2)
            hi = max(oh[1], centre_old + d + (new['h'][1] - new['h'][0]) / 2)
            new = {'h': [lo, hi], 's': [min(old['s'][0], new['s'][0]), 255.0],
                   'v': [min(old['v'][0], new['v'][0]), 255.0], 'samples': old.get('samples', 1) + 1}
        else:
            new['samples'] = 1
        if new['h'][1] - new['h'][0] > 60:
            return False, 'that colour range got too wide - press 0 to forget this colour and train it again'
        self.cfg['colors'][name] = new
        return True, f'{name}: hue {new["h"][0] % 180:.0f}..{new["h"][1] % 180:.0f}, sat >= {new["s"][0]:.0f}, ' \
                     f'value >= {new["v"][0]:.0f} ({new["samples"]} clicks)'

    def mask(self, hsv, name):
        m = self.cfg['colors'][name]
        lo, hi = m['h']
        sv_lo = (m['s'][0], m['v'][0])
        out = None
        # split a hue range that crosses 0 / 180 into two
        ranges = []
        lo_w, hi_w = lo % 180, hi % 180
        if hi - lo >= 180:
            ranges = [(0, 180)]
        elif lo_w <= hi_w:
            ranges = [(lo_w, hi_w)]
        else:
            ranges = [(lo_w, 180), (0, hi_w)]
        for a, b in ranges:
            part = cv2.inRange(hsv, np.array([a, sv_lo[0], sv_lo[1]], np.float32).astype(np.uint8),
                               np.array([min(b, 180), 255, 255], np.float32).astype(np.uint8))
            out = part if out is None else cv2.bitwise_or(out, part)
        return out

    # ------------------------------------------------------------------ ruler
    def set_ruler(self, p0, p1, d_mm, p2=None):
        self.cfg['ruler'] = {'p0': list(map(float, p0)), 'p1': list(map(float, p1)), 'd_mm': float(d_mm),
                             'p2': list(map(float, p2)) if p2 is not None else None}

    def s_mm(self, x, y):
        """Distance (mm) of image point (x, y) from the pick point along the belt."""
        r = self.cfg['ruler']
        if not r:
            return None
        p0, p1 = np.array(r['p0'], float), np.array(r['p1'], float)
        u = p1 - p0
        L = float(np.linalg.norm(u))
        if L < 1:
            return None
        u /= L
        t = float((np.array([x, y]) - p0) @ u)            # pixels along the belt from the pick point
        if r.get('p2'):
            # three points (0, +D, -D): s = a t + b t^2 corrects the perspective of a side view
            t1, t2 = L, float((np.array(r['p2']) - p0) @ u)
            D = r['d_mm']
            A = np.array([[t1, t1 * t1], [t2, t2 * t2]])
            try:
                a, b = np.linalg.solve(A, np.array([D, -D]))
                return a * t + b * t * t
            except np.linalg.LinAlgError:
                pass
        return t * r['d_mm'] / L

    def _zone_points(self, s_target):
        """Image point at s_target mm (inverse of s_mm, by search along the ruler axis)."""
        r = self.cfg['ruler']
        p0, p1 = np.array(r['p0'], float), np.array(r['p1'], float)
        u = (p1 - p0) / max(np.linalg.norm(p1 - p0), 1e-6)
        best, bt = None, 0.0
        for t in np.linspace(-3 * np.linalg.norm(p1 - p0), 3 * np.linalg.norm(p1 - p0), 600):
            p = p0 + u * t
            s = self.s_mm(*p)
            if best is None or abs(s - s_target) < best:
                best, bt = abs(s - s_target), t
        return p0 + u * bt, u

    # ------------------------------------------------------------------ detection
    def detect(self, frame):
        h, w = frame.shape[:2]
        x0, y0, x1, y1 = self.cfg['roi'] or [0, 0, w, h]
        x0, y0 = max(0, int(x0)), max(0, int(y0))
        x1, y1 = min(w, int(x1)), min(h, int(y1))
        sub = cv2.GaussianBlur(frame[y0:y1, x0:x1], (5, 5), 0)
        hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        out = []
        for name in self.cfg['colors']:
            m = self.mask(hsv, name)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                a = cv2.contourArea(c)
                if not self.cfg['min_area'] <= a <= self.cfg['max_area']:
                    continue
                bx, by, bw, bh = cv2.boundingRect(c)
                if a < 0.35 * bw * bh:                    # thin streaks / shadows, not a cube
                    continue
                mo = cv2.moments(c)
                cx, cy = mo['m10'] / mo['m00'] + x0, mo['m01'] / mo['m00'] + y0
                out.append({'color': name, 'cx': cx, 'cy': cy, 'area': a,
                            'box': [bx + x0, by + y0, bw, bh], 's_mm': self.s_mm(cx, cy)})
        out.sort(key=lambda d: -(d['s_mm'] if d['s_mm'] is not None else -1e9))
        return out

    # ------------------------------------------------------------------ drawing
    def draw(self, frame, dets, wanted=None, status=''):
        """Overlay like the demo video: ROI, the two pick-zone lines, boxes with +mm labels."""
        img = frame
        if self.cfg['roi']:
            x0, y0, x1, y1 = map(int, self.cfg['roi'])
            cv2.rectangle(img, (x0, y0), (x1, y1), (200, 200, 200), 1)
        if self.cfg['ruler']:
            lo, hi = self.cfg['window_mm']
            for s in (lo, hi):
                p, u = self._zone_points(s)
                n = np.array([-u[1], u[0]]) * 140
                a, b = (p - n).astype(int), (p + n).astype(int)
                cv2.line(img, tuple(a), tuple(b), (0, 165, 255), 2, cv2.LINE_AA)
            p, u = self._zone_points(hi)
            cv2.putText(img, 'pick zone between these lines', (int(p[0]) + 6, int(p[1]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1, cv2.LINE_AA)
            p0 = tuple(int(v) for v in self.cfg['ruler']['p0'])
            cv2.drawMarker(img, p0, (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        for d in dets:
            x, y, w, h = d['box']
            col = DRAW_BGR.get(d['color'], (255, 255, 255))
            thick = 3 if wanted and wanted(d['color']) else 1
            cv2.rectangle(img, (x, y), (x + w, y + h), col, thick)
            lab = d['color'] + ('' if d['s_mm'] is None else f' {d["s_mm"]:+.0f} mm')
            cv2.putText(img, lab, (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, lab, (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        if status:
            cv2.putText(img, status, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, status, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
        return img
