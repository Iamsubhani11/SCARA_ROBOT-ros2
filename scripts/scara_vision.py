#!/usr/bin/env python3
"""
Colour-puck detection and pixel+depth -> world conversion.

Pure NumPy / OpenCV (no ROS, no cv_bridge), so the same code runs in the
ROS 2 vision node, in the MuJoCo app and in the tests.
"""
import math

import cv2
import numpy as np

try:
    import scara_scene as S
except ImportError:
    from . import scara_scene as S

# HSV ranges (OpenCV: H 0..180). Saturation / value floors keep the wooden
# table, the grey belt and shadows out.
HSV_RANGES = {
    'red':    [((0, 120, 70), (8, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    'yellow': [((21, 120, 120), (35, 255, 255))],
    'green':  [((40, 90, 60), (85, 255, 255))],
    'blue':   [((100, 120, 50), (130, 255, 255))],
}
DRAW_BGR = {'red': (40, 40, 230), 'green': (40, 200, 40), 'blue': (230, 90, 30), 'yellow': (30, 220, 240)}


def intrinsics(width, height, hfov):
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return np.array([[fx, 0, width / 2.0], [0, fx, height / 2.0], [0, 0, 1.0]])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class CameraModel:
    """Camera pose in Gazebo's convention: the camera looks along +X of its link,
    +Y of the link is image-left and +Z is image-up."""

    def __init__(self, xyz, rpy, K):
        self.t = np.asarray(xyz, dtype=float)
        self.R = rpy_matrix(*rpy)          # link -> world
        self.K = np.asarray(K, dtype=float)

    def pixel_to_world(self, u, v, depth):
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        # depth is the planar distance along the optical axis (= link +X)
        p_link = np.array([depth, -(u - cx) / fx * depth, -(v - cy) / fy * depth])
        return self.R @ p_link + self.t

    def world_to_pixel(self, p):
        q = self.R.T @ (np.asarray(p, dtype=float) - self.t)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        return cx - fx * q[1] / q[0], cy - fy * q[2] / q[0], q[0]


def fixed_camera_model(K=None):
    if K is None:
        K = intrinsics(S.FIXED_CAM_WIDTH, S.FIXED_CAM_HEIGHT, S.FIXED_CAM_HFOV)
    return CameraModel(S.FIXED_CAM_XYZ, S.FIXED_CAM_RPY, K)


def color_masks(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out = {}
    for name, ranges in HSV_RANGES.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        out[name] = mask
    return out


def detect_pucks(bgr, depth, cam, min_area=120, belt_only=True):
    """
    Returns a list of dicts: color, u, v, x, y, z (world, centre of the puck's
    top face), yaw (direction of the handle ring, radians, mod pi - the
    gripper's fingers must close across it), s (position along the belt),
    depth, area, bbox.

    Every blob pixel is turned into a 3-D point with the depth image:
      * points on the top face (z = surface + puck height) give the centre -
        the middle of the face's extent, so the handle hiding part of the
        face does not shift it;
      * points higher than the face belong to the handle; their main
        direction (PCA) is the handle's orientation.
    With belt_only, pucks that are not on the conveyor (e.g. in the bins,
    which have the same colours) are dropped; otherwise pucks standing on the
    table are found.
    """
    dets = []
    h, w = depth.shape[:2]
    fx, fy, cx, cy = cam.K[0, 0], cam.K[1, 1], cam.K[0, 2], cam.K[1, 2]
    z_top = (S.BELT_TOP_Z if belt_only else S.TABLE_TOP_Z) + S.PUCK_HEIGHT
    for name, mask in color_masks(bgr).items():
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        for i in range(1, n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area:
                continue
            x0, y0, bw, bh = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                         cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            if x0 <= 1 or y0 <= 1 or x0 + bw >= w - 1 or y0 + bh >= h - 1:
                continue                      # cut off by the image border
            ys, xs = np.nonzero(labels[y0:y0 + bh, x0:x0 + bw] == i)
            ys = ys + y0
            xs = xs + x0
            d = depth[ys, xs].astype(float)
            ok = np.isfinite(d) & (d > 0.05)
            if ok.sum() < 10:
                continue
            ys, xs, d = ys[ok], xs[ok], d[ok]
            p_link = np.stack([d, -(xs - cx) / fx * d, -(ys - cy) / fy * d])
            pts = (cam.R @ p_link).T + cam.t
            top = np.abs(pts[:, 2] - z_top) < 0.006
            if top.sum() < 8:
                continue
            tp = pts[top]
            P = np.array([(tp[:, 0].min() + tp[:, 0].max()) / 2, (tp[:, 1].min() + tp[:, 1].max()) / 2, z_top])
            if max(np.ptp(tp[:, 0]), np.ptp(tp[:, 1])) > 2.6 * S.PUCK_RADIUS:
                P[:2] = tp[:, :2].mean(axis=0)        # two touching pucks merged: fall back to the mean
            hp = pts[(pts[:, 2] > z_top + 0.012) & (np.hypot(pts[:, 0] - P[0], pts[:, 1] - P[1]) < S.HANDLE_OUTER_R + 0.01)]
            yaw = None
            if len(hp) >= 12:
                c = hp[:, :2] - hp[:, :2].mean(axis=0)
                w_, v_ = np.linalg.eigh(c.T @ c)
                if w_[1] > 2.5 * max(w_[0], 1e-12):        # clearly elongated: a handle seen from above
                    yaw = float(math.atan2(v_[1, 1], v_[0, 1]))
                    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
            u, v = float(xs[top].mean()), float(ys[top].mean())
            sb = S.belt_s(P[0], P[1])
            if belt_only:
                on_belt = (abs(S.belt_lat(P[0], P[1])) < S.BELT_WIDTH / 2
                           and S.CONVEYOR_Y_RANGE[0] < sb < S.CONVEYOR_Y_RANGE[1])
                if not on_belt:
                    continue
            dets.append({'color': name, 'u': u, 'v': v, 'depth': float(np.median(d[top])),
                         'x': float(P[0]), 'y': float(P[1]), 'z': float(P[2]), 's': float(sb), 'yaw': yaw,
                         'area': int(area), 'bbox': (x0, y0, bw, bh)})
    dets.sort(key=lambda d: -d['s'])   # most downstream first
    return dets


def draw_detections(bgr, dets, title='Fixed Camera Box Detector'):
    img = bgr.copy()
    for d in dets:
        x0, y0, bw, bh = d['bbox']
        col = DRAW_BGR[d['color']]
        cv2.rectangle(img, (x0, y0), (x0 + bw, y0 + bh), col, 2)
        cv2.circle(img, (int(d['u']), int(d['v'])), 4, (255, 255, 255), -1)
        cv2.putText(img, f"{d['color']} {d['depth']:.3f} m", (x0, max(28, y0 - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.putText(img, f"({int(d['u'])},{int(d['v'])})", (x0, max(42, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
    cv2.putText(img, title, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2, cv2.LINE_AA)
    return img
