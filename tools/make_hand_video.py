#!/usr/bin/env python3
"""A synthetic "camera" video of a hand for the gesture-control test (tools/test_app_v3.py
--hand-video): Chrome plays it as the webcam.

    0 - 6 s   open hand in the middle (the app learns it in the first 2 s)
    6 - 10 s  the hand moves to the side and back      -> the arm turns
    10 - 14 s a fist (smaller), moving a little         -> the gripper closes
    14 - 20 s open hand in the middle again

    python3 tools/make_hand_video.py /tmp/hand.y4m

--real makes a video from real hand photos (tools/testdata/) for the MediaPipe test - the simple
oval above is enough for the colour tracker, but MediaPipe only finds a hand that looks like a hand:

    0 - 3 s   open hand in the middle                  -> the hand is "armed", arm still
    3 - 7 s   the hand moves to the side and back      -> the arm turns
    7 - 10 s  a fist in the middle                     -> the gripper closes
    10 - 12 s open hand in the middle                  -> the gripper opens
    12 - 18 s two hands, the second one goes up        -> the right joystick moves

    python3 tools/make_hand_video.py --real /tmp/hand_real.y4m"""
import math
import os
import sys

import numpy as np

W, H, FPS, SECONDS = 320, 240, 15, 20


def frame(t):
    img = np.empty((H, W, 3), np.float32)
    img[:] = (70, 80, 95)                                   # plain blue-grey wall
    cx, cy, rx, ry = W / 2, H / 2, 34.0, 46.0               # open hand
    if 6 <= t < 10:
        cx -= 75 * math.sin(math.pi * (t - 6) / 4)          # to the side and back
    elif 10 <= t < 14:
        rx, ry = 22.0, 30.0                                 # a fist is about half the area
        cx += 40 * math.sin(math.pi * (t - 10) / 4)
    yy, xx = np.mgrid[0:H, 0:W]
    inside = ((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2 <= 1.0
    img[inside] = (205, 150, 125)                           # skin
    return img


RW, RH, RSECONDS = 480, 360, 18           # the real-photo video


class RealHands:
    """Pastes the photos of tools/testdata/ onto a plain wall, frame by frame."""
    def __init__(self):
        import cv2
        here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testdata')
        rd = lambda n: cv2.cvtColor(cv2.imread(os.path.join(here, n)), cv2.COLOR_BGR2RGB).astype(np.float32)  # noqa: E731
        both = rd('hands_open.jpg')                   # 720 x 382, two open hands
        self.cv2 = cv2
        h = int(RH * 0.62)
        self.both = cv2.resize(both, (int(both.shape[1] * h / both.shape[0]), h))
        half = self.both.shape[1] // 2
        self.left, self.right = self.both[:, :half], self.both[:, half:]   # left: upside-down hand, right: fingers up
        fist = rd('hand_fist.jpg')
        self.fist = cv2.resize(fist, (int(fist.shape[1] * h / fist.shape[0]), h))

    @staticmethod
    def paste(img, patch, cx, cy):
        ph, pw = patch.shape[:2]
        x0, y0 = int(round(cx - pw / 2)), int(round(cy - ph / 2))
        xa, ya, xb, yb = max(0, x0), max(0, y0), min(RW, x0 + pw), min(RH, y0 + ph)
        if xa < xb and ya < yb:
            img[ya:yb, xa:xb] = patch[ya - y0:yb - y0, xa - x0:xb - x0]

    # where the palm centre is inside each photo, measured once with MediaPipe: the photos are moved
    # by this much so the palm lands exactly on the app's circles (middle; 0.27 / 0.73 for two hands)
    OPEN, FIST = (-0.029, -0.100), (0.008, 0.052)              # part of the picture width / height
    TWO_R, TWO_L = (-0.047, -0.093), (0.049, 0.097)

    def at(self, img, patch, x, y, off):
        self.paste(img, patch, (x + off[0]) * RW, (y + off[1]) * RH)

    def frame(self, t):
        img = np.empty((RH, RW, 3), np.float32)
        img[:] = (236, 236, 233)                      # a light wall, like the photos
        if t < 3 or 10 <= t < 12:
            self.at(img, self.right, 0.5, 0.5, self.OPEN)
        elif t < 7:
            self.at(img, self.right, 0.5 + 0.26 * math.sin(math.pi * (t - 3) / 4), 0.5, self.OPEN)
        elif t < 10:
            self.at(img, self.fist, 0.5, 0.5, self.FIST)
        else:          # two hands; after 14 s the one on the right of the (mirrored) picture goes up
            up = 0.0 if t < 14 else min(1.0, (t - 14) / 1.5) * 0.22
            self.at(img, self.left, 0.27, 0.5 - up, self.TWO_L)     # camera's left = right of the mirrored picture
            self.at(img, self.right, 0.73, 0.5, self.TWO_R)
        return img


def to_yuv420_any(rgb):
    h, w = rgb.shape[:2]
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    u = 128 - 0.168736 * r - 0.331264 * g + 0.5 * b
    v = 128 + 0.5 * r - 0.418688 * g - 0.081312 * b
    sub = lambda c: c.reshape(h // 2, 2, w // 2, 2).mean(axis=(1, 3))     # noqa: E731
    return [np.clip(p, 0, 255).astype(np.uint8).tobytes() for p in (y, sub(u), sub(v))]


def main(path, real=False):
    w, h, secs = (RW, RH, RSECONDS) if real else (W, H, SECONDS)
    make = RealHands().frame if real else frame
    with open(path, 'wb') as f:
        f.write(f'YUV4MPEG2 W{w} H{h} F{FPS}:1 Ip A1:1 C420jpeg XYSCSS=420JPEG\n'.encode())
        for i in range(FPS * secs):
            f.write(b'FRAME\n')
            for plane in to_yuv420_any(make(i / FPS)):
                f.write(plane)
    print('wrote', path)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if a != '--real']
    main(args[0] if args else '/tmp/scara_hand.y4m', real='--real' in sys.argv)
