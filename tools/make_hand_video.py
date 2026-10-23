#!/usr/bin/env python3
"""A synthetic "camera" video of a hand for the gesture-control test (tools/test_app_v3.py
--hand-video): Chrome plays it as the webcam.

    0 - 6 s   open hand in the middle (the app learns it in the first 2 s)
    6 - 10 s  the hand moves to the side and back      -> the arm turns
    10 - 14 s a fist (smaller), moving a little         -> the gripper closes
    14 - 20 s open hand in the middle again

    python3 tools/make_hand_video.py /tmp/hand.y4m"""
import math
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


def to_yuv420(rgb):
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    u = 128 - 0.168736 * r - 0.331264 * g + 0.5 * b
    v = 128 + 0.5 * r - 0.418688 * g - 0.081312 * b
    sub = lambda c: c.reshape(H // 2, 2, W // 2, 2).mean(axis=(1, 3))     # noqa: E731
    return [np.clip(p, 0, 255).astype(np.uint8).tobytes() for p in (y, sub(u), sub(v))]


def main(path):
    with open(path, 'wb') as f:
        f.write(f'YUV4MPEG2 W{W} H{H} F{FPS}:1 Ip A1:1 C420jpeg XYSCSS=420JPEG\n'.encode())
        for i in range(FPS * SECONDS):
            f.write(b'FRAME\n')
            for plane in to_yuv420(frame(i / FPS)):
                f.write(plane)
    print('wrote', path)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/tmp/scara_hand.y4m')
