#!/usr/bin/env python3
"""
Forward / inverse kinematics for the SCARA, using the real link lengths from
arm.xacro and the measured offset from the wrist axis to the centre of the
finger pads.  Pure Python: used by the ROS 2 nodes, the MuJoCo apps and the
tests.

Planar model (wrist_joint kept at 0 for picking round pucks):

    grip_xy = column_axis + L1*[cos a1, sin a1] + L2_EFF*[cos(a1+t2), sin(a1+t2)]
    with a1 = BASE_YAW + t1   (the robot base is turned by BASE_YAW on the table)
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import math

try:
    import scara_scene as S
except ImportError:  # when imported as a package module
    from . import scara_scene as S


class IKError(ValueError):
    pass


def fk_xy(t1, t2, t3=0.0):
    """World XY of the finger-pad centre for column=t1, forearm=t2, wrist=t3."""
    cx, cy = S.COLUMN_AXIS_XY
    t1 = t1 + S.BASE_YAW
    ex = cx + S.L1 * math.cos(t1)
    ey = cy + S.L1 * math.sin(t1)
    wx = ex + S.L2 * math.cos(t1 + t2)
    wy = ey + S.L2 * math.sin(t1 + t2)
    a = t1 + t2 + t3
    return (wx + S.GRIP_OFFSET * math.cos(a), wy + S.GRIP_OFFSET * math.sin(a))


def pad_bottom_z(z_joint):
    return S.TABLE_TOP_Z + S.PAD_BOTTOM_AT_Z0 + z_joint


def within(name, v, margin=0.0):
    lo, hi = S.LIMITS[name]
    return lo + margin <= v <= hi - margin


def ik_xy(x, y, prefer=None, margin=0.02):
    """
    Returns (column, forearm) joint angles that put the pad centre at (x, y),
    with wrist_joint = 0.  Both elbow solutions are tried; the one closest to
    `prefer` (a (column, forearm) tuple) wins, otherwise the one with the
    smaller column rotation.
    """
    cx, cy = S.COLUMN_AXIS_XY
    dx, dy = x - cx, y - cy
    r2 = dx * dx + dy * dy
    a, b = S.L1, S.L2_EFF
    c2 = (r2 - a * a - b * b) / (2 * a * b)
    if c2 > 1.0 or c2 < -1.0:
        raise IKError(f'target ({x:.3f}, {y:.3f}) out of reach: r={math.sqrt(r2):.3f} m, '
                      f'reach {abs(a - b):.3f}..{a + b:.3f} m')
    sols = []
    for sgn in (1.0, -1.0):
        t2 = sgn * math.acos(c2)
        t1 = math.atan2(dy, dx) - math.atan2(b * math.sin(t2), a + b * math.cos(t2)) - S.BASE_YAW
        t1 = math.atan2(math.sin(t1), math.cos(t1))
        if within('column_joint', t1, margin) and within('forearm_joint', t2, margin):
            sols.append((t1, t2))
    if not sols:
        raise IKError(f'target ({x:.3f}, {y:.3f}) needs joints outside their limits')
    if prefer is not None:
        return min(sols, key=lambda s: abs(s[0] - prefer[0]) + abs(s[1] - prefer[1]))
    return min(sols, key=lambda s: abs(s[0]))


def ik_xy_yaw(x, y, yaw, prefer=None, margin=0.02, symmetry=math.pi / 2):
    """
    Pad centre at (x, y) AND gripper pointing along `yaw` (world angle of the
    wrist x axis).  The gripper and the bins are square-symmetric, so every
    yaw + k*symmetry is acceptable; the one needing the least wrist rotation
    (and closest to `prefer`) is chosen.  Returns (column, forearm, wrist).
    """
    best = None
    for k in range(-4, 5):
        psi = yaw + k * symmetry
        wx = x - S.GRIP_OFFSET * math.cos(psi)
        wy = y - S.GRIP_OFFSET * math.sin(psi)
        cx, cy = S.COLUMN_AXIS_XY
        dx, dy = wx - cx, wy - cy
        a, b = S.L1, S.L2
        c2 = (dx * dx + dy * dy - a * a - b * b) / (2 * a * b)
        if abs(c2) > 1.0:
            continue
        for sgn in (1.0, -1.0):
            t2 = sgn * math.acos(c2)
            t1 = math.atan2(dy, dx) - math.atan2(b * math.sin(t2), a + b * math.cos(t2)) - S.BASE_YAW
            t1 = math.atan2(math.sin(t1), math.cos(t1))
            t3 = psi - S.BASE_YAW - t1 - t2
            t3 = math.atan2(math.sin(t3), math.cos(t3))
            if not (within('column_joint', t1, margin) and within('forearm_joint', t2, margin)
                    and within('wrist_joint', t3, margin)):
                continue
            cost = abs(t3) * 0.5
            if prefer is not None:
                cost += abs(t1 - prefer[0]) + abs(t2 - prefer[1])
            if best is None or cost < best[0]:
                best = (cost, (t1, t2, t3))
    if best is None:
        raise IKError(f'no joint solution for ({x:.3f}, {y:.3f}) with yaw {math.degrees(yaw):.1f} deg')
    return best[1]


def reachable(x, y, margin=0.02):
    try:
        ik_xy(x, y, margin=margin)
        return True
    except IKError:
        return False


def arm_pose(x, y, z_joint, prefer=None, yaw=None, symmetry=math.pi / 2):
    """
    Full arm joint vector [column, shoulder(Z), forearm, wrist].
    yaw=None keeps the wrist at 0 (fine for round pucks); a yaw value also
    aligns the gripper, e.g. square to a bin's walls.
    """
    if not within('shoulder_joint', z_joint):
        raise IKError(f'Z={z_joint:.3f} outside shoulder_joint limits {S.LIMITS["shoulder_joint"]}')
    if yaw is None:
        t1, t2 = ik_xy(x, y, prefer)
        return [t1, z_joint, t2, 0.0]
    t1, t2, t3 = ik_xy_yaw(x, y, yaw, prefer, symmetry=symmetry)
    return [t1, z_joint, t2, t3]


if __name__ == '__main__':
    # Self-test: IK -> FK round trip over the whole workspace + key targets
    import random
    random.seed(1)
    worst = 0.0
    n = 0
    for _ in range(20000):
        t1 = random.uniform(-1.9, 1.9)
        t2 = random.uniform(-1.9, 1.9)
        x, y = fk_xy(t1, t2)
        try:
            s1, s2 = ik_xy(x, y, prefer=(t1, t2), margin=0.0)
        except IKError:
            continue
        fx, fy = fk_xy(s1, s2)
        worst = max(worst, math.hypot(fx - x, fy - y))
        n += 1
    print(f'IK/FK round trip: {n} samples, worst error {worst * 1000:.6f} mm')
    px, py = S.belt_point(sum(S.PICK_WINDOW_S) / 2)
    print('pick point', [round(v, 3) for v in arm_pose(px, py, S.Z_PICK)])
    for c in S.COLOR_ORDER:
        bx, by = S.bin_xy(c)
        q = arm_pose(bx, by, S.Z_PLACE, yaw=S.bin_yaw(c))
        px, py = fk_xy(q[0], q[2], q[3])
        yaw = math.degrees(S.BASE_YAW + q[0] + q[2] + q[3])
        print(f'{c:6s} bin', [round(v, 3) for v in q], f'pad err {1000 * math.hypot(px - bx, py - by):.4f} mm,',
              f'gripper yaw {yaw:.1f} vs bin {math.degrees(S.bin_yaw(c)):.1f} (mod 90)')
    for sb in (-0.10, -0.05, 0.0, 0.08, 0.15, 0.20, 0.25):
        print(f'belt s={sb:+.2f} reachable:', reachable(*S.belt_point(sb)))
