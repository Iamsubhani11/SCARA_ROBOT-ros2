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
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
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


def jacobian(t1, t2):
    """2 x 2 Jacobian of the pad centre (x, y) with respect to (column, forearm), wrist at 0.

    It tells how fast the pad moves for a small turn of each joint:
        [dx, dy] = J * [dt1, dt2]
    Its determinant is L1 * L2_EFF * sin(t2): zero when the arm is straight (t2 = 0), the
    singularity where the pad cannot move along the arm any more (the app's manipulability)."""
    a1 = t1 + S.BASE_YAW
    a12 = a1 + t2
    l1, l2 = S.L1, S.L2_EFF
    return [[-l1 * math.sin(a1) - l2 * math.sin(a12), -l2 * math.sin(a12)],
            [l1 * math.cos(a1) + l2 * math.cos(a12), l2 * math.cos(a12)]]


def joint_rates(t1, t2, vx, vy, min_sin=0.15):
    """Joint speeds (column, forearm) that move the pad at (vx, vy) m/s: q_dot = J^-1 * v.
    Raises IKError near the straight-arm singularity, where J^-1 would ask for huge speeds."""
    if abs(math.sin(t2)) < min_sin:
        raise IKError(f'too close to the straight-arm singularity (sin t2 = {math.sin(t2):.2f})')
    (a, b), (c, d) = jacobian(t1, t2)
    det = a * d - b * c
    return ((d * vx - b * vy) / det, (-c * vx + a * vy) / det)


def track_line(t1, t2, p0, v, T, dt=0.02, margin=0.02):
    """Follow a point that starts at p0 = (x, y) and moves at v = (vx, vy) m/s for T s
    (a puck on the running belt), with the Jacobian - "resolved-rate" control:

        q(k+1) = q(k) + J^-1(q(k)) * (v + (p_wanted(k) - p_now(k)) / dt) * dt

    The first part is the belt speed turned into joint speeds; the second part pulls the
    pad back onto the puck if the small steps have drifted. Returns a list of (t, t1, t2)
    every dt seconds. Raises IKError if a joint would leave its limits or the arm would
    come too close to a singularity on the way."""
    out = [(0.0, t1, t2)]
    n = max(1, int(round(T / dt)))
    for k in range(1, n + 1):
        t = k * dt
        want = (p0[0] + v[0] * t, p0[1] + v[1] * t)
        now = fk_xy(t1, t2)
        ex, ey = (want[0] - now[0]) / dt, (want[1] - now[1]) / dt
        w1, w2 = joint_rates(t1, t2, ex, ey)
        t1, t2 = t1 + w1 * dt, t2 + w2 * dt
        if not (within('column_joint', t1, margin) and within('forearm_joint', t2, margin)):
            raise IKError('following the belt would take a joint past its limit')
        out.append((t, t1, t2))
    return out


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
    fails = []
    if not (n > 5000 and worst < 1e-6):
        fails.append('IK/FK round trip')
    # the Jacobian against small finite differences of fk_xy
    jworst = 0.0
    for _ in range(2000):
        t1, t2, h = random.uniform(-1.8, 1.8), random.uniform(-1.8, 1.8), 1e-6
        J = jacobian(t1, t2)
        x0, y0 = fk_xy(t1, t2)
        for col, (d1, d2) in enumerate(((h, 0.0), (0.0, h))):
            x1, y1 = fk_xy(t1 + d1, t2 + d2)
            jworst = max(jworst, abs((x1 - x0) / h - J[0][col]), abs((y1 - y0) / h - J[1][col]))
    print(f'Jacobian vs finite differences: worst {jworst:.2e}')
    if jworst > 1e-4:
        fails.append('Jacobian')
    # follow a puck on the running belt with the Jacobian, compare with exact IK
    s0, T = -0.12, 2.6
    p0 = S.belt_point(s0)
    v = (S.BELT_DIR[0] * S.BELT_SPEED, S.BELT_DIR[1] * S.BELT_SPEED)
    q1, q2 = ik_xy(*p0)
    path = track_line(q1, q2, p0, v, T)
    terr = max(math.hypot(fk_xy(a, b)[0] - (p0[0] + v[0] * t), fk_xy(a, b)[1] - (p0[1] + v[1] * t)) for t, a, b in path)
    print(f'belt tracking with the Jacobian: {len(path)} steps over {T} s, worst pad error {terr * 1000:.3f} mm')
    if terr > 0.0005:
        fails.append('belt tracking')
    try:
        track_line(0.0, 0.01, fk_xy(0.0, 0.01), (0.05, 0.0), 1.0)
        fails.append('singularity not refused')
    except IKError:
        print('straight arm (singularity): tracking refused, as it should be')
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
    for f in fails:
        print('FAIL', f)
    print('ALL KINEMATICS CHECKS PASSED' if not fails else f'{len(fails)} FAILED')
    raise SystemExit(1 if fails else 0)
