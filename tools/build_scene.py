#!/usr/bin/env python3
"""
Generates every scene file from common/scara_scene.py so Gazebo and MuJoCo
always match:

    gazebo/worlds/scara_world.sdf          Gazebo Harmonic world (table, conveyor,
                                    bins, floating RGB-D camera)
    mujoco/models/scara_scene_mujoco.xml     MuJoCo scene (same layout + the robot)
    meshes/puck_handle.stl          ring handle mesh used by the pucks

Run from the package root:   python3 tools/build_scene.py
"""
import math
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, d) for d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import scara_scene as S  # noqa: E402
from scara_models import puck_model  # noqa: E402

# camera_link.stl (wrist camera body) is modelled in millimetres around this point
CAM_MESH_CENTER_MM = (140.5, 0.0, 115.0)
ARM_CAM_LENS_AHEAD = 0.012          # lens sits on the front face of camera_link.stl (body is 21 mm deep)


def f(v):
    return f'{v:.5f}'.rstrip('0').rstrip('.') if abs(v) > 1e-12 else '0'


def vec(*vs):
    return ' '.join(f(v) for v in vs)


def rgba(c):
    return vec(*c)


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def gz_cam_to_mj_xyaxes(rpy):
    """Gazebo camera link (looks along +X, +Z up) -> MuJoCo camera xyaxes (looks along -Z)."""
    R = rpy_to_R(*rpy)
    Y = [R[0][1], R[1][1], R[2][1]]
    Z = [R[0][2], R[1][2], R[2][2]]
    return vec(*[-v for v in Y], *Z)


def R_cols_xyaxes(R):
    """MuJoCo xyaxes for a body whose local x/y axes are the first two columns of R."""
    return vec(R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1])


def lookat_xyaxes(pos, target):
    """MuJoCo camera xyaxes for a camera at pos looking at target (image up = world up)."""
    fwd = [t - p for t, p in zip(target, pos)]
    n = math.sqrt(sum(v * v for v in fwd))
    fwd = [v / n for v in fwd]
    right = [fwd[1], -fwd[0], 0.0]
    n = math.hypot(right[0], right[1])
    right = [v / n for v in right]
    up = [right[1] * fwd[2] - right[2] * fwd[1], right[2] * fwd[0] - right[0] * fwd[2], right[0] * fwd[1] - right[1] * fwd[0]]
    return vec(*right, *up)


def lookat_rpy(pos, target):
    """Gazebo pose rpy (camera looks along +X) for a camera at pos looking at target."""
    dx, dy, dz = (t - p for t, p in zip(target, pos))
    return (0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx))


# Viewing camera for the Gazebo GUI, the MuJoCo viewer and the demo videos:
# in front of the cell (the conveyor side), so the robot faces you.
VIEW_POS = (-1.55, -0.32, 1.28)
VIEW_TARGET = (0.02, 0.0, 0.47)
# closer camera for demo videos (MuJoCo viewer: Ctrl + camera list -> showcase)
SHOW_POS = (-1.00, -0.42, 1.12)
SHOW_TARGET = (0.06, 0.04, 0.56)


def fovy_deg(hfov, w, h):
    fx = (w / 2.0) / math.tan(hfov / 2.0)
    return math.degrees(2.0 * math.atan((h / 2.0) / fx))


def arm_cam_mesh_pose():
    """Pose of camera_link.stl in its parent link (S.ARM_CAM_PARENT): body centre at ARM_CAM_XYZ, axes = camera axes."""
    R = rpy_to_R(*S.ARM_CAM_RPY)
    off = [-c / 1000.0 for c in CAM_MESH_CENTER_MM]
    pos = [S.ARM_CAM_XYZ[i] + sum(R[i][k] * off[k] for k in range(3)) for i in range(3)]
    return pos


# --------------------------------------------------------------------------
# Puck handle mesh (torus standing upright on the puck)
# --------------------------------------------------------------------------
def write_torus_stl(path, R, r, nu=48, nv=16):
    tris = []
    cz = R + r

    def p(i, j):
        u = 2 * math.pi * (i % nu) / nu
        v = 2 * math.pi * (j % nv) / nv
        return ((R + r * math.cos(v)) * math.cos(u), r * math.sin(v), cz + (R + r * math.cos(v)) * math.sin(u))
    for i in range(nu):
        for j in range(nv):
            a, b, c, d = p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1)
            tris.append((a, b, c))
            tris.append((a, c, d))
    with open(path, 'wb') as fh:
        fh.write(b'scara puck handle'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(tris)))
        for t in tris:
            ux, uy, uz = (t[1][k] - t[0][k] for k in range(3))
            vx, vy, vz = (t[2][k] - t[0][k] for k in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            ln = math.sqrt(sum(c * c for c in n)) or 1.0
            fh.write(struct.pack('<3f', *(c / ln for c in n)))
            for v in t:
                fh.write(struct.pack('<3f', *v))
            fh.write(b'\x00\x00')


# --------------------------------------------------------------------------
# Shared geometry: (name, centre xyz, size xyz, rgba, collide)
# --------------------------------------------------------------------------
WOOD = (0.78, 0.55, 0.33, 1.0)
WOOD_DARK = (0.62, 0.42, 0.25, 1.0)
BELT = (0.28, 0.28, 0.30, 1.0)     # dark grey belt, light grey frame - like the videos
FRAME = (0.62, 0.63, 0.66, 1.0)
RAIL = (0.84, 0.85, 0.87, 1.0)


def table_boxes():
    tx, ty = S.TABLE_CENTER
    X, Y, T, P = S.TABLE_SIZE_X, S.TABLE_SIZE_Y, S.TABLE_TOP_THICKNESS, S.TABLE_PANEL_THICKNESS
    H = S.TABLE_TOP_Z
    leg_h = H - T
    out = [('table_top', (tx, ty, H - T / 2), (X, Y, T), WOOD, True)]
    # side panels at the left / right ends; the open side faces the viewer (-x)
    for side, sy in (('right', -1), ('left', 1)):
        out.append((f'table_panel_{side}', (tx, ty + sy * (Y / 2 - P / 2), leg_h / 2), (X, P, leg_h), WOOD_DARK, True))
    out.append(('table_shelf', (tx, ty, S.TABLE_SHELF_Z), (X, Y - 2 * P, 0.025), WOOD_DARK, True))
    return out


def conveyor_frame_boxes():
    y0, y1 = S.CONVEYOR_Y_RANGE
    L = y1 - y0
    cy = (y0 + y1) / 2
    x = S.CONVEYOR_X
    bed_top = S.BELT_TOP_Z - 0.021            # 1 mm below the moving belt
    bed_h = bed_top - S.TABLE_TOP_Z
    out = [('conveyor_bed', (x, cy, S.TABLE_TOP_Z + bed_h / 2), (S.BELT_WIDTH + 0.01, L, bed_h), FRAME, True)]
    rail_h = S.RAIL_HEIGHT + 0.021
    for side, sx in (('near', -1), ('far', 1)):
        rx = x + sx * (S.BELT_WIDTH / 2 + S.RAIL_WIDTH / 2 + 0.002)
        out.append((f'rail_{side}', (rx, cy, bed_top + rail_h / 2), (S.RAIL_WIDTH, L, rail_h), RAIL, True))
    # open ends (no bumpers): the belt runs from one edge of the table to the other
    return out


def conveyor_rollers():
    """(name, centre) of the two end rollers (cylinders across the belt), just inside the table edges."""
    y0, y1 = S.CONVEYOR_Y_RANGE
    r = 0.014
    return [(f'roller_{end}', (*S.belt_point(sv), S.BELT_TOP_Z - r)) for end, sv in
            (('start', y0 + r), ('end', y1 - r))], r


def bin_boxes(color, pose=None, rgba_=None):
    """Boxes of one bin: the colour bin of `color`, or a bin at pose=(x, y, yaw)."""
    bx, by, a = pose if pose else (*S.bin_xy(color), S.bin_yaw(color))
    s, w, h = S.BIN_SIZE, S.BIN_WALL, S.BIN_HEIGHT
    z0 = S.TABLE_TOP_Z
    c = rgba_ or S.COLORS[color]
    # floor: a 20 mm block sunk into the table top (thin plates give poor contacts); top at BIN_FLOOR
    parts = [('floor', (0, 0, S.BIN_FLOOR - 0.01), (s, s, 0.02))]
    for nm, (dx, dy, sx, sy) in {'wall_a': (s / 2 - w / 2, 0, w, s), 'wall_b': (-(s / 2 - w / 2), 0, w, s),
                                  'wall_c': (0, s / 2 - w / 2, s, w), 'wall_d': (0, -(s / 2 - w / 2), s, w)}.items():
        parts.append((nm, (dx, dy, h / 2), (sx, sy, h)))
    out = []
    for nm, (dx, dy, dz), size in parts:
        rx = dx * math.cos(a) - dy * math.sin(a)
        ry = dx * math.sin(a) + dy * math.cos(a)
        out.append((f'bin_{color}_{nm}', (bx + rx, by + ry, z0 + dz), size, a, c))
    return out


# --------------------------------------------------------------------------
# Gazebo world
# --------------------------------------------------------------------------
def sdf_box(name, center, size, color, collide=True, yaw=0.0, indent='      '):
    c = rgba(color)
    col = (f'{indent}<collision name="{name}_col"><pose>{vec(*center)} 0 0 {f(yaw)}</pose>'
           f'<geometry><box><size>{vec(*size)}</size></box></geometry></collision>\n') if collide else ''
    return (col + f'{indent}<visual name="{name}_vis"><pose>{vec(*center)} 0 0 {f(yaw)}</pose>'
            f'<geometry><box><size>{vec(*size)}</size></box></geometry>'
            f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.1 0.1 0.1 1</specular></material></visual>\n')


def cell_bins(kind):
    """[(model name, boxes)] for the sorting cell (4 colour bins) or the command cell (1 bin)."""
    if kind == 'command':
        return [('drop_bin', bin_boxes('drop', (*S.command_bin_xy(), S.command_bin_yaw()), S.COMMAND_BIN_COLOR))]
    return [(f'bin_{c}', bin_boxes(c)) for c in S.COLOR_ORDER]


def build_world(manual=False, kind=None):
    """kind 'sort'   : sorting cell (conveyor, 4 colour bins, depth camera)
       kind 'command': same cell with ONE bin (voice / typed colour commands)
       kind 'manual' : table + 3 blue and 3 green pucks for keyboard / joystick / gestures."""
    kind = kind or ('manual' if manual else 'sort')
    manual = kind == 'manual'
    y0, y1 = S.CONVEYOR_Y_RANGE
    L = y1 - y0
    belt_center = (S.CONVEYOR_X, (y0 + y1) / 2, S.BELT_TOP_Z - 0.01)
    w = []
    w.append('<?xml version="1.0"?>\n<!-- GENERATED by tools/build_scene.py from common/scara_scene.py - edit those, not this file -->\n')
    w.append('<sdf version="1.9">\n  <world name="scara_world">\n')
    w.append('''    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.85 0.85 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>@@VIEW_POSE@@</pose>
      </camera>
    </gui>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>
''')
    w.append('    <model name="table">\n      <static>true</static>\n      <link name="link">\n')
    for name, c, s, col, coll in table_boxes():
        w.append(sdf_box(name, c, s, col, coll))
    w.append('      </link>\n    </model>\n')
    if manual:
        for name, (x, y, yaw) in S.MANUAL_PUCKS:
            w.append(puck_model(S.puck_color(name), name, (x, y, S.TABLE_TOP_Z + 0.001, yaw), indent='  '))
        w.append('  </world>\n</sdf>\n')
        return ''.join(w).replace('@@VIEW_POSE@@', f'{vec(*VIEW_POS)} {vec(*lookat_rpy(VIEW_POS, VIEW_TARGET))}')
    w.append('    <model name="conveyor_frame">\n      <static>true</static>\n      <link name="link">\n')
    for name, c, s, col, coll in conveyor_frame_boxes():
        w.append(sdf_box(name, c, s, col, coll))
    rollers, rr = conveyor_rollers()
    for rname, (rx, ry, rz) in rollers:
        w.append(f'      <visual name="{rname}"><pose>{vec(rx, ry, rz)} 0 1.5708 0</pose>'
                 f'<geometry><cylinder><radius>{f(rr)}</radius><length>{f(S.BELT_WIDTH)}</length></cylinder></geometry>'
                 f'<material><ambient>{rgba(RAIL)}</ambient><diffuse>{rgba(RAIL)}</diffuse></material></visual>\n')
    w.append('      </link>\n    </model>\n')
    # Moving belt: TrackController moves the surface along the link's +X, so the
    # model is turned to make the pucks travel along BELT_DIR.
    belt_yaw = math.atan2(S.BELT_DIR[1], S.BELT_DIR[0])
    w.append(f'''    <model name="conveyor_belt">
      <!-- same set-up as Gazebo's own examples/worlds/conveyor.sdf: a STATIC model whose
           collision surface is moved by the TrackController (DART contact surface motion) -->
      <static>true</static>
      <pose>{vec(*belt_center)} 0 0 {f(belt_yaw)}</pose>
      <link name="belt_link">
        <collision name="belt_collision">
          <geometry><box><size>{vec(L, S.BELT_WIDTH, 0.02)}</size></box></geometry>
          <surface><friction><ode><mu>0.7</mu><mu2>150</mu2><fdir1>0 1 0</fdir1></ode></friction></surface>
        </collision>
        <visual name="belt_visual">
          <geometry><box><size>{vec(L, S.BELT_WIDTH, 0.02)}</size></box></geometry>
          <material><ambient>{rgba(BELT)}</ambient><diffuse>{rgba(BELT)}</diffuse><specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
      <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
        <link>belt_link</link>
        <odometry_publish_frequency>1</odometry_publish_frequency>
      </plugin>
    </model>
''')
    for model_name, boxes in cell_bins(kind):
        w.append(f'    <model name="{model_name}">\n      <static>true</static>\n      <link name="link">\n')
        for name, c, s, yaw, col in boxes:
            w.append(sdf_box(name, c, s, col, True, yaw))
        w.append('      </link>\n    </model>\n')
    # Floating RGB-D camera (no stand), like the videos
    cx, cy, cz = S.FIXED_CAM_XYZ
    w.append(f'''    <model name="fixed_camera">
      <static>true</static>
      <pose>{vec(cx, cy, cz)} {vec(*S.FIXED_CAM_RPY)}</pose>
      <link name="camera_link">
        <visual name="body"><pose>-0.015 0 0 0 0 0</pose><geometry><box><size>0.025 0.09 0.025</size></box></geometry>
          <material><ambient>0.15 0.15 0.17 1</ambient><diffuse>0.15 0.15 0.17 1</diffuse></material></visual>
        <visual name="lens"><pose>0.001 0.025 0 0 1.5708 0</pose><geometry><cylinder><radius>0.007</radius><length>0.004</length></cylinder></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material></visual>
        <visual name="lens2"><pose>0.001 -0.025 0 0 1.5708 0</pose><geometry><cylinder><radius>0.007</radius><length>0.004</length></cylinder></geometry>
          <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material></visual>
        <sensor name="fixed_rgbd" type="rgbd_camera">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <visualize>true</visualize>
          <topic>fixed_camera</topic>
          <camera name="fixed_camera">
            <horizontal_fov>{S.FIXED_CAM_HFOV}</horizontal_fov>
            <image><width>{S.FIXED_CAM_WIDTH}</width><height>{S.FIXED_CAM_HEIGHT}</height><format>R8G8B8</format></image>
            <clip><near>{S.CAM_NEAR}</near><far>{S.CAM_FAR}</far></clip>
            <depth_camera><clip><near>{S.CAM_NEAR}</near><far>{S.CAM_FAR}</far></clip></depth_camera>
          </camera>
        </sensor>
      </link>
    </model>
''')
    w.append('  </world>\n</sdf>\n')
    return ''.join(w).replace('@@VIEW_POSE@@', f'{vec(*VIEW_POS)} {vec(*lookat_rpy(VIEW_POS, VIEW_TARGET))}')


# --------------------------------------------------------------------------
# MuJoCo scene
# --------------------------------------------------------------------------
def mj_box(name, center, size, color, collide=True, yaw=0.0, indent='    '):
    half = vec(*(s / 2 for s in size))
    q = f' euler="0 0 {f(yaw)}"' if yaw else ''
    con = '' if collide else ' contype="0" conaffinity="0"'
    return f'{indent}<geom name="{name}" type="box" pos="{vec(*center)}" size="{half}"{q} rgba="{rgba(color)}"{con}/>\n'


def mj_puck(name, pos, yaw=0.0):
    color = S.puck_color(name)
    r, h = S.PUCK_RADIUS, S.PUCK_HEIGHT
    c = rgba(S.COLORS[color])
    return f'''    <body name="{name}" pos="{vec(*pos)}" euler="0 0 {f(yaw)}">
      <freejoint name="{name}_free"/>
      <geom name="{name}" type="cylinder" pos="0 0 {f(h / 2)}" size="{f(r)} {f(h / 2)}" mass="{S.PUCK_MASS}" rgba="{c}" friction="1.5 0.01 0.001" condim="4" contype="1" conaffinity="3"/>
      <geom class="visual" type="mesh" mesh="puck_handle" pos="0 0 {f(S.HANDLE_BASE_Z)}" rgba="{c}"/>
    </body>
'''


NAME_MESH = os.path.join(ROOT, 'meshes', 'shoulder_name.stl')   # optional (tools/make_name_mesh.py)


def build_mujoco(manual=False, kind=None):
    kind = kind or ('manual' if manual else 'sort')
    manual = kind == 'manual'
    bx, by, bz = S.BASE_XYZ
    m = []
    m.append('''<!-- GENERATED by tools/build_scene.py from common/scara_scene.py - edit those, not this file -->
<mujoco model="@@MODEL@@">
  <compiler angle="radian" meshdir="../../meshes"/>
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic" impratio="10">
    <flag multiccd="enable" nativeccd="enable"/>
  </option>
  <size memory="64M"/>

  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/>
    <global offwidth="1920" offheight="1080" azimuth="@@AZ@@" elevation="-28"/>
    <quality shadowsize="4096"/>
  </visual>

  <statistic center="@@CENTER@@" extent="0.9"/>

  <default>
    <geom friction="1.0 0.005 0.0001"/>
    <default class="visual"><geom contype="0" conaffinity="0" group="1" density="0"/></default>
  </default>

  <asset>
    <texture type="skybox" name="sky" builtin="gradient" rgb1="0.93 0.95 0.98" rgb2="0.70 0.76 0.84" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" rgb1="0.86 0.86 0.86" rgb2="0.78 0.78 0.78" width="512" height="512"/>
    <material name="groundplane" texture="groundplane" texrepeat="8 8" reflectance="0.05"/>
    <material name="scara_blue" rgba="0.0 0.4 0.8 1"/>
    <material name="scara_dark_blue" rgba="0.0 0.1 0.4 1"/>
    <material name="scara_grey" rgba="0.7 0.7 0.7 1"/>
    <material name="camera_grey" rgba="0.13 0.13 0.15 1"/>
    <material name="name_white" rgba="0.96 0.96 0.96 1" specular="0.3"/>
    <mesh name="base_link" file="base_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="column_link" file="column_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="shoulder_link" file="shoulder_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="forearm_link" file="forearm_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="wrist_link" file="wrist_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="left_finger_link" file="left_finger_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="right_finger_link" file="right_finger_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="camera_link" file="camera_link.stl" scale="0.001 0.001 0.001"/>
    <mesh name="puck_handle" file="puck_handle.stl"/>@@NAME_ASSET@@
  </asset>

  <worldbody>
    <geom name="floor" size="4 4 0.1" type="plane" material="groundplane"/>
    <light pos="0.6 -0.6 2.5" dir="-0.3 0.4 -1" directional="true" castshadow="true" diffuse="0.6 0.6 0.6"/>
''')
    m.append('    <!-- table -->\n')
    for name, c, s, col, coll in table_boxes():
        m.append(mj_box(name, c, s, col, coll))
    if manual:
        m.append('    <!-- no conveyor in the manual-control world; this hidden slide only keeps act_belt (7th actuator) -->\n')
        m.append('    <body name="belt_plate" pos="0 0 -1"><joint name="belt_joint" type="slide" axis="1 0 0" range="-0.1 3.0" damping="5"/>'
                 '<geom type="box" size="0.01 0.01 0.01" mass="0.1" contype="0" conaffinity="0" rgba="0 0 0 0"/></body>\n')
    for name, c, s, col, coll in ([] if manual else conveyor_frame_boxes()):
        m.append(mj_box(name, c, s, col, coll))
    y0, y1 = S.CONVEYOR_Y_RANGE
    if not manual:
        rollers, rr = conveyor_rollers()
        for rname, (rx, ry, rz) in rollers:
            m.append(f'    <geom name="{rname}" class="visual" type="cylinder" pos="{vec(rx, ry, rz)}" euler="0 1.5708 0" '
                     f'size="{f(rr)} {f(S.BELT_WIDTH / 2)}" rgba="{rgba(RAIL)}"/>\n')
        m.append(mj_box('belt', (S.CONVEYOR_X, (y0 + y1) / 2, S.BELT_TOP_Z - 0.0105), (S.BELT_WIDTH, y1 - y0, 0.02), BELT, False))
    # Moving belt surface: a long invisible plate on a slide joint, driven by a
    # velocity actuator. Pucks ride on it by friction, exactly like a real belt.
    # The plate's front edge sits at the end of the belt; the apps slide it back by
    # BELT_WRAP whenever it has moved that far (it is uniform, so the pucks on it do not
    # notice) - the belt runs forever and pucks still drop off the end, like the videos.
    plate_half = (y1 - y0) / 2 + 0.05
    bdx, bdy = S.BELT_DIR
    cell_parts = []
    pcx, pcy = S.belt_point(0.0)
    s_end = (y1 - y0) / 2                       # belt end, measured along the belt
    off = s_end - plate_half                    # plate's front edge at the belt end
    size_x = abs(bdx) * plate_half + abs(bdy) * S.BELT_WIDTH / 2
    size_y = abs(bdy) * plate_half + abs(bdx) * S.BELT_WIDTH / 2
    cell_parts.append(f'''    <body name="belt_plate" pos="{vec(pcx, pcy, S.BELT_TOP_Z - 0.01)}">
      <joint name="belt_joint" type="slide" axis="{vec(bdx, bdy, 0)}" range="-0.1 3.0" damping="5"/>
      <geom name="belt_surface" type="box" pos="{vec(bdx * off, bdy * off, 0)}" size="{vec(size_x, size_y, 0.01)}" friction="1.2 0.01 0.001" rgba="0.2 0.2 0.2 0" mass="2" contype="2" conaffinity="2"/>
    </body>
''')
    cell_parts.append('    <!-- bins -->\n')
    for _, boxes in cell_bins(kind):
        for name, c, s, yaw, col in boxes:
            cell_parts.append(mj_box(name, c, s, col, True, yaw))
    cell_parts.append('    <!-- floating RGB-D camera -->\n')
    fy = fovy_deg(S.FIXED_CAM_HFOV, S.FIXED_CAM_WIDTH, S.FIXED_CAM_HEIGHT)
    R = rpy_to_R(*S.FIXED_CAM_RPY)
    back = [S.FIXED_CAM_XYZ[i] - 0.015 * R[i][0] for i in range(3)]
    cell_parts.append(f'    <camera name="fixed_camera" pos="{vec(*S.FIXED_CAM_XYZ)}" xyaxes="{gz_cam_to_mj_xyaxes(S.FIXED_CAM_RPY)}" fovy="{fy:.3f}"/>\n')
    cell_parts.append(f'    <geom class="visual" type="box" pos="{vec(*back)}" xyaxes="{R_cols_xyaxes(R)}" size="0.0125 0.045 0.0125" material="camera_grey"/>\n')
    if not manual:
        m.extend(cell_parts)
    m.append(f'    <camera name="overview" pos="{vec(*VIEW_POS)}" xyaxes="{lookat_xyaxes(VIEW_POS, VIEW_TARGET)}" fovy="42"/>\n')
    m.append(f'    <camera name="showcase" pos="{vec(*SHOW_POS)}" xyaxes="{lookat_xyaxes(SHOW_POS, SHOW_TARGET)}" fovy="45"/>\n')

    afy = fovy_deg(S.ARM_CAM_HFOV, S.ARM_CAM_WIDTH, S.ARM_CAM_HEIGHT)
    cam_mesh_pos = arm_cam_mesh_pose()
    Rc = rpy_to_R(*S.ARM_CAM_RPY)
    lens = [S.ARM_CAM_XYZ[i] + ARM_CAM_LENS_AHEAD * Rc[i][0] for i in range(3)]   # just outside the camera body
    m.append(f'''
    <!-- SCARA robot (same joint chain / offsets as urdf/arm.xacro) -->
    <body name="base_link" pos="{vec(bx, by, bz)}" euler="0 0 {f(S.BASE_YAW)}">
      <geom type="mesh" mesh="base_link" material="scara_blue"/>
      <body name="column_link" pos="0.045 0 0.089" gravcomp="1">
        <joint name="column_joint" axis="0 0 1" range="-2.0 2.0" damping="10" armature="0.05"/>
        <geom type="mesh" mesh="column_link" pos="-0.045 0 -0.089" material="scara_blue" contype="0" conaffinity="0"/>
        <inertial pos="0 0 0.2" mass="4.0" diaginertia="0.01 0.01 0.01"/>
        <body name="shoulder_link" pos="0 0 0.28" gravcomp="1">
          <joint name="shoulder_joint" type="slide" axis="0 0 1" range="-0.15 0.02" damping="50" armature="0.1"/>
          <geom type="mesh" mesh="shoulder_link" pos="-0.059954333 -0.059974857 -0.390000" material="scara_blue" contype="0" conaffinity="0"/>@@NAME_GEOM@@
          <inertial pos="0.1 0 0" mass="3.0" diaginertia="0.01 0.01 0.01"/>
          <body name="forearm_link" pos="0.228 0 -0.04" gravcomp="1">
            <joint name="forearm_joint" axis="0 0 1" range="-2.0 2.0" damping="10" armature="0.05"/>
            <geom type="mesh" mesh="forearm_link" pos="-0.273 0 -0.351" material="scara_blue" contype="0" conaffinity="0"/>
            <inertial pos="0.07 0 -0.03" mass="2.0" diaginertia="0.01 0.01 0.01"/>
            <!-- arm camera: camera_link.stl on a bracket at the front of the forearm (does not turn with the wrist) -->
            <camera name="arm_camera" pos="{vec(*lens)}" xyaxes="{gz_cam_to_mj_xyaxes(S.ARM_CAM_RPY)}" fovy="{afy:.3f}"/>
            <geom class="visual" type="mesh" mesh="camera_link" pos="{vec(*cam_mesh_pos)}" euler="{vec(*S.ARM_CAM_RPY)}" material="camera_grey"/>
            <geom class="visual" type="box" pos="0.1875 0 -0.006" size="0.0055 0.016 0.012" material="scara_dark_blue"/>
            <body name="wrist_link" pos="0.1365 0 -0.06" gravcomp="1">
              <joint name="wrist_joint" axis="0 0 1" range="-4.712 4.712" damping="5" armature="0.02"/>
              <geom type="mesh" mesh="wrist_link" pos="-0.4095 0 -0.291" material="scara_dark_blue" contype="0" conaffinity="0"/>
              <inertial pos="0 0 -0.03" mass="1.5" diaginertia="0.01 0.01 0.01"/>
              <site name="tcp" pos="{f(S.GRIP_OFFSET)} 0 -0.0985" size="0.004" rgba="1 0 1 1" group="4"/>
              <body name="left_finger_link" pos="0 0.064 -0.0135" gravcomp="1">
                <joint name="left_finger_joint" type="slide" axis="0 1 0" range="-0.06 0.0" damping="10" armature="0.02"/>
                <geom class="visual" type="mesh" mesh="left_finger_link" pos="-0.409 -0.0626 -0.2775" material="scara_grey"/>
                <geom name="left_pad" type="box" pos="0.03185 -0.00485 -0.085" size="0.04595 0.01345 0.01" friction="1.5 0.01 0.001" condim="4" rgba="0.7 0.7 0.7 0"/>
                <geom name="left_bar" type="box" pos="-0.00095 -0.00725 -0.0325" size="0.01335 0.01405 0.0425" rgba="0.7 0.7 0.7 0"/>
                <inertial pos="0.02 0 -0.05" mass="0.2" diaginertia="0.001 0.001 0.001"/>
              </body>
              <body name="right_finger_link" pos="0 -0.064 -0.0135" gravcomp="1">
                <joint name="right_finger_joint" type="slide" axis="0 -1 0" range="-0.06 0.0" damping="10" armature="0.02"/>
                <geom class="visual" type="mesh" mesh="right_finger_link" pos="-0.409 0.0626 -0.2775" material="scara_grey"/>
                <geom name="right_pad" type="box" pos="0.0350 0.00605 -0.085" size="0.0463 0.01255 0.01" friction="1.5 0.01 0.001" condim="4" rgba="0.7 0.7 0.7 0"/>
                <geom name="right_bar" type="box" pos="0.00195 0.00825 -0.0325" size="0.01335 0.01335 0.0425" rgba="0.7 0.7 0.7 0"/>
                <inertial pos="0.02 0 -0.05" mass="0.2" diaginertia="0.001 0.001 0.001"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
''')
    if manual:
        m.append('    <!-- pucks on the table, like Parts 6-7 of the Mechamind series -->\n')
        for name, (x, y, yaw) in S.MANUAL_PUCKS:
            m.append(mj_puck(name, (x, y, S.TABLE_TOP_Z + 0.001), yaw))
    else:
        m.append('    <!-- pucks: wait far away out of sight; common/scara_feeder.py drops them on the belt one by one -->\n')
        for i, name in enumerate(S.puck_names()):
            m.append(mj_puck(name, S.hidden_xyz(i)))
    m.append('''  </worldbody>

  <!-- Same actuator order as before so all existing MuJoCo scripts keep working -->
  <actuator>
    <position name="act_column" joint="column_joint" kp="600" kv="60" ctrlrange="-2.0 2.0" forcerange="-150 150"/>
    <position name="act_shoulder" joint="shoulder_joint" kp="4000" kv="400" ctrlrange="-0.15 0.02"/>
    <position name="act_forearm" joint="forearm_joint" kp="500" kv="50" ctrlrange="-2.0 2.0"/>
    <position name="act_wrist" joint="wrist_joint" kp="200" kv="20" ctrlrange="-4.712 4.712"/>
    <position name="act_left_finger" joint="left_finger_joint" kp="300" kv="30" ctrlrange="-0.06 0.0"/>
    <position name="act_right_finger" joint="right_finger_joint" kp="300" kv="30" ctrlrange="-0.06 0.0"/>
    <velocity name="act_belt" joint="belt_joint" kv="400" ctrlrange="-0.2 0.2"/>
  </actuator>
</mujoco>
''')
    az = math.degrees(math.atan2(VIEW_TARGET[1] - VIEW_POS[1], VIEW_TARGET[0] - VIEW_POS[0]))
    named = os.path.exists(NAME_MESH)       # named robot: raised name on the shoulder link
    name_asset = '\n    <mesh name="shoulder_name" file="shoulder_name.stl" scale="0.001 0.001 0.001"/>' if named else ''
    name_geom = ('\n          <geom class="visual" type="mesh" mesh="shoulder_name" pos="-0.059954333 -0.059974857 -0.390000"'
                 ' material="name_white"/>') if named else ''
    return (''.join(m).replace('@@NAME_ASSET@@', name_asset).replace('@@NAME_GEOM@@', name_geom)
            .replace('@@AZ@@', f'{az:.0f}').replace('@@CENTER@@', vec(*VIEW_TARGET))
            .replace('@@MODEL@@', {'manual': 'scara_manual_table', 'command': 'scara_command_cell'}.get(kind, 'scara_sorting_cell')))


def main():
    os.makedirs(os.path.join(ROOT, 'gazebo', 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(ROOT, 'mujoco', 'models'), exist_ok=True)
    write_torus_stl(os.path.join(ROOT, 'meshes', 'puck_handle.stl'), S.HANDLE_RING_RADIUS, S.HANDLE_TUBE_RADIUS)
    out = {('gazebo/worlds', 'scara_world.sdf'): build_world(kind='sort'),
           ('gazebo/worlds', 'scara_command_world.sdf'): build_world(kind='command'),
           ('gazebo/worlds', 'scara_manual_world.sdf'): build_world(kind='manual'),
           ('mujoco/models', 'scara_scene_mujoco.xml'): build_mujoco(kind='sort'),
           ('mujoco/models', 'scara_command_mujoco.xml'): build_mujoco(kind='command'),
           ('mujoco/models', 'scara_manual_mujoco.xml'): build_mujoco(kind='manual')}
    for (d, n), text in out.items():
        with open(os.path.join(ROOT, d, n), 'w') as fh:
            fh.write(text)
    print('wrote', ', '.join(f'{d}/{n}' for d, n in out), '+ meshes/puck_handle.stl')
    print('URDF arm camera: joint origin xyz', vec(*S.ARM_CAM_XYZ), 'rpy', vec(*S.ARM_CAM_RPY),
          '| mesh origin (in camera frame)', vec(*[-c / 1000 for c in CAM_MESH_CENTER_MM]))


if __name__ == '__main__':
    main()
