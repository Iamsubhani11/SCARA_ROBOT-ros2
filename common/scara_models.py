#!/usr/bin/env python3
"""
SDF snippets for objects that are spawned at run time (pucks).
Shared by tools/build_scene.py and the puck spawner node.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
try:
    import scara_scene as S
except ImportError:
    from . import scara_scene as S

HANDLE_MESH_URI = 'package://scara_description/meshes/puck_handle.stl'


def _rgba(c):
    return ' '.join(f'{v:.3f}' for v in c)


def puck_model(color, name=None, pose=None, indent=''):
    """<model> element for one puck (70 mm disc + small decorative ring handle; the gripper holds the disc).
    pose = (x, y, z, yaw) places it in a world file."""
    rgba = _rgba(S.COLORS[color])
    name = name or f'puck_{color}'
    r, h, m = S.PUCK_RADIUS, S.PUCK_HEIGHT, S.PUCK_MASS
    ixx = m * (3 * r * r + h * h) / 12.0
    izz = m * r * r / 2.0
    pose_xml = f'\n    <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} 0 0 {pose[3]:.4f}</pose>' if pose else ''
    surf = """<surface>
          <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
          <contact><ode><kp>1e5</kp><kd>10</kd><max_vel>0.1</max_vel><min_depth>0.0005</min_depth></ode></contact>
        </surface>"""
    xml = f"""  <model name="{name}">{pose_xml}
    <link name="link">
      <inertial>
        <pose>0 0 {h / 2:.4f} 0 0 0</pose>
        <mass>{m}</mass>
        <inertia><ixx>{ixx:.3e}</ixx><iyy>{ixx:.3e}</iyy><izz>{izz:.3e}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="body">
        <pose>0 0 {h / 2:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        {surf}
      </collision>
      <visual name="body">
        <pose>0 0 {h / 2:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
      <visual name="handle">
        <pose>0 0 {S.HANDLE_BASE_Z:.4f} 0 0 0</pose>
        <geometry><mesh><uri>{HANDLE_MESH_URI}</uri></mesh></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual>
    </link>
  </model>
"""
    if indent:
        xml = ''.join(indent + line if line.strip() else line for line in xml.splitlines(True))
    return xml


def puck_sdf(color, name=None):
    """Complete SDF document for one puck (used by the spawner)."""
    return '<?xml version="1.0"?>\n<sdf version="1.9">\n' + puck_model(color, name) + '</sdf>\n'
