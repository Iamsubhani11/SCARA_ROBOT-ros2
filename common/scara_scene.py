#!/usr/bin/env python3
"""
Single source of truth for the SCARA work-cell geometry.

Every number used by the Gazebo world, the MuJoCo scene, the vision node,
the inverse kinematics and the sorting logic comes from this file, so the
simulators and the code can never disagree about where things are.

Layout (same as the SCARA Digital Twin app and the Mechamind Labs videos):

  * the robot base stands on the table, the conveyor runs ACROSS the table
    IN FRONT of the robot, so the arm reaches straight out over the belt to pick
  * the colour bins are on the table beside / behind the robot
  * a depth camera floats in the air behind the robot, looking over it at the belt
  * an eye-in-hand camera (camera_link.stl) sits on top of the wrist housing

The cell is described in a "cell frame" (robot at x=-0.20 looking along +x,
conveyor at x=+0.20 running along +y) and then turned by CELL_YAW about the
table centre.  CELL_YAW = pi makes the robot, conveyor and bins FACE THE
VIEWER: the Gazebo / MuJoCo cameras look from -x, so the conveyor is in the
foreground running left-right and the robot stands behind it facing you.
World frame: Z up, metres / radians.
Robot numbers were measured from the STL meshes through the URDF joint chain.
"""
import math


def pkg_file(*parts):
    """A file of this package (urdf/, gui/, config/ ...), from the source tree or a colcon install
    (scripts are installed to lib/scara_description, data to share/scara_description)."""
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, '..', *parts),
             os.path.join(here, '..', '..', 'share', 'scara_description', *parts)]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(os.path.join(get_package_share_directory('scara_description'), *parts))
    except Exception:
        pass
    for c in cands:
        if os.path.exists(c):
            return os.path.abspath(c)
    return os.path.abspath(cands[0])

# --------------------------------------------------------------------------
# Table (wooden "coffee table")
# --------------------------------------------------------------------------
TABLE_TOP_Z = 0.40
# The table is turned so its open side (the shelf you can see into) faces the
# viewer, like the robot and the conveyor: side panels at the left / right ends.
TABLE_SIZE_X = 0.90                  # depth (towards the viewer)
TABLE_SIZE_Y = 1.20                  # width (left-right, along the conveyor)
TABLE_CENTER = (0.0, 0.0)
TABLE_TOP_THICKNESS = 0.04
TABLE_PANEL_THICKNESS = 0.04
TABLE_SHELF_Z = 0.06

# --------------------------------------------------------------------------
# Orientation of the whole cell (robot + conveyor + bins + camera)
# --------------------------------------------------------------------------
CELL_YAW = math.pi          # 0 = robot at -x facing away from the default view; pi = facing the viewer


def _clean(v):
    return 0.0 if abs(v) < 1e-12 else round(v, 12)


def cell_xy(x, y):
    """Cell-frame point -> world XY."""
    c, s = math.cos(CELL_YAW), math.sin(CELL_YAW)
    return (_clean(c * x - s * y), _clean(s * x + c * y))


def cell_inv(x, y):
    """World XY -> cell-frame point."""
    c, s = math.cos(CELL_YAW), math.sin(CELL_YAW)
    return (c * x + s * y, -s * x + c * y)


# --------------------------------------------------------------------------
# Robot mounting (base_link origin; identical to the original table_to_base_joint)
# --------------------------------------------------------------------------
BASE_XYZ = (*cell_xy(-0.20, 0.0), TABLE_TOP_Z)
BASE_YAW = CELL_YAW                                      # robot's +x points at the conveyor
COLUMN_AXIS_XY = cell_xy(-0.20 + 0.045, 0.0)             # column_joint origin is +0.045 in base x

# Arm geometry (from arm.xacro)
L1 = 0.228                  # column axis -> forearm_joint axis
L2 = 0.1365                 # forearm_joint axis -> wrist_joint axis
GRIP_OFFSET = 0.0334        # wrist axis -> centre of the finger pads (along wrist x)
L2_EFF = L2 + GRIP_OFFSET   # used by IK with wrist_joint = 0

# Bottom of the finger pads above the table top when shoulder_joint (Z) = 0.
PAD_BOTTOM_AT_Z0 = 0.1605
PAD_HEIGHT = 0.020

# Gripper: each finger moves by q (negative = inwards = closing).
# Gap between the finger pads = GRIPPER_GAP_OPEN + 2*q
GRIPPER_GAP_OPEN = 0.091
GRIPPER_OPEN = 0.0
GRIPPER_TOUCH = -0.0455     # fingers touch each other (nothing held)

LIMITS = {
    'column_joint': (-2.0, 2.0),
    'shoulder_joint': (-0.15, 0.02),
    'forearm_joint': (-2.0, 2.0),
    'wrist_joint': (-4.712, 4.712),
    'left_finger_joint': (-0.06, 0.0),
    'right_finger_joint': (-0.06, 0.0),
}
ARM_JOINTS = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']
GRIPPER_JOINTS = ['left_finger_joint', 'right_finger_joint']

# --------------------------------------------------------------------------
# Conveyor: in front of the robot, running across the table
# --------------------------------------------------------------------------
_CONVEYOR_CELL_X = 0.20              # belt centre line: 0.355 m in front of the column
CONVEYOR_X = cell_xy(_CONVEYOR_CELL_X, 0.0)[0]
CONVEYOR_Y_RANGE = (-TABLE_SIZE_Y / 2, TABLE_SIZE_Y / 2)   # edge to edge of the table, like the videos
BELT_DIR = cell_xy(0.0, 1.0)         # world direction the pucks travel
BELT_WIDTH = 0.20
BELT_TOP_Z = TABLE_TOP_Z + 0.035
RAIL_WIDTH = 0.010                   # thin light-grey side walls of the conveyor frame
RAIL_HEIGHT = 0.012                  # above the belt top
BELT_SPEED = 0.08                    # m/s while running
BELT_WRAP = 0.02                     # MuJoCo belt plate is slid back by this much whenever it moved this far
PICK_WINDOW_S = (-0.06, 0.04)        # belt stops when a puck enters this stretch (in front of the robot)
END_STOP_S = CONVEYOR_Y_RANGE[1]     # no bumper: pucks nobody asked for drop off the end, like the videos
SPAWN_S = CONVEYOR_Y_RANGE[0] + 0.09   # new pucks are put on at the start of the belt (120 mm puck fully on it)


def belt_s(x, y):
    """Position along the belt (in the direction pucks travel)."""
    return cell_inv(x, y)[1]


def belt_lat(x, y):
    """Sideways offset from the belt centre line."""
    return cell_inv(x, y)[0] - _CONVEYOR_CELL_X


def belt_point(s, lat=0.0):
    return cell_xy(_CONVEYOR_CELL_X + lat, s)


# --------------------------------------------------------------------------
# Pucks: discs with a small ring handle on top, like the Mechamind Labs videos.
# Sized to FIT INSIDE the open gripper: the pads are 91 mm apart when open, the
# puck is 70 mm wide - about 1 cm of room on each side, so the fingers come
# down around the puck and close on its body, exactly like in the videos.
# Puck frame: origin at the centre of the bottom face; the (decorative) handle
# ring stands in the puck's x-z plane.
# --------------------------------------------------------------------------
PUCK_RADIUS = 0.035
PUCK_HEIGHT = 0.035
PUCK_MASS = 0.08
HANDLE_RING_RADIUS = 0.011           # ring centre-line radius (27 mm tall loop, looks only)
HANDLE_TUBE_RADIUS = 0.0025
HANDLE_BASE_Z = PUCK_HEIGHT - HANDLE_TUBE_RADIUS            # ring sits slightly sunk into the top face
HANDLE_CENTER_Z = HANDLE_BASE_Z + HANDLE_RING_RADIUS + HANDLE_TUBE_RADIUS
HANDLE_OUTER_R = HANDLE_RING_RADIUS + HANDLE_TUBE_RADIUS    # 13.5 mm
HANDLE_TOP_Z = HANDLE_CENTER_Z + HANDLE_OUTER_R             # 60 mm above the bottom
HANDLE_THICKNESS = 2 * HANDLE_TUBE_RADIUS
COLORS = {
    'red':    (0.85, 0.10, 0.10, 1.0),
    'green':  (0.12, 0.75, 0.20, 1.0),
    'blue':   (0.08, 0.30, 0.90, 1.0),
    'yellow': (0.95, 0.82, 0.08, 1.0),
}
COLOR_ORDER = ['red', 'green', 'blue', 'yellow']
MIN_PUCK_SPACING = 0.12              # 70 mm pucks + room for the open fingers

# Puck pool for MuJoCo: every puck waits far away out of sight (nothing parked
# near the cell) and is dropped on the belt only when it is its turn.
PUCKS_PER_COLOR = 2
HIDDEN_X = 25.0                      # x beyond HIDDEN_X / 2 means "in the pool"


def puck_names():
    return [f'puck_{c}_{i + 1}' for i in range(PUCKS_PER_COLOR) for c in COLOR_ORDER]


def puck_color(name):
    """'puck_red_2' / 'puck_red_007' / 'puck_red' -> 'red'"""
    return name.split('_')[1]


def hidden_xyz(i):
    return (HIDDEN_X + 0.2 * i, HIDDEN_X, 0.001)


def is_hidden(x, y):
    return x > HIDDEN_X / 2


# Manual-control world (keyboard / joystick / gestures, like Parts 6-7 of the
# series): no conveyor, three blue and three green pucks on the table in front
# of the robot, handles turned towards the robot so they are easy to grab.
def _manual_puck(cell_x, cell_y):
    x, y = cell_xy(cell_x, cell_y)
    cx, cy = cell_xy(-0.155, 0.0)
    return (x, y, math.atan2(y - cy, x - cx))


# All six lie on an arc 31 cm from the column, 25 deg apart (13 cm, so the open
# fingers never touch a neighbour): the comfortable middle of the workspace,
# reachable with the wrist straight - nothing needs awkward joint angles.
MANUAL_ARC_R = 0.31


def _manual_arc(angle_deg):
    a = math.radians(angle_deg)
    return _manual_puck(-0.155 + MANUAL_ARC_R * math.cos(a), MANUAL_ARC_R * math.sin(a))


MANUAL_PUCKS = [
    ('puck_blue_1', _manual_arc(-62.5)), ('puck_blue_2', _manual_arc(-37.5)), ('puck_blue_3', _manual_arc(-12.5)),
    ('puck_green_1', _manual_arc(12.5)), ('puck_green_2', _manual_arc(37.5)), ('puck_green_3', _manual_arc(62.5)),
]

# --------------------------------------------------------------------------
# Sorting bins (one per colour) on the table beside / behind the robot
# --------------------------------------------------------------------------
BIN_RADIUS_FROM_COLUMN = 0.27
BIN_ANGLES_DEG = {'red': -75.0, 'green': -125.0, 'blue': 125.0, 'yellow': 75.0}   # relative to the robot's front
BIN_SIZE = 0.15                      # 140 mm inside: the 70 mm puck and the fingers around it fit
BIN_WALL = 0.005
BIN_HEIGHT = 0.03
BIN_FLOOR = 0.008


def bin_yaw(color):
    """World yaw of a bin (and of the gripper when it places into it)."""
    return math.radians(BIN_ANGLES_DEG[color]) + BASE_YAW


def bin_xy(color):
    a = bin_yaw(color)
    return (COLUMN_AXIS_XY[0] + BIN_RADIUS_FROM_COLUMN * math.cos(a),
            COLUMN_AXIS_XY[1] + BIN_RADIUS_FROM_COLUMN * math.sin(a))


# Voice / typed-command mode: ONE bin. You ask for a colour, the robot picks the
# pucks of that colour off the belt (until the next colour) and drops them here; the rest ride past.
COMMAND_BIN_ANGLE_DEG = -100.0       # relative to the robot's front (the robot's right side)
COMMAND_BIN_COLOR = (0.88, 0.88, 0.90, 1.0)


def command_bin_yaw():
    return math.radians(COMMAND_BIN_ANGLE_DEG) + BASE_YAW


def command_bin_xy():
    a = command_bin_yaw()
    return (COLUMN_AXIS_XY[0] + BIN_RADIUS_FROM_COLUMN * math.cos(a),
            COLUMN_AXIS_XY[1] + BIN_RADIUS_FROM_COLUMN * math.sin(a))


# --------------------------------------------------------------------------
# Motion heights, expressed as shoulder_joint (Z axis) values
# --------------------------------------------------------------------------
def z_for_pad_bottom(world_z):
    """shoulder_joint value that puts the bottom of the finger pads at world_z."""
    return world_z - TABLE_TOP_Z - PAD_BOTTOM_AT_Z0


PAD_ABOVE_SURFACE = 0.006   # pads stop 6 mm above the belt / table: they hold the puck body (6..26 mm)


def z_pick_on(surface_z):
    """shoulder_joint value for gripping a puck standing on surface_z by its body
    (on the table the Z axis bottoms out at -0.148: pads 12 mm up the 35 mm puck)."""
    return max(z_for_pad_bottom(surface_z + PAD_ABOVE_SURFACE), LIMITS['shoulder_joint'][0] + 0.002)


Z_TRAVEL = 0.015                                                  # safe height for moving (carried puck far above the belt)
Z_PICK = z_pick_on(BELT_TOP_Z)                                    # pads on both sides of the puck body
Z_PICK_TABLE = z_pick_on(TABLE_TOP_Z)                             # same for a puck on the table
Z_PRE_PICK = z_for_pad_bottom(BELT_TOP_Z + HANDLE_TOP_Z + 0.02)   # pads just above the puck (and its handle)
Z_PRE_PICK_TABLE = z_for_pad_bottom(TABLE_TOP_Z + HANDLE_TOP_Z + 0.02)
DROP_HEIGHT = 0.015                                               # the puck is let go 15 mm above the bin floor
Z_PLACE = z_pick_on(TABLE_TOP_Z + BIN_FLOOR + DROP_HEIGHT)

# Fingers (each moves by q, negative = inwards): pads 91 mm apart when open,
# they touch the 70 mm puck at q = -(0.091 - 0.070) / 2 = -0.0105, and meet
# each other (nothing held) at -0.0455.
GRIP_PREGRASP = 0.0         # fully open: 10 mm of room on each side of the puck
GRIP_CLOSE = -0.025         # squeeze target, past the puck surface -> firm grip
GRIP_RELEASE = 0.0          # open fully to let go
GRASP_OK_ABOVE = -0.020     # holding: fingers stop near -0.011; empty: they close to -0.025

# Ready pose: arm folded back, out of the fixed camera's view of the belt
READY_XY = (COLUMN_AXIS_XY[0] + 0.25 * math.cos(math.radians(150) + BASE_YAW),
            COLUMN_AXIS_XY[1] + 0.25 * math.sin(math.radians(150) + BASE_YAW))

# --------------------------------------------------------------------------
# Cameras (poses in Gazebo's convention: the camera looks along +X of its link)
# --------------------------------------------------------------------------
# Fixed RGB-D camera floating in the air behind the robot, looking over it at the belt
FIXED_CAM_XYZ = (*cell_xy(-0.08, 0.0), 1.10)
FIXED_CAM_RPY = (0.0, math.radians(64.0), CELL_YAW)
FIXED_CAM_WIDTH = 640
FIXED_CAM_HEIGHT = 480
FIXED_CAM_HFOV = 1.0472
CAM_NEAR = 0.05
CAM_FAR = 3.0

# Arm camera on a bracket at the front of the forearm, beside the wrist.  It is fixed to the
# forearm, so it does not turn with the gripper: turning the wrist can never swing the
# camera into the forearm or the shoulder link (it used to sit on the wrist housing).
# The lens looks down past the wrist housing at the finger pads.  Coordinates are in
# forearm_link (the wrist axis is at x = 0.1365, z = -0.06; the forearm ends at x = 0.183).
ARM_CAM_PARENT = 'forearm_link'
ARM_CAM_XYZ = (0.205, 0.0, 0.0)
ARM_CAM_TARGET = (0.180, 0.0, -0.170)
ARM_CAM_RPY = (0.0, math.pi / 2 + math.atan2(ARM_CAM_XYZ[0] - ARM_CAM_TARGET[0],
                                             ARM_CAM_XYZ[2] - ARM_CAM_TARGET[2]), 0.0)
ARM_CAM_WIDTH = 320
ARM_CAM_HEIGHT = 240
ARM_CAM_HFOV = 1.2217      # 70 degrees


if __name__ == '__main__':
    print('Z_PICK', round(Z_PICK, 4), 'Z_PICK_TABLE', round(Z_PICK_TABLE, 4), 'Z_PRE_PICK', round(Z_PRE_PICK, 4),
          'Z_PRE_PICK_TABLE', round(Z_PRE_PICK_TABLE, 4), 'Z_PLACE', round(Z_PLACE, 4))
    print('manual pucks', [(n, tuple(round(v, 3) for v in p)) for n, p in MANUAL_PUCKS])
    print('arm camera pitch', round(math.degrees(ARM_CAM_RPY[1]), 2), 'deg')
    print('base', BASE_XYZ, 'yaw', round(math.degrees(BASE_YAW), 1), 'conveyor x', CONVEYOR_X, 'belt dir', BELT_DIR)
    print('ready', tuple(round(v, 3) for v in READY_XY))
    for c in COLOR_ORDER:
        print(c, 'bin at', tuple(round(v, 3) for v in bin_xy(c)))
