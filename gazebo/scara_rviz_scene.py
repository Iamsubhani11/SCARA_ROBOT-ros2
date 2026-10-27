#!/usr/bin/env python3
"""
Draws the SCARA cell in RViz: table, conveyor, bins and the pucks - so the
digital twin in RViz shows the robot in its real surroundings.

    ros2 run scara_description scara_rviz_scene.py --ros-args -p cell:=manual
    (started by digital_twin.launch.py; the RViz config already shows /scara/scene_markers)

Parameters
  cell   sort | command | manual | none   which layout to draw (default manual)
Pucks come from Gazebo (/scara/world_poses) when it runs; otherwise the manual
table's pucks are drawn where they start.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scara_scene as S    # noqa: E402


def _yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class SceneMarkers(Node):
    def __init__(self):
        super().__init__('scara_rviz_scene')
        self.declare_parameter('cell', 'manual')
        self.cell = str(self.get_parameter('cell').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, '/scara/scene_markers', qos)
        self.pucks = {}
        self.create_subscription(TFMessage, '/scara/world_poses', self.on_poses, 10)
        self.create_timer(0.5, self.publish)
        self.get_logger().info(f'Drawing the {self.cell} cell on /scara/scene_markers')

    def on_poses(self, msg):
        for t in msg.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_'):
                p = t.transform.translation
                self.pucks[name] = (p.x, p.y, p.z)

    def _m(self, mid, ns, shape, xyz, size, rgba, yaw=0.0):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, shape, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        q = _yaw_q(yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x, m.scale.y, m.scale.z = map(float, size)
        m.color.r, m.color.g, m.color.b, m.color.a = map(float, rgba)
        return m

    def publish(self):
        arr = MarkerArray()
        if self.cell == 'none':
            self.pub.publish(arr)
            return
        tz = S.TABLE_TOP_Z
        arr.markers.append(self._m(0, 'table', Marker.CUBE, (*S.TABLE_CENTER, tz - S.TABLE_TOP_THICKNESS / 2),
                                   (S.TABLE_SIZE_X, S.TABLE_SIZE_Y, S.TABLE_TOP_THICKNESS), (0.80, 0.62, 0.40, 1.0)))
        i = 1
        if self.cell in ('sort', 'command'):
            a, b = S.belt_point(S.CONVEYOR_Y_RANGE[0]), S.belt_point(S.CONVEYOR_Y_RANGE[1])
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            length = math.hypot(b[0] - a[0], b[1] - a[1])
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            h = S.BELT_TOP_Z - tz
            arr.markers.append(self._m(i, 'conveyor', Marker.CUBE, (*mid, tz + h / 2), (length, S.BELT_WIDTH, h),
                                       (0.22, 0.24, 0.27, 1.0), yaw))
            i += 1
            bins = ([('drop', *S.command_bin_xy(), S.command_bin_yaw(), S.COMMAND_BIN_COLOR)] if self.cell == 'command'
                    else [(c, *S.bin_xy(c), S.bin_yaw(c), S.COLORS[c]) for c in S.COLOR_ORDER])
            for _, x, y, yaw, rgba in bins:
                arr.markers.append(self._m(i, 'bins', Marker.CUBE, (x, y, tz + S.BIN_HEIGHT / 2),
                                           (S.BIN_SIZE, S.BIN_SIZE, S.BIN_HEIGHT), (*rgba[:3], 0.45), yaw))
                i += 1
        pucks = dict(self.pucks)
        if not pucks and self.cell == 'manual':
            pucks = {n: (x, y, tz) for n, (x, y, _) in S.MANUAL_PUCKS}
        for k, (name, (x, y, z)) in enumerate(sorted(pucks.items())):
            if S.is_hidden(x, y):
                continue
            rgba = S.COLORS.get(S.puck_color(name), (0.6, 0.6, 0.6, 1.0))
            arr.markers.append(self._m(100 + k, 'pucks', Marker.CYLINDER, (x, y, z + S.PUCK_HEIGHT / 2),
                                       (2 * S.PUCK_RADIUS, 2 * S.PUCK_RADIUS, S.PUCK_HEIGHT), rgba))
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = SceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
