#!/usr/bin/env python3
"""
Puck (and robot) poses from Gazebo on /scara/world_poses, WITH their names.

Why this exists: ros_gz_bridge turns gz.msgs.Pose_V into tf2_msgs/TFMessage, but it takes
child_frame_id from the message header, and Gazebo's /world/<world>/dynamic_pose/info puts
the model names in each pose's `name` field instead - so through the bridge every transform
arrives with an empty name and nobody can tell which puck is where.  (That is why, before,
new pucks could be dropped onto pucks already waiting at the start of the belt, and sorted
pucks were never cleared from the bins.)

This node reads the Gazebo topic directly with the `gz` tool and republishes it with
child_frame_id = model name and frame_id = "world", 20 times a second.

    ros2 run scara_description scara_pose_relay.py --ros-args -p world:=scara_world
"""
import json
import re
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def parse_text(block):
    """gz text (protobuf) format of a Pose_V message -> {name: (x, y, z, qx, qy, qz, qw)}"""
    out = {}
    for pose in re.split(r'\npose \{', '\n' + block)[1:]:
        m = re.search(r'name: "([^"]*)"', pose)
        if not m:
            continue

        def grab(section, keys, default):
            s = re.search(section + r' \{([^}]*)\}', pose)
            vals = dict(default)
            if s:
                for k in keys:
                    v = re.search(r'\b' + k + r': ([-+0-9.eE]+)', s.group(1))
                    if v:
                        vals[k] = float(v.group(1))
            return vals
        p = grab('position', 'xyz', {'x': 0.0, 'y': 0.0, 'z': 0.0})
        q = grab('orientation', 'xyzw', {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
        out[m.group(1)] = (p['x'], p['y'], p['z'], q['x'], q['y'], q['z'], q['w'])
    return out


def parse_json(line):
    msg = json.loads(line)
    out = {}
    for pose in msg.get('pose', []):
        name = pose.get('name')
        if not name:
            continue
        p, q = pose.get('position', {}), pose.get('orientation', {})
        out[name] = (float(p.get('x', 0)), float(p.get('y', 0)), float(p.get('z', 0)),
                     float(q.get('x', 0)), float(q.get('y', 0)), float(q.get('z', 0)), float(q.get('w', 1)))
    return out


class PoseRelay(Node):
    def __init__(self):
        super().__init__('scara_pose_relay')
        self.declare_parameter('world', 'scara_world')
        self.declare_parameter('rate', 20.0)
        self.world = str(self.get_parameter('world').value)
        self.pub = self.create_publisher(TFMessage, '/scara/world_poses', 10)
        self.latest = None
        self.lock = threading.Lock()
        threading.Thread(target=self.reader, daemon=True).start()
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.publish)

    def reader(self):
        topic = f'/world/{self.world}/dynamic_pose/info'
        use_json = True
        while rclpy.ok():
            cmd = ['gz', 'topic', '-e', '-t', topic] + (['--json-output'] if use_json else [])
            try:
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            except FileNotFoundError:
                self.get_logger().error('the `gz` tool is not on the PATH - puck poses are not available')
                return
            block, got = [], 0
            for line in p.stdout:
                if use_json:
                    if line.lstrip().startswith('{'):
                        try:
                            poses = parse_json(line)
                        except ValueError:
                            continue
                        got += 1
                        with self.lock:
                            self.latest = poses
                    elif 'json' in line.lower() and got == 0:      # old gz-transport: no --json-output
                        use_json = False
                        p.kill()
                        break
                else:
                    if line.startswith('header {') and block:
                        poses = parse_text(''.join(block))
                        if poses:
                            with self.lock:
                                self.latest = poses
                        block = []
                    block.append(line)
            p.wait()
            time.sleep(1.0)                                         # Gazebo restarted / not up yet

    def publish(self):
        with self.lock:
            poses = self.latest
        if not poses:
            return
        msg = TFMessage()
        stamp = self.get_clock().now().to_msg()
        for name, (x, y, z, qx, qy, qz, qw) in poses.items():
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'world'
            t.child_frame_id = name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            msg.transforms.append(t)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseRelay()
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
