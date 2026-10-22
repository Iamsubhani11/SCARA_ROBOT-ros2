#!/usr/bin/env python3
"""
Endless puck supply for the Gazebo cell (like the videos): pucks appear at the
start of the conveyor one at a time while the cell runs - nothing is parked
anywhere in the world beforehand.

Parameters
  count       total pucks to spawn (default 0 = endless, until you stop the launch)
  interval    minimum seconds between pucks (default 4.0)
  colors      "random" (default) or a list such as "red green blue yellow"
  world       Gazebo world name (default scara_world)
  mode        sort (colour bins) | command (one bin) - where the bins are (default sort)
  clear_bins  true (default): sorted pucks are taken out of the bins a few
              seconds after they land, pucks that rode to the end of the belt
              (colours you did not ask for) and dropped pucks are removed too,
              so the bins never overflow however long it runs.  false: they
              stay (use the sorter's bin_capacity to get "bin full" warnings).

A new puck is only dropped when the spawn area on the belt is clear (checked
with /scara/detections), so pucks never land on top of each other.
Puck positions come from /scara/world_poses (scara_pose_relay.py republishes Gazebo's
/world/<world>/dynamic_pose/info with the model names); pucks are removed with the
world's "remove" service.

Manual spawn:  ros2 topic pub --once /scara/spawn_puck std_msgs/msg/String "{data: red}"
"""
import json
import math
import os
import random
import shutil
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import scara_scene as S
from scara_feeder import classify
from scara_models import puck_sdf


class PuckSpawner(Node):
    def __init__(self):
        super().__init__('scara_puck_spawner')
        self.declare_parameter('count', 0)
        self.declare_parameter('interval', 4.0)
        self.declare_parameter('colors', 'random')
        self.declare_parameter('world', 'scara_world')
        self.declare_parameter('seed', 0)
        self.declare_parameter('clear_bins', True)
        self.declare_parameter('mode', 'sort')
        self.declare_parameter('clear_after', 2.0)
        self.clear_bins = bool(self.get_parameter('clear_bins').value)
        self.mode = str(self.get_parameter('mode').value)
        self.clear_after = float(self.get_parameter('clear_after').value)
        self.poses = {}            # puck name -> (x, y, z) from Gazebo
        self.since = {}            # puck name -> time it reached its final place
        self.removing = set()
        self.pose_stamp = None
        self.count = int(self.get_parameter('count').value)
        self.interval = float(self.get_parameter('interval').value)
        cols = str(self.get_parameter('colors').value).split()
        self.sequence = None if cols in (['random'], []) else cols
        self.world = self.get_parameter('world').value
        seed = int(self.get_parameter('seed').value)
        self.rng = random.Random(seed if seed else None)
        self.spawned = 0
        self.last_spawn = -1e9
        self.dets = None
        self.manual = []
        self.create_subscription(String, '/scara/detections', self.on_dets, 10)
        self.create_subscription(String, '/scara/spawn_puck', self.on_manual, 10)
        self.create_subscription(TFMessage, '/scara/world_poses', self.on_poses, 10)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.5, self.clean)
        self.get_logger().info(f'Puck spawner: {self.count or "unlimited"} pucks, colours: {self.sequence or "random"}')

    def on_dets(self, msg):
        try:
            self.dets = json.loads(msg.data)
        except ValueError:
            pass

    def on_manual(self, msg):
        c = msg.data.strip().lower()
        if c in S.COLORS:
            self.manual.append(c)
        else:
            self.get_logger().warn(f'unknown colour "{c}"')

    # ---------------------------------------------------------- bin clearing
    def on_poses(self, msg):
        for t in msg.transforms:
            name = (t.child_frame_id or '').split('::')[0]
            if name.startswith('puck_') and name not in self.removing:
                p = t.transform.translation
                self.poses[name] = (p.x, p.y, p.z)
        self.pose_stamp = self.get_clock().now().nanoseconds * 1e-9

    def remove(self, name, where):
        self.removing.add(name)
        self.poses.pop(name, None)
        self.since.pop(name, None)
        req = f'name: "{name}" type: MODEL'
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/remove', '--reqtype', 'gz.msgs.Entity',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info(f'Removed {name} ({where.replace("_", " ")})')

    def clean(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pose_stamp is None:
            if self.clear_bins and self.spawned >= 3 and not getattr(self, '_warned', False):
                self._warned = True
                self.get_logger().warn('No puck poses on /scara/world_poses - bins will not be emptied '
                                       '(is scara_pose_relay.py running?)')
            return
        for name, (x, y, z) in list(self.poses.items()):
            where = classify(x, y, z, self.mode)
            done = where in ('end', 'table', 'lost') or (where.startswith('bin_') and self.clear_bins)
            if not done:
                self.since.pop(name, None)
                continue
            t0 = self.since.setdefault(name, now)
            if now - t0 >= (self.clear_after if where.startswith('bin_') else self.clear_after + 1.0):
                self.remove(name, where)

    def create(self, name, color, x, y, z, yaw):
        """Add a puck to Gazebo: the world's /create service (fast, needs only the `gz` tool),
        or `ros2 run ros_gz_sim create` if `gz` is not on the PATH."""
        if shutil.which('gz'):
            path = os.path.join(tempfile.gettempdir(), f'scara_{name}.sdf')
            with open(path, 'w') as fh:
                fh.write(puck_sdf(color, name))
            qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
            req = (f'sdf_filename: "{path}" name: "{name}" allow_renaming: false '
                   f'pose: {{position: {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} orientation: {{z: {qz:.5f} w: {qw:.5f}}}}}')
            cmd = ['gz', 'service', '-s', f'/world/{self.world}/create', '--reqtype', 'gz.msgs.EntityFactory',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req]
        else:
            cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world, '-name', name,
                   '-string', puck_sdf(color, name), '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', f'{z:.3f}',
                   '-Y', f'{yaw:.3f}']
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self._check_create, args=(p, name), daemon=True).start()

    def _check_create(self, p, name):
        out, _ = p.communicate()
        if p.returncode != 0 or 'false' in (out or '').lower():
            self.get_logger().error(f'Could not spawn {name}: {(out or "").strip()[:200]} '
                                    f'(is Gazebo running the world "{self.world}"?)')

    def spawn_area_clear(self):
        if self.pose_stamp is not None:      # exact puck positions from Gazebo
            return not any(classify(*p, mode=self.mode) == 'belt' and abs(S.belt_s(p[0], p[1]) - S.SPAWN_S) < S.MIN_PUCK_SPACING
                           for p in self.poses.values())
        if self.dets is None:                # no vision either: timing only
            return True
        return not any(abs(d['s'] - S.SPAWN_S) < S.MIN_PUCK_SPACING for d in self.dets)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return                                    # /clock not running yet
        auto = self.count == 0 or self.spawned < self.count
        if not self.manual and not auto:
            return
        if now - self.last_spawn < self.interval or not self.spawn_area_clear():
            return
        if self.manual:
            color = self.manual.pop(0)
        elif self.sequence:
            color = self.sequence[self.spawned % len(self.sequence)]
        else:
            color = self.rng.choice(S.COLOR_ORDER)
        name = f'puck_{color}_{self.spawned:03d}'
        s_pos = S.SPAWN_S
        if self.dets is None:
            # no vision running (e.g. joystick / gesture mode): lay pucks out in a row
            if self.spawned >= 4 and not self.manual:
                return
            s_pos = S.SPAWN_S + S.MIN_PUCK_SPACING * (self.spawned % 4)
        x, y = S.belt_point(s_pos, self.rng.uniform(-0.01, 0.01))
        self.create(name, color, x, y, S.BELT_TOP_Z + 0.002, self.rng.uniform(-math.pi, math.pi))
        self.spawned += 1
        self.last_spawn = now
        self.get_logger().info(f'Spawned {name}')


def main():
    rclpy.init()
    node = PuckSpawner()
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
