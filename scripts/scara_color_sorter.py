#!/usr/bin/env python3
"""
Vision-guided pick & place for Gazebo - same logic as MuJoCo (scara_sorter_core.py).

  mode:=sort     every puck goes into the bin of its own colour (4 bins), endless
  mode:=command  ONE bin.  The belt keeps running; when you ask for a colour
                 (voice: scara_ask.py --voice, typed: scara_ask.py, phone app, GUI or
                 /scara/pick_color) the robot picks EVERY puck of that colour and
                 drops it in the bin - until you give the next colour, "any"
                 (all colours) or "none".  "3 red" picks three, then waits.
                 Everything not asked for rides off the end.

Uses the same decision logic as the MuJoCo demo (scara_sorter_core.py) and
talks to the robot through ros2_control:

  actions   /arm_controller/follow_joint_trajectory
            /gripper_controller/follow_joint_trajectory
  topics    /joint_states            (in)
            /scara/detections        (in, from scara_vision_node.py)
            /scara/pick_color        (in, std_msgs/String) e.g. "red", "red blue", "any", "none", "3 red",
                                     "stop" / "start" (belt), "empty" (bins emptied)
            /scara/sorter_state      (out, std_msgs/String JSON: mode, active colours, totals, running)
            /conveyor/speed          (out, std_msgs/Float64 -> Gazebo belt)
            /scara/status            (out, std_msgs/String)

Parameters: mode (sort | command), belt_sign (+1/-1), bin_capacity (0 = bins emptied)

    ros2 topic pub --once /scara/pick_color std_msgs/msg/String "{data: 'red'}"
"""
import json
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectoryPoint

import scara_scene as S
from scara_sorter_core import Sorter
from scara_voice import parse_command


def to_duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int((t - sec) * 1e9))


class SorterNode(Node):
    def __init__(self):
        super().__init__('scara_color_sorter')
        self.declare_parameter('mode', 'sort')
        self.declare_parameter('colors', 'all')
        self.declare_parameter('belt_sign', 1.0)
        self.declare_parameter('bin_capacity', 0)      # 0 = bins are emptied automatically (spawner clear_bins)
        cb = ReentrantCallbackGroup()
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=cb)
        self.grip = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=cb)
        self.js = {}
        self.dets = []
        self.dets_stamp = 0.0
        self.belt_cmd = 0.0
        self.belt_sign = float(self.get_parameter('belt_sign').value)
        self._belt_probe = None
        self.mode = str(self.get_parameter('mode').value)
        self.running = True                 # the belt runs in both modes; "stop" / "start" pause it
        self.pending = []
        self.create_subscription(JointState, '/joint_states', self.on_js, 20, callback_group=cb)
        self.create_subscription(String, '/scara/detections', self.on_dets, 10, callback_group=cb)
        self.create_subscription(String, '/scara/pick_color', self.on_cmd, 10, callback_group=cb)
        self.belt_pub = self.create_publisher(Float64, '/conveyor/speed', 10)
        self.status_pub = self.create_publisher(String, '/scara/status', 10)
        self.state_pub = self.create_publisher(String, '/scara/sorter_state', 10)
        self.create_timer(0.5, self.publish_state, callback_group=cb)
        # the Gazebo TrackController forgets old commands, so keep repeating it
        self.create_timer(0.1, self.publish_belt, callback_group=cb)
        self.sorter = Sorter(self, self.get_parameter('colors').value,
                             bin_capacity=int(self.get_parameter('bin_capacity').value), mode=self.mode)

    # ------------------------------------------------------------ callbacks
    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.js[n] = p

    def on_dets(self, msg):
        try:
            self.dets = json.loads(msg.data)
            self.dets_stamp = self.now()
        except ValueError:
            pass
        self._check_belt_direction()

    def on_cmd(self, msg):
        """Commands on /scara/pick_color: a colour / "any" (command mode), "stop", "start", "empty"."""
        cmd = parse_command(msg.data)
        if cmd is None and msg.data.strip().lower() in ('any', 'anything', 'next'):
            cmd = 'all'
        if cmd == 'stop':
            self.running = False
            self.log('Conveyor stopped (send "start" to continue)')
        elif cmd == 'start':
            self.running = True
            self.log('Conveyor running')
        elif cmd and cmd.startswith('empty'):
            self.sorter.empty_bins(cmd.split()[1:] or None)
        elif cmd:
            if self.mode != 'command':
                self.log(f'"{msg.data}" ignored: sorting mode picks every colour (launch mode:=command to ask for colours)')
                return
            self.pending.append('any' if cmd == 'all' else cmd)
            self.running = True
        else:
            self.log(f'Did not understand "{msg.data}". Use: {", ".join(S.COLOR_ORDER)}, any, stop, start')

    def publish_state(self):
        so = self.sorter
        self.state_pub.publish(String(data=json.dumps({
            'mode': self.mode, 'active': so.requests, 'count_left': so.count_left, 'sequence': so.sequence,
            'sorted': so.sorted,
            'running': self.running, 'belt': self.belt_cmd, 'failures': so.failures})))

    def publish_belt(self):
        self.belt_pub.publish(Float64(data=self.belt_cmd * self.belt_sign))

    def _check_belt_direction(self):
        """If the belt carries pucks the wrong way, flip the sign once."""
        if self.belt_cmd <= 0 or not self.dets:
            self._belt_probe = None
            return
        mean_x = sum(d['s'] for d in self.dets) / len(self.dets)   # position along the belt
        n = len(self.dets)
        if self._belt_probe is None or self._belt_probe[2] != n:
            self._belt_probe = (self.now(), mean_x, n)
            return
        t0, x0, _ = self._belt_probe
        if self.now() - t0 > 1.0:
            v = (mean_x - x0) / (self.now() - t0)
            if v < -0.02:
                self.belt_sign = -self.belt_sign
                self.log(f'Belt was running backwards - flipping direction (belt_sign={self.belt_sign:+.0f})')
            self._belt_probe = (self.now(), mean_x, n)

    # ------------------------------------------------------------ IO for Sorter
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, dt):
        t_end = self.now() + dt
        wall_end = time.time() + dt * 20 + 5
        while self.now() < t_end and time.time() < wall_end and rclpy.ok():
            time.sleep(0.005)

    def _run_traj(self, client, names, positions, T, tries=6):
        # a controller's action server exists as soon as it is loaded, but it only accepts goals
        # once it is active - right after start-up a goal can be refused for a second or two
        for attempt in range(tries):
            ok = self._run_traj_once(client, names, positions, T, quiet=attempt < tries - 1)
            if ok is not None:
                return ok
            time.sleep(0.5)
        return False

    def _run_traj_once(self, client, names, positions, T, quiet=False):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(names)
        pt.time_from_start = to_duration(T)
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        wall_end = time.time() + 10.0
        while not fut.done() and time.time() < wall_end:
            time.sleep(0.005)
        if not fut.done() or not fut.result().accepted:
            if quiet:
                return None                    # try again
            self.get_logger().error(f'Trajectory for {names} was rejected - is the controller active? '
                                    '(ros2 control list_controllers)')
            return False
        res = fut.result().get_result_async()
        wall_end = time.time() + T * 20 + 10.0
        while not res.done() and time.time() < wall_end:
            time.sleep(0.005)
        return res.done()

    def move_arm(self, q, T):
        self._run_traj(self.arm, S.ARM_JOINTS, q, T)
        self.sleep(0.15)

    def move_gripper(self, q, T):
        self._run_traj(self.grip, S.GRIPPER_JOINTS, [q, q], T)

    def set_belt(self, v):
        self.belt_cmd = v
        self.publish_belt()

    def joints(self):
        return dict(self.js)

    def detections(self):
        return self.dets if self.now() - self.dets_stamp < 1.0 else []

    def log(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    # ------------------------------------------------------------ main loop
    def worker(self):
        # the conveyor runs as soon as the simulation clock runs - it does not wait for the arm
        t_wait = time.time()
        while rclpy.ok() and self.now() <= 0.0:
            if time.time() - t_wait > 10.0:
                self.get_logger().warn('No /clock yet - is Gazebo running and the ros_gz bridge started?')
                t_wait = time.time()
            time.sleep(0.1)
        if self.running:
            self.set_belt(S.BELT_SPEED)
            self.log(f'Conveyor started ({S.BELT_SPEED} m/s on /conveyor/speed)')
        self.log('Waiting for the arm and gripper controllers...')
        t_wait = time.time()
        for c in (self.arm, self.grip):
            while rclpy.ok() and not c.wait_for_server(timeout_sec=2.0):
                if time.time() - t_wait > 15.0:
                    t_wait = time.time()
                    self.log('Arm controllers still not available - check `ros2 control list_controllers` '
                             '(arm_controller, gripper_controller must be active) and GZ_SIM_SYSTEM_PLUGIN_PATH '
                             '(gz_ros2_control). Run: python3 scripts/scara_doctor.py')
        while rclpy.ok() and not self.js:
            time.sleep(0.1)
        self.log('Controllers ready. Moving to the ready pose.')
        self.sorter.gripper_open()
        self.sorter.go_ready()
        if self.mode == 'command':
            self.log('Ready. Ask for a colour: scara_ask.py [--voice], the phone app, or\n'
                     '  ros2 topic pub --once /scara/pick_color std_msgs/msg/String "{data: red}"')
        else:
            self.log('Ready. Sorting every colour into its bin.')
        last_err, no_dets_warned = None, False
        t_start = time.time()
        while rclpy.ok():
            while self.pending:
                self.sorter.request(self.pending.pop(0))
            if not self.running:
                self.set_belt(0.0)
                time.sleep(0.1)
                continue
            if not no_dets_warned and time.time() - t_start > 20.0 and self.dets_stamp == 0.0:
                no_dets_warned = True
                self.log('No /scara/detections yet - the vision node gets no camera images '
                         '(check: ros2 topic hz /fixed_camera/image). The belt keeps running.')
            try:
                self.sorter.step(timeout=1.0)
                last_err = None
            except Exception as e:   # keep running after an unreachable puck etc.
                if str(e) != last_err:
                    self.get_logger().error(f'step failed: {e!r}')
                last_err = str(e)
                try:
                    self.sorter.gripper_open()
                    self.sorter.go_ready()
                except Exception:
                    time.sleep(1.0)
                self.set_belt(S.BELT_SPEED)        # never leave the belt stopped after a hiccup


def main():
    rclpy.init()
    node = SorterNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    t = threading.Thread(target=node.worker, daemon=True)
    t.start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.set_belt(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
