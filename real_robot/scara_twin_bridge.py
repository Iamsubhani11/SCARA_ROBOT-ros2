#!/usr/bin/env python3
"""
Digital-twin bridge: connects the real HowToMechatronics SCARA (Arduino running
real_robot/firmware/scara_twin/scara_twin.ino) with the simulated SCARA.

mode:=sim_to_real   (default) the simulation is the master.  Whatever moves the
                    simulated arm - keyboard, joystick, gestures, voice, the
                    colour sorter, the phone app, RViz sliders - is sent to the
                    real robot.  The measured pose of the real robot comes back
                    on /real/joint_states (shown as a ghost robot in RViz).
mode:=real_to_sim   the real robot is the master (e.g. TEACH_WITH_POTS knobs, or
                    jogging it with its own GUI): the simulated robot follows it.
                    target:=gazebo drives the Gazebo controllers,
                    target:=rviz publishes /joint_states for robot_state_publisher.

Topics
  in   /joint_states                         (sim_to_real) simulated joints
  out  /real/joint_states                    measured joints of the real robot
  out  /arm_controller/joint_trajectory      (real_to_sim, target gazebo)
  out  /gripper_controller/joint_trajectory  (real_to_sim, target gazebo)
  out  /joint_states                         (real_to_sim, target rviz)
  out  /scara/twin_status                    JSON: homed, moving, sim-real error
  in   /scara/twin_cmd                       "zero" (H) | "free" (F) | "lock" (L) | "home" (switches, K) |
                                             "stop" | "conveyor on" | "conveyor off"
  in   /conveyor/speed, /conveyor/cmd_vel,   conveyor of the simulation / the phone app -> the real
       /conveyor/enabled                     conveyor's relay (sim_to_real)

Run without hardware:  fake:=true  (a software copy of the firmware answers)
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import json

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from scara_twin_protocol import HardwareMap, MAP_KEYS, USER_MAP_FILE, _read_keys, open_link

ARM = ['column_joint', 'shoulder_joint', 'forearm_joint', 'wrist_joint']
GRIP = ['left_finger_joint', 'right_finger_joint']


class TwinBridge(Node):
    def __init__(self):
        super().__init__('scara_twin_bridge')
        p = self.declare_parameter
        p('port', '/dev/ttyACM0')
        p('baud', 115200)
        p('fake', False)
        p('mode', 'sim_to_real')
        p('target', 'gazebo')
        p('homing', 'switches')            # switches (K, the original build) | zero (arm at the ready pose -> H) | none
        p('rate', 20.0)
        for k, v in (('j1_sign', 1.0), ('j1_offset_deg', 0.0), ('j2_sign', 1.0), ('j2_offset_deg', 0.0),
                     ('j3_sign', 1.0), ('j3_offset_deg', 0.0), ('z_zero_mm', 150.0), ('z_sign', 1.0)):
            p(k, v)
        g = lambda k: self.get_parameter(k).value          # noqa: E731
        vals = {k: g(k) for k in MAP_KEYS}
        vals.update(_read_keys(USER_MAP_FILE))         # what the app's direction check saved wins
        hw_map = HardwareMap.from_settings(vals)
        self.mode = g('mode')
        self.target = g('target')
        if self.mode not in ('sim_to_real', 'real_to_sim'):
            raise ValueError('mode must be sim_to_real or real_to_sim')
        self.link = open_link(g('port'), int(g('baud')), fake=bool(g('fake')), hw_map=hw_map,
                              on_message=self.on_robot_message)
        self.get_logger().info(f'Twin bridge: {self.mode}, robot on {"FAKE" if g("fake") else g("port")}')
        homing = g('homing')
        if homing == 'switches':
            self.link.home()
        elif homing == 'zero':
            self.link.zero_here()

        self.sim_q = {}
        self.last_warn = 0.0
        self.real_pub = self.create_publisher(JointState, '/real/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/scara/twin_status', 10)
        self.create_subscription(String, '/scara/twin_cmd', self.on_cmd, 10)
        self.belt_on = None
        if self.mode == 'sim_to_real':
            # the real conveyor follows the simulated one / the phone app's conveyor button
            self.create_subscription(Float64, '/conveyor/speed', lambda m: self.set_belt(abs(m.data) > 1e-3), 10)
            self.create_subscription(Twist, '/conveyor/cmd_vel', lambda m: self.set_belt(abs(m.linear.x) > 1e-3), 10)
            self.create_subscription(Bool, '/conveyor/enabled', lambda m: self.set_belt(bool(m.data)), 10)
        if self.mode == 'sim_to_real':
            self.create_subscription(JointState, '/joint_states', self.on_sim, 20)
        elif self.target == 'gazebo':
            self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
            self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        else:
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / float(g('rate')), self.tick)

    # ------------------------------------------------------------------ io
    def on_robot_message(self, kind, text):
        log = {'info': self.get_logger().info, 'warning': self.get_logger().warn}.get(kind, self.get_logger().error)
        log(f'robot: {text}')

    def on_cmd(self, msg):
        c = msg.data.strip().lower()
        actions = {'zero': self.link.zero_here, 'home': self.link.home, 'stop': self.link.stop,
                   'free': self.link.free_motors, 'lock': self.link.lock_motors,
                   'enable': self.link.lock_motors, 'disable': self.link.free_motors,
                   'conveyor on': lambda: self.set_belt(True), 'conveyor off': lambda: self.set_belt(False)}
        if c in actions:
            actions[c]()
        else:
            self.get_logger().warn(f'unknown twin command "{c}" (zero, free, lock, home, stop, conveyor on/off)')

    def set_belt(self, on):
        if on != self.belt_on:
            self.belt_on = on
            self.link.conveyor(on)

    def on_sim(self, msg):
        for n, v in zip(msg.name, msg.position):
            self.sim_q[n] = v

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ loop
    def tick(self):
        if self.mode == 'sim_to_real' and all(n in self.sim_q for n in ARM):
            q = dict(self.sim_q)
            bad = self.link.out_of_range(q)
            if bad and self.now() - self.last_warn > 5.0:
                self.last_warn = self.now()
                self.get_logger().warn(f'{bad} outside the real robot\'s range - it stops at its limit')
            self.link.send_joints(q)

        real = self.link.joints()
        st = self.link.status()
        if real is None:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM + GRIP
        js.position = [real[n] for n in js.name]
        self.real_pub.publish(js)

        if self.mode == 'real_to_sim':
            if self.target == 'gazebo':
                self.publish_traj(self.arm_pub, ARM, [real[n] for n in ARM])
                self.publish_traj(self.grip_pub, GRIP, [real[n] for n in GRIP])
            else:
                self.js_pub.publish(js)

        err = None
        if self.mode == 'sim_to_real' and self.sim_q:
            err = max(abs(self.sim_q.get(n, real[n]) - real[n]) for n in ARM)
        self.status_pub.publish(String(data=json.dumps({
            'mode': self.mode, 'homed': st[3], 'moving': st[2],
            'real': {n: round(real[n], 4) for n in ARM + GRIP[:1]},
            'max_error': None if err is None else round(err, 4)})))

    def publish_traj(self, pub, names, pos):
        t = JointTrajectory()
        t.joint_names = names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start = Duration(sec=0, nanosec=int(1.5e8))
        t.points = [pt]
        pub.publish(t)

    def destroy_node(self):
        try:
            self.link.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TwinBridge()
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
