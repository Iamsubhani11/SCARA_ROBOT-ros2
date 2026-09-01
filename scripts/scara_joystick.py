#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ScaraJoystick(Node):

    def __init__(self):
        super().__init__('scara_joystick')

        # ============================================================
        # ROS 2 Publishers / Subscriber
        # ============================================================

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ============================================================
        # SCARA joints
        # ============================================================

        self.arm_joints = [
            'column_joint',
            'shoulder_joint',
            'forearm_joint',
            'wrist_joint'
        ]

        self.gripper_joint = ['left_finger_joint']

        # ============================================================
        # Current target positions
        # ============================================================

        self.column = 0.0
        self.shoulder = 0.0
        self.forearm = 0.0
        self.wrist = 0.0
        self.gripper = -0.05

        # ============================================================
        # Joint limits
        # ============================================================

        self.COLUMN_MIN = -2.0
        self.COLUMN_MAX = 2.0

        self.SHOULDER_MIN = -0.15
        self.SHOULDER_MAX = 0.02

        self.FOREARM_MIN = -2.0
        self.FOREARM_MAX = 2.0

        self.WRIST_MIN = -4.712
        self.WRIST_MAX = 4.712

        self.GRIPPER_MIN = -0.06
        self.GRIPPER_MAX = 0.0

        # ============================================================
        # Joystick settings
        # ============================================================

        self.DEADZONE = 0.12

        # Maximum movement per update
        self.COLUMN_SPEED = 0.025
        self.SHOULDER_SPEED = 0.004
        self.FOREARM_SPEED = 0.025
        self.WRIST_SPEED = 0.035

        # Publish at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # Latest joystick values
        self.axes = [0.0] * 8
        self.buttons = [0] * 11

        self.last_gripper_button = 0
        self.last_home_button = 0
        self.last_stop_button = 0

        self.get_logger().info(
            '======================================================'
        )
        self.get_logger().info(
            '       SCARA XBOX JOYSTICK CONTROLLER'
        )
        self.get_logger().info(
            '======================================================'
        )
        self.get_logger().info(
            'Left Stick X  -> Column Joint'
        )
        self.get_logger().info(
            'Left Stick Y  -> Z / Shoulder Joint'
        )
        self.get_logger().info(
            'Right Stick X -> Forearm Joint'
        )
        self.get_logger().info(
            'Right Stick Y -> Wrist Joint'
        )
        self.get_logger().info(
            'A -> CLOSE GRIPPER'
        )
        self.get_logger().info(
            'B -> OPEN GRIPPER'
        )
        self.get_logger().info(
            'Y -> HOME'
        )
        self.get_logger().info(
            'X -> STOP / HOLD'
        )
        self.get_logger().info(
            '======================================================'
        )

    # ================================================================
    # Joystick callback
    # ================================================================

    def joy_callback(self, msg):

        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

    # ================================================================
    # Dead zone
    # ================================================================

    def apply_deadzone(self, value):

        if abs(value) < self.DEADZONE:
            return 0.0

        sign = 1.0 if value > 0 else -1.0

        # Remove deadzone and rescale remaining range
        value = (
            (abs(value) - self.DEADZONE)
            / (1.0 - self.DEADZONE)
        )

        return sign * value

    # ================================================================
    # Clamp
    # ================================================================

    def clamp(self, value, minimum, maximum):

        return max(minimum, min(maximum, value))

    # ================================================================
    # Main control loop
    # ================================================================

    def control_loop(self):

        if len(self.axes) < 5:
            return

        if len(self.buttons) < 4:
            return

        # ------------------------------------------------------------
        # Read joystick axes
        # ------------------------------------------------------------

        left_x = self.apply_deadzone(self.axes[0])
        left_y = self.apply_deadzone(self.axes[1])

        right_x = self.apply_deadzone(self.axes[3])
        right_y = self.apply_deadzone(self.axes[4])

        # ------------------------------------------------------------
        # Joint 1 - Column
        # ------------------------------------------------------------

        self.column += left_x * self.COLUMN_SPEED

        self.column = self.clamp(
            self.column,
            self.COLUMN_MIN,
            self.COLUMN_MAX
        )

        # ------------------------------------------------------------
        # Joint 2 - Z / Shoulder
        #
        # Joystick Y is normally:
        # UP   = -1
        # DOWN = +1
        #
        # Therefore invert it.
        # ------------------------------------------------------------

        self.shoulder += -left_y * self.SHOULDER_SPEED

        self.shoulder = self.clamp(
            self.shoulder,
            self.SHOULDER_MIN,
            self.SHOULDER_MAX
        )

        # ------------------------------------------------------------
        # Joint 3 - Forearm
        # ------------------------------------------------------------

        self.forearm += right_x * self.FOREARM_SPEED

        self.forearm = self.clamp(
            self.forearm,
            self.FOREARM_MIN,
            self.FOREARM_MAX
        )

        # ------------------------------------------------------------
        # Joint 4 - Wrist
        # ------------------------------------------------------------

        self.wrist += -right_y * self.WRIST_SPEED

        self.wrist = self.clamp(
            self.wrist,
            self.WRIST_MIN,
            self.WRIST_MAX
        )

        # ------------------------------------------------------------
        # Buttons
        # ------------------------------------------------------------

        # A = Button 0
        if self.buttons[0] == 1:
            self.gripper = 0.0

        # B = Button 1
        if self.buttons[1] == 1:
            self.gripper = -0.05

        # Y = Button 3
        # Home only when button is newly pressed
        if self.buttons[3] == 1 and self.last_home_button == 0:

            self.column = 0.0
            self.shoulder = 0.0
            self.forearm = 0.0
            self.wrist = 0.0
            self.gripper = -0.05

            self.get_logger().info(
                'HOME POSITION'
            )

        self.last_home_button = self.buttons[3]

        # ------------------------------------------------------------
        # X = Button 2
        #
        # STOP / HOLD:
        # The joystick naturally stops changing the targets when
        # released. X does not reset the robot.
        # ------------------------------------------------------------

        if self.buttons[2] == 1:

            self.get_logger().info(
                'STOP / HOLD'
            )

        # ------------------------------------------------------------
        # Publish current targets
        # ------------------------------------------------------------

        self.publish_arm()

        self.publish_gripper()

    # ================================================================
    # Publish arm trajectory
    # ================================================================

    def publish_arm(self):

        msg = JointTrajectory()

        msg.joint_names = self.arm_joints

        point = JointTrajectoryPoint()

        point.positions = [
            float(self.column),
            float(self.shoulder),
            float(self.forearm),
            float(self.wrist)
        ]

        point.time_from_start = Duration(
            sec=0,
            nanosec=100000000
        )

        msg.points = [point]

        self.arm_pub.publish(msg)

    # ================================================================
    # Publish gripper trajectory
    # ================================================================

    def publish_gripper(self):

        msg = JointTrajectory()

        msg.joint_names = self.gripper_joint

        point = JointTrajectoryPoint()

        point.positions = [
            float(self.gripper)
        ]

        point.time_from_start = Duration(
            sec=0,
            nanosec=100000000
        )

        msg.points = [point]

        self.gripper_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ScaraJoystick()

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
