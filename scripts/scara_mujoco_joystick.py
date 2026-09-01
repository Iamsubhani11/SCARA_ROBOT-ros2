#!/usr/bin/env python3

import time
import threading

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ScaraMuJoCoJoystick(Node):

    def __init__(self, model, data):
        super().__init__('scara_mujoco_joystick')

        self.model = model
        self.data = data

        # ============================================================
        # Subscribe to Xbox controller
        # ============================================================

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ============================================================
        # Current SCARA target positions
        # ============================================================

        self.column = 0.0
        self.shoulder = 0.0
        self.forearm = 0.0
        self.wrist = 0.0

        # Gripper
        self.gripper = -0.05

        # ============================================================
        # SCARA limits
        # ============================================================

        self.COLUMN_MIN = -2.0
        self.COLUMN_MAX = 2.0

        self.SHOULDER_MIN = -0.15
        self.SHOULDER_MAX = 0.02

        self.FOREARM_MIN = -2.0
        self.FOREARM_MAX = 2.0

        self.WRIST_MIN = -4.71
        self.WRIST_MAX = 4.71

        self.GRIPPER_MIN = -0.05
        self.GRIPPER_MAX = 0.0

        # ============================================================
        # Joystick settings
        # ============================================================

        self.DEADZONE = 0.12

        # Movement speed per control update
        self.COLUMN_SPEED = 0.020
        self.SHOULDER_SPEED = 0.003
        self.FOREARM_SPEED = 0.020
        self.WRIST_SPEED = 0.030

        # ============================================================
        # Latest joystick state
        # ============================================================

        self.axes = [0.0] * 8
        self.buttons = [0] * 11

        self.joy_received = False

        # Prevent repeated HOME logging
        self.previous_y = 0

        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            '       SCARA MuJoCo XBOX JOYSTICK CONTROL'
        )
        self.get_logger().info(
            '===================================================='
        )
        self.get_logger().info(
            'Left Stick X  -> Column'
        )
        self.get_logger().info(
            'Left Stick Y  -> Z / Shoulder'
        )
        self.get_logger().info(
            'Right Stick X -> Forearm'
        )
        self.get_logger().info(
            'Right Stick Y -> Wrist'
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
            'X -> HOLD'
        )
        self.get_logger().info(
            '===================================================='
        )

    # ================================================================
    # Joystick callback
    # ================================================================

    def joy_callback(self, msg):

        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

        self.joy_received = True

    # ================================================================
    # Dead zone
    # ================================================================

    def deadzone(self, value):

        if abs(value) < self.DEADZONE:
            return 0.0

        sign = 1.0 if value > 0 else -1.0

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
    # HOME
    # ================================================================

    def home(self):

        self.column = 0.0
        self.shoulder = 0.0
        self.forearm = 0.0
        self.wrist = 0.0
        self.gripper = -0.05

        self.get_logger().info(
            'SCARA HOME POSITION'
        )

    # ================================================================
    # Update MuJoCo control
    # ================================================================

    def update_control(self):

        if len(self.axes) < 5:
            return

        if len(self.buttons) < 4:
            return

        # ------------------------------------------------------------
        # Xbox axes
        # ------------------------------------------------------------

        left_x = self.deadzone(self.axes[0])
        left_y = self.deadzone(self.axes[1])

        right_x = self.deadzone(self.axes[3])
        right_y = self.deadzone(self.axes[4])

        # ------------------------------------------------------------
        # COLUMN
        # Left stick X
        # ------------------------------------------------------------

        self.column += (
            left_x * self.COLUMN_SPEED
        )

        self.column = self.clamp(
            self.column,
            self.COLUMN_MIN,
            self.COLUMN_MAX
        )

        # ------------------------------------------------------------
        # Z / SHOULDER
        # Left stick Y
        #
        # Joystick UP normally gives -1
        # Joystick DOWN normally gives +1
        #
        # Invert so UP = positive movement.
        # ------------------------------------------------------------

        self.shoulder += (
            -left_y * self.SHOULDER_SPEED
        )

        self.shoulder = self.clamp(
            self.shoulder,
            self.SHOULDER_MIN,
            self.SHOULDER_MAX
        )

        # ------------------------------------------------------------
        # FOREARM
        # Right stick X
        # ------------------------------------------------------------

        self.forearm += (
            right_x * self.FOREARM_SPEED
        )

        self.forearm = self.clamp(
            self.forearm,
            self.FOREARM_MIN,
            self.FOREARM_MAX
        )

        # ------------------------------------------------------------
        # WRIST
        # Right stick Y
        # ------------------------------------------------------------

        self.wrist += (
            -right_y * self.WRIST_SPEED
        )

        self.wrist = self.clamp(
            self.wrist,
            self.WRIST_MIN,
            self.WRIST_MAX
        )

        # ------------------------------------------------------------
        # A = Button 0
        # CLOSE
        # ------------------------------------------------------------

        if self.buttons[0] == 1:

            self.gripper = self.GRIPPER_MAX

        # ------------------------------------------------------------
        # B = Button 1
        # OPEN
        # ------------------------------------------------------------

        if self.buttons[1] == 1:

            self.gripper = self.GRIPPER_MIN

        # ------------------------------------------------------------
        # Y = Button 3
        # HOME
        # ------------------------------------------------------------

        current_y = self.buttons[3]

        if current_y == 1 and self.previous_y == 0:

            self.home()

        self.previous_y = current_y

        # ------------------------------------------------------------
        # X = Button 2
        #
        # X is HOLD.
        # No target changes are made by X itself.
        # ------------------------------------------------------------

        # ------------------------------------------------------------
        # Write directly to MuJoCo
        # ------------------------------------------------------------

        if self.model.nu >= 6:

            self.data.ctrl[0] = self.column
            print(
    f'\rTARGET={self.column:+.3f} '
    f'ACTUAL={self.data.qpos[0]:+.3f}',
    end='',
    flush=True
)
            self.data.ctrl[1] = self.shoulder
            self.data.ctrl[2] = self.forearm
            self.data.ctrl[3] = self.wrist

            self.data.ctrl[4] = self.gripper
            self.data.ctrl[5] = self.gripper

    # ================================================================
    # Display current state
    # ================================================================

    def print_status(self):

        print(
            f'\r'
            f'Column: {self.column:+.2f} | '
            f'Z: {self.shoulder:+.3f} | '
            f'Forearm: {self.forearm:+.2f} | '
            f'Wrist: {self.wrist:+.2f} | '
            f'Gripper: {self.gripper:+.2f}',
            end='',
            flush=True
        )


def main():

    # ================================================================
    # ROS 2 initialization
    # ================================================================

    rclpy.init()

    # ================================================================
    # Load YOUR existing MuJoCo SCARA scene
    # ================================================================

    xml_path = 'urdf/scara_scene_mujoco.xml'

    print()
    print('==============================================')
    print('     SCARA MuJoCo XBOX JOYSTICK')
    print('==============================================')
    print()
    print('Loading:')
    print(xml_path)
    print()

    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
    except Exception as e:

        print('ERROR: Could not load MuJoCo model.')
        print(e)

        rclpy.shutdown()
        return

    data = mujoco.MjData(model)

    # ================================================================
    # Create joystick controller
    # ================================================================

    node = ScaraMuJoCoJoystick(model, data)

    # ================================================================
    # Start MuJoCo viewer
    # ================================================================

    with mujoco.viewer.launch_passive(model, data) as viewer:

        print('MuJoCo SCARA started.')
        print()
        print('Waiting for Xbox joystick...')
        print()
        print('Controls:')
        print('  Left Stick X   = Column')
        print('  Left Stick Y   = Z')
        print('  Right Stick X  = Forearm')
        print('  Right Stick Y  = Wrist')
        print('  A              = Close gripper')
        print('  B              = Open gripper')
        print('  Y              = Home')
        print('  X              = Hold')
        print()

        last_status = time.time()

        try:

            while viewer.is_running():

                # ----------------------------------------------------
                # Process ROS /joy messages
                # ----------------------------------------------------

                rclpy.spin_once(
                    node,
                    timeout_sec=0.0
                )

                # ----------------------------------------------------
                # Update SCARA
                # ----------------------------------------------------

                node.update_control()

                # ----------------------------------------------------
                # Step MuJoCo simulation
                # ----------------------------------------------------

                mujoco.mj_step(
                    model,
                    data
                )

                # ----------------------------------------------------
                # Update viewer
                # ----------------------------------------------------

                viewer.sync()

                # ----------------------------------------------------
                # Status approximately every 0.5 sec
                # ----------------------------------------------------

                now = time.time()

                if now - last_status > 0.5:

                    node.print_status()

                    last_status = now

                # ----------------------------------------------------
                # Small delay
                # ----------------------------------------------------

                time.sleep(0.01)

        except KeyboardInterrupt:

            print()
            print()
            print('Joystick controller stopped.')

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
