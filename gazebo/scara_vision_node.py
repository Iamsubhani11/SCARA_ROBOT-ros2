#!/usr/bin/env python3
"""
ROS 2 vision node (Gazebo).

Subscribes   /fixed_camera/image, /fixed_camera/depth_image, /fixed_camera/camera_info
             /arm_camera/image                       (display only)
Publishes    /scara/detections   std_msgs/String     JSON list of pucks on the belt:
             [{"color": "red", "x": .., "y": .., "z": .., "u": .., "v": .., "depth": ..}, ...]

Shows two OpenCV windows like the Mechamind Labs videos:
"Fixed Camera Box Detector" and "SCARA Arm Camera"  (parameter show:=false to hide).

No cv_bridge needed - images are converted with NumPy.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

import scara_vision as V


def image_to_numpy(msg):
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        ch = 4 if enc.endswith('a8') else 3
        a = np.frombuffer(msg.data, np.uint8).reshape(h, msg.step)[:, :w * ch].reshape(h, w, ch)
        if enc == 'rgb8':
            return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR)
        if enc == 'bgra8':
            return cv2.cvtColor(a, cv2.COLOR_BGRA2BGR)
        return a.copy()
    if enc == '32fc1':
        dt = np.dtype('>f4') if msg.is_bigendian else np.dtype('<f4')
        return np.frombuffer(msg.data, dt).reshape(h, msg.step // 4)[:, :w].astype(np.float32)
    if enc == '16uc1':
        dt = np.dtype('>u2') if msg.is_bigendian else np.dtype('<u2')
        return np.frombuffer(msg.data, dt).reshape(h, msg.step // 2)[:, :w].astype(np.float32) / 1000.0
    raise ValueError(f'unsupported image encoding {msg.encoding}')


class VisionNode(Node):
    def __init__(self):
        super().__init__('scara_vision')
        self.declare_parameter('show', True)
        self.declare_parameter('rate', 10.0)
        self.show = bool(self.get_parameter('show').value)
        if self.show and not V.can_show():
            self.get_logger().info('No screen here (headless) - camera windows off, detection keeps running')
            self.show = False
        self.rgb = None
        self.depth = None
        self.arm = None
        self.cam = V.fixed_camera_model()
        self.have_info = False
        self.create_subscription(Image, '/fixed_camera/image', self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, '/fixed_camera/depth_image', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/fixed_camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/arm_camera/image', self.on_arm, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/scara/detections', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.process)
        self.warned = False
        self.get_logger().info('Vision node up: waiting for /fixed_camera/image and /fixed_camera/depth_image')

    def on_rgb(self, msg):
        self.rgb = image_to_numpy(msg)
        self.rgb_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # when the camera took it (sim time)

    def on_depth(self, msg):
        self.depth = image_to_numpy(msg)

    def on_arm(self, msg):
        self.arm = image_to_numpy(msg)

    def on_info(self, msg):
        if not self.have_info and msg.k[0] > 0:
            K = np.array(msg.k, dtype=float).reshape(3, 3)
            self.cam = V.fixed_camera_model(K)
            self.have_info = True
            self.get_logger().info(f'Using camera intrinsics fx={K[0, 0]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}')

    def process(self):
        if self.rgb is None or self.depth is None:
            if not self.warned and self.get_clock().now().nanoseconds > 5e9:
                self.get_logger().warn('No camera images yet - is the ros_gz bridge running?')
                self.warned = True
            return
        if self.rgb.shape[:2] != self.depth.shape[:2]:
            return
        dets = V.detect_pucks(self.rgb, self.depth, self.cam)
        # 's' (position along the belt) is what the sorter and the spawner work with - it must be in the message
        # 't': when the picture was taken - belt tracking predicts where the puck is NOW from it
        # (the message arrives later than the picture; at 0.08 m/s even 0.15 s is 12 mm)
        t = getattr(self, 'rgb_t', None)
        out = [dict({k: d[k] for k in ('color', 'x', 'y', 'z', 's', 'yaw', 'u', 'v', 'depth', 'area')}, **({'t': t} if t else {}))
               for d in dets]
        self.pub.publish(String(data=json.dumps(out)))
        if self.show:
            cv2.imshow('Fixed Camera Box Detector', V.draw_detections(self.rgb, dets))
            if self.arm is not None:
                arm = self.arm.copy()
                cv2.putText(arm, 'SCARA Arm Camera', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
                cv2.imshow('SCARA Arm Camera', arm)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
