#!/usr/bin/env python3
"""
Ask the running cell for ONE puck and wait until it is in the bin.

    ros2 launch scara_description gazebo.launch.py mode:=command      # terminal 1
    python3 gazebo/pick_and_place.py red                               # terminal 2: next red puck
    python3 gazebo/pick_and_place.py red 3                             # three red pucks
    python3 gazebo/pick_and_place.py                                   # the next puck of any colour

It publishes "1 red" (a count, so the robot stops after it - a plain colour
stays active until the next one) on /scara/pick_color and prints /scara/status
until the robot reports the puck(s) placed.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    color = args[0].lower() if args else 'any'
    count = int(args[1]) if len(args) > 1 and args[1].isdigit() else 1
    rclpy.init()
    node = Node('scara_pick_once')
    done = {'ok': False, 'n': 0}

    def on_status(msg):
        print('[cell]', msg.data)
        if msg.data.startswith('Placed'):
            done['n'] += 1
            done['ok'] = done['n'] >= count

    node.create_subscription(String, '/scara/status', on_status, 10)
    pub = node.create_publisher(String, '/scara/pick_color', 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.2)       # wait for the sorter node
    if pub.get_subscription_count() == 0:
        print('No colour sorter is listening - start: ros2 launch scara_description gazebo.launch.py mode:=command')
    pub.publish(String(data=f'{count} {color}'))
    print(f'Asked for: {count} x {color}')
    t0 = time.time()
    while rclpy.ok() and not done['ok'] and time.time() - t0 < 300.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
