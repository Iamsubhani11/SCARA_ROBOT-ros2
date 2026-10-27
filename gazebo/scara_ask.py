#!/usr/bin/env python3
"""
Tell the Gazebo robot which colour to pick - by typing or by voice
(command mode: ros2 launch scara_description gazebo.launch.py mode:=command).

    python3 gazebo/scara_ask.py                  # type: red / "blue and green" / any / none / "3 red" / stop / start
                                                  # (a colour stays active until you give the next one)
    python3 gazebo/scara_ask.py --voice          # say "Hey SCARA" ... "the red one"
    python3 gazebo/scara_ask.py --voice --llm    # let ChatGPT understand free sentences (OPENAI_API_KEY)
    python3 gazebo/scara_ask.py --voice --stt whisper|openai|google

Publishes on /scara/pick_color for scara_color_sorter.py and prints the
robot's /scara/status messages.  See common/scara_voice.py for the details.
"""
import os as _os, sys as _sys  # noqa: E401 - the other folders of this package (common/, mujoco/ ...)
_sys.path[:0] = [_os.path.join(_os.path.dirname(_os.path.realpath(__file__)), '..', _d)
                 for _d in ('common', 'mujoco', 'gazebo', 'real_robot', 'server', 'gesture_control')]
import argparse
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from scara_voice import VoiceAssistant, parse_command, start_console  # noqa: F401  (parse_command re-exported)


class Asker(Node):
    def __init__(self):
        super().__init__('scara_ask')
        self.pub = self.create_publisher(String, '/scara/pick_color', 10)
        self.create_subscription(String, '/scara/status', lambda m: print(f'  robot: {m.data}', flush=True), 10)

    def send(self, cmd):
        self.pub.publish(String(data=cmd))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--voice', action='store_true', help='use the microphone')
    ap.add_argument('--llm', action='store_true', help='understand sentences with ChatGPT (OPENAI_API_KEY)')
    ap.add_argument('--stt', default='auto', help='speech-to-text: auto | whisper | openai | google')
    ap.add_argument('--quiet', action='store_true', help='no spoken replies')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Asker()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    assistant = VoiceAssistant(node.send, speak=not args.quiet, require_wake=args.voice, use_llm=args.llm)
    try:
        if args.voice:
            try:
                assistant.listen_forever(engine=args.stt)
            except ImportError:
                print('SpeechRecognition / PyAudio missing:  pip install SpeechRecognition pyaudio  - falling back to typing')
            except OSError as e:
                print(f'No microphone ({e}) - falling back to typing')
        assistant.require_wake = False
        print('Type a colour to pick - it stays active until the next one '
              '(red / green / blue / yellow / any / none / "3 red" / stop / start), Ctrl+C to quit')
        for line in sys.stdin:
            assistant.feed(line)
        time.sleep(0.5)                      # let the last message go out
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
