# SCARA Two-Hand Gesture Control

Webcam + MediaPipe, works in **Gazebo** (ROS 2) and **MuJoCo** with the same gestures.

| Hand | Movement | Robot |
|---|---|---|
| **Left** (blue) | left / right | base column rotation – the robot faces you, so the arm swings the same way as your hand |
| | up / down | Z height |
| **Right** (orange) | left / right | forearm |
| | tilt (roll) | wrist |
| | pinch thumb + index | close gripper (release = open) |
| either | fist | freeze that hand's joints |
| either | out of view | hold |

```bash
# Gazebo: start the manual table world (no conveyor, no sorter)
ros2 launch scara_description gazebo.launch.py mode:=manual
python3 gesture_control/gesture_control_ros2.py          # camera:=1 for another webcam: --ros-args -p camera:=1

# MuJoCo
python3 gesture_control/gesture_control_mujoco.py         # q = quit (manual table world, no conveyor)
```

Files

* `gesture_core.py` – the gesture → joint logic (shared, no ROS/MuJoCo inside)
* `gesture_control_ros2.py` – streams `/arm_controller/joint_trajectory` and commands **both** fingers
* `gesture_control_mujoco.py` – drives the MuJoCo actuators
* `../tools/test_gesture_core.py` – unit tests with synthetic hands

Why it changed: the old version picked a joint by counting fingers and moved it by tilting the
hand – but tilting changes MediaPipe's finger count, so the selected joint kept jumping. The
gripper direction was also reversed and only one finger moved in Gazebo.

Tuning (top of `gesture_core.py`): `LEFT_X`, `RIGHT_X`, `HAND_Y` (where in the picture each hand
works), `MAX_SPEED`, `PINCH_CLOSE` / `PINCH_OPEN`, `SMOOTH`.  If a direction feels reversed,
swap the two numbers of that joint's `*_RANGE`.
