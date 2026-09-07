# SCARA Robot Simulation Workspace

**ROS 2 Humble · Gazebo Harmonic · MuJoCo**

A ROS 2 workspace for a **4-DOF SCARA robot** with dual-finger gripper, conveyor belt, puck payload, Xbox joystick control, hand-gesture control, and pick-and-place support.

## Robot Features

- 4-DOF SCARA robot
- Column / base rotation
- Shoulder / Z-axis movement
- Forearm rotation
- Wrist rotation
- Dual-finger gripper
- Conveyor belt
- Yellow puck payload
- Xbox joystick control
- AI hand-gesture control
- MuJoCo simulation
- Gazebo Harmonic simulation
- ROS 2 `ros2_control`
- Pick-and-place support
- MoveIt-compatible robot description

---

# 🎮 Control Methods

The project supports two simulation environments:

| Simulation | Joystick | Hand Gesture | Keyboard |
|---|---|---|---|
| **MuJoCo** | ✅ | ✅ | ✅ |
| **Gazebo Harmonic** | ✅ | ✅ | ✅ |

> **Important:** Do not run the Gazebo joystick controller and Gazebo gesture controller at the same time. Both command the same Gazebo controllers.

---

# 1. 🎮 Xbox Joystick — MuJoCo

## Terminal 1 — Start Xbox joystick driver

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node
```

Keep this terminal running.

## Terminal 2 — Start MuJoCo joystick controller

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/scara_mujoco_joystick.py
```

## Xbox Controls

| Xbox Control | SCARA Function |
|---|---|
| Left Stick X | Column / Base Rotation |
| Left Stick Y | Shoulder / Z |
| Right Stick X | Forearm |
| Right Stick Y | Wrist |
| A | Close Gripper |
| B | Open Gripper |
| Y | Home |
| X | Hold |

---

# 2. 🖐️ Hand Gesture Control — MuJoCo

Run:

```bash
cd ~/Scara_robot
python3 gesture_control/gesture_control_mujoco.py
```

This starts the MediaPipe hand-gesture controller for the MuJoCo simulation.

---

# 3. 🤖 Gazebo Harmonic

## Gazebo Components

The Gazebo simulation uses:

- ROS 2 Humble
- Gazebo Harmonic / Gazebo Sim 8
- `gz_ros2_control`
- `JointTrajectoryController`
- `JointStateBroadcaster`

## Terminal 1 — Launch Gazebo

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$LD_LIBRARY_PATH

ros2 launch scara_description gazebo.launch.py
```

Keep this terminal running.

---

# 4. 🎮 Xbox Joystick — Gazebo Harmonic

## Terminal 1 — Launch Gazebo

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$LD_LIBRARY_PATH

ros2 launch scara_description gazebo.launch.py
```

## Terminal 2 — Start Xbox joystick driver

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node
```

Keep this terminal running.

## Terminal 3 — Start Gazebo joystick controller

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
python3 scripts/scara_joystick.py
```

## Xbox Controls

| Xbox Control | SCARA Function |
|---|---|
| Left Stick X | Column / Base Rotation |
| Left Stick Y | Shoulder / Z |
| Right Stick X | Forearm |
| Right Stick Y | Wrist |
| A | Close **both** gripper fingers |
| B | Open **both** gripper fingers |
| Y | Home |
| X | Hold |

### Gazebo Gripper

The Gazebo joystick controller directly commands:

```text
left_finger_joint
right_finger_joint
```

The Gazebo version does **not** depend on the unsupported physics-engine mimic constraint.

---

# 5. 🖐️ Hand Gesture Control — Gazebo Harmonic

## Terminal 1 — Launch Gazebo

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$LD_LIBRARY_PATH

ros2 launch scara_description gazebo.launch.py
```

## Terminal 2 — Start Gazebo gesture controller

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 gesture_control/gesture_control_ros2.py
```

---

# 6. ⌨️ Keyboard Teleoperation

## Gazebo

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 scripts/scara_teleop.py
```

## MuJoCo

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/scara_mujoco_teleop.py
```

---

# 🦾 Controller Check — Gazebo

After launching Gazebo:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
```

Check controllers:

```bash
ros2 control list_controllers
```

Expected:

```text
gripper_controller       ... active
joint_state_broadcaster  ... active
arm_controller           ... active
```

Check hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

Expected gripper command interfaces:

```text
left_finger_joint/position  [available] [claimed]
right_finger_joint/position [available] [claimed]
```

---

# 🧪 Manual Gazebo Joint Tests

## Column

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['column_joint']
points:
- positions: [1.0]
  time_from_start: {sec: 2}"
```

### Return Home

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['column_joint']
points:
- positions: [0.0]
  time_from_start: {sec: 2}"
```

## Shoulder / Z

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['shoulder_joint']
points:
- positions: [-0.10]
  time_from_start: {sec: 2}"
```

## Forearm

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['forearm_joint']
points:
- positions: [1.0]
  time_from_start: {sec: 2}"
```

## Wrist

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['wrist_joint']
points:
- positions: [1.0]
  time_from_start: {sec: 2}"
```

## Close Gripper

```bash
ros2 topic pub --once /gripper_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['left_finger_joint','right_finger_joint']
points:
- positions: [-0.05,-0.05]
  time_from_start: {sec: 2}"
```

## Open Gripper

```bash
ros2 topic pub --once /gripper_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory \
"joint_names: ['left_finger_joint','right_finger_joint']
points:
- positions: [0.0,0.0]
  time_from_start: {sec: 2}"
```

---

# 🖐️ AI Hand Gesture Control

Gesture-control files:

```text
gesture_control/
├── gesture_control_mujoco.py
├── gesture_control_ros2.py
└── README.md
```

## MuJoCo

```bash
cd ~/Scara_robot
python3 gesture_control/gesture_control_mujoco.py
```

## Gazebo

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 gesture_control/gesture_control_ros2.py
```

---

# 🌟 Features

- **4-DOF SCARA robot**
- Revolute column joint
- Prismatic shoulder / Z joint
- Revolute forearm joint
- Revolute wrist joint
- Dual-finger gripper
- Conveyor belt
- Yellow puck payload
- Gazebo Harmonic simulation
- MuJoCo simulation
- ROS 2 `ros2_control`
- Xbox joystick control
- AI hand-gesture control
- Keyboard teleoperation
- Pick-and-place support
- MoveIt-compatible robot description

---

# 📂 Repository Structure

```text
Scara_robot/
├── config/
│   └── scara_controllers.yaml
│
├── gesture_control/
│   ├── gesture_control_mujoco.py
│   ├── gesture_control_ros2.py
│   └── README.md
│
├── launch/
│   └── gazebo.launch.py
│
├── meshes/
│   ├── base_link.stl
│   ├── column_link.stl
│   ├── shoulder_link.stl
│   ├── forearm_link.stl
│   ├── wrist_link.stl
│   ├── left_finger_link.stl
│   ├── right_finger_link.stl
│   └── conveyor/
│
├── scripts/
│   ├── scara_joystick.py
│   ├── scara_mujoco_joystick.py
│   ├── scara_teleop.py
│   ├── scara_mujoco_teleop.py
│   ├── pick_and_place.py
│   └── launch_mujoco.py
│
├── urdf/
│   ├── scara.urdf.xacro
│   ├── arm.xacro
│   ├── new_conveyor.xacro
│   ├── table_platform.xacro
│   ├── yellow_puck.sdf
│   └── scara_scene_mujoco.xml
│
├── CMakeLists.txt
└── package.xml
```

---

# 🛠️ Dependencies

## ROS 2 Humble

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joy
```

## Gazebo Harmonic — `gz_ros2_control`

The project uses a **Harmonic-compatible build** of `gz_ros2_control`.

Workspace:

```text
~/gz_ros2_control_ws
```

Before launching Gazebo:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
```

## Python

```bash
python3 -m pip install \
  "numpy<2" \
  "mediapipe==0.10.14" \
  "opencv-python==4.9.0.80" \
  mujoco
```

---

# 🚀 Installation

## Clone the repository

```bash
git clone https://github.com/Iamsubhani11/scara-original-files.git ~/Scara_robot
```

## Build

```bash
cd ~/Scara_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Source the workspace

```bash
source ~/Scara_robot/install/setup.bash
```

---

# 🎯 Quick Start

## MuJoCo + Xbox Joystick

### Terminal 1

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node
```

### Terminal 2

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/scara_mujoco_joystick.py
```

## MuJoCo + Hand Gesture

```bash
cd ~/Scara_robot
python3 gesture_control/gesture_control_mujoco.py
```

## Gazebo + Xbox Joystick

### Terminal 1

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=$HOME/gz_ros2_control_ws/install/gz_ros2_control/lib:$LD_LIBRARY_PATH

ros2 launch scara_description gazebo.launch.py
```

### Terminal 2

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node
```

### Terminal 3

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
python3 scripts/scara_joystick.py
```

## Gazebo + Hand Gesture

### Terminal 1

Use the same Gazebo launch command shown above.

### Terminal 2

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 gesture_control/gesture_control_ros2.py
```

---

# 📌 Important Project Notes

1. Keep the MuJoCo and Gazebo control files separate.
2. Do not delete the MuJoCo joystick or gesture-control files.
3. Gazebo joystick control uses ROS 2 `JointTrajectory` commands.
4. Gazebo gripper control directly commands both finger joints.
5. Do not run Gazebo joystick and Gazebo gesture control simultaneously.
6. Always source the appropriate workspaces before launching Gazebo.
7. Do not change the SCARA mesh alignment without checking the existing reference geometry.

---

# 🤖 Automated Pick and Place

Run:

```bash
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 run scara_description pick_and_place.py
```

---

# 📜 License

This repository is licensed under the **Apache 2.0 License**.
