# SCARA Robot Simulation Workspace (ROS 2 Humble, Gazebo Harmonic & MuJoCo)

A ROS 2 Humble workspace for a **4-DOF SCARA Robot** with:

- Column/Base rotation
- Shoulder/Z-axis movement
- Forearm rotation
- Wrist rotation
- Two-finger gripper
- Conveyor belt
- Yellow puck payload
- Xbox joystick control
- AI hand gesture control
- MuJoCo simulation
- Gazebo Harmonic simulation
- Automated pick-and-place support

---

# 🎮 CONTROL METHODS

The project supports both **MuJoCo** and **Gazebo Harmonic**.

---

# 1. 🎮 Xbox Joystick Control — MuJoCo

## Terminal 1 — Start Xbox joystick driver

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node

erminal 2 — Start MuJoCo joystick controller
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/scara_mujoco_joystick.py
Xbox Controls
Xbox Control	SCARA Function
Left Stick X	Column / Base Rotation
Left Stick Y	Shoulder / Z
Right Stick X	Forearm
Right Stick Y	Wrist
A	Close Gripper
B	Open Gripper
Y	Home
X	Hold
2. 🖐️ Hand Gesture Control — MuJoCo

Run:

cd ~/Scara_robot
python3 gesture_control/gesture_control_mujoco.py

This starts the MediaPipe hand gesture controller for the MuJoCo simulation.

3. 🤖 Gazebo Harmonic Simulation

Gazebo uses:

ROS 2 Humble
Gazebo Harmonic / Gazebo Sim 8
gz_ros2_control
JointTrajectoryController
JointStateBroadcaster
Terminal 1 — Launch Gazebo
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 launch scara_description gazebo.launch.py

Keep this terminal running.

4. 🎮 Xbox Joystick Control — Gazebo Harmonic
Terminal 1 — Gazebo
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 launch scara_description gazebo.launch.py
Terminal 2 — Xbox joystick driver
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node

Keep this terminal running.

Terminal 3 — Gazebo joystick controller
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
python3 scripts/scara_joystick.py
Xbox Controls
Xbox Control	SCARA Function
Left Stick X	Column / Base Rotation
Left Stick Y	Shoulder / Z
Right Stick X	Forearm
Right Stick Y	Wrist
A	Close BOTH gripper fingers
B	Open BOTH gripper fingers
Y	Home
X	Hold
Important

The Gazebo joystick controller directly commands both:

left_finger_joint
right_finger_joint

The Gazebo version does not depend on the unsupported physics-engine mimic constraint.

5. 🖐️ Hand Gesture Control — Gazebo Harmonic
Terminal 1 — Launch Gazebo
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 launch scara_description gazebo.launch.py
Terminal 2 — Start Gazebo gesture controller
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 gesture_control/gesture_control_ros2.py
Important

Do not run the Gazebo joystick controller and Gazebo gesture controller at the same time because both send commands to the same Gazebo controllers.

6. ⌨️ Keyboard Teleoperation — Gazebo
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 scripts/scara_teleop.py
7. ⌨️ Keyboard Teleoperation — MuJoCo
cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/scara_mujoco_teleop.py
🦾 CONTROLLER CHECK — GAZEBO

After launching Gazebo, verify that all controllers are active:

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash

ros2 control list_controllers

Expected:

gripper_controller      ... active
joint_state_broadcaster ... active
arm_controller          ... active

Check hardware interfaces:

ros2 control list_hardware_interfaces

Expected gripper command interfaces:

left_finger_joint/position  [available] [claimed]
right_finger_joint/position [available] [claimed]
🧪 MANUAL GAZEBO JOINT TESTS
Column
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['column_joint']
points:
- positions: [1.0]
  time_from_start: {sec: 2}
"

Return home:

ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['column_joint']
points:
- positions: [0.0]
  time_from_start: {sec: 2}
"
Shoulder / Z
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['shoulder_joint']
points:
- positions: [-0.10]
  time_from_start: {sec: 2}
"
Forearm
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['forearm_joint']
points:
- positions: [1.0]
  time_from_start: {sec: 2}
"
Wrist
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['wrist_joint']
points:
- positions: [1.0]
  time_from_start: {sec: 2}
"
Close Gripper
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['left_finger_joint','right_finger_joint']
points:
- positions: [-0.05,-0.05]
  time_from_start: {sec: 2}
"
Open Gripper
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['left_finger_joint','right_finger_joint']
points:
- positions: [0.0,0.0]
  time_from_start: {sec: 2}
"
🖐️ AI HAND GESTURE CONTROL

Gesture control files:

gesture_control/
├── gesture_control_mujoco.py
├── gesture_control_ros2.py
└── README.md

MuJoCo:

cd ~/Scara_robot
python3 gesture_control/gesture_control_mujoco.py

Gazebo:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 gesture_control/gesture_control_ros2.py
🌟 FEATURES
4-DOF SCARA robot
Revolute column joint
Prismatic shoulder/Z joint
Revolute forearm joint
Revolute wrist joint
Dual-finger gripper
Conveyor belt
Yellow puck payload
Gazebo Harmonic simulation
MuJoCo simulation
ROS 2 ros2_control
Xbox joystick control
AI hand gesture control
Keyboard teleoperation
Pick-and-place support
MoveIt-compatible robot description
📂 REPOSITORY STRUCTURE
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
🛠️ DEPENDENCIES
ROS 2 Humble
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
Gazebo Harmonic gz_ros2_control

The project uses a Harmonic-compatible build of gz_ros2_control.

Workspace:

~/gz_ros2_control_ws

Source it before launching Gazebo:

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
🐍 PYTHON DEPENDENCIES
python3 -m pip install "numpy<2" "mediapipe==0.10.14" "opencv-python==4.9.0.80" mujoco
🚀 INSTALLATION

Clone the repository:

git clone https://github.com/Iamsubhani11/scara-original-files.git ~/Scara_robot

Build:

cd ~/Scara_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install

Source:

source ~/Scara_robot/install/setup.bash
🎯 QUICK START
MuJoCo + Xbox Joystick

Terminal 1:

source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node

Terminal 2:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
python3 scripts/scara_mujoco_joystick.py
MuJoCo + Hand Gesture
cd ~/Scara_robot
python3 gesture_control/gesture_control_mujoco.py
Gazebo + Xbox Joystick

Terminal 1:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 launch scara_description gazebo.launch.py

Terminal 2:

source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run joy joy_node

Terminal 3:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
python3 scripts/scara_joystick.py
Gazebo + Hand Gesture

Terminal 1:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/gz_ros2_control_ws/install/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 launch scara_description gazebo.launch.py

Terminal 2:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
python3 gesture_control/gesture_control_ros2.py
📌 IMPORTANT PROJECT NOTES
Keep the MuJoCo and Gazebo control files separate.
Do not delete the MuJoCo joystick or gesture-control files.
Gazebo joystick control uses ROS 2 JointTrajectory commands.
Gazebo gripper control directly commands both finger joints.
Do not run Gazebo joystick control and Gazebo gesture control simultaneously.
Always source the appropriate workspaces before launching Gazebo.
The SCARA mesh alignment should not be changed without checking the existing reference geometry.
🤖 AUTOMATED PICK AND PLACE

Run:

cd ~/Scara_robot
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/Scara_robot/install/setup.bash
ros2 run scara_description pick_and_place.py
📜 LICENSE

This repository is licensed under the Apache 2.0 License.


