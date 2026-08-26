# SCARA Robot Simulation Workspace (ROS 2 Humble & MuJoCo,Gazebo)

A comprehensive ROS 2 Humble workspace featuring a **4-DOF SCARA Robot** mounted on a wooden workspace table platform, complete with a **conveyor belt system**, **yellow puck payload object**, **automated pick-and-place state machine**, **Finger Selection + Hand Tilt Gesture Control**, and dual simulation engine support (**Gazebo Ignition Fortress** and **MuJoCo 3.12**).

---

## 🖐️ Hand Gesture Control Commands

### 📦 1. Install Gesture Control Dependencies
Run this command to install the required MediaPipe, OpenCV, and NumPy versions:

```bash
python3 -m pip install "numpy<2" "mediapipe==0.10.14" "opencv-python==4.9.0.80" mujoco
```

---

### 🎮 2. Launch Gesture Control in MuJoCo Engine
Run this command to launch real-time AI webcam hand gesture control in **MuJoCo**:

```bash
cd ~/scara_description_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run scara_description gesture_control_mujoco.py
```

---

### 🤖 3. Launch Gesture Control in Gazebo (ROS 2)
Run this command to launch real-time AI webcam hand gesture control in **Gazebo Ignition**:

```bash
cd ~/scara_description_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run scara_description gesture_control_ros2.py
```

---

## 🖐️ Gesture Control Mapping & How to Operate

### Step 1: Select Active Joint by Extended Finger Count
- ☝️ **1 Finger**: Select Joint 1 - **Base Column (`column_joint`)**
- ✌️ **2 Fingers**: Select Joint 2 - **Z-Axis Elevation (`shoulder_joint`)**
- 🤟 **3 Fingers**: Select Joint 3 - **Forearm Elbow (`forearm_joint`)**
- 🖖 **4 Fingers**: Select Joint 4 - **Wrist Rotation (`wrist_joint`)**
- 🖐️ **5 Fingers**: Select Joint 5 - **Gripper (`left_finger_joint`)**

### Step 2: Choose Direction by Tilting Hand
- ↩️ **Tilt Hand LEFT**: Move Selected Joint **LEFT / DOWN / OPEN**
- ⏹️ **Keep Hand LEVEL**: **HOLD / STOP** Selected Joint Position
- ↪️ **Tilt Hand RIGHT**: Move Selected Joint **RIGHT / UP / CLOSE**

---

## 🌟 General Workspace Features

- 🦾 **4-DOF SCARA Robot Model**: Full kinematic chain including `column_joint`, `shoulder_joint` (prismatic Z-axis), `forearm_joint`, `wrist_joint`, and dual-finger parallel gripper.
- 🪵 **Workspace Table Platform**: Rigid base platform (`table_link`) supporting tabletop mounting for the robot and conveyor track.
- 📦 **Conveyor Belt System**: Transport conveyor belt track model with metallic side guard rails.
- 🟡 **Payload Pick-and-Place**: Round yellow payload puck object with top ring handle.
- 🎮 **Dual Simulation Engines**:
  - **Gazebo Sim 6 (Ignition Fortress)**: Full `ros2_control` hardware interface with `JointTrajectoryController` and `JointStateBroadcaster`.
  - **MuJoCo 3.12**: Fast, stable physics engine backend with native 3D interactive viewer and GUI control sliders.
- 🤖 **Automated Pick-and-Place State Machine**: 10-step trajectory state machine picking the yellow puck from the conveyor track and placing it on the wooden table platform.

---

## 📂 Repository Structure

```text
scara_description_ws/
├── config/
│   └── scara_controllers.yaml    # ros2_control JointTrajectoryController configuration
├── launch/
│   └── gazebo.launch.py          # Gazebo Ignition simulation launch file
├── meshes/
│   ├── base_link.stl, column_link.stl, shoulder_link.stl ...  # SCARA robot 3D meshes
│   └── conveyor/                 # Conveyor belt component STL meshes
├── scripts/
│   ├── scara_teleop.py           # Terminal teleop for Gazebo simulation
│   ├── pick_and_place.py         # Automated pick-and-place state machine node
│   ├── launch_mujoco.py          # Interactive MuJoCo simulation launcher & GUI mode
│   ├── scara_mujoco_teleop.py    # Terminal teleop for MuJoCo simulation
│   ├── gesture_control_mujoco.py # Finger Select + Hand Tilt Control for MuJoCo
│   └── gesture_control_ros2.py   # Finger Select + Hand Tilt Control for Gazebo/ROS 2
├── urdf/
│   ├── scara.urdf.xacro          # Root SCARA robot description file
│   ├── arm.xacro                 # Kinematic joints, links, table base, & ros2_control
│   ├── new_conveyor.xacro        # Transport conveyor belt track URDF model
│   ├── table_platform.xacro      # Wooden table platform URDF model
│   ├── yellow_puck.sdf           # Yellow payload puck SDF model
│   └── scara_scene_mujoco.xml    # MuJoCo MJCF full scene file
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Additional Commands

### Gazebo Simulation Launch:
```bash
cd ~/scara_description_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scara_description gazebo.launch.py
```

### Automated Pick and Place Execution:
```bash
ros2 run scara_description pick_and_place.py
```

### MuJoCo Interactive Slider GUI Mode:
```bash
ros2 run scara_description launch_mujoco.py
```

---

## 📜 License

This repository is licensed under the Apache 2.0 License.
