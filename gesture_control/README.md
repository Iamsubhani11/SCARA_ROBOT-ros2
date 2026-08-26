# 🖐️ SCARA Robot AI Hand Gesture Control

This folder contains the **AI Webcam Hand Gesture Teleoperation** nodes for controlling the SCARA robot in both **MuJoCo 3.12** and **Gazebo Ignition** simulations using OpenCV and MediaPipe.

---

## 🖐️ Gesture Control Mapping & How to Operate

### Step 1: Select Active Joint by Extended Finger Count
- ☝️ **1 Finger**: Select Joint 1 - **Base Column (`column_joint`)**
- ✌️ **2 Fingers**: Select Joint 2 - **Z-Axis Elevation (`shoulder_joint`)**
- 🤟 **3 Fingers**: Select Joint 3 - **Forearm Elbow (`forearm_joint`)**
- 🖖 **4 Fingers**: Select Joint 4 - **Wrist Rotation (`wrist_joint`)**
- 🖐️ **5 Fingers**: Select Joint 5 - **Gripper (`left_finger_joint`)**

### Step 2: Choose Motion Direction by Tilting Hand
- ↩️ **Tilt Hand LEFT**: Move Selected Joint **LEFT / DOWN / OPEN**
- ⏹️ **Keep Hand LEVEL**: **HOLD / STOP** Selected Joint Position
- ↪️ **Tilt Hand RIGHT**: Move Selected Joint **RIGHT / UP / CLOSE**

---

## 🛠️ Prerequisites & Installation

Install MediaPipe, OpenCV, NumPy 1.x, and MuJoCo:

```bash
python3 -m pip install "numpy<2" "mediapipe==0.10.14" "opencv-python==4.9.0.80" mujoco
```

---

## 🚀 Gesture Control Launch Commands

### 🎮 1. Launch Gesture Control in MuJoCo
Run this command to control the SCARA robot in **MuJoCo 3.12**:

```bash
cd ~/scara_description_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run scara_description gesture_control_mujoco.py
```

---

### 🤖 2. Launch Gesture Control in Gazebo (ROS 2)
Run this command to control the SCARA robot in **Gazebo Ignition**:

```bash
cd ~/scara_description_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run scara_description gesture_control_ros2.py
```
