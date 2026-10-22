cmake_minimum_required(VERSION 3.8)
project(scara_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch config worlds gesture_control firmware
  DESTINATION share/${PROJECT_NAME}
)

# the control page (browser GUI = the phone app's web page)
install(
  DIRECTORY app/ui
  DESTINATION share/${PROJECT_NAME}/app
)

# Executables + the Python modules they import (installed side by side)
install(
  PROGRAMS
    scripts/scara_scene.py
    scripts/scara_kinematics.py
    scripts/scara_models.py
    scripts/scara_vision.py
    scripts/scara_sorter_core.py
    scripts/scara_vision_node.py
    scripts/scara_color_sorter.py
    scripts/scara_puck_spawner.py
    scripts/scara_ask.py
    scripts/scara_voice.py
    scripts/scara_feeder.py
    scripts/scara_twin_protocol.py
    scripts/scara_twin_bridge.py
    scripts/scara_twin_server.py
    scripts/scara_rviz_scene.py
    scripts/scara_safety.py
    scripts/scara_mujoco_ros.py
    scripts/scara_doctor.py
    scripts/mujoco_twin.py
    scripts/pick_and_place.py
    scripts/scara_teleop.py
    scripts/scara_joy_core.py
    scripts/scara_joystick.py
    scripts/launch_mujoco.py
    scripts/scara_mujoco_teleop.py
    scripts/scara_mujoco_joystick.py
    scripts/scara_cam2d.py
    scripts/scara_camera_setup.py
    scripts/scara_hw_cell.py
    scripts/scara_pose_relay.py
    gesture_control/gesture_core.py
    gesture_control/gesture_control_mujoco.py
    gesture_control/gesture_control_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
