cmake_minimum_required(VERSION 3.8)
project(scara_description)

find_package(ament_cmake REQUIRED)

# Source folders are grouped by what they are for (mujoco/, gazebo/, real_robot/, server/, common/);
# the install keeps the usual ROS layout, so launch files and `ros2 run` work as before.
install(
  DIRECTORY urdf meshes launch config gesture_control
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY gazebo/worlds DESTINATION share/${PROJECT_NAME})          # -> share/.../worlds
install(DIRECTORY real_robot/firmware DESTINATION share/${PROJECT_NAME})    # -> share/.../firmware
install(DIRECTORY mujoco/models DESTINATION share/${PROJECT_NAME}/mujoco)   # -> share/.../mujoco/models

# the control page (browser GUI = the phone app's web page)
install(
  DIRECTORY app/ui
  DESTINATION share/${PROJECT_NAME}/app
)

# Executables + the Python modules they import (installed side by side)
install(
  PROGRAMS
    common/scara_scene.py
    common/scara_kinematics.py
    common/scara_models.py
    common/scara_vision.py
    common/scara_sorter_core.py
    gazebo/scara_vision_node.py
    gazebo/scara_color_sorter.py
    gazebo/scara_puck_spawner.py
    gazebo/scara_ask.py
    common/scara_voice.py
    common/scara_feeder.py
    common/scara_twin_protocol.py
    real_robot/scara_twin_bridge.py
    server/scara_twin_server.py
    gazebo/scara_rviz_scene.py
    common/scara_safety.py
    mujoco/scara_mujoco_ros.py
    gazebo/scara_doctor.py
    mujoco/mujoco_twin.py
    gazebo/pick_and_place.py
    gazebo/scara_teleop.py
    common/scara_joy_core.py
    gazebo/scara_joystick.py
    mujoco/launch_mujoco.py
    mujoco/scara_mujoco_teleop.py
    mujoco/scara_mujoco_joystick.py
    real_robot/scara_cam2d.py
    real_robot/scara_camera_setup.py
    real_robot/scara_hw_cell.py
    gazebo/scara_pose_relay.py
    gesture_control/gesture_core.py
    gesture_control/gesture_control_mujoco.py
    gesture_control/gesture_control_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
