"""
The ROS 2 cell on MuJoCo physics instead of Gazebo (same topics / actions as gazebo.launch.py).
Useful when Gazebo is not installed or too slow - every ROS tool of this package works with it.

    ros2 launch scara_description mujoco_ros.launch.py                      # sorting mode (4 bins)
    ros2 launch scara_description mujoco_ros.launch.py mode:=command        # one bin, then: scara_ask.py
    ros2 launch scara_description mujoco_ros.launch.py mode:=manual         # table: joystick / teleop / gestures / twin GUI

Arguments
  mode        sort | command | manual             (default sort)
  viewer      true = MuJoCo viewer window          (default true)
  rviz        true = RViz with the robot + cell    (default false)
  detections  truth | vision (needs OpenCV)        (default vision)
  pucks       0 = endless, N = drop N pucks         (default 0)
  twin        true = the real robot copies the sim  (default false)
  port        Arduino serial port                   (default /dev/ttyACM0)
  fake        true = software robot, no hardware    (default false)

Digital twin on MuJoCo + RViz (the real robot is the see-through ghost):
    ros2 launch scara_description mujoco_ros.launch.py mode:=manual twin:=true port:=/dev/ttyACM0 rviz:=true
    python3 gazebo/scara_joystick.py        # drive the sim, the robot follows
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def name_plate(pkg):
    """'true' when meshes/shoulder_name.stl exists (the named version of the robot)."""
    return 'true' if os.path.exists(os.path.join(pkg, 'meshes', 'shoulder_name.stl')) else 'false'


def generate_launch_description():
    pkg = get_package_share_directory('scara_description')
    mode = LaunchConfiguration('mode')
    viewer = LaunchConfiguration('viewer')
    rviz = LaunchConfiguration('rviz')
    cell = PythonExpression(["'", mode, "' != 'manual'"])
    description = ParameterValue(Command(['xacro ', os.path.join(pkg, 'urdf', 'scara.urdf.xacro'), ' name_plate:=', name_plate(pkg)]), value_type=str)

    sim = Node(package='scara_description', executable='scara_mujoco_ros.py', name='scara_mujoco_sim', output='screen',
               arguments=['--world', mode, '--detections', LaunchConfiguration('detections'),
                          '--pucks', LaunchConfiguration('pucks'),
                          PythonExpression(["'--viewer' if '", viewer, "' == 'true' else '--no-viewer'"])])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': description}])
    sorter = Node(package='scara_description', executable='scara_color_sorter.py', output='screen',
                  condition=IfCondition(cell), parameters=[{'mode': mode}])
    scene = Node(package='scara_description', executable='scara_rviz_scene.py', condition=IfCondition(rviz),
                 parameters=[{'cell': mode}])
    rviz_node = Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
                     arguments=['-d', os.path.join(pkg, 'config', 'twin.rviz')])
    twin = LaunchConfiguration('twin')
    ghost = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='real',
                 condition=IfCondition(twin), parameters=[{'robot_description': description, 'frame_prefix': 'real/'}])
    ghost_tf = Node(package='tf2_ros', executable='static_transform_publisher', condition=IfCondition(twin),
                    arguments=['--frame-id', 'world', '--child-frame-id', 'real/world'])
    bridge = Node(package='scara_description', executable='scara_twin_bridge.py', name='scara_twin_bridge',
                  output='screen', condition=IfCondition(twin),
                  parameters=[os.path.join(pkg, 'config', 'twin_hardware.yaml'),
                              {'port': LaunchConfiguration('port'), 'fake': LaunchConfiguration('fake'),
                               'mode': 'sim_to_real', 'target': 'gazebo'}])
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sort', description='sort | command | manual'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('detections', default_value='vision', description='truth | vision'),
        DeclareLaunchArgument('pucks', default_value='0'),
        DeclareLaunchArgument('twin', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('fake', default_value='false'),
        sim, rsp, scene, rviz_node, ghost, ghost_tf,
        TimerAction(period=3.0, actions=[sorter, bridge]),
    ])
