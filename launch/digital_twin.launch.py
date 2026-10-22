"""
Digital twin of the HowToMechatronics SCARA (real robot <-> simulation).

    # simulation drives the real robot, Gazebo cell + colour sorter:
    ros2 launch scara_description digital_twin.launch.py port:=/dev/ttyACM0 cell:=sort
    # RViz only, move the sliders -> the real robot moves (ghost = measured real pose):
    ros2 launch scara_description digital_twin.launch.py sim:=rviz port:=/dev/ttyACM0
    # the real robot drives the simulation (teach knobs / its own GUI):
    ros2 launch scara_description digital_twin.launch.py mode:=real_to_sim sim:=rviz port:=/dev/ttyACM0
    # no hardware yet:
    ros2 launch scara_description digital_twin.launch.py sim:=rviz fake:=true

RViz shows the simulated robot (solid), the REAL robot as a see-through ghost
(/real/joint_states, counted steps from the Arduino) and the cell (table,
conveyor, bins, pucks) from scara_rviz_scene.py.
    # control from the SCARA Digital Twin phone app (rosbridge on port 9090):
    ros2 launch scara_description digital_twin.launch.py port:=/dev/ttyACM0 rosbridge:=true

Arguments
  mode      sim_to_real | real_to_sim                       (default sim_to_real)
  sim       gazebo | rviz                                   (default gazebo)
  port      Arduino serial port                             (default /dev/ttyACM0)
  fake      true = software robot, no hardware              (default false)
  homing    switches | zero | none                          (default switches: the 4 limit switches, K)
  cell      sort | command | manual                         (default manual)
            Gazebo world (sort = conveyor + colour bins, command = conveyor +
            one bin, manual = table with pucks) and the cell drawn in RViz
  rviz      show RViz with the real robot as a ghost        (default true)
  rosbridge start rosbridge_server for the phone app        (default false)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def name_plate(pkg):
    """'true' when meshes/shoulder_name.stl exists (the named version of the robot)."""
    return 'true' if os.path.exists(os.path.join(pkg, 'meshes', 'shoulder_name.stl')) else 'false'


def generate_launch_description():
    pkg = get_package_share_directory('scara_description')
    xacro_file = os.path.join(pkg, 'urdf', 'scara.urdf.xacro')
    mode = LaunchConfiguration('mode')
    sim = LaunchConfiguration('sim')
    port = LaunchConfiguration('port')
    fake = LaunchConfiguration('fake')
    homing = LaunchConfiguration('homing')
    cell = LaunchConfiguration('cell')
    rviz = LaunchConfiguration('rviz')
    rosbridge = LaunchConfiguration('rosbridge')

    is_gz = PythonExpression(["'", sim, "' == 'gazebo'"])
    is_rviz = PythonExpression(["'", sim, "' == 'rviz'"])
    rviz_sliders = PythonExpression(["'", sim, "' == 'rviz' and '", mode, "' == 'sim_to_real'"])
    description = ParameterValue(Command(['xacro ', xacro_file, ' name_plate:=', name_plate(pkg)]), value_type=str)

    # scoped: gazebo.launch.py has its own "mode" (the cell mode) - it must not overwrite the
    # twin's "mode" (sim_to_real / real_to_sim) that the bridge below reads
    gazebo = GroupAction(scoped=True, condition=IfCondition(is_gz), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'mode': cell, 'show': LaunchConfiguration('show'),
                          'headless': LaunchConfiguration('headless')}.items())])

    # RViz-only simulation: robot_state_publisher (+ sliders when the sim is the master)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', condition=IfCondition(is_rviz),
               parameters=[{'robot_description': description}])
    sliders = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                   condition=IfCondition(rviz_sliders))

    # ghost of the REAL robot, built from /real/joint_states
    ghost = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='real',
                 parameters=[{'robot_description': description, 'frame_prefix': 'real/',
                              'use_sim_time': is_gz}])
    ghost_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['--frame-id', 'world', '--child-frame-id', 'real/world'],
                    parameters=[{'use_sim_time': is_gz}])

    bridge = Node(package='scara_description', executable='scara_twin_bridge.py', name='scara_twin_bridge',
                  output='screen',
                  parameters=[os.path.join(pkg, 'config', 'twin_hardware.yaml'),
                              {'port': port, 'fake': fake, 'mode': mode, 'homing': homing,
                               'target': sim, 'use_sim_time': is_gz}])

    scene = Node(package='scara_description', executable='scara_rviz_scene.py', name='scara_rviz_scene',
                 condition=IfCondition(rviz), parameters=[{'cell': cell, 'use_sim_time': is_gz}])

    rviz_node = Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
                     arguments=['-d', os.path.join(pkg, 'config', 'twin.rviz')],
                     parameters=[{'use_sim_time': is_gz}])

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('rosbridge_server'), 'launch',
                                                'rosbridge_websocket_launch.xml')),
        condition=IfCondition(rosbridge)) if _has_pkg('rosbridge_server') else None

    actions = [
        DeclareLaunchArgument('mode', default_value='sim_to_real'),
        DeclareLaunchArgument('sim', default_value='gazebo'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('fake', default_value='false'),
        DeclareLaunchArgument('homing', default_value='switches'),
        DeclareLaunchArgument('cell', default_value='manual'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rosbridge', default_value='false'),
        DeclareLaunchArgument('show', default_value='true', description='camera windows (Gazebo cell)'),
        DeclareLaunchArgument('headless', default_value='false', description='Gazebo without a window'),
        gazebo, rsp, sliders, ghost, ghost_tf,
        # in Gazebo wait for the controllers before the robot starts copying the sim
        TimerAction(period=PythonExpression(["8.0 if '", sim, "' == 'gazebo' else 0.5"]), actions=[bridge]),
        rviz_node, scene,
    ]
    if rosbridge_launch is not None:
        actions.append(rosbridge_launch)
    return LaunchDescription(actions)


def _has_pkg(name):
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False
