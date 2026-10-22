"""
SCARA cell in Gazebo Harmonic - three modes, like the Mechamind Labs videos.

    ros2 launch scara_description gazebo.launch.py                   # SORTING: endless conveyor, 4 bins,
                                                                     #   every colour into its own bin
    ros2 launch scara_description gazebo.launch.py mode:=command     # COMMAND: endless conveyor, ONE bin;
                                                                     #   the robot picks the colour you ask for
        python3 scripts/scara_ask.py            # 2nd terminal: type a colour   (red / any / stop / start)
        python3 scripts/scara_ask.py --voice    # or say "Hey SCARA ... the red one"
    ros2 launch scara_description gazebo.launch.py mode:=manual      # MANUAL: table + 3 blue / 3 green pucks,
                                                                     #   no conveyor - keyboard / joystick / gestures

Arguments
  mode       sort | command | manual                               (default sort)
             (old names still work: sorter:=false -> manual, wait:=true -> command)
  pucks      number of pucks to spawn, 0 = endless                 (default 0)
  empty_bins true|false take placed pucks out of the bin(s)          (default true)
             false: after `capacity` pucks a bin is "full" until you send "empty"
  capacity   pucks per bin with empty_bins:=false                  (default 4)
  show       true|false   OpenCV camera windows                     (default true)
  rviz       true|false   RViz with the robot and the cell          (default false)
  headless   true|false   no Gazebo window, server only (cameras still render) (default false)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def plugin_dirs():
    """Where gz_ros2_control's system plugin may live: the ROS install and the usual
    source workspace (~/gz_ros2_control_ws), so GZ_SIM_SYSTEM_PLUGIN_PATH need not be set by hand."""
    distro = os.environ.get('ROS_DISTRO', 'humble')
    cands = [f'/opt/ros/{distro}/lib',
             os.path.expanduser('~/gz_ros2_control_ws/install/gz_ros2_control/lib'),
             os.path.expanduser('~/ros2_ws/install/gz_ros2_control/lib')]
    return os.pathsep.join(c for c in cands if os.path.isdir(c)) or f'/opt/ros/{distro}/lib'


def name_plate(pkg):
    """'true' when meshes/shoulder_name.stl exists (the named version of the robot)."""
    return 'true' if os.path.exists(os.path.join(pkg, 'meshes', 'shoulder_name.stl')) else 'false'


def generate_launch_description():
    pkg = get_package_share_directory('scara_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    worlds = os.path.join(pkg, 'worlds')
    xacro_file = os.path.join(pkg, 'urdf', 'scara.urdf.xacro')
    install_share = os.path.dirname(pkg)   # so package://scara_description/... resolves

    pucks = LaunchConfiguration('pucks')
    show = LaunchConfiguration('show')
    # effective mode (the old arguments sorter:=false / wait:=true still work)
    mode = PythonExpression(["'manual' if '", LaunchConfiguration('sorter'), "' == 'false' else "
                             "('command' if '", LaunchConfiguration('wait'), "' == 'true' else '",
                             LaunchConfiguration('mode'), "')"])
    world = PythonExpression(["{'sort': 'scara_world', 'command': 'scara_command_world', "
                              "'manual': 'scara_manual_world'}.get('", mode, "', 'scara_world')"])
    cell = PythonExpression(["'", mode, "' != 'manual'"])
    empty_bins = LaunchConfiguration('empty_bins')
    capacity = LaunchConfiguration('capacity')

    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' name_plate:=', name_plate(pkg)]), value_type=str),
                         'use_sim_time': True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [PythonExpression(["'-r -s --headless-rendering ' if '", LaunchConfiguration('headless'),
                                                         "' == 'true' else '-r '"]), worlds, '/', world, '.sdf']}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen', parameters=[robot_description])

    spawn_robot = Node(package='ros_gz_sim', executable='create', output='screen',
                       arguments=['-world', 'scara_world', '-topic', 'robot_description', '-name', 'scara',
                                  '-x', '0', '-y', '0', '-z', '0'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/fixed_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/fixed_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/fixed_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/arm_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/arm_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/conveyor_belt/link/belt_link/track_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[('/model/conveyor_belt/link/belt_link/track_cmd_vel', '/conveyor/speed')])

    # puck / robot poses WITH names (the bridge drops the names of Pose_V entries)
    poses = Node(package='scara_description', executable='scara_pose_relay.py', output='screen',
                 parameters=[{'use_sim_time': True, 'world': 'scara_world'}])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager'])

    controllers = RegisterEventHandler(OnProcessExit(
        target_action=spawn_robot,
        on_exit=[spawner('joint_state_broadcaster'), spawner('arm_controller'), spawner('gripper_controller')]))

    vision = Node(package='scara_description', executable='scara_vision_node.py', output='screen',
                  condition=IfCondition(cell), parameters=[{'use_sim_time': True, 'show': show}])
    color_sorter = Node(package='scara_description', executable='scara_color_sorter.py', output='screen',
                        condition=IfCondition(cell),
                        parameters=[{'use_sim_time': True, 'mode': mode,
                                     'bin_capacity': PythonExpression(["0 if '", empty_bins, "' == 'true' else int('", capacity, "')"])}])
    puck_spawner = Node(package='scara_description', executable='scara_puck_spawner.py', output='screen',
                        condition=IfCondition(cell),
                        parameters=[{'use_sim_time': True, 'count': pucks, 'world': 'scara_world', 'mode': mode,
                                     'clear_bins': PythonExpression(["'", empty_bins, "' == 'true'"])}])

    rviz = LaunchConfiguration('rviz')
    scene = Node(package='scara_description', executable='scara_rviz_scene.py', condition=IfCondition(rviz),
                 parameters=[{'use_sim_time': True, 'cell': mode}])
    rviz_node = Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
                     arguments=['-d', os.path.join(pkg, 'config', 'scara.rviz')], parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sort', description='sort | command | manual'),
        DeclareLaunchArgument('sorter', default_value='true', description='(old) false = manual mode'),
        DeclareLaunchArgument('wait', default_value='false', description='(old) true = command mode'),
        DeclareLaunchArgument('pucks', default_value='0'),
        DeclareLaunchArgument('empty_bins', default_value='true'),
        DeclareLaunchArgument('capacity', default_value='4'),
        DeclareLaunchArgument('show', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false', description='true = no Gazebo window'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', install_share),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', install_share),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_dirs()),
        AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_dirs()),
        gazebo,
        rsp,
        spawn_robot,
        bridge,
        poses,
        controllers,
        scene, rviz_node,
        # give the controllers a few seconds before the robot starts working
        TimerAction(period=6.0, actions=[vision, color_sorter, puck_spawner]),
    ])
