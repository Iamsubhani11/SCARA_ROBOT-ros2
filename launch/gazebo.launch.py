import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_scara_description = get_package_share_directory('scara_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths to Xacro / SDF files
    scara_xacro_file = os.path.join(pkg_scara_description, 'urdf', 'scara.urdf.xacro')
    conveyor_xacro_file = os.path.join(pkg_scara_description, 'urdf', 'new_conveyor.xacro')
    yellow_puck_sdf_file = os.path.join(pkg_scara_description, 'urdf', 'yellow_puck.sdf')

    # Process Xacro contents
    scara_description_config = Command(['xacro ', scara_xacro_file])
    scara_robot_description = {'robot_description': ParameterValue(scara_description_config, value_type=str)}

    conveyor_description_config = Command(['xacro ', conveyor_xacro_file])

    # Parent directory of package share so package://scara_description/meshes/... resolves
    install_dir = os.path.dirname(pkg_scara_description)

    # Set Ignition / Gazebo environment variables
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_dir
    )
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_dir
    )
    set_ign_plugin_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )
    set_gz_plugin_path = AppendEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # Include Gazebo Sim launch file
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[scara_robot_description]
    )

    # Spawn SCARA Robot on Top of Wooden Table Base Platform
    spawn_scara_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'scara',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Spawn New Conveyor Belt Track on Wooden Table Surface
    spawn_conveyor_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', conveyor_description_config,
            '-name', 'new_conveyor',
            '-x', '0.35',
            '-y', '-0.18',
            '-z', '0.0'
        ],
        output='screen'
    )

    # Spawn Yellow Puck Payload Object on Conveyor Track
    spawn_yellow_puck_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', yellow_puck_sdf_file,
            '-name', 'yellow_puck',
            '-x', '0.45',
            '-y', '-0.18',
            '-z', '0.035'
        ],
        output='screen'
    )

    # ROS-GZ Bridge for /clock
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Controller Spawners (Sequential)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        set_ign_resource_path,
        set_gz_resource_path,
        set_ign_plugin_path,
        set_gz_plugin_path,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_scara_node,
        spawn_conveyor_node,
        spawn_yellow_puck_node,
        clock_bridge_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_scara_node,
                on_exit=[joint_state_broadcaster_spawner, arm_controller_spawner, gripper_controller_spawner],
            )
        ),
    ])
