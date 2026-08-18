import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('scara_robot')
    pkg_share_parent = os.path.dirname(pkg_share)

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[pkg_share_parent, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_share_parent, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    world_path = os.path.join(pkg_share, 'worlds', 'scara.world.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'scara.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_path} -r'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'scara_robot',
            '-string', robot_description_config,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    scara_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        set_ign_resource_path,
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        joint_state_broadcaster_spawner,
        scara_controller_spawner,
    ])
