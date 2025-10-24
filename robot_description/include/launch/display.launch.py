import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    this_pkg = 'robot_description'
    world_file_name = 'my_world.sdf'
    model_file_name = 'tortoisebot.xacro'

    world_path = os.path.join(
        get_package_share_directory(this_pkg),
        'worlds',
        world_file_name)

    xacro_path = os.path.join(
        get_package_share_directory(this_pkg),
        'urdf',
        model_file_name)

    robot_description_raw = xacro.process_file(xacro_path).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '--verbose', '-r'],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tortoisebot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.335'
        ],
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory('robot_description'),
        'params',
        'bridge_params.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=['depth_camera/depth_image'],
    output='screen',
    )

    return LaunchDescription([
        start_gazebo_ros_bridge_cmd,
        gazebo,
        spawn_entity,
        node_joint_state_publisher,
        node_robot_state_publisher,
        start_gazebo_ros_image_bridge_cmd
    ])
