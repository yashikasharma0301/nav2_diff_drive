import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    slam_pkg = get_package_share_directory('slam_toolbox')
    this_pkg = get_package_share_directory('slam')

    mapping_params = os.path.join(this_pkg, 'params', 'mapper_params_online_async.yaml')

    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value='rmw_cyclonedds_cpp'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': mapping_params,
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        set_rmw,
        slam_toolbox_launch
    ])
