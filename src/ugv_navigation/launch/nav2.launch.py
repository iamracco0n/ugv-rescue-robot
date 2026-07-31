from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_nav = get_package_share_directory('ugv_navigation')
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': '', 
                'use_sim_time': 'True',
                'params_file': params_file,
                'autostart': 'True'
            }.items(),
        )
    ])
