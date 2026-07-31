import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # coverage_enabled_on_boot: false = 스폰 후 대기, true = 즉시 커버리지 시작
    coverage_arg = DeclareLaunchArgument(
        'coverage_enabled_on_boot',
        default_value='false',
        description='스폰 후 즉시 coverage 시작 여부 (기본 OFF — 수동 goal 우선)'
    )

    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_navigation = get_package_share_directory('ugv_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # 1. Gazebo & Robot Description 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. SLAM Toolbox 실행 (4초 후)
    slam_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # 3. Nav2 Navigation 실행 (8초 후)
    nav2_params_file = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    
    nav2_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_params_file,
                    'slam': 'True'
                }.items()
            )
        ]
    )

    # 4. 비전 시스템(YOLO & Target Manager) 실행 (10초 후)
    vision_launch = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ugv_vision',
                executable='yolo_pose_node',
                name='yolo_pose_node',
                parameters=[{'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='ugv_vision',
                executable='target_manager_node',
                name='target_manager_node',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # 5. 시야 커버리지 탐색 네비게이터 (12초 후 — Nav2 활성화 후 시작)
    coverage_launch = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ugv_vision',
                executable='vision_coverage_navigator',
                name='vision_coverage_navigator',
                parameters=[{
                    'use_sim_time': True,
                    'coverage_enabled_on_boot': LaunchConfiguration(
                        'coverage_enabled_on_boot'),
                }],
                output='screen'
            )
        ]
    )

    # 6. RViz Visibility Overlay (12초 후 — 커버리지와 동시)
    overlay_launch = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ugv_vision',
                executable='visibility_overlay_node',
                name='visibility_overlay_node',
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        coverage_arg,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        vision_launch,
        coverage_launch,
        overlay_launch,
    ])
