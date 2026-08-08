import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """경비 순찰 + 화재 감지 통합 시뮬 런치.

    slam_nav_sim.launch.py 와 동일한 기반(Gazebo+SLAM+Nav2+YOLO+포탑)에
    fog 커버리지 대신 fire_detection_node + patrol_navigator 를 얹는다.
    Gazebo/브리지는 gazebo.launch.py 를 그대로 재사용(열화상 브리지 포함).
    """

    patrol_arg = DeclareLaunchArgument(
        'patrol_enabled_on_boot',
        default_value='true',
        description='스폰 후 즉시 순찰 시작 여부'
    )

    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_navigation = get_package_share_directory('ugv_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 월드 선택을 gazebo.launch.py 로 그대로 넘긴다
    #   ros2 launch ugv_bringup patrol_sim.launch.py world:=rescue_building_large
    world_arg = DeclareLaunchArgument(
        'world', default_value='rescue_building',
        description='worlds/ 아래 SDF 이름 (rescue_building | rescue_building_large)')

    # 1. Gazebo + Robot + 브리지 + RViz (열화상 브리지 포함)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 2. SLAM Toolbox (14초 후 — 로봇 스폰 8초 이후)
    slam_launch = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # 3. Nav2 (22초 후 — 무거운 월드에서 lifecycle 타임아웃 방지) — fire_cloud 장애물 소스 포함된 nav2_params.yaml
    nav2_params_file = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')
    nav2_launch = TimerAction(
        period=22.0,
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

    # 4. 비전(YOLO 환자 감지 + 포탑 제어) (26초 후)
    vision_launch = TimerAction(
        period=26.0,
        actions=[
            Node(package='ugv_vision', executable='yolo_pose_node',
                 name='yolo_pose_node',
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='ugv_vision', executable='target_manager_node',
                 name='target_manager_node',
                 parameters=[{'use_sim_time': True}], output='screen'),
        ]
    )

    # 5. 화재 감지 + 순찰 (30초 후 — Nav2 활성화 후)
    patrol_launch = TimerAction(
        period=30.0,
        actions=[
            Node(package='ugv_vision', executable='fire_detection_node',
                 name='fire_detection_node',
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='ugv_vision', executable='patrol_navigator',
                 name='patrol_navigator',
                 parameters=[{
                     'use_sim_time': True,
                     'patrol_enabled_on_boot': LaunchConfiguration('patrol_enabled_on_boot'),
                 }], output='screen'),
        ]
    )

    return LaunchDescription([
        patrol_arg,
        world_arg,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        vision_launch,
        patrol_launch,
    ])
