import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (LaunchConfiguration,
                                  PathJoinSubstitution, TextSubstitution)

def generate_launch_description():

    pkg_desc = get_package_share_directory('ugv_description')
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_config_file = os.path.join(pkg_desc, 'rviz', 'ugv.rviz')

    # 월드 선택. SDF 파일명과 world 이름이 같아야 한다(스폰 시 -world 로 씀).
    #   rescue_building       — 28x20m, 방 4개. 빠른 회귀 검증용(수색 약 9분)
    #   rescue_building_large — 56x40m, 방 10개. 본 검증용(수색 약 35분)
    world_arg = DeclareLaunchArgument(
        'world', default_value='rescue_building',
        description='worlds/ 아래 SDF 이름 (확장자 제외)')
    world = LaunchConfiguration('world')
    world_file = PathJoinSubstitution(
        [pkg_bringup, 'worlds', [world, TextSubstitution(text='.sdf')]])

    # 헤드리스: gz GUI 와 RViz 를 끈다. 자동 채점(tools/run_eval.sh)처럼
    # 화면이 필요 없는 검증에서 CPU 를 크게 아낀다 — GUI 렌더가 서버와
    # 경합하면 Nav2 lifecycle 전환이 타임아웃나기도 한다.
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true 면 gz GUI·RViz 없이 서버만 실행')
    headless = LaunchConfiguration('headless')
    gui_only = UnlessCondition(headless)

    return LaunchDescription([

        world_arg,
        headless_arg,

        # 1. 가제보 실행 (헤드리스면 -s 로 서버만)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r ', world_file]}.items(),
            condition=UnlessCondition(headless),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -s ', world_file]}.items(),
            condition=IfCondition(headless),
        ),

        # 2~4. 로봇 한 대분(상태 퍼블리셔 + 스폰 + 브리지).
        # 여러 대를 띄울 때는 이 include 를 이름만 바꿔 반복한다
        # (multi_robot_sim.launch.py 참조). prefix 를 비워 두면 토픽·프레임
        # 이름이 지금까지와 완전히 같다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'robot.launch.py')
            ),
            launch_arguments={
                'name': 'ugv', 'prefix': '', 'world': world,
                'x': '0.0', 'y': '0.0', 'delay': '8.0',
            }.items(),
        ),

        # 5. 알비즈 실행 (헤드리스면 생략)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=gui_only,
        )
    ])
