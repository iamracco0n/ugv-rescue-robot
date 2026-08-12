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
                os.path.join(pkg_bringup, 'launch', 'sim_robot.launch.py')
            ),
            launch_arguments={
                'name': 'ugv', 'prefix': '', 'world': world,
                # 스폰 위치를 열어 둔다. 특정 대상 앞에서 시작시켜 '거기까지
                # 갔느냐' 를 빼고 재기 위한 것이다.
                #
                # 누운 조난자 검출을 재려 했더니 결과가 런마다 뒤집혔다.
                # 원인은 지표가 '로봇이 900초 안에 그 방에 갔나' 에 좌우된
                # 것이었다 — 가면 관문과 무관하게 찾고, 안 가면 무조건 못
                # 찾는다. 관문 효과가 동선 운에 파묻혔다.
                # 대상 근처에서 시작하면 그 운이 사라져 훨씬 적은 런으로
                # 답이 나온다.
                'x': os.environ.get('UGV_SPAWN_X', '0.0'),
                'y': os.environ.get('UGV_SPAWN_Y', '0.0'),
                'delay': '8.0',
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
