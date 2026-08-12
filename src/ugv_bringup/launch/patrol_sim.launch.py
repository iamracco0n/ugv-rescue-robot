import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """경비 순찰 + 화재 감지 통합 시뮬 런치.

    slam_nav_sim.launch.py 와 동일한 기반(Gazebo+SLAM+Nav2+YOLO+포탑)에
    fog 커버리지 대신 fire_detection_node + patrol_navigator 를 얹는다.
    Gazebo/브리지는 gazebo.launch.py 를 그대로 재사용(열화상 브리지 포함).
    """

    # 누운 사람 관문 완화 스위치(측정용).
    # 기본값은 본 코드 기본값과 같고, 완화 이전 동작을 재려면
    #   UGV_LYING_KPTS=6 UGV_LYING_CONF=0.50
    # 으로 돌리면 된다. 다시 빌드하지 않으므로 두 조건이 같은 바이너리다.
    LYING_KPTS = int(os.environ.get('UGV_LYING_KPTS', '6'))
    LYING_CONF = float(os.environ.get('UGV_LYING_CONF', '0.50'))

    patrol_arg = DeclareLaunchArgument(
        'patrol_enabled_on_boot',
        default_value='true',
        description='스폰 후 즉시 순찰 시작 여부'
    )

    # 구조본부가 알려준 실종자 수. 이 수를 다 찾기 전에는 수색을 끝내지 않고
    # 시야 기록을 지워 재수색한다. 0이면 모름(면적 기준으로만 완료 판정).
    #   rescue_building        조난자 3명
    #   rescue_building_large  조난자 7명
    victims_arg = DeclareLaunchArgument(
        'expected_victims', default_value='0',
        description='실종자 수 (0=모름). 예: world:=rescue_building_large 면 7')

    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_navigation = get_package_share_directory('ugv_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # 월드 선택을 gazebo.launch.py 로 그대로 넘긴다
    #   ros2 launch ugv_bringup patrol_sim.launch.py world:=rescue_building_large
    world_arg = DeclareLaunchArgument(
        'world', default_value='rescue_building',
        description='worlds/ 아래 SDF 이름 (rescue_building | rescue_building_large)')

    # 화면 없이 돌릴지. 자동 채점(tools/run_eval.sh)에서 쓴다.
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true 면 gz GUI·RViz 없이 서버만 실행')

    # 탐사 성향 조절 — 파라미터 스윕으로 최적값을 찾기 위해 런치에서 연다.
    # 노드는 초기화 때 한 번만 읽으므로 런타임 param set 으로는 못 바꾼다.
    budget_arg = DeclareLaunchArgument(
        'goal_dist_penalty', default_value='0.5',
        description='목표 점수에서 거리 1m 에 매기는 벌점(m^2). '
                    '크면 가까운 곳만 맴돌고, 작으면 멀리 나간다')
    radius_arg = DeclareLaunchArgument(
        'frontier_view_r', default_value='8.0',
        description='라이다 경계를 넘었을 때 새로 보이는 깊이(m). '
                    '경계 길이를 넓이로 환산할 때 쓴다')

    # 1. Gazebo + Robot + 브리지 + RViz (열화상 브리지 포함)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items()
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

    # 4. 비전(YOLO 환자 감지 + 포탑 제어) (48초 후)
    #    Nav2 lifecycle 활성화가 끝난 뒤에 띄운다. 예전엔 26초(=Nav2 기동 4초 뒤)라
    #    torch/CUDA 로딩 CPU 스파이크가 lifecycle 전환 도중에 겹쳐서
    #    controller_server change_state 타임아웃 → 스택이 unconfigured 로 죽었다.
    vision_launch = TimerAction(
        period=48.0,
        actions=[
            Node(package='ugv_vision', executable='yolo_pose_node',
                 name='yolo_pose_node',
                 parameters=[{'use_sim_time': True,
                              # 누운 사람 관문 완화를 껐다 켰다 하며 재기 위한 것.
                              # 조건마다 다시 빌드하면 빌드 차이가 섞이므로
                              # 파라미터로 가른다. 6/0.50 이면 완화 이전과 같다.
                              'lying_min_kpts': LYING_KPTS,
                              'lying_kpt_conf': LYING_CONF}],
                 output='screen'),
            Node(package='ugv_vision', executable='target_manager_node',
                 name='target_manager_node',
                 parameters=[{'use_sim_time': True}], output='screen'),
        ]
    )

    # 5. 화재 감지 + 순찰 (58초 후 — Nav2 활성화 + 비전 로딩 후)
    patrol_launch = TimerAction(
        period=58.0,
        actions=[
            Node(package='ugv_vision', executable='fire_detection_node',
                 name='fire_detection_node',
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='ugv_vision', executable='patrol_navigator',
                 name='patrol_navigator',
                 parameters=[{
                     'use_sim_time': True,
                     'patrol_enabled_on_boot': LaunchConfiguration('patrol_enabled_on_boot'),
                     'expected_victims': LaunchConfiguration('expected_victims'),
                     # 실수형으로 강제한다. 런치 인자는 문자열이라 '90' 을
                     # 넘기면 rclpy 가 INTEGER 로 추론해 DOUBLE 파라미터와
                     # 타입이 안 맞고, 노드가 기동 즉시 죽는다
                     # (InvalidParameterTypeException).
                     'goal_dist_penalty': ParameterValue(
                         LaunchConfiguration('goal_dist_penalty'),
                         value_type=float),
                     'frontier_view_r': ParameterValue(
                         LaunchConfiguration('frontier_view_r'),
                         value_type=float),
                 }], output='screen'),
        ]
    )

    return LaunchDescription([
        patrol_arg,
        victims_arg,
        world_arg,
        headless_arg,
        budget_arg,
        radius_arg,
        gazebo_launch,
        slam_launch,
        nav2_launch,
        vision_launch,
        patrol_launch,
    ])
