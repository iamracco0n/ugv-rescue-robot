"""로봇 2대 동시 수색 — 1단계: 각자 스폰·SLAM·Nav2 (아직 협조는 없다).

    ros2 launch ugv_bringup multi_robot_sim.launch.py \
         world:=rescue_building_large headless:=true

1단계에서 확인할 것은 하나다 — **두 대가 서로를 덮어쓰지 않고 각자 움직이는가.**
지도 병합·표적 공유·목표 선점은 2~3단계에서 붙인다.

이름이 갈리는 방식
------------------
  gz 모델      ugv1, ugv2
  TF 프레임    ugv1/base_footprint, ugv2/base_footprint ...
  토픽         /ugv1/scan, /ugv2/scan ...
  노드 이름공간 /ugv1, /ugv2

TF 사슬은 로봇마다 따로 선다.

    ugv1/map ──(slam_toolbox)──> ugv1/odom ──> ugv1/base_footprint
    ugv2/map ──(slam_toolbox)──> ugv2/odom ──> ugv2/base_footprint

두 map 프레임을 하나로 묶는 일(공통 map → ugvN/map 정적 변환 + 격자 병합)은
2단계 몫이다. 1단계에서는 각자 자기 map 프레임 안에서 움직인다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  TextSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def namespaced_nav2_params(src, prefix, out_path):
    """Nav2 설정의 프레임·토픽 이름에 로봇 접두사를 붙여 새 파일로 낸다.

    키 이름으로 일괄 치환하면 안 된다. `global_frame` 은 전역 코스트맵에서는
    'map', 지역 코스트맵에서는 'odom' 으로 **값이 다르다.** 키로 밀어버리면
    지역 코스트맵이 map 을 바라보게 되어 제어가 깨진다.
    그래서 값을 보고 바꾼다.
    """
    subs = {
        'map': f'{prefix}map',
        'odom': f'{prefix}odom',
        'base_footprint': f'{prefix}base_footprint',
        '/scan': f'/{prefix}scan',
        '/odom': f'/{prefix}odom',
        '/camera/points': f'/{prefix}camera/points',
    }

    def walk(node):
        if isinstance(node, dict):
            return {k: walk(v) for k, v in node.items()}
        if isinstance(node, list):
            return [walk(v) for v in node]
        if isinstance(node, str):
            return subs.get(node, node)
        return node

    with open(src, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(walk(data), f, allow_unicode=True)
    return out_path

# 스폰 위치. 같은 문에서 들어가되 서로 안 겹치게 벌려 둔다.
# 건물 양끝에서 시작하면 결과는 좋아지지만 현실성이 떨어진다.
ROBOTS = [
    {'name': 'ugv1', 'x': '0.0', 'y': '0.8'},
    {'name': 'ugv2', 'x': '0.0', 'y': '-0.8'},
]


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_desc = get_package_share_directory('ugv_description')
    pkg_nav = get_package_share_directory('ugv_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    world = LaunchConfiguration('world')
    world_file = PathJoinSubstitution(
        [pkg_bringup, 'worlds', [world, TextSubstitution(text='.sdf')]])
    headless = LaunchConfiguration('headless')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_slam, 'config',
                               'mapper_params_online_async.yaml')

    actions = [
        DeclareLaunchArgument('world', default_value='rescue_building_large'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('expected_victims', default_value='7'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r ', world_file]}.items(),
            condition=UnlessCondition(headless)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s ', world_file]}.items(),
            condition=IfCondition(headless)),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg_desc, 'rviz', 'ugv.rviz')],
             parameters=[{'use_sim_time': True}], output='screen',
             condition=UnlessCondition(headless)),
    ]

    for i, r in enumerate(ROBOTS):
        name = r['name']
        prefix = f'{name}/'

        # 로봇 본체. /clock 은 월드에 하나뿐이어야 하므로 첫 대만 브리지한다.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'robot.launch.py')),
            launch_arguments={
                'name': name, 'prefix': prefix, 'world': world,
                'x': r['x'], 'y': r['y'], 'delay': str(8.0 + i * 2.0),
                'bridge_clock': 'true' if i == 0 else 'false',
            }.items()))

        # SLAM — 로봇마다 자기 지도를 만든다. 프레임 이름이 안 갈리면
        # 두 SLAM 이 같은 map→odom 을 발행해 TF 트리가 깨진다.
        actions.append(TimerAction(period=14.0 + i * 2.0, actions=[
            Node(package='slam_toolbox',
                 executable='async_slam_toolbox_node',
                 name='slam_toolbox', namespace=name,
                 parameters=[slam_params, {
                     'use_sim_time': True,
                     'odom_frame': f'{prefix}odom',
                     'base_frame': f'{prefix}base_footprint',
                     'map_frame': f'{prefix}map',
                     'scan_topic': f'/{prefix}scan',
                 }],
                 # tf 도 같은 이유로 상대 이름으로 되돌린다(robot.launch.py 주석 참조)
                 remappings=[('/map', f'/{name}/map'),
                             ('/map_metadata', f'/{name}/map_metadata'),
                             ('/tf', 'tf'), ('/tf_static', 'tf_static')],
                 output='screen')]))

        # Nav2 — 프레임 이름을 로봇별로 바꿔 끼운다. 안 바꾸면 두 스택이
        # 같은 base_footprint 를 찾아 서로의 로봇을 제어한다.
        #
        # ★ PushRosNamespace 로 감싸야 한다. Humble 의 navigation_launch.py 는
        #   namespace 인자를 '파라미터 파일의 최상위 키' 로만 쓰고 노드에는
        #   씌우지 않는다(GroupAction 안에 PushRosNamespace 가 없다).
        #   그냥 include 하면 노드가 /controller_server 로 떠서 ugv1: 아래
        #   파라미터를 못 읽고 'No critics defined for FollowPath' 로 죽는다.
        actions.append(TimerAction(period=22.0 + i * 4.0, actions=[
            GroupAction([
                PushRosNamespace(name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav2, 'launch',
                                     'navigation_launch.py')),
                    launch_arguments={
                        'namespace': name,
                        'use_sim_time': 'true',
                        'params_file': namespaced_nav2_params(
                            nav2_params, prefix,
                            os.path.join('/tmp', f'nav2_{name}.yaml')),
                        'slam': 'True',
                    }.items()),
            ])]))

    # ── 2단계: 공용 map 프레임과 지도 병합 ───────────────────────────
    # 각 로봇의 map 프레임 원점은 그 로봇이 출발한 자리다(slam_toolbox 규약).
    # 스폰 위치를 우리가 정하므로 오프셋을 이미 안다 → 정렬 탐색이 필요 없다.
    # 첫 로봇의 map 을 공용 map 으로 삼고 나머지를 그만큼 밀어 붙인다.
    #
    # Nav2 는 건드리지 않는다. 각자 자기 map 프레임에서 계속 계획하고,
    # 공용 지도는 '어디를 탐사할지' 를 정하는 순찰 노드만 쓴다. 목표는
    # map 프레임으로 나가고 Nav2 가 자기 프레임으로 변환해 받는다.
    # 이렇게 두면 1단계에서 검증된 Nav2 구성을 흔들지 않는다.
    x0, y0 = float(ROBOTS[0]['x']), float(ROBOTS[0]['y'])
    for r in ROBOTS:
        dx, dy = float(r['x']) - x0, float(r['y']) - y0
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f"map_to_{r['name']}",
            arguments=['--x', str(dx), '--y', str(dy),
                       '--frame-id', 'map',
                       '--child-frame-id', f"{r['name']}/map"],
            parameters=[{'use_sim_time': True}], output='screen'))

    actions.append(TimerAction(period=20.0, actions=[
        Node(package='ugv_vision', executable='map_merge_node',
             name='map_merge_node',
             parameters=[{
                 'use_sim_time': True,
                 'robots': [r['name'] for r in ROBOTS],
                 'spawn_x': [float(r['x']) for r in ROBOTS],
                 'spawn_y': [float(r['y']) for r in ROBOTS],
             }], output='screen')]))

    # 비전·순찰 — Nav2 lifecycle 활성화가 끝난 뒤에 띄운다. 먼저 띄우면
    # torch/CUDA 로딩 CPU 스파이크가 lifecycle 전환과 겹쳐 스택이 죽는다.
    for i, r in enumerate(ROBOTS):
        name = r['name']
        prefix = f'{name}/'
        # TF 리스너는 절대 '/tf' 를 구독한다(tf2_ros 구현). 네임스페이스를
        # 씌워도 안 따라오므로 상대 이름으로 되돌려야 /ugv1/tf 를 본다.
        tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        common = [{'use_sim_time': True,
                   'map_frame': f'{prefix}map',
                   'base_frame': f'{prefix}base_footprint'}]
        actions.append(TimerAction(period=52.0 + i * 4.0, actions=[
            Node(package='ugv_vision', executable='yolo_pose_node',
                 name='yolo_pose_node', namespace=name,
                 remappings=tf_remap,
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='ugv_vision', executable='target_manager_node',
                 name='target_manager_node', namespace=name,
                 remappings=tf_remap, parameters=common, output='screen'),
            Node(package='ugv_vision', executable='fire_detection_node',
                 name='fire_detection_node', namespace=name,
                 remappings=tf_remap, parameters=common, output='screen'),
            # 순찰 노드만 공용 지도를 본다. 상대가 이미 만든 지도를 알아야
            # 같은 곳을 다시 훑지 않는다. 목표는 map 프레임으로 나가고
            # Nav2 가 자기 프레임으로 변환해 받는다.
            Node(package='ugv_vision', executable='patrol_navigator',
                 name='patrol_navigator', namespace=name,
                 remappings=tf_remap + [('map', '/map')],
                 parameters=[{'use_sim_time': True,
                              'map_frame': 'map',
                              'base_frame': f'{prefix}base_footprint'}] + [{
                     'patrol_mode': 'explore',
                     # 런치 인자는 문자열이라 그대로 주면 rclpy 가 STRING 으로
                     # 추론해 INTEGER 파라미터와 안 맞고 노드가 즉시 죽는다.
                     'expected_victims': ParameterValue(
                         LaunchConfiguration('expected_victims'),
                         value_type=int),
                 }], output='screen'),
        ]))

    return LaunchDescription(actions)
