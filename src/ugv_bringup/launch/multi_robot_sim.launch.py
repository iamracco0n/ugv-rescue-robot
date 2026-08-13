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
    # 2대를 한 머신에서 돌리면 코스트맵 갱신이 CPU 를 가장 많이 먹는다.
    # 갱신·발행 빈도를 낮춰 실시간(RTF 1.0)에 가깝게 만든다. 로봇이
    # 0.15m/s 로 느리게 다니므로 5Hz -> 2Hz 로 낮춰도 회피에 지장이 없다.
    rate = float(os.environ.get('UGV_COSTMAP_HZ', '0'))

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

    if rate > 0:
        def slow(node):
            if isinstance(node, dict):
                for k, v in node.items():
                    if k == 'update_frequency':
                        node[k] = rate
                    elif k == 'publish_frequency':
                        node[k] = min(rate, 1.0)
                    else:
                        slow(v)
            elif isinstance(node, list):
                for v in node:
                    slow(v)
        slow(data)

    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(walk(data), f, allow_unicode=True)
    return out_path

# 스폰 위치. 같은 복도로 들어가되 앞뒤로 충분히 떼어 놓는다.
#
# ★ 나란히(y=±0.8, 간격 1.6m) 세웠다가 둘 다 못 움직였다. 로봇 폭 0.4m 에
#   코스트맵 인플레이션이 0.55m 라, 1.6m 간격이면 서로를 장애물로 보고
#   갇힌다. 깊이 카메라를 장애물 입력에 넣은 뒤로는 상대를 더 크게 칠해
#   더 심해졌다. 실측: 3900초 동안 스폰 자리를 못 벗어나고 목표 245회 중
#   243회 도달 실패.
#   복도를 따라 앞뒤로 떼면 서로의 진로를 막지 않는다.
# 구역 분할과 순서를 맞춘다 — 첫 로봇이 앞쪽 구역(서/남)을 맡으므로
# 스폰도 그쪽에 둔다. 어긋나면 출발하자마자 건물을 가로질러야 한다.
ROBOTS = [
    {'name': 'ugv1', 'x': '-6.0', 'y': '0.0'},   # 서쪽 구역
    {'name': 'ugv2', 'x': '6.0',  'y': '0.0'},   # 동쪽 구역
]


def generate_launch_description():
    # 런치 인자는 문자열이므로 여기서 바로 못 읽는다. 환경변수로 받는다.
    n = int(os.environ.get('UGV_N_ROBOTS', '2'))
    # 탐사 범위 [xmin,ymin,xmax,ymax]. 월드 외벽 안쪽으로 잡는다.
    #   큰 월드 56x40 -> +-27, +-19
    #   미니맵  18x12 -> +-8,  +-5
    # 네 값이 다 0 이면 제한 없음.
    BOUNDS = [float(v) for v in
              os.environ.get('UGV_BOUNDS', '-27,-19,27,19').split(',')]
    # 로봇마다 구역을 갈라 준다.
    #
    # 상대 목표 반경을 피하는 방식(3단계 선점)은 반응형이라, 가까운 후보가
    # 다 잘리면 로봇이 멀리 있는 차선책으로 밀려난다. 그게 하필 상대 구역
    # 이면 건물을 가로지른다(실측: ugv1 이 북쪽 y=+7 로 가다 남쪽 y=-15 로
    # 21.9m 를 건너갔다). 2대를 쓰는 값어치가 사라진다.
    #
    # 아예 갈라 배정하면 그럴 일이 없다. 긴 축을 반으로 나눠 스폰 위치에
    # 가까운 쪽을 맡긴다. UGV_SPLIT=0 이면 안 나눈다(예전 동작).
    x0, y0, x1, y1 = BOUNDS
    split = os.environ.get('UGV_SPLIT', '1') != '0'

    # 분할 축. 기본은 긴 축을 반으로 가르는 것이고, UGV_SPLIT_AXIS 로
    # 고정할 수 있다(x/y). 건물 복도가 분할선을 가로지르면 로봇이 자기
    # 구역에 가려고 상대 구역을 통과해야 하므로, 월드에 따라 축을 바꿔
    # 재볼 수 있어야 한다.
    axis = os.environ.get('UGV_SPLIT_AXIS', 'auto')

    def bounds_for(idx, total):
        if not split or total < 2:
            return BOUNDS
        wide = (x1 - x0) >= (y1 - y0) if axis == 'auto' else (axis == 'x')
        if wide:                            # 좌우로 가른다
            w = (x1 - x0) / total
            return [x0 + idx * w, y0, x0 + (idx + 1) * w, y1]
        h = (y1 - y0) / total               # 위아래로 가른다
        return [x0, y0 + idx * h, x1, y0 + (idx + 1) * h]
    # 수색 완료를 인정하기 위한 최소 매핑 면적(m^2). 지도가 거의 없는 상태로
    # '다 훑었다' 고 하는 걸 막는 안전판인데, 월드 크기에 비례해야 한다.
    # 큰 월드는 자유공간이 2098m^2, 미니맵은 200 미만이라 상수 200 이면
    # 미니맵에서는 조건이 영원히 안 선다(BOUNDS 와 같은 종류의 문제).
    MIN_AREA = float(os.environ.get('UGV_MIN_AREA', '200.0'))
    # 로봇 스폰까지 기다릴 초. 월드가 다 뜨기 전에 요청하면 스폰 서비스가
    # 없어 타임아웃난다 — 로봇이 아예 안 생기고 그 뒤 TF·Nav2 가 줄줄이
    # 실패한다(오류 2000줄 넘게).
    #
    #   [create] Request to create entity from service
    #            [/world/<name>/create] timed out
    #
    # 느린 머신에서 실제로 났다(오로라 i7 10세대, 3런 중 2런 실패). 첫 런에
    # 몰리는 이유는 빌드 직후라 디스크 캐시가 비어 로딩이 더 느려서다.
    # 머신에 맞춰 늘릴 수 있게 뺀다. SLAM·Nav2 기동도 같이 밀린다.
    SPAWN = float(os.environ.get('UGV_SPAWN_DELAY', '8.0'))
    # 개선 항목 스위치 — 하나씩 켜 가며 재기 위한 것(ablation).
    # 큰 월드 2대에서 조건당 13~14런을 재 봤고 셋 다 base 와 구분되지 않았다.
    # 기본값을 끔으로 되돌린다. 스위치는 남겨 둔다 — 되켜서 재보려면 필요하다.
    SEEN_DIRS = int(os.environ.get('UGV_SEEN_DIRS', '1'))
    GHOST_NEED = int(os.environ.get('UGV_GHOST_NEED', '0'))
    # 지금 있는 방 안의 후보에 얹는 점수(m^2). 0 이면 꺼짐(=지금 동작).
    # 방을 덜 보고 나가는 것을 줄이려는 것인데, 세게 걸면 반대로 방에서
    # 못 나오는 고장이 난다. 0 부터 올려 가며 재기 위해 열어 둔다.
    ROOM_BONUS = float(os.environ.get('UGV_ROOM_BONUS', '0.0'))
    # 누운 사람 관문. 기본값은 완화 없음(=서있는 사람과 같은 기준).
    LYING_KPTS = int(os.environ.get('UGV_LYING_KPTS', '6'))
    LYING_CONF = float(os.environ.get('UGV_LYING_CONF', '0.50'))
    # YOLO 박스 신뢰도 하한. 검출은 두 단계인데 이게 1단계다.
    #   1단계  YOLO 가 사람 박스를 만든다        <- det_conf
    #   2단계  그 박스의 관절점을 검사한다        <- lying_* 관문
    # 1단계에서 버려진 것은 2단계에 오지 않는다. 실측으로 누운 조난자를
    # 놓친 런은 거의 전부 '후보로 잡은 적도 없음' 이었고 '후보는 잡았는데
    # 관문에서 떨어짐' 은 0건이었다 — 2단계를 아무리 풀어도 소용없던 이유다.
    DET_CONF = float(os.environ.get('UGV_DET_CONF', '0.50'))
    # 탐색 중 포탑 기본 각도(rad). 양수가 아래. 0 이면 지금까지의 수평.
    # 카메라 높이 0.5m·수직 FOV 48.9도라 수평이면 1.1m 안쪽 바닥이 안 찍힌다.
    SEARCH_PITCH = float(os.environ.get('UGV_SEARCH_PITCH', '0.0'))
    # 이 거리보다 가까운 바닥은 '봤음' 으로 치지 않는다. 0.3 이 기존 동작.
    # 카메라가 물리적으로 못 보는 띠를 '봤음' 으로 칠하면, 거기 누운 사람은
    # 영원히 못 찾는다(이미 봤다고 표시돼 다시 안 감).
    CAM_MIN = float(os.environ.get('UGV_CAM_MIN', '0.3'))
    # 같은 머신에서 두 런을 동시에 돌릴 때 임시 파일이 안 겹치게 한다.
    dom = os.environ.get('ROS_DOMAIN_ID', '0')
    robots = ROBOTS[:max(1, min(n, len(ROBOTS)))]
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
        # 3단계(목표 선점·관측 공유·명부 합산)를 껐다 켤 수 있게 한다.
        # 껐을 때와 켰을 때를 비교해야 3단계가 도움이 되는지 알 수 있다.
        DeclareLaunchArgument('team_share', default_value='true'),
        # 대수를 줄여 돌릴 수 있게 한다. 1 로 두면 네임스페이스 구성 자체가
        # 멀쩡한지(로봇끼리 간섭과 무관하게) 가릴 수 있다.
        DeclareLaunchArgument('n_robots', default_value='2'),

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

        # 로봇마다 갈래가 진 토픽(카메라·안개·코스트맵)을 두 벌로 복제한
        # 설정을 쓴다. 1대용 설정을 그대로 쓰면 카메라가 하나만 보이고
        # 안개가 엉뚱한 토픽을 그린다.
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg_desc, 'rviz',
                                           'ugv_multi.rviz')],
             parameters=[{'use_sim_time': True}], output='screen',
             condition=UnlessCondition(headless)),
    ]

    for i, r in enumerate(robots):
        name = r['name']
        prefix = f'{name}/'

        # 로봇 본체. /clock 은 월드에 하나뿐이어야 하므로 첫 대만 브리지한다.
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'sim_robot.launch.py')),
            launch_arguments={
                'name': name, 'prefix': prefix, 'world': world,
                'x': r['x'], 'y': r['y'], 'delay': str(SPAWN + i * 2.0),
                'bridge_clock': 'true' if i == 0 else 'false',
            }.items()))

        # SLAM — 로봇마다 자기 지도를 만든다. 프레임 이름이 안 갈리면
        # 두 SLAM 이 같은 map→odom 을 발행해 TF 트리가 깨진다.
        actions.append(TimerAction(period=SPAWN + 6.0 + i * 2.0, actions=[
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
                 # tf 도 같은 이유로 상대 이름으로 되돌린다(sim_robot.launch.py 주석 참조)
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
        actions.append(TimerAction(period=SPAWN + 14.0 + i * 4.0, actions=[
            GroupAction([
                PushRosNamespace(name),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_nav2, 'launch',
                                     'navigation_launch.py')),
                    launch_arguments={
                        'namespace': name,
                        'use_sim_time': 'true',
                        # ★ 파일 이름에 도메인을 넣는다. 안 그러면 같은
                        #   머신에서 시뮬을 둘 돌릴 때(A/B 비교) 두 런이
                        #   /tmp/nav2_ugv1.yaml 을 서로 덮어쓴다.
                        'params_file': namespaced_nav2_params(
                            nav2_params, prefix,
                            os.path.join('/tmp', f'nav2_{dom}_{name}.yaml')),
                        'slam': 'True',
                    }.items()),
            ])]))

    # ── 2단계: 공용 map 프레임과 지도 병합 ───────────────────────────
    # 두 지도는 이미 같은 원점을 쓴다. gz 의 OdometryPublisher 가 월드 원점
    # 기준으로 odom 을 내기 때문이다(실측: 스폰 (0,0.8) 인 로봇의 첫 odom 이
    # (0,0.8)). 그래서 map → ugvN/map 은 항등이고 병합 오프셋도 0 이다.
    # 스폰 좌표 차를 넣었다가 한쪽 벽의 99.2% 가 상대 지도의 자유공간에
    # 떨어졌다.
    #
    # Nav2 는 건드리지 않는다. 각자 자기 map 프레임에서 계속 계획하고,
    # 공용 지도는 '어디를 탐사할지' 를 정하는 순찰 노드만 쓴다. 목표는
    # map 프레임으로 나가고 Nav2 가 자기 프레임으로 변환해 받는다.
    # 이렇게 두면 1단계에서 검증된 Nav2 구성을 흔들지 않는다.
    #
    # ★ 이 정적 변환은 반드시 그 로봇의 네임스페이스 tf 로 나가야 한다.
    #   전역 /tf_static 에 내면 네임스페이스 노드들이 /ugvN/tf_static 만
    #   보므로 이 변환이 안 보이고, Nav2 가 map 프레임 목표를 자기 프레임으로
    #   변환하지 못해 모든 목표가 실패한다.
    #   실측: 1대만 돌려도 목표 11회 중 10회 도달 실패.
    #   robot_state_publisher, tf2 리스너에 이은 세 번째 같은 함정이다.
    for r in robots:
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f"map_to_{r['name']}", namespace=r['name'],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            arguments=['--x', '0', '--y', '0',
                       '--frame-id', 'map',
                       '--child-frame-id', f"{r['name']}/map"],
            parameters=[{'use_sim_time': True}], output='screen'))

    # RViz 는 전역 /tf 를 본다. 2대 구성에서는 각 로봇이 자기 이름공간의
    # tf 로 발행하므로(/ugv1/tf) 화면에 로봇 본체도 경로선도 안 나온다.
    # 프레임 이름은 이미 갈려 있어(ugv1/base_link) 한 트리에 합쳐도 안
    # 겹치니, 시각화용으로 그대로 옮겨 준다.
    actions.append(TimerAction(period=SPAWN + 10.0, actions=[
        Node(package='ugv_vision', executable='tf_relay_node',
             name='tf_relay_node',
             parameters=[{'use_sim_time': True,
                          'robots': [r['name'] for r in robots]}],
             output='screen')]))

    actions.append(TimerAction(period=SPAWN + 12.0, actions=[
        Node(package='ugv_vision', executable='map_merge_node',
             name='map_merge_node',
             parameters=[{
                 'use_sim_time': True,
                 'robots': [r['name'] for r in robots],
                 'offset_x': [0.0 for _ in robots],
                 'offset_y': [0.0 for _ in robots],
             }], output='screen')]))

    # 비전·순찰 — Nav2 lifecycle 활성화가 끝난 뒤에 띄운다. 먼저 띄우면
    # torch/CUDA 로딩 CPU 스파이크가 lifecycle 전환과 겹쳐 스택이 죽는다.
    for i, r in enumerate(robots):
        name = r['name']
        prefix = f'{name}/'
        # TF 리스너는 절대 '/tf' 를 구독한다(tf2_ros 구현). 네임스페이스를
        # 씌워도 안 따라오므로 상대 이름으로 되돌려야 /ugv1/tf 를 본다.
        tf_remap = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        common = [{'use_sim_time': True,
                   'map_frame': f'{prefix}map',
                   'base_frame': f'{prefix}base_footprint'}]
        actions.append(TimerAction(period=SPAWN + 44.0 + i * 4.0, actions=[
            Node(package='ugv_vision', executable='yolo_pose_node',
                 name='yolo_pose_node', namespace=name,
                 remappings=tf_remap,
                 parameters=[{'use_sim_time': True,
                              # 누운 사람 관문 완화 스위치.
                              # 큰 월드에서 못 찾는 조난자 두 명이 둘 다
                              # 누운 자세다(침대 41%, 바닥 70%. 나머지
                              # 다섯은 89~100%). 작은 맵에는 누운 사람이
                              # 하나뿐이라 신호가 묻혔었다 — 여기서 다시
                              # 잰다.
                              'lying_min_kpts': LYING_KPTS,
                              'lying_kpt_conf': LYING_CONF,
                              'det_conf': DET_CONF}],
                 output='screen'),
            Node(package='ugv_vision', executable='target_manager_node',
                 name='target_manager_node', namespace=name,
                 remappings=tf_remap,
                 parameters=common + [{'ghost_need': GHOST_NEED,
                                       'search_pitch': SEARCH_PITCH}],
                 output='screen'),
            Node(package='ugv_vision', executable='fire_detection_node',
                 name='fire_detection_node', namespace=name,
                 remappings=tf_remap,
                 # 동료가 찾은 화재를 합쳐야 '총 N건' 이 팀 기준이 된다.
                 # 이 노드가 그 숫자를 찍는다(순찰 노드가 아니다).
                 parameters=common + [{
                     'peers': [o['name'] for o in robots
                               if o['name'] != name] or ['']}],
                 output='screen'),
            # 순찰 노드만 공용 지도를 본다. 상대가 이미 만든 지도를 알아야
            # 같은 곳을 다시 훑지 않는다. 목표는 map 프레임으로 나가고
            # Nav2 가 자기 프레임으로 변환해 받는다.
            Node(package='ugv_vision', executable='patrol_navigator',
                 name='patrol_navigator', namespace=name,
                 remappings=tf_remap + [('map', '/map')],
                 parameters=[{'use_sim_time': True,
                              'map_frame': 'map',
                              'base_frame': f'{prefix}base_footprint',
                              # 탐사 범위는 월드마다 다르다. 상수로 박으면
                              # 다른 월드에서 아무 제약이 안 된다 — 실제로
                              # 큰 월드 값(+-27,+-19)을 박아 두는 바람에
                              # 18x12m 미니맵에서는 벽 밖 경계가 그대로
                              # 잡혀 완료 판정이 영영 안 섰다(경계 217셀).
                              # 내 구역(먼저 훑는 곳)
                              'seen_min_dirs': SEEN_DIRS,
                              'room_bonus': ROOM_BONUS,
                              'cam_see_min': CAM_MIN,
                              'explore_bounds': bounds_for(i, len(robots)),
                              # 건물 전체 — 내 구역을 끝내면 여기까지 넓혀
                              # 동료를 돕는다. 안 주면 자기 몫만 하고 논다.
                              'world_bounds': BOUNDS,
                              'min_area_for_sweep': MIN_AREA,
                              # 3단계: 동료의 목표·관측·명부를 받는다
                              'peers': [o['name'] for o in robots
                                        if o['name'] != name] or [''],
                              'team_share': ParameterValue(
                                  LaunchConfiguration('team_share'),
                                  value_type=bool)}] + [{
                     'patrol_mode': 'explore',
                     # 런치 인자는 문자열이라 그대로 주면 rclpy 가 STRING 으로
                     # 추론해 INTEGER 파라미터와 안 맞고 노드가 즉시 죽는다.
                     'expected_victims': ParameterValue(
                         LaunchConfiguration('expected_victims'),
                         value_type=int),
                 }], output='screen'),
        ]))

    return LaunchDescription(actions)
