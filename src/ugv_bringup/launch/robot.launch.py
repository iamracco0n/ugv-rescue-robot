"""로봇 한 대분 — 상태 퍼블리셔 + 스폰 + ROS-Gazebo 브리지.

여러 대를 띄우려면 이 파일을 이름만 바꿔 여러 번 include 한다.
월드와 RViz 는 여기 없다(공용이므로 gazebo.launch.py 가 맡는다).

인자
  name    gz 모델 이름이자 네임스페이스. 1대 구성에서는 'ugv'.
  prefix  TF 프레임·gz 토픽 접두사. 비우면 지금까지와 완전히 같다.
          2대일 때는 'ugv1/' 처럼 슬래시로 끝낸다.
  x, y    스폰 위치.
  delay   스폰까지 기다릴 초. 큰 월드는 로딩이 길어 8초 이상 필요하다
          (짧으면 /world/<name>/create 서비스가 아직 없어 로봇이 안 생긴다).

주의: prefix 는 xacro 인자로도 그대로 넘어간다. 둘이 어긋나면 TF 는
접두사가 붙었는데 브리지는 안 붙은 토픽을 구독해 아무것도 안 온다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def _robot(context, *args, **kwargs):
    name   = LaunchConfiguration('name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    world  = LaunchConfiguration('world').perform(context)
    x      = LaunchConfiguration('x').perform(context)
    y      = LaunchConfiguration('y').perform(context)
    delay  = float(LaunchConfiguration('delay').perform(context))

    pkg_desc = get_package_share_directory('ugv_description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'ugv.urdf.xacro')

    # 네임스페이스가 비면 토픽 이름이 지금까지와 똑같아진다(1대 구성 보존).
    ns = name if prefix else ''
    desc_topic = f'/{name}/robot_description' if ns else '/robot_description'

    # gz 쪽 토픽은 URDF 에서 이미 접두사가 붙어 나온다(/ugv1/scan).
    # ROS 쪽도 같은 이름으로 받아 짝을 맞춘다.
    p = prefix                     # 'ugv1/' 또는 ''
    bridge_args = [
        f'/{p}cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'/{p}odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/model/{p}ugv/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        f'/{p}joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        f'/{p}scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'/{p}turret_yaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        f'/{p}turret_pitch_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        f'/{p}camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        f'/{p}camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        # 깊이 포인트클라우드 — Nav2 장애물 입력(라이다 평면 밖 형상용)
        f'/{p}camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        f'/{p}thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
    ]
    # 비전 노드가 RealSense 이름을 기대하므로 그대로 맞춰준다.
    remaps = [
        (f'/model/{p}ugv/tf', '/tf'),
        (f'/{p}camera/image',
         f'/{p}camera/camera/color/image_raw'),
        (f'/{p}camera/depth_image',
         f'/{p}camera/camera/aligned_depth_to_color/image_raw'),
    ]

    # 클럭은 월드마다 하나면 된다. 두 대가 각자 브리지하면 같은 /clock 에
    # 두 번 발행돼 시간이 튄다. 첫 로봇만 맡는다.
    if LaunchConfiguration('bridge_clock').perform(context) == 'true':
        bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file, ' prefix:=', prefix]),
                    value_type=str),
                'use_sim_time': True,
            }],
            output='screen'),

        TimerAction(period=delay, actions=[
            Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', world, '-topic', desc_topic,
                            '-name', name, '-x', x, '-y', y, '-z', '0.15'],
                 output='screen')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=bridge_args, remappings=remaps,
             parameters=[{'use_sim_time': True}], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name',   default_value='ugv'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('world',  default_value='rescue_building'),
        DeclareLaunchArgument('x',      default_value='0.0'),
        DeclareLaunchArgument('y',      default_value='0.0'),
        DeclareLaunchArgument('delay',  default_value='8.0'),
        DeclareLaunchArgument('bridge_clock', default_value='true'),
        OpaqueFunction(function=_robot),
    ])
