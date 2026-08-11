import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, TextSubstitution)
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_desc = get_package_share_directory('ugv_description')
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_desc, 'urdf', 'ugv.urdf.xacro')
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

        # 2. 로봇 상태 퍼블리셔 (use_sim_time 고정)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 3. 로봇 스폰
        TimerAction(
            # 큰 월드(모델 50개 이상 + Fuel 메시)는 로딩이 길어, 3초에 스폰하면
            # /world/<name>/create 서비스가 아직 안 떠서 로봇이 안 생긴다.
            period=8.0, 
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-world', world, '-topic', 'robot_description', '-name', 'ugv', '-x', '0.0', '-y', '0.0', '-z', '0.15'],
                    output='screen'
                )
            ]
        ),

        # 4. ROS-Gazebo 브리지
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/ugv/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/turret_yaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/turret_pitch_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                # 깊이 포인트클라우드 — Nav2 장애물 입력.
                # 2D 라이다는 지면 0.23m 한 평면만 본다. 누운 사람, 높이 뜬
                # 침상·휠체어는 그 평면을 벗어나 지도에 아예 안 찍혔고
                # (실측: 조난자 7명 중 3명이 SLAM 맵에서 빈 공간),
                # 로봇이 그대로 밀고 들어갔다.
                '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/thermal/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            remappings=[
                ('/model/ugv/tf',      '/tf'),
                ('/camera/image',       '/camera/camera/color/image_raw'),
                ('/camera/depth_image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
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
