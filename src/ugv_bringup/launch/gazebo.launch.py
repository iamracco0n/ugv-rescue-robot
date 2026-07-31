import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_desc = get_package_share_directory('ugv_description')
    pkg_bringup = get_package_share_directory('ugv_bringup') 
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    xacro_file = os.path.join(pkg_desc, 'urdf', 'ugv.urdf.xacro')
    rviz_config_file = os.path.join(pkg_desc, 'rviz', 'ugv.rviz')
    
    world_file = os.path.join(pkg_bringup, 'worlds', 'rescue_building.sdf')

    return LaunchDescription([

        # 1. 가제보 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
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
            period=3.0, 
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-world', 'rescue_building', '-topic', 'robot_description', '-name', 'ugv', '-x', '0.0', '-y', '0.0', '-z', '0.15'],
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

        # 5. 알비즈 실행
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
