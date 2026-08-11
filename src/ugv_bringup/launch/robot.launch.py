import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ugv_bringup')
    pkg_desc = get_package_share_directory('ugv_description')
    pkg_nav = get_package_share_directory('ugv_navigation')
    
    urdf_file = os.path.join(pkg_desc, 'urdf', 'ugv.urdf.xacro')

    # 1. 로봇 뼈대 정보 (URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        output='screen'
    )

    # 2. 리얼센스 D435i (15fps 설정 및 IMU 통합 활성화)
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2'
        }.items()
    )

    # 3. 라이다 (SLLidar A1/A2)
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        parameters=[{'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser_link'}]
    )

    # 4. 차체 고정 USB IMU
    usb_imu = Node(
        package='imu_sensor_driver', 
        executable='imu_node',
        parameters=[{'port': '/dev/ttyUSB1', 'frame_id': 'imu_link'}]
    )

    # 5. 오도메트리 및 센서 퓨전 (EKF)
    wheel_odom = Node(package='ugv_navigation', executable='odometry_node')
    ekf_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'ekf.launch.py'))
    )

    # 6. Teensy 통신 (Micro-ROS)
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0']
    )

    return LaunchDescription([
        robot_state_publisher,
        realsense,
        lidar,
        usb_imu,
        wheel_odom,
        ekf_fusion,
        microros_agent
    ])
