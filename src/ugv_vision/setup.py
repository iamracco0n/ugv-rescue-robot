from setuptools import setup

package_name = 'ugv_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='YOLO vision and Turret Target Manager node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'yolo_pose_node = ugv_vision.yolo_pose_node:main',
            'target_manager_node = ugv_vision.target_manager_node:main',
            'visual_coverage_node = ugv_vision.visual_coverage_map:main',
            'tactical_commander_node = ugv_vision.tactical_commander_node:main',
            'vision_coverage_navigator = ugv_vision.vision_coverage_navigator:main',
            'visibility_overlay_node = ugv_vision.visibility_overlay_node:main',
            'fire_detection_node = ugv_vision.fire_detection_node:main',
            'patrol_navigator = ugv_vision.patrol_navigator:main',
        ],
    },
)
