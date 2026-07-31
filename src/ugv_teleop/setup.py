from setuptools import setup
import os
from glob import glob

package_name = 'ugv_teleop'

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
    maintainer='junyeon',
    maintainer_email='junyeon@todo.todo',
    description='Teleoperation node for CQB Bot using Xbox Controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joy_node = ugv_teleop.teleop_joy_node:main',
            'teleop_keyboard_node = ugv_teleop.teleop_keyboard_node:main'
        ],
    },
)
