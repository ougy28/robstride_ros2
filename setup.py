from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rob_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),   # includes rob_py + robstride_dynamics
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 Python package for RobStride motor control via CAN bus',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mit_control_node = rob_py.mit_control_node:main',
            'motor_scan_node = rob_py.motor_scan_node:main',
            'vel_control_node = rob_py.vel_control_node:main',
            'pp_control_node = rob_py.pp_control_node:main',
            'multi_turn_rs04 = rob_py.multi_turn_rs04:main',
            'multi_turn_rs05 = rob_py.multi_turn_rs05:main',
            'multi_turn_rs06 = rob_py.multi_turn_rs06:main',
            '7_dof_v1 = rob_py.7_dof_v1:main',
        ],
    },
)
