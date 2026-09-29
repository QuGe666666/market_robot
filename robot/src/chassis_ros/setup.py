from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'chassis_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lh',
    maintainer_email='user@example.com',
    description='Woosh Robot Chassis ROS2 Interface Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_monitor = chassis_ros.monitor_node:main',
            'chassis_twist = chassis_ros.twist_node:main',
            'chassis_goto = chassis_ros.goto_node:main',
            'chassis_step = chassis_ros.step_node:main',
            'woosh_compat_bridge = chassis_ros.compat_bridge:main',
            'chassis_terminal_nav = chassis_ros.terminal_nav:main',
        ],
    },
)
