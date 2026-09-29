from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jd_gripper'

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
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='YJing',
    maintainer_email='user@example.com',
    description='ROS2 package for JODELL (钧舵) RG series robot electric gripper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jd_gripper_node = jd_gripper.jd_gripper_node:main',
        ],
    },
)
