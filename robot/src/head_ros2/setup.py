from setuptools import find_packages, setup

package_name = 'head_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/head_test.launch.py', 'launch/head_server.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lh',
    maintainer_email='lh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'head_server = head_ros2.head_server:main',
            'test_client = head_ros2.test_client:main',
        ],
    },
)
