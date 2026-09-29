from setuptools import setup

package_name = 'yolov8_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python', 'numpy'],
    zip_safe=True,
    author='lh',
)
