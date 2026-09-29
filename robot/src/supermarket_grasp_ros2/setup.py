from glob import glob
import os

from setuptools import find_packages, setup


package_name = "supermarket_grasp_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.json")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="lh@localhost",
    description="ROS2 wrapper for the RealSense GraspNet supermarket grasp pipeline.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "grasp_node = supermarket_grasp_ros2.grasp_node:main",
        ],
    },
)
