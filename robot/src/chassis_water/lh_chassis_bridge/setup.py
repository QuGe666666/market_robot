from glob import glob
import os

from setuptools import setup


package_name = "lh_chassis_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Codex",
    maintainer_email="devnull@example.com",
    description="ROS 2 Humble TCP bridge for the WATER chassis.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "water_bridge = lh_chassis_bridge.water_bridge_node:main",
        ],
    },
)
