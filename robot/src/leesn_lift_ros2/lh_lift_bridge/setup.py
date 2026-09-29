from glob import glob
import os

from setuptools import setup


package_name = "lh_lift_bridge"


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
    description="ROS 2 Humble HTTP bridge for the lift service.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lift_http_bridge = lh_lift_bridge.lift_bridge_node:main",
        ],
    },
)
