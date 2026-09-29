from glob import glob
import os

from setuptools import find_packages, setup


package_name = "qwen2_5_vl_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="lh@localhost",
    description="Qwen2.5-VL object grounding for wrist RGB-D cameras.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = qwen2_5_vl_ros2.perception_node:main",
            "prompt_cli = qwen2_5_vl_ros2.prompt_cli:main",
            "save_result = qwen2_5_vl_ros2.save_result:main",
        ]
    },
)
