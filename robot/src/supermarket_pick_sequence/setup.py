from glob import glob
import os

from setuptools import find_packages, setup


package_name = "supermarket_pick_sequence"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name), glob("*.md")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="lh@example.com",
    description="Event-driven dual-arm competition FSM and ROS 2 hardware coordinator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task_sequence_node = supermarket_pick_sequence.competition_fsm_node:main",
            "competition_fsm_node = supermarket_pick_sequence.competition_fsm_node:main",
            "legacy_task_sequence_node = supermarket_pick_sequence.task_sequence_node:main",
        ],
    },
)
