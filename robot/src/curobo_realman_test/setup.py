from glob import glob
import os

from setuptools import find_packages, setup


package_name = "curobo_realman_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("lib", package_name), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="lh@localhost",
    description="Interactive CuRobo trajectory planning test for dual RealMan RM65 arms.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "planner_node = curobo_realman_test.planner_node:main",
            "wrist_camera_tf = curobo_realman_test.wrist_camera_tf:main",
        ]
    },
)
