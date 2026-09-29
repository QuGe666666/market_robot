from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = "grasp_nvblox_curobo_test"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml", "README.md"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (os.path.join("share", PACKAGE_NAME, "docs"), glob("docs/*.md")),
        (os.path.join("share", PACKAGE_NAME, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("lib", PACKAGE_NAME), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lh",
    maintainer_email="lh@localhost",
    description="Minimum validation demo for GraspNet, nvblox, and CuRobo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "grasp_test_node = grasp_nvblox_curobo_test.grasp_test_node:main",
            "timestamped_graspnet_node = grasp_nvblox_curobo_test.timestamped_graspnet_node:main",
            "curobo_validation_node = grasp_nvblox_curobo_test.curobo_validation_node:main",
            "validation_summary = grasp_nvblox_curobo_test.summary:main",
        ]
    },
)
