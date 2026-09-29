from glob import glob
import os

from setuptools import find_packages, setup


package_name = "supermarket_grasp_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (f"share/{package_name}", ["CURRENT_SYSTEM_AUDIT.md", "README.md"]),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "ui_node = supermarket_grasp_ui.main:main",
            "desktop_ui = supermarket_grasp_ui.main:main",
            "competition_console = supermarket_grasp_ui.main:main",
            "legacy_ui = supermarket_grasp_ui.desktop_ui:main",
        ],
    },
)
