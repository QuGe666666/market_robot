from setuptools import find_packages, setup


setup(
    name="robot_brain",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_brain"]),
        ("share/robot_brain", ["package.xml"]),
        ("share/robot_brain/launch", ["launch/competition_bringup.launch.py"]),
        ("share/robot_brain/config", [
            "config/competition.yaml", "config/navigation.yaml", "config/stations.yaml", "config/task_fsm.yaml", "config/manipulation_fsm.yaml", "config/perception.yaml", "config/planning.yaml", "config/recovery.yaml", "config/safety.yaml", "config/interfaces.yaml", "config/runtime.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "competition_manager = robot_brain.competition_manager:main",
        "competition_task_fsm = robot_brain.task_fsm_node:main",
        "world_state_manager = robot_brain.world_state_node:main",
        "safety_supervisor = robot_brain.safety_node:main",
        "health_monitor = robot_brain.health_node:main",
        "robot_brain_component = robot_brain.component_node:main",
    ]},
)
