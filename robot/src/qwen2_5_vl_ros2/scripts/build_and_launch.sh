#!/bin/bash
set -e

workspace=/home/lh/robot
source /opt/ros/humble/setup.bash
cd "$workspace"
colcon build --symlink-install --packages-select qwen2_5_vl_ros2
source "$workspace/install/local_setup.bash"
export AMENT_PREFIX_PATH="$workspace/install/qwen2_5_vl_ros2:${AMENT_PREFIX_PATH}"
export PYTHONPATH="$workspace/build/qwen2_5_vl_ros2:${PYTHONPATH}"
exec ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py "$@"
