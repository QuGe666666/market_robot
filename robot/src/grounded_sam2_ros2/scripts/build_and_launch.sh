#!/bin/bash
set -e

workspace=/home/lh/robot
source /opt/ros/humble/setup.bash
cd "$workspace"
colcon build --symlink-install --packages-select grounded_sam2_ros2
source "$workspace/install/local_setup.bash"
export AMENT_PREFIX_PATH="$workspace/install/grounded_sam2_ros2:${AMENT_PREFIX_PATH}"
export PYTHONPATH="$workspace/build/grounded_sam2_ros2:${PYTHONPATH}"
exec ros2 launch grounded_sam2_ros2 dual_wrist_grounded_sam2.launch.py "$@"
