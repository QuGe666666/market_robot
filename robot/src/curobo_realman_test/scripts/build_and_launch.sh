#!/usr/bin/env bash
set -euo pipefail

workspace="/home/lh/robot"
image_project="/home/lh/robot_env/lerobot-realman-orin"
lock_file="/tmp/curobo_realman_test.lock"

exec 9>"$lock_file"
if ! flock -n 9; then
  echo "curobo_realman_test is already running; stop the existing launch before starting another" >&2
  exit 1
fi

cd "$image_project"
docker compose build robot-runtime

cd "$workspace"
# ROS 2 setup files assume unset variables are allowed.
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --packages-select curobo_realman_test
set +u
source "$workspace/install/setup.bash"
set -u
export AMENT_PREFIX_PATH="$workspace/install/curobo_realman_test:${AMENT_PREFIX_PATH}"
export PYTHONPATH="$workspace/install/curobo_realman_test/lib/python3.10/site-packages:${PYTHONPATH}"
exec ros2 launch curobo_realman_test dual_arm_curobo.launch.py "$@"
