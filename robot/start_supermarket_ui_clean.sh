#!/usr/bin/env bash
set -euo pipefail

# Stop only the Supermarket pipeline processes before starting a fresh UI.
patterns=(
  'ros2 launch supermarket_grasp_ui ui.launch.py'
  'ros2 launch realsense2_camera rs_launch.py'
  'ros2 launch qwen2_5_vl_ros2 wrist_qwen_vl.launch.py'
  'ros2 launch supermarket_grasp_ros2 grasp.launch.py'
  'ros2 launch curobo_realman_test dual_arm_curobo.launch.py'
  '/supermarket_grasp_ros2/lib/supermarket_grasp_ros2/grasp_node'
  '/curobo_realman_test/lib/curobo_realman_test/planner_node'
  '/curobo_realman_test/lib/rm_driver/rm_driver'
  '/omnipicker_gripper/lib/omnipicker_gripper/omnipicker_modbus_node'
)

for pattern in "${patterns[@]}"; do
  pkill -INT -f "$pattern" || true
done

for _ in {1..30}; do
  still_running=0
  for pattern in "${patterns[@]}"; do
    if pgrep -f "$pattern" >/dev/null; then
      still_running=1
      break
    fi
  done
  if (( still_running == 0 )); then
    break
  fi
  sleep 1
done

# Remove stale DDS discovery entries. This does not stop any process.
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

export AMENT_PREFIX_PATH="/home/lh/robot/install/omnipicker_gripper:/home/lh/robot/install/curobo_realman_test:/home/lh/robot/install/qwen2_5_vl_ros2:/home/lh/robot/install/supermarket_grasp_ros2:/home/lh/robot/install/supermarket_grasp_ui:${AMENT_PREFIX_PATH:-}"
exec ros2 launch supermarket_grasp_ui ui.launch.py "${@:-default_arm:=right}"
