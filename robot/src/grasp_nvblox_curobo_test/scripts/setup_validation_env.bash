#!/usr/bin/env bash

set +u
source /opt/ros/humble/setup.bash
source /home/lh/robot/install/setup.bash
set -u

validation_prefix=/home/lh/robot/install/grasp_nvblox_curobo_test
curobo_prefix=/home/lh/robot/install/curobo_realman_test
export AMENT_PREFIX_PATH="$validation_prefix:$curobo_prefix:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$validation_prefix/lib/python3.10/site-packages:$curobo_prefix/lib/python3.10/site-packages:${PYTHONPATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/lh/robot/src/curobo_realman_test/config/fastdds_udp.xml

unset validation_prefix
unset curobo_prefix
