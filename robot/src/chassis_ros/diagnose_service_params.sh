#!/bin/bash

# 服务参数格式诊断脚本
# 用于找出正确的参数格式

echo "=========================================="
echo "  服务参数格式诊断"
echo "=========================================="
echo ""

# 测试 SetOccupancy 的不同格式
echo "=== 测试 SetOccupancy ==="
echo "格式1: {arg: {x: 1.0, y: 1.0, value: 0, map_name: ''}}"
echo "格式2: {x: 1.0, y: 1.0, value: 0, map_name: ''}"
echo "格式3: {arg: {x: 1.0}}"
echo ""

echo "测试命令（请逐个尝试）："
echo "1. ros2 service call /woosh_robot/robot/SetOccupancy woosh_robot_msgs/srv/SetOccupancy '{arg: {x: 1.0, y: 1.0, value: 0, map_name: \"\"}}'"
echo "2. ros2 service call /woosh_robot/robot/SetOccupancy woosh_robot_msgs/srv/SetOccupancy '{x: 1.0, y: 1.0, value: 0, map_name: \"\"}'"
echo ""

echo "=== 测试 ChangeNavMode ==="
echo "格式1: {arg: {mode: {value: 0}}}"
echo "格式2: {mode: {value: 0}}"
echo "格式3: {arg: {mode: 0}}"
echo ""
echo "测试命令（请逐个尝试）："
echo "1. ros2 service call /woosh_robot/robot/ChangeNavMode woosh_robot_msgs/srv/ChangeNavMode '{arg: {mode: {value: 0}}}'"
echo "2. ros2 service call /woosh_robot/robot/ChangeNavMode woosh_robot_msgs/srv/ChangeNavMode '{mode: {value: 0}}'"
echo "3. ros2 service call /woosh_robot/robot/ChangeNavMode woosh_robot_msgs/srv/ChangeNavMode '{arg: {mode: 0}}'"
echo ""

echo "=== 测试 LED ==="
echo "格式1: {arg: {id: {value: 0}, mode: 1, color: 0}}"
echo "格式2: {id: {value: 0}, mode: 1, color: 0}"
echo "格式3: {arg: {id: 0, mode: 1, color: 0}}"
echo ""
echo "测试命令（请逐个尝试）："
echo "1. ros2 service call /woosh_robot/robot/LED woosh_robot_msgs/srv/LED '{arg: {id: {value: 0}, mode: 1, color: 0}}'"
echo "2. ros2 service call /woosh_robot/robot/LED woosh_robot_msgs/srv/LED '{id: {value: 0}, mode: 1, color: 0}'"
echo "3. ros2 service call /woosh_robot/robot/LED woosh_robot_msgs/srv/LED '{arg: {id: 0, mode: 1, color: 0}}'"
echo ""

echo "=========================================="
echo "  诊断方法"
echo "=========================================="
echo ""
echo "如果某个格式成功，请记录下来，格式为："
echo "  服务名: 正确的参数格式"
echo ""
echo "例如："
echo "  SetOccupancy: {arg: {x: 1.0, y: 1.0, value: 0, map_name: ''}}"
echo ""
