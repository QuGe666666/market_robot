#!/bin/bash

# 底盘Topic测试脚本
# 测试所有woosh_robot状态话题
# 使用方法: ./test_chassis_topics.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 计数器
TOTAL=8
PASSED=0
FAILED=0

# 测试函数
test_topic() {
    local topic_name=$1
    local topic_num=$2
    local description=$3

    echo ""
    echo "=========================================="
    echo "[${topic_num}/${TOTAL}] 测试: ${description}"
    echo "话题: ${topic_name}"
    echo "=========================================="

    # 先检查话题是否存在
    if ! ros2 topic list | grep -q "${topic_name}"; then
        echo -e "${RED}✗ 失败: 话题不存在${NC}"
        ((FAILED++))
        return 1
    fi

    # 测试话题是否有数据
    if timeout 5 ros2 topic echo "${topic_name}" --once > /tmp/topic_test.txt 2>&1; then
        if [ -s /tmp/topic_test.txt ]; then
            echo -e "${GREEN}✓ 通过: 成功接收到数据${NC}"
            echo "内容预览:"
            head -n 10 /tmp/topic_test.txt | sed 's/^/  /'
            ((PASSED++))
            return 0
        else
            echo -e "${RED}✗ 失败: 未接收到数据${NC}"
            ((FAILED++))
            return 1
        fi
    else
        echo -e "${RED}✗ 失败: 话题无响应${NC}"
        ((FAILED++))
        return 1
    fi
}

# 打印标题
echo ""
echo "=============================================="
echo "       底盘 ROS2 Topic 测试脚本"
echo "=============================================="
echo "测试时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# 检查ros2是否可用
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ros2命令未找到，请先source ROS2环境${NC}"
    exit 1
fi

# 列出所有woosh_robot话题
echo "当前woosh_robot话题列表:"
ros2 topic list | grep woosh_robot | sed 's/^/  /'
echo ""

# 开始测试
test_topic "/woosh_robot/robot/PoseSpeed" "1" "PoseSpeed - 位姿和速度"
test_topic "/woosh_robot/robot/Battery" "2" "Battery - 电池信息"
test_topic "/woosh_robot/robot/RobotState" "3" "RobotState - 机器人在线状态"
test_topic "/woosh_robot/robot/Mode" "4" "Mode - 工作模式和控制模式"
test_topic "/woosh_robot/robot/Scene" "5" "Scene - 场景信息"
test_topic "/woosh_robot/robot/TaskProc" "6" "TaskProc - 任务处理进度"
test_topic "/woosh_robot/robot/DeviceState" "7" "DeviceState - 设备状态"
test_topic "/woosh_robot/robot/OperationState" "8" "OperationState - 运行状态"

# 打印测试结果摘要
echo ""
echo "=============================================="
echo "              测试结果摘要"
echo "=============================================="
echo -e "总计: ${TOTAL} 个话题"
echo -e "${GREEN}通过: ${PASSED}${NC}"
echo -e "${RED}失败: ${FAILED}${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}==============================================${NC}"
    echo -e "${GREEN}         ✓ 所有测试通过！${NC}"
    echo -e "${GREEN}==============================================${NC}"
    exit 0
else
    echo -e "${RED}==============================================${NC}"
    echo -e "${RED}         ✗ 部分测试失败${NC}"
    echo -e "${RED}==============================================${NC}"
    exit 1
fi
