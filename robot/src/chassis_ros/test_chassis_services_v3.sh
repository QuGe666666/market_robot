#!/bin/bash

# 底盘Service测试脚本（修复版V3 - 正确参数格式）
# 使用方法: ./test_chassis_services_v3.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 计数器
TOTAL=15
PASSED=0
FAILED=0

# 测试函数
test_service() {
    local service_name=$1
    local service_type=$2
    local test_num=$3
    local description=$4
    shift 4
    local request_data=("$@")

    echo ""
    echo "=========================================="
    echo "[${test_num}/${TOTAL}] 测试: ${description}"
    echo "服务: ${service_name}"
    echo "=========================================="

    # 先检查服务是否存在
    if ! ros2 service list | grep -q "${service_name}"; then
        echo -e "${RED}✗ 失败: 服务不存在${NC}"
        ((FAILED++))
        return 1
    fi

    # 调用服务
    echo "调用中..."
    if timeout 10 ros2 service call "${service_name}" "${service_type}" "${request_data[@]}" > /tmp/service_test.txt 2>&1; then
        if [ -s /tmp/service_test.txt ]; then
            echo -e "${GREEN}✓ 通过: 服务调用成功${NC}"
            echo "响应摘要:"
            grep -E "(ok=|ok=|Request succeed|Request failed)" /tmp/service_test.txt | head -n 3 | sed 's/^/  /'
            ((PASSED++))
            return 0
        else
            echo -e "${RED}✗ 失败: 无响应数据${NC}"
            ((FAILED++))
            return 1
        fi
    else
        echo -e "${RED}✗ 失败: 服务调用超时或失败${NC}"
        cat /tmp/service_test.txt 2>/dev/null | head -n 3 | sed 's/^/  /'
        ((FAILED++))
        return 1
    fi
}

# 打印标题
echo ""
echo "=============================================="
echo "       底盘 ROS2 Service 测试脚本"
echo "=============================================="
echo "测试时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# 检查ros2是否可用
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ros2命令未找到，请先source ROS2环境${NC}"
    exit 1
fi

# 列出所有woosh_robot服务
echo "当前woosh_robot服务列表:"
ros2 service list | grep woosh_robot | wc -l | sed 's/^/  共 /' | sed 's/$/ 个服务/'
echo ""

echo -e "${YELLOW}注意: 此脚本已排除以下高风险服务:${NC}"
echo "  - Twist (速度控制)"
echo "  - ExecPreTask (预定义任务)"
echo "  - SwitchMap (切换地图)"
echo "  - ChangeNavPath (修改导航路径)"
echo "  - SwitchFootPrint (切换足迹)"
echo "  - Follow (跟随模式)"
echo "  - PowerOff (关机)"
echo ""

# 开始测试
echo -e "${BLUE}开始测试服务...${NC}"

# 2. InitRobot - 初始化机器人位置（记录当前位置）
test_service "/woosh_robot/robot/InitRobot" "woosh_robot_msgs/srv/InitRobot" "1" "InitRobot - 初始化机器人位置(记录模式)" "{arg: {is_record: true}}"

# 3. RobotInfo - 获取机器人完整信息
test_service "/woosh_robot/robot/RobotInfo" "woosh_robot_msgs/srv/RobotInfo" "2" "RobotInfo - 获取机器人完整信息" "{}"

# 5. General - 获取常规信息
test_service "/woosh_robot/robot/General" "woosh_robot_msgs/srv/General" "3" "General - 获取常规信息" "{}"

# 6. Setting - 获取配置信息
test_service "/woosh_robot/robot/Setting" "woosh_robot_msgs/srv/Setting" "4" "Setting - 获取配置信息" "{}"

# 7. SetRobotPose - 设置机器人位姿
test_service "/woosh_robot/robot/SetRobotPose" "woosh_robot_msgs/srv/SetRobotPose" "5" "SetRobotPose - 设置机器人位姿" "{arg: {pose: {x: 0.0, y: 0.0, theta: 0.0}}}"

# 8. SetOccupancy - 设置占据栅格 (修正参数格式)
test_service "/woosh_robot/robot/SetOccupancy" "woosh_robot_msgs/srv/SetOccupancy" "6" "SetOccupancy - 设置占据栅格" "{arg: {x: 1.0, y: 1.0, value: 0}}"

# 11. ChangeNavMode - 修改导航模式 (修正参数格式: mode是嵌套的value)
test_service "/woosh_robot/robot/ChangeNavMode" "woosh_robot_msgs/srv/ChangeNavMode" "7" "ChangeNavMode - 修改导航模式" "{arg: {mode: {value: 0}}}"

# 12. SwitchControlMode - 切换控制模式 (修正参数格式: mode是嵌套的value)
test_service "/woosh_robot/robot/SwitchControlMode" "woosh_robot_msgs/srv/SwitchControlMode" "8" "SwitchControlMode - 切换控制模式" "{arg: {mode: {value: 0}}}"

# 13. SwitchWorkMode - 切换工作模式 (修正参数格式: mode是嵌套的value)
test_service "/woosh_robot/robot/SwitchWorkMode" "woosh_robot_msgs/srv/SwitchWorkMode" "9" "SwitchWorkMode - 切换工作模式" "{arg: {mode: {value: 1}}}"

# 15. SetMuteCall - 设置静音呼叫 (修正参数格式)
test_service "/woosh_robot/robot/SetMuteCall" "woosh_robot_msgs/srv/SetMuteCall" "10" "SetMuteCall - 设置静音呼叫(禁用)" "{arg: {enable: false}}"

# 16. SetProgramMute - 设置程序静音 (修正参数格式)
test_service "/woosh_robot/robot/SetProgramMute" "woosh_robot_msgs/srv/SetProgramMute" "11" "SetProgramMute - 设置程序静音(禁用)" "{arg: {enable: false}}"

# 17. SetHoldMode - 设置保持模式 (修正参数格式)
test_service "/woosh_robot/robot/SetHoldMode" "woosh_robot_msgs/srv/SetHoldMode" "12" "SetHoldMode - 设置保持模式(禁用)" "{arg: {enable: false}}"

# 18. Speak - 语音播放
test_service "/woosh_robot/robot/Speak" "woosh_robot_msgs/srv/Speak" "13" "Speak - 语音播放测试" '{arg: {text: "你好"}}'

# 20. RobotWiFi - WiFi配置
test_service "/woosh_robot/robot/RobotWiFi" "woosh_robot_msgs/srv/RobotWiFi" "14" "RobotWiFi - 获取WiFi配置" "{}"

# 21. LED - LED控制 (修正参数格式: id是嵌套的value)
test_service "/woosh_robot/robot/LED" "woosh_robot_msgs/srv/LED" "15" "LED - LED控制测试" "{arg: {id: {value: 0}, mode: 1, color: 0}}"

# 打印测试结果摘要
echo ""
echo "=============================================="
echo "              测试结果摘要"
echo "=============================================="
echo -e "总计: ${TOTAL} 个服务"
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
