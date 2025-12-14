#!/bin/bash

# DM Arm 机械臂服务测试脚本
# 用于测试所有机械臂控制服务的命令行工具

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 服务名称
EEF_SERVICE="/dm_arm_server/eef_cmd"
TASK_SERVICE="/dm_arm_server/task_planner"

# 打印带颜色的标题
print_title() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# 打印成功信息
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

# 打印警告信息
print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# 打印错误信息
print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# 检查服务是否存在
check_service() {
    if rosservice list | grep -q "$1"; then
        print_success "服务 $1 已找到"
        return 0
    else
        print_error "服务 $1 未找到，请先启动服务端"
        return 1
    fi
}

# 等待用户按键继续
wait_for_key() {
    echo ""
    read -p "按回车继续..." dummy
    echo ""
}

# ============================================
# 末端执行器控制测试
# ============================================
test_eef_control() {
    print_title "测试末端执行器控制服务"
    
    if ! check_service "$EEF_SERVICE"; then
        return 1
    fi
    
    # 1. 测试回零
    echo -e "\n${YELLOW}1. 测试回零命令${NC}"
    rosservice call $EEF_SERVICE "command: 'zero'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    # 2. 测试获取当前位姿
    echo -e "\n${YELLOW}2. 测试获取当前位姿${NC}"
    rosservice call $EEF_SERVICE "command: 'get_pose'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    # 3. 测试获取关节角度
    echo -e "\n${YELLOW}3. 测试获取关节角度${NC}"
    rosservice call $EEF_SERVICE "command: 'get_joints'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    # 4. 测试移动到目标位置（基座坐标系）
    echo -e "\n${YELLOW}4. 测试移动到目标位置（基座坐标系）${NC}"
    echo "目标位置: x=0.15, y=0.2, z=0.2"
    rosservice call $EEF_SERVICE "command: 'goal_base'
x: 0.15
y: 0.2
z: 0.2
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    # 5. 测试移动到目标位置（末端坐标系）
    echo -e "\n${YELLOW}5. 测试移动到目标位置（末端坐标系）${NC}"
    echo "相对移动: x=-0.1, y=0.0, z=0.0"
    rosservice call $EEF_SERVICE "command: 'goal_eef'
x: -0.1
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    # 6. 测试末端伸缩
    echo -e "\n${YELLOW}6. 测试末端伸缩${NC}"
    echo "伸出5cm"
    rosservice call $EEF_SERVICE "command: 'stretch'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '0.05'
param2: ''
param3: ''"
    wait_for_key
    
    # 7. 测试末端旋转
    echo -e "\n${YELLOW}7. 测试末端旋转${NC}"
    echo "旋转90度"
    rosservice call $EEF_SERVICE "command: 'rotate'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '90.0'
param2: ''
param3: ''"
    wait_for_key
    
    # 8. 测试未知命令
    echo -e "\n${YELLOW}8. 测试未知命令（应该返回错误）${NC}"
    rosservice call $EEF_SERVICE "command: 'unknown_command'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    print_success "末端执行器控制测试完成"
}

# ============================================
# 任务组规划器测试
# ============================================
test_task_planner() {
    print_title "测试任务组规划器服务"
    
    if ! check_service "$TASK_SERVICE"; then
        return 1
    fi

    # 1. 回零
    echo -e "\n${YELLOW}1. 回零${NC}"
    rosservice call $EEF_SERVICE "command: 'zero'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key

    # 2. 清除所有任务
    echo -e "\n${YELLOW}2. 清除所有任务${NC}"
    rosservice call $TASK_SERVICE "command: 'clear_tasks'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    wait_for_key
    
    # 3. 添加任务1 - 移动到位置1
    echo -e "\n${YELLOW}3. 添加任务1 - 移动到位置1${NC}"
    echo "位置: (0.15, 0.2, 0.3), 等待1秒, 动作: NONE"
    rosservice call $TASK_SERVICE "command: 'add_task'
x: 0.15
y: 0.2
z: 0.3
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: 'NONE'
param3: ''"
    wait_for_key
    
    # 4. 添加任务2 - 移动到位置2
    echo -e "\n${YELLOW}4. 添加任务2 - 移动到位置2${NC}"
    echo "位置: (0.2, 0.3, 0.4), 等待1秒, 动作: NONE"
    rosservice call $TASK_SERVICE "command: 'add_task'
x: 0.2
y: 0.3
z: 0.4
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: 'NONE'
param3: ''"
    wait_for_key
    
    # 5. 添加任务3 - 移动到位置3
    echo -e "\n${YELLOW}5. 添加任务3 - 移动到位置3${NC}"
    echo "位置: (0.2, 0.1, 0.4), 等待1秒, 动作: NONE"
    rosservice call $TASK_SERVICE "command: 'add_task'
x: 0.2
y: 0.1
z: 0.4
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: 'NONE'
param3: ''"
    wait_for_key
    
    # 6. 添加任务4 - 伸缩动作
    echo -e "\n${YELLOW}6. 添加任务4 - 伸缩动作${NC}"
    echo "位置: (0.2, 0.4, 0.3), 等待1秒, 动作: STRETCH, 参数: 0.05"
    rosservice call $TASK_SERVICE "command: 'add_task'
x: 0.2
y: 0.4
z: 0.3
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: 'STRETCH'
param3: '0.05'"
    wait_for_key
    
    # 7. 添加任务5 - 旋转动作
    echo -e "\n${YELLOW}7. 添加任务5 - 旋转动作${NC}"
    echo "位置: (-0.15, 0.2, 0.3), 等待1秒, 动作: ROTATE, 参数: 45.0"
    rosservice call $TASK_SERVICE "command: 'add_task'
x: 0.15
y: 0.25
z: 0.3
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: 'ROTATE'
param3: '45.0'"
    wait_for_key
    
    # 8. 执行所有任务
    echo -e "\n${YELLOW}8. 执行所有任务（将自动进行路径优化）${NC}"
    print_warning "注意: 任务将按照贪婪最近邻算法优化后的顺序执行"
    read -p "确认执行? (y/n): " confirm
    if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
        rosservice call $TASK_SERVICE "command: 'exe_all_tasks'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
    else
        print_warning "跳过执行"
    fi
    wait_for_key
    
    print_success "任务组规划器测试完成"
}

# ============================================
# 快速测试（简化版）
# ============================================
quick_test() {
    print_title "快速测试（基本功能）"
    
    echo -e "\n${YELLOW}测试回零${NC}"
    rosservice call $EEF_SERVICE "command: 'zero'" 2>/dev/null
    echo ""
    
    echo -e "${YELLOW}测试获取位姿${NC}"
    rosservice call $EEF_SERVICE "command: 'get_pose'" 2>/dev/null
    echo ""
    
    echo -e "${YELLOW}测试获取关节角度${NC}"
    rosservice call $EEF_SERVICE "command: 'get_joints'" 2>/dev/null
    echo ""
    
    print_success "快速测试完成"
}

# ============================================
# 主菜单
# ============================================
show_menu() {
    clear
    print_title "DM Arm 机械臂服务测试工具"
    echo ""
    echo "1) 测试末端执行器控制服务 (详细)"
    echo "2) 测试任务组规划器服务 (详细)"
    echo "3) 快速测试 (基本功能)"
    echo "4) 检查服务状态"
    echo "5) 显示服务信息"
    echo "0) 退出"
    echo ""
    read -p "请选择 [0-5]: " choice
    
    case $choice in
        1)
            test_eef_control
            read -p "按回车返回主菜单..." dummy
            show_menu
            ;;
        2)
            test_task_planner
            read -p "按回车返回主菜单..." dummy
            show_menu
            ;;
        3)
            quick_test
            read -p "按回车返回主菜单..." dummy
            show_menu
            ;;
        4)
            print_title "检查服务状态"
            check_service "$EEF_SERVICE"
            check_service "$TASK_SERVICE"
            read -p "按回车返回主菜单..." dummy
            show_menu
            ;;
        5)
            print_title "服务信息"
            echo -e "\n${YELLOW}末端执行器控制服务:${NC}"
            rosservice info $EEF_SERVICE 2>/dev/null || print_error "服务未找到"
            echo -e "\n${YELLOW}任务组规划器服务:${NC}"
            rosservice info $TASK_SERVICE 2>/dev/null || print_error "服务未找到"
            read -p "按回车返回主菜单..." dummy
            show_menu
            ;;
        0)
            print_success "退出测试工具"
            exit 0
            ;;
        *)
            print_error "无效选择"
            sleep 1
            show_menu
            ;;
    esac
}

# ============================================
# 脚本入口
# ============================================

# 检查是否在ROS环境中
if [ -z "$ROS_DISTRO" ]; then
    print_error "未检测到ROS环境，请先source ROS"
    exit 1
fi

# 显示主菜单
show_menu
