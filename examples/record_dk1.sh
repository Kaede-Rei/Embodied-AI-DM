#!/bin/bash
# DK1 机器人数据集录制 Bash 脚本（使用 lerobot-record 命令）
# 前提：DK1 已通过插件机制注册（--robot.type=dk1_follower 等可用）
# 使用前确保：
# 1. 已激活包含 LeRobot 的 Python 环境
# 2. 已 hf login（若需 push_to_hub）
# 3. 在 ~/.bashrc 中添加：export HF_HOME="自定义缓存路径"
# 
# 使用方法示例：
# ./examples/record_dk1.sh --repo_id $USER/dk1_my_task
# ./examples/record_dk1.sh --repo_id $USER/dk1_my_task --num_episodes 100 --task_description "Pick and place objects" --push_to_hub
# ./examples/record_dk1.sh --repo_id $USER/dk1_my_task --no_cameras --resume
# 
# 支持的参数：
# --follower_port <port> Follower 臂串口（默认 /dev/ttyACM0）
# --leader_port <port> Leader 臂串口（默认 /dev/ttyUSB0）
# --joint_velocity_scaling <val> 关节速度缩放（默认 1.0）
# --num_episodes <num> 录制 episode 数量（默认 50）
# --episode_time_s <sec> 每个 episode 时长（秒，默认 30）
# --reset_time_s <sec> 重置时间（秒，默认 15）
# --repo_id <repo_id> 必须：数据集 repo_id（如 $USER/dk1_test）
# --task_description <desc> 单任务描述（默认 "Task description."）
# --push_to_hub 录制后自动上传至 Hugging Face Hub（默认不启用）
# --resume 从现有数据集继续录制（默认不启用）
# --no_cameras 不启用摄像头（默认启用两个摄像头）
# --no_display 不启用 rerun.io 实时可视化（默认启用）

# 默认参数
FOLLOWER_PORT="/dev/ttyACM0"
LEADER_PORT="/dev/ttyUSB0"
JOINT_VELOCITY_SCALING=1.0
CAMERAS_CONFIG='{"PC": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}'
NUM_EPISODES=50
EPISODE_TIME_S=30
RESET_TIME_S=15
TASK_DESCRIPTION="Task description."
PUSH_TO_HUB=false
RESUME=false
DISPLAY_DATA=true
REPO_ID=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --follower_port) FOLLOWER_PORT="$2"; shift 2 ;;
        --leader_port) LEADER_PORT="$2"; shift 2 ;;
        --joint_velocity_scaling) JOINT_VELOCITY_SCALING="$2"; shift 2 ;;
        --num_episodes) NUM_EPISODES="$2"; shift 2 ;;
        --episode_time_s) EPISODE_TIME_S="$2"; shift 2 ;;
        --reset_time_s) RESET_TIME_S="$2"; shift 2 ;;
        --repo_id) REPO_ID="$2"; shift 2 ;;
        --task_description) TASK_DESCRIPTION="$2"; shift 2 ;;
        --push_to_hub) PUSH_TO_HUB=true; shift ;;
        --resume) RESUME=true; shift ;;
        --no_cameras) CAMERAS_CONFIG=""; shift ;;
        --no_display) DISPLAY_DATA=false; shift ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# 检查必须参数
if [[ -z "$REPO_ID" ]]; then
    echo "错误：必须指定 --repo_id（例如 $USER/dk1_my_task）"
    exit 1
fi

echo "启动 DK1 数据集录制（使用官方 lerobot-record）..."
echo "Repo ID: $REPO_ID"
echo "本地存储路径: $HF_HOME/lerobot/$REPO_ID（默认 /media/kaerei/.../huggingface/lerobot/$REPO_ID）"
echo "键盘操作提示："
echo " → (右箭头)：提前结束当前 episode 并保存"
echo " ← (左箭头)：取消当前 episode 并重新录制"
echo " ESC：立即停止整个录制过程（保存已录 episode，不失能电机）"
if [ "$DISPLAY_DATA" = true ]; then
    echo "rerun.io 实时可视化已启用。若需查看，请在新终端手动运行：rerun"
fi

# 构建参数数组
ARGS=(
    --robot.type=dk1_follower
    --robot.port="$FOLLOWER_PORT"
    --robot.joint_velocity_scaling="$JOINT_VELOCITY_SCALING"
    --teleop.type=dk1_leader
    --teleop.port="$LEADER_PORT"
    --dataset.repo_id="$REPO_ID"
    --dataset.push_to_hub="$PUSH_TO_HUB"
    --dataset.num_episodes="$NUM_EPISODES"
    --dataset.episode_time_s="$EPISODE_TIME_S"
    --dataset.reset_time_s="$RESET_TIME_S"
    --dataset.single_task="$TASK_DESCRIPTION"
    --robot.disable_torque_on_disconnect=false
)

# 添加可选的摄像头配置
if [ -n "$CAMERAS_CONFIG" ]; then
    ARGS+=(--robot.cameras="$CAMERAS_CONFIG")
fi

# 添加可选的恢复录制参数
if [ "$RESUME" = true ]; then
    ARGS+=(--resume=true)
fi

# 添加可选的显示数据参数
if [ "$DISPLAY_DATA" = true ]; then
    ARGS+=(--display_data=true)
fi

# 执行录制命令
lerobot-record "${ARGS[@]}"

# 检查命令执行是否成功
if [ $? -ne 0 ]; then
    echo "错误：lerobot-record 执行失败。"
    exit 1
fi

echo "录制完成！数据集已保存。"
echo "数据集本地路径：$HF_HOME/lerobot/$REPO_ID"
if [ "$PUSH_TO_HUB" = true ]; then
    echo "已上传至 Hugging Face Hub: https://huggingface.co/datasets/$REPO_ID"
fi