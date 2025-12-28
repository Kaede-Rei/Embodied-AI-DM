#!/bin/bash
# DM 机器人数据集录制 Bash 脚本（使用 lerobot-record 命令）
#
# 前提：DM 已通过插件机制注册（--robot.type=dm_follower 等可用）
# 使用前确保：
# 1. 已激活包含 LeRobot 的 Python 环境
# 2. 已 hf login（若需 push_to_hub）
# 3. 在 ~/.bashrc 中添加：export HF_HOME="自定义缓存路径"
# 
# 使用方法示例：
# ./bash/record_dm.sh --repo_id agro/dm_record_vx（组织文件夹，vx 为版本号）
# ./bash/record_dm.sh --repo_id $USER/dm_my_task（个人文件夹，可用于测试）
# ./bash/record_dm.sh --repo_id $USER/dm_my_task --no_cameras
# ./bash/record_dm.sh --repo_id $USER/dm_my_task --num_episodes 100 --task_description "Pick and place objects" --push_to_hub
# 
# 支持的参数：
# --repo_id <repo_id>                   必须：数据集 repo_id（如 $USER/dm_test）
# --follower_port <port> Follower       从臂串口（默认 /dev/ttyACM0）
# --leader_port <port> Leader           主臂串口（默认 /dev/ttyUSB0）
# --joint_velocity_scaling <val>        关节速度缩放（默认 1.0）
# --num_episodes <num>                  录制 episode 数量（默认 50）
# --episode_time_s <sec>                每个 episode 时长（秒，默认 30）
# --reset_time_s <sec>                  重置时间（秒，默认 0）
# --task_description <desc>             单任务描述（默认 "Task description."）
# --push_to_hub                         录制后自动上传至 Hugging Face Hub（默认不启用）
# --resume                              从现有数据集继续录制（默认启用）
# --no_cameras                          不启用摄像头（默认启用两个摄像头）
# --no_display                          不启用 rerun.io 实时可视化（默认启用）

# 默认参数配置
FOLLOWER_PORT="/dev/ttyACM0"            # Follower 臂串口
LEADER_PORT="/dev/ttyUSB0"              # Leader 臂串口
JOINT_VELOCITY_SCALING=1.0              # 关节速度缩放
# 预设摄像头配置，要严格按照示例格式填写：
# CAMERAS_CONFIG='{"相机名称": {"type": "opencv", "index_or_path": 设备索引或路径, "width": 宽度, "height": 高度, "fps": 帧率}}'
CAMERAS_CONFIG='{"context": {"type": "opencv", "index_or_path": 2, "width": 1280, "height": 720, "fps": 25}}'
NUM_EPISODES=50                         # 录制 episode 数量
EPISODE_TIME_S=30                       # 每个 episode 时长（秒）
RESET_TIME_S=0                          # 重置时间（秒），注意实际总重置时间是 重置时间 + (当前 episode 实际使用的时间 + 重置时间)，所以写 0 即可
TASK_DESCRIPTION="Task description."    # 单任务描述
DATASET_FPS=25                          # 数据集保存的帧率（默认25，必须与相机帧率一致）
PUSH_TO_HUB=false                       # 是否上传至 Hugging Face Hub
RESUME=true                             # 是否从现有数据集继续录制
DISPLAY_DATA=true                       # 是否启用 rerun.io 实时可视化
REPO_ID=""                              # 数据集 repo_id（必须参数）

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
    echo "错误：必须指定 --repo_id（例如 $USER/dm_my_task）"
    exit 1
fi

# 自动检测并验证帧率一致性
if [ -n "$CAMERAS_CONFIG" ]; then
    echo "检查相机帧率与数据集帧率一致性..."
    
    # 从CAMERAS_CONFIG中提取第一个相机的帧率
    EXTRACTED_FPS=$(python3 -c "
import json
config = json.loads('$CAMERAS_CONFIG')
if config:
    first_camera = next(iter(config.values()))
    print(first_camera.get('fps', $DATASET_FPS))
else:
    print($DATASET_FPS)
")
    
    # 如果指定了dataset_fps但相机配置也有帧率，检查是否一致
    if [ "$EXTRACTED_FPS" != "$DATASET_FPS" ]; then
        echo "警告：相机帧率($EXTRACTED_FPS)与数据集帧率($DATASET_FPS)不一致！"
        echo "建议：使用 --dataset_fps $EXTRACTED_FPS 保持一致性"
        read -p "是否自动调整数据集帧率为相机帧率? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            DATASET_FPS=$EXTRACTED_FPS
            echo "已自动调整数据集帧率为: $DATASET_FPS"
        fi
    fi
fi

echo "========================================"
echo "- 启动 DM 数据集录制（使用官方 lerobot-record）..."
echo "- Repo ID: $REPO_ID"
echo "- 数据集帧率: $DATASET_FPS Hz"
echo "- 本地存储路径: $HF_HOME/lerobot/$REPO_ID"
echo "- 键盘操作提示："
echo "-     → (右箭头)：提前结束当前 episode 并保存"
echo "-     ← (左箭头)：取消当前 episode 并重新录制"
echo "-     ESC：立即停止整个录制过程（保存已录 episode，不失能电机）"
if [ "$DISPLAY_DATA" = true ]; then
    echo "- rerun.io 实时可视化已启用"
fi
echo "========================================"

# 构建参数数组
ARGS=(
    --robot.type=dm_follower
    --robot.port="$FOLLOWER_PORT"
    --robot.joint_velocity_scaling="$JOINT_VELOCITY_SCALING"
    --teleop.type=dm_leader
    --teleop.port="$LEADER_PORT"
    --dataset.repo_id="$REPO_ID"
    --dataset.fps="$DATASET_FPS"
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

# 检测数据集是否已存在
DATASET_PATH="$HF_HOME/lerobot/$REPO_ID"
if [[ ! -d "$DATASET_PATH" ]]; then
    RESUME=false
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
  echo "- 错误：lerobot-record 执行失败"
  exit 1
fi

echo "========================================="
echo "- 录制完成！数据集已保存至：$HF_HOME/lerobot/$REPO_ID"
echo "- 数据集帧率: $DATASET_FPS Hz"
if [ "$PUSH_TO_HUB" = true ]; then
    echo "- 已上传至 Hugging Face Hub: https://huggingface.co/datasets/$REPO_ID"
fi
echo "========================================="