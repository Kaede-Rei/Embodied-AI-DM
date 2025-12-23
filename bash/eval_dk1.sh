#!/bin/bash
# DK1 机器人模型评估 Bash 脚本（使用 lerobot-record 加载训练好的模型）
# 前提：DK1 已通过插件机制注册（--robot.type=dk1_follower 等可用）
# 使用前确保：
# 1. 已激活包含 LeRobot 的 Python 环境
# 2. DK1 Follower 臂已正确连接并上电（无需 Leader 臂）
# 3. 对应的模型训练已完成（outputs/train/<policy_repo_name>/checkpoints/last 存在）
# 4. 若需上传评估数据集，已执行 huggingface-cli login
#
# 使用方法示例：
# ./eval_dk1.sh --policy_repo_id $USER/act_dk1_model
# ./eval_dk1.sh --policy_repo_id $USER/act_dk1_model --repo_id $USER/custom_eval
# ./eval_dk1.sh --policy_repo_id $USER/act_dk1_more_data --joint_velocity_scaling 0.7 --push_to_hub
# ./eval_dk1.sh --policy_repo_id $USER/act_dk1_model --no_display --no_cameras --resume
#
# 支持的参数：
# --policy_repo_id <repo_id>            必须：训练时使用的 policy.repo_id（如 $USER/act_dk1_model），用于自动定位模型路径
# --repo_id <repo_id>                   评估数据集的 repo_id（可选，默认自动为 $USER/eval_<policy_repo_name>）
# --follower_port <port> Follower       从臂串口（默认 /dev/ttyACM0）
# --joint_velocity_scaling <val>        关节速度缩放因子（默认 1.0）
# --episode_time_s <sec>                每个 episode 最大执行时长（秒，默认 30）
# --task_description <desc>             评估数据集的单任务描述（默认 "Evaluation of trained ACT policy."）
# --push_to_hub                         评估完成后自动上传数据集至 Hugging Face Hub（默认不启用）
# --resume                              从现有评估数据集继续录制（默认启用）
# --no_cameras                          不启用摄像头录制（默认启用单个 PC 摄像头）
# --no_display                          不启用 rerun.io 实时可视化（默认启用）

# 默认参数配置
FOLLOWER_PORT="/dev/ttyACM0"
JOINT_VELOCITY_SCALING=1.0
NUM_EPISODES=1                  # 固定为 1
EPISODE_TIME_S=30
RESET_TIME_S=0                  # 固定为 0
TASK_DESCRIPTION="Evaluation of trained ACT policy."
PUSH_TO_HUB=false
CAMERAS_CONFIG='{"context": {"type": "opencv", "index_or_path": 0, "width": 1280, "height": 720, "fps": 25}}'
DISPLAY_DATA=true
RESUME=true
POLICY_REPO_ID=""
REPO_ID=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
  case $1 in
    --policy_repo_id) POLICY_REPO_ID="$2"; shift 2 ;;
    --repo_id) REPO_ID="$2"; shift 2 ;;
    --follower_port) FOLLOWER_PORT="$2"; shift 2 ;;
    --joint_velocity_scaling) JOINT_VELOCITY_SCALING="$2"; shift 2 ;;
    --episode_time_s) EPISODE_TIME_S="$2"; shift 2 ;;
    --task_description) TASK_DESCRIPTION="$2"; shift 2 ;;
    --push_to_hub) PUSH_TO_HUB=true; shift ;;
    --resume) RESUME=true; shift ;;
    --no_cameras) CAMERAS_CONFIG=""; shift ;;
    --no_display) DISPLAY_DATA=false; shift ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

# 检查必须参数
if [[ -z "$POLICY_REPO_ID" ]]; then
  echo "错误：必须指定 --policy_repo_id（例如 $USER/act_dk1_model），用于定位训练好的模型"
  exit 1
fi

# 从 policy_repo_id 提取仓库名称部分
POLICY_REPO_NAME=$(basename "$POLICY_REPO_ID")

# 自动构造模型路径
MODEL_PATH="outputs/train/$POLICY_REPO_NAME/checkpoints/last/pretrained_model"

# 检查模型路径是否存在
if [[ ! -d "$MODEL_PATH" ]]; then
  echo "错误：模型路径不存在：$MODEL_PATH"
  echo "请确认对应的训练已完成（policy_repo_id: $POLICY_REPO_ID）"
  exit 1
fi

# 若未指定 --repo_id，则自动生成 eval_ 前缀版本
if [[ -z "$REPO_ID" ]]; then
  REPO_ID="${POLICY_REPO_ID/%${POLICY_REPO_NAME}/eval_${POLICY_REPO_NAME}}"
  echo "提示：未指定 --repo_id，自动使用 $REPO_ID 作为评估数据集 repo_id"
fi

# 输出启动信息
echo "========================================"
echo "- 启动 DK1 模型评估与部署（自主执行 + 录制评估数据集）"
echo "- 训练模型 Policy Repo ID: $POLICY_REPO_ID"
echo "- 自动加载模型路径: $MODEL_PATH"
echo "- 评估数据集 Repo ID: $REPO_ID"
echo "- 本地存储路径: $HF_HOME/lerobot/$REPO_ID"
echo "- 注意：机器人将自主执行任务，请确保环境安全！"
echo "- 键盘操作：Ctrl+C 可紧急停止（会安全断开电机）"
if [ "$DISPLAY_DATA" = true ]; then
  echo "- rerun.io 实时可视化已启用"
fi
echo "========================================"

# 构建 lerobot-record 参数数组
ARGS=(
  --robot.type=dk1_follower
  --robot.port="$FOLLOWER_PORT"
  --robot.joint_velocity_scaling="$JOINT_VELOCITY_SCALING"
  --dataset.repo_id="$REPO_ID"
  --dataset.push_to_hub="$PUSH_TO_HUB"
  --dataset.num_episodes="$NUM_EPISODES"
  --dataset.episode_time_s="$EPISODE_TIME_S"
  --dataset.reset_time_s="$RESET_TIME_S"
  --dataset.single_task="$TASK_DESCRIPTION"
  --policy.path="$MODEL_PATH"
  --robot.disable_torque_on_disconnect=false
)

# 添加可选摄像头配置
if [ -n "$CAMERAS_CONFIG" ]; then
  ARGS+=(--robot.cameras="$CAMERAS_CONFIG")
fi

# 检测评估数据集是否已存在
DATASET_PATH="$HF_HOME/lerobot/$REPO_ID"
if [[ ! -d "$DATASET_PATH" ]]; then
  RESUME=false
fi

# 添加 resume 参数（仅当 RESUME=true 时添加）
if [ "$RESUME" = true ]; then
  ARGS+=(--resume=true)
fi

# 添加可选实时可视化
if [ "$DISPLAY_DATA" = true ]; then
  ARGS+=(--display_data=true)
fi

# 执行评估部署命令
lerobot-record "${ARGS[@]}"

# 检查执行结果
if [ $? -ne 0 ]; then
  echo "- 错误：lerobot-record 执行失败"
  exit 1
fi

# 完成提示
echo "========================================="
echo "- 评估完成！评估数据集已保存至：$HF_HOME/lerobot/$REPO_ID"
echo "- 可使用以下命令本地可视化："
echo " python -m lerobot.scripts.visualize_dataset --repo-id $REPO_ID"
echo "- 或访问 Hugging Face Space 在线查看（若已上传）"
if [ "$PUSH_TO_HUB" = true ]; then
  echo "- 已上传至 Hugging Face Hub: https://huggingface.co/datasets/$REPO_ID"
fi
echo "========================================="