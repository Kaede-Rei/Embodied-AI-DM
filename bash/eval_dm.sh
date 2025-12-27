#!/bin/bash
# DM 机器人模型评估 Bash 脚本（使用 lerobot-record 加载训练好的模型）
#
# 前提：DM 已通过插件机制注册（--robot.type=dm_follower 等可用）
# 使用前确保：
# 1. 已激活包含 LeRobot 的 Python 环境
# 2. DM Follower 臂已正确连接并上电（无需 Leader 臂）
# 3. 若使用本地模型：对应的模型训练已完成（outputs/train/<policy_repo_name>/checkpoints/last 存在）
# 4. 若从 Hugging Face Hub 加载模型或上传评估数据集：已执行 huggingface-cli login
#
# 使用方法示例：
# ./bash/eval_dm.sh --policy_repo_id $USER/act_dm_model
# ./bash/eval_dm.sh --policy_repo_id $USER/act_dm_model --from_hub
# ./bash/eval_dm.sh --policy_repo_id $USER/act_dm_model --repo_id $USER/custom_eval
# ./bash/eval_dm.sh --policy_repo_id $USER/act_dm_model --num_episodes 10 --joint_velocity_scaling 0.7 --push_to_hub
# ./bash/eval_dm.sh --policy_repo_id $USER/act_dm_model --no_display --no_cameras --resume
# ./bash/eval_dm.sh --policy_repo_id $USER/act_dm_model --n_action_steps 10 --temporal_ensemble_coeff 0.02
#
# 支持的参数：
# --policy_repo_id <repo_id>            必须：模型仓库 ID（如 $USER/act_dm_model）
# --from_hub                            可选：若指定，则从 Hugging Face Hub 下载并加载模型（默认不启用，使用本地路径）
# --repo_id <repo_id>                   评估数据集的 repo_id（可选，默认自动为 eval_<policy_repo_name>）
# --follower_port <port> Follower       从臂串口（默认 /dev/ttyACM0）
# --joint_velocity_scaling <val>        关节速度缩放因子（默认 0.1 以确保平滑且安全）
# --episode_time_s <sec>                每个 episode 最大执行时长（秒，默认 30）
# --num_episodes <num>                  要录制的评估 rollout 数量（默认 5，推荐 5-20 以统计性能）
# --task_description <desc>             评估数据集的单任务描述（默认 "Evaluation of trained ACT policy."）
# --push_to_hub                         评估完成后自动上传数据集至 Hugging Face Hub（默认不启用）
# --resume                              从现有评估数据集继续录制（默认启用）
# --no_cameras                          不启用摄像头录制（默认启用单个 PC 摄像头）
# --no_display                          不启用 rerun.io 实时可视化（默认启用）
# --n_action_steps <int> ACT            策略中执行的动作步数（默认 1，启用平滑集成，较小值更平滑但速度慢，一般取 1~50）
# --temporal_ensemble_coeff <float>     时序集成衰减系数（默认 0.01，，仅在 n_action_steps=1 时生效；值越小平滑越强，相当于对输出进行低通滤波）

# 默认参数配置
FOLLOWER_PORT="/dev/ttyACM0"
JOINT_VELOCITY_SCALING=1.0
NUM_EPISODES=5
EPISODE_TIME_S=30
RESET_TIME_S=0                          # 固定为 0
TASK_DESCRIPTION="Evaluation of trained ACT policy."
DATASET_FPS=30                          # 数据集保存的帧率（默认25，必须与相机帧率一致，会自动处理）
PUSH_TO_HUB=false
CAMERAS_CONFIG='{"context": {"type": "opencv", "index_or_path": 0, "width": 1280, "height": 720, "fps": 25}}'
DISPLAY_DATA=true
RESUME=true
FROM_HUB=false
POLICY_REPO_ID=""
REPO_ID=""
N_ACTION_STEPS=1
TEMPORAL_ENSEMBLE_COEFF=0.01

# 解析命令行参数
while [[ $# -gt 0 ]]; do
case $1 in
--policy_repo_id) POLICY_REPO_ID="$2"; shift 2 ;;
--from_hub) FROM_HUB=true; shift ;;
--repo_id) REPO_ID="$2"; shift 2 ;;
--follower_port) FOLLOWER_PORT="$2"; shift 2 ;;
--joint_velocity_scaling) JOINT_VELOCITY_SCALING="$2"; shift 2 ;;
--episode_time_s) EPISODE_TIME_S="$2"; shift 2 ;;
--num_episodes) NUM_EPISODES="$2"; shift 2 ;;
--task_description) TASK_DESCRIPTION="$2"; shift 2 ;;
--push_to_hub) PUSH_TO_HUB=true; shift ;;
--resume) RESUME=true; shift ;;
--no_cameras) CAMERAS_CONFIG=""; shift ;;
--no_display) DISPLAY_DATA=false; shift ;;
--n_action_steps) N_ACTION_STEPS="$2"; shift 2 ;;
--temporal_ensemble_coeff) TEMPORAL_ENSEMBLE_COEFF="$2"; shift 2 ;;
*) echo "未知参数: $1"; exit 1 ;;
esac
done

# 检查必须参数
if [[ -z "$POLICY_REPO_ID" ]]; then
echo "错误：必须指定 --policy_repo_id（例如 $USER/act_dm_model）"
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

# 从 policy_repo_id 提取仓库名称部分
POLICY_REPO_NAME=$(basename "$POLICY_REPO_ID")

# 若未指定 --repo_id，则自动生成 eval_ 前缀版本
if [[ -z "$REPO_ID" ]]; then
REPO_ID="${POLICY_REPO_ID/%${POLICY_REPO_NAME}/eval_${POLICY_REPO_NAME}}"
echo "提示：未指定 --repo_id，自动使用 $REPO_ID 作为评估数据集 repo_id"
fi

# 输出启动信息
echo "========================================"
echo "- 启动 DM 模型评估与部署（自主执行 + 录制评估数据集）"
echo "- 训练模型 Policy Repo ID: $POLICY_REPO_ID"
echo "- 评估 episodes 数量: $NUM_EPISODES"
if [ "$FROM_HUB" = true ]; then
echo "- 模型加载来源: Hugging Face Hub（将自动下载如未缓存）"
else
MODEL_PATH="outputs/train/$POLICY_REPO_NAME/checkpoints/last/pretrained_model"
if [[ ! -d "$MODEL_PATH" ]]; then
echo "错误：本地模型路径不存在：$MODEL_PATH"
echo "请确认对应的训练已完成，或使用 --from_hub 从 Hugging Face Hub 加载"
exit 1
fi
echo "- 自动加载本地模型路径: $MODEL_PATH"
fi
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
    --robot.type=dm_follower
    --robot.port="$FOLLOWER_PORT"
    --robot.joint_velocity_scaling="$JOINT_VELOCITY_SCALING"
    --robot.disable_torque_on_disconnect=false
    --dataset.repo_id="$REPO_ID"
    --dataset.fps="$DATASET_FPS"
    --dataset.push_to_hub="$PUSH_TO_HUB"
    --dataset.num_episodes="$NUM_EPISODES"
    --dataset.episode_time_s="$EPISODE_TIME_S"
    --dataset.reset_time_s="$RESET_TIME_S"
    --dataset.single_task="$TASK_DESCRIPTION"
    --policy.n_action_steps="$N_ACTION_STEPS"
)

# 处理 temporal ensembling 兼容性
if [ "$N_ACTION_STEPS" = 1 ]; then
ARGS+=(--policy.temporal_ensemble_coeff="$TEMPORAL_ENSEMBLE_COEFF")
fi

# 根据加载来源添加 policy 参数
if [ "$FROM_HUB" = true ]; then
ARGS+=(--policy.repo_id="$POLICY_REPO_ID")
else
ARGS+=(--policy.path="$MODEL_PATH")
fi

# 添加可选摄像头配置
if [ -n "$CAMERAS_CONFIG" ]; then
ARGS+=(--robot.cameras="$CAMERAS_CONFIG")
fi

# 检测评估数据集是否已存在
DATASET_PATH="$HF_HOME/lerobot/$REPO_ID"
if [[ ! -d "$DATASET_PATH" ]]; then
RESUME=false
fi

# 添加 resume 参数
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