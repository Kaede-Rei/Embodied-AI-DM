#!/bin/bash
# DM 模型训练 Bash 脚本
#
# 前提：训练集存在且可访问
# 使用前确保：
# 1. 已激活包含 LeRobot 的 Python 环境
# 2. 若需后续上传模型，已执行 huggingface-cli login
# 3. 训练数据集 repo_id 已存在并可访问
#
# 使用方法示例：
# ./bash/train-dm.sh --policy_repo_id $USER/act_dm_model --dataset_repo_id $USER/dm_dataset
# ./bash/train-dm.sh --policy_repo_id $USER/act_dm_more_data --dataset_repo_id $USER/dm_dataset --steps 30000 --batch_size 4
# ./bash/train-dm.sh --policy_repo_id $USER/act_dm_model --dataset_repo_id $USER/dm_dataset --push_to_hub --steps 50000
#
# 支持的参数：
# --policy_repo_id <repo_id>            必须：模型仓库 ID（如 $USER/act_dm_model）
# --dataset_repo_id <repo_id>           必须：数据集仓库 ID（无默认值，必须显式指定）
# --batch_size <num>                    训练批次大小（默认 2）
# --steps <num>                         训练总步数（默认 40000）
# --push_to_hub                         训练完成后自动上传模型至 Hugging Face Hub（默认不启用）
# 其他 lerobot_train 支持的参数也可直接传入（如 --wandb.enable=true 等），脚本会原样透传

# 默认参数配置
VIDEO_BACKEND="pyav"          # 视频后端选择
POLICY_TYPE="act"             # 策略类型
PUSH_TO_HUB=false             # 是否上传至 Hugging Face Hub
DEVICE="cuda"                 # 训练设备选择
USE_VAE=false                 # 是否使用 VAE 编码器
N_ACTION_STEPS=50             # 动作步数（模型部署时采取预测序列的前 N 步）
CHUNK_SIZE=100                # 数据块大小（轨迹片段的时序长度，通常越大越接近原轨迹，越小泛化能力越强）
BATCH_SIZE=2                  # 训练批次大小（越小泛化能力越强但训练不稳定且慢，越大训练越稳定但易过拟合且需更多显存，通常 4-32，但显存不够时要降）
STEPS=40000                   # 训练总步数（episodes < 50时建议 20k-30k，episodes 50-200时建议 10k-40k，episodes > 200时建议 40k+）
DATASET_REPO_ID=""            # 必须参数：数据集仓库 ID
POLICY_REPO_ID=""             # 必须参数：模型仓库 ID

# 解析命令行参数
while [[ $# -gt 0 ]]; do
  case $1 in
    --policy_repo_id) POLICY_REPO_ID="$2"; shift 2 ;;
    --dataset_repo_id) DATASET_REPO_ID="$2"; shift 2 ;;
    --batch_size) BATCH_SIZE="$2"; shift 2 ;;
    --steps) STEPS="$2"; shift 2 ;;
    --push_to_hub) PUSH_TO_HUB=true; shift ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

# 检查必须参数
if [[ -z "$POLICY_REPO_ID" ]]; then
  echo "错误：必须指定 --policy_repo_id（例如 $USER/act_dm_model）"
  exit 1
fi
if [[ -z "$DATASET_REPO_ID" ]]; then
  echo "错误：必须指定 --dataset_repo_id（例如 $USER/dm_dataset）"
  exit 1
fi

# 从 policy_repo_id 提取仓库名称部分（例如 $USER/act_dm_model → act_dm_model）
REPO_NAME=$(basename "$POLICY_REPO_ID")

# 自动构造本地训练输出目录，与 policy_repo_id 的仓库名称保持一致
OUTPUT_DIR="outputs/train/$REPO_NAME"

# 确保 log 目录存在
mkdir -p ./log

# 输出启动信息
echo "========================================"
echo "- 启动 DM ACT 模型训练"
echo "- Policy Repo ID: $POLICY_REPO_ID"
echo "- Dataset Repo ID: $DATASET_REPO_ID"
echo "- 自动生成 Output Dir: $OUTPUT_DIR"
echo "- 日志文件: train_${REPO_NAME}.log"
if [ "$PUSH_TO_HUB" = true ]; then
  echo "- 训练完成后将自动上传模型至 Hugging Face Hub"
fi
echo "========================================"

# 执行后台训练命令（使用子壳确保 $! 获取正确的 PID）
(
  nohup python -m lerobot.scripts.lerobot_train \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.video_backend="$VIDEO_BACKEND" \
    --policy.type="$POLICY_TYPE" \
    --policy.repo_id="$POLICY_REPO_ID" \
    --policy.push_to_hub="$PUSH_TO_HUB" \
    --policy.device="$DEVICE" \
    --policy.use_vae="$USE_VAE" \
    --policy.n_action_steps="$N_ACTION_STEPS" \
    --policy.chunk_size="$CHUNK_SIZE" \
    --batch_size="$BATCH_SIZE" \
    --steps="$STEPS" \
    --output_dir="$OUTPUT_DIR" \
    --job_name="${REPO_NAME}" \
    "${EXTRA_ARGS[@]}" \
    > "./log/train_${REPO_NAME}.log" 2>&1
) &

# 捕获训练进程的 PID
TRAIN_PID=$!

# 生成参数文件 train_${REPO_NAME}.param
PARAM_FILE="./log/train_${REPO_NAME}.param"
{
  echo "TRAIN_PID=$TRAIN_PID"
  echo "POLICY_REPO_ID=$POLICY_REPO_ID"
  echo "DATASET_REPO_ID=$DATASET_REPO_ID"
  echo "OUTPUT_DIR=$OUTPUT_DIR"
  echo "VIDEO_BACKEND=$VIDEO_BACKEND"
  echo "POLICY_TYPE=$POLICY_TYPE"
  echo "PUSH_TO_HUB=$PUSH_TO_HUB"
  echo "DEVICE=$DEVICE"
  echo "USE_VAE=$USE_VAE"
  echo "N_ACTION_STEPS=$N_ACTION_STEPS"
  echo "CHUNK_SIZE=$CHUNK_SIZE"
  echo "BATCH_SIZE=$BATCH_SIZE"
  echo "STEPS=$STEPS"
  echo "EXTRA_ARGS=${EXTRA_ARGS[*]}"
} > "$PARAM_FILE"

# 训练启动后的提示信息
echo "- 训练已在后台启动（进程 PID: $TRAIN_PID）"
echo "- 日志输出至 train_${REPO_NAME}.log"
echo "- 参数及 PID 已保存至 $PARAM_FILE"
echo "- 检查点将保存在 $OUTPUT_DIR/checkpoints/"
echo "- 可使用 tail -f train_${REPO_NAME}.log 实时监控训练进度"
echo "- 可使用 ps -p $TRAIN_PID 或 top/htop 查看进程状态"
echo "- 如需终止训练，可执行 kill $TRAIN_PID"
echo "- 训练完成后，推荐使用 $OUTPUT_DIR/checkpoints/last/pretrained_model 进行评估部署"
echo "========================================="