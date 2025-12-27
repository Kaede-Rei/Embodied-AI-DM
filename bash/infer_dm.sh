#!/bin/bash
# DM 纯推理运行脚本（持续运行，不录制）

# ================= 默认参数 =================

FOLLOWER_PORT="/dev/ttyACM0"
JOINT_VELOCITY_SCALING=1.0
FROM_HUB=false
POLICY_REPO_ID=""
CONTROL_FPS=25
N_ACTION_STEPS=1
TEMPORAL_ENSEMBLE_COEFF=0.01

CAMERAS_CONFIG='{"context": {"type": "opencv", "index_or_path": 0, "width": 1280, "height": 720, "fps": 25}}'
NO_CAMERAS=false

# ================= 解析参数 =================

while [[ $# -gt 0 ]]; do
case $1 in
--policy_repo_id) POLICY_REPO_ID="$2"; shift 2 ;;
--from_hub) FROM_HUB=true; shift ;;
--follower_port) FOLLOWER_PORT="$2"; shift 2 ;;
--joint_velocity_scaling) JOINT_VELOCITY_SCALING="$2"; shift 2 ;;
--control_fps) CONTROL_FPS="$2"; shift 2 ;;
--no_cameras) NO_CAMERAS=true; CAMERAS_CONFIG=""; shift ;;
--n_action_steps) N_ACTION_STEPS="$2"; shift 2 ;;
--temporal_ensemble_coeff) TEMPORAL_ENSEMBLE_COEFF="$2"; shift 2 ;;
*) echo "未知参数: $1"; exit 1 ;;
esac
done

if [[ -z "$POLICY_REPO_ID" ]]; then
echo "错误：必须指定 --policy_repo_id"
exit 1
fi

# 本地模型路径
POLICY_REPO_NAME=$(basename "$POLICY_REPO_ID")
MODEL_PATH="outputs/train/$POLICY_REPO_NAME/checkpoints/last/pretrained_model"

ARGS=(
  --robot.type=dm_follower
  --follower_port="$FOLLOWER_PORT"
  --robot.joint_velocity_scaling="$JOINT_VELOCITY_SCALING"
  --control_fps="$CONTROL_FPS"
  --policy.n_action_steps="$N_ACTION_STEPS"
)

if [ "$N_ACTION_STEPS" = 1 ]; then
  ARGS+=(--policy.temporal_ensemble_coeff="$TEMPORAL_ENSEMBLE_COEFF")
fi

if [ "$FROM_HUB" = true ]; then
  ARGS+=(--policy.repo_id="$POLICY_REPO_ID")
else
  ARGS+=(--policy.path="$MODEL_PATH")
fi

if [ -n "$CAMERAS_CONFIG" ]; then
  ARGS+=(--robot.cameras="$CAMERAS_CONFIG")
fi

echo "启动推理..."
dm-infer "${ARGS[@]}"
