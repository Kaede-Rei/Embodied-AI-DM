import torch
from lerobot.policies.pretrained import PreTrainedPolicy
# 引入 ACT 的时间集成器类
from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
from lerobot_robot_multi_robots.dm_arm import DMFollower, DMFollowerConfig

# ================= 配置区域 =================
# 加载策略
policy = PreTrainedPolicy.from_pretrained(
    "./outputs/train/act_dk1_test_v3/checkpoints/last/pretrained_model"
)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
policy.to(device)
policy.eval()

# 预定义平滑设置列表
# (n_action_steps, TEMPORAL_ENSEMBLE_COEFF)
SMOOTH_SETTINGS = [
    (1,   0.01),    # 官方推荐 (强平滑)
    (1,   0.05),    # 响应更快
    (1,   None),    # 无平滑
]
SELECT_SMOOTH_IDX = 0  # <--- 修改这里选择平滑程度

target_n_action_steps, alpha = SMOOTH_SETTINGS[SELECT_SMOOTH_IDX]
print(f"Applying Settings -> n_action_steps: {target_n_action_steps}, Smoothing (alpha): {alpha}")

# ================= 核心修复逻辑 =================
# 1. 强行给 config 注入可能缺失的属性 (AttributeError 修复补丁)
if not hasattr(policy.config, "n_action_steps"):
    print("Warning: 'n_action_steps' not found in config. Injecting default...")
    policy.config.n_action_steps = 100  # 默认值，稍后会被覆盖

if not hasattr(policy.config, "temporal_ensemble_coeff"):
    print("Warning: 'temporal_ensemble_coeff' not found in config. Injecting default...")
    policy.config.temporal_ensemble_coeff = None

# 2. 修改配置值
policy.config.n_action_steps = target_n_action_steps
policy.config.temporal_ensemble_coeff = alpha

# 3. 手动初始化时间集成器 (关键步骤)
# 因为策略已经初始化过了，修改 config 不会自动创建 temporal_ensembler，必须手动创建
if alpha is not None:
    # chunk_size 通常是 100，如果 config 里也没有，也需要注入
    if not hasattr(policy.config, "chunk_size"):
        policy.config.chunk_size = 100 
        
    print(f"Initializing Temporal Ensembler with chunk_size={policy.config.chunk_size}")
    policy.temporal_ensembler = ACTTemporalEnsembler(alpha, policy.config.chunk_size)
else:
    # 如果不使用平滑，确保属性存在（设为 None 或直接删除）
    policy.temporal_ensembler = None

# 4. 重置状态
policy.reset()
print("Policy re-configured successfully.")

# ================= 机器人连接 =================
robot_config = DMFollowerConfig(
    port="/dev/ttyACM0",
)
robot = DMFollower(robot_config)
robot.connect()

# ================= 观测预处理 =================
def prepare_observation(obs):
    if isinstance(obs, dict):
        return {k: prepare_observation(v) for k, v in obs.items()}
    if isinstance(obs, (int, float, list)):
        return torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0).to(device)
    if isinstance(obs, torch.Tensor):
        return obs.unsqueeze(0).to(device)
    return obs

# ================= 推理循环 =================
try:
    print("Start inference loop...")
    while True:
        # 1. 获取观测
        obs = robot.get_observation()
        obs = prepare_observation(obs)

        # 2. 策略推理
        with torch.no_grad():
            # 现在 policy.config 有了所需属性，select_action 里的 if 判断就能正常工作了
            action = policy.select_action(obs)

        # 3. 发送动作
        action_numpy = action.squeeze(0).cpu().numpy()
        robot.send_action(action_numpy)

except KeyboardInterrupt:
    print("\nStopping inference...")
except Exception as e:
    print(f"An error occurred: {e}")
    import traceback
    traceback.print_exc()
finally:
    robot.disconnect()
    print("Robot disconnected safely.")