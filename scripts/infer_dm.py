import torch
from lerobot.policies.pretrained import PreTrainedPolicy
from lerobot_robot_multi_robots.dm_arm import DMFollower, DMFollowerConfig

# ========== 配 置 区 域 ==========
# 加载预训练模型
policy = PreTrainedPolicy.from_pretrained(
    "./outputs/train/act_dk1_test_v3/checkpoints/last/pretrained_model"
)

# 机械臂配置
robot_config = DMFollowerConfig(
    port="/dev/ttyACM0",
)

# 卡尔曼滤波器参数
action_dim = 6  # 动作维度
process_std = 0.05  # 过程噪声标准差（用于提高灵敏度）
measurement_std = 0.2  # 测量噪声标准差（用于提高平滑度）
# ================================

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class KalmanFilter:
    def __init__(self, action_dim, device, process_std=0.05, measurement_std=0.2):
        """
        初始化卡尔曼滤波器
        :param action_dim: 动作维度
        :param device: 计算设备
        :param process_std: 过程噪声标准差
        :param measurement_std: 测量噪声标准差
        """
        self.dim = action_dim
        self.device = device

        # 状态向量 x: [位置，速度]
        self.x = torch.zeros((2 * action_dim, 1), device=device)

        # 状态转移矩阵 F: x_t = x_{t-1} + v_{t-1}
        # [I, I]
        # [0, I]
        self.F = torch.eye(2 * action_dim, device=device)
        for i in range(action_dim):
            self.F[i, i + action_dim] = 1.0

        # 观测矩阵 H: 只观测位置
        # [I, 0]
        self.H = torch.zeros((action_dim, 2 * action_dim), device=device)
        for i in range(action_dim):
            self.H[i, i] = 1.0

        # 协方差矩阵 P: 初始为单位矩阵
        self.P = torch.eye(2 * action_dim, device=device)

        # 噪声矩阵 Q 和 R
        self.Q = torch.eye(2 * action_dim, device=device) * (process_std**2)
        self.R = torch.eye(action_dim, device=device) * (measurement_std**2)

        self.is_initialized = False

    def update(self, measurement):
        """
        使用测量值更新滤波器状态
        :param measurement: 测量值
        :return: 更新后的状态估计
        """
        z = measurement.unsqueeze(1)

        if not self.is_initialized:
            self.x[: self.dim] = z
            self.is_initialized = True
            return measurement

        # 预测步骤
        # x' = F * x
        x_pred = self.F @ self.x
        # P' = F * P * F_T + Q
        p_pred = self.F @ self.P @ self.F.T + self.Q

        # 更新
        # 卡尔曼增益 K = P' * H_T * inv(H * P' * H_T + R)
        S = self.H @ p_pred @ self.H.T + self.R
        K = p_pred @ self.H.T @ torch.inverse(S)

        # 修正状态 x = x' + K * (z - H * x')
        self.x = x_pred + K @ (z - self.H @ x_pred)

        # 更新协方差 P = (I - K * H) * P'
        I = torch.eye(2 * self.dim, device=self.device)
        self.P = (I - K @ self.H) @ p_pred

        return self.x[: self.dim].squeeze(1)


def prepare_observation(obs):
    """
    观测预处理函数
    该函数将观测数据转换为适合模型输入的格式
    :param obs: 原始观测数据
    :return: 处理后的观测数据
    """
    if isinstance(obs, dict):
        return {k: prepare_observation(v) for k, v in obs.items()}
    if isinstance(obs, (int, float, list)):
        return torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0).to(device)
    if isinstance(obs, torch.Tensor):
        return obs.unsqueeze(0).to(device)
    return obs


def main():
    policy.to(device)
    policy.eval()

    robot = DMFollower(robot_config)
    robot.connect()

    kf = KalmanFilter(
        action_dim=action_dim,
        device=device,
        process_std=process_std,
        measurement_std=measurement_std,
    )

    try:
        while True:
            obs = robot.get_observation()
            obs = prepare_observation(obs)

            with torch.no_grad():
                action_chunk = policy.select_action(obs)

            raw_action = action_chunk.squeeze(0)[0]

            current_action = kf.update(raw_action)

            robot.send_action(current_action.cpu().numpy())

    except KeyboardInterrupt:
        print("\nStopping inference...")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        robot.disconnect()
        print("Robot disconnected safely.")


if __name__ == "__main__":
    main()
