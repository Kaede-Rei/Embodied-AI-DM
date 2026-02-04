#!/usr/bin/env python3
"""
DM 机械臂实时策略推理与执行脚本（LeRobot + ACTPolicy + Kalman Filter）

功能：
1. 加载训练好的 ACTPolicy 预训练模型（from_pretrained）
2. 实时采集机械臂状态 + 相机图像
3. 预处理为策略网络输入格式（state + images）
4. 执行前向推理得到关节动作
5. 可选卡尔曼滤波平滑动作（降低抖动/尖峰）
6. 将动作发送至机械臂闭环控制
7. 退出时自动平滑归零（安全停止，避免急停）

系统依赖：
- Ubuntu22.04
- 串口权限（/dev/ttyACM*）
- CUDA（可选，用于 GPU 推理）

Python 依赖：
- torch
- numpy
- lerobot
- opencv-python
- DMFollower 相关驱动包

模型目录结构示例：
outputs/
└── xxx_model/
    └── checkpoints/
        └── last/
            └── pretrained_model/

使用示例：
1. 默认运行（GPU 自动检测）：
   python dm_infer.py
2. 指定模型路径：
   python dm_infer.py --model_path ./outputs/my_model/checkpoints/last/pretrained_model
3. 开启混合精度（推荐 GPU）：
   python dm_infer.py --use_amp
4. 调整推理频率：
   python dm_infer.py --freq 60
5. 调整卡尔曼滤波强度：
   python dm_infer.py --kalman_process_noise 0.001 --kalman_measurement_noise 0.1
6. 禁用滤波（原始输出）：
   python dm_infer.py --no_kalman
7. 退出不归零（调试用）：
   python dm_infer.py --no_reset

参数说明：
--model_path                模型路径
--device                    cuda / cpu
--freq                      推理频率 Hz
--use_amp                   混合精度推理
--joint_velocity_scaling    关节速度缩放比例
--kalman_process_noise      过程噪声 Q
--kalman_measurement_noise  观测噪声 R
--no_kalman                 禁用滤波
--reset_time                归零时间
--no_reset                  退出不归零（危险）

注意事项：
- 建议开启 KalmanFilter 或低通滤波，否则模型输出可能抖动
- 不要在高负载 CPU 下运行高频控制（会导致丢帧/卡顿）
- 真机测试前先降低 joint_velocity_scaling
- 退出时请保持机械臂工作空间安全
"""

import argparse
import time
from pathlib import Path
from contextlib import nullcontext

import numpy as np
import torch

# LeRobot imports
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.cameras.opencv import OpenCVCameraConfig

# Local imports
from lerobot_robot_multi_robots.dm_arm import DMFollower
from lerobot_robot_multi_robots.config_dm_arm import DMFollowerConfig

# ! ========================= 配 置 区 ========================= ! #

DEFAULT_MODEL_PATH = (
    "./outputs/dk1_test_v2_model/checkpoints/last/pretrained_model"  # 默认模型路径
)
ARM_PORT = "/dev/ttyACM0"  # 机械臂串口
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"  # 使用 CUDA
INFER_FREQ = 30.0  # 推理频率
JOINT_VEL_SCALING = 0.1  # 关节速度缩放(0-1)
RESET_TIME = 2.0  # 归零时间(秒)

# 相机配置
CAMERAS_CONFIG = {
    # "end": OpenCVCameraConfig(
    #     index_or_path="/dev/com-1.2-video",
    #     width=640,
    #     height=480,
    #     fps=30,
    # ),
    # "eye": OpenCVCameraConfig(
    #     index_or_path=4,
    #     width=1280,
    #     height=720,
    #     fps=30,
    # ),
    "context": OpenCVCameraConfig(
        index_or_path=4,
        width=1280,
        height=720,
        fps=30,
    ),
}

# 关节名称列表
JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "gripper",
]


# ! ========================= 工 具 区 ========================= ! #


class KalmanFilter:
    """
    多维卡尔曼滤波器，用于平滑机械臂动作

    状态向量: [position, velocity] for each joint
    观测向量: [position] for each joint
    """

    def __init__(
        self, n_dims: int, process_noise: float = 0.01, measurement_noise: float = 0.1
    ):
        """
        Args:
            n_dims: 维度数（关节数）
            process_noise: 过程噪声（Q），越小越平滑但响应越慢
            measurement_noise: 观测噪声（R），越大越平滑
        """
        self.n_dims = n_dims

        # 状态向量 [pos1, vel1, pos2, vel2, ...]
        self.state_dim = n_dims * 2
        self.x = np.zeros(self.state_dim)  # 状态估计
        self.P = np.eye(self.state_dim) * 1.0  # 状态协方差

        # 状态转移矩阵 (假设 dt=1，后续会动态调整)
        self.F = np.eye(self.state_dim)
        for i in range(n_dims):
            self.F[i * 2, i * 2 + 1] = 1.0  # pos += vel * dt

        # 观测矩阵 (只观测位置)
        self.H = np.zeros((n_dims, self.state_dim))
        for i in range(n_dims):
            self.H[i, i * 2] = 1.0

        # 过程噪声协方差
        self.Q = np.eye(self.state_dim) * process_noise
        # 位置的过程噪声小，速度的过程噪声大
        for i in range(n_dims):
            self.Q[i * 2, i * 2] = process_noise * 0.1  # 位置
            self.Q[i * 2 + 1, i * 2 + 1] = process_noise  # 速度

        # 观测噪声协方差
        self.R = np.eye(n_dims) * measurement_noise

        self.initialized = False

    def reset(self, initial_pos: np.ndarray):
        """用初始位置重置滤波器"""
        self.x = np.zeros(self.state_dim)
        for i in range(self.n_dims):
            self.x[i * 2] = initial_pos[i]
            self.x[i * 2 + 1] = 0.0  # 初始速度为0
        self.P = np.eye(self.state_dim) * 0.1
        self.initialized = True

    def predict(self, dt: float = 1.0):
        """预测步骤"""
        # 更新状态转移矩阵中的 dt
        F = self.F.copy()
        for i in range(self.n_dims):
            F[i * 2, i * 2 + 1] = dt

        # 预测
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z: np.ndarray) -> np.ndarray:
        """
        更新步骤

        Args:
            z: 观测值 (n_dims,)

        Returns:
            滤波后的位置估计 (n_dims,)
        """
        if not self.initialized:
            self.reset(z)
            return z.copy()

        # 卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # 更新
        y = z - self.H @ self.x  # 残差
        self.x = self.x + K @ y
        self.P = (np.eye(self.state_dim) - K @ self.H) @ self.P

        # 提取位置
        pos = np.array([self.x[i * 2] for i in range(self.n_dims)])
        return pos

    def filter(self, z: np.ndarray, dt: float = 1.0) -> np.ndarray:
        """预测 + 更新"""
        self.predict(dt)
        return self.update(z)


def smooth_reset(
    robot: DMFollower,
    control_freq: float = 200.0,
    smooth_time: float = 2.0,
    max_step: float = 0.05,
    timeout: float = 30.0,
    tolerance: float = 0.02,
    open_gripper: bool = True,
):
    """
    平滑将机械臂归零

    Args:
        robot: 机械臂实例
        control_freq: 控制频率 Hz
        smooth_time: 期望归零时间（秒）
        max_step: 单步最大变化量（rad）
        timeout: 超时时间（秒）
        tolerance: 位置容差（rad）
        open_gripper: 是否打开夹爪
    """
    print("\n" + "=" * 50)
    print("开始平滑归零...")
    print("=" * 50)

    joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    try:
        current_obs = robot.get_observation()
    except Exception as e:
        print(f"无法读取机械臂状态: {e}")
        return

    # 记录起始位置
    start_positions = {j: current_obs[f"{j}.pos"] for j in joints}
    target_positions = {j: 0.0 for j in joints}

    print("当前关节位置:")
    for j in joints:
        q = start_positions[j]
        print(f"  {j}: {q:+.3f} rad ({q * 180 / np.pi:+.1f}°)")

    gripper_target = 0.0 if open_gripper else current_obs["gripper.pos"]

    control_period = 1.0 / control_freq
    start_time = time.time()
    last_print = start_time

    while True:
        loop_t0 = time.time()
        elapsed = loop_t0 - start_time

        try:
            current_obs = robot.get_observation()
        except:
            break

        # 时间插值系数
        alpha = min(elapsed / smooth_time, 1.0)

        action = {}
        max_error = 0.0
        all_reached = True

        for j in joints:
            q_start = start_positions[j]
            q_target = target_positions[j]

            # 插值参考位置
            q_ref = (1 - alpha) * q_start + alpha * q_target

            # 当前位置
            q_now = current_obs[f"{j}.pos"]

            # 单步限幅
            dq = q_ref - q_now
            if abs(dq) > max_step:
                q_cmd = q_now + np.sign(dq) * max_step
            else:
                q_cmd = q_ref

            action[f"{j}.pos"] = q_cmd

            # 检查是否到达目标
            err = abs(q_now - q_target)
            max_error = max(max_error, err)
            if err > tolerance:
                all_reached = False

        action["gripper.pos"] = gripper_target

        try:
            robot.send_action(action)
        except:
            break

        # 每秒打印一次
        if loop_t0 - last_print >= 1.0:
            print(f"  [{elapsed:5.1f}s] 最大误差: {max_error:.4f} rad")
            last_print = loop_t0

        if all_reached:
            print(f"\n归零完成，用时 {elapsed:.2f} 秒")
            break

        if elapsed > timeout:
            print("\n归零超时")
            break

        # 控制频率
        sleep_t = control_period - (time.time() - loop_t0)
        if sleep_t > 0:
            time.sleep(sleep_t)

    # 打印最终位置
    try:
        final_obs = robot.get_observation()
        print("\n最终关节位置:")
        for j in joints:
            q = final_obs[f"{j}.pos"]
            print(f"  {j}: {q:+.3f} rad")
    except:
        pass


def parse_args():
    parser = argparse.ArgumentParser(description="DM 机械臂纯推理脚本")
    parser.add_argument(
        "--model_path",
        type=str,
        default=DEFAULT_MODEL_PATH,
        help="预训练模型路径",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=ARM_PORT,
        help="机械臂串口",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=DEVICE,
        help="推理设备 (cuda/cpu)",
    )
    parser.add_argument(
        "--freq",
        type=float,
        default=INFER_FREQ,
        help="推理频率 Hz",
    )
    parser.add_argument(
        "--use_amp",
        action="store_true",
        help="使用混合精度推理",
    )
    parser.add_argument(
        "--joint_velocity_scaling",
        type=float,
        default=JOINT_VEL_SCALING,
        help="关节速度缩放 (0-1)",
    )
    # 卡尔曼滤波参数
    parser.add_argument(
        "--kalman_process_noise",
        type=float,
        default=0.001,
        help="卡尔曼滤波过程噪声 (越小越平滑但响应慢)",
    )
    parser.add_argument(
        "--kalman_measurement_noise",
        type=float,
        default=0.1,
        help="卡尔曼滤波观测噪声 (越大越平滑)",
    )
    parser.add_argument(
        "--no_kalman",
        action="store_true",
        help="禁用卡尔曼滤波",
    )
    # 归零参数
    parser.add_argument(
        "--reset_time",
        type=float,
        default=RESET_TIME,
        help="归零期望时间 (秒)",
    )
    parser.add_argument(
        "--no_reset",
        action="store_true",
        help="退出时不归零",
    )
    return parser.parse_args()


def preprocess_observation(obs_dict: dict, device: torch.device) -> dict:
    """
    将机械臂观测转换为模型输入格式

    Args:
        obs_dict: 来自机械臂的原始观测
        device: 目标设备

    Returns:
        处理后的观测字典
    """
    processed = {}

    # 处理关节状态 (7个关节: joint_1-6 + gripper)
    state_keys = [
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_4.pos",
        "joint_5.pos",
        "joint_6.pos",
        "gripper.pos",
    ]
    state = np.array([obs_dict[key] for key in state_keys], dtype=np.float32)

    # 添加 batch 维度: (7,) -> (1, 7)
    processed["observation.state"] = torch.from_numpy(state).unsqueeze(0).to(device)

    # 处理图像 (HWC uint8 -> CHW float32, normalized)
    for cam_name in CAMERAS_CONFIG.keys():
        if cam_name in obs_dict:
            img = obs_dict[cam_name]  # (H, W, C) uint8

            # 转换为 (C, H, W) float32
            img = np.transpose(img, (2, 0, 1)).astype(np.float32) / 255.0

            # 添加 batch 维度: (C, H, W) -> (1, C, H, W)
            processed[f"observation.images.{cam_name}"] = (
                torch.from_numpy(img).unsqueeze(0).to(device)
            )

    return processed


def postprocess_action(action: torch.Tensor) -> dict:
    """
    将模型输出转换为机械臂动作格式

    Args:
        action: 模型输出的动作张量 (batch, action_dim)

    Returns:
        机械臂动作字典
    """
    # 移除 batch 维度并转为 numpy
    action_np = action.squeeze(0).cpu().numpy()

    action_dict = {
        "joint_1.pos": float(action_np[0]),
        "joint_2.pos": float(action_np[1]),
        "joint_3.pos": float(action_np[2]),
        "joint_4.pos": float(action_np[3]),
        "joint_5.pos": float(action_np[4]),
        "joint_6.pos": float(action_np[5]),
        "gripper.pos": float(action_np[6]),
    }

    return action_dict


# ! ========================= 逻 辑 区 ========================= ! #


def main():
    args = parse_args()

    # 设置设备
    device = torch.device(
        args.device if torch.cuda.is_available() or args.device == "cpu" else "cpu"
    )
    print(f"使用设备: {device}")

    # 设置高性能模式
    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True

    # 加载模型
    print(f"加载模型: {args.model_path}")
    model_path = Path(args.model_path)

    if not model_path.exists():
        raise FileNotFoundError(f"模型路径不存在: {model_path}")

    # 使用 ACTPolicy.from_pretrained 加载模型
    policy = ACTPolicy.from_pretrained(str(model_path))
    policy.to(device)
    policy.eval()
    print(f"模型加载成功: {type(policy).__name__}")
    print(
        f"模型配置: chunk_size={policy.config.chunk_size}, n_action_steps={policy.config.n_action_steps}"
    )

    # 初始化卡尔曼滤波器
    kalman_filter = None
    if not args.no_kalman:
        kalman_filter = KalmanFilter(
            n_dims=7,  # 6 joints + gripper
            process_noise=args.kalman_process_noise,
            measurement_noise=args.kalman_measurement_noise,
        )
        print(
            f"卡尔曼滤波已启用 (Q={args.kalman_process_noise}, R={args.kalman_measurement_noise})"
        )
    else:
        print("卡尔曼滤波已禁用")

    # 初始化机械臂
    print("初始化机械臂...")
    robot_config = DMFollowerConfig(
        port=args.port,
        cameras=CAMERAS_CONFIG,
        joint_velocity_scaling=args.joint_velocity_scaling,
        disable_torque_on_disconnect=True,
    )
    robot = DMFollower(robot_config)
    robot_connected = False

    try:
        robot.connect()
        robot_connected = True
        print("机械臂连接成功")

        # 推理循环
        period = 1.0 / args.freq
        print(f"开始推理循环，频率: {args.freq} Hz")
        print("按 Ctrl+C 停止...")

        step = 0
        last_time = time.perf_counter()
        amp_context = (
            torch.autocast(device_type=device.type) if args.use_amp else nullcontext()
        )

        while True:
            loop_start = time.perf_counter()
            dt = loop_start - last_time
            last_time = loop_start

            # 读取观测
            obs_start = time.perf_counter()
            raw_obs = robot.get_observation()
            obs_time = (time.perf_counter() - obs_start) * 1000

            # 预处理观测
            proc_start = time.perf_counter()
            obs = preprocess_observation(raw_obs, device)
            proc_time = (time.perf_counter() - proc_start) * 1000

            # 模型推理
            infer_start = time.perf_counter()
            with torch.inference_mode(), amp_context:
                action = policy.select_action(obs)
            infer_time = (time.perf_counter() - infer_start) * 1000

            # 后处理动作
            action_dict = postprocess_action(action)

            # 卡尔曼滤波平滑
            filter_start = time.perf_counter()
            if kalman_filter is not None:
                # 提取动作为数组
                action_array = np.array(
                    [
                        action_dict["joint_1.pos"],
                        action_dict["joint_2.pos"],
                        action_dict["joint_3.pos"],
                        action_dict["joint_4.pos"],
                        action_dict["joint_5.pos"],
                        action_dict["joint_6.pos"],
                        action_dict["gripper.pos"],
                    ]
                )

                # 滤波
                filtered_array = kalman_filter.filter(
                    action_array, dt=dt if dt > 0 else period
                )

                # 更新动作字典
                action_dict = {
                    "joint_1.pos": float(filtered_array[0]),
                    "joint_2.pos": float(filtered_array[1]),
                    "joint_3.pos": float(filtered_array[2]),
                    "joint_4.pos": float(filtered_array[3]),
                    "joint_5.pos": float(filtered_array[4]),
                    "joint_6.pos": float(filtered_array[5]),
                    "gripper.pos": float(filtered_array[6]),
                }
            filter_time = (time.perf_counter() - filter_start) * 1000

            # 发送动作
            send_start = time.perf_counter()
            robot.send_action(action_dict)
            send_time = (time.perf_counter() - send_start) * 1000

            # 统计
            total_time = (time.perf_counter() - loop_start) * 1000

            if step % 30 == 0:  # 每秒打印一次
                print(
                    f"[Step {step:5d}] "
                    f"obs: {obs_time:5.1f}ms | "
                    f"proc: {proc_time:5.1f}ms | "
                    f"infer: {infer_time:5.1f}ms | "
                    f"filter: {filter_time:5.1f}ms | "
                    f"send: {send_time:5.1f}ms | "
                    f"total: {total_time:5.1f}ms | "
                    f"gripper: {action_dict['gripper.pos']:.3f}"
                )

            # 控制循环频率
            elapsed = time.perf_counter() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

            step += 1

    except KeyboardInterrupt:
        print("\n用户中断，准备安全停止...")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # 平滑归零
        if robot_connected and not args.no_reset:
            smooth_reset(
                robot,
                control_freq=200.0,
                smooth_time=args.reset_time,
                max_step=0.05,
                timeout=30.0,
                tolerance=0.02,
                open_gripper=True,
            )

        # 断开连接
        print("\n断开机械臂连接...")
        try:
            robot.disconnect()
        except:
            pass


if __name__ == "__main__":
    main()
