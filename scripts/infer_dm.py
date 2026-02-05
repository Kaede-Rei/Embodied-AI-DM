#!/usr/bin/env python3
"""
DM 机械臂实时策略推理与执行脚本（LeRobot + ACTPolicy + Low-Pass Filter）

功能：
1. 加载训练好的 ACTPolicy 预训练模型（from_pretrained）
2. 实时采集机械臂状态 + 相机图像
3. 预处理为策略网络输入格式（state + images）
4. 执行前向推理得到关节动作
5. 低通滤波平滑动作 (降低抖动/尖峰)
6. 安全检查并发送至机械臂闭环控制
7. 退出时自动平滑归零 (安全停止,避免急停)

系统依赖:
- Ubuntu 22.04
- 串口权限 (/dev/ttyACM*) - 需要 sudo adduser $USER dialout
- CUDA 11.8+ (可选,用于 GPU 推理)

Python 依赖:
- torch >= 2.0
- numpy
- lerobot
- opencv-python
- DMFollower 相关驱动包

模型目录结构:
outputs/
└── xxx_model/
    └── checkpoints/
        └── last/
            └── pretrained_model/
                ├── config.json
                ├── model.safetensors
                └── train_config.json

使用示例:
1. 默认运行 (GPU 自动检测):
   python infer_dm.py

2. 指定模型路径:
   python infer_dm.py --model_path ./outputs/my_model/checkpoints/last/pretrained_model

3. 开启混合精度 (推荐 RTX GPU):
   python infer_dm.py --use_amp

4. 调整推理频率:
   python infer_dm.py --freq 60

5. 调整滤波强度:
   python infer_dm.py --filter_tau 0.3

6. 禁用滤波 (原始输出，不推荐):
   python infer_dm.py --no_filter

7. 退出不归零 (调试用，危险):
   python infer_dm.py --no_reset

参数说明:
--model_path              模型路径
--port                    串口路径
--device                  cuda / cpu
--freq                    控制频率 Hz (建议 30)
--use_amp                 混合精度推理
--joint_velocity_scaling  关节速度缩放 (0-1)
--filter_tau              低通滤波时间常数 (秒,越大越平滑)
--no_filter               禁用滤波
--use_async_obs           异步观测采集
--obs_freq                观测采集频率 Hz
--max_velocity            最大关节速度 (rad/s)
--max_change              单步最大变化 (rad)
--reset_time              归零时间 (秒)
--no_reset                退出不归零 (危险)
--compile                 使用 torch.compile

注意事项:
    真机测试前请先降低 joint_velocity_scaling (建议 0.05-0.1)
    不要在高负载 CPU 下运行高频控制 (会导致丢帧/卡顿)
    退出时请保持机械臂工作空间安全,避免碰撞
    建议开启滤波,否则模型输出可能抖动
    首次运行请确保机械臂处于安全位置

日志前缀说明:
[INFO] - 一般信息
[WARN] - 安全警告 (已自动处理)
[WARN!!!!!] - 严重安全问题 (即将停止)
[ERROR] - 运行错误
[RECOVER] - 错误恢复
"""

import argparse
import time
import queue
import threading
import signal
from pathlib import Path
from contextlib import nullcontext
from typing import Dict, Optional, Tuple
from collections import deque
from dataclasses import dataclass

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
    "./outputs/leaf_v0_model/checkpoints/last/pretrained_model"  # 默认模型路径
)
ARM_PORT = "/dev/ttyACM0"  # 机械臂串口
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"  # 使用 CUDA
INFER_FREQ = 30.0  # 推理频率
JOINT_VEL_SCALING = 0.05  # 关节速度缩放(0-1)
RESET_TIME = 2.0  # 归零时间(秒)
MAX_JOINT_VEL = 1.57  # 最大关节速度(rad/s)
MAX_ACTION_CHANGE = 0.1  # 单步最大变化(rad)
MAX_VIOLATIONS = 50  # 最大违规次数
WATCHDOG_TIMEOUT = 1.0  # 看门狗超时时间(秒)

# 相机配置
CAMERAS_CONFIG = {
    "end": OpenCVCameraConfig(
        index_or_path="/dev/com-1.2-video",
        width=640,
        height=480,
        fps=30,
    ),
    "eye": OpenCVCameraConfig(
        index_or_path=4,
        width=1280,
        height=720,
        fps=30,
    ),
    # "context": OpenCVCameraConfig(
    #     index_or_path=4,
    #     width=1280,
    #     height=720,
    #     fps=30,
    # ),
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

# 安全限制
JOINT_LIMITS = {
    "joint_1": (-2.094, 2.094),
    "joint_2": (0.0, 3.142),
    "joint_3": (0.0, 4.712),
    "joint_4": (-1.571, 1.571),
    "joint_5": (-1.571, 1.571),
    "joint_6": (-3.142, 3.142),
    "gripper": (-5.23, 0.0),
}

# ! ========================= 数 据 结 构 ========================= ! #


@dataclass
class PerformanceStats:
    """
    性能统计数据类

    记录控制循环各个阶段的耗时,用于性能分析和优化

    Attributes:
        obs_time: 观测采集耗时 (ms)
        proc_time: 观测预处理耗时 (ms)
        infer_time: 模型推理耗时 (ms)
        filter_time: 动作滤波耗时 (ms)
        send_time: 动作发送耗时 (ms)
        total_time: 总耗时 (ms)
        queue_size: 观测队列大小
    """

    obs_time: float = 0.0
    proc_time: float = 0.0
    infer_time: float = 0.0
    filter_time: float = 0.0
    send_time: float = 0.0
    total_time: float = 0.0
    queue_size: int = 0

    def __str__(self) -> str:
        """格式化输出性能统计"""
        return (
            f"obs:{self.obs_time:5.1f}ms | "
            f"proc:{self.proc_time:5.1f}ms | "
            f"infer:{self.infer_time:5.1f}ms | "
            f"filter:{self.filter_time:5.1f}ms | "
            f"send:{self.send_time:5.1f}ms | "
            f"total:{self.total_time:5.1f}ms | "
            f"queue:{self.queue_size}"
        )


# ! ========================= 工 具 区 ========================= ! #


class LowPassFilter:
    """
    一阶低通滤波器 (指数移动平均)

    用于平滑机械臂动作,减少模型输出的抖动和尖峰

    公式: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    其中 alpha = dt / (dt + tau)
    tau 是时间常数,越大越平滑,响应越慢

    Attributes:
        n_dims: 滤波维度数 (关节数)
        tau: 时间常数 (秒)
        y: 当前滤波输出值
        initialized: 是否已初始化
    """

    def __init__(
        self, n_dims: int, tau: float = 0.1, cutoff_freq: Optional[float] = None
    ):
        """
        初始化低通滤波器

        Args:
            n_dims: 维度数 (关节数)
            tau: 时间常数 (秒),值越大越平滑，tau=0.1 表示 0.1秒达到 63% 响应
            cutoff_freq: 截止频率 (Hz),可替代 tau，计算为 tau = 1.0 / (2.0 * np.pi * cutoff_freq)
        """
        self.n_dims = n_dims

        if cutoff_freq is not None:
            self.tau = 1.0 / (2.0 * np.pi * cutoff_freq)
        else:
            self.tau = tau

        self.y = None
        self.initialized = False

    def reset(self, initial_value: np.ndarray):
        """
        重置滤波器

        Args:
            initial_value: 初始值(n_dims, )
        """
        self.y = initial_value.copy()
        self.initialized = True

    def filter(self, x: np.ndarray, dt: float = 0.033) -> np.ndarray:
        """
        执行滤波

        Args:
            x: 输入值(n_dims, )
            dt: 时间间隔(秒)

        Returns:
            滤波后的值(n_dims, )
        """
        if not self.initialized:
            self.reset(x)
            return x.copy()

        # 计算 alpha
        alpha = dt / (dt + self.tau)
        alpha = np.clip(alpha, 0.0, 1.0)

        # 指数平滑
        self.y = alpha * x + (1.0 - alpha) * self.y

        return self.y.copy()


class SafetyChecker:
    """
    机械臂安全检查器

    负责检查和限幅机械臂动作,防止过速、超限位、单步变化过大等危险情况

    Attributes:
        max_vel: 最大关节速度 (rad/s)
        max_change: 单步最大变化 (rad)
        last_act: 上一次动作
        last_time: 上一次动作时间
        violation_count: 累计违规次数
        max_violations: 最大允许违规次数
    """

    def __init__(
        self,
        max_vel: float = MAX_JOINT_VEL,
        max_change: float = MAX_ACTION_CHANGE,
        max_violations: int = MAX_VIOLATIONS,
    ):
        """
        初始化安全检查器

        Args:
            max_vel: 最大关节速度 (rad/s)
            max_change: 单步最大变化 (rad)
            max_violations: 最大允许违规次数
        """
        self.max_vel = max_vel
        self.max_change = max_change
        self.last_act = None
        self.last_time = None
        self.violation_count = 0
        self.max_violations = max_violations

    def check_limits(self, act_dict: Dict[str, float]) -> Tuple[bool, str]:
        """
        检查关节限位

        Args:
            act_dict: 动作字典 {joint_name.pos: value}

        Returns:
            (是否通过检查, 错误信息)
        """
        for joint, (min_val, max_val) in JOINT_LIMITS.items():
            key = f"{joint}.pos"
            if key in act_dict:
                val = act_dict[key]
                if val < min_val or val > max_val:
                    return (
                        False,
                        f"{joint} 超出限位范围: {val:.3f} 不在 [{min_val:.3f}, {max_val:.3f}]",
                    )
        return True, ""

    def check_velocity(self, act_dict: Dict[str, float], dt: float) -> Tuple[bool, str]:
        """
        检查关节速度

        Args:
            act_dict: 动作字典 {joint_name.pos: value}
            dt: 时间间隔 (秒)

        Returns:
            (是否通过检查, 错误信息)
        """
        if self.last_act is None or dt <= 0:
            return True, ""

        for joint in JOINT_NAMES:
            key = f"{joint}.pos"
            if key in act_dict and key in self.last_act:
                dq = abs(act_dict[key] - self.last_act[key])
                vel = dq / dt

                if vel > self.max_vel:
                    return False, f"{joint} 速度超限: {vel:.3f} > {self.max_vel:.3f}"

        return True, ""

    def check_change(self, act_dict: Dict[str, float]) -> Tuple[bool, str]:
        """
        检查单步变化

        Args:
            act_dict: 动作字典 {joint_name.pos: value}

        Returns:
            (是否通过检查, 错误信息)
        """
        if self.last_act is None:
            return True, ""

        for joint in JOINT_NAMES:
            key = f"{joint}.pos"
            if key in act_dict and key in self.last_act:
                dq = abs(act_dict[key] - self.last_act[key])

                if dq > self.max_change:
                    return (
                        False,
                        f"{joint} 单步变化超限: {dq:.3f} > {self.max_change:.3f}",
                    )

        return True, ""

    def clip_action(self, act_dict: Dict[str, float]) -> Dict[str, float]:
        """
        限幅动作 (安全核心功能)

        Args:
            act_dict: 动作字典 {joint_name.pos: value}

        Returns:
            限幅后的动作字典
        """
        clipped = {}

        for joint in JOINT_NAMES:
            key = f"{joint}.pos"
            if key in act_dict:
                value = act_dict[key]

                # 限位
                min_val, max_val = JOINT_LIMITS[joint]
                value = np.clip(value, min_val, max_val)

                # 单步限幅
                if self.last_act is not None and key in self.last_act:
                    last_val = self.last_act[key]
                    max_step = self.max_change
                    value = np.clip(value, last_val - max_step, last_val + max_step)

                clipped[key] = float(value)

        return clipped

    def validate(
        self, act_dict: Dict[str, float], dt: float
    ) -> Tuple[bool, Dict[str, float], str]:
        """
        验证并修正动作

        Args:
            act_dict: 动作字典 {joint_name.pos: value}
            dt: 时间间隔 (秒)

        Returns:
            (是否安全, 限幅后的动作, 错误信息)
        """
        is_violation = False
        err_msgs = []

        # 检查限位
        ok, msg = self.check_limits(act_dict)
        if not ok:
            is_violation = True
            self.violation_count += 1
            err_msgs.append(msg)

        # 检查速度
        ok, msg = self.check_velocity(act_dict, dt)
        if not ok:
            is_violation = True
            self.violation_count += 1
            err_msgs.append(msg)

        # 检查单步变化
        ok, msg = self.check_change(act_dict)
        if not ok:
            is_violation = True
            self.violation_count += 1
            err_msgs.append(msg)

        # 没有违规则清零违规计数
        if not is_violation:
            self.violation_count = 0

        # 限幅动作 (始终执行)
        clipped_act = self.clip_action(act_dict)

        # 更新历史
        self.last_act = clipped_act.copy()
        self.last_time = time.perf_counter()

        # 检查累计违规次数
        is_safe = self.violation_count < self.max_violations

        combined_msg = "; ".join(err_msgs) if err_msgs else ""
        return is_safe, clipped_act, combined_msg


class ObservationBuffer:
    """
    线程安全的观测缓冲区

    用于在异步观测采集模式下存储和获取最新观测

    Attributes:
        queue: 观测队列
        latest: 最新观测
        lock: 线程锁
    """

    def __init__(self, maxsize: int = 2):
        """
        初始化观测缓冲区

        Args:
            maxsize: 队列最大大小
        """
        self.queue = queue.Queue(maxsize=maxsize)
        self.latest = None
        self.lock = threading.Lock()

    def put(self, obs: dict):
        """
        添加观测 (非阻塞,丢弃旧数据)

        Args:
            obs: 观测字典
        """
        try:
            self.queue.put_nowait(obs)
            with self.lock:
                self.latest = obs
        except queue.Full:
            try:
                self.queue.get_nowait()
                self.queue.put_nowait(obs)
            except:
                pass

    def get(self, timeout: float = 0.1) -> Optional[dict]:
        """
        获取观测

        Args:
            timeout: 超时时间 (秒)

        Returns:
            观测字典或 None
        """
        try:
            return self.queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_latest(self) -> Optional[dict]:
        """
        获取最新观测 (无阻塞)

        Returns:
            最新观测字典或 None
        """
        with self.lock:
            return self.latest


class ThreadSafeRobot:
    """
    线程安全的机器人代理

    用于解决串口并发访问问题，确保观测采集和动作发送不会冲突

    Attributes:
        raw_robot: 原始机器人实例
        lock: 递归锁
    """

    def __init__(self, raw_robot: DMFollower):
        """
        初始化线程安全机器人代理

        Args:
            raw_robot: 原始机器人实例
        """
        self.raw_robot = raw_robot
        self.lock = threading.RLock()

    def connect(self):
        """连接机器人"""
        with self.lock:
            return self.raw_robot.connect()

    def disconnect(self):
        """断开机器人连接"""
        with self.lock:
            return self.raw_robot.disconnect()

    def get_observation(self):
        """获取观测 (线程安全)"""
        with self.lock:
            return self.raw_robot.get_observation()

    def send_action(self, action):
        """发送动作 (线程安全)"""
        with self.lock:
            return self.raw_robot.send_action(action)

    def __getattr__(self, name):
        """代理其他属性"""
        return getattr(self.raw_robot, name)


def smooth_reset(
    robot: ThreadSafeRobot,
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
        robot: 线程安全机械臂实例
        control_freq: 控制频率 Hz
        smooth_time: 期望归零时间（秒）
        max_step: 单步最大变化量（rad）
        timeout: 超时时间（秒）
        tolerance: 位置容差（rad）
        open_gripper: 是否打开夹爪
    """
    print("=========================================")
    print("              开始平滑归零...              ")
    print("=========================================")

    joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    try:
        current_obs = robot.get_observation()
    except Exception as e:
        print(f"[ERROR] 无法读取机械臂状态: {e}")
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
    parser = argparse.ArgumentParser(description="DM 机械臂推理脚本")

    # 基础参数
    parser.add_argument("--model_path", type=str, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--port", type=str, default=ARM_PORT)
    parser.add_argument("--device", type=str, default=DEVICE)
    parser.add_argument("--freq", type=float, default=INFER_FREQ, help="控制频率 Hz")
    parser.add_argument(
        "--use_amp", action="store_true", default=False, help="混合精度推理"
    )
    parser.add_argument(
        "--joint_velocity_scaling", type=float, default=JOINT_VEL_SCALING
    )

    # 滤波参数
    parser.add_argument(
        "--filter_tau",
        type=float,
        default=0.5,
        help="低通滤波时间常数（秒），越大越平滑",
    )
    parser.add_argument(
        "--no_filter", action="store_true", default=False, help="禁用滤波"
    )

    # 异步参数
    parser.add_argument(
        "--use_async_obs", action="store_true", default=True, help="异步观测采集"
    )
    parser.add_argument("--obs_freq", type=float, default=30.0, help="观测采集频率 Hz")

    # 安全参数
    parser.add_argument("--max_velocity", type=float, default=MAX_JOINT_VEL)
    parser.add_argument("--max_change", type=float, default=MAX_ACTION_CHANGE)

    # 其他
    parser.add_argument("--reset_time", type=float, default=RESET_TIME)
    parser.add_argument("--no_reset", action="store_true", default=False)
    parser.add_argument(
        "--compile", action="store_true", default=True, help="使用 torch.compile"
    )

    return parser.parse_args()


# ! ========================= 逻 辑 区 ========================= ! #


class AsyncObserver:
    """
    异步观测采集线程

    在后台线程中持续采集机械臂观测,避免阻塞主控制循环

    Attributes:
        robot: 线程安全机器人实例
        buffer: 观测缓冲区
        freq: 采集频率 Hz
        running: 是否运行中
        thread: 后台线程
        error: 错误信息
    """

    def __init__(
        self, robot: ThreadSafeRobot, buffer: ObservationBuffer, freq: float = 30.0
    ):
        """
        初始化异步观测采集器

        Args:
            robot: 线程安全机器人实例
            buffer: 观测缓冲区
            freq: 采集频率 Hz
        """
        self.robot = robot
        self.buffer = buffer
        self.freq = freq
        self.running = False
        self.thread = None
        self.error = None

    def start(self):
        """启动采集线程"""
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        """停止采集线程"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def run(self):
        """采集循环 (在后台线程中运行)"""
        period = 1.0 / self.freq

        while self.running:
            t0 = time.perf_counter()

            try:
                obs = self.robot.get_observation()
                self.buffer.put(obs)
            except Exception as e:
                self.error = e
                print(f"[ERROR] 观测采集失败: {e}")
                break

            elapsed = time.perf_counter() - t0
            if elapsed < period:
                time.sleep(period - elapsed)


class RobotController:
    """
    机器人控制器 (带安全保护和性能监控)

    整合观测采集、模型推理、动作滤波、安全检查和性能统计

    Attributes:
        policy: 策略网络
        robot: 线程安全机器人实例
        device: 计算设备
        use_amp: 是否使用混合精度
        filter: 低通滤波器
        safety_checker: 安全检查器
        use_async_obs: 是否使用异步观测
        obs_buffer: 观测缓冲区
        observer: 异步观测采集器
        stats: 性能统计
        stats_history: 性能历史
        last_act_time: 上次动作时间
        watchdog_triggered: 看门狗是否触发
        amp_context: 混合精度上下文
    """

    def __init__(
        self,
        policy: ACTPolicy,
        robot: ThreadSafeRobot,
        device: torch.device,
        use_amp: bool = False,
        filter_tau: float = 0.5,
        use_async_obs: bool = True,
        obs_freq: float = 30.0,
    ):
        """
        初始化机器人控制器

        Args:
            policy: 策略网络
            robot: 线程安全机器人实例
            device: 计算设备
            use_amp: 是否使用混合精度
            filter_tau: 滤波器时间常数 (秒)
            use_async_obs: 是否使用异步观测
            obs_freq: 观测采集频率 Hz
        """
        self.policy = policy
        self.robot = robot
        self.device = device
        self.use_amp = use_amp
        self.filter = LowPassFilter(n_dims=len(JOINT_NAMES), tau=filter_tau)
        self.safety_checker = SafetyChecker()
        self.use_async_obs = use_async_obs

        if use_async_obs:
            self.obs_buffer = ObservationBuffer(maxsize=2)
            self.observer = AsyncObserver(robot, self.obs_buffer, obs_freq)
            print(f"[INFO] 启用异步观测采集,频率: {obs_freq} Hz")

        self.stats = PerformanceStats()
        self.stats_history = deque(maxlen=100)
        self.last_act_time = time.perf_counter()
        self.watchdog_triggered = False
        self.amp_context = (
            torch.autocast(device_type=device.type, dtype=torch.float16)
            if use_amp
            else nullcontext()
        )

    def start(self):
        """启动控制器"""
        if self.use_async_obs:
            self.observer.start()
            time.sleep(0.5)  # 等待缓冲填充

    def stop(self):
        """停止控制器"""
        if self.use_async_obs:
            self.observer.stop()

    def get_observation(self) -> Optional[dict]:
        """
        获取观测

        Returns:
            观测字典或 None
        """
        t0 = time.perf_counter()

        if self.use_async_obs:
            obs = self.obs_buffer.get(timeout=0.1)
            if obs is None:
                # 回退到同步模式
                obs = self.robot.get_observation()
        else:
            obs = self.robot.get_observation()

        self.stats.obs_time = (time.perf_counter() - t0) * 1000
        return obs

    def preprocess_observation(self, obs_dict: dict) -> dict:
        """
        预处理观测

        Args:
            obs_dict: 原始观测字典

        Returns:
            处理后的观测字典
        """
        t0 = time.perf_counter()

        processed = {}

        # 状态
        state_keys = [f"{j}.pos" for j in JOINT_NAMES]
        state = np.array([obs_dict[key] for key in state_keys], dtype=np.float32)
        processed["observation.state"] = (
            torch.from_numpy(state).unsqueeze(0).to(self.device, non_blocking=True)
        )

        # 图像
        for cam_name in CAMERAS_CONFIG.keys():
            if cam_name in obs_dict:
                img = obs_dict[cam_name]
                img = np.transpose(img, (2, 0, 1)).astype(np.float32) / 255.0
                processed[f"observation.images.{cam_name}"] = (
                    torch.from_numpy(img)
                    .unsqueeze(0)
                    .to(self.device, non_blocking=True)
                )

        self.stats.proc_time = (time.perf_counter() - t0) * 1000
        return processed

    def infer_action(self, obs: dict) -> np.ndarray:
        """
        推理动作

        Args:
            obs: 处理后的观测字典

        Returns:
            动作数组 (n_joints,)
        """
        t0 = time.perf_counter()

        with torch.inference_mode(), self.amp_context:
            act = self.policy.select_action(obs)

        if self.device.type == "cuda":
            torch.cuda.synchronize()

        action_np = act.squeeze(0).cpu().numpy()

        self.stats.infer_time = (time.perf_counter() - t0) * 1000
        return action_np

    def filter_action(self, act: np.ndarray, dt: float) -> np.ndarray:
        """
        滤波动作

        Args:
            act: 原始动作数组 (n_joints,)
            dt: 时间间隔 (秒)

        Returns:
            滤波后的动作数组 (n_joints,)
        """
        t0 = time.perf_counter()

        filtered = self.filter.filter(act, dt=dt)

        self.stats.filter_time = (time.perf_counter() - t0) * 1000
        return filtered

    def send_action(self, act: np.ndarray, dt: float) -> Tuple[bool, str]:
        """
        发送动作 (带安全检查)

        Args:
            act: 动作数组 (n_joints,)
            dt: 时间间隔 (秒)

        Returns:
            (是否成功, 错误信息)
        """
        t0 = time.perf_counter()

        act_dict = {
            f"{joint}.pos": float(act[i]) for i, joint in enumerate(JOINT_NAMES)
        }

        is_safe, clipped_act, err_msg = self.safety_checker.validate(act_dict, dt)

        if not is_safe:
            msg = f"[WARN!!!!!] 持续检测到安全问题，已违规 {self.safety_checker.violation_count} 次"
            print(msg)
            return False, msg

        if err_msg:
            if self.safety_checker.violation_count % 10 == 1:
                print(f"[WARN] {err_msg}（已自动限幅，继续运行）")

        try:
            self.robot.send_action(clipped_act)
            self.last_act_time = time.perf_counter()
        except Exception as e:
            msg = f"[ERROR] 发送动作时发生异常: {e}"
            print(msg)
            return False, msg

        self.stats.send_time = (time.perf_counter() - t0) * 1000
        return True, ""

    def check_watchdog(self) -> bool:
        """
        检查看门狗

        Returns:
            是否正常
        """
        if self.watchdog_triggered:
            return False

        elapsed = time.perf_counter() - self.last_act_time
        if elapsed > WATCHDOG_TIMEOUT:
            print(f"[ERROR] 看门狗超时 ({elapsed:.2f}s)，触发停止")
            self.watchdog_triggered = True
            return False

        return True

    def step(self, dt: float) -> Tuple[bool, str]:
        """
        执行一步控制

        Args:
            dt: 时间间隔 (秒)

        Returns:
            (是否成功, 错误信息)
        """
        step_start = time.perf_counter()

        # 获取观测
        try:
            obs = self.get_observation()
            if obs is None:
                return False, "无法获取观测"
        except Exception as e:
            return False, f"观测采集失败: {e}"

        # 预处理
        try:
            processed_obs = self.preprocess_observation(obs)
        except Exception as e:
            return False, f"观测预处理失败: {e}"

        # 推理
        try:
            act = self.infer_action(processed_obs)
        except Exception as e:
            return False, f"模型推理失败: {e}"

        # 滤波
        try:
            filtered_act = self.filter_action(act, dt)
        except Exception as e:
            return False, f"动作滤波失败: {e}"

        # 发送
        success, err_msg = self.send_action(filtered_act, dt)

        # 统计
        self.stats.total_time = (time.perf_counter() - step_start) * 1000
        self.stats_history.append(self.stats.total_time)

        return success, err_msg

    def get_avg_latency(self) -> float:
        """
        获取平均延迟

        Returns:
            平均延迟 (ms)
        """
        if not self.stats_history:
            return 0.0
        return sum(self.stats_history) / len(self.stats_history)


def main():
    """主函数"""
    args = parse_args()

    # 信号处理
    shutdown_event = threading.Event()

    def signal_handler(sig, frame):
        print("\n[INFO] 收到中断信号...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 系统配置
    print("======================================")
    print("          DM 机械臂实时推理系统          ")
    print("======================================")

    print("\n【系统配置】")
    device = torch.device(
        args.device if torch.cuda.is_available() or args.device == "cpu" else "cpu"
    )
    print(f"  计算设备: {device}")

    torch.backends.cudnn.benchmark = True
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.set_float32_matmul_precision("high")

    if device.type == "cuda":
        torch.cuda.empty_cache()
        print(f"  GPU 型号: {torch.cuda.get_device_name(0)}")
        print(
            f"  显存大小: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB"
        )
        print(f"  CUDA 版本: {torch.version.cuda}")

    # 模型加载
    print("\n【模型加载】")
    print(f"  模型路径: {args.model_path}")
    model_path = Path(args.model_path)

    if not model_path.exists():
        raise FileNotFoundError(f"[ERROR] 模型路径不存在: {model_path}")

    policy = ACTPolicy.from_pretrained(str(model_path))
    policy.to(device)
    policy.eval()

    if args.compile:
        print("  编译模型: 启用 (首次运行会较慢)...")
        policy = torch.compile(policy, mode="reduce-overhead")
    else:
        print("  编译模型: 禁用")

    print(f"  模型配置: chunk_size={policy.config.chunk_size}")

    # 模型预热
    print("\n【模型预热】")
    warmup_start = time.perf_counter()

    try:
        dummy_obs = {}
        dummy_obs["observation.state"] = torch.zeros(1, 7).to(device)
        for cam_name in CAMERAS_CONFIG.keys():
            cam_config = CAMERAS_CONFIG[cam_name]
            dummy_obs[f"observation.images.{cam_name}"] = torch.zeros(
                1, 3, cam_config.height, cam_config.width
            ).to(device)

        amp_context = (
            torch.autocast(device_type=device.type, dtype=torch.float16)
            if args.use_amp
            else nullcontext()
        )

        for i in range(5):
            with torch.inference_mode(), amp_context:
                _ = policy.select_action(dummy_obs)
            if device.type == "cuda":
                torch.cuda.synchronize()

        warmup_time = time.perf_counter() - warmup_start
        print(f"  预热完成,耗时: {warmup_time:.2f}s")
        print("  推理速度已稳定")

    except Exception as e:
        print(f"[WARN] 预热失败: {e}")
        print("  继续运行,但首次推理可能较慢")

    # 机械臂初始化
    print("\n【机械臂初始化】")
    robot_config = DMFollowerConfig(
        port=args.port,
        cameras=CAMERAS_CONFIG,
        joint_velocity_scaling=args.joint_velocity_scaling,
        disable_torque_on_disconnect=True,
    )
    robot_raw = DMFollower(robot_config)
    robot = ThreadSafeRobot(robot_raw)
    robot_connected = False

    controller = None

    try:
        robot.connect()
        robot_connected = True
        print("  机械臂连接成功")

        # 控制器创建
        print("\n【控制器配置】")
        print(f"  控制频率: {args.freq} Hz")
        print(
            f"  滤波器: {'启用' if not args.no_filter else '禁用'} (tau={args.filter_tau if not args.no_filter else 0}s)"
        )
        print(f"  混合精度: {'启用' if args.use_amp else '禁用'}")

        controller = RobotController(
            policy=policy,
            robot=robot,
            device=device,
            use_amp=args.use_amp,
            filter_tau=args.filter_tau if not args.no_filter else 0.0,
            use_async_obs=args.use_async_obs,
            obs_freq=args.obs_freq,
        )

        controller.start()

        # 主控制循环
        period = 1.0 / args.freq
        print("\n")
        print(f"=======================================================")
        print(f"              开始控制循环 ({args.freq} Hz)              ")
        print(f"=======================================================")
        print("[INFO] 按 Ctrl+C 停止...\n")

        step = 0
        last_time = time.perf_counter()
        error_count = 0
        max_consecutive_errors = 10

        while not shutdown_event.is_set():
            loop_start = time.perf_counter()
            dt = loop_start - last_time
            last_time = loop_start

            success, err_msg = controller.step(dt)

            if not success:
                error_count += 1

                if error_count <= 3:
                    print(f"[ERROR {error_count}/{max_consecutive_errors}] {err_msg}")
                elif error_count % 10 == 0:
                    print(f"[ERROR] 累计错误 {error_count} 次: {err_msg}")

                if error_count >= max_consecutive_errors:
                    print(f"\n[ERROR] 连续 {error_count} 次错误,停止控制循环")
                    print("[INFO] 将执行安全归零...")
                    break

                time.sleep(0.01)
                continue
            else:
                if error_count > 0:
                    if error_count >= 3:
                        print(
                            f"[RECOVER] 控制已恢复正常 (之前累计 {error_count} 次错误)"
                        )
                    error_count = 0

            if step % 30 == 0:
                avg_latency = controller.get_avg_latency()
                print(f"[Step {step:5d}] {controller.stats} | avg:{avg_latency:.1f}ms")

            elapsed = time.perf_counter() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
            elif step % 100 == 0:
                print(f"[WARN] 循环超时: {elapsed*1000:.1f}ms > {period*1000:.1f}ms")

            step += 1

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    except Exception as e:
        print(f"\n[ERROR] 发生异常: {e}")
        import traceback

        traceback.print_exc()
        print("\n[INFO] 检测到异常,执行安全归零...")
    finally:
        # 安全关闭
        if controller:
            print("[INFO] 停止控制器...")
            controller.stop()

        if robot_connected and not args.no_reset:
            print("\n[INFO] 执行平滑归零（保护机械臂）...")
            try:
                smooth_reset(
                    robot,
                    control_freq=200.0,
                    smooth_time=args.reset_time,
                    max_step=0.05,
                    timeout=30.0,
                    tolerance=0.02,
                    open_gripper=True,
                )
            except Exception as e:
                print(f"[WARN] 归零过程出错: {e}")
                print("[WARN] 请手动检查机械臂状态！")
        elif robot_connected and args.no_reset:
            print("\n[WARN] 跳过归零 (--no_reset 模式)")
            print("[WARN] 机械臂可能处于非零位,请注意安全!")

        print("\n[INFO] 断开机械臂连接...")
        try:
            robot.disconnect()
        except:
            pass

        print("[INFO] 退出完成\n")


if __name__ == "__main__":
    main()
