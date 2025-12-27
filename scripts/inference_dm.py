"""
DM 机器人 —— 持续推理脚本（纯推理，不录制）

功能：
1. 使用 lerobot 的 parser.wrap + dataclass 配置系统
2. 支持：
   - 单臂 DM Follower
   - 双臂 Bi-DM Follower
3. 支持：
   - 从本地 / Hugging Face Hub 加载模型
   - 摄像头启用 / 禁用
   - 控制频率
   - ACT 输出平滑（n_action_steps / temporal_ensemble）

按 Ctrl+C 退出
"""

import time
from dataclasses import dataclass

from lerobot.configs import parser
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.policies.pretrained import PreTrainedPolicy

from lerobot.processor import (
    RobotObservation,
    RobotProcessorPipeline,
    PolicyProcessorPipeline,
)

from lerobot.robots import RobotConfig, make_robot_from_config

from lerobot.utils.constants import OBS_STR, ACTION
from lerobot.utils.utils import init_logging, log_say, get_safe_torch_device
from lerobot.utils.control_utils import predict_action
from lerobot.utils.import_utils import register_third_party_devices


# ============================================================
# 一、配置（映射为 CLI 参数）
# ============================================================

@dataclass
class ContinuousInferenceConfig:
    """
    纯推理运行配置
    """

    # 机器人配置（支持 dm_follower / bi_dm_follower）
    robot: RobotConfig

    # 预训练策略
    policy: PreTrainedConfig

    # 控制频率
    control_fps: float = 25.0

    # ACT 推理平滑
    n_action_steps: int = 1
    temporal_ensemble_coeff: float = 0.01

    # ------------------ 单臂 ------------------
    follower_port: str = "/dev/ttyACM0"

    # ------------------ 双臂 ------------------
    left_arm_port: str = "/dev/ttyACM0"
    right_arm_port: str = "/dev/ttyACM1"

    # 关节速度缩放
    joint_velocity_scaling: float = 1.0

    # 摄像头禁用
    no_cameras: bool = False

    # 默认摄像头配置（与 lerobot-record 一致）
    cameras: dict | None = None

    def __post_init__(self):

        # 默认摄像头配置
        if not self.no_cameras and self.cameras is None:
            self.cameras = {
                "context": {
                    "type": "opencv",
                    "index_or_path": 0,
                    "width": 1280,
                    "height": 720,
                    "fps": 25,
                }
            }


# ============================================================
# 二、推理主循环
# ============================================================

@parser.wrap()
def continuous_infer(cfg: ContinuousInferenceConfig):
    """
    CLI 入口：持续推理
    """

    init_logging()
    device = get_safe_torch_device()

    register_third_party_devices()

    log_say("启动 DM 纯推理模式")

    # --------------------------------------------------------
    # 根据单双臂覆盖端口配置
    # --------------------------------------------------------

    if cfg.robot.type.startswith("bi_"):
        cfg.robot.left_arm_port = cfg.left_arm_port
        cfg.robot.right_arm_port = cfg.right_arm_port
    else:
        cfg.robot.port = cfg.follower_port

    cfg.robot.joint_velocity_scaling = cfg.joint_velocity_scaling

    # 摄像头
    if cfg.no_cameras:
        cfg.robot.cameras = None
    else:
        cfg.robot.cameras = cfg.cameras

    # --------------------------------------------------------
    # 创建机器人
    # --------------------------------------------------------
    robot = make_robot_from_config(cfg.robot)
    robot.connect()
    robot.reset()

    # --------------------------------------------------------
    # 加载策略
    # --------------------------------------------------------
    policy: PreTrainedPolicy = make_policy(cfg.policy, device=device)

    preproc, postproc = make_pre_post_processors(cfg.policy)
    robot_pipeline = RobotProcessorPipeline(preproc.robot_processors)
    policy_pipeline = PolicyProcessorPipeline(preproc.policy_processors)

    dt = 1.0 / cfg.control_fps

    log_say("进入持续推理循环（Ctrl+C 退出）")

    while True:
        # 获取观测
        raw_obs = robot.get_observation()
        obs = RobotObservation({OBS_STR: raw_obs})

        # 预处理
        obs_proc = robot_pipeline(obs)
        policy_input = policy_pipeline(obs_proc)

        # 推理
        out = predict_action(policy, policy_input)

        # 转换成机器人动作
        action = make_robot_action(out[ACTION])

        # 下发命令
        robot.step(action)

        time.sleep(dt)


# ============================================================
# 三、CLI main
# ============================================================

def main():
    continuous_infer()


if __name__ == "__main__":
    main()
