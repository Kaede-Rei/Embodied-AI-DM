from dataclasses import dataclass, field

from lerobot.robots import RobotConfig
from lerobot.teleoperators import TeleoperatorConfig
from lerobot.cameras import CameraConfig

@RobotConfig.register_subclass("dual_dm_follower")
@dataclass
class DualDMFollowerConfig(RobotConfig):
    left_port: str
    right_port: str
    disable_torque_on_disconnect: bool = True
    joint_velocity_scaling: float = 0.2
    max_gripper_torque: float = 1.0
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

@TeleoperatorConfig.register_subclass("dual_dm_leader")
@dataclass
class DualDMLeaderConfig(TeleoperatorConfig):
    left_port: str
    right_port: str
    gripper_open_pos: int = 2280
    gripper_closed_pos: int = 1670