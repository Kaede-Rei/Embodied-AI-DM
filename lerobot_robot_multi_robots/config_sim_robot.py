from dataclasses import dataclass, field


from lerobot.robots import Robot, RobotConfig
from lerobot.cameras import CameraConfig

@RobotConfig.register_subclass("sim_robot")
@dataclass
class SimRobotConfig(RobotConfig):
    num_joints: int = 6
    cameras: dict[str, CameraConfig] = field(default_factory=dict)