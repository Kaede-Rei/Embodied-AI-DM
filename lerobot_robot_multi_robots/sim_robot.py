from .config_sim_robot import SimRobotConfig

from typing import Any
import numpy as np

from lerobot.robots import Robot
from lerobot.cameras.utils import make_cameras_from_configs


class SimRobot(Robot):
    config_class = SimRobotConfig
    name = "sim_robot"

    def __init__(self, config: SimRobotConfig):
        super().__init__(config)
        self.config = config
        self.sim_joints = np.zeros(self.config.num_joints)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def observation_features(self) -> dict[str, Any]:
        motors = {f"joint_{i}.pos": float for i in range(self.config.num_joints)}
        cams = {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }
        return {**motors, **cams}

    @property
    def action_features(self) -> dict[str, Any]:
        return {f"joint_{i}.pos": float for i in range(self.config.num_joints)}

    def connect(self) -> None:
        print("模拟连接成功")
        for cam in self.cameras.values():
            cam.connect()

    def get_observation(self) -> dict[str, Any]:
        obs = {f"joint_{i}.pos": np.random.uniform(-1.0, 1.0) for i in range(self.config.num_joints)}
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = np.zeros((cam.height, cam.width, 3), dtype=np.uint8)
        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal_pos = {int(k.removesuffix(".pos").split("_")[1]): v for k, v in action.items() if k.endswith(".pos")}  # 添加 if 过滤
        for i in range(self.config.num_joints):
            self.sim_joints[i] = goal_pos.get(i, self.sim_joints[i])  # 安全更新
        print("模拟动作已发送")
        return action

    def disconnect(self) -> None:
        print("模拟断开成功")
        for cam in self.cameras.values():
            cam.disconnect()
