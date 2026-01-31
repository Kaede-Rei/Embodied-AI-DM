from .config_dual_dm_arm import DualDMFollowerConfig, DualDMLeaderConfig
from .config_dm_arm import DMFollowerConfig, DMLeaderConfig
from .dm_arm import DMFollower, DMLeader

from functools import cached_property
from typing import Any
import logging
import time

from lerobot.robots import Robot
from lerobot.teleoperators import Teleoperator
from lerobot.cameras.utils import make_cameras_from_configs

from lerobot_robot_multi_robots.motors.DM_Control_Python.DM_CAN import *

logger = logging.getLogger(__name__)


def map_range(
    x: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class DualDMFollower(Robot):
    config_class = DualDMFollowerConfig
    name = "dual_dm_follower"

    def __init__(self, config: DualDMFollowerConfig):
        super().__init__(config)

        self.config = config

        left_config = DMFollowerConfig(
            port=self.config.left_port,
            disable_torque_on_disconnect=self.config.disable_torque_on_disconnect,
            joint_velocity_scaling=self.config.joint_velocity_scaling,
            max_gripper_torque=self.config.max_gripper_torque,
        )

        right_config = DMFollowerConfig(
            port=self.config.right_port,
            disable_torque_on_disconnect=self.config.disable_torque_on_disconnect,
            joint_velocity_scaling=self.config.joint_velocity_scaling,
            max_gripper_torque=self.config.max_gripper_torque,
        )

        self.left = DMFollower(left_config)
        self.right = DMFollower(right_config)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"left_{motor}.pos": float for motor in self.left.motors} | {
            f"right_{motor}.pos": float for motor in self.right.motors
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.left.is_connected
            and self.right.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self) -> None:
        self.left.connect()
        self.right.connect()

        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        self.left.configure()
        self.right.configure()

    def get_observation(self) -> dict[str, Any]:
        obs = {}

        left_obs = self.left.get_observation()
        obs.update({f"left_{key}": value for key, value in left_obs.items()})

        right_obs = self.right.get_observation()
        obs.update({f"right_{key}": value for key, value in right_obs.items()})

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1000
            logger.debug(f"Camera '{cam_key}' read took {dt_ms:.2f} ms")

        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        left_action = {
            key.removeprefix("left_"): value
            for key, value in action.items()
            if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value
            for key, value in action.items()
            if key.startswith("right_")
        }

        send_action_left = self.left.send_action(left_action)
        send_action_right = self.right.send_action(right_action)

        profixed_send_action_left = {
            f"left_{key}": value for key, value in send_action_left.items()
        }
        profixed_send_action_right = {
            f"right_{key}": value for key, value in send_action_right.items()
        }

        return {**profixed_send_action_left, **profixed_send_action_right}

    def disconnect(self):
        self.left.disconnect()
        self.right.disconnect()

        for cam in self.cameras.values():
            cam.disconnect()


class DualDMLeader(Teleoperator):
    config_class = DualDMLeaderConfig
    name = "dual_dm_leader"

    def __init__(self, config: DualDMLeaderConfig):
        super().__init__(config)

        self.config = config

        left_config = DMLeaderConfig(
            port=self.config.left_port,
            gripper_open_pos=self.config.gripper_open_pos,
            gripper_closed_pos=self.config.gripper_closed_pos,
        )

        right_config = DMLeaderConfig(
            port=self.config.right_port,
            gripper_open_pos=self.config.gripper_open_pos,
            gripper_closed_pos=self.config.gripper_closed_pos,
        )

        self.left = DMLeader(left_config)
        self.right = DMLeader(right_config)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"left_{motor}.pos": float for motor in self.left.bus.motors} | {
            f"right_{motor}.pos": float for motor in self.right.bus.motors
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left.is_connected and self.right.is_connected

    def connect(self, calibrate: bool = False) -> None:
        self.left.connect()
        self.right.connect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        self.left.configure()
        self.right.configure()

    def setup_motors(self) -> None:
        self.left.setup_motors()
        self.right.setup_motors()

    def get_action(self) -> dict[str, float]:
        action_dict = {}

        left_action = self.left.get_action()
        action_dict.update({f"left_{key}": value for key, value in left_action.items()})

        right_action = self.right.get_action()
        action_dict.update(
            {f"right_{key}": value for key, value in right_action.items()}
        )

        return action_dict

    def send_feedback(self, feedback):
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self):
        self.left.disconnect()
        self.right.disconnect()
