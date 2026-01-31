from .config_dm_arm import DMFollowerConfig, DMLeaderConfig

from functools import cached_property
from typing import Any
import logging
import serial
import time

from lerobot.robots import Robot
from lerobot.teleoperators import Teleoperator
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
from lerobot.motors import Motor, MotorNormMode

from lerobot_robot_multi_robots.motors.DM_Control_Python.DM_CAN import *

logger = logging.getLogger(__name__)


def map_range(
    x: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class DMFollower(Robot):
    """
    TRLC-DK1 Follower Arm designed by The Robot Learning Company.
    """

    config_class = DMFollowerConfig
    name = "dm_follower"

    def __init__(self, config: DMFollowerConfig):
        super().__init__(config)

        # Constants for EMIT control
        self.DM4310_TORQUE_CONSTANT = 0.945  # Nm/A
        self.EMIT_VELOCITY_SCALE = 100  # rad/s
        self.EMIT_CURRENT_SCALE = 1000  # A

        self.JOINT_LIMITS = {
            "joint_4": (-100 / 180 * np.pi, 100 / 180 * np.pi),
            "joint_5": (-90 / 180 * np.pi, 90 / 180 * np.pi),
        }

        self.DM4310_SPEED = 200 / 60 * 2 * np.pi  # rad/s (200  rpm | 20.94 rad/s)
        self.DM4340_SPEED = 52.5 / 60 * 2 * np.pi  # rad/s (52.5 rpm | 5.49  rad/s)

        self.config = config
        self.motors = {
            "joint_1": DM_Motor(DM_Motor_Type.DM4340, 0x01, 0x11),
            "joint_2": DM_Motor(DM_Motor_Type.DM4340, 0x02, 0x12),
            "joint_3": DM_Motor(DM_Motor_Type.DM4340, 0x03, 0x13),
            "joint_4": DM_Motor(DM_Motor_Type.DM4310, 0x04, 0x14),
            "joint_5": DM_Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
            "joint_6": DM_Motor(DM_Motor_Type.DM4310, 0x06, 0x16),
            "gripper": DM_Motor(DM_Motor_Type.DM4310, 0x07, 0x17),
        }
        self.control = None
        self.serial_device = None
        self.bus_connected = False

        # 映射开闭角度（默认 0.0 ~ -4.7）（完全闭合 0.0 ~ -5.23）
        self.gripper_open_pos = 0.0
        self.gripper_closed_pos = -5.23

        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motors}

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
        return self.bus_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.serial_device = serial.Serial(self.config.port, 921600, timeout=0.5)
        time.sleep(0.5)

        self.control = MotorControl(self.serial_device)
        self.bus_connected = True
        self.configure()

        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:

        for key, motor in self.motors.items():
            self.control.addMotor(motor)

            for _ in range(3):
                self.control.refresh_motor_status(motor)
                time.sleep(0.01)

            if self.control.read_motor_param(motor, DM_variable.CTRL_MODE) is not None:
                print(f"{key} ({motor.MotorType.name}) is connected.")

                self.control.switchControlMode(motor, Control_Type.POS_VEL)
                self.control.enable(motor)
            else:
                raise Exception(f"Unable to read from {key} ({motor.MotorType.name}).")

        for joint in ["joint_1", "joint_2", "joint_3"]:
            self.control.change_motor_param(self.motors[joint], DM_variable.ACC, 10.0)
            self.control.change_motor_param(self.motors[joint], DM_variable.DEC, -10.0)
            self.control.change_motor_param(self.motors[joint], DM_variable.KP_APR, 200)
            self.control.change_motor_param(self.motors[joint], DM_variable.KI_APR, 10)

        for joint in ["gripper"]:
            self.control.change_motor_param(self.motors[joint], DM_variable.KP_APR, 100)

        # Open gripper and set zero position
        self.control.switchControlMode(self.motors["gripper"], Control_Type.VEL)
        self.control.control_Vel(self.motors["gripper"], 10.0)
        while True:
            self.control.refresh_motor_status(self.motors["gripper"])
            tau = self.motors["gripper"].getTorque()
            if tau > 1.2:
                self.control.control_Vel(self.motors["gripper"], 0.0)
                self.control.disable(self.motors["gripper"])
                self.control.set_zero_position(self.motors["gripper"])
                time.sleep(0.2)
                self.control.enable(self.motors["gripper"])
                break
            time.sleep(0.01)
        self.control.switchControlMode(self.motors["gripper"], Control_Type.Torque_Pos)

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()

        obs_dict = {}
        for key, motor in self.motors.items():
            self.control.refresh_motor_status(motor)
            if key == "gripper":
                # Normalize gripper position between 1 (closed) and 0 (open)
                obs_dict[f"{key}.pos"] = map_range(
                    motor.getPosition(),
                    self.gripper_open_pos,
                    self.gripper_closed_pos,
                    0.0,
                    1.0,
                )
            else:
                obs_dict[f"{key}.pos"] = motor.getPosition()

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        goal_pos = {
            key.removesuffix(".pos"): val
            for key, val in action.items()
            if key.endswith(".pos")
        }

        # Send goal position to the arm
        for key, motor in self.motors.items():
            if key == "gripper":
                self.control.refresh_motor_status(motor)
                gripper_goal_pos_mapped = map_range(
                    goal_pos[key],
                    0.0,
                    1.0,
                    self.gripper_open_pos,
                    self.gripper_closed_pos,
                )
                self.control.control_pos_force(
                    motor,
                    gripper_goal_pos_mapped,
                    self.DM4310_SPEED * self.EMIT_VELOCITY_SCALE,
                    i_des=self.config.max_gripper_torque
                    / self.DM4310_TORQUE_CONSTANT
                    * self.EMIT_CURRENT_SCALE,
                )
            else:
                if key in self.JOINT_LIMITS:
                    goal_pos[key] = np.clip(
                        goal_pos[key],
                        self.JOINT_LIMITS[key][0],
                        self.JOINT_LIMITS[key][1],
                    )

                self.control.control_Pos_Vel(
                    motor,
                    goal_pos[key],
                    self.config.joint_velocity_scaling * self.DM4340_SPEED,
                )

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.disable_torque_on_disconnect:
            for motor in self.motors.values():
                self.control.disable(motor)
        else:
            self.control.serial_.close()
        self.bus_connected = False

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")


class DMLeader(Teleoperator):
    config_class = DMLeaderConfig
    name = "dm_leader"

    def __init__(self, config: DMLeaderConfig):
        super().__init__(config)
        self.config = config
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "joint_1": Motor(1, "xl330-m288", MotorNormMode.DEGREES),
                "joint_2": Motor(2, "xl330-m288", MotorNormMode.DEGREES),
                "joint_3": Motor(3, "xl330-m288", MotorNormMode.DEGREES),
                "joint_4": Motor(4, "xl330-m288", MotorNormMode.DEGREES),
                "joint_5": Motor(5, "xl330-m077", MotorNormMode.DEGREES),
                "joint_6": Motor(6, "xl330-m077", MotorNormMode.DEGREES),
                "gripper": Motor(7, "xl330-m077", MotorNormMode.DEGREES),
            },
        )

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = False) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect(handshake=False)
        self.bus.set_baudrate(1000000)

        self.configure()

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()

        # Enable torque and set to position to open
        self.bus.write("Torque_Enable", "gripper", 0, normalize=False)
        self.bus.write(
            "Operating_Mode",
            "gripper",
            OperatingMode.CURRENT_POSITION.value,
            normalize=False,
        )
        self.bus.write("Current_Limit", "gripper", 100, normalize=False)
        self.bus.write("Torque_Enable", "gripper", 1, normalize=False)
        self.bus.write(
            "Goal_Position", "gripper", self.config.gripper_open_pos, normalize=False
        )

    def setup_motors(self) -> None:
        for motor in self.bus.motors:
            input(
                f"Connect the controller board to the '{motor}' motor only and press enter."
            )
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()

        action = self.bus.sync_read(normalize=False, data_name="Present_Position")
        action = {
            f"{motor}.pos": (
                (val / 4096 * 2 * np.pi - np.pi) if motor != "gripper" else val
            )
            for motor, val in action.items()
        }

        # # Normalize gripper position between 1 (closed) and 0 (open)
        gripper_range = self.config.gripper_open_pos - self.config.gripper_closed_pos
        action["gripper.pos"] = (
            1 - (action["gripper.pos"] - self.config.gripper_closed_pos) / gripper_range
        )

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")
