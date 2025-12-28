[TOC]

# 达妙机械臂 - LeRobot 使用教程

------

## 1. 安装与环境准备

### 1.1. 环境配置

确保系统有以下软件：

- v4l2-ctl（用于获取摄像头端口）
- USB 权限（用于机械臂串口设备）
- OpenCV

USB 权限建加 udev 规则：

```bash
sudo usermod -aG dialout $USER
sudo reboot		# 重启
```

### 1.2. 硬件连接

- 电源：小电源开 24V 供从臂；大电源开 6-10V(建议8V) 供主臂	<img src="README.assets/image-20251223214229135.png" alt="image-20251223214229135" style="zoom:15%;" />

- 从臂：小电源接转接板连接到底座供电，CAN 口接 USB转CAN模块 再接电脑，一般是 `/dev/ttyACM0` 

  <img src="README.assets/image-20251223214528974.png" alt="image-20251223214528974" style="zoom:20%;" />

- 主臂：大电源降压后接到转接板连接到底座供电和串口信号，Type-C 口接电脑，一般是 `/dev/ttyUSB0`

  <img src="README.assets/image-20251223214814763.png" alt="image-20251223214814763" style="zoom:20%;" />

- 相机：录包时必须接相机，相机 USB 直接连接，然后用下面的脚本识别

## 2. 识别设备串口并进入虚拟环境

不同设备连接后系统会生成 `/dev/tty*`（Linux）或 `/dev/ttyUSB*` 等串口节点。

使用 LeRobot 提供的 CLI 来查找，按提示操作确定对应串口即可：

```bash
lerobot-find-port
```

需要进入虚拟环境 `lerobot` 中进行操作

```bash
如果终端所在目录为：/media/$USER/BF129129748FD44A/home/LeRobot-Workspace/custom-hw-sim
	则：. ./../../activate.sh lerobot 或 source ./../../activate.sh lerobot
也就是启动位于 home/ 目录下的移动硬盘虚拟环境激活
```

## 3. 主从遥操作（teleop.py）

### 3.1. 单臂遥操作

在虚拟环境中进入终端输入 `python scripts/teleop.py` ，具体参数配置见脚本注释，按实际修改

### 3.2. 脚本

在仓库的 `scripts` 中有方便使用的脚本：	

| 脚本路径                          | 作用                               | 注意事项                                                 |
| --------------------------------- | ---------------------------------- | -------------------------------------------------------- |
| `scripts/follower_test.py`        | 读取从臂关节角度，确保有连接上从臂 | 端口要对应上                                             |
| `scripts/leader_test.py`          | 读取主臂关节角度，确保有连接上主臂 | 端口要对应上                                             |
| `scripts/teleop.py`               | 启动单臂遥操作                     | 端口要对应上，可选参数 `--display_data` 启动 LeRobot GUI |
| `scripts/calibration_follower.py` | 将从臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `scripts/calibration_leader.py`   | 将主臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `scripts/bi_teleop.py`            | 启动双臂遥操作                     | 端口要对应上                                             |

## 4. 数据采集（record_dm.sh）

### 4.1. 登录 Hugging Face 及修改训练集存储位置

训练集需要上传到 Hugging Face Hub 上进行模型训练，现在 [Hugging Face](https://huggingface.co/) 上登录并在 `Setting/Access Tokens` 中新建一个 Token，将 Token 复制下来后在虚拟环境中输入命令 `hf auth login` 根据弹出来的提示依次输入 Token 和 Y (Add token as git credential)

> 在登录前先配置 Git 的 credential helpr : `git config --global credential.helper store`
> 		并且注意登录时梯子要挂全局模式（ 服务模式 + Tun 模式）让终端能翻墙登录后

登录后用 `ht auth whoami` 来确认有没有登录成功，同时 Token 也会被保存在 `~./cache/huggingface/stored_tokens` 中

训练集默认是存储在 `~./cache/huggingface/lerobot/<repo_id>/` 中，为了避免空间爆炸，需要在 `.bashrc` 里添加 `export HF_HOME="路径"` （这个指令将 `huggingface/` 搬到指定的位置），例如搬到移动硬盘里，那路径就是 `"/media/.../.../huggingface"` ，如果使用当前移动硬盘版本，则务必做好这一步

### 4.2. 扫描并加入摄像头

- 使用脚本 `scripts/get_uvc_cam_idx.py` 获取可用相机的索引和可用分辨率及对应帧数，一般使用直接 `python scripts/get_uvc_cam_idx.py` 即可，其他参数及具体使用方法见脚本

- 注意使用该脚本前需要自行安装 `v4l-utils` ：

  ```bash
  sudo apt install v4l-utils
  ```

- 获取索引后配置 `scripts/record_dm.sh` 的相机参数 `CAMERAS_CONFIG` ，修改相机索引

- 相机可随意摆放，唯一要求是相机能观测到机械臂本体的完整动作（如果动作涉及交互物，则交互动作不能被遮挡），并且摆放好之后尽量别挪动位置

- 固定摄像头的索引：[Linux 下将摄像头绑定到固定 /dev/video 设备路径的详细指南](https://geek-blogs.com/blog/linux-bind-camera-to-fixed-dev-video/)（固定串口操作同理），后续会写一个自动配置端口的脚本来方便使用

### 4.3. 训练集录包

- 在运行命令行或脚本后的录包操作：

  - 语音播报内容：
    - 开始录制 Y 个 episode 中的 第 X 个 episode：`"Recording episode X of Y"`
    - 重置环境阶段：`"Reset the environment"`

  - 键盘操作：
    - 右箭头键 `→` ：结束当前 episode 并推进到下一个
      - 左箭头键 `←` ：重新录制当前 episode
      - `ESC` 键：结束整个录制过程并根据 `push_to_hub` 参数来保存本地 + 上传 Hub


- 录包脚本 `bash/record_dm.sh` ：

  - 进入虚拟环境后先给脚本权限：`chmod +x bash/record_dm.sh`
  - 在终端输入 `./bash/record_dm.sh` 来启动录包脚本
  - 参数 `--repo_id` 必填，为训练集名称；默认开启续录，当训练集不存在时自动创建
  - 常规使用：`./bash/record_dm.sh --repo_id $USER/name`
  - 目前脚本默认是：`Recording ...` 花了多长时间，则 `Reset ...` 则需要花同等时间
  - 具体参数见脚本注释，主要要按格式填好摄像头配置，然后重置时间 `Reset the environment` 这个注意是重复 `episode` 使用的时间，例如预设的是30s，而实际提前录制完成使用的时间为15s，那 `RESET_TIME_S` 如果设置为10s，那就会再播报 `Reset the environment` 之后会有10s时间来继续遥操作，10s后停止遥操作并给 15s + 10s = 25s 的时间来重置环境；因此 `RESET_TIME_S` 建议设置为0，录制期间就完成动作并复位好机械臂然后按下 `→` 按键进行环境复原，效率会更高
- 录包期间操作：
  - 听语音播报：`Recording episode X` 时开始动作（其中 X 表示为在录第 X 个 episode）
  - 听语音播报：`Reset the environment` 时恢复场景
  - `←` 按键：重新录制当前 episode
  - `→` 按键：提前录制完成当前 episode ，可以开始场景恢复以准备下一个 episode
  - `ESC` 按键：终止录制，终止后小机械臂可以随便动，达妙机械臂会保持力矩在原位姿
- 终止录制后，终端输入 `python scripts/reset.py` 来让达妙机械臂复位并失能

### 4.4. 录包技巧

- 按 LeRobot 官方的推荐，应该 10 个 episodes 为一组，录制多个随机分布的场景，至少有五个场景也就是至少有 50 个 episodes 作为一次最简训练集，同时最佳实践是操作者能仅通过摄像头来完成全部操作，同时操作完整、平滑不突变，并在录包完成后对数据进行 线性插值 + 平滑滤波处理：

  ```
  Tips for gathering data
  
  Once you’re comfortable with data recording, you can create a larger dataset for training. A good starting task is grasping an object at different locations and placing it in a bin. We suggest recording at least 50 episodes, with 10 episodes per location. Keep the cameras fixed and maintain consistent grasping behavior throughout the recordings. Also make sure the object you are manipulating is visible on the camera’s. A good rule of thumb is you should be able to do the task yourself by only looking at the camera images.
  
  In the following sections, you’ll train your neural network. After achieving reliable grasping performance, you can start introducing more variations during data collection, such as additional grasp locations, different grasping techniques, and altering camera positions.
  
  Avoid adding too much variation too quickly, as it may hinder your results.
  ```

- 对于相机的摆布，建议是 一个全局视角相机 + 一个末端视角相机

- 更多社区经验见：[LeRobot Community Datasets: what-makes-a-good-dataset](https://huggingface.co/blog/lerobot-datasets#what-makes-a-good-dataset)

### 4.5. 数据处理

找到自己的训练集所在位置，复制 `<repo_id>/data/chunk-xxx/file-xxx.parquet` 到仓库的 `./data` 中，终端输入 `./scripts/data_processer.py` 具体参数配置见脚本注释，查看处理前后的波形，直到突变点极少、波形平滑后再保存，然后将输出的 `file-xxx_new.parquet` 重命名并覆盖训练集的数据

### 4.6. 官方命令行入口点

| **命令行入口点**          | **主要作用**                                                 |
| ------------------------- | ------------------------------------------------------------ |
| lerobot-calibrate         | 用于校准机器人组件（如关节、电机），支持多种硬件（如 OMX）   |
| lerobot-dataset-viz       | 可视化机器人数据集，包括轨迹、图像和动作序列，支持 rerun.io 或 HTML 输出 |
| lerobot-edit-dataset      | 编辑和转换数据集，例如将图像序列转换为视频格式或清洗数据（仅裁剪） |
| lerobot-eval              | 评估训练好的策略模型，支持模拟环境和真实硬件，计算成功率等指标 |
| lerobot-find-cameras      | 检测和识别连接的摄像头设备，支持预热和配置测试               |
| lerobot-find-joint-limits | 确定机器人的关节极限范围，支持多种机器人类型                 |
| lerobot-find-port         | 查找机器人通信的串口（如 /dev/ttyACM*）                      |
| lerobot-imgtransform-viz  | 可视化图像变换（如裁剪、归一化）效果，用于数据预处理调试     |
| lerobot-info              | 显示机器人设置和配置的详细信息，用于诊断和验证               |
| lerobot-record            | 录制机器人交互数据（遥操作或策略 rollout），并生成数据集，支持上传 Hugging Face Hub |
| lerobot-replay            | 重放录制的机器人数据，用于验证或演示轨迹                     |
| lerobot-setup-motors      | 配置机器人电机（如 ID、波特率），支持多种硬件                |
| lerobot-teleoperate       | 实时遥操作机器人，用于数据采集，支持键盘或 leader-follower 模式 |
| lerobot-train             | 训练机器人策略模型（如 ACT、Diffusion），支持从数据集加载和 WandB 监控 |

## 5. 模型训练（train_dm.sh）

录制好训练集后即可开始训练，运行脚本：

```bash
./bash/train_dm.sh --policy_repo_id 模型名称 --dataset_repo_id 训练集名称
```

训练启动后会进入后台进行训练，训练进度在 `./log/` 下生成的 `训练集名称.log` 中实时打印，第一次训练会有进度条不停打印在下载前置，正式开始训练时会有以下内容，每200步会打印一次进度：

```bash
INFO 2025-12-23 18:25:19 ot_train.py:347 step:200 smpl:400 ep:1 epch:0.05 loss:0.737 grdn:51.002 lr:1.0e-05 updt_s:0.182 data_s:0.005
INFO 2025-12-23 18:25:55 ot_train.py:347 step:400 smpl:800 ep:2 epch:0.09 loss:0.546 grdn:36.100 lr:1.0e-05 updt_s:0.177 data_s:0.004
INFO 2025-12-23 18:26:32 ot_train.py:347 step:600 smpl:1K ep:3 epch:0.14 loss:0.498 grdn:28.908 lr:1.0e-05 updt_s:0.181 data_s:0.004
INFO 2025-12-23 18:27:07 ot_train.py:347 step:800 smpl:2K ep:4 epch:0.18 loss:0.445 grdn:26.680 lr:1.0e-05 updt_s:0.172 data_s:0.004
INFO 2025-12-23 18:27:43 ot_train.py:347 step:1K smpl:2K ep:5 epch:0.23 loss:0.415 grdn:23.008 lr:1.0e-05 updt_s:0.175 data_s:0.004
```

训练完成后会有以下内容：

```bash
INFO 2025-12-23 20:13:54 ot_train.py:347 step:40K smpl:80K ep:182 epch:9.10 loss:0.092 grdn:6.646 lr:1.0e-05 updt_s:0.160 data_s:0.003
INFO 2025-12-23 20:13:54 ot_train.py:357 Checkpoint policy after step 40000
INFO 2025-12-23 20:13:55 ot_train.py:426 End of training
```

训练好的模型会在当前文件夹的 `output` 文件夹下，如果需要终止训练，则见 `./log/训练集名称.param` 里的训练集PID：`TRAIN_PID=pid` ，在终端中输入 `kill pid` 即可停止终止训练，此时 `output` 文件夹里只会有训练了一半的模型（根据步数，在40000步情况下会有 20000 和 40000 两个步数时的模型）

其他参数需要修改的见脚本注释

## 6. 模型评估（eval_dm.sh）

训练好的模型可以运行脚本来评估：

```bash
./bash/eval_dm.sh --policy_repo_id 模型名称
```

注意该评估脚本只能直接运行训练脚本训练出来的模型，如果想运行其他脚本，先上传到 Hub，然后输入：

```bash
# ./eval_dm.sh --policy_repo_id 模型名称 --from_hub
```

评估也相当于一次录制，录制的结果会和训练集存在一起并加上前缀 `eval_`

## 7. 双臂操作

```bash
lerobot-teleoperate \
    --robot.type=bi_dm_follower \
    --robot.right_arm_port=/dev/ttyACM0 \
    --robot.left_arm_port=/dev/ttyACM1 \
    --robot.joint_velocity_scaling=1.0 \
    --robot.cameras="{
        context: {type: opencv, index_or_path: 2, width: 640, height: 360, fps: 30},
        right_wrist: {type: opencv, index_or_path: 4, width: 640, height: 360, fps: 30},
        left_wrist: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30}
      }" \
    --teleop.type=bi_dm_leader \
    --teleop.right_arm_port=/dev/ttyUSB0 \
    --teleop.left_arm_port=/dev/ttyUSB1 \
    --display_data=true \
    --display_url=100.88.6.81
```

说明：

- 支持双臂主从映射
- 需要分配上下文和两个腕部摄像头([GitHub](https://github.com/robot-learning-co/trlc-dm/tree/main?tab=readme-ov-file))

## 8. 自定义硬件

LeRobot 采用插件化架构，可以通过继承 Robot 基类并注册的方式添加自定义插件支持

### 8.1. 核心步骤

1. **定义配置类**：继承 `RobotConfig` 并注册名称

   - 创建一个继承自 `lerobot.robots.RobotConfig` 的数据类，使用 `@RobotConfig.register_subclass("robot_name")` 装饰器进行注册
   - 该类用于存放硬件特定参数（如端口、波特率、相机配置等）

2. **定义实现类**：继承 `Robot` 并实现核心接口

   ​		创建一个继承自` lerobot.robots.Robot` 的类，指定 `config_class` 为上一步定义的配置类；

   ​		必须实现以下关键属性和方法：

   - `observation_features` 和 `action_features`：定义观察和动作的空间结构
   - `connect()`、`disconnect()`：建立和断开硬件通信
   - `get_observation()`：读取传感器状态和图像
   - `send_action()`：向硬件发送控制指令
   - 可选实现 `calibrate()`、`configure()` 等

3. **实现底层通信**：在类中调用硬件驱动（串口、CAN、RS485等）

   ​		在**实现类**中集成实际的硬件通信代码可以直接使用 `serial`、`socket`、`pyserial` 等标准库，或引入第三方驱动（如 CAN 协议库、Dynamixel SDK、Feetech SDK 等）

   ​		所有通信逻辑封装在 `connect`、`get_observation`、`send_action` 等方法中

### 8.2. 代码示例

以 达妙机械臂 - LeRobot - TRLC - dm 的主臂为例，目前就是利用自定义硬件接口实现整机移动到新版 LeRobot 方便在移动硬盘使用和进行强化学习训练

```python
# 导入 dataclasses 模块，用于创建数据类（自动生成 __init__、__repr__ 等方法）
from dataclasses import dataclass, field
# 导入类型提示相关模块
from typing import Any
# 导入串口通信库
import serial
# 导入时间模块，用于延时和计时
import time
# 导入日志模块，用于调试信息输出
import logging

# 从 LeRobot 导入基类和相机相关工具
from lerobot.robots import Robot, RobotConfig                   	# 机器人基类和配置基类
from lerobot.cameras import CameraConfig                          	# 相机配置类
from lerobot.cameras.utils import make_cameras_from_configs       	# 根据配置创建相机实例的工具函数

# 创建日志记录器，用于输出调试信息
logger = logging.getLogger(__name__)

# 定义配置类：继承 RobotConfig，并通过装饰器注册为 "new_robot"
@RobotConfig.register_subclass("dm_follower")	# 注册名称，使 CLI 可通过 --robot.type=dm_follower 调用
@dataclass                                      # 自动生成 __init__、__repr__ 等方法
class dmFollowerConfig(RobotConfig):          	# 继承 LeRobot 的 RobotConfig 基类
    port: str                                   # 必填参数：串口路径（如 "/dev/ttyUSB0"）
    disable_torque_on_disconnect: bool = True   # 可选：断开连接时是否自动禁用电机扭矩（安全考虑）
    joint_velocity_scaling: float = 0.2         # 可选：关节目标速度缩放系数（0~1，防止过快运动）
    max_gripper_torque: float = 1.0             # 可选：夹爪最大力矩（单位：Nm）
    cameras: dict[str, CameraConfig] = field(default_factory=dict)  # 可选：相机配置字典，默认为空


# 2. 定义机器人实现类：继承 Robot
class dmFollower(Robot):
    config_class = dmFollowerConfig                   	# 指定对应的配置类
    name = "dm_follower"                              	# 机器人名称（与注册名称一致）

    def __init__(self, config: dmFollowerConfig):     	# 构造函数，接收配置实例
        super().__init__(config)                       	# 调用父类初始化
        self.config = config                           	# 保存配置便于后续访问
        
        # 导入第三方电机驱动库（此处为 TRLC-dm 专用的 DM_CAN 协议库）
        from trlc_dm.motors.DM_Control_Python.DM_CAN import Motor, MotorControl, DM_Motor_Type
        
        # 定义所有电机实例：键为关节名，值为 Motor 对象（包含电机类型、CAN ID 等）
        self.motors = {
            "joint_1": Motor(DM_Motor_Type.DM4340, 0x01, 0x11),
            "joint_2": Motor(DM_Motor_Type.DM4340, 0x02, 0x12),
            "joint_3": Motor(DM_Motor_Type.DM4340, 0x03, 0x13),
            "joint_4": Motor(DM_Motor_Type.DM4310, 0x04, 0x14),
            "joint_5": Motor(DM_Motor_Type.DM4310, 0x05, 0x15),
            "joint_6": Motor(DM_Motor_Type.DM4310, 0x06, 0x16),
            "gripper": Motor(DM_Motor_Type.DM4310, 0x07, 0x17),
        }
        
        # 初始化底层控制对象和通信状态
        self.control = None                            	# MotorControl 实例（管理所有电机）
        self.serial_device = None                      	# 串口对象
        self.bus_connected = False                     	# 通信总线连接状态标志
        
        # 根据配置创建并初始化所有相机实例
        self.cameras = make_cameras_from_configs(config.cameras)

    # 定义观察空间：LeRobot 要求返回所有观察特征的名称和类型/形状
    @property
    def observation_features(self) -> dict[str, Any]:
        # 电机位置：每个关节一个 float 类型的位置值
        motors = {f"{m}.pos": float for m in self.motors}
        # 相机图像：返回 (height, width, 3) 的形状元组（RGB图像）
        cams = {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }
        return {**motors, **cams}                      	# 合并电机和相机特征

    # 定义动作空间：LeRobot 要求返回所有可控制特征的名称和类型
    @property
    def action_features(self) -> dict[str, Any]:
        return {f"{m}.pos": float for m in self.motors} # 动作仅控制各关节位置

    # 连接硬件：建立串口通信并初始化电机和相机
    def connect(self) -> None:
        if self.bus_connected:
            raise RuntimeError("Already connected")    	# 防止重复连接
        
        # 打开串口（波特率 921600，与硬件匹配）
        self.serial_device = serial.Serial(self.config.port, 921600, timeout=0.5)
        time.sleep(0.5)                                	# 短暂延时等待硬件稳定
        
        # 创建电机控制总线实例
        self.control = MotorControl(self.serial_device)
        self.bus_connected = True                      	# 标记总线已连接
        
        self.configure()                               	# 执行电机参数配置
        
        # 连接所有相机
        for cam in self.cameras.values():
            cam.connect()

    # 配置电机参数（包括使能、控制模式切换、PID 参数设置等）
    def configure(self) -> None:
        # 详细配置逻辑请参考原文件，此处省略以保持简洁
        # 主要操作：添加电机、切换控制模式、设置加减速、PID、夹爪自动归零等
        pass

    # 获取当前观察：读取所有电机位置和相机图像
    def get_observation(self) -> dict[str, Any]:
        if not self.bus_connected:
            raise RuntimeError("Not connected")        	# 未连接时抛出异常
        
        obs = {}                                       	# 观察字典
        
        # 刷新并读取每个电机的当前位置
        for key, motor in self.motors.items():
            self.control.refresh_motor_status(motor)   	# 更新电机状态
            # 夹爪位置需特殊归一化处理（0=张开，1=闭合），其他关节直接读取
            obs[f"{key}.pos"] = motor.getPosition()
        
        # 读取所有相机图像（异步读取以提高效率）
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.async_read()
        
        return obs                                     	# 返回完整观察字典

    # 发送动作：将策略输出的目标位置下发到硬件
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.bus_connected:
            raise RuntimeError("Not connected")
        
        # 从动作字典中提取目标位置（键如 "joint_1.pos" → "joint_1"）
        goal_pos = {k.removesuffix(".pos"): v for k, v in action.items() if k.endswith(".pos")}
        
        # 遍历每个电机下发指令
        for key, motor in self.motors.items():
            if key == "gripper":
                # 夹爪使用力控位置模式（Torque_Pos），限制最大力矩
                pass  # 具体实现见原文件
            else:
                # 普通关节使用位置-速度模式（Pos_Vel），速度受 scaling 限制
                max_speed = 20.94  # 示例最大速度（rad/s），实际应根据电机型号定义
                self.control.control_Pos_Vel(
                    motor, goal_pos[key], self.config.joint_velocity_scaling * max_speed
                )
        
        return action  # LeRobot 要求返回发送的动作（可用于记录）

    # 断开连接：安全关闭通信
    def disconnect(self) -> None:
        if not self.bus_connected:
            return
        
        # 根据配置决定是否先禁用所有电机扭矩（推荐启用以确保安全）
        if self.config.disable_torque_on_disconnect:
            for motor in self.motors.values():
                self.control.disable(motor)
        
        self.bus_connected = False                     	# 标记断开
        
        # 断开所有相机
        for cam in self.cameras.values():
            cam.disconnect()
        
        logger.info("Robot disconnected successfully.")
```



