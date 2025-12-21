[TOC]

# TRLC-DK1 使用教程

------

## 1. 安装与环境准备

### 1.1. 环境配置

确保系统有以下软件：

- Git
- Python >= 3.10
- USB 权限（用于机械臂串口设备）
- OpenCV

USB 权限建加 udev 规则：

```bash
sudo usermod -aG dialout $USER
sudo reboot		# 重启
```

### 1.2. 克隆仓库到本地

```bash
git clone https://github.com/robot-learning-co/trlc-dk1.git
cd trlc-dk1
```

### 1.3. 创建 Python 虚拟环境（推荐用 MiniConda）

```bash
conda create -n dk1 python=3.10
conda activate dk1
```

### 1.4. 安装依赖

在项目根目录执行：

```bash
pip install -e .
```

这个命令会：

- 安装本地开发包（editable 模式）
- 自动拉取仓库依赖
- 包括 **TRLC 的 LeRobot 分支** 作为依赖（对应 `trlc-dk1` 版本）([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

## 2. 识别设备串口并进入虚拟环境

不同设备连接后系统会生成 `/dev/tty*`（Linux）或 `/dev/ttyUSB*` 等串口节点。

使用 LeRobot 提供的 CLI 来查找：

```bash
lerobot-find-port
```

这个命令会列出所有可用的串口，方便后续遥操作和采集命令使用([docs.robot-learning.co](https://docs.robot-learning.co/getting_started?utm_source=chatgpt.com))

需要进入虚拟环境 `dk1` 中进行操作

```bash
conda activate dk1
```

## 3. 主从遥操作（Teleoperation）

### 3.1 单臂遥操作（Follower + Leader）

假设：

- follower（被控臂）串口是 `/dev/ttyACM0`
- leader（示教手）串口是 `/dev/ttyUSB0`

使用命令：

```bash
lerobot-teleoperate \
    --robot.type=dk1_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.joint_velocity_scaling=1.0 \
  	--robot.disable_torque_on_disconnect=true \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/ttyUSB0 \
    --display_data=true
```

```bash
    --robot.cameras="{
        context: {type: opencv, index_or_path: 0, width: 1280, height: 720, fps: 30},
        wrist: {type: opencv, index_or_path: 1, width: 1280, height: 720, fps: 30}
      }" \
```

说明：

- `robot.type=dk1_follower`：定义被控臂类型
- `teleop.type=dk1_leader`：定义主示教臂类型
- `joint_velocity_scaling`：关节速度缩放，建议从 0.2 开始调试
- `--robot.cameras`：配置上下文和腕部摄像头参数([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

### 3.2. 脚本

在仓库的`example`中有方便使用的脚本：	

| 脚本路径                                      | 作用                               | 注意事项                                                 |
| --------------------------------------------- | ---------------------------------- | -------------------------------------------------------- |
| `TRLC-DK1/examples/follower_read_position.py` | 读取从臂关节角度，确保有连接上从臂 | 端口要对应上                                             |
| `TRLC-DK1/examples/leader_test.py`            | 读取主臂关节角度，确保有连接上主臂 | 端口要对应上                                             |
| `TRLC-DK1/examples/teleop.py`                 | 启动单臂遥操作                     | 端口要对应上，可选参数 `--display_data` 启动 LeRobot GUI |
| `TRLC-DK1/examples/calibration_follower.py`   | 将从臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `TRLC-DK1/examples/calibration_leader.py`     | 将主臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `TRLC-DK1/examples/bi_teleop.py`              | 启动双臂遥操作                     | 端口要对应上                                             |

## 4. 数据采集（Recording）

采集示教数据（用于后续训练学习模型）示例：

```bash
lerobot-record \
    --robot.type=dk1_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.joint_velocity_scaling=1.0 \
    --robot.cameras="{
        context: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30},
        wrist: {type: opencv, index_or_path: 1, width: 640, height: 360, fps: 30}
      }" \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/ttyUSB0 \
    --dataset.repo_id=$USER/my_dataset \
    --dataset.push_to_hub=false \
    --dataset.num_episodes=50 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=15 \
    --dataset.single_task="My task description." \
    --resume=true
```

说明：

- `dataset.repo_id`：指定数据集名称
- `num_episodes`：采集的示教次数
- `episode_time_s` & `reset_time_s`：每一集的时长及重置时间
- `push_to_hub=false`：不推到 HuggingFace Hub（默认）([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

## 5. 模型推理（Inference / Evaluation）

使用已训练的模型进行推理：

```bash
lerobot-record  \
  --robot.type=dk1_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.joint_velocity_scaling=0.5 \
  --robot.cameras="{
      context: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30},
      wrist: {type: opencv, index_or_path: 1, width: 640, height: 360, fps: 30}
    }" \
  --display_data=true \
  --dataset.repo_id=$USER/eval_my_model \
  --dataset.single_task="My task description." \
  --dataset.push_to_hub=false \
  --policy.path=outputs/my_model/checkpoints/last/pretrained_model
```

说明：

- `policy.path`：使用你自己的模型参数路径
- `display_data=true`：实时显示传感器和控制数据([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

## 6. 双臂（Bimanual）操作

```bash
lerobot-teleoperate \
    --robot.type=bi_dk1_follower \
    --robot.right_arm_port=/dev/ttyACM0 \
    --robot.left_arm_port=/dev/ttyACM1 \
    --robot.joint_velocity_scaling=1.0 \
    --robot.cameras="{
        context: {type: opencv, index_or_path: 2, width: 640, height: 360, fps: 30},
        right_wrist: {type: opencv, index_or_path: 4, width: 640, height: 360, fps: 30},
        left_wrist: {type: opencv, index_or_path: 0, width: 640, height: 360, fps: 30}
      }" \
    --teleop.type=bi_dk1_leader \
    --teleop.right_arm_port=/dev/ttyUSB0 \
    --teleop.left_arm_port=/dev/ttyUSB1 \
    --display_data=true \
    --display_url=100.88.6.81
```

说明：

- 支持双臂主从映射
- 需要分配上下文和两个腕部摄像头([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

## 7. URDF 模型（可视化 / ROS 集成）

仓库的 README 指向了一个高质量的 URDF（由社区贡献）链接，用于：

- 在 ROS / RViz 中可视化机械臂模型
- 用于模拟训练和轨迹规划

（链接在 README 中可直接访问）([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

## 8. 注意事项

### 8.1. 摄像头编号

不同 Ubuntu 机器摄像头编号可能不一样，在命令行中指定正确的 `--robot.cameras` 索引。

### 8.2. 关节速度缩放

```
--robot.joint_velocity_scaling=0.2
```

确保安全后再提升到 1.0([docs.robot-learning.co](https://docs.robot-learning.co/getting_started?utm_source=chatgpt.com))

### 8.3. USB 权限

没有正确的 udev 权限可能导致串口无法打开

