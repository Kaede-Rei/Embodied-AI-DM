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

### 1.2. 克隆仓库到本地或下载项目中修正后的包

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

### 1.5. 如果是克隆仓库，则将项目中的包的一下部分覆盖原仓库

```bash
- /examples
- /src
- pyproject.toml
- README.md
```

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

### 3.1. 单臂遥操作（Follower + Leader）

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
    --robot.cameras="{
        PC: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
    }" \
    --teleop.type=dk1_leader \
    --teleop.port=/dev/ttyUSB0 \
    --display_data=true
```

说明：

- `robot.type=dk1_follower`：定义被控臂类型
- `teleop.type=dk1_leader`：定义主示教臂类型
- `joint_velocity_scaling`：关节速度缩放，建议从 0.2 开始调试
- `--robot.cameras`：配置上下文和腕部摄像头参数([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

### 3.2. 脚本

在仓库的`example`中有方便使用的脚本：	

| 脚本路径                             | 作用                               | 注意事项                                                 |
| ------------------------------------ | ---------------------------------- | -------------------------------------------------------- |
| `examples/follower_read_position.py` | 读取从臂关节角度，确保有连接上从臂 | 端口要对应上                                             |
| `examples/leader_test.py`            | 读取主臂关节角度，确保有连接上主臂 | 端口要对应上                                             |
| `examples/teleop.py`                 | 启动单臂遥操作                     | 端口要对应上，可选参数 `--display_data` 启动 LeRobot GUI |
| `examples/calibration_follower.py`   | 将从臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `examples/calibration_leader.py`     | 将主臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `examples/bi_teleop.py`              | 启动双臂遥操作                     | 端口要对应上                                             |

## 4. 数据采集（Recording）

### 4.1. 登录 Hugging Face 及修改训练集存储位置

训练集需要上传到 Hugging Face Hub 上进行模型训练，现在 [Hugging Face](https://huggingface.co/) 上登录并在 `Setting/Access Tokens` 中新建一个 Token，将 Token 复制下来后在虚拟环境中输入命令 `hf auth login` 根据弹出来的提示依次输入 Token 和 Y (Add token as git credential)

> 在登录前先配置 Git 的 credential helpr : `git config --global credential.helper store`
> 		并且注意登录时梯子要挂全局模式（ 服务模式 + Tun 模式）让终端能翻墙登录后

登录后用 `ht auth whoami` 来确认有没有登录成功，同时 Token 也会被保存在 `~./cache/huggingface/stored_tokens` 中

训练集默认是存储在 `~./cache/huggingface/lerobot/<repo_id>/` 中，为了避免空间爆炸，需要在 `.bashrc` 里添加 `export HF_DATASETS_CACHE="路径"` 路径根据实际情况存到其他地方

### 4.2. 扫描并加入摄像头

- 使用脚本 `examples/get_uvc_cam_idx.py` 获取可用相机的索引，具体使用方法见脚本

- 获取索引后配置 `examples/record_dk1.sh` 的相机参数 `CAMERAS_CONFIG` 

- 相机可随意摆放，唯一要求是相机能观测到机械臂本体的完整动作（如果动作涉及交互物，则交互动作不能被遮挡），并且摆放好之后尽量别随意挪动位置

### 4.3. 训练集录包

- 在运行命令行或脚本后的录包操作：

  - 语音播报内容：
    - 开始录制 Y 个 episode 中的 第 X 个 episode：`"Recording episode X of Y"`
    - 重置环境阶段：`"Reset the environment"`

  - 键盘操作：
    - 右箭头键 `→` ：结束当前 episode 并推进到下一个
      - 左箭头键 `←` ：重新录制当前 episode
      - `ESC` 键：结束整个录制过程并根据 `push_to_hub` 参数来保存本地 + 上传 Hub


- 录包脚本 `examples/record_dk1.sh` ：
  - 进入虚拟环境后在终端输入 `./examples/record_dk1.sh` 来启动录包脚本
  - 参数 `--repo_id` 必填，为训练集名称；
  - 

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

