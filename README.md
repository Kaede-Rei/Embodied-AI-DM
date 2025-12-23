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
- /scripts
- /src
- pyproject.toml
- README.md
```

### 1.6. 硬件连接

- 电源：小电源开 24V 供从臂；大电源开 6-10V(建议8V) 供主臂	<img src="/home/kaerei/LeRobot_Workspace/trlc-dk1/README.assets/image-20251223214229135.png" alt="image-20251223214229135" style="zoom:10%;" />

- 从臂：小电源接转接板连接到底座供电，CAN 口接 USB转CAN模块 再接电脑，一般是 `/dev/ttyACM0` 

  <img src="/home/kaerei/LeRobot_Workspace/trlc-dk1/README.assets/image-20251223214528974.png" alt="image-20251223214528974" style="zoom:20%;" />

- 主臂：大电源降压后接到转接板连接到底座供电和串口信号，Type-C 口接电脑，一般是 `/dev/ttyUSB0`

  <img src="/home/kaerei/LeRobot_Workspace/trlc-dk1/README.assets/image-20251223214814763.png" alt="image-20251223214814763" style="zoom:20%;" />

- 相机：录包时必须接相机，相机 USB 直接连接，然后用下面的脚本识别

## 2. 识别设备串口并进入虚拟环境

不同设备连接后系统会生成 `/dev/tty*`（Linux）或 `/dev/ttyUSB*` 等串口节点。

使用 LeRobot 提供的 CLI 来查找，按提示操作确定对应串口即可：

```bash
lerobot-find-port
```

需要进入虚拟环境 `dk1` 中进行操作

```bash
conda activate dk1
```

## 3. 主从遥操作（teleop.py）

### 3.1. 单臂遥操作

在虚拟环境中进入终端输入 `python scripts/teleop.py` ，具体参数配置见脚本注释，按实际修改

### 3.2. 脚本

在仓库的`example`中有方便使用的脚本：	

| 脚本路径                            | 作用                               | 注意事项                                                 |
| ----------------------------------- | ---------------------------------- | -------------------------------------------------------- |
| `scripts/follower_read_position.py` | 读取从臂关节角度，确保有连接上从臂 | 端口要对应上                                             |
| `scripts/leader_test.py`            | 读取主臂关节角度，确保有连接上主臂 | 端口要对应上                                             |
| `scripts/teleop.py`                 | 启动单臂遥操作                     | 端口要对应上，可选参数 `--display_data` 启动 LeRobot GUI |
| `scripts/calibration_follower.py`   | 将从臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `scripts/calibration_leader.py`     | 将主臂当前所有角度设为零点         | 注意不稳定的时候别重设零点                               |
| `scripts/bi_teleop.py`              | 启动双臂遥操作                     | 端口要对应上                                             |

## 4. 数据采集（record_dk1.sh）

### 4.1. 登录 Hugging Face 及修改训练集存储位置

训练集需要上传到 Hugging Face Hub 上进行模型训练，现在 [Hugging Face](https://huggingface.co/) 上登录并在 `Setting/Access Tokens` 中新建一个 Token，将 Token 复制下来后在虚拟环境中输入命令 `hf auth login` 根据弹出来的提示依次输入 Token 和 Y (Add token as git credential)

> 在登录前先配置 Git 的 credential helpr : `git config --global credential.helper store`
> 		并且注意登录时梯子要挂全局模式（ 服务模式 + Tun 模式）让终端能翻墙登录后

登录后用 `ht auth whoami` 来确认有没有登录成功，同时 Token 也会被保存在 `~./cache/huggingface/stored_tokens` 中

训练集默认是存储在 `~./cache/huggingface/lerobot/<repo_id>/` 中，为了避免空间爆炸，需要在 `.bashrc` 里添加 `export HF_DATASETS_CACHE="路径"` 路径根据实际情况存到其他地方

### 4.2. 扫描并加入摄像头

- 使用脚本 `scripts/get_uvc_cam_idx.py` 获取可用相机的索引和可用分辨率及对应帧数，一般使用直接 `python scripts/get_uvc_cam_idx.py` 即可，其他参数及具体使用方法见脚本

- 获取索引后配置 `scripts/record_dk1.sh` 的相机参数 `CAMERAS_CONFIG` ，修改相机索引

- 相机可随意摆放，唯一要求是相机能观测到机械臂本体的完整动作（如果动作涉及交互物，则交互动作不能被遮挡），并且摆放好之后尽量别挪动位置

### 4.3. 训练集录包

- 在运行命令行或脚本后的录包操作：

  - 语音播报内容：
    - 开始录制 Y 个 episode 中的 第 X 个 episode：`"Recording episode X of Y"`
    - 重置环境阶段：`"Reset the environment"`

  - 键盘操作：
    - 右箭头键 `→` ：结束当前 episode 并推进到下一个
      - 左箭头键 `←` ：重新录制当前 episode
      - `ESC` 键：结束整个录制过程并根据 `push_to_hub` 参数来保存本地 + 上传 Hub


- 录包脚本 `bash/record_dk1.sh` ：

  - 进入虚拟环境后先给脚本权限：`chmod +x bash/record_dk1.sh`
  - 在终端输入 `./bash/record_dk1.sh` 来启动录包脚本
  - 参数 `--repo_id` 必填，为训练集名称；默认开启续录，当训练集不存在时自动创建
  - 常规使用：`./bash/record_dk1.sh --repo_id $USER/name`
  - 具体参数见脚本注释，主要要按格式填好摄像头配置，然后重置时间 `Reset the environment` 这个注意是重复 `episode` 使用的时间，例如预设的是30s，而实际提前录制完成使用的时间为15s，那 `RESET_TIME_S` 如果设置为10s，那就会再播报 `Reset the environment` 之后会有10s时间来继续遥操作，10s后停止遥操作并给 15s + 10s = 25s 的时间来重置环境；因此 `RESET_TIME_S` 建议设置为0，录制期间就完成动作并复位好机械臂然后按下 `→` 按键进行环境复原，效率会更高

- 录包期间操作：
  - 听语音播报：`Recording episode X` 时开始动作（其中 X 表示为在录第 X 个 episode）
  - 听语音播报：`Reset the environment` 时恢复场景
  - `←` 按键：重新录制当前 episode
  - `→` 按键：提前录制完成当前 episode ，可以开始场景恢复以准备下一个 episode
  - `ESC` 按键：终止录制，终止后小机械臂可以随便动，达妙机械臂会保持力矩在原位姿
- 终止录制后，终端输入 `python scripts/reset.py` 来让达妙机械臂复位并失能

## 5. 模型训练（train_dk1.sh）

录制好训练集后即可开始训练，运行脚本：

```bash
./bash/train_dk1.sh --policy_repo_id 模型名称 --dataset_repo_id 训练集名称
```

训练启动后会进入后台进行训练，训练进度在当前目录下生成的 `训练集名称.log` 中实时打印，第一次训练会有进度条不停打印在下载前置，正式开始训练时会有以下内容，每200步会打印一次进度：

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

训练好的模型会在当前文件夹的 `output` 文件夹下，如果需要终止训练，则见 `训练集名称.param` 里的训练集PID：`TRAIN_PID=pid` ，在终端中输入 `kill pid` 即可停止终止训练，此时 `output` 文件夹里只会有训练了一半的模型（根据步数，在40000步情况下会有 20000 和 40000 两个步数时的模型）

其他参数需要修改的见脚本注释

## 6. 模型评估（eval_dk1.sh）

训练好的模型可以运行脚本来评估：

```bash
./bash/eval_dk1.sh --policy_repo_id 模型名称
```

注意该评估脚本只能直接运行训练脚本训练出来的模型，如果想运行其他脚本，先上传到 Hub，然后输入：

```bash
# ./eval_dk1.sh --policy_repo_id 模型名称 --from_hub
```

评估也相当于一次录制，录制的结果会和训练集存在一起并加上前缀 `eval_`

## 7. 双臂操作

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

## 8. URDF 模型（可视化 / ROS 集成）

仓库的 README 指向了一个高质量的 URDF（由社区贡献）链接，用于：

- 在 ROS / RViz 中可视化机械臂模型
- 用于模拟训练和轨迹规划

（链接在 README 中可直接访问）([GitHub](https://github.com/robot-learning-co/trlc-dk1/tree/main?tab=readme-ov-file))

## 9. 注意事项

### 9.1. 摄像头编号

不同 Ubuntu 机器摄像头编号可能不一样，在命令行中指定正确的 `--robot.cameras` 索引。

### 9.2. 关节速度缩放

```
--robot.joint_velocity_scaling=0.2
```

确保安全后再提升到 1.0([docs.robot-learning.co](https://docs.robot-learning.co/getting_started?utm_source=chatgpt.com))

### 9.3. USB 权限

没有正确的 udev 权限可能导致串口无法打开

