[TOC]

# 达妙六轴机械臂使用文档

---

**Damiao 6-DOF Robotic Arm User Manual**
**版本**：1.0.0
**控制系统**：ROS Noetic
**运动规划框架**：MoveIt! + ros_control
**硬件接口**：达妙CAN转USB（XT30(2+2)端接机械臂，XT30接24V电源，Type-C线接电脑/工控机）

## 1. 系统概述

本机械臂采用 6 个达妙系列无刷电机通过 USB-CAN 适配器与电脑/工控机通信。 底层使用自定义`dm_ht_controller` 硬件接口实现 `hardware_interface::RobotHW`，上层通过 MoveIt 的 `move_group` 节点完成运动规划与轨迹执行。

所有启动流程已集成到一个文件，使用命令`roslaunch dm_ht_moveit_config dm_arm.launch`即可完成：

- 硬件接口初始化

- 电机使能
- ros_control 控制器加载
- MoveIt 配置加载
- RViz 可视化与 MotionPlanning 插件启动

## 2. 硬件连接要求

| 项目           | 要求说明                                |
| -------------- | --------------------------------------- |
| 电源           | 24V工业开关电源/可调电源                |
| USB-CAN 适配器 | 使用达妙官方 USB-CAN 盒子               |
| 串口设备       | 通常为 `/dev/ttyACM0` 或 `/dev/ttyUSB0` |

## 3. 软件依赖（ROS Noetic）

```bash
sudo apt update
sudo apt install ros-noetic-moveit ros-noetic-controller-manager \
                 ros-noetic-joint-trajectory-controller \
                 ros-noetic-rviz ros-noetic-ros-controllers
```

## 4. 源码获取与编译

```bash
# 创建工作空间
mkdir -p ~/dm_arm_ws/src
cd ~/dm_arm_ws/src

# 编译
cd ~/dm_arm_ws
catkin_make
source devel/setup.bash
```

## 5. 参数配置（重要！）

启动前请确认以下参数文件已正确修改：

**文件路径**：`dm_ht_controller/config/dm_controller.yaml`

```yaml
# 硬件接口配置
serial_port: "/dev/ttyACM0"		# 串口设备名称
baudrate: 921600				# 波特率
control_frequency: 500.0		# 控制频率(200~500)

# 控制模式
use_mit_mode: true				# 使用MIT模式
kp: 30.0						# 比例项
kd: 1.0							# 微分项

# 安全参数
max_position_change: 0.5		# 最大单次位置移动(米)
enable_write: true				# 允许写入角度(允许控制)

# 手臂关节配置 - 6个旋转关节
joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6

# 电机ID
motor_ids: [1, 2, 3, 4, 5, 6]

# 电机类型：
# 0 - DM4310
# 1 - DM4310_48V
# 2 - DM4340
# 3 - DM4340_48V
motor_types: [0, 0, 0, 0, 0, 0]

# 关节状态控制器
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

# 手臂控制器
arm_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
```

## 6. 一键启动命令

```bash
roslaunch dm_arm dm_arm.launch
```

该 launch 文件会依次完成以下操作：

1. 加载 URDF 到 parameter server
2. 启动真实硬件接口节点 `dm_control_node`
3. 启动 `controller_manager`
4. 加载 `joint_state_controller` 和 `joint_trajectory_controller`
5. 启动 MoveIt `move_group` 节点
6. 启动 RViz 并加载预配置的 MotionPlanning 插件

启动成功后最终在终端会显示：

```
[INFO] DM Hardware Interface initialized with 6 joints
[INFO] Control mode: MIT
[INFO] Starting control loop at 500.0 Hz
You can start planning now!
```

## 7. 在 RViz 中使用 MoveIt

1. RViz 启动后，左侧 MotionPlanning 面板出现
2. Context 标签页选择 `Planning Scene Topic` 为 `/move_group/monitored_planning_scene`
3. 在 Planning 标签页：  
   - Planning Group 选择 `arm`（或 `manipulator`）
   - 拖动交互标记或在 Goal State 中点击 Random Valid 生成目标位姿
   - 点击 Plan 执行规划
   - 点击 Execute 执行运动
