[TOC]

# 达妙六轴机械臂使用文档

---

**Damiao 6-DOF Robotic Arm User Manual**
**版本**：1.0.0
**控制系统**：ROS Noetic
**运动规划框架**：MoveIt! + ros_control
ur**硬件接口**：达妙CAN转USB（XT30(2+2)端接机械臂，XT30接24V电源，Type-C线接电脑/工控机）

## 1. 系统概述

本机械臂采用 6 个达妙系列无刷电机通过 USB-CAN 适配器与电脑/工控机通信。底层使用自定义 `dm_ht_controller` 硬件接口实现 `hardware_interface::RobotHW`，上层通过 MoveIt `move_group` 规划与 `ros_control` 轨迹执行；业务服务节点由 `dm_arm_service` 提供。

所有启动流程集成到一个文件，使用命令`roslaunch dm_arm_service dm_arm_server.launch`即可完成：

- 统一加载参数服务器内参数
- 加载URDF、启动状态发布器并使能电机（启动硬件接口）
- MoveIt 配置加载并启动服务器
- RViz 可视化与 MotionPlanning 插件启动

该launch可选`use_rviz`(是否使用rviz)和`use_fake_execution`(是否假执行)，后者为true则仅仿真

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

## 4. 源码结构与编译

```bash
# 源码结构：
dm_arm_ws
├── src
│ 	├── dm_arm_controller			# 达妙机械臂控制节点
│ 	├── dm_arm_description			# 达妙机械臂URDF及模型
│   ├── dm_arm_moveit_config        # MoveIt 配置包
│ 	├── dm_arm_msgs_srvs			# 达妙机械臂自定义消息类型及服务类型
│   ├── dm_arm_service              # 机械臂服务节点
│   ├── dm_ht_controller            # 硬件接口节点
│   └──	serial_driver				# 串口及CAN驱动节点
├── README.md                       # 使用文档
├── 达妙机械臂服务器客户端接口文档.md    # 客户端接口说明
└── test_service.sh                 # 测试脚本

# 创建工作空间并编译
cd src
catkin_init_workspace
cd /..
catkin_make
source devel/setup.bash
```

## 5. 参数配置（重要！）

启动前请确认以下参数按实际情况正确修改：

**文件路径**：`dm_arm_service/config/dm_arm_config.yaml`

```yaml
# 硬件接口配置
serial_port: "/dev/ttyACM0"		# 串口设备名称
baudrate: 921600				# 波特率
control_frequency: 500.0		# 控制频率(200~500)

# 控制模式
use_mit_mode: false				# 使用MIT+位置速度混合模式（纯MIT有BUG）

# 安全参数
enable_write: true				# 允许写入角度(允许控制)

# 手臂关节配置
joints:
  names:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
    - gripper_left
  # 电机类型：
  # 0 - DM4310
  # 1 - DM4310_48V
  # 2 - DM4340
  # 3 - DM4340_48V
  motor_ids: [1, 2, 3, 4, 5, 6, 7]     # 电机 ID
  motor_types: [2, 2, 2, 0, 0, 0, 0]   # 电机类型
```

## 6. 一键启动命令及测试脚本

```bash
roslaunch dm_arm_service dm_arm_server.launch
```

该 launch 文件会依次完成以下操作：

1. 加载 URDF 到 parameter server
2. 启动真实硬件接口节点 `dm_ht_controller`
3. 启动 `controller_manager`
4. 加载 `joint_state_controller` 和 `joint_trajectory_controller`
5. 启动 MoveIt `move_group` 节点
6. 启动 RViz 并加载预配置的 MotionPlanning 插件
7. 启动 DM Arm 机械臂控制服务器

启动成功后最终在终端会显示：

```
[INFO] [1765811909.715087159]: ====================================
[INFO] [1765811909.715137026]:    DM Arm 服务器已启动
[INFO] [1765811909.715158397]: ====================================
[INFO] [1765811909.715180956]: 可用的服务：
[INFO] [1765811909.715199953]:   * /dm_arm_server/eef_cmd
[INFO] [1765811909.715220137]:     └─ 末端位姿控制服务
[INFO] [1765811909.715239134]:   * /dm_arm_server/task_planner
[INFO] [1765811909.715258131]:     └─ 任务组规划服务
[INFO] [1765811909.715278315]: ====================================
[INFO] [1765811909.715298499]: 系统就绪，等待客户端请求...
[INFO] [1765811909.715318683]: ====================================
```

测试脚本：`source ./test_service.sh`

## 7. 在 RViz 中使用 MoveIt

1. RViz 启动后，左侧 MotionPlanning 面板出现
2. Context 标签页选择 `Planning Scene Topic` 为 `/move_group/monitored_planning_scene`
3. 在 Planning 标签页：  
   - Planning Group 选择 `arm` 或 `end`
   - 拖动交互标记或在 Goal State 中点击 Random Valid 生成目标位姿
   - 点击 Plan 执行规划
   - 点击 Execute 执行运动
