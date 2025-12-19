[TOC]

# 多机械臂控制器使用文档

----

**Multi Arm Controller User Manual**
**版本**：1.0.0
**控制系统**：ROS Noetic
**运动规划框架**：MoveIt! + fake controller
**测试环境**：Rviz

## 1. 系统概览

本包以 PR2 机器人为示例，提供多机械臂的控制接口。多臂部分基于 MoveIt 的 `MoveGroupInterface` 与 `PlanningSceneMonitor`，实现目标位姿设置、规划与执行、状态同步、便捷伸缩与旋转等功能；单臂控制器仅负责 TF 转换和 A* 单臂末端可达姿态搜索算法

- 多臂控制：注册单机械臂加入多机械臂信息结构体vector，用于设置目标位姿、统一规划与执行、状态同步
- 便捷动作：沿末端指向方向伸缩、绕末端指向轴旋转
- 状态查询：当前末端位姿与关节值
- 单臂能力：坐标系转换(eef↔base)、IK 可达性检测、可达姿态搜索

## 2. 依赖与环境

- Ubuntu20.04 + ROS Noetic + MoveIt
- 基于开源的 PR2 机器人的 `robot_description`（URDF），见 `src/pr2_description/urdf/meshes`
- 已生成并可启动的 MoveIt 配置，见 [pr2_moveit_config](../pr2_moveit_config)

构建与运行示例：
```bash
cd ~/multi_arm_ws/src
catkin_init_workspace

cd ..
catkin_make

# 启动 MoveIt + RViz
roslaunch pr2_moveit_config demo.launch

# 运行测试节点
rosrun pr2_arm_controller test_node
```

## 3. 源码结构

```bash
# 源码结构：
multi_arm_ws
├── src
│ 	├── pr2_arm_controller			# 多机械臂控制
│ 	├── pr2_description				# PR2 机器人URDF及模型
│   ├── pr2_moveit_config       	# MoveIt 配置包
│ 	└── pr2_arm_msgs_srvs			# PR2 机械臂自定义消息类型及服务类型
└── README.md                       # 使用文档
```

## 4. 参数服务器配置

以下参数由控制器在运行时读取：
- **`/pr2_arm/reset_pose/left_arm`**: 左臂重置的 named target（例如 `l_zero`）
- **`/pr2_arm/reset_pose/right_arm`**: 右臂重置的 named target（例如 `r_zero`）
- **`/pr2_arm/reset_pose/multi_arm`**: 双臂重置的 named target（例如 `both_zero`）
- **`end_effector/search/max_iterations`**: 单臂可达姿态搜索的最大扩展次数（整数，默认 100）

示例设置：

```bash
rosparam set /pr2_arm/reset_pose/left_arm l_zero
rosparam set /pr2_arm/reset_pose/right_arm r_zero
rosparam set /pr2_arm/reset_pose/multi_arm both_zero
rosparam set end_effector/search/max_iterations 200
```

## 5. API 使用说明

### 5.1. MultiArmController（多臂）

| 功能分类         | 接口 / 方法                                                  | 说明                                  |
| ---------------- | ------------------------------------------------------------ | ------------------------------------- |
| 构造             | `MultiArmController(const moveit::core::RobotModelPtr& model, const std::string& multi_group)` | 基于 `RobotModel` 与多臂规划组初始化  |
| 臂注册           | `registerArm(const std::string& name, const std::string& ee_link, const std::string& base_link)` | 注册单个机械臂，绑定末端与基坐标系    |
| 目标设置（单臂） | `setGoalPoseForArm(const std::string& name, const geometry_msgs::Pose& tgt_pose) -> bool` | 设置指定机械臂的目标位姿              |
| 目标设置（多臂） | `setGoalPoseForMultiArms(const std::vector<geometry_msgs::Pose>& tgt_pose_group) -> bool` | 一次性设置多臂目标位姿                |
| 目标复位         | `resetGoalState()`                                           | 清空当前所有目标状态                  |
| 规划             | `plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) -> bool` | 基于当前目标进行运动规划              |
| 执行             | `execute(const Plan& plan) -> bool`                          | 执行已规划轨迹                        |
| 状态同步         | `syncCurState()`                                             | 从 `PlanningScene` 同步当前机器人状态 |
| 便捷移动（单臂） | `moveToPoseForArm(...) -> bool`                              | 规划并执行指定机械臂的位姿移动        |
| 便捷移动（多臂） | `moveToPoseForMultiArms(...) -> bool`                        | 多臂协同规划与执行                    |
| 便捷动作         | `stretch(const std::string& name, double dist) -> bool`      | 沿自身 Z 轴伸缩，单位：米             |
| 便捷动作         | `rotate(const std::string& name, double angle) -> bool`      | 绕自身 Z 轴旋转，单位：弧度           |
| 位姿重置（单臂） | `resetArm(const std::string& name) -> bool`                  | 机械臂回到参数服务器中配置的初始位姿  |
| 位姿重置（多臂） | `resetMultiArms() -> bool`                                   | 所有机械臂回到初始位姿                |
| 状态查询         | `getCurPose(const std::string& name) -> geometry_msgs::Pose` | 获取指定机械臂当前末端位姿            |
| 状态查询         | `getCurJointValues(const std::string& name) -> std::vector<double>` | 获取指定机械臂当前关节角              |

### 5.2. ArmController（单臂）

| 功能分类 | 接口 / 方法                                                  | 说明                         |
| -------- | ------------------------------------------------------------ | ---------------------------- |
| 构造     | `ArmController(const ros::NodeHandle& nh, const std::string& arm)` | 初始化单臂控制器             |
| 析构     | `~ArmController()`                                           | 资源释放                     |
| 坐标变换 | `eefTfToBase(PoseStamped& eef, PoseStamped& base)`           | 末端坐标系 → 基坐标系        |
| 坐标变换 | `baseTfToeef(PoseStamped& base, PoseStamped& eef)`           | 基坐标系 → 末端坐标系        |
| IK 校验  | `isIkValid(const Pose& target_pose) -> bool`                 | 判断目标位姿是否存在有效逆解 |
| 可达搜索 | `searchReachablePose(Pose& target_pose, double step_deg, double radius_deg) -> bool` | 在姿态邻域内搜索可达位姿     |

> 说明：单臂控制器仅保留基础几何与 IK 功能，不进行规划执行；执行由 `MultiArmController` 负责。

## 6. 示例用法

完整示例见测试文件 `src/pr2_arm_controller/src/test.cpp`。片段示例：

- **注册与查询当前状态**

  ```cpp
  robot_model_loader::RobotModelLoader loader("robot_description");
  auto model = loader.getModel();
  
  pr2_arm::MultiArmController multi_arm(model, "both_arms");
  
  multi_arm.registerArm("left_arm",  "l_wrist_roll_link", "base_link");
  multi_arm.registerArm("right_arm", "r_wrist_roll_link", "base_link");
  
  multi_arm.syncCurState();
  auto cur_left = multi_arm.getCurPose("left_arm");
  ```

- **指定多臂目标位姿**

  ```cpp
  geometry_msgs::Pose left_pose;
  left_pose.position.x = 0.585315;
  left_pose.position.y = 0.188;
  left_pose.position.z = 1.25001;
  left_pose.orientation.x = 0.0;
  left_pose.orientation.y = -0.88793;
  left_pose.orientation.z = 0.0;
  left_pose.orientation.w = 0.459979;
  
  geometry_msgs::Pose right_pose;
  right_pose.position.x = 0.585315;
  right_pose.position.y = -0.188;
  right_pose.position.z = 1.25001;
  right_pose.orientation.x = 0.0;
  right_pose.orientation.y = -0.88793;
  right_pose.orientation.z = 0.0;
  right_pose.orientation.w = 0.459979;
  
  std::vector<geometry_msgs::Pose> tgt_group;
  tgt_group.push_back(left_pose);
  tgt_group.push_back(right_pose);
  
  multi_arm.moveToPoseForMultiArms(tgt_group)
  ```

- **单臂坐标变换与 IK 检测/搜索**

  ```cpp
  pr2_arm::ArmController left_ctrl(nh, "left_arm");
  geometry_msgs::PoseStamped eef_pose_eef, eef_pose_base;
  eef_pose_eef.pose.orientation.w = 1.0;
  eef_pose_eef.pose.position.z = 0.05;
  left_ctrl.eefTfToBase(eef_pose_eef, eef_pose_base);
  geometry_msgs::Pose probe = eef_pose_base.pose;
  bool ok = left_ctrl.isIkValid(probe);
  geometry_msgs::Pose reachable = probe;
  left_ctrl.searchReachablePose(reachable, 5.0, 45.0);
  ```

## 7. 用 MoveIt Setup Assistant 生成配置（基于 PR2 URDF）

以下流程基于 `src/pr2_description/urdf/robot.xml`：

- **创建 description 包**
  - 先create一个功能包用于存放urdf（如`src/pr2_description/`，编译并 `. devel/setup.bash`

- **启动 Setup Assistant**：
  - 命令：`roslaunch moveit_setup_assistant setup_assistant.launch`
  - 在 "Load Files" 中选择 `pr2_description/urdf/robot.xml`
- **配置 Planning Groups**：
  - `left_arm`：选择`Add Kin. Chain`，头链接选`l_shoulder_pan_link`，尾链接选`l_wrist_roll_link`
  - `right_arm`：同理
  - `both_arms`: 将 `left_arm` 与 `right_arm` 作为子组，同时可以将连接上的关节加入关节组用于`getJointValues`
- **设置 End Effectors**：
  - 左臂 EE：`l_wrist_roll_link` + 规划组 `left_gripper` 自行添加
  - 右臂 EE：`r_wrist_roll_link` + 规划组 `right_gripper` 自行添加
- **虚拟关节/参考系**：
  - `world_joint` 作为参考基座（通常固定或与世界对齐）
- **Kinematics/Planning**：
  - 在 Kinematics 中为 `left_arm` 和 `right_arm` 组选择求解器
  - 在 OMPL 规划配置中勾选常用规划器（RRT）
- **保存生成的配置**：
  - 导出到包目录（例如 `pr2_moveit_config` ）
- **验证**：
  - 通过 `roslaunch pr2_moveit_config demo.launch` 启动 RViz，测试规划与执行

## 8. 注意事项

- **命名一致性**：`registerArm()` 的组名与 MoveIt 中的规划组、末端链接名必须一致。
- **坐标系**：`stretch/rotate` 与 IK/查询均以当前 `RobotState` 与链接全局位姿为基础。
- **参数缺失**：重置位姿依赖参数服务器，若缺失不会报错但会提示并以默认值进行。
- **性能与搜索**：`searchReachablePose` 的步长/半径与最大扩展次数会影响搜索效果与耗时。
