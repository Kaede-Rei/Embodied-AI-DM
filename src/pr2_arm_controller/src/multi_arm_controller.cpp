#include "pr2_arm_controller/multi_arm_controller.hpp"

#include "pr2_arm_controller/arm_controller.hpp"

namespace pr2_arm
{
    // ! ========================= 接 口 变 量 声 明 ========================= ! //



    // ! ========================= 私 有 变 量 / 宏 声 明 ========================= ! //



    // ! ========================= 私 有 类 / 函 数 声 明 ========================= ! //

    static const ArmGroup_t* FindArm(const std::vector<ArmGroup_t>& arm_group, const std::string& name);

    // ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

    /**
     * @brief 多机械臂控制器类构造函数
     * @param model 机器人模型指针
     * @param multi_group 多机械臂规划组名称
     */
    MultiArmController::MultiArmController(const moveit::core::RobotModelPtr& model, const std::string& multi_group)
        : _robot_model_(model), _current_state_(model), _goal_state_(model), _move_group_(multi_group)
    {
        // 初始化当前状态为默认值，并获取多机械臂规划组指针
        _current_state_.setToDefaultValues();
        _multi_jmg_ = _robot_model_->getJointModelGroup(multi_group);
        if(!_multi_jmg_) throw std::runtime_error("没有找到指定的多机械臂规划组：" + multi_group);

        // 初始化场景监控器
        _scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        _scene_monitor_->startSceneMonitor();
        _scene_monitor_->startWorldGeometryMonitor();
        _scene_monitor_->startStateMonitor();

        // 等待场景监控器初始化完成，初始化规划场景指针
        ros::Duration(1.0).sleep();
        _planning_scene_ = _scene_monitor_->getPlanningScene();

        // 同步当前状态
        syncCurState();

        // 复位到默认位姿
        resetMultiArmsToZero();
    }

    /**
     * @brief 多机械臂控制器类析构函数
     */
    MultiArmController::~MultiArmController()
    {

    }

    /**
     * @brief 注册机械臂
     * @param name 机械臂规划组名称
     * @param ee_link 机械臂末端执行器链接名称
     * @param base_link 机械臂基座链接名称
     */
    void MultiArmController::registerArm(const std::string& name, const std::string& ee_link, const std::string& base_link)
    {
        // 构建机械臂信息并验证
        ArmGroup_t arm;
        arm.name = name;
        arm.ee_link = ee_link;
        arm.base_link = base_link;
        arm.jmg = _robot_model_->getJointModelGroup(name);

        if(!arm.jmg) throw std::runtime_error("没有找到指定的机械臂规划组：" + name);
        if(!_multi_jmg_->isSubgroup(arm.name)) throw std::runtime_error("机械臂规划组 " + name + " 不是多机械臂规划组 " + _multi_jmg_->getName() + " 的子组");

        // 添加到机械臂信息数组
        _arm_group_.push_back(arm);
    }

    /**
     * @brief 为指定机械臂设置目标位姿
     * @param name 机械臂规划组名称
     * @param tgt_pose 目标位姿
     * @return 设置成功返回 true，失败返回 false
     */
    bool MultiArmController::setGoalPoseForArm(const std::string& name, const geometry_msgs::Pose& tgt_pose)
    {
        // 查找对应机械臂
        const auto arm = FindArm(_arm_group_, name);
        if(!arm) return false;

        // IK求解目标位姿
        _goal_state_ = _current_state_;
        if(!_goal_state_.setFromIK(arm->jmg, tgt_pose, arm->ee_link, 0.0)){
            _goal_state_ = _current_state_;
            ROS_WARN("机械臂 %s 无法达到目标位姿！", arm->name.c_str());
            return false;
        }

        return true;
    }

    /**
     * @brief 为多机械臂设置目标位姿
     * @param tgt_pose_group 目标位姿组
     * @return 设置成功返回 true，失败返回 false
     */
    bool MultiArmController::setGoalPoseForMultiArms(const std::vector<geometry_msgs::Pose>& tgt_pose_group)
    {
        // 检查目标位姿数量是否与注册机械臂数量匹配
        if(tgt_pose_group.size() != _arm_group_.size()) return false;

        // 依次为各机械臂IK求解目标位姿
        _goal_state_ = _current_state_;
        for(size_t i = 0; i < tgt_pose_group.size(); ++i){
            const auto& arm = _arm_group_[i];
            if(!_goal_state_.setFromIK(arm.jmg, tgt_pose_group[i], arm.ee_link, 0.0)){
                _goal_state_ = _current_state_;
                ROS_WARN("机械臂 %s 无法达到目标位姿！", arm.name.c_str());
                return false;
            }
        }

        return true;
    }

    /**
     * @brief 重置目标状态为当前状态
     */
    void MultiArmController::resetGoalState()
    {
        _goal_state_ = _current_state_;
    }

    /**
     * @brief 规划从当前状态到目标状态的运动轨迹
     * @param plan 规划结果
     * @return 规划成功返回 true，失败返回 false
     */
    bool MultiArmController::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        // 同步当前状态并设置目标状态，进行规划
        _move_group_.setStartStateToCurrentState();
        _move_group_.setJointValueTarget(_goal_state_);
        return (_move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    /**
     * @brief 执行规划结果
     * @param plan 规划结果
     * @return 执行成功返回 true，失败返回 false
     */
    bool MultiArmController::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        return (_move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    /**
     * @brief 同步当前状态
     */
    void MultiArmController::syncCurState()
    {
        // 从规划场景中同步当前状态并更新
        _current_state_ = _scene_monitor_->getPlanningScene()->getCurrentState();
        _current_state_.update();
    }

    /**
     * @brief 移动指定机械臂到目标位姿
     * @param name 机械臂规划组名称
     * @param tgt_pose 目标位姿
     * @return 移动成功返回 true，失败返回 false
     */
    bool MultiArmController::moveToPoseForArm(const std::string& name, const geometry_msgs::Pose& tgt_pose)
    {
        moveit::planning_interface::MoveGroupInterface::Plan the_plan;

        if(!setGoalPoseForArm(name, tgt_pose)){
            ROS_WARN("为机械臂 %s 设置目标位姿失败。", name.c_str());
            return false;
        }
        if(!plan(the_plan)){
            ROS_WARN("机械臂 %s 规划到目标位姿失败。", name.c_str());
            return false;
        }
        if(!execute(the_plan)){
            ROS_WARN("机械臂 %s 执行到目标位姿失败。", name.c_str());
            return false;
        }
        syncCurState();

        return true;
    }

    /**
     * @brief 移动多机械臂到目标位姿
     * @param tgt_pose_group 目标位姿组
     * @return 移动成功返回 true，失败返回 false
     */
    bool MultiArmController::moveToPoseForMultiArms(const std::vector<geometry_msgs::Pose>& tgt_pose_group)
    {
        moveit::planning_interface::MoveGroupInterface::Plan the_plan;

        if(!setGoalPoseForMultiArms(tgt_pose_group)){
            ROS_WARN("为多机械臂设置目标位姿失败。");
            return false;
        }
        if(!plan(the_plan)){
            ROS_WARN("多机械臂规划到目标位姿失败。");
            return false;
        }
        if(!execute(the_plan)){
            ROS_WARN("多机械臂执行到目标位姿失败。");
            return false;
        }
        syncCurState();

        return true;
    }

    /**
     * @brief 机械臂沿末端执行器指向方向伸缩
     * @param name 机械臂规划组名称
     * @param dist 伸缩距离，单位：米
     * @return 成功返回 true，失败返回 false
     */
    bool MultiArmController::stretch(const std::string& name, double dist)
    {
        // 查找对应机械臂
        const auto arm = FindArm(_arm_group_, name);
        if(!arm) return false;

        // 计算目标位姿
        Eigen::Isometry3d ee_tf = _current_state_.getGlobalLinkTransform(arm->ee_link);
        Eigen::Vector3d dx(dist, 0, 0);
        ee_tf.translation() += ee_tf.rotation() * dx;

        geometry_msgs::Pose tgt;
        tgt.position.x = ee_tf.translation().x();
        tgt.position.y = ee_tf.translation().y();
        tgt.position.z = ee_tf.translation().z();

        Eigen::Quaterniond quat(ee_tf.rotation());
        tgt.orientation.x = quat.x();
        tgt.orientation.y = quat.y();
        tgt.orientation.z = quat.z();
        tgt.orientation.w = quat.w();

        return moveToPoseForArm(name, tgt);
    }

    /**
     * @brief 机械臂绕末端执行器指向方向旋转
     * @param name 机械臂规划组名称
     * @param angle 旋转角度，单位：度
     * @return 成功返回 true，失败返回 false
     */
    bool MultiArmController::rotate(const std::string& name, double angle)
    {
        // 角度转弧度
        angle = angle * M_PI / 180.0;

        // 查找对应机械臂
        const auto arm = FindArm(_arm_group_, name);
        if(!arm) return false;

        // 计算目标位姿
        Eigen::Isometry3d ee_tf = _current_state_.getGlobalLinkTransform(arm->ee_link);
        Eigen::AngleAxisd rot_vec(angle, Eigen::Vector3d::UnitX());
        ee_tf.rotate(rot_vec);

        geometry_msgs::Pose tgt;
        tgt.position.x = ee_tf.translation().x();
        tgt.position.y = ee_tf.translation().y();
        tgt.position.z = ee_tf.translation().z();

        Eigen::Quaterniond quat(ee_tf.rotation());
        tgt.orientation.x = quat.x();
        tgt.orientation.y = quat.y();
        tgt.orientation.z = quat.z();
        tgt.orientation.w = quat.w();

        return moveToPoseForArm(name, tgt);
    }

    /**
     * @brief 重置指定机械臂到预定义零位姿
     * @param name 机械臂规划组名称
     * @return 重置成功返回 true，失败返回 false
     */
    bool MultiArmController::resetArmToZero(const std::string& name)
    {
        // 查找对应机械臂
        const auto arm = FindArm(_arm_group_, name);
        if(!arm) return false;

        // 获取预定义零位姿参数并执行
        ros::NodeHandle nh;
        std::string zero_pose;
        std::string param_key = "/pr2_arm/reset_pose/" + name;
        if(!nh.param<std::string>(param_key, zero_pose, "both_zero")) ROS_WARN("未找到机械臂 %s 的重置位姿参数！使用默认值。", name.c_str());

        _move_group_.setNamedTarget(zero_pose);
        _move_group_.move();
        return true;
    }

    /**
     * @brief 重置多机械臂到预定义零位姿
     * @return 重置成功返回 true，失败返回 false
     */
    bool MultiArmController::resetMultiArmsToZero()
    {
        // 获取预定义零位姿参数并执行
        ros::NodeHandle nh;
        std::string zero_pose;
        std::string param_key = "/pr2_arm/reset_pose/multi_arm";
        if(!nh.param<std::string>(param_key, zero_pose, "both_zero")) ROS_WARN("未找到多机械臂的重置位姿参数！使用默认值。");

        _move_group_.setNamedTarget(zero_pose);
        _move_group_.move();
        return true;
    }

    /**
     * @brief 获取指定机械臂当前末端位姿
     * @param name 机械臂规划组名称
     * @return 机械臂当前末端位姿
     */
    geometry_msgs::Pose MultiArmController::getCurPose(const std::string& name) const
    {
        const auto arm = FindArm(_arm_group_, name);
        if(!arm){
            ROS_WARN("未找到机械臂 %s ！", name.c_str());
            return geometry_msgs::Pose();
        }

        const Eigen::Isometry3d ee_tf = _current_state_.getGlobalLinkTransform(arm->ee_link);
        geometry_msgs::Pose cur_pose;
        cur_pose.position.x = ee_tf.translation().x();
        cur_pose.position.y = ee_tf.translation().y();
        cur_pose.position.z = ee_tf.translation().z();

        Eigen::Quaterniond quat(ee_tf.rotation());
        cur_pose.orientation.x = quat.x();
        cur_pose.orientation.y = quat.y();
        cur_pose.orientation.z = quat.z();
        cur_pose.orientation.w = quat.w();

        return cur_pose;
    }

    /**
     * @brief 获取指定机械臂当前关节值
     * @param name 机械臂规划组名称
     * @return 机械臂当前关节值数组
     */
    std::vector<double> MultiArmController::getCurJointValues(const std::string& name) const
    {
        const auto arm = FindArm(_arm_group_, name);
        if(!arm){
            ROS_WARN("未找到机械臂 %s ！", name.c_str());
            return std::vector<double>();
        }

        std::vector<double> joints;
        _current_state_.copyJointGroupPositions(arm->jmg, joints);
        return joints;
    }

    // ! ========================= 私 有 类 / 函 数 实 现 ========================= ! //

    /**
     * @brief 查找机械臂信息
     * @param arm_group 机械臂信息数组
     * @param name 机械臂规划组名称
     * @return 找到返回对应机械臂信息指针，未找到返回 nullptr
     */
    static const ArmGroup_t* FindArm(const std::vector<ArmGroup_t>& arm_group, const std::string& name)
    {
        for(const auto& arm : arm_group){
            if(arm.name == name) return &arm;
        }
        return nullptr;
    }

}
