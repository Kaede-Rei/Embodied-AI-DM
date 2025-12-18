#include "pr2_arm_controller/multi_arm_controller.hpp"

#include "pr2_arm_controller/arm_controller.hpp"

namespace pr2_arm
{
    // ! ========================= 接 口 变 量 声 明 ========================= ! //



    // ! ========================= 私 有 变 量 / 宏 声 明 ========================= ! //



    // ! ========================= 私 有 类 / 函 数 声 明 ========================= ! //



    // ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

    MultiArmController::MultiArmController(const moveit::core::RobotModelPtr& model, const std::string& multi_group)
        : _robot_model_(model), _current_state_(model), _goal_state_(model)
    {
        _current_state_.setToDefaultValues();
        _multi_jmg_ = _robot_model_->getJointModelGroup(multi_group);

        if(!_multi_jmg_) throw std::runtime_error("没有找到指定的多机械臂规划组：" + multi_group);

        // 初始化场景监控器与规划场景指针
        _scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        _scene_monitor_->startSceneMonitor();
        _scene_monitor_->startWorldGeometryMonitor();
        _scene_monitor_->startStateMonitor();

        ros::Duration(1.0).sleep();
        _planning_scene_ = _scene_monitor_->getPlanningScene();
    }

    MultiArmController::~MultiArmController()
    {

    }

    void MultiArmController::addArmGroup(const std::string& name, const std::string& ee_link)
    {
        ArmGroup_t arm;
        arm.name = name;
        arm.ee_link = ee_link;
        arm.jmg = _robot_model_->getJointModelGroup(name);

        if(!arm.jmg) throw std::runtime_error("没有找到指定的机械臂规划组：" + name);
        if(!_multi_jmg_->isSubgroup(arm.name)) throw std::runtime_error("机械臂规划组 " + name + " 不是多机械臂规划组 " + _multi_jmg_->getName() + " 的子组");

        _arm_groups_.push_back(arm);
    }

    bool MultiArmController::applyPoseToOneArm(const std::string& name, const geometry_msgs::Pose& target_pose)
    {
        auto it = std::find_if(_arm_groups_.begin(), _arm_groups_.end(),
            [&name](const ArmGroup_t& arm){ return arm.name == name; });
        if(it == _arm_groups_.end()) return false;

        _goal_state_ = _current_state_;
        if(!_goal_state_.setFromIK(it->jmg, target_pose, it->ee_link, 0.0)){
            _goal_state_ = _current_state_;
            ROS_WARN("机械臂 %s 无法达到目标位姿！", it->name.c_str());
            return false;
        }

        return true;
    }

    bool MultiArmController::applyPoseToMultiArms(const std::vector<geometry_msgs::Pose>& target_pose_group)
    {
        if(target_pose_group.size() != _arm_groups_.size()) return false;

        _goal_state_ = _current_state_;
        for(size_t i = 0; i < target_pose_group.size(); ++i){
            const auto& arm = _arm_groups_[i];
            if(!_goal_state_.setFromIK(arm.jmg, target_pose_group[i], arm.ee_link, 0.0)){
                _goal_state_ = _current_state_;
                ROS_WARN("机械臂 %s 无法达到目标位姿！", arm.name.c_str());
                return false;
            }
        }

        return true;
    }

    bool MultiArmController::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        moveit::planning_interface::MoveGroupInterface move_group(_multi_jmg_->getName());

        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget(_goal_state_);
        return (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    bool MultiArmController::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        moveit::planning_interface::MoveGroupInterface move_group(_multi_jmg_->getName());

        return (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    void MultiArmController::syncExecution()
    {
        _current_state_ = _scene_monitor_->getPlanningScene()->getCurrentState();
    }

    void MultiArmController::resetGoalState()
    {
        _goal_state_ = _current_state_;
    }

    bool MultiArmController::setPoseToMultiArms(const std::vector<geometry_msgs::Pose>& target_pose_group)
    {
        moveit::planning_interface::MoveGroupInterface::Plan the_plan;

        if(!applyPoseToMultiArms(target_pose_group)){
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
        syncExecution();

        return true;
    }

    // ! ========================= 私 有 类 / 函 数 实 现 ========================= ! //



}


