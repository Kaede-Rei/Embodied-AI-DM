#ifndef _MULTI_ARM_CONTROLLER_H_
#define _MULTI_ARM_CONTROLLER_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace pr2_arm
{
    // ! ========================= 接 口 变 量 声 明 ========================= ! //

    typedef struct{
        std::string name;
        std::string ee_link;
        const moveit::core::JointModelGroup* jmg;
    } ArmGroup_t;

    // ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

    class MultiArmController{
    public:
        MultiArmController(const moveit::core::RobotModelPtr& model, const std::string& multi_group);
        ~MultiArmController();

        void addArmGroup(const std::string& name, const std::string& ee_link);
        bool applyPoseToOneArm(const std::string& name, const geometry_msgs::Pose& target_pose);
        bool applyPoseToMultiArms(const std::vector<geometry_msgs::Pose>& target_pose_group);
        bool plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
        bool execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
        void syncExecution();
        void resetGoalState();
        bool setPoseToMultiArms(const std::vector<geometry_msgs::Pose>& target_pose_group);

    private:
        moveit::core::RobotModelPtr _robot_model_;
        moveit::core::RobotState _current_state_;
        moveit::core::RobotState _goal_state_;

        std::vector<ArmGroup_t> _arm_groups_;
        const moveit::core::JointModelGroup* _multi_jmg_;

        planning_scene_monitor::PlanningSceneMonitorPtr _scene_monitor_;
        planning_scene::PlanningScenePtr _planning_scene_;
    };

    // ! ========================= 模 版 实 现 ========================= ! //



}

#endif
