#ifndef _MULTI_ARM_CONTROLLER_H_
#define _MULTI_ARM_CONTROLLER_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace pr2_arm
{
    // ! ========================= 接 口 变 量 声 明 ========================= ! //

    /**
     * @brief 机械臂信息结构体
     * @details 定义了机械臂的相关信息，包括：
     *          - name: 机械臂规划组名称
     *          - ee_link: 机械臂末端执行器链接名称
     *          - base_link: 机械臂基座链接名称
     *          - jmg: 机械臂关节模型组指针
     */
    typedef struct{
        std::string name;
        std::string ee_link;
        std::string base_link;
        const moveit::core::JointModelGroup* jmg;
    } ArmGroup_t;

    // ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

    /**
     * @brief 多机械臂控制器类
     * @details 提供对多机械臂的运动规划与控制功能，包括：
     *          - 机械臂注册
     *          - 目标位姿设置
     *          - 运动规划与执行
     *          - 当前状态同步
     *          - 预定义零位姿复位
     *          - 当前状态查询
     */
    class MultiArmController{
    public:
        MultiArmController(const moveit::core::RobotModelPtr& model, const std::string& multi_group);
        ~MultiArmController();

        void registerArm(const std::string& name, const std::string& ee_link, const std::string& base_link);
        
        bool setGoalPoseForArm(const std::string& name, const geometry_msgs::Pose& tgt_pose);
        bool setGoalPoseForMultiArms(const std::vector<geometry_msgs::Pose>& tgt_pose_group);
        void resetGoalState();

        bool plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
        bool execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
        void syncCurState();

        bool moveToPoseForArm(const std::string& name, const geometry_msgs::Pose& tgt_pose);
        bool moveToPoseForMultiArms(const std::vector<geometry_msgs::Pose>& tgt_pose_group);
        bool stretch(const std::string& name, double dist);
        bool rotate(const std::string& name, double angle);

        bool resetArmToZero(const std::string& name);
        bool resetMultiArmsToZero();

        geometry_msgs::Pose getCurPose(const std::string& name) const;
        std::vector<double> getCurJointValues(const std::string& name) const;

    private:
        /// @brief 机器人模型指针
        moveit::core::RobotModelPtr _robot_model_;
        /// @brief 当前机器人状态
        moveit::core::RobotState _current_state_;
        /// @brief 目标机器人状态
        moveit::core::RobotState _goal_state_;
        /// @brief 机械臂信息数组
        std::vector<ArmGroup_t> _arm_group_;
        /// @brief 多机械臂关节模型组指针
        const moveit::core::JointModelGroup* _multi_jmg_ = nullptr;
        /// @brief MoveIt! 运动规划接口
        moveit::planning_interface::MoveGroupInterface _move_group_;
        /// @brief 规划场景监视器指针
        planning_scene_monitor::PlanningSceneMonitorPtr _scene_monitor_;
        /// @brief 规划场景指针
        planning_scene::PlanningScenePtr _planning_scene_;
    };

    // ! ========================= 模 版 实 现 ========================= ! //



}

#endif
