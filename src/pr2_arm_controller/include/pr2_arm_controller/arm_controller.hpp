#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_ros/transform_listener.h>

namespace pr2_arm
{
    // ! ========================= 接 口 变 量 声 明 ========================= ! //

    /**
     * @brief A*节点结构体
     * @details 定义了A*算法中节点的相关信息，包括：
     *          - droll: 滚转角度偏差
     *          - dpitch: 俯仰角度偏差
     *          - g: 从起点到当前节点的实际代价
     *          - h: 从当前节点到目标节点的估计代价
     *          - f: 总代价（f = g + h）
     */
    typedef struct{
        double droll, dpitch;
        double g, h, f;
    } AStarNode_t;

    /**
     * @brief Pair哈希结构体
     * @details 用于对std::pair<int, int>进行哈希计算
     * @note 使用示例：size_t hash_value = _tuple_hash(std::tuple<int, int>{roll, pitch});
     */
    typedef struct{
        std::size_t operator()(const std::pair<int, int>& t) const noexcept
        {
            return std::hash<int>()(t.first) ^ (std::hash<int>()(t.second) << 1);
        }
    } PairHash_t;

    /**
     * @brief A*节点比较器结构体
     * @details 用于比较A*节点的f值
     * @note 用于优先队列中节点的比较，按照f值从小到大排序
     */
    struct AstarNodeCmper{
        bool operator()(const AStarNode_t& a, const AStarNode_t& b) const noexcept
        {
            return a.f > b.f;
        }
    };

    // ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

    class ArmController{
    public:
        ArmController(const ros::NodeHandle& nh, const std::string& arm);
        ~ArmController();

        void eefTfToBase(geometry_msgs::PoseStamped& target_pose_eef, geometry_msgs::PoseStamped& target_pose_base);
        void baseTfToeef(geometry_msgs::PoseStamped& target_pose_base, geometry_msgs::PoseStamped& target_pose_eef);
        bool isIkValid(const geometry_msgs::Pose& target_pose);

        bool searchReachablePose(geometry_msgs::Pose& target_pose, double step, double radius);
        bool setTgtPoseBase(geometry_msgs::PoseStamped& target_pose, bool allow_tweak = true, bool allow_feedforward = true);
        bool setTgtPoseEef(geometry_msgs::PoseStamped& target_pose, bool allow_tweak = true, bool allow_feedforward = true);
        bool eefStretch(double dist);
        bool eefRotate(double angle);
        void resetToZero(void);

        geometry_msgs::Pose getCurrentEefPose(void);
        std::vector<double> getCurrentJointPose(void);

    private:
        /// @brief ROS节点句柄
        ros::NodeHandle _nh_;
        /// @brief 机械臂规划接口
        moveit::planning_interface::MoveGroupInterface _arm_;
        /// @brief 规划结果
        moveit::planning_interface::MoveGroupInterface::Plan _plan_;
        /// @brief 规划场景监视器指针
        planning_scene_monitor::PlanningSceneMonitorPtr _scene_monitor_;
        /// @brief 规划场景指针
        planning_scene::PlanningScenePtr _planning_scene_;
        /// @brief 规划参考坐标系(基坐标系)名称
        std::string _plan_frame_;
        /// @brief 末端坐标系名称
        std::string _eef_frame_;
        /// @brief 规划结束执行标志
        bool _is_success_;
        /// @brief TF缓存
        tf2_ros::Buffer _tf_buffer_;
        /// @brief TF监听器
        tf2_ros::TransformListener _tf_listener_;
        /// @brief 机械臂关节模型组指针
        const robot_state::JointModelGroup* _jmg_ = nullptr;
        /// @brief 当前机械臂状态指针
        moveit::core::RobotStatePtr _current_state_ = nullptr;
        /// @brief 最大可达距离
        double _max_reach_;
        /// @brief 最小可达距离
        double _min_reach_;
        /// @brief 最大迭代次数
        int _max_iterations_;
    };

    // ! ========================= 模 版 实 现 ========================= ! //



}

#endif
