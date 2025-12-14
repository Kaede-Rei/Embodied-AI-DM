#ifndef _eef_cmd_hpp_
#define _eef_cmd_hpp_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_ros/transform_listener.h>

namespace dm_arm
{
    // ! ========================= Typedef / 量 定 义 ========================= ! //

    /**
     * @brief 任务目标动作枚举类型
     * @details 定义了任务目标可能的动作类型，包括：
     *          - NONE: 无动作
     *          - PICK: 抓取动作
     *          - STRETCH: 伸缩动作
     *          - ROTATE: 旋转动作
     */
    enum class TargetAction_e{
        /// @brief 仅移动到目标位置，无其他动作(默认)
        NONE = 0,
        /// @brief 执行抓取动作
        PICK,
        /// @brief 执行伸缩动作
        STRETCH,
        /// @brief 执行旋转动作
        ROTATE
    };

    /**
     * @brief 任务目标结构体
     * @details 定义了任务目标的相关信息，包括：
     *          - pose: 目标位姿
     *          - wait_time: 到达目标后的等待时间
     *          - action: 目标动作类型
     *          - param1: 预留参数1
     */
    typedef struct{
        /// @brief 目标位姿
        geometry_msgs::Pose pose;
        /// @brief 到达目标后的等待时间（单位：秒）
        double wait_time;
        /// @brief 目标动作类型
        TargetAction_e action;
        /// @brief 预留参数1
        double param1 = 0.0;
    } TaskTarget_t;

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

    // ! ========================= 接 口 A P I 声 明 ========================= ! //

    /**
     * @brief 机械臂末端执行器位姿控制类
     * @details 提供了对机械臂末端执行器进行位姿控制的功能，包括：
     *          - 末端位姿与基坐标系转换
     *          - 逆运动学有效性检查
     *          - A*可达位姿搜索
     *          - 目标位姿设置（基坐标系/末端坐标系）
     *          - 末端伸缩与旋转
     *          - 重置到初始位置
     *          - 获取当前末端位姿与关节位置
     * @warning 必须在ros::init()之后使用
     */
    class EefPoseCmd{
    public:
        EefPoseCmd(ros::NodeHandle& nh, const std::string& plan_group_name);

        void eefTfBase(geometry_msgs::PoseStamped& target_pose_eef, geometry_msgs::PoseStamped& target_pose_base);
        bool isIkValid(const geometry_msgs::Pose& target_pose);

        bool searchReachablePose(geometry_msgs::Pose& target_pose, double step, double radius);
        bool setGoalPoseBase(geometry_msgs::PoseStamped& target_pose, bool allow_tweak = true, bool allow_feedforward = true);
        bool setGoalPoseEef(geometry_msgs::PoseStamped& target_pose, bool allow_tweak = true, bool allow_feedforward = true);
        bool eefStretch(double distance);
        bool eefRotate(double angle);
        void resetToZero(void);

        geometry_msgs::Pose getCurrentEefPose(void);
        std::vector<double> getCurrentJointPose(void);

    private:
        /// @brief 计算位姿之间的欧氏距离
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
    };

    /**
     * @brief 任务组规划类
     * @details 提供了对一组任务目标进行规划与执行的功能，包括：
     *          - 添加任务目标
     *          - 清空任务目标列表
     *          - 执行所有任务目标
     */
    class TaskGroupPlanner{
    public:
        TaskGroupPlanner(EefPoseCmd& eef_cmd);

        void add(const TaskTarget_t& target);
        void clear();
        void executeAll();

    private:
        /// @brief 末端位姿控制对象引用
        EefPoseCmd& _eef_cmd_;
        /// @brief 任务目标列表
        std::vector<TaskTarget_t> _task_list_;
    };

    /**
     * @brief 障碍物管理类
     */
    class Barrier{
    public:

    };

}

#endif
