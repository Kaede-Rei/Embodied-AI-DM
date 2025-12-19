#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
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

    private:
        /// @brief ROS节点句柄
        ros::NodeHandle _nh_;
        /// @brief 机械臂规划接口
        moveit::planning_interface::MoveGroupInterface _arm_;
        /// @brief 规划参考坐标系(基坐标系)名称
        std::string _plan_frame_;
        /// @brief 末端坐标系名称
        std::string _eef_frame_;
        /// @brief TF缓存
        tf2_ros::Buffer _tf_buffer_;
        /// @brief TF监听器
        tf2_ros::TransformListener _tf_listener_;
        /// @brief 机械臂关节模型组指针
        const robot_state::JointModelGroup* _jmg_ = nullptr;
        /// @brief 最大迭代次数：默认为0时无限制
        int _max_iterations_;
    };

    // ! ========================= 模 版 实 现 ========================= ! //



}

#endif
