#include "pr2_arm_controller/arm_controller.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace pr2_arm
{
    // ! ========================= 接 口 变 量 声 明 ========================= ! //



    // ! ========================= 私 有 变 量 / 宏 声 明 ========================= ! //



    // ! ========================= 私 有 函 数 声 明 ========================= ! //



    // ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

    /**
     * @brief 机械臂末端执行器位姿控制类构造函数
     * @param nh ROS节点句柄
     * @param plan_group_name 机械臂规划组名称
     */
    ArmController::ArmController(const ros::NodeHandle& nh, const std::string& plan_group_name)
        : _nh_(nh), _arm_(plan_group_name), _tf_listener_(_tf_buffer_)
    {
        // 获取规划参考坐标系与末端坐标系
        _plan_frame_ = _arm_.getPlanningFrame();
        _eef_frame_ = _arm_.getEndEffectorLink();
        ROS_INFO_STREAM("规划坐标系为：" << _plan_frame_);
        ROS_INFO_STREAM("末端执行器坐标系为：" << _eef_frame_);

        // 读取搜索最大迭代次数
        _nh_.param<int>("end_effector/search/max_iterations", _max_iterations_, 0);

        // 获取关节模型组指针
        _jmg_ = _arm_.getCurrentState()->getJointModelGroup(plan_group_name);
    }

    /**
     * @brief 机械臂末端执行器位姿控制类析构函数
     */
    ArmController::~ArmController()
    {

    }

    /**
     * @brief 将末端执行器位姿从eef坐标系转换到base坐标系
     * @param[in] target_pose_eef 目标位姿（eef坐标系）
     * @param[out] target_pose_base 目标位姿（base坐标系）
     */
    void ArmController::eefTfToBase(geometry_msgs::PoseStamped& target_pose_eef, geometry_msgs::PoseStamped& target_pose_base)
    {
        // 获取末端执行器相对于基座的变换矩阵
        geometry_msgs::TransformStamped tf_stamped = _tf_buffer_.lookupTransform(_plan_frame_, _eef_frame_, ros::Time(0), ros::Duration(1.0));
        // 进行坐标系转换
        tf2::doTransform(target_pose_eef, target_pose_base, tf_stamped);
    }

    /**
     * @brief 将末端执行器位姿从base坐标系转换到eef坐标系
     * @param[in] target_pose_base 目标位姿（base坐标系）
     * @param[out] target_pose_eef 目标位姿（eef坐标系）
     */
    void ArmController::baseTfToeef(geometry_msgs::PoseStamped& target_pose_base, geometry_msgs::PoseStamped& target_pose_eef)
    {
        // 获取基座相对于末端执行器的变换矩阵
        geometry_msgs::TransformStamped tf_stamped = _tf_buffer_.lookupTransform(_eef_frame_, _plan_frame_, ros::Time(0), ros::Duration(1.0));
        // 进行坐标系转换
        tf2::doTransform(target_pose_base, target_pose_eef, tf_stamped);
    }

    /**
     * @brief 判断目标位姿是否可达
     * @param target_pose 目标位姿
     * @return 是否可达
     */
    bool ArmController::isIkValid(const geometry_msgs::Pose& target_pose)
    {
        robot_state::RobotState tmp_state(*_arm_.getCurrentState());
        return tmp_state.setFromIK(_jmg_, target_pose, 0.0);
    }

    /**
     * @brief 基坐标系下搜索机械臂可达的目标位姿
     * @param target_pose 目标位姿
     * @param step 搜索步长(单位: 度)
     * @param radius 搜索半径(单位: 度)
     * @return 是否找到可达位姿，找到则返回true，geometry_msgs::Pose类型的target_pose将被更新为可达位姿
     * @note 当RPY均为0时，进行前馈计算
     */
    bool ArmController::searchReachablePose(geometry_msgs::Pose& target_pose, double step, double radius)
    {
        // 先判断一次目标位姿是否可达
        if(isIkValid(target_pose)) return true;

        // 先判断RPY是否均为0，是则进行前馈计算
        bool need_feedforward = (
            std::abs(target_pose.orientation.w - 1.0) < 1e-8 &&
            std::abs(target_pose.orientation.x) < 1e-8 &&
            std::abs(target_pose.orientation.y) < 1e-8 &&
            std::abs(target_pose.orientation.z) < 1e-8
            );

        // 把目标位姿tf为末端坐标系下
        geometry_msgs::Pose target_pose_eef = target_pose;
        geometry_msgs::TransformStamped tf_stamped = _tf_buffer_.lookupTransform(_eef_frame_, _plan_frame_, ros::Time(0), ros::Duration(1.0));
        geometry_msgs::TransformStamped tf_stamped_inv = _tf_buffer_.lookupTransform(_plan_frame_, _eef_frame_, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(target_pose_eef, target_pose_eef, tf_stamped);

        // 将步长和半径从度转换为弧度
        step = step * M_PI / 180.0;
        radius = radius * M_PI / 180.0;

        // 获取目标位姿的欧拉角表示，并根据判断进行前馈计算
        tf2::Quaternion q_orig;
        double roll_orig, pitch_orig, yaw_orig;
        if(need_feedforward){
            // 获取底座到末端连线向外方向单位向量，作为末端Z轴在基坐标系下的表示
            tf2::Vector3 z_axis(
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            );
            if(z_axis.length() < 1e-8){
                ROS_ERROR("目标位姿位置无效，无法进行前馈计算。");
                return false;
            }
            z_axis.normalize();

            // 默认X轴，若平行于Z轴则改用旋转90°，再计算Y轴
            tf2::Vector3 x_axis(1, 0, 0);
            if(std::abs(z_axis.dot(x_axis)) > 0.9999) x_axis = tf2::Vector3(0, 1, 0);
            tf2::Vector3 y_axis = z_axis.cross(x_axis).normalize();

            // 构造旋转矩阵并转换为四元数
            tf2::Matrix3x3 rot_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z()
            );
            tf2::Quaternion q_feed_base;
            rot_matrix.getRotation(q_feed_base);

            // 转换到末端坐标系下得到roll/pitch偏移
            geometry_msgs::Quaternion q_feed_eef = tf2::toMsg(q_feed_base);
            tf2::doTransform(q_feed_eef, q_feed_eef, tf_stamped);
            target_pose_eef.orientation = q_feed_eef;
        }
        tf2::fromMsg(target_pose_eef.orientation, q_orig);
        tf2::Matrix3x3(q_orig).getRPY(roll_orig, pitch_orig, yaw_orig);

        // A*搜索初始化
        const int step_count = static_cast<int>(std::ceil(radius / step));
        auto heuristic = [](double droll, double dpitch){ return std::hypot(droll, dpitch); };

        std::priority_queue<AStarNode_t, std::vector<AStarNode_t>, AstarNodeCmper> open_set;
        AStarNode_t start_node = {0.0, 0.0, 0.0, heuristic(0.0, 0.0), 0.0};
        start_node.f = start_node.g + start_node.h;
        open_set.push(start_node);

        using Key = std::pair<int, int>;
        std::unordered_map<Key, double, PairHash_t> closed_set;
        closed_set[{0, 0}] = 0.0;

        const int dirs[8][2] = {
            {1, 0}, {-1, 0},
            {0, 1}, {0, -1},
            {1, 1}, {-1, -1},
            {1, -1}, {-1, 1}
        };

        // 使用配置的最大迭代次数作为扩展限制，如果配置为0则使用自动计算的限制
        size_t max_expand = _max_iterations_;
        if(max_expand == 0){
            max_expand = static_cast<size_t>((2 * step_count + 1) * (2 * step_count + 1) * 10);
        }
        size_t expand_count = 0;

        // A*搜索过程
        while(!open_set.empty()){
            if(++expand_count > max_expand){
                ROS_ERROR("A*搜索超出最大扩展节点数 (%zu)，终止搜索。", max_expand);
                break;
            }

            // 取出当前代价最小的节点
            AStarNode_t current_node = open_set.top();
            open_set.pop();

            // 检查当前节点对应的位姿是否可达
            tf2::Quaternion q_candidate;
            double new_roll = roll_orig + current_node.droll;
            double new_pitch = pitch_orig + current_node.dpitch;
            q_candidate.setRPY(new_roll, new_pitch, 0.0);

            geometry_msgs::Pose pose_candidate = target_pose_eef;
            pose_candidate.orientation = tf2::toMsg(q_candidate);

            // 转换回底座坐标系下再进行IK检测
            tf2::doTransform(pose_candidate, pose_candidate, tf_stamped_inv);
            if(isIkValid(pose_candidate)){

                // 更新目标位姿为可达位姿
                ROS_INFO("搜索到可达位姿为：roll: %.2f°, pitch: %.2f°；偏移量为：droll: %.2f°, dpitch: %.2f°",
                    new_roll * 180.0 / M_PI, new_pitch * 180.0 / M_PI, current_node.droll * 180.0 / M_PI, current_node.dpitch * 180.0 / M_PI);
                target_pose = pose_candidate;
                return true;
            }

            // 生成相邻节点
            int cur_roll_idx = static_cast<int>(std::round(current_node.droll / step));
            int cur_pitch_idx = static_cast<int>(std::round(current_node.dpitch / step));

            for(auto& dir : dirs){
                int new_roll_idx = cur_roll_idx + dir[0];
                int new_pitch_idx = cur_pitch_idx + dir[1];

                // 生成新节点并限制在搜索半径内并检查是否已访问
                double new_droll = new_roll_idx * step;
                double new_dpitch = new_pitch_idx * step;
                if(std::hypot(new_droll, new_dpitch) > radius + 1e-12) continue;
                Key new_visited {new_roll_idx, new_pitch_idx};

                // 计算代价：目标为原点，启发式估计使用欧几里得距离
                double move_cost = std::hypot(new_droll - current_node.droll, new_dpitch - current_node.dpitch);
                double new_g = current_node.g + move_cost;
                double new_h = heuristic(new_droll, new_dpitch);
                double new_f = new_g + new_h;

                // 检查是否已在闭集且代价更优，否则加入开放集
                auto it = closed_set.find(new_visited);
                if(it != closed_set.end() && it->second <= new_g) continue;
                closed_set[new_visited] = new_g;
                open_set.push({new_droll, new_dpitch, new_g, new_h, new_f});
            }
        }

        // 未找到可达位姿
        ROS_ERROR("未搜索到可达位姿。");
        return false;
    }

    // ! ========================= 私 有 类 / 函 数 实 现 ========================= ! //

}
