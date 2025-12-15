#include "dm_arm_service/dm_arm_service.hpp"

#include <tf2/LinearMath/Matrix3x3.h>

// ! ========================= 宏 定 义 ========================= ! //

namespace dm_arm
{

    // ! ========================= 接 口 量 声 明 ========================= ! //



    // ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

    static bool CheckLifterIsNeed(double z, STM32Serial& serialer);

    // ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     * @param plan_group 机械臂规划组名称
     * @param stm32_port STM32串口端口
     * @param stm32_baud STM32串口波特率
     */
    Server::Server(ros::NodeHandle& nh, const std::string& plan_group,
                   const std::string& stm32_port, int stm32_baud)
        : _eef_controller_(nh, plan_group), _task_planner_(_eef_controller_),
        _stm32_serialer_(nh, stm32_port, stm32_baud)
    {
        _srv_eef_cmd_ = nh.advertiseService("/dm_arm_server/eef_cmd", &Server::eefPoseCmdCallback, this);
        _srv_task_planner_ = nh.advertiseService("/dm_arm_server/task_planner", &Server::taskGroupPlannerCallback, this);

        _eef_controller_.resetToZero();
    }

    /**
     * @brief 末端位姿控制服务回调函数
     * @details 支持的服务有：
     *          - zero：回到零点
     *          - goal_base：设置末端在底座坐标系下的目标位姿
     *          - goal_eef：设置末端在末端坐标系下的目标位姿
     *          - stretch：末端伸缩
     *          - rotate：末端旋转
     *          - get_pose：获取末端相对底座坐标系的位姿
     *          - get_joints：获取各关节角度
     * @param req 服务请求
     * @param res 服务响应
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Server::eefPoseCmdCallback(dm_arm_msgs_srvs::dm_arm_cmd::Request& req,
        dm_arm_msgs_srvs::dm_arm_cmd::Response& res)
    {
        /// @brief 回到零点
        if(req.command == "zero"){
            _eef_controller_.resetToZero();

            res.success = true;
            res.message = "回到零点成功";
        }

        /// @brief 设置目标位姿
        else if(req.command == "goal_base" || req.command == "goal_eef"){
            geometry_msgs::PoseStamped target_pose;
            tf2::Quaternion qtn;

            qtn.setRPY(req.roll, req.pitch, req.yaw);
            qtn.normalize();

            target_pose.pose.position.x = req.x;
            target_pose.pose.position.y = req.y;
            target_pose.pose.position.z = req.z;
            target_pose.pose.orientation.x = qtn.x();
            target_pose.pose.orientation.y = qtn.y();
            target_pose.pose.orientation.z = qtn.z();
            target_pose.pose.orientation.w = qtn.w();

            bool success = false;
            if(req.command == "goal_base"){
                bool lifter_success = dm_arm::CheckLifterIsNeed(target_pose.pose.position.z, _stm32_serialer_);
                if(!lifter_success){
                    res.success = false;
                    res.message = "升降台升降失败";
                    return true;
                }

                success = _eef_controller_.setGoalPoseBase(target_pose, true, true);
            }
            else if(req.command == "goal_eef"){
                geometry_msgs::PoseStamped target_pose_base;
                _eef_controller_.eefTfBase(target_pose, target_pose_base);

                bool lifter_success = dm_arm::CheckLifterIsNeed(target_pose_base.pose.position.z, _stm32_serialer_);
                if(!lifter_success){
                    res.success = false;
                    res.message = "升降台升降失败";
                    return true;
                }

                success = _eef_controller_.setGoalPoseEef(target_pose, true, true);
            }

            res.success = success;
            if(success) res.message = "设置目标位姿成功";
            else res.message = "设置目标位姿失败";
        }

        /// @brief 末端伸缩
        else if(req.command == "stretch"){
            bool success = _eef_controller_.eefStretch(std::stod(req.param1));

            res.success = success;
            if(success) res.message = "末端伸缩成功";
            else res.message = "末端伸缩失败";
        }

        /// @brief 末端旋转
        else if(req.command == "rotate"){
            bool success = _eef_controller_.eefRotate(std::stod(req.param1));

            res.success = success;
            if(success) res.message = "末端旋转成功";
            else res.message = "末端旋转失败";
        }

        /// @brief 获取末端相对底座坐标系的位姿
        else if(req.command == "get_pose"){
            geometry_msgs::Pose cur_pose = _eef_controller_.getCurrentEefPose();
            tf2::Quaternion qtn(cur_pose.orientation.x,
                cur_pose.orientation.y,
                cur_pose.orientation.z,
                cur_pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

            res.cur_x = cur_pose.position.x; res.cur_y = cur_pose.position.y; res.cur_z = cur_pose.position.z;
            res.cur_roll = roll; res.cur_pitch = pitch; res.cur_yaw = yaw;
            res.message = "获取末端位姿成功";
            res.success = true;
        }

        /// @brief 获取各关节角度
        else if(req.command == "get_joints"){
            res.cur_joint = _eef_controller_.getCurrentJointPose();

            res.message = "获取各关节角度成功";
            res.success = true;
        }

        /// @brief 未知命令
        else{
            res.success = false;
            res.message = "未知命令";
        }

        return true;
    }

    /**
     * @brief 任务组规划服务回调函数
     * @details 支持的服务有：
     *          - add_task：添加任务
     *          - clear_tasks：清除所有任务
     *          - exe_all_tasks：执行所有任务
     * @param req 服务请求
     * @param res 服务响应
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Server::taskGroupPlannerCallback(dm_arm_msgs_srvs::dm_arm_cmd::Request& req,
        dm_arm_msgs_srvs::dm_arm_cmd::Response& res)
    {
        /// @brief 添加任务
        if(req.command == "add_task"){
            geometry_msgs::Pose target_pose;
            tf2::Quaternion qtn;

            qtn.setRPY(req.roll, req.pitch, req.yaw);
            qtn.normalize();

            target_pose.position.x = req.x;
            target_pose.position.y = req.y;
            target_pose.position.z = req.z;
            target_pose.orientation.x = qtn.x();
            target_pose.orientation.y = qtn.y();
            target_pose.orientation.z = qtn.z();
            target_pose.orientation.w = qtn.w();

            TaskTarget_t target;
            target.pose = target_pose;
            target.wait_time = req.param1.empty() ? 0.0 : std::stod(req.param1);
            if(req.param2 == "NONE") target.action = TargetAction_e::NONE;
            else if(req.param2 == "PICK") target.action = TargetAction_e::PICK;
            else if(req.param2 == "STRETCH") target.action = TargetAction_e::STRETCH;
            else if(req.param2 == "ROTATE") target.action = TargetAction_e::ROTATE;
            target.param1 = req.param3.empty() ? 0.0 : std::stod(req.param3);

            _task_planner_.add(target);

            res.success = true;
            res.message = "添加任务成功";
        }

        /// @brief 清除所有任务
        else if(req.command == "clear_tasks"){
            _task_planner_.clear();

            res.success = true;
            res.message = "清除任务成功";
        }

        /// @brief 执行所有任务
        else if(req.command == "exe_all_tasks"){
            _task_planner_.executeAll();

            res.success = true;
            res.message = "开始执行所有任务";
        }

        return true;
    }

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    Client::Client(ros::NodeHandle& nh) : _nh_(nh)
    {
        // 等待服务器可用
        ROS_INFO("等待机械臂服务...");
        ros::service::waitForService("dm_arm_server/eef_cmd");

        // 创建客户端
        _client_eef_cmd_ = _nh_.serviceClient<dm_arm_msgs_srvs::dm_arm_cmd>("dm_arm_server/eef_cmd");

        ROS_INFO("机械臂服务已连接");
    }

    /**
     * @brief 发送机械臂命令
     * @param srv 服务对象
     * @param req 服务请求
     * @param res 服务响应
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::sendCmd(ros::ServiceClient& client,
        const dm_arm_msgs_srvs::dm_arm_cmd::Request& req,
        dm_arm_msgs_srvs::dm_arm_cmd::Response& res)
    {
        dm_arm_msgs_srvs::dm_arm_cmd srv;
        srv.request = req;

        if(client.call(srv)){
            res = srv.response;

            if(!res.success){
                ROS_ERROR("机械臂服务调用失败：%s", res.message.c_str());
            }

            return true;
        }
        else{
            ROS_ERROR("无法请求机械臂服务");
            return false;
        }
    }

    /**
     * @brief 回到零点
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::zero(void)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "zero";

        return sendCmd(_client_eef_cmd_, req, res);
    }

    /**
     * @brief 回到零点
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::zero(std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "zero";

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 设置末端在底座坐标系下的目标位姿
     * @param x 目标x轴位置（单位：米）
     * @param y 目标y轴位置（单位：米）
     * @param z 目标z轴位置（单位：米）
     * @param roll 目标滚转角（单位：弧度）
     * @param pitch 目标俯仰角（单位：弧度）
     * @param yaw 目标偏航角（单位：弧度）
     * @return true:服务调用成功 false:服务调用失败
     * @note RPY均为0时，机械臂启用前馈计算
     */
    bool Client::goalBase(double x, double y, double z, double roll, double pitch, double yaw)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "goal_base";
        req.x = x; req.y = y; req.z = z;
        req.roll = roll; req.pitch = pitch; req.yaw = yaw;

        return sendCmd(_client_eef_cmd_, req, res);
    }

    /**
     * @brief 设置末端在底座坐标系下的目标位姿
     * @param x 目标x轴位置（单位：米）
     * @param y 目标y轴位置（单位：米）
     * @param z 目标z轴位置（单位：米）
     * @param roll 目标滚转角（单位：弧度）
     * @param pitch 目标俯仰角（单位：弧度）
     * @param yaw 目标偏航角（单位：弧度）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     * @note RPY均为0时，机械臂启用前馈计算
     */
    bool Client::goalBase(double x, double y, double z, double roll, double pitch, double yaw, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "goal_base";
        req.x = x; req.y = y; req.z = z;
        req.roll = roll; req.pitch = pitch; req.yaw = yaw;

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 设置末端在末端坐标系下的目标位姿
     * @param x 目标x轴位置（单位：米）
     * @param y 目标y轴位置（单位：米）
     * @param z 目标z轴位置（单位：米）
     * @param roll 目标滚转角（单位：弧度）
     * @param pitch 目标俯仰角（单位：弧度）
     * @param yaw 目标偏航角（单位：弧度）
     * @return true:服务调用成功 false:服务调用失败
     * @note RPY均为0时，机械臂启用前馈计算
     */
    bool Client::goalEef(double x, double y, double z, double roll, double pitch, double yaw)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "goal_eef";
        req.x = x; req.y = y; req.z = z;
        req.roll = roll; req.pitch = pitch; req.yaw = yaw;

        return sendCmd(_client_eef_cmd_, req, res);
    }

    /**
     * @brief 设置末端在末端坐标系下的目标位姿
     * @param x 目标x轴位置（单位：米）
     * @param y 目标y轴位置（单位：米）
     * @param z 目标z轴位置（单位：米）
     * @param roll 目标滚转角（单位：弧度）
     * @param pitch 目标俯仰角（单位：弧度）
     * @param yaw 目标偏航角（单位：弧度）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     * @note RPY均为0时，机械臂启用前馈计算
     */
    bool Client::goalEef(double x, double y, double z, double roll, double pitch, double yaw, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "goal_eef";
        req.x = x; req.y = y; req.z = z;
        req.roll = roll; req.pitch = pitch; req.yaw = yaw;

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 末端伸缩
     * @param length 伸缩长度（单位：米，正值表示伸出，负值表示缩回）
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::stretch(double length)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "stretch";
        req.param1 = std::to_string(length);

        return sendCmd(_client_eef_cmd_, req, res);
    }

    /**
     * @brief 末端伸缩
     * @param length 伸缩长度（单位：米，正值表示伸出，负值表示缩回）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::stretch(double length, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "stretch";
        req.param1 = std::to_string(length);

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 末端旋转
     * @param angle 旋转角度（单位：弧度，正值表示逆时针旋转，负值表示顺时针旋转）
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::rotate(double angle)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "rotate";
        req.param1 = std::to_string(angle);

        return sendCmd(_client_eef_cmd_, req, res);
    }

    /**
     * @brief 末端旋转
     * @param angle 旋转角度（单位：度，正值表示逆时针旋转，负值表示顺时针旋转）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::rotate(double angle, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "rotate";
        req.param1 = std::to_string(angle);

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 获取末端相对底座坐标系的位姿
     * @param x 末端x轴位置（单位：米）
     * @param y 末端y轴位置（单位：米）
     * @param z 末端z轴位置（单位：米）
     * @param roll 末端滚转角（单位：弧度）
     * @param pitch 末端俯仰角（单位：弧度）
     * @param yaw 末端偏航角（单位：弧度）
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "get_pose";

        if(sendCmd(_client_eef_cmd_, req, res)){
            x = res.cur_x; y = res.cur_y; z = res.cur_z;
            roll = res.cur_roll; pitch = res.cur_pitch; yaw = res.cur_yaw;

            return true;
        }
        else return false;
    }

    /**
     * @brief 获取末端相对底座坐标系的位姿
     * @param x 末端x轴位置（单位：米）
     * @param y 末端y轴位置（单位：米）
     * @param z 末端z轴位置（单位：米）
     * @param roll 末端滚转角（单位：弧度）
     * @param pitch 末端俯仰角（单位：弧度）
     * @param yaw 末端偏航角（单位：弧度）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "get_pose";

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        if(success && res.success){
            x = res.cur_x; y = res.cur_y; z = res.cur_z;
            roll = res.cur_roll; pitch = res.cur_pitch; yaw = res.cur_yaw;

            return true;
        }
        else return false;
    }

    /**
     * @brief 获取各关节角度
     * @param joints 关节角度数组（单位：弧度）
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::getJoints(std::vector<double>& joints)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "get_joints";

        if(sendCmd(_client_eef_cmd_, req, res)){
            joints = res.cur_joint;
            return true;
        }
        else return false;
    }

    /**
     * @brief 获取各关节角度
     * @param joints 关节角度数组（单位：弧度）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::getJoints(std::vector<double>& joints, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "get_joints";

        bool success = sendCmd(_client_eef_cmd_, req, res);
        message = res.message;

        if(success && res.success){
            joints = res.cur_joint;
            return true;
        }
        else return false;
    }

    /**
     * @brief 添加任务
     * @param target_pose 目标位姿
     * @param wait_time 到达目标后的等待时间（单位：秒）
     * @param action 目标动作（"NONE"：无动作，"PICK"：抓取，"STRETCH"：伸缩，"ROTATE"：旋转）
     * @param param1 动作参数（伸缩长度或旋转角度，单位：米或弧度）
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "add_task";
        req.x = target_pose.position.x;
        req.y = target_pose.position.y;
        req.z = target_pose.position.z;

        tf2::Quaternion qtn(target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

        req.roll = roll;
        req.pitch = pitch;
        req.yaw = yaw;

        req.param1 = std::to_string(wait_time);
        req.param2 = action;
        req.param3 = std::to_string(param1);

        return sendCmd(_client_task_planner_, req, res);
    }

    /**
     * @brief 添加任务
     * @param target_pose 目标位姿
     * @param wait_time 到达目标后的等待时间（单位：秒）
     * @param action 目标动作（"NONE"：无动作，"PICK"：抓取，"STRETCH"：伸缩，"ROTATE"：旋转）
     * @param param1 动作参数（伸缩长度或旋转角度，单位：米或弧度）
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1, std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "add_task";
        req.x = target_pose.position.x;
        req.y = target_pose.position.y;
        req.z = target_pose.position.z;

        tf2::Quaternion qtn(target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(qtn).getRPY(roll, pitch, yaw);

        req.roll = roll;
        req.pitch = pitch;
        req.yaw = yaw;

        req.param1 = std::to_string(wait_time);
        req.param2 = action;
        req.param3 = std::to_string(param1);

        bool success = sendCmd(_client_task_planner_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 清除所有任务
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::clearTasks(void)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "clear_tasks";

        return sendCmd(_client_task_planner_, req, res);
    }

    /**
     * @brief 清除所有任务
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::clearTasks(std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "clear_tasks";

        bool success = sendCmd(_client_task_planner_, req, res);
        message = res.message;

        return success && res.success;
    }

    /**
     * @brief 执行所有任务
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::exeAllTasks(void)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "exe_all_tasks";

        return sendCmd(_client_task_planner_, req, res);
    }

    /**
     * @brief 执行所有任务
     * @param message 服务响应消息
     * @return true:服务调用成功 false:服务调用失败
     */
    bool Client::exeAllTasks(std::string& message)
    {
        dm_arm_msgs_srvs::dm_arm_cmd::Request req;
        dm_arm_msgs_srvs::dm_arm_cmd::Response res;

        req.command = "exe_all_tasks";

        bool success = sendCmd(_client_task_planner_, req, res);
        message = res.message;

        return success && res.success;
    }

    // ! ========================= 私 有 函 数 实 现 ========================= ! //

    /**
     * @brief 检查是否需要升降台升降
     * @param z 末端在底座坐标系下的目标z轴高度
     * @param serialer STM32串口对象
     */
    static bool CheckLifterIsNeed(double z, STM32Serial& serialer)
    {
        // TODO: z轴太高或太低通知升降台
        if(z > 0.65 || z < -0.15){
            double offset = 0.0f;

            if(z > 0.65) offset = std::round((z - 0.75) * 100.0) / 100.0;
            else offset = std::round((z + 0.25) * 100.0) / 100.0;

            std::string data = "$LIFTER:" + std::to_string(offset) + "#";
            serialer.sendData(data);

            std::string res_data = serialer.rcvdData(1000);
            if(res_data != "$LIFTER:START#") return false;

            res_data = serialer.rcvdData(0);
            if(res_data != "$LIFTER:OK#") return false;
        }

        return true;
    }

}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dm_arm_server");
    ros::NodeHandle nh;

    ROS_INFO("====================================");
    ROS_INFO("   DM Arm 服务器启动中...");
    ROS_INFO("====================================");

    // 获取服务器配置
    int num_threads;
    double timeout_hardware, timeout_moveit;
    bool use_fake_execution;

    nh.param<int>("server/num_threads", num_threads, 12);
    nh.param<double>("server/timeout_hardware", timeout_hardware, 20.0);
    nh.param<double>("server/timeout_moveit", timeout_moveit, 20.0);
    nh.param<bool>("server/use_fake_execution", use_fake_execution, false);

    // 获取 MoveIt 配置
    std::string planning_group;
    nh.param<std::string>("moveit/planning_group", planning_group, "arm");

    // 获取日志级别
    std::string log_level;
    nh.param<std::string>("logging/level", log_level, "info");

    // 获取 STM32 串口配置
    std::string stm32_port;
    int stm32_baud;
    nh.param<std::string>("stm32_serial/port", stm32_port, "/dev/ttyACM1");
    nh.param<int>("stm32_serial/baudrate", stm32_baud, 115200);

    // 设置日志级别
    if(log_level == "debug"){
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
            ros::console::levels::Debug)){
            ros::console::notifyLoggerLevelsChanged();
        }
    }

    ROS_INFO("配置加载完成:");
    ROS_INFO("  * 线程数: %d", num_threads);
    ROS_INFO("  * 规划组: %s", planning_group.c_str());
    ROS_INFO("  * 日志级别: %s", log_level.c_str());

    // 等待硬件接口就绪
    if(!use_fake_execution){
        ROS_INFO("等待硬件接口就绪...");
        auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>(
            "/joint_states", nh, ros::Duration(timeout_hardware)
        );

        if(!msg){
            ROS_WARN("等待机器人状态超时: 未收到 /joint_states 消息");
            ROS_WARN("将使用 fake execution 模式");
            use_fake_execution = true;
        }
        else{
            ROS_INFO("硬件接口已就绪");
        }
    }
    else{
        ROS_INFO("使用 fake execution 模式");
    }

    // 等待 MoveIt 就绪
    ROS_INFO("等待 MoveIt 就绪...");
    bool service_exists = ros::service::waitForService(
        "/move_group/trajectory_execution/set_parameters",
        ros::Duration(timeout_moveit)
    );

    if(!service_exists){
        ROS_ERROR("等待 MoveIt 服务超时！");
        ROS_ERROR("请确保已启动 move_group");
        return 1;
    }
    ROS_INFO("MoveIt 已就绪");

    // 连接 MoveIt 规划组
    try{
        moveit::planning_interface::MoveGroupInterface arm(planning_group);
        ROS_INFO("成功连接到 MoveIt 规划组：%s", arm.getName().c_str());

        ROS_INFO("  * 末端执行器: %s", arm.getEndEffectorLink().c_str());
        ROS_INFO("  * 规划参考坐标系: %s", arm.getPlanningFrame().c_str());

        // 读取并应用 MoveIt 配置
        double velocity_scaling, acceleration_scaling;
        nh.param<double>("end_effector/velocity_scaling", velocity_scaling, 0.3);
        nh.param<double>("end_effector/acceleration_scaling", acceleration_scaling, 0.3);

        arm.setMaxVelocityScalingFactor(velocity_scaling);
        arm.setMaxAccelerationScalingFactor(acceleration_scaling);

        ROS_INFO("  * 速度缩放: %.2f", velocity_scaling);
        ROS_INFO("  * 加速度缩放: %.2f", acceleration_scaling);
    }
    catch(const std::exception& e){
        ROS_ERROR("无法连接到 MoveIt 规划组 '%s': %s",
            planning_group.c_str(), e.what());
        return 1;
    }

    // 启动多线程
    ros::AsyncSpinner spinner(num_threads);
    spinner.start();

    // 初始化服务器

    try{
        dm_arm::Server srv(nh, planning_group, stm32_port, stm32_baud);

        ROS_INFO("====================================");
        ROS_INFO("   DM Arm 服务器已启动");
        ROS_INFO("====================================");
        ROS_INFO("可用的服务：");
        ROS_INFO("  * /dm_arm_server/eef_cmd");
        ROS_INFO("    └─ 末端位姿控制服务");
        ROS_INFO("  * /dm_arm_server/task_planner");
        ROS_INFO("    └─ 任务组规划服务");
        ROS_INFO("====================================");
        ROS_INFO("系统就绪，等待客户端请求...");
        ROS_INFO("====================================");

        ros::waitForShutdown();
    }
    catch(const std::exception& e){
        ROS_ERROR("服务器初始化失败: %s", e.what());
        return 1;
    }

    ROS_INFO("DM Arm 服务器已关闭");
    return 0;
}
