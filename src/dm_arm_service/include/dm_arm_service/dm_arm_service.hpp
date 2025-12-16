#ifndef _dm_arm_service_hpp_
#define _dm_arm_service_hpp_

#include <ros/ros.h>

#include "dm_arm_controller/eef_cmd.hpp"
#include "dm_arm_msgs_srvs/dm_arm_cmd.h"
#include "serial_driver/serial_driver.hpp"

namespace dm_arm
{

    // ! ========================= Typedef / 量 定 义 ========================= ! //



    // ! ========================= 接 口 A P I 声 明 ========================= ! //

    class Server{
    public:
        Server(ros::NodeHandle& nh, const std::string& plan_group, 
               const std::string& stm32_port, int stm32_baud);
        bool eefPoseCmdCallback(dm_arm_msgs_srvs::dm_arm_cmd::Request& req,
            dm_arm_msgs_srvs::dm_arm_cmd::Response& res);
        bool taskGroupPlannerCallback(dm_arm_msgs_srvs::dm_arm_cmd::Request& req,
            dm_arm_msgs_srvs::dm_arm_cmd::Response& res);

    private:
        ros::ServiceServer _srv_eef_cmd_;
        ros::ServiceServer _srv_task_planner_;
        dm_arm::EefPoseCmd _eef_controller_;
        dm_arm::TaskGroupPlanner _task_planner_;
        STM32Serial _stm32_serialer_;
        ros::ServiceClient _gripper_client_;
    };

    class Client{
    public:
        Client(ros::NodeHandle& nh);
        bool sendCmd(ros::ServiceClient& client,
            const dm_arm_msgs_srvs::dm_arm_cmd::Request& req, 
            dm_arm_msgs_srvs::dm_arm_cmd::Response& res);

        bool zero(void);
        bool zero(std::string& message);
        bool goalBase(double x, double y, double z, double roll, double pitch, double yaw);
        bool goalBase(double x, double y, double z, double roll, double pitch, double yaw, std::string& message);
        bool goalEef(double x, double y, double z, double roll, double pitch, double yaw);
        bool goalEef(double x, double y, double z, double roll, double pitch, double yaw, std::string& message);
        bool stretch(double length);
        bool stretch(double length, std::string& message);
        bool rotate(double angle);
        bool rotate(double angle, std::string& message);
        bool getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw);
        bool getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw, std::string& message);
        bool getJoints(std::vector<double>& joints);
        bool getJoints(std::vector<double>& joints, std::string& message);

        bool addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1);
        bool addTask(geometry_msgs::Pose target_pose, double wait_time, const std::string& action, double param1, std::string& message);
        bool clearTasks(void);
        bool clearTasks(std::string& message);
        bool exeAllTasks(void);
        bool exeAllTasks(std::string& message);

    private:
        ros::NodeHandle _nh_;
        ros::ServiceClient _client_eef_cmd_;
        ros::ServiceClient _client_task_planner_;
    };
}

#endif
