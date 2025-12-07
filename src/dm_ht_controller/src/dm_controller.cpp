#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "dm_ht_controller/dm_hardware_interface.h"
#include <signal.h>

bool g_quit = false;

void quitRequested(int sig)
{
    g_quit = true;
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dm_control_node");
    ros::NodeHandle nh;

    // 设置信号处理
    signal(SIGINT, quitRequested);
    signal(SIGTERM, quitRequested);

    // 创建硬件接口
    DMHardwareInterface robot(nh);

    if(!robot.init()){
        ROS_ERROR("Failed to initialize robot hardware interface");
        return -1;
    }

    // 创建控制器管理器
    controller_manager::ControllerManager cm(&robot, nh);

    // 获取控制频率
    double control_frequency;
    nh.param<double>("control_frequency", control_frequency, 500.0);
    ros::Rate rate(control_frequency);

    ROS_INFO("Starting control loop at %.1f Hz", control_frequency);

    // 异步spinner用于处理service calls等
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time last_time = ros::Time::now();

    while(ros::ok() && !g_quit){
        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - last_time;
        last_time = current_time;
        
        robot.read();
        cm.update(current_time, period);
        robot.write();

        rate.sleep();
    }

    spinner.stop();
    ROS_INFO("Control node shutting down");

    return 0;
}
