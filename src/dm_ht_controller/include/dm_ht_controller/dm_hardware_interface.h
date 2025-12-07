#ifndef DM_HARDWARE_INTERFACE_H
#define DM_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "dm_ht_controller/damiao.h"
#include <vector>
#include <memory>
#include <string>

class DMHardwareInterface : public hardware_interface::RobotHW{
public:
    DMHardwareInterface(ros::NodeHandle& nh);
    ~DMHardwareInterface();

    bool init();
    void read();
    void write();

private:
    ros::NodeHandle nh_;

    // 达妙电机控制器
    std::shared_ptr<damiao::Motor_Control> motor_controller_;

    // 电机对象列表
    std::vector<std::shared_ptr<damiao::Motor>> motors_;

    // 配置参数
    std::string serial_port_;
    int baudrate_;
    std::vector<std::string> joint_names_;
    std::vector<int> motor_ids_;
    std::vector<int> motor_types_;  // 对应damiao::DM_Motor_Type

    // 关节数据
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;

    // 上一次的命令位置（用于计算速度）
    std::vector<double> joint_position_command_prev_;

    // ros_control接口
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // 控制参数
    double control_frequency_;
    bool use_mit_mode_;  // true: MIT模式, false: 位置速度模式
    double kp_;  // MIT模式的比例增益
    double kd_;  // MIT模式的微分增益

    // 安全限制
    double max_position_change_;  // 单次最大位置变化
};

#endif
