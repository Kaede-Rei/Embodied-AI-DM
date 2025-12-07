#include "dm_ht_controller/dm_hardware_interface.h"

DMHardwareInterface::DMHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh),
    control_frequency_(1000.0),
    use_mit_mode_(true),
    kp_(30.0),
    kd_(1.0),
    max_position_change_(0.5)
{ }

DMHardwareInterface::~DMHardwareInterface()
{
    // 失能所有电机
    for(auto& motor : motors_){
        try{
            motor_controller_->disable(*motor);
        }
        catch(const std::exception& e){
            ROS_ERROR("Error disabling motor: %s", e.what());
        }
    }
}

bool DMHardwareInterface::init()
{
    // 读取参数
    nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyACM0");
    nh_.param<int>("baudrate", baudrate_, 921600);
    nh_.param<double>("control_frequency", control_frequency_, 1000.0);
    nh_.param<bool>("use_mit_mode", use_mit_mode_, true);
    nh_.param<double>("kp", kp_, 30.0);
    nh_.param<double>("kd", kd_, 1.0);
    nh_.param<double>("max_position_change", max_position_change_, 0.5);

    if(!nh_.getParam("joint_names", joint_names_)){
        ROS_ERROR("Failed to get joint_names parameter");
        return false;
    }

    if(!nh_.getParam("motor_ids", motor_ids_)){
        ROS_ERROR("Failed to get motor_ids parameter");
        return false;
    }

    if(!nh_.getParam("motor_types", motor_types_)){
        ROS_ERROR("Failed to get motor_types parameter");
        return false;
    }

    if(joint_names_.size() != motor_ids_.size() ||
        joint_names_.size() != motor_types_.size()){
        ROS_ERROR("Size mismatch: joint_names, motor_ids, and motor_types must have the same length");
        return false;
    }

    int num_joints = joint_names_.size();

    // 初始化数据容器
    joint_position_.resize(num_joints, 0.0);
    joint_velocity_.resize(num_joints, 0.0);
    joint_effort_.resize(num_joints, 0.0);
    joint_position_command_.resize(num_joints, 0.0);
    joint_position_command_prev_.resize(num_joints, 0.0);

    // 创建串口对象
    try{
        auto serial = std::make_shared<SerialPort>(serial_port_, baudrate_);
        motor_controller_ = std::make_shared<damiao::Motor_Control>(serial);
        ROS_INFO("Serial port %s opened successfully", serial_port_.c_str());
    }
    catch(const std::exception& e){
        ROS_ERROR("Failed to open serial port: %s", e.what());
        return false;
    }

    // 创建电机对象并使能
    for(size_t i = 0; i < num_joints; ++i){
        // 创建电机对象
        auto motor_type = static_cast<damiao::DM_Motor_Type>(motor_types_[i]);
        auto motor = std::make_shared<damiao::Motor>(
            motor_type,
            motor_ids_[i],  // Slave ID (电机ID)
            0x00           // Master ID
        );

        motors_.push_back(motor);

        // 将电机添加到控制器
        motor_controller_->addMotor(motor.get());

        ROS_INFO("Added motor: joint=%s, id=%d, type=%d",
            joint_names_[i].c_str(), motor_ids_[i], motor_types_[i]);
    }

    // 使能所有电机
    for(size_t i = 0; i < motors_.size(); ++i){
        try{
            motor_controller_->enable(*motors_[i]);
            ROS_INFO("Enabled motor %s (id=%d)",
                joint_names_[i].c_str(), motor_ids_[i]);
            usleep(200000);  // 等待200ms
        }
        catch(const std::exception& e){
            ROS_ERROR("Failed to enable motor %s: %s",
                joint_names_[i].c_str(), e.what());
            return false;
        }
    }

    // 切换到合适的控制模式
    if(!use_mit_mode_){
        for(size_t i = 0; i < motors_.size(); ++i){
            try{
                motor_controller_->switchControlMode(*motors_[i], damiao::POS_VEL_MODE);
                ROS_INFO("Switched motor %s to POS_VEL mode", joint_names_[i].c_str());
                usleep(100000);
            }
            catch(const std::exception& e){
                ROS_ERROR("Failed to switch control mode: %s", e.what());
            }
        }
    }

    // 读取初始位置
    usleep(100000);
    read();

    // 将初始位置设为命令位置
    for(size_t i = 0; i < num_joints; ++i){
        joint_position_command_[i] = joint_position_[i];
        joint_position_command_prev_[i] = joint_position_[i];
    }

    // 注册到hardware_interface
    for(size_t i = 0; i < num_joints; ++i){
        // 注册joint_state_interface
        hardware_interface::JointStateHandle state_handle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]
        );
        joint_state_interface_.registerHandle(state_handle);

        // 注册position_joint_interface
        hardware_interface::JointHandle position_handle(
            joint_state_interface_.getHandle(joint_names_[i]),
            &joint_position_command_[i]
        );
        position_joint_interface_.registerHandle(position_handle);
    }

    // 注册接口
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    ROS_INFO("DM Hardware Interface initialized with %zu joints", num_joints);
    ROS_INFO("Control mode: %s", use_mit_mode_ ? "MIT" : "POS_VEL");

    return true;
}

void DMHardwareInterface::read()
{
    // 从电机读取状态
    for(size_t i = 0; i < motors_.size(); ++i){
        joint_position_[i] = motors_[i]->Get_Position();
        joint_velocity_[i] = motors_[i]->Get_Velocity();
        joint_effort_[i] = motors_[i]->Get_tau();
    }
}

void DMHardwareInterface::write()
{
    double dt = 1.0 / control_frequency_;

    for(size_t i = 0; i < motors_.size(); ++i){
        // 安全限制：检查位置变化是否过大
        double position_change = joint_position_command_[i] - joint_position_command_prev_[i];
        if(std::abs(position_change) > max_position_change_){
            ROS_WARN_THROTTLE(1.0, "Joint %s: Position change too large (%.3f rad), limiting to %.3f rad",
                joint_names_[i].c_str(), position_change, max_position_change_);
            position_change = std::copysign(max_position_change_, position_change);
            joint_position_command_[i] = joint_position_command_prev_[i] + position_change;
        }

        try{
            if(use_mit_mode_){
                // MIT模式：位置+速度+kp+kd控制
                double target_velocity = position_change / dt;

                motor_controller_->control_mit(
                    *motors_[i],
                    kp_,                          // kp
                    kd_,                          // kd
                    joint_position_command_[i],   // 目标位置
                    target_velocity,              // 目标速度
                    0.0f                          // 前馈力矩
                );
            }
            else{
                // 位置速度模式
                double target_velocity = position_change / dt;

                motor_controller_->control_pos_vel(
                    *motors_[i],
                    joint_position_command_[i],   // 目标位置
                    target_velocity               // 目标速度
                );
            }
        }
        catch(const std::exception& e){
            ROS_ERROR_THROTTLE(1.0, "Error controlling motor %s: %s",
                joint_names_[i].c_str(), e.what());
        }

        // 更新上一次的命令
        joint_position_command_prev_[i] = joint_position_command_[i];
    }
}
