#include "dm_ht_controller/dm_hardware_interface.h"

DMHardwareInterface::DMHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh),
    control_frequency_(500.0),
    use_mit_mode_(false),
    kp_(30.0),
    kd_(1.0),
    max_position_change_(0.5),
    max_velocity_(3.0)
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
    nh_.param<double>("control_frequency", control_frequency_, 500.0);
    nh_.param<bool>("use_mit_mode", use_mit_mode_, false);
    nh_.param<double>("kp", kp_, 30.0);
    nh_.param<double>("kd", kd_, 1.0);
    nh_.param<double>("max_position_change", max_position_change_, 0.5);
    nh_.param<double>("max_velocity", max_velocity_, 3.0);

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
    joint_velocity_command_.resize(num_joints, 0.0);
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

    // 创建电机对象
    for(size_t i = 0; i < num_joints; ++i){
        auto motor_type = static_cast<damiao::DM_Motor_Type>(motor_types_[i]);
        auto motor = std::make_shared<damiao::Motor>(
            motor_type,
            motor_ids_[i],
            0x00
        );

        motors_.push_back(motor);
        motor_controller_->addMotor(motor.get());

        ROS_INFO("Added motor: joint=%s, id=%d, type=%d",
            joint_names_[i].c_str(), motor_ids_[i], motor_types_[i]);
    }

    // 使能电机
    for(size_t i = 0; i < motors_.size(); ++i){
        try{
            motor_controller_->enable(*motors_[i]);
            ROS_INFO("Enabled motor %s (id=%d)",
                joint_names_[i].c_str(), motor_ids_[i]);
        }
        catch(const std::exception& e){
            ROS_ERROR("Failed to enable motor %s: %s",
                joint_names_[i].c_str(), e.what());
            return false;
        }
    }

    // 除了 JOINT1 外切换到位置速度模式
    if(!use_mit_mode_){
        for(size_t i = 1; i < motors_.size(); ++i){
            try{
                bool success = motor_controller_->switchControlMode(
                    *motors_[i], damiao::POS_VEL_MODE);

                if(success){
                    ROS_INFO("Motor %s switched to POS_VEL mode",
                        joint_names_[i].c_str());
                }
                else{
                    ROS_WARN("Motor %s may not have switched mode correctly",
                        joint_names_[i].c_str());
                }
                usleep(200 * 1000);
            }
            catch(const std::exception& e){
                ROS_ERROR("Failed to switch control mode for %s: %s",
                    joint_names_[i].c_str(), e.what());
            }
        }
    }

    // 读取初始位置
    usleep(200 * 1000);
    std::vector<double> pos_sum(num_joints, 0.0);
    int read_count = 5;

    for(int j = 0; j < read_count; ++j){
        read();
        for(size_t i = 0; i < num_joints; ++i){
            pos_sum[i] += joint_position_[i];
        }
        usleep(20 * 1000);
    }

    // 计算平均值并设置为初始命令
    for(size_t i = 0; i < num_joints; ++i){
        joint_position_[i] = pos_sum[i] / read_count;
        joint_position_command_[i] = joint_position_[i];
        joint_position_command_prev_[i] = joint_position_[i];
        joint_velocity_command_[i] = 0.0;

        ROS_INFO("Joint %s initial position: %.3f rad",
            joint_names_[i].c_str(), joint_position_[i]);
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
    ROS_INFO("Control mode: %s, Frequency: %.1f Hz",
        use_mit_mode_ ? "MIT" : "POS_VEL", control_frequency_);

    return true;
}

void DMHardwareInterface::read()
{
    // 从电机读取状态
    for(size_t i = 0; i < motors_.size(); ++i){
        try{
            joint_position_[i] = motors_[i]->Get_Position();
            joint_velocity_[i] = motors_[i]->Get_Velocity();
            joint_effort_[i] = motors_[i]->Get_tau();

            // 检测异常值
            if(std::isnan(joint_position_[i]) || std::isinf(joint_position_[i])){
                ROS_WARN_THROTTLE(1.0, "Invalid position reading for joint %s",
                    joint_names_[i].c_str());
                joint_position_[i] = joint_position_command_prev_[i];
            }
        }
        catch(const std::exception& e){
            ROS_ERROR_THROTTLE(1.0, "Error reading motor %s: %s",
                joint_names_[i].c_str(), e.what());
        }
    }
}

void DMHardwareInterface::write()
{
    double dt = 1.0 / control_frequency_;

    for(size_t i = 0; i < motors_.size(); ++i){
        try{
            // 计算位置误差和变化量
            double position_error = joint_position_command_[i] - joint_position_[i];
            double position_change = joint_position_command_[i] - joint_position_command_prev_[i];

            // 安全限制：限制单次位置变化
            if(std::abs(position_change) > max_position_change_){
                ROS_WARN_THROTTLE(1.0,
                    "Joint %s: Position change too large (%.3f rad), limiting to %.3f rad",
                    joint_names_[i].c_str(), position_change, max_position_change_);
                position_change = std::copysign(max_position_change_, position_change);
                joint_position_command_[i] = joint_position_command_prev_[i] + position_change;
            }

            // 计算目标速度 - 关键改进点
            double target_velocity;

            if(use_mit_mode_){

                // MIT模式：使用位置变化计算速度
                target_velocity = position_change / dt;
            }
            else{

                // 位置速度模式：使用比例控制 + 速度前馈
                const double kp_tracking = 10.0;
                target_velocity = kp_tracking * position_error;

                // 添加基于位置变化的前馈
                if(std::abs(position_change) > 0.0001){
                    target_velocity += position_change / dt;
                }
            }

            // 限制目标速度
            target_velocity = std::max(-max_velocity_,
                std::min(max_velocity_, target_velocity));

            // 发送控制命令
            if(use_mit_mode_){
                motor_controller_->control_mit(
                    *motors_[i],
                    kp_,
                    kd_,
                    joint_position_command_[i],
                    target_velocity,
                    0.0f
                );
            }
            else{
                if(i == 0){

                    // JOINT1使用MIT模式控制
                    motor_controller_->control_mit(
                        *motors_[i],
                        kp_,
                        kd_,
                        joint_position_command_[i],
                        target_velocity,
                        0.0f
                    );
                }
                else{
                    motor_controller_->control_pos_vel(
                        *motors_[i],
                        joint_position_command_[i],
                        target_velocity
                    );
                }
            }

            // 更新上一次的命令
            joint_position_command_prev_[i] = joint_position_command_[i];
        }
        catch(const std::exception& e){
            ROS_ERROR_THROTTLE(1.0, "Error controlling motor %s: %s",
                joint_names_[i].c_str(), e.what());
        }
    }
}
