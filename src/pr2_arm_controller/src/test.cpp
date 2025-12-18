#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pr2_arm_controller/multi_arm_controller.hpp"
#include "pr2_arm_controller/arm_controller.hpp"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "multi_arm_controller_test");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;

    ROS_INFO("========== 多机械臂控制器测试节点启动 ==========");

    /* 加载机器人模型 */
    robot_model_loader::RobotModelLoader loader("robot_description");
    moveit::core::RobotModelPtr robot_model = loader.getModel();

    if(!robot_model){
        ROS_ERROR("加载 robot_description 失败，无法获取机器人模型！");
        return 1;
    }

    ROS_INFO("机器人模型加载成功。");

    /* 初始化多臂控制器 */
    pr2_arm::MultiArmController multi_arm(robot_model, "both_arms");

    multi_arm.registerArm(
        "left_arm",
        "l_wrist_roll_link",
        "base_link"
    );

    multi_arm.registerArm(
        "right_arm",
        "r_wrist_roll_link",
        "base_link"
    );

    ROS_INFO("多机械臂控制器初始化完成。");

    /* 单臂 IK / 可达性检查器 */
    pr2_arm::ArmController left_arm_checker(nh, "left_arm");
    pr2_arm::ArmController right_arm_checker(nh, "right_arm");

    ros::Duration(2.0).sleep();

    /* 获取当前末端位姿 */
    ROS_INFO("读取当前左右机械臂末端位姿……");

    geometry_msgs::Pose left_cur = multi_arm.getCurPose("left_arm");
    geometry_msgs::Pose right_cur = multi_arm.getCurPose("right_arm");

    ROS_INFO_STREAM("左臂当前末端位姿：\n" << left_cur);
    ROS_INFO_STREAM("右臂当前末端位姿：\n" << right_cur);

    ros::Duration(3.0).sleep();

    /* 测试：左臂沿末端 X 方向前伸 */
    ROS_INFO("测试 1：左臂末端沿自身 X 轴方向前伸 0.10 米");

    if(!multi_arm.stretch("left_arm", 0.1)){
        ROS_WARN("左臂前伸动作执行失败！");
    }else{
        ROS_INFO("左臂前伸动作执行成功。");
    }

    ros::Duration(4.0).sleep();

    /* 测试：右臂末端绕自身 X 轴旋转 */
    ROS_INFO("测试 2：右臂末端绕自身 X 轴旋转 90 度");

    if(!multi_arm.rotate("right_arm", 90.0)){
        ROS_WARN("右臂旋转动作执行失败！");
    }else{
        ROS_INFO("右臂旋转动作执行成功。");
    }

    ros::Duration(4.0).sleep();

    /* 测试：单臂 moveToPoseForArm */
    ROS_INFO("测试 3：单臂位姿控制（左臂）");

    geometry_msgs::Pose left_single_tgt;  
    left_single_tgt.position.x = 0.455315;
    left_single_tgt.position.y = 0.258;
    left_single_tgt.position.z = 1.25001;
    left_single_tgt.orientation.x = 0.0;
    left_single_tgt.orientation.y = -0.88793;
    left_single_tgt.orientation.z = 0.0;
    left_single_tgt.orientation.w = 0.459979;

    if(!multi_arm.moveToPoseForArm("left_arm", left_single_tgt)){
        ROS_WARN("左臂单臂位姿控制失败！");
    }else{
        ROS_INFO("左臂单臂位姿控制成功。");
    }

    ros::Duration(4.0).sleep();

    /* 测试：双臂同步位姿控制 */
    ROS_INFO("测试 4：双臂同步位姿控制");

    geometry_msgs::Pose left_tgt, right_tgt;

    left_tgt.position.x = 0.45;
    left_tgt.position.y = 0.25;
    left_tgt.position.z = 0.40;
    left_tgt.orientation.w = 1.0;

    right_tgt.position.x = 0.45;
    right_tgt.position.y = -0.25;
    right_tgt.position.z = 0.40;
    right_tgt.orientation.w = 1.0;

    std::vector<geometry_msgs::Pose> tgt_group;
    tgt_group.push_back(left_tgt);
    tgt_group.push_back(right_tgt);

    if(!multi_arm.moveToPoseForMultiArms(tgt_group)){
        ROS_WARN("双臂同步位姿控制失败！");
    }else{
        ROS_INFO("双臂同步位姿控制成功。");
    }

    ros::Duration(5.0).sleep();

    /* 测试：复位双臂 */
    ROS_INFO("测试 5：双臂复位到预定义零位");

    if(!multi_arm.resetMultiArmsToZero()){
        ROS_WARN("双臂复位失败（可能尚未配置零位参数）！");
    }else{
        ROS_INFO("双臂复位成功。");
    }

    ros::Duration(3.0).sleep();

    ROS_INFO("========== 多机械臂控制器测试完成 ==========");

    ros::shutdown();
    return 0;
}
