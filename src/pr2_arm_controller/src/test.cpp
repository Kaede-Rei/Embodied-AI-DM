#include "pr2_arm_controller/arm_controller.hpp"
#include "pr2_arm_controller/multi_arm_controller.hpp"

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2_eigen/tf2_eigen.h>


int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "multi_arm_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    robot_model_loader::RobotModelLoader loader("robot_description");
    pr2_arm::MultiArmController multi_arm(loader.getModel(), "both_arms");
    multi_arm.addArmGroup("left_arm", "l_wrist_roll_link");
    multi_arm.addArmGroup("right_arm", "r_wrist_roll_link");

    robot_state::RobotState current_state(loader.getModel());
    current_state.setToDefaultValues();

    const auto* ee_link =
        loader.getModel()->getLinkModel("l_wrist_roll_link");

    Eigen::Isometry3d ee_tf =
        current_state.getGlobalLinkTransform(ee_link);

    geometry_msgs::Pose ee_pose;
    ee_pose = tf2::toMsg(ee_tf);

    ROS_INFO_STREAM("当前左臂末端 pose:\n" << ee_pose);

    geometry_msgs::Pose left_pose;
    left_pose.position.x = 0.585315;
    left_pose.position.y = 0.188;
    left_pose.position.z = 1.25001;
    left_pose.orientation.x = 0.0;
    left_pose.orientation.y = -0.88793;
    left_pose.orientation.z = 0.0;
    left_pose.orientation.w = 0.459979;

    geometry_msgs::Pose right_pose;
    right_pose.position.x = 0.585315;
    right_pose.position.y = -0.188;
    right_pose.position.z = 1.25001;
    right_pose.orientation.x = 0.0;
    right_pose.orientation.y = -0.88793;
    right_pose.orientation.z = 0.0;
    right_pose.orientation.w = 0.459979;

    std::vector<geometry_msgs::Pose> targets;
    targets.push_back(left_pose);
    targets.push_back(right_pose);

    if(!multi_arm.setPoseToMultiArms(targets)){
        ROS_ERROR("设置多机械臂目标位姿失败！");
        return -1;
    }

    ros::spinOnce();

    return 0;
}
