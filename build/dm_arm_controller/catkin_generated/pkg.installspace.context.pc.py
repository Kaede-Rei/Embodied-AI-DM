# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib;geometry_msgs;moveit_core;moveit_ros_planning_interface;roscpp;rospy;std_msgs;std_srvs;tf2_geometry_msgs;tf2_ros".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldm_arm_controller".split(';') if "-ldm_arm_controller" != "" else []
PROJECT_NAME = "dm_arm_controller"
PROJECT_SPACE_DIR = "/home/kaerei/ROS_Workspace/dm_ht_arm/install"
PROJECT_VERSION = "0.0.0"
