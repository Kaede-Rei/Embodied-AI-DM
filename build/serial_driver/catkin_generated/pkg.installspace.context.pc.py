# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;roscpp;serial;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lserial_driver".split(';') if "-lserial_driver" != "" else []
PROJECT_NAME = "serial_driver"
PROJECT_SPACE_DIR = "/home/kaerei/ROS_Workspace/dm_ht_arm/install"
PROJECT_VERSION = "0.0.0"
