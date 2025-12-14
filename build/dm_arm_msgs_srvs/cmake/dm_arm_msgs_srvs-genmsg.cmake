# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dm_arm_msgs_srvs: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dm_arm_msgs_srvs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" NAME_WE)
add_custom_target(_dm_arm_msgs_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dm_arm_msgs_srvs" "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(dm_arm_msgs_srvs
  "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dm_arm_msgs_srvs
)

### Generating Module File
_generate_module_cpp(dm_arm_msgs_srvs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dm_arm_msgs_srvs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dm_arm_msgs_srvs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dm_arm_msgs_srvs_generate_messages dm_arm_msgs_srvs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" NAME_WE)
add_dependencies(dm_arm_msgs_srvs_generate_messages_cpp _dm_arm_msgs_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dm_arm_msgs_srvs_gencpp)
add_dependencies(dm_arm_msgs_srvs_gencpp dm_arm_msgs_srvs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dm_arm_msgs_srvs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(dm_arm_msgs_srvs
  "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dm_arm_msgs_srvs
)

### Generating Module File
_generate_module_eus(dm_arm_msgs_srvs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dm_arm_msgs_srvs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dm_arm_msgs_srvs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dm_arm_msgs_srvs_generate_messages dm_arm_msgs_srvs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" NAME_WE)
add_dependencies(dm_arm_msgs_srvs_generate_messages_eus _dm_arm_msgs_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dm_arm_msgs_srvs_geneus)
add_dependencies(dm_arm_msgs_srvs_geneus dm_arm_msgs_srvs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dm_arm_msgs_srvs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(dm_arm_msgs_srvs
  "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dm_arm_msgs_srvs
)

### Generating Module File
_generate_module_lisp(dm_arm_msgs_srvs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dm_arm_msgs_srvs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dm_arm_msgs_srvs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dm_arm_msgs_srvs_generate_messages dm_arm_msgs_srvs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" NAME_WE)
add_dependencies(dm_arm_msgs_srvs_generate_messages_lisp _dm_arm_msgs_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dm_arm_msgs_srvs_genlisp)
add_dependencies(dm_arm_msgs_srvs_genlisp dm_arm_msgs_srvs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dm_arm_msgs_srvs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(dm_arm_msgs_srvs
  "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dm_arm_msgs_srvs
)

### Generating Module File
_generate_module_nodejs(dm_arm_msgs_srvs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dm_arm_msgs_srvs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dm_arm_msgs_srvs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dm_arm_msgs_srvs_generate_messages dm_arm_msgs_srvs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" NAME_WE)
add_dependencies(dm_arm_msgs_srvs_generate_messages_nodejs _dm_arm_msgs_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dm_arm_msgs_srvs_gennodejs)
add_dependencies(dm_arm_msgs_srvs_gennodejs dm_arm_msgs_srvs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dm_arm_msgs_srvs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(dm_arm_msgs_srvs
  "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dm_arm_msgs_srvs
)

### Generating Module File
_generate_module_py(dm_arm_msgs_srvs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dm_arm_msgs_srvs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dm_arm_msgs_srvs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dm_arm_msgs_srvs_generate_messages dm_arm_msgs_srvs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kaerei/ROS_Workspace/dm_ht_arm/src/dm_arm_msgs_srvs/srv/dm_arm_cmd.srv" NAME_WE)
add_dependencies(dm_arm_msgs_srvs_generate_messages_py _dm_arm_msgs_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dm_arm_msgs_srvs_genpy)
add_dependencies(dm_arm_msgs_srvs_genpy dm_arm_msgs_srvs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dm_arm_msgs_srvs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dm_arm_msgs_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dm_arm_msgs_srvs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dm_arm_msgs_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dm_arm_msgs_srvs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dm_arm_msgs_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dm_arm_msgs_srvs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dm_arm_msgs_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dm_arm_msgs_srvs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dm_arm_msgs_srvs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dm_arm_msgs_srvs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dm_arm_msgs_srvs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dm_arm_msgs_srvs_generate_messages_py std_msgs_generate_messages_py)
endif()
