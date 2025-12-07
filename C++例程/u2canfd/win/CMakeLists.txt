cmake_minimum_required(VERSION 3.16)
project(dm_main)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 查找 libusb-1.0 库
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)



# 编译封装DM_MOTOR库
add_library(motor STATIC
    src/protocol/damiao.cpp 
)
# 添加DM_MOTOR库头文件搜索路径
target_include_directories(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBUSB_INCLUDE_DIRS}     # 添加 libusb 头文件路径
)
# motor库链接libu2canfd.a，libu2canfd.a需要链接线程和libusb
target_link_libraries(motor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libu2canfd.a
        ${LIBUSB_LIBRARIES}
)


# 添加可执行文件
add_executable(${PROJECT_NAME}
    src/main.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${LIBUSB_INCLUDE_DIRS}     # 添加 libusb 头文件路径
)
# 链接库到可执行文件
target_link_libraries(${PROJECT_NAME} PUBLIC
    motor
)


# 添加可执行文件
add_executable(dev_sn
        src/dev_sn.cpp
)
target_include_directories(dev_sn PUBLIC
        ${LIBUSB_INCLUDE_DIRS}     # 添加 libusb 头文件路径
)
# 链接库到可执行文件
target_link_libraries(dev_sn PUBLIC
        ${LIBUSB_LIBRARIES}
)















