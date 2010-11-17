cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_add_compile_flags("-O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/pose.cpp src/motion.cpp src/posemanager.cpp)

rosbuild_add_executable(control_movement src/control_movement.cpp)
target_link_libraries(control_movement ${PROJECT_NAME})

rosbuild_add_executable(balance_movement src/balance_movement.cpp)


