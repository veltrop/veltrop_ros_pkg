cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE MinSizeRel)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

rosbuild_add_executable(joint_state_controlled src/joint_state_controlled.cpp src/servo.cpp) #rosbuild_add_executable(interactive_poser src/interactive_poser.cpp src/servo.cpp)

ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
