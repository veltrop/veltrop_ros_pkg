cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE MinSizeRel)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#TODO: proper cmakelists stuff for glut, etc

rosbuild_add_executable(teleop_kb src/teleop_kb.cpp)
rosbuild_add_executable(teleop_joy src/teleop_joy.cpp)
rosbuild_add_executable(teleop_kinect src/teleop_kinect.cpp src/KinectController.cpp src/KinectDisplay.cpp)
target_link_libraries(teleop_kinect glut)

