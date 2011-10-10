cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE MinSizeRel)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

#rosbuild_add_executable(sim_pointcloud src/experimental/sim_pointcloud.cpp)
#rosbuild_add_executable(get_state_validity_test src/experimental/get_state_validity_test.cpp)

rosbuild_add_executable(republish_stereo src/republish_stereo.cpp)
	
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  rosbuild_add_executable(acquire_stereo src/acquire_stereo.cpp src/uvc_cam.cpp)
#  rosbuild_add_executable(test_video src/experimental/test_video.cpp src/uvc_cam.cpp)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
