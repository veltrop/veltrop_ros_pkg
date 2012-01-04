cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE MinSizeRel)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

rosbuild_add_executable(poll_ad src/ad/poll_ad.cpp src/ad/ad_device.cpp src/ad/ad_device_2y0a21.cpp src/ad/ad_device_2d120x.cpp src/ad/ad_device_krg3.cpp)

#rosbuild_add_executable(configure_HMC6343 src/i2c/configure_HMC6343.cpp)

	rosbuild_add_executable(i2c_device_manager_node src/i2c/i2c_device_manager_node.cpp src/i2c/i2c_device_manager.cpp src/i2c/i2c_device.cpp src/i2c/i2c_device_hmc6343.cpp src/i2c/i2c_device_srf08.cpp src/i2c/i2c_device_itg3200.cpp)

ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
