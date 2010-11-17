cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_add_compile_flags("-O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##########################################################################
# following is for pose_gui QT app
# http://ros-users.122217.n3.nabble.com/using-Qt-IDE-in-ROS-td985317.html
# http://qtnode.net/wiki/Qt4_with_cmake

find_package(Qt4)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/pose_gui/*.cpp)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/pose_gui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/pose_gui/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/pose_gui/*.h)
QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

rosbuild_add_executable(pose_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(pose_gui ${QT_LIBRARIES} veltrobot_movement)