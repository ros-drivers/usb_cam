cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)
rosbuild_init(usb_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_gensrv()
rosbuild_add_boost_directories()

add_subdirectory(src)
