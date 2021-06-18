cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

include_directories(include
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/usb_cam.cpp
)

## Declare a cpp executable
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/usb_cam_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  # TODO(lucasw) should this have been in libavcodec?
  #avutil
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "usb_cam::UsbCamNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

install(
  PROGRAMS scripts/show_image.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
