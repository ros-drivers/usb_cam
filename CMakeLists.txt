cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default ro C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)


###########
## Build ##
###########

include_directories(include)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "avcodec"
  "swscale")

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "rmw_implementation"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "avcodec"
  "swscale")
  
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION bin
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_package()
