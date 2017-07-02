cmake_minimum_required(VERSION 3.5)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# find_package(camera_info_manager REQUIRED)
# find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

include_directories(include
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  # TODO(lucasw) should this have been in libavcodec?
  avutil
)

ament_target_dependencies(${PROJECT_NAME}
  "builtin_interfaces"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/show_image.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
