cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if($ENV{ROS_VERSION} EQUAL 2)
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  endif()
endif()

# Remove once ROS 1 Noetic is deprecated
if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    camera_info_manager
    image_transport)
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
endif()

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

if(EXISTS ${avcodec})
  message(STATUS "Found libavcodec: ${avcodec}")
endif()

if(EXISTS ${avutil})
  message(STATUS "Found libavutil: ${avutil}")
endif()

if(EXISTS ${swscale})
  message(STATUS "Found libswscale: ${swscale}")
endif()

## Build the USB camera library
## Do not use ament_auto here so as to not link to rclcpp
add_library(${PROJECT_NAME} SHARED
  src/usb_cam.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "include"
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
  ${OpenCV_LIBRARIES})

# Remove once ROS 1 Noetic is deprecated
if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )

  add_executable(${PROJECT_NAME}_node src/ros1/usb_cam_node.cpp)
  target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME}_node PUBLIC
    ${catkin_INCLUDE_DIRS})
else()
  ament_export_libraries(${PROJECT_NAME})

  ## Declare a ROS 2 composible node as a library
  ament_auto_add_library(${PROJECT_NAME}_node SHARED
    src/ros2/usb_cam_node.cpp
  )

  target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME})

  ## Use node to generate an executable
  rclcpp_components_register_node(${PROJECT_NAME}_node
    PLUGIN "usb_cam::UsbCamNode"
    EXECUTABLE ${PROJECT_NAME}_node_exe
  )
  if(SANITIZE)
    target_compile_options(${PROJECT_NAME} PUBLIC -fsanitize=address -fsanitize=leak)
    target_link_libraries(${PROJECT_NAME} -fsanitize=address -fsanitize=leak)
    target_compile_options(${PROJECT_NAME}_node PUBLIC -fsanitize=address -fsanitize=leak)
    target_link_libraries(${PROJECT_NAME}_node -fsanitize=address -fsanitize=leak)
    target_link_libraries(${PROJECT_NAME}_node_exe -fsanitize=address -fsanitize=leak)
  endif()
endif()

if(BUILD_TESTING)
  if($ENV{ROS_VERSION} EQUAL 2)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)

    # Unit tests
    ament_add_gtest(test_usb_cam_utils
      test/test_usb_cam_utils.cpp)
    target_link_libraries(test_usb_cam_utils
      ${PROJECT_NAME})
    ament_add_gtest(test_pixel_formats
      test/test_pixel_formats.cpp)
    target_link_libraries(test_pixel_formats
      ${PROJECT_NAME})
    # TODO(flynneva): rewrite this test in another PR
    # Integration tests
    # ament_add_gtest(test_usb_cam_lib
    #  test/test_usb_cam_lib.cpp)
    # target_link_libraries(test_usb_cam_lib
    #   ${PROJECT_NAME})
  endif()
endif()


if($ENV{ROS_VERSION} EQUAL 1)
  install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  ## Copy launch files
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
else()
  install(
    PROGRAMS scripts/show_image.py
    DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )

  ament_auto_package(
    INSTALL_TO_SHARE
      launch
      config
  )
endif()
