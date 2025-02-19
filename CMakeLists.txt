cmake_minimum_required(VERSION 3.5)
project(usb_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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

## Pure C++ V4L base library
add_library(v4l_lib
    src/v4l/device/base.cpp
    src/v4l/device.cpp
    src/v4l/device/capability.cpp
    src/v4l/device/video/input.cpp
    src/v4l/device/video/output.cpp
    src/v4l/device/audio/input.cpp
    src/v4l/device/audio/output.cpp
)
target_include_directories(v4l_lib PUBLIC "include")

## Pure C++ CLI for using the usb_cam library
add_executable(v4l_cli src/v4l/cli.cpp)
target_include_directories(v4l_cli PUBLIC "include")
target_link_libraries(v4l_cli v4l_lib)

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

ament_export_libraries(${PROJECT_NAME})

## Declare a ROS 2 composible node as a library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/usb_cam_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME})

## Use node to generate an executable
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "usb_cam::UsbCamNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# To fix `DSO missing from command line` error with clang
target_link_libraries(${PROJECT_NAME}_node_exe stdc++)

if(SANITIZE)
  target_compile_options(${PROJECT_NAME} PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME} -fsanitize=address -fsanitize=leak)
  target_compile_options(${PROJECT_NAME}_node PUBLIC -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME}_node -fsanitize=address -fsanitize=leak)
  target_link_libraries(${PROJECT_NAME}_node_exe -fsanitize=address -fsanitize=leak)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)
  # Unit tests
  ament_add_gtest(test_usb_cam_utils
    test/test_usb_cam_utils.cpp)
  target_link_libraries(test_usb_cam_utils
    ${PROJECT_NAME} m stdc++)
  ament_add_gtest(test_pixel_formats
    test/test_pixel_formats.cpp)
  target_link_libraries(test_pixel_formats
    ${PROJECT_NAME} m stdc++)
  if(INTEGRATION_TESTS)
    ament_add_gtest(test_usb_cam_lib
      test/test_usb_cam_lib.cpp)
    target_link_libraries(test_usb_cam_lib
      ${PROJECT_NAME})
  endif()
endif()

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

install(
  TARGETS v4l_cli
  DESTINATION bin
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
