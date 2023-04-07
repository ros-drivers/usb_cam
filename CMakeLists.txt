cmake_minimum_required(VERSION 3.2)
project(usb_cam)

message(STATUS "Searching required modules")
find_package(PkgConfig REQUIRED)
# FFMPEG
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(avutil libavutil REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)
pkg_check_modules(video4linux libv4l2 REQUIRED)
# OPENCV
find_package(OpenCV REQUIRED)
# Reporting
if(avcodec_FOUND)
    message(STATUS "Found libavcodec: ${avcodec_VERSION}")
endif()
if(avutil_FOUND)
    message(STATUS "Found libavutil: ${avutil_VERSION}")
endif()
if(swscale_FOUND)
    message(STATUS "Found libswscale: ${swscale_VERSION}")
endif()
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
endif()
if(video4linux_FOUND)
    message(STATUS "Found Video4Linux kernel library: ${video4linux_VERSION}")
endif()

# Catkin
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    camera_info_manager
)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES usb_cam
)

# Build scenario
option(SUPPRESS_BIND_GLOBAL_PLACEHOLDERS
    "Suppresses Boost >= 1.68 deprecation warning message for global placeholders in boost::bind instantiation"
    ON
)
# Workaround for deprecation message from instantiation of Boost::bind
# Edit for CMake < 3.12 and ROS_DISTRO < melodic by knorth55
if("$ENV{ROS_DISTRO}" STRGREATER "melodic")
    if(SUPPRESS_BIND_GLOBAL_PLACEHOLDERS)
        add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)
    endif()
endif()

include_directories(include)
link_directories(
    ${video4linux_LIBRARY_DIRS}
    ${avcodec_LIBRARY_DIRS}
    ${avutil_LIBRARY_DIRS}
    ${swscale_LIBRARY_DIRS}
)

# Shared V4L2 driver library
set(v4l_driver_SHARED_HEADERS
    include/usb_cam/types.h
    include/usb_cam/util.h
    include/usb_cam/converters.h
    include/usb_cam/camera_driver.h
)
add_library(v4l_driver
    ${v4l_driver_SHARED_HEADERS}
    src/util.cpp
    src/converters.cpp
    src/camera_driver.cpp
)
target_include_directories(v4l_driver PUBLIC
    ${video4linux_INCLUDE_DIRS}
    ${avcodec_INCLUDE_DIRS}
    ${swscale_INCLUDE_DIRS}
    ${avutil_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(v4l_driver
    ${video4linux_LIBRARIES}
    ${avcodec_LIBRARIES}
    ${avutil_LIBRARIES}
    ${swscale_LIBRARIES}
    ${OpenCV_LIBS}
)

set(usb_cam_SHARED_HEADERS
    include/usb_cam/usb_cam.h
)
add_library(usb_cam
    ${usb_cam_SHARED_HEADERS}
    src/usb_cam.cpp
)
target_include_directories(usb_cam PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(usb_cam
    ${catkin_LIBRARIES}
    v4l_driver
)

## Declare a cpp executable
add_executable(usb_cam_node
    src/usb_cam_node.cpp
)
target_include_directories(usb_cam_node PRIVATE
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(usb_cam_node
    usb_cam
    ${catkin_LIBRARIES}
)

# Installation scenario
# Executables and libraries
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
# Launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
# Include files
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
