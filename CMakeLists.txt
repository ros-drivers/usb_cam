cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)
find_package(catkin 
	REQUIRED COMPONENTS 
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs 
		self_test
        camera_info_manager)


set(ROS_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs
		self_test
        camera_info_manager
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
)
add_library(usb_cam
	src/usb_cam.cpp
)
target_link_libraries(usb_cam avcodec swscale ${catkin_LIBRARIES})

add_executable(usb_cam_node nodes/usb_cam_node.cpp)
target_link_libraries(usb_cam_node usb_cam avcodec swscale ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS usb_cam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS usb_cam
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
