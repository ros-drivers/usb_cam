rosbuild_add_library(usb_cam usb_cam.cpp)
target_link_libraries(usb_cam avcodec swscale)
