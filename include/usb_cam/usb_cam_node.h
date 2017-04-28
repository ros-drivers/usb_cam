/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/
#ifndef USB_CAM_USB_CAM_NODE_H_
#define USB_CAM_USB_CAM_NODE_H_

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Empty.h>
#include <ros/node_handle.h>
#include <usb_cam/usb_cam.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>

namespace usb_cam {

class UsbCamNode : public ::nodelet::Nodelet {
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;

  // Image thread.
  std::thread image_receiver_;
  std::thread image_grabber_;
  std::mutex image_mutex_;
  std::condition_variable image_cv_;
  std::atomic<bool> image_available_;
  std::queue<sensor_msgs::Image> image_queue_;  // Shared resource.

  UsbCamNode();
  virtual ~UsbCamNode();

  void onInit();

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );
  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );

 private:
  // Grab new image from USB cam at framerate.
  void grab();
  // Publish grabbed images.
  void publish();
};
}  // namespace usb_cam
#endif // USB_CAM_USB_CAM_NODE_H_
