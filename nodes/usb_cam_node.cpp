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

#include "usb_cam/usb_cam_node.h"

#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <ctime>

PLUGINLIB_EXPORT_CLASS(usb_cam::UsbCamNode, nodelet::Nodelet)

namespace usb_cam {
UsbCamNode::UsbCamNode() {}

UsbCamNode::~UsbCamNode()
{
  if (image_grabber_.joinable()) image_grabber_.join();
  if (image_receiver_.joinable()) image_receiver_.join();
  cam_.shutdown();
}

void UsbCamNode::onInit()
{
  node_ = getMTPrivateNodeHandle();
  // advertise the main image topic
  image_transport::ImageTransport it(node_);
  image_pub_ = it.advertiseCamera("image_raw", 1);

  // grab the parameters
  node_.param("video_device", video_device_name_, std::string("/dev/video0"));
  node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
  node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
  node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
  node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
  // possible values: mmap, read, userptr
  node_.param("io_method", io_method_name_, std::string("mmap"));
  node_.param("image_width", image_width_, 640);
  node_.param("image_height", image_height_, 480);
  node_.param("framerate", framerate_, 30);
  // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
  node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
  // enable/disable autofocus
  node_.param("autofocus", autofocus_, false);
  node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
  // enable/disable autoexposure
  node_.param("autoexposure", autoexposure_, true);
  node_.param("exposure", exposure_, 100);
  node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
  // enable/disable auto white balance temperature
  node_.param("auto_white_balance", auto_white_balance_, true);
  node_.param("white_balance", white_balance_, 4000);

  // load the camera info
  node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
  node_.param("camera_name", camera_name_, std::string("head_camera"));
  node_.param("camera_info_url", camera_info_url_, std::string(""));
  cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

  // create Services
  service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
  service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

  // check for default camera info
  if (!cinfo_->isCalibrated())
  {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
  }


  ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
           image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

  // set the IO method
  UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
  if(io_method == UsbCam::IO_METHOD_UNKNOWN)
  {
    ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
    node_.shutdown();
    return;
  }

  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
  {
    ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
    node_.shutdown();
    return;
  }

  // start the camera
  cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
             image_height_, framerate_);

  // set camera parameters
  if (brightness_ >= 0)
  {
    cam_.set_v4l_parameter("brightness", brightness_);
  }

  if (contrast_ >= 0)
  {
    cam_.set_v4l_parameter("contrast", contrast_);
  }

  if (saturation_ >= 0)
  {
    cam_.set_v4l_parameter("saturation", saturation_);
  }

  if (sharpness_ >= 0)
  {
    cam_.set_v4l_parameter("sharpness", sharpness_);
  }

  if (gain_ >= 0)
  {
    cam_.set_v4l_parameter("gain", gain_);
  }

  // check auto white balance
  if (auto_white_balance_)
  {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
  }
  else
  {
    cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
    cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
  }

  // check auto exposure
  if (!autoexposure_)
  {
    // turn down exposure control (from max of 3)
    cam_.set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    cam_.set_v4l_parameter("exposure_absolute", exposure_);
  }

  // check auto focus
  if (autofocus_)
  {
    cam_.set_auto_focus(1);
    cam_.set_v4l_parameter("focus_auto", 1);
  }
  else
  {
    cam_.set_v4l_parameter("focus_auto", 0);
    if (focus_ >= 0)
    {
      cam_.set_v4l_parameter("focus_absolute", focus_);
    }
  }

  image_receiver_ = std::thread(&UsbCamNode::publish, this);
  image_grabber_ = std::thread(&UsbCamNode::grab, this);
}

bool UsbCamNode::service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  cam_.start_capturing();
  return true;
}

bool UsbCamNode::service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  cam_.stop_capturing();
  return true;
}

void UsbCamNode::publish() {
  while(ros::ok()) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    image_cv_.wait(lock, [this] { return !image_queue_.empty(); });
    while (!image_queue_.empty()) {
      img_ = image_queue_.front();
      image_queue_.pop();

      // Grab the camera info.
      sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
      ci->header.frame_id = img_.header.frame_id;
      ci->header.stamp = img_.header.stamp;

      // publish the image
      image_pub_.publish(img_, *ci);
    }
  }
}

void UsbCamNode::grab() {
  static constexpr std::chrono::duration<double, std::milli> MinSleepDuration(0);
  double dt = 100;
  while (ros::ok()) {
    std::clock_t start = std::clock();
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      sensor_msgs::Image image_msg;
      cam_.grab_image(&image_msg);
      image_queue_.push(image_msg);
      image_available_ = true;
    }

    std::clock_t end = std::clock();
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(dt - (end - start)));
    image_cv_.notify_all();
  }
}
}  // namespace usb_cam
