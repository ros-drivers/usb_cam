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

#include <rclcpp/rclcpp.hpp>
#include <usb_cam/usb_cam.h>
#include <sstream>
#include <std_srvs/srv/empty.h>

namespace usb_cam {

class UsbCamNode : public rclcpp::Node
{
public:
  // shared image message
  sensor_msgs::msg::Image img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  
  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;

  UsbCam cam_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  UsbCamNode() : Node("usb_cam")
  {
    // advertise the main image topic
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", rmw_qos_profile_sensor_data);

    // grab the parameters
    get_parameter_or("video_device", video_device_name_, std::string("/dev/video0"));
    get_parameter_or("brightness", brightness_, -1); //0-255, -1 "leave alone"
    get_parameter_or("contrast", contrast_, -1); //0-255, -1 "leave alone"
    get_parameter_or("saturation", saturation_, -1); //0-255, -1 "leave alone"
    get_parameter_or("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    
    // possible values: mmap, read, userptr
    get_parameter_or("io_method", io_method_name_, std::string("mmap"));
    get_parameter_or("image_width", image_width_, 640);
    get_parameter_or("image_height", image_height_, 480);
    get_parameter_or("framerate", framerate_, 30);    
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    get_parameter_or("pixel_format", pixel_format_name_, std::string("yuyv"));
    // enable/disable autofocus
    get_parameter_or("autofocus", autofocus_, false);
    get_parameter_or("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    get_parameter_or("autoexposure", autoexposure_, true);
    get_parameter_or("exposure", exposure_, 100);
    get_parameter_or("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    get_parameter_or("auto_white_balance", auto_white_balance_, true);
    get_parameter_or("white_balance", white_balance_, 4000);

    // load the camera info
    get_parameter_or("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    get_parameter_or("camera_name", camera_name_, std::string("head_camera"));
    get_parameter_or("camera_info_url", camera_info_url_, std::string(""));

    RCLCPP_INFO(get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      RCLCPP_FATAL(get_logger(), "Unknown IO method '%s'", io_method_name_.c_str());
      rclcpp::shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      RCLCPP_FATAL(get_logger(), "Unknown pixel format '%s'", pixel_format_name_.c_str());
      rclcpp::shutdown();
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

    auto rate = std::make_unique<rclcpp::Rate>(double(framerate_));
    while (rclcpp::ok())
    {
      update();
      RCLCPP_DEBUG(get_logger(), "Got new image");
      rate->sleep();
    }
  }
  
  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // publish the image
    image_pub_->publish(img_);

    return true;
  }

  void update()
  {
    if (cam_.is_capturing()) {
      if (!take_and_send_image()) {
        RCLCPP_WARN(get_logger(), "USB camera did not respond in time.");
      }
    }
  }

};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto usb_cam = std::make_shared<usb_cam::UsbCamNode>();
  rclcpp::spin(usb_cam);
  rclcpp::shutdown();
  return 0;
}
