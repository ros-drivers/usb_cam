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
// #include <image_transport/image_transport.h>
// #include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/msg/image.hpp>
#include <sstream>
// #include <std_srvs/srv/Empty.h>

std::ostream& operator<<(std::ostream& ostr, const rclcpp::Time& tm) {
  ostr << tm.nanoseconds();
  return ostr;
}

using namespace std::chrono_literals;

namespace usb_cam {

class UsbCamNode : public rclcpp::Node
{
public:
  // shared image message
  sensor_msgs::msg::Image::SharedPtr img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // parameters
  std::string video_device_name_ = "/dev/video0";
  std::string frame_id_ = "map";

  std::string io_method_name_ = "mmap";
  // these parameters all have to be a combination supported by the device
  // Use
  // v4l2-ctl --device=0 --list-formats-ext to discover them,
  // or guvcview
  std::string pixel_format_name_ = "yuyv";
  int image_width_ = 640;
  int image_height_ = 480;
  int framerate_ = 15;

  // std::string start_service_name_, start_service_name_;
  // TODO(lucasw) use v4l2ucp for these?
  // int exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
  //    white_balance_, gain_;
  // bool autofocus_, autoexposure_, auto_white_balance_;

  std::string camera_name_;
  // std::string camera_info_url_;
  // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  rclcpp::TimerBase::SharedPtr timer_;

#if 0
  ros::ServiceServer service_start_, service_stop_;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }
#endif

  UsbCamNode() : Node("usb_cam")
  {
    // TODO(lucasw) use unique_ptr to reduce copies, see
    // demos/intra_process_demo/include/image_pipeline/camera_node.hpp
    img_ = std::make_shared<sensor_msgs::msg::Image>();
    // advertise the main image topic
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw");
  }

  void init()
  {
    // grab the parameters
    get_parameter_or("video_device", video_device_name_, video_device_name_);

    // possible values: mmap, read, userptr
    get_parameter_or("io_method", io_method_name_, io_method_name_);
    get_parameter_or("frame_id", frame_id_, frame_id_);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    get_parameter_or("pixel_format", pixel_format_name_, pixel_format_name_);
    get_parameter_or("framerate", framerate_, framerate_);
    if (framerate_ <= 0)
    {
      RCLCPP_ERROR(get_logger(), "bad framerate %d", framerate_);
      return;
    }
    get_parameter_or("image_width", image_width_, image_width_);
    get_parameter_or("image_height", image_height_, image_height_);

#if 0
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
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
#endif

    get_parameter_or("camera_name", camera_name_, std::string("head_camera"));
#if 0
    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
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
#endif
    img_->header.frame_id = frame_id_;

    RCLCPP_INFO(this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
        camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(),
        pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown IO method '%s'", io_method_name_.c_str());
      rclcpp::shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown pixel format '%s'", pixel_format_name_.c_str());
      rclcpp::shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

#if 0
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
#endif

    // TODO(lucasw) should this check a little faster than expected frame rate?
    // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
    const int period_ms = 1000.0 / framerate_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<long int>(period_ms)),
        std::bind(&UsbCamNode::update, this));
    INFO("starting timer");
  }

  virtual ~UsbCamNode()
  {
    WARN("shutting down");
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    if (!cam_.get_image(img_->header.stamp, img_->encoding, img_->height, img_->width,
        img_->step, img_->data)) {
      ERROR("grab failed");
      return false;
    }

#if 0
    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);
#endif
    // INFO(img_->data.size() << " " << img_->width << " " << img_->height << " " << img_->step);
    image_pub_->publish(img_);

    return true;
  }

  void update()
  {
    if (cam_.is_capturing()) {
      // TODO(lucasw) if framerate is 8 or below, then grabbing an image takes
      // one ms, but if it is higher then the time here jumps to 130 ms
      // auto t0 = now();
      if (!take_and_send_image()) {
        WARN("USB camera did not respond in time.");
      }
      // auto diff = now() - t0;
      // INFO(diff.nanoseconds() / 1e6 << " " << int(t0.nanoseconds() / 1e9));
    }
  }
};

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto usb_cam_node = std::make_shared<usb_cam::UsbCamNode>();
  usb_cam_node->init();
  rclcpp::spin(usb_cam_node);
  INFO("node done");
  rclcpp::shutdown();
  return 0;
}
