// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "usb_cam/usb_cam_node.hpp"

#include <sstream>
// #include <std_srvs/srv/Empty.h>

#include <string>
#include <memory>


namespace usb_cam
{

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("usb_cam", node_options),
  img_(new sensor_msgs::msg::Image()),
  image_pub_(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, "image_raw", rclcpp::QoS{100}.get_rmw_qos_profile()))),
  service_capture_(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &UsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("video_device", "/dev/video0");

  get_params();
  init();
  parameters_callback_handle_ = add_on_set_parameters_callback(
    std::bind(
      &UsbCamNode::parametersCallback, this,
      std::placeholders::_1));
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "shutting down");
  cam_.shutdown();
}

void UsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    cam_.start_capturing();
    response->message = "Start Capturing";
  } else {
    cam_.stop_capturing();
    response->message = "Stop Capturing";
  }
}

void UsbCamNode::init()
{
  while (frame_id_ == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  cinfo_.reset(new camera_info_manager::CameraInfoManager(this, camera_name_, camera_info_url_));
  // check for default camera info
  if (!cinfo_->isCalibrated()) {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.frame_id = img_->header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    cinfo_->setCameraInfo(camera_info);
  }

  img_->header.frame_id = frame_id_;
  RCLCPP_INFO(
    this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
    camera_name_.c_str(), video_device_name_.c_str(),
    image_width_, image_height_, io_method_name_.c_str(),
    pixel_format_name_.c_str(), framerate_);
  // set the IO method
  UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
  if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown IO method '%s'", io_method_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  // set the pixel format
  UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
  if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown pixel format '%s'", pixel_format_name_.c_str());
    rclcpp::shutdown();
    return;
  }
  // start the camera
  cam_.start(
    video_device_name_.c_str(), io_method, pixel_format, image_width_,
    image_height_, framerate_);
  cam_.get_formats();

  // TODO(lucasw) should this check a little faster than expected frame rate?
  // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
  const int period_ms = 1000.0 / framerate_;
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&UsbCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "starting timer " << period_ms);
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {"camera_name", "camera_info_url", "frame_id", "framerate",
      "image_height", "image_width", "io_method", "pixel_format", "video_device"});
  assign_params(parameters);
}

void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      camera_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      camera_info_url_ = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      frame_id_ = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      framerate_ = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      image_height_ = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      image_width_ = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      io_method_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      pixel_format_name_ = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      video_device_name_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

bool UsbCamNode::take_and_send_image()
{
  // grab the image
  if (!cam_.get_image(
      img_->header.stamp, img_->encoding, img_->height, img_->width,
      img_->step, img_->data))
  {
    RCLCPP_ERROR(this->get_logger(), "grab failed");
    return false;
  }

  // INFO(img_->data.size() << " " << img_->width << " " << img_->height << " " << img_->step);
  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
  ci->header = img_->header;
  image_pub_->publish(*img_, *ci);
  return true;
}

rcl_interfaces::msg::SetParametersResult UsbCamNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(get_logger(), "Setting parameters for %s", camera_name_.c_str());
  timer_->reset();
  cam_.shutdown();
  assign_params(parameters);
  init();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void UsbCamNode::update()
{
  if (cam_.is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    if (!take_and_send_image()) {
      RCLCPP_WARN(this->get_logger(), "USB camera did not respond in time.");
    }
    // auto diff = now() - t0;
    // INFO(diff.nanoseconds() / 1e6 << " " << int(t0.nanoseconds() / 1e9));
  }
}
}  // namespace usb_cam


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
