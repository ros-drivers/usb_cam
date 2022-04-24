// Copyright 2021 Evan Flynn
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
//    * Neither the name of the Evan Flynn nor the names of its
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


#ifndef USB_CAM__USB_CAM_NODE_HPP_
#define USB_CAM__USB_CAM_NODE_HPP_
#include "usb_cam/usb_cam.hpp"


#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

std::ostream & operator<<(std::ostream & ostr, const rclcpp::Time & tm)
{
  ostr << tm.nanoseconds();
  return ostr;
}


namespace usb_cam
{


class UsbCamNode : public rclcpp::Node
{
public:
  UsbCamNode(const rclcpp::NodeOptions & node_options);
  ~UsbCamNode();

  void init();
  void get_params();
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
  void update();
  bool take_and_send_image();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  UsbCam cam_;

  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_;
  std::shared_ptr<image_transport::CameraPublisher> image_pub_;
  // parameters
  std::string video_device_name_;
  std::string frame_id_;

  std::string io_method_name_;
  // these parameters all have to be a combination supported by the device
  // Use
  // v4l2-ctl --device=0 --list-formats-ext
  // to discover them,
  // or guvcview
  std::string pixel_format_name_;
  int image_width_;
  int image_height_;
  int framerate_;

  // TODO(lucasw) use v4l2ucp for these?
  // int exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
  //    white_balance_, gain_;
  // bool autofocus_, autoexposure_, auto_white_balance_;

  std::string camera_name_;
  std::string camera_info_url_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_capture_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};
}  // namespace usb_cam
#endif  // USB_CAM__USB_CAM_NODE_HPP_
