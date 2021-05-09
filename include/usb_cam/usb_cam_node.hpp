#include "usb_cam/usb_cam.h"

#include <rclcpp/rclcpp.hpp>
// #include <image_transport/image_transport.h>
#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

std::ostream& operator<<(std::ostream& ostr, const rclcpp::Time& tm) {
  ostr << tm.nanoseconds();
  return ostr;
}

using namespace std::chrono_literals;


namespace usb_cam {


class UsbCamNode : public rclcpp::Node
{
public:
  UsbCamNode();
  virtual ~UsbCamNode() override;

  void init();
  void get_params();
  void update();
  bool take_and_send_image();

  void service_capture(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request>  request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  UsbCam cam_;

  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
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
};
}  // namespace usb_cam