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
#include <sstream>
#include <string>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "camera_info_manager/camera_info_manager.h"
#include "std_srvs/Empty.h"

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/utils.hpp"


namespace usb_cam
{

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle m_node;

  // shared image message
  sensor_msgs::Image m_image;
  image_transport::CameraPublisher m_image_pub;

  // parameters
  std::string m_video_device_name, m_io_method_str, m_pixel_format_str, m_camera_name,
    m_camera_info_url;
  int m_image_width, m_image_height, m_framerate, m_exposure, m_brightness, m_contrast,
    m_saturation, m_sharpness, m_focus, m_white_balance, m_gain;
  bool m_auto_focus, m_auto_exposure, m_auto_white_balance;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> m_camera_info;

  UsbCam m_camera;

  ros::ServiceServer m_service_start, m_service_stop;


  bool m_service_startcap(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
  {
    (void)req;
    (void)res;
    m_camera.start_capturing();
    return true;
  }


  bool m_service_stopcap(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
  {
    (void)req;
    (void)res;
    m_camera.stop_capturing();
    return true;
  }

  UsbCamNode()
  : m_node("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(m_node);
    m_image_pub = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    m_node.param("video_device", m_video_device_name, std::string("/dev/video0"));
    m_node.param("brightness", m_brightness, -1);  // 0-255, -1 "leave alone"
    m_node.param("contrast", m_contrast, -1);  // 0-255, -1 "leave alone"
    m_node.param("saturation", m_saturation, -1);  // 0-255, -1 "leave alone"
    m_node.param("sharpness", m_sharpness, -1);  // 0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    m_node.param("io_method", m_io_method_str, std::string("mmap"));
    m_node.param("image_width", m_image_width, 640);
    m_node.param("image_height", m_image_height, 480);
    m_node.param("framerate", m_framerate, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    m_node.param("pixel_format", m_pixel_format_str, std::string("mjpeg"));
    // enable/disable autofocus
    m_node.param("autofocus", m_auto_focus, false);
    m_node.param("focus", m_focus, -1);  // 0-255, -1 "leave alone"
    // enable/disable autoexposure
    m_node.param("autoexposure", m_auto_exposure, true);
    m_node.param("exposure", m_exposure, 100);
    m_node.param("gain", m_gain, -1);  // 0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    m_node.param("auto_white_balance", m_auto_white_balance, true);
    m_node.param("white_balance", m_white_balance, 4000);

    // load the camera info
    m_node.param("camera_frame_id", m_image.header.frame_id, std::string("head_camera"));
    m_node.param("camera_name", m_camera_name, std::string("head_camera"));
    m_node.param("camera_info_url", m_camera_info_url, std::string(""));
    m_camera_info.reset(
      new camera_info_manager::CameraInfoManager(
        m_node, m_camera_name, m_camera_info_url));

    // create Services
    m_service_start = \
      m_node.advertiseService("start_capture", &UsbCamNode::m_service_startcap, this);
    m_service_stop = \
      m_node.advertiseService("stop_capture", &UsbCamNode::m_service_stopcap, this);

    // check for default camera info
    if (!m_camera_info->isCalibrated()) {
      m_camera_info->setCameraName(m_video_device_name);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = m_image.header.frame_id;
      camera_info.width = m_image_width;
      camera_info.height = m_image_height;
      m_camera_info->setCameraInfo(camera_info);
    }


    ROS_INFO(
      "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
      m_camera_name.c_str(), m_video_device_name.c_str(),
      m_image_width, m_image_height, m_io_method_str.c_str(),
      m_pixel_format_str.c_str(), m_framerate);

    // set the IO method
    io_method_t io_method = usb_cam::utils::io_method_from_string(m_io_method_str);
    if (io_method == io_method_t::IO_METHOD_UNKNOWN) {
      ROS_FATAL("Unknown IO method '%s'", m_io_method_str.c_str());
      m_node.shutdown();
      return;
    }

    // start the camera
    m_camera.start(
      m_video_device_name.c_str(), io_method, m_pixel_format_str, m_image_width,
      m_image_height, m_framerate);

    set_v4l2_params();
  }

  virtual ~UsbCamNode()
  {
    m_camera.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the new image
    auto new_image = m_camera.get_image();

    // fill in the image message
    m_image.header.stamp.sec = new_image->stamp.tv_sec;
    m_image.header.stamp.nsec = new_image->stamp.tv_nsec;

    // Only resize if required
    if (m_image.data.size() != static_cast<size_t>(new_image->step * new_image->height)) {
      m_image.width = new_image->width;
      m_image.height = new_image->height;
      m_image.encoding = new_image->encoding;
      m_image.step = new_image->step;
      m_image.data.resize(new_image->step * new_image->height);
    }

    // Fill in image data
    memcpy(&m_image.data[0], new_image->image, m_image.data.size());

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(m_camera_info->getCameraInfo()));
    ci->header.frame_id = m_image.header.frame_id;
    ci->header.stamp = m_image.header.stamp;

    // publish the image
    m_image_pub.publish(m_image, *ci);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->m_framerate);
    while (m_node.ok()) {
      if (m_camera.is_capturing()) {
        if (!take_and_send_image()) {ROS_WARN("USB camera did not respond in time.");}
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }

  void set_v4l2_params()
  {
    // set camera parameters
    if (m_brightness >= 0) {
      m_camera.set_v4l_parameter("brightness", m_brightness);
    }

    if (m_contrast >= 0) {
      m_camera.set_v4l_parameter("contrast", m_contrast);
    }

    if (m_saturation >= 0) {
      m_camera.set_v4l_parameter("saturation", m_saturation);
    }

    if (m_sharpness >= 0) {
      m_camera.set_v4l_parameter("sharpness", m_sharpness);
    }

    if (m_gain >= 0) {
      m_camera.set_v4l_parameter("gain", m_gain);
    }

    // check auto white balance
    if (m_auto_white_balance) {
      m_camera.set_v4l_parameter("white_balance_temperature_auto", 1);
    } else {
      m_camera.set_v4l_parameter("white_balance_temperature_auto", 0);
      m_camera.set_v4l_parameter("white_balance_temperature", m_white_balance);
    }

    // check auto exposure
    if (!m_auto_exposure) {
      // turn down exposure control (from max of 3)
      m_camera.set_v4l_parameter("m_exposureauto", 1);
      // change the exposure level
      m_camera.set_v4l_parameter("m_exposureabsolute", m_exposure);
    }

    // check auto focus
    if (m_auto_focus) {
      m_camera.set_auto_focus(1);
      m_camera.set_v4l_parameter("m_focusauto", 1);
    } else {
      m_camera.set_v4l_parameter("m_focusauto", 0);
      if (m_focus >= 0) {
        m_camera.set_v4l_parameter("m_focusabsolute", m_focus);
      }
    }
  }
};

}  // namespace usb_cam

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
