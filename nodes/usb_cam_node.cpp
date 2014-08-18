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

#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  usb_cam_camera_image_t *camera_image_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_;
  std::string io_method_name_;
  int image_width_, image_height_, framerate_;
  std::string pixel_format_name_;
  bool autofocus_;
  std::string camera_name_;
  std::string camera_info_url_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    ROS_INFO("Camera name: %s", camera_name_.c_str());
    ROS_INFO("Camera frame: %s", img_.header.frame_id.c_str());
    ROS_INFO("Camera info url: %s", camera_info_url_.c_str());
    ROS_INFO("usb_cam video_device set to [%s]\n", video_device_name_.c_str());
    ROS_INFO("usb_cam io_method set to [%s]\n", io_method_name_.c_str());
    ROS_INFO("usb_cam image_width set to [%d]\n", image_width_);
    ROS_INFO("usb_cam image_height set to [%d]\n", image_height_);
    ROS_INFO("usb_cam pixel_format set to [%s]\n", pixel_format_name_.c_str());
    ROS_INFO("usb_cam autofocus set to [%d]\n", autofocus_);

    // set the IO method
    usb_cam_io_method io_method;
    if (io_method_name_ == "mmap")
      io_method = IO_METHOD_MMAP;
    else if (io_method_name_ == "read")
      io_method = IO_METHOD_READ;
    else if (io_method_name_ == "userptr")
      io_method = IO_METHOD_USERPTR;
    else
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    usb_cam_pixel_format pixel_format;
    if (pixel_format_name_ == "yuyv")
      pixel_format = PIXEL_FORMAT_YUYV;
    else if (pixel_format_name_ == "uyvy")
      pixel_format = PIXEL_FORMAT_UYVY;
    else if (pixel_format_name_ == "mjpeg")
      pixel_format = PIXEL_FORMAT_MJPEG;
    else if (pixel_format_name_ == "yuvmono10")
      pixel_format = PIXEL_FORMAT_YUVMONO10;
    else
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    camera_image_ = usb_cam_camera_start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
                                         image_height_, framerate_);

    // check auto focus
    if (autofocus_)
      usb_cam_camera_set_auto_focus(1);
  }

  virtual ~UsbCamNode()
  {
    usb_cam_camera_shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    usb_cam_camera_grab_image(camera_image_);
    // fill the info
    fillImage(img_, "rgb8", camera_image_->height, camera_image_->width, 3 * camera_image_->width,
              camera_image_->image);
    // stamp the image
    img_.header.stamp = ros::Time::now();

    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);
    return true;
  }

  bool spin()
  {
    while (node_.ok())
    {
      if (!take_and_send_image())
        ROS_WARN("USB camera did not respond in time.");
      ros::spinOnce();
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
