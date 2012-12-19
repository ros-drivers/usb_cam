/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
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
#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <usb_cam/usb_cam.h>
#include <self_test/self_test.h>
#include <image_transport/image_transport.h>

class UsbCamNode
{
public:
  ros::NodeHandle node_;
  sensor_msgs::Image img_;

  std::string video_device_name_;
  std::string io_method_name_;
  int image_width_,image_height_;
  std::string pixel_format_name_;
  bool autofocus_;

  sensor_msgs::CameraInfo info_;

  ros::Time next_time_;
  int count_;

  usb_cam_camera_image_t* camera_image_;

  image_transport::CameraPublisher image_pub_;

  UsbCamNode() :
      node_("~")
  {
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("io_method", io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg
    node_.param("autofocus", autofocus_, false); // enable/disable autofocus

    {
      XmlRpc::XmlRpcValue double_list;
      info_.height = image_height_;
      info_.width = image_width_;

      node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
      info_.header.frame_id = img_.header.frame_id;

      node_.getParam("K", double_list);
      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.K[i] = double_list[i];
        }
      }

      node_.getParam("D", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray)) {
        info_.D.resize(double_list.size());
        for (int i=0; i<double_list.size(); i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.D[i] = double_list[i];
        }
      }

      node_.getParam("R", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.R[i] = double_list[i];
        }
      }

      node_.getParam("P", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 12)) {
        for (int i=0; i<12; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.P[i] = double_list[i];
        }
      }
    }

    printf("usb_cam video_device set to [%s]\n", video_device_name_.c_str());
    printf("usb_cam io_method set to [%s]\n", io_method_name_.c_str());
    printf("usb_cam image_width set to [%d]\n", image_width_);
    printf("usb_cam image_height set to [%d]\n", image_height_);
    printf("usb_cam pixel_format set to [%s]\n", pixel_format_name_.c_str());
    printf("usb_cam auto_focus set to [%d]\n", autofocus_);

    usb_cam_io_method io_method;
    if(io_method_name_ == "mmap")
      io_method = IO_METHOD_MMAP;
    else if(io_method_name_ == "read")
      io_method = IO_METHOD_READ;
    else if(io_method_name_ == "userptr")
      io_method = IO_METHOD_USERPTR;
    else {
      ROS_FATAL("Unknown io method.");
      node_.shutdown();
      return;
    }

    usb_cam_pixel_format pixel_format;
    if(pixel_format_name_ == "yuyv")
      pixel_format = PIXEL_FORMAT_YUYV;
    else if(pixel_format_name_ == "uyvy")
      pixel_format = PIXEL_FORMAT_UYVY;
    else if(pixel_format_name_ == "mjpeg") {
      pixel_format = PIXEL_FORMAT_MJPEG;
    }
    else {
      ROS_FATAL("Unknown pixel format.");
      node_.shutdown();
      return;
    }

    camera_image_ = usb_cam_camera_start(video_device_name_.c_str(),
        io_method,
        pixel_format,
        image_width_,
        image_height_);

    if(autofocus_) {
      usb_cam_camera_set_auto_focus(1);
    }

    next_time_ = ros::Time::now();
    count_ = 0;
  }

  virtual ~UsbCamNode()
  {

    usb_cam_camera_shutdown();
  }

  bool take_and_send_image()
  {
    usb_cam_camera_grab_image(camera_image_);
    fillImage(img_, "rgb8", camera_image_->height, camera_image_->width, 3 * camera_image_->width, camera_image_->image);

    img_.header.stamp = ros::Time::now();
    info_.header.stamp = img_.header.stamp;
    image_pub_.publish(img_, info_);
    return true;
  }


  bool spin()
  {
    while (node_.ok())
    {
      if (take_and_send_image())
      {
        count_++;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time_) {
          std::cout << count_ << " frames/sec at " << now_time << std::endl;
          count_ = 0;
          next_time_ = next_time_ + ros::Duration(1,0);
        }
      } else {
        ROS_ERROR("couldn't take image.");
        usleep(1000000);
      }
//      self_test_.checkTest();
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  UsbCamNode a;
  a.spin();
  return 0;
}
