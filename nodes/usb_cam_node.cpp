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
#include <sstream>

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
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, 128); //0-255
    node_.param("contrast", contrast_, 32); //0-255
    node_.param("saturation", saturation_, 32); //0-255
    node_.param("sharpness", sharpness_, 22); //0-255
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, 51);
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
             image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

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
    else if (pixel_format_name_ == "rgb24")
      pixel_format = PIXEL_FORMAT_RGB24;
    else
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    camera_image_ = usb_cam_camera_start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
                                         image_height_, framerate_);

    // set camera parameters
    std::stringstream paramstream;
    paramstream << "brightness=" << brightness_;
    this->set_v4l_parameters(video_device_name_, paramstream.str());
    paramstream.str("");
    paramstream << "contrast=" << contrast_;
    this->set_v4l_parameters(video_device_name_, paramstream.str());
    paramstream.str("");
    paramstream << "saturation=" << saturation_;
    this->set_v4l_parameters(video_device_name_, paramstream.str());
    paramstream.str("");
    paramstream << "sharpness=" << sharpness_;
    this->set_v4l_parameters(video_device_name_, paramstream.str());

    // check auto white balance
    if (auto_white_balance_)
    {
      this->set_v4l_parameters(video_device_name_, "white_balance_temperature_auto=1");
    }
    else
    {
      this->set_v4l_parameters(video_device_name_, "white_balance_temperature_auto=0");
      std::stringstream ss;
      ss << "white_balance_temperature=" << white_balance_;
      this->set_v4l_parameters(video_device_name_, ss.str());
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      this->set_v4l_parameters(video_device_name_, "exposure_auto=1");
      // change the exposure level
      std::stringstream ss;
      ss << "exposure_absolute=" << exposure_;
      this->set_v4l_parameters(video_device_name_, ss.str());
    }

    // check auto focus
    if (autofocus_)
    {
      usb_cam_camera_set_auto_focus(1);
      this->set_v4l_parameters(video_device_name_, "focus_auto=1");
    }
    else
    {
      this->set_v4l_parameters(video_device_name_, "focus_auto=0");
      std::stringstream ss;
      ss << "focus_absolute=" << focus_;
      this->set_v4l_parameters(video_device_name_, ss.str());
    }
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
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (!take_and_send_image())
        ROS_WARN("USB camera did not respond in time.");
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }

private:

  /**
   * Set video device parameters via calls to v4l-utils.
   *
   * @param dev The device (e.g., "/dev/video0")
   * @param param The full parameter to set (e.g., "focus_auto=1")
   */
  void set_v4l_parameters(std::string dev, std::string param)
  {
    // build the command
    std::stringstream ss;
    ss << "v4l2-ctl --device=" << dev << " -c " << param << " 2>&1";
    std::string cmd = ss.str();

    // capture the output
    std::string output;
    int buffer_size = 256;
    char buffer[buffer_size];
    FILE *stream = popen(cmd.c_str(), "r");
    if (stream)
    {
      while (!feof(stream))
        if (fgets(buffer, buffer_size, stream) != NULL)
          output.append(buffer);
      pclose(stream);
      // any output should be an error
      if (output.length() > 0)
        ROS_WARN("%s", output.c_str());
    }
    else
      ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
