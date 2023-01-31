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

#include <linux/videodev2.h>
#include <ros/ros.h>
#include <sstream>
#include "usb_cam/usb_cam.h"

using namespace usb_cam;

bool UsbCam::create_suspended = false;

/* ROS */
ros::Timer* UsbCam::frame_timer = nullptr;
sensor_msgs::Image* UsbCam::img_msg = nullptr;
image_transport::CameraPublisher* UsbCam::image_pub = nullptr;
camera_info_manager::CameraInfoManager* UsbCam::camera_info = nullptr;
ros::ServiceServer* UsbCam::service_start = nullptr;
ros::ServiceServer* UsbCam::service_stop = nullptr;
ros::ServiceServer* UsbCam::service_supported_formats = nullptr;
image_transport::ImageTransport* UsbCam::image_transport = nullptr;

/* Node parameters */
std::string UsbCam::camera_name = "head_camera";
std::string UsbCam::camera_frame_id = "head_camera";
std::string UsbCam::camera_transport_suffix = "image_raw";
std::string UsbCam::camera_info_url = "";

/* ROS Service callback functions */
bool UsbCam::service_start_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return start_capture();
}

bool UsbCam::service_stop_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return suspend();
}

bool UsbCam::service_supported_formats_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    get_supported_formats();
    std::stringstream output_stream;
    std::cout << "SUPPORTED INPUT FORMATS FOR V4L DEVICE " << video_device_name << std::endl;
    for(auto fmt : supported_formats)
    {
        output_stream << " | " << fmt.format.description
                      << " [" << fmt.interval.width << " x "
                      << fmt.interval.height << "], "
                      << fmt.interval.discrete.denominator / fmt.interval.discrete.numerator
                      << " fps";
        std::cout << "\t" << fmt.format.description
                  << " [" << fmt.interval.width << " x "
                  << fmt.interval.height << "], "
                  << fmt.interval.discrete.denominator / fmt.interval.discrete.numerator
                  << " fps" << std::endl;
    }
    response.success = true;
    response.message = output_stream.str();
    return true;
}

UsbCam::UsbCam():
    node("~"),
    _img_msg(),
    _image_transport(node),
    AbstractV4LUSBCam()
{
    img_msg = &_img_msg;
    image_transport = &_image_transport;
    /* Loading parameters */
    // Mandatory parameters are defined with getParam(), otherwise with param<>()
    node.getParam("video_device", video_device_name);
    node.getParam("io_method", io_method_name);
    node.getParam("pixel_format", pixel_format_name);
    node.getParam("color_format", color_format_name);
    node.param<bool>("create_suspended", create_suspended, false);
    node.param<bool>("full_ffmpeg_log", full_ffmpeg_log, false);
    node.getParam("camera_name", camera_name);
    node.getParam("camera_frame_id", camera_frame_id);
    node.param<std::string>("camera_transport_suffix", camera_transport_suffix, "image_raw");
    node.param<std::string>("camera_info_url", camera_info_url, "");
    node.getParam("image_width", image_width);
    node.getParam("image_height", image_height);
    node.getParam("framerate", framerate);
    node.param<int>("exposure", exposure, 100);
    node.param<int>("brightness", brightness, -1);
    node.param<int>("contrast", contrast, -1);
    node.param<int>("saturation", saturation, -1);
    node.param<int>("sharpness", sharpness, -1);
    node.param<int>("focus", focus, -1);
    node.param<int>("white_balance", white_balance, 4000);
    node.param<int>("gain", gain, -1);
    node.param<bool>("autofocus", autofocus, false);
    node.param<bool>("autoexposure", autoexposure, true);
    node.param<bool>("auto_white_balance", auto_white_balance, false);
    node.param<std::string>("start_service_name", _service_start_name, "start_capture");
    node.param<std::string>("stop_service_name", _service_stop_name, "stop_capture");

    // Advertising camera
    ROS_INFO("Initializing ROS V4L USB camera '%s' (%s) at %dx%d via %s (%s) at %i FPS",
             camera_name.c_str(),
             video_device_name.c_str(),
             image_width,
             image_height,
             io_method_name.c_str(),
             pixel_format_name.c_str(),
             framerate);
    _image_pub = image_transport->advertiseCamera(camera_transport_suffix, 1);
    image_pub = &_image_pub;
    camera_info = new camera_info_manager::CameraInfoManager(node, camera_name, camera_info_url);
    img_msg->header.frame_id = camera_frame_id;
    if(!camera_info->isCalibrated())
    {
        camera_info->setCameraName(video_device_name);
        sensor_msgs::CameraInfo camera_info_msg;
        camera_info_msg.header.frame_id = img_msg->header.frame_id;
        camera_info_msg.width = image_width;
        camera_info_msg.height = image_height;
        camera_info->setCameraInfo(camera_info_msg);
    }

    if(!init())
    {
        ROS_ERROR("Initialization error or wrong parameters");
        node.shutdown();
        return;
    }

    /* Advertising services */
    ROS_INFO("Advertising std_srvs::Empty start service under name '%s'", _service_start_name.c_str());
    _service_start = node.advertiseService(_service_start_name, &UsbCam::service_start_callback);
    service_start = &_service_start;
    ROS_INFO("Advertising std_srvs::Empty suspension service under name '%s'", _service_stop_name.c_str());
    _service_stop = node.advertiseService(_service_stop_name, &UsbCam::service_stop_callback);
    service_stop = &_service_stop;
    ROS_INFO("Advertising std_srvs::Trigger supported formats information service under name 'supported_formats'");
    _service_supported_formats = node.advertiseService("supported_formats", &UsbCam::service_supported_formats_callback);
    service_supported_formats = &_service_supported_formats;

    /* All parameters set, running frame grabber */
    if(!start())
    {
        ROS_ERROR("Error starting device");
        node.shutdown();
        return;
    }
    adjust_camera();

    // Creating timer
    ros::Duration frame_period(1.f / static_cast<float>(framerate));
    _frame_timer = node.createTimer(frame_period, &UsbCam::frame_timer_callback, false, true);
    frame_timer = &_frame_timer;
    // Running capture engine
    if(!create_suspended)
        if(!start_capture())
        {
            ROS_ERROR("Error starting capture device");
            node.shutdown();
            return;
        }
}

void UsbCam::frame_timer_callback(const ros::TimerEvent &event)
{
    if(streaming_status)
    {
        camera_image_t* new_image = read_frame();
        if(new_image == nullptr)
        {
            ROS_ERROR("Video4linux: frame grabber failed");
            return;
        }
        img_msg->header.stamp.sec = new_image->stamp.tv_sec;
        img_msg->header.stamp.nsec = new_image->stamp.tv_nsec;
        if (img_msg->data.size() != static_cast<size_t>(new_image->step * new_image->height))
        {
            img_msg->width = new_image->width;
            img_msg->height = new_image->height;
            img_msg->encoding = new_image->encoding;
            img_msg->step = new_image->step;
            img_msg->data.resize(new_image->step * new_image->height);
        }
        // Fill in image data
        memcpy(&img_msg->data[0], new_image->image, img_msg->data.size());
        auto ci = std::make_unique<sensor_msgs::CameraInfo>(camera_info->getCameraInfo());
        ci->header = img_msg->header;
        image_pub->publish((*img_msg), (*ci));
    }
}

UsbCam::~UsbCam()
{
    delete camera_info;
}

usb_cam::UsbCam &usb_cam::UsbCam::Instance()
{
    static UsbCam instance;
    return instance;
}

